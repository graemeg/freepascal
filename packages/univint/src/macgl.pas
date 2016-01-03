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

unit macgl;
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
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

//#ifndef __gl_h_
//#define __gl_h_


{
** License Applicability. Except to the extent portions of this file are
** made subject to an alternative license as permitted in the SGI Free
** Software License B, Version 1.1 (the "License"), the contents of this
** file are subject only to the provisions of the License. You may not use
** this file except in compliance with the License. You may obtain a copy
** of the License at Silicon Graphics, Inc., attn: Legal Services, 1600
** Amphitheatre Parkway, Mountain View, CA 94043-1351, or at:
** 
** http://oss.sgi.com/projects/FreeB
** 
** Note that, as provided in the License, the Software is distributed on an
** "AS IS" basis, with ALL EXPRESS AND IMPLIED WARRANTIES AND CONDITIONS
** DISCLAIMED, INCLUDING, WITHOUT LIMITATION, ANY IMPLIED WARRANTIES AND
** CONDITIONS OF MERCHANTABILITY, SATISFACTORY QUALITY, FITNESS FOR A
** PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
** 
** Original Code. The Original Code is: OpenGL Sample Implementation,
** Version 1.2.1, released January 26, 2000, developed by Silicon Graphics,
** Inc. The Original Code is Copyright (c) 1991-2000 Silicon Graphics, Inc.
** Copyright in any portions created by third parties is as indicated
** elsewhere herein. All Rights Reserved.
** 
** Additional Notice Provisions: This software was created using the
** OpenGL(R) version 1.2.1 Sample Implementation published by SGI, but has
** not been independently verified as being compliant with the OpenGL(R)
** version 1.2.1 Specification.
}

// switches to providing function pointers
//#define GL_GLEXT_FUNCTION_POINTERS 1

type
	GLenum = UInt32;
	PGLenum = ^GLenum;
	
	GLboolean = UInt8;
	PGLboolean = ^GLboolean;
	
	GLbitfield = UInt32;
	PGLbitfield = ^GLbitfield;
	
	GLbyte = SInt8;
	PGLbyte = ^GLbyte;
	
	GLshort = SInt16;
	PGLshort = ^GLshort;
	
	GLint = SInt32;
	PGLint = ^GLint;
	
	GLsizei = SInt32;
	PGLsizei = ^GLsizei;
	
	GLubyte = UInt8;
	PGLubyte = ^GLubyte;
	
	GLushort = UInt16;
	PGLushort = ^GLushort;
	
	GLuint = UInt32;
	PGLuint = ^GLuint;
	
	GLfloat = Float32;
	PGLfloat = ^GLfloat;
	
	GLclampf = Float32;
	PGLclampf = ^GLclampf;
	
	GLdouble = Float64;
	PGLdouble = ^GLdouble;
	
	GLclampd = Float64;
	PGLclampd = ^GLclampd;
	
//	GLvoid = void;

type
	GLintptr = SIGNEDLONG;
	GLsizeiptr = SIGNEDLONG;

{$ifc undefined GL_TYPEDEFS_2_0}
{$setc GL_TYPEDEFS_2_0 := TRUE}
type
	GLchar = char;
{$endc}


{***********************************************************}

{ Version }
{$setc GL_VERSION_1_1 := TRUE}
{$setc GL_VERSION_1_2 := TRUE}
{$setc GL_VERSION_1_3 := TRUE}
{$setc GL_VERSION_1_4 := TRUE}
{$setc GL_VERSION_1_5 := TRUE}
{$setc GL_VERSION_2_0 := TRUE}
{$setc GL_VERSION_2_1 := TRUE}

{ AccumOp }
const GL_ACCUM                          = $0100;
const GL_LOAD                           = $0101;
const GL_RETURN                         = $0102;
const GL_MULT                           = $0103;
const GL_ADD                            = $0104;

{ AlphaFunction }
const GL_NEVER                          = $0200;
const GL_LESS                           = $0201;
const GL_EQUAL                          = $0202;
const GL_LEQUAL                         = $0203;
const GL_GREATER                        = $0204;
const GL_NOTEQUAL                       = $0205;
const GL_GEQUAL                         = $0206;
const GL_ALWAYS                         = $0207;

{ AttribMask }
const GL_CURRENT_BIT                    = $00000001;
const GL_POINT_BIT                      = $00000002;
const GL_LINE_BIT                       = $00000004;
const GL_POLYGON_BIT                    = $00000008;
const GL_POLYGON_STIPPLE_BIT            = $00000010;
const GL_PIXEL_MODE_BIT                 = $00000020;
const GL_LIGHTING_BIT                   = $00000040;
const GL_FOG_BIT                        = $00000080;
const GL_DEPTH_BUFFER_BIT               = $00000100;
const GL_ACCUM_BUFFER_BIT               = $00000200;
const GL_STENCIL_BUFFER_BIT             = $00000400;
const GL_VIEWPORT_BIT                   = $00000800;
const GL_TRANSFORM_BIT                  = $00001000;
const GL_ENABLE_BIT                     = $00002000;
const GL_COLOR_BUFFER_BIT               = $00004000;
const GL_HINT_BIT                       = $00008000;
const GL_EVAL_BIT                       = $00010000;
const GL_LIST_BIT                       = $00020000;
const GL_TEXTURE_BIT                    = $00040000;
const GL_SCISSOR_BIT                    = $00080000;
const GL_ALL_ATTRIB_BITS                = $000fffff;

{ BeginMode }
const GL_POINTS                         = $0000;
const GL_LINES                          = $0001;
const GL_LINE_LOOP                      = $0002;
const GL_LINE_STRIP                     = $0003;
const GL_TRIANGLES                      = $0004;
const GL_TRIANGLE_STRIP                 = $0005;
const GL_TRIANGLE_FAN                   = $0006;
const GL_QUADS                          = $0007;
const GL_QUAD_STRIP                     = $0008;
const GL_POLYGON                        = $0009;

{ BlendEquationMode }
{      GL_LOGIC_OP }
{      GL_FUNC_ADD }
{      GL_MIN }
{      GL_MAX }
{      GL_FUNC_SUBTRACT }
{      GL_FUNC_REVERSE_SUBTRACT }

{ BlendingFactorDest }
const
	GL_ZERO = 0;
const
	GL_ONE = 1;
const GL_SRC_COLOR                      = $0300;
const GL_ONE_MINUS_SRC_COLOR            = $0301;
const GL_SRC_ALPHA                      = $0302;
const GL_ONE_MINUS_SRC_ALPHA            = $0303;
const GL_DST_ALPHA                      = $0304;
const GL_ONE_MINUS_DST_ALPHA            = $0305;
{      GL_CONSTANT_COLOR }
{      GL_ONE_MINUS_CONSTANT_COLOR }
{      GL_CONSTANT_ALPHA }
{      GL_ONE_MINUS_CONSTANT_ALPHA }

{ BlendingFactorSrc }
{      GL_ZERO }
{      GL_ONE }
const GL_DST_COLOR                      = $0306;
const GL_ONE_MINUS_DST_COLOR            = $0307;
const GL_SRC_ALPHA_SATURATE             = $0308;
{      GL_SRC_ALPHA }
{      GL_ONE_MINUS_SRC_ALPHA }
{      GL_DST_ALPHA }
{      GL_ONE_MINUS_DST_ALPHA }
{      GL_CONSTANT_COLOR }
{      GL_ONE_MINUS_CONSTANT_COLOR }
{      GL_CONSTANT_ALPHA }
{      GL_ONE_MINUS_CONSTANT_ALPHA }

{ Boolean }
const
	GL_TRUE = 1;
const
	GL_FALSE = 0;

{ ClearBufferMask }
{      GL_COLOR_BUFFER_BIT }
{      GL_ACCUM_BUFFER_BIT }
{      GL_STENCIL_BUFFER_BIT }
{      GL_DEPTH_BUFFER_BIT }

{ ClientArrayType }
{      GL_VERTEX_ARRAY }
{      GL_NORMAL_ARRAY }
{      GL_COLOR_ARRAY }
{      GL_INDEX_ARRAY }
{      GL_TEXTURE_COORD_ARRAY }
{      GL_EDGE_FLAG_ARRAY }

{ ClipPlaneName }
const GL_CLIP_PLANE0                    = $3000;
const GL_CLIP_PLANE1                    = $3001;
const GL_CLIP_PLANE2                    = $3002;
const GL_CLIP_PLANE3                    = $3003;
const GL_CLIP_PLANE4                    = $3004;
const GL_CLIP_PLANE5                    = $3005;

{ ColorMaterialFace }
{      GL_FRONT }
{      GL_BACK }
{      GL_FRONT_AND_BACK }

{ ColorMaterialParameter }
{      GL_AMBIENT }
{      GL_DIFFUSE }
{      GL_SPECULAR }
{      GL_EMISSION }
{      GL_AMBIENT_AND_DIFFUSE }

{ ColorPointerType }
{      GL_BYTE }
{      GL_UNSIGNED_BYTE }
{      GL_SHORT }
{      GL_UNSIGNED_SHORT }
{      GL_INT }
{      GL_UNSIGNED_INT }
{      GL_FLOAT }
{      GL_DOUBLE }

{ ColorTableParameterPName }
{      GL_COLOR_TABLE_SCALE }
{      GL_COLOR_TABLE_BIAS }

{ ColorTableTarget }
{      GL_COLOR_TABLE }
{      GL_POST_CONVOLUTION_COLOR_TABLE }
{      GL_POST_COLOR_MATRIX_COLOR_TABLE }
{      GL_PROXY_COLOR_TABLE }
{      GL_PROXY_POST_CONVOLUTION_COLOR_TABLE }
{      GL_PROXY_POST_COLOR_MATRIX_COLOR_TABLE }

{ ConvolutionBorderMode }
{      GL_REDUCE }
{      GL_IGNORE_BORDER }
{      GL_CONSTANT_BORDER }

{ ConvolutionParameter }
{      GL_CONVOLUTION_BORDER_MODE }
{      GL_CONVOLUTION_FILTER_SCALE }
{      GL_CONVOLUTION_FILTER_BIAS }

{ ConvolutionTarget }
{      GL_CONVOLUTION_1D }
{      GL_CONVOLUTION_2D }

{ CullFaceMode }
{      GL_FRONT }
{      GL_BACK }
{      GL_FRONT_AND_BACK }

{ DataType }
const GL_BYTE                           = $1400;
const GL_UNSIGNED_BYTE                  = $1401;
const GL_SHORT                          = $1402;
const GL_UNSIGNED_SHORT                 = $1403;
const GL_INT                            = $1404;
const GL_UNSIGNED_INT                   = $1405;
const GL_FLOAT                          = $1406;
const GL_2_BYTES                        = $1407;
const GL_3_BYTES                        = $1408;
const GL_4_BYTES                        = $1409;
const GL_DOUBLE                         = $140A;

{ DepthFunction }
{      GL_NEVER }
{      GL_LESS }
{      GL_EQUAL }
{      GL_LEQUAL }
{      GL_GREATER }
{      GL_NOTEQUAL }
{      GL_GEQUAL }
{      GL_ALWAYS }

{ DrawBufferMode }
const
	GL_NONE = 0;
const GL_FRONT_LEFT                     = $0400;
const GL_FRONT_RIGHT                    = $0401;
const GL_BACK_LEFT                      = $0402;
const GL_BACK_RIGHT                     = $0403;
const GL_FRONT                          = $0404;
const GL_BACK                           = $0405;
const GL_LEFT                           = $0406;
const GL_RIGHT                          = $0407;
const GL_FRONT_AND_BACK                 = $0408;
const GL_AUX0                           = $0409;
const GL_AUX1                           = $040A;
const GL_AUX2                           = $040B;
const GL_AUX3                           = $040C;

{ Enable }
{      GL_FOG }
{      GL_LIGHTING }
{      GL_TEXTURE_1D }
{      GL_TEXTURE_2D }
{      GL_LINE_STIPPLE }
{      GL_POLYGON_STIPPLE }
{      GL_CULL_FACE }
{      GL_ALPHA_TEST }
{      GL_BLEND }
{      GL_INDEX_LOGIC_OP }
{      GL_COLOR_LOGIC_OP }
{      GL_DITHER }
{      GL_STENCIL_TEST }
{      GL_DEPTH_TEST }
{      GL_CLIP_PLANE0 }
{      GL_CLIP_PLANE1 }
{      GL_CLIP_PLANE2 }
{      GL_CLIP_PLANE3 }
{      GL_CLIP_PLANE4 }
{      GL_CLIP_PLANE5 }
{      GL_LIGHT0 }
{      GL_LIGHT1 }
{      GL_LIGHT2 }
{      GL_LIGHT3 }
{      GL_LIGHT4 }
{      GL_LIGHT5 }
{      GL_LIGHT6 }
{      GL_LIGHT7 }
{      GL_TEXTURE_GEN_S }
{      GL_TEXTURE_GEN_T }
{      GL_TEXTURE_GEN_R }
{      GL_TEXTURE_GEN_Q }
{      GL_MAP1_VERTEX_3 }
{      GL_MAP1_VERTEX_4 }
{      GL_MAP1_COLOR_4 }
{      GL_MAP1_INDEX }
{      GL_MAP1_NORMAL }
{      GL_MAP1_TEXTURE_COORD_1 }
{      GL_MAP1_TEXTURE_COORD_2 }
{      GL_MAP1_TEXTURE_COORD_3 }
{      GL_MAP1_TEXTURE_COORD_4 }
{      GL_MAP2_VERTEX_3 }
{      GL_MAP2_VERTEX_4 }
{      GL_MAP2_COLOR_4 }
{      GL_MAP2_INDEX }
{      GL_MAP2_NORMAL }
{      GL_MAP2_TEXTURE_COORD_1 }
{      GL_MAP2_TEXTURE_COORD_2 }
{      GL_MAP2_TEXTURE_COORD_3 }
{      GL_MAP2_TEXTURE_COORD_4 }
{      GL_POINT_SMOOTH }
{      GL_LINE_SMOOTH }
{      GL_POLYGON_SMOOTH }
{      GL_SCISSOR_TEST }
{      GL_COLOR_MATERIAL }
{      GL_NORMALIZE }
{      GL_AUTO_NORMAL }
{      GL_VERTEX_ARRAY }
{      GL_NORMAL_ARRAY }
{      GL_COLOR_ARRAY }
{      GL_INDEX_ARRAY }
{      GL_TEXTURE_COORD_ARRAY }
{      GL_EDGE_FLAG_ARRAY }
{      GL_POLYGON_OFFSET_POINT }
{      GL_POLYGON_OFFSET_LINE }
{      GL_POLYGON_OFFSET_FILL }
{      GL_COLOR_TABLE }
{      GL_POST_CONVOLUTION_COLOR_TABLE }
{      GL_POST_COLOR_MATRIX_COLOR_TABLE }
{      GL_CONVOLUTION_1D }
{      GL_CONVOLUTION_2D }
{      GL_SEPARABLE_2D }
{      GL_HISTOGRAM }
{      GL_MINMAX }
{      GL_RESCALE_NORMAL }
{      GL_TEXTURE_3D }

{ ErrorCode }
const
	GL_NO_ERROR = 0;
const GL_INVALID_ENUM                   = $0500;
const GL_INVALID_VALUE                  = $0501;
const GL_INVALID_OPERATION              = $0502;
const GL_STACK_OVERFLOW                 = $0503;
const GL_STACK_UNDERFLOW                = $0504;
const GL_OUT_OF_MEMORY                  = $0505;
{      GL_TABLE_TOO_LARGE }

{ FeedBackMode }
const GL_2D                             = $0600;
const GL_3D                             = $0601;
const GL_3D_COLOR                       = $0602;
const GL_3D_COLOR_TEXTURE               = $0603;
const GL_4D_COLOR_TEXTURE               = $0604;

{ FeedBackToken }
const GL_PASS_THROUGH_TOKEN             = $0700;
const GL_POINT_TOKEN                    = $0701;
const GL_LINE_TOKEN                     = $0702;
const GL_POLYGON_TOKEN                  = $0703;
const GL_BITMAP_TOKEN                   = $0704;
const GL_DRAW_PIXEL_TOKEN               = $0705;
const GL_COPY_PIXEL_TOKEN               = $0706;
const GL_LINE_RESET_TOKEN               = $0707;

{ FogMode }
{      GL_LINEAR }
const GL_EXP                            = $0800;
const GL_EXP2                           = $0801;

{ FogParameter }
{      GL_FOG_COLOR }
{      GL_FOG_DENSITY }
{      GL_FOG_END }
{      GL_FOG_INDEX }
{      GL_FOG_MODE }
{      GL_FOG_START }

{ FrontFaceDirection }
const GL_CW                             = $0900;
const GL_CCW                            = $0901;

{ GetColorTableParameterPName }
{      GL_COLOR_TABLE_SCALE }
{      GL_COLOR_TABLE_BIAS }
{      GL_COLOR_TABLE_FORMAT }
{      GL_COLOR_TABLE_WIDTH }
{      GL_COLOR_TABLE_RED_SIZE }
{      GL_COLOR_TABLE_GREEN_SIZE }
{      GL_COLOR_TABLE_BLUE_SIZE }
{      GL_COLOR_TABLE_ALPHA_SIZE }
{      GL_COLOR_TABLE_LUMINANCE_SIZE }
{      GL_COLOR_TABLE_INTENSITY_SIZE }

{ GetConvolutionParameterPName }
{      GL_CONVOLUTION_BORDER_COLOR }
{      GL_CONVOLUTION_BORDER_MODE }
{      GL_CONVOLUTION_FILTER_SCALE }
{      GL_CONVOLUTION_FILTER_BIAS }
{      GL_CONVOLUTION_FORMAT }
{      GL_CONVOLUTION_WIDTH }
{      GL_CONVOLUTION_HEIGHT }
{      GL_MAX_CONVOLUTION_WIDTH }
{      GL_MAX_CONVOLUTION_HEIGHT }

{ GetHistogramParameterPName }
{      GL_HISTOGRAM_WIDTH }
{      GL_HISTOGRAM_FORMAT }
{      GL_HISTOGRAM_RED_SIZE }
{      GL_HISTOGRAM_GREEN_SIZE }
{      GL_HISTOGRAM_BLUE_SIZE }
{      GL_HISTOGRAM_ALPHA_SIZE }
{      GL_HISTOGRAM_LUMINANCE_SIZE }
{      GL_HISTOGRAM_SINK }

{ GetMapTarget }
const GL_COEFF                          = $0A00;
const GL_ORDER                          = $0A01;
const GL_DOMAIN                         = $0A02;

{ GetMinmaxParameterPName }
{      GL_MINMAX_FORMAT }
{      GL_MINMAX_SINK }

{ GetPixelMap }
{      GL_PIXEL_MAP_I_TO_I }
{      GL_PIXEL_MAP_S_TO_S }
{      GL_PIXEL_MAP_I_TO_R }
{      GL_PIXEL_MAP_I_TO_G }
{      GL_PIXEL_MAP_I_TO_B }
{      GL_PIXEL_MAP_I_TO_A }
{      GL_PIXEL_MAP_R_TO_R }
{      GL_PIXEL_MAP_G_TO_G }
{      GL_PIXEL_MAP_B_TO_B }
{      GL_PIXEL_MAP_A_TO_A }

{ GetPointerTarget }
{      GL_VERTEX_ARRAY_POINTER }
{      GL_NORMAL_ARRAY_POINTER }
{      GL_COLOR_ARRAY_POINTER }
{      GL_INDEX_ARRAY_POINTER }
{      GL_TEXTURE_COORD_ARRAY_POINTER }
{      GL_EDGE_FLAG_ARRAY_POINTER }

{ GetTarget }
const GL_CURRENT_COLOR                  = $0B00;
const GL_CURRENT_INDEX                  = $0B01;
const GL_CURRENT_NORMAL                 = $0B02;
const GL_CURRENT_TEXTURE_COORDS         = $0B03;
const GL_CURRENT_RASTER_COLOR           = $0B04;
const GL_CURRENT_RASTER_INDEX           = $0B05;
const GL_CURRENT_RASTER_TEXTURE_COORDS  = $0B06;
const GL_CURRENT_RASTER_POSITION        = $0B07;
const GL_CURRENT_RASTER_POSITION_VALID  = $0B08;
const GL_CURRENT_RASTER_DISTANCE        = $0B09;
const GL_POINT_SMOOTH                   = $0B10;
const GL_POINT_SIZE                     = $0B11;
const GL_POINT_SIZE_RANGE               = $0B12;
const GL_POINT_SIZE_GRANULARITY         = $0B13;
const GL_LINE_SMOOTH                    = $0B20;
const GL_LINE_WIDTH                     = $0B21;
const GL_LINE_WIDTH_RANGE               = $0B22;
const GL_LINE_WIDTH_GRANULARITY         = $0B23;
const GL_LINE_STIPPLE                   = $0B24;
const GL_LINE_STIPPLE_PATTERN           = $0B25;
const GL_LINE_STIPPLE_REPEAT            = $0B26;
{      GL_SMOOTH_POINT_SIZE_RANGE }
{      GL_SMOOTH_POINT_SIZE_GRANULARITY }
{      GL_SMOOTH_LINE_WIDTH_RANGE }
{      GL_SMOOTH_LINE_WIDTH_GRANULARITY }
{      GL_ALIASED_POINT_SIZE_RANGE }
{      GL_ALIASED_LINE_WIDTH_RANGE }
const GL_LIST_MODE                      = $0B30;
const GL_MAX_LIST_NESTING               = $0B31;
const GL_LIST_BASE                      = $0B32;
const GL_LIST_INDEX                     = $0B33;
const GL_POLYGON_MODE                   = $0B40;
const GL_POLYGON_SMOOTH                 = $0B41;
const GL_POLYGON_STIPPLE                = $0B42;
const GL_EDGE_FLAG                      = $0B43;
const GL_CULL_FACE                      = $0B44;
const GL_CULL_FACE_MODE                 = $0B45;
const GL_FRONT_FACE                     = $0B46;
const GL_LIGHTING                       = $0B50;
const GL_LIGHT_MODEL_LOCAL_VIEWER       = $0B51;
const GL_LIGHT_MODEL_TWO_SIDE           = $0B52;
const GL_LIGHT_MODEL_AMBIENT            = $0B53;
const GL_SHADE_MODEL                    = $0B54;
const GL_COLOR_MATERIAL_FACE            = $0B55;
const GL_COLOR_MATERIAL_PARAMETER       = $0B56;
const GL_COLOR_MATERIAL                 = $0B57;
const GL_FOG                            = $0B60;
const GL_FOG_INDEX                      = $0B61;
const GL_FOG_DENSITY                    = $0B62;
const GL_FOG_START                      = $0B63;
const GL_FOG_END                        = $0B64;
const GL_FOG_MODE                       = $0B65;
const GL_FOG_COLOR                      = $0B66;
const GL_DEPTH_RANGE                    = $0B70;
const GL_DEPTH_TEST                     = $0B71;
const GL_DEPTH_WRITEMASK                = $0B72;
const GL_DEPTH_CLEAR_VALUE              = $0B73;
const GL_DEPTH_FUNC                     = $0B74;
const GL_ACCUM_CLEAR_VALUE              = $0B80;
const GL_STENCIL_TEST                   = $0B90;
const GL_STENCIL_CLEAR_VALUE            = $0B91;
const GL_STENCIL_FUNC                   = $0B92;
const GL_STENCIL_VALUE_MASK             = $0B93;
const GL_STENCIL_FAIL                   = $0B94;
const GL_STENCIL_PASS_DEPTH_FAIL        = $0B95;
const GL_STENCIL_PASS_DEPTH_PASS        = $0B96;
const GL_STENCIL_REF                    = $0B97;
const GL_STENCIL_WRITEMASK              = $0B98;
const GL_MATRIX_MODE                    = $0BA0;
const GL_NORMALIZE                      = $0BA1;
const GL_VIEWPORT                       = $0BA2;
const GL_MODELVIEW_STACK_DEPTH          = $0BA3;
const GL_PROJECTION_STACK_DEPTH         = $0BA4;
const GL_TEXTURE_STACK_DEPTH            = $0BA5;
const GL_MODELVIEW_MATRIX               = $0BA6;
const GL_PROJECTION_MATRIX              = $0BA7;
const GL_TEXTURE_MATRIX                 = $0BA8;
const GL_ATTRIB_STACK_DEPTH             = $0BB0;
const GL_CLIENT_ATTRIB_STACK_DEPTH      = $0BB1;
const GL_ALPHA_TEST                     = $0BC0;
const GL_ALPHA_TEST_FUNC                = $0BC1;
const GL_ALPHA_TEST_REF                 = $0BC2;
const GL_DITHER                         = $0BD0;
const GL_BLEND_DST                      = $0BE0;
const GL_BLEND_SRC                      = $0BE1;
const GL_BLEND                          = $0BE2;
const GL_LOGIC_OP_MODE                  = $0BF0;

const GL_INDEX_LOGIC_OP                 = $0BF1;
{ For compatibility with OpenGL v1.0 }
const GL_LOGIC_OP						= GL_INDEX_LOGIC_OP;

const GL_COLOR_LOGIC_OP                 = $0BF2;
const GL_AUX_BUFFERS                    = $0C00;
const GL_DRAW_BUFFER                    = $0C01;
const GL_READ_BUFFER                    = $0C02;
const GL_SCISSOR_BOX                    = $0C10;
const GL_SCISSOR_TEST                   = $0C11;
const GL_INDEX_CLEAR_VALUE              = $0C20;
const GL_INDEX_WRITEMASK                = $0C21;
const GL_COLOR_CLEAR_VALUE              = $0C22;
const GL_COLOR_WRITEMASK                = $0C23;
const GL_INDEX_MODE                     = $0C30;
const GL_RGBA_MODE                      = $0C31;
const GL_DOUBLEBUFFER                   = $0C32;
const GL_STEREO                         = $0C33;
const GL_RENDER_MODE                    = $0C40;
const GL_PERSPECTIVE_CORRECTION_HINT    = $0C50;
const GL_POINT_SMOOTH_HINT              = $0C51;
const GL_LINE_SMOOTH_HINT               = $0C52;
const GL_POLYGON_SMOOTH_HINT            = $0C53;
const GL_FOG_HINT                       = $0C54;
const GL_TEXTURE_GEN_S                  = $0C60;
const GL_TEXTURE_GEN_T                  = $0C61;
const GL_TEXTURE_GEN_R                  = $0C62;
const GL_TEXTURE_GEN_Q                  = $0C63;
const GL_PIXEL_MAP_I_TO_I               = $0C70;
const GL_PIXEL_MAP_S_TO_S               = $0C71;
const GL_PIXEL_MAP_I_TO_R               = $0C72;
const GL_PIXEL_MAP_I_TO_G               = $0C73;
const GL_PIXEL_MAP_I_TO_B               = $0C74;
const GL_PIXEL_MAP_I_TO_A               = $0C75;
const GL_PIXEL_MAP_R_TO_R               = $0C76;
const GL_PIXEL_MAP_G_TO_G               = $0C77;
const GL_PIXEL_MAP_B_TO_B               = $0C78;
const GL_PIXEL_MAP_A_TO_A               = $0C79;
const GL_PIXEL_MAP_I_TO_I_SIZE          = $0CB0;
const GL_PIXEL_MAP_S_TO_S_SIZE          = $0CB1;
const GL_PIXEL_MAP_I_TO_R_SIZE          = $0CB2;
const GL_PIXEL_MAP_I_TO_G_SIZE          = $0CB3;
const GL_PIXEL_MAP_I_TO_B_SIZE          = $0CB4;
const GL_PIXEL_MAP_I_TO_A_SIZE          = $0CB5;
const GL_PIXEL_MAP_R_TO_R_SIZE          = $0CB6;
const GL_PIXEL_MAP_G_TO_G_SIZE          = $0CB7;
const GL_PIXEL_MAP_B_TO_B_SIZE          = $0CB8;
const GL_PIXEL_MAP_A_TO_A_SIZE          = $0CB9;
const GL_UNPACK_SWAP_BYTES              = $0CF0;
const GL_UNPACK_LSB_FIRST               = $0CF1;
const GL_UNPACK_ROW_LENGTH              = $0CF2;
const GL_UNPACK_SKIP_ROWS               = $0CF3;
const GL_UNPACK_SKIP_PIXELS             = $0CF4;
const GL_UNPACK_ALIGNMENT               = $0CF5;
const GL_PACK_SWAP_BYTES                = $0D00;
const GL_PACK_LSB_FIRST                 = $0D01;
const GL_PACK_ROW_LENGTH                = $0D02;
const GL_PACK_SKIP_ROWS                 = $0D03;
const GL_PACK_SKIP_PIXELS               = $0D04;
const GL_PACK_ALIGNMENT                 = $0D05;
const GL_MAP_COLOR                      = $0D10;
const GL_MAP_STENCIL                    = $0D11;
const GL_INDEX_SHIFT                    = $0D12;
const GL_INDEX_OFFSET                   = $0D13;
const GL_RED_SCALE                      = $0D14;
const GL_RED_BIAS                       = $0D15;
const GL_ZOOM_X                         = $0D16;
const GL_ZOOM_Y                         = $0D17;
const GL_GREEN_SCALE                    = $0D18;
const GL_GREEN_BIAS                     = $0D19;
const GL_BLUE_SCALE                     = $0D1A;
const GL_BLUE_BIAS                      = $0D1B;
const GL_ALPHA_SCALE                    = $0D1C;
const GL_ALPHA_BIAS                     = $0D1D;
const GL_DEPTH_SCALE                    = $0D1E;
const GL_DEPTH_BIAS                     = $0D1F;
const GL_MAX_EVAL_ORDER                 = $0D30;
const GL_MAX_LIGHTS                     = $0D31;
const GL_MAX_CLIP_PLANES                = $0D32;
const GL_MAX_TEXTURE_SIZE               = $0D33;
const GL_MAX_PIXEL_MAP_TABLE            = $0D34;
const GL_MAX_ATTRIB_STACK_DEPTH         = $0D35;
const GL_MAX_MODELVIEW_STACK_DEPTH      = $0D36;
const GL_MAX_NAME_STACK_DEPTH           = $0D37;
const GL_MAX_PROJECTION_STACK_DEPTH     = $0D38;
const GL_MAX_TEXTURE_STACK_DEPTH        = $0D39;
const GL_MAX_VIEWPORT_DIMS              = $0D3A;
const GL_MAX_CLIENT_ATTRIB_STACK_DEPTH  = $0D3B;
const GL_SUBPIXEL_BITS                  = $0D50;
const GL_INDEX_BITS                     = $0D51;
const GL_RED_BITS                       = $0D52;
const GL_GREEN_BITS                     = $0D53;
const GL_BLUE_BITS                      = $0D54;
const GL_ALPHA_BITS                     = $0D55;
const GL_DEPTH_BITS                     = $0D56;
const GL_STENCIL_BITS                   = $0D57;
const GL_ACCUM_RED_BITS                 = $0D58;
const GL_ACCUM_GREEN_BITS               = $0D59;
const GL_ACCUM_BLUE_BITS                = $0D5A;
const GL_ACCUM_ALPHA_BITS               = $0D5B;
const GL_NAME_STACK_DEPTH               = $0D70;
const GL_AUTO_NORMAL                    = $0D80;
const GL_MAP1_COLOR_4                   = $0D90;
const GL_MAP1_INDEX                     = $0D91;
const GL_MAP1_NORMAL                    = $0D92;
const GL_MAP1_TEXTURE_COORD_1           = $0D93;
const GL_MAP1_TEXTURE_COORD_2           = $0D94;
const GL_MAP1_TEXTURE_COORD_3           = $0D95;
const GL_MAP1_TEXTURE_COORD_4           = $0D96;
const GL_MAP1_VERTEX_3                  = $0D97;
const GL_MAP1_VERTEX_4                  = $0D98;
const GL_MAP2_COLOR_4                   = $0DB0;
const GL_MAP2_INDEX                     = $0DB1;
const GL_MAP2_NORMAL                    = $0DB2;
const GL_MAP2_TEXTURE_COORD_1           = $0DB3;
const GL_MAP2_TEXTURE_COORD_2           = $0DB4;
const GL_MAP2_TEXTURE_COORD_3           = $0DB5;
const GL_MAP2_TEXTURE_COORD_4           = $0DB6;
const GL_MAP2_VERTEX_3                  = $0DB7;
const GL_MAP2_VERTEX_4                  = $0DB8;
const GL_MAP1_GRID_DOMAIN               = $0DD0;
const GL_MAP1_GRID_SEGMENTS             = $0DD1;
const GL_MAP2_GRID_DOMAIN               = $0DD2;
const GL_MAP2_GRID_SEGMENTS             = $0DD3;
const GL_TEXTURE_1D                     = $0DE0;
const GL_TEXTURE_2D                     = $0DE1;
const GL_FEEDBACK_BUFFER_POINTER        = $0DF0;
const GL_FEEDBACK_BUFFER_SIZE           = $0DF1;
const GL_FEEDBACK_BUFFER_TYPE           = $0DF2;
const GL_SELECTION_BUFFER_POINTER       = $0DF3;
const GL_SELECTION_BUFFER_SIZE          = $0DF4;
{      GL_TEXTURE_BINDING_1D }
{      GL_TEXTURE_BINDING_2D }
{      GL_TEXTURE_BINDING_3D }
{      GL_VERTEX_ARRAY }
{      GL_NORMAL_ARRAY }
{      GL_COLOR_ARRAY }
{      GL_INDEX_ARRAY }
{      GL_TEXTURE_COORD_ARRAY }
{      GL_EDGE_FLAG_ARRAY }
{      GL_VERTEX_ARRAY_SIZE }
{      GL_VERTEX_ARRAY_TYPE }
{      GL_VERTEX_ARRAY_STRIDE }
{      GL_NORMAL_ARRAY_TYPE }
{      GL_NORMAL_ARRAY_STRIDE }
{      GL_COLOR_ARRAY_SIZE }
{      GL_COLOR_ARRAY_TYPE }
{      GL_COLOR_ARRAY_STRIDE }
{      GL_INDEX_ARRAY_TYPE }
{      GL_INDEX_ARRAY_STRIDE }
{      GL_TEXTURE_COORD_ARRAY_SIZE }
{      GL_TEXTURE_COORD_ARRAY_TYPE }
{      GL_TEXTURE_COORD_ARRAY_STRIDE }
{      GL_EDGE_FLAG_ARRAY_STRIDE }
{      GL_POLYGON_OFFSET_FACTOR }
{      GL_POLYGON_OFFSET_UNITS }
{      GL_COLOR_TABLE }
{      GL_POST_CONVOLUTION_COLOR_TABLE }
{      GL_POST_COLOR_MATRIX_COLOR_TABLE }
{      GL_CONVOLUTION_1D }
{      GL_CONVOLUTION_2D }
{      GL_SEPARABLE_2D }
{      GL_POST_CONVOLUTION_RED_SCALE }
{      GL_POST_CONVOLUTION_GREEN_SCALE }
{      GL_POST_CONVOLUTION_BLUE_SCALE }
{      GL_POST_CONVOLUTION_ALPHA_SCALE }
{      GL_POST_CONVOLUTION_RED_BIAS }
{      GL_POST_CONVOLUTION_GREEN_BIAS }
{      GL_POST_CONVOLUTION_BLUE_BIAS }
{      GL_POST_CONVOLUTION_ALPHA_BIAS }
{      GL_COLOR_MATRIX }
{      GL_COLOR_MATRIX_STACK_DEPTH }
{      GL_MAX_COLOR_MATRIX_STACK_DEPTH }
{      GL_POST_COLOR_MATRIX_RED_SCALE }
{      GL_POST_COLOR_MATRIX_GREEN_SCALE }
{      GL_POST_COLOR_MATRIX_BLUE_SCALE }
{      GL_POST_COLOR_MATRIX_ALPHA_SCALE }
{      GL_POST_COLOR_MATRIX_RED_BIAS }
{      GL_POST_COLOR_MATRIX_GREEN_BIAS }
{      GL_POST_COLOR_MATRIX_BLUE_BIAS }
{      GL_POST_COLOR_MATRIX_ALPHA_BIAS }
{      GL_HISTOGRAM }
{      GL_MINMAX }
{      GL_MAX_ELEMENTS_VERTICES }
{      GL_MAX_ELEMENTS_INDICES }
{      GL_RESCALE_NORMAL }
{      GL_LIGHT_MODEL_COLOR_CONTROL }
{      GL_PACK_SKIP_IMAGES }
{      GL_PACK_IMAGE_HEIGHT }
{      GL_UNPACK_SKIP_IMAGES }
{      GL_UNPACK_IMAGE_HEIGHT }
{      GL_TEXTURE_3D }
{      GL_MAX_3D_TEXTURE_SIZE }
{      GL_BLEND_COLOR }
{      GL_BLEND_EQUATION }

{ GetTextureParameter }
{      GL_TEXTURE_MAG_FILTER }
{      GL_TEXTURE_MIN_FILTER }
{      GL_TEXTURE_WRAP_S }
{      GL_TEXTURE_WRAP_T }
const GL_TEXTURE_WIDTH                  = $1000;
const GL_TEXTURE_HEIGHT                 = $1001;

{ For compatibility with OpenGL v1.0 }
const GL_TEXTURE_INTERNAL_FORMAT        = $1003;
const GL_TEXTURE_COMPONENTS				= GL_TEXTURE_INTERNAL_FORMAT;

const GL_TEXTURE_BORDER_COLOR           = $1004;
const GL_TEXTURE_BORDER                 = $1005;
{      GL_TEXTURE_RED_SIZE }
{      GL_TEXTURE_GREEN_SIZE }
{      GL_TEXTURE_BLUE_SIZE }
{      GL_TEXTURE_ALPHA_SIZE }
{      GL_TEXTURE_LUMINANCE_SIZE }
{      GL_TEXTURE_INTENSITY_SIZE }
{      GL_TEXTURE_PRIORITY }
{      GL_TEXTURE_RESIDENT }
{      GL_TEXTURE_DEPTH }
{      GL_TEXTURE_WRAP_R }
{      GL_TEXTURE_MIN_LOD }
{      GL_TEXTURE_MAX_LOD }
{      GL_TEXTURE_BASE_LEVEL }
{      GL_TEXTURE_MAX_LEVEL }

{ HintMode }
const GL_DONT_CARE                      = $1100;
const GL_FASTEST                        = $1101;
const GL_NICEST                         = $1102;

{ HintTarget }
{      GL_PERSPECTIVE_CORRECTION_HINT }
{      GL_POINT_SMOOTH_HINT }
{      GL_LINE_SMOOTH_HINT }
{      GL_POLYGON_SMOOTH_HINT }
{      GL_FOG_HINT }

{ HistogramTarget }
{      GL_HISTOGRAM }
{      GL_PROXY_HISTOGRAM }

{ IndexPointerType }
{      GL_SHORT }
{      GL_INT }
{      GL_FLOAT }
{      GL_DOUBLE }

{ LightModelColorControl }
{      GL_SINGLE_COLOR }
{      GL_SEPARATE_SPECULAR_COLOR }

{ LightModelParameter }
{      GL_LIGHT_MODEL_AMBIENT }
{      GL_LIGHT_MODEL_LOCAL_VIEWER }
{      GL_LIGHT_MODEL_TWO_SIDE }
{      GL_LIGHT_MODEL_COLOR_CONTROL }

{ LightName }
const GL_LIGHT0                         = $4000;
const GL_LIGHT1                         = $4001;
const GL_LIGHT2                         = $4002;
const GL_LIGHT3                         = $4003;
const GL_LIGHT4                         = $4004;
const GL_LIGHT5                         = $4005;
const GL_LIGHT6                         = $4006;
const GL_LIGHT7                         = $4007;

{ LightParameter }
const GL_AMBIENT                        = $1200;
const GL_DIFFUSE                        = $1201;
const GL_SPECULAR                       = $1202;
const GL_POSITION                       = $1203;
const GL_SPOT_DIRECTION                 = $1204;
const GL_SPOT_EXPONENT                  = $1205;
const GL_SPOT_CUTOFF                    = $1206;
const GL_CONSTANT_ATTENUATION           = $1207;
const GL_LINEAR_ATTENUATION             = $1208;
const GL_QUADRATIC_ATTENUATION          = $1209;

{ InterleavedArrays }
{      GL_V2F }
{      GL_V3F }
{      GL_C4UB_V2F }
{      GL_C4UB_V3F }
{      GL_C3F_V3F }
{      GL_N3F_V3F }
{      GL_C4F_N3F_V3F }
{      GL_T2F_V3F }
{      GL_T4F_V4F }
{      GL_T2F_C4UB_V3F }
{      GL_T2F_C3F_V3F }
{      GL_T2F_N3F_V3F }
{      GL_T2F_C4F_N3F_V3F }
{      GL_T4F_C4F_N3F_V4F }

{ ListMode }
const GL_COMPILE                        = $1300;
const GL_COMPILE_AND_EXECUTE            = $1301;

{ ListNameType }
{      GL_BYTE }
{      GL_UNSIGNED_BYTE }
{      GL_SHORT }
{      GL_UNSIGNED_SHORT }
{      GL_INT }
{      GL_UNSIGNED_INT }
{      GL_FLOAT }
{      GL_2_BYTES }
{      GL_3_BYTES }
{      GL_4_BYTES }

{ LogicOp }
const GL_CLEAR                          = $1500;
const GL_AND                            = $1501;
const GL_AND_REVERSE                    = $1502;
const GL_COPY                           = $1503;
const GL_AND_INVERTED                   = $1504;
const GL_NOOP                           = $1505;
const GL_XOR                            = $1506;
const GL_OR                             = $1507;
const GL_NOR                            = $1508;
const GL_EQUIV                          = $1509;
const GL_INVERT                         = $150A;
const GL_OR_REVERSE                     = $150B;
const GL_COPY_INVERTED                  = $150C;
const GL_OR_INVERTED                    = $150D;
const GL_NAND                           = $150E;
const GL_SET                            = $150F;

{ MapTarget }
{      GL_MAP1_COLOR_4 }
{      GL_MAP1_INDEX }
{      GL_MAP1_NORMAL }
{      GL_MAP1_TEXTURE_COORD_1 }
{      GL_MAP1_TEXTURE_COORD_2 }
{      GL_MAP1_TEXTURE_COORD_3 }
{      GL_MAP1_TEXTURE_COORD_4 }
{      GL_MAP1_VERTEX_3 }
{      GL_MAP1_VERTEX_4 }
{      GL_MAP2_COLOR_4 }
{      GL_MAP2_INDEX }
{      GL_MAP2_NORMAL }
{      GL_MAP2_TEXTURE_COORD_1 }
{      GL_MAP2_TEXTURE_COORD_2 }
{      GL_MAP2_TEXTURE_COORD_3 }
{      GL_MAP2_TEXTURE_COORD_4 }
{      GL_MAP2_VERTEX_3 }
{      GL_MAP2_VERTEX_4 }

{ MaterialFace }
{      GL_FRONT }
{      GL_BACK }
{      GL_FRONT_AND_BACK }

{ MaterialParameter }
const GL_EMISSION                       = $1600;
const GL_SHININESS                      = $1601;
const GL_AMBIENT_AND_DIFFUSE            = $1602;
const GL_COLOR_INDEXES                  = $1603;
{      GL_AMBIENT }
{      GL_DIFFUSE }
{      GL_SPECULAR }

{ MatrixMode }
const GL_MODELVIEW                      = $1700;
const GL_PROJECTION                     = $1701;
const GL_TEXTURE                        = $1702;

{ MeshMode1 }
{      GL_POINT }
{      GL_LINE }

{ MeshMode2 }
{      GL_POINT }
{      GL_LINE }
{      GL_FILL }

{ MinmaxTarget }
{      GL_MINMAX }

{ NormalPointerType }
{      GL_BYTE }
{      GL_SHORT }
{      GL_INT }
{      GL_FLOAT }
{      GL_DOUBLE }

{ PixelCopyType }
const GL_COLOR                          = $1800;
const GL_DEPTH                          = $1801;
const GL_STENCIL                        = $1802;

{ PixelFormat }
const GL_COLOR_INDEX                    = $1900;
const GL_STENCIL_INDEX                  = $1901;
const GL_DEPTH_COMPONENT                = $1902;
const GL_RED                            = $1903;
const GL_GREEN                          = $1904;
const GL_BLUE                           = $1905;
const GL_ALPHA                          = $1906;
const GL_RGB                            = $1907;
const GL_RGBA                           = $1908;
const GL_LUMINANCE                      = $1909;
const GL_LUMINANCE_ALPHA                = $190A;
{      GL_ABGR }

{ PixelInternalFormat }
{      GL_ALPHA4 }
{      GL_ALPHA8 }
{      GL_ALPHA12 }
{      GL_ALPHA16 }
{      GL_LUMINANCE4 }
{      GL_LUMINANCE8 }
{      GL_LUMINANCE12 }
{      GL_LUMINANCE16 }
{      GL_LUMINANCE4_ALPHA4 }
{      GL_LUMINANCE6_ALPHA2 }
{      GL_LUMINANCE8_ALPHA8 }
{      GL_LUMINANCE12_ALPHA4 }
{      GL_LUMINANCE12_ALPHA12 }
{      GL_LUMINANCE16_ALPHA16 }
{      GL_INTENSITY }
{      GL_INTENSITY4 }
{      GL_INTENSITY8 }
{      GL_INTENSITY12 }
{      GL_INTENSITY16 }
{      GL_R3_G3_B2 }
{      GL_RGB4 }
{      GL_RGB5 }
{      GL_RGB8 }
{      GL_RGB10 }
{      GL_RGB12 }
{      GL_RGB16 }
{      GL_RGBA2 }
{      GL_RGBA4 }
{      GL_RGB5_A1 }
{      GL_RGBA8 }
{      GL_RGB10_A2 }
{      GL_RGBA12 }
{      GL_RGBA16 }

{ PixelMap }
{      GL_PIXEL_MAP_I_TO_I }
{      GL_PIXEL_MAP_S_TO_S }
{      GL_PIXEL_MAP_I_TO_R }
{      GL_PIXEL_MAP_I_TO_G }
{      GL_PIXEL_MAP_I_TO_B }
{      GL_PIXEL_MAP_I_TO_A }
{      GL_PIXEL_MAP_R_TO_R }
{      GL_PIXEL_MAP_G_TO_G }
{      GL_PIXEL_MAP_B_TO_B }
{      GL_PIXEL_MAP_A_TO_A }

{ PixelStore }
{      GL_UNPACK_SWAP_BYTES }
{      GL_UNPACK_LSB_FIRST }
{      GL_UNPACK_ROW_LENGTH }
{      GL_UNPACK_SKIP_ROWS }
{      GL_UNPACK_SKIP_PIXELS }
{      GL_UNPACK_ALIGNMENT }
{      GL_PACK_SWAP_BYTES }
{      GL_PACK_LSB_FIRST }
{      GL_PACK_ROW_LENGTH }
{      GL_PACK_SKIP_ROWS }
{      GL_PACK_SKIP_PIXELS }
{      GL_PACK_ALIGNMENT }
{      GL_PACK_SKIP_IMAGES }
{      GL_PACK_IMAGE_HEIGHT }
{      GL_UNPACK_SKIP_IMAGES }
{      GL_UNPACK_IMAGE_HEIGHT }

{ PixelTransfer }
{      GL_MAP_COLOR }
{      GL_MAP_STENCIL }
{      GL_INDEX_SHIFT }
{      GL_INDEX_OFFSET }
{      GL_RED_SCALE }
{      GL_RED_BIAS }
{      GL_GREEN_SCALE }
{      GL_GREEN_BIAS }
{      GL_BLUE_SCALE }
{      GL_BLUE_BIAS }
{      GL_ALPHA_SCALE }
{      GL_ALPHA_BIAS }
{      GL_DEPTH_SCALE }
{      GL_DEPTH_BIAS }
{      GL_POST_CONVOLUTION_RED_SCALE }
{      GL_POST_CONVOLUTION_GREEN_SCALE }
{      GL_POST_CONVOLUTION_BLUE_SCALE }
{      GL_POST_CONVOLUTION_ALPHA_SCALE }
{      GL_POST_CONVOLUTION_RED_BIAS }
{      GL_POST_CONVOLUTION_GREEN_BIAS }
{      GL_POST_CONVOLUTION_BLUE_BIAS }
{      GL_POST_CONVOLUTION_ALPHA_BIAS }
{      GL_POST_COLOR_MATRIX_RED_SCALE }
{      GL_POST_COLOR_MATRIX_GREEN_SCALE }
{      GL_POST_COLOR_MATRIX_BLUE_SCALE }
{      GL_POST_COLOR_MATRIX_ALPHA_SCALE }
{      GL_POST_COLOR_MATRIX_RED_BIAS }
{      GL_POST_COLOR_MATRIX_GREEN_BIAS }
{      GL_POST_COLOR_MATRIX_BLUE_BIAS }
{      GL_POST_COLOR_MATRIX_ALPHA_BIAS }

{ PixelType }
const GL_BITMAP                         = $1A00;
{      GL_BYTE }
{      GL_UNSIGNED_BYTE }
{      GL_SHORT }
{      GL_UNSIGNED_SHORT }
{      GL_INT }
{      GL_UNSIGNED_INT }
{      GL_FLOAT }
{      GL_BGR }
{      GL_BGRA }
{      GL_UNSIGNED_BYTE_3_3_2 }
{      GL_UNSIGNED_SHORT_4_4_4_4 }
{      GL_UNSIGNED_SHORT_5_5_5_1 }
{      GL_UNSIGNED_INT_8_8_8_8 }
{      GL_UNSIGNED_INT_10_10_10_2 }
{      GL_UNSIGNED_SHORT_5_6_5 }
{      GL_UNSIGNED_BYTE_2_3_3_REV }
{      GL_UNSIGNED_SHORT_5_6_5_REV }
{      GL_UNSIGNED_SHORT_4_4_4_4_REV }
{      GL_UNSIGNED_SHORT_1_5_5_5_REV }
{      GL_UNSIGNED_INT_8_8_8_8_REV }
{      GL_UNSIGNED_INT_2_10_10_10_REV }

{ PolygonMode }
const GL_POINT                          = $1B00;
const GL_LINE                           = $1B01;
const GL_FILL                           = $1B02;

{ ReadBufferMode }
{      GL_FRONT_LEFT }
{      GL_FRONT_RIGHT }
{      GL_BACK_LEFT }
{      GL_BACK_RIGHT }
{      GL_FRONT }
{      GL_BACK }
{      GL_LEFT }
{      GL_RIGHT }
{      GL_AUX0 }
{      GL_AUX1 }
{      GL_AUX2 }
{      GL_AUX3 }

{ RenderingMode }
const GL_RENDER                         = $1C00;
const GL_FEEDBACK                       = $1C01;
const GL_SELECT                         = $1C02;

{ SeparableTarget }
{      GL_SEPARABLE_2D }

{ ShadingModel }
const GL_FLAT                           = $1D00;
const GL_SMOOTH                         = $1D01;

{ StencilFunction }
{      GL_NEVER }
{      GL_LESS }
{      GL_EQUAL }
{      GL_LEQUAL }
{      GL_GREATER }
{      GL_NOTEQUAL }
{      GL_GEQUAL }
{      GL_ALWAYS }

{ StencilOp }
{      GL_ZERO }
const GL_KEEP                           = $1E00;
const GL_REPLACE                        = $1E01;
const GL_INCR                           = $1E02;
const GL_DECR                           = $1E03;
{      GL_INVERT }

{ StringName }
const GL_VENDOR                         = $1F00;
const GL_RENDERER                       = $1F01;
const GL_VERSION                        = $1F02;
const GL_EXTENSIONS                     = $1F03;

{ TextureCoordName }
const GL_S                              = $2000;
const GL_T                              = $2001;
const GL_R                              = $2002;
const GL_Q                              = $2003;

{ TexCoordPointerType }
{      GL_SHORT }
{      GL_INT }
{      GL_FLOAT }
{      GL_DOUBLE }

{ TextureEnvMode }
const GL_MODULATE                       = $2100;
const GL_DECAL                          = $2101;
{      GL_BLEND }
{      GL_REPLACE }

{ TextureEnvParameter }
const GL_TEXTURE_ENV_MODE               = $2200;
const GL_TEXTURE_ENV_COLOR              = $2201;

{ TextureEnvTarget }
const GL_TEXTURE_ENV                    = $2300;

{ TextureGenMode }
const GL_EYE_LINEAR                     = $2400;
const GL_OBJECT_LINEAR                  = $2401;
const GL_SPHERE_MAP                     = $2402;

{ TextureGenParameter }
const GL_TEXTURE_GEN_MODE               = $2500;
const GL_OBJECT_PLANE                   = $2501;
const GL_EYE_PLANE                      = $2502;

{ TextureMagFilter }
const GL_NEAREST                        = $2600;
const GL_LINEAR                         = $2601;

{ TextureMinFilter }
{      GL_NEAREST }
{      GL_LINEAR }
const GL_NEAREST_MIPMAP_NEAREST         = $2700;
const GL_LINEAR_MIPMAP_NEAREST          = $2701;
const GL_NEAREST_MIPMAP_LINEAR          = $2702;
const GL_LINEAR_MIPMAP_LINEAR           = $2703;

{ TextureParameterName }
const GL_TEXTURE_MAG_FILTER             = $2800;
const GL_TEXTURE_MIN_FILTER             = $2801;
const GL_TEXTURE_WRAP_S                 = $2802;
const GL_TEXTURE_WRAP_T                 = $2803;
{      GL_TEXTURE_BORDER_COLOR }
{      GL_TEXTURE_PRIORITY }
{      GL_TEXTURE_WRAP_R }
{      GL_TEXTURE_MIN_LOD }
{      GL_TEXTURE_MAX_LOD }
{      GL_TEXTURE_BASE_LEVEL }
{      GL_TEXTURE_MAX_LEVEL }

{ TextureTarget }
{      GL_TEXTURE_1D }
{      GL_TEXTURE_2D }
{      GL_PROXY_TEXTURE_1D }
{      GL_PROXY_TEXTURE_2D }
{      GL_TEXTURE_3D }
{      GL_PROXY_TEXTURE_3D }

{ TextureWrapMode }
const GL_CLAMP                          = $2900;
const GL_REPEAT                         = $2901;
{      GL_CLAMP_TO_EDGE }

{ VertexPointerType }
{      GL_SHORT }
{      GL_INT }
{      GL_FLOAT }
{      GL_DOUBLE }

{ ClientAttribMask }
const GL_CLIENT_PIXEL_STORE_BIT         = $00000001;
const GL_CLIENT_VERTEX_ARRAY_BIT        = $00000002;
const GL_CLIENT_ALL_ATTRIB_BITS         = $ffffffff;

{ polygon_offset }
const GL_POLYGON_OFFSET_FACTOR          = $8038;
const GL_POLYGON_OFFSET_UNITS           = $2A00;
const GL_POLYGON_OFFSET_POINT           = $2A01;
const GL_POLYGON_OFFSET_LINE            = $2A02;
const GL_POLYGON_OFFSET_FILL            = $8037;

{ texture }
const GL_ALPHA4                         = $803B;
const GL_ALPHA8                         = $803C;
const GL_ALPHA12                        = $803D;
const GL_ALPHA16                        = $803E;
const GL_LUMINANCE4                     = $803F;
const GL_LUMINANCE8                     = $8040;
const GL_LUMINANCE12                    = $8041;
const GL_LUMINANCE16                    = $8042;
const GL_LUMINANCE4_ALPHA4              = $8043;
const GL_LUMINANCE6_ALPHA2              = $8044;
const GL_LUMINANCE8_ALPHA8              = $8045;
const GL_LUMINANCE12_ALPHA4             = $8046;
const GL_LUMINANCE12_ALPHA12            = $8047;
const GL_LUMINANCE16_ALPHA16            = $8048;
const GL_INTENSITY                      = $8049;
const GL_INTENSITY4                     = $804A;
const GL_INTENSITY8                     = $804B;
const GL_INTENSITY12                    = $804C;
const GL_INTENSITY16                    = $804D;
const GL_R3_G3_B2                       = $2A10;
const GL_RGB4                           = $804F;
const GL_RGB5                           = $8050;
const GL_RGB8                           = $8051;
const GL_RGB10                          = $8052;
const GL_RGB12                          = $8053;
const GL_RGB16                          = $8054;
const GL_RGBA2                          = $8055;
const GL_RGBA4                          = $8056;
const GL_RGB5_A1                        = $8057;
const GL_RGBA8                          = $8058;
const GL_RGB10_A2                       = $8059;
const GL_RGBA12                         = $805A;
const GL_RGBA16                         = $805B;
const GL_TEXTURE_RED_SIZE               = $805C;
const GL_TEXTURE_GREEN_SIZE             = $805D;
const GL_TEXTURE_BLUE_SIZE              = $805E;
const GL_TEXTURE_ALPHA_SIZE             = $805F;
const GL_TEXTURE_LUMINANCE_SIZE         = $8060;
const GL_TEXTURE_INTENSITY_SIZE         = $8061;
const GL_PROXY_TEXTURE_1D               = $8063;
const GL_PROXY_TEXTURE_2D               = $8064;

{ texture_object }
const GL_TEXTURE_PRIORITY               = $8066;
const GL_TEXTURE_RESIDENT               = $8067;
const GL_TEXTURE_BINDING_1D             = $8068;
const GL_TEXTURE_BINDING_2D             = $8069;
const GL_TEXTURE_BINDING_3D             = $806A;

{ vertex_array }
const GL_VERTEX_ARRAY                   = $8074;
const GL_NORMAL_ARRAY                   = $8075;
const GL_COLOR_ARRAY                    = $8076;
const GL_INDEX_ARRAY                    = $8077;
const GL_TEXTURE_COORD_ARRAY            = $8078;
const GL_EDGE_FLAG_ARRAY                = $8079;
const GL_VERTEX_ARRAY_SIZE              = $807A;
const GL_VERTEX_ARRAY_TYPE              = $807B;
const GL_VERTEX_ARRAY_STRIDE            = $807C;
const GL_NORMAL_ARRAY_TYPE              = $807E;
const GL_NORMAL_ARRAY_STRIDE            = $807F;
const GL_COLOR_ARRAY_SIZE               = $8081;
const GL_COLOR_ARRAY_TYPE               = $8082;
const GL_COLOR_ARRAY_STRIDE             = $8083;
const GL_INDEX_ARRAY_TYPE               = $8085;
const GL_INDEX_ARRAY_STRIDE             = $8086;
const GL_TEXTURE_COORD_ARRAY_SIZE       = $8088;
const GL_TEXTURE_COORD_ARRAY_TYPE       = $8089;
const GL_TEXTURE_COORD_ARRAY_STRIDE     = $808A;
const GL_EDGE_FLAG_ARRAY_STRIDE         = $808C;
const GL_VERTEX_ARRAY_POINTER           = $808E;
const GL_NORMAL_ARRAY_POINTER           = $808F;
const GL_COLOR_ARRAY_POINTER            = $8090;
const GL_INDEX_ARRAY_POINTER            = $8091;
const GL_TEXTURE_COORD_ARRAY_POINTER    = $8092;
const GL_EDGE_FLAG_ARRAY_POINTER        = $8093;
const GL_V2F                            = $2A20;
const GL_V3F                            = $2A21;
const GL_C4UB_V2F                       = $2A22;
const GL_C4UB_V3F                       = $2A23;
const GL_C3F_V3F                        = $2A24;
const GL_N3F_V3F                        = $2A25;
const GL_C4F_N3F_V3F                    = $2A26;
const GL_T2F_V3F                        = $2A27;
const GL_T4F_V4F                        = $2A28;
const GL_T2F_C4UB_V3F                   = $2A29;
const GL_T2F_C3F_V3F                    = $2A2A;
const GL_T2F_N3F_V3F                    = $2A2B;
const GL_T2F_C4F_N3F_V3F                = $2A2C;
const GL_T4F_C4F_N3F_V4F                = $2A2D;

{ bgra }
const GL_BGR                            = $80E0;
const GL_BGRA                           = $80E1;

{ blend_color }
const GL_CONSTANT_COLOR                 = $8001;
const GL_ONE_MINUS_CONSTANT_COLOR       = $8002;
const GL_CONSTANT_ALPHA                 = $8003;
const GL_ONE_MINUS_CONSTANT_ALPHA       = $8004;
const GL_BLEND_COLOR                    = $8005;

{ blend_minmax }
const GL_FUNC_ADD                       = $8006;
const GL_MIN                            = $8007;
const GL_MAX                            = $8008;
const GL_BLEND_EQUATION                 = $8009;

{ blend_equation_separate }
const GL_BLEND_EQUATION_RGB             = $8009;
const GL_BLEND_EQUATION_ALPHA           = $883D;

{ blend_subtract }
const GL_FUNC_SUBTRACT                  = $800A;
const GL_FUNC_REVERSE_SUBTRACT          = $800B;

{ color_matrix }
const GL_COLOR_MATRIX                   = $80B1;
const GL_COLOR_MATRIX_STACK_DEPTH       = $80B2;
const GL_MAX_COLOR_MATRIX_STACK_DEPTH   = $80B3;
const GL_POST_COLOR_MATRIX_RED_SCALE    = $80B4;
const GL_POST_COLOR_MATRIX_GREEN_SCALE  = $80B5;
const GL_POST_COLOR_MATRIX_BLUE_SCALE   = $80B6;
const GL_POST_COLOR_MATRIX_ALPHA_SCALE  = $80B7;
const GL_POST_COLOR_MATRIX_RED_BIAS     = $80B8;
const GL_POST_COLOR_MATRIX_GREEN_BIAS   = $80B9;
const GL_POST_COLOR_MATRIX_BLUE_BIAS    = $80BA;
const GL_POST_COLOR_MATRIX_ALPHA_BIAS   = $80BB;

{ color_table }
const GL_COLOR_TABLE                    = $80D0;
const GL_POST_CONVOLUTION_COLOR_TABLE   = $80D1;
const GL_POST_COLOR_MATRIX_COLOR_TABLE  = $80D2;
const GL_PROXY_COLOR_TABLE              = $80D3;
const GL_PROXY_POST_CONVOLUTION_COLOR_TABLE = $80D4;
const GL_PROXY_POST_COLOR_MATRIX_COLOR_TABLE = $80D5;
const GL_COLOR_TABLE_SCALE              = $80D6;
const GL_COLOR_TABLE_BIAS               = $80D7;
const GL_COLOR_TABLE_FORMAT             = $80D8;
const GL_COLOR_TABLE_WIDTH              = $80D9;
const GL_COLOR_TABLE_RED_SIZE           = $80DA;
const GL_COLOR_TABLE_GREEN_SIZE         = $80DB;
const GL_COLOR_TABLE_BLUE_SIZE          = $80DC;
const GL_COLOR_TABLE_ALPHA_SIZE         = $80DD;
const GL_COLOR_TABLE_LUMINANCE_SIZE     = $80DE;
const GL_COLOR_TABLE_INTENSITY_SIZE     = $80DF;

{ convolution }
const GL_CONVOLUTION_1D                 = $8010;
const GL_CONVOLUTION_2D                 = $8011;
const GL_SEPARABLE_2D                   = $8012;
const GL_CONVOLUTION_BORDER_MODE        = $8013;
const GL_CONVOLUTION_FILTER_SCALE       = $8014;
const GL_CONVOLUTION_FILTER_BIAS        = $8015;
const GL_REDUCE                         = $8016;
const GL_CONVOLUTION_FORMAT             = $8017;
const GL_CONVOLUTION_WIDTH              = $8018;
const GL_CONVOLUTION_HEIGHT             = $8019;
const GL_MAX_CONVOLUTION_WIDTH          = $801A;
const GL_MAX_CONVOLUTION_HEIGHT         = $801B;
const GL_POST_CONVOLUTION_RED_SCALE     = $801C;
const GL_POST_CONVOLUTION_GREEN_SCALE   = $801D;
const GL_POST_CONVOLUTION_BLUE_SCALE    = $801E;
const GL_POST_CONVOLUTION_ALPHA_SCALE   = $801F;
const GL_POST_CONVOLUTION_RED_BIAS      = $8020;
const GL_POST_CONVOLUTION_GREEN_BIAS    = $8021;
const GL_POST_CONVOLUTION_BLUE_BIAS     = $8022;
const GL_POST_CONVOLUTION_ALPHA_BIAS    = $8023;
const GL_CONSTANT_BORDER                = $8151;
const GL_REPLICATE_BORDER               = $8153;
const GL_CONVOLUTION_BORDER_COLOR       = $8154;

{ draw_range_elements }
const GL_MAX_ELEMENTS_VERTICES          = $80E8;
const GL_MAX_ELEMENTS_INDICES           = $80E9;

{ histogram }
const GL_HISTOGRAM                      = $8024;
const GL_PROXY_HISTOGRAM                = $8025;
const GL_HISTOGRAM_WIDTH                = $8026;
const GL_HISTOGRAM_FORMAT               = $8027;
const GL_HISTOGRAM_RED_SIZE             = $8028;
const GL_HISTOGRAM_GREEN_SIZE           = $8029;
const GL_HISTOGRAM_BLUE_SIZE            = $802A;
const GL_HISTOGRAM_ALPHA_SIZE           = $802B;
const GL_HISTOGRAM_LUMINANCE_SIZE       = $802C;
const GL_HISTOGRAM_SINK                 = $802D;
const GL_MINMAX                         = $802E;
const GL_MINMAX_FORMAT                  = $802F;
const GL_MINMAX_SINK                    = $8030;
const GL_TABLE_TOO_LARGE                = $8031;

{ packed_pixels }
const GL_UNSIGNED_BYTE_3_3_2            = $8032;
const GL_UNSIGNED_SHORT_4_4_4_4         = $8033;
const GL_UNSIGNED_SHORT_5_5_5_1         = $8034;
const GL_UNSIGNED_INT_8_8_8_8           = $8035;
const GL_UNSIGNED_INT_10_10_10_2        = $8036;
const GL_UNSIGNED_BYTE_2_3_3_REV        = $8362;
const GL_UNSIGNED_SHORT_5_6_5           = $8363;
const GL_UNSIGNED_SHORT_5_6_5_REV       = $8364;
const GL_UNSIGNED_SHORT_4_4_4_4_REV     = $8365;
const GL_UNSIGNED_SHORT_1_5_5_5_REV     = $8366;
const GL_UNSIGNED_INT_8_8_8_8_REV       = $8367;
const GL_UNSIGNED_INT_2_10_10_10_REV    = $8368;

{ rescale_normal }
const GL_RESCALE_NORMAL                 = $803A;

{ separate_specular_color }
const GL_LIGHT_MODEL_COLOR_CONTROL      = $81F8;
const GL_SINGLE_COLOR                   = $81F9;
const GL_SEPARATE_SPECULAR_COLOR        = $81FA;

{ texture3D }
const GL_PACK_SKIP_IMAGES               = $806B;
const GL_PACK_IMAGE_HEIGHT              = $806C;
const GL_UNPACK_SKIP_IMAGES             = $806D;
const GL_UNPACK_IMAGE_HEIGHT            = $806E;
const GL_TEXTURE_3D                     = $806F;
const GL_PROXY_TEXTURE_3D               = $8070;
const GL_TEXTURE_DEPTH                  = $8071;
const GL_TEXTURE_WRAP_R                 = $8072;
const GL_MAX_3D_TEXTURE_SIZE            = $8073;

{ texture_edge_clamp }
const GL_CLAMP_TO_EDGE                  = $812F;
const GL_CLAMP_TO_BORDER                = $812D;

{ texture_lod }
const GL_TEXTURE_MIN_LOD                = $813A;
const GL_TEXTURE_MAX_LOD                = $813B;
const GL_TEXTURE_BASE_LEVEL             = $813C;
const GL_TEXTURE_MAX_LEVEL              = $813D;

{ GetTarget1_2 }
const GL_SMOOTH_POINT_SIZE_RANGE        = $0B12;
const GL_SMOOTH_POINT_SIZE_GRANULARITY  = $0B13;
const GL_SMOOTH_LINE_WIDTH_RANGE        = $0B22;
const GL_SMOOTH_LINE_WIDTH_GRANULARITY  = $0B23;
const GL_ALIASED_POINT_SIZE_RANGE       = $846D;
const GL_ALIASED_LINE_WIDTH_RANGE       = $846E;

const GL_TEXTURE0                       = $84C0;
const GL_TEXTURE1                       = $84C1;
const GL_TEXTURE2                       = $84C2;
const GL_TEXTURE3                       = $84C3;
const GL_TEXTURE4                       = $84C4;
const GL_TEXTURE5                       = $84C5;
const GL_TEXTURE6                       = $84C6;
const GL_TEXTURE7                       = $84C7;
const GL_TEXTURE8                       = $84C8;
const GL_TEXTURE9                       = $84C9;
const GL_TEXTURE10                      = $84CA;
const GL_TEXTURE11                      = $84CB;
const GL_TEXTURE12                      = $84CC;
const GL_TEXTURE13                      = $84CD;
const GL_TEXTURE14                      = $84CE;
const GL_TEXTURE15                      = $84CF;
const GL_TEXTURE16                      = $84D0;
const GL_TEXTURE17                      = $84D1;
const GL_TEXTURE18                      = $84D2;
const GL_TEXTURE19                      = $84D3;
const GL_TEXTURE20                      = $84D4;
const GL_TEXTURE21                      = $84D5;
const GL_TEXTURE22                      = $84D6;
const GL_TEXTURE23                      = $84D7;
const GL_TEXTURE24                      = $84D8;
const GL_TEXTURE25                      = $84D9;
const GL_TEXTURE26                      = $84DA;
const GL_TEXTURE27                      = $84DB;
const GL_TEXTURE28                      = $84DC;
const GL_TEXTURE29                      = $84DD;
const GL_TEXTURE30                      = $84DE;
const GL_TEXTURE31                      = $84DF;
const GL_ACTIVE_TEXTURE                 = $84E0;
const GL_CLIENT_ACTIVE_TEXTURE          = $84E1;
const GL_MAX_TEXTURE_UNITS              = $84E2;

const GL_COMBINE                        = $8570;
const GL_COMBINE_RGB                    = $8571;
const GL_COMBINE_ALPHA                  = $8572;
const GL_RGB_SCALE                      = $8573;
const GL_ADD_SIGNED                     = $8574;
const GL_INTERPOLATE                    = $8575;
const GL_CONSTANT                       = $8576;
const GL_PRIMARY_COLOR                  = $8577;
const GL_PREVIOUS                       = $8578;
const GL_SUBTRACT                       = $84E7;

const GL_SRC0_RGB                       = $8580;
const GL_SRC1_RGB                       = $8581;
const GL_SRC2_RGB                       = $8582;
const GL_SRC3_RGB                       = $8583;
const GL_SRC4_RGB                       = $8584;
const GL_SRC5_RGB                       = $8585;
const GL_SRC6_RGB                       = $8586;
const GL_SRC7_RGB                       = $8587;
const GL_SRC0_ALPHA                     = $8588;
const GL_SRC1_ALPHA                     = $8589;
const GL_SRC2_ALPHA                     = $858A;
const GL_SRC3_ALPHA                     = $858B;
const GL_SRC4_ALPHA                     = $858C;
const GL_SRC5_ALPHA                     = $858D;
const GL_SRC6_ALPHA                     = $858E;
const GL_SRC7_ALPHA                     = $858F;

{ Obsolete }
const GL_SOURCE0_RGB                    = $8580;
const GL_SOURCE1_RGB                    = $8581;
const GL_SOURCE2_RGB                    = $8582;
const GL_SOURCE3_RGB                    = $8583;
const GL_SOURCE4_RGB                    = $8584;
const GL_SOURCE5_RGB                    = $8585;
const GL_SOURCE6_RGB                    = $8586;
const GL_SOURCE7_RGB                    = $8587;
const GL_SOURCE0_ALPHA                  = $8588;
const GL_SOURCE1_ALPHA                  = $8589;
const GL_SOURCE2_ALPHA                  = $858A;
const GL_SOURCE3_ALPHA                  = $858B;
const GL_SOURCE4_ALPHA                  = $858C;
const GL_SOURCE5_ALPHA                  = $858D;
const GL_SOURCE6_ALPHA                  = $858E;
const GL_SOURCE7_ALPHA                  = $858F;

const GL_OPERAND0_RGB                   = $8590;
const GL_OPERAND1_RGB                   = $8591;
const GL_OPERAND2_RGB                   = $8592;
const GL_OPERAND3_RGB                   = $8593;
const GL_OPERAND4_RGB                   = $8594;
const GL_OPERAND5_RGB                   = $8595;
const GL_OPERAND6_RGB                   = $8596;
const GL_OPERAND7_RGB                   = $8597;
const GL_OPERAND0_ALPHA                 = $8598;
const GL_OPERAND1_ALPHA                 = $8599;
const GL_OPERAND2_ALPHA                 = $859A;
const GL_OPERAND3_ALPHA                 = $859B;
const GL_OPERAND4_ALPHA                 = $859C;
const GL_OPERAND5_ALPHA                 = $859D;
const GL_OPERAND6_ALPHA                 = $859E;
const GL_OPERAND7_ALPHA                 = $859F;

const GL_DOT3_RGB                       = $86AE;
const GL_DOT3_RGBA                      = $86AF;

const GL_TRANSPOSE_MODELVIEW_MATRIX     = $84E3;
const GL_TRANSPOSE_PROJECTION_MATRIX    = $84E4;
const GL_TRANSPOSE_TEXTURE_MATRIX       = $84E5;
const GL_TRANSPOSE_COLOR_MATRIX         = $84E6;

const GL_NORMAL_MAP                     = $8511;
const GL_REFLECTION_MAP                 = $8512;
const GL_TEXTURE_CUBE_MAP               = $8513;
const GL_TEXTURE_BINDING_CUBE_MAP       = $8514;
const GL_TEXTURE_CUBE_MAP_POSITIVE_X    = $8515;
const GL_TEXTURE_CUBE_MAP_NEGATIVE_X    = $8516;
const GL_TEXTURE_CUBE_MAP_POSITIVE_Y    = $8517;
const GL_TEXTURE_CUBE_MAP_NEGATIVE_Y    = $8518;
const GL_TEXTURE_CUBE_MAP_POSITIVE_Z    = $8519;
const GL_TEXTURE_CUBE_MAP_NEGATIVE_Z    = $851A;
const GL_PROXY_TEXTURE_CUBE_MAP         = $851B;
const GL_MAX_CUBE_MAP_TEXTURE_SIZE      = $851C;

const GL_COMPRESSED_ALPHA               = $84E9;
const GL_COMPRESSED_LUMINANCE           = $84EA;
const GL_COMPRESSED_LUMINANCE_ALPHA     = $84EB;
const GL_COMPRESSED_INTENSITY           = $84EC;
const GL_COMPRESSED_RGB                 = $84ED;
const GL_COMPRESSED_RGBA                = $84EE;
const GL_TEXTURE_COMPRESSION_HINT       = $84EF;
const GL_TEXTURE_COMPRESSED_IMAGE_SIZE  = $86A0;
const GL_TEXTURE_COMPRESSED             = $86A1;
const GL_NUM_COMPRESSED_TEXTURE_FORMATS = $86A2;
const GL_COMPRESSED_TEXTURE_FORMATS     = $86A3;

const GL_MULTISAMPLE                    = $809D;
const GL_SAMPLE_ALPHA_TO_COVERAGE       = $809E;
const GL_SAMPLE_ALPHA_TO_ONE            = $809F;
const GL_SAMPLE_COVERAGE                = $80A0;
const GL_SAMPLE_BUFFERS                 = $80A8;
const GL_SAMPLES                        = $80A9;
const GL_SAMPLE_COVERAGE_VALUE          = $80AA;
const GL_SAMPLE_COVERAGE_INVERT         = $80AB;
const GL_MULTISAMPLE_BIT                = $20000000;

const GL_DEPTH_COMPONENT16              = $81A5;
const GL_DEPTH_COMPONENT24              = $81A6;
const GL_DEPTH_COMPONENT32              = $81A7;
const GL_TEXTURE_DEPTH_SIZE             = $884A;
const GL_DEPTH_TEXTURE_MODE             = $884B;

const GL_TEXTURE_COMPARE_MODE           = $884C;
const GL_TEXTURE_COMPARE_FUNC           = $884D;
const GL_COMPARE_R_TO_TEXTURE           = $884E;

{ occlusion_query }
const GL_QUERY_COUNTER_BITS             = $8864;
const GL_CURRENT_QUERY                  = $8865;
const GL_QUERY_RESULT                   = $8866;
const GL_QUERY_RESULT_AVAILABLE         = $8867;
const GL_SAMPLES_PASSED                 = $8914;

const GL_FOG_COORD_SRC                  = $8450;
const GL_FOG_COORD                      = $8451;
const GL_FRAGMENT_DEPTH                 = $8452;
const GL_CURRENT_FOG_COORD              = $8453;  
const GL_FOG_COORD_ARRAY_TYPE           = $8454;
const GL_FOG_COORD_ARRAY_STRIDE         = $8455;
const GL_FOG_COORD_ARRAY_POINTER        = $8456;
const GL_FOG_COORD_ARRAY                = $8457;

{ Obsolete }
const GL_FOG_COORDINATE_SOURCE          = $8450;
const GL_FOG_COORDINATE                 = $8451;
const GL_CURRENT_FOG_COORDINATE         = $8453;  
const GL_FOG_COORDINATE_ARRAY_TYPE      = $8454;
const GL_FOG_COORDINATE_ARRAY_STRIDE    = $8455;
const GL_FOG_COORDINATE_ARRAY_POINTER   = $8456;
const GL_FOG_COORDINATE_ARRAY           = $8457;

const GL_COLOR_SUM                      = $8458;
const GL_CURRENT_SECONDARY_COLOR        = $8459;
const GL_SECONDARY_COLOR_ARRAY_SIZE     = $845A;
const GL_SECONDARY_COLOR_ARRAY_TYPE     = $845B;
const GL_SECONDARY_COLOR_ARRAY_STRIDE   = $845C;
const GL_SECONDARY_COLOR_ARRAY_POINTER  = $845D;
const GL_SECONDARY_COLOR_ARRAY          = $845E;

const GL_POINT_SIZE_MIN                 = $8126;
const GL_POINT_SIZE_MAX                 = $8127;
const GL_POINT_FADE_THRESHOLD_SIZE      = $8128;
const GL_POINT_DISTANCE_ATTENUATION     = $8129;

const GL_BLEND_DST_RGB                  = $80C8;
const GL_BLEND_SRC_RGB                  = $80C9;
const GL_BLEND_DST_ALPHA                = $80CA;
const GL_BLEND_SRC_ALPHA                = $80CB;

const GL_GENERATE_MIPMAP                = $8191;
const GL_GENERATE_MIPMAP_HINT           = $8192;

const GL_INCR_WRAP                      = $8507;
const GL_DECR_WRAP                      = $8508;

const GL_MIRRORED_REPEAT                = $8370;

const GL_MAX_TEXTURE_LOD_BIAS           = $84FD;
const GL_TEXTURE_FILTER_CONTROL         = $8500;
const GL_TEXTURE_LOD_BIAS               = $8501;

{ vertex_buffer_object }
const GL_ARRAY_BUFFER                                = $8892;
const GL_ELEMENT_ARRAY_BUFFER                        = $8893;
const GL_ARRAY_BUFFER_BINDING                        = $8894;
const GL_ELEMENT_ARRAY_BUFFER_BINDING                = $8895;
const GL_VERTEX_ARRAY_BUFFER_BINDING                 = $8896;
const GL_NORMAL_ARRAY_BUFFER_BINDING                 = $8897;
const GL_COLOR_ARRAY_BUFFER_BINDING                  = $8898;
const GL_INDEX_ARRAY_BUFFER_BINDING                  = $8899;
const GL_TEXTURE_COORD_ARRAY_BUFFER_BINDING          = $889A;
const GL_EDGE_FLAG_ARRAY_BUFFER_BINDING              = $889B;
const GL_SECONDARY_COLOR_ARRAY_BUFFER_BINDING        = $889C;
const GL_FOG_COORD_ARRAY_BUFFER_BINDING              = $889D;
const GL_WEIGHT_ARRAY_BUFFER_BINDING                 = $889E;
const GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING          = $889F;
const GL_STREAM_DRAW                                 = $88E0;
const GL_STREAM_READ                                 = $88E1;
const GL_STREAM_COPY                                 = $88E2;
const GL_STATIC_DRAW                                 = $88E4;
const GL_STATIC_READ                                 = $88E5;
const GL_STATIC_COPY                                 = $88E6;
const GL_DYNAMIC_DRAW                                = $88E8;
const GL_DYNAMIC_READ                                = $88E9;
const GL_DYNAMIC_COPY                                = $88EA;
const GL_READ_ONLY                                   = $88B8;
const GL_WRITE_ONLY                                  = $88B9;
const GL_READ_WRITE                                  = $88BA;
const GL_BUFFER_SIZE                                 = $8764;
const GL_BUFFER_USAGE                                = $8765;
const GL_BUFFER_ACCESS                               = $88BB;
const GL_BUFFER_MAPPED                               = $88BC;
const GL_BUFFER_MAP_POINTER                          = $88BD;
{ Obsolete }
const GL_FOG_COORDINATE_ARRAY_BUFFER_BINDING         = $889D;

{ OpenGL 2.0 }
const GL_CURRENT_PROGRAM                = $8B8D;
const GL_SHADER_TYPE                    = $8B4F;
const GL_DELETE_STATUS                  = $8B80;
const GL_COMPILE_STATUS                 = $8B81;
const GL_LINK_STATUS                    = $8B82;
const GL_VALIDATE_STATUS                = $8B83;
const GL_INFO_LOG_LENGTH                = $8B84;
const GL_ATTACHED_SHADERS               = $8B85;
const GL_ACTIVE_UNIFORMS                = $8B86;
const GL_ACTIVE_UNIFORM_MAX_LENGTH      = $8B87;
const GL_SHADER_SOURCE_LENGTH           = $8B88;
const GL_FLOAT_VEC2                     = $8B50;
const GL_FLOAT_VEC3                     = $8B51;
const GL_FLOAT_VEC4                     = $8B52;
const GL_INT_VEC2                       = $8B53;
const GL_INT_VEC3                       = $8B54;
const GL_INT_VEC4                       = $8B55;
const GL_BOOL                           = $8B56;
const GL_BOOL_VEC2                      = $8B57;
const GL_BOOL_VEC3                      = $8B58;
const GL_BOOL_VEC4                      = $8B59;
const GL_FLOAT_MAT2                     = $8B5A;
const GL_FLOAT_MAT3                     = $8B5B;
const GL_FLOAT_MAT4                     = $8B5C;
const GL_SAMPLER_1D                     = $8B5D;
const GL_SAMPLER_2D                     = $8B5E;
const GL_SAMPLER_3D                     = $8B5F;
const GL_SAMPLER_CUBE                   = $8B60;
const GL_SAMPLER_1D_SHADOW              = $8B61;
const GL_SAMPLER_2D_SHADOW              = $8B62;
const GL_SHADING_LANGUAGE_VERSION       = $8B8C;
const GL_VERTEX_SHADER                  = $8B31;
const GL_MAX_VERTEX_UNIFORM_COMPONENTS  = $8B4A;
const GL_MAX_VARYING_FLOATS             = $8B4B;
const GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS = $8B4C;
const GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS = $8B4D;
const GL_ACTIVE_ATTRIBUTES              = $8B89;
const GL_ACTIVE_ATTRIBUTE_MAX_LENGTH    = $8B8A;
const GL_FRAGMENT_SHADER                = $8B30;
const GL_MAX_FRAGMENT_UNIFORM_COMPONENTS = $8B49;
const GL_FRAGMENT_SHADER_DERIVATIVE_HINT = $8B8B;
const GL_MAX_VERTEX_ATTRIBS             = $8869;
const GL_VERTEX_ATTRIB_ARRAY_ENABLED    = $8622;
const GL_VERTEX_ATTRIB_ARRAY_SIZE       = $8623;
const GL_VERTEX_ATTRIB_ARRAY_STRIDE     = $8624;
const GL_VERTEX_ATTRIB_ARRAY_TYPE       = $8625;
const GL_VERTEX_ATTRIB_ARRAY_NORMALIZED = $886A;
const GL_CURRENT_VERTEX_ATTRIB          = $8626;
const GL_VERTEX_ATTRIB_ARRAY_POINTER    = $8645;
const GL_VERTEX_PROGRAM_POINT_SIZE      = $8642;
const GL_VERTEX_PROGRAM_TWO_SIDE        = $8643;
const GL_MAX_TEXTURE_COORDS             = $8871;
const GL_MAX_TEXTURE_IMAGE_UNITS        = $8872;
const GL_MAX_DRAW_BUFFERS               = $8824;
const GL_DRAW_BUFFER0                   = $8825;
const GL_DRAW_BUFFER1                   = $8826;
const GL_DRAW_BUFFER2                   = $8827;
const GL_DRAW_BUFFER3                   = $8828;
const GL_DRAW_BUFFER4                   = $8829;
const GL_DRAW_BUFFER5                   = $882A;
const GL_DRAW_BUFFER6                   = $882B;
const GL_DRAW_BUFFER7                   = $882C;
const GL_DRAW_BUFFER8                   = $882D;
const GL_DRAW_BUFFER9                   = $882E;
const GL_DRAW_BUFFER10                  = $882F;
const GL_DRAW_BUFFER11                  = $8830;
const GL_DRAW_BUFFER12                  = $8831;
const GL_DRAW_BUFFER13                  = $8832;
const GL_DRAW_BUFFER14                  = $8833;
const GL_DRAW_BUFFER15                  = $8834;
const GL_POINT_SPRITE                   = $8861;
const GL_COORD_REPLACE                  = $8862;
const GL_POINT_SPRITE_COORD_ORIGIN      = $8CA0;
const GL_LOWER_LEFT                     = $8CA1;
const GL_UPPER_LEFT                     = $8CA2;
const GL_STENCIL_BACK_FUNC              = $8800;
const GL_STENCIL_BACK_VALUE_MASK        = $8CA4;
const GL_STENCIL_BACK_REF               = $8CA3;
const GL_STENCIL_BACK_FAIL              = $8801;
const GL_STENCIL_BACK_PASS_DEPTH_FAIL   = $8802;
const GL_STENCIL_BACK_PASS_DEPTH_PASS   = $8803;
const GL_STENCIL_BACK_WRITEMASK         = $8CA5;

{ OpenGL 2.1 }
const GL_CURRENT_RASTER_SECONDARY_COLOR = $845F;
const GL_PIXEL_PACK_BUFFER              = $88EB;
const GL_PIXEL_UNPACK_BUFFER            = $88EC;
const GL_PIXEL_PACK_BUFFER_BINDING      = $88ED;
const GL_PIXEL_UNPACK_BUFFER_BINDING    = $88EF;
const GL_FLOAT_MAT2x3                   = $8B65;
const GL_FLOAT_MAT2x4                   = $8B66;
const GL_FLOAT_MAT3x2                   = $8B67;
const GL_FLOAT_MAT3x4                   = $8B68;
const GL_FLOAT_MAT4x2                   = $8B69;
const GL_FLOAT_MAT4x3                   = $8B6A;
const GL_SRGB                           = $8C40;
const GL_SRGB8                          = $8C41;
const GL_SRGB_ALPHA                     = $8C42;
const GL_SRGB8_ALPHA8                   = $8C43;
const GL_SLUMINANCE_ALPHA               = $8C44;
const GL_SLUMINANCE8_ALPHA8             = $8C45;
const GL_SLUMINANCE                     = $8C46;
const GL_SLUMINANCE8                    = $8C47;
const GL_COMPRESSED_SRGB                = $8C48;
const GL_COMPRESSED_SRGB_ALPHA          = $8C49;
const GL_COMPRESSED_SLUMINANCE          = $8C4A;
const GL_COMPRESSED_SLUMINANCE_ALPHA    = $8C4B;

{***********************************************************}

{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glAccumProcPtr = procedure( op: GLenum; value: GLfloat );
	glAlphaFuncProcPtr = procedure( func: GLenum; ref: GLclampf );
	glAreTexturesResidentProcPtr = function( n: GLsizei; const textures: PGLuint; residences: PGLboolean ): GLboolean;
	glArrayElementProcPtr = procedure( i: GLint );
	glBeginProcPtr = procedure( mode: GLenum );
	glBindTextureProcPtr = procedure( target: GLenum; texture: GLuint );
	glBitmapProcPtr = procedure( width: GLsizei; height: GLsizei; xorig: GLfloat; yorig: GLfloat; xmove: GLfloat; ymove: GLfloat; const bitmap: PGLubyte );
	glBlendColorProcPtr = procedure( red: GLclampf; green: GLclampf; blue: GLclampf; alpha: GLclampf );
	glBlendEquationProcPtr = procedure( mode: GLenum );
	glBlendEquationSeparateProcPtr = procedure( modeRGB: GLenum; modeAlpha: GLenum );
	glBlendFuncProcPtr = procedure( sfactor: GLenum; dfactor: GLenum );
	glCallListProcPtr = procedure( list: GLuint );
	glCallListsProcPtr = procedure( n: GLsizei; typ: GLenum; lists: univ ptr );
	glClearProcPtr = procedure( mask: GLbitfield );
	glClearAccumProcPtr = procedure( red: GLfloat; green: GLfloat; blue: GLfloat; alpha: GLfloat );
	glClearColorProcPtr = procedure( red: GLclampf; green: GLclampf; blue: GLclampf; alpha: GLclampf );
	glClearDepthProcPtr = procedure( depth: GLclampd );
	glClearIndexProcPtr = procedure( c: GLfloat );
	glClearStencilProcPtr = procedure( s: GLint );
	glClipPlaneProcPtr = procedure( plane: GLenum; const equation: PGLdouble );
	glColor3bProcPtr = procedure( red: GLbyte; green: GLbyte; blue: GLbyte );
	glColor3bvProcPtr = procedure( const v: PGLbyte );
	glColor3dProcPtr = procedure( red: GLdouble; green: GLdouble; blue: GLdouble );
	glColor3dvProcPtr = procedure( const v: PGLdouble );
	glColor3fProcPtr = procedure( red: GLfloat; green: GLfloat; blue: GLfloat );
	glColor3fvProcPtr = procedure( const v: PGLfloat );
	glColor3iProcPtr = procedure( red: GLint; green: GLint; blue: GLint );
	glColor3ivProcPtr = procedure( const v: PGLint );
	glColor3sProcPtr = procedure( red: GLshort; green: GLshort; blue: GLshort );
	glColor3svProcPtr = procedure( const v: PGLshort );
	glColor3ubProcPtr = procedure( red: GLubyte; green: GLubyte; blue: GLubyte );
	glColor3ubvProcPtr = procedure( const v: PGLubyte );
	glColor3uiProcPtr = procedure( red: GLuint; green: GLuint; blue: GLuint );
	glColor3uivProcPtr = procedure( const v: PGLuint );
	glColor3usProcPtr = procedure( red: GLushort; green: GLushort; blue: GLushort );
	glColor3usvProcPtr = procedure( const v: PGLushort );
	glColor4bProcPtr = procedure( red: GLbyte; green: GLbyte; blue: GLbyte; alpha: GLbyte );
	glColor4bvProcPtr = procedure( const v: PGLbyte );
	glColor4dProcPtr = procedure( red: GLdouble; green: GLdouble; blue: GLdouble; alpha: GLdouble );
	glColor4dvProcPtr = procedure( const v: PGLdouble );
	glColor4fProcPtr = procedure( red: GLfloat; green: GLfloat; blue: GLfloat; alpha: GLfloat );
	glColor4fvProcPtr = procedure( const v: PGLfloat );
	glColor4iProcPtr = procedure( red: GLint; green: GLint; blue: GLint; alpha: GLint );
	glColor4ivProcPtr = procedure( const v: PGLint );
	glColor4sProcPtr = procedure( red: GLshort; green: GLshort; blue: GLshort; alpha: GLshort );
	glColor4svProcPtr = procedure( const v: PGLshort );
	glColor4ubProcPtr = procedure( red: GLubyte; green: GLubyte; blue: GLubyte; alpha: GLubyte );
	glColor4ubvProcPtr = procedure( const v: PGLubyte );
	glColor4uiProcPtr = procedure( red: GLuint; green: GLuint; blue: GLuint; alpha: GLuint );
	glColor4uivProcPtr = procedure( const v: PGLuint );
	glColor4usProcPtr = procedure( red: GLushort; green: GLushort; blue: GLushort; alpha: GLushort );
	glColor4usvProcPtr = procedure( const v: PGLushort );
	glColorMaskProcPtr = procedure( red: GLboolean; green: GLboolean; blue: GLboolean; alpha: GLboolean );
	glColorMaterialProcPtr = procedure( face: GLenum; mode: GLenum );
	glColorPointerProcPtr = procedure( size: GLint; typ: GLenum; stride: GLsizei; pointr: UnivPtr );
	glColorSubTableProcPtr = procedure( target: GLenum; start: GLsizei; count: GLsizei; format: GLenum; typ: GLenum; const data: UnivPtr );
	glColorTableProcPtr = procedure( target: GLenum; internalformat: GLenum; width: GLsizei; format: GLenum; typ: GLenum; const table: UnivPtr );
	glColorTableParameterfvProcPtr = procedure( target: GLenum; pname: GLenum; const params: PGLfloat );
	glColorTableParameterivProcPtr = procedure( target: GLenum; pname: GLenum; const params: PGLint );
	glConvolutionFilter1DProcPtr = procedure( target: GLenum; internalformat: GLenum; width: GLsizei; format: GLenum; typ: GLenum; const image: UnivPtr );
	glConvolutionFilter2DProcPtr = procedure( target: GLenum; internalformat: GLenum; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const image: UnivPtr );
	glConvolutionParameterfProcPtr = procedure( target: GLenum; pname: GLenum; params: GLfloat );
	glConvolutionParameterfvProcPtr = procedure( target: GLenum; pname: GLenum; const params: PGLfloat );
	glConvolutionParameteriProcPtr = procedure( target: GLenum; pname: GLenum; params: GLint );
	glConvolutionParameterivProcPtr = procedure( target: GLenum; pname: GLenum; const params: PGLint );
	glCopyColorSubTableProcPtr = procedure( target: GLenum; start: GLsizei; x: GLint; y: GLint; width: GLsizei );
	glCopyColorTableProcPtr = procedure( target: GLenum; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei );
	glCopyConvolutionFilter1DProcPtr = procedure( target: GLenum; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei );
	glCopyConvolutionFilter2DProcPtr = procedure( target: GLenum; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei; height: GLsizei );
	glCopyPixelsProcPtr = procedure( x: GLint; y: GLint; width: GLsizei; height: GLsizei; typ: GLenum );
	glCopyTexImage1DProcPtr = procedure( target: GLenum; level: GLint; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei; border: GLint );
	glCopyTexImage2DProcPtr = procedure( target: GLenum; level: GLint; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei; height: GLsizei; border: GLint );
	glCopyTexSubImage1DProcPtr = procedure( target: GLenum; level: GLint; xoffset: GLint; x: GLint; y: GLint; width: GLsizei );
	glCopyTexSubImage2DProcPtr = procedure( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; x: GLint; y: GLint; width: GLsizei; height: GLsizei );
	glCopyTexSubImage3DProcPtr = procedure( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; zoffset: GLint; x: GLint; y: GLint; width: GLsizei; height: GLsizei );
	glCullFaceProcPtr = procedure( mode: GLenum );
	glDeleteListsProcPtr = procedure( list: GLuint; range: GLsizei );
	glDeleteTexturesProcPtr = procedure( n: GLsizei; const textures: PGLuint );
	glDepthFuncProcPtr = procedure( func: GLenum );
	glDepthMaskProcPtr = procedure( flag: GLboolean );
	glDepthRangeProcPtr = procedure( zNear: GLclampd; zFar: GLclampd );
	glDisableProcPtr = procedure( cap: GLenum );
	glDisableClientStateProcPtr = procedure( arry: GLenum );
	glDrawArraysProcPtr = procedure( mode: GLenum; first: GLint; count: GLsizei );
	glDrawBufferProcPtr = procedure( mode: GLenum );
	glDrawElementsProcPtr = procedure( mode: GLenum; count: GLsizei; typ: GLenum; const indices: UnivPtr );
	glDrawPixelsProcPtr = procedure( width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr );
	glDrawRangeElementsProcPtr = procedure( mode: GLenum; start: GLuint; finish: GLuint; count: GLsizei; typ: GLenum; const indices: UnivPtr );
	glEdgeFlagProcPtr = procedure( flag: GLboolean );
	glEdgeFlagPointerProcPtr = procedure( stride: GLsizei; const pointr: UnivPtr );
	glEdgeFlagvProcPtr = procedure( const flag: PGLboolean );
	glEnableProcPtr = procedure( cap: GLenum );
	glEnableClientStateProcPtr = procedure( arry: GLenum );
	glEndProcPtr = procedure;
	glEndListProcPtr = procedure;
	glEvalCoord1dProcPtr = procedure( u: GLdouble );
	glEvalCoord1dvProcPtr = procedure( const u: PGLdouble );
	glEvalCoord1fProcPtr = procedure( u: GLfloat );
	glEvalCoord1fvProcPtr = procedure( const u: PGLfloat );
	glEvalCoord2dProcPtr = procedure( u: GLdouble; v: GLdouble );
	glEvalCoord2dvProcPtr = procedure( const u: PGLdouble );
	glEvalCoord2fProcPtr = procedure( u: GLfloat; v: GLfloat );
	glEvalCoord2fvProcPtr = procedure( const u: PGLfloat );
	glEvalMesh1ProcPtr = procedure( mode: GLenum; i1: GLint; i2: GLint );
	glEvalMesh2ProcPtr = procedure( mode: GLenum; i1: GLint; i2: GLint; j1: GLint; j2: GLint );
	glEvalPoint1ProcPtr = procedure( i: GLint );
	glEvalPoint2ProcPtr = procedure( i: GLint; j: GLint );
	glFeedbackBufferProcPtr = procedure( size: GLsizei; typ: GLenum; buffer: PGLfloat );
	glFinishProcPtr = procedure;
	glFlushProcPtr = procedure;
	glFogfProcPtr = procedure( pname: GLenum; param: GLfloat );
	glFogfvProcPtr = procedure( pname: GLenum; const params: PGLfloat );
	glFogiProcPtr = procedure( pname: GLenum; param: GLint );
	glFogivProcPtr = procedure( pname: GLenum; const params: PGLint );
	glFrontFaceProcPtr = procedure( mode: GLenum );
	glFrustumProcPtr = procedure( left: GLdouble; right: GLdouble; bottom: GLdouble; top: GLdouble; zNear: GLdouble; zFar: GLdouble );
	glGenListsProcPtr = function( range: GLsizei ): GLuint;
	glGenTexturesProcPtr = procedure( n: GLsizei; textures: PGLuint );
	glGetBooleanvProcPtr = procedure( pname: GLenum; params: PGLboolean );
	glGetClipPlaneProcPtr = procedure( plane: GLenum; equation: PGLdouble );
	glGetColorTableProcPtr = procedure( target: GLenum; format: GLenum; typ: GLenum; table: UnivPtr );
	glGetColorTableParameterfvProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLfloat );
	glGetColorTableParameterivProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glGetConvolutionFilterProcPtr = procedure( target: GLenum; format: GLenum; typ: GLenum; image: UnivPtr );
	glGetConvolutionParameterfvProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLfloat );
	glGetConvolutionParameterivProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glGetDoublevProcPtr = procedure( pname: GLenum; params: PGLdouble );
	glGetErrorProcPtr = function: GLenum;
	glGetFloatvProcPtr = procedure( pname: GLenum; params: PGLfloat );
	glGetHistogramProcPtr = procedure( target: GLenum; reset: GLboolean; format: GLenum; typ: GLenum; values: UnivPtr );
	glGetHistogramParameterfvProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLfloat );
	glGetHistogramParameterivProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glGetIntegervProcPtr = procedure( pname: GLenum; params: PGLint );
	glGetLightfvProcPtr = procedure( light: GLenum; pname: GLenum; params: PGLfloat );
	glGetLightivProcPtr = procedure( light: GLenum; pname: GLenum; params: PGLint );
	glGetMapdvProcPtr = procedure( target: GLenum; query: GLenum; v: PGLdouble );
	glGetMapfvProcPtr = procedure( target: GLenum; query: GLenum; v: PGLfloat );
	glGetMapivProcPtr = procedure( target: GLenum; query: GLenum; v: PGLint );
	glGetMaterialfvProcPtr = procedure( face: GLenum; pname: GLenum; params: PGLfloat );
	glGetMaterialivProcPtr = procedure( face: GLenum; pname: GLenum; params: PGLint );
	glGetMinmaxProcPtr = procedure( target: GLenum; reset: GLboolean; format: GLenum; typ: GLenum; values: UnivPtr );
	glGetMinmaxParameterfvProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLfloat );
	glGetMinmaxParameterivProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glGetPixelMapfvProcPtr = procedure( map: GLenum; values: PGLfloat );
	glGetPixelMapuivProcPtr = procedure( map: GLenum; values: PGLuint );
	glGetPixelMapusvProcPtr = procedure( map: GLenum; values: PGLushort );
	glGetPointervProcPtr = procedure( pname: GLenum; params: UnivPtr );
	glGetPolygonStippleProcPtr = procedure( mask: PGLubyte );
	glGetSeparableFilterProcPtr = procedure( target: GLenum; format: GLenum; typ: GLenum; row: UnivPtr; column: UnivPtr; span: UnivPtr );
	glGetStringProcPtr = function( name: GLenum ): PChar;
	glGetTexEnvfvProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLfloat );
	glGetTexEnvivProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glGetTexGendvProcPtr = procedure( coord: GLenum; pname: GLenum; params: PGLdouble );
	glGetTexGenfvProcPtr = procedure( coord: GLenum; pname: GLenum; params: PGLfloat );
	glGetTexGenivProcPtr = procedure( coord: GLenum; pname: GLenum; params: PGLint );
	glGetTexImageProcPtr = procedure( target: GLenum; level: GLint; format: GLenum; typ: GLenum; pixels: UnivPtr );
	glGetTexLevelParameterfvProcPtr = procedure( target: GLenum; level: GLint; pname: GLenum; params: PGLfloat );
	glGetTexLevelParameterivProcPtr = procedure( target: GLenum; level: GLint; pname: GLenum; params: PGLint );
	glGetTexParameterfvProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLfloat );
	glGetTexParameterivProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glHintProcPtr = procedure( target: GLenum; mode: GLenum );
	glHistogramProcPtr = procedure( target: GLenum; width: GLsizei; internalformat: GLenum; sink: GLboolean );
	glIndexMaskProcPtr = procedure( mask: GLuint );
	glIndexPointerProcPtr = procedure( typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
	glIndexdProcPtr = procedure( c: GLdouble );
	glIndexdvProcPtr = procedure( const c: PGLdouble );
	glIndexfProcPtr = procedure( c: GLfloat );
	glIndexfvProcPtr = procedure( const c: PGLfloat );
	glIndexiProcPtr = procedure( c: GLint );
	glIndexivProcPtr = procedure( const c: PGLint );
	glIndexsProcPtr = procedure( c: GLshort );
	glIndexsvProcPtr = procedure( const c: PGLshort );
	glIndexubProcPtr = procedure( c: GLubyte );
	glIndexubvProcPtr = procedure( const c: PGLubyte );
	glInitNamesProcPtr = procedure;
	glInterleavedArraysProcPtr = procedure( format: GLenum; stride: GLsizei; const pointr: UnivPtr );
	glIsEnabledProcPtr = function( cap: GLenum ): GLboolean;
	glIsListProcPtr = function( list: GLuint ): GLboolean;
	glIsTextureProcPtr = function( texture: GLuint ): GLboolean;
	glLightModelfProcPtr = procedure( pname: GLenum; param: GLfloat );
	glLightModelfvProcPtr = procedure( pname: GLenum; const params: PGLfloat );
	glLightModeliProcPtr = procedure( pname: GLenum; param: GLint );
	glLightModelivProcPtr = procedure( pname: GLenum; const params: PGLint );
	glLightfProcPtr = procedure( light: GLenum; pname: GLenum; param: GLfloat );
	glLightfvProcPtr = procedure( light: GLenum; pname: GLenum; const params: PGLfloat );
	glLightiProcPtr = procedure( light: GLenum; pname: GLenum; param: GLint );
	glLightivProcPtr = procedure( light: GLenum; pname: GLenum; const params: PGLint );
	glLineStippleProcPtr = procedure( factor: GLint; pattern: GLushort );
	glLineWidthProcPtr = procedure( width: GLfloat );
	glListBaseProcPtr = procedure( base: GLuint );
	glLoadIdentityProcPtr = procedure;
	glLoadMatrixdProcPtr = procedure( const m: PGLdouble );
	glLoadMatrixfProcPtr = procedure( const m: PGLfloat );
	glLoadNameProcPtr = procedure( name: GLuint );
	glLogicOpProcPtr = procedure( opcode: GLenum );
	glMap1dProcPtr = procedure( target: GLenum; u1: GLdouble; u2: GLdouble; stride: GLint; order: GLint; const points: PGLdouble );
	glMap1fProcPtr = procedure( target: GLenum; u1: GLfloat; u2: GLfloat; stride: GLint; order: GLint; const points: PGLfloat );
	glMap2dProcPtr = procedure( target: GLenum; u1: GLdouble; u2: GLdouble; ustride: GLint; uorder: GLint; v1: GLdouble; v2: GLdouble; vstride: GLint; vorder: GLint; const points: PGLdouble );
	glMap2fProcPtr = procedure( target: GLenum; u1: GLfloat; u2: GLfloat; ustride: GLint; uorder: GLint; v1: GLfloat; v2: GLfloat; vstride: GLint; vorder: GLint; const points: PGLfloat );
	glMapGrid1dProcPtr = procedure( un: GLint; u1: GLdouble; u2: GLdouble );
	glMapGrid1fProcPtr = procedure( un: GLint; u1: GLfloat; u2: GLfloat );
	glMapGrid2dProcPtr = procedure( un: GLint; u1: GLdouble; u2: GLdouble; vn: GLint; v1: GLdouble; v2: GLdouble );
	glMapGrid2fProcPtr = procedure( un: GLint; u1: GLfloat; u2: GLfloat; vn: GLint; v1: GLfloat; v2: GLfloat );
	glMaterialfProcPtr = procedure( face: GLenum; pname: GLenum; param: GLfloat );
	glMaterialfvProcPtr = procedure( face: GLenum; pname: GLenum; const params: PGLfloat );
	glMaterialiProcPtr = procedure( face: GLenum; pname: GLenum; param: GLint );
	glMaterialivProcPtr = procedure( face: GLenum; pname: GLenum; const params: PGLint );
	glMatrixModeProcPtr = procedure( mode: GLenum );
	glMinmaxProcPtr = procedure( target: GLenum; internalformat: GLenum; sink: GLboolean );
	glMultMatrixdProcPtr = procedure( const m: PGLdouble );
	glMultMatrixfProcPtr = procedure( const m: PGLfloat );
	glNewListProcPtr = procedure( list: GLuint; mode: GLenum );
	glNormal3bProcPtr = procedure( nx: GLbyte; ny: GLbyte; nz: GLbyte );
	glNormal3bvProcPtr = procedure( const v: PGLbyte );
	glNormal3dProcPtr = procedure( nx: GLdouble; ny: GLdouble; nz: GLdouble );
	glNormal3dvProcPtr = procedure( const v: PGLdouble );
	glNormal3fProcPtr = procedure( nx: GLfloat; ny: GLfloat; nz: GLfloat );
	glNormal3fvProcPtr = procedure( const v: PGLfloat );
	glNormal3iProcPtr = procedure( nx: GLint; ny: GLint; nz: GLint );
	glNormal3ivProcPtr = procedure( const v: PGLint );
	glNormal3sProcPtr = procedure( nx: GLshort; ny: GLshort; nz: GLshort );
	glNormal3svProcPtr = procedure( const v: PGLshort );
	glNormalPointerProcPtr = procedure( typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
	glOrthoProcPtr = procedure( left: GLdouble; right: GLdouble; bottom: GLdouble; top: GLdouble; zNear: GLdouble; zFar: GLdouble );
	glPassThroughProcPtr = procedure( token: GLfloat );
	glPixelMapfvProcPtr = procedure( map: GLenum; mapsize: GLint; const values: PGLfloat );
	glPixelMapuivProcPtr = procedure( map: GLenum; mapsize: GLint; const values: PGLuint );
	glPixelMapusvProcPtr = procedure( map: GLenum; mapsize: GLint; const values: PGLushort );
	glPixelStorefProcPtr = procedure( pname: GLenum; param: GLfloat );
	glPixelStoreiProcPtr = procedure( pname: GLenum; param: GLint );
	glPixelTransferfProcPtr = procedure( pname: GLenum; param: GLfloat );
	glPixelTransferiProcPtr = procedure( pname: GLenum; param: GLint );
	glPixelZoomProcPtr = procedure( xfactor: GLfloat; yfactor: GLfloat );
	glPointSizeProcPtr = procedure( size: GLfloat );
	glPolygonModeProcPtr = procedure( face: GLenum; mode: GLenum );
	glPolygonOffsetProcPtr = procedure( factor: GLfloat; units: GLfloat );
	glPolygonStippleProcPtr = procedure( const mask: PGLubyte );
	glPopAttribProcPtr = procedure;
	glPopClientAttribProcPtr = procedure;
	glPopMatrixProcPtr = procedure;
	glPopNameProcPtr = procedure;
	glPrioritizeTexturesProcPtr = procedure( n: GLsizei; const textures: PGLuint; const priorities: PGLclampf );
	glPushAttribProcPtr = procedure( mask: GLbitfield );
	glPushClientAttribProcPtr = procedure( mask: GLbitfield );
	glPushMatrixProcPtr = procedure;
	glPushNameProcPtr = procedure( name: GLuint );
	glRasterPos2dProcPtr = procedure( x: GLdouble; y: GLdouble );
	glRasterPos2dvProcPtr = procedure( const v: PGLdouble );
	glRasterPos2fProcPtr = procedure( x: GLfloat; y: GLfloat );
	glRasterPos2fvProcPtr = procedure( const v: PGLfloat );
	glRasterPos2iProcPtr = procedure( x: GLint; y: GLint );
	glRasterPos2ivProcPtr = procedure( const v: PGLint );
	glRasterPos2sProcPtr = procedure( x: GLshort; y: GLshort );
	glRasterPos2svProcPtr = procedure( const v: PGLshort );
	glRasterPos3dProcPtr = procedure( x: GLdouble; y: GLdouble; z: GLdouble );
	glRasterPos3dvProcPtr = procedure( const v: PGLdouble );
	glRasterPos3fProcPtr = procedure( x: GLfloat; y: GLfloat; z: GLfloat );
	glRasterPos3fvProcPtr = procedure( const v: PGLfloat );
	glRasterPos3iProcPtr = procedure( x: GLint; y: GLint; z: GLint );
	glRasterPos3ivProcPtr = procedure( const v: PGLint );
	glRasterPos3sProcPtr = procedure( x: GLshort; y: GLshort; z: GLshort );
	glRasterPos3svProcPtr = procedure( const v: PGLshort );
	glRasterPos4dProcPtr = procedure( x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
	glRasterPos4dvProcPtr = procedure( const v: PGLdouble );
	glRasterPos4fProcPtr = procedure( x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
	glRasterPos4fvProcPtr = procedure( const v: PGLfloat );
	glRasterPos4iProcPtr = procedure( x: GLint; y: GLint; z: GLint; w: GLint );
	glRasterPos4ivProcPtr = procedure( const v: PGLint );
	glRasterPos4sProcPtr = procedure( x: GLshort; y: GLshort; z: GLshort; w: GLshort );
	glRasterPos4svProcPtr = procedure( const v: PGLshort );
	glReadBufferProcPtr = procedure( mode: GLenum );
	glReadPixelsProcPtr = procedure( x: GLint; y: GLint; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; pixels: UnivPtr );
	glRectdProcPtr = procedure( x1: GLdouble; y1: GLdouble; x2: GLdouble; y2: GLdouble );
	glRectdvProcPtr = procedure( const v1: PGLdouble; const v2: PGLdouble );
	glRectfProcPtr = procedure( x1: GLfloat; y1: GLfloat; x2: GLfloat; y2: GLfloat );
	glRectfvProcPtr = procedure( const v1: PGLfloat; const v2: PGLfloat );
	glRectiProcPtr = procedure( x1: GLint; y1: GLint; x2: GLint; y2: GLint );
	glRectivProcPtr = procedure( const v1: PGLint; const v2: PGLint );
	glRectsProcPtr = procedure( x1: GLshort; y1: GLshort; x2: GLshort; y2: GLshort );
	glRectsvProcPtr = procedure( const v1: PGLshort; const v2: PGLshort );
	glRenderModeProcPtr = function( mode: GLenum ): GLint;
	glResetHistogramProcPtr = procedure( target: GLenum );
	glResetMinmaxProcPtr = procedure( target: GLenum );
	glRotatedProcPtr = procedure( angle: GLdouble; x: GLdouble; y: GLdouble; z: GLdouble );
	glRotatefProcPtr = procedure( angle: GLfloat; x: GLfloat; y: GLfloat; z: GLfloat );
	glScaledProcPtr = procedure( x: GLdouble; y: GLdouble; z: GLdouble );
	glScalefProcPtr = procedure( x: GLfloat; y: GLfloat; z: GLfloat );
	glScissorProcPtr = procedure( x: GLint; y: GLint; width: GLsizei; height: GLsizei );
	glSelectBufferProcPtr = procedure( size: GLsizei; buffer: PGLuint );
	glSeparableFilter2DProcPtr = procedure( target: GLenum; internalformat: GLenum; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const row: UnivPtr; const column: UnivPtr );
	glShadeModelProcPtr = procedure( mode: GLenum );
	glStencilFuncProcPtr = procedure( func: GLenum; ref: GLint; mask: GLuint );
	glStencilMaskProcPtr = procedure( mask: GLuint );
	glStencilOpProcPtr = procedure( fail: GLenum; zfail: GLenum; zpass: GLenum );
	glTexCoord1dProcPtr = procedure( s: GLdouble );
	glTexCoord1dvProcPtr = procedure( const v: PGLdouble );
	glTexCoord1fProcPtr = procedure( s: GLfloat );
	glTexCoord1fvProcPtr = procedure( const v: PGLfloat );
	glTexCoord1iProcPtr = procedure( s: GLint );
	glTexCoord1ivProcPtr = procedure( const v: PGLint );
	glTexCoord1sProcPtr = procedure( s: GLshort );
	glTexCoord1svProcPtr = procedure( const v: PGLshort );
	glTexCoord2dProcPtr = procedure( s: GLdouble; t: GLdouble );
	glTexCoord2dvProcPtr = procedure( const v: PGLdouble );
	glTexCoord2fProcPtr = procedure( s: GLfloat; t: GLfloat );
	glTexCoord2fvProcPtr = procedure( const v: PGLfloat );
	glTexCoord2iProcPtr = procedure( s: GLint; t: GLint );
	glTexCoord2ivProcPtr = procedure( const v: PGLint );
	glTexCoord2sProcPtr = procedure( s: GLshort; t: GLshort );
	glTexCoord2svProcPtr = procedure( const v: PGLshort );
	glTexCoord3dProcPtr = procedure( s: GLdouble; t: GLdouble; r: GLdouble );
	glTexCoord3dvProcPtr = procedure( const v: PGLdouble );
	glTexCoord3fProcPtr = procedure( s: GLfloat; t: GLfloat; r: GLfloat );
	glTexCoord3fvProcPtr = procedure( const v: PGLfloat );
	glTexCoord3iProcPtr = procedure( s: GLint; t: GLint; r: GLint );
	glTexCoord3ivProcPtr = procedure( const v: PGLint );
	glTexCoord3sProcPtr = procedure( s: GLshort; t: GLshort; r: GLshort );
	glTexCoord3svProcPtr = procedure( const v: PGLshort );
	glTexCoord4dProcPtr = procedure( s: GLdouble; t: GLdouble; r: GLdouble; q: GLdouble );
	glTexCoord4dvProcPtr = procedure( const v: PGLdouble );
	glTexCoord4fProcPtr = procedure( s: GLfloat; t: GLfloat; r: GLfloat; q: GLfloat );
	glTexCoord4fvProcPtr = procedure( const v: PGLfloat );
	glTexCoord4iProcPtr = procedure( s: GLint; t: GLint; r: GLint; q: GLint );
	glTexCoord4ivProcPtr = procedure( const v: PGLint );
	glTexCoord4sProcPtr = procedure( s: GLshort; t: GLshort; r: GLshort; q: GLshort );
	glTexCoord4svProcPtr = procedure( const v: PGLshort );
	glTexCoordPointerProcPtr = procedure( size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
	glTexEnvfProcPtr = procedure( target: GLenum; pname: GLenum; param: GLfloat );
	glTexEnvfvProcPtr = procedure( target: GLenum; pname: GLenum; const params: PGLfloat );
	glTexEnviProcPtr = procedure( target: GLenum; pname: GLenum; param: GLint );
	glTexEnvivProcPtr = procedure( target: GLenum; pname: GLenum; const params: PGLint );
	glTexGendProcPtr = procedure( coord: GLenum; pname: GLenum; param: GLdouble );
	glTexGendvProcPtr = procedure( coord: GLenum; pname: GLenum; const params: PGLdouble );
	glTexGenfProcPtr = procedure( coord: GLenum; pname: GLenum; param: GLfloat );
	glTexGenfvProcPtr = procedure( coord: GLenum; pname: GLenum; const params: PGLfloat );
	glTexGeniProcPtr = procedure( coord: GLenum; pname: GLenum; param: GLint );
	glTexGenivProcPtr = procedure( coord: GLenum; pname: GLenum; const params: PGLint );
	glTexImage1DProcPtr = procedure( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; border: GLint; format: GLenum; typ: GLenum; const pixels: UnivPtr );
	glTexImage2DProcPtr = procedure( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; height: GLsizei; border: GLint; format: GLenum; typ: GLenum; const pixels: UnivPtr );
	glTexImage3DProcPtr = procedure( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; height: GLsizei; depth: GLsizei; border: GLint; format: GLenum; typ: GLenum; const pixels: UnivPtr );
	glTexParameterfProcPtr = procedure( target: GLenum; pname: GLenum; param: GLfloat );
	glTexParameterfvProcPtr = procedure( target: GLenum; pname: GLenum; const params: PGLfloat );
	glTexParameteriProcPtr = procedure( target: GLenum; pname: GLenum; param: GLint );
	glTexParameterivProcPtr = procedure( target: GLenum; pname: GLenum; const params: PGLint );
	glTexSubImage1DProcPtr = procedure( target: GLenum; level: GLint; xoffset: GLint; width: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr );
	glTexSubImage2DProcPtr = procedure( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr );
	glTexSubImage3DProcPtr = procedure( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; zoffset: GLint; width: GLsizei; height: GLsizei; depth: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr );
	glTranslatedProcPtr = procedure( x: GLdouble; y: GLdouble; z: GLdouble );
	glTranslatefProcPtr = procedure( x: GLfloat; y: GLfloat; z: GLfloat );
	glVertex2dProcPtr = procedure( x: GLdouble; y: GLdouble );
	glVertex2dvProcPtr = procedure( const v: PGLdouble );
	glVertex2fProcPtr = procedure( x: GLfloat; y: GLfloat );
	glVertex2fvProcPtr = procedure( const v: PGLfloat );
	glVertex2iProcPtr = procedure( x: GLint; y: GLint );
	glVertex2ivProcPtr = procedure( const v: PGLint );
	glVertex2sProcPtr = procedure( x: GLshort; y: GLshort );
	glVertex2svProcPtr = procedure( const v: PGLshort );
	glVertex3dProcPtr = procedure( x: GLdouble; y: GLdouble; z: GLdouble );
	glVertex3dvProcPtr = procedure( const v: PGLdouble );
	glVertex3fProcPtr = procedure( x: GLfloat; y: GLfloat; z: GLfloat );
	glVertex3fvProcPtr = procedure( const v: PGLfloat );
	glVertex3iProcPtr = procedure( x: GLint; y: GLint; z: GLint );
	glVertex3ivProcPtr = procedure( const v: PGLint );
	glVertex3sProcPtr = procedure( x: GLshort; y: GLshort; z: GLshort );
	glVertex3svProcPtr = procedure( const v: PGLshort );
	glVertex4dProcPtr = procedure( x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
	glVertex4dvProcPtr = procedure( const v: PGLdouble );
	glVertex4fProcPtr = procedure( x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
	glVertex4fvProcPtr = procedure( const v: PGLfloat );
	glVertex4iProcPtr = procedure( x: GLint; y: GLint; z: GLint; w: GLint );
	glVertex4ivProcPtr = procedure( const v: PGLint );
	glVertex4sProcPtr = procedure( x: GLshort; y: GLshort; z: GLshort; w: GLshort );
	glVertex4svProcPtr = procedure( const v: PGLshort );
	glVertexPointerProcPtr = procedure( size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
	glViewportProcPtr = procedure( x: GLint; y: GLint; width: GLsizei; height: GLsizei );

type
	glSampleCoverageProcPtr = procedure( value: GLclampf; invert: GLboolean );
	glSamplePassProcPtr = procedure( pass: GLenum );

type
	glLoadTransposeMatrixfProcPtr = procedure( const m: PGLfloat );
	glLoadTransposeMatrixdProcPtr = procedure( const m: PGLdouble );
	glMultTransposeMatrixfProcPtr = procedure( const m: PGLfloat );
	glMultTransposeMatrixdProcPtr = procedure( const m: PGLdouble );

type
	glCompressedTexImage3DProcPtr = procedure( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; height: GLsizei; depth: GLsizei; border: GLint; imageSize: GLsizei; const data: UnivPtr );
	glCompressedTexImage2DProcPtr = procedure( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; height: GLsizei; border: GLint; imageSize: GLsizei; const data: UnivPtr );
	glCompressedTexImage1DProcPtr = procedure( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; border: GLint; imageSize: GLsizei; const data: UnivPtr );
	glCompressedTexSubImage3DProcPtr = procedure( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; zoffset: GLint; width: GLsizei; height: GLsizei; depth: GLsizei; format: GLenum; imageSize: GLsizei; const data: UnivPtr );
	glCompressedTexSubImage2DProcPtr = procedure( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; width: GLsizei; height: GLsizei; format: GLenum; imageSize: GLsizei; const data: UnivPtr );
	glCompressedTexSubImage1DProcPtr = procedure( target: GLenum; level: GLint; xoffset: GLint; width: GLsizei; format: GLenum; imageSize: GLsizei; const data: UnivPtr );
	glGetCompressedTexImageProcPtr = procedure( target: GLenum; lod: GLint; img: UnivPtr );

type
	glActiveTextureProcPtr = procedure( texture: GLenum );
	glClientActiveTextureProcPtr = procedure( texture: GLenum );
	glMultiTexCoord1dProcPtr = procedure( target: GLenum; s: GLdouble );
	glMultiTexCoord1dvProcPtr = procedure( target: GLenum; const v: PGLdouble );
	glMultiTexCoord1fProcPtr = procedure( target: GLenum; s: GLfloat );
	glMultiTexCoord1fvProcPtr = procedure( target: GLenum; const v: PGLfloat );
	glMultiTexCoord1iProcPtr = procedure( target: GLenum; s: GLint );
	glMultiTexCoord1ivProcPtr = procedure( target: GLenum; const v: PGLint );
	glMultiTexCoord1sProcPtr = procedure( target: GLenum; s: GLshort );
	glMultiTexCoord1svProcPtr = procedure( target: GLenum; const v: PGLshort );
	glMultiTexCoord2dProcPtr = procedure( target: GLenum; s: GLdouble; t: GLdouble );
	glMultiTexCoord2dvProcPtr = procedure( target: GLenum; const v: PGLdouble );
	glMultiTexCoord2fProcPtr = procedure( target: GLenum; s: GLfloat; t: GLfloat );
	glMultiTexCoord2fvProcPtr = procedure( target: GLenum; const v: PGLfloat );
	glMultiTexCoord2iProcPtr = procedure( target: GLenum; s: GLint; t: GLint );
	glMultiTexCoord2ivProcPtr = procedure( target: GLenum; const v: PGLint );
	glMultiTexCoord2sProcPtr = procedure( target: GLenum; s: GLshort; t: GLshort );
	glMultiTexCoord2svProcPtr = procedure( target: GLenum; const v: PGLshort );
	glMultiTexCoord3dProcPtr = procedure( target: GLenum; s: GLdouble; t: GLdouble; r: GLdouble );
	glMultiTexCoord3dvProcPtr = procedure( target: GLenum; const v: PGLdouble );
	glMultiTexCoord3fProcPtr = procedure( target: GLenum; s: GLfloat; t: GLfloat; r: GLfloat );
	glMultiTexCoord3fvProcPtr = procedure( target: GLenum; const v: PGLfloat );
	glMultiTexCoord3iProcPtr = procedure( target: GLenum; s: GLint; t: GLint; r: GLint );
	glMultiTexCoord3ivProcPtr = procedure( target: GLenum; const v: PGLint );
	glMultiTexCoord3sProcPtr = procedure( target: GLenum; s: GLshort; t: GLshort; r: GLshort );
	glMultiTexCoord3svProcPtr = procedure( target: GLenum; const v: PGLshort );
	glMultiTexCoord4dProcPtr = procedure( target: GLenum; s: GLdouble; t: GLdouble; r: GLdouble; q: GLdouble );
	glMultiTexCoord4dvProcPtr = procedure( target: GLenum; const v: PGLdouble );
	glMultiTexCoord4fProcPtr = procedure( target: GLenum; s: GLfloat; t: GLfloat; r: GLfloat; q: GLfloat );
	glMultiTexCoord4fvProcPtr = procedure( target: GLenum; const v: PGLfloat );
	glMultiTexCoord4iProcPtr = procedure( target: GLenum; GLint; s: GLint; t: GLint; r: GLint );
	glMultiTexCoord4ivProcPtr = procedure( target: GLenum; const v: PGLint );
	glMultiTexCoord4sProcPtr = procedure( target: GLenum; s: GLshort; t: GLshort; r: GLshort; q: GLshort );
	glMultiTexCoord4svProcPtr = procedure( target: GLenum; const v: PGLshort );

type
	glFogCoordfProcPtr = procedure( coord: GLfloat );
	glFogCoordfvProcPtr = procedure( const coord: PGLfloat );  
	glFogCoorddProcPtr = procedure( coord: GLdouble );
	glFogCoorddvProcPtr = procedure( const coord: PGLdouble );   
	glFogCoordPointerProcPtr = procedure( typ: GLenum; stride: GLsizei; const pointr: UnivPtr );

type
	glSecondaryColor3bProcPtr = procedure( red: GLbyte; green: GLbyte; blue: GLbyte );
	glSecondaryColor3bvProcPtr = procedure( const v: PGLbyte );
	glSecondaryColor3dProcPtr = procedure( red: GLdouble; green: GLdouble; blue: GLdouble );
	glSecondaryColor3dvProcPtr = procedure( const v: PGLdouble );
	glSecondaryColor3fProcPtr = procedure( red: GLfloat; green: GLfloat; blue: GLfloat );
	glSecondaryColor3fvProcPtr = procedure( const v: PGLfloat );
	glSecondaryColor3iProcPtr = procedure( red: GLint; green: GLint; blue: GLint );
	glSecondaryColor3ivProcPtr = procedure( const v: PGLint );
	glSecondaryColor3sProcPtr = procedure( red: GLshort; green: GLshort; blue: GLshort );
	glSecondaryColor3svProcPtr = procedure( const v: PGLshort );
	glSecondaryColor3ubProcPtr = procedure( red: GLubyte; green: GLubyte; blue: GLubyte );
	glSecondaryColor3ubvProcPtr = procedure( const v: PGLubyte );
	glSecondaryColor3uiProcPtr = procedure( red: GLuint; green: GLuint; blue: GLuint );
	glSecondaryColor3uivProcPtr = procedure( const v: PGLuint );
	glSecondaryColor3usProcPtr = procedure( red: GLushort; green: GLushort; blue: GLushort );
	glSecondaryColor3usvProcPtr = procedure( const v: PGLushort );
	glSecondaryColorPointerProcPtr = procedure( size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );

type
	glPointParameterfProcPtr = procedure( pname: GLenum; param: GLfloat ); 
	glPointParameterfvProcPtr = procedure( pname: GLenum; const params: PGLfloat );
	glPointParameteriProcPtr = procedure( pname: GLenum; param: GLint ); 
	glPointParameterivProcPtr = procedure( pname: GLenum; const params: PGLint );

type
	glBlendFuncSeparateProcPtr = procedure( srcRGB: GLenum; dstRGB: GLenum; srcAlpha: GLenum; dstAlpha: GLenum );

type
	glMultiDrawArraysProcPtr = procedure( mode: GLenum; const first: PGLint; const count: PGLsizei; primcount: GLsizei );
	glMultiDrawElementsProcPtr = procedure( mode: GLenum; const count: PGLsizei; typ: GLenum; {const} indices: UnivPtrPtr; primcount: GLsizei );

type
	glWindowPos2dProcPtr = procedure( x: GLdouble; y: GLdouble );
	glWindowPos2dvProcPtr = procedure( const v: PGLdouble );
	glWindowPos2fProcPtr = procedure( x: GLfloat; y: GLfloat );
	glWindowPos2fvProcPtr = procedure( const v: PGLfloat );
	glWindowPos2iProcPtr = procedure( x: GLint; y: GLint ); 
	glWindowPos2ivProcPtr = procedure( const v: PGLint );
	glWindowPos2sProcPtr = procedure( x: GLshort; y: GLshort );
	glWindowPos2svProcPtr = procedure( const v: PGLshort );
	glWindowPos3dProcPtr = procedure( x: GLdouble; y: GLdouble; z: GLdouble );
	glWindowPos3dvProcPtr = procedure( const v: PGLdouble );
	glWindowPos3fProcPtr = procedure( x: GLfloat; y: GLfloat; z: GLfloat );
	glWindowPos3fvProcPtr = procedure( const v: PGLfloat );
	glWindowPos3iProcPtr = procedure( x: GLint; y: GLint; z: GLint );
	glWindowPos3ivProcPtr = procedure( const v: PGLint );
	glWindowPos3sProcPtr = procedure( x: GLshort; y: GLshort; z: GLshort );
	glWindowPos3svProcPtr = procedure( const v: PGLshort );

type
	glGenQueriesProcPtr = procedure( n: GLsizei; ids: PGLuint );
	glDeleteQueriesProcPtr = procedure( n: GLsizei; const ids: PGLuint );
	glIsQueryProcPtr = function( id: GLuint ): GLboolean;
	glBeginQueryProcPtr = procedure( target: GLenum; id: GLuint );
	glEndQueryProcPtr = procedure( target: GLenum );
	glGetQueryivProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glGetQueryObjectivProcPtr = procedure( id: GLuint; pname: GLenum; params: PGLint );
	glGetQueryObjectuivProcPtr = procedure( id: GLuint; pname: GLenum; params: PGLuint );

type
	glBindBufferProcPtr = procedure( target: GLenum; buffer: GLuint );
	glDeleteBuffersProcPtr = procedure( n: GLsizei; const buffers: PGLuint );
	glGenBuffersProcPtr = procedure( n: GLsizei; buffers: PGLuint );
	glIsBufferProcPtr = function( buffer: GLuint ): GLboolean;
	glBufferDataProcPtr = procedure( target: GLenum; size: GLsizeiptr; const data: UnivPtr; usage: GLenum );
	glBufferSubDataProcPtr = procedure( target: GLenum; offset: GLintptr; size: GLsizeiptr; const data: UnivPtr );
	glGetBufferSubDataProcPtr = procedure( target: GLenum; offset: GLintptr; size: GLsizeiptr; data: UnivPtr );
	glMapBufferProcPtr = function( target: GLenum; access: GLenum ): UnivPtr;
	glUnmapBufferProcPtr = function( target: GLenum ): GLboolean;
	glGetBufferParameterivProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glGetBufferPointervProcPtr = procedure( target: GLenum; pname: GLenum; params: UnivPtrPtr );

type
	glDrawBuffersProcPtr = procedure( n: GLsizei; const bufs: PGLenum );
	glVertexAttrib1dProcPtr = procedure( index: GLuint; x: GLdouble );
	glVertexAttrib1dvProcPtr = procedure( index: GLuint; const v: PGLdouble );
	glVertexAttrib1fProcPtr = procedure( index: GLuint; x: GLfloat );
	glVertexAttrib1fvProcPtr = procedure( index: GLuint; const v: PGLfloat );
	glVertexAttrib1sProcPtr = procedure( index: GLuint; x: GLshort );
	glVertexAttrib1svProcPtr = procedure( index: GLuint; const v: PGLshort );
	glVertexAttrib2dProcPtr = procedure( index: GLuint; x: GLdouble; y: GLdouble );
	glVertexAttrib2dvProcPtr = procedure( index: GLuint; const v: PGLdouble );
	glVertexAttrib2fProcPtr = procedure( index: GLuint; x: GLfloat; y: GLfloat );
	glVertexAttrib2fvProcPtr = procedure( index: GLuint; const v: PGLfloat );
	glVertexAttrib2sProcPtr = procedure( index: GLuint; x: GLshort; y: GLshort );
	glVertexAttrib2svProcPtr = procedure( index: GLuint; const v: PGLshort );
	glVertexAttrib3dProcPtr = procedure( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble );
	glVertexAttrib3dvProcPtr = procedure( index: GLuint; const v: PGLdouble );
	glVertexAttrib3fProcPtr = procedure( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat );
	glVertexAttrib3fvProcPtr = procedure( index: GLuint; const v: PGLfloat );
	glVertexAttrib3sProcPtr = procedure( index: GLuint; x: GLshort; y: GLshort; z: GLshort );
	glVertexAttrib3svProcPtr = procedure( index: GLuint; const v: PGLshort );
	glVertexAttrib4NbvProcPtr = procedure( index: GLuint; const v: PGLbyte );
	glVertexAttrib4NivProcPtr = procedure( index: GLuint; const v: PGLint );
	glVertexAttrib4NsvProcPtr = procedure( index: GLuint; const v: PGLshort );
	glVertexAttrib4NubProcPtr = procedure( index: GLuint; x: GLubyte; y: GLubyte; z: GLubyte; w: GLubyte );
	glVertexAttrib4NubvProcPtr = procedure( index: GLuint; const v: PGLubyte );
	glVertexAttrib4NuivProcPtr = procedure( index: GLuint; const v: PGLuint );
	glVertexAttrib4NusvProcPtr = procedure( index: GLuint; const v: PGLushort );
	glVertexAttrib4bvProcPtr = procedure( index: GLuint; const v: PGLbyte );
	glVertexAttrib4dProcPtr = procedure( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
	glVertexAttrib4dvProcPtr = procedure( index: GLuint; const v: PGLdouble );
	glVertexAttrib4fProcPtr = procedure( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
	glVertexAttrib4fvProcPtr = procedure( index: GLuint; const v: PGLfloat );
	glVertexAttrib4ivProcPtr = procedure( index: GLuint; const v: PGLint );
	glVertexAttrib4sProcPtr = procedure( index: GLuint; x: GLshort; y: GLshort; z: GLshort; w: GLshort );
	glVertexAttrib4svProcPtr = procedure( index: GLuint; const v: PGLshort );
	glVertexAttrib4ubvProcPtr = procedure( index: GLuint; const v: PGLubyte );
	glVertexAttrib4uivProcPtr = procedure( index: GLuint; const v: PGLuint );
	glVertexAttrib4usvProcPtr = procedure( index: GLuint; const v: PGLushort );
	glVertexAttribPointerProcPtr = procedure( index: GLuint; size: GLint; typ: GLenum; normalized: GLboolean; stride: GLsizei; const pointr: UnivPtr );
	glEnableVertexAttribArrayProcPtr = procedure( index: GLuint );
	glDisableVertexAttribArrayProcPtr = procedure( index: GLuint );
	glGetVertexAttribdvProcPtr = procedure( index: GLuint; pname: GLenum; params: PGLdouble );
	glGetVertexAttribfvProcPtr = procedure( index: GLuint; pname: GLenum; params: PGLfloat );
	glGetVertexAttribivProcPtr = procedure( index: GLuint; pname: GLenum; params: PGLint );
	glGetVertexAttribPointervProcPtr = procedure( index: GLuint; pname: GLenum; pointr: UnivPtrPtr );
	glDeleteShaderProcPtr = procedure( shader: GLuint );
	glDetachShaderProcPtr = procedure( program_: GLuint; shader: GLuint );
	glCreateShaderProcPtr = function( typ: GLenum ): GLuint;
	glShaderSourceProcPtr = procedure( shader: GLuint; count: GLsizei; {const} strng: PPChar; const length: PGLint );
	glCompileShaderProcPtr = procedure( shader: GLuint );
	glCreateProgramProcPtr = function: GLuint;
	glAttachShaderProcPtr = procedure( program_: GLuint; shader: GLuint );
	glLinkProgramProcPtr = procedure( program_: GLuint );
	glUseProgramProcPtr = procedure( program_: GLuint );
	glDeleteProgramProcPtr = procedure( program_: GLuint );
	glValidateProgramProcPtr = procedure( program_: GLuint );
	glUniform1fProcPtr = procedure( location: GLint; v0: GLfloat );
	glUniform2fProcPtr = procedure( location: GLint; v0: GLfloat; v1: GLfloat );
	glUniform3fProcPtr = procedure( location: GLint; v0: GLfloat; v1: GLfloat; v2: GLfloat );
	glUniform4fProcPtr = procedure( location: GLint; v0: GLfloat; v1: GLfloat; v2: GLfloat; v3: GLfloat );
	glUniform1iProcPtr = procedure( location: GLint; v0: GLint );
	glUniform2iProcPtr = procedure( location: GLint; v0: GLint; v1: GLint );
	glUniform3iProcPtr = procedure( location: GLint; v0: GLint; v1: GLint; v2: GLint );
	glUniform4iProcPtr = procedure( location: GLint; v0: GLint; v1: GLint; v2: GLint; v3: GLint );
	glUniform1fvProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLfloat );
	glUniform2fvProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLfloat );
	glUniform3fvProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLfloat );
	glUniform4fvProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLfloat );
	glUniform1ivProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLint );
	glUniform2ivProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLint );
	glUniform3ivProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLint );
	glUniform4ivProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLint );
	glUniformMatrix2fvProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	glUniformMatrix3fvProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	glUniformMatrix4fvProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	glIsShaderProcPtr = function( shader: GLuint ): GLboolean;
	glIsProgramProcPtr = function( program_: GLuint ): GLboolean;
	glGetShaderivProcPtr = procedure( shader: GLuint; pname: GLenum; params: PGLint );
	glGetProgramivProcPtr = procedure( program_: GLuint; pname: GLenum; params: PGLint );
	glGetAttachedShadersProcPtr = procedure( program_: GLuint; maxCount: GLsizei; count: PGLsizei; shaders: PGLuint );
	glGetShaderInfoLogProcPtr = procedure( shader: GLuint; bufSize: GLsizei; length: PGLsizei; infoLog: PChar );
	glGetProgramInfoLogProcPtr = procedure( program_: GLuint; bufSize: GLsizei; length: PGLsizei; infoLog: PChar );
	glGetUniformLocationProcPtr = function( program_: GLuint; const name: PChar ): GLint;
	glGetActiveUniformProcPtr = procedure( program_: GLuint; index: GLuint; bufSize: GLsizei; length: PGLsizei; size: PGLint; typ: PGLenum; name: PChar );
	glGetUniformfvProcPtr = procedure( program_: GLuint; location: GLint; params: PGLfloat );
	glGetUniformivProcPtr = procedure( program_: GLuint; location: GLint; params: PGLint );
	glGetShaderSourceProcPtr = procedure( shader: GLuint; bufSize: GLsizei; length: PGLsizei; source: PChar );
	glBindAttribLocationProcPtr = procedure( program_: GLuint; index: GLuint; const name: PChar );
	glGetActiveAttribProcPtr = procedure( program_: GLuint; index: GLuint; bufSize: GLsizei; length: PGLsizei; size: PGLint; typ: PGLenum; name: PChar );
	glGetAttribLocationProcPtr = function( program_: GLuint; const name: PChar ): GLint;
	glStencilFuncSeparateProcPtr = procedure( face: GLenum; func: GLenum; ref: GLint; mask: GLuint );
	glStencilOpSeparateProcPtr = procedure( face: GLenum; fail: GLenum; zfail: GLenum; zpass: GLenum );
	glStencilMaskSeparateProcPtr = procedure( face: GLenum; mask: GLuint );

type
	glUniformMatrix2x3fvProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	glUniformMatrix3x2fvProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	glUniformMatrix2x4fvProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	glUniformMatrix4x2fvProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	glUniformMatrix3x4fvProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	glUniformMatrix4x3fvProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );

{$elsec} { GL_GLEXT_FUNCTION_POINTERS }

procedure glAccum( op: GLenum; value: GLfloat ); external name '_glAccum';
procedure glAlphaFunc( func: GLenum; ref: GLclampf ); external name '_glAlphaFunc';
function glAreTexturesResident( n: GLsizei; const textures: PGLuint; residences: PGLboolean ): GLboolean; external name '_glAreTexturesResident';
procedure glArrayElement( i: GLint ); external name '_glArrayElement';
procedure glBegin( mode: GLenum ); external name '_glBegin';
procedure glBindTexture( target: GLenum; texture: GLuint ); external name '_glBindTexture';
procedure glBitmap( width: GLsizei; height: GLsizei; xorig: GLfloat; yorig: GLfloat; xmove: GLfloat; ymove: GLfloat; const bitmap: PGLubyte ); external name '_glBitmap';
procedure glBlendColor( red: GLclampf; green: GLclampf; blue: GLclampf; alpha: GLclampf ); external name '_glBlendColor';
procedure glBlendEquation( mode: GLenum ); external name '_glBlendEquation';
procedure glBlendEquationSeparate( modeRGB: GLenum; modeAlpha: GLenum ); external name '_glBlendEquationSeparate';
procedure glBlendFunc( sfactor: GLenum; dfactor: GLenum ); external name '_glBlendFunc';
procedure glCallList( list: GLuint ); external name '_glCallList';
procedure glCallLists( n: GLsizei; typ: GLenum; const lists: UnivPtr ); external name '_glCallLists';
procedure glClear( mask: GLbitfield ); external name '_glClear';
procedure glClearAccum( red: GLfloat; green: GLfloat; blue: GLfloat; alpha: GLfloat ); external name '_glClearAccum';
procedure glClearColor( red: GLclampf; green: GLclampf; blue: GLclampf; alpha: GLclampf ); external name '_glClearColor';
procedure glClearDepth( depth: GLclampd ); external name '_glClearDepth';
procedure glClearIndex( c: GLfloat ); external name '_glClearIndex';
procedure glClearStencil( s: GLint ); external name '_glClearStencil';
procedure glClipPlane( plane: GLenum; const equation: PGLdouble ); external name '_glClipPlane';
procedure glColor3b( red: GLbyte; green: GLbyte; blue: GLbyte ); external name '_glColor3b';
procedure glColor3bv( const v: PGLbyte ); external name '_glColor3bv';
procedure glColor3d( red: GLdouble; green: GLdouble; blue: GLdouble ); external name '_glColor3d';
procedure glColor3dv( const v: PGLdouble ); external name '_glColor3dv';
procedure glColor3f( red: GLfloat; green: GLfloat; blue: GLfloat ); external name '_glColor3f';
procedure glColor3fv( const v: PGLfloat ); external name '_glColor3fv';
procedure glColor3i( red: GLint; green: GLint; blue: GLint ); external name '_glColor3i';
procedure glColor3iv( const v: PGLint ); external name '_glColor3iv';
procedure glColor3s( red: GLshort; green: GLshort; blue: GLshort ); external name '_glColor3s';
procedure glColor3sv( const v: PGLshort ); external name '_glColor3sv';
procedure glColor3ub( red: GLubyte; green: GLubyte; blue: GLubyte ); external name '_glColor3ub';
procedure glColor3ubv( const v: PGLubyte ); external name '_glColor3ubv';
procedure glColor3ui( red: GLuint; green: GLuint; blue: GLuint ); external name '_glColor3ui';
procedure glColor3uiv( const v: PGLuint ); external name '_glColor3uiv';
procedure glColor3us( red: GLushort; green: GLushort; blue: GLushort ); external name '_glColor3us';
procedure glColor3usv( const v: PGLushort ); external name '_glColor3usv';
procedure glColor4b( red: GLbyte; green: GLbyte; blue: GLbyte; alpha: GLbyte ); external name '_glColor4b';
procedure glColor4bv( const v: PGLbyte ); external name '_glColor4bv';
procedure glColor4d( red: GLdouble; green: GLdouble; blue: GLdouble; alpha: GLdouble ); external name '_glColor4d';
procedure glColor4dv( const v: PGLdouble ); external name '_glColor4dv';
procedure glColor4f( red: GLfloat; green: GLfloat; blue: GLfloat; alpha: GLfloat ); external name '_glColor4f';
procedure glColor4fv( const v: PGLfloat ); external name '_glColor4fv';
procedure glColor4i( red: GLint; green: GLint; blue: GLint; alpha: GLint ); external name '_glColor4i';
procedure glColor4iv( const v: PGLint ); external name '_glColor4iv';
procedure glColor4s( red: GLshort; green: GLshort; blue: GLshort; alpha: GLshort ); external name '_glColor4s';
procedure glColor4sv( const v: PGLshort ); external name '_glColor4sv';
procedure glColor4ub( red: GLubyte; green: GLubyte; blue: GLubyte; alpha: GLubyte ); external name '_glColor4ub';
procedure glColor4ubv( const v: PGLubyte ); external name '_glColor4ubv';
procedure glColor4ui( red: GLuint; green: GLuint; blue: GLuint; alpha: GLuint ); external name '_glColor4ui';
procedure glColor4uiv( const v: PGLuint ); external name '_glColor4uiv';
procedure glColor4us( red: GLushort; green: GLushort; blue: GLushort; alpha: GLushort ); external name '_glColor4us';
procedure glColor4usv( const v: PGLushort ); external name '_glColor4usv';
procedure glColorMask( red: GLboolean; green: GLboolean; blue: GLboolean; alpha: GLboolean ); external name '_glColorMask';
procedure glColorMaterial( face: GLenum; mode: GLenum ); external name '_glColorMaterial';
procedure glColorPointer( size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr ); external name '_glColorPointer';
procedure glColorSubTable( target: GLenum; start: GLsizei; count: GLsizei; format: GLenum; typ: GLenum; const data: UnivPtr ); external name '_glColorSubTable';
procedure glColorTable( target: GLenum; internalformat: GLenum; width: GLsizei; format: GLenum; typ: GLenum; const table: UnivPtr ); external name '_glColorTable';
procedure glColorTableParameterfv( target: GLenum; pname: GLenum; const params: PGLfloat ); external name '_glColorTableParameterfv';
procedure glColorTableParameteriv( target: GLenum; pname: GLenum; const params: PGLint ); external name '_glColorTableParameteriv';
procedure glConvolutionFilter1D( target: GLenum; internalformat: GLenum; width: GLsizei; format: GLenum; typ: GLenum; const image: UnivPtr ); external name '_glConvolutionFilter1D';
procedure glConvolutionFilter2D( target: GLenum; internalformat: GLenum; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const image: UnivPtr ); external name '_glConvolutionFilter2D';
procedure glConvolutionParameterf( target: GLenum; pname: GLenum; params: GLfloat ); external name '_glConvolutionParameterf';
procedure glConvolutionParameterfv( target: GLenum; pname: GLenum; const params: PGLfloat ); external name '_glConvolutionParameterfv';
procedure glConvolutionParameteri( target: GLenum; pname: GLenum; params: GLint ); external name '_glConvolutionParameteri';
procedure glConvolutionParameteriv( target: GLenum; pname: GLenum; const params: PGLint ); external name '_glConvolutionParameteriv';
procedure glCopyColorSubTable( target: GLenum; start: GLsizei; x: GLint; y: GLint; width: GLsizei ); external name '_glCopyColorSubTable';
procedure glCopyColorTable( target: GLenum; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei ); external name '_glCopyColorTable';
procedure glCopyConvolutionFilter1D( target: GLenum; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei ); external name '_glCopyConvolutionFilter1D';
procedure glCopyConvolutionFilter2D( target: GLenum; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei; height: GLsizei ); external name '_glCopyConvolutionFilter2D';
procedure glCopyPixels( x: GLint; y: GLint; width: GLsizei; height: GLsizei; typ: GLenum ); external name '_glCopyPixels';
procedure glCopyTexImage1D( target: GLenum; level: GLint; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei; border: GLint ); external name '_glCopyTexImage1D';
procedure glCopyTexImage2D( target: GLenum; level: GLint; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei; height: GLsizei; border: GLint ); external name '_glCopyTexImage2D';
procedure glCopyTexSubImage1D( target: GLenum; level: GLint; xoffset: GLint; x: GLint; y: GLint; width: GLsizei ); external name '_glCopyTexSubImage1D';
procedure glCopyTexSubImage2D( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; x: GLint; y: GLint; width: GLsizei; height: GLsizei ); external name '_glCopyTexSubImage2D';
procedure glCopyTexSubImage3D( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; zoffset: GLint; x: GLint; y: GLint; width: GLsizei; height: GLsizei ); external name '_glCopyTexSubImage3D';
procedure glCullFace( mode: GLenum ); external name '_glCullFace';
procedure glDeleteLists( list: GLuint; range: GLsizei ); external name '_glDeleteLists';
procedure glDeleteTextures( n: GLsizei; const textures: PGLuint ); external name '_glDeleteTextures';
procedure glDepthFunc( func: GLenum ); external name '_glDepthFunc';
procedure glDepthMask( flag: GLboolean ); external name '_glDepthMask';
procedure glDepthRange( zNear: GLclampd; zFar: GLclampd ); external name '_glDepthRange';
procedure glDisable( cap: GLenum ); external name '_glDisable';
procedure glDisableClientState( arry: GLenum ); external name '_glDisableClientState';
procedure glDrawArrays( mode: GLenum; first: GLint; count: GLsizei ); external name '_glDrawArrays';
procedure glDrawBuffer( mode: GLenum ); external name '_glDrawBuffer';
procedure glDrawElements( mode: GLenum; count: GLsizei; typ: GLenum; const indices: UnivPtr ); external name '_glDrawElements';
procedure glDrawPixels( width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr ); external name '_glDrawPixels';
procedure glDrawRangeElements( mode: GLenum; start: GLuint; finish: GLuint; count: GLsizei; typ: GLenum; const indices: UnivPtr ); external name '_glDrawRangeElements';
procedure glEdgeFlag( flag: GLboolean ); external name '_glEdgeFlag';
procedure glEdgeFlagPointer( stride: GLsizei; const pointr: UnivPtr ); external name '_glEdgeFlagPointer';
procedure glEdgeFlagv( const flag: PGLboolean ); external name '_glEdgeFlagv';
procedure glEnable( cap: GLenum ); external name '_glEnable';
procedure glEnableClientState( arry: GLenum ); external name '_glEnableClientState';
procedure glEnd; external name '_glEnd';
procedure glEndList; external name '_glEndList';
procedure glEvalCoord1d( u: GLdouble ); external name '_glEvalCoord1d';
procedure glEvalCoord1dv( const u: PGLdouble ); external name '_glEvalCoord1dv';
procedure glEvalCoord1f( u: GLfloat ); external name '_glEvalCoord1f';
procedure glEvalCoord1fv( const u: PGLfloat ); external name '_glEvalCoord1fv';
procedure glEvalCoord2d( u: GLdouble; v: GLdouble ); external name '_glEvalCoord2d';
procedure glEvalCoord2dv( const u: PGLdouble ); external name '_glEvalCoord2dv';
procedure glEvalCoord2f( u: GLfloat; v: GLfloat ); external name '_glEvalCoord2f';
procedure glEvalCoord2fv( const u: PGLfloat ); external name '_glEvalCoord2fv';
procedure glEvalMesh1( mode: GLenum; i1: GLint; i2: GLint ); external name '_glEvalMesh1';
procedure glEvalMesh2( mode: GLenum; i1: GLint; i2: GLint; j1: GLint; j2: GLint ); external name '_glEvalMesh2';
procedure glEvalPoint1( i: GLint ); external name '_glEvalPoint1';
procedure glEvalPoint2( i: GLint; j: GLint ); external name '_glEvalPoint2';
procedure glFeedbackBuffer( size: GLsizei; typ: GLenum; buffer: PGLfloat ); external name '_glFeedbackBuffer';
procedure glFinish; external name '_glFinish';
procedure glFlush; external name '_glFlush';
procedure glFogf( pname: GLenum; param: GLfloat ); external name '_glFogf';
procedure glFogfv( pname: GLenum; const params: PGLfloat ); external name '_glFogfv';
procedure glFogi( pname: GLenum; param: GLint ); external name '_glFogi';
procedure glFogiv( pname: GLenum; const params: PGLint ); external name '_glFogiv';
procedure glFrontFace( mode: GLenum ); external name '_glFrontFace';
procedure glFrustum( left: GLdouble; right: GLdouble; bottom: GLdouble; top: GLdouble; zNear: GLdouble; zFar: GLdouble ); external name '_glFrustum';
function glGenLists( range: GLsizei ): GLuint; external name '_glGenLists';
procedure glGenTextures( n: GLsizei; textures: PGLuint ); external name '_glGenTextures';
procedure glGetBooleanv( pname: GLenum; params: PGLboolean ); external name '_glGetBooleanv';
procedure glGetClipPlane( plane: GLenum; equation: PGLdouble ); external name '_glGetClipPlane';
procedure glGetColorTable( target: GLenum; format: GLenum; typ: GLenum; table: UnivPtr ); external name '_glGetColorTable';
procedure glGetColorTableParameterfv( target: GLenum; pname: GLenum; params: PGLfloat ); external name '_glGetColorTableParameterfv';
procedure glGetColorTableParameteriv( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetColorTableParameteriv';
procedure glGetConvolutionFilter( target: GLenum; format: GLenum; typ: GLenum; image: UnivPtr ); external name '_glGetConvolutionFilter';
procedure glGetConvolutionParameterfv( target: GLenum; pname: GLenum; params: PGLfloat ); external name '_glGetConvolutionParameterfv';
procedure glGetConvolutionParameteriv( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetConvolutionParameteriv';
procedure glGetDoublev( pname: GLenum; params: PGLdouble ); external name '_glGetDoublev';
function glGetError: GLenum; external name '_glGetError';
procedure glGetFloatv( pname: GLenum; params: PGLfloat ); external name '_glGetFloatv';
procedure glGetHistogram( target: GLenum; reset: GLboolean; format: GLenum; typ: GLenum; values: UnivPtr ); external name '_glGetHistogram';
procedure glGetHistogramParameterfv( target: GLenum; pname: GLenum; params: PGLfloat ); external name '_glGetHistogramParameterfv';
procedure glGetHistogramParameteriv( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetHistogramParameteriv';
procedure glGetIntegerv( pname: GLenum; params: PGLint ); external name '_glGetIntegerv';
procedure glGetLightfv( light: GLenum; pname: GLenum; params: PGLfloat ); external name '_glGetLightfv';
procedure glGetLightiv( light: GLenum; pname: GLenum; params: PGLint ); external name '_glGetLightiv';
procedure glGetMapdv( target: GLenum; query: GLenum; v: PGLdouble ); external name '_glGetMapdv';
procedure glGetMapfv( target: GLenum; query: GLenum; v: PGLfloat ); external name '_glGetMapfv';
procedure glGetMapiv( target: GLenum; query: GLenum; v: PGLint ); external name '_glGetMapiv';
procedure glGetMaterialfv( face: GLenum; pname: GLenum; params: PGLfloat ); external name '_glGetMaterialfv';
procedure glGetMaterialiv( face: GLenum; pname: GLenum; params: PGLint ); external name '_glGetMaterialiv';
procedure glGetMinmax( target: GLenum; reset: GLboolean; format: GLenum; typ: GLenum; values: UnivPtr ); external name '_glGetMinmax';
procedure glGetMinmaxParameterfv( target: GLenum; pname: GLenum; params: PGLfloat ); external name '_glGetMinmaxParameterfv';
procedure glGetMinmaxParameteriv( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetMinmaxParameteriv';
procedure glGetPixelMapfv( map: GLenum; values: PGLfloat ); external name '_glGetPixelMapfv';
procedure glGetPixelMapuiv( map: GLenum; values: PGLuint ); external name '_glGetPixelMapuiv';
procedure glGetPixelMapusv( map: GLenum; values: PGLushort ); external name '_glGetPixelMapusv';
procedure glGetPointerv( pname: GLenum; params: UnivPtrPtr ); external name '_glGetPointerv';
procedure glGetPolygonStipple( mask: PGLubyte ); external name '_glGetPolygonStipple';
procedure glGetSeparableFilter( target: GLenum; format: GLenum; typ: GLenum; row: UnivPtr; column: UnivPtr; span: UnivPtr ); external name '_glGetSeparableFilter';
function glGetString( name: GLenum ): PChar; external name '_glGetString';
procedure glGetTexEnvfv( target: GLenum; pname: GLenum; params: PGLfloat ); external name '_glGetTexEnvfv';
procedure glGetTexEnviv( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetTexEnviv';
procedure glGetTexGendv( coord: GLenum; pname: GLenum; params: PGLdouble ); external name '_glGetTexGendv';
procedure glGetTexGenfv( coord: GLenum; pname: GLenum; params: PGLfloat ); external name '_glGetTexGenfv';
procedure glGetTexGeniv( coord: GLenum; pname: GLenum; params: PGLint ); external name '_glGetTexGeniv';
procedure glGetTexImage( target: GLenum; level: GLint; format: GLenum; typ: GLenum; pixels: UnivPtr ); external name '_glGetTexImage';
procedure glGetTexLevelParameterfv( target: GLenum; level: GLint; pname: GLenum; params: PGLfloat ); external name '_glGetTexLevelParameterfv';
procedure glGetTexLevelParameteriv( target: GLenum; level: GLint; pname: GLenum; params: PGLint ); external name '_glGetTexLevelParameteriv';
procedure glGetTexParameterfv( target: GLenum; pname: GLenum; params: PGLfloat ); external name '_glGetTexParameterfv';
procedure glGetTexParameteriv( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetTexParameteriv';
procedure glHint( target: GLenum; mode: GLenum ); external name '_glHint';
procedure glHistogram( target: GLenum; width: GLsizei; internalformat: GLenum; sink: GLboolean ); external name '_glHistogram';
procedure glIndexMask( mask: GLuint ); external name '_glIndexMask';
procedure glIndexPointer( typ: GLenum; stride: GLsizei; const pointr: UnivPtr ); external name '_glIndexPointer';
procedure glIndexd( c: GLdouble ); external name '_glIndexd';
procedure glIndexdv( const c: PGLdouble ); external name '_glIndexdv';
procedure glIndexf( c: GLfloat ); external name '_glIndexf';
procedure glIndexfv( const c: PGLfloat ); external name '_glIndexfv';
procedure glIndexi( c: GLint ); external name '_glIndexi';
procedure glIndexiv( const c: PGLint ); external name '_glIndexiv';
procedure glIndexs( c: GLshort ); external name '_glIndexs';
procedure glIndexsv( const c: PGLshort ); external name '_glIndexsv';
procedure glIndexub( c: GLubyte ); external name '_glIndexub';
procedure glIndexubv( const c: PGLubyte ); external name '_glIndexubv';
procedure glInitNames; external name '_glInitNames';
procedure glInterleavedArrays( format: GLenum; stride: GLsizei; const pointr: UnivPtr ); external name '_glInterleavedArrays';
function glIsEnabled( cap: GLenum ): GLboolean; external name '_glIsEnabled';
function glIsList( list: GLuint ): GLboolean; external name '_glIsList';
function glIsTexture( texture: GLuint ): GLboolean; external name '_glIsTexture';
procedure glLightModelf( pname: GLenum; param: GLfloat ); external name '_glLightModelf';
procedure glLightModelfv( pname: GLenum; const params: PGLfloat ); external name '_glLightModelfv';
procedure glLightModeli( pname: GLenum; param: GLint ); external name '_glLightModeli';
procedure glLightModeliv( pname: GLenum; const params: PGLint ); external name '_glLightModeliv';
procedure glLightf( light: GLenum; pname: GLenum; param: GLfloat ); external name '_glLightf';
procedure glLightfv( light: GLenum; pname: GLenum; const params: PGLfloat ); external name '_glLightfv';
procedure glLighti( light: GLenum; pname: GLenum; param: GLint ); external name '_glLighti';
procedure glLightiv( light: GLenum; pname: GLenum; const params: PGLint ); external name '_glLightiv';
procedure glLineStipple( factor: GLint; pattern: GLushort ); external name '_glLineStipple';
procedure glLineWidth( width: GLfloat ); external name '_glLineWidth';
procedure glListBase( base: GLuint ); external name '_glListBase';
procedure glLoadIdentity; external name '_glLoadIdentity';
procedure glLoadMatrixd( const m: PGLdouble ); external name '_glLoadMatrixd';
procedure glLoadMatrixf( const m: PGLfloat ); external name '_glLoadMatrixf';
procedure glLoadName( name: GLuint ); external name '_glLoadName';
procedure glLogicOp( opcode: GLenum ); external name '_glLogicOp';
procedure glMap1d( target: GLenum; u1: GLdouble; u2: GLdouble; stride: GLint; order: GLint; const points: PGLdouble ); external name '_glMap1d';
procedure glMap1f( target: GLenum; u1: GLfloat; u2: GLfloat; stride: GLint; order: GLint; const points: PGLfloat ); external name '_glMap1f';
procedure glMap2d( target: GLenum; u1: GLdouble; u2: GLdouble; ustride: GLint; uorder: GLint; v1: GLdouble; v2: GLdouble; vstride: GLint; vorder: GLint; const points: PGLdouble ); external name '_glMap2d';
procedure glMap2f( target: GLenum; u1: GLfloat; u2: GLfloat; ustride: GLint; uorder: GLint; v1: GLfloat; v2: GLfloat; vstride: GLint; vorder: GLint; const points: PGLfloat ); external name '_glMap2f';
procedure glMapGrid1d( un: GLint; u1: GLdouble; u2: GLdouble ); external name '_glMapGrid1d';
procedure glMapGrid1f( un: GLint; u1: GLfloat; u2: GLfloat ); external name '_glMapGrid1f';
procedure glMapGrid2d( un: GLint; u1: GLdouble; u2: GLdouble; vn: GLint; v1: GLdouble; v2: GLdouble ); external name '_glMapGrid2d';
procedure glMapGrid2f( un: GLint; u1: GLfloat; u2: GLfloat; vn: GLint; v1: GLfloat; v2: GLfloat ); external name '_glMapGrid2f';
procedure glMaterialf( face: GLenum; pname: GLenum; param: GLfloat ); external name '_glMaterialf';
procedure glMaterialfv( face: GLenum; pname: GLenum; const params: PGLfloat ); external name '_glMaterialfv';
procedure glMateriali( face: GLenum; pname: GLenum; param: GLint ); external name '_glMateriali';
procedure glMaterialiv( face: GLenum; pname: GLenum; const params: PGLint ); external name '_glMaterialiv';
procedure glMatrixMode( mode: GLenum ); external name '_glMatrixMode';
procedure glMinmax( target: GLenum; internalformat: GLenum; sink: GLboolean ); external name '_glMinmax';
procedure glMultMatrixd( const m: PGLdouble ); external name '_glMultMatrixd';
procedure glMultMatrixf( const m: PGLfloat ); external name '_glMultMatrixf';
procedure glNewList( list: GLuint; mode: GLenum ); external name '_glNewList';
procedure glNormal3b( nx: GLbyte; ny: GLbyte; nz: GLbyte ); external name '_glNormal3b';
procedure glNormal3bv( const v: PGLbyte ); external name '_glNormal3bv';
procedure glNormal3d( nx: GLdouble; ny: GLdouble; nz: GLdouble ); external name '_glNormal3d';
procedure glNormal3dv( const v: PGLdouble ); external name '_glNormal3dv';
procedure glNormal3f( nx: GLfloat; ny: GLfloat; nz: GLfloat ); external name '_glNormal3f';
procedure glNormal3fv( const v: PGLfloat ); external name '_glNormal3fv';
procedure glNormal3i( nx: GLint; ny: GLint; nz: GLint ); external name '_glNormal3i';
procedure glNormal3iv( const v: PGLint ); external name '_glNormal3iv';
procedure glNormal3s( nx: GLshort; ny: GLshort; nz: GLshort ); external name '_glNormal3s';
procedure glNormal3sv( const v: PGLshort ); external name '_glNormal3sv';
procedure glNormalPointer( typ: GLenum; stride: GLsizei; const pointr: UnivPtr ); external name '_glNormalPointer';
procedure glOrtho( left: GLdouble; right: GLdouble; bottom: GLdouble; top: GLdouble; zNear: GLdouble; zFar: GLdouble ); external name '_glOrtho';
procedure glPassThrough( token: GLfloat ); external name '_glPassThrough';
procedure glPixelMapfv( map: GLenum; mapsize: GLint; const values: PGLfloat ); external name '_glPixelMapfv';
procedure glPixelMapuiv( map: GLenum; mapsize: GLint; const values: PGLuint ); external name '_glPixelMapuiv';
procedure glPixelMapusv( map: GLenum; mapsize: GLint; const values: PGLushort ); external name '_glPixelMapusv';
procedure glPixelStoref( pname: GLenum; param: GLfloat ); external name '_glPixelStoref';
procedure glPixelStorei( pname: GLenum; param: GLint ); external name '_glPixelStorei';
procedure glPixelTransferf( pname: GLenum; param: GLfloat ); external name '_glPixelTransferf';
procedure glPixelTransferi( pname: GLenum; param: GLint ); external name '_glPixelTransferi';
procedure glPixelZoom( xfactor: GLfloat; yfactor: GLfloat ); external name '_glPixelZoom';
procedure glPointSize( size: GLfloat ); external name '_glPointSize';
procedure glPolygonMode( face: GLenum; mode: GLenum ); external name '_glPolygonMode';
procedure glPolygonOffset( factor: GLfloat; units: GLfloat ); external name '_glPolygonOffset';
procedure glPolygonStipple( const mask: PGLubyte ); external name '_glPolygonStipple';
procedure glPopAttrib; external name '_glPopAttrib';
procedure glPopClientAttrib; external name '_glPopClientAttrib';
procedure glPopMatrix; external name '_glPopMatrix';
procedure glPopName; external name '_glPopName';
procedure glPrioritizeTextures( n: GLsizei; const textures: PGLuint; const priorities: PGLclampf ); external name '_glPrioritizeTextures';
procedure glPushAttrib( mask: GLbitfield ); external name '_glPushAttrib';
procedure glPushClientAttrib( mask: GLbitfield ); external name '_glPushClientAttrib';
procedure glPushMatrix; external name '_glPushMatrix';
procedure glPushName( name: GLuint ); external name '_glPushName';
procedure glRasterPos2d( x: GLdouble; y: GLdouble ); external name '_glRasterPos2d';
procedure glRasterPos2dv( const v: PGLdouble ); external name '_glRasterPos2dv';
procedure glRasterPos2f( x: GLfloat; y: GLfloat ); external name '_glRasterPos2f';
procedure glRasterPos2fv( const v: PGLfloat ); external name '_glRasterPos2fv';
procedure glRasterPos2i( x: GLint; y: GLint ); external name '_glRasterPos2i';
procedure glRasterPos2iv( const v: PGLint ); external name '_glRasterPos2iv';
procedure glRasterPos2s( x: GLshort; y: GLshort ); external name '_glRasterPos2s';
procedure glRasterPos2sv( const v: PGLshort ); external name '_glRasterPos2sv';
procedure glRasterPos3d( x: GLdouble; y: GLdouble; z: GLdouble ); external name '_glRasterPos3d';
procedure glRasterPos3dv( const v: PGLdouble ); external name '_glRasterPos3dv';
procedure glRasterPos3f( x: GLfloat; y: GLfloat; z: GLfloat ); external name '_glRasterPos3f';
procedure glRasterPos3fv( const v: PGLfloat ); external name '_glRasterPos3fv';
procedure glRasterPos3i( x: GLint; y: GLint; z: GLint ); external name '_glRasterPos3i';
procedure glRasterPos3iv( const v: PGLint ); external name '_glRasterPos3iv';
procedure glRasterPos3s( x: GLshort; y: GLshort; z: GLshort ); external name '_glRasterPos3s';
procedure glRasterPos3sv( const v: PGLshort ); external name '_glRasterPos3sv';
procedure glRasterPos4d( x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble ); external name '_glRasterPos4d';
procedure glRasterPos4dv( const v: PGLdouble ); external name '_glRasterPos4dv';
procedure glRasterPos4f( x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat ); external name '_glRasterPos4f';
procedure glRasterPos4fv( const v: PGLfloat ); external name '_glRasterPos4fv';
procedure glRasterPos4i( x: GLint; y: GLint; z: GLint; w: GLint ); external name '_glRasterPos4i';
procedure glRasterPos4iv( const v: PGLint ); external name '_glRasterPos4iv';
procedure glRasterPos4s( x: GLshort; y: GLshort; z: GLshort; w: GLshort ); external name '_glRasterPos4s';
procedure glRasterPos4sv( const v: PGLshort ); external name '_glRasterPos4sv';
procedure glReadBuffer( mode: GLenum ); external name '_glReadBuffer';
procedure glReadPixels( x: GLint; y: GLint; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; pixels: UnivPtr ); external name '_glReadPixels';
procedure glRectd( x1: GLdouble; y1: GLdouble; x2: GLdouble; y2: GLdouble ); external name '_glRectd';
procedure glRectdv( const v1: PGLdouble; const v2: PGLdouble ); external name '_glRectdv';
procedure glRectf( x1: GLfloat; y1: GLfloat; x2: GLfloat; y2: GLfloat ); external name '_glRectf';
procedure glRectfv( const v1: PGLfloat; const v2: PGLfloat ); external name '_glRectfv';
procedure glRecti( x1: GLint; y1: GLint; x2: GLint; y2: GLint ); external name '_glRecti';
procedure glRectiv( const v1: PGLint; const v2: PGLint ); external name '_glRectiv';
procedure glRects( x1: GLshort; y1: GLshort; x2: GLshort; y2: GLshort ); external name '_glRects';
procedure glRectsv( const v1: PGLshort; const v2: PGLshort ); external name '_glRectsv';
function glRenderMode( mode: GLenum ): GLint; external name '_glRenderMode';
procedure glResetHistogram( target: GLenum ); external name '_glResetHistogram';
procedure glResetMinmax( target: GLenum ); external name '_glResetMinmax';
procedure glRotated( angle: GLdouble; x: GLdouble; y: GLdouble; z: GLdouble ); external name '_glRotated';
procedure glRotatef( angle: GLfloat; x: GLfloat; y: GLfloat; z: GLfloat ); external name '_glRotatef';
procedure glScaled( x: GLdouble; y: GLdouble; z: GLdouble ); external name '_glScaled';
procedure glScalef( x: GLfloat; y: GLfloat; z: GLfloat ); external name '_glScalef';
procedure glScissor( x: GLint; y: GLint; width: GLsizei; height: GLsizei ); external name '_glScissor';
procedure glSelectBuffer( size: GLsizei; buffer: PGLuint ); external name '_glSelectBuffer';
procedure glSeparableFilter2D( target: GLenum; internalformat: GLenum; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const row: UnivPtr; const column: UnivPtr ); external name '_glSeparableFilter2D';
procedure glShadeModel( mode: GLenum ); external name '_glShadeModel';
procedure glStencilFunc( func: GLenum; ref: GLint; mask: GLuint ); external name '_glStencilFunc';
procedure glStencilMask( mask: GLuint ); external name '_glStencilMask';
procedure glStencilOp( fail: GLenum; zfail: GLenum; zpass: GLenum ); external name '_glStencilOp';
procedure glTexCoord1d( s: GLdouble ); external name '_glTexCoord1d';
procedure glTexCoord1dv( const v: PGLdouble ); external name '_glTexCoord1dv';
procedure glTexCoord1f( s: GLfloat ); external name '_glTexCoord1f';
procedure glTexCoord1fv( const v: PGLfloat ); external name '_glTexCoord1fv';
procedure glTexCoord1i( s: GLint ); external name '_glTexCoord1i';
procedure glTexCoord1iv( const v: PGLint ); external name '_glTexCoord1iv';
procedure glTexCoord1s( s: GLshort ); external name '_glTexCoord1s';
procedure glTexCoord1sv( const v: PGLshort ); external name '_glTexCoord1sv';
procedure glTexCoord2d( s: GLdouble; t: GLdouble ); external name '_glTexCoord2d';
procedure glTexCoord2dv( const v: PGLdouble ); external name '_glTexCoord2dv';
procedure glTexCoord2f( s: GLfloat; t: GLfloat ); external name '_glTexCoord2f';
procedure glTexCoord2fv( const v: PGLfloat ); external name '_glTexCoord2fv';
procedure glTexCoord2i( s: GLint; t: GLint ); external name '_glTexCoord2i';
procedure glTexCoord2iv( const v: PGLint ); external name '_glTexCoord2iv';
procedure glTexCoord2s( s: GLshort; t: GLshort ); external name '_glTexCoord2s';
procedure glTexCoord2sv( const v: PGLshort ); external name '_glTexCoord2sv';
procedure glTexCoord3d( s: GLdouble; t: GLdouble; r: GLdouble ); external name '_glTexCoord3d';
procedure glTexCoord3dv( const v: PGLdouble ); external name '_glTexCoord3dv';
procedure glTexCoord3f( s: GLfloat; t: GLfloat; r: GLfloat ); external name '_glTexCoord3f';
procedure glTexCoord3fv( const v: PGLfloat ); external name '_glTexCoord3fv';
procedure glTexCoord3i( s: GLint; t: GLint; r: GLint ); external name '_glTexCoord3i';
procedure glTexCoord3iv( const v: PGLint ); external name '_glTexCoord3iv';
procedure glTexCoord3s( s: GLshort; t: GLshort; r: GLshort ); external name '_glTexCoord3s';
procedure glTexCoord3sv( const v: PGLshort ); external name '_glTexCoord3sv';
procedure glTexCoord4d( s: GLdouble; t: GLdouble; r: GLdouble; q: GLdouble ); external name '_glTexCoord4d';
procedure glTexCoord4dv( const v: PGLdouble ); external name '_glTexCoord4dv';
procedure glTexCoord4f( s: GLfloat; t: GLfloat; r: GLfloat; q: GLfloat ); external name '_glTexCoord4f';
procedure glTexCoord4fv( const v: PGLfloat ); external name '_glTexCoord4fv';
procedure glTexCoord4i( s: GLint; t: GLint; r: GLint; q: GLint ); external name '_glTexCoord4i';
procedure glTexCoord4iv( const v: PGLint ); external name '_glTexCoord4iv';
procedure glTexCoord4s( s: GLshort; t: GLshort; r: GLshort; q: GLshort ); external name '_glTexCoord4s';
procedure glTexCoord4sv( const v: PGLshort ); external name '_glTexCoord4sv';
procedure glTexCoordPointer( size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr ); external name '_glTexCoordPointer';
procedure glTexEnvf( target: GLenum; pname: GLenum; param: GLfloat ); external name '_glTexEnvf';
procedure glTexEnvfv( target: GLenum; pname: GLenum; const params: PGLfloat ); external name '_glTexEnvfv';
procedure glTexEnvi( target: GLenum; pname: GLenum; param: GLint ); external name '_glTexEnvi';
procedure glTexEnviv( target: GLenum; pname: GLenum; const params: PGLint ); external name '_glTexEnviv';
procedure glTexGend( coord: GLenum; pname: GLenum; param: GLdouble ); external name '_glTexGend';
procedure glTexGendv( coord: GLenum; pname: GLenum; const params: PGLdouble ); external name '_glTexGendv';
procedure glTexGenf( coord: GLenum; pname: GLenum; param: GLfloat ); external name '_glTexGenf';
procedure glTexGenfv( coord: GLenum; pname: GLenum; const params: PGLfloat ); external name '_glTexGenfv';
procedure glTexGeni( coord: GLenum; pname: GLenum; param: GLint ); external name '_glTexGeni';
procedure glTexGeniv( coord: GLenum; pname: GLenum; const params: PGLint ); external name '_glTexGeniv';
procedure glTexImage1D( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; border: GLint; format: GLenum; typ: GLenum; const pixels: UnivPtr ); external name '_glTexImage1D';
procedure glTexImage2D( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; height: GLsizei; border: GLint; format: GLenum; typ: GLenum; const pixels: UnivPtr ); external name '_glTexImage2D';
procedure glTexImage3D( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; height: GLsizei; depth: GLsizei; border: GLint; format: GLenum; typ: GLenum; const pixels: UnivPtr ); external name '_glTexImage3D';
procedure glTexParameterf( target: GLenum; pname: GLenum; param: GLfloat ); external name '_glTexParameterf';
procedure glTexParameterfv( target: GLenum; pname: GLenum; const params: PGLfloat ); external name '_glTexParameterfv';
procedure glTexParameteri( target: GLenum; pname: GLenum; param: GLint ); external name '_glTexParameteri';
procedure glTexParameteriv( target: GLenum; pname: GLenum; const params: PGLint ); external name '_glTexParameteriv';
procedure glTexSubImage1D( target: GLenum; level: GLint; xoffset: GLint; width: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr ); external name '_glTexSubImage1D';
procedure glTexSubImage2D( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr ); external name '_glTexSubImage2D';
procedure glTexSubImage3D( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; zoffset: GLint; width: GLsizei; height: GLsizei; depth: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr ); external name '_glTexSubImage3D';
procedure glTranslated( x: GLdouble; y: GLdouble; z: GLdouble ); external name '_glTranslated';
procedure glTranslatef( x: GLfloat; y: GLfloat; z: GLfloat ); external name '_glTranslatef';
procedure glVertex2d( x: GLdouble; y: GLdouble ); external name '_glVertex2d';
procedure glVertex2dv( const v: PGLdouble ); external name '_glVertex2dv';
procedure glVertex2f( x: GLfloat; y: GLfloat ); external name '_glVertex2f';
procedure glVertex2fv( const v: PGLfloat ); external name '_glVertex2fv';
procedure glVertex2i( x: GLint; y: GLint ); external name '_glVertex2i';
procedure glVertex2iv( const v: PGLint ); external name '_glVertex2iv';
procedure glVertex2s( x: GLshort; y: GLshort ); external name '_glVertex2s';
procedure glVertex2sv( const v: PGLshort ); external name '_glVertex2sv';
procedure glVertex3d( x: GLdouble; y: GLdouble; z: GLdouble ); external name '_glVertex3d';
procedure glVertex3dv( const v: PGLdouble ); external name '_glVertex3dv';
procedure glVertex3f( x: GLfloat; y: GLfloat; z: GLfloat ); external name '_glVertex3f';
procedure glVertex3fv( const v: PGLfloat ); external name '_glVertex3fv';
procedure glVertex3i( x: GLint; y: GLint; z: GLint ); external name '_glVertex3i';
procedure glVertex3iv( const v: PGLint ); external name '_glVertex3iv';
procedure glVertex3s( x: GLshort; y: GLshort; z: GLshort ); external name '_glVertex3s';
procedure glVertex3sv( const v: PGLshort ); external name '_glVertex3sv';
procedure glVertex4d( x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble ); external name '_glVertex4d';
procedure glVertex4dv( const v: PGLdouble ); external name '_glVertex4dv';
procedure glVertex4f( x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat ); external name '_glVertex4f';
procedure glVertex4fv( const v: PGLfloat ); external name '_glVertex4fv';
procedure glVertex4i( x: GLint; y: GLint; z: GLint; w: GLint ); external name '_glVertex4i';
procedure glVertex4iv( const v: PGLint ); external name '_glVertex4iv';
procedure glVertex4s( x: GLshort; y: GLshort; z: GLshort; w: GLshort ); external name '_glVertex4s';
procedure glVertex4sv( const v: PGLshort ); external name '_glVertex4sv';
procedure glVertexPointer( size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr ); external name '_glVertexPointer';
procedure glViewport( x: GLint; y: GLint; width: GLsizei; height: GLsizei ); external name '_glViewport';

procedure glSampleCoverage( value: GLclampf; invert: GLboolean ); external name '_glSampleCoverage';
procedure glSamplePass( pass: GLenum ); external name '_glSamplePass';

procedure glLoadTransposeMatrixf( const m: PGLfloat ); external name '_glLoadTransposeMatrixf';
procedure glLoadTransposeMatrixd( const m: PGLdouble ); external name '_glLoadTransposeMatrixd';
procedure glMultTransposeMatrixf( const m: PGLfloat ); external name '_glMultTransposeMatrixf';
procedure glMultTransposeMatrixd( const m: PGLdouble ); external name '_glMultTransposeMatrixd';

procedure glCompressedTexImage3D( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; height: GLsizei; depth: GLsizei; border: GLint; imageSize: GLsizei; const data: UnivPtr ); external name '_glCompressedTexImage3D';
procedure glCompressedTexImage2D( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; height: GLsizei; border: GLint; imageSize: GLsizei; const data: UnivPtr ); external name '_glCompressedTexImage2D';
procedure glCompressedTexImage1D( target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; border: GLint; imageSize: GLsizei; const data: UnivPtr ); external name '_glCompressedTexImage1D';
procedure glCompressedTexSubImage3D( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; zoffset: GLint; width: GLsizei; height: GLsizei; depth: GLsizei; format: GLenum; imageSize: GLsizei; const data: UnivPtr ); external name '_glCompressedTexSubImage3D';
procedure glCompressedTexSubImage2D( target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; width: GLsizei; height: GLsizei; format: GLenum; imageSize: GLsizei; const data: UnivPtr ); external name '_glCompressedTexSubImage2D';
procedure glCompressedTexSubImage1D( target: GLenum; level: GLint; xoffset: GLint; width: GLsizei; format: GLenum; imageSize: GLsizei; const data: UnivPtr ); external name '_glCompressedTexSubImage1D';
procedure glGetCompressedTexImage( target: GLenum; lod: GLint; img: UnivPtr ); external name '_glGetCompressedTexImage';

procedure glActiveTexture( texture: GLenum ); external name '_glActiveTexture';
procedure glClientActiveTexture( texture: GLenum ); external name '_glClientActiveTexture';
procedure glMultiTexCoord1d( target: GLenum; s: GLdouble ); external name '_glMultiTexCoord1d';
procedure glMultiTexCoord1dv( target: GLenum; const v: PGLdouble ); external name '_glMultiTexCoord1dv';
procedure glMultiTexCoord1f( target: GLenum; s: GLfloat ); external name '_glMultiTexCoord1f';
procedure glMultiTexCoord1fv( target: GLenum; const v: PGLfloat ); external name '_glMultiTexCoord1fv';
procedure glMultiTexCoord1i( target: GLenum; s: GLint ); external name '_glMultiTexCoord1i';
procedure glMultiTexCoord1iv( target: GLenum; const v: PGLint ); external name '_glMultiTexCoord1iv';
procedure glMultiTexCoord1s( target: GLenum; s: GLshort ); external name '_glMultiTexCoord1s';
procedure glMultiTexCoord1sv( target: GLenum; const v: PGLshort ); external name '_glMultiTexCoord1sv';
procedure glMultiTexCoord2d( target: GLenum; s: GLdouble; t: GLdouble ); external name '_glMultiTexCoord2d';
procedure glMultiTexCoord2dv( target: GLenum; const v: PGLdouble ); external name '_glMultiTexCoord2dv';
procedure glMultiTexCoord2f( target: GLenum; s: GLfloat; t: GLfloat ); external name '_glMultiTexCoord2f';
procedure glMultiTexCoord2fv( target: GLenum; const v: PGLfloat ); external name '_glMultiTexCoord2fv';
procedure glMultiTexCoord2i( target: GLenum; s: GLint; t: GLint ); external name '_glMultiTexCoord2i';
procedure glMultiTexCoord2iv( target: GLenum; const v: PGLint ); external name '_glMultiTexCoord2iv';
procedure glMultiTexCoord2s( target: GLenum; s: GLshort; t: GLshort ); external name '_glMultiTexCoord2s';
procedure glMultiTexCoord2sv( target: GLenum; const v: PGLshort ); external name '_glMultiTexCoord2sv';
procedure glMultiTexCoord3d( target: GLenum; s: GLdouble; t: GLdouble; r: GLdouble ); external name '_glMultiTexCoord3d';
procedure glMultiTexCoord3dv( target: GLenum; const v: PGLdouble ); external name '_glMultiTexCoord3dv';
procedure glMultiTexCoord3f( target: GLenum; s: GLfloat; t: GLfloat; r: GLfloat ); external name '_glMultiTexCoord3f';
procedure glMultiTexCoord3fv( target: GLenum; const v: PGLfloat ); external name '_glMultiTexCoord3fv';
procedure glMultiTexCoord3i( target: GLenum; s: GLint; t: GLint; r: GLint ); external name '_glMultiTexCoord3i';
procedure glMultiTexCoord3iv( target: GLenum; const v: PGLint ); external name '_glMultiTexCoord3iv';
procedure glMultiTexCoord3s( target: GLenum; s: GLshort; t: GLshort; r: GLshort ); external name '_glMultiTexCoord3s';
procedure glMultiTexCoord3sv( target: GLenum; const v: PGLshort ); external name '_glMultiTexCoord3sv';
procedure glMultiTexCoord4d( target: GLenum; s: GLdouble; t: GLdouble; r: GLdouble; q: GLdouble ); external name '_glMultiTexCoord4d';
procedure glMultiTexCoord4dv( target: GLenum; const v: PGLdouble ); external name '_glMultiTexCoord4dv';
procedure glMultiTexCoord4f( target: GLenum; s: GLfloat; t: GLfloat; r: GLfloat; q: GLfloat ); external name '_glMultiTexCoord4f';
procedure glMultiTexCoord4fv( target: GLenum; const v: PGLfloat ); external name '_glMultiTexCoord4fv';
procedure glMultiTexCoord4i( target: GLenum; param1 : GLint; s: GLint; t: GLint; r: GLint ); external name '_glMultiTexCoord4i';
procedure glMultiTexCoord4iv( target: GLenum; const v: PGLint ); external name '_glMultiTexCoord4iv';
procedure glMultiTexCoord4s( target: GLenum; s: GLshort; t: GLshort; r: GLshort; q: GLshort ); external name '_glMultiTexCoord4s';
procedure glMultiTexCoord4sv( target: GLenum; const v: PGLshort ); external name '_glMultiTexCoord4sv';

procedure glFogCoordf( coord: GLfloat ); external name '_glFogCoordf';
procedure glFogCoordfv( const coord: PGLfloat ); external name '_glFogCoordfv';  
procedure glFogCoordd( coord: GLdouble ); external name '_glFogCoordd';
procedure glFogCoorddv( const coord: PGLdouble ); external name '_glFogCoorddv';   
procedure glFogCoordPointer( typ: GLenum; stride: GLsizei; const pointr: UnivPtr ); external name '_glFogCoordPointer';

procedure glSecondaryColor3b( red: GLbyte; green: GLbyte; blue: GLbyte ); external name '_glSecondaryColor3b';
procedure glSecondaryColor3bv( const v: PGLbyte ); external name '_glSecondaryColor3bv';
procedure glSecondaryColor3d( red: GLdouble; green: GLdouble; blue: GLdouble ); external name '_glSecondaryColor3d';
procedure glSecondaryColor3dv( const v: PGLdouble ); external name '_glSecondaryColor3dv';
procedure glSecondaryColor3f( red: GLfloat; green: GLfloat; blue: GLfloat ); external name '_glSecondaryColor3f';
procedure glSecondaryColor3fv( const v: PGLfloat ); external name '_glSecondaryColor3fv';
procedure glSecondaryColor3i( red: GLint; green: GLint; blue: GLint ); external name '_glSecondaryColor3i';
procedure glSecondaryColor3iv( const v: PGLint ); external name '_glSecondaryColor3iv';
procedure glSecondaryColor3s( red: GLshort; green: GLshort; blue: GLshort ); external name '_glSecondaryColor3s';
procedure glSecondaryColor3sv( const v: PGLshort ); external name '_glSecondaryColor3sv';
procedure glSecondaryColor3ub( red: GLubyte; green: GLubyte; blue: GLubyte ); external name '_glSecondaryColor3ub';
procedure glSecondaryColor3ubv( const v: PGLubyte ); external name '_glSecondaryColor3ubv';
procedure glSecondaryColor3ui( red: GLuint; green: GLuint; blue: GLuint ); external name '_glSecondaryColor3ui';
procedure glSecondaryColor3uiv( const v: PGLuint ); external name '_glSecondaryColor3uiv';
procedure glSecondaryColor3us( red: GLushort; green: GLushort; blue: GLushort ); external name '_glSecondaryColor3us';
procedure glSecondaryColor3usv( const v: PGLushort ); external name '_glSecondaryColor3usv';
procedure glSecondaryColorPointer( size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr ); external name '_glSecondaryColorPointer';

procedure glPointParameterf( pname: GLenum; param: GLfloat ); external name '_glPointParameterf'; 
procedure glPointParameterfv( pname: GLenum; const params: PGLfloat ); external name '_glPointParameterfv';
procedure glPointParameteri( pname: GLenum; param: GLint ); external name '_glPointParameteri'; 
procedure glPointParameteriv( pname: GLenum; const params: PGLint ); external name '_glPointParameteriv';

procedure glBlendFuncSeparate( srcRGB: GLenum; dstRGB: GLenum; srcAlpha: GLenum; dstAlpha: GLenum ); external name '_glBlendFuncSeparate';

procedure glMultiDrawArrays( mode: GLenum; const first: PGLint; const count: PGLsizei; primcount: GLsizei ); external name '_glMultiDrawArrays';
procedure glMultiDrawElements( mode: GLenum; const count: PGLsizei; typ: GLenum; {const} indices: UnivPtr; primcount: GLsizei ); external name '_glMultiDrawElements';

procedure glWindowPos2d( x: GLdouble; y: GLdouble ); external name '_glWindowPos2d';
procedure glWindowPos2dv( const v: PGLdouble ); external name '_glWindowPos2dv';
procedure glWindowPos2f( x: GLfloat; y: GLfloat ); external name '_glWindowPos2f';
procedure glWindowPos2fv( const v: PGLfloat ); external name '_glWindowPos2fv';
procedure glWindowPos2i( x: GLint; y: GLint ); external name '_glWindowPos2i'; 
procedure glWindowPos2iv( const v: PGLint ); external name '_glWindowPos2iv';
procedure glWindowPos2s( x: GLshort; y: GLshort ); external name '_glWindowPos2s';
procedure glWindowPos2sv( const v: PGLshort ); external name '_glWindowPos2sv';
procedure glWindowPos3d( x: GLdouble; y: GLdouble; z: GLdouble ); external name '_glWindowPos3d';
procedure glWindowPos3dv( const v: PGLdouble ); external name '_glWindowPos3dv';
procedure glWindowPos3f( x: GLfloat; y: GLfloat; z: GLfloat ); external name '_glWindowPos3f';
procedure glWindowPos3fv( const v: PGLfloat ); external name '_glWindowPos3fv';
procedure glWindowPos3i( x: GLint; y: GLint; z: GLint ); external name '_glWindowPos3i';
procedure glWindowPos3iv( const v: PGLint ); external name '_glWindowPos3iv';
procedure glWindowPos3s( x: GLshort; y: GLshort; z: GLshort ); external name '_glWindowPos3s';
procedure glWindowPos3sv( const v: PGLshort ); external name '_glWindowPos3sv';

procedure glGenQueries( n: GLsizei; ids: PGLuint ); external name '_glGenQueries';
procedure glDeleteQueries( n: GLsizei; const ids: PGLuint ); external name '_glDeleteQueries';
function glIsQuery( id: GLuint ): GLboolean; external name '_glIsQuery';
procedure glBeginQuery( target: GLenum; id: GLuint ); external name '_glBeginQuery';
procedure glEndQuery( target: GLenum ); external name '_glEndQuery';
procedure glGetQueryiv( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetQueryiv';
procedure glGetQueryObjectiv( id: GLuint; pname: GLenum; params: PGLint ); external name '_glGetQueryObjectiv';
procedure glGetQueryObjectuiv( id: GLuint; pname: GLenum; params: PGLuint ); external name '_glGetQueryObjectuiv';

procedure glBindBuffer( target: GLenum; buffer: GLuint ); external name '_glBindBuffer';
procedure glDeleteBuffers( n: GLsizei; const buffers: PGLuint ); external name '_glDeleteBuffers';
procedure glGenBuffers( n: GLsizei; buffers: PGLuint ); external name '_glGenBuffers';
function glIsBuffer( buffer: GLuint ): GLboolean; external name '_glIsBuffer';
procedure glBufferData( target: GLenum; size: GLsizeiptr; const data: UnivPtr; usage: GLenum ); external name '_glBufferData';
procedure glBufferSubData( target: GLenum; offset: GLintptr; size: GLsizeiptr; const data: UnivPtr ); external name '_glBufferSubData';
procedure glGetBufferSubData( target: GLenum; offset: GLintptr; size: GLsizeiptr; data: UnivPtr ); external name '_glGetBufferSubData';
function glMapBuffer( target: GLenum; access: GLenum ): UnivPtr; external name '_glMapBuffer';
function glUnmapBuffer( target: GLenum ): GLboolean; external name '_glUnmapBuffer';
procedure glGetBufferParameteriv( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetBufferParameteriv';
procedure glGetBufferPointerv( target: GLenum; pname: GLenum; params: UnivPtr ); external name '_glGetBufferPointerv';

procedure glDrawBuffers( n: GLsizei; const bufs: PGLenum ); external name '_glDrawBuffers';
procedure glVertexAttrib1d( index: GLuint; x: GLdouble ); external name '_glVertexAttrib1d';
procedure glVertexAttrib1dv( index: GLuint; const v: PGLdouble ); external name '_glVertexAttrib1dv';
procedure glVertexAttrib1f( index: GLuint; x: GLfloat ); external name '_glVertexAttrib1f';
procedure glVertexAttrib1fv( index: GLuint; const v: PGLfloat ); external name '_glVertexAttrib1fv';
procedure glVertexAttrib1s( index: GLuint; x: GLshort ); external name '_glVertexAttrib1s';
procedure glVertexAttrib1sv( index: GLuint; const v: PGLshort ); external name '_glVertexAttrib1sv';
procedure glVertexAttrib2d( index: GLuint; x: GLdouble; y: GLdouble ); external name '_glVertexAttrib2d';
procedure glVertexAttrib2dv( index: GLuint; const v: PGLdouble ); external name '_glVertexAttrib2dv';
procedure glVertexAttrib2f( index: GLuint; x: GLfloat; y: GLfloat ); external name '_glVertexAttrib2f';
procedure glVertexAttrib2fv( index: GLuint; const v: PGLfloat ); external name '_glVertexAttrib2fv';
procedure glVertexAttrib2s( index: GLuint; x: GLshort; y: GLshort ); external name '_glVertexAttrib2s';
procedure glVertexAttrib2sv( index: GLuint; const v: PGLshort ); external name '_glVertexAttrib2sv';
procedure glVertexAttrib3d( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble ); external name '_glVertexAttrib3d';
procedure glVertexAttrib3dv( index: GLuint; const v: PGLdouble ); external name '_glVertexAttrib3dv';
procedure glVertexAttrib3f( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat ); external name '_glVertexAttrib3f';
procedure glVertexAttrib3fv( index: GLuint; const v: PGLfloat ); external name '_glVertexAttrib3fv';
procedure glVertexAttrib3s( index: GLuint; x: GLshort; y: GLshort; z: GLshort ); external name '_glVertexAttrib3s';
procedure glVertexAttrib3sv( index: GLuint; const v: PGLshort ); external name '_glVertexAttrib3sv';
procedure glVertexAttrib4Nbv( index: GLuint; const v: PGLbyte ); external name '_glVertexAttrib4Nbv';
procedure glVertexAttrib4Niv( index: GLuint; const v: PGLint ); external name '_glVertexAttrib4Niv';
procedure glVertexAttrib4Nsv( index: GLuint; const v: PGLshort ); external name '_glVertexAttrib4Nsv';
procedure glVertexAttrib4Nub( index: GLuint; x: GLubyte; y: GLubyte; z: GLubyte; w: GLubyte ); external name '_glVertexAttrib4Nub';
procedure glVertexAttrib4Nubv( index: GLuint; const v: PGLubyte ); external name '_glVertexAttrib4Nubv';
procedure glVertexAttrib4Nuiv( index: GLuint; const v: PGLuint ); external name '_glVertexAttrib4Nuiv';
procedure glVertexAttrib4Nusv( index: GLuint; const v: PGLushort ); external name '_glVertexAttrib4Nusv';
procedure glVertexAttrib4bv( index: GLuint; const v: PGLbyte ); external name '_glVertexAttrib4bv';
procedure glVertexAttrib4d( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble ); external name '_glVertexAttrib4d';
procedure glVertexAttrib4dv( index: GLuint; const v: PGLdouble ); external name '_glVertexAttrib4dv';
procedure glVertexAttrib4f( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat ); external name '_glVertexAttrib4f';
procedure glVertexAttrib4fv( index: GLuint; const v: PGLfloat ); external name '_glVertexAttrib4fv';
procedure glVertexAttrib4iv( index: GLuint; const v: PGLint ); external name '_glVertexAttrib4iv';
procedure glVertexAttrib4s( index: GLuint; x: GLshort; y: GLshort; z: GLshort; w: GLshort ); external name '_glVertexAttrib4s';
procedure glVertexAttrib4sv( index: GLuint; const v: PGLshort ); external name '_glVertexAttrib4sv';
procedure glVertexAttrib4ubv( index: GLuint; const v: PGLubyte ); external name '_glVertexAttrib4ubv';
procedure glVertexAttrib4uiv( index: GLuint; const v: PGLuint ); external name '_glVertexAttrib4uiv';
procedure glVertexAttrib4usv( index: GLuint; const v: PGLushort ); external name '_glVertexAttrib4usv';
procedure glVertexAttribPointer( index: GLuint; size: GLint; typ: GLenum; normalized: GLboolean; stride: GLsizei; const pointr: UnivPtr ); external name '_glVertexAttribPointer';
procedure glEnableVertexAttribArray( index: GLuint ); external name '_glEnableVertexAttribArray';
procedure glDisableVertexAttribArray( index: GLuint ); external name '_glDisableVertexAttribArray';
procedure glGetVertexAttribdv( index: GLuint; pname: GLenum; params: PGLdouble ); external name '_glGetVertexAttribdv';
procedure glGetVertexAttribfv( index: GLuint; pname: GLenum; params: PGLfloat ); external name '_glGetVertexAttribfv';
procedure glGetVertexAttribiv( index: GLuint; pname: GLenum; params: PGLint ); external name '_glGetVertexAttribiv';
procedure glGetVertexAttribPointerv( index: GLuint; pname: GLenum; pointr: UnivPtr ); external name '_glGetVertexAttribPointerv';
procedure glDeleteShader( shader: GLuint ); external name '_glDeleteShader';
procedure glDetachShader( program_: GLuint; shader: GLuint ); external name '_glDetachShader';
function glCreateShader( typ: GLenum ): GLuint; external name '_glCreateShader';

procedure glShaderSource( shader: GLuint; count: GLsizei; {const} strng: PPChar; const length: PGLint ); external name '_glShaderSource';

procedure glCompileShader( shader: GLuint ); external name '_glCompileShader';
function glCreateProgram: GLuint; external name '_glCreateProgram';
procedure glAttachShader( program_: GLuint; shader: GLuint ); external name '_glAttachShader';
procedure glLinkProgram( program_: GLuint ); external name '_glLinkProgram';
procedure glUseProgram( program_: GLuint ); external name '_glUseProgram';
procedure glDeleteProgram( program_: GLuint ); external name '_glDeleteProgram';
procedure glValidateProgram( program_: GLuint ); external name '_glValidateProgram';
procedure glUniform1f( location: GLint; v0: GLfloat ); external name '_glUniform1f';
procedure glUniform2f( location: GLint; v0: GLfloat; v1: GLfloat ); external name '_glUniform2f';
procedure glUniform3f( location: GLint; v0: GLfloat; v1: GLfloat; v2: GLfloat ); external name '_glUniform3f';
procedure glUniform4f( location: GLint; v0: GLfloat; v1: GLfloat; v2: GLfloat; v3: GLfloat ); external name '_glUniform4f';
procedure glUniform1i( location: GLint; v0: GLint ); external name '_glUniform1i';
procedure glUniform2i( location: GLint; v0: GLint; v1: GLint ); external name '_glUniform2i';
procedure glUniform3i( location: GLint; v0: GLint; v1: GLint; v2: GLint ); external name '_glUniform3i';
procedure glUniform4i( location: GLint; v0: GLint; v1: GLint; v2: GLint; v3: GLint ); external name '_glUniform4i';
procedure glUniform1fv( location: GLint; count: GLsizei; const value: PGLfloat ); external name '_glUniform1fv';
procedure glUniform2fv( location: GLint; count: GLsizei; const value: PGLfloat ); external name '_glUniform2fv';
procedure glUniform3fv( location: GLint; count: GLsizei; const value: PGLfloat ); external name '_glUniform3fv';
procedure glUniform4fv( location: GLint; count: GLsizei; const value: PGLfloat ); external name '_glUniform4fv';
procedure glUniform1iv( location: GLint; count: GLsizei; const value: PGLint ); external name '_glUniform1iv';
procedure glUniform2iv( location: GLint; count: GLsizei; const value: PGLint ); external name '_glUniform2iv';
procedure glUniform3iv( location: GLint; count: GLsizei; const value: PGLint ); external name '_glUniform3iv';
procedure glUniform4iv( location: GLint; count: GLsizei; const value: PGLint ); external name '_glUniform4iv';
procedure glUniformMatrix2fv( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix2fv';
procedure glUniformMatrix3fv( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix3fv';
procedure glUniformMatrix4fv( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix4fv';
function glIsShader( shader: GLuint ): GLboolean; external name '_glIsShader';
function glIsProgram( program_: GLuint ): GLboolean; external name '_glIsProgram';
procedure glGetShaderiv( shader: GLuint; pname: GLenum; params: PGLint ); external name '_glGetShaderiv';
procedure glGetProgramiv( program_: GLuint; pname: GLenum; params: PGLint ); external name '_glGetProgramiv';
procedure glGetAttachedShaders( program_: GLuint; maxCount: GLsizei; count: PGLsizei; shaders: PGLuint ); external name '_glGetAttachedShaders';
procedure glGetShaderInfoLog( shader: GLuint; bufSize: GLsizei; length: PGLsizei; infoLog: PChar ); external name '_glGetShaderInfoLog';
procedure glGetProgramInfoLog( program_: GLuint; bufSize: GLsizei; length: PGLsizei; infoLog: PChar ); external name '_glGetProgramInfoLog';
function glGetUniformLocation( program_: GLuint; const name: PChar ): GLint; external name '_glGetUniformLocation';
procedure glGetActiveUniform( program_: GLuint; index: GLuint; bufSize: GLsizei; length: PGLsizei; size: PGLint; typ: PGLenum; name: PChar ); external name '_glGetActiveUniform';
procedure glGetUniformfv( program_: GLuint; location: GLint; params: PGLfloat ); external name '_glGetUniformfv';
procedure glGetUniformiv( program_: GLuint; location: GLint; params: PGLint ); external name '_glGetUniformiv';
procedure glGetShaderSource( shader: GLuint; bufSize: GLsizei; length: PGLsizei; source: PChar ); external name '_glGetShaderSource';
procedure glBindAttribLocation( program_: GLuint; index: GLuint; const name: PChar ); external name '_glBindAttribLocation';
procedure glGetActiveAttrib( program_: GLuint; index: GLuint; bufSize: GLsizei; length: PGLsizei; size: PGLint; typ: PGLenum; name: PChar ); external name '_glGetActiveAttrib';
function glGetAttribLocation( program_: GLuint; const name: PChar ): GLint; external name '_glGetAttribLocation';
procedure glStencilFuncSeparate( face: GLenum; func: GLenum; ref: GLint; mask: GLuint ); external name '_glStencilFuncSeparate';
procedure glStencilOpSeparate( face: GLenum; fail: GLenum; zfail: GLenum; zpass: GLenum ); external name '_glStencilOpSeparate';
procedure glStencilMaskSeparate( face: GLenum; mask: GLuint ); external name '_glStencilMaskSeparate';

procedure glUniformMatrix2x3fv( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix2x3fv';
procedure glUniformMatrix3x2fv( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix3x2fv';
procedure glUniformMatrix2x4fv( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix2x4fv';
procedure glUniformMatrix4x2fv( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix4x2fv';
procedure glUniformMatrix3x4fv( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix3x4fv';
procedure glUniformMatrix4x3fv( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix4x3fv';


{$endc} { GL_GLEXT_FUNCTION_POINTERS }

{$endc} {TARGET_OS_MAC}

//#endif { __gl_h_ }
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
