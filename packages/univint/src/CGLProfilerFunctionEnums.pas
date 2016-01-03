{
	File:		CGLProfilerFunctionEnum.h
	Copyright:	(c) 2004 by Apple Computer, Inc., all rights reserved.
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

unit CGLProfilerFunctionEnums;
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


const
{ CGL commands }
	kCGLFECGLChoosePixelFormat = 0;
	kCGLFECGLClearDrawable = 1;
	kCGLFECGLComment = 2;
	kCGLFECGLCopyContext = 3;
	kCGLFECGLCreateContext = 4;
	kCGLFECGLCreatePBuffer = 5;
	kCGLFECGLDescribePBuffer = 6;
	kCGLFECGLDescribePixelFormat = 7;
	kCGLFECGLDescribeRenderer = 8;
	kCGLFECGLDestroyContext = 9;
	kCGLFECGLDestroyPBuffer = 10;
	kCGLFECGLDestroyPixelFormat = 11;
	kCGLFECGLDestroyRendererInfo = 12;
	kCGLFECGLDisable = 13;
	kCGLFECGLEnable = 14;
	kCGLFECGLFlushDrawable = 15;
	kCGLFECGLGetCurrentContext = 16;
	kCGLFECGLGetFullScreen = 17;
	kCGLFECGLGetOffScreen = 18;
	kCGLFECGLGetOption = 19;
	kCGLFECGLGetParameter = 20;
	kCGLFECGLGetPBuffer = 21;
	kCGLFECGLGetSurface = 22;
	kCGLFECGLGetVersion = 23;
	kCGLFECGLGetVirtualScreen = 24;
	kCGLFECGLIsEnabled = 25;
	kCGLFECGLQueryRendererInfo = 26;
	kCGLFECGLSetCurrentContext = 27;
	kCGLFECGLSetFullScreen = 28;
	kCGLFECGLSetOffScreen = 29;
	kCGLFECGLSetOption = 30;
	kCGLFECGLSetParameter = 31;
	kCGLFECGLSetPBuffer = 32;
	kCGLFECGLSetSurface = 33;
	kCGLFECGLSetVirtualScreen = 34;
	kCGLFECGLTexImagePBuffer = 35;
	kCGLFECGLUpdateContext = 36;

    { OpenGL commands }
	kCGLFEglAccum = 37;
	kCGLFEglActiveStencilFaceEXT = 38;
	kCGLFEglActiveTexture = 39;
	kCGLFEglAlphaFunc = 40;
	kCGLFEglAreTexturesResident = 41;
	kCGLFEglArrayElement = 42;
	kCGLFEglAttachObjectARB = 43;
	kCGLFEglBegin = 44;
	kCGLFEglBeginQuery = 45;
	kCGLFEglBindAttribLocationARB = 46;
	kCGLFEglBindBuffer = 47;
	kCGLFEglBindFramebufferEXT = 48;
	kCGLFEglBindProgramARB = 49;
	kCGLFEglBindRenderbufferEXT = 50;
	kCGLFEglBindTexture = 51;
	kCGLFEglBindVertexArrayEXT = 52;
	kCGLFEglBitmap = 53;
	kCGLFEglBlendColor = 54;
	kCGLFEglBlendEquation = 55;
	kCGLFEglBlendEquationSeparateEXT = 56;
	kCGLFEglBlendFunc = 57;
	kCGLFEglBlendFuncSeparate = 58;
	kCGLFEglBufferData = 59;
	kCGLFEglBufferSubData = 60;
	kCGLFEglCallList = 61;
	kCGLFEglCallLists = 62;
	kCGLFEglCheckFramebufferStatusEXT = 63;
	kCGLFEglClear = 64;
	kCGLFEglClearAccum = 65;
	kCGLFEglClearColor = 66;
	kCGLFEglClearDepth = 67;
	kCGLFEglClearIndex = 68;
	kCGLFEglClearStencil = 69;
	kCGLFEglClientActiveTexture = 70;
	kCGLFEglClipPlane = 71;
	kCGLFEglColor3b = 72;
	kCGLFEglColor3bv = 73;
	kCGLFEglColor3d = 74;
	kCGLFEglColor3dv = 75;
	kCGLFEglColor3f = 76;
	kCGLFEglColor3fv = 77;
	kCGLFEglColor3i = 78;
	kCGLFEglColor3iv = 79;
	kCGLFEglColor3s = 80;
	kCGLFEglColor3sv = 81;
	kCGLFEglColor3ub = 82;
	kCGLFEglColor3ubv = 83;
	kCGLFEglColor3ui = 84;
	kCGLFEglColor3uiv = 85;
	kCGLFEglColor3us = 86;
	kCGLFEglColor3usv = 87;
	kCGLFEglColor4b = 88;
	kCGLFEglColor4bv = 89;
	kCGLFEglColor4d = 90;
	kCGLFEglColor4dv = 91;
	kCGLFEglColor4f = 92;
	kCGLFEglColor4fv = 93;
	kCGLFEglColor4i = 94;
	kCGLFEglColor4iv = 95;
	kCGLFEglColor4s = 96;
	kCGLFEglColor4sv = 97;
	kCGLFEglColor4ub = 98;
	kCGLFEglColor4ubv = 99;
	kCGLFEglColor4ui = 100;
	kCGLFEglColor4uiv = 101;
	kCGLFEglColor4us = 102;
	kCGLFEglColor4usv = 103;
	kCGLFEglColorMask = 104;
	kCGLFEglColorMaterial = 105;
	kCGLFEglColorPointer = 106;
	kCGLFEglColorSubTable = 107;
	kCGLFEglColorTable = 108;
	kCGLFEglColorTableParameterfv = 109;
	kCGLFEglColorTableParameteriv = 110;
	kCGLFEglCombinerInputNV = 111;
	kCGLFEglCombinerOutputNV = 112;
	kCGLFEglCombinerParameterfNV = 113;
	kCGLFEglCombinerParameterfvNV = 114;
	kCGLFEglCombinerParameteriNV = 115;
	kCGLFEglCombinerParameterivNV = 116;
	kCGLFEglCombinerStageParameterfvNV = 117;
	kCGLFEglCompileShaderARB = 118;
	kCGLFEglCompressedTexImage1D = 119;
	kCGLFEglCompressedTexImage2D = 120;
	kCGLFEglCompressedTexImage3D = 121;
	kCGLFEglCompressedTexSubImage1D = 122;
	kCGLFEglCompressedTexSubImage2D = 123;
	kCGLFEglCompressedTexSubImage3D = 124;
	kCGLFEglConvolutionFilter1D = 125;
	kCGLFEglConvolutionFilter2D = 126;
	kCGLFEglConvolutionParameterf = 127;
	kCGLFEglConvolutionParameterfv = 128;
	kCGLFEglConvolutionParameteri = 129;
	kCGLFEglConvolutionParameteriv = 130;
	kCGLFEglCopyColorSubTable = 131;
	kCGLFEglCopyColorTable = 132;
	kCGLFEglCopyConvolutionFilter1D = 133;
	kCGLFEglCopyConvolutionFilter2D = 134;
	kCGLFEglCopyPixels = 135;
	kCGLFEglCopyTexImage1D = 136;
	kCGLFEglCopyTexImage2D = 137;
	kCGLFEglCopyTexSubImage1D = 138;
	kCGLFEglCopyTexSubImage2D = 139;
	kCGLFEglCopyTexSubImage3D = 140;
	kCGLFEglCreateProgramObjectARB = 141;
	kCGLFEglCreateShaderObjectARB = 142;
	kCGLFEglCullFace = 143;
	kCGLFEglDeleteBuffers = 144;
	kCGLFEglDeleteFencesAPPLE = 145;
	kCGLFEglDeleteFramebuffersEXT = 146;
	kCGLFEglDeleteLists = 147;
	kCGLFEglDeleteObjectARB = 148;
	kCGLFEglDeleteProgramsARB = 149;
	kCGLFEglDeleteQueries = 150;
	kCGLFEglDeleteRenderbuffersEXT = 151;
	kCGLFEglDeleteTextures = 152;
	kCGLFEglDeleteVertexArraysEXT = 153;
	kCGLFEglDepthBoundsEXT = 154;
	kCGLFEglDepthFunc = 155;
	kCGLFEglDepthMask = 156;
	kCGLFEglDepthRange = 157;
	kCGLFEglDetachObjectARB = 158;
	kCGLFEglDisable = 159;
	kCGLFEglDisableClientState = 160;
	kCGLFEglDisableVertexAttribARB = 161;
	kCGLFEglDisableVertexAttribArrayARB = 162;
	kCGLFEglDrawArrays = 163;
	kCGLFEglDrawBuffer = 164;
	kCGLFEglDrawBuffersARB = 165;
	kCGLFEglDrawElementArrayAPPLE = 166;
	kCGLFEglDrawElements = 167;
	kCGLFEglDrawPixels = 168;
	kCGLFEglDrawRangeElementArrayAPPLE = 169;
	kCGLFEglDrawRangeElements = 170;
	kCGLFEglEdgeFlag = 171;
	kCGLFEglEdgeFlagPointer = 172;
	kCGLFEglEdgeFlagv = 173;
	kCGLFEglElementPointerAPPLE = 174;
	kCGLFEglEnable = 175;
	kCGLFEglEnableClientState = 176;
	kCGLFEglEnableVertexAttribARB = 177;
	kCGLFEglEnableVertexAttribArrayARB = 178;
	kCGLFEglEnd = 179;
	kCGLFEglEndList = 180;
	kCGLFEglEndQuery = 181;
	kCGLFEglEvalCoord1d = 182;
	kCGLFEglEvalCoord1dv = 183;
	kCGLFEglEvalCoord1f = 184;
	kCGLFEglEvalCoord1fv = 185;
	kCGLFEglEvalCoord2d = 186;
	kCGLFEglEvalCoord2dv = 187;
	kCGLFEglEvalCoord2f = 188;
	kCGLFEglEvalCoord2fv = 189;
	kCGLFEglEvalMesh1 = 190;
	kCGLFEglEvalMesh2 = 191;
	kCGLFEglEvalPoint1 = 192;
	kCGLFEglEvalPoint2 = 193;
	kCGLFEglFeedbackBuffer = 194;
	kCGLFEglFinalCombinerInputNV = 195;
	kCGLFEglFinish = 196;
	kCGLFEglFinishFenceAPPLE = 197;
	kCGLFEglFinishObjectAPPLE = 198;
	kCGLFEglFinishRenderAPPLE = 199;
	kCGLFEglFlush = 200;
	kCGLFEglFlushRenderAPPLE = 201;
	kCGLFEglFlushVertexArrayRangeEXT = 202;
	kCGLFEglFogCoordPointer = 203;
	kCGLFEglFogCoordd = 204;
	kCGLFEglFogCoorddv = 205;
	kCGLFEglFogCoordf = 206;
	kCGLFEglFogCoordfv = 207;
	kCGLFEglFogf = 208;
	kCGLFEglFogfv = 209;
	kCGLFEglFogi = 210;
	kCGLFEglFogiv = 211;
	kCGLFEglFramebufferRenderbufferEXT = 212;
	kCGLFEglFramebufferTexture1DEXT = 213;
	kCGLFEglFramebufferTexture2DEXT = 214;
	kCGLFEglFramebufferTexture3DEXT = 215;
	kCGLFEglFrontFace = 216;
	kCGLFEglFrustum = 217;
	kCGLFEglGenBuffers = 218;
	kCGLFEglGenerateMipmapEXT = 219;
	kCGLFEglGenFencesAPPLE = 220;
	kCGLFEglGenFramebuffersEXT = 221;
	kCGLFEglGenLists = 222;
	kCGLFEglGenProgramsARB = 223;
	kCGLFEglGenQueries = 224;
	kCGLFEglGenRenderbuffersEXT = 225;
	kCGLFEglGenTextures = 226;
	kCGLFEglGenVertexArraysEXT = 227;
	kCGLFEglGetActiveAttribARB = 228;
	kCGLFEglGetActiveUniformARB = 229;
	kCGLFEglGetAttachedObjectsARB = 230;
	kCGLFEglGetAttribLocationARB = 231;
	kCGLFEglGetBooleanv = 232;
	kCGLFEglGetBufferParameteriv = 233;
	kCGLFEglGetBufferPointerv = 234;
	kCGLFEglGetBufferSubData = 235;
	kCGLFEglGetClipPlane = 236;
	kCGLFEglGetColorTable = 237;
	kCGLFEglGetColorTableParameterfv = 238;
	kCGLFEglGetColorTableParameteriv = 239;
	kCGLFEglGetCombinerInputParameterfvNV = 240;
	kCGLFEglGetCombinerInputParameterivNV = 241;
	kCGLFEglGetCombinerOutputParameterfvNV = 242;
	kCGLFEglGetCombinerOutputParameterivNV = 243;
	kCGLFEglGetCombinerStageParameterfvNV = 244;
	kCGLFEglGetCompressedTexImage = 245;
	kCGLFEglGetConvolutionFilter = 246;
	kCGLFEglGetConvolutionParameterfv = 247;
	kCGLFEglGetConvolutionParameteriv = 248;
	kCGLFEglGetDoublev = 249;
	kCGLFEglGetError = 250;
	kCGLFEglGetFinalCombinerInputParameterfvNV = 251;
	kCGLFEglGetFinalCombinerInputParameterivNV = 252;
	kCGLFEglGetFloatv = 253;
	kCGLFEglGetFramebufferAttachmentParameterivEXT = 254;
	kCGLFEglGetHandleARB = 255;
	kCGLFEglGetHistogram = 256;
	kCGLFEglGetHistogramParameterfv = 257;
	kCGLFEglGetHistogramParameteriv = 258;
	kCGLFEglGetInfoLogARB = 259;
	kCGLFEglGetIntegerv = 260;
	kCGLFEglGetLightfv = 261;
	kCGLFEglGetLightiv = 262;
	kCGLFEglGetMapdv = 263;
	kCGLFEglGetMapfv = 264;
	kCGLFEglGetMapiv = 265;
	kCGLFEglGetMaterialfv = 266;
	kCGLFEglGetMaterialiv = 267;
	kCGLFEglGetMinmax = 268;
	kCGLFEglGetMinmaxParameterfv = 269;
	kCGLFEglGetMinmaxParameteriv = 270;
	kCGLFEglGetObjectParameterfvARB = 271;
	kCGLFEglGetObjectParameterivARB = 272;
	kCGLFEglGetPixelMapfv = 273;
	kCGLFEglGetPixelMapuiv = 274;
	kCGLFEglGetPixelMapusv = 275;
	kCGLFEglGetPointerv = 276;
	kCGLFEglGetPolygonStipple = 277;
	kCGLFEglGetProgramEnvParameterdvARB = 278;
	kCGLFEglGetProgramEnvParameterfvARB = 279;
	kCGLFEglGetProgramInfoLog = 280;
	kCGLFEglGetProgramLocalParameterdvARB = 281;
	kCGLFEglGetProgramLocalParameterfvARB = 282;
	kCGLFEglGetProgramStringARB = 283;
	kCGLFEglGetProgramiv = 284;
	kCGLFEglGetProgramivARB = 285;
	kCGLFEglGetQueryObjectiv = 286;
	kCGLFEglGetQueryObjectuiv = 287;
	kCGLFEglGetQueryiv = 288;
	kCGLFEglGetRenderbufferParameterivEXT = 289;
	kCGLFEglGetSeparableFilter = 290;
	kCGLFEglGetShaderInfoLog = 291;
	kCGLFEglGetShaderSourceARB = 292;
	kCGLFEglGetShaderiv = 293;
	kCGLFEglGetString = 294;
	kCGLFEglGetTexEnvfv = 295;
	kCGLFEglGetTexEnviv = 296;
	kCGLFEglGetTexGendv = 297;
	kCGLFEglGetTexGenfv = 298;
	kCGLFEglGetTexGeniv = 299;
	kCGLFEglGetTexImage = 300;
	kCGLFEglGetTexLevelParameterfv = 301;
	kCGLFEglGetTexLevelParameteriv = 302;
	kCGLFEglGetTexParameterPointervAPPLE = 303;
	kCGLFEglGetTexParameterfv = 304;
	kCGLFEglGetTexParameteriv = 305;
	kCGLFEglGetUniformLocationARB = 306;
	kCGLFEglGetUniformfvARB = 307;
	kCGLFEglGetUniformivARB = 308;
	kCGLFEglGetVertexAttribPointervARB = 309;
	kCGLFEglGetVertexAttribdvARB = 310;
	kCGLFEglGetVertexAttribfvARB = 311;
	kCGLFEglGetVertexAttribivARB = 312;
	kCGLFEglHint = 313;
	kCGLFEglHistogram = 314;
	kCGLFEglIndexMask = 315;
	kCGLFEglIndexPointer = 316;
	kCGLFEglIndexd = 317;
	kCGLFEglIndexdv = 318;
	kCGLFEglIndexf = 319;
	kCGLFEglIndexfv = 320;
	kCGLFEglIndexi = 321;
	kCGLFEglIndexiv = 322;
	kCGLFEglIndexs = 323;
	kCGLFEglIndexsv = 324;
	kCGLFEglIndexub = 325;
	kCGLFEglIndexubv = 326;
	kCGLFEglInitNames = 327;
	kCGLFEglInterleavedArrays = 328;
	kCGLFEglIsBuffer = 329;
	kCGLFEglIsEnabled = 330;
	kCGLFEglIsFenceAPPLE = 331;
	kCGLFEglIsFramebufferEXT = 332;
	kCGLFEglIsList = 333;
	kCGLFEglIsProgram = 334;
	kCGLFEglIsProgramARB = 335;
	kCGLFEglIsQuery = 336;
	kCGLFEglIsRenderbufferEXT = 337;
	kCGLFEglIsShader = 338;
	kCGLFEglIsTexture = 339;
	kCGLFEglIsVertexArrayEXT = 340;
	kCGLFEglIsVertexAttribEnabledARB = 341;
	kCGLFEglLightModelf = 342;
	kCGLFEglLightModelfv = 343;
	kCGLFEglLightModeli = 344;
	kCGLFEglLightModeliv = 345;
	kCGLFEglLightf = 346;
	kCGLFEglLightfv = 347;
	kCGLFEglLighti = 348;
	kCGLFEglLightiv = 349;
	kCGLFEglLineStipple = 350;
	kCGLFEglLineWidth = 351;
	kCGLFEglLinkProgramARB = 352;
	kCGLFEglListBase = 353;
	kCGLFEglLoadIdentity = 354;
	kCGLFEglLoadMatrixd = 355;
	kCGLFEglLoadMatrixf = 356;
	kCGLFEglLoadName = 357;
	kCGLFEglLoadTransposeMatrixd = 358;
	kCGLFEglLoadTransposeMatrixf = 359;
	kCGLFEglLockArraysEXT = 360;
	kCGLFEglLogicOp = 361;
	kCGLFEglMap1d = 362;
	kCGLFEglMap1f = 363;
	kCGLFEglMap2d = 364;
	kCGLFEglMap2f = 365;
	kCGLFEglMapBuffer = 366;
	kCGLFEglMapGrid1d = 367;
	kCGLFEglMapGrid1f = 368;
	kCGLFEglMapGrid2d = 369;
	kCGLFEglMapGrid2f = 370;
	kCGLFEglMapVertexAttrib1dARB = 371;
	kCGLFEglMapVertexAttrib1fARB = 372;
	kCGLFEglMapVertexAttrib2dARB = 373;
	kCGLFEglMapVertexAttrib2fARB = 374;
	kCGLFEglMaterialf = 375;
	kCGLFEglMaterialfv = 376;
	kCGLFEglMateriali = 377;
	kCGLFEglMaterialiv = 378;
	kCGLFEglMatrixMode = 379;
	kCGLFEglMinmax = 380;
	kCGLFEglMultMatrixd = 381;
	kCGLFEglMultMatrixf = 382;
	kCGLFEglMultTransposeMatrixd = 383;
	kCGLFEglMultTransposeMatrixf = 384;
	kCGLFEglMultiDrawArrays = 385;
	kCGLFEglMultiDrawElementArrayAPPLE = 386;
	kCGLFEglMultiDrawElements = 387;
	kCGLFEglMultiDrawRangeElementArrayAPPLE = 388;
	kCGLFEglMultiTexCoord1d = 389;
	kCGLFEglMultiTexCoord1dv = 390;
	kCGLFEglMultiTexCoord1f = 391;
	kCGLFEglMultiTexCoord1fv = 392;
	kCGLFEglMultiTexCoord1i = 393;
	kCGLFEglMultiTexCoord1iv = 394;
	kCGLFEglMultiTexCoord1s = 395;
	kCGLFEglMultiTexCoord1sv = 396;
	kCGLFEglMultiTexCoord2d = 397;
	kCGLFEglMultiTexCoord2dv = 398;
	kCGLFEglMultiTexCoord2f = 399;
	kCGLFEglMultiTexCoord2fv = 400;
	kCGLFEglMultiTexCoord2i = 401;
	kCGLFEglMultiTexCoord2iv = 402;
	kCGLFEglMultiTexCoord2s = 403;
	kCGLFEglMultiTexCoord2sv = 404;
	kCGLFEglMultiTexCoord3d = 405;
	kCGLFEglMultiTexCoord3dv = 406;
	kCGLFEglMultiTexCoord3f = 407;
	kCGLFEglMultiTexCoord3fv = 408;
	kCGLFEglMultiTexCoord3i = 409;
	kCGLFEglMultiTexCoord3iv = 410;
	kCGLFEglMultiTexCoord3s = 411;
	kCGLFEglMultiTexCoord3sv = 412;
	kCGLFEglMultiTexCoord4d = 413;
	kCGLFEglMultiTexCoord4dv = 414;
	kCGLFEglMultiTexCoord4f = 415;
	kCGLFEglMultiTexCoord4fv = 416;
	kCGLFEglMultiTexCoord4i = 417;
	kCGLFEglMultiTexCoord4iv = 418;
	kCGLFEglMultiTexCoord4s = 419;
	kCGLFEglMultiTexCoord4sv = 420;
	kCGLFEglNewList = 421;
	kCGLFEglNormal3b = 422;
	kCGLFEglNormal3bv = 423;
	kCGLFEglNormal3d = 424;
	kCGLFEglNormal3dv = 425;
	kCGLFEglNormal3f = 426;
	kCGLFEglNormal3fv = 427;
	kCGLFEglNormal3i = 428;
	kCGLFEglNormal3iv = 429;
	kCGLFEglNormal3s = 430;
	kCGLFEglNormal3sv = 431;
	kCGLFEglNormalPointer = 432;
	kCGLFEglOrtho = 433;
	kCGLFEglPassThrough = 434;
	kCGLFEglPixelMapfv = 435;
	kCGLFEglPixelMapuiv = 436;
	kCGLFEglPixelMapusv = 437;
	kCGLFEglPixelStoref = 438;
	kCGLFEglPixelStorei = 439;
	kCGLFEglPixelTransferf = 440;
	kCGLFEglPixelTransferi = 441;
	kCGLFEglPixelZoom = 442;
	kCGLFEglPnTrianglesfATI = 443;
	kCGLFEglPnTrianglesiATI = 444;
	kCGLFEglPointParameterf = 445;
	kCGLFEglPointParameterfv = 446;
	kCGLFEglPointParameteri = 447;
	kCGLFEglPointParameteriv = 448;
	kCGLFEglPointSize = 449;
	kCGLFEglPolygonMode = 450;
	kCGLFEglPolygonOffset = 451;
	kCGLFEglPolygonStipple = 452;
	kCGLFEglPopAttrib = 453;
	kCGLFEglPopClientAttrib = 454;
	kCGLFEglPopMatrix = 455;
	kCGLFEglPopName = 456;
	kCGLFEglPrioritizeTextures = 457;
	kCGLFEglProgramEnvParameter4dARB = 458;
	kCGLFEglProgramEnvParameter4dvARB = 459;
	kCGLFEglProgramEnvParameter4fARB = 460;
	kCGLFEglProgramEnvParameter4fvARB = 461;
	kCGLFEglProgramLocalParameter4dARB = 462;
	kCGLFEglProgramLocalParameter4dvARB = 463;
	kCGLFEglProgramLocalParameter4fARB = 464;
	kCGLFEglProgramLocalParameter4fvARB = 465;
	kCGLFEglProgramStringARB = 466;
	kCGLFEglPushAttrib = 467;
	kCGLFEglPushClientAttrib = 468;
	kCGLFEglPushMatrix = 469;
	kCGLFEglPushName = 470;
	kCGLFEglRasterPos2d = 471;
	kCGLFEglRasterPos2dv = 472;
	kCGLFEglRasterPos2f = 473;
	kCGLFEglRasterPos2fv = 474;
	kCGLFEglRasterPos2i = 475;
	kCGLFEglRasterPos2iv = 476;
	kCGLFEglRasterPos2s = 477;
	kCGLFEglRasterPos2sv = 478;
	kCGLFEglRasterPos3d = 479;
	kCGLFEglRasterPos3dv = 480;
	kCGLFEglRasterPos3f = 481;
	kCGLFEglRasterPos3fv = 482;
	kCGLFEglRasterPos3i = 483;
	kCGLFEglRasterPos3iv = 484;
	kCGLFEglRasterPos3s = 485;
	kCGLFEglRasterPos3sv = 486;
	kCGLFEglRasterPos4d = 487;
	kCGLFEglRasterPos4dv = 488;
	kCGLFEglRasterPos4f = 489;
	kCGLFEglRasterPos4fv = 490;
	kCGLFEglRasterPos4i = 491;
	kCGLFEglRasterPos4iv = 492;
	kCGLFEglRasterPos4s = 493;
	kCGLFEglRasterPos4sv = 494;
	kCGLFEglReadBuffer = 495;
	kCGLFEglReadPixels = 496;
	kCGLFEglRectd = 497;
	kCGLFEglRectdv = 498;
	kCGLFEglRectf = 499;
	kCGLFEglRectfv = 500;
	kCGLFEglRecti = 501;
	kCGLFEglRectiv = 502;
	kCGLFEglRects = 503;
	kCGLFEglRectsv = 504;
	kCGLFEglRenderbufferStorageEXT = 505;
	kCGLFEglRenderMode = 506;
	kCGLFEglResetHistogram = 507;
	kCGLFEglResetMinmax = 508;
	kCGLFEglRotated = 509;
	kCGLFEglRotatef = 510;
	kCGLFEglSampleCoverage = 511;
	kCGLFEglSamplePass = 512;
	kCGLFEglScaled = 513;
	kCGLFEglScalef = 514;
	kCGLFEglScissor = 515;
	kCGLFEglSecondaryColor3b = 516;
	kCGLFEglSecondaryColor3bv = 517;
	kCGLFEglSecondaryColor3d = 518;
	kCGLFEglSecondaryColor3dv = 519;
	kCGLFEglSecondaryColor3f = 520;
	kCGLFEglSecondaryColor3fv = 521;
	kCGLFEglSecondaryColor3i = 522;
	kCGLFEglSecondaryColor3iv = 523;
	kCGLFEglSecondaryColor3s = 524;
	kCGLFEglSecondaryColor3sv = 525;
	kCGLFEglSecondaryColor3ub = 526;
	kCGLFEglSecondaryColor3ubv = 527;
	kCGLFEglSecondaryColor3ui = 528;
	kCGLFEglSecondaryColor3uiv = 529;
	kCGLFEglSecondaryColor3us = 530;
	kCGLFEglSecondaryColor3usv = 531;
	kCGLFEglSecondaryColorPointer = 532;
	kCGLFEglSelectBuffer = 533;
	kCGLFEglSeparableFilter2D = 534;
	kCGLFEglSetFenceAPPLE = 535;
	kCGLFEglShadeModel = 536;
	kCGLFEglShaderSourceARB = 537;
	kCGLFEglStencilFunc = 538;
	kCGLFEglStencilFuncSeparate = 539;
	kCGLFEglStencilFuncSeparateATI = 540;
	kCGLFEglStencilMask = 541;
	kCGLFEglStencilMaskSeparate = 542;
	kCGLFEglStencilOp = 543;
	kCGLFEglStencilOpSeparateATI = 544;
	kCGLFEglTestFenceAPPLE = 545;
	kCGLFEglTestObjectAPPLE = 546;
	kCGLFEglTexCoord1d = 547;
	kCGLFEglTexCoord1dv = 548;
	kCGLFEglTexCoord1f = 549;
	kCGLFEglTexCoord1fv = 550;
	kCGLFEglTexCoord1i = 551;
	kCGLFEglTexCoord1iv = 552;
	kCGLFEglTexCoord1s = 553;
	kCGLFEglTexCoord1sv = 554;
	kCGLFEglTexCoord2d = 555;
	kCGLFEglTexCoord2dv = 556;
	kCGLFEglTexCoord2f = 557;
	kCGLFEglTexCoord2fv = 558;
	kCGLFEglTexCoord2i = 559;
	kCGLFEglTexCoord2iv = 560;
	kCGLFEglTexCoord2s = 561;
	kCGLFEglTexCoord2sv = 562;
	kCGLFEglTexCoord3d = 563;
	kCGLFEglTexCoord3dv = 564;
	kCGLFEglTexCoord3f = 565;
	kCGLFEglTexCoord3fv = 566;
	kCGLFEglTexCoord3i = 567;
	kCGLFEglTexCoord3iv = 568;
	kCGLFEglTexCoord3s = 569;
	kCGLFEglTexCoord3sv = 570;
	kCGLFEglTexCoord4d = 571;
	kCGLFEglTexCoord4dv = 572;
	kCGLFEglTexCoord4f = 573;
	kCGLFEglTexCoord4fv = 574;
	kCGLFEglTexCoord4i = 575;
	kCGLFEglTexCoord4iv = 576;
	kCGLFEglTexCoord4s = 577;
	kCGLFEglTexCoord4sv = 578;
	kCGLFEglTexCoordPointer = 579;
	kCGLFEglTexEnvf = 580;
	kCGLFEglTexEnvfv = 581;
	kCGLFEglTexEnvi = 582;
	kCGLFEglTexEnviv = 583;
	kCGLFEglTexGend = 584;
	kCGLFEglTexGendv = 585;
	kCGLFEglTexGenf = 586;
	kCGLFEglTexGenfv = 587;
	kCGLFEglTexGeni = 588;
	kCGLFEglTexGeniv = 589;
	kCGLFEglTexImage1D = 590;
	kCGLFEglTexImage2D = 591;
	kCGLFEglTexImage3D = 592;
	kCGLFEglTexParameterf = 593;
	kCGLFEglTexParameterfv = 594;
	kCGLFEglTexParameteri = 595;
	kCGLFEglTexParameteriv = 596;
	kCGLFEglTexSubImage1D = 597;
	kCGLFEglTexSubImage2D = 598;
	kCGLFEglTexSubImage3D = 599;
	kCGLFEglTextureRangeAPPLE = 600;
	kCGLFEglTranslated = 601;
	kCGLFEglTranslatef = 602;
	kCGLFEglUniform1fARB = 603;
	kCGLFEglUniform1fvARB = 604;
	kCGLFEglUniform1iARB = 605;
	kCGLFEglUniform1ivARB = 606;
	kCGLFEglUniform2fARB = 607;
	kCGLFEglUniform2fvARB = 608;
	kCGLFEglUniform2iARB = 609;
	kCGLFEglUniform2ivARB = 610;
	kCGLFEglUniform3fARB = 611;
	kCGLFEglUniform3fvARB = 612;
	kCGLFEglUniform3iARB = 613;
	kCGLFEglUniform3ivARB = 614;
	kCGLFEglUniform4fARB = 615;
	kCGLFEglUniform4fvARB = 616;
	kCGLFEglUniform4iARB = 617;
	kCGLFEglUniform4ivARB = 618;
	kCGLFEglUniformMatrix2fvARB = 619;
	kCGLFEglUniformMatrix3fvARB = 620;
	kCGLFEglUniformMatrix4fvARB = 621;
	kCGLFEglUnlockArraysEXT = 622;
	kCGLFEglUnmapBuffer = 623;
	kCGLFEglUseProgramObjectARB = 624;
	kCGLFEglValidateProgramARB = 625;
	kCGLFEglVertex2d = 626;
	kCGLFEglVertex2dv = 627;
	kCGLFEglVertex2f = 628;
	kCGLFEglVertex2fv = 629;
	kCGLFEglVertex2i = 630;
	kCGLFEglVertex2iv = 631;
	kCGLFEglVertex2s = 632;
	kCGLFEglVertex2sv = 633;
	kCGLFEglVertex3d = 634;
	kCGLFEglVertex3dv = 635;
	kCGLFEglVertex3f = 636;
	kCGLFEglVertex3fv = 637;
	kCGLFEglVertex3i = 638;
	kCGLFEglVertex3iv = 639;
	kCGLFEglVertex3s = 640;
	kCGLFEglVertex3sv = 641;
	kCGLFEglVertex4d = 642;
	kCGLFEglVertex4dv = 643;
	kCGLFEglVertex4f = 644;
	kCGLFEglVertex4fv = 645;
	kCGLFEglVertex4i = 646;
	kCGLFEglVertex4iv = 647;
	kCGLFEglVertex4s = 648;
	kCGLFEglVertex4sv = 649;
	kCGLFEglVertexArrayParameteriEXT = 650;
	kCGLFEglVertexArrayRangeEXT = 651;
	kCGLFEglVertexAttrib1dARB = 652;
	kCGLFEglVertexAttrib1dvARB = 653;
	kCGLFEglVertexAttrib1fARB = 654;
	kCGLFEglVertexAttrib1fvARB = 655;
	kCGLFEglVertexAttrib1sARB = 656;
	kCGLFEglVertexAttrib1svARB = 657;
	kCGLFEglVertexAttrib2dARB = 658;
	kCGLFEglVertexAttrib2dvARB = 659;
	kCGLFEglVertexAttrib2fARB = 660;
	kCGLFEglVertexAttrib2fvARB = 661;
	kCGLFEglVertexAttrib2sARB = 662;
	kCGLFEglVertexAttrib2svARB = 663;
	kCGLFEglVertexAttrib3dARB = 664;
	kCGLFEglVertexAttrib3dvARB = 665;
	kCGLFEglVertexAttrib3fARB = 666;
	kCGLFEglVertexAttrib3fvARB = 667;
	kCGLFEglVertexAttrib3sARB = 668;
	kCGLFEglVertexAttrib3svARB = 669;
	kCGLFEglVertexAttrib4bvARB = 670;
	kCGLFEglVertexAttrib4dARB = 671;
	kCGLFEglVertexAttrib4dvARB = 672;
	kCGLFEglVertexAttrib4fARB = 673;
	kCGLFEglVertexAttrib4fvARB = 674;
	kCGLFEglVertexAttrib4ivARB = 675;
	kCGLFEglVertexAttrib4nbvARB = 676;
	kCGLFEglVertexAttrib4nivARB = 677;
	kCGLFEglVertexAttrib4nsvARB = 678;
	kCGLFEglVertexAttrib4nubARB = 679;
	kCGLFEglVertexAttrib4nubvARB = 680;
	kCGLFEglVertexAttrib4nuivARB = 681;
	kCGLFEglVertexAttrib4nusvARB = 682;
	kCGLFEglVertexAttrib4sARB = 683;
	kCGLFEglVertexAttrib4svARB = 684;
	kCGLFEglVertexAttrib4ubvARB = 685;
	kCGLFEglVertexAttrib4uivARB = 686;
	kCGLFEglVertexAttrib4usvARB = 687;
	kCGLFEglVertexAttribPointerARB = 688;
	kCGLFEglVertexBlendARB = 689;
	kCGLFEglVertexPointer = 690;
	kCGLFEglViewport = 691;
	kCGLFEglWeightPointerARB = 692;
	kCGLFEglWeightbvARB = 693;
	kCGLFEglWeightdvARB = 694;
	kCGLFEglWeightfvARB = 695;
	kCGLFEglWeightivARB = 696;
	kCGLFEglWeightsvARB = 697;
	kCGLFEglWeightubvARB = 698;
	kCGLFEglWeightuivARB = 699;
	kCGLFEglWeightusvARB = 700;
	kCGLFEglWindowPos2d = 701;
	kCGLFEglWindowPos2dv = 702;
	kCGLFEglWindowPos2f = 703;
	kCGLFEglWindowPos2fv = 704;
	kCGLFEglWindowPos2i = 705;
	kCGLFEglWindowPos2iv = 706;
	kCGLFEglWindowPos2s = 707;
	kCGLFEglWindowPos2sv = 708;
	kCGLFEglWindowPos3d = 709;
	kCGLFEglWindowPos3dv = 710;
	kCGLFEglWindowPos3f = 711;
	kCGLFEglWindowPos3fv = 712;
	kCGLFEglWindowPos3i = 713;
	kCGLFEglWindowPos3iv = 714;
	kCGLFEglWindowPos3s = 715;
	kCGLFEglWindowPos3sv = 716;

	{ APPLE_flush_buffer_range }
	kCGLFEglBufferParameteriAPPLE = 717;
	kCGLFEglFlushMappedBufferRangeAPPLE = 718;
	
	{ GL_EXT_gpu_program_parameters }
	kCGLFEglProgramEnvParameters4fvEXT = 719;
	kCGLFEglProgramLocalParameters4fvEXT = 720;
	
	{ GL_APPLE_object_purgeable }
	kCGLFEglObjectPurgeableAPPLE = 721;
	kCGLFEglObjectUnpurgeableAPPLE = 722;
	kCGLFEglGetObjectParameterivAPPLE = 723;

	{ GL_EXT_geometry_shader4 }
	kCGLFEglProgramParameteriEXT = 724;
	kCGLFEglFramebufferTextureEXT = 725;
	kCGLFEglFramebufferTextureLayerEXT = 726;
	kCGLFEglFramebufferTextureFaceEXT = 727;

	{ GL_EXT_transform_feedback }
	kCGLFEglBindBufferRangeEXT = 728;
	kCGLFEglBindBufferOffsetEXT = 729;
	kCGLFEglBindBufferBaseEXT = 730;
	kCGLFEglBeginTransformFeedbackEXT = 731;
	kCGLFEglEndTransformFeedbackEXT = 732;
	kCGLFEglTransformFeedbackVaryingsEXT = 733;
	kCGLFEglGetTransformFeedbackVaryingEXT = 734;

	{ GL_EXT_transform_feedback || GL_EXT_draw_buffers2 }
	kCGLFEglGetIntegerIndexedvEXT = 735;
	kCGLFEglGetBooleanIndexedvEXT = 736;

	{ GL_EXT_bindable_uniform }
	kCGLFEglUniformBufferEXT = 737;
	kCGLFEglGetUniformBufferSizeEXT = 738;
	kCGLFEglGetUniformOffsetEXT = 739;

	{ GL_EXT_texture_integer }
	kCGLFEglClearColorIiEXT = 740;
	kCGLFEglClearColorIuiEXT = 741;
	kCGLFEglTexParameterIivEXT = 742;
	kCGLFEglTexParameterIuivEXT = 743;
	kCGLFEglGetTexParameterIivEXT = 744;
	kCGLFEglGetTexParameterIuivEXT = 745;

	{ GL_EXT_gpu_shader4 }
	kCGLFEglVertexAttribI1iEXT = 746;
	kCGLFEglVertexAttribI2iEXT = 747;
	kCGLFEglVertexAttribI3iEXT = 748;
	kCGLFEglVertexAttribI4iEXT = 749;
	kCGLFEglVertexAttribI1uiEXT = 750;
	kCGLFEglVertexAttribI2uiEXT = 751;
	kCGLFEglVertexAttribI3uiEXT = 752;
	kCGLFEglVertexAttribI4uiEXT = 753;
	kCGLFEglVertexAttribI1ivEXT = 754;
	kCGLFEglVertexAttribI2ivEXT = 755;
	kCGLFEglVertexAttribI3ivEXT = 756;
	kCGLFEglVertexAttribI4ivEXT = 757;
	kCGLFEglVertexAttribI1uivEXT = 758;
	kCGLFEglVertexAttribI2uivEXT = 759;
	kCGLFEglVertexAttribI3uivEXT = 760;
	kCGLFEglVertexAttribI4uivEXT = 761;
	kCGLFEglVertexAttribI4bvEXT = 762;
	kCGLFEglVertexAttribI4svEXT = 763;
	kCGLFEglVertexAttribI4ubvEXT = 764;
	kCGLFEglVertexAttribI4usvEXT = 765;
	kCGLFEglVertexAttribIPointerEXT = 766;
	kCGLFEglGetVertexAttribIivEXT = 767;
	kCGLFEglGetVertexAttribIuivEXT = 768;
	kCGLFEglUniform1uiEXT = 769;
	kCGLFEglUniform2uiEXT = 770;
	kCGLFEglUniform3uiEXT = 771;
	kCGLFEglUniform4uiEXT = 772;
	kCGLFEglUniform1uivEXT = 773;
	kCGLFEglUniform2uivEXT = 774;
	kCGLFEglUniform3uivEXT = 775;
	kCGLFEglUniform4uivEXT = 776;
	kCGLFEglGetUniformuivEXT = 777;
	kCGLFEglBindFragDataLocationEXT = 778;
	kCGLFEglGetFragDataLocationEXT = 779;

	{ EXT_draw_buffers2 }
	kCGLFEglColorMaskIndexedEXT = 780;
	kCGLFEglEnableIndexedEXT = 781;
	kCGLFEglDisableIndexedEXT = 782;
	kCGLFEglIsEnabledIndexedEXT = 783;

	{ OpenGL 2.1 }
	kCGLFEglUniformMatrix2x3fv = 784;
	kCGLFEglUniformMatrix3x2fv = 785;
	kCGLFEglUniformMatrix2x4fv = 786;
	kCGLFEglUniformMatrix4x2fv = 787;
	kCGLFEglUniformMatrix3x4fv = 788;
	kCGLFEglUniformMatrix4x3fv = 789;

	{ EXT_framebuffer_blit and EXT_framebuffer_multisample }
	kCGLFEglBlitFramebufferEXT = 790;
	kCGLFEglRenderbufferStorageMultisampleEXT = 791;

	{ NV_conditional_render }
	kCGLFEglBeginConditionalRenderNV = 792;
	kCGLFEglEndConditionalRenderNV = 793;

	{ OpenGL 2.1 }
	kCGLFEglGetAttachedShaders = 794;

	{ APPLE_uniform_buffer_object }
	kCGLFEglAttachUniformBufferAPPLE = 795;
	kCGLFEglGetActivePartitionivAPPLE = 796;
	kCGLFEglGetActivePartitionNameAPPLE = 797;
	kCGLFEglGetActiveUniformsivAPPLE = 798;
	kCGLFEglGetActiveUniformsNamesAPPLE = 799;
	kCGLFEglGetPartitionIndexAPPLE = 800;
	kCGLFEglGetUniformIndicesAPPLE = 801;
	kCGLFENumFunctions = 802;


type
	CGLProfilerFunctionEnum = SInt32;

{$endc} {TARGET_OS_MAC}
	{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
