{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright:	(c) 1999-2008 Apple Inc. All rights reserved.
}

{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2008 }
{	Pascal Translation Update: Gorazd Krosl, <gorazd_1967@yahoo.ca>, October 2009 }

{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
	Copyright:	(c) 1999 by Apple Computer, Inc., all rights reserved.
=======
	Copyright:	(c) 1999-2008 Apple Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
}

{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2008 }
{	Pascal Translation Update: Gorazd Krosl, <gorazd_1967@yahoo.ca>, October 2009 }

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

unit CGLTypes;
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
=======
>>>>>>> origin/cpstrnew
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
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> origin/cpstrnew
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
<<<<<<< HEAD
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
uses MacTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$ALIGN POWER}

{
** CGL opaque data.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	_CGLContextObject = Record end;
	CGLContextObj = ^_CGLContextObject;
	PCGLContextObj = ^CGLContextObj;
	
	_CGLPixelFormatObject = Record end;
	CGLPixelFormatObj = ^_CGLPixelFormatObject;
	PCGLPixelFormatObj = ^CGLPixelFormatObj;
	
	_CGLRendererInfoObject = Record end;
	CGLRendererInfoObj = ^_CGLRendererInfoObject;
	PCGLRendererInfoObj	= ^CGLRendererInfoObj;
	
	_CGLPBufferObject = Record end;
	CGLPBufferObj = ^_CGLPBufferObject;
	PCGLPBufferObj = ^CGLPBufferObj;
<<<<<<< HEAD

{ Test with gcc shows that sizeof all enumerations (constants in Pascal) = 4 in 32 and 64 bit }
{
** Attribute names for CGLChoosePixelFormat and CGLDescribePixelFormat.
}
{ CGLPixelFormatAttribute }

type
	CGLPixelFormatAttribute	= SInt32;
	PCGLPixelFormatAttribute = ^CGLPixelFormatAttribute;
	
const
	kCGLPFAAllRenderers       =   1;	{ choose from all available renderers          }
	kCGLPFADoubleBuffer       =   5;	{ choose a double buffered pixel format        }
	kCGLPFAStereo             =   6;	{ stereo buffering supported                   }
	kCGLPFAAuxBuffers         =   7;	{ number of aux buffers                        }
	kCGLPFAColorSize          =   8;	{ number of color buffer bits                  }
	kCGLPFAAlphaSize          =  11;	{ number of alpha component bits               }
	kCGLPFADepthSize          =  12;	{ number of depth buffer bits                  }
	kCGLPFAStencilSize        =  13;	{ number of stencil buffer bits                }
	kCGLPFAAccumSize          =  14;	{ number of accum buffer bits                  }
	kCGLPFAMinimumPolicy      =  51;	{ never choose smaller buffers than requested  }
	kCGLPFAMaximumPolicy      =  52;	{ choose largest buffers of type requested     }
	kCGLPFAOffScreen          =  53;	{ choose an off-screen capable renderer        }
	kCGLPFAFullScreen         =  54;	{ choose a full-screen capable renderer        }
	kCGLPFASampleBuffers      =  55;	{ number of multi sample buffers               }
	kCGLPFASamples            =  56;	{ number of samples per multi sample buffer    }
	kCGLPFAAuxDepthStencil    =  57;	{ each aux buffer has its own depth stencil    }
	kCGLPFAColorFloat         =  58;	{ color buffers store floating point pixels    }
	kCGLPFAMultisample        =  59;	{ choose multisampling                         }
	kCGLPFASupersample        =  60;	{ choose supersampling                         }
	kCGLPFASampleAlpha        =  61;	{ request alpha filtering                      }

	kCGLPFARendererID         =  70;	{ request renderer by ID                       }
	kCGLPFASingleRenderer     =  71;	{ choose a single renderer for all screens     }
	kCGLPFANoRecovery         =  72;	{ disable all failure recovery systems         }
	kCGLPFAAccelerated        =  73;	{ choose a hardware accelerated renderer       }
	kCGLPFAClosestPolicy      =  74;	{ choose the closest color buffer to request   }
	kCGLPFABackingStore       =  76;	{ back buffer contents are valid after swap    }
	kCGLPFAWindow             =  80;	{ can be used to render to an onscreen window  }
	kCGLPFACompliant          =  83;	{ renderer is opengl compliant                 }
	kCGLPFADisplayMask        =  84;	{ mask limiting supported displays             }
	kCGLPFAPBuffer            =  90;	{ can be used to render to a pbuffer           }
	kCGLPFARemotePBuffer      =  91;	{ can be used to render offline to a pbuffer   }
	kCGLPFAAllowOfflineRenderers = 96;	{ show offline renderers in pixel formats      }
	kCGLPFAAcceleratedCompute =  97;	{ choose a hardware accelerated compute device }
	kCGLPFAVirtualScreenCount = 128;	{ number of virtual screens in this format     }

{
	Note: 
		kCGLPFARobust, kCGLPFAMPSafe and kCGLPFAMultiScreen attributes will be deprecated in 10.5
		Applications with these attributes will continue to work but these are being
		deprecated for new applications.
}
	kCGLPFARobust             =  75;	{ renderer does not need failure recovery      }
	kCGLPFAMPSafe             =  78;	{ renderer is multi-processor safe             }
	kCGLPFAMultiScreen        =  81;	{ single window can span multiple screens      }
=======
=======
>>>>>>> origin/fixes_2_2
	CGLContextObj      = ^SInt32; { an opaque 32-bit type }
	CGLPixelFormatObj  = ^SInt32; { an opaque 32-bit type }
	CGLRendererInfoObj = ^SInt32; { an opaque 32-bit type }
	CGLPBufferObj      = ^SInt32; { an opaque 32-bit type }
=======
>>>>>>> origin/fixes_2.4

{ Test with gcc shows that sizeof all enumerations (constants in Pascal) = 4 in 32 and 64 bit }
{
** Attribute names for CGLChoosePixelFormat and CGLDescribePixelFormat.
}
{ CGLPixelFormatAttribute }

type
	CGLPixelFormatAttribute	= SInt32;
	PCGLPixelFormatAttribute = ^CGLPixelFormatAttribute;
	
const
<<<<<<< HEAD
	kCGLPFAAllRenderers       = 1;	{ choose from all available renderers          }
	kCGLPFADoubleBuffer       = 5;	{ choose a double buffered pixel format        }
	kCGLPFAStereo             = 6;	{ stereo buffering supported                   }
	kCGLPFAAuxBuffers         = 7;	{ number of aux buffers                        }
	kCGLPFAColorSize          = 8;	{ number of color buffer bits                  }
	kCGLPFAAlphaSize          = 11;	{ number of alpha component bits               }
	kCGLPFADepthSize          = 12;	{ number of depth buffer bits                  }
	kCGLPFAStencilSize        = 13;	{ number of stencil buffer bits                }
	kCGLPFAAccumSize          = 14;	{ number of accum buffer bits                  }
	kCGLPFAMinimumPolicy      = 51;	{ never choose smaller buffers than requested  }
	kCGLPFAMaximumPolicy      = 52;	{ choose largest buffers of type requested     }
	kCGLPFAOffScreen          = 53;	{ choose an off-screen capable renderer        }
	kCGLPFAFullScreen         = 54;	{ choose a full-screen capable renderer        }
	kCGLPFASampleBuffers      = 55;	{ number of multi sample buffers               }
	kCGLPFASamples            = 56;	{ number of samples per multi sample buffer    }
	kCGLPFAAuxDepthStencil    = 57;	{ each aux buffer has its own depth stencil    }
	kCGLPFAColorFloat         = 58;	{ color buffers store floating point pixels    }
	kCGLPFAMultisample        = 59;	{ choose multisampling                         }
	kCGLPFASupersample        = 60;	{ choose supersampling                         }
	kCGLPFASampleAlpha        = 61;	{ request alpha filtering                      }

	kCGLPFARendererID         = 70;	{ request renderer by ID                       }
	kCGLPFASingleRenderer     = 71;	{ choose a single renderer for all screens     }
	kCGLPFANoRecovery         = 72;	{ disable all failure recovery systems         }
	kCGLPFAAccelerated        = 73;	{ choose a hardware accelerated renderer       }
	kCGLPFAClosestPolicy      = 74;	{ choose the closest color buffer to request   }
	kCGLPFARobust             = 75;	{ renderer does not need failure recovery      }
	kCGLPFABackingStore       = 76;	{ back buffer contents are valid after swap    }
	kCGLPFAMPSafe             = 78;	{ renderer is multi-processor safe             }
	kCGLPFAWindow             = 80;	{ can be used to render to an onscreen window  }
	kCGLPFAMultiScreen        = 81;	{ single window can span multiple screens      }
	kCGLPFACompliant          = 83;	{ renderer is opengl compliant                 }
	kCGLPFADisplayMask        = 84;	{ mask limiting supported displays             }
	kCGLPFAPBuffer            = 90;	{ can be used to render to a pbuffer           }
	kCGLPFARemotePBuffer      = 91;	{ can be used to render offline to a pbuffer   }
	kCGLPFAVirtualScreenCount = 128; 	{ number of virtual screens in this format     }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kCGLPFAAllRenderers       =   1;	{ choose from all available renderers          }
	kCGLPFADoubleBuffer       =   5;	{ choose a double buffered pixel format        }
	kCGLPFAStereo             =   6;	{ stereo buffering supported                   }
	kCGLPFAAuxBuffers         =   7;	{ number of aux buffers                        }
	kCGLPFAColorSize          =   8;	{ number of color buffer bits                  }
	kCGLPFAAlphaSize          =  11;	{ number of alpha component bits               }
	kCGLPFADepthSize          =  12;	{ number of depth buffer bits                  }
	kCGLPFAStencilSize        =  13;	{ number of stencil buffer bits                }
	kCGLPFAAccumSize          =  14;	{ number of accum buffer bits                  }
	kCGLPFAMinimumPolicy      =  51;	{ never choose smaller buffers than requested  }
	kCGLPFAMaximumPolicy      =  52;	{ choose largest buffers of type requested     }
	kCGLPFAOffScreen          =  53;	{ choose an off-screen capable renderer        }
	kCGLPFAFullScreen         =  54;	{ choose a full-screen capable renderer        }
	kCGLPFASampleBuffers      =  55;	{ number of multi sample buffers               }
	kCGLPFASamples            =  56;	{ number of samples per multi sample buffer    }
	kCGLPFAAuxDepthStencil    =  57;	{ each aux buffer has its own depth stencil    }
	kCGLPFAColorFloat         =  58;	{ color buffers store floating point pixels    }
	kCGLPFAMultisample        =  59;	{ choose multisampling                         }
	kCGLPFASupersample        =  60;	{ choose supersampling                         }
	kCGLPFASampleAlpha        =  61;	{ request alpha filtering                      }

	kCGLPFARendererID         =  70;	{ request renderer by ID                       }
	kCGLPFASingleRenderer     =  71;	{ choose a single renderer for all screens     }
	kCGLPFANoRecovery         =  72;	{ disable all failure recovery systems         }
	kCGLPFAAccelerated        =  73;	{ choose a hardware accelerated renderer       }
	kCGLPFAClosestPolicy      =  74;	{ choose the closest color buffer to request   }
	kCGLPFABackingStore       =  76;	{ back buffer contents are valid after swap    }
	kCGLPFAWindow             =  80;	{ can be used to render to an onscreen window  }
	kCGLPFACompliant          =  83;	{ renderer is opengl compliant                 }
	kCGLPFADisplayMask        =  84;	{ mask limiting supported displays             }
	kCGLPFAPBuffer            =  90;	{ can be used to render to a pbuffer           }
	kCGLPFARemotePBuffer      =  91;	{ can be used to render offline to a pbuffer   }
	kCGLPFAAllowOfflineRenderers = 96;	{ show offline renderers in pixel formats      }
	kCGLPFAAcceleratedCompute =  97;	{ choose a hardware accelerated compute device }
	kCGLPFAVirtualScreenCount = 128;	{ number of virtual screens in this format     }

{
	Note: 
		kCGLPFARobust, kCGLPFAMPSafe and kCGLPFAMultiScreen attributes will be deprecated in 10.5
		Applications with these attributes will continue to work but these are being
		deprecated for new applications.
}
	kCGLPFARobust             =  75;	{ renderer does not need failure recovery      }
	kCGLPFAMPSafe             =  78;	{ renderer is multi-processor safe             }
	kCGLPFAMultiScreen        =  81;	{ single window can span multiple screens      }
>>>>>>> origin/fixes_2.4

{
** Property names for CGLDescribeRenderer.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{ CGLRendererProperty }
type
	CGLRendererProperty = SInt32;
	PCGLRendererProperty = ^CGLRendererProperty;
	
<<<<<<< HEAD
const
	kCGLRPOffScreen           =  53;
	kCGLRPFullScreen          =  54;
	kCGLRPRendererID          =  70;
	kCGLRPAccelerated         =  73;
	kCGLRPRobust              =  75;
	kCGLRPBackingStore        =  76;
	kCGLRPMPSafe              =  78;
	kCGLRPWindow              =  80;
	kCGLRPMultiScreen         =  81;
	kCGLRPCompliant           =  83;
	kCGLRPDisplayMask         =  84;
=======
=======
>>>>>>> origin/fixes_2_2
const
	kCGLRPOffScreen           = 53;
	kCGLRPFullScreen          = 54;
	kCGLRPRendererID          = 70;
	kCGLRPAccelerated         = 73;
	kCGLRPRobust              = 75;
	kCGLRPBackingStore        = 76;
	kCGLRPMPSafe              = 78;
	kCGLRPWindow              = 80;
	kCGLRPMultiScreen         = 81;
	kCGLRPCompliant           = 83;
	kCGLRPDisplayMask         = 84;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kCGLRPOffScreen           =  53;
	kCGLRPFullScreen          =  54;
	kCGLRPRendererID          =  70;
	kCGLRPAccelerated         =  73;
	kCGLRPRobust              =  75;
	kCGLRPBackingStore        =  76;
	kCGLRPMPSafe              =  78;
	kCGLRPWindow              =  80;
	kCGLRPMultiScreen         =  81;
	kCGLRPCompliant           =  83;
	kCGLRPDisplayMask         =  84;
>>>>>>> origin/fixes_2.4
	kCGLRPBufferModes         = 100;	{ a bitfield of supported buffer modes          }
	kCGLRPColorModes          = 103;	{ a bitfield of supported color buffer formats  }
	kCGLRPAccumModes          = 104;	{ a bitfield of supported accum buffer formats  }
	kCGLRPDepthModes          = 105;	{ a bitfield of supported depth buffer depths   }
	kCGLRPStencilModes        = 106;	{ a bitfield of supported stencil buffer depths }
	kCGLRPMaxAuxBuffers       = 107;	{ maximum number of auxilliary buffers          }
	kCGLRPMaxSampleBuffers    = 108;	{ maximum number of sample buffers              }
	kCGLRPMaxSamples          = 109;	{ maximum number of samples                     }
	kCGLRPSampleModes         = 110;	{ a bitfield of supported sample modes          }
	kCGLRPSampleAlpha         = 111;	{ support for alpha sampling                    }
	kCGLRPVideoMemory         = 120;	{ total video memory                            }
	kCGLRPTextureMemory       = 121;	{ video memory useable for texture storage      }
	kCGLRPGPUVertProcCapable  = 122;	{ renderer capable of GPU vertex processing     }
	kCGLRPGPUFragProcCapable  = 123;	{ renderer capable of GPU fragment processing   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCGLRPRendererCount       = 128;	{ the number of renderers in this renderer info }
	kCGLRPOnline              = 129;	{ a boolean stating if renderer is on/offline   }
	kCGLRPAcceleratedCompute  = 130;	{ hardware accelerated compute device           }
=======
	kCGLRPRendererCount       = 128; 	{ the number of renderers in this renderer info }
>>>>>>> graemeg/fixes_2_2
=======
	kCGLRPRendererCount       = 128; 	{ the number of renderers in this renderer info }
>>>>>>> origin/fixes_2_2
=======
	kCGLRPRendererCount       = 128;	{ the number of renderers in this renderer info }
	kCGLRPOnline              = 129;	{ a boolean stating if renderer is on/offline   }
	kCGLRPAcceleratedCompute  = 130;	{ hardware accelerated compute device           }
>>>>>>> origin/fixes_2.4

{
** Enable names for CGLEnable, CGLDisable, and CGLIsEnabled.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
type
	CGLContextEnable = SInt32;
	PCGLContextEnable = ^CGLContextEnable;
	
{ CGLContextEnable }
<<<<<<< HEAD
const
	kCGLCESwapRectangle    = 201;	{ Enable or disable the swap rectangle          }
	kCGLCESwapLimit        = 203;	{ Enable or disable the swap async limit        }
	kCGLCERasterization    = 221;	{ Enable or disable all rasterization           }
	kCGLCEStateValidation  = 301;	{ Validate state for multi-screen functionality }
	kCGLCESurfaceBackingSize = 305;  { Enable or disable surface backing size override }
	kCGLCEDisplayListOptimization = 307;  { Ability to turn off display list optimizer }
	kCGLCEMPEngine         = 313; 	{ Enable or disable multi-threaded GL engine    }
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	kCGLCESwapRectangle    = 201;	{ Enable or disable the swap rectangle          }
	kCGLCESwapLimit        = 203;	{ Enable or disable the swap async limit        }
	kCGLCERasterization    = 221;	{ Enable or disable all rasterization           }
	kCGLCEStateValidation  = 301;	{ Validate state for multi-screen functionality }
	kCGLCESurfaceBackingSize = 305;  { Enable or disable surface backing size override }
	kCGLCEDisplayListOptimization = 307;  { Ability to turn off display list optimizer }
<<<<<<< HEAD
	kCGLCEMPEngine            = 313;    { Enable or disable multi-threaded GL engine    }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kCGLCEMPEngine         = 313; 	{ Enable or disable multi-threaded GL engine    }
>>>>>>> origin/fixes_2.4

{
** Parameter names for CGLSetParameter and CGLGetParameter.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
type
	CGLContextParameter = SInt32;
	PCGLContextParameter = ^CGLContextParameter;
	
{ CGLContextParameter }
<<<<<<< HEAD
const
	kCGLCPSwapRectangle          = 200; { 4 params.  Set or get the swap rectangle (x, y, w, h)        }
	kCGLCPSwapInterval           = 222; { 1 param.   0 -> Don't sync, 1 -> Sync to vertical retrace    }
	kCGLCPDispatchTableSize      = 224; { 1 param.   Get the dispatch table size                       }
	{ Note: kCGLCPClientStorage is always a pointer-sized parameter, even though the API claims GLint. }
	kCGLCPClientStorage          = 226; { 1 param.   Context specific generic storage                  }
	kCGLCPSurfaceTexture         = 228; { 3 params.  SID, target, internal_format                      }
{  - Used by AGL - }
{  AGL_STATE_VALIDATION           230 }
{  AGL_BUFFER_NAME                231 }
{  AGL_ORDER_CONTEXT_TO_FRONT     232 }
{  AGL_CONTEXT_SURFACE_ID         233 }
{  AGL_CONTEXT_DISPLAY_ID         234 }
	kCGLCPSurfaceOrder           = 235; { 1 param.   1 -> Above window, -1 -> Below Window             }
	kCGLCPSurfaceOpacity         = 236; { 1 param.   1 -> Surface is opaque (default), 0 -> non-opaque }
{  - Used by AGL - }
{  AGL_CLIP_REGION                254 }
{  AGL_FS_CAPTURE_SINGLE          255 }
	kCGLCPSurfaceBackingSize     = 304; { 2 params.  Width/height of surface backing size              }
{  AGL_SURFACE_VOLATILE           306 }
	kCGLCPSurfaceSurfaceVolatile = 306; { 1 param.   Surface volatile state                            }
	kCGLCPReclaimResources       = 308; { 0 params.                                                    }
	kCGLCPCurrentRendererID      = 309; { 1 param.   Retrieves the current renderer ID                 }
	kCGLCPGPUVertexProcessing    = 310; { 1 param.   Currently processing vertices with GPU (get)      }
	kCGLCPGPUFragmentProcessing  = 311; { 1 param.   Currently processing fragments with GPU (get)     }
	kCGLCPHasDrawable			 = 314; { 1 param.   Boolean returned if drawable is attached			}
	kCGLCPMPSwapsInFlight		 = 315; { 1 param.   Max number of swaps queued by the MP GL engine	}
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	kCGLCPSwapRectangle          = 200; { 4 params.  Set or get the swap rectangle (x, y, w, h)        }
	kCGLCPSwapInterval           = 222; { 1 param.   0 -> Don't sync, 1 -> Sync to vertical retrace    }
	kCGLCPDispatchTableSize      = 224; { 1 param.   Get the dispatch table size                       }
	{ Note: kCGLCPClientStorage is always a pointer-sized parameter, even though the API claims GLint. }
	kCGLCPClientStorage          = 226; { 1 param.   Context specific generic storage                  }
	kCGLCPSurfaceTexture         = 228; { 3 params.  SID, target, internal_format                      }
{  - Used by AGL - }
{  AGL_STATE_VALIDATION           230 }
{  AGL_BUFFER_NAME                231 }
{  AGL_ORDER_CONTEXT_TO_FRONT     232 }
{  AGL_CONTEXT_SURFACE_ID         233 }
{  AGL_CONTEXT_DISPLAY_ID         234 }
	kCGLCPSurfaceOrder           = 235; { 1 param.   1 -> Above window, -1 -> Below Window             }
	kCGLCPSurfaceOpacity         = 236; { 1 param.   1 -> Surface is opaque (default), 0 -> non-opaque }
{  - Used by AGL - }
<<<<<<< HEAD
{  AGL_CLIP_REGION          254   }
{  AGL_FS_CAPTURE_SINGLE    255   }
	kCGLCPSurfaceBackingSize  = 304;  { 2 params.   Width/height of surface backing size     }
{ AGL_SURFACE_VOLATILE		 306	}
	kCGLCPSurfaceSurfaceVolatile = 306;  { 1 param.   Surface volatile state					}
	kCGLCPReclaimResources       = 308;  { 0 params.  }
	kCGLCPCurrentRendererID      = 309;  { 1 param.   Retrieves the current renderer ID         }
	kCGLCPGPUVertexProcessing    = 310;  { 1 param.   Currently processing vertices with GPU (get) }
	kCGLCPGPUFragmentProcessing  = 311;  { 1 param.   Currently processing fragments with GPU (get) }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{  AGL_CLIP_REGION                254 }
{  AGL_FS_CAPTURE_SINGLE          255 }
	kCGLCPSurfaceBackingSize     = 304; { 2 params.  Width/height of surface backing size              }
{  AGL_SURFACE_VOLATILE           306 }
	kCGLCPSurfaceSurfaceVolatile = 306; { 1 param.   Surface volatile state                            }
	kCGLCPReclaimResources       = 308; { 0 params.                                                    }
	kCGLCPCurrentRendererID      = 309; { 1 param.   Retrieves the current renderer ID                 }
	kCGLCPGPUVertexProcessing    = 310; { 1 param.   Currently processing vertices with GPU (get)      }
	kCGLCPGPUFragmentProcessing  = 311; { 1 param.   Currently processing fragments with GPU (get)     }
	kCGLCPHasDrawable			 = 314; { 1 param.   Boolean returned if drawable is attached			}
	kCGLCPMPSwapsInFlight		 = 315; { 1 param.   Max number of swaps queued by the MP GL engine	}
>>>>>>> origin/fixes_2.4

{
** Option names for CGLSetOption and CGLGetOption.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
type
	CGLGlobalOption = SInt32;
	PCGLGlobalOption = ^CGLGlobalOption;
	
{ CGLGlobalOption }
<<<<<<< HEAD
const
	kCGLGOFormatCacheSize  = 501;	{ Set the size of the pixel format cache        }
	kCGLGOClearFormatCache = 502;	{ Reset the pixel format cache if true          }
	kCGLGORetainRenderers  = 503;	{ Whether to retain loaded renderers in memory  }
	kCGLGOResetLibrary     = 504;	{ *** DEPRECATED in MacOS X 10.4 ***            }
	                             	{ Do a soft reset of the CGL library if true    }
	kCGLGOUseErrorHandler  = 505;	{ Call the Core Graphics handler on CGL errors  }
	kCGLGOUseBuildCache    = 506;	{ Enable the function compilation block cache.  }
	                                { Off by default.  Must be enabled at startup.  }
=======
=======
>>>>>>> origin/fixes_2_2
const
	kCGLGOFormatCacheSize     = 501;	{ Set the size of the pixel format cache        }
	kCGLGOClearFormatCache    = 502;	{ Reset the pixel format cache if true          }
	kCGLGORetainRenderers     = 503;	{ Whether to retain loaded renderers in memory  }
	kCGLGOResetLibrary        = 504;	{ *** DEPRECATED in MacOS X 10.4 ***            }
	                             		{ Do a soft reset of the CGL library if true    }
	kCGLGOUseErrorHandler     = 505;	{ Call the Core Graphics handler on CGL errors  }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kCGLGOFormatCacheSize  = 501;	{ Set the size of the pixel format cache        }
	kCGLGOClearFormatCache = 502;	{ Reset the pixel format cache if true          }
	kCGLGORetainRenderers  = 503;	{ Whether to retain loaded renderers in memory  }
	kCGLGOResetLibrary     = 504;	{ *** DEPRECATED in MacOS X 10.4 ***            }
	                             	{ Do a soft reset of the CGL library if true    }
	kCGLGOUseErrorHandler  = 505;	{ Call the Core Graphics handler on CGL errors  }
	kCGLGOUseBuildCache    = 506;	{ Enable the function compilation block cache.  }
	                                { Off by default.  Must be enabled at startup.  }
>>>>>>> origin/fixes_2.4

{
** Error return values from CGLGetError.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
type
	CGLError	= SInt32; { in case there will be negative errors in the future }
	PCGLError = ^CGLError;
	
{ CGLError }
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	kCGLNoError               = 0;      { no error }
	kCGLBadAttribute          = 10000;	{ invalid pixel format attribute  }
	kCGLBadProperty           = 10001;	{ invalid renderer property       }
	kCGLBadPixelFormat        = 10002;	{ invalid pixel format            }
	kCGLBadRendererInfo       = 10003;	{ invalid renderer info           }
	kCGLBadContext            = 10004;	{ invalid context                 }
	kCGLBadDrawable           = 10005;	{ invalid drawable                }
	kCGLBadDisplay            = 10006;	{ invalid graphics device         }
	kCGLBadState              = 10007;	{ invalid context state           }
	kCGLBadValue              = 10008;	{ invalid numerical value         }
	kCGLBadMatch              = 10009;	{ invalid share context           }
	kCGLBadEnumeration        = 10010;	{ invalid enumerant               }
	kCGLBadOffScreen          = 10011;	{ invalid offscreen drawable      }
	kCGLBadFullScreen         = 10012;	{ invalid offscreen drawable      }
	kCGLBadWindow             = 10013;	{ invalid window                  }
	kCGLBadAddress            = 10014;	{ invalid pointer                 }
	kCGLBadCodeModule         = 10015;	{ invalid code module             }
	kCGLBadAlloc              = 10016;	{ invalid memory allocation       }
	kCGLBadConnection         = 10017; 	{ invalid CoreGraphics connection }


{ 
** Buffer modes
}
const
	kCGLMonoscopicBit   = $00000001;
	kCGLStereoscopicBit = $00000002;
	kCGLSingleBufferBit = $00000004;
	kCGLDoubleBufferBit = $00000008;

{
** Depth and stencil buffer depths
}
	kCGL0Bit            = $00000001;
	kCGL1Bit            = $00000002;
	kCGL2Bit            = $00000004;
	kCGL3Bit            = $00000008;
	kCGL4Bit            = $00000010;
	kCGL5Bit            = $00000020;
	kCGL6Bit            = $00000040;
	kCGL8Bit            = $00000080;
	kCGL10Bit           = $00000100;
	kCGL12Bit           = $00000200;
	kCGL16Bit           = $00000400;
	kCGL24Bit           = $00000800;
	kCGL32Bit           = $00001000;
	kCGL48Bit           = $00002000;
	kCGL64Bit           = $00004000;
	kCGL96Bit           = $00008000;
	kCGL128Bit          = $00010000;

{
** Color and accumulation buffer formats.
}
	kCGLRGB444Bit       = $00000040;  { 16 rgb bit/pixel,    R=11:8, G=7:4, B=3:0              }
	kCGLARGB4444Bit     = $00000080;  { 16 argb bit/pixel,   A=15:12, R=11:8, G=7:4, B=3:0     }
	kCGLRGB444A8Bit     = $00000100;  { 8-16 argb bit/pixel, A=7:0, R=11:8, G=7:4, B=3:0       }
	kCGLRGB555Bit       = $00000200;  { 16 rgb bit/pixel,    R=14:10, G=9:5, B=4:0             }
	kCGLARGB1555Bit     = $00000400;  { 16 argb bit/pixel,   A=15, R=14:10, G=9:5, B=4:0       }
	kCGLRGB555A8Bit     = $00000800;  { 8-16 argb bit/pixel, A=7:0, R=14:10, G=9:5, B=4:0      }
	kCGLRGB565Bit       = $00001000;  { 16 rgb bit/pixel,    R=15:11, G=10:5, B=4:0            }
	kCGLRGB565A8Bit     = $00002000;  { 8-16 argb bit/pixel, A=7:0, R=15:11, G=10:5, B=4:0     }
	kCGLRGB888Bit       = $00004000;  { 32 rgb bit/pixel,    R=23:16, G=15:8, B=7:0            }
	kCGLARGB8888Bit     = $00008000;  { 32 argb bit/pixel,   A=31:24, R=23:16, G=15:8, B=7:0   }
	kCGLRGB888A8Bit     = $00010000;  { 8-32 argb bit/pixel, A=7:0, R=23:16, G=15:8, B=7:0     }
	kCGLRGB101010Bit    = $00020000;  { 32 rgb bit/pixel,    R=29:20, G=19:10, B=9:0           }
	kCGLARGB2101010Bit  = $00040000;  { 32 argb bit/pixel,   A=31:30  R=29:20, G=19:10, B=9:0  }
	kCGLRGB101010_A8Bit = $00080000;  { 8-32 argb bit/pixel, A=7:0  R=29:20, G=19:10, B=9:0    }
	kCGLRGB121212Bit    = $00100000;  { 48 rgb bit/pixel,    R=35:24, G=23:12, B=11:0          }
	kCGLARGB12121212Bit = $00200000;  { 48 argb bit/pixel,   A=47:36, R=35:24, G=23:12, B=11:0 }
	kCGLRGB161616Bit    = $00400000;  { 64 rgb bit/pixel,    R=63:48, G=47:32, B=31:16         }
	kCGLRGBA16161616Bit = $00800000;  { 64 argb bit/pixel,   R=63:48, G=47:32, B=31:16, A=15:0 }
	kCGLRGBFloat64Bit   = $01000000;  { 64 rgb bit/pixel,    half float                        }
	kCGLRGBAFloat64Bit  = $02000000;  { 64 argb bit/pixel,   half float                        }
	kCGLRGBFloat128Bit  = $04000000;  { 128 rgb bit/pixel,   ieee float                        }
	kCGLRGBAFloat128Bit = $08000000;  { 128 argb bit/pixel,  ieee float                        }
	kCGLRGBFloat256Bit  = $10000000;  { 256 rgb bit/pixel,   ieee double                       }
	kCGLRGBAFloat256Bit = $20000000;  { 256 argb bit/pixel,  ieee double                       }

{
** Sampling modes
}
	kCGLSupersampleBit = $00000001;
	kCGLMultisampleBit = $00000002;

{ Obsolete }
	kCGLARGB16161616Bit = kCGLRGBA16161616Bit;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
end.
>>>>>>> graemeg/fixes_2_2
=======
end.
>>>>>>> origin/fixes_2_2
=======
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
