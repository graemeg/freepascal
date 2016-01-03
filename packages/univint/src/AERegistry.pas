{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       AE/AERegistry.h
 
     Contains:   AppleEvents Registry Interface.
 
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
 
     Copyright:  © 1993-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       AERegistry.p
=======
     File:       AE/AERegistry.h
>>>>>>> origin/fixes_2.4
 
     Contains:   AppleEvents Registry Interface.
 
     Version:    AppleEvents-496~1
 
<<<<<<< HEAD
     Copyright:  © 1993-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1993-2008 by Apple Computer, Inc., all rights reserved
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
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
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
                     http://www.freepascal.org/bugs.html
 
}
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

unit AERegistry;
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
<<<<<<< HEAD
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
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
uses MacTypes,ATSTypes,MacErrors,AppleEvents,AEDataModel;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======
uses MacTypes,ATSTypes,MacErrors,AppleEvents;
=======
uses MacTypes,ATSTypes,MacErrors,AppleEvents,AEDataModel;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,ATSTypes,MacErrors,AppleEvents;

>>>>>>> origin/fixes_2_2

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}


const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	cAEList = FourCharCode('list'); {  0x6c697374  }
	cApplication = FourCharCode('capp'); {  0x63617070  }
	cArc = FourCharCode('carc'); {  0x63617263  }
	cBoolean = FourCharCode('bool'); {  0x626f6f6c  }
	cCell = FourCharCode('ccel'); {  0x6363656c  }
	cChar = FourCharCode('cha '); {  0x63686120  }
	cColorTable = FourCharCode('clrt'); {  0x636c7274  }
	cColumn = FourCharCode('ccol'); {  0x63636f6c  }
	cDocument = FourCharCode('docu'); {  0x646f6375  }
	cDrawingArea = FourCharCode('cdrw'); {  0x63647277  }
	cEnumeration = FourCharCode('enum'); {  0x656e756d  }
	cFile = FourCharCode('file'); {  0x66696c65  }
	cFixed = FourCharCode('fixd'); {  0x66697864  }
	cFixedPoint = FourCharCode('fpnt'); {  0x66706e74  }
	cFixedRectangle = FourCharCode('frct'); {  0x66726374  }
	cGraphicLine = FourCharCode('glin'); {  0x676c696e  }
	cGraphicObject = FourCharCode('cgob'); {  0x63676f62  }
	cGraphicShape = FourCharCode('cgsh'); {  0x63677368  }
	cGraphicText = FourCharCode('cgtx'); {  0x63677478  }
	cGroupedGraphic = FourCharCode('cpic'); {  0x63706963  }

const
	cInsertionLoc = FourCharCode('insl'); {  0x696e736c  }
	cInsertionPoint = FourCharCode('cins'); {  0x63696e73  }
	cIntlText = FourCharCode('itxt'); {  0x69747874  }
	cIntlWritingCode = FourCharCode('intl'); {  0x696e746c  }
	cItem = FourCharCode('citm'); {  0x6369746d  }
	cLine = FourCharCode('clin'); {  0x636c696e  }
	cLongDateTime = FourCharCode('ldt '); {  0x6c647420  }
	cLongFixed = FourCharCode('lfxd'); {  0x6c667864  }
	cLongFixedPoint = FourCharCode('lfpt'); {  0x6c667074  }
	cLongFixedRectangle = FourCharCode('lfrc'); {  0x6c667263  }
	cLongInteger = FourCharCode('long'); {  0x6c6f6e67  }
	cLongPoint = FourCharCode('lpnt'); {  0x6c706e74  }
	cLongRectangle = FourCharCode('lrct'); {  0x6c726374  }
	cMachineLoc = FourCharCode('mLoc'); {  0x6d4c6f63  }
	cMenu = FourCharCode('cmnu'); {  0x636d6e75  }
	cMenuItem = FourCharCode('cmen'); {  0x636d656e  }
	cObject = FourCharCode('cobj'); {  0x636f626a  }
	cObjectSpecifier = FourCharCode('obj '); {  0x6f626a20  }
	cOpenableObject = FourCharCode('coob'); {  0x636f6f62  }
	cOval = FourCharCode('covl'); {  0x636f766c  }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

const
	cParagraph = FourCharCode('cpar'); {  0x63706172  }
	cPICT = FourCharCode('PICT'); {  0x50494354  }
	cPixel = FourCharCode('cpxl'); {  0x6370786c  }
	cPixelMap = FourCharCode('cpix'); {  0x63706978  }
	cPolygon = FourCharCode('cpgn'); {  0x6370676e  }
	cProperty = FourCharCode('prop'); {  0x70726f70  }
	cQDPoint = FourCharCode('QDpt'); {  0x51447074  }
	cQDRectangle = FourCharCode('qdrt'); {  0x71647274  }
	cRectangle = FourCharCode('crec'); {  0x63726563  }
	cRGBColor = FourCharCode('cRGB'); {  0x63524742  }
	cRotation = FourCharCode('trot'); {  0x74726f74  }
	cRoundedRectangle = FourCharCode('crrc'); {  0x63727263  }
	cRow = FourCharCode('crow'); {  0x63726f77  }
	cSelection = FourCharCode('csel'); {  0x6373656c  }
	cShortInteger = FourCharCode('shor'); {  0x73686f72  }
	cTable = FourCharCode('ctbl'); {  0x6374626c  }
	cText = FourCharCode('ctxt'); {  0x63747874  }
	cTextFlow = FourCharCode('cflo'); {  0x63666c6f  }
	cTextStyles = FourCharCode('tsty'); {  0x74737479  }
	cType = FourCharCode('type'); {  0x74797065  }

const
	cVersion = FourCharCode('vers'); {  0x76657273  }
	cWindow = FourCharCode('cwin'); {  0x6377696e  }
	cWord = FourCharCode('cwor'); {  0x63776f72  }
	enumArrows = FourCharCode('arro'); {  0x6172726f  }
	enumJustification = FourCharCode('just'); {  0x6a757374  }
	enumKeyForm = FourCharCode('kfrm'); {  0x6b66726d  }
	enumPosition = FourCharCode('posi'); {  0x706f7369  }
	enumProtection = FourCharCode('prtn'); {  0x7072746e  }
	enumQuality = FourCharCode('qual'); {  0x7175616c  }
	enumSaveOptions = FourCharCode('savo'); {  0x7361766f  }
	enumStyle = FourCharCode('styl'); {  0x7374796c  }
	enumTransferMode = FourCharCode('tran'); {  0x7472616e  }
	kAEAbout = FourCharCode('abou'); {  0x61626f75  }
	kAEAfter = FourCharCode('afte'); {  0x61667465  }
	kAEAliasSelection = FourCharCode('sali'); {  0x73616c69  }
	kAEAllCaps = FourCharCode('alcp'); {  0x616c6370  }
	kAEArrowAtEnd = FourCharCode('aren'); {  0x6172656e  }
	kAEArrowAtStart = FourCharCode('arst'); {  0x61727374  }
	kAEArrowBothEnds = FourCharCode('arbo'); {  0x6172626f  }

const
	kAEAsk = FourCharCode('ask '); {  0x61736b20  }
	kAEBefore = FourCharCode('befo'); {  0x6265666f  }
	kAEBeginning = FourCharCode('bgng'); {  0x62676e67  }
	kAEBeginsWith = FourCharCode('bgwt'); {  0x62677774  }
	kAEBeginTransaction = FourCharCode('begi'); {  0x62656769  }
	kAEBold = FourCharCode('bold'); {  0x626f6c64  }
	kAECaseSensEquals = FourCharCode('cseq'); {  0x63736571  }
	kAECentered = FourCharCode('cent'); {  0x63656e74  }
	kAEChangeView = FourCharCode('view'); {  0x76696577  }
	kAEClone = FourCharCode('clon'); {  0x636c6f6e  }
	kAEClose = FourCharCode('clos'); {  0x636c6f73  }
	kAECondensed = FourCharCode('cond'); {  0x636f6e64  }
	kAEContains = FourCharCode('cont'); {  0x636f6e74  }
	kAECopy = FourCharCode('copy'); {  0x636f7079  }
	kAECoreSuite = FourCharCode('core'); {  0x636f7265  }
	kAECountElements = FourCharCode('cnte'); {  0x636e7465  }
	kAECreateElement = FourCharCode('crel'); {  0x6372656c  }
	kAECreatePublisher = FourCharCode('cpub'); {  0x63707562  }
	kAECut = FourCharCode('cut '); {  0x63757420  }
	kAEDelete = FourCharCode('delo'); {  0x64656c6f  }

const
	kAEDoObjectsExist = FourCharCode('doex'); {  0x646f6578  }
	kAEDoScript = FourCharCode('dosc'); {  0x646f7363  }
	kAEDrag = FourCharCode('drag'); {  0x64726167  }
	kAEDuplicateSelection = FourCharCode('sdup'); {  0x73647570  }
	kAEEditGraphic = FourCharCode('edit'); {  0x65646974  }
	kAEEmptyTrash = FourCharCode('empt'); {  0x656d7074  }
	kAEEnd = FourCharCode('end '); {  0x656e6420  }
	kAEEndsWith = FourCharCode('ends'); {  0x656e6473  }
	kAEEndTransaction = FourCharCode('endt'); {  0x656e6474  }
	kAEEquals = FourCharCode('=   '); {  0x3d202020  }
	kAEExpanded = FourCharCode('pexp'); {  0x70657870  }
	kAEFast = FourCharCode('fast'); {  0x66617374  }
	kAEFinderEvents = FourCharCode('FNDR'); {  0x464e4452  }
	kAEFormulaProtect = FourCharCode('fpro'); {  0x6670726f  }
	kAEFullyJustified = FourCharCode('full'); {  0x66756c6c  }
	kAEGetClassInfo = FourCharCode('qobj'); {  0x716f626a  }
	kAEGetData = FourCharCode('getd'); {  0x67657464  }
	kAEGetDataSize = FourCharCode('dsiz'); {  0x6473697a  }
	kAEGetEventInfo = FourCharCode('gtei'); {  0x67746569  }
	kAEGetInfoSelection = FourCharCode('sinf'); {  0x73696e66  }

const
	kAEGetPrivilegeSelection = FourCharCode('sprv'); {  0x73707276  }
	kAEGetSuiteInfo = FourCharCode('gtsi'); {  0x67747369  }
	kAEGreaterThan = FourCharCode('>   '); {  0x3e202020  }
	kAEGreaterThanEquals = FourCharCode('>=  '); {  0x3e3d2020  }
	kAEGrow = FourCharCode('grow'); {  0x67726f77  }
	kAEHidden = FourCharCode('hidn'); {  0x6869646e  }
	kAEHiQuality = FourCharCode('hiqu'); {  0x68697175  }
	kAEImageGraphic = FourCharCode('imgr'); {  0x696d6772  }
	kAEIsUniform = FourCharCode('isun'); {  0x6973756e  }
	kAEItalic = FourCharCode('ital'); {  0x6974616c  }
	kAELeftJustified = FourCharCode('left'); {  0x6c656674  }
	kAELessThan = FourCharCode('<   '); {  0x3c202020  }
	kAELessThanEquals = FourCharCode('<=  '); {  0x3c3d2020  }
	kAELowercase = FourCharCode('lowc'); {  0x6c6f7763  }
	kAEMakeObjectsVisible = FourCharCode('mvis'); {  0x6d766973  }
	kAEMiscStandards = FourCharCode('misc'); {  0x6d697363  }
	kAEModifiable = FourCharCode('modf'); {  0x6d6f6466  }
	kAEMove = FourCharCode('move'); {  0x6d6f7665  }
	kAENo = FourCharCode('no  '); {  0x6e6f2020  }
	kAENoArrow = FourCharCode('arno'); {  0x61726e6f  }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

const
	kAENonmodifiable = FourCharCode('nmod'); {  0x6e6d6f64  }
	kAEOpen = FourCharCode('odoc'); {  0x6f646f63  }
	kAEOpenSelection = FourCharCode('sope'); {  0x736f7065  }
	kAEOutline = FourCharCode('outl'); {  0x6f75746c  }
	kAEPageSetup = FourCharCode('pgsu'); {  0x70677375  }
	kAEPaste = FourCharCode('past'); {  0x70617374  }
	kAEPlain = FourCharCode('plan'); {  0x706c616e  }
	kAEPrint = FourCharCode('pdoc'); {  0x70646f63  }
	kAEPrintSelection = FourCharCode('spri'); {  0x73707269  }
	kAEPrintWindow = FourCharCode('pwin'); {  0x7077696e  }
	kAEPutAwaySelection = FourCharCode('sput'); {  0x73707574  }
	kAEQDAddOver = FourCharCode('addo'); {  0x6164646f  }
	kAEQDAddPin = FourCharCode('addp'); {  0x61646470  }
	kAEQDAdMax = FourCharCode('admx'); {  0x61646d78  }
	kAEQDAdMin = FourCharCode('admn'); {  0x61646d6e  }
	kAEQDBic = FourCharCode('bic '); {  0x62696320  }
	kAEQDBlend = FourCharCode('blnd'); {  0x626c6e64  }
	kAEQDCopy = FourCharCode('cpy '); {  0x63707920  }
	kAEQDNotBic = FourCharCode('nbic'); {  0x6e626963  }
	kAEQDNotCopy = FourCharCode('ncpy'); {  0x6e637079  }

const
	kAEQDNotOr = FourCharCode('ntor'); {  0x6e746f72  }
	kAEQDNotXor = FourCharCode('nxor'); {  0x6e786f72  }
	kAEQDOr = FourCharCode('or  '); {  0x6f722020  }
	kAEQDSubOver = FourCharCode('subo'); {  0x7375626f  }
	kAEQDSubPin = FourCharCode('subp'); {  0x73756270  }
	kAEQDSupplementalSuite = FourCharCode('qdsp'); {  0x71647370  }
	kAEQDXor = FourCharCode('xor '); {  0x786f7220  }
	kAEQuickdrawSuite = FourCharCode('qdrw'); {  0x71647277  }
	kAEQuitAll = FourCharCode('quia'); {  0x71756961  }
	kAERedo = FourCharCode('redo'); {  0x7265646f  }
	kAERegular = FourCharCode('regl'); {  0x7265676c  }
	kAEReopenApplication = FourCharCode('rapp'); {  0x72617070  }
	kAEReplace = FourCharCode('rplc'); {  0x72706c63  }
	kAERequiredSuite = FourCharCode('reqd'); {  0x72657164  }
	kAERestart = FourCharCode('rest'); {  0x72657374  }
	kAERevealSelection = FourCharCode('srev'); {  0x73726576  }
	kAERevert = FourCharCode('rvrt'); {  0x72767274  }
	kAERightJustified = FourCharCode('rght'); {  0x72676874  }
	kAESave = FourCharCode('save'); {  0x73617665  }
	kAESelect = FourCharCode('slct'); {  0x736c6374  }
	kAESetData = FourCharCode('setd'); {  0x73657464  }

const
	kAESetPosition = FourCharCode('posn'); {  0x706f736e  }
	kAEShadow = FourCharCode('shad'); {  0x73686164  }
	kAEShowClipboard = FourCharCode('shcl'); {  0x7368636c  }
	kAEShutDown = FourCharCode('shut'); {  0x73687574  }
	kAESleep = FourCharCode('slep'); {  0x736c6570  }
	kAESmallCaps = FourCharCode('smcp'); {  0x736d6370  }
	kAESpecialClassProperties = FourCharCode('c@#!'); {  0x63402321  }
	kAEStrikethrough = FourCharCode('strk'); {  0x7374726b  }
	kAESubscript = FourCharCode('sbsc'); {  0x73627363  }
	kAESuperscript = FourCharCode('spsc'); {  0x73707363  }
	kAETableSuite = FourCharCode('tbls'); {  0x74626c73  }
	kAETextSuite = FourCharCode('TEXT'); {  0x54455854  }
	kAETransactionTerminated = FourCharCode('ttrm'); {  0x7474726d  }
	kAEUnderline = FourCharCode('undl'); {  0x756e646c  }
	kAEUndo = FourCharCode('undo'); {  0x756e646f  }
	kAEWholeWordEquals = FourCharCode('wweq'); {  0x77776571  }
	kAEYes = FourCharCode('yes '); {  0x79657320  }
	kAEZoom = FourCharCode('zoom'); {  0x7a6f6f6d  }

{ events that can be sent to the "system" process (eg, loginwindow) on OS X 10.2 or later }
const
	kAELogOut = FourCharCode('logo');
	kAEReallyLogOut = FourCharCode('rlgo');
	kAEShowRestartDialog = FourCharCode('rrst');
	kAEShowShutdownDialog = FourCharCode('rsdn');

{ EventRecord Classes and EventIDs }
const
	kAEMouseClass = FourCharCode('mous');
	kAEDown = FourCharCode('down');
	kAEUp = FourCharCode('up  ');
	kAEMoved = FourCharCode('move');
	kAEStoppedMoving = FourCharCode('stop');
	kAEWindowClass = FourCharCode('wind');
	kAEUpdate = FourCharCode('updt');
	kAEActivate = FourCharCode('actv');
	kAEDeactivate = FourCharCode('dact');
	kAECommandClass = FourCharCode('cmnd'); { Modern Command Event Class }
	kAEKeyClass = FourCharCode('keyc');
	kAERawKey = FourCharCode('rkey'); { Modern Raw Key Event }
	kAEVirtualKey = FourCharCode('keyc'); { Modern Virtual Key Event }
	kAENavigationKey = FourCharCode('nave'); { Modern Navigation Key Event }
	kAEAutoDown = FourCharCode('auto');
	kAEApplicationClass = FourCharCode('appl');
	kAESuspend = FourCharCode('susp');
	kAEResume = FourCharCode('rsme');
	kAEDiskEvent = FourCharCode('disk');
	kAENullEvent = FourCharCode('null');
	kAEWakeUpEvent = FourCharCode('wake');
	kAEScrapEvent = FourCharCode('scrp');
	kAEHighLevel = FourCharCode('high');
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

const
	keyAEAngle = FourCharCode('kang'); {  0x6b616e67  }
	keyAEArcAngle = FourCharCode('parc'); {  0x70617263  }

const
	keyAEBaseAddr = FourCharCode('badd'); {  0x62616464  }
	keyAEBestType = FourCharCode('pbst'); {  0x70627374  }
	keyAEBgndColor = FourCharCode('kbcl'); {  0x6b62636c  }
	keyAEBgndPattern = FourCharCode('kbpt'); {  0x6b627074  }
	keyAEBounds = FourCharCode('pbnd'); {  0x70626e64  }
	keyAECellList = FourCharCode('kclt'); {  0x6b636c74  }
	keyAEClassID = FourCharCode('clID'); {  0x636c4944  }
	keyAEColor = FourCharCode('colr'); {  0x636f6c72  }
	keyAEColorTable = FourCharCode('cltb'); {  0x636c7462  }
	keyAECurveHeight = FourCharCode('kchd'); {  0x6b636864  }
	keyAECurveWidth = FourCharCode('kcwd'); {  0x6b637764  }
	keyAEDashStyle = FourCharCode('pdst'); {  0x70647374  }
	keyAEData = FourCharCode('data'); {  0x64617461  }
	keyAEDefaultType = FourCharCode('deft'); {  0x64656674  }
	keyAEDefinitionRect = FourCharCode('pdrt'); {  0x70647274  }
	keyAEDescType = FourCharCode('dstp'); {  0x64737470  }
	keyAEDestination = FourCharCode('dest'); {  0x64657374  }
	keyAEDoAntiAlias = FourCharCode('anta'); {  0x616e7461  }
	keyAEDoDithered = FourCharCode('gdit'); {  0x67646974  }
	keyAEDoRotate = FourCharCode('kdrt'); {  0x6b647274  }

const
	keyAEDoScale = FourCharCode('ksca'); {  0x6b736361  }
	keyAEDoTranslate = FourCharCode('ktra'); {  0x6b747261  }
	keyAEEditionFileLoc = FourCharCode('eloc'); {  0x656c6f63  }
	keyAEElements = FourCharCode('elms'); {  0x656c6d73  }
	keyAEEndPoint = FourCharCode('pend'); {  0x70656e64  }
	keyAEEventClass = FourCharCode('evcl'); {  0x6576636c  }
	keyAEEventID = FourCharCode('evti'); {  0x65767469  }
	keyAEFile = FourCharCode('kfil'); {  0x6b66696c  }
	keyAEFileType = FourCharCode('fltp'); {  0x666c7470  }
	keyAEFillColor = FourCharCode('flcl'); {  0x666c636c  }
	keyAEFillPattern = FourCharCode('flpt'); {  0x666c7074  }
	keyAEFlipHorizontal = FourCharCode('kfho'); {  0x6b66686f  }
	keyAEFlipVertical = FourCharCode('kfvt'); {  0x6b667674  }
	keyAEFont = FourCharCode('font'); {  0x666f6e74  }
	keyAEFormula = FourCharCode('pfor'); {  0x70666f72  }
	keyAEGraphicObjects = FourCharCode('gobs'); {  0x676f6273  }
	keyAEID = FourCharCode('ID  '); {  0x49442020  }
	keyAEImageQuality = FourCharCode('gqua'); {  0x67717561  }
	keyAEInsertHere = FourCharCode('insh'); {  0x696e7368  }
	keyAEKeyForms = FourCharCode('keyf'); {  0x6b657966  }

const
	keyAEKeyword = FourCharCode('kywd'); {  0x6b797764  }
	keyAELevel = FourCharCode('levl'); {  0x6c65766c  }
	keyAELineArrow = FourCharCode('arro'); {  0x6172726f  }
	keyAEName = FourCharCode('pnam'); {  0x706e616d  }
	keyAENewElementLoc = FourCharCode('pnel'); {  0x706e656c  }
	keyAEObject = FourCharCode('kobj'); {  0x6b6f626a  }
	keyAEObjectClass = FourCharCode('kocl'); {  0x6b6f636c  }
	keyAEOffStyles = FourCharCode('ofst'); {  0x6f667374  }
	keyAEOnStyles = FourCharCode('onst'); {  0x6f6e7374  }
	keyAEParameters = FourCharCode('prms'); {  0x70726d73  }
	keyAEParamFlags = FourCharCode('pmfg'); {  0x706d6667  }
	keyAEPenColor = FourCharCode('ppcl'); {  0x7070636c  }
	keyAEPenPattern = FourCharCode('pppa'); {  0x70707061  }
	keyAEPenWidth = FourCharCode('ppwd'); {  0x70707764  }
	keyAEPixelDepth = FourCharCode('pdpt'); {  0x70647074  }
	keyAEPixMapMinus = FourCharCode('kpmm'); {  0x6b706d6d  }
	keyAEPMTable = FourCharCode('kpmt'); {  0x6b706d74  }
	keyAEPointList = FourCharCode('ptlt'); {  0x70746c74  }
	keyAEPointSize = FourCharCode('ptsz'); {  0x7074737a  }
	keyAEPosition = FourCharCode('kpos'); {  0x6b706f73  }

const
	keyAEPropData = FourCharCode('prdt'); {  0x70726474  }
	keyAEProperties = FourCharCode('qpro'); {  0x7170726f  }
	keyAEProperty = FourCharCode('kprp'); {  0x6b707270  }
	keyAEPropFlags = FourCharCode('prfg'); {  0x70726667  }
	keyAEPropID = FourCharCode('prop'); {  0x70726f70  }
	keyAEProtection = FourCharCode('ppro'); {  0x7070726f  }
	keyAERenderAs = FourCharCode('kren'); {  0x6b72656e  }
	keyAERequestedType = FourCharCode('rtyp'); {  0x72747970  }
	keyAEResult = FourCharCode('----'); {  0x2d2d2d2d  }
	keyAEResultInfo = FourCharCode('rsin'); {  0x7273696e  }
	keyAERotation = FourCharCode('prot'); {  0x70726f74  }
	keyAERotPoint = FourCharCode('krtp'); {  0x6b727470  }
	keyAERowList = FourCharCode('krls'); {  0x6b726c73  }
	keyAESaveOptions = FourCharCode('savo'); {  0x7361766f  }
	keyAEScale = FourCharCode('pscl'); {  0x7073636c  }
	keyAEScriptTag = FourCharCode('psct'); {  0x70736374  }
	keyAESearchText = FourCharCode('stxt'); {  0x73747874  }
	keyAEShowWhere = FourCharCode('show'); {  0x73686f77  }
	keyAEStartAngle = FourCharCode('pang'); {  0x70616e67  }
	keyAEStartPoint = FourCharCode('pstp'); {  0x70737470  }
	keyAEStyles = FourCharCode('ksty'); {  0x6b737479  }

const
	keyAESuiteID = FourCharCode('suit'); {  0x73756974  }
	keyAEText = FourCharCode('ktxt'); {  0x6b747874  }
	keyAETextColor = FourCharCode('ptxc'); {  0x70747863  }
	keyAETextFont = FourCharCode('ptxf'); {  0x70747866  }
	keyAETextPointSize = FourCharCode('ptps'); {  0x70747073  }
	keyAETextStyles = FourCharCode('txst'); {  0x74787374  }
	keyAETextLineHeight = FourCharCode('ktlh'); { type ShortInteger }
	keyAETextLineAscent = FourCharCode('ktas'); { type ShortInteger }
	keyAETheText = FourCharCode('thtx'); {  0x74687478  }
	keyAETransferMode = FourCharCode('pptm'); {  0x7070746d  }
	keyAETranslation = FourCharCode('ptrs'); {  0x70747273  }
	keyAETryAsStructGraf = FourCharCode('toog'); {  0x746f6f67  }
	keyAEUniformStyles = FourCharCode('ustl'); {  0x7573746c  }
	keyAEUpdateOn = FourCharCode('pupd'); {  0x70757064  }
	keyAEUserTerm = FourCharCode('utrm'); {  0x7574726d  }
	keyAEWindow = FourCharCode('wndw'); {  0x776e6477  }
	keyAEWritingCode = FourCharCode('wrcd'); {  0x77726364  }

const
	keyMiscellaneous = FourCharCode('fmsc'); {  0x666d7363  }
	keySelection = FourCharCode('fsel'); {  0x6673656c  }
	keyWindow = FourCharCode('kwnd'); {  0x6b776e64  }
                                        { EventRecord keys }
	keyWhen = FourCharCode('when');
	keyWhere = FourCharCode('wher');
	keyModifiers = FourCharCode('mods');
	keyKey = FourCharCode('key ');
	keyKeyCode = FourCharCode('code');
	keyKeyboard = FourCharCode('keyb');
	keyDriveNumber = FourCharCode('drv#');
	keyErrorCode = FourCharCode('err#');
	keyHighLevelClass = FourCharCode('hcls');
	keyHighLevelID = FourCharCode('hid ');

const
	pArcAngle = FourCharCode('parc'); {  0x70617263  }
	pBackgroundColor = FourCharCode('pbcl'); {  0x7062636c  }
	pBackgroundPattern = FourCharCode('pbpt'); {  0x70627074  }
	pBestType = FourCharCode('pbst'); {  0x70627374  }
	pBounds = FourCharCode('pbnd'); {  0x70626e64  }
	pClass = FourCharCode('pcls'); {  0x70636c73  }
	pClipboard = FourCharCode('pcli'); {  0x70636c69  }
	pColor = FourCharCode('colr'); {  0x636f6c72  }
	pColorTable = FourCharCode('cltb'); {  0x636c7462  }
	pContents = FourCharCode('pcnt'); {  0x70636e74  }
	pCornerCurveHeight = FourCharCode('pchd'); {  0x70636864  }
	pCornerCurveWidth = FourCharCode('pcwd'); {  0x70637764  }
	pDashStyle = FourCharCode('pdst'); {  0x70647374  }
	pDefaultType = FourCharCode('deft'); {  0x64656674  }
	pDefinitionRect = FourCharCode('pdrt'); {  0x70647274  }
	pEnabled = FourCharCode('enbl'); {  0x656e626c  }
	pEndPoint = FourCharCode('pend'); {  0x70656e64  }
	pFillColor = FourCharCode('flcl'); {  0x666c636c  }
	pFillPattern = FourCharCode('flpt'); {  0x666c7074  }
	pFont = FourCharCode('font'); {  0x666f6e74  }

const
	pFormula = FourCharCode('pfor'); {  0x70666f72  }
	pGraphicObjects = FourCharCode('gobs'); {  0x676f6273  }
	pHasCloseBox = FourCharCode('hclb'); {  0x68636c62  }
	pHasTitleBar = FourCharCode('ptit'); {  0x70746974  }
	pID = FourCharCode('ID  '); {  0x49442020  }
	pIndex = FourCharCode('pidx'); {  0x70696478  }
	pInsertionLoc = FourCharCode('pins'); {  0x70696e73  }
	pIsFloating = FourCharCode('isfl'); {  0x6973666c  }
	pIsFrontProcess = FourCharCode('pisf'); {  0x70697366  }
	pIsModal = FourCharCode('pmod'); {  0x706d6f64  }
	pIsModified = FourCharCode('imod'); {  0x696d6f64  }
	pIsResizable = FourCharCode('prsz'); {  0x7072737a  }
	pIsStationeryPad = FourCharCode('pspd'); {  0x70737064  }
	pIsZoomable = FourCharCode('iszm'); {  0x69737a6d  }
	pIsZoomed = FourCharCode('pzum'); {  0x707a756d  }
	pItemNumber = FourCharCode('itmn'); {  0x69746d6e  }
	pJustification = FourCharCode('pjst'); {  0x706a7374  }
	pLineArrow = FourCharCode('arro'); {  0x6172726f  }
	pMenuID = FourCharCode('mnid'); {  0x6d6e6964  }
	pName = FourCharCode('pnam'); {  0x706e616d  }

const
	pNewElementLoc = FourCharCode('pnel'); {  0x706e656c  }
	pPenColor = FourCharCode('ppcl'); {  0x7070636c  }
	pPenPattern = FourCharCode('pppa'); {  0x70707061  }
	pPenWidth = FourCharCode('ppwd'); {  0x70707764  }
	pPixelDepth = FourCharCode('pdpt'); {  0x70647074  }
	pPointList = FourCharCode('ptlt'); {  0x70746c74  }
	pPointSize = FourCharCode('ptsz'); {  0x7074737a  }
	pProtection = FourCharCode('ppro'); {  0x7070726f  }
	pRotation = FourCharCode('prot'); {  0x70726f74  }
	pScale = FourCharCode('pscl'); {  0x7073636c  }
	pScript = FourCharCode('scpt'); {  0x73637074  }
	pScriptTag = FourCharCode('psct'); {  0x70736374  }
	pSelected = FourCharCode('selc'); {  0x73656c63  }
	pSelection = FourCharCode('sele'); {  0x73656c65  }
	pStartAngle = FourCharCode('pang'); {  0x70616e67  }
	pStartPoint = FourCharCode('pstp'); {  0x70737470  }
	pTextColor = FourCharCode('ptxc'); {  0x70747863  }
	pTextFont = FourCharCode('ptxf'); {  0x70747866  }
	pTextItemDelimiters = FourCharCode('txdl'); {  0x7478646c  }
	pTextPointSize = FourCharCode('ptps'); {  0x70747073  }

const
	pTextStyles = FourCharCode('txst'); {  0x74787374  }
	pTransferMode = FourCharCode('pptm'); {  0x7070746d  }
	pTranslation = FourCharCode('ptrs'); {  0x70747273  }
	pUniformStyles = FourCharCode('ustl'); {  0x7573746c  }
	pUpdateOn = FourCharCode('pupd'); {  0x70757064  }
	pUserSelection = FourCharCode('pusl'); {  0x7075736c  }
	pVersion = FourCharCode('vers'); {  0x76657273  }
	pVisible = FourCharCode('pvis'); {  0x70766973  }

const
	typeAEText = FourCharCode('tTXT'); {  0x74545854  }
	typeArc = FourCharCode('carc'); {  0x63617263  }
	typeBest = FourCharCode('best'); {  0x62657374  }
	typeCell = FourCharCode('ccel'); {  0x6363656c  }
	typeClassInfo = FourCharCode('gcli'); {  0x67636c69  }
	typeColorTable = FourCharCode('clrt'); {  0x636c7274  }
	typeColumn = FourCharCode('ccol'); {  0x63636f6c  }
	typeDashStyle = FourCharCode('tdas'); {  0x74646173  }
	typeData = FourCharCode('tdta'); {  0x74647461  }
	typeDrawingArea = FourCharCode('cdrw'); {  0x63647277  }
	typeElemInfo = FourCharCode('elin'); {  0x656c696e  }
	typeEnumeration = FourCharCode('enum'); {  0x656e756d  }
	typeEPS = FourCharCode('EPS '); {  0x45505320  }
	typeEventInfo = FourCharCode('evin'); {  0x6576696e  }

const
	typeFinderWindow = FourCharCode('fwin'); {  0x6677696e  }
	typeFixedPoint = FourCharCode('fpnt'); {  0x66706e74  }
	typeFixedRectangle = FourCharCode('frct'); {  0x66726374  }
	typeGraphicLine = FourCharCode('glin'); {  0x676c696e  }
	typeGraphicText = FourCharCode('cgtx'); {  0x63677478  }
	typeGroupedGraphic = FourCharCode('cpic'); {  0x63706963  }
	typeInsertionLoc = FourCharCode('insl'); {  0x696e736c  }
	typeIntlText = FourCharCode('itxt'); {  0x69747874  }
	typeIntlWritingCode = FourCharCode('intl'); {  0x696e746c  }
	typeLongDateTime = FourCharCode('ldt '); {  0x6c647420  }
	typeCFAbsoluteTime = FourCharCode('cfat');
	typeISO8601DateTime = FourCharCode('isot'); {  0x69736f74  data is ascii text of an ISO8601 date }
	typeLongFixed = FourCharCode('lfxd'); {  0x6c667864  }
	typeLongFixedPoint = FourCharCode('lfpt'); {  0x6c667074  }
	typeLongFixedRectangle = FourCharCode('lfrc'); {  0x6c667263  }
	typeLongPoint = FourCharCode('lpnt'); {  0x6c706e74  }
	typeLongRectangle = FourCharCode('lrct'); {  0x6c726374  }
	typeMachineLoc = FourCharCode('mLoc'); {  0x6d4c6f63  }
	typeOval = FourCharCode('covl'); {  0x636f766c  }
	typeParamInfo = FourCharCode('pmin'); {  0x706d696e  }
	typePict = FourCharCode('PICT'); {  0x50494354  }

const
	typePixelMap = FourCharCode('cpix'); {  0x63706978  }
	typePixMapMinus = FourCharCode('tpmm'); {  0x74706d6d  }
	typePolygon = FourCharCode('cpgn'); {  0x6370676e  }
	typePropInfo = FourCharCode('pinf'); {  0x70696e66  }
	typePtr = FourCharCode('ptr '); {  0x70747220  }
	typeQDPoint = FourCharCode('QDpt'); {  0x51447074  }
	typeQDRegion = FourCharCode('Qrgn'); {  0x51447074  (data is actual region data, including rectangle and size, _not_ region handle or ptr)}
	typeRectangle = FourCharCode('crec'); {  0x63726563  }
	typeRGB16 = FourCharCode('tr16'); {  0x74723136  }
	typeRGB96 = FourCharCode('tr96'); {  0x74723936  }
	typeRGBColor = FourCharCode('cRGB'); {  0x63524742  }
	typeRotation = FourCharCode('trot'); {  0x74726f74  }
	typeRoundedRectangle = FourCharCode('crrc'); {  0x63727263  }
	typeRow = FourCharCode('crow'); {  0x63726f77  }
	typeScrapStyles = FourCharCode('styl'); {  0x7374796c  }
	typeScript = FourCharCode('scpt'); {  0x73637074  }
	typeStyledText = FourCharCode('STXT'); {  0x53545854  }
	typeSuiteInfo = FourCharCode('suin'); {  0x7375696e  }
	typeTable = FourCharCode('ctbl'); {  0x6374626c  }
	typeTextStyles = FourCharCode('tsty'); {  0x74737479  }

const
	typeTIFF = FourCharCode('TIFF'); {  0x54494646  }
	typeJPEG = FourCharCode('JPEG');
	typeGIF = FourCharCode('GIFf');
	typeVersion = FourCharCode('vers'); {  0x76657273  }

const
	kAEMenuClass = FourCharCode('menu');
	kAEMenuSelect = FourCharCode('mhit');
	kAEMouseDown = FourCharCode('mdwn');
	kAEMouseDownInBack = FourCharCode('mdbk');
	kAEKeyDown = FourCharCode('kdwn');
	kAEResized = FourCharCode('rsiz');
	kAEPromise = FourCharCode('prom');
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

const
	keyMenuID = FourCharCode('mid ');
	keyMenuItem = FourCharCode('mitm');
	keyCloseAllWindows = FourCharCode('caw ');
	keyOriginalBounds = FourCharCode('obnd');
	keyNewBounds = FourCharCode('nbnd');
	keyLocalWhere = FourCharCode('lwhr');
<<<<<<< HEAD

const
=======

const
	keyMenuID = FourCharCode('mid ');
	keyMenuItem = FourCharCode('mitm');
	keyCloseAllWindows = FourCharCode('caw ');
	keyOriginalBounds = FourCharCode('obnd');
	keyNewBounds = FourCharCode('nbnd');
	keyLocalWhere = FourCharCode('lwhr');

const
>>>>>>> graemeg/cpstrnew
=======

const
	keyAEAngle = FourCharCode('kang'); {  0x6b616e67  }
	keyAEArcAngle = FourCharCode('parc'); {  0x70617263  }

const
	keyAEBaseAddr = FourCharCode('badd'); {  0x62616464  }
	keyAEBestType = FourCharCode('pbst'); {  0x70627374  }
	keyAEBgndColor = FourCharCode('kbcl'); {  0x6b62636c  }
	keyAEBgndPattern = FourCharCode('kbpt'); {  0x6b627074  }
	keyAEBounds = FourCharCode('pbnd'); {  0x70626e64  }
	keyAECellList = FourCharCode('kclt'); {  0x6b636c74  }
	keyAEClassID = FourCharCode('clID'); {  0x636c4944  }
	keyAEColor = FourCharCode('colr'); {  0x636f6c72  }
	keyAEColorTable = FourCharCode('cltb'); {  0x636c7462  }
	keyAECurveHeight = FourCharCode('kchd'); {  0x6b636864  }
	keyAECurveWidth = FourCharCode('kcwd'); {  0x6b637764  }
	keyAEDashStyle = FourCharCode('pdst'); {  0x70647374  }
	keyAEData = FourCharCode('data'); {  0x64617461  }
	keyAEDefaultType = FourCharCode('deft'); {  0x64656674  }
	keyAEDefinitionRect = FourCharCode('pdrt'); {  0x70647274  }
	keyAEDescType = FourCharCode('dstp'); {  0x64737470  }
	keyAEDestination = FourCharCode('dest'); {  0x64657374  }
	keyAEDoAntiAlias = FourCharCode('anta'); {  0x616e7461  }
	keyAEDoDithered = FourCharCode('gdit'); {  0x67646974  }
	keyAEDoRotate = FourCharCode('kdrt'); {  0x6b647274  }

const
	keyAEDoScale = FourCharCode('ksca'); {  0x6b736361  }
	keyAEDoTranslate = FourCharCode('ktra'); {  0x6b747261  }
	keyAEEditionFileLoc = FourCharCode('eloc'); {  0x656c6f63  }
	keyAEElements = FourCharCode('elms'); {  0x656c6d73  }
	keyAEEndPoint = FourCharCode('pend'); {  0x70656e64  }
	keyAEEventClass = FourCharCode('evcl'); {  0x6576636c  }
	keyAEEventID = FourCharCode('evti'); {  0x65767469  }
	keyAEFile = FourCharCode('kfil'); {  0x6b66696c  }
	keyAEFileType = FourCharCode('fltp'); {  0x666c7470  }
	keyAEFillColor = FourCharCode('flcl'); {  0x666c636c  }
	keyAEFillPattern = FourCharCode('flpt'); {  0x666c7074  }
	keyAEFlipHorizontal = FourCharCode('kfho'); {  0x6b66686f  }
	keyAEFlipVertical = FourCharCode('kfvt'); {  0x6b667674  }
	keyAEFont = FourCharCode('font'); {  0x666f6e74  }
	keyAEFormula = FourCharCode('pfor'); {  0x70666f72  }
	keyAEGraphicObjects = FourCharCode('gobs'); {  0x676f6273  }
	keyAEID = FourCharCode('ID  '); {  0x49442020  }
	keyAEImageQuality = FourCharCode('gqua'); {  0x67717561  }
	keyAEInsertHere = FourCharCode('insh'); {  0x696e7368  }
	keyAEKeyForms = FourCharCode('keyf'); {  0x6b657966  }

const
	keyAEKeyword = FourCharCode('kywd'); {  0x6b797764  }
	keyAELevel = FourCharCode('levl'); {  0x6c65766c  }
	keyAELineArrow = FourCharCode('arro'); {  0x6172726f  }
	keyAEName = FourCharCode('pnam'); {  0x706e616d  }
	keyAENewElementLoc = FourCharCode('pnel'); {  0x706e656c  }
	keyAEObject = FourCharCode('kobj'); {  0x6b6f626a  }
	keyAEObjectClass = FourCharCode('kocl'); {  0x6b6f636c  }
	keyAEOffStyles = FourCharCode('ofst'); {  0x6f667374  }
	keyAEOnStyles = FourCharCode('onst'); {  0x6f6e7374  }
	keyAEParameters = FourCharCode('prms'); {  0x70726d73  }
	keyAEParamFlags = FourCharCode('pmfg'); {  0x706d6667  }
	keyAEPenColor = FourCharCode('ppcl'); {  0x7070636c  }
	keyAEPenPattern = FourCharCode('pppa'); {  0x70707061  }
	keyAEPenWidth = FourCharCode('ppwd'); {  0x70707764  }
	keyAEPixelDepth = FourCharCode('pdpt'); {  0x70647074  }
	keyAEPixMapMinus = FourCharCode('kpmm'); {  0x6b706d6d  }
	keyAEPMTable = FourCharCode('kpmt'); {  0x6b706d74  }
	keyAEPointList = FourCharCode('ptlt'); {  0x70746c74  }
	keyAEPointSize = FourCharCode('ptsz'); {  0x7074737a  }
	keyAEPosition = FourCharCode('kpos'); {  0x6b706f73  }

const
	keyAEPropData = FourCharCode('prdt'); {  0x70726474  }
	keyAEProperties = FourCharCode('qpro'); {  0x7170726f  }
	keyAEProperty = FourCharCode('kprp'); {  0x6b707270  }
	keyAEPropFlags = FourCharCode('prfg'); {  0x70726667  }
	keyAEPropID = FourCharCode('prop'); {  0x70726f70  }
	keyAEProtection = FourCharCode('ppro'); {  0x7070726f  }
	keyAERenderAs = FourCharCode('kren'); {  0x6b72656e  }
	keyAERequestedType = FourCharCode('rtyp'); {  0x72747970  }
	keyAEResult = FourCharCode('----'); {  0x2d2d2d2d  }
	keyAEResultInfo = FourCharCode('rsin'); {  0x7273696e  }
	keyAERotation = FourCharCode('prot'); {  0x70726f74  }
	keyAERotPoint = FourCharCode('krtp'); {  0x6b727470  }
	keyAERowList = FourCharCode('krls'); {  0x6b726c73  }
	keyAESaveOptions = FourCharCode('savo'); {  0x7361766f  }
	keyAEScale = FourCharCode('pscl'); {  0x7073636c  }
	keyAEScriptTag = FourCharCode('psct'); {  0x70736374  }
	keyAESearchText = FourCharCode('stxt'); {  0x73747874  }
	keyAEShowWhere = FourCharCode('show'); {  0x73686f77  }
	keyAEStartAngle = FourCharCode('pang'); {  0x70616e67  }
	keyAEStartPoint = FourCharCode('pstp'); {  0x70737470  }
	keyAEStyles = FourCharCode('ksty'); {  0x6b737479  }

const
	keyAESuiteID = FourCharCode('suit'); {  0x73756974  }
	keyAEText = FourCharCode('ktxt'); {  0x6b747874  }
	keyAETextColor = FourCharCode('ptxc'); {  0x70747863  }
	keyAETextFont = FourCharCode('ptxf'); {  0x70747866  }
	keyAETextPointSize = FourCharCode('ptps'); {  0x70747073  }
	keyAETextStyles = FourCharCode('txst'); {  0x74787374  }
	keyAETextLineHeight = FourCharCode('ktlh'); { type ShortInteger }
	keyAETextLineAscent = FourCharCode('ktas'); { type ShortInteger }
	keyAETheText = FourCharCode('thtx'); {  0x74687478  }
	keyAETransferMode = FourCharCode('pptm'); {  0x7070746d  }
	keyAETranslation = FourCharCode('ptrs'); {  0x70747273  }
	keyAETryAsStructGraf = FourCharCode('toog'); {  0x746f6f67  }
	keyAEUniformStyles = FourCharCode('ustl'); {  0x7573746c  }
	keyAEUpdateOn = FourCharCode('pupd'); {  0x70757064  }
	keyAEUserTerm = FourCharCode('utrm'); {  0x7574726d  }
	keyAEWindow = FourCharCode('wndw'); {  0x776e6477  }
	keyAEWritingCode = FourCharCode('wrcd'); {  0x77726364  }

const
	keyMiscellaneous = FourCharCode('fmsc'); {  0x666d7363  }
	keySelection = FourCharCode('fsel'); {  0x6673656c  }
	keyWindow = FourCharCode('kwnd'); {  0x6b776e64  }
                                        { EventRecord keys }
	keyWhen = FourCharCode('when');
	keyWhere = FourCharCode('wher');
	keyModifiers = FourCharCode('mods');
	keyKey = FourCharCode('key ');
	keyKeyCode = FourCharCode('code');
	keyKeyboard = FourCharCode('keyb');
	keyDriveNumber = FourCharCode('drv#');
	keyErrorCode = FourCharCode('err#');
	keyHighLevelClass = FourCharCode('hcls');
	keyHighLevelID = FourCharCode('hid ');

const
	pArcAngle = FourCharCode('parc'); {  0x70617263  }
	pBackgroundColor = FourCharCode('pbcl'); {  0x7062636c  }
	pBackgroundPattern = FourCharCode('pbpt'); {  0x70627074  }
	pBestType = FourCharCode('pbst'); {  0x70627374  }
	pBounds = FourCharCode('pbnd'); {  0x70626e64  }
	pClass = FourCharCode('pcls'); {  0x70636c73  }
	pClipboard = FourCharCode('pcli'); {  0x70636c69  }
	pColor = FourCharCode('colr'); {  0x636f6c72  }
	pColorTable = FourCharCode('cltb'); {  0x636c7462  }
	pContents = FourCharCode('pcnt'); {  0x70636e74  }
	pCornerCurveHeight = FourCharCode('pchd'); {  0x70636864  }
	pCornerCurveWidth = FourCharCode('pcwd'); {  0x70637764  }
	pDashStyle = FourCharCode('pdst'); {  0x70647374  }
	pDefaultType = FourCharCode('deft'); {  0x64656674  }
	pDefinitionRect = FourCharCode('pdrt'); {  0x70647274  }
	pEnabled = FourCharCode('enbl'); {  0x656e626c  }
	pEndPoint = FourCharCode('pend'); {  0x70656e64  }
	pFillColor = FourCharCode('flcl'); {  0x666c636c  }
	pFillPattern = FourCharCode('flpt'); {  0x666c7074  }
	pFont = FourCharCode('font'); {  0x666f6e74  }

const
	pFormula = FourCharCode('pfor'); {  0x70666f72  }
	pGraphicObjects = FourCharCode('gobs'); {  0x676f6273  }
	pHasCloseBox = FourCharCode('hclb'); {  0x68636c62  }
	pHasTitleBar = FourCharCode('ptit'); {  0x70746974  }
	pID = FourCharCode('ID  '); {  0x49442020  }
	pIndex = FourCharCode('pidx'); {  0x70696478  }
	pInsertionLoc = FourCharCode('pins'); {  0x70696e73  }
	pIsFloating = FourCharCode('isfl'); {  0x6973666c  }
	pIsFrontProcess = FourCharCode('pisf'); {  0x70697366  }
	pIsModal = FourCharCode('pmod'); {  0x706d6f64  }
	pIsModified = FourCharCode('imod'); {  0x696d6f64  }
	pIsResizable = FourCharCode('prsz'); {  0x7072737a  }
	pIsStationeryPad = FourCharCode('pspd'); {  0x70737064  }
	pIsZoomable = FourCharCode('iszm'); {  0x69737a6d  }
	pIsZoomed = FourCharCode('pzum'); {  0x707a756d  }
	pItemNumber = FourCharCode('itmn'); {  0x69746d6e  }
	pJustification = FourCharCode('pjst'); {  0x706a7374  }
	pLineArrow = FourCharCode('arro'); {  0x6172726f  }
	pMenuID = FourCharCode('mnid'); {  0x6d6e6964  }
	pName = FourCharCode('pnam'); {  0x706e616d  }

const
	pNewElementLoc = FourCharCode('pnel'); {  0x706e656c  }
	pPenColor = FourCharCode('ppcl'); {  0x7070636c  }
	pPenPattern = FourCharCode('pppa'); {  0x70707061  }
	pPenWidth = FourCharCode('ppwd'); {  0x70707764  }
	pPixelDepth = FourCharCode('pdpt'); {  0x70647074  }
	pPointList = FourCharCode('ptlt'); {  0x70746c74  }
	pPointSize = FourCharCode('ptsz'); {  0x7074737a  }
	pProtection = FourCharCode('ppro'); {  0x7070726f  }
	pRotation = FourCharCode('prot'); {  0x70726f74  }
	pScale = FourCharCode('pscl'); {  0x7073636c  }
	pScript = FourCharCode('scpt'); {  0x73637074  }
	pScriptTag = FourCharCode('psct'); {  0x70736374  }
	pSelected = FourCharCode('selc'); {  0x73656c63  }
	pSelection = FourCharCode('sele'); {  0x73656c65  }
	pStartAngle = FourCharCode('pang'); {  0x70616e67  }
	pStartPoint = FourCharCode('pstp'); {  0x70737470  }
	pTextColor = FourCharCode('ptxc'); {  0x70747863  }
	pTextFont = FourCharCode('ptxf'); {  0x70747866  }
	pTextItemDelimiters = FourCharCode('txdl'); {  0x7478646c  }
	pTextPointSize = FourCharCode('ptps'); {  0x70747073  }

const
	pTextStyles = FourCharCode('txst'); {  0x74787374  }
	pTransferMode = FourCharCode('pptm'); {  0x7070746d  }
	pTranslation = FourCharCode('ptrs'); {  0x70747273  }
	pUniformStyles = FourCharCode('ustl'); {  0x7573746c  }
	pUpdateOn = FourCharCode('pupd'); {  0x70757064  }
	pUserSelection = FourCharCode('pusl'); {  0x7075736c  }
	pVersion = FourCharCode('vers'); {  0x76657273  }
	pVisible = FourCharCode('pvis'); {  0x70766973  }

const
	typeAEText = FourCharCode('tTXT'); {  0x74545854  }
	typeArc = FourCharCode('carc'); {  0x63617263  }
	typeBest = FourCharCode('best'); {  0x62657374  }
	typeCell = FourCharCode('ccel'); {  0x6363656c  }
	typeClassInfo = FourCharCode('gcli'); {  0x67636c69  }
	typeColorTable = FourCharCode('clrt'); {  0x636c7274  }
	typeColumn = FourCharCode('ccol'); {  0x63636f6c  }
	typeDashStyle = FourCharCode('tdas'); {  0x74646173  }
	typeData = FourCharCode('tdta'); {  0x74647461  }
	typeDrawingArea = FourCharCode('cdrw'); {  0x63647277  }
	typeElemInfo = FourCharCode('elin'); {  0x656c696e  }
	typeEnumeration = FourCharCode('enum'); {  0x656e756d  }
	typeEPS = FourCharCode('EPS '); {  0x45505320  }
	typeEventInfo = FourCharCode('evin'); {  0x6576696e  }

const
	typeFinderWindow = FourCharCode('fwin'); {  0x6677696e  }
	typeFixedPoint = FourCharCode('fpnt'); {  0x66706e74  }
	typeFixedRectangle = FourCharCode('frct'); {  0x66726374  }
	typeGraphicLine = FourCharCode('glin'); {  0x676c696e  }
	typeGraphicText = FourCharCode('cgtx'); {  0x63677478  }
	typeGroupedGraphic = FourCharCode('cpic'); {  0x63706963  }
	typeInsertionLoc = FourCharCode('insl'); {  0x696e736c  }
	typeIntlText = FourCharCode('itxt'); {  0x69747874  }
	typeIntlWritingCode = FourCharCode('intl'); {  0x696e746c  }
	typeLongDateTime = FourCharCode('ldt '); {  0x6c647420  }
	typeCFAbsoluteTime = FourCharCode('cfat');
	typeISO8601DateTime = FourCharCode('isot'); {  0x69736f74  data is ascii text of an ISO8601 date }
	typeLongFixed = FourCharCode('lfxd'); {  0x6c667864  }
	typeLongFixedPoint = FourCharCode('lfpt'); {  0x6c667074  }
	typeLongFixedRectangle = FourCharCode('lfrc'); {  0x6c667263  }
	typeLongPoint = FourCharCode('lpnt'); {  0x6c706e74  }
	typeLongRectangle = FourCharCode('lrct'); {  0x6c726374  }
	typeMachineLoc = FourCharCode('mLoc'); {  0x6d4c6f63  }
	typeOval = FourCharCode('covl'); {  0x636f766c  }
	typeParamInfo = FourCharCode('pmin'); {  0x706d696e  }
	typePict = FourCharCode('PICT'); {  0x50494354  }

const
	typePixelMap = FourCharCode('cpix'); {  0x63706978  }
	typePixMapMinus = FourCharCode('tpmm'); {  0x74706d6d  }
	typePolygon = FourCharCode('cpgn'); {  0x6370676e  }
	typePropInfo = FourCharCode('pinf'); {  0x70696e66  }
	typePtr = FourCharCode('ptr '); {  0x70747220  }
	typeQDPoint = FourCharCode('QDpt'); {  0x51447074  }
	typeQDRegion = FourCharCode('Qrgn'); {  0x51447074  (data is actual region data, including rectangle and size, _not_ region handle or ptr)}
	typeRectangle = FourCharCode('crec'); {  0x63726563  }
	typeRGB16 = FourCharCode('tr16'); {  0x74723136  }
	typeRGB96 = FourCharCode('tr96'); {  0x74723936  }
	typeRGBColor = FourCharCode('cRGB'); {  0x63524742  }
	typeRotation = FourCharCode('trot'); {  0x74726f74  }
	typeRoundedRectangle = FourCharCode('crrc'); {  0x63727263  }
	typeRow = FourCharCode('crow'); {  0x63726f77  }
	typeScrapStyles = FourCharCode('styl'); {  0x7374796c  }
	typeScript = FourCharCode('scpt'); {  0x73637074  }
	typeStyledText = FourCharCode('STXT'); {  0x53545854  }
	typeSuiteInfo = FourCharCode('suin'); {  0x7375696e  }
	typeTable = FourCharCode('ctbl'); {  0x6374626c  }
	typeTextStyles = FourCharCode('tsty'); {  0x74737479  }

const
	typeTIFF = FourCharCode('TIFF'); {  0x54494646  }
	typeJPEG = FourCharCode('JPEG');
	typeGIF = FourCharCode('GIFf');
	typeVersion = FourCharCode('vers'); {  0x76657273  }

const
	kAEMenuClass = FourCharCode('menu');
	kAEMenuSelect = FourCharCode('mhit');
	kAEMouseDown = FourCharCode('mdwn');
	kAEMouseDownInBack = FourCharCode('mdbk');
	kAEKeyDown = FourCharCode('kdwn');
	kAEResized = FourCharCode('rsiz');
	kAEPromise = FourCharCode('prom');

const
	keyMenuID = FourCharCode('mid ');
	keyMenuItem = FourCharCode('mitm');
	keyCloseAllWindows = FourCharCode('caw ');
	keyOriginalBounds = FourCharCode('obnd');
	keyNewBounds = FourCharCode('nbnd');
	keyLocalWhere = FourCharCode('lwhr');

const
>>>>>>> graemeg/cpstrnew
=======

const
	kAENonmodifiable = FourCharCode('nmod'); {  0x6e6d6f64  }
	kAEOpen = FourCharCode('odoc'); {  0x6f646f63  }
	kAEOpenSelection = FourCharCode('sope'); {  0x736f7065  }
	kAEOutline = FourCharCode('outl'); {  0x6f75746c  }
	kAEPageSetup = FourCharCode('pgsu'); {  0x70677375  }
	kAEPaste = FourCharCode('past'); {  0x70617374  }
	kAEPlain = FourCharCode('plan'); {  0x706c616e  }
	kAEPrint = FourCharCode('pdoc'); {  0x70646f63  }
	kAEPrintSelection = FourCharCode('spri'); {  0x73707269  }
	kAEPrintWindow = FourCharCode('pwin'); {  0x7077696e  }
	kAEPutAwaySelection = FourCharCode('sput'); {  0x73707574  }
	kAEQDAddOver = FourCharCode('addo'); {  0x6164646f  }
	kAEQDAddPin = FourCharCode('addp'); {  0x61646470  }
	kAEQDAdMax = FourCharCode('admx'); {  0x61646d78  }
	kAEQDAdMin = FourCharCode('admn'); {  0x61646d6e  }
	kAEQDBic = FourCharCode('bic '); {  0x62696320  }
	kAEQDBlend = FourCharCode('blnd'); {  0x626c6e64  }
	kAEQDCopy = FourCharCode('cpy '); {  0x63707920  }
	kAEQDNotBic = FourCharCode('nbic'); {  0x6e626963  }
	kAEQDNotCopy = FourCharCode('ncpy'); {  0x6e637079  }

const
	kAEQDNotOr = FourCharCode('ntor'); {  0x6e746f72  }
	kAEQDNotXor = FourCharCode('nxor'); {  0x6e786f72  }
	kAEQDOr = FourCharCode('or  '); {  0x6f722020  }
	kAEQDSubOver = FourCharCode('subo'); {  0x7375626f  }
	kAEQDSubPin = FourCharCode('subp'); {  0x73756270  }
	kAEQDSupplementalSuite = FourCharCode('qdsp'); {  0x71647370  }
	kAEQDXor = FourCharCode('xor '); {  0x786f7220  }
	kAEQuickdrawSuite = FourCharCode('qdrw'); {  0x71647277  }
	kAEQuitAll = FourCharCode('quia'); {  0x71756961  }
	kAERedo = FourCharCode('redo'); {  0x7265646f  }
	kAERegular = FourCharCode('regl'); {  0x7265676c  }
	kAEReopenApplication = FourCharCode('rapp'); {  0x72617070  }
	kAEReplace = FourCharCode('rplc'); {  0x72706c63  }
	kAERequiredSuite = FourCharCode('reqd'); {  0x72657164  }
	kAERestart = FourCharCode('rest'); {  0x72657374  }
	kAERevealSelection = FourCharCode('srev'); {  0x73726576  }
	kAERevert = FourCharCode('rvrt'); {  0x72767274  }
	kAERightJustified = FourCharCode('rght'); {  0x72676874  }
	kAESave = FourCharCode('save'); {  0x73617665  }
	kAESelect = FourCharCode('slct'); {  0x736c6374  }
	kAESetData = FourCharCode('setd'); {  0x73657464  }

const
	kAESetPosition = FourCharCode('posn'); {  0x706f736e  }
	kAEShadow = FourCharCode('shad'); {  0x73686164  }
	kAEShowClipboard = FourCharCode('shcl'); {  0x7368636c  }
	kAEShutDown = FourCharCode('shut'); {  0x73687574  }
	kAESleep = FourCharCode('slep'); {  0x736c6570  }
	kAESmallCaps = FourCharCode('smcp'); {  0x736d6370  }
	kAESpecialClassProperties = FourCharCode('c@#!'); {  0x63402321  }
	kAEStrikethrough = FourCharCode('strk'); {  0x7374726b  }
	kAESubscript = FourCharCode('sbsc'); {  0x73627363  }
	kAESuperscript = FourCharCode('spsc'); {  0x73707363  }
	kAETableSuite = FourCharCode('tbls'); {  0x74626c73  }
	kAETextSuite = FourCharCode('TEXT'); {  0x54455854  }
	kAETransactionTerminated = FourCharCode('ttrm'); {  0x7474726d  }
	kAEUnderline = FourCharCode('undl'); {  0x756e646c  }
	kAEUndo = FourCharCode('undo'); {  0x756e646f  }
	kAEWholeWordEquals = FourCharCode('wweq'); {  0x77776571  }
	kAEYes = FourCharCode('yes '); {  0x79657320  }
	kAEZoom = FourCharCode('zoom'); {  0x7a6f6f6d  }

{ events that can be sent to the "system" process (eg, loginwindow) on OS X 10.2 or later }
const
	kAELogOut = FourCharCode('logo');
	kAEReallyLogOut = FourCharCode('rlgo');
	kAEShowRestartDialog = FourCharCode('rrst');
	kAEShowShutdownDialog = FourCharCode('rsdn');

{ EventRecord Classes and EventIDs }
const
	kAEMouseClass = FourCharCode('mous');
	kAEDown = FourCharCode('down');
	kAEUp = FourCharCode('up  ');
	kAEMoved = FourCharCode('move');
	kAEStoppedMoving = FourCharCode('stop');
	kAEWindowClass = FourCharCode('wind');
	kAEUpdate = FourCharCode('updt');
	kAEActivate = FourCharCode('actv');
	kAEDeactivate = FourCharCode('dact');
	kAECommandClass = FourCharCode('cmnd'); { Modern Command Event Class }
	kAEKeyClass = FourCharCode('keyc');
	kAERawKey = FourCharCode('rkey'); { Modern Raw Key Event }
	kAEVirtualKey = FourCharCode('keyc'); { Modern Virtual Key Event }
	kAENavigationKey = FourCharCode('nave'); { Modern Navigation Key Event }
	kAEAutoDown = FourCharCode('auto');
	kAEApplicationClass = FourCharCode('appl');
	kAESuspend = FourCharCode('susp');
	kAEResume = FourCharCode('rsme');
	kAEDiskEvent = FourCharCode('disk');
	kAENullEvent = FourCharCode('null');
	kAEWakeUpEvent = FourCharCode('wake');
	kAEScrapEvent = FourCharCode('scrp');
	kAEHighLevel = FourCharCode('high');

const
	keyAEAngle = FourCharCode('kang'); {  0x6b616e67  }
	keyAEArcAngle = FourCharCode('parc'); {  0x70617263  }

const
	keyAEBaseAddr = FourCharCode('badd'); {  0x62616464  }
	keyAEBestType = FourCharCode('pbst'); {  0x70627374  }
	keyAEBgndColor = FourCharCode('kbcl'); {  0x6b62636c  }
	keyAEBgndPattern = FourCharCode('kbpt'); {  0x6b627074  }
	keyAEBounds = FourCharCode('pbnd'); {  0x70626e64  }
	keyAECellList = FourCharCode('kclt'); {  0x6b636c74  }
	keyAEClassID = FourCharCode('clID'); {  0x636c4944  }
	keyAEColor = FourCharCode('colr'); {  0x636f6c72  }
	keyAEColorTable = FourCharCode('cltb'); {  0x636c7462  }
	keyAECurveHeight = FourCharCode('kchd'); {  0x6b636864  }
	keyAECurveWidth = FourCharCode('kcwd'); {  0x6b637764  }
	keyAEDashStyle = FourCharCode('pdst'); {  0x70647374  }
	keyAEData = FourCharCode('data'); {  0x64617461  }
	keyAEDefaultType = FourCharCode('deft'); {  0x64656674  }
	keyAEDefinitionRect = FourCharCode('pdrt'); {  0x70647274  }
	keyAEDescType = FourCharCode('dstp'); {  0x64737470  }
	keyAEDestination = FourCharCode('dest'); {  0x64657374  }
	keyAEDoAntiAlias = FourCharCode('anta'); {  0x616e7461  }
	keyAEDoDithered = FourCharCode('gdit'); {  0x67646974  }
	keyAEDoRotate = FourCharCode('kdrt'); {  0x6b647274  }

const
	keyAEDoScale = FourCharCode('ksca'); {  0x6b736361  }
	keyAEDoTranslate = FourCharCode('ktra'); {  0x6b747261  }
	keyAEEditionFileLoc = FourCharCode('eloc'); {  0x656c6f63  }
	keyAEElements = FourCharCode('elms'); {  0x656c6d73  }
	keyAEEndPoint = FourCharCode('pend'); {  0x70656e64  }
	keyAEEventClass = FourCharCode('evcl'); {  0x6576636c  }
	keyAEEventID = FourCharCode('evti'); {  0x65767469  }
	keyAEFile = FourCharCode('kfil'); {  0x6b66696c  }
	keyAEFileType = FourCharCode('fltp'); {  0x666c7470  }
	keyAEFillColor = FourCharCode('flcl'); {  0x666c636c  }
	keyAEFillPattern = FourCharCode('flpt'); {  0x666c7074  }
	keyAEFlipHorizontal = FourCharCode('kfho'); {  0x6b66686f  }
	keyAEFlipVertical = FourCharCode('kfvt'); {  0x6b667674  }
	keyAEFont = FourCharCode('font'); {  0x666f6e74  }
	keyAEFormula = FourCharCode('pfor'); {  0x70666f72  }
	keyAEGraphicObjects = FourCharCode('gobs'); {  0x676f6273  }
	keyAEID = FourCharCode('ID  '); {  0x49442020  }
	keyAEImageQuality = FourCharCode('gqua'); {  0x67717561  }
	keyAEInsertHere = FourCharCode('insh'); {  0x696e7368  }
	keyAEKeyForms = FourCharCode('keyf'); {  0x6b657966  }

const
	keyAEKeyword = FourCharCode('kywd'); {  0x6b797764  }
	keyAELevel = FourCharCode('levl'); {  0x6c65766c  }
	keyAELineArrow = FourCharCode('arro'); {  0x6172726f  }
	keyAEName = FourCharCode('pnam'); {  0x706e616d  }
	keyAENewElementLoc = FourCharCode('pnel'); {  0x706e656c  }
	keyAEObject = FourCharCode('kobj'); {  0x6b6f626a  }
	keyAEObjectClass = FourCharCode('kocl'); {  0x6b6f636c  }
	keyAEOffStyles = FourCharCode('ofst'); {  0x6f667374  }
	keyAEOnStyles = FourCharCode('onst'); {  0x6f6e7374  }
	keyAEParameters = FourCharCode('prms'); {  0x70726d73  }
	keyAEParamFlags = FourCharCode('pmfg'); {  0x706d6667  }
	keyAEPenColor = FourCharCode('ppcl'); {  0x7070636c  }
	keyAEPenPattern = FourCharCode('pppa'); {  0x70707061  }
	keyAEPenWidth = FourCharCode('ppwd'); {  0x70707764  }
	keyAEPixelDepth = FourCharCode('pdpt'); {  0x70647074  }
	keyAEPixMapMinus = FourCharCode('kpmm'); {  0x6b706d6d  }
	keyAEPMTable = FourCharCode('kpmt'); {  0x6b706d74  }
	keyAEPointList = FourCharCode('ptlt'); {  0x70746c74  }
	keyAEPointSize = FourCharCode('ptsz'); {  0x7074737a  }
	keyAEPosition = FourCharCode('kpos'); {  0x6b706f73  }

const
	keyAEPropData = FourCharCode('prdt'); {  0x70726474  }
	keyAEProperties = FourCharCode('qpro'); {  0x7170726f  }
	keyAEProperty = FourCharCode('kprp'); {  0x6b707270  }
	keyAEPropFlags = FourCharCode('prfg'); {  0x70726667  }
	keyAEPropID = FourCharCode('prop'); {  0x70726f70  }
	keyAEProtection = FourCharCode('ppro'); {  0x7070726f  }
	keyAERenderAs = FourCharCode('kren'); {  0x6b72656e  }
	keyAERequestedType = FourCharCode('rtyp'); {  0x72747970  }
	keyAEResult = FourCharCode('----'); {  0x2d2d2d2d  }
	keyAEResultInfo = FourCharCode('rsin'); {  0x7273696e  }
	keyAERotation = FourCharCode('prot'); {  0x70726f74  }
	keyAERotPoint = FourCharCode('krtp'); {  0x6b727470  }
	keyAERowList = FourCharCode('krls'); {  0x6b726c73  }
	keyAESaveOptions = FourCharCode('savo'); {  0x7361766f  }
	keyAEScale = FourCharCode('pscl'); {  0x7073636c  }
	keyAEScriptTag = FourCharCode('psct'); {  0x70736374  }
	keyAESearchText = FourCharCode('stxt'); {  0x73747874  }
	keyAEShowWhere = FourCharCode('show'); {  0x73686f77  }
	keyAEStartAngle = FourCharCode('pang'); {  0x70616e67  }
	keyAEStartPoint = FourCharCode('pstp'); {  0x70737470  }
	keyAEStyles = FourCharCode('ksty'); {  0x6b737479  }

const
	keyAESuiteID = FourCharCode('suit'); {  0x73756974  }
	keyAEText = FourCharCode('ktxt'); {  0x6b747874  }
	keyAETextColor = FourCharCode('ptxc'); {  0x70747863  }
	keyAETextFont = FourCharCode('ptxf'); {  0x70747866  }
	keyAETextPointSize = FourCharCode('ptps'); {  0x70747073  }
	keyAETextStyles = FourCharCode('txst'); {  0x74787374  }
	keyAETextLineHeight = FourCharCode('ktlh'); { type ShortInteger }
	keyAETextLineAscent = FourCharCode('ktas'); { type ShortInteger }
	keyAETheText = FourCharCode('thtx'); {  0x74687478  }
	keyAETransferMode = FourCharCode('pptm'); {  0x7070746d  }
	keyAETranslation = FourCharCode('ptrs'); {  0x70747273  }
	keyAETryAsStructGraf = FourCharCode('toog'); {  0x746f6f67  }
	keyAEUniformStyles = FourCharCode('ustl'); {  0x7573746c  }
	keyAEUpdateOn = FourCharCode('pupd'); {  0x70757064  }
	keyAEUserTerm = FourCharCode('utrm'); {  0x7574726d  }
	keyAEWindow = FourCharCode('wndw'); {  0x776e6477  }
	keyAEWritingCode = FourCharCode('wrcd'); {  0x77726364  }

const
	keyMiscellaneous = FourCharCode('fmsc'); {  0x666d7363  }
	keySelection = FourCharCode('fsel'); {  0x6673656c  }
	keyWindow = FourCharCode('kwnd'); {  0x6b776e64  }
                                        { EventRecord keys }
	keyWhen = FourCharCode('when');
	keyWhere = FourCharCode('wher');
	keyModifiers = FourCharCode('mods');
	keyKey = FourCharCode('key ');
	keyKeyCode = FourCharCode('code');
	keyKeyboard = FourCharCode('keyb');
	keyDriveNumber = FourCharCode('drv#');
	keyErrorCode = FourCharCode('err#');
	keyHighLevelClass = FourCharCode('hcls');
	keyHighLevelID = FourCharCode('hid ');

const
	pArcAngle = FourCharCode('parc'); {  0x70617263  }
	pBackgroundColor = FourCharCode('pbcl'); {  0x7062636c  }
	pBackgroundPattern = FourCharCode('pbpt'); {  0x70627074  }
	pBestType = FourCharCode('pbst'); {  0x70627374  }
	pBounds = FourCharCode('pbnd'); {  0x70626e64  }
	pClass = FourCharCode('pcls'); {  0x70636c73  }
	pClipboard = FourCharCode('pcli'); {  0x70636c69  }
	pColor = FourCharCode('colr'); {  0x636f6c72  }
	pColorTable = FourCharCode('cltb'); {  0x636c7462  }
	pContents = FourCharCode('pcnt'); {  0x70636e74  }
	pCornerCurveHeight = FourCharCode('pchd'); {  0x70636864  }
	pCornerCurveWidth = FourCharCode('pcwd'); {  0x70637764  }
	pDashStyle = FourCharCode('pdst'); {  0x70647374  }
	pDefaultType = FourCharCode('deft'); {  0x64656674  }
	pDefinitionRect = FourCharCode('pdrt'); {  0x70647274  }
	pEnabled = FourCharCode('enbl'); {  0x656e626c  }
	pEndPoint = FourCharCode('pend'); {  0x70656e64  }
	pFillColor = FourCharCode('flcl'); {  0x666c636c  }
	pFillPattern = FourCharCode('flpt'); {  0x666c7074  }
	pFont = FourCharCode('font'); {  0x666f6e74  }

const
	pFormula = FourCharCode('pfor'); {  0x70666f72  }
	pGraphicObjects = FourCharCode('gobs'); {  0x676f6273  }
	pHasCloseBox = FourCharCode('hclb'); {  0x68636c62  }
	pHasTitleBar = FourCharCode('ptit'); {  0x70746974  }
	pID = FourCharCode('ID  '); {  0x49442020  }
	pIndex = FourCharCode('pidx'); {  0x70696478  }
	pInsertionLoc = FourCharCode('pins'); {  0x70696e73  }
	pIsFloating = FourCharCode('isfl'); {  0x6973666c  }
	pIsFrontProcess = FourCharCode('pisf'); {  0x70697366  }
	pIsModal = FourCharCode('pmod'); {  0x706d6f64  }
	pIsModified = FourCharCode('imod'); {  0x696d6f64  }
	pIsResizable = FourCharCode('prsz'); {  0x7072737a  }
	pIsStationeryPad = FourCharCode('pspd'); {  0x70737064  }
	pIsZoomable = FourCharCode('iszm'); {  0x69737a6d  }
	pIsZoomed = FourCharCode('pzum'); {  0x707a756d  }
	pItemNumber = FourCharCode('itmn'); {  0x69746d6e  }
	pJustification = FourCharCode('pjst'); {  0x706a7374  }
	pLineArrow = FourCharCode('arro'); {  0x6172726f  }
	pMenuID = FourCharCode('mnid'); {  0x6d6e6964  }
	pName = FourCharCode('pnam'); {  0x706e616d  }

const
	pNewElementLoc = FourCharCode('pnel'); {  0x706e656c  }
	pPenColor = FourCharCode('ppcl'); {  0x7070636c  }
	pPenPattern = FourCharCode('pppa'); {  0x70707061  }
	pPenWidth = FourCharCode('ppwd'); {  0x70707764  }
	pPixelDepth = FourCharCode('pdpt'); {  0x70647074  }
	pPointList = FourCharCode('ptlt'); {  0x70746c74  }
	pPointSize = FourCharCode('ptsz'); {  0x7074737a  }
	pProtection = FourCharCode('ppro'); {  0x7070726f  }
	pRotation = FourCharCode('prot'); {  0x70726f74  }
	pScale = FourCharCode('pscl'); {  0x7073636c  }
	pScript = FourCharCode('scpt'); {  0x73637074  }
	pScriptTag = FourCharCode('psct'); {  0x70736374  }
	pSelected = FourCharCode('selc'); {  0x73656c63  }
	pSelection = FourCharCode('sele'); {  0x73656c65  }
	pStartAngle = FourCharCode('pang'); {  0x70616e67  }
	pStartPoint = FourCharCode('pstp'); {  0x70737470  }
	pTextColor = FourCharCode('ptxc'); {  0x70747863  }
	pTextFont = FourCharCode('ptxf'); {  0x70747866  }
	pTextItemDelimiters = FourCharCode('txdl'); {  0x7478646c  }
	pTextPointSize = FourCharCode('ptps'); {  0x70747073  }

const
	pTextStyles = FourCharCode('txst'); {  0x74787374  }
	pTransferMode = FourCharCode('pptm'); {  0x7070746d  }
	pTranslation = FourCharCode('ptrs'); {  0x70747273  }
	pUniformStyles = FourCharCode('ustl'); {  0x7573746c  }
	pUpdateOn = FourCharCode('pupd'); {  0x70757064  }
	pUserSelection = FourCharCode('pusl'); {  0x7075736c  }
	pVersion = FourCharCode('vers'); {  0x76657273  }
	pVisible = FourCharCode('pvis'); {  0x70766973  }

const
	typeAEText = FourCharCode('tTXT'); {  0x74545854  }
	typeArc = FourCharCode('carc'); {  0x63617263  }
	typeBest = FourCharCode('best'); {  0x62657374  }
	typeCell = FourCharCode('ccel'); {  0x6363656c  }
	typeClassInfo = FourCharCode('gcli'); {  0x67636c69  }
	typeColorTable = FourCharCode('clrt'); {  0x636c7274  }
	typeColumn = FourCharCode('ccol'); {  0x63636f6c  }
	typeDashStyle = FourCharCode('tdas'); {  0x74646173  }
	typeData = FourCharCode('tdta'); {  0x74647461  }
	typeDrawingArea = FourCharCode('cdrw'); {  0x63647277  }
	typeElemInfo = FourCharCode('elin'); {  0x656c696e  }
	typeEnumeration = FourCharCode('enum'); {  0x656e756d  }
	typeEPS = FourCharCode('EPS '); {  0x45505320  }
	typeEventInfo = FourCharCode('evin'); {  0x6576696e  }

const
	typeFinderWindow = FourCharCode('fwin'); {  0x6677696e  }
	typeFixedPoint = FourCharCode('fpnt'); {  0x66706e74  }
	typeFixedRectangle = FourCharCode('frct'); {  0x66726374  }
	typeGraphicLine = FourCharCode('glin'); {  0x676c696e  }
	typeGraphicText = FourCharCode('cgtx'); {  0x63677478  }
	typeGroupedGraphic = FourCharCode('cpic'); {  0x63706963  }
	typeInsertionLoc = FourCharCode('insl'); {  0x696e736c  }
	typeIntlText = FourCharCode('itxt'); {  0x69747874  }
	typeIntlWritingCode = FourCharCode('intl'); {  0x696e746c  }
	typeLongDateTime = FourCharCode('ldt '); {  0x6c647420  }
	typeCFAbsoluteTime = FourCharCode('cfat');
	typeISO8601DateTime = FourCharCode('isot'); {  0x69736f74  data is ascii text of an ISO8601 date }
	typeLongFixed = FourCharCode('lfxd'); {  0x6c667864  }
	typeLongFixedPoint = FourCharCode('lfpt'); {  0x6c667074  }
	typeLongFixedRectangle = FourCharCode('lfrc'); {  0x6c667263  }
	typeLongPoint = FourCharCode('lpnt'); {  0x6c706e74  }
	typeLongRectangle = FourCharCode('lrct'); {  0x6c726374  }
	typeMachineLoc = FourCharCode('mLoc'); {  0x6d4c6f63  }
	typeOval = FourCharCode('covl'); {  0x636f766c  }
	typeParamInfo = FourCharCode('pmin'); {  0x706d696e  }
	typePict = FourCharCode('PICT'); {  0x50494354  }

const
	typePixelMap = FourCharCode('cpix'); {  0x63706978  }
	typePixMapMinus = FourCharCode('tpmm'); {  0x74706d6d  }
	typePolygon = FourCharCode('cpgn'); {  0x6370676e  }
	typePropInfo = FourCharCode('pinf'); {  0x70696e66  }
	typePtr = FourCharCode('ptr '); {  0x70747220  }
	typeQDPoint = FourCharCode('QDpt'); {  0x51447074  }
	typeQDRegion = FourCharCode('Qrgn'); {  0x51447074  (data is actual region data, including rectangle and size, _not_ region handle or ptr)}
	typeRectangle = FourCharCode('crec'); {  0x63726563  }
	typeRGB16 = FourCharCode('tr16'); {  0x74723136  }
	typeRGB96 = FourCharCode('tr96'); {  0x74723936  }
	typeRGBColor = FourCharCode('cRGB'); {  0x63524742  }
	typeRotation = FourCharCode('trot'); {  0x74726f74  }
	typeRoundedRectangle = FourCharCode('crrc'); {  0x63727263  }
	typeRow = FourCharCode('crow'); {  0x63726f77  }
	typeScrapStyles = FourCharCode('styl'); {  0x7374796c  }
	typeScript = FourCharCode('scpt'); {  0x73637074  }
	typeStyledText = FourCharCode('STXT'); {  0x53545854  }
	typeSuiteInfo = FourCharCode('suin'); {  0x7375696e  }
	typeTable = FourCharCode('ctbl'); {  0x6374626c  }
	typeTextStyles = FourCharCode('tsty'); {  0x74737479  }

const
	typeTIFF = FourCharCode('TIFF'); {  0x54494646  }
	typeJPEG = FourCharCode('JPEG');
	typeGIF = FourCharCode('GIFf');
	typeVersion = FourCharCode('vers'); {  0x76657273  }

const
	kAEMenuClass = FourCharCode('menu');
	kAEMenuSelect = FourCharCode('mhit');
	kAEMouseDown = FourCharCode('mdwn');
	kAEMouseDownInBack = FourCharCode('mdbk');
	kAEKeyDown = FourCharCode('kdwn');
	kAEResized = FourCharCode('rsiz');
	kAEPromise = FourCharCode('prom');

const
	keyMenuID = FourCharCode('mid ');
	keyMenuItem = FourCharCode('mitm');
	keyCloseAllWindows = FourCharCode('caw ');
	keyOriginalBounds = FourCharCode('obnd');
	keyNewBounds = FourCharCode('nbnd');
	keyLocalWhere = FourCharCode('lwhr');

const
>>>>>>> graemeg/cpstrnew
=======

const
	cParagraph = FourCharCode('cpar'); {  0x63706172  }
	cPICT = FourCharCode('PICT'); {  0x50494354  }
	cPixel = FourCharCode('cpxl'); {  0x6370786c  }
	cPixelMap = FourCharCode('cpix'); {  0x63706978  }
	cPolygon = FourCharCode('cpgn'); {  0x6370676e  }
	cProperty = FourCharCode('prop'); {  0x70726f70  }
	cQDPoint = FourCharCode('QDpt'); {  0x51447074  }
	cQDRectangle = FourCharCode('qdrt'); {  0x71647274  }
	cRectangle = FourCharCode('crec'); {  0x63726563  }
	cRGBColor = FourCharCode('cRGB'); {  0x63524742  }
	cRotation = FourCharCode('trot'); {  0x74726f74  }
	cRoundedRectangle = FourCharCode('crrc'); {  0x63727263  }
	cRow = FourCharCode('crow'); {  0x63726f77  }
	cSelection = FourCharCode('csel'); {  0x6373656c  }
	cShortInteger = FourCharCode('shor'); {  0x73686f72  }
	cTable = FourCharCode('ctbl'); {  0x6374626c  }
	cText = FourCharCode('ctxt'); {  0x63747874  }
	cTextFlow = FourCharCode('cflo'); {  0x63666c6f  }
	cTextStyles = FourCharCode('tsty'); {  0x74737479  }
	cType = FourCharCode('type'); {  0x74797065  }

const
	cVersion = FourCharCode('vers'); {  0x76657273  }
	cWindow = FourCharCode('cwin'); {  0x6377696e  }
	cWord = FourCharCode('cwor'); {  0x63776f72  }
	enumArrows = FourCharCode('arro'); {  0x6172726f  }
	enumJustification = FourCharCode('just'); {  0x6a757374  }
	enumKeyForm = FourCharCode('kfrm'); {  0x6b66726d  }
	enumPosition = FourCharCode('posi'); {  0x706f7369  }
	enumProtection = FourCharCode('prtn'); {  0x7072746e  }
	enumQuality = FourCharCode('qual'); {  0x7175616c  }
	enumSaveOptions = FourCharCode('savo'); {  0x7361766f  }
	enumStyle = FourCharCode('styl'); {  0x7374796c  }
	enumTransferMode = FourCharCode('tran'); {  0x7472616e  }
	kAEAbout = FourCharCode('abou'); {  0x61626f75  }
	kAEAfter = FourCharCode('afte'); {  0x61667465  }
	kAEAliasSelection = FourCharCode('sali'); {  0x73616c69  }
	kAEAllCaps = FourCharCode('alcp'); {  0x616c6370  }
	kAEArrowAtEnd = FourCharCode('aren'); {  0x6172656e  }
	kAEArrowAtStart = FourCharCode('arst'); {  0x61727374  }
	kAEArrowBothEnds = FourCharCode('arbo'); {  0x6172626f  }

const
	kAEAsk = FourCharCode('ask '); {  0x61736b20  }
	kAEBefore = FourCharCode('befo'); {  0x6265666f  }
	kAEBeginning = FourCharCode('bgng'); {  0x62676e67  }
	kAEBeginsWith = FourCharCode('bgwt'); {  0x62677774  }
	kAEBeginTransaction = FourCharCode('begi'); {  0x62656769  }
	kAEBold = FourCharCode('bold'); {  0x626f6c64  }
	kAECaseSensEquals = FourCharCode('cseq'); {  0x63736571  }
	kAECentered = FourCharCode('cent'); {  0x63656e74  }
	kAEChangeView = FourCharCode('view'); {  0x76696577  }
	kAEClone = FourCharCode('clon'); {  0x636c6f6e  }
	kAEClose = FourCharCode('clos'); {  0x636c6f73  }
	kAECondensed = FourCharCode('cond'); {  0x636f6e64  }
	kAEContains = FourCharCode('cont'); {  0x636f6e74  }
	kAECopy = FourCharCode('copy'); {  0x636f7079  }
	kAECoreSuite = FourCharCode('core'); {  0x636f7265  }
	kAECountElements = FourCharCode('cnte'); {  0x636e7465  }
	kAECreateElement = FourCharCode('crel'); {  0x6372656c  }
	kAECreatePublisher = FourCharCode('cpub'); {  0x63707562  }
	kAECut = FourCharCode('cut '); {  0x63757420  }
	kAEDelete = FourCharCode('delo'); {  0x64656c6f  }

const
	kAEDoObjectsExist = FourCharCode('doex'); {  0x646f6578  }
	kAEDoScript = FourCharCode('dosc'); {  0x646f7363  }
	kAEDrag = FourCharCode('drag'); {  0x64726167  }
	kAEDuplicateSelection = FourCharCode('sdup'); {  0x73647570  }
	kAEEditGraphic = FourCharCode('edit'); {  0x65646974  }
	kAEEmptyTrash = FourCharCode('empt'); {  0x656d7074  }
	kAEEnd = FourCharCode('end '); {  0x656e6420  }
	kAEEndsWith = FourCharCode('ends'); {  0x656e6473  }
	kAEEndTransaction = FourCharCode('endt'); {  0x656e6474  }
	kAEEquals = FourCharCode('=   '); {  0x3d202020  }
	kAEExpanded = FourCharCode('pexp'); {  0x70657870  }
	kAEFast = FourCharCode('fast'); {  0x66617374  }
	kAEFinderEvents = FourCharCode('FNDR'); {  0x464e4452  }
	kAEFormulaProtect = FourCharCode('fpro'); {  0x6670726f  }
	kAEFullyJustified = FourCharCode('full'); {  0x66756c6c  }
	kAEGetClassInfo = FourCharCode('qobj'); {  0x716f626a  }
	kAEGetData = FourCharCode('getd'); {  0x67657464  }
	kAEGetDataSize = FourCharCode('dsiz'); {  0x6473697a  }
	kAEGetEventInfo = FourCharCode('gtei'); {  0x67746569  }
	kAEGetInfoSelection = FourCharCode('sinf'); {  0x73696e66  }

const
	kAEGetPrivilegeSelection = FourCharCode('sprv'); {  0x73707276  }
	kAEGetSuiteInfo = FourCharCode('gtsi'); {  0x67747369  }
	kAEGreaterThan = FourCharCode('>   '); {  0x3e202020  }
	kAEGreaterThanEquals = FourCharCode('>=  '); {  0x3e3d2020  }
	kAEGrow = FourCharCode('grow'); {  0x67726f77  }
	kAEHidden = FourCharCode('hidn'); {  0x6869646e  }
	kAEHiQuality = FourCharCode('hiqu'); {  0x68697175  }
	kAEImageGraphic = FourCharCode('imgr'); {  0x696d6772  }
	kAEIsUniform = FourCharCode('isun'); {  0x6973756e  }
	kAEItalic = FourCharCode('ital'); {  0x6974616c  }
	kAELeftJustified = FourCharCode('left'); {  0x6c656674  }
	kAELessThan = FourCharCode('<   '); {  0x3c202020  }
	kAELessThanEquals = FourCharCode('<=  '); {  0x3c3d2020  }
	kAELowercase = FourCharCode('lowc'); {  0x6c6f7763  }
	kAEMakeObjectsVisible = FourCharCode('mvis'); {  0x6d766973  }
	kAEMiscStandards = FourCharCode('misc'); {  0x6d697363  }
	kAEModifiable = FourCharCode('modf'); {  0x6d6f6466  }
	kAEMove = FourCharCode('move'); {  0x6d6f7665  }
	kAENo = FourCharCode('no  '); {  0x6e6f2020  }
	kAENoArrow = FourCharCode('arno'); {  0x61726e6f  }

const
	kAENonmodifiable = FourCharCode('nmod'); {  0x6e6d6f64  }
	kAEOpen = FourCharCode('odoc'); {  0x6f646f63  }
	kAEOpenSelection = FourCharCode('sope'); {  0x736f7065  }
	kAEOutline = FourCharCode('outl'); {  0x6f75746c  }
	kAEPageSetup = FourCharCode('pgsu'); {  0x70677375  }
	kAEPaste = FourCharCode('past'); {  0x70617374  }
	kAEPlain = FourCharCode('plan'); {  0x706c616e  }
	kAEPrint = FourCharCode('pdoc'); {  0x70646f63  }
	kAEPrintSelection = FourCharCode('spri'); {  0x73707269  }
	kAEPrintWindow = FourCharCode('pwin'); {  0x7077696e  }
	kAEPutAwaySelection = FourCharCode('sput'); {  0x73707574  }
	kAEQDAddOver = FourCharCode('addo'); {  0x6164646f  }
	kAEQDAddPin = FourCharCode('addp'); {  0x61646470  }
	kAEQDAdMax = FourCharCode('admx'); {  0x61646d78  }
	kAEQDAdMin = FourCharCode('admn'); {  0x61646d6e  }
	kAEQDBic = FourCharCode('bic '); {  0x62696320  }
	kAEQDBlend = FourCharCode('blnd'); {  0x626c6e64  }
	kAEQDCopy = FourCharCode('cpy '); {  0x63707920  }
	kAEQDNotBic = FourCharCode('nbic'); {  0x6e626963  }
	kAEQDNotCopy = FourCharCode('ncpy'); {  0x6e637079  }

const
	kAEQDNotOr = FourCharCode('ntor'); {  0x6e746f72  }
	kAEQDNotXor = FourCharCode('nxor'); {  0x6e786f72  }
	kAEQDOr = FourCharCode('or  '); {  0x6f722020  }
	kAEQDSubOver = FourCharCode('subo'); {  0x7375626f  }
	kAEQDSubPin = FourCharCode('subp'); {  0x73756270  }
	kAEQDSupplementalSuite = FourCharCode('qdsp'); {  0x71647370  }
	kAEQDXor = FourCharCode('xor '); {  0x786f7220  }
	kAEQuickdrawSuite = FourCharCode('qdrw'); {  0x71647277  }
	kAEQuitAll = FourCharCode('quia'); {  0x71756961  }
	kAERedo = FourCharCode('redo'); {  0x7265646f  }
	kAERegular = FourCharCode('regl'); {  0x7265676c  }
	kAEReopenApplication = FourCharCode('rapp'); {  0x72617070  }
	kAEReplace = FourCharCode('rplc'); {  0x72706c63  }
	kAERequiredSuite = FourCharCode('reqd'); {  0x72657164  }
	kAERestart = FourCharCode('rest'); {  0x72657374  }
	kAERevealSelection = FourCharCode('srev'); {  0x73726576  }
	kAERevert = FourCharCode('rvrt'); {  0x72767274  }
	kAERightJustified = FourCharCode('rght'); {  0x72676874  }
	kAESave = FourCharCode('save'); {  0x73617665  }
	kAESelect = FourCharCode('slct'); {  0x736c6374  }
	kAESetData = FourCharCode('setd'); {  0x73657464  }

const
	kAESetPosition = FourCharCode('posn'); {  0x706f736e  }
	kAEShadow = FourCharCode('shad'); {  0x73686164  }
	kAEShowClipboard = FourCharCode('shcl'); {  0x7368636c  }
	kAEShutDown = FourCharCode('shut'); {  0x73687574  }
	kAESleep = FourCharCode('slep'); {  0x736c6570  }
	kAESmallCaps = FourCharCode('smcp'); {  0x736d6370  }
	kAESpecialClassProperties = FourCharCode('c@#!'); {  0x63402321  }
	kAEStrikethrough = FourCharCode('strk'); {  0x7374726b  }
	kAESubscript = FourCharCode('sbsc'); {  0x73627363  }
	kAESuperscript = FourCharCode('spsc'); {  0x73707363  }
	kAETableSuite = FourCharCode('tbls'); {  0x74626c73  }
	kAETextSuite = FourCharCode('TEXT'); {  0x54455854  }
	kAETransactionTerminated = FourCharCode('ttrm'); {  0x7474726d  }
	kAEUnderline = FourCharCode('undl'); {  0x756e646c  }
	kAEUndo = FourCharCode('undo'); {  0x756e646f  }
	kAEWholeWordEquals = FourCharCode('wweq'); {  0x77776571  }
	kAEYes = FourCharCode('yes '); {  0x79657320  }
	kAEZoom = FourCharCode('zoom'); {  0x7a6f6f6d  }

{ events that can be sent to the "system" process (eg, loginwindow) on OS X 10.2 or later }
const
	kAELogOut = FourCharCode('logo');
	kAEReallyLogOut = FourCharCode('rlgo');
	kAEShowRestartDialog = FourCharCode('rrst');
	kAEShowShutdownDialog = FourCharCode('rsdn');

{ EventRecord Classes and EventIDs }
const
	kAEMouseClass = FourCharCode('mous');
	kAEDown = FourCharCode('down');
	kAEUp = FourCharCode('up  ');
	kAEMoved = FourCharCode('move');
	kAEStoppedMoving = FourCharCode('stop');
	kAEWindowClass = FourCharCode('wind');
	kAEUpdate = FourCharCode('updt');
	kAEActivate = FourCharCode('actv');
	kAEDeactivate = FourCharCode('dact');
	kAECommandClass = FourCharCode('cmnd'); { Modern Command Event Class }
	kAEKeyClass = FourCharCode('keyc');
	kAERawKey = FourCharCode('rkey'); { Modern Raw Key Event }
	kAEVirtualKey = FourCharCode('keyc'); { Modern Virtual Key Event }
	kAENavigationKey = FourCharCode('nave'); { Modern Navigation Key Event }
	kAEAutoDown = FourCharCode('auto');
	kAEApplicationClass = FourCharCode('appl');
	kAESuspend = FourCharCode('susp');
	kAEResume = FourCharCode('rsme');
	kAEDiskEvent = FourCharCode('disk');
	kAENullEvent = FourCharCode('null');
	kAEWakeUpEvent = FourCharCode('wake');
	kAEScrapEvent = FourCharCode('scrp');
	kAEHighLevel = FourCharCode('high');

const
	keyAEAngle = FourCharCode('kang'); {  0x6b616e67  }
	keyAEArcAngle = FourCharCode('parc'); {  0x70617263  }

const
	keyAEBaseAddr = FourCharCode('badd'); {  0x62616464  }
	keyAEBestType = FourCharCode('pbst'); {  0x70627374  }
	keyAEBgndColor = FourCharCode('kbcl'); {  0x6b62636c  }
	keyAEBgndPattern = FourCharCode('kbpt'); {  0x6b627074  }
	keyAEBounds = FourCharCode('pbnd'); {  0x70626e64  }
	keyAECellList = FourCharCode('kclt'); {  0x6b636c74  }
	keyAEClassID = FourCharCode('clID'); {  0x636c4944  }
	keyAEColor = FourCharCode('colr'); {  0x636f6c72  }
	keyAEColorTable = FourCharCode('cltb'); {  0x636c7462  }
	keyAECurveHeight = FourCharCode('kchd'); {  0x6b636864  }
	keyAECurveWidth = FourCharCode('kcwd'); {  0x6b637764  }
	keyAEDashStyle = FourCharCode('pdst'); {  0x70647374  }
	keyAEData = FourCharCode('data'); {  0x64617461  }
	keyAEDefaultType = FourCharCode('deft'); {  0x64656674  }
	keyAEDefinitionRect = FourCharCode('pdrt'); {  0x70647274  }
	keyAEDescType = FourCharCode('dstp'); {  0x64737470  }
	keyAEDestination = FourCharCode('dest'); {  0x64657374  }
	keyAEDoAntiAlias = FourCharCode('anta'); {  0x616e7461  }
	keyAEDoDithered = FourCharCode('gdit'); {  0x67646974  }
	keyAEDoRotate = FourCharCode('kdrt'); {  0x6b647274  }

const
	keyAEDoScale = FourCharCode('ksca'); {  0x6b736361  }
	keyAEDoTranslate = FourCharCode('ktra'); {  0x6b747261  }
	keyAEEditionFileLoc = FourCharCode('eloc'); {  0x656c6f63  }
	keyAEElements = FourCharCode('elms'); {  0x656c6d73  }
	keyAEEndPoint = FourCharCode('pend'); {  0x70656e64  }
	keyAEEventClass = FourCharCode('evcl'); {  0x6576636c  }
	keyAEEventID = FourCharCode('evti'); {  0x65767469  }
	keyAEFile = FourCharCode('kfil'); {  0x6b66696c  }
	keyAEFileType = FourCharCode('fltp'); {  0x666c7470  }
	keyAEFillColor = FourCharCode('flcl'); {  0x666c636c  }
	keyAEFillPattern = FourCharCode('flpt'); {  0x666c7074  }
	keyAEFlipHorizontal = FourCharCode('kfho'); {  0x6b66686f  }
	keyAEFlipVertical = FourCharCode('kfvt'); {  0x6b667674  }
	keyAEFont = FourCharCode('font'); {  0x666f6e74  }
	keyAEFormula = FourCharCode('pfor'); {  0x70666f72  }
	keyAEGraphicObjects = FourCharCode('gobs'); {  0x676f6273  }
	keyAEID = FourCharCode('ID  '); {  0x49442020  }
	keyAEImageQuality = FourCharCode('gqua'); {  0x67717561  }
	keyAEInsertHere = FourCharCode('insh'); {  0x696e7368  }
	keyAEKeyForms = FourCharCode('keyf'); {  0x6b657966  }

const
	keyAEKeyword = FourCharCode('kywd'); {  0x6b797764  }
	keyAELevel = FourCharCode('levl'); {  0x6c65766c  }
	keyAELineArrow = FourCharCode('arro'); {  0x6172726f  }
	keyAEName = FourCharCode('pnam'); {  0x706e616d  }
	keyAENewElementLoc = FourCharCode('pnel'); {  0x706e656c  }
	keyAEObject = FourCharCode('kobj'); {  0x6b6f626a  }
	keyAEObjectClass = FourCharCode('kocl'); {  0x6b6f636c  }
	keyAEOffStyles = FourCharCode('ofst'); {  0x6f667374  }
	keyAEOnStyles = FourCharCode('onst'); {  0x6f6e7374  }
	keyAEParameters = FourCharCode('prms'); {  0x70726d73  }
	keyAEParamFlags = FourCharCode('pmfg'); {  0x706d6667  }
	keyAEPenColor = FourCharCode('ppcl'); {  0x7070636c  }
	keyAEPenPattern = FourCharCode('pppa'); {  0x70707061  }
	keyAEPenWidth = FourCharCode('ppwd'); {  0x70707764  }
	keyAEPixelDepth = FourCharCode('pdpt'); {  0x70647074  }
	keyAEPixMapMinus = FourCharCode('kpmm'); {  0x6b706d6d  }
	keyAEPMTable = FourCharCode('kpmt'); {  0x6b706d74  }
	keyAEPointList = FourCharCode('ptlt'); {  0x70746c74  }
	keyAEPointSize = FourCharCode('ptsz'); {  0x7074737a  }
	keyAEPosition = FourCharCode('kpos'); {  0x6b706f73  }

const
	keyAEPropData = FourCharCode('prdt'); {  0x70726474  }
	keyAEProperties = FourCharCode('qpro'); {  0x7170726f  }
	keyAEProperty = FourCharCode('kprp'); {  0x6b707270  }
	keyAEPropFlags = FourCharCode('prfg'); {  0x70726667  }
	keyAEPropID = FourCharCode('prop'); {  0x70726f70  }
	keyAEProtection = FourCharCode('ppro'); {  0x7070726f  }
	keyAERenderAs = FourCharCode('kren'); {  0x6b72656e  }
	keyAERequestedType = FourCharCode('rtyp'); {  0x72747970  }
	keyAEResult = FourCharCode('----'); {  0x2d2d2d2d  }
	keyAEResultInfo = FourCharCode('rsin'); {  0x7273696e  }
	keyAERotation = FourCharCode('prot'); {  0x70726f74  }
	keyAERotPoint = FourCharCode('krtp'); {  0x6b727470  }
	keyAERowList = FourCharCode('krls'); {  0x6b726c73  }
	keyAESaveOptions = FourCharCode('savo'); {  0x7361766f  }
	keyAEScale = FourCharCode('pscl'); {  0x7073636c  }
	keyAEScriptTag = FourCharCode('psct'); {  0x70736374  }
	keyAESearchText = FourCharCode('stxt'); {  0x73747874  }
	keyAEShowWhere = FourCharCode('show'); {  0x73686f77  }
	keyAEStartAngle = FourCharCode('pang'); {  0x70616e67  }
	keyAEStartPoint = FourCharCode('pstp'); {  0x70737470  }
	keyAEStyles = FourCharCode('ksty'); {  0x6b737479  }

const
	keyAESuiteID = FourCharCode('suit'); {  0x73756974  }
	keyAEText = FourCharCode('ktxt'); {  0x6b747874  }
	keyAETextColor = FourCharCode('ptxc'); {  0x70747863  }
	keyAETextFont = FourCharCode('ptxf'); {  0x70747866  }
	keyAETextPointSize = FourCharCode('ptps'); {  0x70747073  }
	keyAETextStyles = FourCharCode('txst'); {  0x74787374  }
	keyAETextLineHeight = FourCharCode('ktlh'); { type ShortInteger }
	keyAETextLineAscent = FourCharCode('ktas'); { type ShortInteger }
	keyAETheText = FourCharCode('thtx'); {  0x74687478  }
	keyAETransferMode = FourCharCode('pptm'); {  0x7070746d  }
	keyAETranslation = FourCharCode('ptrs'); {  0x70747273  }
	keyAETryAsStructGraf = FourCharCode('toog'); {  0x746f6f67  }
	keyAEUniformStyles = FourCharCode('ustl'); {  0x7573746c  }
	keyAEUpdateOn = FourCharCode('pupd'); {  0x70757064  }
	keyAEUserTerm = FourCharCode('utrm'); {  0x7574726d  }
	keyAEWindow = FourCharCode('wndw'); {  0x776e6477  }
	keyAEWritingCode = FourCharCode('wrcd'); {  0x77726364  }

const
	keyMiscellaneous = FourCharCode('fmsc'); {  0x666d7363  }
	keySelection = FourCharCode('fsel'); {  0x6673656c  }
	keyWindow = FourCharCode('kwnd'); {  0x6b776e64  }
                                        { EventRecord keys }
	keyWhen = FourCharCode('when');
	keyWhere = FourCharCode('wher');
	keyModifiers = FourCharCode('mods');
	keyKey = FourCharCode('key ');
	keyKeyCode = FourCharCode('code');
	keyKeyboard = FourCharCode('keyb');
	keyDriveNumber = FourCharCode('drv#');
	keyErrorCode = FourCharCode('err#');
	keyHighLevelClass = FourCharCode('hcls');
	keyHighLevelID = FourCharCode('hid ');

const
	pArcAngle = FourCharCode('parc'); {  0x70617263  }
	pBackgroundColor = FourCharCode('pbcl'); {  0x7062636c  }
	pBackgroundPattern = FourCharCode('pbpt'); {  0x70627074  }
	pBestType = FourCharCode('pbst'); {  0x70627374  }
	pBounds = FourCharCode('pbnd'); {  0x70626e64  }
	pClass = FourCharCode('pcls'); {  0x70636c73  }
	pClipboard = FourCharCode('pcli'); {  0x70636c69  }
	pColor = FourCharCode('colr'); {  0x636f6c72  }
	pColorTable = FourCharCode('cltb'); {  0x636c7462  }
	pContents = FourCharCode('pcnt'); {  0x70636e74  }
	pCornerCurveHeight = FourCharCode('pchd'); {  0x70636864  }
	pCornerCurveWidth = FourCharCode('pcwd'); {  0x70637764  }
	pDashStyle = FourCharCode('pdst'); {  0x70647374  }
	pDefaultType = FourCharCode('deft'); {  0x64656674  }
	pDefinitionRect = FourCharCode('pdrt'); {  0x70647274  }
	pEnabled = FourCharCode('enbl'); {  0x656e626c  }
	pEndPoint = FourCharCode('pend'); {  0x70656e64  }
	pFillColor = FourCharCode('flcl'); {  0x666c636c  }
	pFillPattern = FourCharCode('flpt'); {  0x666c7074  }
	pFont = FourCharCode('font'); {  0x666f6e74  }

const
	pFormula = FourCharCode('pfor'); {  0x70666f72  }
	pGraphicObjects = FourCharCode('gobs'); {  0x676f6273  }
	pHasCloseBox = FourCharCode('hclb'); {  0x68636c62  }
	pHasTitleBar = FourCharCode('ptit'); {  0x70746974  }
	pID = FourCharCode('ID  '); {  0x49442020  }
	pIndex = FourCharCode('pidx'); {  0x70696478  }
	pInsertionLoc = FourCharCode('pins'); {  0x70696e73  }
	pIsFloating = FourCharCode('isfl'); {  0x6973666c  }
	pIsFrontProcess = FourCharCode('pisf'); {  0x70697366  }
	pIsModal = FourCharCode('pmod'); {  0x706d6f64  }
	pIsModified = FourCharCode('imod'); {  0x696d6f64  }
	pIsResizable = FourCharCode('prsz'); {  0x7072737a  }
	pIsStationeryPad = FourCharCode('pspd'); {  0x70737064  }
	pIsZoomable = FourCharCode('iszm'); {  0x69737a6d  }
	pIsZoomed = FourCharCode('pzum'); {  0x707a756d  }
	pItemNumber = FourCharCode('itmn'); {  0x69746d6e  }
	pJustification = FourCharCode('pjst'); {  0x706a7374  }
	pLineArrow = FourCharCode('arro'); {  0x6172726f  }
	pMenuID = FourCharCode('mnid'); {  0x6d6e6964  }
	pName = FourCharCode('pnam'); {  0x706e616d  }

const
	pNewElementLoc = FourCharCode('pnel'); {  0x706e656c  }
	pPenColor = FourCharCode('ppcl'); {  0x7070636c  }
	pPenPattern = FourCharCode('pppa'); {  0x70707061  }
	pPenWidth = FourCharCode('ppwd'); {  0x70707764  }
	pPixelDepth = FourCharCode('pdpt'); {  0x70647074  }
	pPointList = FourCharCode('ptlt'); {  0x70746c74  }
	pPointSize = FourCharCode('ptsz'); {  0x7074737a  }
	pProtection = FourCharCode('ppro'); {  0x7070726f  }
	pRotation = FourCharCode('prot'); {  0x70726f74  }
	pScale = FourCharCode('pscl'); {  0x7073636c  }
	pScript = FourCharCode('scpt'); {  0x73637074  }
	pScriptTag = FourCharCode('psct'); {  0x70736374  }
	pSelected = FourCharCode('selc'); {  0x73656c63  }
	pSelection = FourCharCode('sele'); {  0x73656c65  }
	pStartAngle = FourCharCode('pang'); {  0x70616e67  }
	pStartPoint = FourCharCode('pstp'); {  0x70737470  }
	pTextColor = FourCharCode('ptxc'); {  0x70747863  }
	pTextFont = FourCharCode('ptxf'); {  0x70747866  }
	pTextItemDelimiters = FourCharCode('txdl'); {  0x7478646c  }
	pTextPointSize = FourCharCode('ptps'); {  0x70747073  }

const
	pTextStyles = FourCharCode('txst'); {  0x74787374  }
	pTransferMode = FourCharCode('pptm'); {  0x7070746d  }
	pTranslation = FourCharCode('ptrs'); {  0x70747273  }
	pUniformStyles = FourCharCode('ustl'); {  0x7573746c  }
	pUpdateOn = FourCharCode('pupd'); {  0x70757064  }
	pUserSelection = FourCharCode('pusl'); {  0x7075736c  }
	pVersion = FourCharCode('vers'); {  0x76657273  }
	pVisible = FourCharCode('pvis'); {  0x70766973  }

const
	typeAEText = FourCharCode('tTXT'); {  0x74545854  }
	typeArc = FourCharCode('carc'); {  0x63617263  }
	typeBest = FourCharCode('best'); {  0x62657374  }
	typeCell = FourCharCode('ccel'); {  0x6363656c  }
	typeClassInfo = FourCharCode('gcli'); {  0x67636c69  }
	typeColorTable = FourCharCode('clrt'); {  0x636c7274  }
	typeColumn = FourCharCode('ccol'); {  0x63636f6c  }
	typeDashStyle = FourCharCode('tdas'); {  0x74646173  }
	typeData = FourCharCode('tdta'); {  0x74647461  }
	typeDrawingArea = FourCharCode('cdrw'); {  0x63647277  }
	typeElemInfo = FourCharCode('elin'); {  0x656c696e  }
	typeEnumeration = FourCharCode('enum'); {  0x656e756d  }
	typeEPS = FourCharCode('EPS '); {  0x45505320  }
	typeEventInfo = FourCharCode('evin'); {  0x6576696e  }

const
	typeFinderWindow = FourCharCode('fwin'); {  0x6677696e  }
	typeFixedPoint = FourCharCode('fpnt'); {  0x66706e74  }
	typeFixedRectangle = FourCharCode('frct'); {  0x66726374  }
	typeGraphicLine = FourCharCode('glin'); {  0x676c696e  }
	typeGraphicText = FourCharCode('cgtx'); {  0x63677478  }
	typeGroupedGraphic = FourCharCode('cpic'); {  0x63706963  }
	typeInsertionLoc = FourCharCode('insl'); {  0x696e736c  }
	typeIntlText = FourCharCode('itxt'); {  0x69747874  }
	typeIntlWritingCode = FourCharCode('intl'); {  0x696e746c  }
	typeLongDateTime = FourCharCode('ldt '); {  0x6c647420  }
	typeCFAbsoluteTime = FourCharCode('cfat');
	typeISO8601DateTime = FourCharCode('isot'); {  0x69736f74  data is ascii text of an ISO8601 date }
	typeLongFixed = FourCharCode('lfxd'); {  0x6c667864  }
	typeLongFixedPoint = FourCharCode('lfpt'); {  0x6c667074  }
	typeLongFixedRectangle = FourCharCode('lfrc'); {  0x6c667263  }
	typeLongPoint = FourCharCode('lpnt'); {  0x6c706e74  }
	typeLongRectangle = FourCharCode('lrct'); {  0x6c726374  }
	typeMachineLoc = FourCharCode('mLoc'); {  0x6d4c6f63  }
	typeOval = FourCharCode('covl'); {  0x636f766c  }
	typeParamInfo = FourCharCode('pmin'); {  0x706d696e  }
	typePict = FourCharCode('PICT'); {  0x50494354  }

const
	typePixelMap = FourCharCode('cpix'); {  0x63706978  }
	typePixMapMinus = FourCharCode('tpmm'); {  0x74706d6d  }
	typePolygon = FourCharCode('cpgn'); {  0x6370676e  }
	typePropInfo = FourCharCode('pinf'); {  0x70696e66  }
	typePtr = FourCharCode('ptr '); {  0x70747220  }
	typeQDPoint = FourCharCode('QDpt'); {  0x51447074  }
	typeQDRegion = FourCharCode('Qrgn'); {  0x51447074  (data is actual region data, including rectangle and size, _not_ region handle or ptr)}
	typeRectangle = FourCharCode('crec'); {  0x63726563  }
	typeRGB16 = FourCharCode('tr16'); {  0x74723136  }
	typeRGB96 = FourCharCode('tr96'); {  0x74723936  }
	typeRGBColor = FourCharCode('cRGB'); {  0x63524742  }
	typeRotation = FourCharCode('trot'); {  0x74726f74  }
	typeRoundedRectangle = FourCharCode('crrc'); {  0x63727263  }
	typeRow = FourCharCode('crow'); {  0x63726f77  }
	typeScrapStyles = FourCharCode('styl'); {  0x7374796c  }
	typeScript = FourCharCode('scpt'); {  0x73637074  }
	typeStyledText = FourCharCode('STXT'); {  0x53545854  }
	typeSuiteInfo = FourCharCode('suin'); {  0x7375696e  }
	typeTable = FourCharCode('ctbl'); {  0x6374626c  }
	typeTextStyles = FourCharCode('tsty'); {  0x74737479  }

const
	typeTIFF = FourCharCode('TIFF'); {  0x54494646  }
	typeJPEG = FourCharCode('JPEG');
	typeGIF = FourCharCode('GIFf');
	typeVersion = FourCharCode('vers'); {  0x76657273  }

const
	kAEMenuClass = FourCharCode('menu');
	kAEMenuSelect = FourCharCode('mhit');
	kAEMouseDown = FourCharCode('mdwn');
	kAEMouseDownInBack = FourCharCode('mdbk');
	kAEKeyDown = FourCharCode('kdwn');
	kAEResized = FourCharCode('rsiz');
	kAEPromise = FourCharCode('prom');

const
	keyMenuID = FourCharCode('mid ');
	keyMenuItem = FourCharCode('mitm');
	keyCloseAllWindows = FourCharCode('caw ');
	keyOriginalBounds = FourCharCode('obnd');
	keyNewBounds = FourCharCode('nbnd');
	keyLocalWhere = FourCharCode('lwhr');

const
>>>>>>> origin/cpstrnew
	typeHIMenu = FourCharCode('mobj');
	typeHIWindow = FourCharCode('wobj');

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAEQuitPreserveState = FourCharCode('stat'); { in a kAEQuitApplication event, this optional parameter hints the application as to whether it should write out }
                                        { persistent state which may be restored on the next launch. The possible values are kAENo ( the default ), or kAEYes }
	kAEQuitReason = FourCharCode('why?'); { in a kAEQuitApplication event, this parameter if present is the reason the quit is being sent.  The possible values are kAEQuitAll, kAEShutDown, kAERestart, kAEReallyLogOut }
=======
	kAEQuitReason = FourCharCode('why?'); { in a kAEQuitApplication event, this property if present is the reason the quit is being sent.  The possible values are kAEQuitAll, kAEShutDown, kAERestart, kAEReallyLogOut }
>>>>>>> graemeg/cpstrnew
=======
	kAEQuitReason = FourCharCode('why?'); { in a kAEQuitApplication event, this property if present is the reason the quit is being sent.  The possible values are kAEQuitAll, kAEShutDown, kAERestart, kAEReallyLogOut }
>>>>>>> graemeg/cpstrnew
=======
	kAEQuitReason = FourCharCode('why?'); { in a kAEQuitApplication event, this property if present is the reason the quit is being sent.  The possible values are kAEQuitAll, kAEShutDown, kAERestart, kAEReallyLogOut }
>>>>>>> graemeg/cpstrnew
=======
	kAEQuitReason = FourCharCode('why?'); { in a kAEQuitApplication event, this property if present is the reason the quit is being sent.  The possible values are kAEQuitAll, kAEShutDown, kAERestart, kAEReallyLogOut }
>>>>>>> origin/cpstrnew

const
	kBySmallIcon = 0;
	kByIconView = 1;
	kByNameView = 2;
	kByDateView = 3;
	kBySizeView = 4;
	kByKindView = 5;
	kByCommentView = 6;
	kByLabelView = 7;
	kByVersionView = 8;

const
	kAEInfo = 11;
	kAEMain = 0;
	kAESharing = 13;

const
	kAEZoomIn = 7;
	kAEZoomOut = 8;

const
	kTextServiceClass = FourCharCode('tsvc');
	kUpdateActiveInputArea = FourCharCode('updt'); { update the active input area }
	kShowHideInputWindow = FourCharCode('shiw'); { show or hide the input window }
	kPos2Offset = FourCharCode('p2st'); { converting global coordinates to char position }
	kOffset2Pos = FourCharCode('st2p'); { converting char position to global coordinates }
	kUnicodeNotFromInputMethod = FourCharCode('unim'); { Unicode text when event not handled by Input Method or no Input Method }
	kGetSelectedText = FourCharCode('gtxt'); { Get text for current selection }
	keyAETSMDocumentRefcon = FourCharCode('refc'); { TSM document refcon, typeLongInteger }
	keyAEServerInstance = FourCharCode('srvi'); { component instance }
	keyAETheData = FourCharCode('kdat'); { typeText }
	keyAEFixLength = FourCharCode('fixl'); { fix len }
	keyAEUpdateRange = FourCharCode('udng'); { typeTextRangeArray }
	keyAECurrentPoint = FourCharCode('cpos'); { current point }
	keyAEBufferSize = FourCharCode('buff'); { buffer size to get the text }
	keyAEMoveView = FourCharCode('mvvw'); { move view flag }
	keyAENextBody = FourCharCode('nxbd'); { next or previous body }
	keyAETSMScriptTag = FourCharCode('sclg');
	keyAETSMTextFont = FourCharCode('ktxf'); { FMFontFamily or FOND ID }
	keyAETSMTextFMFont = FourCharCode('ktxm'); { FMFont }
	keyAETSMTextPointSize = FourCharCode('ktps');
	keyAETSMEventRecord = FourCharCode('tevt'); { Low level Event Record, typeLowLevelEventRecord }
	keyAETSMEventRef = FourCharCode('tevr'); { Carbon EventRef, typeEventRef }
	keyAETextServiceEncoding = FourCharCode('tsen'); { Text Service encoding, mac or Unicode in UpdateActiveInputArea or GetSelectedText events. }
	keyAETextServiceMacEncoding = FourCharCode('tmen'); { Target mac encoding for TSM conversion of text from Unicode text service. }
	keyAETSMGlyphInfoArray = FourCharCode('tgia'); { typeGlyphInfoArray }
	typeTextRange = FourCharCode('txrn'); { TextRange }
	typeComponentInstance = FourCharCode('cmpi'); { server instance }
	typeOffsetArray = FourCharCode('ofay'); { offset array }
	typeTextRangeArray = FourCharCode('tray');
	typeLowLevelEventRecord = FourCharCode('evtr'); { Low Level Event Record }
	typeGlyphInfoArray = FourCharCode('glia'); { Glyph/FMFont info array for sub ranges of Unicode text.  See GlyphInfoArray in TextServices.h  }
	typeEventRef = FourCharCode('evrf'); { Carbon EventRef }
	typeText = typeChar; { Plain text }


{ Desc type constants }
const
	kTSMOutsideOfBody = 1;
	kTSMInsideOfBody = 2;
	kTSMInsideOfActiveInputArea = 3;

const
	kNextBody = 1;
	kPreviousBody = 2;

type
	TextRange = record
		fStart: SInt32;
		fEnd: SInt32;
		fHiliteStyle: SInt16;
	end;
	TextRangePtr = ^TextRange;
type
	TextRangeHandle = ^TextRangePtr;
	TextRangeArray = record
		fNumOfRanges: SInt16;           { specify the size of the fRange array }
		fRange: array [0..0] of TextRange;              { when fNumOfRanges > 1, the size of this array has to be calculated }
	end;
	TextRangeArrayPtr = ^TextRangeArray;
type
	TextRangeArrayHandle = ^TextRangeArrayPtr;
	OffsetArray = record
		fNumOfOffsets: SInt16;          { specify the size of the fOffset array }
		fOffset: array [0..0] of SInt32;             { when fNumOfOffsets > 1, the size of this array has to be calculated }
	end;
	OffsetArrayPtr = ^OffsetArray;
type
	OffsetArrayHandle = ^OffsetArrayPtr;
	WritingCode = record
		theScriptCode: ScriptCode;
		theLangCode: LangCode;
	end;
	WritingCodePtr = ^WritingCode;
type
	IntlText = record
		theScriptCode: ScriptCode;
		theLangCode: LangCode;
		theText: SInt8;             { variable length data }
	end;
	IntlTextPtr = ^IntlText;

{ Hilite styles }
const
	kTSMHiliteCaretPosition = 1;    { specify caret position }
	kTSMHiliteRawText = 2;    { specify range of raw text }
	kTSMHiliteSelectedRawText = 3;    { specify range of selected raw text }
	kTSMHiliteConvertedText = 4;    { specify range of converted text }
	kTSMHiliteSelectedConvertedText = 5;  { specify range of selected converted text }
	kTSMHiliteBlockFillText = 6;    { Block Fill hilite style }
	kTSMHiliteOutlineText = 7;    { Outline hilite style }
	kTSMHiliteSelectedText = 8;    { Selected hilite style }
	kTSMHiliteNoHilite = 9;     { specify range of non-hilited text }

=======
=======
>>>>>>> origin/fixes_2_2
	cAEList						= FourCharCode('list');						{   0x6c697374   }
	cApplication				= FourCharCode('capp');						{   0x63617070   }
	cArc						= FourCharCode('carc');						{   0x63617263   }
	cBoolean					= FourCharCode('bool');						{   0x626f6f6c   }
	cCell						= FourCharCode('ccel');						{   0x6363656c   }
	cChar						= FourCharCode('cha ');						{   0x63686120   }
	cColorTable					= FourCharCode('clrt');						{   0x636c7274   }
	cColumn						= FourCharCode('ccol');						{   0x63636f6c   }
	cDocument					= FourCharCode('docu');						{   0x646f6375   }
	cDrawingArea				= FourCharCode('cdrw');						{   0x63647277   }
	cEnumeration				= FourCharCode('enum');						{   0x656e756d   }
	cFile						= FourCharCode('file');						{   0x66696c65   }
	cFixed						= FourCharCode('fixd');						{   0x66697864   }
	cFixedPoint					= FourCharCode('fpnt');						{   0x66706e74   }
	cFixedRectangle				= FourCharCode('frct');						{   0x66726374   }
	cGraphicLine				= FourCharCode('glin');						{   0x676c696e   }
	cGraphicObject				= FourCharCode('cgob');						{   0x63676f62   }
	cGraphicShape				= FourCharCode('cgsh');						{   0x63677368   }
	cGraphicText				= FourCharCode('cgtx');						{   0x63677478   }
	cGroupedGraphic				= FourCharCode('cpic');						{   0x63706963   }

	cInsertionLoc				= FourCharCode('insl');						{   0x696e736c   }
	cInsertionPoint				= FourCharCode('cins');						{   0x63696e73   }
	cIntlText					= FourCharCode('itxt');						{   0x69747874   }
	cIntlWritingCode			= FourCharCode('intl');						{   0x696e746c   }
	cItem						= FourCharCode('citm');						{   0x6369746d   }
	cLine						= FourCharCode('clin');						{   0x636c696e   }
	cLongDateTime				= FourCharCode('ldt ');						{   0x6c647420   }
	cLongFixed					= FourCharCode('lfxd');						{   0x6c667864   }
	cLongFixedPoint				= FourCharCode('lfpt');						{   0x6c667074   }
	cLongFixedRectangle			= FourCharCode('lfrc');						{   0x6c667263   }
	cLongInteger				= FourCharCode('long');						{   0x6c6f6e67   }
	cLongPoint					= FourCharCode('lpnt');						{   0x6c706e74   }
	cLongRectangle				= FourCharCode('lrct');						{   0x6c726374   }
	cMachineLoc					= FourCharCode('mLoc');						{   0x6d4c6f63   }
	cMenu						= FourCharCode('cmnu');						{   0x636d6e75   }
	cMenuItem					= FourCharCode('cmen');						{   0x636d656e   }
	cObject						= FourCharCode('cobj');						{   0x636f626a   }
	cObjectSpecifier			= FourCharCode('obj ');						{   0x6f626a20   }
	cOpenableObject				= FourCharCode('coob');						{   0x636f6f62   }
	cOval						= FourCharCode('covl');						{   0x636f766c   }

	cParagraph					= FourCharCode('cpar');						{   0x63706172   }
	cPICT						= FourCharCode('PICT');						{   0x50494354   }
	cPixel						= FourCharCode('cpxl');						{   0x6370786c   }
	cPixelMap					= FourCharCode('cpix');						{   0x63706978   }
	cPolygon					= FourCharCode('cpgn');						{   0x6370676e   }
	cProperty					= FourCharCode('prop');						{   0x70726f70   }
	cQDPoint					= FourCharCode('QDpt');						{   0x51447074   }
	cQDRectangle				= FourCharCode('qdrt');						{   0x71647274   }
	cRectangle					= FourCharCode('crec');						{   0x63726563   }
	cRGBColor					= FourCharCode('cRGB');						{   0x63524742   }
	cRotation					= FourCharCode('trot');						{   0x74726f74   }
	cRoundedRectangle			= FourCharCode('crrc');						{   0x63727263   }
	cRow						= FourCharCode('crow');						{   0x63726f77   }
	cSelection					= FourCharCode('csel');						{   0x6373656c   }
	cShortInteger				= FourCharCode('shor');						{   0x73686f72   }
	cTable						= FourCharCode('ctbl');						{   0x6374626c   }
	cText						= FourCharCode('ctxt');						{   0x63747874   }
	cTextFlow					= FourCharCode('cflo');						{   0x63666c6f   }
	cTextStyles					= FourCharCode('tsty');						{   0x74737479   }
	cType						= FourCharCode('type');						{   0x74797065   }

	cVersion					= FourCharCode('vers');						{   0x76657273   }
	cWindow						= FourCharCode('cwin');						{   0x6377696e   }
	cWord						= FourCharCode('cwor');						{   0x63776f72   }
	enumArrows					= FourCharCode('arro');						{   0x6172726f   }
	enumJustification			= FourCharCode('just');						{   0x6a757374   }
	enumKeyForm					= FourCharCode('kfrm');						{   0x6b66726d   }
	enumPosition				= FourCharCode('posi');						{   0x706f7369   }
	enumProtection				= FourCharCode('prtn');						{   0x7072746e   }
	enumQuality					= FourCharCode('qual');						{   0x7175616c   }
	enumSaveOptions				= FourCharCode('savo');						{   0x7361766f   }
	enumStyle					= FourCharCode('styl');						{   0x7374796c   }
	enumTransferMode			= FourCharCode('tran');						{   0x7472616e   }
	formUniqueID				= FourCharCode('ID  ');						{   0x49442020   }
	kAEAbout					= FourCharCode('abou');						{   0x61626f75   }
	kAEAfter					= FourCharCode('afte');						{   0x61667465   }
	kAEAliasSelection			= FourCharCode('sali');						{   0x73616c69   }
	kAEAllCaps					= FourCharCode('alcp');						{   0x616c6370   }
	kAEArrowAtEnd				= FourCharCode('aren');						{   0x6172656e   }
	kAEArrowAtStart				= FourCharCode('arst');						{   0x61727374   }
	kAEArrowBothEnds			= FourCharCode('arbo');						{   0x6172626f   }

	kAEAsk						= FourCharCode('ask ');						{   0x61736b20   }
	kAEBefore					= FourCharCode('befo');						{   0x6265666f   }
	kAEBeginning				= FourCharCode('bgng');						{   0x62676e67   }
	kAEBeginsWith				= FourCharCode('bgwt');						{   0x62677774   }
	kAEBeginTransaction			= FourCharCode('begi');						{   0x62656769   }
	kAEBold						= FourCharCode('bold');						{   0x626f6c64   }
	kAECaseSensEquals			= FourCharCode('cseq');						{   0x63736571   }
	kAECentered					= FourCharCode('cent');						{   0x63656e74   }
	kAEChangeView				= FourCharCode('view');						{   0x76696577   }
	kAEClone					= FourCharCode('clon');						{   0x636c6f6e   }
	kAEClose					= FourCharCode('clos');						{   0x636c6f73   }
	kAECondensed				= FourCharCode('cond');						{   0x636f6e64   }
	kAEContains					= FourCharCode('cont');						{   0x636f6e74   }
	kAECopy						= FourCharCode('copy');						{   0x636f7079   }
	kAECoreSuite				= FourCharCode('core');						{   0x636f7265   }
	kAECountElements			= FourCharCode('cnte');						{   0x636e7465   }
	kAECreateElement			= FourCharCode('crel');						{   0x6372656c   }
	kAECreatePublisher			= FourCharCode('cpub');						{   0x63707562   }
	kAECut						= FourCharCode('cut ');						{   0x63757420   }
	kAEDelete					= FourCharCode('delo');						{   0x64656c6f   }

	kAEDoObjectsExist			= FourCharCode('doex');						{   0x646f6578   }
	kAEDoScript					= FourCharCode('dosc');						{   0x646f7363   }
	kAEDrag						= FourCharCode('drag');						{   0x64726167   }
	kAEDuplicateSelection		= FourCharCode('sdup');						{   0x73647570   }
	kAEEditGraphic				= FourCharCode('edit');						{   0x65646974   }
	kAEEmptyTrash				= FourCharCode('empt');						{   0x656d7074   }
	kAEEnd						= FourCharCode('end ');						{   0x656e6420   }
	kAEEndsWith					= FourCharCode('ends');						{   0x656e6473   }
	kAEEndTransaction			= FourCharCode('endt');						{   0x656e6474   }
	kAEEquals					= FourCharCode('=   ');						{   0x3d202020   }
	kAEExpanded					= FourCharCode('pexp');						{   0x70657870   }
	kAEFast						= FourCharCode('fast');						{   0x66617374   }
	kAEFinderEvents				= FourCharCode('FNDR');						{   0x464e4452   }
	kAEFormulaProtect			= FourCharCode('fpro');						{   0x6670726f   }
	kAEFullyJustified			= FourCharCode('full');						{   0x66756c6c   }
	kAEGetClassInfo				= FourCharCode('qobj');						{   0x716f626a   }
	kAEGetData					= FourCharCode('getd');						{   0x67657464   }
	kAEGetDataSize				= FourCharCode('dsiz');						{   0x6473697a   }
	kAEGetEventInfo				= FourCharCode('gtei');						{   0x67746569   }
	kAEGetInfoSelection			= FourCharCode('sinf');						{   0x73696e66   }

	kAEGetPrivilegeSelection	= FourCharCode('sprv');						{   0x73707276   }
	kAEGetSuiteInfo				= FourCharCode('gtsi');						{   0x67747369   }
	kAEGreaterThan				= FourCharCode('>   ');						{   0x3e202020   }
	kAEGreaterThanEquals		= FourCharCode('>=  ');						{   0x3e3d2020   }
	kAEGrow						= FourCharCode('grow');						{   0x67726f77   }
	kAEHidden					= FourCharCode('hidn');						{   0x6869646e   }
	kAEHiQuality				= FourCharCode('hiqu');						{   0x68697175   }
	kAEImageGraphic				= FourCharCode('imgr');						{   0x696d6772   }
	kAEIsUniform				= FourCharCode('isun');						{   0x6973756e   }
	kAEItalic					= FourCharCode('ital');						{   0x6974616c   }
	kAELeftJustified			= FourCharCode('left');						{   0x6c656674   }
	kAELessThan					= FourCharCode('<   ');						{   0x3c202020   }
	kAELessThanEquals			= FourCharCode('<=  ');						{   0x3c3d2020   }
	kAELowercase				= FourCharCode('lowc');						{   0x6c6f7763   }
	kAEMakeObjectsVisible		= FourCharCode('mvis');						{   0x6d766973   }
	kAEMiscStandards			= FourCharCode('misc');						{   0x6d697363   }
	kAEModifiable				= FourCharCode('modf');						{   0x6d6f6466   }
	kAEMove						= FourCharCode('move');						{   0x6d6f7665   }
	kAENo						= FourCharCode('no  ');						{   0x6e6f2020   }
	kAENoArrow					= FourCharCode('arno');						{   0x61726e6f   }

	kAENonmodifiable			= FourCharCode('nmod');						{   0x6e6d6f64   }
	kAEOpen						= FourCharCode('odoc');						{   0x6f646f63   }
	kAEOpenSelection			= FourCharCode('sope');						{   0x736f7065   }
	kAEOutline					= FourCharCode('outl');						{   0x6f75746c   }
	kAEPageSetup				= FourCharCode('pgsu');						{   0x70677375   }
	kAEPaste					= FourCharCode('past');						{   0x70617374   }
	kAEPlain					= FourCharCode('plan');						{   0x706c616e   }
	kAEPrint					= FourCharCode('pdoc');						{   0x70646f63   }
	kAEPrintSelection			= FourCharCode('spri');						{   0x73707269   }
	kAEPrintWindow				= FourCharCode('pwin');						{   0x7077696e   }
	kAEPutAwaySelection			= FourCharCode('sput');						{   0x73707574   }
	kAEQDAddOver				= FourCharCode('addo');						{   0x6164646f   }
	kAEQDAddPin					= FourCharCode('addp');						{   0x61646470   }
	kAEQDAdMax					= FourCharCode('admx');						{   0x61646d78   }
	kAEQDAdMin					= FourCharCode('admn');						{   0x61646d6e   }
	kAEQDBic					= FourCharCode('bic ');						{   0x62696320   }
	kAEQDBlend					= FourCharCode('blnd');						{   0x626c6e64   }
	kAEQDCopy					= FourCharCode('cpy ');						{   0x63707920   }
	kAEQDNotBic					= FourCharCode('nbic');						{   0x6e626963   }
	kAEQDNotCopy				= FourCharCode('ncpy');						{   0x6e637079   }

	kAEQDNotOr					= FourCharCode('ntor');						{   0x6e746f72   }
	kAEQDNotXor					= FourCharCode('nxor');						{   0x6e786f72   }
	kAEQDOr						= FourCharCode('or  ');						{   0x6f722020   }
	kAEQDSubOver				= FourCharCode('subo');						{   0x7375626f   }
	kAEQDSubPin					= FourCharCode('subp');						{   0x73756270   }
	kAEQDSupplementalSuite		= FourCharCode('qdsp');						{   0x71647370   }
	kAEQDXor					= FourCharCode('xor ');						{   0x786f7220   }
	kAEQuickdrawSuite			= FourCharCode('qdrw');						{   0x71647277   }
	kAEQuitAll					= FourCharCode('quia');						{   0x71756961   }
	kAERedo						= FourCharCode('redo');						{   0x7265646f   }
	kAERegular					= FourCharCode('regl');						{   0x7265676c   }
	kAEReopenApplication		= FourCharCode('rapp');						{   0x72617070   }
	kAEReplace					= FourCharCode('rplc');						{   0x72706c63   }
	kAERequiredSuite			= FourCharCode('reqd');						{   0x72657164   }
	kAERestart					= FourCharCode('rest');						{   0x72657374   }
	kAERevealSelection			= FourCharCode('srev');						{   0x73726576   }
	kAERevert					= FourCharCode('rvrt');						{   0x72767274   }
	kAERightJustified			= FourCharCode('rght');						{   0x72676874   }
	kAESave						= FourCharCode('save');						{   0x73617665   }
	kAESelect					= FourCharCode('slct');						{   0x736c6374   }
	kAESetData					= FourCharCode('setd');						{   0x73657464   }

	kAESetPosition				= FourCharCode('posn');						{   0x706f736e   }
	kAEShadow					= FourCharCode('shad');						{   0x73686164   }
	kAEShowClipboard			= FourCharCode('shcl');						{   0x7368636c   }
	kAEShutDown					= FourCharCode('shut');						{   0x73687574   }
	kAESleep					= FourCharCode('slep');						{   0x736c6570   }
	kAESmallCaps				= FourCharCode('smcp');						{   0x736d6370   }
	kAESpecialClassProperties	= FourCharCode('c@#!');						{   0x63402321   }
	kAEStrikethrough			= FourCharCode('strk');						{   0x7374726b   }
	kAESubscript				= FourCharCode('sbsc');						{   0x73627363   }
	kAESuperscript				= FourCharCode('spsc');						{   0x73707363   }
	kAETableSuite				= FourCharCode('tbls');						{   0x74626c73   }
	kAETextSuite				= FourCharCode('TEXT');						{   0x54455854   }
	kAETransactionTerminated	= FourCharCode('ttrm');						{   0x7474726d   }
	kAEUnderline				= FourCharCode('undl');						{   0x756e646c   }
	kAEUndo						= FourCharCode('undo');						{   0x756e646f   }
	kAEWholeWordEquals			= FourCharCode('wweq');						{   0x77776571   }
	kAEYes						= FourCharCode('yes ');						{   0x79657320   }
	kAEZoom						= FourCharCode('zoom');						{   0x7a6f6f6d   }

    { events that can be sent to the "system" process (eg, loginwindow) on OS X 10.2 or later }
	kAELogOut                   = FourCharCode('logo');
	kAEReallyLogOut             = FourCharCode('rlgo');
	kAEShowRestartDialog        = FourCharCode('rrst');
	kAEShowShutdownDialog       = FourCharCode('rsdn');

	{	 EventRecord Classes and EventIDs 	}
	kAEMouseClass				= FourCharCode('mous');
	kAEDown						= FourCharCode('down');
	kAEUp						= FourCharCode('up  ');
	kAEMoved					= FourCharCode('move');
	kAEStoppedMoving			= FourCharCode('stop');
	kAEWindowClass				= FourCharCode('wind');
	kAEUpdate					= FourCharCode('updt');
	kAEActivate					= FourCharCode('actv');
	kAEDeactivate				= FourCharCode('dact');
	kAECommandClass				= FourCharCode('cmnd');						{  Modern Command Event Class  }
	kAEKeyClass					= FourCharCode('keyc');
	kAERawKey					= FourCharCode('rkey');						{  Modern Raw Key Event  }
	kAEVirtualKey				= FourCharCode('keyc');						{  Modern Virtual Key Event  }
	kAENavigationKey			= FourCharCode('nave');						{  Modern Navigation Key Event  }
	kAEAutoDown					= FourCharCode('auto');
	kAEApplicationClass			= FourCharCode('appl');
	kAESuspend					= FourCharCode('susp');
	kAEResume					= FourCharCode('rsme');
	kAEDiskEvent				= FourCharCode('disk');
	kAENullEvent				= FourCharCode('null');
	kAEWakeUpEvent				= FourCharCode('wake');
	kAEScrapEvent				= FourCharCode('scrp');
	kAEHighLevel				= FourCharCode('high');

	keyAEAngle					= FourCharCode('kang');						{   0x6b616e67   }
	keyAEArcAngle				= FourCharCode('parc');						{   0x70617263   }

	keyAEBaseAddr				= FourCharCode('badd');						{   0x62616464   }
	keyAEBestType				= FourCharCode('pbst');						{   0x70627374   }
	keyAEBgndColor				= FourCharCode('kbcl');						{   0x6b62636c   }
	keyAEBgndPattern			= FourCharCode('kbpt');						{   0x6b627074   }
	keyAEBounds					= FourCharCode('pbnd');						{   0x70626e64   }
	keyAECellList				= FourCharCode('kclt');						{   0x6b636c74   }
	keyAEClassID				= FourCharCode('clID');						{   0x636c4944   }
	keyAEColor					= FourCharCode('colr');						{   0x636f6c72   }
	keyAEColorTable				= FourCharCode('cltb');						{   0x636c7462   }
	keyAECurveHeight			= FourCharCode('kchd');						{   0x6b636864   }
	keyAECurveWidth				= FourCharCode('kcwd');						{   0x6b637764   }
	keyAEDashStyle				= FourCharCode('pdst');						{   0x70647374   }
	keyAEData					= FourCharCode('data');						{   0x64617461   }
	keyAEDefaultType			= FourCharCode('deft');						{   0x64656674   }
	keyAEDefinitionRect			= FourCharCode('pdrt');						{   0x70647274   }
	keyAEDescType				= FourCharCode('dstp');						{   0x64737470   }
	keyAEDestination			= FourCharCode('dest');						{   0x64657374   }
	keyAEDoAntiAlias			= FourCharCode('anta');						{   0x616e7461   }
	keyAEDoDithered				= FourCharCode('gdit');						{   0x67646974   }
	keyAEDoRotate				= FourCharCode('kdrt');						{   0x6b647274   }

	keyAEDoScale				= FourCharCode('ksca');						{   0x6b736361   }
	keyAEDoTranslate			= FourCharCode('ktra');						{   0x6b747261   }
	keyAEEditionFileLoc			= FourCharCode('eloc');						{   0x656c6f63   }
	keyAEElements				= FourCharCode('elms');						{   0x656c6d73   }
	keyAEEndPoint				= FourCharCode('pend');						{   0x70656e64   }
	keyAEEventClass				= FourCharCode('evcl');						{   0x6576636c   }
	keyAEEventID				= FourCharCode('evti');						{   0x65767469   }
	keyAEFile					= FourCharCode('kfil');						{   0x6b66696c   }
	keyAEFileType				= FourCharCode('fltp');						{   0x666c7470   }
	keyAEFillColor				= FourCharCode('flcl');						{   0x666c636c   }
	keyAEFillPattern			= FourCharCode('flpt');						{   0x666c7074   }
	keyAEFlipHorizontal			= FourCharCode('kfho');						{   0x6b66686f   }
	keyAEFlipVertical			= FourCharCode('kfvt');						{   0x6b667674   }
	keyAEFont					= FourCharCode('font');						{   0x666f6e74   }
	keyAEFormula				= FourCharCode('pfor');						{   0x70666f72   }
	keyAEGraphicObjects			= FourCharCode('gobs');						{   0x676f6273   }
	keyAEID						= FourCharCode('ID  ');						{   0x49442020   }
	keyAEImageQuality			= FourCharCode('gqua');						{   0x67717561   }
	keyAEInsertHere				= FourCharCode('insh');						{   0x696e7368   }
	keyAEKeyForms				= FourCharCode('keyf');						{   0x6b657966   }

	keyAEKeyword				= FourCharCode('kywd');						{   0x6b797764   }
	keyAELevel					= FourCharCode('levl');						{   0x6c65766c   }
	keyAELineArrow				= FourCharCode('arro');						{   0x6172726f   }
	keyAEName					= FourCharCode('pnam');						{   0x706e616d   }
	keyAENewElementLoc			= FourCharCode('pnel');						{   0x706e656c   }
	keyAEObject					= FourCharCode('kobj');						{   0x6b6f626a   }
	keyAEObjectClass			= FourCharCode('kocl');						{   0x6b6f636c   }
	keyAEOffStyles				= FourCharCode('ofst');						{   0x6f667374   }
	keyAEOnStyles				= FourCharCode('onst');						{   0x6f6e7374   }
	keyAEParameters				= FourCharCode('prms');						{   0x70726d73   }
	keyAEParamFlags				= FourCharCode('pmfg');						{   0x706d6667   }
	keyAEPenColor				= FourCharCode('ppcl');						{   0x7070636c   }
	keyAEPenPattern				= FourCharCode('pppa');						{   0x70707061   }
	keyAEPenWidth				= FourCharCode('ppwd');						{   0x70707764   }
	keyAEPixelDepth				= FourCharCode('pdpt');						{   0x70647074   }
	keyAEPixMapMinus			= FourCharCode('kpmm');						{   0x6b706d6d   }
	keyAEPMTable				= FourCharCode('kpmt');						{   0x6b706d74   }
	keyAEPointList				= FourCharCode('ptlt');						{   0x70746c74   }
	keyAEPointSize				= FourCharCode('ptsz');						{   0x7074737a   }
	keyAEPosition				= FourCharCode('kpos');						{   0x6b706f73   }

	keyAEPropData				= FourCharCode('prdt');						{   0x70726474   }
	keyAEProperties				= FourCharCode('qpro');						{   0x7170726f   }
	keyAEProperty				= FourCharCode('kprp');						{   0x6b707270   }
	keyAEPropFlags				= FourCharCode('prfg');						{   0x70726667   }
	keyAEPropID					= FourCharCode('prop');						{   0x70726f70   }
	keyAEProtection				= FourCharCode('ppro');						{   0x7070726f   }
	keyAERenderAs				= FourCharCode('kren');						{   0x6b72656e   }
	keyAERequestedType			= FourCharCode('rtyp');						{   0x72747970   }
	keyAEResult					= FourCharCode('----');						{   0x2d2d2d2d   }
	keyAEResultInfo				= FourCharCode('rsin');						{   0x7273696e   }
	keyAERotation				= FourCharCode('prot');						{   0x70726f74   }
	keyAERotPoint				= FourCharCode('krtp');						{   0x6b727470   }
	keyAERowList				= FourCharCode('krls');						{   0x6b726c73   }
	keyAESaveOptions			= FourCharCode('savo');						{   0x7361766f   }
	keyAEScale					= FourCharCode('pscl');						{   0x7073636c   }
	keyAEScriptTag				= FourCharCode('psct');						{   0x70736374   }
	keyAEShowWhere				= FourCharCode('show');						{   0x73686f77   }
	keyAEStartAngle				= FourCharCode('pang');						{   0x70616e67   }
	keyAEStartPoint				= FourCharCode('pstp');						{   0x70737470   }
	keyAEStyles					= FourCharCode('ksty');						{   0x6b737479   }

	keyAESuiteID				= FourCharCode('suit');						{   0x73756974   }
	keyAEText					= FourCharCode('ktxt');						{   0x6b747874   }
	keyAETextColor				= FourCharCode('ptxc');						{   0x70747863   }
	keyAETextFont				= FourCharCode('ptxf');						{   0x70747866   }
	keyAETextPointSize			= FourCharCode('ptps');						{   0x70747073   }
	keyAETextStyles				= FourCharCode('txst');						{   0x74787374   }
	keyAETextLineHeight			= FourCharCode('ktlh');						{  type ShortInteger  }
	keyAETextLineAscent			= FourCharCode('ktas');						{  type ShortInteger  }
	keyAETheText				= FourCharCode('thtx');						{   0x74687478   }
	keyAETransferMode			= FourCharCode('pptm');						{   0x7070746d   }
	keyAETranslation			= FourCharCode('ptrs');						{   0x70747273   }
	keyAETryAsStructGraf		= FourCharCode('toog');						{   0x746f6f67   }
	keyAEUniformStyles			= FourCharCode('ustl');						{   0x7573746c   }
	keyAEUpdateOn				= FourCharCode('pupd');						{   0x70757064   }
	keyAEUserTerm				= FourCharCode('utrm');						{   0x7574726d   }
	keyAEWindow					= FourCharCode('wndw');						{   0x776e6477   }
	keyAEWritingCode			= FourCharCode('wrcd');						{   0x77726364   }

	keyMiscellaneous			= FourCharCode('fmsc');						{   0x666d7363   }
	keySelection				= FourCharCode('fsel');						{   0x6673656c   }
	keyWindow					= FourCharCode('kwnd');						{   0x6b776e64   }
																{  EventRecord keys  }
	keyWhen						= FourCharCode('when');
	keyWhere					= FourCharCode('wher');
	keyModifiers				= FourCharCode('mods');
	keyKey						= FourCharCode('key ');
	keyKeyCode					= FourCharCode('code');
	keyKeyboard					= FourCharCode('keyb');
	keyDriveNumber				= FourCharCode('drv#');
	keyErrorCode				= FourCharCode('err#');
	keyHighLevelClass			= FourCharCode('hcls');
	keyHighLevelID				= FourCharCode('hid ');

	pArcAngle					= FourCharCode('parc');						{   0x70617263   }
	pBackgroundColor			= FourCharCode('pbcl');						{   0x7062636c   }
	pBackgroundPattern			= FourCharCode('pbpt');						{   0x70627074   }
	pBestType					= FourCharCode('pbst');						{   0x70627374   }
	pBounds						= FourCharCode('pbnd');						{   0x70626e64   }
	pClass						= FourCharCode('pcls');						{   0x70636c73   }
	pClipboard					= FourCharCode('pcli');						{   0x70636c69   }
	pColor						= FourCharCode('colr');						{   0x636f6c72   }
	pColorTable					= FourCharCode('cltb');						{   0x636c7462   }
	pContents					= FourCharCode('pcnt');						{   0x70636e74   }
	pCornerCurveHeight			= FourCharCode('pchd');						{   0x70636864   }
	pCornerCurveWidth			= FourCharCode('pcwd');						{   0x70637764   }
	pDashStyle					= FourCharCode('pdst');						{   0x70647374   }
	pDefaultType				= FourCharCode('deft');						{   0x64656674   }
	pDefinitionRect				= FourCharCode('pdrt');						{   0x70647274   }
	pEnabled					= FourCharCode('enbl');						{   0x656e626c   }
	pEndPoint					= FourCharCode('pend');						{   0x70656e64   }
	pFillColor					= FourCharCode('flcl');						{   0x666c636c   }
	pFillPattern				= FourCharCode('flpt');						{   0x666c7074   }
	pFont						= FourCharCode('font');						{   0x666f6e74   }

	pFormula					= FourCharCode('pfor');						{   0x70666f72   }
	pGraphicObjects				= FourCharCode('gobs');						{   0x676f6273   }
	pHasCloseBox				= FourCharCode('hclb');						{   0x68636c62   }
	pHasTitleBar				= FourCharCode('ptit');						{   0x70746974   }
	pID							= FourCharCode('ID  ');						{   0x49442020   }
	pIndex						= FourCharCode('pidx');						{   0x70696478   }
	pInsertionLoc				= FourCharCode('pins');						{   0x70696e73   }
	pIsFloating					= FourCharCode('isfl');						{   0x6973666c   }
	pIsFrontProcess				= FourCharCode('pisf');						{   0x70697366   }
	pIsModal					= FourCharCode('pmod');						{   0x706d6f64   }
	pIsModified					= FourCharCode('imod');						{   0x696d6f64   }
	pIsResizable				= FourCharCode('prsz');						{   0x7072737a   }
	pIsStationeryPad			= FourCharCode('pspd');						{   0x70737064   }
	pIsZoomable					= FourCharCode('iszm');						{   0x69737a6d   }
	pIsZoomed					= FourCharCode('pzum');						{   0x707a756d   }
	pItemNumber					= FourCharCode('itmn');						{   0x69746d6e   }
	pJustification				= FourCharCode('pjst');						{   0x706a7374   }
	pLineArrow					= FourCharCode('arro');						{   0x6172726f   }
	pMenuID						= FourCharCode('mnid');						{   0x6d6e6964   }
	pName						= FourCharCode('pnam');						{   0x706e616d   }

	pNewElementLoc				= FourCharCode('pnel');						{   0x706e656c   }
	pPenColor					= FourCharCode('ppcl');						{   0x7070636c   }
	pPenPattern					= FourCharCode('pppa');						{   0x70707061   }
	pPenWidth					= FourCharCode('ppwd');						{   0x70707764   }
	pPixelDepth					= FourCharCode('pdpt');						{   0x70647074   }
	pPointList					= FourCharCode('ptlt');						{   0x70746c74   }
	pPointSize					= FourCharCode('ptsz');						{   0x7074737a   }
	pProtection					= FourCharCode('ppro');						{   0x7070726f   }
	pRotation					= FourCharCode('prot');						{   0x70726f74   }
	pScale						= FourCharCode('pscl');						{   0x7073636c   }
	pScript						= FourCharCode('scpt');						{   0x73637074   }
	pScriptTag					= FourCharCode('psct');						{   0x70736374   }
	pSelected					= FourCharCode('selc');						{   0x73656c63   }
	pSelection					= FourCharCode('sele');						{   0x73656c65   }
	pStartAngle					= FourCharCode('pang');						{   0x70616e67   }
	pStartPoint					= FourCharCode('pstp');						{   0x70737470   }
	pTextColor					= FourCharCode('ptxc');						{   0x70747863   }
	pTextFont					= FourCharCode('ptxf');						{   0x70747866   }
	pTextItemDelimiters			= FourCharCode('txdl');						{   0x7478646c   }
	pTextPointSize				= FourCharCode('ptps');						{   0x70747073   }

	pTextStyles					= FourCharCode('txst');						{   0x74787374   }
	pTransferMode				= FourCharCode('pptm');						{   0x7070746d   }
	pTranslation				= FourCharCode('ptrs');						{   0x70747273   }
	pUniformStyles				= FourCharCode('ustl');						{   0x7573746c   }
	pUpdateOn					= FourCharCode('pupd');						{   0x70757064   }
	pUserSelection				= FourCharCode('pusl');						{   0x7075736c   }
	pVersion					= FourCharCode('vers');						{   0x76657273   }
	pVisible					= FourCharCode('pvis');						{   0x70766973   }

	typeAEText					= FourCharCode('tTXT');						{   0x74545854   }
	typeArc						= FourCharCode('carc');						{   0x63617263   }
	typeBest					= FourCharCode('best');						{   0x62657374   }
	typeCell					= FourCharCode('ccel');						{   0x6363656c   }
	typeClassInfo				= FourCharCode('gcli');						{   0x67636c69   }
	typeColorTable				= FourCharCode('clrt');						{   0x636c7274   }
	typeColumn					= FourCharCode('ccol');						{   0x63636f6c   }
	typeDashStyle				= FourCharCode('tdas');						{   0x74646173   }
	typeData					= FourCharCode('tdta');						{   0x74647461   }
	typeDrawingArea				= FourCharCode('cdrw');						{   0x63647277   }
	typeElemInfo				= FourCharCode('elin');						{   0x656c696e   }
	typeEnumeration				= FourCharCode('enum');						{   0x656e756d   }
	typeEPS						= FourCharCode('EPS ');						{   0x45505320   }
	typeEventInfo				= FourCharCode('evin');						{   0x6576696e   }

	typeFinderWindow			= FourCharCode('fwin');						{   0x6677696e   }
	typeFixedPoint				= FourCharCode('fpnt');						{   0x66706e74   }
	typeFixedRectangle			= FourCharCode('frct');						{   0x66726374   }
	typeGraphicLine				= FourCharCode('glin');						{   0x676c696e   }
	typeGraphicText				= FourCharCode('cgtx');						{   0x63677478   }
	typeGroupedGraphic			= FourCharCode('cpic');						{   0x63706963   }
	typeInsertionLoc			= FourCharCode('insl');						{   0x696e736c   }
	typeIntlText				= FourCharCode('itxt');						{   0x69747874   }
	typeIntlWritingCode			= FourCharCode('intl');						{   0x696e746c   }
	typeLongDateTime			= FourCharCode('ldt ');						{   0x6c647420   }
	typeISO8601DateTime         = FourCharCode('isot');                       {   0x69736f74  data is ascii text of an ISO8601 date }
	typeLongFixed				= FourCharCode('lfxd');						{   0x6c667864   }
	typeLongFixedPoint			= FourCharCode('lfpt');						{   0x6c667074   }
	typeLongFixedRectangle		= FourCharCode('lfrc');						{   0x6c667263   }
	typeLongPoint				= FourCharCode('lpnt');						{   0x6c706e74   }
	typeLongRectangle			= FourCharCode('lrct');						{   0x6c726374   }
	typeMachineLoc				= FourCharCode('mLoc');						{   0x6d4c6f63   }
	typeOval					= FourCharCode('covl');						{   0x636f766c   }
	typeParamInfo				= FourCharCode('pmin');						{   0x706d696e   }
	typePict					= FourCharCode('PICT');						{   0x50494354   }

	typePixelMap				= FourCharCode('cpix');						{   0x63706978   }
	typePixMapMinus				= FourCharCode('tpmm');						{   0x74706d6d   }
	typePolygon					= FourCharCode('cpgn');						{   0x6370676e   }
	typePropInfo				= FourCharCode('pinf');						{   0x70696e66   }
	typePtr						= FourCharCode('ptr ');						{   0x70747220   }
	typeQDPoint					= FourCharCode('QDpt');						{   0x51447074   }
	typeQDRegion				= FourCharCode('Qrgn');						{   0x51447074  (data is actual region data, including rectangle and size, _not_ region handle or ptr) }
	typeRectangle				= FourCharCode('crec');						{   0x63726563   }
	typeRGB16					= FourCharCode('tr16');						{   0x74723136   }
	typeRGB96					= FourCharCode('tr96');						{   0x74723936   }
	typeRGBColor				= FourCharCode('cRGB');						{   0x63524742   }
	typeRotation				= FourCharCode('trot');						{   0x74726f74   }
	typeRoundedRectangle		= FourCharCode('crrc');						{   0x63727263   }
	typeRow						= FourCharCode('crow');						{   0x63726f77   }
	typeScrapStyles				= FourCharCode('styl');						{   0x7374796c   }
	typeScript					= FourCharCode('scpt');						{   0x73637074   }
	typeStyledText				= FourCharCode('STXT');						{   0x53545854   }
	typeSuiteInfo				= FourCharCode('suin');						{   0x7375696e   }
	typeTable					= FourCharCode('ctbl');						{   0x6374626c   }
	typeTextStyles				= FourCharCode('tsty');						{   0x74737479   }

	typeTIFF					= FourCharCode('TIFF');						{   0x54494646   }
	typeJPEG                    = FourCharCode('JPEG');
	typeGIF                     = FourCharCode('GIFf');
	typeVersion					= FourCharCode('vers');						{   0x76657273   }

	kAEMenuClass				= FourCharCode('menu');
	kAEMenuSelect				= FourCharCode('mhit');
	kAEMouseDown				= FourCharCode('mdwn');
	kAEMouseDownInBack			= FourCharCode('mdbk');
	kAEKeyDown					= FourCharCode('kdwn');
	kAEResized					= FourCharCode('rsiz');
	kAEPromise					= FourCharCode('prom');

	keyMenuID					= FourCharCode('mid ');
	keyMenuItem					= FourCharCode('mitm');
	keyCloseAllWindows			= FourCharCode('caw ');
	keyOriginalBounds			= FourCharCode('obnd');
	keyNewBounds				= FourCharCode('nbnd');
	keyLocalWhere				= FourCharCode('lwhr');

	typeHIMenu					= FourCharCode('mobj');
	typeHIWindow				= FourCharCode('wobj');

	kBySmallIcon				= 0;
	kByIconView					= 1;
	kByNameView					= 2;
	kByDateView					= 3;
	kBySizeView					= 4;
	kByKindView					= 5;
	kByCommentView				= 6;
	kByLabelView				= 7;
	kByVersionView				= 8;

	kAEInfo						= 11;
	kAEMain						= 0;
	kAESharing					= 13;

	kAEZoomIn					= 7;
	kAEZoomOut					= 8;

	kTextServiceClass			= FourCharCode('tsvc');
	kUpdateActiveInputArea		= FourCharCode('updt');						{  update the active input area  }
	kShowHideInputWindow		= FourCharCode('shiw');						{  show or hide the input window  }
	kPos2Offset					= FourCharCode('p2st');						{  converting global coordinates to char position  }
	kOffset2Pos					= FourCharCode('st2p');						{  converting char position to global coordinates  }
	kUnicodeNotFromInputMethod	= FourCharCode('unim');						{  Unicode text when event not handled by Input Method or no Input Method  }
	kGetSelectedText			= FourCharCode('gtxt');						{  Get text for current selection  }
	keyAETSMDocumentRefcon		= FourCharCode('refc');						{  TSM document refcon, typeLongInteger  }
	keyAEServerInstance			= FourCharCode('srvi');						{  component instance  }
	keyAETheData				= FourCharCode('kdat');						{  typeText  }
	keyAEFixLength				= FourCharCode('fixl');						{  fix len  }
	keyAEUpdateRange			= FourCharCode('udng');						{  typeTextRangeArray  }
	keyAECurrentPoint			= FourCharCode('cpos');						{  current point  }
	keyAEBufferSize				= FourCharCode('buff');						{  buffer size to get the text  }
	keyAEMoveView				= FourCharCode('mvvw');						{  move view flag  }
	keyAENextBody				= FourCharCode('nxbd');						{  next or previous body  }
	keyAETSMScriptTag			= FourCharCode('sclg');
	keyAETSMTextFont			= FourCharCode('ktxf');						{  FMFontFamily or FOND ID  }
	keyAETSMTextFMFont			= FourCharCode('ktxm');						{  FMFont  }
	keyAETSMTextPointSize		= FourCharCode('ktps');
	keyAETSMEventRecord			= FourCharCode('tevt');						{  Low level Event Record, typeLowLevelEventRecord  }
	keyAETSMEventRef			= FourCharCode('tevr');						{  Carbon EventRef, typeEventRef  }
	keyAETextServiceEncoding	= FourCharCode('tsen');						{  Text Service encoding, mac or Unicode in UpdateActiveInputArea or GetSelectedText events.  }
	keyAETextServiceMacEncoding	= FourCharCode('tmen');						{  Target mac encoding for TSM conversion of text from Unicode text service.  }
	typeTextRange				= FourCharCode('txrn');						{  TextRange  }
	typeComponentInstance		= FourCharCode('cmpi');						{  server instance  }
	typeOffsetArray				= FourCharCode('ofay');						{  offset array  }
	typeTextRangeArray			= FourCharCode('tray');
	typeLowLevelEventRecord		= FourCharCode('evtr');						{  Low Level Event Record  }
	typeEventRef				= FourCharCode('evrf');						{  Carbon EventRef  }
	typeText					= FourCharCode('TEXT');						{  Plain text  }


	{	 Desc type constants 	}
	kTSMOutsideOfBody			= 1;
	kTSMInsideOfBody			= 2;
	kTSMInsideOfActiveInputArea	= 3;

	kNextBody					= 1;
	kPreviousBody				= 2;
=======
>>>>>>> origin/fixes_2.4

const
	typeHIMenu = FourCharCode('mobj');
	typeHIWindow = FourCharCode('wobj');

const
	kAEQuitReason = FourCharCode('why?'); { in a kAEQuitApplication event, this property if present is the reason the quit is being sent.  The possible values are kAEQuitAll, kAEShutDown, kAERestart, kAEReallyLogOut }

const
	kBySmallIcon = 0;
	kByIconView = 1;
	kByNameView = 2;
	kByDateView = 3;
	kBySizeView = 4;
	kByKindView = 5;
	kByCommentView = 6;
	kByLabelView = 7;
	kByVersionView = 8;

const
	kAEInfo = 11;
	kAEMain = 0;
	kAESharing = 13;

const
	kAEZoomIn = 7;
	kAEZoomOut = 8;

const
	kTextServiceClass = FourCharCode('tsvc');
	kUpdateActiveInputArea = FourCharCode('updt'); { update the active input area }
	kShowHideInputWindow = FourCharCode('shiw'); { show or hide the input window }
	kPos2Offset = FourCharCode('p2st'); { converting global coordinates to char position }
	kOffset2Pos = FourCharCode('st2p'); { converting char position to global coordinates }
	kUnicodeNotFromInputMethod = FourCharCode('unim'); { Unicode text when event not handled by Input Method or no Input Method }
	kGetSelectedText = FourCharCode('gtxt'); { Get text for current selection }
	keyAETSMDocumentRefcon = FourCharCode('refc'); { TSM document refcon, typeLongInteger }
	keyAEServerInstance = FourCharCode('srvi'); { component instance }
	keyAETheData = FourCharCode('kdat'); { typeText }
	keyAEFixLength = FourCharCode('fixl'); { fix len }
	keyAEUpdateRange = FourCharCode('udng'); { typeTextRangeArray }
	keyAECurrentPoint = FourCharCode('cpos'); { current point }
	keyAEBufferSize = FourCharCode('buff'); { buffer size to get the text }
	keyAEMoveView = FourCharCode('mvvw'); { move view flag }
	keyAENextBody = FourCharCode('nxbd'); { next or previous body }
	keyAETSMScriptTag = FourCharCode('sclg');
	keyAETSMTextFont = FourCharCode('ktxf'); { FMFontFamily or FOND ID }
	keyAETSMTextFMFont = FourCharCode('ktxm'); { FMFont }
	keyAETSMTextPointSize = FourCharCode('ktps');
	keyAETSMEventRecord = FourCharCode('tevt'); { Low level Event Record, typeLowLevelEventRecord }
	keyAETSMEventRef = FourCharCode('tevr'); { Carbon EventRef, typeEventRef }
	keyAETextServiceEncoding = FourCharCode('tsen'); { Text Service encoding, mac or Unicode in UpdateActiveInputArea or GetSelectedText events. }
	keyAETextServiceMacEncoding = FourCharCode('tmen'); { Target mac encoding for TSM conversion of text from Unicode text service. }
	keyAETSMGlyphInfoArray = FourCharCode('tgia'); { typeGlyphInfoArray }
	typeTextRange = FourCharCode('txrn'); { TextRange }
	typeComponentInstance = FourCharCode('cmpi'); { server instance }
	typeOffsetArray = FourCharCode('ofay'); { offset array }
	typeTextRangeArray = FourCharCode('tray');
	typeLowLevelEventRecord = FourCharCode('evtr'); { Low Level Event Record }
	typeGlyphInfoArray = FourCharCode('glia'); { Glyph/FMFont info array for sub ranges of Unicode text.  See GlyphInfoArray in TextServices.h  }
	typeEventRef = FourCharCode('evrf'); { Carbon EventRef }
	typeText = typeChar; { Plain text }


{ Desc type constants }
const
	kTSMOutsideOfBody = 1;
	kTSMInsideOfBody = 2;
	kTSMInsideOfActiveInputArea = 3;

const
	kNextBody = 1;
	kPreviousBody = 2;

type
	TextRange = record
		fStart: SInt32;
		fEnd: SInt32;
		fHiliteStyle: SInt16;
	end;
	TextRangePtr = ^TextRange;
type
	TextRangeHandle = ^TextRangePtr;
	TextRangeArray = record
		fNumOfRanges: SInt16;           { specify the size of the fRange array }
		fRange: array [0..0] of TextRange;              { when fNumOfRanges > 1, the size of this array has to be calculated }
	end;
	TextRangeArrayPtr = ^TextRangeArray;
type
	TextRangeArrayHandle = ^TextRangeArrayPtr;
	OffsetArray = record
		fNumOfOffsets: SInt16;          { specify the size of the fOffset array }
		fOffset: array [0..0] of SInt32;             { when fNumOfOffsets > 1, the size of this array has to be calculated }
	end;
	OffsetArrayPtr = ^OffsetArray;
type
	OffsetArrayHandle = ^OffsetArrayPtr;
	WritingCode = record
		theScriptCode: ScriptCode;
		theLangCode: LangCode;
	end;
	WritingCodePtr = ^WritingCode;
type
	IntlText = record
		theScriptCode: ScriptCode;
		theLangCode: LangCode;
		theText: SInt8;             { variable length data }
	end;
	IntlTextPtr = ^IntlText;

{ Hilite styles }
const
	kTSMHiliteCaretPosition = 1;    { specify caret position }
	kTSMHiliteRawText = 2;    { specify range of raw text }
	kTSMHiliteSelectedRawText = 3;    { specify range of selected raw text }
	kTSMHiliteConvertedText = 4;    { specify range of converted text }
	kTSMHiliteSelectedConvertedText = 5;  { specify range of selected converted text }
<<<<<<< HEAD
	kTSMHiliteBlockFillText     = 6;    { Block Fill hilite style }
	kTSMHiliteOutlineText       = 7;    { Outline hilite style }
	kTSMHiliteSelectedText      = 8;    { Selected hilite style }
	kTSMHiliteNoHilite          = 9;     { specify range of non-hilited text }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kTSMHiliteBlockFillText = 6;    { Block Fill hilite style }
	kTSMHiliteOutlineText = 7;    { Outline hilite style }
	kTSMHiliteSelectedText = 8;    { Selected hilite style }
	kTSMHiliteNoHilite = 9;     { specify range of non-hilited text }

>>>>>>> origin/fixes_2.4

{$ifc OLDROUTINENAMES}
{ Hilite styles }
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kCaretPosition = kTSMHiliteCaretPosition;
	kRawText = kTSMHiliteRawText;
	kSelectedRawText = kTSMHiliteSelectedRawText;
	kConvertedText = kTSMHiliteConvertedText;
	kSelectedConvertedText = kTSMHiliteSelectedConvertedText;
	kBlockFillText = kTSMHiliteBlockFillText;
	kOutlineText = kTSMHiliteOutlineText;
	kSelectedText = kTSMHiliteSelectedText;
<<<<<<< HEAD

{$endc}  {OLDROUTINENAMES}

const
	keyAEHiliteRange = FourCharCode('hrng'); { typeTextRangeArray for System 7, typeHiliteRangeArray for System 8 }
	keyAEPinRange = FourCharCode('pnrg'); { typeTextRange for System 7, typeTextRegionRange for System 8   }
	keyAEClauseOffsets = FourCharCode('clau'); { typeOffsetArray for System 7, typeClauseOffsetArray for System 8 }
	keyAEOffset = FourCharCode('ofst'); { typeLongInteger for System 7, typeByteOffset for System 8  }
	keyAEPoint = FourCharCode('gpos'); { typePoint for System 7, typeQDPoint for System 8 }
	keyAELeftSide = FourCharCode('klef'); { typeBoolean }
	keyAERegionClass = FourCharCode('rgnc'); { typeShortInteger for System 7, typeRegionClass for System 8 }
	keyAEDragging = FourCharCode('bool'); { typeBoolean }


{$ifc OLDROUTINENAMES}
const
	keyAELeadingEdge = keyAELeftSide;

{$endc}  {OLDROUTINENAMES}

const
{ AppleScript 1.3: Unit types }
	typeMeters = FourCharCode('metr'); { Base Unit }
	typeInches = FourCharCode('inch');
	typeFeet = FourCharCode('feet');
	typeYards = FourCharCode('yard');
	typeMiles = FourCharCode('mile');
	typeKilometers = FourCharCode('kmtr');
	typeCentimeters = FourCharCode('cmtr');
	typeSquareMeters = FourCharCode('sqrm'); { Base Unit }
	typeSquareFeet = FourCharCode('sqft');
	typeSquareYards = FourCharCode('sqyd');
	typeSquareMiles = FourCharCode('sqmi');
	typeSquareKilometers = FourCharCode('sqkm');
	typeLiters = FourCharCode('litr'); { Base Unit }
	typeQuarts = FourCharCode('qrts');
	typeGallons = FourCharCode('galn');
	typeCubicMeters = FourCharCode('cmet'); { Base Unit }
	typeCubicFeet = FourCharCode('cfet');
	typeCubicInches = FourCharCode('cuin');
	typeCubicCentimeter = FourCharCode('ccmt');
	typeCubicYards = FourCharCode('cyrd');
	typeKilograms = FourCharCode('kgrm'); { Base Unit }
	typeGrams = FourCharCode('gram');
	typeOunces = FourCharCode('ozs ');
	typePounds = FourCharCode('lbs ');
	typeDegreesC = FourCharCode('degc'); { Base Unit }
	typeDegreesF = FourCharCode('degf');
	typeDegreesK = FourCharCode('degk');

const
{ AppleScript 1.3: Folder Actions }
	kFAServerApp = FourCharCode('ssrv'); { Creator code for Folder Actions Server}
	kDoFolderActionEvent = FourCharCode('fola'); { Event the Finder sends to the Folder Actions FBA}
	kFolderActionCode = FourCharCode('actn'); { Parameter that contains the Folder Action}
	kFolderOpenedEvent = FourCharCode('fopn'); { Value of kFolderActionCode parameter; sent to script as event}
	kFolderClosedEvent = FourCharCode('fclo');
	kFolderWindowMovedEvent = FourCharCode('fsiz');
	kFolderItemsAddedEvent = FourCharCode('fget');
	kFolderItemsRemovedEvent = FourCharCode('flos');
	kItemList = FourCharCode('flst'); { List parameter for added and removed items}
	kNewSizeParameter = FourCharCode('fnsz'); { Parameter for moved window}
	kFASuiteCode = FourCharCode('faco'); { Suite code for the following events}
	kFAAttachCommand = FourCharCode('atfa'); { Attach event id}
	kFARemoveCommand = FourCharCode('rmfa'); { Remove event id}
	kFAEditCommand = FourCharCode('edfa'); { Edit event id}
	kFAFileParam = FourCharCode('faal'); { Key for file parameter for Attach}
	kFAIndexParam = FourCharCode('indx'); { Key for index (0-based) parameter for Remove and Edit}

{ AppleScript 1.3 Internet Suite }
const
{ Suite code }
	kAEInternetSuite = FourCharCode('gurl');
	kAEISWebStarSuite = $575757BD;

const
{ Events }
	kAEISGetURL = FourCharCode('gurl');
	KAEISHandleCGI = FourCharCode('sdoc');

const
{ Classes }
	cURL = FourCharCode('url ');
	cInternetAddress = FourCharCode('IPAD');
	cHTML = FourCharCode('html');
	cFTPItem = FourCharCode('ftp ');

const
{ Parameters }
	kAEISHTTPSearchArgs = FourCharCode('kfor');
	kAEISPostArgs = FourCharCode('post');
	kAEISMethod = FourCharCode('meth');
	kAEISClientAddress = FourCharCode('addr');
	kAEISUserName = FourCharCode('user');
	kAEISPassword = FourCharCode('pass');
	kAEISFromUser = FourCharCode('frmu');
	kAEISServerName = FourCharCode('svnm');
	kAEISServerPort = FourCharCode('svpt');
	kAEISScriptName = FourCharCode('scnm');
	kAEISContentType = FourCharCode('ctyp');
	kAEISReferrer = FourCharCode('refr');
	kAEISUserAgent = FourCharCode('Agnt');
	kAEISAction = FourCharCode('Kact');
	kAEISActionPath = FourCharCode('Kapt');
	kAEISClientIP = FourCharCode('Kcip');
	kAEISFullRequest = FourCharCode('Kfrq');

const
{ Properties }
	pScheme = FourCharCode('pusc');
	pHost = FourCharCode('HOST');
	pPath = FourCharCode('FTPc');
	pUserName = FourCharCode('RAun');
	pUserPassword = FourCharCode('RApw');
	pDNSForm = FourCharCode('pDNS');
	pURL = FourCharCode('pURL');
	pTextEncoding = FourCharCode('ptxe');
	pFTPKind = FourCharCode('kind');

const
{ Scheme enumerations }
	eScheme = FourCharCode('esch');
	eurlHTTP = FourCharCode('http'); { RFC 2068 }
	eurlHTTPS = FourCharCode('htps');
	eurlFTP = FourCharCode('ftp '); { RFC 1738 }
	eurlMail = FourCharCode('mail'); { RFC 2638 }
	eurlFile = FourCharCode('file'); { RFC 1738 }
	eurlGopher = FourCharCode('gphr'); { RFC 1738 }
	eurlTelnet = FourCharCode('tlnt'); { RFC 1738 }
	eurlNews = FourCharCode('news'); { RFC 1738 }
	eurlSNews = FourCharCode('snws');
	eurlNNTP = FourCharCode('nntp'); { RFC 1738 }
	eurlMessage = FourCharCode('mess');
	eurlMailbox = FourCharCode('mbox');
	eurlMulti = FourCharCode('mult');
	eurlLaunch = FourCharCode('laun');
	eurlAFP = FourCharCode('afp ');
	eurlAT = FourCharCode('at  ');
	eurlEPPC = FourCharCode('eppc');
	eurlRTSP = FourCharCode('rtsp'); { RFC 2326 }
	eurlIMAP = FourCharCode('imap'); { RFC 2192 }
	eurlNFS = FourCharCode('unfs'); { RFC 2224 }
	eurlPOP = FourCharCode('upop'); { RFC 2384 }
	eurlLDAP = FourCharCode('uldp'); { RFC 2255 }
	eurlUnknown = FourCharCode('url?');

const
{ AppleScript 1.3: Connectivity Suite in aeut }
	kConnSuite = FourCharCode('macc');
	cDevSpec = FourCharCode('cdev');
	cAddressSpec = FourCharCode('cadr');
	cADBAddress = FourCharCode('cadb');
	cAppleTalkAddress = FourCharCode('cat ');
	cBusAddress = FourCharCode('cbus');
	cEthernetAddress = FourCharCode('cen ');
	cFireWireAddress = FourCharCode('cfw ');
	cIPAddress = FourCharCode('cip ');
	cLocalTalkAddress = FourCharCode('clt ');
	cSCSIAddress = FourCharCode('cscs');
	cTokenRingAddress = FourCharCode('ctok');
	cUSBAddress = FourCharCode('cusb'); { }
                                        { Properties }
	pDeviceType = FourCharCode('pdvt');
	pDeviceAddress = FourCharCode('pdva');
	pConduit = FourCharCode('pcon');
	pProtocol = FourCharCode('pprt'); { cde 4/27/98 was 'ppro' conflicted with DB suite }
	pATMachine = FourCharCode('patm');
	pATZone = FourCharCode('patz');
	pATType = FourCharCode('patt');
	pDottedDecimal = FourCharCode('pipd');
	pDNS = FourCharCode('pdns');
	pPort = FourCharCode('ppor');
	pNetwork = FourCharCode('pnet');
	pNode = FourCharCode('pnod');
	pSocket = FourCharCode('psoc');
	pSCSIBus = FourCharCode('pscb');
	pSCSILUN = FourCharCode('pslu'); { cde 5/22/98 per WWDC developer request }
                                        { Enumerations and enumerators }
	eDeviceType = FourCharCode('edvt');
	eAddressSpec = FourCharCode('eads');
	eConduit = FourCharCode('econ');
	eProtocol = FourCharCode('epro');
	eADB = FourCharCode('eadb');
	eAnalogAudio = FourCharCode('epau');
	eAppleTalk = FourCharCode('epat');
	eAudioLineIn = FourCharCode('ecai');
	eAudioLineOut = FourCharCode('ecal'); { cde 4/24/98 changed from 'ecao' to not conflict }
	eAudioOut = FourCharCode('ecao');
	eBus = FourCharCode('ebus');
	eCDROM = FourCharCode('ecd ');
	eCommSlot = FourCharCode('eccm');
	eDigitalAudio = FourCharCode('epda');
	eDisplay = FourCharCode('edds');
	eDVD = FourCharCode('edvd');
	eEthernet = FourCharCode('ecen');
	eFireWire = FourCharCode('ecfw');
	eFloppy = FourCharCode('efd ');
	eHD = FourCharCode('ehd ');
	eInfrared = FourCharCode('ecir');
	eIP = FourCharCode('epip');
	eIrDA = FourCharCode('epir');
	eIRTalk = FourCharCode('epit');
	eKeyboard = FourCharCode('ekbd');
	eLCD = FourCharCode('edlc');
	eLocalTalk = FourCharCode('eclt');
	eMacIP = FourCharCode('epmi');
	eMacVideo = FourCharCode('epmv');
	eMicrophone = FourCharCode('ecmi');
	eModemPort = FourCharCode('ecmp');
	eModemPrinterPort = FourCharCode('empp');
	eModem = FourCharCode('edmm');
	eMonitorOut = FourCharCode('ecmn');
	eMouse = FourCharCode('emou');
	eNuBusCard = FourCharCode('ednb');
	eNuBus = FourCharCode('enub');
	ePCcard = FourCharCode('ecpc');
	ePCIbus = FourCharCode('ecpi');
	ePCIcard = FourCharCode('edpi');
	ePDSslot = FourCharCode('ecpd');
	ePDScard = FourCharCode('epds');
	ePointingDevice = FourCharCode('edpd');
	ePostScript = FourCharCode('epps');
	ePPP = FourCharCode('eppp');
	ePrinterPort = FourCharCode('ecpp');
	ePrinter = FourCharCode('edpr');
	eSvideo = FourCharCode('epsv');
	eSCSI = FourCharCode('ecsc');
	eSerial = FourCharCode('epsr');
	eSpeakers = FourCharCode('edsp');
	eStorageDevice = FourCharCode('edst');
	eSVGA = FourCharCode('epsg');
	eTokenRing = FourCharCode('etok');
	eTrackball = FourCharCode('etrk');
	eTrackpad = FourCharCode('edtp');
	eUSB = FourCharCode('ecus');
	eVideoIn = FourCharCode('ecvi');
	eVideoMonitor = FourCharCode('edvm');
	eVideoOut = FourCharCode('ecvo');

const
{ AppleScript 1.3: Keystroke class }
	cKeystroke = FourCharCode('kprs');
	pKeystrokeKey = FourCharCode('kMsg');
	pModifiers = FourCharCode('kMod');
	pKeyKind = FourCharCode('kknd');
	eModifiers = FourCharCode('eMds');
	eOptionDown = FourCharCode('Kopt');
	eCommandDown = FourCharCode('Kcmd');
	eControlDown = FourCharCode('Kctl');
	eShiftDown = FourCharCode('Ksft');
	eCapsLockDown = FourCharCode('Kclk');
	eKeyKind = FourCharCode('ekst'); { }
                                        { Special keys all start with 'ks' }
	eEscapeKey = $6B733500; { Third byte is virtual key code byte        }
	eDeleteKey = $6B733300; { (see IM Mac Toolbox Essentials, pp. 2-43) }
	eTabKey = $6B733000;
	eReturnKey = $6B732400;
	eClearKey = $6B734700;
	eEnterKey = $6B734C00;
	eUpArrowKey = $6B737E00;
	eDownArrowKey = $6B737D00;
	eLeftArrowKey = $6B737B00;
	eRightArrowKey = $6B737C00;
	eHelpKey = $6B737200;
	eHomeKey = $6B737300;
	ePageUpKey = $6B737400;
	ePageDownKey = $6B737900;
	eForwardDelKey = $6B737500;
	eEndKey = $6B737700;
	eF1Key = $6B737A00;
	eF2Key = $6B737800;
	eF3Key = $6B736300;
	eF4Key = $6B737600;
	eF5Key = $6B736000;
	eF6Key = $6B736100;
	eF7Key = $6B736200;
	eF8Key = $6B736400;
	eF9Key = $6B736500;
	eF10Key = $6B736D00;
	eF11Key = $6B736700;
	eF12Key = $6B736F00;
	eF13Key = $6B736900;
	eF14Key = $6B736B00;
	eF15Key = $6B737100;

const
	keyAELaunchedAsLogInItem = FourCharCode('lgit'); { If present in a kAEOpenApplication event, application was launched as a login item and probably shouldn't open up untitled documents, etc. Mac OS X 10.4 and later. }
	keyAELaunchedAsServiceItem = FourCharCode('svit'); { If present in a kAEOpenApplication event, application was launched as a service item and probably shouldn't open up untitled documents, etc. Mac OS X 10.4 and later. }


{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
	kCaretPosition             = kTSMHiliteCaretPosition;
	kRawText                   = kTSMHiliteRawText;
	kSelectedRawText           = kTSMHiliteSelectedRawText;
	kConvertedText             = kTSMHiliteConvertedText;
	kSelectedConvertedText     = kTSMHiliteSelectedConvertedText;
	kBlockFillText             = kTSMHiliteBlockFillText;
	kOutlineText               = kTSMHiliteOutlineText;
	kSelectedText              = kTSMHiliteSelectedText;
=======
>>>>>>> origin/fixes_2.4

{$endc}  {OLDROUTINENAMES}

const
	keyAEHiliteRange = FourCharCode('hrng'); { typeTextRangeArray for System 7, typeHiliteRangeArray for System 8 }
	keyAEPinRange = FourCharCode('pnrg'); { typeTextRange for System 7, typeTextRegionRange for System 8   }
	keyAEClauseOffsets = FourCharCode('clau'); { typeOffsetArray for System 7, typeClauseOffsetArray for System 8 }
	keyAEOffset = FourCharCode('ofst'); { typeLongInteger for System 7, typeByteOffset for System 8  }
	keyAEPoint = FourCharCode('gpos'); { typePoint for System 7, typeQDPoint for System 8 }
	keyAELeftSide = FourCharCode('klef'); { typeBoolean }
	keyAERegionClass = FourCharCode('rgnc'); { typeShortInteger for System 7, typeRegionClass for System 8 }
	keyAEDragging = FourCharCode('bool'); { typeBoolean }


{$ifc OLDROUTINENAMES}
const
	keyAELeadingEdge = keyAELeftSide;

{$endc}  {OLDROUTINENAMES}

const
{ AppleScript 1.3: Unit types }
	typeMeters = FourCharCode('metr'); { Base Unit }
	typeInches = FourCharCode('inch');
	typeFeet = FourCharCode('feet');
	typeYards = FourCharCode('yard');
	typeMiles = FourCharCode('mile');
	typeKilometers = FourCharCode('kmtr');
	typeCentimeters = FourCharCode('cmtr');
	typeSquareMeters = FourCharCode('sqrm'); { Base Unit }
	typeSquareFeet = FourCharCode('sqft');
	typeSquareYards = FourCharCode('sqyd');
	typeSquareMiles = FourCharCode('sqmi');
	typeSquareKilometers = FourCharCode('sqkm');
	typeLiters = FourCharCode('litr'); { Base Unit }
	typeQuarts = FourCharCode('qrts');
	typeGallons = FourCharCode('galn');
	typeCubicMeters = FourCharCode('cmet'); { Base Unit }
	typeCubicFeet = FourCharCode('cfet');
	typeCubicInches = FourCharCode('cuin');
	typeCubicCentimeter = FourCharCode('ccmt');
	typeCubicYards = FourCharCode('cyrd');
	typeKilograms = FourCharCode('kgrm'); { Base Unit }
	typeGrams = FourCharCode('gram');
	typeOunces = FourCharCode('ozs ');
	typePounds = FourCharCode('lbs ');
	typeDegreesC = FourCharCode('degc'); { Base Unit }
	typeDegreesF = FourCharCode('degf');
	typeDegreesK = FourCharCode('degk');

const
{ AppleScript 1.3: Folder Actions }
	kFAServerApp = FourCharCode('ssrv'); { Creator code for Folder Actions Server}
	kDoFolderActionEvent = FourCharCode('fola'); { Event the Finder sends to the Folder Actions FBA}
	kFolderActionCode = FourCharCode('actn'); { Parameter that contains the Folder Action}
	kFolderOpenedEvent = FourCharCode('fopn'); { Value of kFolderActionCode parameter; sent to script as event}
	kFolderClosedEvent = FourCharCode('fclo');
	kFolderWindowMovedEvent = FourCharCode('fsiz');
	kFolderItemsAddedEvent = FourCharCode('fget');
	kFolderItemsRemovedEvent = FourCharCode('flos');
	kItemList = FourCharCode('flst'); { List parameter for added and removed items}
	kNewSizeParameter = FourCharCode('fnsz'); { Parameter for moved window}
	kFASuiteCode = FourCharCode('faco'); { Suite code for the following events}
	kFAAttachCommand = FourCharCode('atfa'); { Attach event id}
	kFARemoveCommand = FourCharCode('rmfa'); { Remove event id}
	kFAEditCommand = FourCharCode('edfa'); { Edit event id}
	kFAFileParam = FourCharCode('faal'); { Key for file parameter for Attach}
	kFAIndexParam = FourCharCode('indx'); { Key for index (0-based) parameter for Remove and Edit}

{ AppleScript 1.3 Internet Suite }
const
{ Suite code }
	kAEInternetSuite = FourCharCode('gurl');
	kAEISWebStarSuite = $575757BD;

const
{ Events }
	kAEISGetURL = FourCharCode('gurl');
	KAEISHandleCGI = FourCharCode('sdoc');

const
{ Classes }
	cURL = FourCharCode('url ');
	cInternetAddress = FourCharCode('IPAD');
	cHTML = FourCharCode('html');
	cFTPItem = FourCharCode('ftp ');

const
{ Parameters }
	kAEISHTTPSearchArgs = FourCharCode('kfor');
	kAEISPostArgs = FourCharCode('post');
	kAEISMethod = FourCharCode('meth');
	kAEISClientAddress = FourCharCode('addr');
	kAEISUserName = FourCharCode('user');
	kAEISPassword = FourCharCode('pass');
	kAEISFromUser = FourCharCode('frmu');
	kAEISServerName = FourCharCode('svnm');
	kAEISServerPort = FourCharCode('svpt');
	kAEISScriptName = FourCharCode('scnm');
	kAEISContentType = FourCharCode('ctyp');
	kAEISReferrer = FourCharCode('refr');
	kAEISUserAgent = FourCharCode('Agnt');
	kAEISAction = FourCharCode('Kact');
	kAEISActionPath = FourCharCode('Kapt');
	kAEISClientIP = FourCharCode('Kcip');
	kAEISFullRequest = FourCharCode('Kfrq');

const
{ Properties }
	pScheme = FourCharCode('pusc');
	pHost = FourCharCode('HOST');
	pPath = FourCharCode('FTPc');
	pUserName = FourCharCode('RAun');
	pUserPassword = FourCharCode('RApw');
	pDNSForm = FourCharCode('pDNS');
	pURL = FourCharCode('pURL');
	pTextEncoding = FourCharCode('ptxe');
	pFTPKind = FourCharCode('kind');

const
{ Scheme enumerations }
	eScheme = FourCharCode('esch');
	eurlHTTP = FourCharCode('http'); { RFC 2068 }
	eurlHTTPS = FourCharCode('htps');
	eurlFTP = FourCharCode('ftp '); { RFC 1738 }
	eurlMail = FourCharCode('mail'); { RFC 2638 }
	eurlFile = FourCharCode('file'); { RFC 1738 }
	eurlGopher = FourCharCode('gphr'); { RFC 1738 }
	eurlTelnet = FourCharCode('tlnt'); { RFC 1738 }
	eurlNews = FourCharCode('news'); { RFC 1738 }
	eurlSNews = FourCharCode('snws');
	eurlNNTP = FourCharCode('nntp'); { RFC 1738 }
	eurlMessage = FourCharCode('mess');
	eurlMailbox = FourCharCode('mbox');
	eurlMulti = FourCharCode('mult');
	eurlLaunch = FourCharCode('laun');
	eurlAFP = FourCharCode('afp ');
	eurlAT = FourCharCode('at  ');
	eurlEPPC = FourCharCode('eppc');
	eurlRTSP = FourCharCode('rtsp'); { RFC 2326 }
	eurlIMAP = FourCharCode('imap'); { RFC 2192 }
	eurlNFS = FourCharCode('unfs'); { RFC 2224 }
	eurlPOP = FourCharCode('upop'); { RFC 2384 }
	eurlLDAP = FourCharCode('uldp'); { RFC 2255 }
	eurlUnknown = FourCharCode('url?');

const
{ AppleScript 1.3: Connectivity Suite in aeut }
	kConnSuite = FourCharCode('macc');
	cDevSpec = FourCharCode('cdev');
	cAddressSpec = FourCharCode('cadr');
	cADBAddress = FourCharCode('cadb');
	cAppleTalkAddress = FourCharCode('cat ');
	cBusAddress = FourCharCode('cbus');
	cEthernetAddress = FourCharCode('cen ');
	cFireWireAddress = FourCharCode('cfw ');
	cIPAddress = FourCharCode('cip ');
	cLocalTalkAddress = FourCharCode('clt ');
	cSCSIAddress = FourCharCode('cscs');
	cTokenRingAddress = FourCharCode('ctok');
	cUSBAddress = FourCharCode('cusb'); { }
                                        { Properties }
	pDeviceType = FourCharCode('pdvt');
	pDeviceAddress = FourCharCode('pdva');
	pConduit = FourCharCode('pcon');
	pProtocol = FourCharCode('pprt'); { cde 4/27/98 was 'ppro' conflicted with DB suite }
	pATMachine = FourCharCode('patm');
	pATZone = FourCharCode('patz');
	pATType = FourCharCode('patt');
	pDottedDecimal = FourCharCode('pipd');
	pDNS = FourCharCode('pdns');
	pPort = FourCharCode('ppor');
	pNetwork = FourCharCode('pnet');
	pNode = FourCharCode('pnod');
	pSocket = FourCharCode('psoc');
	pSCSIBus = FourCharCode('pscb');
	pSCSILUN = FourCharCode('pslu'); { cde 5/22/98 per WWDC developer request }
                                        { Enumerations and enumerators }
	eDeviceType = FourCharCode('edvt');
	eAddressSpec = FourCharCode('eads');
	eConduit = FourCharCode('econ');
	eProtocol = FourCharCode('epro');
	eADB = FourCharCode('eadb');
	eAnalogAudio = FourCharCode('epau');
	eAppleTalk = FourCharCode('epat');
	eAudioLineIn = FourCharCode('ecai');
	eAudioLineOut = FourCharCode('ecal'); { cde 4/24/98 changed from 'ecao' to not conflict }
	eAudioOut = FourCharCode('ecao');
	eBus = FourCharCode('ebus');
	eCDROM = FourCharCode('ecd ');
	eCommSlot = FourCharCode('eccm');
	eDigitalAudio = FourCharCode('epda');
	eDisplay = FourCharCode('edds');
	eDVD = FourCharCode('edvd');
	eEthernet = FourCharCode('ecen');
	eFireWire = FourCharCode('ecfw');
	eFloppy = FourCharCode('efd ');
	eHD = FourCharCode('ehd ');
	eInfrared = FourCharCode('ecir');
	eIP = FourCharCode('epip');
	eIrDA = FourCharCode('epir');
	eIRTalk = FourCharCode('epit');
	eKeyboard = FourCharCode('ekbd');
	eLCD = FourCharCode('edlc');
	eLocalTalk = FourCharCode('eclt');
	eMacIP = FourCharCode('epmi');
	eMacVideo = FourCharCode('epmv');
	eMicrophone = FourCharCode('ecmi');
	eModemPort = FourCharCode('ecmp');
	eModemPrinterPort = FourCharCode('empp');
	eModem = FourCharCode('edmm');
	eMonitorOut = FourCharCode('ecmn');
	eMouse = FourCharCode('emou');
	eNuBusCard = FourCharCode('ednb');
	eNuBus = FourCharCode('enub');
	ePCcard = FourCharCode('ecpc');
	ePCIbus = FourCharCode('ecpi');
	ePCIcard = FourCharCode('edpi');
	ePDSslot = FourCharCode('ecpd');
	ePDScard = FourCharCode('epds');
	ePointingDevice = FourCharCode('edpd');
	ePostScript = FourCharCode('epps');
	ePPP = FourCharCode('eppp');
	ePrinterPort = FourCharCode('ecpp');
	ePrinter = FourCharCode('edpr');
	eSvideo = FourCharCode('epsv');
	eSCSI = FourCharCode('ecsc');
	eSerial = FourCharCode('epsr');
	eSpeakers = FourCharCode('edsp');
	eStorageDevice = FourCharCode('edst');
	eSVGA = FourCharCode('epsg');
	eTokenRing = FourCharCode('etok');
	eTrackball = FourCharCode('etrk');
	eTrackpad = FourCharCode('edtp');
	eUSB = FourCharCode('ecus');
	eVideoIn = FourCharCode('ecvi');
	eVideoMonitor = FourCharCode('edvm');
	eVideoOut = FourCharCode('ecvo');

const
{ AppleScript 1.3: Keystroke class }
	cKeystroke = FourCharCode('kprs');
	pKeystrokeKey = FourCharCode('kMsg');
	pModifiers = FourCharCode('kMod');
	pKeyKind = FourCharCode('kknd');
	eModifiers = FourCharCode('eMds');
	eOptionDown = FourCharCode('Kopt');
	eCommandDown = FourCharCode('Kcmd');
	eControlDown = FourCharCode('Kctl');
	eShiftDown = FourCharCode('Ksft');
	eCapsLockDown = FourCharCode('Kclk');
	eKeyKind = FourCharCode('ekst'); { }
                                        { Special keys all start with 'ks' }
	eEscapeKey = $6B733500; { Third byte is virtual key code byte        }
	eDeleteKey = $6B733300; { (see IM Mac Toolbox Essentials, pp. 2-43) }
	eTabKey = $6B733000;
	eReturnKey = $6B732400;
	eClearKey = $6B734700;
	eEnterKey = $6B734C00;
	eUpArrowKey = $6B737E00;
	eDownArrowKey = $6B737D00;
	eLeftArrowKey = $6B737B00;
	eRightArrowKey = $6B737C00;
	eHelpKey = $6B737200;
	eHomeKey = $6B737300;
	ePageUpKey = $6B737400;
	ePageDownKey = $6B737900;
	eForwardDelKey = $6B737500;
	eEndKey = $6B737700;
	eF1Key = $6B737A00;
	eF2Key = $6B737800;
	eF3Key = $6B736300;
	eF4Key = $6B737600;
	eF5Key = $6B736000;
	eF6Key = $6B736100;
	eF7Key = $6B736200;
	eF8Key = $6B736400;
	eF9Key = $6B736500;
	eF10Key = $6B736D00;
	eF11Key = $6B736700;
	eF12Key = $6B736F00;
	eF13Key = $6B736900;
	eF14Key = $6B736B00;
	eF15Key = $6B737100;

const
	keyAELaunchedAsLogInItem = FourCharCode('lgit'); { If present in a kAEOpenApplication event, application was launched as a login item and probably shouldn't open up untitled documents, etc. Mac OS X 10.4 and later. }
	keyAELaunchedAsServiceItem = FourCharCode('svit'); { If present in a kAEOpenApplication event, application was launched as a service item and probably shouldn't open up untitled documents, etc. Mac OS X 10.4 and later. }


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
