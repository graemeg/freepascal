{==================================================================================================
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    File:       CoreAudio/CoreAudioTypes.h

    Contains:   Definitions types common to the Core Audio APIs

    Copyright:  (c) 1985-2010 by Apple, Inc., all rights reserved.

    Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:

                     http://bugs.freepascal.org

==================================================================================================}
{  Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
	File:       CoreAudio/CoreAudioTypes.h

	Contains:   Definitions types common to the Core Audio APIs

	Copyright:  (c) 1985-2008 by Apple Inc., all rights reserved.

	Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:

=======
	File:       CoreAudio/CoreAudioTypes.h

	Contains:   Definitions types common to the Core Audio APIs

	Copyright:  (c) 1985-2008 by Apple Inc., all rights reserved.

	Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:

>>>>>>> origin/cpstrnew
                     http://www.freepascal.org/bugs.html

==================================================================================================}
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
	File:       CoreAudio/CoreAudioTypes.h

	Contains:   Definitions types common to the Core Audio APIs

	Copyright:  (c) 1985-2008 by Apple Inc., all rights reserved.

	Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:

                     http://www.freepascal.org/bugs.html

==================================================================================================}
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
	File:       CoreAudio/CoreAudioTypes.h

	Contains:   Definitions types common to the Core Audio APIs

	Copyright:  (c) 1985-2008 by Apple Inc., all rights reserved.

	Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:

                     http://www.freepascal.org/bugs.html

==================================================================================================}
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
     File:       CoreAudio/CoreAudioTypes.h
=======
	File:       CoreAudio/CoreAudioTypes.h
>>>>>>> origin/fixes_2.4

	Contains:   Definitions types common to the Core Audio APIs

	Copyright:  (c) 1985-2008 by Apple Inc., all rights reserved.

	Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:

                     http://www.freepascal.org/bugs.html

==================================================================================================}
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit CoreAudioTypes;
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
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
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
=======
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
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
{$endc} {not MACOSALLINCLUDE}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}

{!
    @header CoreAudioTypes
    This header defines the types and constants that all the CoreAudio APIs have in common.
}

//==================================================================================================

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
const
	COREAUDIOTYPES_VERSION = 1051;


{$ifc not defined(CA_PREFER_FIXED_POINT)}
    {$ifc TARGET_OS_IPHONE}
        {$ifc (TARGET_CPU_X86 or TARGET_CPU_X86_64 or TARGET_CPU_PPC or TARGET_CPU_PPC64) and not TARGET_IPHONE_SIMULATOR}
            {$setc CA_PREFER_FIXED_POINT := 0}
        {$elsec}
            {$setc CA_PREFER_FIXED_POINT := 1}
        {$endc}
    {$elsec}
        {$setc CA_PREFER_FIXED_POINT := 0}
    {$endc}
{$endc}


//==================================================================================================
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{!
    @enum           General Audio error codes
    @abstract       These are the error codes returned from the APIs found through Core Audio related frameworks.
    @constant       kAudio_UnimplementedError 
                        Unimplemented core routine.
    @constant       kAudio_FileNotFoundError 
                        File not found.
    @constant       kAudio_ParamError 
                        Error in user parameter list.
    @constant       kAudio_MemFullError 
                        Not enough room in heap zone.
}

const
	kAudio_UnimplementedError = -4;
	kAudio_FileNotFoundError = -43;
	kAudio_ParamError = -50;
	kAudio_MemFullError = -108;
      
//==================================================================================================
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{!
    @struct         AudioValueRange
    @abstract       This structure holds a pair of numbers that represent a continuous range of
                    values.
    @field          mMinimum
                        The minimum value.
    @field          mMaximum
                        The maximum value.
}
type
	AudioValueRange = record
		mMinimum: Float64;
		mMaximum: Float64;
	end;

{!
    @struct         AudioValueTranslation
    @abstract       This stucture holds the buffers necessary for translation operations.
    @field          mInputData
                        The buffer containing the data to be translated.
    @field          mInputDataSize
                        The number of bytes in the buffer pointed at by mInputData.
    @field          mOutputData
                        The buffer to hold the result of the translation.
    @field          mOutputDataSize
                        The number of bytes in the buffer pointed at by mOutputData.
}
type
	AudioValueTranslation = record
		mInputData: UnivPtr;
		mInputDataSize: UInt32;
		mOutputData: UnivPtr;
		mOutputDataSize: UInt32;
	end;

{!
    @struct         AudioBuffer
    @abstract       A structure to hold a buffer of audio data.
    @field          mNumberChannels
                        The number of interleaved channels in the buffer.
    @field          mDataByteSize
                        The number of bytes in the buffer pointed at by mData.
    @field          mData
                        A pointer to the buffer of audio data.
}
type
	AudioBuffer = record
		mNumberChannels: UInt32;
		mDataByteSize: UInt32;
		mData: UnivPtr;
	end;

{!
    @struct         AudioBufferList
    @abstract       A variable length array of AudioBuffer structures.
    @field          mNumberBuffers
                        The number of AudioBuffers in the mBuffers array.
    @field          mBuffers
                        A variable length array of AudioBuffers.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AudioBufferListPtr = ^AudioBufferList;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AudioBufferListPtrPtr = ^AudioBufferListPtr;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	AudioBufferList = record
		mNumberBuffers: UInt32;
		mBuffers: array[0..0] of AudioBuffer;
	end;

{!
    @typedef        AudioSampleType
    @abstract       The canonical audio sample type used by the various CoreAudio APIs
}
{$ifc not CA_PREFER_FIXED_POINT}
type
	AudioSampleType = Float32;
	AudioUnitSampleType = Float32;
{$elsec} {not CA_PREFER_FIXED_POINT}
type
	AudioSampleType = SInt16;
	AudioUnitSampleType = SInt32;
const
	kAudioUnitSampleFractionBits = 24;
{$endc} {not CA_PREFER_FIXED_POINT}

{!
=======
=======
>>>>>>> origin/fixes_2_2
=======
	AudioBufferListPtr = ^AudioBufferList;
>>>>>>> origin/fixes_2.4
	AudioBufferList = record
		mNumberBuffers: UInt32;
		mBuffers: array[0..0] of AudioBuffer;
	end;

{!
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    @typedef        AudioSampleType
    @abstract       The canonical audio sample type used by the various CoreAudio APIs
}
{$ifc not CA_PREFER_FIXED_POINT}
type
	AudioSampleType = Float32;
	AudioUnitSampleType = Float32;
{$elsec} {not CA_PREFER_FIXED_POINT}
type
	AudioSampleType = SInt16;
	AudioUnitSampleType = SInt32;
const
	kAudioUnitSampleFractionBits = 24;
{$endc} {not CA_PREFER_FIXED_POINT}

{!
>>>>>>> origin/fixes_2.4
    @struct         AudioStreamBasicDescription
    @abstract       This structure encapsulates all the information for describing the basic
                    format properties of a stream of audio data.
    @discussion     This structure is sufficient to describe any constant bit rate format that  has
                    channels that are the same size. Extensions are required for variable bit rate
                    data and for constant bit rate data where the channels have unequal sizes.
                    However, where applicable, the appropriate fields will be filled out correctly
                    for these kinds of formats (the extra data is provided via separate properties).
                    In all fields, a value of 0 indicates that the field is either unknown, not
                    applicable or otherwise is inapproprate for the format and should be ignored.
                    Note that 0 is still a valid value for most formats in the mFormatFlags field.

                    In audio data a frame is one sample across all channels. In non-interleaved
                    audio, the per frame fields identify one channel. In interleaved audio, the per
                    frame fields identify the set of n channels. In uncompressed audio, a Packet is
                    one frame, (mFramesPerPacket == 1). In compressed audio, a Packet is an
                    indivisible chunk of compressed data, for example an AAC packet will contain
                    1024 sample frames.
    @field          mSampleRate
                        The number of sample frames per second of the data in the stream.
    @field          mFormatID
                        A four char code indicating the general kind of data in the stream.
    @field          mFormatFlags
                        Flags specific to each format.
    @field          mBytesPerPacket
                        The number of bytes in a packet of data.
    @field          mFramesPerPacket
                        The number of sample frames in each packet of data.
    @field          mBytesPerFrame
                        The number of bytes in a single sample frame of data.
    @field          mChannelsPerFrame
                        The number of channels in each frame of data.
    @field          mBitsPerChannel
                        The number of bits of sample data for each channel in a frame of data.
    @field          mReserved
                        Pads the structure out to force an even 8 byte alignment.
}
type
	AudioStreamBasicDescription = record
		mSampleRate: Float64;
		mFormatID: UInt32;
		mFormatFlags: UInt32;
		mBytesPerPacket: UInt32;
		mFramesPerPacket: UInt32;
		mBytesPerFrame: UInt32;
		mChannelsPerFrame: UInt32;
		mBitsPerChannel: UInt32;
		mReserved: UInt32;
	end;

{!
    @enum           AudioStreamBasicDescription Constants
    @abstract       Constants for use with AudioStreamBasicDescription
    @constant       kAudioStreamAnyRate
                        The format can use any sample rate. Note that this constant can only appear
                        in listings of supported formats. It will never appear in a current format.
}
const
	kAudioStreamAnyRate = 0;

{!
    @enum           Format IDs
    @abstract       The four char code IDs used to identify individual formats of audio data.
    @constant       kAudioFormatLinearPCM
                        Linear PCM, uses the standard flags.
    @constant       kAudioFormatAC3
                        AC-3, has no flags.
    @constant       kAudioFormat60958AC3
                        AC-3 packaged for transport over an IEC 60958 compliant digital audio
                        interface. Uses the standard flags.
    @constant       kAudioFormatAppleIMA4
                        Apples implementation of IMA 4:1 ADPCM, has no flags.
    @constant       kAudioFormatMPEG4AAC
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        MPEG-4 Low Complexity AAC audio object, has no flags.
    @constant       kAudioFormatMPEG4CELP
                        MPEG-4 CELP audio object, has no flags.
    @constant       kAudioFormatMPEG4HVXC
                        MPEG-4 HVXC audio object, has no flags.
    @constant       kAudioFormatMPEG4TwinVQ
                        MPEG-4 TwinVQ audio object type, has no flags.
=======
=======
>>>>>>> origin/fixes_2_2
                        MPEG-4 AAC, the flags field contains the MPEG-4 audio object type constant
                        indicating the specific kind of data.
=======
                        MPEG-4 Low Complexity AAC audio object, has no flags.
>>>>>>> origin/fixes_2.4
    @constant       kAudioFormatMPEG4CELP
                        MPEG-4 CELP audio object, has no flags.
    @constant       kAudioFormatMPEG4HVXC
                        MPEG-4 HVXC audio object, has no flags.
    @constant       kAudioFormatMPEG4TwinVQ
<<<<<<< HEAD
                        MPEG-4 TwinVQ, the flags field contains the MPEG-4 audio object type
                        constant indicating the specific kind of data.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
                        MPEG-4 TwinVQ audio object type, has no flags.
>>>>>>> origin/fixes_2.4
    @constant       kAudioFormatMACE3
                        MACE 3:1, has no flags.
    @constant       kAudioFormatMACE6
                        MACE 6:1, has no flags.
    @constant       kAudioFormatULaw
                        µLaw 2:1, has no flags.
    @constant       kAudioFormatALaw
                        aLaw 2:1, has no flags.
    @constant       kAudioFormatQDesign
                        QDesign music, has no flags
    @constant       kAudioFormatQDesign2
                        QDesign2 music, has no flags
    @constant       kAudioFormatQUALCOMM
                        QUALCOMM PureVoice, has no flags
    @constant       kAudioFormatMPEGLayer1
                        MPEG-1/2, Layer 1 audio, has no flags
    @constant       kAudioFormatMPEGLayer2
                        MPEG-1/2, Layer 2 audio, has no flags
    @constant       kAudioFormatMPEGLayer3
                        MPEG-1/2, Layer 3 audio, has no flags
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
    @constant       kAudioFormatDVAudio
                        DV Audio, has no flags
    @constant       kAudioFormatVariableDurationDVAudio
                        Variable duration DV Audio, has no flags.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
    @constant       kAudioFormatTimeCode
                        A stream of IOAudioTimeStamps, uses the IOAudioTimeStamp flags (see
                        IOKit/audio/IOAudioTypes.h).
    @constant       kAudioFormatMIDIStream
                        A stream of MIDIPacketLists where the time stamps in the MIDIPacketList are
                        sample offsets in the stream. The mSampleRate field is used to describe how
                        time is passed in this kind of stream and an AudioUnit that receives or
                        generates this stream can use this sample rate, the number of frames it is
                        rendering and the sample offsets within the MIDIPacketList to define the
                        time for any MIDI event within this list. It has no flags.
    @constant       kAudioFormatParameterValueStream
                        A "side-chain" of Float32 data that can be fed or generated by an AudioUnit
                        and is used to send a high density of parameter value control information.
                        An AU will typically run a ParameterValueStream at either the sample rate of
                        the AudioUnit's audio data, or some integer divisor of this (say a half or a
                        third of the sample rate of the audio). The Sample Rate of the ASBD
                        describes this relationship. It has no flags.
    @constant       kAudioFormatAppleLossless
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
                        Apple Lossless, the flags indicate the bit depth of the source material.
    @constant       kAudioFormatMPEG4AAC_HE
                        MPEG-4 High Efficiency AAC audio object, has no flags.
    @constant       kAudioFormatMPEG4AAC_LD
                        MPEG-4 AAC Low Delay audio object, has no flags.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @constant       kAudioFormatMPEG4AAC_ELD
                        MPEG-4 AAC Enhanced Low Delay audio object, has no flags. This is the formatID of
                        the base layer without the SBR extension. See also kAudioFormatMPEG4AAC_ELD_SBR
    @constant       kAudioFormatMPEG4AAC_ELD_SBR
                        MPEG-4 AAC Enhanced Low Delay audio object with SBR extension layer, has no flags.
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
    @constant       kAudioFormatMPEG4AAC_HE_V2
                        MPEG-4 High Efficiency AAC Version 2 audio object, has no flags. 
    @constant       kAudioFormatMPEG4AAC_Spatial
                        MPEG-4 Spatial Audio audio object, has no flags.
    @constant       kAudioFormatAMR
                        The AMR Narrow Band speech codec.
    @constant       kAudioFormatAudible
                        The format used for Audible audio books. It has no flags.
    @constant       kAudioFormatiLBC
                        The iLBC narrow band speech codec. It has no flags.
    @constant       kAudioFormatDVIIntelIMA
                        DVI/Intel IMA ADPCM - ACM code 17.
    @constant       kAudioFormatMicrosoftGSM
                        Microsoft GSM 6.10 - ACM code 49.
    @constant       kAudioFormatAES3
                        This format is defined by AES3-2003, and adopted into MXF and MPEG-2
                        containers and SDTI transport streams with SMPTE specs 302M-2002 and
                        331M-2000. It has no flags.
<<<<<<< HEAD
=======
                        Apple Lossless, has no flags.
>>>>>>> graemeg/fixes_2_2
=======
                        Apple Lossless, has no flags.
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
}
const
	kAudioFormatLinearPCM = FourCharCode('lpcm');
	kAudioFormatAC3 = FourCharCode('ac-3');
	kAudioFormat60958AC3 = FourCharCode('cac3');
	kAudioFormatAppleIMA4 = FourCharCode('ima4');
	kAudioFormatMPEG4AAC = FourCharCode('aac ');
	kAudioFormatMPEG4CELP = FourCharCode('celp');
	kAudioFormatMPEG4HVXC = FourCharCode('hvxc');
	kAudioFormatMPEG4TwinVQ = FourCharCode('twvq');
	kAudioFormatMACE3 = FourCharCode('MAC3');
	kAudioFormatMACE6 = FourCharCode('MAC6');
	kAudioFormatULaw = FourCharCode('ulaw');
	kAudioFormatALaw = FourCharCode('alaw');
	kAudioFormatQDesign = FourCharCode('QDMC');
	kAudioFormatQDesign2 = FourCharCode('QDM2');
	kAudioFormatQUALCOMM = FourCharCode('Qclp');
	kAudioFormatMPEGLayer1 = FourCharCode('.mp1');
	kAudioFormatMPEGLayer2 = FourCharCode('.mp2');
	kAudioFormatMPEGLayer3 = FourCharCode('.mp3');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
	kAudioFormatDVAudio = FourCharCode('dvca');
	kAudioFormatVariableDurationDVAudio = FourCharCode('vdva');
>>>>>>> graemeg/fixes_2_2
=======
	kAudioFormatDVAudio = FourCharCode('dvca');
	kAudioFormatVariableDurationDVAudio = FourCharCode('vdva');
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kAudioFormatTimeCode = FourCharCode('time');
	kAudioFormatMIDIStream = FourCharCode('midi');
	kAudioFormatParameterValueStream = FourCharCode('apvs');
	kAudioFormatAppleLossless = FourCharCode('alac');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioFormatMPEG4AAC_HE = FourCharCode('aach');
	kAudioFormatMPEG4AAC_LD = FourCharCode('aacl');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioFormatMPEG4AAC_ELD = FourCharCode('aace');
	kAudioFormatMPEG4AAC_ELD_SBR = FourCharCode('aacf');
	kAudioFormatMPEG4AAC_ELD_V2 = FourCharCode('aacg');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
	kAudioFormatMPEG4AAC_HE = FourCharCode('aach');
	kAudioFormatMPEG4AAC_LD = FourCharCode('aacl');
>>>>>>> origin/fixes_2.4
	kAudioFormatMPEG4AAC_HE_V2 = FourCharCode('aacp');
	kAudioFormatMPEG4AAC_Spatial = FourCharCode('aacs');
	kAudioFormatAMR = FourCharCode('samr');
	kAudioFormatAudible = FourCharCode('AUDB');
	kAudioFormatiLBC = FourCharCode('ilbc');
	kAudioFormatDVIIntelIMA = $6D730011;
	kAudioFormatMicrosoftGSM = $6D730031;
	kAudioFormatAES3 = FourCharCode('aes3');
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{!
    @enum           Standard Flag Values for AudioStreamBasicDescription
    @abstract       These are the standard flags for use in the mFormatFlags field of the
                    AudioStreamBasicDescription structure.
    @discussion     Typically, when an ASBD is being used, the fields describe the complete layout
                    of the sample data in the buffers that are represented by this description -
                    where typically those buffers are represented by an AudioBuffer that is
                    contained in an AudioBufferList.

                    However, when an ASBD has the kAudioFormatFlagIsNonInterleaved flag, the
                    AudioBufferList has a different structure and semantic. In this case, the ASBD
                    fields will describe the format of ONE of the AudioBuffers that are contained in
                    the list, AND each AudioBuffer in the list is determined to have a single (mono)
                    channel of audio data. Then, the ASBD's mChannelsPerFrame will indicate the
                    total number of AudioBuffers that are contained within the AudioBufferList -
                    where each buffer contains one channel. This is used primarily with the
                    AudioUnit (and AudioConverter) representation of this list - and won't be found
                    in the AudioHardware usage of this structure.
    @constant       kAudioFormatFlagIsFloat
                        Set for floating point, clear for integer.
    @constant       kAudioFormatFlagIsBigEndian
                        Set for big endian, clear for little endian.
    @constant       kAudioFormatFlagIsSignedInteger
                        Set for signed integer, clear for unsigned integer. This is only valid if
                        kAudioFormatFlagIsFloat is clear.
    @constant       kAudioFormatFlagIsPacked
                        Set if the sample bits occupy the entire available bits for the channel,
<<<<<<< HEAD
<<<<<<< HEAD
                        clear if they are high or low aligned within the channel. Note that even if
                        this flag is clear, it is implied that this flag is set if the
                        AudioStreamBasicDescription is filled out such that the fields have the
                        following relationship:
                           ((mBitsPerSample / 8) * mChannelsPerFrame) == mBytesPerFrame
=======
                        clear if they are high or low aligned within the channel.
>>>>>>> graemeg/fixes_2_2
=======
                        clear if they are high or low aligned within the channel.
>>>>>>> origin/fixes_2_2
    @constant       kAudioFormatFlagIsAlignedHigh
                        Set if the sample bits are placed into the high bits of the channel, clear
                        for low bit placement. This is only valid if kAudioFormatFlagIsPacked is
                        clear.
    @constant       kAudioFormatFlagIsNonInterleaved
                        Set if the samples for each channel are located contiguously and the
                        channels are layed out end to end, clear if the samples for each frame are
                        layed out contiguously and the frames layed out end to end.
    @constant       kAudioFormatFlagIsNonMixable
                        Set to indicate when a format is non-mixable. Note that this flag is only
                        used when interacting with the HAL's stream format information. It is not a
                        valid flag for any other uses.
    @constant       kAudioFormatFlagsAreAllClear
                        Set if all the flags would be clear in order to preserve 0 as the wild card
                        value.
    @constant       kLinearPCMFormatFlagIsFloat
                        Synonym for kAudioFormatFlagIsFloat.
    @constant       kLinearPCMFormatFlagIsBigEndian
                        Synonym for kAudioFormatFlagIsBigEndian.
    @constant       kLinearPCMFormatFlagIsSignedInteger
                        Synonym for kAudioFormatFlagIsSignedInteger.
    @constant       kLinearPCMFormatFlagIsPacked
                        Synonym for kAudioFormatFlagIsPacked.
    @constant       kLinearPCMFormatFlagIsAlignedHigh
                        Synonym for kAudioFormatFlagIsAlignedHigh.
    @constant       kLinearPCMFormatFlagIsNonInterleaved
                        Synonym for kAudioFormatFlagIsNonInterleaved.
    @constant       kLinearPCMFormatFlagIsNonMixable
                        Synonym for kAudioFormatFlagIsNonMixable.
    @constant       kLinearPCMFormatFlagsAreAllClear
                        Synonym for kAudioFormatFlagsAreAllClear.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
    @constant       kLinearPCMFormatFlagsSampleFractionShift
                        The linear PCM flags contain a 6-bit bitfield indicating that an integer
                        format is to be interpreted as fixed point. The value indicates the number
                        of bits are used to represent the fractional portion of each sample value.
                        This constant indicates the bit position (counting from the right) of the
                        bitfield in mFormatFlags.
    @constant       kLinearPCMFormatFlagsSampleFractionMask
                        number_fractional_bits = (mFormatFlags & 
                        kLinearPCMFormatFlagsSampleFractionMask) >>
                        kLinearPCMFormatFlagsSampleFractionShift
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
    @constant       kAppleLosslessFormatFlag_16BitSourceData
                        This flag is set for Apple Lossless data that was sourced from 16 bit native
                        endian signed integer data.
    @constant       kAppleLosslessFormatFlag_20BitSourceData
                        This flag is set for Apple Lossless data that was sourced from 20 bit native
                        endian signed integer data aligned high in 24 bits.
    @constant       kAppleLosslessFormatFlag_24BitSourceData
                        This flag is set for Apple Lossless data that was sourced from 24 bit native
                        endian signed integer data.
    @constant       kAppleLosslessFormatFlag_32BitSourceData
                        This flag is set for Apple Lossless data that was sourced from 32 bit native
                        endian signed integer data.
}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kAudioFormatFlagIsFloat = 1 shl 0;     // 0x1
	kAudioFormatFlagIsBigEndian = 1 shl 1;     // 0x2
	kAudioFormatFlagIsSignedInteger = 1 shl 2;     // 0x4
	kAudioFormatFlagIsPacked = 1 shl 3;     // 0x8
	kAudioFormatFlagIsAlignedHigh = 1 shl 4;     // 0x10
	kAudioFormatFlagIsNonInterleaved = 1 shl 5;     // 0x20
	kAudioFormatFlagIsNonMixable = 1 shl 6;     // 0x40
<<<<<<< HEAD
	kAudioFormatFlagsAreAllClear = 1 shl 31;
=======
=======
>>>>>>> origin/fixes_2_2
	kAudioFormatFlagIsFloat = 1 shl 0;
	kAudioFormatFlagIsBigEndian = 1 shl 1;
	kAudioFormatFlagIsSignedInteger = 1 shl 2;
	kAudioFormatFlagIsPacked = 1 shl 3;
	kAudioFormatFlagIsAlignedHigh = 1 shl 4;
	kAudioFormatFlagIsNonInterleaved = 1 shl 5;
	kAudioFormatFlagIsNonMixable = 1 shl 6;
	kAudioFormatFlagsAreAllClear = 1 shl 31;
	
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kAudioFormatFlagsAreAllClear = 1 shl 31;
>>>>>>> origin/fixes_2.4
	kLinearPCMFormatFlagIsFloat = kAudioFormatFlagIsFloat;
	kLinearPCMFormatFlagIsBigEndian = kAudioFormatFlagIsBigEndian;
	kLinearPCMFormatFlagIsSignedInteger = kAudioFormatFlagIsSignedInteger;
	kLinearPCMFormatFlagIsPacked = kAudioFormatFlagIsPacked;
	kLinearPCMFormatFlagIsAlignedHigh = kAudioFormatFlagIsAlignedHigh;
	kLinearPCMFormatFlagIsNonInterleaved = kAudioFormatFlagIsNonInterleaved;
	kLinearPCMFormatFlagIsNonMixable = kAudioFormatFlagIsNonMixable;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kLinearPCMFormatFlagsSampleFractionShift = 7;
	kLinearPCMFormatFlagsSampleFractionMask = $3F shl kLinearPCMFormatFlagsSampleFractionShift;
	kLinearPCMFormatFlagsAreAllClear = kAudioFormatFlagsAreAllClear;
=======
	kLinearPCMFormatFlagsAreAllClear = kAudioFormatFlagsAreAllClear;
	
>>>>>>> graemeg/fixes_2_2
=======
	kLinearPCMFormatFlagsAreAllClear = kAudioFormatFlagsAreAllClear;
	
>>>>>>> origin/fixes_2_2
=======
	kLinearPCMFormatFlagsSampleFractionShift = 7;
	kLinearPCMFormatFlagsSampleFractionMask = $3F shl kLinearPCMFormatFlagsSampleFractionShift;
	kLinearPCMFormatFlagsAreAllClear = kAudioFormatFlagsAreAllClear;
>>>>>>> origin/fixes_2.4
	kAppleLosslessFormatFlag_16BitSourceData = 1;
	kAppleLosslessFormatFlag_20BitSourceData = 2;
	kAppleLosslessFormatFlag_24BitSourceData = 3;
	kAppleLosslessFormatFlag_32BitSourceData = 4;

{!
    @enum           Commonly Used Combinations of ASBD Flags
    @abstract       Some commonly used combinations of flags for AudioStreamBasicDescriptions.
    @constant       kAudioFormatFlagsNativeEndian
                        Defined to set or clear kAudioFormatFlagIsBigEndian depending on the
                        endianness of the processor at build time.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
    @constant       kAudioFormatFlagsCanonical
                        The flags for the canonical audio sample type. This matches AudioSampleType.
    @constant       kAudioFormatFlagsAudioUnitCanonical
                        The flags for the canonical audio unit sample type. This matches
                        AudioUnitSampleType.
<<<<<<< HEAD
    @constant       kAudioFormatFlagsNativeFloatPacked
                        The flags for fully packed, native endian floating point data.
=======
    @constant       kAudioFormatFlagsNativeFloatPacked
                        The flags for the canonical format of fully packed, native endian floating
                        point data.
>>>>>>> graemeg/fixes_2_2
=======
    @constant       kAudioFormatFlagsNativeFloatPacked
                        The flags for the canonical format of fully packed, native endian floating
                        point data.
>>>>>>> origin/fixes_2_2
=======
    @constant       kAudioFormatFlagsNativeFloatPacked
                        The flags for fully packed, native endian floating point data.
>>>>>>> origin/fixes_2.4
}
const
{$ifc TARGET_RT_BIG_ENDIAN}
	kAudioFormatFlagsNativeEndian = kAudioFormatFlagIsBigEndian;
{$elsec}
	kAudioFormatFlagsNativeEndian = 0;
{$endc}  {TARGET_RT_BIG_ENDIAN}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$ifc not CA_PREFER_FIXED_POINT}
	kAudioFormatFlagsCanonical = kAudioFormatFlagIsFloat or kAudioFormatFlagsNativeEndian or kAudioFormatFlagIsPacked;
	kAudioFormatFlagsAudioUnitCanonical = kAudioFormatFlagIsFloat or kAudioFormatFlagsNativeEndian or kAudioFormatFlagIsPacked or kAudioFormatFlagIsNonInterleaved;
{$elsec}
	kAudioFormatFlagsCanonical = kAudioFormatFlagIsSignedInteger or kAudioFormatFlagsNativeEndian or kAudioFormatFlagIsPacked;
	kAudioFormatFlagsAudioUnitCanonical = kAudioFormatFlagIsSignedInteger or kAudioFormatFlagsNativeEndian or kAudioFormatFlagIsPacked or kAudioFormatFlagIsNonInterleaved or (kAudioUnitSampleFractionBits shl kLinearPCMFormatFlagsSampleFractionShift);
{$endc}
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kAudioFormatFlagsNativeFloatPacked = kAudioFormatFlagIsFloat or kAudioFormatFlagsNativeEndian or kAudioFormatFlagIsPacked;

{!
    @defined    TestAudioFormatNativeEndian
    @abstract   A macro for checking if an ASBD indicates native endian linear PCM data.
}
// #define TestAudioFormatNativeEndian(f)  ((f.mFormatID == kAudioFormatLinearPCM) && ((f.mFormatFlags & kAudioFormatFlagIsBigEndian) == kAudioFormatFlagsNativeEndian))

{!
    @function   IsAudioFormatNativeEndian
    @abstract   A C++ inline function for checking if an ASBD indicates native endian linear PCM
                data.
    @param      f
                    The AudioStreamBasicDescription to examine.
    @result     Whether or not the ASBD indicates native endian linear PCM data.
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{!
    @function   CalculateLPCMFlags
    @abstract   A C++ inline function for calculating the mFormatFlags for linear PCM data. Note
                that this function does not support specifying sample formats that are either
                unsigned integer or low-aligned.
    @param      inSampleRate
                    
    @param      inValidBitsPerChannel
                    The number of valid bits in each sample.
    @param      inTotalBitsPerChannel
                    The total number of bits in each sample.
    @param      inIsFloat
                    Whether or not the samples are represented with floating point numbers.
    @param      isIsBigEndian
                    Whether the samples are big endian or little endian.
    @result     A UInt32 containing the format flags.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
}

{!
    @function   FillOutASBDForLPCM
    @abstract   A C++ inline function for filling out an AudioStreamBasicDescription to describe
                linear PCM data. Note that this function does not support specifying sample formats
                that are either unsigned integer or low-aligned.
    @param      outASBD
                    The AudioStreamBasicDescription to fill out.
    @param      inSampleRate
                    The number of sample frames per second of the data in the stream.
    @param      inChannelsPerFrame
                    The number of channels in each frame of data.
    @param      inValidBitsPerChannel
                    The number of valid bits in each sample.
    @param      inTotalBitsPerChannel
                    The total number of bits in each sample.
    @param      inIsFloat
                    Whether or not the samples are represented with floating point numbers.
    @param      isIsBigEndian
                    Whether the samples are big endian or little endian.
}

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/cpstrnew
}

{!
    @function   FillOutASBDForLPCM
    @abstract   A C++ inline function for filling out an AudioStreamBasicDescription to describe
                linear PCM data. Note that this function does not support specifying sample formats
                that are either unsigned integer or low-aligned.
    @param      outASBD
                    The AudioStreamBasicDescription to fill out.
    @param      inSampleRate
                    The number of sample frames per second of the data in the stream.
    @param      inChannelsPerFrame
                    The number of channels in each frame of data.
    @param      inValidBitsPerChannel
                    The number of valid bits in each sample.
    @param      inTotalBitsPerChannel
                    The total number of bits in each sample.
    @param      inIsFloat
                    Whether or not the samples are represented with floating point numbers.
    @param      isIsBigEndian
                    Whether the samples are big endian or little endian.
}

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
{! @enum           MPEG-4 Audio Object IDs
    @abstract       Constants that describe the various kinds of MPEG-4 audio data.
    @discussion     These constants are used in the flags field of an AudioStreamBasicDescription
                    that describes an MPEG-4 audio stream.
}
const
	kMPEG4Object_AAC_Main = 1;
	kMPEG4Object_AAC_LC = 2;
	kMPEG4Object_AAC_SSR = 3;
	kMPEG4Object_AAC_LTP = 4;
	kMPEG4Object_AAC_SBR = 5;
	kMPEG4Object_AAC_Scalable = 6;
	
	kMPEG4Object_TwinVQ = 7;
	kMPEG4Object_CELP = 8;
	kMPEG4Object_HVXC = 9;    
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
}

{!
    @function   FillOutASBDForLPCM
    @abstract   A C++ inline function for filling out an AudioStreamBasicDescription to describe
                linear PCM data. Note that this function does not support specifying sample formats
                that are either unsigned integer or low-aligned.
    @param      outASBD
                    The AudioStreamBasicDescription to fill out.
    @param      inSampleRate
                    The number of sample frames per second of the data in the stream.
    @param      inChannelsPerFrame
                    The number of channels in each frame of data.
    @param      inValidBitsPerChannel
                    The number of valid bits in each sample.
    @param      inTotalBitsPerChannel
                    The total number of bits in each sample.
    @param      inIsFloat
                    Whether or not the samples are represented with floating point numbers.
    @param      isIsBigEndian
                    Whether the samples are big endian or little endian.
}

>>>>>>> origin/fixes_2.4

{!
    @struct         AudioStreamPacketDescription
    @abstract       This structure describes the packet layout of a buffer of data where the size of
                    each packet may not be the same or where there is extraneous data between
                    packets.
    @field          mStartOffset
                        The number of bytes from the start of the buffer to the beginning of the
                        packet.
    @field          mVariableFramesInPacket
                        The number of sample frames of data in the packet. For formats with a
                        constant number of frames per packet, this field is set to 0.
    @field          mDataByteSize
                        The number of bytes in the packet.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AudioStreamPacketDescriptionPtr = ^AudioStreamPacketDescription;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	AudioStreamPacketDescriptionPtr = ^AudioStreamPacketDescription;
>>>>>>> origin/fixes_2.4
	AudioStreamPacketDescription = record
		mStartOffset: SInt64;
		mVariableFramesInPacket: UInt32;
		mDataByteSize: UInt32;
	end;

//  SMPTETime is also defined in the CoreVideo headers.
//#if !defined(__SMPTETime__)
//#define __SMPTETime__
{GRP translation note:  As far as MacOSX10.4u.sdk public headers is concerned, this 
  header/interface is the only place these declarations are made. }
  
{!
    @struct         SMPTETime
    @abstract       A structure for holding a SMPTE time.
    @field          mSubframes
                        The number of subframes in the full message.
    @field          mSubframeDivisor
                        The number of subframes per frame (typically 80).
    @field          mCounter
                        The total number of messages received.
    @field          mType
                        The kind of SMPTE time using the SMPTE time type constants.
    @field          mFlags
                        A set of flags that indicate the SMPTE state.
    @field          mHours
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        The number of hours in the full message.
=======
                        The number of hourse in the full message.
>>>>>>> graemeg/fixes_2_2
=======
                        The number of hourse in the full message.
>>>>>>> origin/fixes_2_2
=======
                        The number of hours in the full message.
>>>>>>> origin/fixes_2.4
    @field          mMinutes
                        The number of minutes in the full message.
    @field          mSeconds
                        The number of seconds in the full message.
    @field          mFrames
                        The number of frames in the full message.
}
type
	SMPTETime = record
		mSubframes: SInt16;
		mSubframeDivisor: SInt16;
		mCounter: UInt32;
		mType: UInt32;
		mFlags: UInt32;
		mHours: SInt16;
		mMinutes: SInt16;
		mSeconds: SInt16;
		mFrames: SInt16;
	end;

{!
    @enum           SMPTE Time Types
    @abstract       Constants that describe the type of SMPTE time.
    @constant       kSMPTETimeType24
                        24 Frame
    @constant       kSMPTETimeType25
                        25 Frame
    @constant       kSMPTETimeType30Drop
                        30 Drop Frame
    @constant       kSMPTETimeType30
                        30 Frame
    @constant       kSMPTETimeType2997
                        29.97 Frame
    @constant       kSMPTETimeType2997Drop
                        29.97 Drop Frame
    @constant       kSMPTETimeType60
                        60 Frame
    @constant       kSMPTETimeType5994
                        59.94 Frame
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
    @constant       kSMPTETimeType60Drop
                        60 Drop Frame
    @constant       kSMPTETimeType5994Drop
                        59.94 Drop Frame
    @constant       kSMPTETimeType50
                        50 Frame
    @constant       kSMPTETimeType2398
                        23.98 Frame
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
}
const
	kSMPTETimeType24 = 0;
	kSMPTETimeType25 = 1;
	kSMPTETimeType30Drop = 2;
	kSMPTETimeType30 = 3;
	kSMPTETimeType2997 = 4;
	kSMPTETimeType2997Drop = 5;
	kSMPTETimeType60 = 6;
	kSMPTETimeType5994 = 7;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kSMPTETimeType60Drop = 8;
	kSMPTETimeType5994Drop = 9;
	kSMPTETimeType50 = 10;
	kSMPTETimeType2398 = 11;
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{!
    @enum           SMPTE State Flags
    @abstract       Flags that describe the SMPTE time state.
    @constant       kSMPTETimeValid
                        The full time is valid.
    @constant       kSMPTETimeRunning
                        Time is running.
}
const
	kSMPTETimeValid = 1 shl 0;
	kSMPTETimeRunning = 1 shl 1;

//#endif

{!
    @struct         AudioTimeStamp
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @abstract       A structure that holds different representations of the same point in time.
    @field          mSampleTime
                        The absolute sample frame time.
    @field          mHostTime
                        The host machine's time base, mach_absolute_time.
=======
=======
>>>>>>> origin/fixes_2_2
    @abstract       A strucutre that holds different representations of the same point in time.
    @field          mSampleTime
                        The absolute sample frame time.
    @field          mHostTime
                        The host machine's time base (see <CoreAudio/HostTime.h>).
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    @abstract       A structure that holds different representations of the same point in time.
    @field          mSampleTime
                        The absolute sample frame time.
    @field          mHostTime
                        The host machine's time base, mach_absolute_time.
>>>>>>> origin/fixes_2.4
    @field          mRateScalar
                        The ratio of actual host ticks per sample frame to the nominal host ticks
                        per sample frame.
    @field          mWordClockTime
                        The word clock time.
    @field          mSMPTETime
                        The SMPTE time.
    @field          mFlags
                        A set of flags indicating which representations of the time are valid.
    @field          mReserved
                        Pads the structure out to force an even 8 byte alignment.
}
type
	AudioTimeStamp = record
		mSampleTime: Float64;
		mHostTime: UInt64;
		mRateScalar: Float64;
		mWordClockTime: UInt64;
		mSMPTETime: SMPTETime;
		mFlags: UInt32;
		mReserved: UInt32;
	end;
	AudioTimeStampPtr = ^AudioTimeStamp;

{!
    @enum           AudioTimeStamp Flags
    @abstract       The flags that indicate which fields in an AudioTimeStamp structure are valid.
    @constant       kAudioTimeStampSampleTimeValid
                        The sample frame time is valid.
    @constant       kAudioTimeStampHostTimeValid
                        The host time is valid.
    @constant       kAudioTimeStampRateScalarValid
                        The rate scalar is valid.
    @constant       kAudioTimeStampWordClockTimeValid
                        The word clock time is valid.
    @constant       kAudioTimeStampSMPTETimeValid
                        The SMPTE time is valid.
}
const
	kAudioTimeStampSampleTimeValid = 1 shl 0;
	kAudioTimeStampHostTimeValid = 1 shl 1;
	kAudioTimeStampRateScalarValid = 1 shl 2;
	kAudioTimeStampWordClockTimeValid = 1 shl 3;
	kAudioTimeStampSMPTETimeValid = 1 shl 4;

{!
    @enum           Commonly Used Combinations of AudioTimeStamp Flags
    @abstract       Some commonly used combinations of AudioTimeStamp flags.
    @constant       kAudioTimeStampSampleHostTimeValid
                        The sample frame time and the host time are valid.
}
const
	kAudioTimeStampSampleHostTimeValid = kAudioTimeStampSampleTimeValid or kAudioTimeStampHostTimeValid;

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
    @function   FillOutAudioTimeStampWithSampleTime
    @abstract   A C++ inline function for filling out an AudioTimeStamp with a sample time
    @param      outATS
                    The AudioTimeStamp to fill out.
    @param      inSampleTime
                    The sample time to put in the AudioTimeStamp.
}

{!
    @function   FillOutAudioTimeStampWithHostTime
    @abstract   A C++ inline function for filling out an AudioTimeStamp with a host time
    @param      outATS
                    The AudioTimeStamp to fill out.
    @param      inHostTime
                    The host time to put in the AudioTimeStamp.
}

{!
    @function   FillOutAudioTimeStampWithSampleAndHostTime
    @abstract   A C++ inline function for filling out an AudioTimeStamp with a sample time and a
                host time
    @param      outATS
                    The AudioTimeStamp to fill out.
    @param      inSampleTime
                    The sample time to put in the AudioTimeStamp.
    @param      inHostTime
                    The host time to put in the AudioTimeStamp.
}

{!
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
    @struct         AudioClassDescription
    @abstract       This structure is used to describe codecs installed on the system.
    @field          mType
                        The four char code codec type.
    @field          mSubType
                        The four char code codec subtype.
    @field          mManufacturer
                        The four char code codec manufacturer.
}
type
	AudioClassDescription = record
		mType: OSType;
		mSubType: OSType;
		mManufacturer: OSType;
	end;


{!
    @typedef        AudioChannelLabel
    @abstract       A tag idenitfying how the channel is to be used.
}
type
	AudioChannelLabel = UInt32;

{!
    @typedef        AudioChannelLayoutTag
    @abstract       A tag identifying a particular pre-defined channel layout.
}
type
	AudioChannelLayoutTag = UInt32;

{!
    @struct         AudioChannelDescription
    @abstract       This structure describes a single channel.
    @field          mChannelLabel
                        The AudioChannelLabel that describes the channel.
    @field          mChannelFlags
                        Flags that control the interpretation of mCoordinates.
    @field          mCoordinates
                        An ordered triple that specifies a precise speaker location.
}
type
	AudioChannelDescription = record
		mChannelLabel: AudioChannelLabel;
		mChannelFlags: UInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		mCoordinates: array[0..2] of Float32;
=======
        mCoordinates: array[0..2] of Float32;
>>>>>>> graemeg/fixes_2_2
=======
        mCoordinates: array[0..2] of Float32;
>>>>>>> origin/fixes_2_2
=======
		mCoordinates: array[0..2] of Float32;
>>>>>>> origin/fixes_2.4
	end;

{!
    @struct         AudioChannelLayout
    @abstract       This structure is used to specify channel layouts in files and hardware.
    @field          mChannelLayoutTag
                        The AudioChannelLayoutTag that indicates the layout.
    @field          mChannelBitmap
                        If mChannelLayoutTag is set to kAudioChannelLayoutTag_UseChannelBitmap, this
                        field is the channel usage bitmap.
    @field          mNumberChannelDescriptions
                        The number of items in the mChannelDescriptions array.
    @field          mChannelDescriptions
                        A variable length array of AudioChannelDescriptions that describe the
                        layout.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AudioChannelLayoutPtr = ^AudioChannelLayout;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	AudioChannelLayoutPtr = ^AudioChannelLayout;
>>>>>>> origin/fixes_2.4
	AudioChannelLayout = record
		mChannelLayoutTag: AudioChannelLayoutTag;
		mChannelBitmap: UInt32;
		mNumberChannelDescriptions: UInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		mChannelDescriptions: array[0..0] of AudioChannelDescription;
=======
        mChannelDescriptions: array[0..0] of AudioChannelDescription;
>>>>>>> graemeg/fixes_2_2
=======
        mChannelDescriptions: array[0..0] of AudioChannelDescription;
>>>>>>> origin/fixes_2_2
=======
		mChannelDescriptions: array[0..0] of AudioChannelDescription;
>>>>>>> origin/fixes_2.4
	end;

{!
    @enum           AudioChannelLabel Constants
    @abstract       These constants are for use in the mChannelLabel field of an
                    AudioChannelDescription structure.
    @discussion     These channel labels attempt to list all labels in common use. Due to the
                    ambiguities in channel labeling by various groups, there may be some overlap or
                    duplication in the labels below. Use the label which most clearly describes what
                    you mean.

                    WAVE files seem to follow the USB spec for the channel flags. A channel map lets
                    you put these channels in any order, however a WAVE file only supports labels
                    1-18 and if present, they must be in the order given below. The integer values
                    for the labels below match the bit position of the label in the USB bitmap and
                    thus also the WAVE file bitmap.
}
const
	kAudioChannelLabel_Unknown = $FFFFFFFF;   // unknown or unspecified other use
	kAudioChannelLabel_Unused = 0;            // channel is present, but has no intended use or destination
	kAudioChannelLabel_UseCoordinates = 100;          // channel is described by the mCoordinates fields.

	kAudioChannelLabel_Left = 1;
	kAudioChannelLabel_Right = 2;
	kAudioChannelLabel_Center = 3;
	kAudioChannelLabel_LFEScreen = 4;
	kAudioChannelLabel_LeftSurround = 5;            // WAVE: "Back Left"
	kAudioChannelLabel_RightSurround = 6;            // WAVE: "Back Right"
	kAudioChannelLabel_LeftCenter = 7;
	kAudioChannelLabel_RightCenter = 8;
	kAudioChannelLabel_CenterSurround = 9;            // WAVE: "Back Center" or plain "Rear Surround"
	kAudioChannelLabel_LeftSurroundDirect = 10;           // WAVE: "Side Left"
	kAudioChannelLabel_RightSurroundDirect = 11;           // WAVE: "Side Right"
	kAudioChannelLabel_TopCenterSurround = 12;
	kAudioChannelLabel_VerticalHeightLeft = 13;           // WAVE: "Top Front Left"
	kAudioChannelLabel_VerticalHeightCenter = 14;           // WAVE: "Top Front Center"
	kAudioChannelLabel_VerticalHeightRight = 15;           // WAVE: "Top Front Right"

	kAudioChannelLabel_TopBackLeft = 16;
	kAudioChannelLabel_TopBackCenter = 17;
	kAudioChannelLabel_TopBackRight = 18;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
	
>>>>>>> graemeg/fixes_2_2
=======
	
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kAudioChannelLabel_RearSurroundLeft = 33;
	kAudioChannelLabel_RearSurroundRight = 34;
	kAudioChannelLabel_LeftWide = 35;
	kAudioChannelLabel_RightWide = 36;
	kAudioChannelLabel_LFE2 = 37;
	kAudioChannelLabel_LeftTotal = 38;           // matrix encoded 4 channels
	kAudioChannelLabel_RightTotal = 39;           // matrix encoded 4 channels
	kAudioChannelLabel_HearingImpaired = 40;
	kAudioChannelLabel_Narration = 41;
	kAudioChannelLabel_Mono = 42;
	kAudioChannelLabel_DialogCentricMix = 43;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioChannelLabel_CenterSurroundDirect = 44;           // back center, non diffuse
    
	kAudioChannelLabel_Haptic = 45;
=======
	
	kAudioChannelLabel_CenterSurroundDirect = 44;           // back center, non diffuse
>>>>>>> graemeg/fixes_2_2
=======
	
	kAudioChannelLabel_CenterSurroundDirect = 44;           // back center, non diffuse
>>>>>>> origin/fixes_2_2
=======
	kAudioChannelLabel_CenterSurroundDirect = 44;           // back center, non diffuse
    
	kAudioChannelLabel_Haptic = 45;
>>>>>>> origin/fixes_2.4

    // first order ambisonic channels
	kAudioChannelLabel_Ambisonic_W = 200;
	kAudioChannelLabel_Ambisonic_X = 201;
	kAudioChannelLabel_Ambisonic_Y = 202;
	kAudioChannelLabel_Ambisonic_Z = 203;

    // Mid/Side Recording
	kAudioChannelLabel_MS_Mid = 204;
	kAudioChannelLabel_MS_Side = 205;

    // X-Y Recording
	kAudioChannelLabel_XY_X = 206;
	kAudioChannelLabel_XY_Y = 207;

    // other
	kAudioChannelLabel_HeadphonesLeft = 301;
	kAudioChannelLabel_HeadphonesRight = 302;
	kAudioChannelLabel_ClickTrack = 304;
	kAudioChannelLabel_ForeignLanguage = 305;

    // generic discrete channel
	kAudioChannelLabel_Discrete = 400;

    // numbered discrete channel
	kAudioChannelLabel_Discrete_0 = (1 shl 16) or 0;
	kAudioChannelLabel_Discrete_1 = (1 shl 16) or 1;
	kAudioChannelLabel_Discrete_2 = (1 shl 16) or 2;
	kAudioChannelLabel_Discrete_3 = (1 shl 16) or 3;
	kAudioChannelLabel_Discrete_4 = (1 shl 16) or 4;
	kAudioChannelLabel_Discrete_5 = (1 shl 16) or 5;
	kAudioChannelLabel_Discrete_6 = (1 shl 16) or 6;
	kAudioChannelLabel_Discrete_7 = (1 shl 16) or 7;
	kAudioChannelLabel_Discrete_8 = (1 shl 16) or 8;
	kAudioChannelLabel_Discrete_9 = (1 shl 16) or 9;
	kAudioChannelLabel_Discrete_10 = (1 shl 16) or 10;
	kAudioChannelLabel_Discrete_11 = (1 shl 16) or 11;
	kAudioChannelLabel_Discrete_12 = (1 shl 16) or 12;
	kAudioChannelLabel_Discrete_13 = (1 shl 16) or 13;
	kAudioChannelLabel_Discrete_14 = (1 shl 16) or 14;
	kAudioChannelLabel_Discrete_15 = (1 shl 16) or 15;
	kAudioChannelLabel_Discrete_65535 = (1 shl 16) or 65535;

{!
    @enum           Channel Bitmap Constants
    @abstract       These constants are for use in the mChannelBitmap field of an
                    AudioChannelLayout structure.
}
const
	kAudioChannelBit_Left = 1 shl 0;
	kAudioChannelBit_Right = 1 shl 1;
	kAudioChannelBit_Center = 1 shl 2;
	kAudioChannelBit_LFEScreen = 1 shl 3;
	kAudioChannelBit_LeftSurround = 1 shl 4;      // WAVE: "Back Left"
	kAudioChannelBit_RightSurround = 1 shl 5;      // WAVE: "Back Right"
	kAudioChannelBit_LeftCenter = 1 shl 6;
	kAudioChannelBit_RightCenter = 1 shl 7;
	kAudioChannelBit_CenterSurround = 1 shl 8;      // WAVE: "Back Center"
	kAudioChannelBit_LeftSurroundDirect = 1 shl 9;      // WAVE: "Side Left"
	kAudioChannelBit_RightSurroundDirect = 1 shl 10;     // WAVE: "Side Right"
	kAudioChannelBit_TopCenterSurround = 1 shl 11;
	kAudioChannelBit_VerticalHeightLeft = 1 shl 12;     // WAVE: "Top Front Left"
	kAudioChannelBit_VerticalHeightCenter = 1 shl 13;     // WAVE: "Top Front Center"
	kAudioChannelBit_VerticalHeightRight = 1 shl 14;     // WAVE: "Top Front Right"
	kAudioChannelBit_TopBackLeft = 1 shl 15;
	kAudioChannelBit_TopBackCenter = 1 shl 16;
	kAudioChannelBit_TopBackRight = 1 shl 17;

{!
    @enum           Channel Coordinate Flags
    @abstract       These constants are used in the mChannelFlags field of an
                    AudioChannelDescription structure.
    @constant       kAudioChannelFlags_RectangularCoordinates
                        The channel is specified by the cartesioan coordinates of the speaker
                        position. This flag is mutally exclusive with
                        kAudioChannelFlags_SphericalCoordinates.
    @constant       kAudioChannelFlags_SphericalCoordinates
                        The channel is specified by the spherical coordinates of the speaker
                        position. This flag is mutally exclusive with
                        kAudioChannelFlags_RectangularCoordinates.
    @constant       kAudioChannelFlags_Meters
                        Set to indicate the units are in meters, clear to indicate the units are
                        relative to the unit cube or unit sphere.
}
const
	kAudioChannelFlags_AllOff = 0;
	kAudioChannelFlags_RectangularCoordinates = 1 shl 0;
	kAudioChannelFlags_SphericalCoordinates = 1 shl 1;
	kAudioChannelFlags_Meters = 1 shl 2;

// these are indices for acessing the mCoordinates array in struct AudioChannelDescription
{!
    @enum           Channel Coordinate Index Constants
    @abstract       Constants for indexing the mCoordinates array in an AudioChannelDescription
                    structure.
    @constant       kAudioChannelCoordinates_LeftRight
                        For rectangulare coordinates, negative is left and positive is right.
    @constant       kAudioChannelCoordinates_BackFront
                        For rectangulare coordinates, negative is back and positive is front.
    @constant       kAudioChannelCoordinates_DownUp
                        For rectangulare coordinates, negative is below ground level, 0 is ground
                        level, and positive is above ground level.
    @constant       kAudioChannelCoordinates_Azimuth
                        For spherical coordinates, 0 is front center, positive is right, negative is
                        left. This is measured in degrees.
    @constant       kAudioChannelCoordinates_Elevation
                        For spherical coordinates, +90 is zenith, 0 is horizontal, -90 is nadir.
                        This is measured in degrees.
    @constant       kAudioChannelCoordinates_Distance
                        For spherical coordinates, the units are described by flags.
}
const
	kAudioChannelCoordinates_LeftRight = 0;
	kAudioChannelCoordinates_BackFront = 1;
	kAudioChannelCoordinates_DownUp = 2;
	kAudioChannelCoordinates_Azimuth = 0;
	kAudioChannelCoordinates_Elevation = 1;
	kAudioChannelCoordinates_Distance = 2;

{!
    @function       AudioChannelLayoutTag_GetNumberOfChannels
    @abstract       A macro to get the number of channels out of an AudioChannelLayoutTag
    @discussion     The low 16 bits of an AudioChannelLayoutTag gives the number of channels except
                    for kAudioChannelLayoutTag_UseChannelDescriptions and
                    kAudioChannelLayoutTag_UseChannelBitmap.
    @param          layoutTag
                        The AudioChannelLayoutTag to examine.
    @result         The number of channels the tag indicates.
}
//#define AudioChannelLayoutTag_GetNumberOfChannels(layoutTag) ((UInt32)((layoutTag) & 0x0000FFFF))

{!
    @enum           AudioChannelLayoutTag Constants
    @abstract       These constants are used in the mChannelLayoutTag field of an AudioChannelLayout
                    structure.
}
const
// Some channel abbreviations used below:
    // L - left
    // R - right
    // C - center
    // Ls - left surround
    // Rs - right surround
    // Cs - center surround
    // Rls - rear left surround
    // Rrs - rear right surround
    // Lw - left wide
    // Rw - right wide
    // Lsd - left surround direct
    // Rsd - right surround direct
    // Lc - left center
    // Rc - right center
    // Ts - top surround
    // Vhl - vertical height left
    // Vhc - vertical height center
    // Vhr - vertical height right
    // Lt - left matrix total. for matrix encoded stereo.
    // Rt - right matrix total. for matrix encoded stereo.

    //  General layouts
	kAudioChannelLayoutTag_UseChannelDescriptions = (0 shl 16) or 0;     // use the array of AudioChannelDescriptions to define the mapping.
	kAudioChannelLayoutTag_UseChannelBitmap = (1 shl 16) or 0;     // use the bitmap to define the mapping.

	kAudioChannelLayoutTag_Mono = (100 shl 16) or 1;   // a standard mono stream
	kAudioChannelLayoutTag_Stereo = (101 shl 16) or 2;   // a standard stereo stream (L R) - implied playback
	kAudioChannelLayoutTag_StereoHeadphones = (102 shl 16) or 2;   // a standard stereo stream (L R) - implied headphone playbac
	kAudioChannelLayoutTag_MatrixStereo = (103 shl 16) or 2;   // a matrix encoded stereo stream (Lt, Rt)
	kAudioChannelLayoutTag_MidSide = (104 shl 16) or 2;   // mid/side recording
	kAudioChannelLayoutTag_XY = (105 shl 16) or 2;   // coincident mic pair (often 2 figure 8's)
	kAudioChannelLayoutTag_Binaural = (106 shl 16) or 2;   // binaural stereo (left, right)
	kAudioChannelLayoutTag_Ambisonic_B_Format = (107 shl 16) or 4;   // W, X, Y, Z

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kAudioChannelLayoutTag_Quadraphonic = (108 shl 16) or 4;   // L R Ls Rs  -- 90 degree speaker separation
	kAudioChannelLayoutTag_Pentagonal = (109 shl 16) or 5;   // L R Ls Rs C  -- 72 degree speaker separation
	kAudioChannelLayoutTag_Hexagonal = (110 shl 16) or 6;   // L R Ls Rs C Cs  -- 60 degree speaker separation
	kAudioChannelLayoutTag_Octagonal = (111 shl 16) or 8;   // L R Ls Rs C Cs Lw Rw  -- 45 degree speaker separation
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	kAudioChannelLayoutTag_Quadraphonic = (108 shl 16) or 4;   // front left, front right, back left, back right

	kAudioChannelLayoutTag_Pentagonal = (109 shl 16) or 5;   // left, right, rear left, rear right, center

	kAudioChannelLayoutTag_Hexagonal = (110 shl 16) or 6;   // left, right, rear left, rear right, center, rear

	kAudioChannelLayoutTag_Octagonal = (111 shl 16) or 8;   // front left, front right, rear left, rear right,
                                                                        // front center, rear center, side left, side right

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kAudioChannelLayoutTag_Cube = (112 shl 16) or 8;   // left, right, rear left, rear right
                                                                        // top left, top right, top rear left, top rear right

    //  MPEG defined layouts
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioChannelLayoutTag_MPEG_1_0 = kAudioChannelLayoutTag_Mono;         //  C
	kAudioChannelLayoutTag_MPEG_2_0 = kAudioChannelLayoutTag_Stereo;       //  L R
=======
	kAudioChannelLayoutTag_MPEG_1_0 = kAudioChannelLayoutTag_Mono;          //  C
	kAudioChannelLayoutTag_MPEG_2_0 = kAudioChannelLayoutTag_Stereo;        //  L R
>>>>>>> graemeg/fixes_2_2
=======
	kAudioChannelLayoutTag_MPEG_1_0 = kAudioChannelLayoutTag_Mono;          //  C
	kAudioChannelLayoutTag_MPEG_2_0 = kAudioChannelLayoutTag_Stereo;        //  L R
>>>>>>> origin/fixes_2_2
=======
	kAudioChannelLayoutTag_MPEG_1_0 = kAudioChannelLayoutTag_Mono;         //  C
	kAudioChannelLayoutTag_MPEG_2_0 = kAudioChannelLayoutTag_Stereo;       //  L R
>>>>>>> origin/fixes_2.4
	kAudioChannelLayoutTag_MPEG_3_0_A = (113 shl 16) or 3;                       //  L R C
	kAudioChannelLayoutTag_MPEG_3_0_B = (114 shl 16) or 3;                       //  C L R
	kAudioChannelLayoutTag_MPEG_4_0_A = (115 shl 16) or 4;                       //  L R C Cs
	kAudioChannelLayoutTag_MPEG_4_0_B = (116 shl 16) or 4;                       //  C L R Cs
	kAudioChannelLayoutTag_MPEG_5_0_A = (117 shl 16) or 5;                       //  L R C Ls Rs
	kAudioChannelLayoutTag_MPEG_5_0_B = (118 shl 16) or 5;                       //  L R Ls Rs C
	kAudioChannelLayoutTag_MPEG_5_0_C = (119 shl 16) or 5;                       //  L C R Ls Rs
	kAudioChannelLayoutTag_MPEG_5_0_D = (120 shl 16) or 5;                       //  C L R Ls Rs
	kAudioChannelLayoutTag_MPEG_5_1_A = (121 shl 16) or 6;                       //  L R C LFE Ls Rs
	kAudioChannelLayoutTag_MPEG_5_1_B = (122 shl 16) or 6;                       //  L R Ls Rs C LFE
	kAudioChannelLayoutTag_MPEG_5_1_C = (123 shl 16) or 6;                       //  L C R Ls Rs LFE
	kAudioChannelLayoutTag_MPEG_5_1_D = (124 shl 16) or 6;                       //  C L R Ls Rs LFE
	kAudioChannelLayoutTag_MPEG_6_1_A = (125 shl 16) or 7;                       //  L R C LFE Ls Rs Cs
	kAudioChannelLayoutTag_MPEG_7_1_A = (126 shl 16) or 8;                       //  L R C LFE Ls Rs Lc Rc
	kAudioChannelLayoutTag_MPEG_7_1_B = (127 shl 16) or 8;                       //  C Lc Rc L R Ls Rs LFE    (doc: IS-13818-7 MPEG2-AAC Table 3.1)
	kAudioChannelLayoutTag_MPEG_7_1_C = (128 shl 16) or 8;                       //  L R C LFE Ls R Rls Rrs
	kAudioChannelLayoutTag_Emagic_Default_7_1 = (129 shl 16) or 8;                       //  L R Ls Rs C LFE Lc Rc
	kAudioChannelLayoutTag_SMPTE_DTV = (130 shl 16) or 8;                       //  L R C LFE Ls Rs Lt Rt
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
																						   //      (kAudioChannelLayoutTag_ITU_5_1 plus a matrix encoded stereo mix)

    //  ITU defined layouts
	kAudioChannelLayoutTag_ITU_1_0 = kAudioChannelLayoutTag_Mono;         //  C
	kAudioChannelLayoutTag_ITU_2_0 = kAudioChannelLayoutTag_Stereo;       //  L R

	kAudioChannelLayoutTag_ITU_2_1 = (131 shl 16) or 3;                       //  L R Cs
	kAudioChannelLayoutTag_ITU_2_2 = (132 shl 16) or 4;                       //  L R Ls Rs
	kAudioChannelLayoutTag_ITU_3_0 = kAudioChannelLayoutTag_MPEG_3_0_A;   //  L R C
	kAudioChannelLayoutTag_ITU_3_1 = kAudioChannelLayoutTag_MPEG_4_0_A;   //  L R C Cs

	kAudioChannelLayoutTag_ITU_3_2 = kAudioChannelLayoutTag_MPEG_5_0_A;   //  L R C Ls Rs
	kAudioChannelLayoutTag_ITU_3_2_1 = kAudioChannelLayoutTag_MPEG_5_1_A;   //  L R C LFE Ls Rs
	kAudioChannelLayoutTag_ITU_3_4_1 = kAudioChannelLayoutTag_MPEG_7_1_C;   //  L R C LFE Ls Rs Rls Rrs

    // DVD defined layouts
	kAudioChannelLayoutTag_DVD_0 = kAudioChannelLayoutTag_Mono;         // C (mono)
	kAudioChannelLayoutTag_DVD_1 = kAudioChannelLayoutTag_Stereo;       // L R
	kAudioChannelLayoutTag_DVD_2 = kAudioChannelLayoutTag_ITU_2_1;      // L R Cs
	kAudioChannelLayoutTag_DVD_3 = kAudioChannelLayoutTag_ITU_2_2;      // L R Ls Rs
	kAudioChannelLayoutTag_DVD_4 = (133 shl 16) or 3;                       // L R LFE
	kAudioChannelLayoutTag_DVD_5 = (134 shl 16) or 4;                       // L R LFE Cs
	kAudioChannelLayoutTag_DVD_6 = (135 shl 16) or 5;                       // L R LFE Ls Rs
	kAudioChannelLayoutTag_DVD_7 = kAudioChannelLayoutTag_MPEG_3_0_A;   // L R C
	kAudioChannelLayoutTag_DVD_8 = kAudioChannelLayoutTag_MPEG_4_0_A;   // L R C Cs
	kAudioChannelLayoutTag_DVD_9 = kAudioChannelLayoutTag_MPEG_5_0_A;   // L R C Ls Rs
	kAudioChannelLayoutTag_DVD_10 = (136 shl 16) or 4;                       // L R C LFE
	kAudioChannelLayoutTag_DVD_11 = (137 shl 16) or 5;                       // L R C LFE Cs
	kAudioChannelLayoutTag_DVD_12 = kAudioChannelLayoutTag_MPEG_5_1_A;   // L R C LFE Ls Rs
    // 13 through 17 are duplicates of 8 through 12.
	kAudioChannelLayoutTag_DVD_13 = kAudioChannelLayoutTag_DVD_8;        // L R C Cs
	kAudioChannelLayoutTag_DVD_14 = kAudioChannelLayoutTag_DVD_9;        // L R C Ls Rs
	kAudioChannelLayoutTag_DVD_15 = kAudioChannelLayoutTag_DVD_10;       // L R C LFE
	kAudioChannelLayoutTag_DVD_16 = kAudioChannelLayoutTag_DVD_11;       // L R C LFE Cs
	kAudioChannelLayoutTag_DVD_17 = kAudioChannelLayoutTag_DVD_12;       // L R C LFE Ls Rs
	kAudioChannelLayoutTag_DVD_18 = (138 shl 16) or 5;                       // L R Ls Rs LFE
	kAudioChannelLayoutTag_DVD_19 = kAudioChannelLayoutTag_MPEG_5_0_B;   // L R Ls Rs C
	kAudioChannelLayoutTag_DVD_20 = kAudioChannelLayoutTag_MPEG_5_1_B;   // L R Ls Rs C LFE
=======
=======
>>>>>>> origin/fixes_2_2
                                                                                            //      (kAudioChannelLayoutTag_ITU_5_1 plus a matrix encoded stereo mix)
=======
																						   //      (kAudioChannelLayoutTag_ITU_5_1 plus a matrix encoded stereo mix)
>>>>>>> origin/fixes_2.4

    //  ITU defined layouts
	kAudioChannelLayoutTag_ITU_1_0 = kAudioChannelLayoutTag_Mono;         //  C
	kAudioChannelLayoutTag_ITU_2_0 = kAudioChannelLayoutTag_Stereo;       //  L R

	kAudioChannelLayoutTag_ITU_2_1 = (131 shl 16) or 3;                       //  L R Cs
	kAudioChannelLayoutTag_ITU_2_2 = (132 shl 16) or 4;                       //  L R Ls Rs
	kAudioChannelLayoutTag_ITU_3_0 = kAudioChannelLayoutTag_MPEG_3_0_A;   //  L R C
	kAudioChannelLayoutTag_ITU_3_1 = kAudioChannelLayoutTag_MPEG_4_0_A;   //  L R C Cs

	kAudioChannelLayoutTag_ITU_3_2 = kAudioChannelLayoutTag_MPEG_5_0_A;   //  L R C Ls Rs
	kAudioChannelLayoutTag_ITU_3_2_1 = kAudioChannelLayoutTag_MPEG_5_1_A;   //  L R C LFE Ls Rs
	kAudioChannelLayoutTag_ITU_3_4_1 = kAudioChannelLayoutTag_MPEG_7_1_C;   //  L R C LFE Ls Rs Rls Rrs

    // DVD defined layouts
	kAudioChannelLayoutTag_DVD_0 = kAudioChannelLayoutTag_Mono;         // C (mono)
	kAudioChannelLayoutTag_DVD_1 = kAudioChannelLayoutTag_Stereo;       // L R
	kAudioChannelLayoutTag_DVD_2 = kAudioChannelLayoutTag_ITU_2_1;      // L R Cs
	kAudioChannelLayoutTag_DVD_3 = kAudioChannelLayoutTag_ITU_2_2;      // L R Ls Rs
	kAudioChannelLayoutTag_DVD_4 = (133 shl 16) or 3;                       // L R LFE
	kAudioChannelLayoutTag_DVD_5 = (134 shl 16) or 4;                       // L R LFE Cs
	kAudioChannelLayoutTag_DVD_6 = (135 shl 16) or 5;                       // L R LFE Ls Rs
	kAudioChannelLayoutTag_DVD_7 = kAudioChannelLayoutTag_MPEG_3_0_A;   // L R C
	kAudioChannelLayoutTag_DVD_8 = kAudioChannelLayoutTag_MPEG_4_0_A;   // L R C Cs
	kAudioChannelLayoutTag_DVD_9 = kAudioChannelLayoutTag_MPEG_5_0_A;   // L R C Ls Rs
	kAudioChannelLayoutTag_DVD_10 = (136 shl 16) or 4;                       // L R C LFE
	kAudioChannelLayoutTag_DVD_11 = (137 shl 16) or 5;                       // L R C LFE Cs
	kAudioChannelLayoutTag_DVD_12 = kAudioChannelLayoutTag_MPEG_5_1_A;   // L R C LFE Ls Rs
    // 13 through 17 are duplicates of 8 through 12.
	kAudioChannelLayoutTag_DVD_13 = kAudioChannelLayoutTag_DVD_8;        // L R C Cs
	kAudioChannelLayoutTag_DVD_14 = kAudioChannelLayoutTag_DVD_9;        // L R C Ls Rs
	kAudioChannelLayoutTag_DVD_15 = kAudioChannelLayoutTag_DVD_10;       // L R C LFE
	kAudioChannelLayoutTag_DVD_16 = kAudioChannelLayoutTag_DVD_11;       // L R C LFE Cs
	kAudioChannelLayoutTag_DVD_17 = kAudioChannelLayoutTag_DVD_12;       // L R C LFE Ls Rs
	kAudioChannelLayoutTag_DVD_18 = (138 shl 16) or 5;                       // L R Ls Rs LFE
<<<<<<< HEAD
	kAudioChannelLayoutTag_DVD_19 = kAudioChannelLayoutTag_MPEG_5_0_B;    // L R Ls Rs C
	kAudioChannelLayoutTag_DVD_20 = kAudioChannelLayoutTag_MPEG_5_1_B;    // L R Ls Rs C LFE
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kAudioChannelLayoutTag_DVD_19 = kAudioChannelLayoutTag_MPEG_5_0_B;   // L R Ls Rs C
	kAudioChannelLayoutTag_DVD_20 = kAudioChannelLayoutTag_MPEG_5_1_B;   // L R Ls Rs C LFE
>>>>>>> origin/fixes_2.4

    // These layouts are recommended for AudioUnit usage
        // These are the symmetrical layouts
	kAudioChannelLayoutTag_AudioUnit_4 = kAudioChannelLayoutTag_Quadraphonic;
	kAudioChannelLayoutTag_AudioUnit_5 = kAudioChannelLayoutTag_Pentagonal;
	kAudioChannelLayoutTag_AudioUnit_6 = kAudioChannelLayoutTag_Hexagonal;
	kAudioChannelLayoutTag_AudioUnit_8 = kAudioChannelLayoutTag_Octagonal;
        // These are the surround-based layouts
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioChannelLayoutTag_AudioUnit_5_0 = kAudioChannelLayoutTag_MPEG_5_0_B;   // L R Ls Rs C
	kAudioChannelLayoutTag_AudioUnit_6_0 = (139 shl 16) or 6;                       // L R Ls Rs C Cs
	kAudioChannelLayoutTag_AudioUnit_7_0 = (140 shl 16) or 7;                       // L R Ls Rs C Rls Rrs
	kAudioChannelLayoutTag_AudioUnit_7_0_Front = (148 shl 16) or 7;                       // L R Ls Rs C Lc Rc
	kAudioChannelLayoutTag_AudioUnit_5_1 = kAudioChannelLayoutTag_MPEG_5_1_A;   // L R C LFE Ls Rs
	kAudioChannelLayoutTag_AudioUnit_6_1 = kAudioChannelLayoutTag_MPEG_6_1_A;   // L R C LFE Ls Rs Cs
	kAudioChannelLayoutTag_AudioUnit_7_1 = kAudioChannelLayoutTag_MPEG_7_1_C;   // L R C LFE Ls Rs Rls Rrs
	kAudioChannelLayoutTag_AudioUnit_7_1_Front = kAudioChannelLayoutTag_MPEG_7_1_A;   // L R C LFE Ls Rs Lc Rc

	kAudioChannelLayoutTag_AAC_3_0 = kAudioChannelLayoutTag_MPEG_3_0_B;   // C L R
	kAudioChannelLayoutTag_AAC_Quadraphonic = kAudioChannelLayoutTag_Quadraphonic; // L R Ls Rs
	kAudioChannelLayoutTag_AAC_4_0 = kAudioChannelLayoutTag_MPEG_4_0_B;   // C L R Cs
	kAudioChannelLayoutTag_AAC_5_0 = kAudioChannelLayoutTag_MPEG_5_0_D;   // C L R Ls Rs
	kAudioChannelLayoutTag_AAC_5_1 = kAudioChannelLayoutTag_MPEG_5_1_D;   // C L R Ls Rs Lfe
	kAudioChannelLayoutTag_AAC_6_0 = (141 shl 16) or 6;                       // C L R Ls Rs Cs
	kAudioChannelLayoutTag_AAC_6_1 = (142 shl 16) or 7;                       // C L R Ls Rs Cs Lfe
	kAudioChannelLayoutTag_AAC_7_0 = (143 shl 16) or 7;                       // C L R Ls Rs Rls Rrs
	kAudioChannelLayoutTag_AAC_7_1 = kAudioChannelLayoutTag_MPEG_7_1_B;   // C Lc Rc L R Ls Rs Lfe
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioChannelLayoutTag_AAC_7_1_B = (183 shl 16) or 8;                       // C L R Ls Rs Rls Rrs LFE
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kAudioChannelLayoutTag_AAC_Octagonal = (144 shl 16) or 8;                       // C L R Ls Rs Rls Rrs Cs

	kAudioChannelLayoutTag_TMH_10_2_std = (145 shl 16) or 16;                      // L R C Vhc Lsd Rsd Ls Rs Vhl Vhr Lw Rw Csd Cs LFE1 LFE2
	kAudioChannelLayoutTag_TMH_10_2_full = (146 shl 16) or 21;                      // TMH_10_2_std plus: Lc Rc HI VI Haptic
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	kAudioChannelLayoutTag_AC3_1_0_1 = (149 shl 16) or 2;                       // C LFE
	kAudioChannelLayoutTag_AC3_3_0 = (150 shl 16) or 3;                       // L C R
	kAudioChannelLayoutTag_AC3_3_1 = (151 shl 16) or 4;                       // L C R Cs
	kAudioChannelLayoutTag_AC3_3_0_1 = (152 shl 16) or 4;                       // L C R LFE
	kAudioChannelLayoutTag_AC3_2_1_1 = (153 shl 16) or 4;                       // L R Cs LFE
	kAudioChannelLayoutTag_AC3_3_1_1 = (154 shl 16) or 5;                       // L C R Cs LFE

	kAudioChannelLayoutTag_EAC_6_0_A = (155 shl 16) or 6;                       // L C R Ls Rs Cs
	kAudioChannelLayoutTag_EAC_7_0_A = (156 shl 16) or 7;                       // L C R Ls Rs Rls Rrs

	kAudioChannelLayoutTag_EAC3_6_1_A = (157 shl 16) or 7;                       // L C R Ls Rs LFE Cs
	kAudioChannelLayoutTag_EAC3_6_1_B = (158 shl 16) or 7;                       // L C R Ls Rs LFE Ts
	kAudioChannelLayoutTag_EAC3_6_1_C = (159 shl 16) or 7;                       // L C R Ls Rs LFE Vhc
	kAudioChannelLayoutTag_EAC3_7_1_A = (160 shl 16) or 8;                       // L C R Ls Rs LFE Rls Rrs
	kAudioChannelLayoutTag_EAC3_7_1_B = (161 shl 16) or 8;                       // L C R Ls Rs LFE Lc Rc
	kAudioChannelLayoutTag_EAC3_7_1_C = (162 shl 16) or 8;                       // L C R Ls Rs LFE Lsd Rsd
	kAudioChannelLayoutTag_EAC3_7_1_D = (163 shl 16) or 8;                       // L C R Ls Rs LFE Lw Rw
	kAudioChannelLayoutTag_EAC3_7_1_E = (164 shl 16) or 8;                       // L C R Ls Rs LFE Vhl Vhr

	kAudioChannelLayoutTag_EAC3_7_1_F = (165 shl 16) or 8;                        // L C R Ls Rs LFE Cs Ts
	kAudioChannelLayoutTag_EAC3_7_1_G = (166 shl 16) or 8;                        // L C R Ls Rs LFE Cs Vhc
	kAudioChannelLayoutTag_EAC3_7_1_H = (167 shl 16) or 8;                        // L C R Ls Rs LFE Ts Vhc

	kAudioChannelLayoutTag_DTS_3_1 = (168 shl 16) or 4;                        // C L R LFE
	kAudioChannelLayoutTag_DTS_4_1 = (169 shl 16) or 5;                        // C L R Cs LFE
	kAudioChannelLayoutTag_DTS_6_0_A = (170 shl 16) or 6;                        // Lc Rc L R Ls Rs
	kAudioChannelLayoutTag_DTS_6_0_B = (171 shl 16) or 6;                        // C L R Rls Rrs Ts
	kAudioChannelLayoutTag_DTS_6_0_C = (172 shl 16) or 6;                        // C Cs L R Rls Rrs
	kAudioChannelLayoutTag_DTS_6_1_A = (173 shl 16) or 7;                        // Lc Rc L R Ls Rs LFE
	kAudioChannelLayoutTag_DTS_6_1_B = (174 shl 16) or 7;                        // C L R Rls Rrs Ts LFE
	kAudioChannelLayoutTag_DTS_6_1_C = (175 shl 16) or 7;                        // C Cs L R Rls Rrs LFE
	kAudioChannelLayoutTag_DTS_7_0 = (176 shl 16) or 7;                        // Lc C Rc L R Ls Rs
	kAudioChannelLayoutTag_DTS_7_1 = (177 shl 16) or 8;                        // Lc C Rc L R Ls Rs LFE    
	kAudioChannelLayoutTag_DTS_8_0_A = (178 shl 16) or 8;                        // Lc Rc L R Ls Rs Rls Rrs
	kAudioChannelLayoutTag_DTS_8_0_B = (179 shl 16) or 8;                        // Lc C Rc L R Ls Cs Rs
	kAudioChannelLayoutTag_DTS_8_1_A = (180 shl 16) or 9;                        // Lc Rc L R Ls Rs Rls Rrs LFE
	kAudioChannelLayoutTag_DTS_8_1_B = (181 shl 16) or 9;                        // Lc C Rc L R Ls Cs Rs LFE
	kAudioChannelLayoutTag_DTS_6_1_D = (182 shl 16) or 7;                        // C L R Ls Rs LFE Cs

	kAudioChannelLayoutTag_DiscreteInOrder = (147 shl 16) or 0;                       // needs to be ORed with the actual number of channels  
	kAudioChannelLayoutTag_Unknown = $FFFF0000;                           // needs to be ORed with the actual number of channels  


// Deprecated constants

{! @enum           MPEG-4 Audio Object IDs
    @deprecated     in version 10.5

=======

	kAudioChannelLayoutTag_AC3_1_0_1 = (149 shl 16) or 2;                       // C LFE
	kAudioChannelLayoutTag_AC3_3_0 = (150 shl 16) or 3;                       // L C R
	kAudioChannelLayoutTag_AC3_3_1 = (151 shl 16) or 4;                       // L C R Cs
	kAudioChannelLayoutTag_AC3_3_0_1 = (152 shl 16) or 4;                       // L C R LFE
	kAudioChannelLayoutTag_AC3_2_1_1 = (153 shl 16) or 4;                       // L R Cs LFE
	kAudioChannelLayoutTag_AC3_3_1_1 = (154 shl 16) or 5;                       // L C R Cs LFE

=======

	kAudioChannelLayoutTag_AC3_1_0_1 = (149 shl 16) or 2;                       // C LFE
	kAudioChannelLayoutTag_AC3_3_0 = (150 shl 16) or 3;                       // L C R
	kAudioChannelLayoutTag_AC3_3_1 = (151 shl 16) or 4;                       // L C R Cs
	kAudioChannelLayoutTag_AC3_3_0_1 = (152 shl 16) or 4;                       // L C R LFE
	kAudioChannelLayoutTag_AC3_2_1_1 = (153 shl 16) or 4;                       // L R Cs LFE
	kAudioChannelLayoutTag_AC3_3_1_1 = (154 shl 16) or 5;                       // L C R Cs LFE

>>>>>>> graemeg/cpstrnew
=======

	kAudioChannelLayoutTag_AC3_1_0_1 = (149 shl 16) or 2;                       // C LFE
	kAudioChannelLayoutTag_AC3_3_0 = (150 shl 16) or 3;                       // L C R
	kAudioChannelLayoutTag_AC3_3_1 = (151 shl 16) or 4;                       // L C R Cs
	kAudioChannelLayoutTag_AC3_3_0_1 = (152 shl 16) or 4;                       // L C R LFE
	kAudioChannelLayoutTag_AC3_2_1_1 = (153 shl 16) or 4;                       // L R Cs LFE
	kAudioChannelLayoutTag_AC3_3_1_1 = (154 shl 16) or 5;                       // L C R Cs LFE

>>>>>>> graemeg/cpstrnew
=======

	kAudioChannelLayoutTag_AC3_1_0_1 = (149 shl 16) or 2;                       // C LFE
	kAudioChannelLayoutTag_AC3_3_0 = (150 shl 16) or 3;                       // L C R
	kAudioChannelLayoutTag_AC3_3_1 = (151 shl 16) or 4;                       // L C R Cs
	kAudioChannelLayoutTag_AC3_3_0_1 = (152 shl 16) or 4;                       // L C R LFE
	kAudioChannelLayoutTag_AC3_2_1_1 = (153 shl 16) or 4;                       // L R Cs LFE
	kAudioChannelLayoutTag_AC3_3_1_1 = (154 shl 16) or 5;                       // L C R Cs LFE

>>>>>>> origin/cpstrnew
	kAudioChannelLayoutTag_DiscreteInOrder = (147 shl 16) or 0;                       // needs to be ORed with the actual number of channels  
	kAudioChannelLayoutTag_Unknown = $FFFF0000;                           // needs to be ORed with the actual number of channels  


// Deprecated constants

{! @enum           MPEG-4 Audio Object IDs
    @deprecated     in version 10.5

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
    @abstract       Constants that describe the various kinds of MPEG-4 audio data.
    @discussion     These constants are used in the flags field of an AudioStreamBasicDescription
                    that describes an MPEG-4 audio stream.
}
const
	kMPEG4Object_AAC_Main = 1;
	kMPEG4Object_AAC_LC = 2;
	kMPEG4Object_AAC_SSR = 3;
	kMPEG4Object_AAC_LTP = 4;
	kMPEG4Object_AAC_SBR = 5;
	kMPEG4Object_AAC_Scalable = 6;
	kMPEG4Object_TwinVQ = 7;
	kMPEG4Object_CELP = 8;
	kMPEG4Object_HVXC = 9;    

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
	kAudioChannelLayoutTag_AudioUnit_5_0 = kAudioChannelLayoutTag_MPEG_5_0_B;    // L R Ls Rs C
=======
	kAudioChannelLayoutTag_AudioUnit_5_0 = kAudioChannelLayoutTag_MPEG_5_0_B;   // L R Ls Rs C
>>>>>>> origin/fixes_2.4
	kAudioChannelLayoutTag_AudioUnit_6_0 = (139 shl 16) or 6;                       // L R Ls Rs C Cs
	kAudioChannelLayoutTag_AudioUnit_7_0 = (140 shl 16) or 7;                       // L R Ls Rs C Rls Rrs
	kAudioChannelLayoutTag_AudioUnit_7_0_Front = (148 shl 16) or 7;                       // L R Ls Rs C Lc Rc
	kAudioChannelLayoutTag_AudioUnit_5_1 = kAudioChannelLayoutTag_MPEG_5_1_A;   // L R C LFE Ls Rs
	kAudioChannelLayoutTag_AudioUnit_6_1 = kAudioChannelLayoutTag_MPEG_6_1_A;   // L R C LFE Ls Rs Cs
	kAudioChannelLayoutTag_AudioUnit_7_1 = kAudioChannelLayoutTag_MPEG_7_1_C;   // L R C LFE Ls Rs Rls Rrs
	kAudioChannelLayoutTag_AudioUnit_7_1_Front = kAudioChannelLayoutTag_MPEG_7_1_A;   // L R C LFE Ls Rs Lc Rc

	kAudioChannelLayoutTag_AAC_3_0 = kAudioChannelLayoutTag_MPEG_3_0_B;   // C L R
	kAudioChannelLayoutTag_AAC_Quadraphonic = kAudioChannelLayoutTag_Quadraphonic; // L R Ls Rs
	kAudioChannelLayoutTag_AAC_4_0 = kAudioChannelLayoutTag_MPEG_4_0_B;   // C L R Cs
	kAudioChannelLayoutTag_AAC_5_0 = kAudioChannelLayoutTag_MPEG_5_0_D;   // C L R Ls Rs
	kAudioChannelLayoutTag_AAC_5_1 = kAudioChannelLayoutTag_MPEG_5_1_D;   // C L R Ls Rs Lfe
	kAudioChannelLayoutTag_AAC_6_0 = (141 shl 16) or 6;                       // C L R Ls Rs Cs
	kAudioChannelLayoutTag_AAC_6_1 = (142 shl 16) or 7;                       // C L R Ls Rs Cs Lfe
	kAudioChannelLayoutTag_AAC_7_0 = (143 shl 16) or 7;                       // C L R Ls Rs Rls Rrs
	kAudioChannelLayoutTag_AAC_7_1 = kAudioChannelLayoutTag_MPEG_7_1_B;   // C Lc Rc L R Ls Rs Lfe
	kAudioChannelLayoutTag_AAC_Octagonal = (144 shl 16) or 8;                       // C L R Ls Rs Rls Rrs Cs

	kAudioChannelLayoutTag_TMH_10_2_std = (145 shl 16) or 16;                      // L R C Vhc Lsd Rsd Ls Rs Vhl Vhr Lw Rw Csd Cs LFE1 LFE2
	kAudioChannelLayoutTag_TMH_10_2_full = (146 shl 16) or 21;                      // TMH_10_2_std plus: Lc Rc HI VI Haptic

	kAudioChannelLayoutTag_AC3_1_0_1 = (149 shl 16) or 2;                       // C LFE
	kAudioChannelLayoutTag_AC3_3_0 = (150 shl 16) or 3;                       // L C R
	kAudioChannelLayoutTag_AC3_3_1 = (151 shl 16) or 4;                       // L C R Cs
	kAudioChannelLayoutTag_AC3_3_0_1 = (152 shl 16) or 4;                       // L C R LFE
	kAudioChannelLayoutTag_AC3_2_1_1 = (153 shl 16) or 4;                       // L R Cs LFE
	kAudioChannelLayoutTag_AC3_3_1_1 = (154 shl 16) or 5;                       // L C R Cs LFE

	kAudioChannelLayoutTag_DiscreteInOrder = (147 shl 16) or 0;                       // needs to be ORed with the actual number of channels  
	kAudioChannelLayoutTag_Unknown = $FFFF0000;                           // needs to be ORed with the actual number of channels  


// Deprecated constants

{! @enum           MPEG-4 Audio Object IDs
    @deprecated     in version 10.5

    @abstract       Constants that describe the various kinds of MPEG-4 audio data.
    @discussion     These constants are used in the flags field of an AudioStreamBasicDescription
                    that describes an MPEG-4 audio stream.
}
const
	kMPEG4Object_AAC_Main = 1;
	kMPEG4Object_AAC_LC = 2;
	kMPEG4Object_AAC_SSR = 3;
	kMPEG4Object_AAC_LTP = 4;
	kMPEG4Object_AAC_SBR = 5;
	kMPEG4Object_AAC_Scalable = 6;
	kMPEG4Object_TwinVQ = 7;
	kMPEG4Object_CELP = 8;
	kMPEG4Object_HVXC = 9;    

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
