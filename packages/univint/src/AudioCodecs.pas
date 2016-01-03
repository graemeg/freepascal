{==================================================================================================
     File:       AudioUnit/AudioCodec.h

     Contains:   A component API for encoding/decoding audio data.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  (c) 1985-2008 by Apple, Inc., all rights reserved.
=======
     Copyright:  (c) 1985-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Copyright:  (c) 1985-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Copyright:  (c) 1985-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Copyright:  (c) 1985-2008 by Apple Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
     Copyright:  (c) 1985-2008 by Apple Inc., all rights reserved.
>>>>>>> origin/fixes_2.4

     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org

==================================================================================================}
{  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, October 2012 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
                     http://www.freepascal.org/bugs.html

==================================================================================================}
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4

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

unit AudioCodecs;
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
uses MacTypes,CoreAudioTypes,AudioComponents;
{$endc} {not MACOSALLINCLUDE}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}

{!
	@header AudioCodec
 
	This header defines the property sets and the public API for various audio codecs.

	<h2>Theory of Operation</h2>
 
	AudioCodec components translate audio data from one format to another. There
	are three kinds of AudioCodec components. Decoder components ('adec') 
	translate data that isn't in linear PCM into linear PCM formatted data. 
	Encoder components ('aenc') translate linear PCM data into some other format. 
	Unity codecs ('acdc') translate between different flavors of the same type 
	(e.g. 16 bit signed integer linear PCM into 32 bit floating point linear PCM).
 
	AudioCodec components are standard components and are managed by the Component
	Manager.
 
	Once an AudioCodec is found that implements the translation in question,
	it has to be set up to do the translation. This can be done by setting the
	appropriate properties or by calling AudioCodecInitialize. If the translation
	is specified by properties, AudioCodecInitialize still needs to be called
	prior to appending input data or producing output data.
 
	AudioCodecInitialize puts the codec into the "initialized" state. In this state,
	the format information for the translation cannot be changed. The codec
	has to be in the initialized state for AudioCodecAppendInputData and
	AudioCodecProduceOutputData to work. They will return kAudioCodecStateError
	if the codec isn't initialized.
 
	AudioCodecUninitialize will return the codec to the uninitialized state and
	release any allocated resources. The codec may then be configured freely. It is not
	necessary to call AudioCodecUninitialize prior to closing the codec.
 
	Once in the initialized state, the codec is ready to receive input and produce
	output using the AudioCodecAppendInputData and AudioCodecProduceOutputData
	routines. Input data can be fed into an encoder and some decoders in any size (even 
	byte by byte). Input data fed to a decoder should be in terms of whole packets in the 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	encoded format if the format is variable bit rate and is not self framing (e.g. MPEG-4 AAC). 
=======
	encoded format if the format is variable bit rate and is not self framing (e.g. MPEG 4 AAC). 
>>>>>>> graemeg/cpstrnew
=======
	encoded format if the format is variable bit rate and is not self framing (e.g. MPEG 4 AAC). 
>>>>>>> graemeg/cpstrnew
=======
	encoded format if the format is variable bit rate and is not self framing (e.g. MPEG 4 AAC). 
>>>>>>> graemeg/cpstrnew
=======
	encoded format if the format is variable bit rate and is not self framing (e.g. MPEG 4 AAC). 
>>>>>>> origin/cpstrnew
=======
	encoded format if the format is variable bit rate and is not self framing (e.g. MPEG 4 AAC). 
>>>>>>> origin/fixes_2.4
	Output data can only be produced in whole packet sizes. Both routines will return 
	the amount of data they consume/produce.
 
	AudioCodecProduceOutputData also returns a status code to the caller that
	indicates the result of the operation (success or failure) as well as the
	state of the input buffer.
	
	The combination of AppendInputData and ProduceOutputPackets can be thought of a "push-pull"
	model of data handling. First, the input data is pushed into the component and the 
	resulting output data gets pulled out of that same component.
 
	Basic Workflow
	1. Find the appropriate codec component
	2. Open the codec component
	3. Configure it (AudioCodecGetPropertyInfo, AudioCodecGetProperty, AudioCodecSetProperty)
	4. AudioCodecInitialize
	5. Loop
		a. AppendInputData (EOF is signaled by passing a 0-sized buffer)
		b. ProduceOutputPackets
	6. Close the codec component
	
 }

//=============================================================================


//=============================================================================
//#pragma mark Types specific to AudioCodecs
//=============================================================================


type
	AudioCodec = AudioComponentInstance;
	AudioCodecPropertyID = UInt32;

{!
    @struct AudioCodecMagicCookieInfo
 
	@abstract Structure holding the <em>magic cookie</em> information.
 
	@discussion Passed as input to AudioCodecGetProperty for kAudioCodecPropertyFormatList.
				The first four + sizeof(void *) bytes of the buffer pointed at by outPropertyData
				will contain this struct.
 
	@field mMagicCookieSize
        The size of the magic cookie
	@field mMagicCookie
        Generic const pointer to magic cookie
}
type
	AudioCodecMagicCookieInfo = record
		mMagicCookieSize: UInt32;
		mMagicCookie: (*const*) UnivPtr;
	end;
	AudioCodecMagicCookieInfoPtr = ^AudioCodecMagicCookieInfo;
	
//=============================================================================
//#pragma mark AudioCodec Component Constants
//=============================================================================


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_OS_IPHONE}
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
{!
	@enum           AudioCodecComponentType
 
	@discussion     Collection of audio codec component types
 
	@constant		kAudioDecoderComponentType
					A codec that translates data in some other format into linear PCM.
					The component subtype specifies the format ID of the other format.
	@constant		kAudioEncoderComponentType
					A codec that translates linear PCM data into some other format
					The component subtype specifies the format ID of the other format
	@constant		kAudioUnityCodecComponentType
					A codec that translates between different flavors of the same format
					The component subtype specifies the format ID of this format.
}
const
	kAudioDecoderComponentType = FourCharCode('adec');
	kAudioEncoderComponentType = FourCharCode('aenc');
	kAudioUnityCodecComponentType = FourCharCode('acdc');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {TARGET_OS_IPHONE}
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


//=============================================================================
//#pragma	mark Global Codec Properties

//	Used with the AudioCodecXXXXPropertyXXXX family of routines.
//	All Audio Codec properties are readable only.
//=============================================================================

{!
	@enum		AudioCodecGlobalProperty

	@discussion	These properties reflect the capabilities of the underlying codec.
				The values of these properties are independent of the codec's internal
				state.
				
				These properties can be read at any time the codec is open.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@constant	kAudioCodecPropertyNameCFString
					The name of the codec component as a CFStringRef. The CFStringRef
					retrieved via this property must be released by the caller.
	@constant	kAudioCodecPropertyManufacturerCFString
					The manufacturer of the codec as a CFStringRef. The CFStringRef 
					retrieved via this property must be released by the caller.
	@constant	kAudioCodecPropertyFormatCFString
					The name of the codec's format as a CFStringRef. The CFStringRef
					retrieved via this property must be released by the caller.
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
	@constant	kAudioCodecPropertySupportedInputFormats
					An array of AudioStreamBasicDescription structs describing what formats 
					the codec supports for input data
	@constant	kAudioCodecPropertySupportedOutputFormats
					An array of AudioStreamBasicDescription structs describing what formats 
					the codec supports for output data
 	@constant	kAudioCodecPropertyAvailableInputSampleRates
					An array of AudioValueRange indicating the valid ranges for the
					input sample rate of the codec.
					Required for encoders.
					(see also kAudioCodecPropertyApplicableInputSampleRates)
	@constant	kAudioCodecPropertyAvailableOutputSampleRates
					An array of AudioValueRange indicating the valid ranges for the
					output sample rate of the codec.
					Required for encoders.
					(see also kAudioCodecPropertyApplicableOutputSampleRates)
	@constant	kAudioCodecPropertyAvailableBitRateRange
					An array of AudioValueRange that indicate the target bit rates
					supported by the encoder. This can be total bit rate or bit
					rate per channel as appropriate. 
					This property is only relevant to encoders.
					(see also kAudioCodecPropertyApplicableBitRateRange)
	@constant	kAudioCodecPropertyMinimumNumberInputPackets
					A UInt32 indicating the minimum number of input packets
					that need to be supplied to the codec. The actual input the
					codec accepts could be less than this.
					For most codecs this value will be 1.
	@constant	kAudioCodecPropertyMinimumNumberOutputPackets
					A UInt32 indicating the minimum number of output packets
					that need to be handled from the codec. The actual output
					might be less than this.
					For most codecs this value will be 1.
	@constant	kAudioCodecPropertyAvailableNumberChannels
					An array of UInt32 that specifies the number of channels the codec is
					capable of encoding or decoding to. 0xFFFFFFFF means any number
					of channels.
	@constant	kAudioCodecPropertyDoesSampleRateConversion
					A UInt32 indicating if the codec wants to do a sample rate conversion (if 
					necessary) because it can do it in a way that is meaningful for quality.
					Value is 1 if true, 0 otherwise.
	@constant	kAudioCodecPropertyAvailableInputChannelLayoutTags
					An array of AudioChannelLayoutTag that specifies what channel layouts the codec is
					capable of using on input.
	@constant	kAudioCodecPropertyAvailableOutputChannelLayoutTags
					An array of AudioChannelLayoutTag that specifies what channel layouts the codec is
					capable of using on output.
	@constant	kAudioCodecPropertyInputFormatsForOutputFormat
					An array of AudioStreamBasicDescription indicating what the codec supports
					for input data given an output format that's passed in as the first member of
					the array (and is overwritten on the reply). Always a subset of 
					kAudioCodecPropertySupportedInputFormats
	@constant	kAudioCodecPropertyOutputFormatsForInputFormat
					An array of AudioStreamBasicDescription indicating what the codec supports
					for output data given an input format that's passed in as the first member of
					the array (and is overwritten on the reply). Always a subset of 
					kAudioCodecPropertySupportedOutputFormats
	@constant	kAudioCodecPropertyFormatInfo
					Takes an AudioFormatInfo on input. This AudioformatInfo is validated either through
					the provided magic cookie or the AudioStreamBasicDescription and where applicable,
					wildcards are overwritten with default values.
}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	kAudioCodecPropertyNameCFString = FourCharCode('lnam');
	kAudioCodecPropertyManufacturerCFString = FourCharCode('lmak');
	kAudioCodecPropertyFormatCFString = FourCharCode('lfor');
	kAudioCodecPropertyHasVariablePacketByteSizes = FourCharCode('vpk?');
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
	kAudioCodecPropertySupportedInputFormats = FourCharCode('ifm#');
	kAudioCodecPropertySupportedOutputFormats = FourCharCode('ofm#');
	kAudioCodecPropertyAvailableInputSampleRates = FourCharCode('aisr');
	kAudioCodecPropertyAvailableOutputSampleRates = FourCharCode('aosr');
	kAudioCodecPropertyAvailableBitRateRange = FourCharCode('abrt');
	kAudioCodecPropertyMinimumNumberInputPackets = FourCharCode('mnip');
	kAudioCodecPropertyMinimumNumberOutputPackets = FourCharCode('mnop');
	kAudioCodecPropertyAvailableNumberChannels = FourCharCode('cmnc');
	kAudioCodecPropertyDoesSampleRateConversion = FourCharCode('lmrc');
	kAudioCodecPropertyAvailableInputChannelLayoutTags = FourCharCode('aicl');
	kAudioCodecPropertyAvailableOutputChannelLayoutTags = FourCharCode('aocl');
	kAudioCodecPropertyInputFormatsForOutputFormat = FourCharCode('if4o');
	kAudioCodecPropertyOutputFormatsForInputFormat = FourCharCode('of4i');
	kAudioCodecPropertyFormatInfo = FourCharCode('acfi');

//=============================================================================
//#pragma	mark Instance Codec Properties

//	Used with the AudioCodecXXXXPropertyXXXX family of routines.
//=============================================================================

{!
	@enum			AudioCodecInstanceProperty
 
	@discussion		Properties which can be set or read on an instance of the
					underlying audio codec. These properties are dependent on the 
					codec's current state. A property may be read/write or read
					only, depending on the data format of the codec.
					
					These properties may have different values depending on whether the
					codec is initialized or not. All properties can be read at any time
					the codec is open. However, to ensure the codec is in a valid 
					operational state and therefore the property value is valid the codec
					must be initialized at the time the property is read.
					
					Properties that are writable are only writable when the codec
					is not initialized.
 
	@constant		kAudioCodecPropertyInputBufferSize
						A UInt32 indicating the maximum input buffer size for the codec
						in bytes. 
						Not writable, but can vary on some codecs depending on the bit stream 
						format being handled.
	@constant		kAudioCodecPropertyPacketFrameSize
						A UInt32 indicating the number of frames of audio data encapsulated in each
						packet of data in the codec's format. For encoders, this is the
						output format. For decoders this is the input format.
						Formats with variable frames per packet should return a maximum value 
						for this property.
						Not writable.
	@constant		kAudioCodecPropertyHasVariablePacketByteSizes
						A UInt32 where 0 indicates that all packets in the codec's format
						have the same byte size (sometimes referred to as CBR codecs),
						and 1 indicates that they vary in size (sometimes referred to as 
						VBR codecs). The maximum size of a variable packet is up to 
						the one indicated in kAudioCodecPropertyMaximumPacketByteSize.
						Any codec that reports 1 for this property must be able to handle packet
						descriptions, though it does not have to require them.
						May be writable.
	@constant		kAudioCodecPropertyMaximumPacketByteSize
						A UInt32 indicating the maximum number of bytes a packet of data
						in the codec's format will be. If the format is constant bit rate,
						all packets will be this size. If it is variable bit rate, the packets
						will never exceed this size.
						This always refers to the encoded data, so for encoders it refers to the
						output data and for decoders the input data.
						Not writable.
	@constant		kAudioCodecPropertyCurrentInputFormat
						An AudioStreamBasicDescription describing the format the codec
						expects its input data in
						Almost always writable, but if the codec only supports one unique input format
						it does not have to be
	@constant		kAudioCodecPropertyCurrentOutputFormat
						An AudioStreamBasicDescription describing the format the codec
						provides its output data in
						Almost always writable, but if the codec only supports one unique output format
						it does not have to be
	@constant		kAudioCodecPropertyMagicCookie
						An untyped buffer of out of band configuration data the codec
						requires to process the stream of data correctly. The contents
						of this data is private to the codec. 
						Not all codecs have magic cookies. If a call to AudioCodecGetPropertyInfo
						returns a size greater than 0 then the codec may take one.
						Writable if present.
	@constant		kAudioCodecPropertyUsedInputBufferSize
						A UInt32 indicating the number of bytes in the codec's input
						buffer. The amount of available buffer space is	simply the
						answer from kAudioCodecPropertyInputBufferSize minus the answer
						from this property.
						Not writable.
	@constant		kAudioCodecPropertyIsInitialized
						A UInt32 where 0 means the codec is uninitialized and anything
						else means the codec is initialized. This should never be settable directly.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Must be set by AudioCodecInitialize and AudioCodecUninitialize.
=======
						Must be set by AudioCodecInitialize and AudioCodecUnitialize.
>>>>>>> graemeg/cpstrnew
=======
						Must be set by AudioCodecInitialize and AudioCodecUnitialize.
>>>>>>> graemeg/cpstrnew
=======
						Must be set by AudioCodecInitialize and AudioCodecUnitialize.
>>>>>>> graemeg/cpstrnew
=======
						Must be set by AudioCodecInitialize and AudioCodecUnitialize.
>>>>>>> origin/cpstrnew
=======
						Must be set by AudioCodecInitialize and AudioCodecUnitialize.
>>>>>>> origin/fixes_2.4
	@constant		kAudioCodecPropertyCurrentTargetBitRate
						A UInt32 containing the number of bits per second to aim for when encoding
						data. This property is usually only relevant to encoders, but if a decoder
						can know what bit rate it's set to it may report it.
						This property is irrelevant if the encoder is configured as kAudioCodecBitRateControlMode_Variable.
						Writable on encoders if supported.
	@constant		kAudioCodecPropertyCurrentInputSampleRate
						A Float64 containing the current input sample rate in Hz. No Default.
						May be writable. If only one sample rate is supported it does not have to be.
	@constant		kAudioCodecPropertyCurrentOutputSampleRate
						A Float64 containing the current output sample rate in Hz. No Default.
						May be writable. If only one sample rate is supported it does not have to be.
	@constant		kAudioCodecPropertyQualitySetting
						A UInt32 that sets the tradeoff between sound quality and CPU time consumption.
						The property value is between [0 - 0x7F].
						Some enum constants are defined below for convenience.
						Writable if supported.
	@constant		kAudioCodecPropertyApplicableBitRateRange
						An array of AudioValueRange indicating the target bit rates
						supported by the encoder in its current configuration.
						This property is only relevant to encoders.
						See also kAudioCodecPropertyAvailableBitRateRange.
						Not writable.
	@constant		kAudioCodecPropertyApplicableInputSampleRates
						An array of AudioValueRange indicating the valid ranges for the
						input sample rate of the codec for the current bit rate. 
						This property is only relevant to encoders.
						See also kAudioCodecPropertyAvailableInputSampleRates.
						Not writable.
	@constant		kAudioCodecPropertyApplicableOutputSampleRates
						An array of AudioValueRange indicating the valid ranges for the
						output sample rate of the codec for the current bit rate. 
						This property is only relevant to encoders.
						See also kAudioCodecPropertyAvailableOutputSampleRates.
						Not writable.
	@constant		kAudioCodecPropertyPaddedZeros
						A UInt32 indicating the number of zeros (samples) that were appended
						to the last packet of input data to make a complete packet encoding.
						Encoders only. No default.
						Not writable.
	@constant		kAudioCodecPropertyPrimeMethod
						A UInt32 specifying priming method.
						See enum below.
						May be writable. Some encoders offer the option of padding out the last packet, and this 
						may be set here.
	@constant		kAudioCodecPropertyPrimeInfo
						A pointer to an AudioCodecPrimeInfo struct.
						Not writable on encoders. On decoders this may be writable, telling the decoder to trim the
						first and/or last packet.
	@constant		kAudioCodecPropertyCurrentInputChannelLayout
						An AudioChannelLayout that specifies the channel layout that the codec is using for input.
						May be writable. If only one channel layout is supported it does not have to be.
	@constant		kAudioCodecPropertyCurrentOutputChannelLayout
						An AudioChannelLayout that specifies the channel layout that the codec is using for output.
						If settable on a encoder, it means the encoder can re-map channels
						May be writable. If only one channel layout is supported or the codec does no channel remapping
						(ie, output channel layout always equals the input channel layout) it does not have to be.
	@constant		kAudioCodecPropertySettings
						A CFDictionaryRef that lists both the settable codec settings and their values.
						Encoders only.
						Obviously this will be linked to many of the other properties listed herein and as such
						it potentially will cause synchronization problems. Therefore, when setting this property
						on an encoder a GetProperty should be done first to retrieve the current dictionary, 
						and only one setting within the dictionary should change with each SetProperty call, 
						as it is not guaranteed that changing one property will not have side effects.
						Writable if supported.
	@constant		kAudioCodecPropertyBitRateControlMode
						A UInt32 indicating which bit rate control mode will be applied to encoders that 
						can produce variable packet sizes (sometimes referred to as VBR encoders).
						Although the packet size may be variable, a constant bit rate can be maintained 
						over a transmission channel when decoding in real-time with a fixed end-to-end audio delay. 
						E.g., MP3 and MPEG-AAC use a bit reservoir mechanism to meet that constraint.
						See enum below. 
						Only needs to be settable if the codec supports multiple bit rate control strategies.
	@constant		kAudioCodecPropertyFormatList
						An array of AudioFormatListItem structs list all formats that can be handled by the decoder
						For decoders, takes a Magic Cookie that gets passed in on the GetProperty call. No default.
						On input, the outPropertyData parameter passed to GetProperty should begin with a 
						AudioCodecMagicCookieInfo struct which will be overwritten by the AudioFormatListItems 
						returned from the property. For encoders, returns a list of formats which will be in the
						bitstream. No input data required.
						Important note: this encoder property is only applicable to audio formats which are made of
						two or more layers where the base layers(s) can be decoded by systems which aren't capable of
						handling the enhancement layers. For example, a High Efficiency AAC bitstream which contains 
						an AAC Low Complexity base layer can be decoded by any AAC decoder.
	@constant		kAudioCodecPropertySoundQualityForVBR
						A UInt32 that sets a target sound quality level.
						Unlike kAudioCodecPropertyQualitySetting which is relevant to all BitRate Control Modes,
						this property only needs to be set by an encoder configured at kAudioCodecBitRateControlMode_Variable.
						The property value is between [0 - 0x7F].
						See also kAudioCodecPropertyQualitySetting
						Writable if supported.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @constant		kAudioCodecPropertyDelayMode
                        A UInt32 specifying the delay mode. See enum below.                        
                        Encoders only, writable if supported.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@constant		kAudioCodecPropertyMinimumDelayMode
						A UInt32 equal 1 sets the encoder, where applicable, in it's lowest possible delay mode. An encoder
						may prepend zero valued samples to the input signal in order to make additional delays, like e.g.
						from a filter, coincide on a block boundary. This operation, however, results in an increased
						encoding/ decoding delay which may be undesired and turned off with this property.
						Writable if supported.
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
 }
const
	kAudioCodecPropertyInputBufferSize = FourCharCode('tbuf');
	kAudioCodecPropertyPacketFrameSize = FourCharCode('pakf');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioCodecPropertyHasVariablePacketByteSizes = FourCharCode('vpk?');
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
	kAudioCodecPropertyMaximumPacketByteSize = FourCharCode('pakb');
	kAudioCodecPropertyCurrentInputFormat = FourCharCode('ifmt');
	kAudioCodecPropertyCurrentOutputFormat = FourCharCode('ofmt');
	kAudioCodecPropertyMagicCookie = FourCharCode('kuki');
	kAudioCodecPropertyUsedInputBufferSize = FourCharCode('ubuf');
	kAudioCodecPropertyIsInitialized = FourCharCode('init');
	kAudioCodecPropertyCurrentTargetBitRate = FourCharCode('brat');
	kAudioCodecPropertyCurrentInputSampleRate = FourCharCode('cisr');
	kAudioCodecPropertyCurrentOutputSampleRate = FourCharCode('cosr');
	kAudioCodecPropertyQualitySetting = FourCharCode('srcq');
	kAudioCodecPropertyApplicableBitRateRange = FourCharCode('brta');
	kAudioCodecPropertyApplicableInputSampleRates = FourCharCode('isra');
	kAudioCodecPropertyApplicableOutputSampleRates = FourCharCode('osra');
	kAudioCodecPropertyPaddedZeros = FourCharCode('pad0');
	kAudioCodecPropertyPrimeMethod = FourCharCode('prmm');
	kAudioCodecPropertyPrimeInfo = FourCharCode('prim');
	kAudioCodecPropertyCurrentInputChannelLayout = FourCharCode('icl ');
	kAudioCodecPropertyCurrentOutputChannelLayout = FourCharCode('ocl ');
	kAudioCodecPropertySettings = FourCharCode('acs ');
	kAudioCodecPropertyFormatList = FourCharCode('acfl');
	kAudioCodecPropertyBitRateControlMode = FourCharCode('acbf');
	kAudioCodecPropertySoundQualityForVBR = FourCharCode('vbrq');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioCodecPropertyDelayMode = FourCharCode('dmod');
=======
	kAudioCodecPropertyMinimumDelayMode = FourCharCode('mdel');
>>>>>>> graemeg/cpstrnew
=======
	kAudioCodecPropertyMinimumDelayMode = FourCharCode('mdel');
>>>>>>> graemeg/cpstrnew
=======
	kAudioCodecPropertyMinimumDelayMode = FourCharCode('mdel');
>>>>>>> graemeg/cpstrnew
=======
	kAudioCodecPropertyMinimumDelayMode = FourCharCode('mdel');
>>>>>>> origin/cpstrnew
=======
	kAudioCodecPropertyMinimumDelayMode = FourCharCode('mdel');
>>>>>>> origin/fixes_2.4


{!
	@enum			AudioCodecQuality
 
	@discussion		Constants to be used with kAudioCodecPropertyQualitySetting
 
	@constant		kAudioCodecQuality_Max
	@constant		kAudioCodecQuality_High
	@constant		kAudioCodecQuality_Medium
	@constant		kAudioCodecQuality_Low
	@constant		kAudioCodecQuality_Min
}
const
	kAudioCodecQuality_Max = $7F;
	kAudioCodecQuality_High = $60;
	kAudioCodecQuality_Medium = $40;
	kAudioCodecQuality_Low = $20;
	kAudioCodecQuality_Min = 0;


{!
	@enum			AudioCodecPrimeMethod
 
	@discussion		Constants to be used with kAudioCodecPropertyPrimeMethod.
 
	@constant		kAudioCodecPrimeMethod_Pre
						Primes with leading and trailing input frames
	@constant		kAudioCodecPrimeMethod_Normal
						Only primes with trailing (zero latency)
						leading frames are assumed to be silence
	@constant		kAudioCodecPrimeMethod_None
						Acts in "latency" mode
						both leading and trailing frames assumed to be silence
}
const
	kAudioCodecPrimeMethod_Pre = 0;
	kAudioCodecPrimeMethod_Normal = 1;
	kAudioCodecPrimeMethod_None = 2;


{!
	@enum			kAudioCodecPropertyBitRateControlMode
 
	@discussion		Constants defining various bit rate control modes
					to be used with kAudioCodecPropertyBitRateControlMode.
					These modes are only applicable to encoders that can produce
					variable packet sizes, such as AAC.

	@constant		kAudioCodecBitRateControlMode_Constant
						The encoder maintains a constant bit rate suitable for use over a transmission 
						channel when decoding in real-time with a fixed end-to-end audio delay.  
						Note that while a constant bit rate is maintained in this mode, the number of bits 
						allocated to encode each fixed length of audio data may be variable 
						(ie. packet sizes are variable).
						E.g., MP3 and MPEG-AAC use a bit reservoir mechanism to meet that constraint.
	@constant		kAudioCodecBitRateControlMode_LongTermAverage
						 The provided target bit rate is achieved over a long term average
						 (typically after the first 1000 packets). This mode is similar to 
						 kAudioCodecBitRateControlMode_Constant in the sense that the 
						 target bit rate will be maintained in a long term average. However, it does not 
						 provide constant delay when using constant bit rate transmission. This mode offers 
						 a better sound quality than kAudioCodecBitRateControlMode_Constant 
						 can, that is, a more efficient encoding is performed. 
	@constant		kAudioCodecBitRateControlMode_VariableConstrained
						Encoder dynamically allocates the bit resources according to the characteristics
						of the underlying signal. However, some constraints are applied in order to limit 
						the variation of the bit rate.
	@constant		kAudioCodecBitRateControlMode_Variable
						Similar to the VBR constrained mode, however the packet size is virtually unconstrained.
						The coding process targets constant sound quality, and the sound quality level is 
						set by kAudioCodecPropertySoundQualityForVBR.
						This mode usually provides the best tradeoff between quality and bit rate.
}
const
	kAudioCodecBitRateControlMode_Constant = 0;
	kAudioCodecBitRateControlMode_LongTermAverage = 1;
	kAudioCodecBitRateControlMode_VariableConstrained = 2;
	kAudioCodecBitRateControlMode_Variable = 3;

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @enum			AudioCodecDelayMode
 
    @discussion		Constants defining various delay modes to be used with kAudioCodecPropertyDelayMode.
                    The resulting priming frames are reflected in the kAudioCodecPropertyPrimeInfo property.
                    Note that for layered streams like aach and aacp, the priming information always refers
                    to the base layer.
    @constant		kAudioCodecDelayMode_Compatibility
                        In compatibility delay mode, the resulting priming corresponds to the default value defined by the
                        underlying codecs. For aac this number equals 2112 regardless of the sample rate and other settings.
    @constant		kAudioCodecDelayMode_Minimum
                        Sets the encoder, where applicable, in it's lowest possible delay mode. Any additional delays, like the one
                        introduced by filters/sample rate converters etc, aren't compensated by the encoder.
    @constant		kAudioCodecDelayMode_Optimal
                        In this mode, the resulting bitstream has the minimum amount of priming necessary for the decoder.
                        For aac this number is 1024 which corresponds to exactly one packet.
 }
const
	kAudioCodecDelayMode_Compatibility = 0;
	kAudioCodecDelayMode_Minimum = 1;
	kAudioCodecDelayMode_Optimal = 2;

{!
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
	@struct			AudioCodecPrimeInfo 
 
	@discussion		Specifies the number of leading and trailing empty frames
					which have to be inserted.
 
	@field			leadingFrames
						An unsigned integer specifying the number of leading empty frames
	@field			trailingFrames
						An unsigned integer specifying the number of trailing empty frames 
}
type
	AudioCodecPrimeInfo = record
		leadingFrames: UInt32;
		trailingFrames: UInt32;
	end;
	AudioCodecPrimeInfoPtr = ^AudioCodecPrimeInfo;
	

//=============================================================================
//#pragma mark -
//#pragma mark Constants for kAudioCodecPropertySettings
//=============================================================================

const
	kAudioSettings_TopLevelKey = 'name';
const
	kAudioSettings_Version = 'version';
const
	kAudioSettings_Parameters = 'parameters';
const
	kAudioSettings_SettingKey = 'key';
const
	kAudioSettings_SettingName = 'name';
const
	kAudioSettings_ValueType = 'value type';
const
	kAudioSettings_AvailableValues = 'available values';
const
	kAudioSettings_LimitedValues = 'limited values';
const
	kAudioSettings_CurrentValue = 'current value';
const
	kAudioSettings_Summary = 'summary';
const
	kAudioSettings_Hint = 'hint';
const
	kAudioSettings_Unit = 'unit';


{!
	@enum			AudioSettingsFlag
 
	@discussion		Constants to be used with kAudioSettings_Hint
					in the kAudioCodecPropertySettings property dictionary.
					Indicates any special characteristics of each parameter within the dictionary, 

	@constant		kAudioSettingsFlags_ExpertParameter
						If set, then the parameter is an expert parameter.
	@constant		kAudioSettingsFlags_InvisibleParameter
						If set, then the parameter should not be displayed. 
	@constant		kAudioSettingsFlags_MetaParameter
						If set, then changing this parameter may affect the values of other parameters. 
						If not set, then this parameter can be set without affecting the values of other parameters.
	@constant		kAudioSettingsFlags_UserInterfaceParameter
						If set, then this is only a user interface element and not reflected in the codec's bit stream.
}
const
	kAudioSettingsFlags_ExpertParameter = 1 shl 0;
	kAudioSettingsFlags_InvisibleParameter = 1 shl 1;
	kAudioSettingsFlags_MetaParameter = 1 shl 2;
	kAudioSettingsFlags_UserInterfaceParameter = 1 shl 3;


//=============================================================================
//#pragma mark -
//#pragma mark Status values returned from the AudioCodecProduceOutputPackets routine
//=============================================================================
{!
	@enum			AudioCodecProduceOutputPacketStatus
 
	@discussion		Possible return status
 
	@constant		kAudioCodecProduceOutputPacketFailure
						Couldn't complete the request due to an error. It is possible
						that some output data was produced. This is reflected in the value
						returned in ioNumberPackets. 
	@constant		kAudioCodecProduceOutputPacketSuccess
						The number of requested output packets was produced without incident
						and there isn't any more input data to process
	@constant		kAudioCodecProduceOutputPacketSuccessHasMore
						The number of requested output packets was produced and there is
						enough input data to produce at least one more packet of output data
	@constant		kAudioCodecProduceOutputPacketNeedsMoreInputData
						There was insufficient input data to produce the requested
						number of output packets, The value returned in ioNumberPackets
						holds the number of output packets produced.
	@constant		kAudioCodecProduceOutputPacketAtEOF
						The end-of-file marker was hit during the processing. Fewer
						than the requested number of output packets may have been
						produced. Check the value returned in ioNumberPackets for the
						actual number produced. Note that not all formats have EOF
						markers in them. 
}
const
	kAudioCodecProduceOutputPacketFailure = 1;
	kAudioCodecProduceOutputPacketSuccess = 2;
	kAudioCodecProduceOutputPacketSuccessHasMore = 3;
	kAudioCodecProduceOutputPacketNeedsMoreInputData = 4;
	kAudioCodecProduceOutputPacketAtEOF = 5;


//=============================================================================
//#pragma mark -
//#pragma mark Selectors for the component routines (preliminary)
//=============================================================================
{!
	@enum			AudioCodecSelectors
 
	@discussion		Allows selection of component routines supported the the AudioCodec API
					Used by the Component Manager.
 
	@constant		kAudioCodecGetPropertyInfoSelect
	@constant		kAudioCodecGetPropertySelect
	@constant		kAudioCodecSetPropertySelect
	@constant		kAudioCodecInitializeSelect
	@constant		kAudioCodecUninitializeSelect
	@constant		kAudioCodecAppendInputDataSelect
	@constant		kAudioCodecProduceOutputDataSelect
	@constant		kAudioCodecResetSelect
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@constant		kAudioCodecAppendInputBufferListSelect
	@constant		kAudioCodecProduceOutputBufferListSelect
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
}
const
	kAudioCodecGetPropertyInfoSelect = $0001;
	kAudioCodecGetPropertySelect = $0002;
	kAudioCodecSetPropertySelect = $0003;
	kAudioCodecInitializeSelect = $0004;
	kAudioCodecUninitializeSelect = $0005;
	kAudioCodecAppendInputDataSelect = $0006;
	kAudioCodecProduceOutputDataSelect = $0007;
	kAudioCodecResetSelect = $0008;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioCodecAppendInputBufferListSelect = $0009;
	kAudioCodecProduceOutputBufferListSelect = $000A;
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


//=============================================================================
//#pragma mark -
//#pragma mark Errors
//=============================================================================
{!
	@enum			AudioCodecErrors
 
	@discussion		Possible errors returned by audio codec components
 
	@constant		kAudioCodecNoError
	@constant		kAudioCodecUnspecifiedError
	@constant		kAudioCodecUnknownPropertyError
	@constant		kAudioCodecBadPropertySizeError
	@constant		kAudioCodecIllegalOperationError
	@constant		kAudioCodecUnsupportedFormatError
	@constant		kAudioCodecStateError
	@constant		kAudioCodecNotEnoughBufferSpaceError
}
const
	kAudioCodecNoError = 0;
	kAudioCodecUnspecifiedError = FourCharCode('what');
	kAudioCodecUnknownPropertyError = FourCharCode('who?');
	kAudioCodecBadPropertySizeError = FourCharCode('!siz');
	kAudioCodecIllegalOperationError = FourCharCode('nope');
	kAudioCodecUnsupportedFormatError = FourCharCode('!dat');
	kAudioCodecStateError = FourCharCode('!stt');
	kAudioCodecNotEnoughBufferSpaceError = FourCharCode('!buf');


//=============================================================================
//#pragma mark -
//#pragma mark Codec Property Management
//=============================================================================

{!
	@function		AudioCodecGetPropertyInfo
 
	@discussion		Retrieve information about the given property. The outSize argument
					will return the size in bytes of the current value of the property.
					The outWritable argument will return whether or not the property
					in question can be changed.
 
	@param			inCodec
						An AudioCodec instance
	@param			inPropertyID
						Property ID whose value should be read
	@param			outSize
						Size in bytes of the property
	@param			outWritable
						Flag indicating wether the underlying property can be modified or not 
 
	@result			The OSStatus value
}
function AudioCodecGetPropertyInfo( inCodec: AudioCodec; inPropertyID: AudioCodecPropertyID; var outSize: UInt32; var outWritable: Boolean ): OSStatus; external name '_AudioCodecGetPropertyInfo';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{!
	@function		AudioCodecGetProperty
 
	@discussion		Retrieve the indicated property data. On input, ioDataSize has the size
					of the data pointed to by outPropertyData. On output, ioDataSize will contain
					the amount written.
 
	@param			inCodec
						An AudioCodec instance
	@param			inPropertyID
						Property ID whose value should be read
	@param			ioPropertyDataSize
						Size in bytes of the property data
	@param			outPropertyData
						Pointer to the property data buffer

	@result			The OSStatus value
}
function AudioCodecGetProperty( inCodec: AudioCodec; inPropertyID: AudioCodecPropertyID; var ioPropertyDataSize: UInt32; outPropertyData: UnivPtr ): OSStatus; external name '_AudioCodecGetProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{!
	@function		AudioCodecSetProperty

	@discussion		Set the indicated property data.
 
	@param			inCodec
						An AudioCodec instance
	@param			inPropertyID
						Property ID whose value should be changed
	@param			inPropertyDataSize
						Size in bytes of the property data
	@param			inPropertyData
						Pointer to the property data buffer
 
	@result			The OSStatus value
}
function AudioCodecSetProperty( inCodec: AudioCodec; inPropertyID: AudioCodecPropertyID; inPropertyDataSize: UInt32; inPropertyData: {const} UnivPtr ): OSStatus; external name '_AudioCodecSetProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


//=============================================================================
//#pragma mark -
//#pragma mark Codec Data Handling Routines
//=============================================================================

{!
	@function		AudioCodecInitialize
 
	@discussion		This call will allocate any buffers needed and otherwise set the codec
					up to perform the indicated translation. If an argument is NULL, any
					previously set properties will be used for preparing the codec for work.
					Note that this routine will also validate the format information as useable.
 
	@param			inCodec
						An AudioCodec instance
	@param			inInputFormat
						Pointer to an input format structure
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@param			inOutputFormat
=======
	@param			inInputFormat
>>>>>>> graemeg/cpstrnew
=======
	@param			inInputFormat
>>>>>>> graemeg/cpstrnew
=======
	@param			inInputFormat
>>>>>>> graemeg/cpstrnew
=======
	@param			inInputFormat
>>>>>>> origin/cpstrnew
=======
	@param			inInputFormat
>>>>>>> origin/fixes_2.4
						Pointer to an output format structure
	@param			inMagicCookie
						Pointer to the magic cookie
	@param			inMagicCookieByteSize
						Size in bytes of the magic cookie
  
	@result			The OSStatus value
}
function AudioCodecInitialize( inCodec: AudioCodec; const (*var*) inInputFormat: AudioStreamBasicDescription; const (*var*) inOutputFormat: AudioStreamBasicDescription; inMagicCookie: {const} UnivPtr; inMagicCookieByteSize: UInt32 ): OSStatus; external name '_AudioCodecInitialize';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{!
	@function		AudioCodecUninitialize
  
	@discussion		This call will move the codec from the initialized state back to the
					uninitialized state. The codec will release any resources it allocated
					or claimed in AudioCodecInitialize.
 
	@param			inCodec
						An AudioCodec instance
 
	@result			The OSStatus value
}
function AudioCodecUninitialize( inCodec: AudioCodec ): OSStatus; external name '_AudioCodecUninitialize';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{!
	@function		AudioCodecAppendInputData
 
	@discussion		Append as much of the given data in inInputData to the codec's input buffer as possible
					and return in ioInputDataByteSize the amount of data used.
 
					The inPacketDescription argument is an array of AudioStreamPacketDescription
					structs that describes the packet layout. The number of elements in this array
					is indicated on input by ioNumberPackets. On return, this number indicates the number
					of packets consumed.
 
					Note also in this case that it is an error to supply less than a full packet
					of data at a time.
 
	@param			inCodec
						An AudioCodec instance
	@param			inInputData
						A const pointer to the input data
	@param			ioInputDataByteSize
						The size in bytes of the input data in inInputData on input,
						the number of bytes consumed on output
	@param			ioNumberPackets
						The number of packets
	@param			inPacketDescription
						The packet description pointer
 
	@result			The OSStatus value
}
function AudioCodecAppendInputData( inCodec: AudioCodec; inInputData: {const} UnivPtr; var ioInputDataByteSize: UInt32; var ioNumberPackets: UInt32; const (*var*) inPacketDescription: AudioStreamPacketDescription ): OSStatus; external name '_AudioCodecAppendInputData';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{!
	@function		AudioCodecProduceOutputPackets

	@discussion		Produce as many output packets as requested and the amount of input data
					allows for. The outStatus argument returns information about the codec's
					status to allow for proper data management. See the constants above for
					the possible values that can be returned.
 
					The outPacketDescription argument is an array of AudioStreamPacketDescription
					structs that describes the packet layout returned in outOutputData. This
					argument is optional. Pass NULL if this information is not to be returned.
					Note that this information is only provided when the output format isn't
					linear PCM.

					Note that decoders will always only produce linear PCM data in multiples of
					the number frames in a packet of the encoded format (as returned by
					kAudioCodecPropertyPacketFrameSize). Encoders will consume this many frames
					of linear PCM data to produce a packet of their format.
 
	@param			inCodec
						The AudioCodec instance
	@param			outOutputData
						Pointer to the output data buffer
	@param			ioOutputDataByteSize
						A pointer to the size
	@param			ioNumberPackets
						number of input/output packets
	@result			The OSStatus value
}
function AudioCodecProduceOutputPackets( inCodec: AudioCodec; outOutputData: UnivPtr; var ioOutputDataByteSize: UInt32; var ioNumberPackets: UInt32; var outPacketDescription: AudioStreamPacketDescription; var outStatus: UInt32 ): OSStatus; external name '_AudioCodecProduceOutputPackets';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_OS_IPHONE}
function AudioCodecAppendInputBufferList( inCodec: AudioCodec; const (*var*) inBufferList: AudioBufferList; var ioNumberPackets: UInt32; const (*var*) inPacketDescription: AudioStreamPacketDescription; var outBytesConsumed: UInt32 ): OSStatus; external name '_AudioCodecAppendInputBufferList';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7,__IPHONE_NA) *)

function AudioCodecProduceOutputBufferList( inCodec: AudioCodec; var ioBufferList: AudioBufferList; var ioNumberPackets: UInt32; var outPacketDescription: AudioStreamPacketDescription; var outStatus: UInt32 ): OSStatus; external name '_AudioCodecProduceOutputBufferList';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7,__IPHONE_NA) *)
{$endc} {not TARGET_OS_IPHONE}

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
{!
	@function		AudioCodecReset

	@discussion		Flushes all the data in the codec and clears the input buffer. Note that
					the formats, and magic cookie will be retained so they won't need to be
					set up again to decode the same data.
 
	@param			inCodec The audio codec descriptor
 
	@result			the OSStatus value
}
function AudioCodecReset( inCodec: AudioCodec ): OSStatus; external name '_AudioCodecReset';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
//=====================================================================================================================
type
	AudioCodecGetPropertyInfoProc = function( self: UnivPtr; inPropertyID: AudioCodecPropertyID; var outSize: UInt32; var outWritable: Boolean ): OSStatus;

type
	AudioCodecGetPropertyProc = function( self: UnivPtr; inPropertyID: AudioCodecPropertyID; var ioPropertyDataSize: UInt32; outPropertyData: UnivPtr ): OSStatus;

type
	AudioCodecSetPropertyProc = function( self: UnivPtr; inPropertyID: AudioCodecPropertyID; inPropertyDataSize: UInt32; inPropertyData: {const} UnivPtr ): OSStatus;

type
	AudioCodecInitializeProc = function( self: UnivPtr; const (*var*) inInputFormat: AudioStreamBasicDescription; const (*var*) inOutputFormat: AudioStreamBasicDescription; inMagicCookie: {const} UnivPtr; inMagicCookieByteSize: UInt32 ): OSStatus;

type
	AudioCodecUninitializeProc = function( self: UnivPtr ): OSStatus;

type
	AudioCodecAppendInputDataProc = function( self: UnivPtr; inInputData: {const} UnivPtr; var ioInputDataByteSize: UInt32; var ioNumberPackets: UInt32; const (*var*) inPacketDescription: AudioStreamPacketDescription ): OSStatus;

type
	AudioCodecProduceOutputPacketsProc = function( self: UnivPtr; outOutputData: UnivPtr; var ioOutputDataByteSize: UInt32; var ioNumberPackets: UInt32; var outPacketDescription: AudioStreamPacketDescription; var outStatus: UInt32 ): OSStatus;

type
	AudioCodecResetProc = function( self: UnivPtr ): OSStatus;

type
	AudioCodecAppendInputBufferListProc = function( self: UnivPtr; const (*var*) ioBufferList: AudioBufferList; var inNumberPackets: UInt32; const (*var*) inPacketDescription: AudioStreamPacketDescription; var outBytesConsumed: UInt32 ): OSStatus;

type
	AudioCodecProduceOutputBufferListProc = function( self: UnivPtr; var ioBufferList: AudioBufferList; var ioNumberPackets: UInt32; var outPacketDescription: AudioStreamPacketDescription; var outStatus: UInt32 ): OSStatus;

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

//=====================================================================================================================
//#pragma mark -
//#pragma mark Deprecated Properties

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{!
    @enum		AudioCodecProperty
    @deprecated
 
    @constant	kAudioCodecPropertyMinimumDelayMode
                    A UInt32 equal 1 sets the encoder, where applicable, in it's lowest possible delay mode. An encoder
                    may prepend zero valued samples to the input signal in order to make additional delays, like e.g.
                    from a filter, coincide on a block boundary. This operation, however, results in an increased
                    encoding/ decoding delay which may be undesired and turned off with this property.
                    Use the kAudioCodecPropertyDelayMode property instead with the value set to kAudioCodecDelayMode_Minimum
}
const
	kAudioCodecPropertyMinimumDelayMode = FourCharCode('mdel');

{!
	@enum		AudioCodecProperty
	@deprecated	in version 10.7

	@constant	kAudioCodecPropertyNameCFString
					The name of the codec component as a CFStringRef. The CFStringRef
					retrieved via this property must be released by the caller.
	@constant	kAudioCodecPropertyManufacturerCFString
					The manufacturer of the codec as a CFStringRef. The CFStringRef 
					retrieved via this property must be released by the caller.
	@constant	kAudioCodecPropertyFormatCFString
					The name of the codec's format as a CFStringRef. The CFStringRef
					retrieved via this property must be released by the caller.

}
const
	kAudioCodecPropertyNameCFString = FourCharCode('lnam');
	kAudioCodecPropertyManufacturerCFString = FourCharCode('lmak');
	kAudioCodecPropertyFormatCFString = FourCharCode('lfor');		
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

{!
	@enum		AudioCodecProperty
	@deprecated	in version 10.5
 
	@constant	kAudioCodecPropertyRequiresPacketDescription
					A UInt32 where a non-zero value indicates that the format the codec implements
					requires that an AudioStreamPacketDescription array must be supplied with any data
					in that format. Note that this implies that data must also be handled strictly in
					packets. For a decoder, this applies to input data. For an encoder, it applies to
					output data, which means that the encoder will be filling out provided packet descriptions
					on output.
					A decoder must be able to handle packet descriptions even if it does not require them.
					An encoder does not have to fill out packet descriptions if it does not require them.
					Redundant due to kAudioCodecPropertyHasVariablePacketByteSizes. Codecs with variable-sized 
					packets must handle packet descriptions while codecs with constant-sized packets do not 
					have to.
	@constant	kAudioCodecPropertyAvailableBitRates
					An array of UInt32 that indicates the target bit rates
					supported by the encoder. This property is only relevant to
					encoders. Replaced with kAudioCodecPropertyAvailableBitRateRange
	@constant	kAudioCodecExtendFrequencies
					A UInt32 indicating whether an encoder should extend its cutoff frequency
					if such an option exists. 0 == extended frequencies off, 1 == extended frequencies on
					e.g. some encoders normally cut off the signal at 16 kHz but can encode up to 20 kHz if 
					asked to.
					Redundant.
	@constant	kAudioCodecUseRecommendedSampleRate
					For encoders that do sample rate conversion, a UInt32 indicating whether or
					not the encoder is using the recommended sample rate for the given input. 
					A value of 0 indicates it isn't, 1 indicates it is.
					This property is read only and indicates whether or not a user has explicitly set an output 
					sample rate.
					Redundant as 0.0 for a sample rate means let the codec decide.
	@constant	kAudioCodecOutputPrecedence
					For encoders that do sample rate conversion, a UInt32 indicating whether the
					bit rate, sample rate, or neither have precedence over the other. See enum below.
					Redundant because precedence is implicitly set by either providing a non-zero bit rate or 
					sample rate and setting the other to zero (which allows the encoder to choose any applicable rate). 
					If both values are set to non-zero, neither value has precedence.
	@constant	kAudioCodecDoesSampleRateConversion
					Renamed to kAudioCodecPropertyDoesSampleRateConversion
	@constant	kAudioCodecBitRateFormat
					Renamed to kAudioCodecPropertyBitRateControlMode
	@constant	kAudioCodecInputFormatsForOutputFormat
					Renamed to kAudioCodecPropertyInputFormatsForOutputFormat
	@constant	kAudioCodecOutputFormatsForInputFormat
					Renamed to kAudioCodecPropertyOutputFormatsForInputFormat
	@constant	kAudioCodecPropertyInputChannelLayout
					Renamed to kAudioCodecPropertyCurrentInputChannelLayout
	@constant	kAudioCodecPropertyOutputChannelLayout
					Renamed to kAudioCodecPropertyCurrentOutputChannelLayout
	@constant	kAudioCodecPropertyZeroFramesPadded
					Renamed to kAudioCodecPropertyPaddedZeros
 }
const
	kAudioCodecPropertyRequiresPacketDescription = FourCharCode('pakd');
	kAudioCodecPropertyAvailableBitRates = FourCharCode('brt#');
	kAudioCodecExtendFrequencies = FourCharCode('acef');
	kAudioCodecUseRecommendedSampleRate = FourCharCode('ursr');
	kAudioCodecOutputPrecedence = FourCharCode('oppr');
	kAudioCodecBitRateFormat = kAudioCodecPropertyBitRateControlMode;
	kAudioCodecDoesSampleRateConversion = kAudioCodecPropertyDoesSampleRateConversion;
	kAudioCodecInputFormatsForOutputFormat = kAudioCodecPropertyInputFormatsForOutputFormat;
	kAudioCodecOutputFormatsForInputFormat = kAudioCodecPropertyOutputFormatsForInputFormat;
	kAudioCodecPropertyInputChannelLayout = kAudioCodecPropertyCurrentInputChannelLayout;
	kAudioCodecPropertyOutputChannelLayout = kAudioCodecPropertyCurrentOutputChannelLayout;
	kAudioCodecPropertyAvailableInputChannelLayouts = kAudioCodecPropertyAvailableInputChannelLayoutTags;
	kAudioCodecPropertyAvailableOutputChannelLayouts = kAudioCodecPropertyAvailableOutputChannelLayoutTags;
	kAudioCodecPropertyZeroFramesPadded = kAudioCodecPropertyPaddedZeros;

{!
	@enum		AudioCodecBitRateFormat

	@deprecated	in version 10.5

	@discussion	Constants to be used with kAudioCodecBitRateFormat.
					This is deprecated. 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					Use kAudioCodecPropertyBitRateControlMode instead.
=======
					Use kAudioCodecVariablePacketSizeBitRateControlMode instead.
>>>>>>> graemeg/cpstrnew
=======
					Use kAudioCodecVariablePacketSizeBitRateControlMode instead.
>>>>>>> graemeg/cpstrnew
=======
					Use kAudioCodecVariablePacketSizeBitRateControlMode instead.
>>>>>>> graemeg/cpstrnew
=======
					Use kAudioCodecVariablePacketSizeBitRateControlMode instead.
>>>>>>> origin/cpstrnew
=======
					Use kAudioCodecVariablePacketSizeBitRateControlMode instead.
>>>>>>> origin/fixes_2.4
 
	@constant	kAudioCodecBitRateFormat_CBR is mapped to kAudioCodecBitRateControlMode_Constant
	@constant	kAudioCodecBitRateFormat_ABR is mapped to kAudioCodecBitRateControlMode_LongTermAverage
	@constant	kAudioCodecBitRateFormat_VBR is mapped to kAudioCodecBitRateControlMode_VariableConstrained
 }
const
	kAudioCodecBitRateFormat_CBR = kAudioCodecBitRateControlMode_Constant;
	kAudioCodecBitRateFormat_ABR = kAudioCodecBitRateControlMode_LongTermAverage;
	kAudioCodecBitRateFormat_VBR = kAudioCodecBitRateControlMode_VariableConstrained;

{!
	@enum		AudioCodecOutputPrecedence

	@deprecated	in version 10.5

	@discussion	Constants to be used with kAudioCodecOutputPrecedence
 
	@constant	kAudioCodecOutputPrecedenceNone
					Change in the bit rate or the sample rate are constrained by
					the other value.
	@constant	kAudioCodecOutputPrecedenceBitRate
					The bit rate may be changed freely,
					adjusting the sample rate if necessary
	@constant	kAudioCodecOutputPrecedenceSampleRate
					The sample rate may be changed freely,
					adjusting the bit rate if necessary
 }
const
	kAudioCodecOutputPrecedenceNone = 0;
	kAudioCodecOutputPrecedenceBitRate = 1;
	kAudioCodecOutputPrecedenceSampleRate = 2;

{!
	@typedef	MagicCookieInfo
 
	@deprecated	in version 10.5
 
	@discussion	renamed to AudioCodecMagicCookieInfo 
 }
type
	MagicCookieInfo = AudioCodecMagicCookieInfo;

{!
	@enum		AudioCodecSettingsHint
 
	@deprecated	in version 10.4
 
	@discussion	Constants to be used with kAudioSettings_Hint.
				This is deprecated.
				Use	AudioSettingsFlag instead.
 
	@constant	kHintBasic
	@constant	kHintAdvanced
	@constant	kHintHidden
 }
const
	kHintBasic = 0;
	kHintAdvanced = 1;
	kHintHidden = 2;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> origin/cpstrnew
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
