{
	File:		AUComponent.h

	Contains:	AudioUnit Interfaces

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright:	© 2002-2008 by Apple, Inc., all rights reserved.
=======
	Copyright:	© 2002-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
	Copyright:	© 2002-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
	Copyright:	© 2002-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
	Copyright:	© 2002-2008 by Apple Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
	Copyright:	© 2002-2008 by Apple Inc., all rights reserved.
>>>>>>> origin/fixes_2.4

	Bugs?:		For bug reports, consult the following page on
				the World Wide Web:

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					http://bugs.freepascal.org
}
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
}
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

unit AUComponent;
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
uses MacTypes,AudioComponents,CoreAudioTypes;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


//=====================================================================================================================
//#pragma mark Overview

{!
    @header AUComponent
	
	@discussion
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	This file defines the collection of API calls for an audio unit. An audio unit is a plugin 
	that can be loaded into an application's process and used to process or generate audio. An 
	audio unit is an AudioComponent type and so
	the AudioComponent APIs are used to find specific types of audio units, open and close them.
	
	Audio units use a general notion of description to specify an unique instance. The Type is the 
	general category of an audio unit. The SubType is an unique identifier specified by the 
	Manufacturer (provider) of the audio unit. The IDs that are used for Type are specified by 
	Apple, the Manufacturer ID should be specified by an unique identifier
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	This file defines the collection of API calls for an audio unit. An audio unit is a plugin that can be loaded
	into an application's process and used to process or generate audio. An audio unit is an AudioComponent type and so
	the AudioComponent APIs are used to find specific types of audio units, open and close them.
	
	Audio units use a general notion of description to specify an unique instance. The Type is the general category of 
	an audio unit. The SubType is an unique identifier specifed by the Manufacturer (provider) of the audio unit.
	The IDs that are used for Type are specified by apple, the Manufacturer ID should be specified by an unique identifier
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
	(as registered with apple). See AudioComponentDescription.
	
	Audio unit types are of the following (see below for more information)

		kAudioUnitType_Output					= 'auou',
		kAudioUnitType_MusicDevice				= 'aumu',
		kAudioUnitType_MusicEffect				= 'aumf',
		kAudioUnitType_FormatConverter			= 'aufc',	
		kAudioUnitType_Effect					= 'aufx',	
		kAudioUnitType_Mixer					= 'aumx',
		kAudioUnitType_Panner					= 'aupn',
		kAudioUnitType_OfflineEffect			= 'auol',
		kAudioUnitType_Generator				= 'augn',

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	An audio unit's general operations are:
=======
	An audio unit's general opeartions are:
>>>>>>> graemeg/cpstrnew
=======
	An audio unit's general opeartions are:
>>>>>>> graemeg/cpstrnew
=======
	An audio unit's general opeartions are:
>>>>>>> graemeg/cpstrnew
=======
	An audio unit's general opeartions are:
>>>>>>> origin/cpstrnew
=======
	An audio unit's general opeartions are:
>>>>>>> origin/fixes_2.4
		Open an audio unit (AudioComponentInstanceNew)
		Configure it based on the context - AudioUnitSetProperty
		Initialise the audio unit (AudioUnitInitialize)
			- at this point the audio unit is in a state where it can render audio
		Render audio (AudioUnitRender)
		
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	An important part of a render operation for an audio unit is to manipulate the various controls 
	that the unit provides to change the render effects; for instance to change the decay time of 
	a reverb, the cut off frequency of a filter, etc. These are called parameters, and 
	AudioUnitGetParameter and AudioUnitSetParameter are used to interact with these.
=======
	An important part of a render operation for an audio unit is to manipulate the various controls that the unit provides
	to change the render effects; for instance to change the decay time of a reverb, the cut off frequency of a filter, etc.
	These are called parameters, and AudioUnitGetParameter and AudioUnitSetParameter are used to interact with these.
>>>>>>> graemeg/cpstrnew
=======
	An important part of a render operation for an audio unit is to manipulate the various controls that the unit provides
	to change the render effects; for instance to change the decay time of a reverb, the cut off frequency of a filter, etc.
	These are called parameters, and AudioUnitGetParameter and AudioUnitSetParameter are used to interact with these.
>>>>>>> graemeg/cpstrnew
=======
	An important part of a render operation for an audio unit is to manipulate the various controls that the unit provides
	to change the render effects; for instance to change the decay time of a reverb, the cut off frequency of a filter, etc.
	These are called parameters, and AudioUnitGetParameter and AudioUnitSetParameter are used to interact with these.
>>>>>>> graemeg/cpstrnew
=======
	An important part of a render operation for an audio unit is to manipulate the various controls that the unit provides
	to change the render effects; for instance to change the decay time of a reverb, the cut off frequency of a filter, etc.
	These are called parameters, and AudioUnitGetParameter and AudioUnitSetParameter are used to interact with these.
>>>>>>> origin/cpstrnew
=======
	An important part of a render operation for an audio unit is to manipulate the various controls that the unit provides
	to change the render effects; for instance to change the decay time of a reverb, the cut off frequency of a filter, etc.
	These are called parameters, and AudioUnitGetParameter and AudioUnitSetParameter are used to interact with these.
>>>>>>> origin/fixes_2.4
	
	If any reconfiguration of the audio unit is required, then:
		uninitialise (AudioUnitUninitialise)
		Configure it based on the context - AudioUnitSetProperty
		Initialise the audio unit (AudioUnitInitialize)

	Once the host is finished with an audio unit, it closes it:
		Dispose audio unit (AudioComponentInstanceDispose)
	
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Audio units can be used programmatically (for instance a mixers could be used to render audio 
	for a game, a generator to play audio files, etc), or they can be hosted in Digital 
	Audio Workstation (DAW) applications such as Logic, Garage Band. In the DAW case, it is common 
	for an audio unit to provide a custom view to allow the user to interact with what can be 
	complex DSP operations that the audio unit performs. The view is retrieved from an audio unit 
	through AudioUnitGetProperty and then the host instantiates it 
	(see <AudioUnit/AUCocoaUIView.h>)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	Audio units can be used programmatically (for instance a mixers could be used to render audio for a game, a generator 
	to play audio files, etc), or they can be hosted in Digital Audio Workstation (DAW) applications such as Logic, Garage Band.
	In the DAW case, it is common for an audio unit to provide a custom view to allow the user to interact with what can be
	complex DSP opearations that the audio unit performs. The view is retrieved from an audio unit through AudioUnitGetProperty
	and then the host instantiates it (see <AudioUnit/AUCocoaUIView.h>)
	
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


//#if !__LP64__
//	#if PRAGMA_STRUCT_ALIGN
//		#pragma options align=mac68k
//	#elif PRAGMA_STRUCT_PACKPUSH
//		#pragma pack(push, 2)
//	#elif PRAGMA_STRUCT_PACK
//		#pragma pack(2)
//	#endif
//#endif
{$ifc not TARGET_CPU_64}
{$ALIGN MAC68K}
{$endc}

//=====================================================================================================================
//#pragma mark -
//#pragma mark Audio Unit Types
{!
	@typedef			AudioUnit
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion			An audio unit is of type AudioComponentInstance as defined in 
						AudioComponent.h
=======
	@discussion			An audio unit is of type AudioComponentInstance as defined in AudioComponent.h
>>>>>>> graemeg/cpstrnew
=======
	@discussion			An audio unit is of type AudioComponentInstance as defined in AudioComponent.h
>>>>>>> graemeg/cpstrnew
=======
	@discussion			An audio unit is of type AudioComponentInstance as defined in AudioComponent.h
>>>>>>> graemeg/cpstrnew
=======
	@discussion			An audio unit is of type AudioComponentInstance as defined in AudioComponent.h
>>>>>>> origin/cpstrnew
=======
	@discussion			An audio unit is of type AudioComponentInstance as defined in AudioComponent.h
>>>>>>> origin/fixes_2.4
}
type
	AudioUnit = AudioComponentInstance;


{!
    @enum           Audio Unit Types
    @abstract		different types of audio units
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		Audio unit's are classified into different types, where those types perform 
					different roles and functions.
					There are some general categories of functionality that apply across different 
					types of audio units:
					(1) Real-time usage
						The audio unit will complete its operations in less time that is 
						represented by the render buffer. All audio units with the exception of 
						the OfflineEffect should meet this criteria
					(2) Real-time I/O
						Will request the same amount of audio input as it is being asked to 
						produce for output. Effects, Panners, Mixers and MusicDevices are required 
						to adhere to this restriction. FormatConverter's can with some constraints
						be used in this situation (for instance, sample rate conversion, float-int),
						but the host of the audio unit is responsible for insuring this.
					(3) UI versus Programmatic usage
						UI usage covers the case of using an audio unit in a Digital Audio 
						Workstation (DAW) with appropriate UI (for example a filter in Garage Band 
						or Logic). Effects, Panners, MusicDevices are all expected to be usable 
						within this context. 
						Programmatic usage is where an audio unit is used by a host app as part of
						a general signal processing chain.
						For instance, a mixer audio unit can be used to take several different 
						audio sources in a game and mix them together. Mixers, Output units are 
						for programmatic usage. FormatConverter and Generator types are generally
						programmatic audio units, but if they can be used in a UI situation, they 
						specify a custom view. The app can then use that to decide that, with 
						appropriate constraints, the audio unit could be presented in a DAW type 
						application. For instance, the AUConverter audio unit from apple can do 
						sample rate conversion, etc, but has not general utility for a user in a 
						DAW app. Apple's Varispeed or AUTimePitch audio units can be used to change
						the playback rate and pitch and so could be used to good affect by a user 
						in a DAW type environment, as well as just providing this general 
						functionality to any program.
					
	@constant		kAudioUnitType_Output
					An output unit can be used as the head of an AUGraph. Apple provides a number 
					of output units that interface directly with an audio device
					
	@constant		kAudioUnitType_MusicDevice
					Used to describe software musical instruments such as samplers and 
					synthesisers. They respond to MIDI and create notes, which are then controlled
					through parameters or MIDI control messages. See <AudioUnit/MusicDevice.h>
										
	@constant		kAudioUnitType_MusicEffect
					Is an effect that is also able to respond directly to MIDI control messages, 
					typically through the mapping of these MIDI messages to different parameters 
					of the effect's DSP algorithm.
					
	@constant		kAudioUnitType_FormatConverter
					A format converter is a general category for audio units that can change the 
					format (for instance, sample rate conversion) from an input to an output, as 
					well as other, non-I/O type manipulations (like a deferred render or varispeed 
					type of operation). As such, a format converter can ask for as much or as 
					little audio input to produce a given output. They are still expected to
					complete their rendering within the time represented by the output buffer. 
					For format converters that have some utility as an "audio effect or processor", 
					it is quite common to provide an offline version of this audio unit as well. 
					For instance, Apple ships a format converter (for use in a "real-time" like 
					situation) and an offline version (for processing audio files) of the Time 
					Pitch and Varispeed audio units.
					
	@constant		kAudioUnitType_Effect
					An audio unit that will process some x number of audio input samples to produce 
					x number of audio output samples. The common case for an effect is to have a 
					single input to a single output, though some effects take side-chain inputs as 
					well. Effects can be run in "offline" contexts (such as processing a file), but 
					they are expected to run in real-time. A delay unit or reverb is a good 
					example of this.
					
	@constant		kAudioUnitType_Mixer
					An audio unit that takes some number of inputs, mixing them to provide 1 or 
					more audio outputs. A stere mixer (mono and stereo inputs to produce one 
					stereo output) is an example of this.
					
	@constant		kAudioUnitType_Panner
					A panner is a specialised effect that will pan a single audio input to a single
					output. Panner units are required to support a collection of standardised 
					parameters that specify the panning coordinates (aside from whatever custom 
					parameters the panner may provide). A surround panner is an example of this
					
	@constant		kAudioUnitType_Generator
					A generator will have no audio input, but will just produce audio output. In 
					some ways it is similar to a MusicDevice, except that a generator provides no 
					MIDI input, or notion of "notes". A tone generator is a good example of this.
					
	@constant		kAudioUnitType_OfflineEffect
					An offline effect is used to process data from a file and is also used to 
					publish a capability that cannot be run in real-time. For instance, the process
					of normalisation requires seeing the entire audio input before the scalar to 
					apply in the normalisation process can be estimated. As such, offline effects 
					also have a notion of a priming stage that can be performed before the actual 
					rendering/processing phase is executed.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		Audio unit's are classified into different types, where those types perform different roles and functions.
					There are some general categories of functionality that apply across different types of audio units:
					(1) Real-time usage
						The audio unit will complete its operations in less time that is represented by the render buffer.
						All audio units with the exception of the OfflineEffect should meet this criteria
					(2) Real-time I/O
						Will request the same amount of audio input as it is being asked to produce for output. Effects, Panners, 
						Mixers and MusicDevices are required to adhere to this restriction. FormatConverter's can with some contraints
						be used in this situation (for instance, sample rate conversion, float-int), but the host of the
						audio unit is responsible for insuring this.
					(3) UI versus Programmatic usage
						UI usage covers the case of using an audio unit in a Digital Audio Workstation (DAW) with appropriate
						UI (for example a filter in Garage Band or Logic). Effects, Panners, MusicDevices are all expected to be 
						usable within this context. 
						Programmatic usage is where an audio unit is used by a host app as part of a general signal processing chain.
						For instance, a mixer audio unit can be used to take several different audio sources in a game and mix them together.
						Mixers, Output units are for programmatic usage. 
						FormatConverter and Generator types are generally programmatic audio units, but if they can be used in a UI situation,
						they specify a custom view. The app can then use that to decide that, with appropraite constraints, the audio unit
						could be presented in a DAW type application. For instance, the AUConveter audio unit from apple can do sample rate
						conversion, etc, but has not general utility for a user in a DAW app. Apple's Varispeed or AUTimePitch audio units 
						can be used to change the playback rate and pitch and so could be used to good affect by a user in a DAW type environment,
						as well as just providing this general functionality to any program.
					
	@constant		kAudioUnitType_Output
					An output unit can be used as the head of an AUGraph. Apple provides a number of output units that interface
					directly with an audio device
					
	@constant		kAudioUnitType_MusicDevice
					Used to describe software musical instruments such as samplers and synthesisers. They respond to MIDI and create
					notes, which are then controlled through parameters or MIDI control messages. See <AudioUnit/MusicDevice.h>
										
	@constant		kAudioUnitType_MusicEffect
					Is an effect that is also able to respond directly to MIDI control messages, typically through the mapping of
					these MIDI messages to different parameters of the effect's DSP algorithm.
					
	@constant		kAudioUnitType_FormatConverter
					A format converter is a general category for audio units that can change the format (for instance, sample rate conversion)
					from an input to an output, as well as other, non-I/O type manipulations (like a deferred render or varispeed type of operation).
					As such, a format converter can ask for as much or as little audio input to produce a given output. They are still expected to
					complete their rendering within the time represented by the output buffer. 
					For format converters that have some utility as an "audio effect or processor", it is quite common to provide an offline version
					of this audio unit as well. For instance, Apple ships a format converter (for use in a "real-time" like situation) and an offline
					version (for processing audio files) of the Time Pitch and Varispeed audio units.
					
	@constant		kAudioUnitType_Effect
					An audio unit that will process some x number of audio input samples to produce x number of audio output samples. The common case for an 
					effect is to have a single input to a single output, though some effects take side-chain inputs as well. Effects can be run in
					"offline" contexts (such as procesing a file), but they are expected to run in real-time. A delay unit or reverb is
					a good example of this.
					
	@constant		kAudioUnitType_Mixer
					An audio unit that takes some number of inputs, mixing them to provide 1 or more audio outputs. A stere mixer (mono and stereo inputs
					to produce one stereo output) is an example of this.
					
	@constant		kAudioUnitType_Panner
					A panner is a specialised effect that will pan a single audio input to a single output. Panner units are required to support
					a collection of standardised parameters that specify the panning coordinates (aside from whatever custom parameters the
					panner may provide). A surround panner is an example of this
					
	@constant		kAudioUnitType_Generator
					A generator will have no audio input, but will just produce audio output. In some ways it is similar to a MusicDevice, except that a
					generator provides no MIDI input, or notion of "notes". A tone generator is a good example of this.
					
	@constant		kAudioUnitType_OfflineEffect
					An offline effect is used to process data from a file and is also used to publish a capability that cannot be run in real-time. For instance,
					the process of normalisation requires seeing the entire audio input before the scalar to apply in the normalisation process can be estimated.
					As such, offline effects also have a notion of a priming stage that can be performed before the actual rendering/processing phase is executed.
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
	kAudioUnitType_Output = FourCharCode('auou');
	kAudioUnitType_MusicDevice = FourCharCode('aumu');
	kAudioUnitType_MusicEffect = FourCharCode('aumf');
	kAudioUnitType_FormatConverter = FourCharCode('aufc');
	kAudioUnitType_Effect = FourCharCode('aufx');
	kAudioUnitType_Mixer = FourCharCode('aumx');
	kAudioUnitType_Panner = FourCharCode('aupn');
	kAudioUnitType_Generator = FourCharCode('augn');
	kAudioUnitType_OfflineEffect = FourCharCode('auol');

//=====================================================================================================================
//#pragma mark -
//#pragma mark Apple Audio Units
{!
    @enum           Apple audio unit manufacturer ID.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @discussion		the unique ID used to identifier audio units provided by Apple, Inc.
=======
    @discussion		the unique ID used to identifier audio units provided by Apple Inc.
>>>>>>> graemeg/cpstrnew
=======
    @discussion		the unique ID used to identifier audio units provided by Apple Inc.
>>>>>>> graemeg/cpstrnew
=======
    @discussion		the unique ID used to identifier audio units provided by Apple Inc.
>>>>>>> graemeg/cpstrnew
=======
    @discussion		the unique ID used to identifier audio units provided by Apple Inc.
>>>>>>> origin/cpstrnew
=======
    @discussion		the unique ID used to identifier audio units provided by Apple Inc.
>>>>>>> origin/fixes_2.4
}
const
	kAudioUnitManufacturer_Apple = FourCharCode('appl');
	
{!
	@enum			Apple output audio unit sub types 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		These are the subtypes for the various output units that apple ships. Output 
					units add an additional notion of Start and Stop
					see <AudioUnit/AudioOutputUnit.h>

	@constant		kAudioUnitSubType_GenericOutput
					A generic output unit provides the start/stop API, and provides the basic 
					services to convert Linear PCM formats.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		These are the subtypes for the various output units that apple ships. Output units add an additional notion of Start and Stop
					see <AudioUnit/AudioOutputUnit.h>

	@constant		kAudioUnitSubType_GenericOutput
					A generic output unit provides the start/stop API, and provides the basic services to convert Linear PCM formats.
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


	@constant		kAudioUnitSubType_HALOutput			
						- desktop only
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					The audio unit that interfaces to any audio device. The user specifies which 
					audio device to track. The audio unit can do input from the device as well as 
					output to the device. Bus 0 is used for the output side, bus 1 is used
=======
					The audio unit that interfaces to any audio device. The user specifies which audio device to track. The audio unit
					can do input from the device as well as output to the device. Bus 0 is used for the output side, bus 1 is used
>>>>>>> graemeg/cpstrnew
=======
					The audio unit that interfaces to any audio device. The user specifies which audio device to track. The audio unit
					can do input from the device as well as output to the device. Bus 0 is used for the output side, bus 1 is used
>>>>>>> graemeg/cpstrnew
=======
					The audio unit that interfaces to any audio device. The user specifies which audio device to track. The audio unit
					can do input from the device as well as output to the device. Bus 0 is used for the output side, bus 1 is used
>>>>>>> graemeg/cpstrnew
=======
					The audio unit that interfaces to any audio device. The user specifies which audio device to track. The audio unit
					can do input from the device as well as output to the device. Bus 0 is used for the output side, bus 1 is used
>>>>>>> origin/cpstrnew
=======
					The audio unit that interfaces to any audio device. The user specifies which audio device to track. The audio unit
					can do input from the device as well as output to the device. Bus 0 is used for the output side, bus 1 is used
>>>>>>> origin/fixes_2.4
					to get audio input from the device.
					
	@constant		kAudioUnitSubType_DefaultOutput		
						- desktop only
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					A specialisation of AUHAL that is used to track the user's selection of the 
					default device as set in the Sound Prefs
					
	@constant		kAudioUnitSubType_SystemOutput		
						- desktop only
					A specialisation of AUHAL that is used to track the user's selection of the 
					device to use for sound effects, alerts
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
					A specialisation of AUHAL that is used to track the user's selection of the default device as set in the Sound Prefs
					
	@constant		kAudioUnitSubType_SystemOutput		
						- desktop only
					A specialisation of AUHAL that is used to track the user's selection of the device to use for sound effects, alerts
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
					and other UI sounds.

	@constant		kAudioUnitSubType_RemoteIO			
						- iPhone only
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					The audio unit that interfaces to the audio I/O pathways of the iPhone. The 
					audio unit can do input as well as output. Bus 0 is used for the output side, 
					bus 1 is used to get audio input.
	@constant		kAudioUnitSubType_VoiceProcessingIO
						- Available on the desktop and with iPhone 3.0 or greater
					This audio unit can do input as well as output. Bus 0 is used for the output 
					side, bus 1 is used to get audio input (thus, on the iPhone, it works in a 
					very similar way to the Remote I/O). This audio unit does signal processing on 
					the incoming audio (taking out any of the audio that is played from the device 
					at a given time from the incoming audio).
=======
					The audio unit that interfaces to the audio I/O pathways of the iPhone. The audio unit
					can do input as well as output. Bus 0 is used for the output side, bus 1 is used
					to get audio input.
>>>>>>> graemeg/cpstrnew
=======
					The audio unit that interfaces to the audio I/O pathways of the iPhone. The audio unit
					can do input as well as output. Bus 0 is used for the output side, bus 1 is used
					to get audio input.
>>>>>>> graemeg/cpstrnew
=======
					The audio unit that interfaces to the audio I/O pathways of the iPhone. The audio unit
					can do input as well as output. Bus 0 is used for the output side, bus 1 is used
					to get audio input.
>>>>>>> graemeg/cpstrnew
=======
					The audio unit that interfaces to the audio I/O pathways of the iPhone. The audio unit
					can do input as well as output. Bus 0 is used for the output side, bus 1 is used
					to get audio input.
>>>>>>> origin/cpstrnew
=======
					The audio unit that interfaces to the audio I/O pathways of the iPhone. The audio unit
					can do input as well as output. Bus 0 is used for the output side, bus 1 is used
					to get audio input.
>>>>>>> origin/fixes_2.4
}
const
	kAudioUnitSubType_GenericOutput = FourCharCode('genr');
//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}
	kAudioUnitSubType_HALOutput = FourCharCode('ahal');
	kAudioUnitSubType_DefaultOutput = FourCharCode('def ');
	kAudioUnitSubType_SystemOutput = 'sys ';
//#else
{$elsec} { not TARGET_OS_IPHONE }
	kAudioUnitSubType_RemoteIO				= 'rioc';
//#endif;
{$endc} { not TARGET_OS_IPHONE }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitSubType_VoiceProcessingIO = FourCharCode('vpio');
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
	@enum			Apple music instrument audio unit sub types 

	@constant		kAudioUnitSubType_DLSSynth			
						- desktop only
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					A multi-timbral music device that can use sample banks in either DLS or 
					SoundFont formats. It fully supports GM-MIDI and the basic extensions of 
					GS-MIDI
	@constant		kAudioUnitSubType_Sampler			
					A mono-timbral music device which is a sampler synthesizer and supports full 
					interactive editing of all state.
=======
					A multi-timbral music device that can use sample banks in either DLS or SoundFont formats. It fully supports GM-MIDI
					and the basic extensions of GS-MIDI
>>>>>>> graemeg/cpstrnew
=======
					A multi-timbral music device that can use sample banks in either DLS or SoundFont formats. It fully supports GM-MIDI
					and the basic extensions of GS-MIDI
>>>>>>> graemeg/cpstrnew
=======
					A multi-timbral music device that can use sample banks in either DLS or SoundFont formats. It fully supports GM-MIDI
					and the basic extensions of GS-MIDI
>>>>>>> graemeg/cpstrnew
=======
					A multi-timbral music device that can use sample banks in either DLS or SoundFont formats. It fully supports GM-MIDI
					and the basic extensions of GS-MIDI
>>>>>>> origin/cpstrnew
=======
					A multi-timbral music device that can use sample banks in either DLS or SoundFont formats. It fully supports GM-MIDI
					and the basic extensions of GS-MIDI
>>>>>>> origin/fixes_2.4
}
//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}
const
	kAudioUnitSubType_DLSSynth = FourCharCode('dls ');
//#endif
{$endc} { not TARGET_OS_IPHONE }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitSubType_Sampler = FourCharCode('samp');

{!
	@enum			Apple converter audio unit sub types 
	@discussion		These are the subtypes for the various converter units that apple ships. 
					Except for AUConverter, which is available on both desktop and iPhone, these 
					audio units are only available on the desktop.

	@constant		kAudioUnitSubType_AUConverter
					An audio unit that uses an AudioConverter to do Linear PCM conversions (sample 
					rate, bit depth, interleaving).

	@constant		kAudioUnitSubType_TimePitch
						- desktop only
					An audio unit that can be used to have independent control of both playback 
					rate and pitch. It provides a generic view, so can be used in both a UI and 
					programmatic context. It also comes in an Offline version so can be used to 
					process audio files.
					
	@constant		kAudioUnitSubType_Varispeed
					An audio unit that can be used to control playback rate (as the rate is faster, 
					the pitch is higher). It provides a generic view, so can be used in both a UI 
					and programmatic context. It also comes in an Offline version so can be used 
					to process audio files.
					
	@constant		kAudioUnitSubType_DeferredRenderer
						- desktop only
					An audio unit that is used to get its input from a separate thread than the 
					thread that its render method is called. It thus allows an application to 
					introduce multiple threads into a rendering graph. There is a buffer sized 
					delay introduced between the input and output
					
	@constant		kAudioUnitSubType_Splitter
						- desktop only
					An audio unit that provides 2 output buses and 1 input bus. The audio unit 
					splits (duplicates) the input signal to the two output buses
					
	@constant		kAudioUnitSubType_Merger
						- desktop only
					An audio unit that provides 2 input buses and 2 output bus. The audio unit 
					merges the two inputs to the single output

	@constant		kAudioUnitSubType_NewTimePitch
					An audio unit that provides good quality time stretching and pitch shifting 
					while still being very fast.

	@constant		kAudioUnitSubType_AUiPodTime
						- iPhone only
					An audio unit that provides simple (and limited) control over playback rate 
					and time.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

{!
	@enum			Apple converter audio unit sub types 
	@discussion		These are the subtypes for the various converter units that apple ships. Except for AUConverter, which is available
					on both desktop and iPhone, these audio units are only available on the desktop.

	@constant		kAudioUnitSubType_AUConverter
					An audio unit that uses an AudioConverter to do Linear PCM conversions (sample rate, bit depth, interleaving).

	@constant		kAudioUnitSubType_TimePitch
						- desktop only
					An audio unit that can be used to have independent control of both playback rate and pitch. It provides a generic view,
					so can be used in both a UI and programmatic context. It also comes in an Offline version so can be used to process
					audio files.
					
	@constant		kAudioUnitSubType_Varispeed
						- desktop only
					An audio unit that can be used to control playback rate (as the rate is faster, the pitch is higher). It provides a generic view,
					so can be used in both a UI and programmatic context. It also comes in an Offline version so can be used to process
					audio files.
					
	@constant		kAudioUnitSubType_DeferredRenderer
						- desktop only
					An audio unit that is used to get its input from a separate thread than the thread that its render method is called. It thus
					allows an application to introduce multiple threads into a rendering graph. There is a buffer sized delay introduced between
					the input and output
					
	@constant		kAudioUnitSubType_Splitter
						- desktop only
					An audio unit that provides 2 output buses and 1 input bus. The audio unit splits (duplicates) the input signal to the two 
					output buses
					
	@constant		kAudioUnitSubType_Merger
						- desktop only
					An audio unit that provides 2 input buses and 2 output bus. The audio unit merges the two inputs to the single output

	@constant		kAudioUnitSubType_AUiPodTime
						- iPhone only
					An audio unit that provides simple (and limited) control over playback rate and time.
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
	kAudioUnitSubType_AUConverter = FourCharCode('conv');
	
//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitSubType_NewTimePitch = FourCharCode('nutp');
	kAudioUnitSubType_TimePitch = FourCharCode('tmpt');
	kAudioUnitSubType_DeferredRenderer = FourCharCode('defr');
	kAudioUnitSubType_Splitter = FourCharCode('splt');
	kAudioUnitSubType_Merger = FourCharCode('merg');
	kAudioUnitSubType_Varispeed = FourCharCode('vari');
	kAudioUnitSubType_RoundTripAAC = 'raac';
//#else
{$elsec} { not TARGET_OS_IPHONE }
	kAudioUnitSubType_AUiPodTime			= 'iptm';
	kAudioUnitSubType_AUiPodTimeOther = 'ipto';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	kAudioUnitSubType_TimePitch = FourCharCode('tmpt');
	kAudioUnitSubType_Varispeed = FourCharCode('vari');
	kAudioUnitSubType_DeferredRenderer = FourCharCode('defr');
	kAudioUnitSubType_Splitter = FourCharCode('splt');
	kAudioUnitSubType_Merger = FourCharCode('merg');
//#else
{$elsec} { not TARGET_OS_IPHONE }
	kAudioUnitSubType_AUiPodTime			= FourCharCode('iptm');
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
//#endif;
{$endc} { not TARGET_OS_IPHONE }

{!
	@enum			Apple effect audio unit sub types 
	@discussion		These are the subtypes for the various effect units that apple ships

	@constant		kAudioUnitSubType_Delay					
						- desktop only
					A delay audio unit
					
	@constant		kAudioUnitSubType_LowPassFilter			
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					A filter that passes frequencies below a specified cut-off frequency
					
	@constant		kAudioUnitSubType_HighPassFilter		
					A filter that passes frequencies above a specified cut-off frequency
					
	@constant		kAudioUnitSubType_BandPassFilter		
					A filter that passes frequencies between a low and high cut-off frequency.
					
	@constant		kAudioUnitSubType_HighShelfFilter		
					A filter that can be used to implement a "treble" control
					
	@constant		kAudioUnitSubType_LowShelfFilter		
					A filter that can be used to implement a "bass" control

	@constant		kAudioUnitSubType_ParametricEQ			
					A parametric EQ filter
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
						- desktop only
					A filter that passes frequencies below a specified cut-off frequency
					
	@constant		kAudioUnitSubType_HighPassFilter		
						- desktop only
					A filter that passes frequencies above a specified cut-off frequency
					
	@constant		kAudioUnitSubType_BandPassFilter		
						- desktop only
					A filter that passes frequencies between a low and high cut-off frequency.
					
	@constant		kAudioUnitSubType_HighShelfFilter		
						- desktop only
					A filter that can be used to implement a "treble" control
					
	@constant		kAudioUnitSubType_LowShelfFilter		
						- desktop only
					A filter that can be used to implement a "bass" control

	@constant		kAudioUnitSubType_ParametricEQ			
						- desktop only
					A parameteric EQ filter
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
					
	@constant		kAudioUnitSubType_GraphicEQ				
						- desktop only
					A 10 or 31 band Graphic EQ
					
	@constant		kAudioUnitSubType_PeakLimiter			
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					A peak limiter
					
	@constant		kAudioUnitSubType_DynamicsProcessor		
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
						- desktop only
					A peak limiter
					
	@constant		kAudioUnitSubType_DynamicsProcessor		
						- desktop only
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
					A dynamics compressor/expander
					
	@constant		kAudioUnitSubType_MultiBandCompressor	
						- desktop only
					A 4 band compressor/expander
					
	@constant		kAudioUnitSubType_MatrixReverb			
						- desktop only
					A reverb that can be used to simulate various and different spaces
					
	@constant		kAudioUnitSubType_SampleDelay			
						- desktop only
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					A delay that is used to delay the input a specified number of samples until 
					the output
=======
					A delay that is used to delay the input a specified number of samples until the output
>>>>>>> graemeg/cpstrnew
=======
					A delay that is used to delay the input a specified number of samples until the output
>>>>>>> graemeg/cpstrnew
=======
					A delay that is used to delay the input a specified number of samples until the output
>>>>>>> graemeg/cpstrnew
=======
					A delay that is used to delay the input a specified number of samples until the output
>>>>>>> origin/cpstrnew
=======
					A delay that is used to delay the input a specified number of samples until the output
>>>>>>> origin/fixes_2.4
					
	@constant		kAudioUnitSubType_Pitch					
						- desktop only
					An audio unit used to change the pitch
					
	@constant		kAudioUnitSubType_AUFilter				
						- desktop only
					A filter unit that combines 5 different filters (low, 3 mids, high)
					
	@constant		kAudioUnitSubType_NetSend				
						- desktop only
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					An audio unit that is used in conjunction with _NetReceive to send audio 
					across the network (or between different applications)
					
	@constant		kAudioUnitSubType_Distortion			
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
					An audio unit that is used in conjunction with _NetReceive to send audio across the network (or between different applications)
					
	@constant		kAudioUnitSubType_Distortion			
						- desktop only
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
					A distortion audio unit 
					
	@constant		kAudioUnitSubType_RogerBeep				
						- desktop only
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					An audio unit that can be used to emit a short tone in gaps between speech 
					- similar to the tones used in a walkie-talkie
					
	@constant		kAudioUnitSubType_AUiPodEQ				
						- iPhone only
					A simple graphic EQ with common presets
	
	@constant		kAudioUnitSubType_NBandEQ
						- iPhone only
					A generalized N-band graphic EQ with specifiable filter types per-band
	
	@constant		kAudioUnitSubType_Reverb2
						- iPhone only
					A reverb for iOS
}
const
	kAudioUnitSubType_PeakLimiter = FourCharCode('lmtr');
	kAudioUnitSubType_DynamicsProcessor = FourCharCode('dcmp');
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
					An audio unit that can be used to emit a short tone in gaps between speech - similar to the tones used in a walkie-talkie
					
	@constant		kAudioUnitSubType_AUiPodEQ				
						- iPhone only
					A graphic EQ
}
const
//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}
	kAudioUnitSubType_Delay = FourCharCode('dely');
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
	kAudioUnitSubType_LowPassFilter = FourCharCode('lpas');
	kAudioUnitSubType_HighPassFilter = FourCharCode('hpas');
	kAudioUnitSubType_BandPassFilter = FourCharCode('bpas');
	kAudioUnitSubType_HighShelfFilter = FourCharCode('hshf');
	kAudioUnitSubType_LowShelfFilter = FourCharCode('lshf');
	kAudioUnitSubType_ParametricEQ = FourCharCode('pmeq');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitSubType_Distortion = FourCharCode('dist');
{$ifc not TARGET_OS_IPHONE}
	kAudioUnitSubType_Delay = FourCharCode('dely');
	kAudioUnitSubType_GraphicEQ = FourCharCode('greq');
=======
	kAudioUnitSubType_GraphicEQ = FourCharCode('greq');
	kAudioUnitSubType_PeakLimiter = FourCharCode('lmtr');
	kAudioUnitSubType_DynamicsProcessor = FourCharCode('dcmp');
>>>>>>> graemeg/cpstrnew
=======
	kAudioUnitSubType_GraphicEQ = FourCharCode('greq');
	kAudioUnitSubType_PeakLimiter = FourCharCode('lmtr');
	kAudioUnitSubType_DynamicsProcessor = FourCharCode('dcmp');
>>>>>>> graemeg/cpstrnew
=======
	kAudioUnitSubType_GraphicEQ = FourCharCode('greq');
	kAudioUnitSubType_PeakLimiter = FourCharCode('lmtr');
	kAudioUnitSubType_DynamicsProcessor = FourCharCode('dcmp');
>>>>>>> graemeg/cpstrnew
=======
	kAudioUnitSubType_GraphicEQ = FourCharCode('greq');
	kAudioUnitSubType_PeakLimiter = FourCharCode('lmtr');
	kAudioUnitSubType_DynamicsProcessor = FourCharCode('dcmp');
>>>>>>> origin/cpstrnew
=======
	kAudioUnitSubType_GraphicEQ = FourCharCode('greq');
	kAudioUnitSubType_PeakLimiter = FourCharCode('lmtr');
	kAudioUnitSubType_DynamicsProcessor = FourCharCode('dcmp');
>>>>>>> origin/fixes_2.4
	kAudioUnitSubType_MultiBandCompressor = FourCharCode('mcmp');
	kAudioUnitSubType_MatrixReverb = FourCharCode('mrev');
	kAudioUnitSubType_SampleDelay = FourCharCode('sdly');
	kAudioUnitSubType_Pitch = FourCharCode('tmpt');
	kAudioUnitSubType_AUFilter = FourCharCode('filt');
	kAudioUnitSubType_NetSend = FourCharCode('nsnd');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitSubType_RogerBeep = FourCharCode('rogr');
{$elsec} { not TARGET_OS_IPHONE }
	kAudioUnitSubType_Reverb2 = FourCharCode('rvb2');
	kAudioUnitSubType_AUiPodEQ = FourCharCode('ipeq');
	kAudioUnitSubType_NBandEQ = 'nbeq';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	kAudioUnitSubType_Distortion = FourCharCode('dist');
	kAudioUnitSubType_RogerBeep = FourCharCode('rogr');
//#else
{$elsec} { not TARGET_OS_IPHONE }
	kAudioUnitSubType_AUiPodEQ				= FourCharCode('ipeq');
//#endif
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
{$endif} { not TARGET_OS_IPHONE }

{!
	@enum			Apple mixer audio unit sub types 
	@discussion		These are the subtypes for the various mixer units that apple ships

	@constant		kAudioUnitSubType_MultiChannelMixer
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					Can have any number of inputs, with any number of channels on any input to one 
					output bus with any number of channels on  the desktop. On iPhone the output 
					number of channels is 2.
=======
					Can have any number of inputs, with any number of channels on any input to one output bus with any number of channels on 
					the desktop. On iPhone the output number of channels is 2.
>>>>>>> graemeg/cpstrnew
=======
					Can have any number of inputs, with any number of channels on any input to one output bus with any number of channels on 
					the desktop. On iPhone the output number of channels is 2.
>>>>>>> graemeg/cpstrnew
=======
					Can have any number of inputs, with any number of channels on any input to one output bus with any number of channels on 
					the desktop. On iPhone the output number of channels is 2.
>>>>>>> graemeg/cpstrnew
=======
					Can have any number of inputs, with any number of channels on any input to one output bus with any number of channels on 
					the desktop. On iPhone the output number of channels is 2.
>>>>>>> origin/cpstrnew
=======
					Can have any number of inputs, with any number of channels on any input to one output bus with any number of channels on 
					the desktop. On iPhone the output number of channels is 2.
>>>>>>> origin/fixes_2.4
				
	@constant		kAudioUnitSubType_StereoMixer
						- desktop only
					Inputs can be mono or stereo. Single stereo output
					
	@constant		kAudioUnitSubType_3DMixer
						- desktop only
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					Inputs can be mono, in which case they can be panned around using 3D 
					coordinates and parameters. 
=======
					Inputs can be mono, in which case they can be panned around using 3D coordinates and parameters. 
>>>>>>> graemeg/cpstrnew
=======
					Inputs can be mono, in which case they can be panned around using 3D coordinates and parameters. 
>>>>>>> graemeg/cpstrnew
=======
					Inputs can be mono, in which case they can be panned around using 3D coordinates and parameters. 
>>>>>>> graemeg/cpstrnew
=======
					Inputs can be mono, in which case they can be panned around using 3D coordinates and parameters. 
>>>>>>> origin/cpstrnew
=======
					Inputs can be mono, in which case they can be panned around using 3D coordinates and parameters. 
>>>>>>> origin/fixes_2.4
					Stereo inputs are passed directly through to the output.
					4 channel "ambisonic" inputs will be rendered to the output configuration
					A single output of 2, 4, 5, 6, 7 or 8 channels.
					
	@constant		kAudioUnitSubType_MatrixMixer
						- desktop only
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					Any number of input and output buses with any number of channels on any bus. 
					The mix is presented as a matrix of channels that can be controlled through 
					input volume per channel, "cross-point" volume (a given input channel to a 
					given output channel), output volume per channel and a global volume across 
					the whole matrix

	@constant		kAudioUnitSubType_AU3DMixerEmbedded
						- iPhone only
					A scaled-down version of the AU3DMixer that presents a stereo output, mono or 
					stereo inputs
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
					Any number of input and output buses with any number of channels on any bus. The mix is presented as a matrix
					of channels that can be controlled through input volume per channel, "cross-point" volume (a given input channel to a given output channel),
					output volume per channel and a global volume across the whole matrix

	@constant		kAudioUnitSubType_AU3DMixerEmbedded
						- iPhone only
					A scaled-down version of the AU3DMixer that presents a stereo output, mono or stereo inputs
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
	kAudioUnitSubType_MultiChannelMixer = FourCharCode('mcmx');
//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}
	kAudioUnitSubType_StereoMixer = FourCharCode('smxr');
	kAudioUnitSubType_3DMixer = FourCharCode('3dmx');
	kAudioUnitSubType_MatrixMixer = FourCharCode('mxmx');
//#else
{$elsec} { not TARGET_OS_IPHONE }
	kAudioUnitSubType_AU3DMixerEmbedded		= FourCharCode('3dem');
//#endif;
{$endc} { not TARGET_OS_IPHONE }

{!
	@enum			Apple panner audio unit sub types 
	@discussion		These are the subtypes for the various panner units that apple ships

	@constant		kAudioUnitSubType_SphericalHeadPanner
						- desktop only
					A panner unit that uses the spherical head model to pan to a stereo output
					
	@constant		kAudioUnitSubType_VectorPanner
						- desktop only
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					A panner unit that uses a moving pan between the two closes, adjacent channels 
					in a 3D space to a
=======
					A panner unit that uses a moving pan between the two closes, adjacent channels in a 3D space to a
>>>>>>> graemeg/cpstrnew
=======
					A panner unit that uses a moving pan between the two closes, adjacent channels in a 3D space to a
>>>>>>> graemeg/cpstrnew
=======
					A panner unit that uses a moving pan between the two closes, adjacent channels in a 3D space to a
>>>>>>> graemeg/cpstrnew
=======
					A panner unit that uses a moving pan between the two closes, adjacent channels in a 3D space to a
>>>>>>> origin/cpstrnew
=======
					A panner unit that uses a moving pan between the two closes, adjacent channels in a 3D space to a
>>>>>>> origin/fixes_2.4
					surround output
					
	@constant		kAudioUnitSubType_SoundFieldPanner
						- desktop only
					A panner unit that uses a sound-field notion to pan to a surround output
					
	@constant		kAudioUnitSubType_HRTFPanner
						- desktop only
					A panner unit that uses a generic "HRTF" model to pan to a stereo output
}
//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}
const
	kAudioUnitSubType_SphericalHeadPanner = FourCharCode('sphr');
	kAudioUnitSubType_VectorPanner = FourCharCode('vbas');
	kAudioUnitSubType_SoundFieldPanner = FourCharCode('ambi');
	kAudioUnitSubType_HRTFPanner = FourCharCode('hrtf');
//#endif
{$endc} { not TARGET_OS_IPHONE }

{!
	@enum			Apple generator audio unit sub types 
	@discussion		These are the subtypes for the various generator units that apple ships

	@constant		kAudioUnitSubType_ScheduledSoundPlayer
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					A generator unit that can be used to schedule slices of audio to be played at 
					a specified time. The audio is scheduled using the time stamps for the render 
					operation, and can be scheduled from any thread.
					
	@constant		kAudioUnitSubType_AudioFilePlayer
					A generator unit that is used to play a file. It presents a custom UI so can 
					be used in a UI context as well
					
	@constant		kAudioUnitSubType_NetReceive
						- desktop only
					A generator unit that is paired with _NetSend to receive the audio that unit 
					sends. It presents a custom UI so can be used in a UI context as well
}
const
	kAudioUnitSubType_NetReceive = FourCharCode('nrcv');
{$ifc not TARGET_OS_IPHONE}
	kAudioUnitSubType_ScheduledSoundPlayer = FourCharCode('sspl');
	kAudioUnitSubType_AudioFilePlayer = FourCharCode('afpl');
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
						- desktop only
					A generator unit that can be used to schedule slices of audio to be played at a specified time. The audio
					is scheduled using the time stamps for the render operation, and can be scheduled from any thread.
					
	@constant		kAudioUnitSubType_AudioFilePlayer
						- desktop only
					A generator unit that is used to play a file. It presents a custom UI so can be used in a UI context as well
					
	@constant		kAudioUnitSubType_NetReceive
						- desktop only
					A generator unit that is paired with _NetSend to receive the audio that unit sends. 
					It presents a custom UI so can be used in a UI context as well
}
//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}
const
	kAudioUnitSubType_ScheduledSoundPlayer = FourCharCode('sspl');
	kAudioUnitSubType_AudioFilePlayer = FourCharCode('afpl');
	kAudioUnitSubType_NetReceive = FourCharCode('nrcv');
//#endif
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
{$endc} { not TARGET_OS_IPHONE }

//=====================================================================================================================
//#pragma mark -
//#pragma mark Audio Unit Constants and typedefs
{!
	@enum			AudioUnitRenderActionFlags
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		These flags can be set in a callback from an audio unit during an audio unit 
					render operation from either the RenderNotify Proc or the render input 
					callback.

	@constant		kAudioUnitRenderAction_PreRender
					Called on a render notification Proc - which is called either before or after 
					the render operation of the audio unit. If this flag is set, the proc is being 
					called before the render operation is performed.
					
	@constant		kAudioUnitRenderAction_PostRender
					Called on a render notification Proc - which is called either before or after 
					the render operation of the audio unit. If this flag is set, the proc is being 
					called after the render operation is completed.

	@constant		kAudioUnitRenderAction_OutputIsSilence
					This flag can be set in a render input callback (or in the audio unit's render 
					operation itself) and is used to indicate that the render buffer contains only 
					silence. It can then be used by the caller as a hint to whether the buffer 
					needs to be processed or not.
					
	@constant		kAudioOfflineUnitRenderAction_Preflight
					This is used with offline audio units (of type 'auol'). It is used when an 
					offline unit is being preflighted, which is performed prior to the actual 
					offline rendering actions are performed. It is used for those cases where the 
					offline process needs it (for example, with an offline unit that normalises an 
					audio file, it needs to see all of the audio data first before it can perform 
					its normalisation)
					
	@constant		kAudioOfflineUnitRenderAction_Render
					Once an offline unit has been successfully preflighted, it is then put into 
					its render mode. So this flag is set to indicate to the audio unit that it is 
					now in that state and that it should perform its processing on the input data.
					
	@constant		kAudioOfflineUnitRenderAction_Complete
					This flag is set when an offline unit has completed either its preflight or 
					performed render operations
					
	@constant		kAudioUnitRenderAction_PostRenderError
					If this flag is set on the post-render call an error was returned by the 
					AUs render operation. In this case, the error can be retrieved through the 
					lastRenderError property and the audio data in ioData handed to the post-render 
					notification will be invalid.
	@constant		kAudioUnitRenderAction_DoNotCheckRenderArgs
					If this flag is set, then checks that are done on the arguments provided to render 
					are not performed. This can be useful to use to save computation time in
					situations where you are sure you are providing the correct arguments
					and structures to the various render calls
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		These flags can be set in a callback from an audio unit during an audio unit render operation from 
					either the RenderNotify Proc or the render input callback.

	@constant		kAudioUnitRenderAction_PreRender
					Called on a render notification Proc - which is called either before or after the render operation 
					of the audio unit. If this flag is set, the proc is being called before the render operation is performed.
					
	@constant		kAudioUnitRenderAction_PostRender
					Called on a render notification Proc - which is called either before or after the render operation 
					of the audio unit. If this flag is set, the proc is being called after the render operation is completed.

	@constant		kAudioUnitRenderAction_OutputIsSilence
					This flag can be set in a render input callback (or in the audio unit's render operation itself) and is used to 
					indicate that the render buffer contains only silence. It can then be used by the caller as a hint to whether 
					the buffer needs to be processed or not.
					
	@constant		kAudioOfflineUnitRenderAction_Preflight
					This is used with offline audio units (of type 'auol'). It is used when an offline unit is being preflighted, which
					is performed prior to the actual offline rendering actions are performed. It is used for those cases where the 
					offline process needs it (for example, with an offline unit that normalises an audio file, it needs to see all
					of the audio data first before it can perform its normalisation)
					
	@constant		kAudioOfflineUnitRenderAction_Render
					Once an offline unit has been successfully preflighted, it is then put into its render mode. So this flag is set
					to indicate to the audio unit that it is now in that state and that it should perform its processing on the input data.
					
	@constant		kAudioOfflineUnitRenderAction_Complete	= (1 << 7),
					This flag is set when an offline unit has completed either its preflight or performed render operations
					
	@constant		kAudioUnitRenderAction_PostRenderError
					If this flag is set on the post-render call an error was returned by the AUs render operation. 
					In this case, the error can be retrieved through the lastRenderError property and the audio data in
					ioData handed to the post-render notification will be invalid.
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
	kAudioUnitRenderAction_PreRender = 1 shl 2;
	kAudioUnitRenderAction_PostRender = 1 shl 3;
	kAudioUnitRenderAction_OutputIsSilence = 1 shl 4;
	kAudioOfflineUnitRenderAction_Preflight = 1 shl 5;
	kAudioOfflineUnitRenderAction_Render = 1 shl 6;
	kAudioOfflineUnitRenderAction_Complete = 1 shl 7;
	kAudioUnitRenderAction_PostRenderError = 1 shl 8;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitRenderAction_DoNotCheckRenderArgs = 1 shl 9;
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
type
	AudioUnitRenderActionFlags = UInt32;

{!
	@enum			Audio unit errors
	@discussion		These are the various errors that can be returned by AudioUnit... API calls

	@constant		kAudioUnitErr_InvalidProperty
					The property is not supported
	@constant		kAudioUnitErr_InvalidParameter
					The parameter is not supported
	@constant		kAudioUnitErr_InvalidElement
					The specified element is not valid
	@constant		kAudioUnitErr_NoConnection
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					There is no connection (generally an audio unit is asked to render but it has 
					not input from which to gather data)
	@constant		kAudioUnitErr_FailedInitialization
					The audio unit is unable to be initialised
	@constant		kAudioUnitErr_TooManyFramesToProcess
					When an audio unit is initialised it has a value which specifies the max 
					number of frames it will be asked to render at any given time. If an audio 
					unit is asked to render more than this, this error is returned.
	@constant		kAudioUnitErr_InvalidFile
					If an audio unit uses external files as a data source, this error is returned 
					if a file is invalid (Apple's DLS synth returns this error)
	@constant		kAudioUnitErr_FormatNotSupported
					Returned if an input or output format is not supported
	@constant		kAudioUnitErr_Uninitialized
					Returned if an operation requires an audio unit to be initialised and it is 
					not.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
					There is no connection (generally an audio unit is asked to render but it has not input from which to gather data)
	@constant		kAudioUnitErr_FailedInitialization
					The audio unit is unable to be initialised
	@constant		kAudioUnitErr_TooManyFramesToProcess
					When an audio unit is initialised it has a value which specifies the max number of frames it will be asked to render at any 
					given time. If an audio unit is asked to render more than this, this error is returned.
	@constant		kAudioUnitErr_InvalidFile
					If an audio unit uses external files as a data source, this error is returned if a file is invalid (Apple's DLS synth returns this error)
=======
					There is no connection (generally an audio unit is asked to render but it has not input from which to gather data)
	@constant		kAudioUnitErr_FailedInitialization
					The audio unit is unable to be initialised
	@constant		kAudioUnitErr_TooManyFramesToProcess
					When an audio unit is initialised it has a value which specifies the max number of frames it will be asked to render at any 
					given time. If an audio unit is asked to render more than this, this error is returned.
	@constant		kAudioUnitErr_InvalidFile
					If an audio unit uses external files as a data source, this error is returned if a file is invalid (Apple's DLS synth returns this error)
>>>>>>> origin/fixes_2.4
	@constant		kAudioUnitErr_FormatNotSupported
					Returned if an input or output format is not supported
	@constant		kAudioUnitErr_Uninitialized
					Returned if an operation requires an audio unit to be initialised and it is not.
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
	@constant		kAudioUnitErr_InvalidScope
					The specified scope is invalid
	@constant		kAudioUnitErr_PropertyNotWritable
					The property cannot be written
	@constant		kAudioUnitErr_CannotDoInCurrentContext
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					Returned when an audio unit is in a state where it can't perform the requested 
					action now - but it could later. Its usually used to guard a render operation 
					when a reconfiguration of its internal state is being performed.
	@constant		kAudioUnitErr_InvalidPropertyValue
					The property is valid, but the value of the property being provided is not
	@constant		kAudioUnitErr_PropertyNotInUse
					Returned when a property is valid, but it hasn't been set to a valid value at 
					this time.	
	@constant		kAudioUnitErr_Initialized
					Indicates the operation cannot be performed because the audio unit is 
					initialized.
	@constant		kAudioUnitErr_InvalidOfflineRender
					Used to indicate that the offline render operation is invalid. For instance,
					when the audio unit needs to be pre-flighted, 
					but it hasn't been.
	@constant		kAudioUnitErr_Unauthorized
					Returned by either Open or Initialise, this error is used to indicate that the 
					audio unit is not authorised, that it cannot be used. A host can then present 
					a UI to notify the user the audio unit is not able to be used in its current 
					state.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
					Returned when an audio unit is in a state where it can't perform the requested action now - but it could later. Its usually used
					to guard a render operation when a reconfiguration of its internal state is being performed.
	@constant		kAudioUnitErr_InvalidPropertyValue
					The property is valid, but the value of the property being provided is not
	@constant		kAudioUnitErr_PropertyNotInUse
					Returned when a property is valid, but it hasn't been set to a valid value at this time.	
	@constant		kAudioUnitErr_Initialized
					Indicates the operation cannot be performed because the audio unit is initialized.
	@constant		kAudioUnitErr_InvalidOfflineRender
					Used to indicate that the offline render operation is invalid. For instance, when the audio unit needs to be pre-flighted, 
					but it hasn't been.
	@constant		kAudioUnitErr_Unauthorized
					Returned by either Open or Initialise, this error is used to indicate that the audio unit is not authorised, that it cannot
					be used. A host can then present a UI to notify the user the audio unit is not able to be used in its current state.
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
	kAudioUnitErr_InvalidProperty = -10879;
	kAudioUnitErr_InvalidParameter = -10878;
	kAudioUnitErr_InvalidElement = -10877;
	kAudioUnitErr_NoConnection = -10876;
	kAudioUnitErr_FailedInitialization = -10875;
	kAudioUnitErr_TooManyFramesToProcess = -10874;
	kAudioUnitErr_InvalidFile = -10871;
	kAudioUnitErr_FormatNotSupported = -10868;
	kAudioUnitErr_Uninitialized = -10867;
	kAudioUnitErr_InvalidScope = -10866;
	kAudioUnitErr_PropertyNotWritable = -10865;
	kAudioUnitErr_CannotDoInCurrentContext = -10863;
	kAudioUnitErr_InvalidPropertyValue = -10851;
	kAudioUnitErr_PropertyNotInUse = -10850;
	kAudioUnitErr_Initialized = -10849;
	kAudioUnitErr_InvalidOfflineRender = -10848;
	kAudioUnitErr_Unauthorized = -10847;

{!
	@typedef			AudioUnitPropertyID
	@discussion			Type used for audio unit properties. 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Properties are used to describe the state of an audio unit (for instance, 
						the input or output audio format)
=======
						Properties are used to describe the state of an audio unit (for instance, the input or output audio format)
>>>>>>> graemeg/cpstrnew
=======
						Properties are used to describe the state of an audio unit (for instance, the input or output audio format)
>>>>>>> graemeg/cpstrnew
=======
						Properties are used to describe the state of an audio unit (for instance, the input or output audio format)
>>>>>>> graemeg/cpstrnew
=======
						Properties are used to describe the state of an audio unit (for instance, the input or output audio format)
>>>>>>> origin/cpstrnew
=======
						Properties are used to describe the state of an audio unit (for instance, the input or output audio format)
>>>>>>> origin/fixes_2.4
}
type
	AudioUnitPropertyID = UInt32;
{!
	@typedef			AudioUnitScope
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion			Type used for audio unit scopes. Apple reserves the 0 < 1024 range for 
						audio unit scope identifiers.  
						Scopes are used to delineate a major attribute of an audio unit 
						(for instance, global, input, output)
=======
	@discussion			Type used for audio unit scopes. Apple reserves the 0 < 1024 range for audio unit scope identifiers.  
						Scopes are used to delineate a major attribute of an audio unit (for instance, global, input, output)
>>>>>>> graemeg/cpstrnew
=======
	@discussion			Type used for audio unit scopes. Apple reserves the 0 < 1024 range for audio unit scope identifiers.  
						Scopes are used to delineate a major attribute of an audio unit (for instance, global, input, output)
>>>>>>> graemeg/cpstrnew
=======
	@discussion			Type used for audio unit scopes. Apple reserves the 0 < 1024 range for audio unit scope identifiers.  
						Scopes are used to delineate a major attribute of an audio unit (for instance, global, input, output)
>>>>>>> graemeg/cpstrnew
=======
	@discussion			Type used for audio unit scopes. Apple reserves the 0 < 1024 range for audio unit scope identifiers.  
						Scopes are used to delineate a major attribute of an audio unit (for instance, global, input, output)
>>>>>>> origin/cpstrnew
=======
	@discussion			Type used for audio unit scopes. Apple reserves the 0 < 1024 range for audio unit scope identifiers.  
						Scopes are used to delineate a major attribute of an audio unit (for instance, global, input, output)
>>>>>>> origin/fixes_2.4
}
type
	AudioUnitScope = UInt32;
{!
	@typedef			AudioUnitElement
	@discussion			Type used for audio unit elements.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Scopes can have one or more member, and a member of a scope is 
						addressed / described by its element
=======
						Scopes can have one or more member, and a member of a scope is addressed / described by its element
>>>>>>> graemeg/cpstrnew
=======
						Scopes can have one or more member, and a member of a scope is addressed / described by its element
>>>>>>> graemeg/cpstrnew
=======
						Scopes can have one or more member, and a member of a scope is addressed / described by its element
>>>>>>> graemeg/cpstrnew
=======
						Scopes can have one or more member, and a member of a scope is addressed / described by its element
>>>>>>> origin/cpstrnew
=======
						Scopes can have one or more member, and a member of a scope is addressed / described by its element
>>>>>>> origin/fixes_2.4
						For instance, input bus 1 is input scope, element 1
}
type
	AudioUnitElement = UInt32;
{!
	@typedef			AudioUnitParameterID
	@discussion			Type used for audio unit parameters. 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Parameters are typically used to control and set render state 
						(for instance, filter cut-off frequency)
=======
						Parameters are typically used to control and set render state (for instance, filter cut-off frequency)
>>>>>>> graemeg/cpstrnew
=======
						Parameters are typically used to control and set render state (for instance, filter cut-off frequency)
>>>>>>> graemeg/cpstrnew
=======
						Parameters are typically used to control and set render state (for instance, filter cut-off frequency)
>>>>>>> graemeg/cpstrnew
=======
						Parameters are typically used to control and set render state (for instance, filter cut-off frequency)
>>>>>>> origin/cpstrnew
=======
						Parameters are typically used to control and set render state (for instance, filter cut-off frequency)
>>>>>>> origin/fixes_2.4
}
type
	AudioUnitParameterID = UInt32;
{!
	@typedef			AudioUnitParameterValue
	@discussion			Type used for audio unit parameter values. 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						The value of a given parameter is specified using this type 
						(typically a Float32)
=======
						The value of a given parameter is specified using this type (typically a Float32)
>>>>>>> graemeg/cpstrnew
=======
						The value of a given parameter is specified using this type (typically a Float32)
>>>>>>> graemeg/cpstrnew
=======
						The value of a given parameter is specified using this type (typically a Float32)
>>>>>>> graemeg/cpstrnew
=======
						The value of a given parameter is specified using this type (typically a Float32)
>>>>>>> origin/cpstrnew
=======
						The value of a given parameter is specified using this type (typically a Float32)
>>>>>>> origin/fixes_2.4
}
type
	AudioUnitParameterValue = Float32;
	AudioUnitParameterValuePtr = ^AudioUnitParameterValue;

{!
	@enum			AUParameterEventType
	@discussion		The type of a parameter event (see AudioUnitScheduleParameter)

	@constant		kParameterEvent_Immediate
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					The parameter event describes an immediate change to the parameter value to 
					the new value
	@constant		kParameterEvent_Ramped
					The parameter event describes a change to the parameter value that should
					be applied over the specified period of time
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
					The parameter event describes an immediate change to the parameter value to the new value
	@constant		kParameterEvent_Ramped
					The parameter event describes a change to the parameter value that should be applied over
					the specified period of time
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
	kParameterEvent_Immediate = 1;
	kParameterEvent_Ramped = 2;
type
	AUParameterEventType = UInt32;

{!
	@struct			AudioUnitParameterEvent
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		A parameter event describes a change to a parameter's value, where the type of 
					the event describes how that change is to be applied (see AUParameterEventType). 
					A parameter is uniquely defined through the triplet of scope, element and 
					parameterID.
=======
	@discussion		A parameter event describes a change to a parameter's value, where the type of the event describes
					how that change is to be applied (see AUParameterEventType). A parameter is uniquely defined through
					the triplet of scope, element and parameterID.
>>>>>>> graemeg/cpstrnew
=======
	@discussion		A parameter event describes a change to a parameter's value, where the type of the event describes
					how that change is to be applied (see AUParameterEventType). A parameter is uniquely defined through
					the triplet of scope, element and parameterID.
>>>>>>> graemeg/cpstrnew
=======
	@discussion		A parameter event describes a change to a parameter's value, where the type of the event describes
					how that change is to be applied (see AUParameterEventType). A parameter is uniquely defined through
					the triplet of scope, element and parameterID.
>>>>>>> graemeg/cpstrnew
=======
	@discussion		A parameter event describes a change to a parameter's value, where the type of the event describes
					how that change is to be applied (see AUParameterEventType). A parameter is uniquely defined through
					the triplet of scope, element and parameterID.
>>>>>>> origin/cpstrnew
=======
	@discussion		A parameter event describes a change to a parameter's value, where the type of the event describes
					how that change is to be applied (see AUParameterEventType). A parameter is uniquely defined through
					the triplet of scope, element and parameterID.
>>>>>>> origin/fixes_2.4
					
					See AudioUnitScheduleParameters

	@field			scope	
					The scope for the parameter
	@field			element
					The element for the parameter
	@field			parameter
					The parameterID for the parameter
	
	@field			eventType
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					The event type. This field further defines how the union described by 
					eventValues is to be interpreted.
	
	@field			eventValues
					If the parameter event type is _Immediate, then the immediate struct of this 
					union should be used.
					If the parameter event type is _Ramped, then the ramp struct of this union 
					should be used.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
					The event type. This field further defines how the union described by eventValues is to be interpreted.
	
	@field			eventValues
					If the parameter event type is _Immediate, then the immediate struct of this union should be used.
					If the parameter event type is _Ramped, then the ramp struct of this union should be used.
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
type
	AudioUnitParameterEventRampRec = record
		startBufferOffset: SInt32;
		durationInFrames: UInt32;
		startValue: AudioUnitParameterValue;
		endValue: AudioUnitParameterValue;
	end;
	AudioUnitParameterEventImmediateRec = record
		bufferOffset: UInt32;
		value: AudioUnitParameterValue;
	end;
	AudioUnitParameterEventEventValues = record
		case UInt8 of
			0:	(ramp : AudioUnitParameterEventRampRec);
			1:	(immediate : AudioUnitParameterEventImmediateRec);
	end;
	
	AudioUnitParameterEvent = record
		scope: AudioUnitScope;
		element: AudioUnitElement;
		parameter: AudioUnitParameterID;
	
		eventType: AUParameterEventType;
		eventValues: AudioUnitParameterEventEventValues;
	end;
	AudioUnitParameterEventPtr = ^AudioUnitParameterEvent;
	
{!
	@struct			AudioUnitParameter
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		An audio unit parameter is defined by the triplet of audio unit scope, element 
					and parameterID. This struct is used with the functions in AudioUnitUtilities.h 
					to deal with audio unit parameters, but is included in this header file for 
					completeness
=======
	@discussion		An audio unit parameter is defined by the triplet of audio unit scope, element and parameterID. This struct
					is used with the functions in AudioUnitUtilities.h to deal with audio unit parameters, but is included
					in this header file for completeness
>>>>>>> graemeg/cpstrnew
=======
	@discussion		An audio unit parameter is defined by the triplet of audio unit scope, element and parameterID. This struct
					is used with the functions in AudioUnitUtilities.h to deal with audio unit parameters, but is included
					in this header file for completeness
>>>>>>> graemeg/cpstrnew
=======
	@discussion		An audio unit parameter is defined by the triplet of audio unit scope, element and parameterID. This struct
					is used with the functions in AudioUnitUtilities.h to deal with audio unit parameters, but is included
					in this header file for completeness
>>>>>>> graemeg/cpstrnew
=======
	@discussion		An audio unit parameter is defined by the triplet of audio unit scope, element and parameterID. This struct
					is used with the functions in AudioUnitUtilities.h to deal with audio unit parameters, but is included
					in this header file for completeness
>>>>>>> origin/cpstrnew
=======
	@discussion		An audio unit parameter is defined by the triplet of audio unit scope, element and parameterID. This struct
					is used with the functions in AudioUnitUtilities.h to deal with audio unit parameters, but is included
					in this header file for completeness
>>>>>>> origin/fixes_2.4

	@field			mAudioUnit
					The audio unit instance which the specified parameter applies too
	@field			mParameterID
					The parameterID for the parameter
	@field			mScope	
					The scope for the parameter
	@field			mElement
					The element for the parameter
}
type
	AudioUnitParameter = record
		mAudioUnit: AudioUnit;
		mParameterID: AudioUnitParameterID;
		mScope: AudioUnitScope;
		mElement: AudioUnitElement;
	end;
	AudioUnitParameterPtr = ^AudioUnitParameter;
	
{!
	@struct			AudioUnitProperty
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		An audio unit property is defined by the triplet of audio unit scope, element
					and propertyID. This struct is used with the functions in AudioUnitUtilities.h 
					to deal with audio unit properties, but is included in this header file for 
					completeness
=======
	@discussion		An audio unit property is defined by the triplet of audio unit scope, element and propertyID. This struct
					is used with the functions in AudioUnitUtilities.h to deal with audio unit properties, but is included
					in this header file for completeness
>>>>>>> graemeg/cpstrnew
=======
	@discussion		An audio unit property is defined by the triplet of audio unit scope, element and propertyID. This struct
					is used with the functions in AudioUnitUtilities.h to deal with audio unit properties, but is included
					in this header file for completeness
>>>>>>> graemeg/cpstrnew
=======
	@discussion		An audio unit property is defined by the triplet of audio unit scope, element and propertyID. This struct
					is used with the functions in AudioUnitUtilities.h to deal with audio unit properties, but is included
					in this header file for completeness
>>>>>>> graemeg/cpstrnew
=======
	@discussion		An audio unit property is defined by the triplet of audio unit scope, element and propertyID. This struct
					is used with the functions in AudioUnitUtilities.h to deal with audio unit properties, but is included
					in this header file for completeness
>>>>>>> origin/cpstrnew
=======
	@discussion		An audio unit property is defined by the triplet of audio unit scope, element and propertyID. This struct
					is used with the functions in AudioUnitUtilities.h to deal with audio unit properties, but is included
					in this header file for completeness
>>>>>>> origin/fixes_2.4

	@field			mAudioUnit
					The audio unit instance which the specified property applies too
	@field			mPropertyID
					The propertyID for the property
	@field			mScope	
					The scope for the property
	@field			mElement
					The element for the property
}
type
	AudioUnitProperty = record
		mAudioUnit: AudioUnit;
		mPropertyID: AudioUnitPropertyID;
		mScope: AudioUnitScope;
		mElement: AudioUnitElement;
	end;
	AudioUnitPropertyPtr = ^AudioUnitProperty;
	

{!
	@typedef		AURenderCallback
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		This is the prototype for a function callback Proc that is used both with the 
					AudioUnit render notification API and the render input callback. See 
					kAudioUnitProperty_SetRenderCallback property or AudioUnitAddRenderNotify.
					This callback is part of the process of a call to AudioUnitRender. As a 
					notification it is called either before or after the audio unit's render 
					operations. As a render input callback, it is called to provide input data for
					the particular input bus the callback is attached too.
	
	@param			inRefCon
					The client data that is provided either with the AURenderCallbackStruct or as 
					specified with the Add API call
	@param			ioActionFlags
					Flags used to describe more about the context of this call (pre or post in the 
					notify case for instance) 
	@param			inTimeStamp
					The times stamp associated with this call of audio unit render
	@param			inBusNumber
					The bus number associated with this call of audio unit render
	@param			inNumberFrames
					The number of sample frames that will be represented in the audio data in the 
					provided ioData parameter
	@param			ioData
					The AudioBufferList that will be used to contain the rendered or provided 
					audio data. These buffers will be aligned to 16 byte boundaries (which is 
					normally what malloc will return).
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		This is the prototype for a function callback Proc that is used both with the AudioUnit render notification
					API and the render input callback. See kAudioUnitProperty_SetRenderCallback property or AudioUnitAddRenderNotify.
					This callback is part of the process of a call to AudioUnitRender. As a notification it is called either before
					or after the audio unit's render operations. As a render input callback, it is called to provide input data for
					the particular input bus the callback is attached too.
	
	@param			inRefCon
					The client data that is provided either with the AURenderCallbackStruct or as specified with the Add API call
	@param			ioActionFlags
					Flags used to describe more about the context of this call (pre or post in the notify case for instance) 
	@param			inTimeStamp
					The times stamp associated with this call of audio unit render
	@param			inBusNumber
					The bus numbeer associated with this call of audio unit render
	@param			inNumberFrames
					The number of sample frames that will be represented in the audio data in the provided ioData parameter
	@param			ioData
					The AudioBufferList that will be used to contain the rendered or provided audio data
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
type
	AURenderCallback = function( inRefCon: UnivPtr; var ioActionFlags: AudioUnitRenderActionFlags; const (*var*) inTimeStamp: AudioTimeStamp; inBusNumber: UInt32; inNumberFrames: UInt32; var ioData: AudioBufferList ): OSStatus;

{!
	@typedef		AudioUnitPropertyListenerProc
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		This is the prototype for a function callback Proc that is registered with an 
					audio unit to notify the caller of any changes to a value of an audio unit 
					property. See AudioUnitAddPropertyListener
=======
	@discussion		This is the prototype for a function callback Proc that is registered with an audio unit to notify the caller
					of any changes to a value of an audio unit property. See AudioUnitAddPropertyListener
>>>>>>> graemeg/cpstrnew
=======
	@discussion		This is the prototype for a function callback Proc that is registered with an audio unit to notify the caller
					of any changes to a value of an audio unit property. See AudioUnitAddPropertyListener
>>>>>>> graemeg/cpstrnew
=======
	@discussion		This is the prototype for a function callback Proc that is registered with an audio unit to notify the caller
					of any changes to a value of an audio unit property. See AudioUnitAddPropertyListener
>>>>>>> graemeg/cpstrnew
=======
	@discussion		This is the prototype for a function callback Proc that is registered with an audio unit to notify the caller
					of any changes to a value of an audio unit property. See AudioUnitAddPropertyListener
>>>>>>> origin/cpstrnew
=======
	@discussion		This is the prototype for a function callback Proc that is registered with an audio unit to notify the caller
					of any changes to a value of an audio unit property. See AudioUnitAddPropertyListener
>>>>>>> origin/fixes_2.4
	
	@param			inRefCon
					The client data that is provided with the add property listener registration
	@param			inUnit
					The audio unit upon which the specified property value has changed
	@param			inID
					The property whose value has changed
	@param			inScope
					The scope of the property whose value has changed
	@param			inElement
					The element ID on the scope of the property whose value has changed
}
type
	AudioUnitPropertyListenerProc = procedure( inRefCon: UnivPtr; inUnit: AudioUnit; inID: AudioUnitPropertyID; inScope: AudioUnitScope; inElement: AudioUnitElement );

{!
	@typedef		AUInputSamplesInOutputCallback
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		This is the prototype for a function callback Proc that is registered with an 
					audio unit to notify the caller of for the user of a varispeed or AUTimePitch 
					audio unit where it is not clear what input sample is represented in the 
					rendered output samples.
=======
	@discussion		This is the prototype for a function callback Proc that is registered with an audio unit to notify the caller
					of for the user of a varispeed or AUTimePitch audio unit where it is not clear what input sample is represented
					in the rendered output samples.
>>>>>>> graemeg/cpstrnew
=======
	@discussion		This is the prototype for a function callback Proc that is registered with an audio unit to notify the caller
					of for the user of a varispeed or AUTimePitch audio unit where it is not clear what input sample is represented
					in the rendered output samples.
>>>>>>> graemeg/cpstrnew
=======
	@discussion		This is the prototype for a function callback Proc that is registered with an audio unit to notify the caller
					of for the user of a varispeed or AUTimePitch audio unit where it is not clear what input sample is represented
					in the rendered output samples.
>>>>>>> graemeg/cpstrnew
=======
	@discussion		This is the prototype for a function callback Proc that is registered with an audio unit to notify the caller
					of for the user of a varispeed or AUTimePitch audio unit where it is not clear what input sample is represented
					in the rendered output samples.
>>>>>>> origin/cpstrnew
=======
	@discussion		This is the prototype for a function callback Proc that is registered with an audio unit to notify the caller
					of for the user of a varispeed or AUTimePitch audio unit where it is not clear what input sample is represented
					in the rendered output samples.
>>>>>>> origin/fixes_2.4
	
	@param			inRefCon
					The client data that is provided with the add property listener registration
	@param			inOutputTimeStamp
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					The time stamp that corresponds to the first sample of audio data produced in 
					AudioUnitRender (its output data)
	@param			inInputSample
					The sample number of the input that is represented in the first sample of that 
					output time stamp
=======
					The time stamp that corresponds to the first sample of audio data produced in AudioUnitRender (its output data)
	@param			inInputSample
					The sample number of the input that is represented in the first sample of that output time stamp
>>>>>>> graemeg/cpstrnew
=======
					The time stamp that corresponds to the first sample of audio data produced in AudioUnitRender (its output data)
	@param			inInputSample
					The sample number of the input that is represented in the first sample of that output time stamp
>>>>>>> graemeg/cpstrnew
=======
					The time stamp that corresponds to the first sample of audio data produced in AudioUnitRender (its output data)
	@param			inInputSample
					The sample number of the input that is represented in the first sample of that output time stamp
>>>>>>> graemeg/cpstrnew
=======
					The time stamp that corresponds to the first sample of audio data produced in AudioUnitRender (its output data)
	@param			inInputSample
					The sample number of the input that is represented in the first sample of that output time stamp
>>>>>>> origin/cpstrnew
=======
					The time stamp that corresponds to the first sample of audio data produced in AudioUnitRender (its output data)
	@param			inInputSample
					The sample number of the input that is represented in the first sample of that output time stamp
>>>>>>> origin/fixes_2.4
	@param			inNumberInputSamples
					The number of input samples that are represented in an output buffer
}
type
	AUInputSamplesInOutputCallback = procedure( inRefCon: UnivPtr; const (*var*) inOutputTimeStamp: AudioTimeStamp; inInputSample: Float64; inNumberInputSamples: Float64 );

//=====================================================================================================================
//#pragma mark -
//#pragma mark Functions

{!
	@function		AudioUnitInitialize
	@abstract		initialize an audio unit
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		Upon success, the audio unit has been successfully initialised. This means 
					that the formats for input and output are valid and can be supported and it 
					has based its allocations on the max number of frames that it is able to 
					render at any given time. Once initialised, it is in a state where it can be 
					asked to render.
					
					In common practice major state of an audio unit (such as its I/O formats, 
					memory allocations) cannot be changed while an audio unit is inialized.
	
	@param			inUnit
					The audio unit to initialise
	@result			noErr, or an error representing the reasons why the audio unit was not able 
					to be initialised successfully
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		Upon success, the audio unit has been successfully initialised. This means that the formats for input
					and output are valid and can be supported and it has based its allocations on the max number of frames
					that it is able to render at any given time. Once initialised, it is in a state where it can be asked 
					to render.
					
					In common practise major state of an audio unit (such as its I/O formats, memory allocations) cannot be changed
					while an audio unit is inialized.
	
	@param			inUnit
					The audio unit to initialise
	@result			noErr, or an error representing the reasons why the audio unit was not able to be initialised successfully
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
function AudioUnitInitialize( inUnit: AudioUnit ): OSStatus; external name '_AudioUnitInitialize';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0,__IPHONE_2_0) *)

{!
	@function		AudioUnitUninitialize
	@abstract		uninitialize and audio unit
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		Once an audio unit has been initialised, to change its state in response to 
					some kind of environmental change, the audio unit should be uninitialised. 
					This will have the effect of the audio unit de-allocating its resources.
					The caller can then reconfigure the audio unit to match the new environment 
					(for instance, the sample rate to process audio is different than it was) and 
					then re-initialise the audio unit when those changes have been applied.
	
	@param			inUnit
					The audio unit to uninitialise
	@result			noErr, or an error representing the reasons why the audio unit was not able 
					to be initialised successfully. Typically this call won't return an error 
					unless the audio unit in question is no longer valid.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		Once an audio unit has been initialised, to change its state in response to some kind of environmental change,
					the audio unit should be uninitialised. This will have the effect of the audio unit de-allocating its resources.
					The caller can then reconfigure the audio unit to match the new environment (for instance, the sample rate to proces
					audio is different than it was) and then re-initialise the audio unit when those changes have been applied.
	
	@param			inUnit
					The audio unit to uninitialise
	@result			noErr, or an error representing the reasons why the audio unit was not able to be initialised successfully. Typically
					this call won't return an error unless the audio unit in question is no longer valid.
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
function AudioUnitUninitialize( inUnit: AudioUnit ): OSStatus; external name '_AudioUnitUninitialize';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0,__IPHONE_2_0) *)


{!
	@function		AudioUnitGetPropertyInfo
	@abstract		retrieves information about a specified property
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		The API can be used to retrieve both the size of the property, and whether it 
					is writable or not. In order to get a general answer on the capability of an 
					audio unit, this function should be called before the audio unit
					is initialized (as some properties are writable when the audio unit is 
					initialized, and others not)
=======
	@discussion		The API can be used to retrieve both the size of the property, and whether it is writable or not. In order to 
					get a general answer on the capability of an audio unit, this function should be called before the audio unit
					is initialized (as some properties are writable when the audio unit is initialized, and others not)
>>>>>>> graemeg/cpstrnew
=======
	@discussion		The API can be used to retrieve both the size of the property, and whether it is writable or not. In order to 
					get a general answer on the capability of an audio unit, this function should be called before the audio unit
					is initialized (as some properties are writable when the audio unit is initialized, and others not)
>>>>>>> graemeg/cpstrnew
=======
	@discussion		The API can be used to retrieve both the size of the property, and whether it is writable or not. In order to 
					get a general answer on the capability of an audio unit, this function should be called before the audio unit
					is initialized (as some properties are writable when the audio unit is initialized, and others not)
>>>>>>> graemeg/cpstrnew
=======
	@discussion		The API can be used to retrieve both the size of the property, and whether it is writable or not. In order to 
					get a general answer on the capability of an audio unit, this function should be called before the audio unit
					is initialized (as some properties are writable when the audio unit is initialized, and others not)
>>>>>>> origin/cpstrnew
=======
	@discussion		The API can be used to retrieve both the size of the property, and whether it is writable or not. In order to 
					get a general answer on the capability of an audio unit, this function should be called before the audio unit
					is initialized (as some properties are writable when the audio unit is initialized, and others not)
>>>>>>> origin/fixes_2.4

	@param			inUnit
					the audio unit
	@param			inID
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					the property identifier
=======
					the proeprty identifier
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
>>>>>>> origin/cpstrnew
=======
					the proeprty identifier
>>>>>>> origin/fixes_2.4
	@param			inScope
					the scope of the property
	@param			inElement
					the element of the scope
	@param			outDataSize
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					if not null, then will retrieve the maximum size for the property. if null, 
					then it is ignored
	@param			outWritable	
					if not null, then will retrieve whether the property can be written or not. 
					if null, then it is ignored
=======
					if not null, then will retrieve the maximum size for the property. if null, then it is ignored
	@param			outWritable	
					if not null, then will retrieve whether the property can be written or not. if null, then it is ignored
>>>>>>> graemeg/cpstrnew
=======
					if not null, then will retrieve the maximum size for the property. if null, then it is ignored
	@param			outWritable	
					if not null, then will retrieve whether the property can be written or not. if null, then it is ignored
>>>>>>> graemeg/cpstrnew
=======
					if not null, then will retrieve the maximum size for the property. if null, then it is ignored
	@param			outWritable	
					if not null, then will retrieve whether the property can be written or not. if null, then it is ignored
>>>>>>> graemeg/cpstrnew
=======
					if not null, then will retrieve the maximum size for the property. if null, then it is ignored
	@param			outWritable	
					if not null, then will retrieve whether the property can be written or not. if null, then it is ignored
>>>>>>> origin/cpstrnew
=======
					if not null, then will retrieve the maximum size for the property. if null, then it is ignored
	@param			outWritable	
					if not null, then will retrieve whether the property can be written or not. if null, then it is ignored
>>>>>>> origin/fixes_2.4
	
	@result			noErr, or various audio unit errors related to properties
}
function AudioUnitGetPropertyInfo( inUnit: AudioUnit; inID: AudioUnitPropertyID; inScope: AudioUnitScope; inElement: AudioUnitElement; outDataSize: UInt32Ptr; outWritable: BooleanPtr ): OSStatus; external name '_AudioUnitGetPropertyInfo';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0,__IPHONE_2_0) *)

{!
	@function		AudioUnitGetProperty
	@abstract		retrieves the value of a specified property
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		The API can is used to retrieve the value of the property. Property values for 
					audio units are always passed by reference
=======
	@discussion		The API can is used to retrieve the value of the property. Property values for audio units are always passed
					by reference
>>>>>>> graemeg/cpstrnew
=======
	@discussion		The API can is used to retrieve the value of the property. Property values for audio units are always passed
					by reference
>>>>>>> graemeg/cpstrnew
=======
	@discussion		The API can is used to retrieve the value of the property. Property values for audio units are always passed
					by reference
>>>>>>> graemeg/cpstrnew
=======
	@discussion		The API can is used to retrieve the value of the property. Property values for audio units are always passed
					by reference
>>>>>>> origin/cpstrnew
=======
	@discussion		The API can is used to retrieve the value of the property. Property values for audio units are always passed
					by reference
>>>>>>> origin/fixes_2.4
					
	@param			inUnit
					the audio unit
	@param			inID
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					the property identifier
=======
					the proeprty identifier
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
>>>>>>> origin/cpstrnew
=======
					the proeprty identifier
>>>>>>> origin/fixes_2.4
	@param			inScope
					the scope of the property
	@param			inElement
					the element of the scope
	@param			outData
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					used to retrieve the value of the property. It should point to memory at least 
					as large as the value
					described by ioDataSize 
	@param			ioDataSize	
					on input contains the size of the data pointed to by outData, on output, the 
					size of the data that was returned.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
					used to retrieve the value of the property. It should point to memory at least as large as the value
					desribed by ioDataSize 
	@param			ioDataSize	
					on input contains the size of the data pointed to by outData, on output, the size of the data that was returned.
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

	@result			noErr, or various audio unit errors related to properties
}
function AudioUnitGetProperty( inUnit: AudioUnit; inID: AudioUnitPropertyID; inScope: AudioUnitScope; inElement: AudioUnitElement; outData: UnivPtr; var ioDataSize: UInt32 ): OSStatus; external name '_AudioUnitGetProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0,__IPHONE_2_0) *)

{!
	@function		AudioUnitSetProperty
	@abstract		sets the value of a specified property
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		The API can is used to set the value of the property. Property values for 
					audio units are always passed by reference
=======
	@discussion		The API can is used to set the value of the property. Property values for audio units are always passed
					by reference
>>>>>>> graemeg/cpstrnew
=======
	@discussion		The API can is used to set the value of the property. Property values for audio units are always passed
					by reference
>>>>>>> graemeg/cpstrnew
=======
	@discussion		The API can is used to set the value of the property. Property values for audio units are always passed
					by reference
>>>>>>> graemeg/cpstrnew
=======
	@discussion		The API can is used to set the value of the property. Property values for audio units are always passed
					by reference
>>>>>>> origin/cpstrnew
=======
	@discussion		The API can is used to set the value of the property. Property values for audio units are always passed
					by reference
>>>>>>> origin/fixes_2.4
					
	@param			inUnit
					the audio unit
	@param			inID
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					the property identifier
=======
					the proeprty identifier
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
>>>>>>> origin/cpstrnew
=======
					the proeprty identifier
>>>>>>> origin/fixes_2.4
	@param			inScope
					the scope of the property
	@param			inElement
					the element of the scope
	@param			inData
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					if not null, then is the new value for the property that will be set. If null, 
					then inDataSize should be zero, and the call is then used to remove a 
					previously set value for a property. This removal is only valid for
					some properties, as most properties will always have a default value if not 
					set.
=======
					if not null, then is the new value for the property that will be set. If null, then inDataSize should be zero,
					and the call is then used to remove a previously set value for a property. This removal is only valid for
					some properties, as most properties will always have a default value if not set.
>>>>>>> graemeg/cpstrnew
=======
					if not null, then is the new value for the property that will be set. If null, then inDataSize should be zero,
					and the call is then used to remove a previously set value for a property. This removal is only valid for
					some properties, as most properties will always have a default value if not set.
>>>>>>> graemeg/cpstrnew
=======
					if not null, then is the new value for the property that will be set. If null, then inDataSize should be zero,
					and the call is then used to remove a previously set value for a property. This removal is only valid for
					some properties, as most properties will always have a default value if not set.
>>>>>>> graemeg/cpstrnew
=======
					if not null, then is the new value for the property that will be set. If null, then inDataSize should be zero,
					and the call is then used to remove a previously set value for a property. This removal is only valid for
					some properties, as most properties will always have a default value if not set.
>>>>>>> origin/cpstrnew
=======
					if not null, then is the new value for the property that will be set. If null, then inDataSize should be zero,
					and the call is then used to remove a previously set value for a property. This removal is only valid for
					some properties, as most properties will always have a default value if not set.
>>>>>>> origin/fixes_2.4
	@param			inDataSize	
					the size of the data being provided in inData

	@result			noErr, or various audio unit errors related to properties
}
function AudioUnitSetProperty( inUnit: AudioUnit; inID: AudioUnitPropertyID; inScope: AudioUnitScope; inElement: AudioUnitElement; inData: {const} UnivPtr; inDataSize: UInt32 ): OSStatus; external name '_AudioUnitSetProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0,__IPHONE_2_0) *)

{!
	@function		AudioUnitAddPropertyListener
	@abstract		registration call to receive notifications for when a property changes
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		When an audio unit property value changes, a notification callback can be 
					called by the audio unit to  inform interested parties that this event has 
					occurred. The notification is defined by the tuple of inProc and 
					inProcUserData as paired to the specified property ID, so the previously 
					defined AudioUnitRemovePropertyListener is deprecated because it didn't allow 
					for the provision of the inProcUserData to remove a given listener (so, 
					you should use AudioUnitRemovePropertyListenerWithUserData).
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		When an audio unit property value changes, a notification callback can be called by the audio unit to 
					inform interested parties that this event has occurred. The notification is defined by the tuple of
					inProc and inProcUserData as paired to the specified property ID, so the previously defined 
					AudioUnitRemovePropertyListener is dperecated because it didn't allow for the provision of
					the inProcUserData to remove a given listener (so, you should use AudioUnitRemovePropertyListenerWithUserData).
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
					
	@param			inUnit
					the audio unit
	@param			inID
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					the property identifier
	@param			inProc
					the procedure to call when the property changes (on any scope or element)
=======
					the proeprty identifier
	@param			inProc
					the proceedure to call when the property changes (on any scope or element)
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
	@param			inProc
					the proceedure to call when the property changes (on any scope or element)
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
	@param			inProc
					the proceedure to call when the property changes (on any scope or element)
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
	@param			inProc
					the proceedure to call when the property changes (on any scope or element)
>>>>>>> origin/cpstrnew
=======
					the proeprty identifier
	@param			inProc
					the proceedure to call when the property changes (on any scope or element)
>>>>>>> origin/fixes_2.4
	@param			inProcUserData
					the user data to provide with the callback

	@result			noErr, or various audio unit errors related to properties
}
function AudioUnitAddPropertyListener( inUnit: AudioUnit; inID: AudioUnitPropertyID; inProc: AudioUnitPropertyListenerProc; inProcUserData: UnivPtr ): OSStatus; external name '_AudioUnitAddPropertyListener';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0,__IPHONE_2_0) *)

{!
	@function		AudioUnitRemovePropertyListenerWithUserData
	@abstract		remove a previously registered property listener
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		Removes a previously registered property listener as specified by the inProc 
					and inProcUser data as paired to the property identifier
=======
	@discussion		Removes a previously registered property listener as specified by the inProc and inProcUser data
					as paired to the property identifier
>>>>>>> graemeg/cpstrnew
=======
	@discussion		Removes a previously registered property listener as specified by the inProc and inProcUser data
					as paired to the property identifier
>>>>>>> graemeg/cpstrnew
=======
	@discussion		Removes a previously registered property listener as specified by the inProc and inProcUser data
					as paired to the property identifier
>>>>>>> graemeg/cpstrnew
=======
	@discussion		Removes a previously registered property listener as specified by the inProc and inProcUser data
					as paired to the property identifier
>>>>>>> origin/cpstrnew
=======
	@discussion		Removes a previously registered property listener as specified by the inProc and inProcUser data
					as paired to the property identifier
>>>>>>> origin/fixes_2.4
					
	@param			inUnit
					the audio unit
	@param			inID
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					the property identifier
	@param			inProc
					the procedure previously registered
=======
					the proeprty identifier
	@param			inProc
					the proceedure previously registered
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
	@param			inProc
					the proceedure previously registered
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
	@param			inProc
					the proceedure previously registered
>>>>>>> graemeg/cpstrnew
=======
					the proeprty identifier
	@param			inProc
					the proceedure previously registered
>>>>>>> origin/cpstrnew
=======
					the proeprty identifier
	@param			inProc
					the proceedure previously registered
>>>>>>> origin/fixes_2.4
	@param			inProcUserData
					the user data paired with the provided inProc

	@result			noErr, or various audio unit errors related to properties
}
function AudioUnitRemovePropertyListenerWithUserData( inUnit: AudioUnit; inID: AudioUnitPropertyID; inProc: AudioUnitPropertyListenerProc; inProcUserData: UnivPtr ): OSStatus; external name '_AudioUnitRemovePropertyListenerWithUserData';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{!
	@function		AudioUnitAddRenderNotify
	@abstract		a notification callback to call when an audio unit is asked to render
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		allows an application to register a callback with an audio unit for whenever 
					the audio unit is asked to render. The callback is called both before the 
					audio unit performs its render operations (the render flag's pre-render bit 
					is set) and after the audio unit has completed its render operations (the 
					render flag's post-render bit is set). On post-render, the audio buffer list 
					(ioData) will contain valid audio data that was rendered by the audio unit.
					
					The inProc and inProcUserData are treated as a tuple entity, so when wanting 
					to remove one, both the inProc and its inProcUserData must be specified
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		allows an application to register a callback with an audio unit for whenever the audio unit is asked
					to render. The callback is called both before the audio unit performs its render operations (the render
					flag's pre-render bit is set) and after the audio unit has completed its render operations (the render
					flag's post-render bit is set). On post-render, the audio buffer list (ioData) will contain valid
					audio data that was rendered by the audio unit.
					
					The inProc and inProcUserData are treated as a tuple entity, so when wanting to remove one, both the inProc
					and its inProcUserData must be specified
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
					
	@param			inUnit
					the audio unit
	@param			inProc
					an AURenderCallback proc
	@param			inProcUserData
					the user data that will be provided with the proc when it is called

	@result			noErr, or an audio unit error code
}
function AudioUnitAddRenderNotify( inUnit: AudioUnit; inProc: AURenderCallback; inProcUserData: UnivPtr ): OSStatus; external name '_AudioUnitAddRenderNotify';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)

{!
	@function		AudioUnitRemoveRenderNotify
	@abstract		remove a previously registered render notification proc
	
	@param			inUnit
					the audio unit
	@param			inProc
					an AURenderCallback proc
	@param			inProcUserData
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					the user data that was provided with the proc when it was previously 
					registered
=======
					the user data that was provided with the proc when it was previously registered
>>>>>>> graemeg/cpstrnew
=======
					the user data that was provided with the proc when it was previously registered
>>>>>>> graemeg/cpstrnew
=======
					the user data that was provided with the proc when it was previously registered
>>>>>>> graemeg/cpstrnew
=======
					the user data that was provided with the proc when it was previously registered
>>>>>>> origin/cpstrnew
=======
					the user data that was provided with the proc when it was previously registered
>>>>>>> origin/fixes_2.4

	@result			noErr, or an audio unit error code
}
function AudioUnitRemoveRenderNotify( inUnit: AudioUnit; inProc: AURenderCallback; inProcUserData: UnivPtr ): OSStatus; external name '_AudioUnitRemoveRenderNotify';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)

{!
	@function		AudioUnitGetParameter
	@abstract		Get the value of a parameter
	@discussion		Get the value of a parameter as specified by its ID, scope and element. 
	
	@param			inUnit
					the audio unit
	@param			inID
					the parameter ID
	@param			inScope
					the scope for the parameter
	@param			inElement
					the element on the scope for the parameter
	@param			outValue
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					Must be non-null, and upon success will contain the current value for the 
					specified parameter
=======
					Must be non-null, and upon success will contain the current value for the specified parameter
>>>>>>> graemeg/cpstrnew
=======
					Must be non-null, and upon success will contain the current value for the specified parameter
>>>>>>> graemeg/cpstrnew
=======
					Must be non-null, and upon success will contain the current value for the specified parameter
>>>>>>> graemeg/cpstrnew
=======
					Must be non-null, and upon success will contain the current value for the specified parameter
>>>>>>> origin/cpstrnew
=======
					Must be non-null, and upon success will contain the current value for the specified parameter
>>>>>>> origin/fixes_2.4
	
	@result			noErr, or an audio unit error code (such as InvalidParameter)
}
function AudioUnitGetParameter( inUnit: AudioUnit; inID: AudioUnitParameterID; inScope: AudioUnitScope; inElement: AudioUnitElement; var outValue: AudioUnitParameterValue ): OSStatus; external name '_AudioUnitGetParameter';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0,__IPHONE_2_0) *)

{!
	@function		AudioUnitSetParameter
	@abstract		Set the value of a parameter
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		Set the value of a parameter as specified by its ID, scope and element. 
					Parameter IDs are consistent across all of the elements in a scope - so for a 
					mixer, the "input volume" parameter can be applied on any input, and the 
					particular input is specified by the elementID.
=======
	@discussion		Set the value of a parameter as specified by its ID, scope and element. Parameter IDs are consistent
					across all of the elements in a scope - so for a mixer, the "input volume" parameter can be applied
					on any input, and the particular input is specified by the elementID.
>>>>>>> graemeg/cpstrnew
=======
	@discussion		Set the value of a parameter as specified by its ID, scope and element. Parameter IDs are consistent
					across all of the elements in a scope - so for a mixer, the "input volume" parameter can be applied
					on any input, and the particular input is specified by the elementID.
>>>>>>> graemeg/cpstrnew
=======
	@discussion		Set the value of a parameter as specified by its ID, scope and element. Parameter IDs are consistent
					across all of the elements in a scope - so for a mixer, the "input volume" parameter can be applied
					on any input, and the particular input is specified by the elementID.
>>>>>>> graemeg/cpstrnew
=======
	@discussion		Set the value of a parameter as specified by its ID, scope and element. Parameter IDs are consistent
					across all of the elements in a scope - so for a mixer, the "input volume" parameter can be applied
					on any input, and the particular input is specified by the elementID.
>>>>>>> origin/cpstrnew
=======
	@discussion		Set the value of a parameter as specified by its ID, scope and element. Parameter IDs are consistent
					across all of the elements in a scope - so for a mixer, the "input volume" parameter can be applied
					on any input, and the particular input is specified by the elementID.
>>>>>>> origin/fixes_2.4
	
	@param			inUnit
					the audio unit
	@param			inID
					the parameter ID
	@param			inScope
					the scope for the parameter
	@param			inElement
					the element on the scope for the parameter
	@param			inValue
					the new value for the parameter.
	@param			inBufferOffsetInFrames
					generally should be set to zero - see AudioUnitScheduleParameters
	
	@result			noErr, or an audio unit error code (such as InvalidParameter)
}
function AudioUnitSetParameter( inUnit: AudioUnit; inID: AudioUnitParameterID; inScope: AudioUnitScope; inElement: AudioUnitElement; inValue: AudioUnitParameterValue; inBufferOffsetInFrames: UInt32 ): OSStatus; external name '_AudioUnitSetParameter';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0,__IPHONE_2_0) *)

{!
	@function		AudioUnitScheduleParameters
	@abstract		Schedule changes to the value of a parameter
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		This API is used to schedule intra-buffer changes to the value of a parameter 
					(immediate) or to ramp a parameter from a start value to an end value for a 
					specified number of samples (ramp)
					
					The API allows for the scheduling of multiple parameter events with the one 
					call. All of the parameter events must apply to the current (and only apply to 
					the current) audio unit render call, so the events are scheduled as a part of 
					the pre-render notification callback.
					
					When scheduling an immediate parameter event, the new value at the specified 
					sample buffer offset is provided
					
					When scheduling a ramped parameter, the ramp is scheduled each audio unit 
					render for the duration of the ramp. Each schedule of the the new audio unit 
					render specifies the progress of the ramp.
					
					Parameters that can have events scheduled to them will indicate this through 
					their parameter info struct
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		This API is used to schedule intra-buffer changes to the value of a parameter (immediate) or to
					ramp a parameter from a start value to an end value for a specified number of samples (ramp)
					
					The API allows for the scheduling of multiple parameter events with the one call. All of the
					parameter events must apply to the current (and only apply to the current) audio unit render
					call, so the events are scheduled as a part of the pre-render notification callback.
					
					When scheduling an immediate parameter event, the new value at the specified sample buffer offset
					is provided
					
					When scheduling a ramped parameter, the ramp is scheduled each audio unit render for the duration
					of the ramp. Each schedule of the the new audio unit render specifies the progress of the ramp.
					
					Parameters that can have events scheduled to them will indicate this through their parameter info struct
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
					
	@param			inUnit
					the audio unit
	@param			inParameterEvent
					a pointer to an array of parameter event structs
	@param			inNumParamEvents
					the number of parameter event structs pointed to by inParameterEvent
	
	@result			noErr, or an audio unit error code (such as InvalidParameter)
}
function AudioUnitScheduleParameters( inUnit: AudioUnit; const inParameterEvent: AudioUnitParameterEventPtr; inNumParamEvents: UInt32 ): OSStatus; external name '_AudioUnitScheduleParameters';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)

{!
	@function		AudioUnitRender
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@abstract		the render operation where ioData will contain the results of the audio unit's
					render operations
	@discussion		an audio unit will render the amount of audio data described by 
					inNumberOfFrames and the results of that render will be contained within 
					ioData. The caller should provide audio time stamps where at least the sample 
					time is valid and it is incrementing sequentially from its previous call 
					(so, the next time stamp will be the current time stamp + inNumberFrames) 
					If the sample time is not incrementing sequentially, the audio unit will infer
					that there is some discontinuity with the timeline it is rendering for
	
					The caller must provide a valid ioData AudioBufferList that matches the 
					expected topology for the current audio format for the given bus. The buffer 
					list can be of two variants:
					(1) If the mData pointers are non-null then the audio unit will render its 
					output into those buffers. These buffers should be aligned to 16 byte 
					boundaries (which is normally what malloc will return).
					(2) If the mData pointers are null, then the audio unit can provide pointers 
					to its own buffers. In this case the audio unit is required to keep those
					buffers valid for the duration of the calling thread's I/O cycle
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@abstract		the render operation where ioData will contain the results of the audio unit's render operations
	@discussion		an audio unit will render the amount of audio data described by inNumberOfFrames and the results
					of that render will be contained within ioData. The caller should provide audio time stamps 
					where at least the sample time is valid and it is incrementing sequentially from its
					previous call (so, the next time stamp will be the current time stamp + inNumberFrames) 
					If the sample time is not incrementing sequentially, the audio unit will infer
					that there is some discontinuity with the timeline it is rendering for
	
					The caller must provide a valid ioData AudioBufferList that matches the expected topology for the
					current audio format for the given bus. The buffer list can be of two variants:
					(1) If the mData pointers are non-null then the audio unit will render its output into those buffers
					(2) If the mData pointers are null, then the audio unit can provide pointers to its own buffers. In this case
					the audio unit is required to keep those buffers valid for the duration of the calling thread's I/O cycle
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
					 
	@param			inUnit
					the audio unit
	@param			ioActionFlags
					any appropriate action flags for the render operation
	@param			inTimeStamp
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					the time stamp that applies to this particular render operation. when 
					rendering for multiple output buses the time stamp will generally be the same 
					for each output bus, so the audio unit is able to determine without doubt that 
					this the same render operation
=======
					the time stamp that applies to this particular render operation. when rendering for multiple output buses
					the time stamp will generally be the same for each output bus, so the audio unit is able to determine
					without doubt that this the same render operation
>>>>>>> graemeg/cpstrnew
=======
					the time stamp that applies to this particular render operation. when rendering for multiple output buses
					the time stamp will generally be the same for each output bus, so the audio unit is able to determine
					without doubt that this the same render operation
>>>>>>> graemeg/cpstrnew
=======
					the time stamp that applies to this particular render operation. when rendering for multiple output buses
					the time stamp will generally be the same for each output bus, so the audio unit is able to determine
					without doubt that this the same render operation
>>>>>>> graemeg/cpstrnew
=======
					the time stamp that applies to this particular render operation. when rendering for multiple output buses
					the time stamp will generally be the same for each output bus, so the audio unit is able to determine
					without doubt that this the same render operation
>>>>>>> origin/cpstrnew
=======
					the time stamp that applies to this particular render operation. when rendering for multiple output buses
					the time stamp will generally be the same for each output bus, so the audio unit is able to determine
					without doubt that this the same render operation
>>>>>>> origin/fixes_2.4
	@param			inOutputBusNumber
					the output bus to render for
	@param			inNumberFrames
					the number of sample frames to render
	@param			ioData
					the audio buffer list that the audio unit is to render into.
	
	@result			noErr, or an audio unit render error
}
function AudioUnitRender( inUnit: AudioUnit; var ioActionFlags: AudioUnitRenderActionFlags; const (*var*) inTimeStamp: AudioTimeStamp; inOutputBusNumber: UInt32; inNumberFrames: UInt32; ioData: AudioBufferListPtr ): OSStatus; external name '_AudioUnitRender';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_OS_MAC}
function AudioUnitProcess( inUnit: AudioUnit; var ioActionFlags: AudioUnitRenderActionFlags; const (*var*) inTimeStamp: AudioTimeStamp; inNumberFrames: UInt32; var ioData: AudioBufferList ): OSStatus; external name '_AudioUnitProcess';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7,__IPHONE_NA) *)

function AudioUnitProcessMultiple( inUnit: AudioUnit; var ioActionFlags: AudioUnitRenderActionFlags; const (*var*) inTimeStamp: AudioTimeStamp; inNumberFrames: UInt32; inNumberInputBufferLists: UInt32; {const} inInputBufferLists: AudioBufferListPtrPtr; inNumberOutputBufferLists: UInt32; ioOutputBufferLists: AudioBufferListPtrPtr ): OSStatus; external name '_AudioUnitProcessMultiple';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7,__IPHONE_NA) *)
{$endc} {TARGET_OS_MAC}

{!
	@function		AudioUnitReset
	@abstract		reset an audio unit's render state
	@discussion		This call will clear any render state of an audio unit. For instance, with a 
					delay or reverb type of audio unit reset will clear any of the delay lines 
					maintained within the audio unit. Typically, this call is made when an
					audio unit was previously rendering, and was taken out of the render chain 
					(say, the track it is in was muted) and is being added back in (unmuted). 
					The host should reset the audio unit before adding it back so that it doesn't 
					produce audio from its delay lines that is no longer valid.
					
					The call should only clear memory, it should NOT allocate or free memory 
					resources (this is done in the Initialize calls).
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{!
	@function		AudioUnitReset
	@abstract		reset an audio unit's render state
	@discussion		This call will clear any render state of an audio unit. For instance, with a delay or reverb type of audio unit
					reset will clear any of the delay lines maintained within the audio unit. Typically, this call is made when an
					audio unit was previously rendering, and was taken out of the render chain (say, the track it is in was muted)
					and is being added back in (unmuted). The host should reset the audio unit before adding it back so that 
					it doesn't produce audio from its delay lines that is no longer valid.
					
					The call should only clear memory, it should NOT allocate or free memory resources (this is done in the 
					Initialize calls).
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
	
	@param			inUnit
					the audio unit
	@param			inScope
					the scope - typically this is set to GlobalScope
	@param			inElement
					the element - typically this is set to 0
	
	@result			noErr, or an audio unit error
}
function AudioUnitReset( inUnit: AudioUnit; inScope: AudioUnitScope; inElement: AudioUnitElement ): OSStatus; external name '_AudioUnitReset';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0,__IPHONE_2_0) *)


{!
	@enum			AudioUnitRange
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		the range of selectors that are used to dispatch through to the various audio 
					unit API
=======
	@discussion		the range of selectors that are used to dispatch through to the various audio unit API
>>>>>>> graemeg/cpstrnew
=======
	@discussion		the range of selectors that are used to dispatch through to the various audio unit API
>>>>>>> graemeg/cpstrnew
=======
	@discussion		the range of selectors that are used to dispatch through to the various audio unit API
>>>>>>> graemeg/cpstrnew
=======
	@discussion		the range of selectors that are used to dispatch through to the various audio unit API
>>>>>>> origin/cpstrnew
=======
	@discussion		the range of selectors that are used to dispatch through to the various audio unit API
>>>>>>> origin/fixes_2.4

	@constant		kAudioUnitRange
	@constant		kAudioUnitInitializeSelect
	@constant		kAudioUnitUninitializeSelect
	@constant		kAudioUnitGetPropertyInfoSelect
	@constant		kAudioUnitGetPropertySelect
	@constant		kAudioUnitSetPropertySelect
	@constant		kAudioUnitAddPropertyListenerSelect
	@constant		kAudioUnitRemovePropertyListenerSelect
	@constant		kAudioUnitRemovePropertyListenerWithUserDataSelect
	@constant		kAudioUnitAddRenderNotifySelect
	@constant		kAudioUnitRemoveRenderNotifySelect
	@constant		kAudioUnitGetParameterSelect
	@constant		kAudioUnitSetParameterSelect
	@constant		kAudioUnitScheduleParametersSelect
	@constant		kAudioUnitRenderSelect
	@constant		kAudioUnitResetSelect
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@constant		kAudioUnitComplexRenderSelect
	@constant		kAudioUnitProcessSelect
	@constant		kAudioUnitProcessMultipleSelect
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
	kAudioUnitRange = $0000;	// range of selectors for audio units
	kAudioUnitInitializeSelect = $0001;
	kAudioUnitUninitializeSelect = $0002;
	kAudioUnitGetPropertyInfoSelect = $0003;
	kAudioUnitGetPropertySelect = $0004;
	kAudioUnitSetPropertySelect = $0005;
	kAudioUnitAddPropertyListenerSelect = $000A;
	kAudioUnitRemovePropertyListenerSelect = $000B;
	kAudioUnitRemovePropertyListenerWithUserDataSelect = $0012;
	kAudioUnitAddRenderNotifySelect = $000F;
	kAudioUnitRemoveRenderNotifySelect = $0010;
	kAudioUnitGetParameterSelect = $0006;
	kAudioUnitSetParameterSelect = $0007;
	kAudioUnitScheduleParametersSelect = $0011;
	kAudioUnitRenderSelect = $000E;
	kAudioUnitResetSelect = $0009;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitComplexRenderSelect = $0013;
	kAudioUnitProcessSelect = $0014;
	kAudioUnitProcessMultipleSelect = $0015;

//================================================================================================
//#pragma mark -
//#pragma mark Dispatch function prototypes

type
	AudioUnitInitializeProc = function( self: UnivPtr ): OSStatus;

type
	AudioUnitUninitializeProc = function( self: UnivPtr ): OSStatus;

type
	AudioUnitGetPropertyInfoProc = function( self: UnivPtr; prop: AudioUnitPropertyID; scope: AudioUnitScope; elem: AudioUnitElement; var outDataSize: UInt32; var outWritable: Boolean ): OSStatus;

type
	AudioUnitGetPropertyProc = function( self: UnivPtr; inID: AudioUnitPropertyID; inScope: AudioUnitScope; inElement: AudioUnitElement; outData: UnivPtr; var ioDataSize: UInt32 ): OSStatus;

type
	AudioUnitSetPropertyProc = function( self: UnivPtr; inID: AudioUnitPropertyID; inScope: AudioUnitScope; inElement: AudioUnitElement; inData: {const} UnivPtr; inDataSize: UInt32 ): OSStatus;

type
	AudioUnitAddPropertyListenerProc = function( self: UnivPtr; prop: AudioUnitPropertyID; proc: AudioUnitPropertyListenerProc; userData: UnivPtr ): OSStatus;

type
	AudioUnitRemovePropertyListenerProc = function( self: UnivPtr; prop: AudioUnitPropertyID; proc: AudioUnitPropertyListenerProc ): OSStatus;

type
	AudioUnitRemovePropertyListenerWithUserDataProc = function( self: UnivPtr; prop: AudioUnitPropertyID; proc: AudioUnitPropertyListenerProc; userData: UnivPtr ): OSStatus;

type
	AudioUnitAddRenderNotifyProc = function( self: UnivPtr; proc: AURenderCallback; userData: UnivPtr ): OSStatus;

type
	AudioUnitRemoveRenderNotifyProc = function( self: UnivPtr; proc: AURenderCallback; userData: UnivPtr ): OSStatus;

type
	AudioUnitScheduleParametersProc = function( self: UnivPtr; const (*var*) events: AudioUnitParameterEvent; numEvents: UInt32 ): OSStatus;

type
	AudioUnitResetProc = function( self: UnivPtr; inScope: AudioUnitScope; inElement: AudioUnitElement ): OSStatus;

type
	AudioUnitComplexRenderProc = function( self: UnivPtr; var ioActionFlags: AudioUnitRenderActionFlags; const (*var*) inTimeStamp: AudioTimeStamp; inOutputBusNumber: UInt32; inNumberOfPackets: UInt32; var outNumberOfPackets: UInt32; var outPacketDescriptions: AudioStreamPacketDescription; ioData: AudioBufferListPtr; outMetadata: UnivPtr; var outMetadataByteSize: UInt32 ): OSStatus;

type
	AudioUnitProcessProc = function( self: UnivPtr; var ioActionFlags: AudioUnitRenderActionFlags; const (*var*) inTimeStamp: AudioTimeStamp; inNumberFrames: UInt32; ioData: AudioBufferListPtr ): OSStatus;

type
	AudioUnitProcessMultipleProc = function( self: UnivPtr; var ioActionFlags: AudioUnitRenderActionFlags; const (*var*) inTimeStamp: AudioTimeStamp; inNumberFrames: UInt32; inNumberInputBufferLists: UInt32; {const} inInputBufferLists: AudioBufferListPtrPtr; inNumberOutputBufferLists: UInt32; ioOutputBufferLists: AudioBufferListPtrPtr ): OSStatus;


{!
	@typedef		AudioUnitGetParameterProc
	@discussion		This proc can be exported through the FastDispatch property or is used as the prototype for
					an audio component dispatch for this selector. 
=======
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
//#pragma mark Fast-dispatch function prototypes
{!
	@typedef		AudioUnitGetParameterProc
	@discussion		This proc can be exported through the FastDispatch property. A host can
					then retrieve this Proc using that property and thus make a direct
					call into your audio unit bypassing the component dispatch glue which
					can add considerable overhead and is desirable to avoid.
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
					
					The arguments are the same as are provided to the corresponding API call
	
	@param			inComponentStorage
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					For a component manager component, this is the component instance storage 
					pointer
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> graemeg/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> graemeg/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> graemeg/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> origin/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> origin/fixes_2.4
	@param			inID
	@param			inScope
	@param			inElement
	@param			outValue
}
type
	AudioUnitGetParameterProc = function( inComponentStorage: UnivPtr; inID: AudioUnitParameterID; inScope: AudioUnitScope; inElement: AudioUnitElement; var outValue: AudioUnitParameterValue ): OSStatus;
								
{!
	@typedef		AudioUnitSetParameterProc
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		This proc can be exported through the FastDispatch property or is used as the prototype for
					an audio component dispatch for this selector. 
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		This proc can be exported through the FastDispatch property. A host can
					then retrieve this Proc using that property and thus make a direct
					call into your audio unit bypassing the component dispatch glue which
					can add considerable overhead and is desirable to avoid.
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
					
					The arguments are the same as are provided to the corresponding API call
	
	@param			inComponentStorage
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					For a component manager component, this is the component instance storage 
					pointer
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> graemeg/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> graemeg/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> graemeg/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> origin/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> origin/fixes_2.4
	@param			inID
	@param			inScope
	@param			inElement
	@param			inValue
	@param			inBufferOffsetInFrames
}
type
	AudioUnitSetParameterProc = function( inComponentStorage: UnivPtr; inID: AudioUnitParameterID; inScope: AudioUnitScope; inElement: AudioUnitElement; inValue: AudioUnitParameterValue; inBufferOffsetInFrames: UInt32 ): OSStatus;
								
{!
	@typedef		AudioUnitRenderProc
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		This proc can be exported through the FastDispatch property or is used as the prototype for
					an audio component dispatch for this selector. 
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		This proc can be exported through the FastDispatch property. A host can
					then retrieve this Proc using that property and thus make a direct
					call into your audio unit bypassing the component dispatch glue which
					can add considerable overhead and is desirable to avoid.
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
					
					The arguments are the same as are provided to the corresponding API call
	
	@param			inComponentStorage
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					For a component manager component, this is the component instance storage 
					pointer
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> graemeg/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> graemeg/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> graemeg/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> origin/cpstrnew
=======
					For a component manager component, this is the component instance storage pointer
>>>>>>> origin/fixes_2.4
	@param			ioActionFlags
	@param			inTimeStamp
	@param			inOutputBusNumber
	@param			inNumberFrames
	@param			ioData
}
type
	AudioUnitRenderProc = function( inComponentStorage: UnivPtr; var ioActionFlags: AudioUnitRenderActionFlags; const (*var*) inTimeStamp: AudioTimeStamp; inOutputBusNumber: UInt32; inNumberFrames: UInt32; ioData: AudioBufferListPtr ): OSStatus;


//=====================================================================================================================
//#pragma mark -
//#pragma mark Deprecated
{!
	@enum			deprecated - Audio unit errors
	@discussion		These are the various errors that can be returned by AudioUnit... API calls

	@constant		kAudioUnitErr_IllegalInstrument
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					Apple's DLS synth returns this error if information about a particular 
					instrument patch is requested, but is not valid.
	@constant		kAudioUnitErr_InstrumentTypeNotFound
					Apple's DLS synth returns this error if information about a particular 
					instrument patch is requested, but is not valid.
	@constant		kAudioUnitErr_UnknownFileType
					If an audio unit uses external files as a data source, this error is returned
					if a file is invalid (Apple's DLS synth returns this error)
	@constant		kAudioUnitErr_FileNotSpecified
					If an audio unit uses external files as a data source, this error is returned
					if a file hasn't been set on it 
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
					Apple's DLS synth returns this error if information about a particular instrument patch is requested, but is not valid.
	@constant		kAudioUnitErr_InstrumentTypeNotFound
					Apple's DLS synth returns this error if information about a particular instrument patch is requested, but is not valid.
	@constant		kAudioUnitErr_UnknownFileType
					If an audio unit uses external files as a data source, this error is returned if a file is invalid (Apple's DLS synth returns this error)
	@constant		kAudioUnitErr_FileNotSpecified
					If an audio unit uses external files as a data source, this error is returned if a file hasn't been set on it 
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
					(Apple's DLS synth returns this error)
}
const
	kAudioUnitErr_IllegalInstrument = -10873;
	kAudioUnitErr_InstrumentTypeNotFound = -10872;
	kAudioUnitErr_UnknownFileType = -10870;
	kAudioUnitErr_FileNotSpecified = -10869;

//#if !__LP64__ && !TARGET_OS_IPHONE
{$ifc not TARGET_CPU_64 and not TARGET_OS_IPHONE}
// this call is deprecated and replaced by AudioUnitRemovePropertyListenerWithUserData
// this allows apps to use the same function pointer more than once
// you provide the same function ptr and user data as provided when you add a property listener
function AudioUnitRemovePropertyListener( inUnit: AudioUnit; inID: AudioUnitPropertyID; inProc: AudioUnitPropertyListenerProc ): OSStatus; external name '_AudioUnitRemovePropertyListener';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
//#endif
{$endc}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
