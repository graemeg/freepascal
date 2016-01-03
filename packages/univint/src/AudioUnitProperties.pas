{
     File:       AudioUnitProperties.h
 
     Contains:   Property constants for AudioUnits
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  (c) 2001-2008 by Apple, Inc., all rights reserved.
=======
     Copyright:  (c) 2001-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Copyright:  (c) 2001-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Copyright:  (c) 2001-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Copyright:  (c) 2001-2008 by Apple Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
     Copyright:  (c) 2001-2008 by Apple Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
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

unit AudioUnitProperties;
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
>>>>>>> origin/cpstrnew
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
uses MacTypes,AUComponent,CoreAudioTypes,MIDIServices,CFBase,CFURL;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


//=====================================================================================================================
//#pragma mark Overview
{!
    @header AudioUnitProperties
	This file defines a collection of property IDs and their accompanying structures that are used
	by audio units. Properties form the basis of much of the audio unit API. You use them with 
	the Audio Unit framework's Get and Set property API calls declared in the AUComponent.h header
	file:
	
		AudioUnitGetPropertyInfo
		AudioUnitGetProperty
		AudioUnitSetProperty
	
	This file first lists generic audio unit properties (those that are potentially applicable to 
	any audio unit), followed by properties specific to Apple audio units.

	Apple reserves property IDs from 0 -> 63999. Developers are free to use property IDs above this 
	range.
	
	All property values are passed by reference. When a property's value type is listed below,
	that value type is always passed by reference. For example, CFStringRef is passed as 
	&myCFString.
	
	Properties are described below using a general form:
		Scope:				The audio unit scope that the property applies to. For example, Global, 
							Input, Output, etc. For an explanation of audio unit scopes, see the 
							Audio Unit Programming Guide in the ADC Reference Library.
		Value Type:			The data type used to hold the value associated with the property. For 
							example, CFStringRef or UInt32
		Access:				How a host application can access the property in a hosted audio unit: 
							read only, write only, or read/write.
		Description:		A description of the property's role.
	
	The descriptions in this file apply to typical or recommended usage. Audio unit developers can vary 
	the way each property is used. For example, a property may be described as applying to both input and  
	output scopes, but a given audio unit may implement the property on the input scope only. As another 
	example, a property may be described here as having read/write access, but an audio unit may 
	implement the property as read only.
	
	The properties are divided into two primary sections:
		(1) Core/Embedded Implementation
			- these properties are available on all platforms where audio units are available
		
		(2) Desktop 
			- these properties are available on only available on desktop platforms
	
	The following file is organised into these two sections
}

//#pragma mark -
//#pragma mark Core Implementation
//#pragma mark -
{!
    @enum           Audio Unit scope types
    @abstract       The scope IDs for audio units define basic roles and contexts for an audio unit's state.
    @discussion		Each scope is a discrete context. Apple reserves scope IDs from 0 through 1024.

	@constant		kAudioUnitScope_Global	The context for audio unit characteristics that apply to the audio unit as a 
											whole
	@constant		kAudioUnitScope_Input	The context for audio data coming into an audio unit
	@constant		kAudioUnitScope_Output	The context for audio data leaving an audio unit
	@constant		kAudioUnitScope_Group	A context specific to the control scope of parameters (for instance, 
											MIDI Channels is an example of this scope)
	@constant		kAudioUnitScope_Part	A distinct rendering context. For instance a single timbre in a multi-timbral 
											instrument, a single loop in a multi looping capable looper unit, etc.
	@constant		kAudioUnitScope_Note	A scope that can be used to apply changes to an individual note. The 
											elementID used with this scope is the unique note ID returned from
											a started note (see MusicDeviceStartNote)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@constant		kAudioUnitScope_Layer	A context which functions as a layer within a part and allows
											grouped control of LayerItem-scope parameters.
											An example is the percussive attack layer for an electric organ instrument
	@constant		kAudioUnitScope_LayerItem	A scope which represents an indivual element within a particular Layer scope.
											The individual sample zones, envelope generators, and filters within a synth are
											examples of this.
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
	kAudioUnitScope_Global = 0;
	kAudioUnitScope_Input = 1;
	kAudioUnitScope_Output = 2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitScope_Group = 3;
	kAudioUnitScope_Part = 4;
	kAudioUnitScope_Note = 5;
	kAudioUnitScope_Layer = 6;
	kAudioUnitScope_LayerItem = 7;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
//#if !TARGET_OS_IPHONE;
{$ifc not TARGET_OS_IPHONE}
	kAudioUnitScope_Group = 3;
	kAudioUnitScope_Part = 4;
	kAudioUnitScope_Note = 5;
//#endif;
{$endc}
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


//=====================================================================================================================
//#pragma mark Audio Unit Properties

{!
    @enum           Generic Property IDs
    @abstract       Properties that can apply to any audio unit.
	
	@constant		kAudioUnitProperty_ClassInfo
						Scope:			Global (or Part for a part scope preset)
						Value Type:		CFDictionaryRef
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
						
						The complete state of an audio unit if on global scope. An audio unit that supports part scope, may also support presets on the part scope
						that apply to individual parts.
						
						After a host sets this property it needs to notify the parameter listeners that the values of the parameters of an AU may have changed (so
						views, etc, can update their state). Something like the following code should be used:
						
						<code>
						
						AudioUnitParameter changedUnit;
						changedUnit.mAudioUnit = theChangedAU;
						changedUnit.mParameterID = kAUParameterListener_AnyParameter;
						AUParameterListenerNotify (NULL, NULL, &changedUnit);
						</code>

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
						Access:			Read / Write
						
						The complete state of an audio unit if on global scope. An audio unit that supports part scope, may also support presets on the part scope
						that apply to individual parts
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
						
	@constant		kAudioUnitProperty_MakeConnection
						Scope:			Input
						Value Type:		AudioUnitConnection
						Access:			Write
	
	@constant		kAudioUnitProperty_SampleRate
						Scope:			Input / Output
						Value Type:		Float64
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/fixes_2.4
						
	@constant		kAudioUnitProperty_ParameterList
						Scope:			Any
						Value Type:		AudioUnitParameterID
						Access:			Read
						
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						The list of parameter IDs on the specified scope
=======
						The list of parameter IDs on the specifed scope
>>>>>>> graemeg/cpstrnew
=======
						The list of parameter IDs on the specifed scope
>>>>>>> graemeg/cpstrnew
=======
						The list of parameter IDs on the specifed scope
>>>>>>> graemeg/cpstrnew
=======
						The list of parameter IDs on the specifed scope
>>>>>>> origin/cpstrnew
=======
						The list of parameter IDs on the specifed scope
>>>>>>> origin/fixes_2.4
						
	@constant		kAudioUnitProperty_ParameterInfo
						Scope:			Any
						Value Type:		AudioUnitParameterInfo
						Access:
						
						The info struct describes the general characteristics of an individual parameterID
						
	@constant		kAudioUnitProperty_FastDispatch
						Scope:			Global
						Value Type:		void* (function pointer)
						Access:			Read
						
						The caller provides the selector for a given audio unit API, and retrieves a function pointer for that selector. For instance,
						this enables the caller to retrieve the function pointer for the AudioUnitRender call, so that call can be made directly
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						through to the audio unit to avoid the overhead of the ComponentMgr's dispatch.
=======
						through to the audio unit to avoid the overhead of the Component Mgr's dispatch.
>>>>>>> graemeg/cpstrnew
=======
						through to the audio unit to avoid the overhead of the Component Mgr's dispatch.
>>>>>>> graemeg/cpstrnew
=======
						through to the audio unit to avoid the overhead of the Component Mgr's dispatch.
>>>>>>> graemeg/cpstrnew
=======
						through to the audio unit to avoid the overhead of the Component Mgr's dispatch.
>>>>>>> origin/cpstrnew
=======
						through to the audio unit to avoid the overhead of the Component Mgr's dispatch.
>>>>>>> origin/fixes_2.4
	
	@constant		kAudioUnitProperty_CPULoad
						Scope:			Global
						Value Type:		Float64
						Access:			Read
						
						Can be used to retrieve the duty cycle (as a value from 0 to 1) of the render time that an audio unit is spending in its render call.
	
	@constant		kAudioUnitProperty_StreamFormat
						Scope:			Input / Output
						Value Type:		AudioStreamBasicDescription
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/fixes_2.4
						
						An AudioStreamBasicDescription is used to specify the basic format for an audio data path. For instance, 2 channels, 44.1KHz, Float32 linear pcm.
						The value can be both set and retrieve from an I/O element (bus)
	
	@constant		kAudioUnitProperty_ElementCount
						Scope:			Any (though Global scope will always have and element count of 1)
						Value Type:		UInt32
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/fixes_2.4
						
						Most audio units will only implement the read version of this call, thus they would have a fixed bus topology (number of input and output elements/buses).
						Some audio units possess the capability to add or remove elements, so in that case this property will be writable.
						
	@constant		kAudioUnitProperty_Latency
						Scope:			Global
						Value Type:		Float64
						Access:			Read
						
						The processing latency (the time it takes an audio unit to represent an input in its audio output) specified in seconds
						
	@constant		kAudioUnitProperty_SupportedNumChannels
						Scope:			Global
						Value Type:		AUChannelInfo array
						Access:			Read
						
						The size of this property will represent the number of AUChannelInfo structs that an audio unit provides. Each entry describes a particular number of 
						channels on any input, matched to a particular number of channels on any output. Thus an entry (2, 2) says the audio unit will support a channel configuration
						of 2 channels on an input and 2 channels on an output.
						
						Negative numbers (-1, -2) are used to indicate *any* number of channels. So (-1, -1) means any number of channels on input and output as long as they are the same.
						(1, -2) means any number of channels on input or output buses
						
						A negative number less than -2 is used to indicate a total number of channels across every bus on that scope, regardless of how many channels are set on any
						particular bus.
						
						Zero on any side (typically only input) means that the audio unit doesn't have any input elements, and is expressing the capability of configuring its output channels.
						
	@constant		kAudioUnitProperty_MaximumFramesPerSlice
						Scope:			Global
						Value Type:		UInt32
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/fixes_2.4
						
						This property is used to describe to an audio unit the maximum number of samples it will be asked to produce on any single given call to audio unit render. 
						
						If an audio unit can require more or less input data than its output request, then it should limit any given request for input to this number of frames (that is,
						it should "break up" its input pulls).
	
	@constant		kAudioUnitProperty_SetExternalBuffer
						Scope:			Global
						Value Type:		AudioUnitExternalBuffer
						Access:			Write
						
						This is used to provide to an audio unit a buffer that it can use with its input render callback's audio buffer list
	
	@constant		kAudioUnitProperty_ParameterValueStrings
						Scope:			Any
						Value Type:		CFArrayRef
						Access:			Read
						
						Some audio unit parameters that are of an index type, can also provide names for each value of the parameter. This property returns an array containing CFStrings, where
						each element in the array is the name that should be used for that parameter value. The size of the array should be the same as the range between the parameters min and max values.
						The array's strings can then be used to build a menu for that parameter.
	
	@constant		kAudioUnitProperty_GetUIComponentList
						Scope:			Any
						Value Type:		AudioComponentDescription array
						Access:			Read
						
						Presents an array of AudioComponentDescription that are of type 'auvw' (AudioUnitCarbonView). These are the carbon based custom views for that audio unit. 
	
	@constant		kAudioUnitProperty_AudioChannelLayout
						Scope:			Input/Output
						Value Type:		struct AudioChannelLayout
						Access:			read/write

						Description:
						Describes for a given scope/element the order of channels within a given stream. 
						The number of channels it describes must match the number of channels set for that 
						scope/element. Each input and output bus in an audio unit can have one instance of 
						this property. 
						
						Some audio units require this property. For example, the 3DMixer unit must 
						implement this property on its output bus. If a host application attempts to 
						clear the value of this property on a bus that requires a valid value, the
						audio unit will return a kAudioUnitErr_InvalidPropertyValue error.
						
						Input and output buses can be in one of three states in regard to Audio 
						channel layout:
						
						1. implemented and set
						2. implemented but not set 
						3. unimplemented 
						
						Requesting the value of this property when it is implemented but not set
						results in a kAudioUnitErr_PropertyNotInUse error.
						
						Use the kAudioUnitProperty_AudioChannelLayout property whenever channel
						layout is relevant. By comparison, the kAudioUnitProperty_StreamFormat 
						property cannot specify channel layout or purpose.
						
						See also: kAudioUnitProperty_SupportedChannelLayoutTags, 
						kAudioUnitProperty_StreamFormat.
	 
	@constant		kAudioUnitProperty_TailTime
						Scope:			Global
						Value Type:		Float64
						Access:			Read
						
						The time in seconds that will remain after the last valid input of any audio unit has been processed before the output is silent. For example, this could be the total
						decay time of a reverb or a delay. In general this will be a conservative estimate.
	
	@constant		kAudioUnitProperty_BypassEffect
						Scope:			Global
						Value Type:		UInt32
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/fixes_2.4
						
						A boolean value that can be used to bypass the processing in an effect unit, so that the input is passed unchanged through to the output
	
	@constant		kAudioUnitProperty_LastRenderError
						Scope:			Global
						Value Type:		OSStatus
						Access:			Read
						
						This property is set if there is an error in AudioUnitRender. The AU will then fire a property changed notification to any listeners on this property and 
						those listeners can then use this property ID to retrieve that error.
	
	@constant		kAudioUnitProperty_SetRenderCallback
						Scope:			Input
						Value Type:		AURenderCallbackStruct
						Access:			Write
						
						This is used to provide the audio unit with input on the specified element (input bus) with audio data from the provided callback. The callback is delivered a buffer list
						which it must fill in with audio data. If no data is available, it should set the audio data to 0 (silence). In the normal case, f an error is returned, the audio is not processed 
						and the audio unit will return an error from AudioUnitRender.

	@constant		kAudioUnitProperty_FactoryPresets
						Scope:			Global
						Value Type:		CFArray of AUPreset structures
						Access:			Read
						
						An array of preset structures that provide a name and number for each preset. A factory preset is then chosen using the PresentPreset property. 
	
	@constant		kAudioUnitProperty_ContextName
						Scope:			Global
						Value Type:		CFString
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/fixes_2.4
						
						The host can set this as information to the audio unit to describe something about the context within which the audio unit is instantiated. For instance, "track 3" could
						be set as the context, so that the audio unit's view could then display "My audio unit on track 3" as information to the user of the particular context for any audio unit.
						
	@constant		kAudioUnitProperty_RenderQuality
						Scope:			Global
						Value Type:		UInt32
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/fixes_2.4
						
						A value (0 - 127) that can be used to control the quality (complexity) of the rendering operation. A typical usage is to set render quality to maximum for best quality, but
						if CPU usage is a concern a lesser quality can be set to trade off render quality.
	
	@constant		kAudioUnitProperty_HostCallbacks
						Scope:			Global
						Value Type:		HostCallbackInfo
						Access:			Write
						
						The audio unit should only call the host callbacks while it is in its render function. The audio unit must provide the client info when calling the callbacks as provided
						by the host. They are provided as a means for an audio unit to gain information from the host about parameters that may affect its rendering operation. 
						For example, what is the current beat of the host, is the transport running, and so forth.
						
						Any of the parameters of the callback function, when called by the audio unit, can be NULL. This indicates that the unit doesn't want to know that particular information. 
						The exception is that the unit must always specify the HostUserData which was be supplied to the unit when the property was set.

						If the host is unable to provide the requested information then it can return the kAudioUnitErr_CannotDoInCurrentContext error code

	@constant		kAudioUnitProperty_InPlaceProcessing
						Scope:			Global
						Value Type:		UInt32
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/fixes_2.4
						
						A property that can be used to determine if the audio unit can process input data on the same data as is provided to it, and if so this can be turned off if the host
						has a particular buffer management strategy and such an operation would defeat that.
	
	@constant		kAudioUnitProperty_ElementName
						Scope:				any
						Value Type:			CFStringRef
						Access:				read/write
						Description:
						The name of the specified element. The Host owns a reference to this property value 
						(as with all other CF properties), and should release the string retrieved or used when setting.
	
	@constant		kAudioUnitProperty_CocoaUI
						Scope:				Global
						Value Type:			struct AudioUnitCocoaViewInfo
						Access:				read

						Publishes the audio unit's custom Cocoa NSViews. The Host can determine how big this structure is by 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						querying the size of the property (i.e., How many alternate UI classes there are for the unit)
=======
						querying the size of the property (ie. How many alternate UI classes there are for the unit)
>>>>>>> graemeg/cpstrnew
=======
						querying the size of the property (ie. How many alternate UI classes there are for the unit)
>>>>>>> graemeg/cpstrnew
=======
						querying the size of the property (ie. How many alternate UI classes there are for the unit)
>>>>>>> graemeg/cpstrnew
=======
						querying the size of the property (ie. How many alternate UI classes there are for the unit)
>>>>>>> origin/cpstrnew
=======
						querying the size of the property (ie. How many alternate UI classes there are for the unit)
>>>>>>> origin/fixes_2.4
						Typically, most audio units will provide 1 UI class per unit
	
	@constant		kAudioUnitProperty_SupportedChannelLayoutTags
						Scope:				Input/Output
						Value Type:			AudioChannelLayoutTags[ variable number of elements ]
						Access:				read only

						Used with GetProperty to ascertain what an audio unit understands about
						laying out of channel orders. This will normally return one or more of the specified layout tags.
						
						When a specific set of layouts are returned, the client then uses the 
						kAudioUnitProperty_AudioChannelLayout property (with one of those layout tags specified) to set 
						the unit to use that layout. In this case the client (and the audio unit when reporting its 
						AudioChannelLayout) is only expected to have set an AudioChannelLayout which only sets the 
						layout tag as the valid field.
						
						Custom Channel Maps:
						Some audio units may return the tag:
							kAudioChannelLayoutTag_UseChannelDescriptions
						
						In this case, the host then can look at supported number of channels on that scope
						(using the kAudioUnitProperty_SupportedNumChannels), and supply an AudioChannelLayout with the 
						kAudioUnitProperty_AudioChannelLayout property to specify the layout, number of channels
						and location of each of those channels. This custom channel map MUST have a channel valence
						that is supported by the Audio Unit.
						
						The UseChannelBitmap field is NOT used within the context of the AudioUnit.
	
	@constant		kAudioUnitProperty_ParameterIDName
						Scope:				any
						Value Type:			AudioUnitParameterNameInfo
						Access:				read

						An audio unit returns the full parameter name in the GetParameterInfo struct/property.
						In some display situations however, there may only be room for a few characters, and
						truncating this full name may give a less than optimal name for the user. Thus, 
						this property can be used to ask the audio unit whether it can supply a truncated name, with
						the host suggesting a length (number of characters). If the unit returns a longer
						name than the host requests, that name maybe truncated to the requested characters in display.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						The unit could return a shorter name than requested as well. The unit returns a CFString
=======
						The unit could return a shorter name than requeseted as well. The unit returns a CFString
>>>>>>> graemeg/cpstrnew
=======
						The unit could return a shorter name than requeseted as well. The unit returns a CFString
>>>>>>> graemeg/cpstrnew
=======
						The unit could return a shorter name than requeseted as well. The unit returns a CFString
>>>>>>> graemeg/cpstrnew
=======
						The unit could return a shorter name than requeseted as well. The unit returns a CFString
>>>>>>> origin/cpstrnew
=======
						The unit could return a shorter name than requeseted as well. The unit returns a CFString
>>>>>>> origin/fixes_2.4
						that should be released by the host. When using this property, the host asks for
						the name in the same scope and element as the unit publishes the parameter.

	@constant		kAudioUnitProperty_ParameterClumpName
						Scope:				any
						Value Type:			AudioUnitParameterNameInfo
						Access:				read

						This works in a similar manner to the ParameterIDName property, except that the inID
						value is one of the clumpID's that are returned with the audio unit's ParameterInfo 
						structure.
	
	@constant		kAudioUnitProperty_PresentPreset
						Scope:				Global/Part
						Value Type:			AUPreset
						Access:				read/write

						This property replaces the deprecated CurrentPreset property, due to the ambiguity of
						ownership of the CFString of the preset name in the older CurrentPreset property. 
						With PresentPreset the client of the audio unit owns the CFString when it retrieves the
						preset with PresentPreset and is expected to release this (as with ALL properties 
						that retrieve a CF object from an audio unit).
	
	@constant		kAudioUnitProperty_OfflineRender
						Scope:				Global
						Value Type:			UInt32
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:				read/write
=======
						Access:				Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:				Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:				Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:				Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:				Read / Write
>>>>>>> origin/fixes_2.4
						
						This is used by the host to indicate when an audio unit (that normally operates within a general real-time calling model) is 
						rendering in an offline context. A typical usage of this is to set this to true when the rendering operation an audio unit is being used within is 
						going to write out the results to a file. The value defaults to false, as the common usage of audio units is for real-time processing
	
	@constant		kAudioUnitProperty_ParameterStringFromValue
						Scope:				any
						Value Type:			AudioUnitParameterStringFromValue
						Access:				read

						This property is used with parameters that are marked with the
						kAudioUnitParameterFlag_HasName parameter info flag. This indicates that some
						(or all) of the values represented by the parameter can and should be
						represented by a special display string.
						
						This is NOT to be confused with kAudioUnitProperty_ParameterValueStrings. That property
						is used with parameters that are indexed and is typically used for instance to build
						a menu item of choices for one of several parameter values.
						
						kAudioUnitProperty_ParameterStringFromValue can have a continuous range, and merely states
						to the host that if it is displaying those parameter's values, they should request
						a name any time any value of the parameter is set when displaying that parameter.
						
						For instance (a trivial example), a unit may present a gain parameter in a dB scale,
						and wish to display its minimum value as "negative infinity". In this case, the audio unit
						will not return names for any parameter value greater than its minimum value - so the host
						will then just display the parameter value as is. For values less than or equal to the 
						minimum value, the audio unit will return a string for "negative infinity" which the host can
						use to display appropriately.
						
						A less trivial example might be a parameter that presents its values as seconds. However,
						in some situations this value should be better displayed in a SMPTE style of display:
							HH:MM:SS:FF
						In this case, the audio unit would return a name for any value of the parameter.
						
						The GetProperty call is used in the same scope and element as the inParamID 
						that is declared in the struct passed in to this property.
						
						If the *inValue member is NULL, then the audio unit should take the current value
						of the specified parameter. If the *inValue member is NOT NULL, then the audio unit should
						return the name used for the specified value.
						
						On exit, the outName may point to a CFStringRef (which if so must be released by the caller).
						If the parameter has no special name that should be applied to that parameter value,
						then outName will be NULL, and the host should display the parameter value as
						appropriate.
					
	
	@constant		kAudioUnitProperty_ParameterValueFromString
						Scope:				any
						Value Type:			AudioUnitParameterValueFromString
						Access:				read
	
						This property returns the value of a parameter from its string representation. See kAudioUnitProperty_ParameterStringFromValue.
						
	@constant		kAudioUnitProperty_IconLocation	
						Scope:				Global
						Value Type:			CFURLRef
						Access:				Read
						
						A URL that will specify the location of an icon file that can be used when presenting UI for this audio unit.
	
	@constant		kAudioUnitProperty_PresentationLatency
						Scope:				Input/Output
						Value Type:			Float64
						Access:				write

						This property is set by a host to describe to the audio unit the presentation latency of both
						any of its input and/or output audio data. 
						It describes this latency in seconds. A value of zero means either no latency
						or an unknown latency.
						
						This is a write only property because the host is telling the audio unit the latency of both the 
						data it provides it for input and the latency from getting the data from the unit until it is 
						presented.
						
						The property is should be set on each active input and output bus (Scope/Element pair). 
						For example, an audio unit with multiple outputs will have the output data it produces processed 
						by different audio units, etc before it is mixed and presented. Thus, in this case, each output 
						element could have a different presentation latency.
						
						This should not be confused with the Latency property, where the audio unit describes to the host 
						any processing latency it introduces between its input and its output.
						
						For input:
							Describes how long ago the audio given to an audio unit was acquired. For instance, when 
							reading from a file to the first audio unit, then its input presentation latency will be zero. 
							When processing audio input from a  device, then this initial input latency will be the 
							presentation latency of the device itself, the device's safety offset and latency.
							
							The next audio unit's (connected to that first unit) input presentation latency will be the 
							input presentation latency of the first unit, plus the processing latency (as expressed by 
							kAudioUnitProperty_Latency) of the first unit.
							
						For output:
							Describes how long before the output audio of an audio unit is to be presented. For instance, 
							when writing to a file, then the last audio unit's output presentation latency will be zero. 
							When the audio from that audio unit is to be played to an AudioDevice, then that initial 
							presentation latency will be the latency of the device itself - which is the I/O buffer size, 
							and the device's safety offset and latency
							
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
							The previous audio unit's (connected to this last unit) output presentation latency will be that 
=======
							The previous audio unit's (connected to this last unit) output presenation latency will be that 
>>>>>>> graemeg/cpstrnew
=======
							The previous audio unit's (connected to this last unit) output presenation latency will be that 
>>>>>>> graemeg/cpstrnew
=======
							The previous audio unit's (connected to this last unit) output presenation latency will be that 
>>>>>>> graemeg/cpstrnew
=======
							The previous audio unit's (connected to this last unit) output presenation latency will be that 
>>>>>>> origin/cpstrnew
=======
							The previous audio unit's (connected to this last unit) output presenation latency will be that 
>>>>>>> origin/fixes_2.4
							initial presentation latency plus the processing latency (as expressed by 
							kAudioUnitProperty_Latency) of the last unit.
							
						So, for a given audio unit anywhere within a mixing graph, the input and output presentation 
						latencies describe to that unit how long from the moment of generation it will take for its 
						input to arrive, and how long it will take for its output to be presented. 
						
						You can use this property, for example, to provide metering for for an audio unit that 
						is generating output to be presented to the user at a future time.
	
	@constant		kAudioUnitProperty_DependentParameters
						Scope:				any
						Value Type:			array of AUDependentParameter
						Access:				read
		
						This property is used for parameters with the kAudioUnitParameterFlag_IsGlobalMeta 
						or kAudioUnitParameterFlag_IsElementMeta flags set. Hosts applications (and the 
						AudioUnitParameterListener mechanism) can interrogate this property to determine which parameters 
						are dependent on a
						meta-parameter.
						
						For parameters marked with kAudioUnitParameterFlag_IsGlobalMeta, any non-global
						dependent parameters are assumed to be dependent in every element of their scope.
						
						For parameters marked with kAudioUnitParameterFlag_IsElementMeta, then its dependent
						parameters must all be the same scope, and are assumed to apply only within a single element,
						not to other instances of the same parameter in other elements.
	
	@constant		kAudioUnitProperty_AUHostIdentifier
						Scope:				Global
						Value Type:			AUHostVersionIdentifier
						Access:				write

						Determine which application (and which version) an audio unit is being hosted by. 
						This is made more complex through the intervention of audio units such as Kore, that are hosting 
						other audio units (in this case of course, the real host of the audio unit is the hosting unit, 
						not the host application, so the previous mechanism of getting the main bundle ID is no longer 
						correct). 
						
						There are also inconsistencies in the way that bundle identifiers are applied (with apps changing 
						these from version to version), and we'd prefer to see a more consistent identifier used with 
						this property. This is in spirit similar to the string returned by CFBundle API, except that we 
						require this host string be consistent and reliable through different revisions of the host.

						The audio unit is responsible for retaining the hostName string if it needs to use it past the 
						duration of the actual call. The host should set this property as early as possible within the 
						lifetime of the unit in a session.
						
						This API used to take a NumVersion struct. It is redefined to take an AUHostVersionIdentifier struct
						which is binary compatible with the existing usage, but not source compatible.
						
	@constant		kAudioUnitProperty_MIDIOutputCallbackInfo
						Scope:				Global
						Value Type:			CFArrayRef
						Access:				read

						The host will also need to determine how many MIDI output streams the audio unit can generate 
						(and the name for each of these outputs). Each MIDI output is a complete MIDI data stream, 
						such as embodied by a MIDIEndpointRef in CoreMIDI.
						
						To do, the host uses this property and retrieves an array of CFStringRefs:
							- the size of the array is the number of MIDI Outputs the audio unit supports
							- each item in the array is the name for that output at that index
						
						The host should release the array when it is finished with it.
						
						Once the host has determined the audio unit supports this feature, it then instantiates a callback 
						with the unit that the unit will call with MIDI data (see kAudioUnitProperty_MIDIOutputCallback).
	
	@constant		kAudioUnitProperty_MIDIOutputCallback
						Scope:				Global
						Value Type:			AUMIDIOutputCallbackStruct
						Access:				write

						The host sets this property on the audio unit with the callback (and its user data) set 
						appropriately.
						
						Operational Parameters:
							In the render call, just as is the expected usage of the AUHostCallbacks, the audio unit can 
							call the provided callback to provide MIDI data to the host that it will associate with the 
							current AudioUnitRender call in process.

						The audio unit in the callback provides:
							- the user data provided by the host when the callback was established
							- the AudioTimeStamp that was provided to the audio unit for this particular call of 
								AudioUnitRender
							- the output number to associate this MIDI data with
							- a MIDI Packet List containing MIDI data. The time stamp values contained within the 
							MIDIPackets in this list are **sample offsets*** from the AudioTimeStamp provided. 
							This allows MIDI data to be time-stamped with a sample offset that is directly associated 
							with the audio data it is generating in the current call to the AudioUnitRender function

						There is no implied or expected association between the number (or position) of an audio unit's 
						audio or MIDI outputs.

	@constant		kAudioUnitProperty_InputSamplesInOutput
						Scope:			Global
						Value Type:		struct AUInputSamplesInOutputCallbackStruct
						Access: 		read/write
 
						An audio unit calls this callback at the end of its render call. The audio unit supplies the 
						following information:

						outputTime      - The timestamp passed in to the audio unit's render call. This timestamp 
											represents the time of the first output sample.
						inputSample     - The sample number of the first input sample that is present in the output 
											audio.
						numInputSamples - The number of input samples that were used and are present in the output 
											audio.

						This property allows a host application to determine which input samples correspond to a sample 
						in the output buffer. It is useful only for audio units that do time-stretching, such as the 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						AUVarispeed and AUTimePitch units, where the relationship between input and output samples is 
=======
						AUVaripseed and AUTimePitch units, where the relationship between input and output samples is 
>>>>>>> graemeg/cpstrnew
=======
						AUVaripseed and AUTimePitch units, where the relationship between input and output samples is 
>>>>>>> graemeg/cpstrnew
=======
						AUVaripseed and AUTimePitch units, where the relationship between input and output samples is 
>>>>>>> graemeg/cpstrnew
=======
						AUVaripseed and AUTimePitch units, where the relationship between input and output samples is 
>>>>>>> origin/cpstrnew
=======
						AUVaripseed and AUTimePitch units, where the relationship between input and output samples is 
>>>>>>> origin/fixes_2.4
						non-trivial. For these units, the range of input samples that correspond to an output buffer 
						typically differs from the range of input samples that were pulled for that render call. 
						This difference arises because of internal buffering, processing latency, and other factors.
	
	@constant		kAudioUnitProperty_ClassInfoFromDocument
						Scope:			Global
						Value Type:		CFDictionary
						Access: 		read/write
 
						If the audio unit implements this property then it is going to do different actions establishing
						its state from a document rather than from a user preset. Thus, a host app should use this property
						first (instead of kAudioUnitProperty_ClassInfo) when restoring the state of an audio unit when 
						opening a document. If the audio unit returns an error (or doesn't implement this property) then
						the host should use the same preset with the kAudioUnitProperty_ClassInfo. 
 

	@constant		kAudioUnitProperty_ShouldAllocateBuffer
						Scope:			input/output elements (settable per element)
						Value Type:		UInt32
						Access: 		read/write
 
						By default this value is true. This affects the allocations of the buffers for I/O (the mData field
						of the AudioBufferList used with AudioUnitRender, callbacks and connections)
						
						If true, the element will create a buffer for rendering into.
						
						If false, the element will not create a buffer for rendering.
						
						For example, if the audio unit is only ever going to have a connection as its input and never a callback, then
						it should not need to create a buffer (the API contract expects an audio unit to provide a buffer for
						callbacks, but no buffer for connections).
						
						If the audio unit is always going to be pulled for audio with the client providing audio data buffers to 
						the AudioUnitRender call, then it will never need to create an audio buffer on the output side.
						
						So, this property can be used to control the default allocation strategy of an audio unit. If the audio unit
						needs a buffer, but one hasn't been allocated, then an error will be thrown from that call to AudioUnitRender.
						
						This property cannot be set on Initialised audio units as it may end up reallocating memory.


	@constant		kAudioUnitProperty_FrequencyResponse
						Scope:			input/output elements (settable per element)
						Value Type:		AudioUnitFrequencyResponseBin
						Access: 		read
						
						The property provides a way for a user interface view to get points for drawing a graph of the frequency
						response of the AU. 
						
						An array of AudioUnitFrequencyResponseBin are passed in to kAudioUnitProperty_FrequencyResponse
						with the mFrequency field filled in. The array is returned with the mMagnitude fields filled in.
						If fewer than kNumberOfResponseFrequencies are needed, then the first unused bin should be marked with 
						a negative frequency.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	
	@constant		kAudioUnitProperty_ParameterHistoryInfo
						Scope:			Global
						Value Type:		AudioUnitParameterHistoryInfo
						Access: 		read
						
						For parameters which have kAudioUnitParameterFlag_PlotHistory set, getting this property fills out the 
						AudioUnitParameterHistoryInfo struct containing the recommended update rate and history duration.
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
// range (0 -> 999)
	kAudioUnitProperty_ClassInfo = 0;
	kAudioUnitProperty_MakeConnection = 1;
	kAudioUnitProperty_SampleRate = 2;
	kAudioUnitProperty_ParameterList = 3;
	kAudioUnitProperty_ParameterInfo = 4;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitProperty_CPULoad = 6;
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
	kAudioUnitProperty_StreamFormat = 8;
	kAudioUnitProperty_ElementCount = 11;
	kAudioUnitProperty_Latency = 12;
	kAudioUnitProperty_SupportedNumChannels = 13;
	kAudioUnitProperty_MaximumFramesPerSlice = 14;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitProperty_ParameterValueStrings = 16;
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
	kAudioUnitProperty_AudioChannelLayout = 19;
	kAudioUnitProperty_TailTime = 20;
	kAudioUnitProperty_BypassEffect = 21;
	kAudioUnitProperty_LastRenderError = 22;
	kAudioUnitProperty_SetRenderCallback = 23;
	kAudioUnitProperty_FactoryPresets = 24;
	kAudioUnitProperty_RenderQuality = 26;
	kAudioUnitProperty_InPlaceProcessing = 29;
	kAudioUnitProperty_ElementName = 30;
	kAudioUnitProperty_SupportedChannelLayoutTags = 32;
	kAudioUnitProperty_PresentPreset = 36;
	kAudioUnitProperty_ShouldAllocateBuffer = 51;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitProperty_ParameterHistoryInfo = 53;
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

//#if !TARGET_OS_IPHONE;
{$ifc not TARGET_OS_IPHONE}
	kAudioUnitProperty_FastDispatch = 5;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitProperty_SetExternalBuffer = 15;
=======
	kAudioUnitProperty_CPULoad = 6;
	kAudioUnitProperty_SetExternalBuffer = 15;
	kAudioUnitProperty_ParameterValueStrings = 16;
>>>>>>> graemeg/cpstrnew
=======
	kAudioUnitProperty_CPULoad = 6;
	kAudioUnitProperty_SetExternalBuffer = 15;
	kAudioUnitProperty_ParameterValueStrings = 16;
>>>>>>> graemeg/cpstrnew
=======
	kAudioUnitProperty_CPULoad = 6;
	kAudioUnitProperty_SetExternalBuffer = 15;
	kAudioUnitProperty_ParameterValueStrings = 16;
>>>>>>> graemeg/cpstrnew
=======
	kAudioUnitProperty_CPULoad = 6;
	kAudioUnitProperty_SetExternalBuffer = 15;
	kAudioUnitProperty_ParameterValueStrings = 16;
>>>>>>> origin/cpstrnew
=======
	kAudioUnitProperty_CPULoad = 6;
	kAudioUnitProperty_SetExternalBuffer = 15;
	kAudioUnitProperty_ParameterValueStrings = 16;
>>>>>>> origin/fixes_2.4
	kAudioUnitProperty_GetUIComponentList = 18;
	kAudioUnitProperty_ContextName = 25;
	kAudioUnitProperty_HostCallbacks = 27;
	kAudioUnitProperty_CocoaUI = 31;
	kAudioUnitProperty_ParameterIDName = 34;
	kAudioUnitProperty_ParameterClumpName = 35;
	kAudioUnitProperty_ParameterStringFromValue = 33;
	kAudioUnitProperty_OfflineRender = 37;
	kAudioUnitProperty_ParameterValueFromString = 38;
	kAudioUnitProperty_IconLocation = 39;
	kAudioUnitProperty_PresentationLatency = 40;
	kAudioUnitProperty_DependentParameters = 45;
	kAudioUnitProperty_AUHostIdentifier = 46;
	kAudioUnitProperty_MIDIOutputCallbackInfo = 47;
	kAudioUnitProperty_MIDIOutputCallback = 48;
	kAudioUnitProperty_InputSamplesInOutput = 49;
	kAudioUnitProperty_ClassInfoFromDocument = 50;
	kAudioUnitProperty_FrequencyResponse = 52;
//#endif;
{$endc}

{!
    @abstract       Keys contains in an audio unit preset (ClassInfo) dictionary
    @discussion     These strings are used as keys in the AUPreset-"classInfo" dictionary
					The actual keys are CFStrings to use these keys you define the key as:
						static const CFStringRef kMyVersionString = CFSTR(kAUPresetVersionKey);
}
const
	kAUPresetVersionKey = 'version';
const
	kAUPresetTypeKey = 'type';
const
	kAUPresetSubtypeKey = 'subtype';
const
	kAUPresetManufacturerKey = 'manufacturer';
const
	kAUPresetDataKey = 'data';
const
	kAUPresetNameKey = 'name';
const
	kAUPresetRenderQualityKey = 'render-quality';
const
	kAUPresetCPULoadKey = 'cpu-load';
const
	kAUPresetElementNameKey = 'element-name';
const
	kAUPresetExternalFileRefs = 'file-references';

//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}
	// these are keys to use when a preset contains data from other plugin formats
	// vstdata is used to signify VST state from a vst "bank"
const kAUPresetVSTDataKey = 'vstdata';
	// vstpreset is used to signify VST state from a vst "preset"
const kAUPresetVSTPresetKey = 'vstpreset';
const kAUPresetMASDataKey = 'masdata';
//#endif
{$endc}

{!
    @defined        kAUPresetPartKey
    @discussion     This key if present, distinguishes a global preset that is set 
					on the global scope with a part-based preset that is set on the part scope. 
					The value of this key is audio unit defined
}
const
	kAUPresetPartKey = 'part';


{!
    @struct         AudioUnitConnection
    @abstract       This structure contains the information needed to make a connection between a source
					and destination audio unit.
	@discussion		The structure is set on the destination audio unit's input element
    @field          sourceAudioUnit
						The audio unit that is the source for the connection
    @field          sourceOutputNumber
						The source audio unit's output element to be used in the connection
    @field          destInputNumber
						The destination audio unit's input element to be used in the connection						
}
type
	AudioUnitConnection = record
		sourceAudioUnit: AudioUnit;
		sourceOutputNumber: UInt32;
		destInputNumber: UInt32;
	end;
	AudioUnitConnectionPtr = ^AudioUnitConnection;
	
{!
	@struct			AUChannelInfo
	@abstract		Define an audio unit's channel handling capabilities
}
type
	AUChannelInfo = record
		inChannels: SInt16;
		outChannels: SInt16;
	end;
	AUChannelInfoPtr = ^AUChannelInfo;
	
{!
	@struct			AudioUnitExternalBuffer
	@abstract		Allow a host to tell an audio unit to use the provided memory for its input callback
}
type
	AudioUnitExternalBuffer = record
		buffer: BytePtr;
		size: UInt32;
	end;
	AudioUnitExternalBufferPtr = ^AudioUnitExternalBuffer;
	
{!
	@struct			AURenderCallbackStruct
	@abstract		Used by a host when registering a callback with the audio unit to provide input
}
type
	AURenderCallbackStruct = record
		inputProc: AURenderCallback;
		inputProcRefCon: UnivPtr;
	end;
	AURenderCallbackStructPtr = ^AURenderCallbackStruct;
	
{!
	@struct			AUPreset
	@abstract		Used to publish and set factory presets on an audio unit
	@field			presetNumber
						If < 0, then preset is a user preset
						If >= 0, then this field is used to select the factory preset
	@field			presetName
						If a factory preset, the name of the specified factory preset
}
type
	AUPreset = record
		presetNumber: SInt32;
		presetName: CFStringRef;
	end;
	AUPresetPtr = ^AUPreset;
	
{!
	@enum			RenderQuality
	@abstract		Used to get/set a render quality setting on an audio unit
	@discussion		Typically, this property is used to trade-off between CPU usage, latency
					and the quality of the audio unit's processing/output.
}
const
	kRenderQuality_Max = $7F;
	kRenderQuality_High = $60;
	kRenderQuality_Medium = $40;
	kRenderQuality_Low = $20;
	kRenderQuality_Min = 0;


//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}
{!
	@enum			kNumberOfResponseFrequencies
	@abstract		The maximum number of frequency response bins for kAudioUnitProperty_FrequencyResponse.
	@discussion		An array of AudioUnitFrequencyResponseBin are passed in to kAudioUnitProperty_FrequencyResponse
					with the mFrequency field filled in. The array is returned with the mMagnitude fields filled in.
					If fewer than kNumberOfResponseFrequencies are needed, then the first unused bin should be marked with 
					a negative frequency.
}
const
	kNumberOfResponseFrequencies = 1024;

{!
	@struct			AudioUnitFrequencyResponseBin
	@abstract		Structure used to get the magnitude of the frequency response at a particular frequency via kAudioUnitProperty_FrequencyResponse. 
	@discussion		An array of AudioUnitFrequencyResponseBin are passed in to kAudioUnitProperty_FrequencyResponse
					with the mFrequency field filled in. The array is returned with the mMagnitude fields filled in.
					If fewer than kNumberOfResponseFrequencies are needed, then the first unused bin should be marked with 
					a negative frequency.
}
type
	AudioUnitFrequencyResponseBin = record
		mFrequency: Float64;
		mMagnitude: Float64;
	end;
	AudioUnitFrequencyResponseBinPtr = ^AudioUnitFrequencyResponseBin;

{!
	@typedef		HostCallback_GetBeatAndTempo
	@abstract		Retrieve information about the current beat and/or tempo
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion		If the host app has set this callback, then the audio unit can use this to get the current beat and tempo as they relate to the first sample in the render buffer. The audio unit can call this callback only from within the audio unit render call (otherwise the host is unable to provide information accurately to the audio unit as the information obtained is relate to the current AudioUnitRender call). If the host cannot provide the requested information, it will return kAudioUnitErr_CannotDoInCurrentContext.
	
			The AudioUnit can provide NULL for any of the requested parameters (except for inHostUserData) if it is not interested in that particular piece of information

	@param			inHostUserData			Must be provided by the audio unit when it makes this call. It is the client data provided by the host when it set the HostCallbacks property
	@param			outCurrentBeat			The current beat, where 0 is the first beat. Tempo is defined as the number of whole-number (integer) beat values (as indicated by the outCurrentBeat field) per minute.
	@param			outCurrentTempo			The current tempo
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
type
	HostCallback_GetBeatAndTempo = function( inHostUserData: UnivPtr; var outCurrentBeat: Float64; var outCurrentTempo: Float64 ): OSStatus;

{!
	@typedef		HostCallback_GetMusicalTimeLocation
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@abstract		Retrieve information about the musical time state of the host
	@discussion		If the host app has set this callback, then the audio unit can use this to obtain information about the state of musical time in the host. The audio unit can call this callback only from within the audio unit render call (otherwise the host is unable to provide information accurately to the audio unit as the information obtained is relate to the current AudioUnitRender call). If the host cannot provide the requested information, it will return kAudioUnitErr_CannotDoInCurrentContext.
	
			The AudioUnit can provide NULL for any of the requested parameters (except for inHostUserData) if it is not interested in that particular piece of information

	@param			inHostUserData					Must be provided by the audio unit when it makes this call. It is the client data provided by the host when it set the HostCallbacks property
	@param			outDeltaSampleOffsetToNextBeat	The number of samples until the next whole beat from the start sample of the current rendering buffer
	@param			outTimeSig_Numerator			The Numerator of the current time signature
	@param			outTimeSig_Denominator			The Denominator of the current time signature (4 is a quarter note, etc)
	@param			outCurrentMeasureDownBeat		The beat that corresponds to the downbeat (first beat) of the current measure that is being rendered

=======
	@abstract		Retrieve information about the general musical time state of the host
>>>>>>> graemeg/cpstrnew
=======
	@abstract		Retrieve information about the general musical time state of the host
>>>>>>> graemeg/cpstrnew
=======
	@abstract		Retrieve information about the general musical time state of the host
>>>>>>> graemeg/cpstrnew
=======
	@abstract		Retrieve information about the general musical time state of the host
>>>>>>> origin/cpstrnew
=======
	@abstract		Retrieve information about the general musical time state of the host
>>>>>>> origin/fixes_2.4
}
type
	HostCallback_GetMusicalTimeLocation = function( inHostUserData: UnivPtr; var outDeltaSampleOffsetToNextBeat: UInt32; var outTimeSig_Numerator: Float32; var outTimeSig_Denominator: UInt32; var outCurrentMeasureDownBeat: Float64 ): OSStatus;

{!
	@typedef		HostCallback_GetTransportState
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@abstract		Retrieve information about the time line's (or transport) state of the host. 
	@discussion		If the host app has set this callback, then the audio unit can use this to obtain information about the transport state of the host's time line. The audio unit can call this callback only from within the audio unit render call (otherwise the host is unable to provide information accurately to the audio unit as the information obtained is relate to the current AudioUnitRender call. If the host cannot provide the requested information, it will return kAudioUnitErr_CannotDoInCurrentContext.
	
			The AudioUnit can provide NULL for any of the requested parameters (except for inHostUserData) if it is not interested in that particular piece of information
	
	@param			inHostUserData					Must be provided by the audio unit when it makes this call. It is the client data provided by the host when it set the HostCallbacks property
	@param			outIsPlaying					Returns true if the host's transport is currently playing, false if stopped
	@param			outTransportStateChanged		Returns true if there was a change to the state of, or discontinuities in, the host's transport (generally since the callback was last called). Can indicate such state changes as start/top, time moves (jump from one time line to another).
	@param			outCurrentSampleInTimeLine		Returns the current sample count in the time line of the host's transport time.  
	@param			outIsCycling					Returns true if the host's transport is currently cycling or looping
	@param			outCycleStartBeat				If cycling is true, the start beat of the cycle or loop point in the host's transport
	@param			outCycleEndBeat					If cycling is true, the end beat of the cycle or loop point in the host's transport
	
=======
	@abstract		Retrieve information about the time line's (or transport) state of the host
>>>>>>> graemeg/cpstrnew
=======
	@abstract		Retrieve information about the time line's (or transport) state of the host
>>>>>>> graemeg/cpstrnew
=======
	@abstract		Retrieve information about the time line's (or transport) state of the host
>>>>>>> graemeg/cpstrnew
=======
	@abstract		Retrieve information about the time line's (or transport) state of the host
>>>>>>> origin/cpstrnew
=======
	@abstract		Retrieve information about the time line's (or transport) state of the host
>>>>>>> origin/fixes_2.4
}
type
	HostCallback_GetTransportState = function( inHostUserData: UnivPtr; var outIsPlaying: Boolean; var outTransportStateChanged: Boolean; var outCurrentSampleInTimeLine: Float64; var outIsCycling: Boolean; var outCycleStartBeat: Float64; var outCycleEndBeat: Float64 ): OSStatus;

{!
	@struct			HostCallbackInfo
	@abstract		Contains the various callbacks (or NULL) for an audio unit to call
}
type
	HostCallbackInfo = record
		hostUserData: UnivPtr;
		beatAndTempoProc: HostCallback_GetBeatAndTempo;
		musicalTimeLocationProc: HostCallback_GetMusicalTimeLocation;
		transportStateProc: HostCallback_GetTransportState;
	end;
	HostCallbackInfoPtr = ^HostCallbackInfo;
	

{!
	@struct			AudioUnitCocoaViewInfo
	@abstract		The name and how many, NSView objects an audio unit publishes as a custom Cocoa view.
	@field			mCocoaAUViewBundleLocation
						Contains the location of the bundle which the host app can then use to locate the bundle
	@field			mCocoaAUViewClass
						Contains the names of the classes that implements the required protocol for an AUView
}
type
	AudioUnitCocoaViewInfo = record
		mCocoaAUViewBundleLocation: CFURLRef;
		mCocoaAUViewClass: array[0..0] of CFStringRef;
	end;
	AudioUnitCocoaViewInfoPtr = ^AudioUnitCocoaViewInfo;
	
{!
	@struct			AUDependentParameter
	@abstract		Used to represent a dependent parameter that can change as a result of its parent meta-parameter
					changing
}
type
	AUDependentParameter = record
		mScope: AudioUnitScope;
		mParameterID: AudioUnitParameterID;
	end;
	AUDependentParameterPtr = ^AUDependentParameter;
	
{!
	@struct			AUHostVersionIdentifier
	@abstract		Used to describe the name and version of the audio unit's host
}
type
	AUHostVersionIdentifier = record
		hostName: CFStringRef;	
		hostVersion: UInt32;
	end;
	AUHostVersionIdentifierPtr = ^AUHostVersionIdentifier;
	
{
	@typedef		AUMIDIOutputCallback
	@abstract		A callback used by an audio unit to provide MIDI data to a host application
}
type
	AUMIDIOutputCallback = function( userData: UnivPtr; const (*var*) timeStamp: AudioTimeStamp; midiOutNum: UInt32; {const} pktlist: MIDIPacketListPtr ): OSStatus;

{!
	@struct			AUMIDIOutputCallbackStruct
	@abstract		Set by host application to provide the callback and user data for an audio 
					unit that provides MIDI output
}
type
	AUMIDIOutputCallbackStruct = record
		midiOutputCallback: AUMIDIOutputCallback;
		userData: UnivPtr;
	end;
	AUMIDIOutputCallbackStructPtr = ^AUMIDIOutputCallbackStruct;
	
{!
	@struct			AUInputSamplesInOutputCallbackStruct
	@abstract		Used by a host when registering a callback with an audio unit, to provide 
					input-to-output samples mapping
}
type
	AUInputSamplesInOutputCallbackStruct = record
		inputToOutputCallback: AUInputSamplesInOutputCallback;
		userData: UnivPtr;
	end;
	AUInputSamplesInOutputCallbackStructPtr = ^AUInputSamplesInOutputCallbackStruct;
	
{$endc} { not TARGET_OS_IPHONE }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{!
	@struct			AudioUnitParameterHistoryInfo
	@abstract		This structure contains the suggested update rate and history duration for parameters which have the kAudioUnitParameterFlag_PlotHistory flag set.
					The structure is filled out by getting kAudioUnitProperty_ParameterHistoryInfo.
	@field			updatesPerSecond
						This is the number of times per second that it is suggested that the host get the value of this parameter.
	@field			historyDurationInSeconds
						This is the duration in seconds of history that should be plotted.
}
type
	AudioUnitParameterHistoryInfo = record
		updatesPerSecond: Float32;
		historyDurationInSeconds: Float32;
	end;
	AudioUnitParameterHistoryInfoPtr = ^AudioUnitParameterHistoryInfo;

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
//#pragma mark - Parameter Definitions

// assume kAudioUnitParameterUnit_Generic if not found in this enum
{!
	@enum			AudioUnitParameterUnit
	@constant		kAudioUnitParameterUnit_Generic
						untyped value generally between 0.0 and 1.0
	@constant		kAudioUnitParameterUnit_Indexed
						takes an integer value (good for menu selections)
	@constant		kAudioUnitParameterUnit_Boolean
						0.0 means FALSE, non-zero means TRUE
	@constant		kAudioUnitParameterUnit_Percent
						usually from 0 -> 100, sometimes -50 -> +50
	@constant		kAudioUnitParameterUnit_Seconds
						absolute or relative time
	@constant		kAudioUnitParameterUnit_SampleFrames
						one sample frame equals (1.0/sampleRate) seconds
	@constant		kAudioUnitParameterUnit_Phase
						-180 to 180 degrees
	@constant		kAudioUnitParameterUnit_Rate
						rate multiplier, for playback speed, etc. (e.g. 2.0 == twice as fast)
	@constant		kAudioUnitParameterUnit_Hertz
						absolute frequency/pitch in cycles/second
	@constant		kAudioUnitParameterUnit_Cents
						unit of relative pitch
	@constant		kAudioUnitParameterUnit_RelativeSemiTones
						useful for coarse detuning
	@constant		kAudioUnitParameterUnit_MIDINoteNumber
						absolute pitch as defined in the MIDI spec (exact freq may depend on tuning table)
	@constant		kAudioUnitParameterUnit_MIDIController
						a generic MIDI controller value from 0 -> 127
	@constant		kAudioUnitParameterUnit_Decibels
						logarithmic relative gain
	@constant		kAudioUnitParameterUnit_LinearGain
						linear relative gain
	@constant		kAudioUnitParameterUnit_Degrees
						-180 to 180 degrees, similar to phase but more general (good for 3D coord system)
	@constant		kAudioUnitParameterUnit_EqualPowerCrossfade
						0 -> 100, crossfade mix two sources according to sqrt(x) and sqrt(1.0 - x)
	@constant		kAudioUnitParameterUnit_MixerFaderCurve1
						0.0 -> 1.0, pow(x, 3.0) -> linear gain to simulate a reasonable mixer channel fader response
	@constant		kAudioUnitParameterUnit_Pan
						standard left to right mixer pan
	@constant		kAudioUnitParameterUnit_Meters
						distance measured in meters
	@constant		kAudioUnitParameterUnit_AbsoluteCents
						absolute frequency measurement : 
						if f is freq in hertz then absoluteCents = 1200 * log2(f / 440) + 6900
	@constant		kAudioUnitParameterUnit_Octaves
						octaves in relative pitch where a value of 1 is equal to 1200 cents
	@constant		kAudioUnitParameterUnit_BPM
						beats per minute, ie tempo
    @constant		kAudioUnitParameterUnit_Beats
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						time relative to tempo, i.e., 1.0 at 120 BPM would equal 1/2 a second
=======
						time relative to tempo, ie. 1.0 at 120 BPM would equal 1/2 a second
>>>>>>> graemeg/cpstrnew
=======
						time relative to tempo, ie. 1.0 at 120 BPM would equal 1/2 a second
>>>>>>> graemeg/cpstrnew
=======
						time relative to tempo, ie. 1.0 at 120 BPM would equal 1/2 a second
>>>>>>> graemeg/cpstrnew
=======
						time relative to tempo, ie. 1.0 at 120 BPM would equal 1/2 a second
>>>>>>> origin/cpstrnew
=======
						time relative to tempo, ie. 1.0 at 120 BPM would equal 1/2 a second
>>>>>>> origin/fixes_2.4
	@constant		kAudioUnitParameterUnit_Milliseconds
						parameter is expressed in milliseconds
	@constant		kAudioUnitParameterUnit_Ratio
						for compression, expansion ratio, etc.
	@constant		kAudioUnitParameterUnit_CustomUnit
						this is the parameter unit type for parameters that present a custom unit name
}
const
	kAudioUnitParameterUnit_Generic = 0;
	kAudioUnitParameterUnit_Indexed = 1;
	kAudioUnitParameterUnit_Boolean = 2;
	kAudioUnitParameterUnit_Percent = 3;
	kAudioUnitParameterUnit_Seconds = 4;
	kAudioUnitParameterUnit_SampleFrames = 5;
	kAudioUnitParameterUnit_Phase = 6;
	kAudioUnitParameterUnit_Rate = 7;
	kAudioUnitParameterUnit_Hertz = 8;
	kAudioUnitParameterUnit_Cents = 9;
	kAudioUnitParameterUnit_RelativeSemiTones = 10;
	kAudioUnitParameterUnit_MIDINoteNumber = 11;
	kAudioUnitParameterUnit_MIDIController = 12;
	kAudioUnitParameterUnit_Decibels = 13;
	kAudioUnitParameterUnit_LinearGain = 14;
	kAudioUnitParameterUnit_Degrees = 15;
	kAudioUnitParameterUnit_EqualPowerCrossfade = 16;
	kAudioUnitParameterUnit_MixerFaderCurve1 = 17;
	kAudioUnitParameterUnit_Pan = 18;
	kAudioUnitParameterUnit_Meters = 19;
	kAudioUnitParameterUnit_AbsoluteCents = 20;
	kAudioUnitParameterUnit_Octaves = 21;
	kAudioUnitParameterUnit_BPM = 22;
	kAudioUnitParameterUnit_Beats = 23;
	kAudioUnitParameterUnit_Milliseconds = 24;
	kAudioUnitParameterUnit_Ratio = 25;
	kAudioUnitParameterUnit_CustomUnit = 26;
{!
	@typedef		AudioUnitParameterUnit
}
type
	AudioUnitParameterUnit = UInt32;

{!
	@struct			AudioUnitParameterInfo
	@field			name
						UNUSED - set to zero - UTF8 encoded C string (originally). 
	@field			unitName
						only valid if kAudioUnitParameterUnit_CustomUnit is set. If kAudioUnitParameterUnit_CustomUnit
						is set, this field must contain a valid CFString.
	@field			clumpID
						only valid if kAudioUnitParameterFlag_HasClump
	@field			cfNameString
						only valid if kAudioUnitParameterFlag_HasCFNameString
	@field			unit				
						if the "unit" field contains a value not in the enum above, then assume 
						kAudioUnitParameterUnit_Generic
	@field			minValue
	@field			maxValue
	@field			defaultValue
	@field			flags
						Due to some vagaries about the ways in which Parameter's CFNames have been described, it was
						necessary to add a flag: kAudioUnitParameterFlag_CFNameRelease
						In normal usage a parameter name is essentially a static object, but sometimes an audio unit will 
						generate parameter names dynamically.. As these are expected to be CFStrings, in that case
						the host should release those names when it is finished with them, but there was no way
						to communicate this distinction in behavior.
						Thus, if an audio unit will (or could) generate a name dynamically, it should set this flag in 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						the parameter's info. The host should check for this flag, and if present, release the parameter
=======
						the paramter's info.. The host should check for this flag, and if present, release the parameter
>>>>>>> graemeg/cpstrnew
=======
						the paramter's info.. The host should check for this flag, and if present, release the parameter
>>>>>>> graemeg/cpstrnew
=======
						the paramter's info.. The host should check for this flag, and if present, release the parameter
>>>>>>> graemeg/cpstrnew
=======
						the paramter's info.. The host should check for this flag, and if present, release the parameter
>>>>>>> origin/cpstrnew
=======
						the paramter's info.. The host should check for this flag, and if present, release the parameter
>>>>>>> origin/fixes_2.4
						name when it is finished with it.
}
type
	AudioUnitParameterInfo = record
		name : packed array [0..51] of char;
		unitName: CFStringRef;
		clumpID: UInt32;
		cfNameString: CFStringRef;
		unt: AudioUnitParameterUnit;						
		minValue: AudioUnitParameterValue;			
		maxValue: AudioUnitParameterValue;			
		defaultValue: AudioUnitParameterValue;		
		flags: UInt32;
	end;
	AudioUnitParameterInfoPtr = ^AudioUnitParameterInfo;
	
{!
	@enum			Audio Unit Parameter Flags
	@discussion		Bit positions 18, 17, and 16 are set aside for display scales. Bit 19 is reserved.
	@constant		kAudioUnitParameterFlag_CFNameRelease
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@constant		kAudioUnitParameterFlag_PlotHistory
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
	@constant		kAudioUnitParameterFlag_MeterReadOnly
	@constant		kAudioUnitParameterFlag_DisplayMask
	@constant		kAudioUnitParameterFlag_DisplaySquareRoot
	@constant		kAudioUnitParameterFlag_DisplaySquared
	@constant		kAudioUnitParameterFlag_DisplayCubed
	@constant		kAudioUnitParameterFlag_DisplayCubeRoot
	@constant		kAudioUnitParameterFlag_DisplayExponential
	@constant		kAudioUnitParameterFlag_HasClump
	@constant		kAudioUnitParameterFlag_ValuesHaveStrings
	@constant		kAudioUnitParameterFlag_DisplayLogarithmic		
	@constant		kAudioUnitParameterFlag_IsHighResolution
	@constant		kAudioUnitParameterFlag_NonRealTime
	@constant		kAudioUnitParameterFlag_CanRamp
	@constant		kAudioUnitParameterFlag_ExpertMode
	@constant		kAudioUnitParameterFlag_HasCFNameString
	@constant		kAudioUnitParameterFlag_IsGlobalMeta
	@constant		kAudioUnitParameterFlag_IsElementMeta
	@constant		kAudioUnitParameterFlag_IsReadable
	@constant		kAudioUnitParameterFlag_IsWritable
}
const
	kAudioUnitParameterFlag_CFNameRelease = 1 shl 4;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAudioUnitParameterFlag_PlotHistory = 1 shl 14;
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
	kAudioUnitParameterFlag_MeterReadOnly = 1 shl 15;
	
	// bit positions 18,17,16 are set aside for display scales. bit 19 is reserved.
	kAudioUnitParameterFlag_DisplayMask = (7 shl 16) or (1 shl 22);
	kAudioUnitParameterFlag_DisplaySquareRoot = 1 shl 16;
	kAudioUnitParameterFlag_DisplaySquared = 2 shl 16;
	kAudioUnitParameterFlag_DisplayCubed = 3 shl 16;
	kAudioUnitParameterFlag_DisplayCubeRoot = 4 shl 16;
	kAudioUnitParameterFlag_DisplayExponential = 5 shl 16;
	kAudioUnitParameterFlag_HasClump = 1 shl 20;
	kAudioUnitParameterFlag_ValuesHaveStrings = 1 shl 21;
	kAudioUnitParameterFlag_DisplayLogarithmic = 1 shl 22;
	kAudioUnitParameterFlag_IsHighResolution = 1 shl 23;
	kAudioUnitParameterFlag_NonRealTime = 1 shl 24;
	kAudioUnitParameterFlag_CanRamp = 1 shl 25;
	kAudioUnitParameterFlag_ExpertMode = 1 shl 26;
	kAudioUnitParameterFlag_HasCFNameString = 1 shl 27;
	kAudioUnitParameterFlag_IsGlobalMeta = 1 shl 28;
	kAudioUnitParameterFlag_IsElementMeta = 1 shl 29;
	kAudioUnitParameterFlag_IsReadable = 1 shl 30;
	kAudioUnitParameterFlag_IsWritable = 1 shl 31;

{!
	@enum			Audio Unit Clump ID
	@discussion		Audio unit developers should not use a clump ID of 0. This value is reserved for system use.
}
const
	kAudioUnitClumpID_System = 0;
function GetAudioUnitParameterDisplayType(flags : UInt32) : UInt32; inline;
function AudioUnitDisplayTypeIsLogarithmic(flags : UInt32) : Boolean; inline;
function AudioUnitDisplayTypeIsSquareRoot(flags : UInt32) : Boolean; inline;
function AudioUnitDisplayTypeIsSquared(flags : UInt32) : Boolean; inline;
function AudioUnitDisplayTypeIsCubed(flags : UInt32) : Boolean; inline;
function AudioUnitDisplayTypeIsCubeRoot(flags : UInt32) : Boolean; inline;
function AudioUnitDisplayTypeIsExponential(flags : UInt32) : Boolean; inline;
procedure SetAudioUnitParameterDisplayType(var flags : UInt32; displayType : UInt32); inline;


//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}
{
	The following properties are used with display names and are only available
	in the full desktop environment
}

{!
	@enum			Audio Unit Parameter Full Name
	@discussion		Used with the AudioUnitParameterNameInfo.inDesiredLength field to indicate the full name
					of the requested parameter.
}
const
	kAudioUnitParameterName_Full = -1;

{!
	@struct			AudioUnitParameterNameInfo
	@abstract		Used to provide shorter names for a specified parameter
}
type
	AudioUnitParameterNameInfo = record
		inID: AudioUnitParameterID;
		inDesiredLength: SInt32;
		outName: CFStringRef;
	end;
	AudioUnitParameterNameInfoPtr = ^AudioUnitParameterNameInfo;
	
{!
	@struct			AudioUnitParameterStringFromValue
	@abstract		Provide a string representation of a parameter's value
}
type
	AudioUnitParameterStringFromValue = record
		inParamID: AudioUnitParameterID;
		inValue: { const } AudioUnitParameterValuePtr;	
		outString: CFStringRef;
	end;
	AudioUnitParameterStringFromValuePtr = ^AudioUnitParameterStringFromValue;
	
{!
	@struct			AudioUnitParameterValueFromString
	@abstract		Provide the parameter's value for a given string representation of it
}
type
	AudioUnitParameterValueFromString = record
		inParamID: AudioUnitParameterID;
		inString: CFStringRef;
		outValue: AudioUnitParameterValue;
	end;
	AudioUnitParameterValueFromStringPtr = ^AudioUnitParameterValueFromString;
	
{$endc} { not TARGET_OS_IPHONE}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
//	These strings are used as keys to the dictionary of configuration info returned by
//	AudioComponentGetConfiguationInfo(). Informaton about them is presented inline with the
//	declaration.

{!
	@define		kAudioUnitConfigurationInfo_HasCustomView
	@discussion	This is a boolean value that indicates whether or not the AU has a custom view
}
const
	kAudioUnitConfigurationInfo_HasCustomView = 'HasCustomView';

{!
	@define		kAudioUnitConfigurationInfo_ChannelConfigurations
	@discussion	This is an array of pairs of values where each item in the array is an array of two
				numbers and is the equivalent of an AUChannelInfo. If the AudioUnit is an effect and
				it doesn't implement kAudioUnitProperty_SupportedNumChannels, the array will contain
				only the single entry, ( -1, -1). If the AudioUnit is an instrument or a generator
				and doesn't implement kAudioUnitProperty_SupportedNumChannels, the array will be
				empty and means that the AU's initial state is all that is supported.
}
const
	kAudioUnitConfigurationInfo_ChannelConfigurations = 'ChannelConfigurations';

{!
	@define		kAudioUnitConfigurationInfo_InitialInputs
	@discussion	An array of numbers whose size is equal to the number of input buses posessed by the
				AU. Each number in the array represents the number of channels for the corresponding
				bus.
}
const
	kAudioUnitConfigurationInfo_InitialInputs = 'InitialInputs';

{!
	@define		kAudioUnitConfigurationInfo_InitialOutputs
	@discussion	An array of numbers whose size is equal to the number of output buses posessed by
				the AU. Each number in the array represents the number of channels for the
				corresponding bus.
}
const
	kAudioUnitConfigurationInfo_InitialOutputs = 'InitialOutputs';

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
//#pragma mark - Output Unit
{!
    @enum           Output Unit Properties
    @abstract       The collection of properties for output units
	@constant		kAudioOutputUnitProperty_IsRunning
	@discussion			Scope:
						Value Type:
						Access:
}
const
// range  (2000 -> 2999)
	kAudioOutputUnitProperty_IsRunning = 2001;

//#pragma mark -
//#pragma mark Desktop Availability

//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}

//=====================================================================================================================
//#pragma mark - Music Effects and Instruments
{!
    @enum           Music Effect and Instrument Unit (MusicDevice) Properties
    @abstract       The collection of Music Effects and Instrument Unit Property IDs
	@discussion
		These properties are used to:
			Describe a current set of mappings between MIDI messages and Parameter value setting
			Create a mapping between a parameter and a MIDI message through either:
				- explicitly adding (or removing) the mapping
				- telling the audio unit to hot-map the next MIDI message to a specified Parameter
			The same MIDI Message can map to one or more parameters
			
		These properties normally apply only to the two types of audio units that implement 
		the MIDI API, instrument units ('aumu') and music effects ('aumf').

		These properties are used in the Global scope. The scope and element members of the structure describe
		the scope and element of the parameter. In all usages, mScope, mElement and mParameterID must be
		correctly specified.

		
		* The AUParameterMIDIMapping Structure

		Command				mStatus			mData1			
		Note Off			0x8n			Note Num		
		Note On				0x9n			Note Num		
		Key Pressure		0xAn			Note Num		
		Control Change		0xBn			ControllerID	
		Patch Change		0xCn			Patch Num		
		Channel Pressure	DxDn			0 (Unused)		
		Pitch Bend			0xEn			0 (Unused)		
		
		(where n is 0-0xF to correspond to MIDI channels 1-16)
		
		Details:

		In general MIDI Commands can be mapped to either a specific channel as specified in the mStatus bit.
		If the kAUParameterMIDIMapping_AnyChannelFlag bit is set mStatus is a MIDI channel message, then the 
		MIDI channel number in the status byte is ignored; the mapping is from the specified MIDI message on ANY channel.
		
		For note commands (note on, note off, key pressure), the MIDI message can trigger either with just a specific
		note number, or any note number if the kAUParameterMIDIMapping_AnyNoteFlag bit is set. In these instances, the
		note number is used as the trigger value (for instance, a note message could be used to set the 
		cut off frequency of a filter).
		
		When the parameter mapping list changes through addition/replace, removal, the implementation should
		fire a notification on the kAudioUnitProperty_AllParameterMIDIMappings property. The host can then
		retrieve the full set of mappings for the audio unit.
		
		When a hot mapping is made, the notification should just be delivered for the HotMap property. The host can
		retrieve the last current hot mapping made through getting the value of that property.
	
	@constant		kAudioUnitProperty_AllParameterMIDIMappings
	@discussion			Scope:				any
						Value Type:			array of AUParameterMIDIMapping
						Access:				read/write

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						This property allows setting and retrieving the current mapping state between 
=======
						This property allows setting and retreiving the current mapping state between 
>>>>>>> graemeg/cpstrnew
=======
						This property allows setting and retreiving the current mapping state between 
>>>>>>> graemeg/cpstrnew
=======
						This property allows setting and retreiving the current mapping state between 
>>>>>>> graemeg/cpstrnew
=======
						This property allows setting and retreiving the current mapping state between 
>>>>>>> origin/cpstrnew
=======
						This property allows setting and retreiving the current mapping state between 
>>>>>>> origin/fixes_2.4
						(some/many/all of) an audio unit's parameters and MIDI messages. When set, it should replace 
						any previous mapped settings the audio unit had.
					
						If this property is implemented by a non-MIDI capable audio unit (such as an 'aufx' type), 
						then the property is read only and should recommend a suggested set of mappings for the host 
						to perform. In this case, it is the host's responsibility to map MIDI message to the audio 
						unit parameters.
						
						This property's size varies depending on the number of mappings currently in effect. A host 
						application should always get the size of this property before retrieving it. The audio
						unit should return an error if the host doesn't provide enough space to return all of the 
						current mappings.
	
	@constant		kAudioUnitProperty_AddParameterMIDIMapping
	@discussion			Scope:				any
						Value Type:			array of AUParameterMIDIMapping
						Access:				write
						
						Use this property to add parameter-to-MIDI mappings to an audio unit's existing set of 
						mappings. There can be only one mapping per parameter. When you set a mapping for a parameter, 
						it replaces the previous mapping.
	
	@constant		kAudioUnitProperty_RemoveParameterMIDIMapping
	@discussion			Scope:				any
						Value Type:			array of AUParameterMIDIMapping
						Access:				write

						Use this property to remove mappings from an audio unit. If a mapping is specified that
						does not currently exist in an audio unit, then the audio unit should ignore the request. 
						The Scope/Element/ParameterID is used to find the mapping to remove.

	@constant		kAudioUnitProperty_HotMapParameterMIDIMapping
	@discussion			Scope:				any
						Value Type:			AUParameterMIDIMapping
						Access:				read/write

						This property can be used in two ways, determined by the value supplied by the host 
						application.
						
						(1) If a mapping structure is provided, then that structure provides all the information 
						that the audio unit should use to map the parameter, except for the MIDI message. The audio 
						unit should then listen for the next MIDI message and associate that MIDI message with the 
						supplied parameter mapping. When this MIDI message is received and the mapping made, the 
						audio unit should also issue a notification on this property to indicate to the host that 
						the mapping has been made. The host can then retrieve the mapping that was made by getting the 
						value of this property.
						
						To avoid possible confusion, it is recommended that once the host has retrieved this mapping 
						(if it is presenting a user interface to describe the mappings, for example), that the host
						should then clear the mapping state, as described in (2).
						
						The only time this property will return a valid value is when an audio unit has implemented the 
						requested mapping. If the audio unit's mapping state has been cleared (or if it has not been 
						asked to make a mapping), then the audio unit should return a kAudioUnitErr_InvalidPropertyValue 
						error when the host tries to read this property's value.
						
						(2) If the value passed in this property is NULL, and if the audio unit had a parameter that 
						it was in the process of mapping, the audio unit should disregard the parameter mapping request 
						and discard the partially mapped structure. If the value is NULL and the audio unit is not in 
						the process of mapping, the audio unit can just ignore the request.
						
						At all times, the _AllMappings property will completely describe the current known state of an 
						audio unit's mappings of MIDI messages to parameters.		
}
const
	kAudioUnitProperty_AllParameterMIDIMappings = 41;
	kAudioUnitProperty_AddParameterMIDIMapping = 42;
	kAudioUnitProperty_RemoveParameterMIDIMapping = 43;
	kAudioUnitProperty_HotMapParameterMIDIMapping = 44;
	
{!
	@enum			ParameterMIDIMappings
	@abstract		General defined values to customize the behavior of parameter-to-MIDI mappings

	@constant		kAUParameterMIDIMapping_AnyChannelFlag
	@discussion			If this flag is set and the value of the mStatus field is a MIDI channel message, then 
						the MIDI channel number in the status byte is ignored; the mapping is from the specified 
						MIDI message on any channel.
	
	@constant		kAUParameterMIDIMapping_AnyNoteFlag
	@discussion			If this flag is set and the value of the mStatus field is a Note On, Note Off, or 
						Polyphonic Pressure message, the message's note number is ignored. The mapping is from 
						any note number.
	
	@constant		kAUParameterMIDIMapping_SubRange
	@discussion			Set this flag if the MIDI control should map only to a sub-range of the parameter's value.
						Then specify that range in the mSubRangeMin and mSubRangeMax member fields.
						
	@constant		kAUParameterMIDIMapping_Toggle
	@discussion			Intended for Boolean typed parameters. When this property is set, it means that the 
						parameter's value should be toggled when the mapped MIDI message is received. For example,
						if the parameter's value is currently TRUE, when the mapped MIDI message is received
						the value changes to FALSE. 

	@constant		kAUParameterMIDIMapping_Bipolar
	@discussion			This property is useful when mapping a parameter to a MIDI Controller. When set, it 
						indicates that the parameter can assume only two values: on or off. For this reason, a 
						parameter associated with this property is typically Boolean. For example, if this 
						property is set for a parameter mapped to a sustain pedal MIDI controller, controller 
						values from 0 to 64 result in the parameter switched to its "off" state; controller
						values from 65 to 127 result in the parameter switched to its "on" state.
						
						This property works in connection with the kAUParameterMIDIMapping_Bipolar_On property.
						The value of the kAUParameterMIDIMapping_Bipolar_On property 

	@constant		kAUParameterMIDIMapping_Bipolar_On
	@discussion			Determines whether the  "on" state of a parameter is mapped to the "on" or "off" state 
						of the associated MIDI controller. Only valid  if the  kAUParameterMIDIMapping_Bipolar 
						property is set.
}
const
	kAUParameterMIDIMapping_AnyChannelFlag = 1 shl 0;
	kAUParameterMIDIMapping_AnyNoteFlag = 1 shl 1;
	kAUParameterMIDIMapping_SubRange = 1 shl 2;
	kAUParameterMIDIMapping_Toggle = 1 shl 3;
	kAUParameterMIDIMapping_Bipolar = 1 shl 4;
	kAUParameterMIDIMapping_Bipolar_On = 1 shl 5;

{!
	@struct			AUParameterMIDIMapping
	@abstract		Represents a mapping between a MIDI message and an audio unit's parameter.
	@discussion		The reserved fields in this structure are for future use. In the current implementation, 
					they help align the structure to 64 bit size. Do not use the names of these fields in a 
					host application. They are subject to change.
}
type
	AUParameterMIDIMapping = record
		mScope: AudioUnitScope;
		mElement: AudioUnitElement;
		mParameterID: AudioUnitParameterID;
		mFlags: UInt32;
		mSubRangeMin: AudioUnitParameterValue;
		mSubRangeMax: AudioUnitParameterValue;
		mStatus: UInt8;
		mData1: UInt8;
		reserved1: UInt8; // MUST be set to zero
		reserved2: UInt8; // MUST be set to zero
		reserved3: UInt32; // MUST be set to zero
	end;
	AUParameterMIDIMappingPtr = ^AUParameterMIDIMapping;

//=====================================================================================================================
//#pragma mark - Music Device
{!
    @enum           Instrument Unit (MusicDevice) Properties
    @abstract       The collection of Instrument Unit Property IDs

	@constant		kMusicDeviceProperty_InstrumentCount
	@discussion			Scope:				Global
						Value Type:			UInt32
						Access:				read
						
						For a mono-timbral music instrument, this property should return 0 (it should be implemented).
						
						For a multi-timbral music instrument, this property can return the number of independent patches that
						are available to be chosen as an active patch for the instrument. For instance, for Apple's DLS Music Device
						this value returns the number of patches that are found in a given DLS or SoundFont file when loaded.

	@constant		kMusicDeviceProperty_MIDIXMLNames
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_PartGroup
	@discussion			Scope:				Part
						Value Type:			AudioUnitElement
						Access:				read/write

						This property's value specifies the group ID (the Group scope's element) 
						that the part is (or should be) assigned to. The property is used in the Part scope, 
						where the element ID is the part that is being queried (or assigned).
						
						This property may be implemented in an audio unit as read only, as writeable only if the 
						audio unit is uninitialized, or as read/write. Apple recommends that it should be 
						writable at any time.
						
						The effect of assigning a new group to a part is undefined. Typically, however, it can be 
						expected that all existing notes would be turned off before the re-assignment is made by 
						the audio unit.

	@constant		kMusicDeviceProperty_DualSchedulingMode
	@discussion			Scope:				Global
						Value Type:			UInt32
						Access:				write

						Some instrument units need to distinguish realtime note and control events (such as from 
						incoming MIDI) from sequenced or pre-scheduled events. To support this, a host application 
						may set this property to 1. If the instrument unit returns a value of noErr, it supports 
						an alternate interpretation of the inOffsetSampleFrame parameter for the following 
						functions:

							MusicDeviceMIDIEvent
							MusicDeviceStartNote
							MusicDeviceStopNote
							AudioUnitSetParameter

						Once the host sets this property to 1 and the instrument unit returns noErr, the 
						inOffsetSampleFrame field becomes a bitfield:

							kMusicDeviceSampleFrameMask_SampleOffset = 0xFFFFFF // AND with this to obtain sample offset
							kMusicDeviceSampleFrameMask_IsScheduled = 0x01000000

						The IsScheduled bit should be set on events which are being scheduled ahead of time from 
						a prerecorded track. The IsScheduled bit should be clear on events which are being sent 
						to the instrument unit in response to realtime events, such as incoming MIDI or control 
						changes in a view.

	@constant		kMusicDeviceProperty_SupportsStartStopNote
	@discussion			Scope:				Global
						Value Type:			UInt32
						Access:				read
						
						The MusicDeviceStartNote and MusicDeviceStopNote APIs have been available since Mac OS X v10.0. 
						However, many third-party audio units do not implement these calls. This property can
						be used to determine if an audio unit does provide a compliant implementation. A compliant
						audio unit will both implement the property and return !0 as the value for the property. 
						Apple's DLSMusicDevice unit has implemented MusicDeviceStartNote and MusicDeviceStopNote
						since Mac OS X v10.0. The kMusicDeviceProperty_SupportsStartStopNote property was introduced
						with Mac OS X v10.5, so the DLSMusicDevice unit will not return an appropriate value for
						this property on a pre-10.5 system.
						
}	
const
// range  (1000 -> 1999)
	kMusicDeviceProperty_InstrumentCount = 1000;
	kMusicDeviceProperty_MIDIXMLNames = 1006;
	kMusicDeviceProperty_PartGroup = 1010;
	kMusicDeviceProperty_DualSchedulingMode = 1013;
	kMusicDeviceProperty_SupportsStartStopNote = 1014;

{!
	@enum			DualSchedulingMode
}
const
	kMusicDeviceSampleFrameMask_SampleOffset = $FFFFFF; // AND with this to obtain the sample offset
	kMusicDeviceSampleFrameMask_IsScheduled = $01000000;

//=====================================================================================================================
//#pragma mark - Offline Unit
{!
    @enum           Offline Unit Properties
    @abstract       The collection of properties for offline units 
	@constant		kAudioUnitOfflineProperty_InputSize
	@discussion			Scope:			Global
						Value Type:		UInt64
						Access:			read/write

						Once this property is set, an audio unit will assume that its input samples 
						have been reset to a new region. Setting this property will also cause the 
						audio unit's internal DSP state to be reset. That is, the audio unit calls 
						the AudioUnitReset function on itself.
						
						This property tells the offline unit how many samples to process. Once it 
						knows this number it will then request from 0 to (nSamples-1) in its input
						callback. The host of the audio unit is then required to provide the samples 
						specified in the sample count field of that Input's callback.

	@constant		kAudioUnitOfflineProperty_OutputSize
	@discussion			Scope:			Global
						Value Type:		UInt64
						Access:			read
						
						The host can use this property to estimate how many output samples an audio 
						unit will produce for the specified input samples. The property value
						is invalid if InputSize is not set. 
						
						The host cannot assume that the value returned is exact.
						It is a guide only, so is suitable for use in a progress bar, for instance.
						
						Termination of processing is solely determined by the setting of the 
						kAudioUnitStatus_OfflineRenderComplete property in the 
						ioRenderActionFlags from the AudioUnitRender function.

	@constant		kAudioUnitOfflineProperty_StartOffset
	@discussion			Scope:			Global
						Value Type:		UInt64
						Access:			read/write

						The host sets this property to tell an audio unit that the start offset of 
						the data it is processing has been changed. This should be set along with 
						the InputSize property, so that the unit knows its input data has been set 
						or changed.

	@constant		kAudioUnitOfflineProperty_PreflightRequirements
	@discussion			Scope:			Global
						Value Type:		UInt32
						Access:			read

						Returns one of the kOfflinePreflight_ results (see the Offline Preflight 
						enumeration).
						
	@constant		kAudioUnitOfflineProperty_PreflightName
	@discussion			Scope:			Global
						Value Type:		CFStringRef
						Access:			read

						For an audio unit that allows or requires preflighting, this property lets
						the unit give its host application a name to describe the preflight 
						operations.
}
const
// range (3020->3040)
	kAudioUnitOfflineProperty_InputSize = 3020;
	kAudioUnitOfflineProperty_OutputSize = 3021;
	kAudioUnitOfflineProperty_StartOffset = 3022;
	kAudioUnitOfflineProperty_PreflightRequirements = 3023;
	kAudioUnitOfflineProperty_PreflightName = 3024;

{!
	@enum		Offline Preflight Flags
	@abstract	Used to indicate an Offline Unit's preflight requirements
	@constant	kOfflinePreflight_NotRequired
	@discussion		Offline unit does not require preflight
	@constant	kOfflinePreflight_Optional
	@discussion		Offline unit will generally behave better if it is preflighted, but it is not
					required to be preflighted.
	@constant	kOfflinePreflight_Required
	@discussion		Offline unit requires preflighting or it cannot do its work

}
const
	kOfflinePreflight_NotRequired = 0;
	kOfflinePreflight_Optional = 1;
	kOfflinePreflight_Required = 2;

//=====================================================================================================================
//#pragma mark - Panner Unit
{!
    @enum           Panner Unit Properties
    @abstract       The collection of properties for panner units
	@constant		kAudioUnitProperty_DistanceAttenuationData
	@discussion			Scope:			Global
						Value Type:		AUDistanceAttenuationData
						Access:			Read
}
const
// range (3060->3999)
	kAudioUnitProperty_DistanceAttenuationData = 3600;

{!
	@struct			AUDistanceAttenuationData
}
type
	AUDistanceAttenuationDataPairsRec = record
		inDistance: Float32;	// 0-1000
		outGain: Float32;	// 0-1
	end;
	AUDistanceAttenuationDataPairsRecPtr = ^AUDistanceAttenuationDataPairsRec;
	
	AUDistanceAttenuationData = record
		inNumberOfPairs: UInt32;
		pairs: array[0..0] of AUDistanceAttenuationDataPairsRec; // this is a variable length array of inNumberOfPairs elements
	end;
	AUDistanceAttenuationDataPtr = ^AUDistanceAttenuationData;


//=====================================================================================================================
//#pragma mark - Translation Service
{!
    @enum           Translation Properties
    @abstract       The collection of properties for migrating data from other audio plug-ins to the 
    				Audio Unit architecture
	@discussion		While this is a general service, there are two formats that are explicitly defined: 
					MAS and VST. An audio unit may have MAS settings given to it in one of two ways:

						(1) The settings may have a single setting. This may be set multiple times with 
						different settings each time. In this case, numberOfSettings will be 1.

						(2) The settings may be set in one hit, providing all SettingData at once. 
						In this case, numberOfSettings may be more than 1, and will be the number of 
						settings the host has from the MAS plugin.

						AU-VST - the CFDataRef data contains VST chunk data with no additional information. 

					In addition, this can be used to migrate settings from an older audio unit; this allows manufacturers 
					to deprecate older audio units and replace them with new ones. The data for the older audio unit is 
					the audio unit preset CFDictionary that that unit generated.

	@constant		kAudioUnitMigrateProperty_FromPlugin
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAudioUnitMigrateProperty_OldAutomation
	@discussion			Scope:
						Value Type:
						Access:
}
const
// range (4000->4020)
	kAudioUnitMigrateProperty_FromPlugin = 4000;
	kAudioUnitMigrateProperty_OldAutomation = 4001;

{!
	@enum			Other Plug-in Formats
}
const
	kOtherPluginFormat_Undefined = 0; //reserving this value for future use
	kOtherPluginFormat_kMAS = 1;
	kOtherPluginFormat_kVST = 2;
	kOtherPluginFormat_AU = 3;

{!
	@struct			AudioUnitOtherPluginDesc
	@discussion

	@field			format
	@discussion			One of the OtherPluginFormat values
						
	@field			plugin
	@discussion			struct AudioClassDescription (
							OSType mType;
							OSType mSubType;
							OSType mManufacturer;
						);
						is defined in <CoreAudio/CoreAudioTypes.h>

					mType specifies a generic, plug-in format defined descriptor
							mSubType is usually left to the manufacturer to use at their discretion
							mManufacturer is a registered code to identify all plugins from the same manufacturer

}
type
	AudioUnitOtherPluginDesc = record
		format: UInt32;
		plugin: AudioClassDescription;
	end;
	AudioUnitOtherPluginDescPtr = ^AudioUnitOtherPluginDesc;

{!
	@struct			AudioUnitParameterValueTranslation
	@abstract		Used to translate another plug-in's parameter values to  audio unit parameter 
					values
}
type
	AudioUnitParameterValueTranslation = record
		otherDesc: AudioUnitOtherPluginDesc;
		otherParamID: UInt32;
		otherValue: Float32;
		auParamID: AudioUnitParameterID;
		auValue: AudioUnitParameterValue;
	end;
	AudioUnitParameterValueTranslationPtr = ^AudioUnitParameterValueTranslation;

{!
	@struct			AudioUnitPresetMAS_SettingData
	@discussion		AU-MAS specific structs for the data contained in the "masdata" key of an audio 
					unit preset dictionary
}
type
	AudioUnitPresetMAS_SettingData = record
		isStockSetting: UInt32; // zero or 1  i.e. "long bool"
		settingID: UInt32;
		dataLen: UInt32; //length of following data
		data: array[0..0] of UInt8;
	end;
	AudioUnitPresetMAS_SettingDataPtr = ^AudioUnitPresetMAS_SettingData;
	
{!
	@struct			AudioUnitPresetMAS_Settings
	@discussion		See MAS documentation
}
type
	AudioUnitPresetMAS_Settings = record
		manufacturerID: UInt32;
		effectID: UInt32;
		variantID: UInt32;
		settingsVersion: UInt32;
		numberOfSettings: UInt32;
	 	settings: array[0..0] of AudioUnitPresetMAS_SettingData;
	end;

{$endc}	{ not TARGET_OS_IPHONE }

//=====================================================================================================================
//#pragma mark -
//#pragma mark Apple Specific Properties

//=====================================================================================================================
//#pragma mark - AUConverter
{!
    @enum           Apple AUConverter Property IDs
    @abstract       The collection of property IDs for Apple AUConverter
	
	@constant		kAudioUnitProperty_SampleRateConverterComplexity
	@discussion			Scope:			Global
						Value Type:		UInt32
						Access:			read/write
}
const
	kAudioUnitProperty_SampleRateConverterComplexity = 3014;

{!
	@enum			Audio Unit Sample Rate Converter Complexity
	@discussion		The lowest quality of the Mastering algorithm is higher than the highest quality of the Normal algorithm.
	@constant		kAudioUnitSampleRateConverterComplexity_Normal
	@discussion			Normal quality sample rate conversion.
	@constant		kAudioUnitSampleRateConverterComplexity_Mastering
	@discussion			Mastering quality sample rate conversion. More expensive.
}
const
	kAudioUnitSampleRateConverterComplexity_Linear = FourCharCode('line');	// linear interpolation
	kAudioUnitSampleRateConverterComplexity_Normal = FourCharCode('norm');	// the default
	kAudioUnitSampleRateConverterComplexity_Mastering = FourCharCode('bats');	// higher quality, more expensive

//=====================================================================================================================
//#pragma mark - AUHAL and device units
{!
    @enum           Apple Output Property IDs
    @abstract       The collection of property IDs for Apple output units
	
	@constant		kAudioOutputUnitProperty_CurrentDevice
	@discussion			Scope:			Global
						Value Type:		AudioDeviceID
						Access:			read/write
						
						The audio device being used (or to be used) by and output device unit
						
	@constant		kAudioOutputUnitProperty_ChannelMap
	@discussion			Scope:			Input/Output
						Value Type:		Array of UInt32
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/fixes_2.4

						This will also work with AUConverter. This property is used to map input channels from an input (source) to a destination.
						The number of channels represented in the channel map is the number of channels of the destination. The channel map entries
						contain a channel number of the source that should be mapped to that destination channel. If -1 is specified, than that 
						destination channel will not contain any channel from the source (so it will be silent)
						
	@constant		kAudioOutputUnitProperty_EnableIO
	@discussion			Scope: ( scope output, element 0 = output ) ( scope input, element 1 = input )
						Value Type: UInt32
						Access: read/write
							Output units default to output-only operation. Host applications may disable 
							output or enable input operation using this property, if the output unit 
							supports it. 0=disabled, 1=enabled using I/O proc.

	@constant		kAudioOutputUnitProperty_StartTime
	@discussion			Scope: Global
						Value Type: AudioOutputUnitStartAtTimeParams
						Access: write only
							When this property is set on an output unit, it will cause the next Start request
							(but no subsequent Starts) to use AudioDeviceStartAtTime, using the specified 
							timestamp, passing false for inRequestedStartTimeIsInput.

	@constant		kAudioOutputUnitProperty_SetInputCallback
	@discussion			Scope: Global
						Value Type: AURenderCallbackStruct
						Access: read/write
							When an output unit has been enabled for input operation, this callback can be 
							used to provide a single callback to the host application from the input 
							I/O proc, in order to notify the host that input is available and may be 
							obtained by calling the AudioUnitRender function.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
							Note that the inputProc will always receive a NULL AudioBufferList in ioData.
							You must call AudioUnitRender in order to obtain the audio.

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
	@constant		kAudioOutputUnitProperty_HasIO
	@discussion			Scope: ( scope output, element 0 = output ) ( scope input, element 1 = input )
						Value Type: UInt32
						Access:
							See kAudioOutputUnitProperty_EnableIO
							Property value is 1 if input or output is enabled on the specified element.

	@constant		kAudioOutputUnitProperty_StartTimestampsAtZero
	@discussion			Scope: Global
						Value Type: UInt32
						Access: read/write

							Apple output units typically begin their stream of timestamps presented to their
							inputs at sample time 0. Some applications may wish to receive the HAL's timestamps
							directly instead. When this property is set to false, the output unit's sample times
							will be direct reflections of the HAL's -- except when a sample rate conversion
							makes this impossible.
							
							This property also applies to AUConverter. Its value defaults to 1 for AUHAL;
							1 for other AUs.
}
const
	kAudioOutputUnitProperty_CurrentDevice = 2000;
	kAudioOutputUnitProperty_ChannelMap = 2002; // this will also work with AUConverter
	kAudioOutputUnitProperty_EnableIO = 2003;
	kAudioOutputUnitProperty_StartTime = 2004;
	kAudioOutputUnitProperty_SetInputCallback = 2005;
	kAudioOutputUnitProperty_HasIO = 2006;
	kAudioOutputUnitProperty_StartTimestampsAtZero = 2007;	// this will also work with AUConverter

{!
	@struct			AudioOutputUnitStartAtTimeParams
}
type
	AudioOutputUnitStartAtTimeParams = record
	// see AudioDeviceStartAtTime
		mTimestamp: AudioTimeStamp;
		mFlags: UInt32;
	end;
	AudioOutputUnitStartAtTimeParamsPtr = ^AudioOutputUnitStartAtTimeParams;

//=====================================================================================================================
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
//#pragma mark - AUVoiceProcessing unit
{!
	@enum           Apple Voice Processing Property IDs
	@abstract       The collection of property IDs for Apple voice processing units.

	@constant		kAUVoiceIOProperty_BypassVoiceProcessing
	@discussion			Scope: Global
						Value Type: UInt32
						Access: read/write
							Bypass all processing done by the voice processing unit. When set to 0 
							(default), the processing is activated otherwise it is disabled.

	@constant		kAUVoiceIOProperty_VoiceProcessingEnableAGC
	@discussion			Scope: Global
						Value Type: UInt32
						Access: read/write
							Enable automatic gain control on the processed microphone/uplink 
                            signal. On by default.
 
	 @constant		kAUVoiceIOProperty_MuteOutput
	 @discussion		Scope: Global
						Value Type: UInt32
						Access: read/write
							Mutes the output of the voice processing unit. 
							0 (default) = muting off. 1 = mute output.  
}
const
	kAUVoiceIOProperty_BypassVoiceProcessing = 2100;
	kAUVoiceIOProperty_VoiceProcessingEnableAGC = 2101;
	kAUVoiceIOProperty_MuteOutput = 2104;

//#pragma mark - AUVoiceProcessing unit deprecated properties
{!
 @enum           Apple Voice Processing Property IDs that are being deprecated
 @abstract       The collection of property IDs for Apple voice processing units that are
 				 being deprecated.
 
 @constant		kAUVoiceIOProperty_VoiceProcessingQuality
 @discussion		Scope: Global
                    Value Type: UInt32
                    Access: read/write
                DEPRECATED. Sets the quality of the voice processing unit. Quality values
                are comprised between 0 (lowest) and 127 (highest).
 }
const
	kAUVoiceIOProperty_VoiceProcessingQuality = 2103; // deprecated

{!
 @enum         Apple Voice Processing AudioUnit Error IDs
 @abstract     These are the various error IDs returned by Voice Processing audio unit.
 
 @constant     kAUVoiceIOErr_UnexpectedNumberOfInputChannels
               This error indicates that an unexpected number of input channels was encountered during initialization of voice processing audio unit

}

const
	kAUVoiceIOErr_UnexpectedNumberOfInputChannels = -66784;

//=====================================================================================================================
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
//#pragma mark - Mixers
{!
    @enum           Apple Mixer Property IDs
    @abstract       The collection of property IDs for Apple mixers
	
	@constant		kAudioUnitProperty_MeteringMode
	@discussion			Scope: ( scope / element )
						Value Type: UInt32
						Access: read/write
						
						Enable or disable metering on a particular scope/element

	@constant		kAudioUnitProperty_MatrixLevels
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion			This property can be used for both the AUMatrixMixer and AUMultiChannelMixer.
	
						AUMatrixMixer
						Scope:			Global
						Value Type:		Float32 array
						Access:			read/write
=======
	@discussion			Scope:			Global
						Value Type:		Float32 array
						Access:			Read
>>>>>>> graemeg/cpstrnew
=======
	@discussion			Scope:			Global
						Value Type:		Float32 array
						Access:			Read
>>>>>>> graemeg/cpstrnew
=======
	@discussion			Scope:			Global
						Value Type:		Float32 array
						Access:			Read
>>>>>>> graemeg/cpstrnew
=======
	@discussion			Scope:			Global
						Value Type:		Float32 array
						Access:			Read
>>>>>>> origin/cpstrnew
=======
	@discussion			Scope:			Global
						Value Type:		Float32 array
						Access:			Read
>>>>>>> origin/fixes_2.4
						
						This property is used to retrieve the entire state of a matrix mixer. The size required is
						the number of (input  channels + 1) * (output channels + 1) - see _MatrixDimensions
						
						So a matrix mixer that has 2 input channels and 2 output channels, will need a 3 x 3 array of Float32
						  
						Global volume is stored at volumes[2][2]
						Input volumes are stored in the last column (volumes[0][2] for the first input channel,  volumes[1][2] for the second)
						Output volumes are stored in the last row (volumes [2][0] and [2][1])
						Cross point volumes are stored at their expected locations ([0][1], etc)
						
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						AUMultiChannelMixer
						Scope:			Input
						Value Type:		Float32 array
						Access:			read/write
						
						Gets/sets the matrix levels for one input element. This allows arbitrary mixing configurations
						from all input channels to all output channels.
						The size required is the number of (input channels) * (output channels).
						The matrix stores only the crosspoint gains, there are no overall input or output channel gains.
						
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
	@constant		kAudioUnitProperty_MatrixDimensions
	@discussion			Scope:			Global
						Value Type:		2 x UInt32
						Access:			Read only
							
						Returns the total number of channels for input and output of a given matrix mixer

	@constant		kAudioUnitProperty_MeterClipping
	@discussion			Scope:			Global
						Value Type:		AudioUnitMeterClipping
						Access:			Read
						
						A mixer returns an AudioUnitMeterClipping structure.

}
const
// General mixers
	kAudioUnitProperty_MeteringMode = 3007;
	
	// Matrix Mixer
	kAudioUnitProperty_MatrixLevels = 3006;
	kAudioUnitProperty_MatrixDimensions = 3009;
	kAudioUnitProperty_MeterClipping = 3011;

{!
	@struct			AudioUnitMeterClipping
	
	@field			peakValueSinceLastCall; 
	@discussion			The maximum value seen on the channel since the last time the property was retrieved.
	@field			sawInfinity;
	@discussion			TRUE if there was an infinite value on this channel.
	@field			sawNotANumber
	@discussion			TRUE if there was a floating point Not-A-Number value on this channel.
}
type
	AudioUnitMeterClipping = record
		peakValueSinceLastCall: Float32; 
		sawInfinity: Boolean;
		sawNotANumber: Boolean;
	end;
	AudioUnitMeterClippingPtr = ^AudioUnitMeterClipping;

//=====================================================================================================================
//#pragma mark - _3DMixer
{!
    @enum           Apple Mixer Property IDs
    @abstract       The collection of property IDs for Apple mixers
	
	@constant		kAudioUnitProperty_MeteringMode
	@discussion			Scope: ( scope / element )
						Value Type: UInt32
						Access: read/write
						
						Enable or disable metering on a particular scope/element

	@constant		kAudioUnitProperty_SpatializationAlgorithm
	@discussion			Scope:			Input
						Value Type:		UInt32
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/fixes_2.4
						
						Used to set the spatialisation algorithm used by an input of the 3DMixer. See kSpatializationAlgorithm_
						
	@constant		kAudioUnitProperty_DopplerShift
	@discussion			Scope:			Input
						Value Type:		UInt32
						Access:			Write
						
						Use a boolean true/false value to enable doppler shift for any specified input
						
	@constant		kAudioUnitProperty_3DMixerRenderingFlags
	@discussion			Scope:			Input
						Value Type:		UInt32
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						Access:			read/write
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> graemeg/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/cpstrnew
=======
						Access:			Read / Write
>>>>>>> origin/fixes_2.4

						Used to enable various rendering operations on a given input for the 3DMixer. See k3DMixerRenderingFlags_
						
	@constant		kAudioUnitProperty_3DMixerDistanceAtten
	@discussion			Scope:			
						Value Type:
						Access:

	@constant		kAudioUnitProperty_3DMixerDistanceParams
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAudioUnitProperty_ReverbPreset
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAudioUnitProperty_3DMixerAttenuationCurve 
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAudioUnitProperty_MatrixLevels
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAudioUnitProperty_MatrixDimensions
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAudioUnitProperty_MeterClipping
	@discussion			Scope:
						Value Type: AudioUnitMeterClipping
						Access:
							A mixer returns an AudioUnitMeterClipping structure.

}
const
	kAudioUnitProperty_3DMixerDistanceParams = 3010;
	kAudioUnitProperty_3DMixerAttenuationCurve = 3013;
	kAudioUnitProperty_SpatializationAlgorithm = 3000;
	kAudioUnitProperty_DopplerShift = 3002;
	kAudioUnitProperty_3DMixerRenderingFlags = 3003;
	kAudioUnitProperty_3DMixerDistanceAtten = 3004;
	kAudioUnitProperty_ReverbPreset = 3012;

{!
	@enum 3D Mixer Attenuation Curves
}
const
	k3DMixerAttenuationCurve_Power = 0;
	k3DMixerAttenuationCurve_Exponential = 1;
	k3DMixerAttenuationCurve_Inverse = 2;
	k3DMixerAttenuationCurve_Linear = 3;

{!
	@struct			MixerDistanceParams
}
type
	MixerDistanceParams = record
		mReferenceDistance: Float32;
		mMaxDistance: Float32;
		mMaxAttenuation: Float32;	// in decibels
	end;
	MixerDistanceParamsPtr = ^MixerDistanceParams;

{!
	@enum 	Spatialization Algorithms
}
const
	kSpatializationAlgorithm_EqualPowerPanning = 0;
	kSpatializationAlgorithm_SphericalHead = 1;
	kSpatializationAlgorithm_HRTF = 2;
	kSpatializationAlgorithm_SoundField = 3;
	kSpatializationAlgorithm_VectorBasedPanning = 4;
	kSpatializationAlgorithm_StereoPassThrough = 5;

{!
	@enum	3D Mixer Rendering Flags
}
const
	k3DMixerRenderingFlags_InterAuralDelay = 1 shl 0;
	k3DMixerRenderingFlags_DopplerShift = 1 shl 1;
	k3DMixerRenderingFlags_DistanceAttenuation = 1 shl 2;
	k3DMixerRenderingFlags_DistanceFilter = 1 shl 3;
	k3DMixerRenderingFlags_DistanceDiffusion = 1 shl 4;
	k3DMixerRenderingFlags_LinearDistanceAttenuation = 1 shl 5;
	k3DMixerRenderingFlags_ConstantReverbBlend = 1 shl 6;

//=====================================================================================================================
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
//#pragma mark -
//#pragma mark Desktop Apple Specific Properties


//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}

//=====================================================================================================================
//#pragma mark - DLSMusicDevice and Internal Reverb
{!
    @enum           Generic Property IDs
    @abstract       The collection of general audio unit property IDs
	
	@constant		kAudioUnitProperty_ReverbRoomType
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAudioUnitProperty_UsesInternalReverb
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_InstrumentName
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_InstrumentNumber
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_BankName
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_SoundBankData
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_StreamFromDisk
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_SoundBankFSRef
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_SoundBankURL
	@discussion			Scope:
						Value Type:
						Access:
}
const
	kAudioUnitProperty_ReverbRoomType = 10;

	// 3DMixer, DLSMusicDevice
	kAudioUnitProperty_UsesInternalReverb = 1005;

	// DLS Music Device
	kMusicDeviceProperty_InstrumentName = 1001;
	kMusicDeviceProperty_InstrumentNumber = 1004;
	kMusicDeviceProperty_UsesInternalReverb = kAudioUnitProperty_UsesInternalReverb;
	kMusicDeviceProperty_BankName = 1007;
	kMusicDeviceProperty_SoundBankData = 1008;
	kMusicDeviceProperty_StreamFromDisk = 1011;
	kMusicDeviceProperty_SoundBankFSRef = 1012;
	kMusicDeviceProperty_SoundBankURL = 1100;

{!
	@enum	Reverb Room Types
	@discussion Used to specify room type (as identified by a factory preset number) on Apple audio 
				units that use internal reverb.
}
const
	kReverbRoomType_SmallRoom = 0;
	kReverbRoomType_MediumRoom = 1;
	kReverbRoomType_LargeRoom = 2;
	kReverbRoomType_MediumHall = 3;
	kReverbRoomType_LargeHall = 4;
	kReverbRoomType_Plate = 5;
	kReverbRoomType_MediumChamber = 6;
	kReverbRoomType_LargeChamber = 7;
	kReverbRoomType_Cathedral = 8;
	kReverbRoomType_LargeRoom2 = 9;
	kReverbRoomType_MediumHall2 = 10;
	kReverbRoomType_MediumHall3 = 11;
	kReverbRoomType_LargeHall2 = 12;

//=====================================================================================================================
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
//#pragma mark - AUScheduledSoundPlayer
{!
    @enum           Apple AUScheduledSoundPlayer Property IDs
    @abstract       The collection of property IDs for the Apple AUScheduledSoundPlayer audio unit.

	@discussion		The AUScheduledSoundPlayer audio unit lets a client schedule audio buffers for
					future playback, with sample-accurate timing.
					
					
					Elements and Formats
					
					This unit has one output element and no input elements. The output's format
					should be a canonical audio unit stream format (native Float32, deinterleaved).


					Scheduling

					To schedule slices of audio for future playback, set the
					kAudioUnitProperty_ScheduleAudioSlice property, with a ScheduledAudioSlice
					structure as the property value. The slice's mTimeStamp.mSampleTime field
					determines when the slice will be played. This sample number is relative to
					the unit's start time, which you must set using the
					kAudioUnitProperty_ScheduleStartTimeStamp property before playback will
					begin.

					You must retain, unmodified, the ScheduledAudioSlice structure, including
					its mBufferList and the buffers to which it points, until the slice has been
					completely played, or until you stop playback by uninitializing or resetting
					the unit. The format of the slice's buffer list must match the unit's output
					stream format.
					
					(The fields other than mSampleTime and mFlags in the mTimestamp structure are 
					currently ignored.)
					
					
					Completion
					
					To receive a callback when the slice has been played, store a pointer to a
					callback function in the mCompletionProc field. This function will be called 
					(from the audio unit's rendering thread) when the slice has been completely 
					played -- or when the slice is determined to be unplayable due to an error. 
					As an alternative, you may also poll the slice's 
					(mFlags & kScheduledAudioSliceFlag_Complete).

					Upon completion, you can test (mFlags & kScheduledAudioSliceFlag_BeganToRenderLate) 
					to determine whether some portion of the slice was not played due to its having 
					been scheduled too late relative to the current playback time.
					
					
					Start Time
					
					The audio unit will not play any slices following initialization or reset, until
					its start time has been set. The start time establishes the beginning of a
					timeline: the timestamps of all slices in the schedule are relative to the
					start time.

					Set a start time by setting the kAudioUnitProperty_ScheduleStartTimeStamp 
					property with an AudioTimeStamp structure. If the timestamp contains a valid 
					sample time (timestamp.mFlags & kAudioTimeStampSampleTimeValid), then playback 
					begins when the timestamp passed to the AudioUnitRender function reaches the 
					specified sample time. If the specified sample time is -1, playback begins on 
					the next render cycle.
					
					If the start timestamp does not contain a valid sample time, but does contain a
					valid host time (timestamp.mFlags & kAudioTimeStampHostTimeValid), then the
					specified host time is translated to the sample time at which playback will
					begin. A host time of 0 means "start on the next render cycle."
					
					The kAudioUnitProperty_ScheduleStartTimeStamp property may be queried to obtain 
					the time at which playback began. If the start time has not yet been reached,
					the timestamp returned will be whatever the host application last set.
					
					
					Current play time
					
					The kAudioUnitProperty_CurrentPlayTime property may be queried to determine the 
					audio unit's current time offset from its start time. This is useful, for 
					example, to monitor playback progress.
					
					
					Unscheduling events
					
					To clear an audio unit's play schedule, call the AudioUnitReset function. The 
					completion proc (if any) for each slice in the schedule will called. Playback 
					will not resume until a new start time has been set. This also happens when 
					the audio unit is uninitialized.
	
	@constant		kAudioUnitProperty_ScheduleAudioSlice
	@discussion			Scope:
						Value Type: ScheduledAudioSlice
						Access:

	@constant		kAudioUnitProperty_ScheduleStartTimeStamp
	@discussion			Scope:
						Value Type: AudioTimeStamp
						Access:
							Sample time or host time valid. Sample time takes precedence, 
							-1 means "now". Host time of 0 means "now."
							
	@constant		kAudioUnitProperty_CurrentPlayTime
	@discussion			Scope:
						Value Type: AudioTimeStamp
						Access:
							AudioTimeStamp, relative to start time, sample time of -1 if not yet started.
}
const
	kAudioUnitProperty_ScheduleAudioSlice = 3300;
	kAudioUnitProperty_ScheduleStartTimeStamp = 3301;
	kAudioUnitProperty_CurrentPlayTime = 3302;

{!
	@enum			ScheduledAudioSlice
	@abstract		bits in ScheduledAudioSlice.mFlags
	@constant		kScheduledAudioSliceFlag_Complete
						Set if the unit is done with this slice
	@constant		kScheduledAudioSliceFlag_BeganToRender
						Set if any portion of the buffer has been played
	@constant		kScheduledAudioSliceFlag_BeganToRenderLate
						Set if any portion of the buffer was not played because it was scheduled late
}
const
	kScheduledAudioSliceFlag_Complete = 1;
	kScheduledAudioSliceFlag_BeganToRender = 2;
	kScheduledAudioSliceFlag_BeganToRenderLate = 4;

type
	// forward dec, see definition below
	ScheduledAudioSlicePtr = ^ScheduledAudioSlice;
{!
	@typedef			ScheduledAudioSliceCompletionProc
}

	ScheduledAudioSliceCompletionProc = procedure(userData : UnivPtr; bufferList : ScheduledAudioSlicePtr);
				
{
	@struct				ScheduledAudioSlice
	@field				mTimeStamp;
	@field				mCompletionProc;
							May be null
	@field				mCompletionProcUserData;
	@field				mFlags;
	@field				mReserved;
							Must be 0
	@field				mReserved2;
							For internal use
	@field				mNumberFrames;
							Must be consistent with byte count of mBufferList
	@field				mBufferList;
							Must contain deinterleaved Float32
}
	ScheduledAudioSlice = record
		mTimeStamp: AudioTimeStamp;
		mCompletionProc: ScheduledAudioSliceCompletionProc;	// may be null
		mCompletionProcUserData: UnivPtr;
		mFlags: UInt32;
		mReserved: UInt32;		// must be 0
		mReserved2: UnivPtr;		// for internal use
		mNumberFrames: UInt32;  // must be consistent with byte count of mBufferList
		mBufferList: AudioBufferListPtr;	// must contain deinterleaved Float32
	end;

//=====================================================================================================================
//#pragma mark - AUAudioFilePlayer
{!
    @enum           Apple AUAudioFilePlayer Property IDs
    @abstract       The collection of property IDs for Apple AUAudioFilePlayer
	@discussion		This audio unit lets you schedule regions of audio files for future playback,
					with sample-accurate timing.

					The unit is a subclass of AUScheduledSoundPlayer and inherits all of its
					behavior. In particular, this unit implements the kAudioUnitProperty_ScheduleStartTimeStamp 
					and kAudioUnitProperty_CurrentPlayTime properties. Instead of scheduling 
					slices (buffers) of audio to be played (via kAudioUnitProperty_ScheduleAudioSlice), 
					however, you schedule regions of audio files to be played. The unit reads and
					converts audio file data into its own internal buffers. It performs disk I/O
					on a high-priority thread shared among all instances of this unit within a
					process. Upon completion of a disk read, the unit internally schedules
					buffers for playback.


					Elements and Formats
					
					This unit has one output element and no input elements. The output's format
					should be a canonical audio unit stream format (native Float32,
					deinterleaved). This format should have at least as many channels are in the
					audio file(s) to be played (otherwise channels will be dropped). During
					playback, all audio file data is converted to the unit's output format.
					

					Audio Files

					Before starting playback, you must first open all audio files to be played
					using the AudioFile API's (see AudioToolbox/AudioFile.h), and pass their
					AudioFileIDs to the unit by setting the kAudioUnitProperty_ScheduledFileIDs
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					property. This property must not be set during playback. The audio files must
=======
					propery. This property must not be set during playback. The audio files must
>>>>>>> graemeg/cpstrnew
=======
					propery. This property must not be set during playback. The audio files must
>>>>>>> graemeg/cpstrnew
=======
					propery. This property must not be set during playback. The audio files must
>>>>>>> graemeg/cpstrnew
=======
					propery. This property must not be set during playback. The audio files must
>>>>>>> origin/cpstrnew
=======
					propery. This property must not be set during playback. The audio files must
>>>>>>> origin/fixes_2.4
					be kept open for the duration of playback.


					Scheduling Regions
					
					To schedule the playback of a region of an audio file, set the
					kAudioUnitProperty_ScheduledFileRegion property. This is a
					ScheduledAudioFileRegion structure. mTimeStamp.mSampleTime must be valid and
					is interpreted relative to the unit's start time -- the start time semantics
					(using kAudioUnitProperty_ScheduleStartTimeStamp) are identical to those of
					AUScheduledSoundPlayer. Unlike the ScheduledAudioSlice structures, the unit
					makes copies of ScheduledAudioFileRegions, so you may create them on the
					stack or otherwise reuse/dispose of them immediately after scheduling them.
					
					
					Priming
					
					You should set kAudioUnitProperty_ScheduledFilePrime after scheduling
					initial file regions to be played and before starting playback. This SetProperty call
					will begin reading the audio files and not return until the number of frames
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					specified by the property value have been read.
=======
					specifed by the property value have been read.
>>>>>>> graemeg/cpstrnew
=======
					specifed by the property value have been read.
>>>>>>> graemeg/cpstrnew
=======
					specifed by the property value have been read.
>>>>>>> graemeg/cpstrnew
=======
					specifed by the property value have been read.
>>>>>>> origin/cpstrnew
=======
					specifed by the property value have been read.
>>>>>>> origin/fixes_2.4
					
					
					Completion Callbacks
					
					A region's completion callback (mCompletionProc) is called when it has been
					completely scheduled for reading from disk. This callback is issued on the disk 
					read thread. If the region is not read from disk in time to play at its
					scheduled time, mCompletionProc is called a second time with an error code,
					also from the read thread. Note that the region passed to the callback will not
					be the same memory object as was passed by the client (since the unit copies the region).
					
					
					Start Time and Current Time
					
					These properties work identically as in AUScheduledSoundPlayer.
					
					
					Unscheduling regions
					
					To clear the unit's play schedule, call the AudioUnitReset function. The completion proc
					(if any) for each file region in the schedule will be called. Playback will
					not resume until a new start time has been set. This also happens when the
					unit is uninitialized.
					
					
					Customization
					
					The size and number of the player's disk read buffers default to "sensible"
					values, but may be configured with the properties:
						kAudioUnitProperty_ScheduledFileBufferSizeFrames
						kAudioUnitProperty_ScheduledFileNumberBuffers
					

					Bugs
					
					kAudioUnitProperty_ScheduledFileBufferSizeFrames
					kAudioUnitProperty_ScheduledFileNumberBuffers
						are currently unimplemented

					An option to make the unit not perform conversion from the audio file sample
					rate to the unit's output rate may be desirable.
	
	@constant		kAudioUnitProperty_ScheduledFileIDs
	@discussion			Scope:
						Value Type: Array of AudioFileIDs
						Access:
							Must set this property on scheduled file player for all files to be played

	@constant		kAudioUnitProperty_ScheduledFileRegion
	@discussion			Scope:
						Value Type: ScheduledAudioFileRegion
						Access:

	@constant		kAudioUnitProperty_ScheduledFilePrime
	@discussion			Scope:
						Value Type: UInt32
						Access:
							The number of frames to read from disk before returning, or 0 to specify use 
							of a default value

	@constant		kAudioUnitProperty_ScheduledFileBufferSizeFrames
	@discussion			Scope:
						Value Type: UInt32
						Access:

	@constant		kAudioUnitProperty_ScheduledFileNumberBuffers
	@discussion			Scope:
						Value Type: UInt32
						Access:
}
const
	kAudioUnitProperty_ScheduledFileIDs = 3310;
	kAudioUnitProperty_ScheduledFileRegion = 3311;
	kAudioUnitProperty_ScheduledFilePrime = 3312;
	kAudioUnitProperty_ScheduledFileBufferSizeFrames = 3313;
	kAudioUnitProperty_ScheduledFileNumberBuffers = 3314;

type
	ScheduledAudioFileRegionPtr = ^ScheduledAudioFileRegion; //forward declaraion, see definition below
{!
	@typedef		ScheduledAudioFileRegionCompletionProc
}
	ScheduledAudioFileRegionCompletionProc = procedure(userData : UnivPtr; fileRegion : ScheduledAudioFileRegionPtr; aResult : OSStatus);

{!
	@struct			ScheduledAudioFileRegion
	@field			mTimeStamp
	@field			mCompletionProc
						may be NULL
	@field			mCompletionProcUserData
	@field			mAudioFile
						must be a valid and open AudioFileID
						defined in AudioToolbox/AudioFile.h: typedef	struct OpaqueAudioFileID	*AudioFileID;
	@field			mLoopCount
						0 = don't loop
	@field			mStartFrame
						offset into file
	@field			mFramesToPlay
						number of frames to play
	
}

	ScheduledAudioFileRegion = record
		mTimeStamp: AudioTimeStamp;
		mCompletionProc: ScheduledAudioFileRegionCompletionProc;
		mCompletionProcUserData: UnivPtr;
		mAudioFile: Pointer; { struct OpaqueAudioFileID * }
		mLoopCount: UInt32;
		mStartFrame: SInt64;
		mFramesToPlay: UInt32;
	end;

//=====================================================================================================================
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
//#pragma mark -
//#pragma mark Desktop Apple Specific Properties


{$ifc not TARGET_OS_IPHONE}

//=====================================================================================================================
//#pragma mark - DLSMusicDevice and Internal Reverb
{!
    @enum           Generic Property IDs
    @abstract       The collection of general audio unit property IDs
	
	@constant		kAudioUnitProperty_ReverbRoomType
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAudioUnitProperty_UsesInternalReverb
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_InstrumentName
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_InstrumentNumber
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_BankName
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_SoundBankData
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_StreamFromDisk
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_SoundBankFSRef
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kMusicDeviceProperty_SoundBankURL
	@discussion			Scope:
						Value Type:
						Access:
}
const
	kAudioUnitProperty_ReverbRoomType = 10;

	// 3DMixer, DLSMusicDevice
	kAudioUnitProperty_UsesInternalReverb = 1005;

	// DLS Music Device
	kMusicDeviceProperty_InstrumentName = 1001;
	kMusicDeviceProperty_InstrumentNumber = 1004;
	kMusicDeviceProperty_UsesInternalReverb = kAudioUnitProperty_UsesInternalReverb;
	kMusicDeviceProperty_BankName = 1007;
	kMusicDeviceProperty_SoundBankData = 1008;
	kMusicDeviceProperty_StreamFromDisk = 1011;
	kMusicDeviceProperty_SoundBankFSRef = 1012;
	kMusicDeviceProperty_SoundBankURL = 1100;

{!
	@enum	Reverb Room Types
	@discussion Used to specify room type (as identified by a factory preset number) on Apple audio 
				units that use internal reverb.
}
const
	kReverbRoomType_SmallRoom = 0;
	kReverbRoomType_MediumRoom = 1;
	kReverbRoomType_LargeRoom = 2;
	kReverbRoomType_MediumHall = 3;
	kReverbRoomType_LargeHall = 4;
	kReverbRoomType_Plate = 5;
	kReverbRoomType_MediumChamber = 6;
	kReverbRoomType_LargeChamber = 7;
	kReverbRoomType_Cathedral = 8;
	kReverbRoomType_LargeRoom2 = 9;
	kReverbRoomType_MediumHall2 = 10;
	kReverbRoomType_MediumHall3 = 11;
	kReverbRoomType_LargeHall2 = 12;

//=====================================================================================================================
//#pragma mark - AUSampler

{!
	@enum			Apple AUSampler Property IDs
	@abstract		The collection of property IDs for the Apple AUSampler audio unit.
 
	@discussion		The AUSampler audio unit lets a client create an editable, interactive
					sampler synthesizer instrument.
 
	@constant		kAUSamplerProperty_LoadInstrument
	@discussion			Scope:			Global
						Value Type:		AUSamplerInstrumentData
						Access:			Write
							Load an instrument from an external DLS or Soundfont2 bank file, or from other file formats.
 
	@constant		kAUSamplerProperty_LoadAudioFiles
	@discussion			Scope:			Global
						Value Type:		CFArrayRef
						Access:			Write
							Create a new preset from a list of audio file paths.  The CFArray should contain a set
							of CFURLRefs, one per file.  The previous preset will be completely cleared.
 }
const
// range (4100->4999)
	kAUSamplerProperty_LoadInstrument = 4102;
	kAUSamplerProperty_LoadAudioFiles = 4101;

{
	@struct			AUSamplerInstrumentData
	@abstract		Used for loading an instrument from either an external bank file (i.e. DLS or SoundFont), an Apple
 					.aupreset, a Logic or GarageBand EXS24 sampler instrument, or creating a new default instrument from
 					a single audio file.  The path to the bank or instrument file is specified in the fileURL field.
 					The instrumentType field distinguishes between the instrument types.  The remaining fields of this
 					struct are used only for the kInstrumentType_DLSPreset and kInstrumentType_SF2Preset types to
 					identify the particular bank and preset IDs for the instrument you wish to load from the bank.
 					They represent values for MIDI controllers 0 and 32 and the MIDI present change message that would be
 					sent to a GM2-compatible synth for program changes.  Use the provided constants
 					(kAUSampler_DefaultMelodicBankMSB,  kAUSampler_DefaultPercussionBankMSB) to designate  melodic or
 					percussion banks per the GM2 specification (GM-compatible DLS or Soundfont banks).  For custom
 					non-GM-compatible DLS and Soundfont banks, use the actual MSB/LSB values associated with the desired preset.

	@field			fileURL
						The URL of the path to the bank or instrument file.   Caller is responsible for releasing the
 						provided CFURLRef.
	@field			instrumentType
						The type of instrument being loaded or created.  For example, use kInstrumentType_DLSPreset to load an
 						instrument from a DLS bank file.
	@field			bankMSB
						For the preset instruments, the most significant byte value for a particular bank variation within that
 						file.  Range is 0 to 127.  Use kAUSampler_DefaultMelodicBankMSB by default.
	@field			bankLSB
						For the preset instruments, the least significant byte value for a particular bank variation within that
 						file.  Range is 0 to 127.  Use kAUSampler_DefaultBankLSB by default.
	@field			presetID
						For the preset instruments, the numeric ID of a particular preset within that bank to load.
 						Range is 0 to 127.
 }

type
	AUSamplerInstrumentData = record
		fileURL: CFURLRef;
		instrumentType: UInt8;
		bankMSB: UInt8;
		bankLSB: UInt8;
		presetID: UInt8;
	end;
	AUSamplerInstrumentDataPtr = ^AUSamplerInstrumentData;

{
	@enum			InstrumentTypes
	@abstract		Values to specify the type of instrument being loaded.

	@constant		kInstrumentType_DLSPreset
	@discussion			A preset from a DLS bank file.  Bank MSB, LSB and preset ID must be specified.
 
	@constant		kInstrumentType_SF2Preset
	@discussion			A preset from a SoundFont2 bank file.  Bank MSB, LSB and preset ID must be specified.
 
	@constant		kInstrumentType_AUPreset
	@discussion			A native Apple .aupreset file created using the AUSampler's custom view.
 
	@constant		kInstrumentType_Audiofile
	@discussion			An audio file which will be used to create a default instrument with that file as its sole sample.
	
 	@constant		kInstrumentType_EXS24
	@discussion			A Logic or GarageBand sampler instrument.

 }

const
	kInstrumentType_DLSPreset = 1;
	kInstrumentType_SF2Preset = kInstrumentType_DLSPreset;
	kInstrumentType_AUPreset = 2;
	kInstrumentType_Audiofile = 3;
	kInstrumentType_EXS24 = 4;

const
	kAUSampler_DefaultPercussionBankMSB = $78;
	kAUSampler_DefaultMelodicBankMSB = $79;
	kAUSampler_DefaultBankLSB = $00;


//=====================================================================================================================
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
//#pragma mark - AUDeferredRenderer
{!
	@enum			AUDeferredRenderer
	@discussion		This audio unit has one input element and one output element. They must both have
					the same format, which must be canonical (Float32 deinterleaved) and must have 
					the same number of channels.

					The AUDeferredRenderer unit creates a high-priority producer thread, on which 
					calls by this AU for input are performed at a constant buffer size. This buffer size may be
					set with the kAudioUnitProperty_DeferredRendererPullSize property. The deferred 
					renderer may be asked to render at different buffer sizes by a downstream unit or
					host application, but it always pulls upstream at its constant buffer size.
					The upstream pull size MUST be greater than or equal to the downstream pull
					size.

					The upstream producer thread runs in advance of calls to its Render
					function, with respect to the timestamps being passed to Render and
					PullInput. The difference between these timestamps is the unit's "latency",
					which is always at least one upstream pull buffer. The client may specify
					additional latency with the property
					kAudioUnitProperty_DeferredRendererExtraLatency, which is a number of sample
					frames.

					It is possible, at Render time, for the producer thread to have not yet
					finished rendering the necessary data. This generates an error. In order to
					give the producer a small amount of extra time to finish rendering, the
					client may set the unit's property
					kAudioUnitProperty_DeferredRendererWaitFrames. If this property is non-zero,
					then when Render finds that insufficient data has been produced, it will
					sleep for the amount of realtime corresponding to the number of wait frames.
					It will then check again to see if the required amount of data has been
					produced, and fail if it hasn't.

	@constant		kAudioUnitProperty_DeferredRendererPullSize
	@discussion			Scope:
						Value Type: UInt32
						Access:

	@constant		kAudioUnitProperty_DeferredRendererExtraLatency
	@discussion			Scope:
						Value Type: UInt32
						Access:

	@constant		kAudioUnitProperty_DeferredRendererWaitFrames
	@discussion			Scope:
						Value Type: UInt32
						Access:
}
const
	kAudioUnitProperty_DeferredRendererPullSize = 3320;
	kAudioUnitProperty_DeferredRendererExtraLatency = 3321;
	kAudioUnitProperty_DeferredRendererWaitFrames = 3322;

//=====================================================================================================================
//#pragma mark - AUNetReceive
{!
	@enum			AUNetReceive
	@constant		kAUNetReceiveProperty_Hostname
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion			Scope: Global
						Value Type: CFStringRef
						Access:
						The hostname from which you wish to receive audio.
						For GetProperty, the returned CFStringRef is a copy and therefore must be released by the caller.
						The UI view for AUNetReceive does the resolution of Bonjour service names to hostnames. 
						Clients who are using this AU programmatically using Bonjour will have to do this resolution themselves. 
						It is not done by the AU.

	@constant		kAUNetReceiveProperty_Password
	@discussion			Scope: Global
						Value Type: CFStringRef
						Access: Read / Write
						The password to send to the sender. Leave unset or set to the empty string for no password.
						For GetProperty, the returned CFStringRef is a copy and therefore must be released by the caller.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAUNetReceiveProperty_Password
	@discussion			Scope:
						Value Type:
						Access:
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
	kAUNetReceiveProperty_Hostname = 3511;
	kAUNetReceiveProperty_Password = 3512;
	
//=====================================================================================================================
//#pragma mark - AUNetSend
{!
	@enum			AUNetSend
	@constant		kAUNetSendProperty_PortNum
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion			Scope: Global
						Value Type: UInt32
						Access: Read / Write
						The network port number on which to send.

	@constant		kAUNetSendProperty_TransmissionFormat
	@discussion			Scope: Global
						Value Type: AudioStreamBasicDescription
						Access: Read / Write
						Get or set an arbitrary format that will be used to transmit the audio.
						For compressed formats, it is recommended to use kAUNetSendProperty_TransmissionFormatIndex instead of this property,
						since there is no way to specify a bit rate with this property.

	@constant		kAUNetSendProperty_TransmissionFormatIndex
	@discussion			Scope: Global
						Value Type: UInt32
						Access: Read / Write
						Get or set the index of the preset format that will be used to transmit the audio.
						The format indices can be found in the NetSendPresetFormat enum.

	@constant		kAUNetSendProperty_ServiceName
	@discussion			Scope: Global
						Value Type: CFStringRef
						Access: Read / Write
						The name you want to publish for this network service.
						For GetProperty, the returned CFStringRef is a copy and therefore must be released by the caller.

	@constant		kAUNetSendProperty_Disconnect
	@discussion			Scope: Global
						Value Type: UInt32
						Access: Read / Write
						In order to disconnect, call this with a non-zero value.
						In order to connect, call this with a zero value. 
						For GetProperty, the returned value the last value set by the caller.
						To get the current connection status, get the value of the parameter kAUNetReceiveParam_Status.

	@constant		kAUNetSendProperty_Password
	@discussion			Scope: Global
						Value Type: CFStringRef
						Access: Read / Write
						The password that must be used by the receiver. Leave unset or set to the empty string for no password.
						For GetProperty, the returned CFStringRef is a copy and therefore must be released by the caller.

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAUNetSendProperty_TransmissionFormat
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAUNetSendProperty_TransmissionFormatIndex
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAUNetSendProperty_ServiceName
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAUNetSendProperty_Disconnect
	@discussion			Scope:
						Value Type:
						Access:

	@constant		kAUNetSendProperty_Password
	@discussion			Scope:
						Value Type:
						Access:
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
	kAUNetSendProperty_PortNum = 3513;
	kAUNetSendProperty_TransmissionFormat = 3514;
	kAUNetSendProperty_TransmissionFormatIndex = 3515;
	kAUNetSendProperty_ServiceName = 3516;
	kAUNetSendProperty_Disconnect = 3517;
	kAUNetSendProperty_Password = 3518;

{!
	@enum			NetSendPresetFormat
	@constant		kAUNetSendPresetFormat_PCMFloat32
	@discussion			1411 kilobits per second per channel @ 44100KHz (kilo == 1000 not 1024)
	@constant		kAUNetSendPresetFormat_PCMInt24
	@discussion			1058 kilobits per second per channel @ 44100KHz
	@constant		kAUNetSendPresetFormat_PCMInt16
	@discussion			706 kilobits per second per channel @ 44100KHz
	@constant		kAUNetSendPresetFormat_Lossless24
	@discussion			650 kilobits per second per channel @ 44100KHz
	@constant		kAUNetSendPresetFormat_Lossless16
	@discussion			350 kilobits per second per channel @ 44100KHz
	@constant		kAUNetSendPresetFormat_ULaw
	@discussion			353 kilobits per second per channel @ 44100KHz
	@constant		kAUNetSendPresetFormat_IMA4
	@discussion			176 kilobits per second per channel @ 44100KHz
	@constant		kAUNetSendPresetFormat_AAC_128kbpspc
	@discussion			128 kilobits per second per channel
	@constant		kAUNetSendPresetFormat_AAC_96kbpspc
	@discussion			96 kilobits per second per channel
	@constant		kAUNetSendPresetFormat_AAC_80kbpspc
	@discussion			80 kilobits per second per channel
	@constant		kAUNetSendPresetFormat_AAC_64kbpspc
	@discussion			64 kilobits per second per channel
	@constant		kAUNetSendPresetFormat_AAC_48kbpspc
	@discussion			48 kilobits per second per channel
	@constant		kAUNetSendPresetFormat_AAC_40kbpspc
	@discussion			40 kilobits per second per channel
	@constant		kAUNetSendPresetFormat_AAC_32kbpspc
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion			32 kilobits per second per channel
=======
	@discussion			kilobits per second per channel
>>>>>>> graemeg/cpstrnew
=======
	@discussion			kilobits per second per channel
>>>>>>> graemeg/cpstrnew
=======
	@discussion			kilobits per second per channel
>>>>>>> graemeg/cpstrnew
=======
	@discussion			kilobits per second per channel
>>>>>>> origin/cpstrnew
=======
	@discussion			kilobits per second per channel
>>>>>>> origin/fixes_2.4
	@constant		kAUNetSendNumPresetFormats = 14
}
const
	kAUNetSendPresetFormat_PCMFloat32 = 0;
	kAUNetSendPresetFormat_PCMInt24 = 1;
	kAUNetSendPresetFormat_PCMInt16 = 2;
	kAUNetSendPresetFormat_Lossless24 = 3;
	kAUNetSendPresetFormat_Lossless16 = 4;
	kAUNetSendPresetFormat_ULaw = 5;
	kAUNetSendPresetFormat_IMA4 = 6;
	kAUNetSendPresetFormat_AAC_128kbpspc = 7;
	kAUNetSendPresetFormat_AAC_96kbpspc = 8;
	kAUNetSendPresetFormat_AAC_80kbpspc = 9;
	kAUNetSendPresetFormat_AAC_64kbpspc = 10;
	kAUNetSendPresetFormat_AAC_48kbpspc = 11;
	kAUNetSendPresetFormat_AAC_40kbpspc = 12;
	kAUNetSendPresetFormat_AAC_32kbpspc = 13;
	kAUNetSendPresetFormat_AAC_LD_64kbpspc = 14;
	kAUNetSendPresetFormat_AAC_LD_48kbpspc = 15;
	kAUNetSendPresetFormat_AAC_LD_40kbpspc = 16;
	kAUNetSendPresetFormat_AAC_LD_32kbpspc = 17;
	kAUNetSendNumPresetFormats = 18;

{$endc}	{ not TARGET_OS_IPHONE }

//=====================================================================================================================
//#pragma mark -
//#pragma mark Deprecated Properties

//#if !TARGET_OS_IPHONE
{$ifc not TARGET_OS_IPHONE}

// NumVersion is no longer used (originally from MacTypes.h)
type
//#if TARGET_RT_BIG_ENDIAN
{$ifc TARGET_RT_BIG_ENDIAN}

	AUNumVersion = record
{ Numeric version part of 'vers' resource }
		majorRev: UInt8;               {1st part of version number in BCD}
		minorAndBugRev: UInt8;         {2nd & 3rd part of version number share a byte}
		stage: UInt8;                  {stage code: dev, alpha, beta, final}
		nonRelRev: UInt8;              {revision level of non-released version}
	end;
{$elsec} { TARGET_RT_BIG_ENDIAN }

	AUNumVersion = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Numeric version part of 'vers' resource accessible in little endian format }
=======
{ Numeric version part of 'vers' resource accessable in little endian format }
>>>>>>> graemeg/cpstrnew
=======
{ Numeric version part of 'vers' resource accessable in little endian format }
>>>>>>> graemeg/cpstrnew
=======
{ Numeric version part of 'vers' resource accessable in little endian format }
>>>>>>> graemeg/cpstrnew
=======
{ Numeric version part of 'vers' resource accessable in little endian format }
>>>>>>> origin/cpstrnew
=======
{ Numeric version part of 'vers' resource accessable in little endian format }
>>>>>>> origin/fixes_2.4
		nonRelRev: UInt8;              {revision level of non-released version}
		stage: UInt8;                  {stage code: dev, alpha, beta, final}
		minorAndBugRev: UInt8;         {2nd & 3rd part of version number share a byte}
		majorRev: UInt8;               {1st part of version number in BCD}
	end;
{$endc}  { TARGET_RT_BIG_ENDIAN }
	AUNumVersionPtr = ^AUNumVersion;
	AUNumVersionHandle = ^AUNumVersionPtr;
	
{!
	@struct			AUHostIdentifier
	@abstract		Used to describe the name and version of the audio unit's host
}
type
	AUHostIdentifier = record
		hostName: CFStringRef;	
		hostVersion: AUNumVersion;
	end;


// $$$ THESE NEED TO BE REMOVED FROM 64bit apps
//=====================================================================================================================
// GENERIC
const
	kAudioUnitParameterFlag_Global = 1 shl 0;	//	parameter scope is global
	kAudioUnitParameterFlag_Input = 1 shl 1;	//	parameter scope is input
	kAudioUnitParameterFlag_Output = 1 shl 2;	//	parameter scope is output
	kAudioUnitParameterFlag_Group = 1 shl 3;		//	parameter scope is group

const
	kAudioUnitParameterFlag_HasName = kAudioUnitParameterFlag_ValuesHaveStrings;

const
//kAudioUnitProperty_SetInputCallback		= 7 -> deprecated
	kAudioUnitProperty_SRCAlgorithm = 9; // see kAudioUnitProperty_SampleRateConverterComplexity
	kAudioUnitProperty_MIDIControlMapping = 17; // see ParameterMIDIMapping Properties
	kAudioUnitProperty_CurrentPreset = 28; // see PresentPreset

	kAudioUnitProperty_ParameterValueName = kAudioUnitProperty_ParameterStringFromValue;
	kAudioUnitProperty_BusCount = kAudioUnitProperty_ElementCount;
	kAudioOfflineUnitProperty_InputSize = kAudioUnitOfflineProperty_InputSize;
	kAudioOfflineUnitProperty_OutputSize = kAudioUnitOfflineProperty_OutputSize;

const
	kAudioUnitSRCAlgorithm_Polyphase = FourCharCode('poly');	// same as kAudioUnitSampleRateConverterComplexity_Normal
	kAudioUnitSRCAlgorithm_MediumQuality = FourCharCode('csrc');	// same as kAudioUnitSampleRateConverterComplexity_Normal


// Deprecated in Mac OS X v10.2. See AUParameterMIDIMapping.
type
	AudioUnitMIDIControlMapping = record
		midiNRPN: UInt16;
		midiControl: UInt8;
		scope: UInt8;
		element: AudioUnitElement;
		parameter: AudioUnitParameterID;
	end;

// Deprecated. See AudioUnitParameterStringFromValue for equivalent structure, but with clearer field names
type
	AudioUnitParameterValueName = record
		inParamID: AudioUnitParameterID;
		inValue: {const} Float32Ptr;	// may be NULL if should translate current parameter value
		outName: CFStringRef;  	// see comments for kAudioUnitProperty_ParameterStringFromValue
	end;


//=====================================================================================================================
// Deprecated. These properties are Apple specific.
const
	kMusicDeviceProperty_GroupOutputBus = 1002;
	kMusicDeviceProperty_SoundBankFSSpec = 1003;
	kAudioUnitProperty_PannerMode = 3008;

///$$$ THESE NEED TO BE EXCLUDED FROM 64BIT $$$

const
	kAudioUnitProperty_SpeakerConfiguration = 3001;

	// Deprecated in favor of the newer AudioChannelLayout
	// structure and its supporting property.
const
	kSpeakerConfiguration_HeadPhones = 0;
	kSpeakerConfiguration_Stereo = 1;
	kSpeakerConfiguration_Quad = 2;
	kSpeakerConfiguration_5_0 = 3;
	kSpeakerConfiguration_5_1 = kSpeakerConfiguration_5_0;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	AUSamplerBankPresetData = record
		bankURL: CFURLRef;
		bankMSB: UInt8;
		bankLSB: UInt8;
		presetID: UInt8;
		reserved: UInt8;
	end;
	AUSamplerBankPresetDataPtr = ^AUSamplerBankPresetData;

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
//#endif !TARGET_OS_IPHONE
{$endc}	{ not TARGET_OS_IPHONE }

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation


function GetAudioUnitParameterDisplayType(flags : UInt32) : UInt32; inline;
begin
	GetAudioUnitParameterDisplayType := flags and kAudioUnitParameterFlag_DisplayMask
end;

function AudioUnitDisplayTypeIsLogarithmic(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsLogarithmic := GetAudioUnitParameterDisplayType(flags) = 	kAudioUnitParameterFlag_DisplayLogarithmic
end;

function AudioUnitDisplayTypeIsSquareRoot(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsSquareRoot := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplaySquareRoot
end;

function AudioUnitDisplayTypeIsSquared(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsSquared := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplaySquared
end;

function AudioUnitDisplayTypeIsCubed(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsCubed := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayCubed
end;

function AudioUnitDisplayTypeIsCubeRoot(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsCubeRoot := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayCubeRoot
end;

function AudioUnitDisplayTypeIsExponential(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsExponential := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayExponential
end;

procedure SetAudioUnitParameterDisplayType(var flags : UInt32; displayType : UInt32); inline;
begin
	flags := (flags and (not kAudioUnitParameterFlag_DisplayMask)) or displayType
end;

end.

{$endc} {not MACOSALLINCLUDE}
