{
    File:       CoreMIDI/MIDIThruConnection.h
 
    Contains:   Routines for creating MIDI play-through connections.
 
 	Copyright:  (c) 2000-2008 by Apple Inc., all rights reserved.
 
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

unit MIDIThruConnection;
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
uses MacTypes,CFBase,CFData,MIDIServices;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


//  -----------------------------------------------------------------------------
{!
    @header MIDIThruConnection.h

    This header defines functions to create MIDI play-through connections
    between the MIDI sources and destinations.  These connections may be
    persistent or transitory, owned by a client.
    
    By using connections instead of doing MIDI Thru operations themselves,
    the overhead of moving MIDI messages between the server and the client
    for thru-ing is reduced.
    
    The aim of these functions is to permit as flexible a set of transformations 
    as possible while keeping the API and data structures relatively simple.    
}
//  -----------------------------------------------------------------------------

{!
    @typedef        MIDIThruConnectionRef
    
    @discussion     An opaque reference to a play-through connection.
}
//#if __LP64__
{$ifc TARGET_CPU_64}
type
	MIDIThruConnectionRef = MIDIObjectRef;
{$elsec}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	MIDIThruConnectionRef = ^OpaqueMIDIThruConnection; { an opaque type }
	OpaqueMIDIThruConnection = record end;
=======
	MIDIThruConnectionRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MIDIThruConnectionRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MIDIThruConnectionRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MIDIThruConnectionRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	MIDIThruConnectionRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
{$endc}

{!
    @struct         MIDIValueMap
    
    @discussion     A custom mapping function to transform MIDI 7-bit values,
                    as contained in note numbers, velocities, control values,
                    etc.  y = value[x], where x is the input MIDI value, y the
                    output.
}
type
	MIDIValueMap = record
		value: packed array [0..127] of UInt8;
	end;

{!
    @enum           MIDITransformType
    
    @discussion     Values specifying a type of MIDI transformation, as found in the transform member of MIDITransform.
    
    @constant   kMIDITransform_None
        no transformation (param unused)
    @constant   kMIDITransform_FilterOut
        filter out the specified event type (param unused)
    @constant   kMIDITransform_MapControl
        transform one control number to another; param is destination control number
    @constant   kMIDITransform_Add
        add param to values
    @constant   kMIDITransform_Scale
        multiple value by the fixed point number in param, which is in fixed point: bbbb.bbbb bbbb bbbb
    @constant   kMIDITransform_MinValue
        the value's minimum value is param
    @constant   kMIDITransform_MaxValue
        the value's maximum value is param
    @constant   kMIDITransform_MapValue
        transform the value using a map; param is the index of the map in the connection's array of maps.
}
const
// MIDITransformType
	kMIDITransform_None = 0;
	kMIDITransform_FilterOut = 1;
	kMIDITransform_MapControl = 2;
	kMIDITransform_Add = 8;
	kMIDITransform_Scale = 9;
	kMIDITransform_MinValue = 10;
	kMIDITransform_MaxValue = 11;
	kMIDITransform_MapValue = 12;
type
	MIDITransformType = UInt16;

const
	kMIDIThruConnection_MaxEndpoints = 8;

{!
    @enum       MIDITransformControlType
    
    @discussion Specifies how control numbers are interpreted.
    @constant   kMIDIControlType_7Bit
        control numbers may be 0-127
    @constant   kMIDIControlType_14Bit
        control numbers may be 0-31
    @constant   kMIDIControlType_7BitRPN
        control numbers may be 0-16383
    @constant   kMIDIControlType_14BitRPN
    @constant   kMIDIControlType_7BitNRPN
    @constant   kMIDIControlType_14BitNRPN
}
const
// MIDITransformControlType
	kMIDIControlType_7Bit = 0;
	kMIDIControlType_14Bit = 1;
	kMIDIControlType_7BitRPN = 2;
	kMIDIControlType_14BitRPN = 3;
	kMIDIControlType_7BitNRPN = 4;
	kMIDIControlType_14BitNRPN = 5;
type
	MIDITransformControlType = UInt8;

{!
    @struct     MIDITransform
    @abstract   Describes how a single type of MIDI event is transformed.
    @discussion This structure controls the transformation of various MIDI events other than control changes.
    @field      transform   The type of transformation to be applied to the event values.
    @field      param       An argument to the transformation method (see description of MIDITransformType).
}
type
	MIDITransform = record
		transform: MIDITransformType;
		param: SInt16;
	end;

{!
    @struct     MIDIControlTransform
    @abstract   Describes a transformation of MIDI control change events.
    @discussion
        A single MIDIThruConnectionParams may describe any number of transformations to control
        events. It is important that multiple transformations are ordered correctly: filter out,
        remap, then alter values.
        
        All transformations are done internally using 14-bit values, so for example, when doing
        an add/min/max transform on a 7-bit control value, the parameter must be a 14-bit value.
        For example, to add 10 to a control value, param must be (10 << 7) = 1280.
    @field  controlType         The type of control specified by controlNumber
    @field  remappedControlType If transform is kMIDITransform_MapControl, the output control type
    @field  controlNumber       The control number to be affected.
    @field  transform           The type of transformation to be applied to the event values.
    @field  param               An argument to the transformation method (see description of MIDITransformType).
}
type
	MIDIControlTransform = record
		controlType: MIDITransformControlType;
		remappedControlType: MIDITransformControlType;
		controlNumber: UInt16;
		transform: MIDITransformType;
		param: SInt16;
	end;

{!
    @struct     MIDIThruConnectionEndpoint
    @abstract   Describes a source or destination in a MIDIThruConnection.
    @field  endpointRef     The endpoint specified as a MIDIEndpointRef.
    @field  uniqueID        The endpoint specified by its uniqueID.
    @discussion
        When creating one of these, you can leave uniqueID 0 if the endpoint exists and you are passing
        its MIDIEndpointRef.
        
        When obtaining one of these from CoreMIDI, endpointRef may be NULL if it doesn't exist, but the
        uniqueID will always be non-zero.
}
type
	MIDIThruConnectionEndpoint = record
		endpointRef: MIDIEndpointRef;
		uniqueID: MIDIUniqueID;
	end;

{!
    @struct     MIDIThruConnectionParams
    @abstract   Describes a set of MIDI routings and transformations.
    @field      version     Version of this structure; must be 0.
    @field      numSources  The number of valid sources in the following array.
    @field      sources     All MIDI generated by these sources is routed into this connection for processing
                            and distribution to destinations.
    @field      numDestinations The number of valid destinations in the following array.
    @field      destinations    All MIDI output from the connection is routed to these destinations.
    @field      channelMap      Maps each of the source 16 MIDI channels to channel 0-15 (1-16) or 0xFF when
                                MIDI from a channel is to be filtered out.
    @field      lowVelocity     Note events with a velocity less than this value are filtered out.
    @field      highVelocity    Note events with a velocity greater than this, if it is not 0, are filtered out.
    @field      lowNote         See highNote.
    @field      highNote        If highNote >= lowNote, then notes outside this range are filtered out.
                                If lowNote > highNote, then notes <i>inside</i> this range are filtered out.
                                This applies to note and polyphonic key pressure events.
                                These fields are ignored if a there is a MIDIValueMap applying to noteNumber.
    @field      noteNumber      Specifies how MIDI note numbers are transformed.
    @field      velocity        Specifies how MIDI note velocities are transformed.
    @field      keyPressure     Specifies how MIDI polyphonic key pressure events are transformed.
    @field      channelPressure Specifies how MIDI monophonic (channel) pressure events are transformed.
    @field      programChange   Specifies how MIDI program change events are transformed.
    @field      pitchBend       Specifies how MIDI pitch bend events are transformed.
    @field      filterOutSysEx  If 1, specifies that system-exclusive messages are to be filtered out.
    @field      filterOutMTC    If 1, specifies that MIDI Time Code messages are to be filtered out.
    @field      filterOutBeatClock  If 1, specifies the MIDI clock, play, stop, and resume messages are to
                                be filtered out.
    @field      filterOutTuneRequest    If 1, specifies that MIDI Tune Request messages are to be filtered out.
    @field      reserved2       Must be 0.
    @field      filterOutAllControls    If 1, specifies that all MIDI continuous control messages are to be filtered out.
    @field      numControlTransforms    The number of control transformations in the variable-length portion of the struct.
    @field      numMaps                 The number of MIDIValueMaps in the variable-length portion of the struct.
    @field      reserved3       Must be 0.
    
    @discussion
        The remainder of the structure is variably-sized. It contains numControlTransform instances of 
        MIDIControlTransform, followed by numMaps instances of MIDIValueMap.
}
type
	MIDIThruConnectionParams = record
		version: UInt32;
		numSources: UInt32;
        sources: array[0..kMIDIThruConnection_MaxEndpoints-1] of MIDIThruConnectionEndpoint;
		numDestinations: UInt32;
    	destinations: array[0..kMIDIThruConnection_MaxEndpoints-1] of MIDIThruConnectionEndpoint;
    
        channelMap: packed array[0..15] of UInt8;
		lowVelocity, highVelocity: UInt8;
		lowNote, highNote: UInt8;
		noteNumber: MIDITransform;
		velocity: MIDITransform;
		keyPressure: MIDITransform;
		channelPressure: MIDITransform;
		programChange: MIDITransform;
		pitchBend: MIDITransform;
    
		filterOutSysEx: UInt8;
		filterOutMTC: UInt8;
		filterOutBeatClock: UInt8;
		filterOutTuneRequest: UInt8;
    	reserved2: packed array[0..2] of UInt8;
		filterOutAllControls: UInt8;
    
		numControlTransforms: UInt16;
		numMaps: UInt16;
        reserved3: array[0..3] of UInt16;

    // remainder of structure is variable-length:
    //      MIDIControlTransform    controls[];
    //      MIDIValueMap            maps[];
	end;
	MIDIThruConnectionParamsPtr = ^MIDIThruConnectionParams;
	
{!
    @defined    MIDIThruConnectionParamsSize
    @abstract   Returns the size of a MIDIThruConnectionParams.
    @discussion Accounts for the variable-length elements in the structure and returns its true
                size in bytes.
}

function MIDIThruConnectionParamsSize(ptr : MIDIThruConnectionParamsPtr) : size_t; inline;

{!
    @function   MIDIThruConnectionParamsInitialize
    @abstract   Fills a MIDIThruConnectionParams with default values.
    @param      inConnectionParams  The struct to be initialized.
    @result     An OSStatus result code.
    @discussion
        This convenience function fills the connection structure with default values:
no endpoints, no transformations (mostly zeroes except for the channel map).
Then, just filling in the source and adding one destination will create a simple, 
unmodified thru connection.
}
procedure MIDIThruConnectionParamsInitialize( var inConnectionParams: MIDIThruConnectionParams ); external name '_MIDIThruConnectionParamsInitialize';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{!
    @function   MIDIThruConnectionCreate
    @abstract   Creates a thru connection.
    @param      inPersistentOwnerID
                    If null, then the connection is marked as owned by the client
                    and will be automatically disposed with the client.  if it is non-null, then it
                    should be a unique identifier, e.g. "com.mycompany.MyCoolProgram".
    @param      inConnectionParams
                    A MIDIThruConnectionParams contained in a CFDataRef.
    @param      outConnection
                    On successful return, a reference to the newly-created connection.
    @result     An OSStatus result code.
    @discussion
}
function MIDIThruConnectionCreate( inPersistentOwnerID: CFStringRef; inConnectionParams: CFDataRef; var outConnection: MIDIThruConnectionRef ): OSStatus; external name '_MIDIThruConnectionCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{!
    @function   MIDIThruConnectionDispose
    @abstract   Disposes a thru connection.
    @param      connection
                    The connection to be disposed
    @result     An OSStatus result code.
    @discussion
}
function MIDIThruConnectionDispose( connection: MIDIThruConnectionRef ): OSStatus; external name '_MIDIThruConnectionDispose';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{!
    @function   MIDIThruConnectionGetParams
    @abstract   Obtains a thru connection's MIDIThruConnectionParams.
    @param      connection
                    The connection to be disposed.
    @param      outConnectionParams
                    On successful return, the connection's MIDIThruConnectionParams in a CFDataRef
    @result     An OSStatus result code.
    @discussion
        The returned CFDataRef contains a MIDIThruConnectionParams structure. The caller is responsible
        for releasing it.
}
function MIDIThruConnectionGetParams( connection: MIDIThruConnectionRef; var outConnectionParams: CFDataRef ): OSStatus; external name '_MIDIThruConnectionGetParams';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{!
    @function   MIDIThruConnectionSetParams
    @abstract   Alters a thru connection's MIDIThruConnectionParams.
    @param      connection
                    The connection to be modified.
    @param      inConnectionParams
                    The connection's new MIDIThruConnectionParams in a CFDataRef
    @result     An OSStatus result code.
    @discussion
}
function MIDIThruConnectionSetParams( connection: MIDIThruConnectionRef; inConnectionParams: CFDataRef ): OSStatus; external name '_MIDIThruConnectionSetParams';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{!
    @function   MIDIThruConnectionFind
    @abstract   Returns all of the persistent thru connections created by a client.
    @param      inPersistentOwnerID
                    The ID of the owner whose connections are to be returned.
    @param      outConnectionList
                    On successful return, a CFDataRef containing an array of MIDIThruConnectionRef's.
    @result     An OSStatus result code.
    @discussion
}
function MIDIThruConnectionFind( inPersistentOwnerID: CFStringRef; var outConnectionList: CFDataRef ): OSStatus; external name '_MIDIThruConnectionFind';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{$endc} { TARGET_OS_MAC }
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation
{$ifc TARGET_OS_MAC}


function MIDIThruConnectionParamsSize(ptr : MIDIThruConnectionParamsPtr) : size_t; inline;
begin
	MIDIThruConnectionParamsSize := sizeof(MIDIThruConnectionParams) +
									(ptr^.numControlTransforms * sizeof(MIDIControlTransform)) +
									(ptr^.numMaps * sizeof(MIDIValueMap))
end;
{$endc} { TARGET_OS_MAC }
end.

{$endc} {not MACOSALLINCLUDE}
