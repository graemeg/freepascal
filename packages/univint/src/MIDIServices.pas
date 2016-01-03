{
 	File:   	CoreMIDI/MIDIServices.h
 
 	Contains:   API for communicating with MIDI hardware
 
 	Copyright:  (c) 2000-2008 by Apple Inc., all rights reserved.
 
 	Bugs?:  	For bug reports, consult the following page on
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

unit MIDIServices;
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
uses MacTypes,CFBase,CFData,CFDictionary;
{$endc} {not MACOSALLINCLUDE}

{$ifc TARGET_OS_MAC}
{$ALIGN POWER}

{!
	@header MIDIServices.h

	This is the header file for Mac OS X's MIDI system services.

<h2>API Overview</h2>
	Drivers own and control Devices, e.g. USB interfaces, PCI cards, etc.  A device is defined
	as a physical object that would be represented by a single icon if there were a graphical
	view of the studio.

	Devices may have multiple logically distinct sub-components, e.g. a MIDI synthesizer and a
	pair of MIDI ports, both addressable via a USB port. These are called Entities.

	Entities have any number of Endpoints, sources and destinations of 16-channel MIDI streams. 
	By grouping a device's endpoints into entities, the system has enough information for an
	application to make reasonable default assumptions about how to communicate in a
	bi-directional manner with each entity, as is necessary in MIDI librarian applications.

	CoreMIDI attaches a collection of properties to each object it manages. Some properties are
	dynamic characteristics of a device (e.g. MIDI receive channel and system-exclusive ID's),
	and some are a matter of user preference (choice of icon, whether the device should appear
	in lists of possible controllers). Other properties are static and could be looked up in a
	database, using the device's manufacturer and model names as a key.

<h2>Implementation overview</h2>
	The client API is implemented as the CoreMIDI framework, which uses IPC to communicate with
	a server process, MIDIServer.

	The server process loads, and manages all communication with, MIDI drivers.

	"Drivers" are not I/O Kit drivers.  They are bundles containing CFPlugins.

	Many MIDI drivers can simply be user-side I/O Kit clients (probably for serial, USB,
	Firewire).

	PCI card drivers will need their MIDI drivers to communicate with a separate kernel
	extension.

<h2>Note about CoreFoundation data types (CFString, CFData, CFDictionary)</h2>
	When passing a CF object to a MIDI function, the MIDI function will never consume a
	reference to the object; the caller always retains a reference which it is responsible for
	releasing with CFRelease().

	When receiving a CF object as a return value from a MIDI function, the caller always
	receives a new reference to the object, and is responsible for releasing it.
}

//=============================================================================
//#pragma mark    Includes


//=============================================================================
//#pragma mark	Error Constants
{!
    @enum           Error Constants
    @abstract       The error constants unique to Core MIDI.
    @discussion     These are the error constants that are unique to Core MIDI. Note that Core MIDI
                    functions may return other codes that are not listed here.
	
	@constant		kMIDIInvalidClient
						An invalid MIDIClientRef was passed.
	@constant		kMIDIInvalidPort
						An invalid MIDIPortRef was passed.
	@constant		kMIDIWrongEndpointType
						A source endpoint was passed to a function expecting a destination, or vice versa.
	@constant		kMIDINoConnection
						Attempt to close a non-existant connection.
	@constant		kMIDIUnknownEndpoint
						An invalid MIDIEndpointRef was passed.
	@constant		kMIDIUnknownProperty
						Attempt to query a property not set on the object.
	@constant		kMIDIWrongPropertyType
						Attempt to set a property with a value not of the correct type.
	@constant		kMIDINoCurrentSetup
						Internal error; there is no current MIDI setup object.
	@constant		kMIDIMessageSendErr
						Communication with MIDIServer failed.
	@constant		kMIDIServerStartErr
						Unable to start MIDIServer.
	@constant		kMIDISetupFormatErr
						Unable to read the saved state.
	@constant		kMIDIWrongThread
						A driver is calling a non-I/O function in the server from a thread other than
						the server's main thread.
	@constant		kMIDIObjectNotFound
						The requested object does not exist.
	@constant		kMIDIIDNotUnique
						Attempt to set a non-unique kMIDIPropertyUniqueID on an object.
}
const
	kMIDIInvalidClient = -10830;
	kMIDIInvalidPort = -10831;
	kMIDIWrongEndpointType = -10832;
	kMIDINoConnection = -10833;
	kMIDIUnknownEndpoint = -10834;
	kMIDIUnknownProperty = -10835;
	kMIDIWrongPropertyType = -10836;
	kMIDINoCurrentSetup = -10837;
	kMIDIMessageSendErr = -10838;
	kMIDIServerStartErr = -10839;
	kMIDISetupFormatErr = -10840;
	kMIDIWrongThread = -10841;
	kMIDIObjectNotFound = -10842;
	kMIDIIDNotUnique = -10843;

//=============================================================================
//#pragma mark	Types

//#if __LP64__
{$ifc TARGET_CPU_64}
{!
	@typedef		MIDIObjectRef
	@abstract		The base class of many CoreMIDI objects.
	@discussion
		MIDIObject is the base class for many of the objects in CoreMIDI.  They have properties,
		and often an "owner" object, from which they inherit any properties they do not
		themselves have.

		Developers may add their own private properties, whose names must begin with their
		company's inverted domain name, as in Java package names, but with underscores instead
		of dots, e.g.: com_apple_APrivateAppleProperty
}
type
	MIDIObjectRef = UInt32;

{!
	@typedef		MIDIClientRef
	@abstract		An object maintaining per-client state.
	@discussion
		Derives from MIDIObjectRef, does not have an owner object.

		To use CoreMIDI, an application creates a MIDIClientRef, to which it can add
		MIDIPortRef's, through which it can send and receive MIDI.
}
type
	MIDIClientRef = MIDIObjectRef;

{!
	@typedef		MIDIPortRef
	@abstract		A MIDI connection port owned by a client.
	@discussion
		Derives from MIDIObjectRef, owned by a MIDIClientRef.

		A MIDIPortRef, which may be an input port or output port, is an object through which a
		client may communicate with any number of MIDI sources or destinations.
}
type
	MIDIPortRef = MIDIObjectRef;

{!
	@typedef		MIDIDeviceRef
	@abstract		A MIDI device or external device, containing entities.
	@discussion
		Derives from MIDIObjectRef, does not have an owner object.

		A MIDI device, which either attaches directly to the computer and is controlled by a
		MIDI driver, or which is "external," meaning that it is connected to a driver-controlled
		device via a standard MIDI cable.

		A MIDIDeviceRef has properties and contains MIDIEntityRef's.
}
type
	MIDIDeviceRef = MIDIObjectRef;

{!
	@typedef		MIDIEntityRef
	@abstract		A MIDI entity, owned by a device, containing endpoints.
	@discussion
		Derives from MIDIObjectRef, owned by a MIDIDeviceRef.

		Devices may have multiple logically distinct sub-components, e.g. a MIDI synthesizer and
		a pair of MIDI ports, both addressable via a USB port.

		By grouping a device's endpoints into entities, the system has enough information for an
		application to make reasonable assumptions about how to communicate in a bi-directional
		manner with each entity, as is desirable in MIDI librarian applications.

		These sub-components are MIDIEntityRef's.
}
type
	MIDIEntityRef = MIDIObjectRef;

{!
	@typedef		MIDIEndpointRef
	@abstract		A MIDI source or destination, owned by an entity.
	@discussion
		Derives from MIDIObjectRef, owned by a MIDIEntityRef, unless it is a virtual endpoint,
		in which case there is no owning entity.

		Entities have any number of MIDIEndpointRef's, sources and destinations of 16-channel
		MIDI streams.
}
type
	MIDIEndpointRef = MIDIObjectRef;
{$elsec}
type
	MIDIObjectRef = UnivPtr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	MIDIClientRef = ^OpaqueMIDIClient; { an opaque type }
	OpaqueMIDIClient = record end;
	MIDIPortRef = ^OpaqueMIDIPort; { an opaque type }
	OpaqueMIDIPort = record end;
	MIDIDeviceRef = ^OpaqueMIDIDevice; { an opaque type }
	OpaqueMIDIDevice = record end;
	MIDIEntityRef = ^OpaqueMIDIEntity; { an opaque type }
	OpaqueMIDIEntity = record end;
	MIDIEndpointRef = ^OpaqueMIDIEndpoint; { an opaque type }
	OpaqueMIDIEndpoint = record end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	MIDIClientRef = ^SInt32; { an opaque type }
	MIDIPortRef = ^SInt32; { an opaque type }
	MIDIDeviceRef = ^SInt32; { an opaque type }
	MIDIEntityRef = ^SInt32; { an opaque type }
	MIDIEndpointRef = ^SInt32; { an opaque type }
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
{$endc} { TARGET_CPU_64 }


{!
	@typedef		MIDITimeStamp
	@abstract		A host clock time.
	@discussion
		A host clock time representing the time of an event, as returned by
		mach_absolute_time() or UpTime().

		Since MIDI applications will tend to do a fair amount of math with the times of events,
		it's more convenient to use a UInt64 than an AbsoluteTime.
		
		See CoreAudio/HostTime.h.
}
type
	MIDITimeStamp = UInt64;

{!
	@enum			MIDIObjectType
	@abstract		Signifies the type of a MIDIObject.
	@discussion
		Signifies the real type of a MIDIObjectRef instance.
}
const
// MIDIObjectType
	kMIDIObjectType_Other = -1;
	kMIDIObjectType_Device = 0;
	kMIDIObjectType_Entity = 1;
	kMIDIObjectType_Source = 2;
	kMIDIObjectType_Destination = 3;
	kMIDIObjectType_ExternalMask = $10;
	kMIDIObjectType_ExternalDevice = kMIDIObjectType_ExternalMask or kMIDIObjectType_Device;
	kMIDIObjectType_ExternalEntity = kMIDIObjectType_ExternalMask or kMIDIObjectType_Entity;
	kMIDIObjectType_ExternalSource = kMIDIObjectType_ExternalMask or kMIDIObjectType_Source;
	kMIDIObjectType_ExternalDestination = kMIDIObjectType_ExternalMask or kMIDIObjectType_Destination;
type
	MIDIObjectType = SInt32;

{!
	@typedef		MIDIUniqueID
	@abstract		A unique identifier for a MIDIObjectRef.
	@discussion
		An integer which uniquely identifies a MIDIObjectRef.
}
type
	MIDIUniqueID = SInt32;

const
	kMIDIInvalidUniqueID = 0;


// forward structure declarations
type
	MIDIPacketListPtr = ^MIDIPacketList;
	MIDISysexSendRequestPtr = ^MIDISysexSendRequest;
	MIDINotificationPtr = ^MIDINotification;

//=============================================================================
//#pragma mark	Callback Functions

{!
	@typedef		MIDINotifyProc
	@abstract		A callback function for notifying clients of state changes.
	@discussion
		This callback function is called when some aspect of the current MIDI setup changes. It
		is called on the runloop (thread) on which MIDIClientCreate was first called.

	@param			message	
						A structure containing information about what changed.
	@param			refCon
						The client's refCon passed to MIDIClientCreate.
}

	MIDINotifyProc = procedure( (*const*) message: MIDINotificationPtr; refCon: UnivPtr );

{!
	@typedef		MIDIReadProc
	@abstract		A function receiving MIDI input.
	@discussion
		This is a callback function through which a client receives incoming MIDI messages.

		A MIDIReadProc function pointer is passed to the MIDIInputPortCreate and
		MIDIDestinationCreate functions.  The CoreMIDI framework will create a high-priority
		receive thread on your client's behalf, and from that thread, your MIDIReadProc will be
		called when incoming MIDI messages arrive. Because this function is called from a
		separate thread, be aware of the synchronization issues when accessing data in this
		callback.

	@param			pktlist
						The incoming MIDI message(s).
	@param			readProcRefCon	
						The refCon you passed to MIDIInputPortCreate or
						MIDIDestinationCreate
	@param			srcConnRefCon
						A refCon you passed to MIDIPortConnectSource, which
						identifies the source of the data.
}

	MIDIReadProc = procedure( (*const*) pktlist: MIDIPacketListPtr; readProcRefCon: UnivPtr; srcConnRefCon: UnivPtr );

{!
	@typedef		MIDICompletionProc
	@abstract		A function called when a system-exclusive event has been completely sent.
	@discussion
		Callback function to notify the client of the completion of a call to MIDISendSysex.
	
	@param			request
						The MIDISysexSendRequest which has completed, or been
						aborted.
}

	MIDICompletionProc = procedure( request: MIDISysexSendRequestPtr );

//=============================================================================
//#pragma mark	Structures

//#pragma pack(push, 4)
{!
	@struct			MIDIPacket
	@abstract		A collection of simultaneous MIDI events.
	
	@field			timeStamp
						The time at which the events occurred, if receiving MIDI,
						or, if sending MIDI, the time at which the events are to
						be played.  Zero means "now."  The time stamp applies
						to the first MIDI byte in the packet.
	@field			length		
						The number of valid MIDI bytes which follow, in data. (It
						may be larger than 256 bytes if the packet is dynamically
						allocated.)
	@field			data
						A variable-length stream of MIDI messages.  Running status
						is not allowed.  In the case of system-exclusive
						messages, a packet may only contain a single message, or
						portion of one, with no other MIDI events.
						
						The MIDI messages in the packet must always be complete,
						except for system-exclusive.

						(This is declared to be 256 bytes in length so clients
						don't have to create custom data structures in simple
						situations.)
}

	MIDIPacket = record
		timeStamp: MIDITimeStamp;
		length: UInt16;
		data: packed array [0..255] of Byte;
	end;
	MIDIPacketPtr = ^MIDIPacket;
	
{!
	@struct			MIDIPacketList
	@abstract		A list of MIDI events being received from, or being sent to,
					one endpoint.
	@discussion
					The timestamps in the packet list must be in ascending order.
					
					Note that the packets in the list, while defined as an array, may not be
					accessed as an array, since they are variable-length.  To iterate through
					the packets in a packet list, use a loop such as:
<pre>
@textblock
  MIDIPacket *packet = &packetList->packet[0];
  for (int i = 0; i < packetList->numPackets; ++i) (
    ...
    packet = MIDIPacketNext(packet);
  )
@/textblock
</pre>
	
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					The MIDIPacketNext macro is especially important when considering that
					the alignment requirements of MIDIPacket may differ between CPU architectures.
					On Intel and PowerPC, MIDIPacket is unaligned. 
					
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
	@field			numPackets
						The number of MIDIPackets in the list.
	@field			packet
						An open-ended array of variable-length MIDIPackets.
}

	MIDIPacketList = record
		numPackets: UInt32;	
		packet: array [0..0] of MIDIPacket;
	end;
{$ALIGN POWER}

{!
	@struct			MIDISysexSendRequest
	@abstract		A request to transmit a system-exclusive event.
	
	@field			destination
						The endpoint to which the event is to be sent.
	@field			data
						Initially, a pointer to the sys-ex event to be sent.
						MIDISendSysex will advance this pointer as bytes are
						sent.
	@field			bytesToSend
						Initially, the number of bytes to be sent.  MIDISendSysex
						will decrement this counter as bytes are sent.
	@field			complete
						The client may set this to true at any time to abort
						transmission.  The implementation sets this to true when
						all bytes have been sent.
	@field			completionProc
						Called when all bytes have been sent, or after the client
						has set complete to true.
	@field			completionRefCon
						Passed as a refCon to completionProc.

	@discussion
		This represents a request to send a single system-exclusive MIDI event to
		a MIDI destination asynchronously.
}

	MIDISysexSendRequest = record
		destination: MIDIEndpointRef;
		data: (*const*) BytePtr;
		bytesToSend: UInt32;
		complete: Boolean;
		reserved: packed array [0..2] of Byte;
		completionProc: MIDICompletionProc;
		completionRefCon: UnivPtr;
	end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{ moved here to make type available in MIDINotification }
	MIDINotificationMessageID = SInt32;
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
	@struct			MIDINotification
	@abstract		A message describing a system state change.
	@discussion
		A MIDINotification is a structure passed to a MIDINotifyProc, when CoreMIDI wishes to
		inform a client of a change in the state of the system.
	
	@field			messageID
						type of message
	@field			messageSize
						size of the entire message, including messageID and
						messageSize
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

	MIDINotificationMessageID = SInt32;

>>>>>>> graemeg/cpstrnew
=======

	MIDINotificationMessageID = SInt32;

>>>>>>> graemeg/cpstrnew
=======

	MIDINotificationMessageID = SInt32;

>>>>>>> graemeg/cpstrnew
=======

	MIDINotificationMessageID = SInt32;

>>>>>>> origin/cpstrnew
=======

	MIDINotificationMessageID = SInt32;

>>>>>>> origin/fixes_2.4
	MIDINotification = record
		messageID: MIDINotificationMessageID;
		messageSize: UInt32;
	// additional data may follow, depending on messageID
	end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

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
	@enum		MIDINotificationMessageID
	@abstract	Signifies the type of a MIDINotification.

	@constant	kMIDIMsgSetupChanged	Some aspect of the current MIDISetup
										has changed.  No data.  Should ignore this
										message if messages 2-6 are handled.
	@constant	kMIDIMsgObjectAdded		A device, entity or endpoint was added.
										Structure is MIDIObjectAddRemoveNotification.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
										New in Mac OS X 10.2.
	@constant	kMIDIMsgObjectRemoved	A device, entity or endpoint was removed.
										Structure is MIDIObjectAddRemoveNotification.
										New in Mac OS X 10.2.
	@constant	kMIDIMsgPropertyChanged	An object's property was changed.
										Structure is MIDIObjectPropertyChangeNotification.
										New in Mac OS X 10.2.
	@constant	kMIDIMsgThruConnectionsChanged	A persistent MIDI Thru connection was created
										or destroyed.  No data.  New in Mac OS X 10.2.
	@constant	kMIDIMsgSerialPortOwnerChanged	A persistent MIDI Thru connection was created
										or destroyed.  No data.  New in Mac OS X 10.2.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
										New for CoreMIDI 1.3.
	@constant	kMIDIMsgObjectRemoved	A device, entity or endpoint was removed.
										Structure is MIDIObjectAddRemoveNotification.
										New for CoreMIDI 1.3.
	@constant	kMIDIMsgPropertyChanged	An object's property was changed.
										Structure is MIDIObjectPropertyChangeNotification.
										New for CoreMIDI 1.3.
	@constant	kMIDIMsgThruConnectionsChanged	A persistent MIDI Thru connection was created
										or destroyed.  No data.  New for CoreMIDI 1.3.
	@constant	kMIDIMsgSerialPortOwnerChanged	A persistent MIDI Thru connection was created
										or destroyed.  No data.  New for CoreMIDI 1.3.
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
	@constant	kMIDIMsgIOError			A driver I/O error occurred.
}
const
// MIDINotificationMessageID
	kMIDIMsgSetupChanged = 1;
	kMIDIMsgObjectAdded = 2;
	kMIDIMsgObjectRemoved = 3;
	kMIDIMsgPropertyChanged = 4;
	kMIDIMsgThruConnectionsChanged = 5;
	kMIDIMsgSerialPortOwnerChanged = 6;
	kMIDIMsgIOError = 7;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
	@struct			MIDIObjectAddRemoveNotification
	@abstract		A message describing the addition or removal of an object.
	
	@field			messageID
						type of message
	@field			messageSize
						size of the entire message, including messageID and messageSize
	@field			parent
						the parent of the added or removed object (possibly NULL)
	@field			parentType
						the type of the parent object (undefined if parent is NULL)
	@field			child
						the added or removed object
	@field			childType
						the type of the added or removed object
}
type
	MIDIObjectAddRemoveNotification = record
		messageID: MIDINotificationMessageID;
		messageSize: UInt32;
		parent: MIDIObjectRef;
		parentType: MIDIObjectType;
		child: MIDIObjectRef;
		childType: MIDIObjectType;
	end;

{!
	@struct			MIDIObjectPropertyChangeNotification
	@abstract		A message describing the addition or removal of an object.
	
	@field			messageID
						type of message
	@field			messageSize
						size of the entire message, including messageID and messageSize
	@field			object
						the object whose property has changed
	@field			objectType
						the type of the object whose property has changed
	@field			propertyName
						the name of the changed property
}
type
	MIDIObjectPropertyChangeNotification = record
		messageID: MIDINotificationMessageID;
		messageSize: UInt32;
		objct: MIDIObjectRef;
		objectType: MIDIObjectType;
		propertyName: CFStringRef;
	end;

type
	MIDIIOErrorNotification = record
		messageID: MIDINotificationMessageID;
		messageSize: UInt32;
		driverDevice: MIDIDeviceRef;
		errorCode: OSStatus;
	end;

//=============================================================================
//	Property name constants
//=============================================================================

{!
	@constant		kMIDIPropertyName
	@discussion
		device/entity/endpoint property, string
	
		Devices, entities, and endpoints may all have names.  The recommended way to display an
		endpoint's name is to ask for the endpoint name, and display only that name if it is
		unique.  If it is non-unique, prepend the device name.

		A setup editor may allow the user to set the names of both driver-owned and external
		devices.
}
var kMIDIPropertyName: CFStringRef; external name '_kMIDIPropertyName'; (* attribute const *)		
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
	@constant		kMIDIPropertyManufacturer
	@discussion
		device/endpoint property, string

		Drivers should set this property on their devices.

		Setup editors may allow the user to set this property on external devices.

		Creators of virtual endpoints may set this property on their endpoints.
}
var kMIDIPropertyManufacturer: CFStringRef; external name '_kMIDIPropertyManufacturer'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
	@constant		kMIDIPropertyModel
	@discussion
		device/endpoint property, string

		Drivers should set this property on their devices.

		Setup editors may allow the user to set this property on external devices.

		Creators of virtual endpoints may set this property on their endpoints.
}
var kMIDIPropertyModel: CFStringRef; external name '_kMIDIPropertyModel'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
	@constant		kMIDIPropertyUniqueID
	@discussion
		devices, entities, endpoints all have unique ID's, integer

		The system assigns unique ID's to all objects.  Creators of virtual endpoints may set
		this property on their endpoints, though doing so may fail if the chosen ID is not
		unique.
}
var kMIDIPropertyUniqueID: CFStringRef; external name '_kMIDIPropertyUniqueID'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
	@constant		kMIDIPropertyDeviceID
	@discussion
		device/entity property, integer

		The entity's system-exclusive ID, in user-visible form

		Drivers may set this property on their devices or entities.

		Setup editors may allow the user to set this property on external devices.
}
var kMIDIPropertyDeviceID: CFStringRef; external name '_kMIDIPropertyDeviceID'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
	@constant		kMIDIPropertyReceiveChannels
	@discussion
		endpoint property, integer

		The value is a bitmap of channels on which the object receives: 1=ch 1, 2=ch 2, 4=ch 3
		... 0x8000=ch 16.

		Drivers may set this property on their entities or endpoints.

		Setup editors may allow the user to set this property on external endpoints.

		Virtual destination may set this property on their endpoints.
}
var kMIDIPropertyReceiveChannels: CFStringRef; external name '_kMIDIPropertyReceiveChannels'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
	@constant		kMIDIPropertyTransmitChannels
	@discussion
		endpoint property, integer

		The value is a bitmap of channels on which the object transmits: 1=ch 1, 2=ch 2, 4=ch 3
		... 0x8000=ch 16.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyTransmitChannels: CFStringRef; external name '_kMIDIPropertyTransmitChannels'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyTransmitChannels: CFStringRef; external name '_kMIDIPropertyTransmitChannels'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyMaxSysExSpeed
	@discussion
		device/entity/endpoint property, integer

		Set by the owning driver; should not be touched by other clients.
		The value is the maximum rate, in bytes/second, at which sysex messages may
		be sent reliably to this object. (The default value is 3125, as with MIDI 1.0)
}
var kMIDIPropertyMaxSysExSpeed: CFStringRef; external name '_kMIDIPropertyMaxSysExSpeed'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
	@constant		kMIDIPropertyAdvanceScheduleTimeMuSec
	@discussion
		device/entity/endpoint property, integer

		Set by the owning driver; should not be touched by other clients. If it is non-zero,
		then it is a recommendation of how many microseconds in advance clients should schedule
		output. Clients should treat this value as a minimum.  For devices with a non-zero
		advance schedule time, drivers will receive outgoing messages to the device at the time
		they are sent by the client, via MIDISend, and the driver is responsible for scheduling
		events to be played at the right times according to their timestamps.

		As of CoreMIDI 1.3, this property may also be set on virtual destinations (but only the
		creator of the destination should do so). When a client sends to a virtual destination
		with an advance schedule time of 0, the virtual destination receives its messages at
		their scheduled delivery time.  If a virtual destination has a non-zero advance schedule
		time, it receives timestamped messages as soon as they are sent, and must do its own
		internal scheduling of received events.
}
var kMIDIPropertyAdvanceScheduleTimeMuSec: CFStringRef; external name '_kMIDIPropertyAdvanceScheduleTimeMuSec'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
	@constant		kMIDIPropertyIsEmbeddedEntity
	@discussion
		entity/endpoint property, integer

		0 if there are external MIDI connectors, 1 if not.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyIsEmbeddedEntity: CFStringRef; external name '_kMIDIPropertyIsEmbeddedEntity'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.1 (Mac OS X 10.1)
}
var kMIDIPropertyIsEmbeddedEntity: CFStringRef; external name '_kMIDIPropertyIsEmbeddedEntity'; (* attribute const *)
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


{!
	@constant		kMIDIPropertyIsBroadcast
	@discussion
		entity/endpoint property, integer

		1 if the endpoint broadcasts messages to all of the other endpoints in the device, 0 if
		not.  Set by the owning driver; should not be touched by other clients.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyIsBroadcast: CFStringRef; external name '_kMIDIPropertyIsBroadcast'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyIsBroadcast: CFStringRef; external name '_kMIDIPropertyIsBroadcast'; (* attribute const *)
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

{!
	@constant		kMIDIPropertySingleRealtimeEntity
	@discussion
		device property, integer

		Some MIDI interfaces cannot route MIDI realtime messages to individual outputs; they are
		broadcast.  On such devices the inverse is usually also true -- incoming realtime
		messages cannot be identified as originating from any particular source.

		When this property is set on a driver device, it signifies the 0-based index of the
		entity on which incoming realtime messages from the device will appear to have
		originated from.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertySingleRealtimeEntity: CFStringRef; external name '_kMIDIPropertySingleRealtimeEntity'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertySingleRealtimeEntity: CFStringRef; external name '_kMIDIPropertySingleRealtimeEntity'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyConnectionUniqueID
	@discussion
		device/entity/endpoint property, integer or CFDataRef

		UniqueID of an external device/entity/endpoint attached to this one. As of Mac OS X
		10.3, Audio MIDI Setup maintains endpoint-to-external endpoint connections (in 10.2, it
		connected devices to devices).

		The property is non-existant or 0 if there is no connection.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		Beginning with CoreMIDI 1.3 (Mac OS X 10.2), this property may also be a CFDataRef containing an array of
=======
		New for CoreMIDI 1.1 (Mac OS X 10.1)
		
		Beginning with CoreMIDI 1.3, this property may also be a CFDataRef containing an array of
>>>>>>> graemeg/cpstrnew
=======
		New for CoreMIDI 1.1 (Mac OS X 10.1)
		
		Beginning with CoreMIDI 1.3, this property may also be a CFDataRef containing an array of
>>>>>>> graemeg/cpstrnew
=======
		New for CoreMIDI 1.1 (Mac OS X 10.1)
		
		Beginning with CoreMIDI 1.3, this property may also be a CFDataRef containing an array of
>>>>>>> graemeg/cpstrnew
=======
		New for CoreMIDI 1.1 (Mac OS X 10.1)
		
		Beginning with CoreMIDI 1.3, this property may also be a CFDataRef containing an array of
>>>>>>> origin/cpstrnew
=======
		New for CoreMIDI 1.1 (Mac OS X 10.1)
		
		Beginning with CoreMIDI 1.3, this property may also be a CFDataRef containing an array of
>>>>>>> origin/fixes_2.4
		big-endian SInt32's, to allow specifying that a driver object connects to multiple
		external objects (via MIDI thru-ing or splitting).

		This property may also exist for external devices/entities/endpoints, in which case it
		signifies a MIDI Thru connection to another external device/entity/endpoint (again,
		it is strongly recommended that it be an endpoint).
}
var kMIDIPropertyConnectionUniqueID: CFStringRef; external name '_kMIDIPropertyConnectionUniqueID'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)
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
	@constant		kMIDIPropertyOffline
	@discussion
		device/entity/endpoint property, integer

		1 = device is offline (is temporarily absent), 0 = present. Set by the owning driver, on
		the device; should not be touched by other clients. Property is inherited from the
		device by its entities and endpoints.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyOffline: CFStringRef; external name '_kMIDIPropertyOffline'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.1 (Mac OS X 10.1)
}
var kMIDIPropertyOffline: CFStringRef; external name '_kMIDIPropertyOffline'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyPrivate
	@discussion
		device/entity/endpoint property, integer

		1 = endpoint is private, hidden from other clients. May be set on a device or entity,
		but they will still appear in the API; only affects whether the owned endpoints are
		hidden.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyPrivate: CFStringRef; external name '_kMIDIPropertyPrivate'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyPrivate: CFStringRef; external name '_kMIDIPropertyPrivate'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyDriverOwner
	@discussion
		device/entity/endpoint property, string

		Name of the driver that owns a device. Set by the owning driver, on the device; should
		not be touched by other clients. Property is inherited from the device by its entities
		and endpoints.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyDriverOwner: CFStringRef; external name '_kMIDIPropertyDriverOwner'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.1 (Mac OS X 10.1)
}
var kMIDIPropertyDriverOwner: CFStringRef; external name '_kMIDIPropertyDriverOwner'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyFactoryPatchNameFile
	@discussion
		device/entity/endpoint property, CFData containing AliasHandle.

		An alias to the device's current factory patch name file.

		Added in CoreMIDI 1.1 (Mac OS X 10.1).  DEPRECATED as of CoreMIDI 1.3. Use
		kMIDIPropertyNameConfiguration instead.
}
var kMIDIPropertyFactoryPatchNameFile: CFStringRef; external name '_kMIDIPropertyFactoryPatchNameFile'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_2, __IPHONE_NA, __IPHONE_NA) *)
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
	@constant		kMIDIPropertyUserPatchNameFile
	@discussion
		device/entity/endpoint property, CFData containing AliasHandle

		An alias to the device's current user patch name file.

		Added in CoreMIDI 1.1 (Mac OS X 10.1).  DEPRECATED as of CoreMIDI 1.3. Use
		kMIDIPropertyNameConfiguration instead.
}
var kMIDIPropertyUserPatchNameFile: CFStringRef; external name '_kMIDIPropertyUserPatchNameFile'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_2, __IPHONE_NA, __IPHONE_NA) *)
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
	@constant		kMIDIPropertyNameConfiguration
	@discussion
		device/entity/endpoint property, CFDictionary

		This specifies the device's current patch, note and control name values using the
		MIDINameDocument XML format.  This specification requires the use of higher-level,
		OS-specific constructs outside of the specification, to fully define the current names
		for a device.

		The MIDINameConfiguration property is implementated as a CFDictionary:

		key "master" maps to a CFDataRef containing an AliasHandle referring to the device's
		master name document.

		key "banks" maps to a CFDictionaryRef.  This dictionary's keys are CFStringRef names of
		patchBank elements in the master document, and its values are each a CFDictionaryRef:
		key "file" maps to a CFDataRef containing an AliasHandle to a document containing
		patches that override those in the master document, and key "patchNameList" maps to a
		CFStringRef which is the name of the patchNameList element in the overriding document.

		key "currentChannelNameSets" maps to a 16-element CFArrayRef, each element of which is a
		CFStringRef of the name of the current mode for each of the 16 MIDI channels.

		key "currentDeviceMode" maps to a CFStringRef containing the name of the device's mode.

		Clients setting this property must take particular care to preserve dictionary values
		other than the ones they are interested in changing, and to properly structure the
		dictionary.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyNameConfiguration: CFStringRef; external name '_kMIDIPropertyNameConfiguration'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyNameConfiguration: CFStringRef; external name '_kMIDIPropertyNameConfiguration'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyImage
	@discussion
		device property, CFStringRef which is a full POSIX path to a device or external device's
		icon, stored in any standard graphic file format such as JPEG, GIF, PNG and TIFF are all
		acceptable.  (See CFURL for functions to convert between POSIX paths and other ways of
		specifying files.)  The image's maximum size should be 128x128.

		Drivers should set the icon on the devices they add.

		A studio setup editor should allow the user to choose icons for external devices.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyImage: CFStringRef; external name '_kMIDIPropertyImage'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyImage: CFStringRef; external name '_kMIDIPropertyImage'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyDriverVersion
	@discussion
		device/entity/endpoint property, integer, returns the driver version API of the owning
		driver (only for driver- owned devices).  Drivers need not set this property;
		applications should not write to it.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyDriverVersion: CFStringRef; external name '_kMIDIPropertyDriverVersion'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyDriverVersion: CFStringRef; external name '_kMIDIPropertyDriverVersion'; (* attribute const *)
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

{!
	@constant		kMIDIPropertySupportsGeneralMIDI
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity implements
		the General MIDI specification.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertySupportsGeneralMIDI: CFStringRef; external name '_kMIDIPropertySupportsGeneralMIDI'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertySupportsGeneralMIDI: CFStringRef; external name '_kMIDIPropertySupportsGeneralMIDI'; (* attribute const *)
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

{!
	@constant		kMIDIPropertySupportsMMC
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity implements
		the MIDI Machine Control portion of the MIDI specification.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertySupportsMMC: CFStringRef; external name '_kMIDIPropertySupportsMMC'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertySupportsMMC: CFStringRef; external name '_kMIDIPropertySupportsMMC'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyCanRoute
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity can route
		MIDI messages to or from other external MIDI devices (as with MIDI patch bays). This
		should NOT be set on devices which are controlled by drivers.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyCanRoute: CFStringRef; external name '_kMIDIPropertyCanRoute'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyCanRoute: CFStringRef; external name '_kMIDIPropertyCanRoute'; (* attribute const *)		// e.g. is patch bay
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

{!
	@constant		kMIDIPropertyReceivesClock
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity  responds
		to MIDI beat clock messages.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyReceivesClock: CFStringRef; external name '_kMIDIPropertyReceivesClock'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyReceivesClock: CFStringRef; external name '_kMIDIPropertyReceivesClock'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyReceivesMTC
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity responds
		to MIDI Time Code messages.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyReceivesMTC: CFStringRef; external name '_kMIDIPropertyReceivesMTC'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyReceivesMTC: CFStringRef; external name '_kMIDIPropertyReceivesMTC'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyReceivesNotes
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity responds
		to MIDI Note On messages.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyReceivesNotes: CFStringRef; external name '_kMIDIPropertyReceivesNotes'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyReceivesNotes: CFStringRef; external name '_kMIDIPropertyReceivesNotes'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyReceivesProgramChanges
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity responds
		to MIDI program change messages.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyReceivesProgramChanges: CFStringRef; external name '_kMIDIPropertyReceivesProgramChanges'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyReceivesProgramChanges: CFStringRef; external name '_kMIDIPropertyReceivesProgramChanges'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyReceivesBankSelectMSB
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity responds
		to MIDI bank select MSB messages (control 0).
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyReceivesBankSelectMSB: CFStringRef; external name '_kMIDIPropertyReceivesBankSelectMSB'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyReceivesBankSelectMSB: CFStringRef; external name '_kMIDIPropertyReceivesBankSelectMSB'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyReceivesBankSelectLSB
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity responds
		to MIDI bank select LSB messages (control 32).
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyReceivesBankSelectLSB: CFStringRef; external name '_kMIDIPropertyReceivesBankSelectLSB'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyReceivesBankSelectLSB: CFStringRef; external name '_kMIDIPropertyReceivesBankSelectLSB'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyTransmitsClock
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity transmits
		MIDI beat clock messages.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyTransmitsClock: CFStringRef; external name '_kMIDIPropertyTransmitsClock'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.3.
}
var kMIDIPropertyTransmitsClock: CFStringRef; external name '_kMIDIPropertyTransmitsClock'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyTransmitsMTC
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity transmits
		MIDI Time Code messages.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyTransmitsMTC: CFStringRef; external name '_kMIDIPropertyTransmitsMTC'; (* attribute const *)

{!
	@constant		kMIDIPropertyTransmitsNotes
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity transmits
		MIDI note messages.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyTransmitsNotes: CFStringRef; external name '_kMIDIPropertyTransmitsNotes'; (* attribute const *)

{!
	@constant		kMIDIPropertyTransmitsProgramChanges
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity transmits
		MIDI program change messages.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyTransmitsProgramChanges: CFStringRef; external name '_kMIDIPropertyTransmitsProgramChanges'; (* attribute const *)

{!
	@constant		kMIDIPropertyTransmitsBankSelectMSB
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity transmits
		MIDI bank select MSB messages (control 0).
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyTransmitsBankSelectMSB: CFStringRef; external name '_kMIDIPropertyTransmitsBankSelectMSB'; (* attribute const *)

{!
	@constant		kMIDIPropertyTransmitsBankSelectLSB
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity transmits
		MIDI bank select LSB messages (control 32).
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyTransmitsBankSelectLSB: CFStringRef; external name '_kMIDIPropertyTransmitsBankSelectLSB'; (* attribute const *)

{!
	@constant		kMIDIPropertyPanDisruptsStereo
	@discussion
		device/entity property, integer (0/1). Indicates whether the MIDI pan messages (control
		10), when sent to the device or entity, cause undesirable effects when playing stereo
		sounds (e.g. converting the signal to mono).
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyPanDisruptsStereo: CFStringRef; external name '_kMIDIPropertyPanDisruptsStereo'; (* attribute const *)

{!
	@constant		kMIDIPropertyIsSampler
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity plays
		audio samples in response to MIDI note messages.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyIsSampler: CFStringRef; external name '_kMIDIPropertyIsSampler'; (* attribute const *)

{!
	@constant		kMIDIPropertyIsDrumMachine
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity's sound
		presets tend to be collections of non-transposable samples (e.g. drum kits).
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyIsDrumMachine: CFStringRef; external name '_kMIDIPropertyIsDrumMachine'; (* attribute const *)

{!
	@constant		kMIDIPropertyIsMixer
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity mixes
		external audio signals, controlled by MIDI messages.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyIsMixer: CFStringRef; external name '_kMIDIPropertyIsMixer'; (* attribute const *)

{!
	@constant		kMIDIPropertyIsEffectUnit
	@discussion
		device/entity property, integer (0/1). Indicates whether the device or entity is
		primarily a MIDI-controlled audio effect unit (i.e. does not generate sound on its own).
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyIsEffectUnit: CFStringRef; external name '_kMIDIPropertyIsEffectUnit'; (* attribute const *)

{!
	@constant		kMIDIPropertyMaxReceiveChannels
	@discussion
		device/entity property, integer (0-16). Indicates the maximum number of MIDI channels on
		which a device may simultaneously receive MIDI Channel Messages. Common values are 0
		(devices which only respond to System Messages), 1 (non-multitimbral devices), and 16
		(fully multitimbral devices). Other values are possible, for example devices which are
		multi-timbral but have fewer than 16 "parts".
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyMaxReceiveChannels: CFStringRef; external name '_kMIDIPropertyMaxReceiveChannels'; (* attribute const *)

{!
	@constant		kMIDIPropertyMaxTransmitChannels
	@discussion
		device/entity property, integer (0/1). Indicates the maximum number of MIDI channels on
		which a device may simultaneously transmit MIDI Channel Messages. Common values are 0, 1
		and 16.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> graemeg/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/cpstrnew
=======

		New for CoreMIDI 1.3.
>>>>>>> origin/fixes_2.4
}
var kMIDIPropertyMaxTransmitChannels: CFStringRef; external name '_kMIDIPropertyMaxTransmitChannels'; (* attribute const *)

{!
	@constant		kMIDIPropertyDriverDeviceEditorApp
	@discussion
		device property, string, contains the full path to an application which knows how to
		configure this driver-owned devices. Drivers may set this property on their owned
		devices. Applications must not write to it.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyDriverDeviceEditorApp: CFStringRef; external name '_kMIDIPropertyDriverDeviceEditorApp'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.4.
}
var kMIDIPropertyDriverDeviceEditorApp: CFStringRef; external name '_kMIDIPropertyDriverDeviceEditorApp'; (* attribute const *)
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

{!
	@constant		kMIDIPropertySupportsShowControl
	@discussion
		device/entity property, integer (0/1). Indicates whether the device implements the MIDI
		Show Control specification.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertySupportsShowControl: CFStringRef; external name '_kMIDIPropertySupportsShowControl'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.5.
}
var kMIDIPropertySupportsShowControl: CFStringRef; external name '_kMIDIPropertySupportsShowControl'; (* attribute const *)
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

{!
	@constant		kMIDIPropertyDisplayName
	@discussion
		device/entity/endpoint property, string.

		Provides the Apple-recommended user-visible name for an endpoint, by combining the
		device and endpoint names.

		For objects other than endpoints, the display name is the same as the name.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
var kMIDIPropertyDisplayName: CFStringRef; external name '_kMIDIPropertyDisplayName'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

		New for CoreMIDI 1.5.
}
var kMIDIPropertyDisplayName: CFStringRef; external name '_kMIDIPropertyDisplayName'; (* attribute const *)
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

//==================================================================================================
//#pragma mark	Clients
{!
	@functiongroup	Clients
}

{!
	@function		MIDIClientCreate

	@abstract 		Creates a MIDIClient object.

	@param			name
						The client's name.
	@param			notifyProc	
						An optional (may be NULL) callback function through which the client
						will receive notifications of changes to the system.
	@param			notifyRefCon 
						A refCon passed back to notifyRefCon
	@param			outClient	
						On successful return, points to the newly-created MIDIClientRef.
	@result			An OSStatus result code.
	
	@discussion
		Note that notifyProc will always be called on the run loop which was current when
		MIDIClientCreate was first called.
	
}
function MIDIClientCreate( name: CFStringRef; notifyProc: MIDINotifyProc; notifyRefCon: UnivPtr; var outClient: MIDIClientRef ): OSStatus; external name '_MIDIClientCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


{!
	@function   	MIDIClientDispose

	@abstract   	Disposes a MIDIClient object.
	
	@param  		client
						The client to dispose.
	@result			An OSStatus result code.

	@discussion
		It is not essential to call this function; the CoreMIDI framework will automatically
		dispose all MIDIClients when an application terminates.
}
function MIDIClientDispose( client: MIDIClientRef ): OSStatus; external name '_MIDIClientDispose';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

//==================================================================================================
//#pragma mark	Ports
{!
	@functiongroup	Ports
}

{!
	@function		MIDIInputPortCreate

	@abstract 		Creates an input port through which the client may receive
					incoming MIDI messages from any MIDI source.

	@param			client
						The client to own the newly-created port.
	@param			portName
						The name of the port.
	@param			readProc
						The MIDIReadProc which will be called with incoming MIDI,
						from sources connected to this port.
	@param			refCon
						The refCon passed to readHook.
	@param			outPort
						On successful return, points to the newly-created
						MIDIPort.
	@result			An OSStatus result code.

	@discussion
		After creating a port, use MIDIPortConnectSource to establish an input connection from
		any number of sources to your port.
		
		readProc will be called on a separate high-priority thread owned by CoreMIDI.
}
function MIDIInputPortCreate( client: MIDIClientRef; portName: CFStringRef; readProc: MIDIReadProc; refCon: UnivPtr; var outPort: MIDIPortRef ): OSStatus; external name '_MIDIInputPortCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIOutputPortCreate

	@abstract 		Creates an output port through which the client may send
					outgoing MIDI messages to any MIDI destination.

	@param			client
						The client to own the newly-created port
	@param			portName
						The name of the port.
	@param			outPort
						On successful return, points to the newly-created
						MIDIPort.
	@result			An OSStatus result code.

	@discussion
		Output ports provide a mechanism for MIDI merging.  CoreMIDI assumes that each output
		port will be responsible for sending only a single MIDI stream to each destination,
		although a single port may address all of the destinations in the system.

		Multiple output ports are only necessary when an application is capable of directing
		multiple simultaneous MIDI streams to the same destination.
}
function MIDIOutputPortCreate( client: MIDIClientRef; portName: CFStringRef; var outPort: MIDIPortRef ): OSStatus; external name '_MIDIOutputPortCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIPortDispose

	@abstract 		Disposes a MIDIPort object.

	@param			port
						The port to dispose.
	@result			An OSStatus result code.

	@discussion
		It is not usually necessary to call this function; when an application's MIDIClient's
		are automatically disposed at termination, or explicitly, via MIDIClientDispose, the
		client's ports are automatically disposed at that time.
}
function MIDIPortDispose( port: MIDIPortRef ): OSStatus; external name '_MIDIPortDispose';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


{!
	@function		MIDIPortConnectSource

	@abstract 		Establishes a connection from a source to a client's input port.

	@param			port
						The port to which to create the connection.  This port's
						readProc is called with incoming MIDI from the source.
	@param			source
						The source from which to create the connection.
	@param			connRefCon	
						This refCon is passed to the MIDIReadProc, as a way to
						identify the source.
	@result			An OSStatus result code.

	@discussion
}
function MIDIPortConnectSource( port: MIDIPortRef; source: MIDIEndpointRef; connRefCon: UnivPtr ): OSStatus; external name '_MIDIPortConnectSource';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


{!
	@function		MIDIPortDisconnectSource
	
	@abstract 		Closes a previously-established source-to-input port 
					connection.

	@param			port
						The port whose connection is being closed.
	@param			source
						The source from which to close a connection to the
						specified port.
	@result			An OSStatus result code.

	@discussion
}
function MIDIPortDisconnectSource( port: MIDIPortRef; source: MIDIEndpointRef ): OSStatus; external name '_MIDIPortDisconnectSource';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


//==================================================================================================
//#pragma mark	Devices
{!
	@functiongroup	Devices
}

{!
	@function		MIDIGetNumberOfDevices

	@abstract 		Returns the number of devices in the system.

	@result			The number of devices in the system, or 0 if an error 
					occurred.
}
function MIDIGetNumberOfDevices: ItemCount; external name '_MIDIGetNumberOfDevices';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


{!
	@function		MIDIGetDevice

	@abstract 		Returns one of the devices in the system.

	@param			deviceIndex0
						The index (0...MIDIGetNumberOfDevices()-1) of the device
						to return.
	@result			A reference to a device, or NULL if an error occurred.

	@discussion
		Use this to enumerate the devices in the system.

		To enumerate the entities in the system, you can walk through the devices, then walk
		through the devices' entities.

		Note: If a client iterates through the devices and entities in the system, it will not
		ever visit any virtual sources and destinations created by other clients.  Also, a
		device iteration will return devices which are "offline" (were present in the past but
		are not currently present), while iterations through the system's sources and
		destinations will not include the endpoints of offline devices.

		Thus clients should usually use MIDIGetNumberOfSources, MIDIGetSource,
		MIDIGetNumberOfDestinations and MIDIGetDestination, rather iterating through devices and
		entities to locate endpoints.
}
function MIDIGetDevice( deviceIndex0: ItemCount ): MIDIDeviceRef; external name '_MIDIGetDevice';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIDeviceGetNumberOfEntities

	@abstract 		Returns the number of entities in a given device.
	
	@param			device
						The device being queried.

	@result			The number of entities the device contains, or 0 if an
					error occurred.
}
function MIDIDeviceGetNumberOfEntities( device: MIDIDeviceRef ): ItemCount; external name '_MIDIDeviceGetNumberOfEntities';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIDeviceGetEntity

	@abstract 		Returns one of a given device's entities.
	
	@param			device
						The device being queried.
	@param			entityIndex0
						The index (0...MIDIDeviceGetNumberOfEntities(device)-1)
						of the entity to return

	@result			A reference to an entity, or NULL if an error occurred.
}
function MIDIDeviceGetEntity( device: MIDIDeviceRef; entityIndex0: ItemCount ): MIDIEntityRef; external name '_MIDIDeviceGetEntity';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

//==================================================================================================
//#pragma mark	Entities
{!
	@functiongroup	Entities
}

{!
	@function		MIDIEntityGetNumberOfSources
	
	@abstract 		Returns the number of sources in a given entity.
	
	@param			entity
						The entity being queried

	@result			The number of sources the entity contains, or 0 if an
					error occurred.
}
function MIDIEntityGetNumberOfSources( entity: MIDIEntityRef ): ItemCount; external name '_MIDIEntityGetNumberOfSources';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIEntityGetSource

	@abstract 		Returns one of a given entity's sources.
	
	@param			entity
						The entity being queried.
	@param			sourceIndex0
						The index (0...MIDIEntityGetNumberOfSources(entity)-1) of
						the source to return

	@result			A reference to a source, or NULL if an error occurred.
}
function MIDIEntityGetSource( entity: MIDIEntityRef; sourceIndex0: ItemCount ): MIDIEndpointRef; external name '_MIDIEntityGetSource';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIEntityGetNumberOfDestinations

	@abstract 		Returns the number of destinations in a given entity.
	
	@param			entity
						The entity being queried

	@result			The number of destinations the entity contains, or 0
					if an error occurred.
}
function MIDIEntityGetNumberOfDestinations( entity: MIDIEntityRef ): ItemCount; external name '_MIDIEntityGetNumberOfDestinations';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIEntityGetDestination

	@abstract 		Returns one of a given entity's destinations.
	
	@param			entity
						The entity being queried.
	@param			destIndex0
						The index (0...MIDIEntityGetNumberOfDestinations(entity)
						- 1) of the destination to return

	@result			A reference to a destination, or NULL if an error occurred.
}
function MIDIEntityGetDestination( entity: MIDIEntityRef; destIndex0: ItemCount ): MIDIEndpointRef; external name '_MIDIEntityGetDestination';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIEntityGetDevice

	@abstract 		Returns an entity's device.
	
	@param			inEntity
						The entity being queried.
	@param			outDevice
						On successful return, the entity's owning device.
}
function MIDIEntityGetDevice( inEntity: MIDIEntityRef; var outDevice: MIDIDeviceRef ): OSStatus; external name '_MIDIEntityGetDevice';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

//==================================================================================================
//#pragma mark	Endpoints
{!
	@functiongroup	Endpoints
}

{!
	@function		MIDIGetNumberOfSources

	@abstract 		Returns the number of sources in the system.

	@result			The number of sources in the system, or 0 if an error
					occurred.
}
function MIDIGetNumberOfSources: ItemCount; external name '_MIDIGetNumberOfSources';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


{!
	@function		MIDIGetSource

	@abstract 		Returns one of the sources in the system.

	@param			sourceIndex0	
						The index (0...MIDIGetNumberOfSources()-1) of the source
						to return
	@result			A reference to a source, or NULL if an error occurred.
}
function MIDIGetSource( sourceIndex0: ItemCount ): MIDIEndpointRef; external name '_MIDIGetSource';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


{!
	@function		MIDIGetNumberOfDestinations

	@abstract 		Returns the number of destinations in the system.

	@result			The number of destinations in the system, or 0 if an error 
					occurred.
}
function MIDIGetNumberOfDestinations: ItemCount; external name '_MIDIGetNumberOfDestinations';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


{!
	@function		MIDIGetDestination

	@abstract 		Returns one of the destinations in the system.

	@param			destIndex0	
						The index (0...MIDIGetNumberOfDestinations()-1) of the
						destination to return
	@result			A reference to a destination, or NULL if an error occurred.
}
function MIDIGetDestination( destIndex0: ItemCount ): MIDIEndpointRef; external name '_MIDIGetDestination';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIEndpointGetEntity

	@abstract 		Returns an endpoint's entity.
	
	@param			inEndpoint
						The endpoint being queried.
	@param			outEntity
						On exit, the endpoint's owning entity, or NULL if none.
	
	@discussion
		Virtual sources and destinations don't have entities.
}
function MIDIEndpointGetEntity( inEndpoint: MIDIEndpointRef; var outEntity: MIDIEntityRef ): OSStatus; external name '_MIDIEndpointGetEntity';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{!
	@function		MIDIDestinationCreate

	@abstract 		Creates a virtual destination in a client.

	@param			client
						The client owning the virtual destination.
	@param			name
						The name of the virtual destination.
	@param			readProc
						The MIDIReadProc to be called when a client sends MIDI to
						the virtual destination.
	@param			refCon
						The refCon to be passed to the readProc.
	@param			outDest
						On successful return, a pointer to the newly-created
						destination.
	@result			An OSStatus result code.

	@discussion
		The specified readProc gets called when clients send MIDI to your virtual destination.

		Drivers need not call this; when they create devices and entities, sources and
		destinations are created at that time.
		
		After creating a virtual destination, it's a good idea to assign it the same unique ID
		it had the last time your application created it. (Although you should be prepared for
		this to fail in the unlikely event of a collision.) This will permit other clients
		to retain persistent references to your virtual destination more easily.
		
		See the discussion of kMIDIPropertyAdvanceScheduleTimeMuSec for notes about the
		relationship between when a sender sends MIDI to the destination and when it is
		received.
}
function MIDIDestinationCreate( client: MIDIClientRef; name: CFStringRef; readProc: MIDIReadProc; refCon: UnivPtr; var outDest: MIDIEndpointRef ): OSStatus; external name '_MIDIDestinationCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


{!
	@function		MIDISourceCreate

	@abstract 		Creates a virtual source in a client.

	@param			client
						The client owning the virtual source.
	@param			name
						The name of the virtual source.
	@param			outSrc
						On successful return, a pointer to the newly-created
						source.
	@result			An OSStatus result code.

	@discussion
		Drivers need not call this; when they create devices and entities, sources and
		destinations are created at that time.

		After creating a virtual source, use MIDIReceived to transmit MIDI messages from your
		virtual source to any clients connected to the virtual source.

		After creating a virtual source, it's a good idea to assign it the same unique ID it had
		the last time your application created it. (Although you should be prepared for this to
		fail in the unlikely event of a collision.) This will permit other clients to retain
		persistent references to your virtual source more easily.
}
function MIDISourceCreate( client: MIDIClientRef; name: CFStringRef; var outSrc: MIDIEndpointRef ): OSStatus; external name '_MIDISourceCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


{!
	@function		MIDIEndpointDispose

	@abstract 		Disposes a virtual source or destination your client created.

	@param			endpt
						The endpoint to be disposed.

	@result			An OSStatus result code.
}
function MIDIEndpointDispose( endpt: MIDIEndpointRef ): OSStatus; external name '_MIDIEndpointDispose';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


//==================================================================================================
//#pragma mark	External Devices
{!
	@functiongroup	External Devices
}
{!
	@function		MIDIGetNumberOfExternalDevices

	@abstract 		Returns the number of external MIDI devices in the system.
					
	@result			The number of external devices in the system, or 0 if an error 
					occurred.

	@discussion
		External MIDI devices are MIDI devices connected to driver endpoints via a standard MIDI
		cable. Their presence is completely optional, only when a UI (such as Audio MIDI Setup)
		adds them.
}
function MIDIGetNumberOfExternalDevices: ItemCount; external name '_MIDIGetNumberOfExternalDevices';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)

{!
	@function		MIDIGetExternalDevice

	@abstract 		Returns one of the external devices in the system.

	@param			deviceIndex0
						The index (0...MIDIGetNumberOfDevices()-1) of the device
						to return.
	@result			A reference to a device, or NULL if an error occurred.

	@discussion
		Use this to enumerate the external devices in the system.
}
function MIDIGetExternalDevice( deviceIndex0: ItemCount ): MIDIDeviceRef; external name '_MIDIGetExternalDevice';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)

//==================================================================================================
//#pragma mark	Objects and Properties
{!
	@functiongroup	Objects and Properties
}

{!
	@function		MIDIObjectGetIntegerProperty

	@abstract 		Gets an object's integer-type property.
	
	@param			obj
						The object whose property is to be returned.
	@param			propertyID
						Name of the property to return.
	@param			outValue
						On successful return, the value of the property.
	@result			An OSStatus result code.

	@discussion
		(See the MIDIObjectRef documentation for information about properties.)
}
function MIDIObjectGetIntegerProperty( obj: MIDIObjectRef; propertyID: CFStringRef; var outValue: SInt32 ): OSStatus; external name '_MIDIObjectGetIntegerProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIObjectSetIntegerProperty

	@abstract 		Sets an object's integer-type property.
	
	@param			obj
						The object whose property is to be altered.
	@param			propertyID
						Name of the property to set.
	@param			value
						New value of the property.
	@result			An OSStatus result code.

	@discussion
		(See the MIDIObjectRef documentation for information about properties.)
}
function MIDIObjectSetIntegerProperty( obj: MIDIObjectRef; propertyID: CFStringRef; value: SInt32 ): OSStatus; external name '_MIDIObjectSetIntegerProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIObjectGetStringProperty

	@abstract 		Gets an object's string-type property.

	@param			obj
						The object whose property is to be returned.
	@param			propertyID
						Name of the property to return.
	@param			str
						On successful return, the value of the property.
	@result			An OSStatus result code.

	@discussion
		(See the MIDIObjectRef documentation for information about properties.)
}
function MIDIObjectGetStringProperty( obj: MIDIObjectRef; propertyID: CFStringRef; var str: CFStringRef ): OSStatus; external name '_MIDIObjectGetStringProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIObjectSetStringProperty

	@abstract 		Sets an object's string-type property.
	
	@param			obj
						The object whose property is to be altered.
	@param			propertyID
						Name of the property to set.
	@param			str
						New value of the property.
	@result			An OSStatus result code.

	@discussion
		(See the MIDIObjectRef documentation for information about properties.)
}
function MIDIObjectSetStringProperty( obj: MIDIObjectRef; propertyID: CFStringRef; str: CFStringRef ): OSStatus; external name '_MIDIObjectSetStringProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIObjectGetDataProperty

	@abstract 		Gets an object's data-type property.

	@discussion
(See the MIDIObjectRef documentation for information
					about properties.)
		
	@param			obj
						The object whose property is to be returned.
	@param			propertyID
						Name of the property to return.
	@param			outData
						On successful return, the value of the property.
	@result			An OSStatus result code.
}
function MIDIObjectGetDataProperty( obj: MIDIObjectRef; propertyID: CFStringRef; var outData: CFDataRef ): OSStatus; external name '_MIDIObjectGetDataProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIObjectSetDataProperty

	@abstract 		Sets an object's data-type property.
	
	@param			obj
						The object whose property is to be altered.
	@param			propertyID
						Name of the property to set.
	@param			data
						New value of the property.
	@result			An OSStatus result code.

	@discussion
		(See the MIDIObjectRef documentation for information about properties.)
}
function MIDIObjectSetDataProperty( obj: MIDIObjectRef; propertyID: CFStringRef; data: CFDataRef ): OSStatus; external name '_MIDIObjectSetDataProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIObjectGetDictionaryProperty

	@abstract 		Gets an object's dictionary-type property.

	@param			obj
						The object whose property is to be returned.
	@param			propertyID
						Name of the property to return.
	@param			outDict
						On successful return, the value of the property.
	@result			An OSStatus result code.

	@discussion
		(See the MIDIObjectRef documentation for information about properties.)
}
function MIDIObjectGetDictionaryProperty( obj: MIDIObjectRef; propertyID: CFStringRef; var outDict: CFDictionaryRef ): OSStatus; external name '_MIDIObjectGetDictionaryProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{!
	@function		MIDIObjectSetDictionaryProperty

	@abstract 		Sets an object's dictionary-type property.
	
	@param			obj
						The object whose property is to be altered.
	@param			propertyID
						Name of the property to set.
	@param			dict
						New value of the property.
	@result			An OSStatus result code.

	@discussion
		(See the MIDIObjectRef documentation for information about properties.)
}
function MIDIObjectSetDictionaryProperty( obj: MIDIObjectRef; propertyID: CFStringRef; data: CFDictionaryRef ): OSStatus; external name '_MIDIObjectSetDictionaryProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{!
	@function		MIDIObjectGetProperties
	
	@abstract		Gets all of an object's properties.

	@param			obj
						The object whose properties are to be returned.
	@param			outProperties
						On successful return, the object's properties.
	@param			deep
						true if the object's child objects are to be included
						(e.g. a device's entities, or an entity's endpoints).
	@result			An OSStatus result code.

	@discussion
		Returns a CFPropertyList of all of an object's properties. The property list may be a
		dictionary or an array. Dictionaries map property names (CFString) to values, which may
		be CFNumber, CFString, or CFData.  Arrays are arrays of such values.

		Properties which an object inherits from its owning object (if any) are not included.
}
function MIDIObjectGetProperties( obj: MIDIObjectRef; var outProperties: CFPropertyListRef; deep: Boolean ): OSStatus; external name '_MIDIObjectGetProperties';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)

{!
	@function		MIDIObjectRemoveProperty
	
	@abstract		Removes an object's property.

	@param			obj
						The object whose property is to be removed.
	@param			propertyID
						The property to be removed.
	@result			An OSStatus result code.

	@discussion
}
function MIDIObjectRemoveProperty( obj: MIDIObjectRef; propertyID: CFStringRef ): OSStatus; external name '_MIDIObjectRemoveProperty';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{!
	@function		MIDIObjectFindByUniqueID

	@abstract 		Locates a device, external device, entity, or endpoint
					by its uniqueID.
	@param			inUniqueID
						The uniqueID of the object to search for.  (This should
						be the result of an earlier call to MIDIObjectGetIntegerProperty
						for the property kMIDIPropertyUniqueID).
	@param			outObject
						The returned object, or NULL if the object was not found or
						an error occurred.  This should be cast to the appropriate
						type (MIDIDeviceRef, MIDIEntityRef, MIDIEndpointRef),
						according to *outObjectType.
	@param			outObjectType
						On exit, the type of object which was found; undefined
						if none found.
	@result			An OSStatus error code, including kMIDIObjectNotFound if there
					is no object with the specified uniqueID.

	@discussion
}
function MIDIObjectFindByUniqueID( inUniqueID: MIDIUniqueID; var outObject: MIDIObjectRef; var outObjectType: MIDIObjectType ): OSStatus; external name '_MIDIObjectFindByUniqueID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

//==================================================================================================
//#pragma mark	MIDI I/O
{!
	@functiongroup	I/O
}

{!
	@function		MIDISend

	@abstract 		Sends MIDI to a destination.

	@param			port
						The output port through which the MIDI is to be sent.
	@param			dest
						The destination to receive the events.
	@param			pktlist
						The MIDI events to be sent.
	@result			An OSStatus result code.

	@discussion
		Events with future timestamps are scheduled for future delivery.  CoreMIDI performs
		any needed MIDI merging.
}
function MIDISend( port: MIDIPortRef; dest: MIDIEndpointRef; const (*var*) pktlist: MIDIPacketList ): OSStatus; external name '_MIDISend';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
				
{!
	@function		MIDISendSysex

	@abstract 		Sends a single system-exclusive event, asynchronously.

	@param			request	
						Contains the destination, and a pointer to the MIDI data to be sent.
	@result			An OSStatus result code.

	@discussion
		request->data must point to a single MIDI system-exclusive message, or portion thereof.
}
function MIDISendSysex( var request: MIDISysexSendRequest ): OSStatus; external name '_MIDISendSysex';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIReceived

	@abstract 		Distributes incoming MIDI from a source to the client input ports
					which are connected to that source.

	@param			src
						The source which is transmitting MIDI.
	@param			pktlist
						The MIDI events to be transmitted.
	@result			An OSStatus result code.

	@discussion
		Drivers should call this function when receiving MIDI from a source.

		Clients which have created virtual sources, using MIDISourceCreate, should call this
		function when the source is generating MIDI.
		
		Unlike MIDISend(), a timestamp of 0 is not equivalent to "now"; the driver or virtual
		source is responsible for putting proper timestamps in the packets.
}
function MIDIReceived( src: MIDIEndpointRef; const (*var*) pktlist: MIDIPacketList ): OSStatus; external name '_MIDIReceived';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{!
	@function		MIDIFlushOutput
	
	@abstract		Unschedules previously-sent packets.
					
	@param			dest
						All pending events scheduled to be sent to this destination
						are unscheduled.  If NULL, the operation applies to
						all destinations.

	@discussion
		Clients may use MIDIFlushOutput to cancel the sending of packets that were previously
		scheduled for future delivery.
}
function MIDIFlushOutput( dest: MIDIEndpointRef ): OSStatus; external name '_MIDIFlushOutput';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)

{!
	@function		MIDIRestart

	@abstract 		Stops and restarts MIDI I/O.
	
	@discussion
		This is useful for forcing CoreMIDI to ask its drivers to rescan for hardware.
	
	@result			An OSStatus result code.
}
function MIDIRestart: OSStatus; external name '_MIDIRestart';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)


//==================================================================================================
//#pragma mark Packet Lists

{!
	@functiongroup	Packet Lists
}

{*!
	@function		MIDIPacketNext

	@abstract 		Advances a MIDIPacket pointer to the MIDIPacket which immediately follows it 
					in memory if it is part of a MIDIPacketList.

	@param			pkt
						A pointer to a MIDIPacket in a MIDIPacketList.

	@result			The subsequent packet in the MIDIPacketList.

	@discussion
		This is implemented as a macro for efficiency and to avoid const problems.
*}

//#define MIDIPacketNext(pkt)	((MIDIPacket *)&(pkt)->data[(pkt)->length])
function MIDIPacketNext(pkt : MIDIPacketPtr) : MIDIPacketPtr; inline;

{!
	@function		MIDIPacketListInit

	@abstract 		Prepares a MIDIPacketList to be built up dynamically.
	
	@param			pktlist
						The packet list to be initialized.

	@result			A pointer to the first MIDIPacket in the packet list.
}
function MIDIPacketListInit( var pktlist: MIDIPacketList ): MIDIPacketPtr; external name '_MIDIPacketListInit';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


{!
	@function		MIDIPacketListAdd

	@abstract 		Adds a MIDI event to a MIDIPacketList.
	
	@param			pktlist
						The packet list to which the event is to be added.
	@param			listSize
						The size, in bytes, of the packet list.
	@param			curPacket
						A packet pointer returned by a previous call to
						MIDIPacketListInit or MIDIPacketListAdd for this packet
						list.
	@param			time
						The new event's time.
	@param			nData
						The length of the new event, in bytes.
	@param			data
						The new event.  May be a single MIDI event, or a partial
						sys-ex event.  Running status is <b>not</b> permitted.
	@result			Returns null if there was not room in the packet for the
					event; otherwise returns a packet pointer which should be
					passed as curPacket in a subsequent call to this function.

	@discussion
		The maximum size of a packet list is 65536 bytes. Large sysex messages must be sent in
		smaller packet lists.
}
function MIDIPacketListAdd( var pktlist: MIDIPacketList; listSize: ByteCount; var curPacket: MIDIPacket; time: MIDITimeStamp; nData: ByteCount; const (*var*) data: Byte ): MIDIPacketPtr; external name '_MIDIPacketListAdd';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)


{$endc} { TARGET_OS_MAC }

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation
{$ifc TARGET_OS_MAC}


{$R-}
function MIDIPacketNext(pkt : MIDIPacketPtr) : MIDIPacketPtr; inline;
begin
	MIDIPacketNext := MIDIPacketPtr(@pkt^.data[pkt^.length])
end;
{$endc} { TARGET_OS_MAC }
end.

{$endc} {not MACOSALLINCLUDE}
