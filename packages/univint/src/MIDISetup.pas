{
 	File:   	CoreMIDI/MIDISetup.h
 
 	Contains:   Specialized configuration-editing routines for CoreMIDI.
 
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

unit MIDISetup;
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
uses MacTypes,CFBase,CFData,CFArray,MIDIServices;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


//  -----------------------------------------------------------------------------
{!
	@header MIDISetup.h

	This header defines functions that manipulate and customize the global
	state of the MIDI system.  These functions are generally only needed by 
	applications which wish to allow the user some flexibility in how
	the MIDI system's state is presented, and by MIDI drivers, which may
	dynamically modify the system state as hardware is connected and 
	disconnected.
}

//  -----------------------------------------------------------------------------
{!
	@typedef		MIDISetupRef
	
	@discussion		Derives from MIDIObjectRef, does not have an owner object.

					This represents the global state of the MIDI system,
					containing lists of the MIDI devices and serial port
					owners.
					
					Generally, only MIDI drivers and specialized configuration
					editors will need to manipulate MIDISetup objects, not the
					average MIDI client application.  As of CoreMIDI 1.1, the
					MIDIServer maintains a single global MIDISetupRef, stored
					persistently in a preference file.
}
//#if __LP64__
{$ifc not undefined TARGET_CPU_64 and TARGET_CPU_64}
type
	MIDISetupRef = MIDIObjectRef;
{$elsec}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	MIDISetupRef = ^OpaqueMIDISetup; { an opaque type }
	OpaqueMIDISetup = record end;
=======
	MIDISetupRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MIDISetupRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MIDISetupRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MIDISetupRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	MIDISetupRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
{$endif}


// ______________________________________________________________________________
//	MIDISetup
// ______________________________________________________________________________

//  -----------------------------------------------------------------------------
{!
	@function		MIDISetupCreate

	@abstract 		Interrogates drivers, to discover what hardware is present.
	
					As of CoreMIDI 1.1, it is usually not necessary to call
					this function, as CoreMIDI manages a single persistent
					MIDISetup itself.

	@param			outSetup
						On successful return, points to a newly-created MIDISetup
						object.  The caller is responsible for disposing it,
						or transferring ownership of the object back to the
						system, with MIDISetupInstall.
	@result			An OSStatus result code.
}
function MIDISetupCreate( var outSetup: MIDISetupRef ): OSStatus; external name '_MIDISetupCreate';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)


//  -----------------------------------------------------------------------------
{!
	@function		MIDISetupDispose

	@abstract 		Dispose a MIDISetup object.

					As of CoreMIDI 1.1, it is usually not necessary to call
					this function, as CoreMIDI manages a single persistent
					MIDISetup itself.
	
	@param			setup
						The MIDISetup to be disposed.
	@result			An OSStatus result code.
}
function MIDISetupDispose( setup: MIDISetupRef ): OSStatus; external name '_MIDISetupDispose';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDISetupInstall

	@abstract 		Install a MIDISetup as the system's current state.

					A client can create a MIDISetup object using
					MIDISetupCreate, or MIDISetupFromData.  This function will
					install this state as the current state of the system,
					possibly changing the devices visible to clients.

					As of CoreMIDI 1.1, it is usually not necessary to call
					this function, as CoreMIDI manages a single persistent
					MIDISetup itself.

	@param			setup
						The MIDISetup object to install.  Ownership of this
						object is transferred from the client to the system; the
						client must <b>not</b> dispose of this MIDISetup.
	@result			An OSStatus result code.
}
function MIDISetupInstall( setup: MIDISetupRef ): OSStatus; external name '_MIDISetupInstall';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)


//  -----------------------------------------------------------------------------
{!
	@function		MIDISetupGetCurrent

	@abstract 		Return the system's current MIDISetup.
	
					As of CoreMIDI 1.1, it is usually not necessary to call
					this function, as CoreMIDI manages a single persistent
					MIDISetup itself.
	
	@param			outSetup
						On successful return, points to the system's most
						recently installed MIDISetup.  The system retains
						ownership of the object; the client must <b>not</b>
						dispose of this MIDISetup.
	@result			An OSStatus result code.
}
function MIDISetupGetCurrent( var outSetup: MIDISetupRef ): OSStatus; external name '_MIDISetupGetCurrent';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)


//  -----------------------------------------------------------------------------
{!
	@function		MIDISetupToData

	@abstract 		Create an XML representation of a MIDISetup object.
	
					As of CoreMIDI 1.1, it is usually not necessary to call
					this function, as CoreMIDI manages a single persistent
					MIDISetup itself.
	
	@param			setup
						The MIDISetup object whose XML representation is to be
						returned.
	@param			outData
						On successful return, points to a newly-created CFDataRef
						containing the XML text.  The client is responsible for
						releasing this CFData object when done with it.
	@result			An OSStatus result code.
}
function MIDISetupToData( setup: MIDISetupRef; var outData: CFDataRef ): OSStatus; external name '_MIDISetupToData';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDISetupFromData

	@abstract 		Create a MIDISetup object from an XML stream.
	
					As of CoreMIDI 1.1, it is usually not necessary to call
					this function, as CoreMIDI manages a single persistent
					MIDISetup itself.
	
	@param			data
						The XML text from which a MIDISetup object is to be built.
	@param			outSetup
						On successful return, points to a newly-created MIDISetup
						object.  The caller is responsible for disposing it, or
						transferring ownership of the object back to the system,
						with MIDISetupInstall.
	@result			An OSStatus result code.
}
function MIDISetupFromData( data: CFDataRef; var outSetup: MIDISetupRef ): OSStatus; external name '_MIDISetupFromData';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDIDeviceAddEntity

	@discussion		Drivers call this function to specify one of the entities that 
					comprise a device.
					
					Non-drivers may call this function as of CoreMIDI 1.1, to
					add entities to external devices.
	
	@param			device
						The device to which an entity is to be added.
	@param			name
						The name of the new entity.
	@param			embedded
						True if this entity is inside the device, false if the
						entity simply consists of external connectors to which
						other devices can be attached.
	@param			numSourceEndpoints
						The number of source endpoints the entity has.
	@param			numDestinationEndpoints
						The number of destination endpoints the entity has.
	@param			newEntity
						On successful return, points to the newly-created entity.
	@result			An OSStatus result code.
}
function MIDIDeviceAddEntity( device: MIDIDeviceRef; name: CFStringRef; embedded: Boolean; numSourceEndpoints: ItemCount; numDestinationEndpoints: ItemCount; var newEntity: MIDIEntityRef ): OSStatus; external name '_MIDIDeviceAddEntity';
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDIDeviceRemoveEntity

	@discussion		Drivers may call this function to remove one of a device's
					entities.
					
					New for CoreMIDI 1.1.
	
	@param			device
						The device from which an entity is to be removed.
	@param			entity
						The entity to be removed.
	@result			An OSStatus result code.
}
function MIDIDeviceRemoveEntity( device: MIDIDeviceRef; entity: MIDIEntityRef ): OSStatus; external name '_MIDIDeviceRemoveEntity';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDIEntityAddOrRemoveEndpoints

	@discussion		Drivers and configuration editors may call this function to add to 
					or remove an entity's endpoints.
					
					New for CoreMIDI 1.3.
	
	@param			entity
						The entity whose endpoints are to be manipulated.
	@param			numSourceEndpoints
						The desired new number of source endpoints.
	@param			numDestinationEndpoints
						The desired new number of destination endpoints.
	@result			An OSStatus result code.
}
function MIDIEntityAddOrRemoveEndpoints( entity: MIDIEntityRef; numSourceEndpoints: ItemCount; numDestinationEndpoints: ItemCount ): OSStatus; external name '_MIDIEntityAddOrRemoveEndpoints';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDISetupAddDevice

	@abstract 		Adds a driver-owner MIDI device to the current MIDISetup
	
	@discussion		Only MIDI drivers may make this call; it is in this header
					file only for consistency with MIDISetupRemoveDevice.
	
					New for CoreMIDI 1.1.
	
	@param			device
						The device to be added.
}
function MIDISetupAddDevice( device: MIDIDeviceRef ): OSStatus; external name '_MIDISetupAddDevice';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDISetupRemoveDevice

	@abstract 		Removes a driver-owned MIDI device from the current MIDISetup
	
	@discussion		Generally this should only be called from a studio configuration
					editor, to remove a device which is offline and which the user
					has specified as being permanently missing.
					
					Instead of removing devices from the setup, drivers should
					set the device's kMIDIPropertyOffline to 1 so that if the
					device reappears later, none of its properties are lost.
	
					New for CoreMIDI 1.1.
	
	@param			device
						The device to be added.
}
function MIDISetupRemoveDevice( device: MIDIDeviceRef ): OSStatus; external name '_MIDISetupRemoveDevice';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDISetupAddExternalDevice

	@abstract 		Adds an external MIDI device to the current MIDISetup
	
	@discussion		Useful for a studio configuration editor.  New for CoreMIDI 1.1.
	
	@param			device
						The device to be added.
}
function MIDISetupAddExternalDevice( device: MIDIDeviceRef ): OSStatus; external name '_MIDISetupAddExternalDevice';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDISetupRemoveExternalDevice

	@abstract 		Removes an external MIDI device from the current MIDISetup
	
	@discussion		Useful for a studio configuration editor.  New for CoreMIDI 1.1.
	
	@param			device
						The device to be removed.
}
function MIDISetupRemoveExternalDevice( device: MIDIDeviceRef ): OSStatus; external name '_MIDISetupRemoveExternalDevice';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDIGetSerialPortOwner

	@abstract 		Returns the MIDI driver that owns a serial port.
	
	@discussion		The current MIDISetup tracks ownership of serial ports
					to one of the MIDI drivers installed in the system.
	
					Serial ports can be enumerated using IOServiceMatching(
					kIOSerialBSDServiceValue).  The port's unique name is
					the IOService's kIOTTYDeviceKey property. 

					New for CoreMIDI 1.1.
					
					A previous version of this documentation specified an incorrect
					key for obtaining the port's unique name (IOTTYBaseName).
	
	@param			portName
						The name of a serial port.
	@param			outDriverName
						On exit, the name of the driver owning the port,
						or NULL if no driver owns it.

	@result			An OSStatus result code.	
}
function MIDIGetSerialPortOwner( portName: CFStringRef; var outDriverName: CFStringRef ): OSStatus; external name '_MIDIGetSerialPortOwner';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDISetSerialPortOwner

	@abstract 		Specifies the MIDI driver that owns a serial port.
	
	@discussion		Use this to assign ownership of a serial port
					to one of the MIDI drivers installed in the system.
	
					New for CoreMIDI 1.1.
	
	@param			portName
						The name of a serial port.
	@param			driverName
						The name of the driver that owns the serial port,
						or NULL to specify that no driver owns it.

	@result			An OSStatus result code.	
}
function MIDISetSerialPortOwner( portName: CFStringRef; driverName: CFStringRef ): OSStatus; external name '_MIDISetSerialPortOwner';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDIGetSerialPortDrivers

	@abstract 		Returns a list of installed MIDI drivers for serial port
					MIDI devices.
	
	@discussion		Use this to determine which of the installed MIDI drivers
					are for devices which may attach to serial ports.
	
					New for CoreMIDI 1.1.
	
	@param			outDriverNames
						On exit, a CFArrayRef containing a list of CFStringRef's
						which are the names of the serial port MIDI drivers.
						The array should be released by the caller.

	@result			An OSStatus result code.	
}
function MIDIGetSerialPortDrivers( var outDriverNames: CFArrayRef ): OSStatus; external name '_MIDIGetSerialPortDrivers';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

//  -----------------------------------------------------------------------------
{!
	@function		MIDIExternalDeviceCreate

	@abstract		Create a new external MIDI device.

	@discussion		Non-drivers may call this function as of CoreMIDI 1.1, to
					create external devices.

					The new device is not added to the current MIDISetupRef;
					to do this, use MIDISetupAddExternalDevice.
	
	@param			name
						The name of the new device.
	@param			manufacturer
						The name of the device's manufacturer.
	@param			model
						The device's model name.
	@param			outDevice
						On successful return, points to the newly-created device.
	@result			An OSStatus result code.
}
function MIDIExternalDeviceCreate( name: CFStringRef; manufacturer: CFStringRef; model: CFStringRef; var outDevice: MIDIDeviceRef ): OSStatus; external name '_MIDIExternalDeviceCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)


{$endc} { TARGET_OS_MAC }{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
