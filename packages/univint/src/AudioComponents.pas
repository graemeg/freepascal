{
     File:      AudioUnit/AudioComponent.h
 
     Contains:  API for finding and opening audio components
  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright: (c) 2007 - 2010 by Apple, Inc., all rights reserved.
=======
     Copyright: (c) 2007 - 2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Copyright: (c) 2007 - 2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Copyright: (c) 2007 - 2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Copyright: (c) 2007 - 2008 by Apple Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
     Copyright: (c) 2007 - 2008 by Apple Inc., all rights reserved.
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
                     http://www.freepascal.org/bugs.html
}
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
}
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
}
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
}
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> origin/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
}
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
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

unit AudioComponents;
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
>>>>>>> origin/fixes_2.4
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
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,CoreAudioTypes,Components,CFBase,CFDictionary;
=======
uses MacTypes,CoreAudioTypes,Components,CFBase;
>>>>>>> graemeg/cpstrnew
=======
uses MacTypes,CoreAudioTypes,Components,CFBase;
>>>>>>> graemeg/cpstrnew
=======
uses MacTypes,CoreAudioTypes,Components,CFBase;
>>>>>>> graemeg/cpstrnew
=======
uses MacTypes,CoreAudioTypes,Components,CFBase;
>>>>>>> origin/cpstrnew
=======
uses MacTypes,CoreAudioTypes,Components,CFBase;
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


//=====================================================================================================================
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
//#pragma mark Overview
=======
////#pragma mark Overview
>>>>>>> graemeg/cpstrnew
=======
////#pragma mark Overview
>>>>>>> graemeg/cpstrnew
=======
////#pragma mark Overview
>>>>>>> graemeg/cpstrnew
=======
////#pragma mark Overview
>>>>>>> origin/cpstrnew
=======
////#pragma mark Overview
>>>>>>> origin/fixes_2.4

{!
    @header AudioComponent

	@discussion
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	
    This file defines a collection of APIs that are designed to be used when finding and opening
    audio components (such as audio units, audio codecs, and audio file components).
    
    Traditionally, the Component Manager has been used for the registration, discovery, and
    packaging of these loadable code modules. However, this may not always be the case. In order
    to provide an API that will be supported going forward from SnowLeopard (10.6) and iPhone
    2.0, it is advised that applications use the Audio Component APIs to find and load (open)
	audio components such as audio units. 
	
    The type "AudioComponent" or "AudioComponentInstance" should be seen and used as a distinct
    type from the Component Manager types of "Component" and "ComponentInstance". It is not the
    case that it is safe to assume a direct cast is compatible between this type and the other.

    The AudioComponent APIs provide a general, and mostly complete service for doing the basic
    task of finding and using audio components. If however, you find that you need functionality
    that is not provided by this API that was previously supported by the Component Manager then
    you should use the ComponentMgr APIs exclusively to find and open the Component Manager
    components. In other words, you should not assume that an audio component instance is always
    interchangeable with the existing ComponentInstance type.
    
    
    Beginning with Mac OS X 10.7, AudioComponents can be registered and used directly without
    involving the Component Manager. The system scans certain directories for bundles with names
    ending in ".audiocomp" or ".component" (the latter permits registering plug-ins in a single
    bundle with both the Component Manager and the Audio Component system). These directories
    are scanned non-recursively:
    
        ~/Library/Audio/Plug-Ins/Components
        /Library/Audio/Plug-Ins/Components
        /System/Library/Components

    Bundles' Info.plist dictionaries should contain an "AudioComponents" item whose value
    is an array of dictionaries, e.g.

        <key>AudioComponents</key>
        <array>
            <dict>
                <key>type</key>
                <string>aufx</string>
                <key>subtype</key>
                <string>XMPL</string>
                <key>manufacturer</key>
                <string>ACME</string>
                <key>name</key>
                <string>AUExample</string>
                <key>version</key>
                <integer>12345</integer>
                <key>factoryFunction</key>
                <string>AUExampleFactory</string>
                
                <!-- An AudioComponent is sandbox safe -->
                
                <key>sandboxSafe</key>
                <true/>
                
                <!-- or it can describe it's resource usage -->
                
                <key>resourceUsage</key>
                <dict>
                    <key>iokit.user-client</key>
                    <array>
                        <string>CustomUserClient1</string>
                        <string>CustomUserClient2</string>
                    </array>
                    <key>mach-lookup.global-name</key>
                    <array>
                        <string>MachServiceName1</string>
                        <string>MachServiceName2</string>
                    </array>
                    <key>network.client</key>
                    <true/>
                    <key>temporary-exception.files.all.read-write</key>
                    </true>
                </dict>
            </dict>
        </array>

    The type, subtype and manufacturer keys correspond to the OSType fields of the 
    AudioComponentDescription structure. They can be strings if they are 4 ASCII characters; 
    otherwise they must be 32-bit integers.
    
    The "factoryFunction" is the name of a AudioComponentFactoryFunction in the bundle's binary.
    
    The "sandboxSafe" key is used to indicate whether or not an AudioComponent can be loaded
    directly into a sandboxed process. This key is reflected in the componentFlags field of the the
    AudioComponentDescription for the AudioComponent with the constant, kAudioComponentFlag_SandboxSafe.
    Note that if this key is not present, it is assumed that the AudioComponent is not sandbox safe.
    
    The "resourceUsage" key describes the system resources used by an AudioComponent that is not
    sandobox safe. The keys for this dictionary are described below. If the "sandboxSafe" key is
    true, this dictionary should not be included.
    
    The "iokit.user-client" key is a "resourceUsage" key that describes the IOKit user-client
    objects the AudioComponent will open. It is an array of the user-clients' class names.
    
    The "mach-lookup.global-name" key is a "resourceUsage" key that describes the mach services the
    AudioComponent needs to connect to. It is an array of the names of the services. Note that these
    services can be direct mach services found via bootstrap_look_up() or XPC services found via
    xpc_connection_create_mach_service().
    
    The "network.client" key is a "resourceUsage" key that indicates that the AudioComponent will
    receive data from the network.
    
    The "temporary-exception.files.all.read-write" key is a "resourceUsage" key that indicates that
    the AudioComponent needs arbitrary access to the file system. This is for backward compatibility
    for AudioComponents that have not yet adopted the usage of security scope bookmarks and/or the
    usage of the standard file dialog for discovering, accessing and storing persistent references
    to files on the file system. In a future OS release, this key will not be supported.
    
    Note that a sandbox safe AudioComponent can function correctly in even the most severely
    sandboxed process. This means that the process will have curtailed or no access to common system
    resources like the file system, device drivers, the network, and communication with other
    processes.
    
    When instantiating a sandbox unsafe AudioComponent in a sandboxed process, the system evaluates
    the "resourceUsage" information against the restrictions the process is under. If the
    "resourceUsage" will not violate those restrictions, the AudioComponent will be instantiated and
    can be used as normal. Note that the system will set kAudioComponentFlag_SandboxSafe in the
    AudioComponentDescription in this case.
    
    If the "resourceUsage" information includes things that can't be accessed from the process and
    the process has the entitlement, "com.apple.security.temporary-exception.audio-unit-host", the
    system will ask the user whether or not it is acceptable for the process to open the unsafe
    AudioComponent. If the user says yes, the system will suspend the process's sandbox and allow
    the unsafe AudioComponent to be opened and used.
}


//=====================================================================================================================
//#pragma mark Constants

{!
	@constant	kAudioComponentFlag_Unsearchable

	When this bit in AudioComponentDescription's componentFlags is set, AudioComponentFindNext
	will only return this component when performing a specific, non-wildcard search for the
	component, i.e. with non-zero values of componentType, componentSubType, and
	componentManufacturer. This can be useful when privately registering a component.
	Available starting in Mac OS X 10.7 and iOS 5.0
	
	@constant	kAudioComponentFlag_SandboxSafe
	
	An AudioComponent sets this bit in it's componentFlags to indicate to the system that the
	AudioComponent is safe to open in a sandboxed process.
	Available starting in Mac OS X 10.8.
}
const
	kAudioComponentFlag_Unsearchable = 1;
	kAudioComponentFlag_SandboxSafe = 2;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	This file defines a collection of APIs that are designed to be used when finding and opening audio components
	(such as audio units, audio codecs, audio file components).
	
	Traditionally, the ComponentManager has been used for the registration, discovery, and packaging of these loadable
	code modules. However, this may not always be the case. In order to provide an API that will be supported going forward
	from SnowLeopard (10.6) and iPhone 2.0, it is advised that applications use the AudioComponent APIs to find and load(open)
	audio components such as audio units. 
	
	The type "AudioComponent" or "AudioComponentInstance" should be seen and used as a distinct type from the Component Manager
	types of "Component" and "ComponentInstance". It is not the case that it is safe to assume a direct cast is compatible between
	this type and the other.
	
	The AudioComponent APIs provide a general, and mostly complete service for doing the basic task of finding and using audio components. 
	If however, you find that you need functionality that is not provided by this API that was previously supported by the ComponentMgr then
	you should use the ComponentMgr APIs exclusively to find and open the Component Mgr components. In other words, you should not assume that
	an audio component instance is always interchangeable with the existing Component Instance type.
}

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
//#pragma mark Data Types

{!
	@struct			AudioComponentDescription
	@discussion		A structure used to describe the unique and identifying IDs of an audio component 
	@field			componentType
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        A unique 4-byte code identifying the generic type of an audio component
=======
						A unique 4-byte code indentifying the generic type of an audio component
>>>>>>> graemeg/cpstrnew
=======
						A unique 4-byte code indentifying the generic type of an audio component
>>>>>>> graemeg/cpstrnew
=======
						A unique 4-byte code indentifying the generic type of an audio component
>>>>>>> graemeg/cpstrnew
=======
						A unique 4-byte code indentifying the generic type of an audio component
>>>>>>> origin/cpstrnew
=======
						A unique 4-byte code indentifying the generic type of an audio component
>>>>>>> origin/fixes_2.4
	@field			componentSubType
						the particular flavor of this instance
	@field			componentManufacturer
						vendor identification
	@field			componentFlags
						must be set to zero unless a known specific value is requested
	@field			componentFlagsMask
						must be set to zero unless a known specific value is requested
}
//#pragma pack(push, 4)

type
	AudioComponentDescription = record
		componentType: OSType;
		componentSubType: OSType;
		componentManufacturer: OSType;
		componentFlags: UInt32;
		componentFlagsMask: UInt32;
	end;
	AudioComponentDescriptionPtr = ^AudioComponentDescription;
//#pragma pack(pop)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{!
	@typedef		AudioComponent
	@abstract		The type used to represent a class of particular audio components
    @discussion     An audio component is usually found through a search and is then uniquely
                    identified by the triple of an audio component's type, subtype and
                    manufacturer.
					
					It can have properties associated with it (such as a name, a version).
					
                    It is then used as a factory (like a class in an object-oriented programming
                    language) from which to create particular instances. The instances are used
                    to do the actual work.
					
					For example:
                    'aufx', 'dely', 'appl' describes the digital delay, audio unit effect from
                    Apple, Inc. You can find this component through searching explicitly for the
                    audio component that matches this pattern (this is an unique identifier -
                    there is only one match to this triple ID). Then once found, the Apple delay
                    effect audio unit can be created from its audio component and used to apply
                    that effect to an audio signal. While the audio component is a singleton,
                    you can of course create as many instances of a given audio component that
                    you need to do the work you need to do.
}
type
	AudioComponent = ^OpaqueAudioComponent; { an opaque type }
	OpaqueAudioComponent = record end;
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
	@typedef		AudioComponent
	@abstract		The type used to represent a class of particular audio components
	@discussion		An audio component is usually found through a search and is then uniquely identified by the triple of
					an audio component's type, subtype and manufacturer. 
					
					It can have properties associated with it (such as a name, a version).
					
					It is then used as a factory (like a class in an OO language) from which to create particular instances. The
					instances are used to do the actualy work.
					
					For example:
					'aufx', 'dely', 'appl' describes the digital delay, audio unit effect from Apple Inc. You can find this component
					through searching explicitly for the audio component that matches this pattern (this is an unique identifier - there is
					only one match to this triple ID). Then once found, the Apple delay effect audio unit can be created from its audio component
					and used to apply that effect to an audio signal. While the audio component is a singleton, you can of course create as many
					instances of a given audio component that you need to do the work you need to do.
}
type
	AudioComponent = ^SInt32; { an opaque type }
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
	@typedef		AudioComponentInstance
	@abstract		The type used to represent an instance of a particular audio component
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @discussion     An audio component instance is created from its factory/producer audio
                    component. It is the body of code that does the work.
	
                    A special note: While on the desktop this is typedef'd to a
                    ComponentInstanceRecord *, you should not assume that this will always be
                    compatible and usable with Component Manager calls.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@discussion		An audio component instance is created from its factory/producer audio component. It is the body of code that does the work.
	
					A special note: While on the desktop this is typedef'd to a ComponentInstanceRecord *, you should not always assume that this
					will always be compatible and usable with ComponentMgr calls. 
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
//#if TARGET_OS_IPHONE
{$ifc TARGET_OS_IPHONE}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AudioComponentInstance = ^OpaqueAudioComponentInstance; { an opaque type }
	OpaqueAudioComponentInstance = record end;
=======
	AudioComponentInstance = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	AudioComponentInstance = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	AudioComponentInstance = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	AudioComponentInstance = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	AudioComponentInstance = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
//#else
{$elsec}
type
	AudioComponentInstance = ComponentInstanceRecordPtr;
//#endif
{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{!
    @typedef        AudioComponentMethod
    @abstract       The broad prototype for an audio plugin method
    @discussion     Every audio plugin will implement a collection of methods that match a particular
					selector. For example, the AudioUnitInitialize API call is implemented by a
					plugin implementing the kAudioUnitInitializeSelect selector. Any function implementing
					an audio plugin selector conforms to the basic pattern where the first argument
					is a pointer to the plugin instance structure, has 0 or more specific arguments,  
					and returns an OSStatus.
}
type
	AudioComponentMethod = function( self: UnivPtr; ... ): OSStatus;

{!
    @struct         AudioComponentPlugInInterface
    @discussion     A structure used to represent an audio plugin's routines 
    @field          Open
                        the function used to open (or create) an audio plugin instance
    @field          Close
                        the function used to close (or dispose) an audio plugin instance
    @field          Lookup
                        this is used to return a function pointer for a given selector, 
						or NULL if that selector is not implemented
    @field          reserved
                        must be NULL
}
type
	AudioComponentPlugInInterface = record
	  Open: function (self: UnivPtr; mInstance: AudioComponentInstance): OSStatus;
	  Close: function (self: UnivPtr): OSStatus;
	  Lookup: function (selector: SInt16): AudioComponentMethod;
	  reserved: UnivPtr; // set to NULL
	end;
	AudioComponentPlugInInterfacePtr = ^AudioComponentPlugInInterface;

{!
    @typedef        AudioComponentFactoryFunction
    @abstract       A function that creates AudioComponentInstances.
    @discussion
                    Authors of AudioComponents may register them from bundles as described
                    above in the discussion of this header file, or dynamically within a single
                    process, using AudioComponentRegister.
    
    @param          inDesc
                        The AudioComponentDescription specifying the component to be instantiated.
    @result         A pointer to a structure whose first member is a pointer to an
                    AudioComponentPlugInInterface. Thus the return type could be considered
                    AudioComponentPlugInInterface **, but since in practice it is never
                    this basic type, but rather, AudioUnitPlugInInterface, AudioCodecPlugInInterface,
                    etc., it is declared void * for simplicity.
}
type
	AudioComponentFactoryFunction = function( const (*var*) inDesc: AudioComponentDescription ): AudioComponentPlugInInterfacePtr;
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
//#pragma mark Functions


{!
    @function       AudioComponentFindNext
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @abstract       Finds an audio component.
    @discussion     This function is used to find an audio component that is the closest match
                    to the provided values.
    @param          inComponent
                        If NULL, then the search starts from the beginning until an audio
                        component is found that matches the description provided by inDesc.
                        If non-NULL, then the search starts (continues) from the previously
                        found audio component specified by inComponent, and will return the next
                        found audio component.
	@param			inDesc
                        The type, subtype and manufacturer fields are used to specify the audio
                        component to search for. A value of 0 (zero) for any of these fields is
                        a wildcard, so the first match found is returned.
    @result         An audio component that matches the search parameters, or NULL if none found.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
    @abstract       find an audio component
    @discussion     This function is used to find an audio component that is the closest match to the provide values.
    @param          inComponent
						If NULL, then the search starts from the beginning until an audio component is found that matches
						the description provided by inDesc
						If not-NULL, then the search starts (continues) from the previously found audio component specified
						by inComponent, and will return the nextfound audio component
	@param			inDesc
						The type, subtype and manufacturer fields are used to specify the audio component to search for.
						A value of 0 (zero) for any of these fiels is a wildcard, so the first match found is returned
	@result			An audio component that matches the search parameters, or NULL if none found
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
function AudioComponentFindNext( inComponent: AudioComponent; const (*var*) inDesc: AudioComponentDescription ): AudioComponent; external name '_AudioComponentFindNext';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)

{!
    @function       AudioComponentCount
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @abstract       Counts audio components.
    @discussion     Returns the number of AudioComponents that match the specified
                    AudioComponentDescription.
	@param			inDesc
                        The type, subtype and manufacturer fields are used to specify the audio
                        components to count A value of 0 (zero) for any of these fields is a
                        wildcard, so will match any value for this field
    @result         a UInt32. 0 (zero) means no audio components were found that matched the
                    search parameters.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
    @abstract       number of audio components
    @discussion     the number of AudioComponenents that match the specified AudioComponentDescription. 
	@param			inDesc
						The type, subtype and manufacturer fields are used to specify the audio components to count
						A value of 0 (zero) for any of these fiels is a wildcard, so will match any value for this field
	@result			a UInt32. 0 (zero) means no audio components were found that matched the search parameters.
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
function AudioComponentCount( const (*var*) inDesc: AudioComponentDescription ): UInt32; external name '_AudioComponentCount';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)

{!
    @function       AudioComponentCopyName
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @abstract       Retrieves the name of an audio component.
=======
    @abstract       used to retrieve the name of an audio component
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to retrieve the name of an audio component
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to retrieve the name of an audio component
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to retrieve the name of an audio component
>>>>>>> origin/cpstrnew
=======
    @abstract       used to retrieve the name of an audio component
>>>>>>> origin/fixes_2.4
    @discussion     the name of an audio component
	@param			inComponent
						the audio component (must not be NULL)
	@param			outName
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        a CFString that is the name of the audio component. This string should
                        be released by the caller.
=======
						a CFString that is the name of the audio component. This string should be released by the caller.
>>>>>>> graemeg/cpstrnew
=======
						a CFString that is the name of the audio component. This string should be released by the caller.
>>>>>>> graemeg/cpstrnew
=======
						a CFString that is the name of the audio component. This string should be released by the caller.
>>>>>>> graemeg/cpstrnew
=======
						a CFString that is the name of the audio component. This string should be released by the caller.
>>>>>>> origin/cpstrnew
=======
						a CFString that is the name of the audio component. This string should be released by the caller.
>>>>>>> origin/fixes_2.4
    @result			an OSStatus result code.
}
function AudioComponentCopyName( inComponent: AudioComponent; var outName: CFStringRef ): OSStatus; external name '_AudioComponentCopyName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)

{!
    @function       AudioComponentGetDescription
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @abstract       Retrieve an audio component's description.
    @discussion     This will return the fully specified audio component description for the
                    provided audio component.
=======
    @abstract       used to retrieve the audio component's description
    @discussion     will return the fully specified audio component description for the provided audio component
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to retrieve the audio component's description
    @discussion     will return the fully specified audio component description for the provided audio component
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to retrieve the audio component's description
    @discussion     will return the fully specified audio component description for the provided audio component
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to retrieve the audio component's description
    @discussion     will return the fully specified audio component description for the provided audio component
>>>>>>> origin/cpstrnew
=======
    @abstract       used to retrieve the audio component's description
    @discussion     will return the fully specified audio component description for the provided audio component
>>>>>>> origin/fixes_2.4
	@param			inComponent
						the audio component (must not be NULL)
	@param			outDesc
						the audio component description for the specified audio component
    @result			an OSStatus result code.
}
function AudioComponentGetDescription( inComponent: AudioComponent; var outDesc: AudioComponentDescription ): OSStatus; external name '_AudioComponentGetDescription';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)

{!
    @function       AudioComponentGetVersion
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @abstract       Retrieve an audio component's version.
    @discussion
=======
    @abstract       used to retrieve the audio component's version
    @discussion     will return the version number of a given audio component
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to retrieve the audio component's version
    @discussion     will return the version number of a given audio component
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to retrieve the audio component's version
    @discussion     will return the version number of a given audio component
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to retrieve the audio component's version
    @discussion     will return the version number of a given audio component
>>>>>>> origin/cpstrnew
=======
    @abstract       used to retrieve the audio component's version
    @discussion     will return the version number of a given audio component
>>>>>>> origin/fixes_2.4
	@param			inComponent
						the audio component (must not be NULL)
	@param			outVersion
						the audio component's version in the form of 0xMMMMmmDD (Major, Minor, Dot)
    @result			an OSStatus result code.
}
function AudioComponentGetVersion( inComponent: AudioComponent; var outVersion: UInt32 ): OSStatus; external name '_AudioComponentGetVersion';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)

{!
    @function       AudioComponentInstanceNew
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @abstract       Creates an audio component instance.
    @discussion     This function creates an instance of a given audio component. The audio
                    component instance is the object that does all of the work, whereas the
                    audio component is the way an application finds and then creates this object
                    to do this work. For example, an audio unit is a type of audio component
                    instance, so to use an audio unit, one finds its audio component, and then
                    creates a new instance of that component. This instance is then used to
                    perform the audio tasks for which it was designed (process, mix, synthesise,
                    etc.).
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
    @abstract       used to create an audio component instance
    @discussion     this call creates an instance of a given audio component. The audio component instance is the object
					that does all of the work, whereas the audio component is the way that a program finds and then makes this object
					to do this work. For example, an audio unit is a type of audio component instance, so to use an audio unit, one
					finds its audio component, and then creates a new instance of that component. This instance is then used
					to process, mix, synthesise, etc the audio tasks for which it was designed.
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
	@param			inComponent
						the audio component (must not be NULL)
	@param			outInstance
						the audio component instance
    @result			an OSStatus result code.
}
function AudioComponentInstanceNew( inComponent: AudioComponent; var outInstance: AudioComponentInstance ): OSStatus; external name '_AudioComponentInstanceNew';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)

{!
    @function       AudioComponentInstanceDispose
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @abstract       Disposes of an audio component instance.
    @discussion     This function will dispose the audio component instance that was created
                    with the New call. It will deallocate any resources that the instance was using.
=======
    @abstract       used to dispose of an audio component instance
    @discussion     this call will dispose the audio component instance that was created with the New call. It will deallocate any resources
					that the instance was using.
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to dispose of an audio component instance
    @discussion     this call will dispose the audio component instance that was created with the New call. It will deallocate any resources
					that the instance was using.
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to dispose of an audio component instance
    @discussion     this call will dispose the audio component instance that was created with the New call. It will deallocate any resources
					that the instance was using.
>>>>>>> graemeg/cpstrnew
=======
    @abstract       used to dispose of an audio component instance
    @discussion     this call will dispose the audio component instance that was created with the New call. It will deallocate any resources
					that the instance was using.
>>>>>>> origin/cpstrnew
=======
    @abstract       used to dispose of an audio component instance
    @discussion     this call will dispose the audio component instance that was created with the New call. It will deallocate any resources
					that the instance was using.
>>>>>>> origin/fixes_2.4
	@param			inInstance
						the audio component instance to dispose (must not be NULL)
    @result			an OSStatus result code.
}
function AudioComponentInstanceDispose( inInstance: AudioComponentInstance ): OSStatus; external name '_AudioComponentInstanceDispose';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// retrieves the class object associated with the instance
{!
    @function       AudioComponentInstanceGetComponent
    @abstract       Retrieve the audio component from its instance
    @discussion     Allows the application at any time to retrieve the audio component that is
                    the factory object of a given instance (i.e., the audio component that was
                    used to create the instance in the first place). This allows the application
                    to retrieve general information about a particular audio component (its
                    name, version, etc) when one just has an audio component instance to work
                    with
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
// retrieves the class object assosciated with the instance
{!
    @function       AudioComponentInstanceGetComponent
    @abstract       used to retrieve the audio component from its instance
    @discussion     allows the application at any time to retrieve the audio component that is the factory object of a given instance 
					(ie, the audio component that was used to create the instnace in the first place). This allows the application to
					retrieve general information about a particular audio component (its name, version, etc) when one just has an
					audio component instance to work with 
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
	@param			inInstance
						the audio component instance (must not be NULL, and instance must be valid - that is, not disposed)
    @result			a valid audio component or NULL if no component was found.
}
function AudioComponentInstanceGetComponent( inInstance: AudioComponentInstance ): AudioComponent; external name '_AudioComponentInstanceGetComponent';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)

{!
    @function       AudioComponentInstanceCanDo
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @discussion     Determines if an audio component instance implements a particular component
                    API call as signified by the specified selector identifier token.
=======
    @discussion     this call is used to determine if an audio component instance implements a particular component API call
					as signified by the specified selector identifier token.
>>>>>>> graemeg/cpstrnew
=======
    @discussion     this call is used to determine if an audio component instance implements a particular component API call
					as signified by the specified selector identifier token.
>>>>>>> graemeg/cpstrnew
=======
    @discussion     this call is used to determine if an audio component instance implements a particular component API call
					as signified by the specified selector identifier token.
>>>>>>> graemeg/cpstrnew
=======
    @discussion     this call is used to determine if an audio component instance implements a particular component API call
					as signified by the specified selector identifier token.
>>>>>>> origin/cpstrnew
=======
    @discussion     this call is used to determine if an audio component instance implements a particular component API call
					as signified by the specified selector identifier token.
>>>>>>> origin/fixes_2.4
	@param			inInstance
						the audio component instance
	@param			inSelectorID
						a number to signify the audio component API (component selector) as appropriate for the instance's component type.
    @result			a boolean
}
function AudioComponentInstanceCanDo( inInstance: AudioComponentInstance; inSelectorID: SInt16 ): Boolean; external name '_AudioComponentInstanceCanDo';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_3_0) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{!
    @function       AudioComponentRegister
    @abstract       Dynamically registers an AudioComponent within the current process
    @discussion
        AudioComponents are registered either when found in appropriate bundles in the filesystem,
        or via this call. AudioComponents registered via this call are available only within
        the current process.
    
    @param          inDesc
                        The AudioComponentDescription that describes the AudioComponent. Note that
                        the registrar needs to be sure to set the flag kAudioComponentFlag_SandboxSafe
                        in the componentFlags field of the AudioComponentDescription to indicate that
                        the AudioComponent can be loaded directly into a sandboxed process.
    @param          inName
                        the AudioComponent's name
    @param          inVersion
                        the AudioComponent's version
    @param          inFactory
                        an AudioComponentFactoryFunction which will create instances of your
                        AudioComponent
    @result         an AudioComponent object
}
function AudioComponentRegister( const (*var*) inDesc: AudioComponentDescription; inName: CFStringRef; inVersion: UInt32; inFactory: AudioComponentFactoryFunction ): AudioComponent; external name '_AudioComponentRegister';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7,__IPHONE_5_0) *)

{$ifc TARGET_OS_MAC}
{!
    @function       AudioComponentCopyConfigurationInfo
    @abstract       Fetches the basic configuration info about a given AudioComponent
    @discussion     Currently, only AudioUnits can supply this information.
    @param          inComponent
                        The AudioComponent whose info is being fetched.
    @param          outConfigurationInfo
                        On exit, this is CFDictionaryRef that contains information describing the
                        capabilities of the AudioComoponent. The specific information depends on the
                        type of AudioComponent. The keys for the dictionary are defined in
                        AudioUnitProperties.h (or other headers as appropriate for the component type).
    @result         An OSStatus indicating success or failure.
}
function AudioComponentCopyConfigurationInfo( inComponent: AudioComponent; var outConfigurationInfo: CFDictionaryRef ): OSStatus; external name '_AudioComponentCopyConfigurationInfo';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7,__IPHONE_NA) *)
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
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
