<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
//------------------------------------------------------------------------------------------------------------------------------
//
//  ImageCapture/ICAApplication.h
//
//  Copyright (c) 2004-2007 Apple Inc. All rights reserved.
//
//------------------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
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
{
     File:       ICAApplication.p
 
     Contains:   General purpose Image Capture definitions
 
     Version:    Technology: 1.0
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  © 2000-2002 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}


=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/fixes_2.4
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

unit ICAApplication;
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
=======
=======
>>>>>>> origin/cpstrnew
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
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	{$setc TARGET_CPU_PPC64 := FALSE}
>>>>>>> origin/fixes_2.4
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
uses MacTypes,AEDataModel,Files,CFBase,CFArray,CFData,CFDictionary;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======


{$ifc TARGET_OS_MAC}
>>>>>>> origin/cpstrnew

{$ALIGN MAC68K}

//------------------------------------------------------------------------------------------------------------------------------
{!
    @header
        ICAApplication.h
    @discussion
      ICAApplication.h defines structures and functions that are used by clients of Image Capture framework. 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

//------------------------------------------------------------------------------------------------------------------------------

=======

//------------------------------------------------------------------------------------------------------------------------------

>>>>>>> graemeg/cpstrnew
=======

//------------------------------------------------------------------------------------------------------------------------------

>>>>>>> graemeg/cpstrnew
=======

//------------------------------------------------------------------------------------------------------------------------------

>>>>>>> graemeg/cpstrnew
=======

//------------------------------------------------------------------------------------------------------------------------------

>>>>>>> origin/cpstrnew
type
	ICAError = OSErr;

//------------------------------------------------------------------------------------------------------ Parameter block version
{!
    @enum Parameter block version
    @discussion
        Parameter block version.
    @constant kICAPBVersion
        Version 1 parameter block.
}
const
	kICAPBVersion = $00010000;

//------------------------------------------------------------------------------------------------------------------ Error codes
// Image Capture error code range = -9900 to -9949
{!
    @enum Error codes
    @discussion
        Definition of error codes returned by Image Capture framework
    @constant kICACommunicationErr
        An error occurred in communication between different components of Image Capture framework.
    @constant kICADeviceNotFoundErr
        The specified device is not found.
    @constant kICADeviceNotOpenErr
        The specified device is not open.
    @constant kICAFileCorruptedErr
        Encountered a corrupt file.
    @constant kICAIOPendingErr
        There is a pending I/O.
    @constant kICAInvalidObjectErr
        The specified object is invalid.
    @constant kICAInvalidPropertyErr
        The specified property is invalid.
    @constant kICAIndexOutOfRangeErr
        The specified index is out of range.
    @constant kICAPropertyTypeNotFoundErr
        A property with the specified property type is not found.
    @constant kICACannotYieldDevice
        The device module cannot yield the specified device to the requestor.
    @constant kICADataTypeNotFoundErr
        Data with the specified data type is not found.
    @constant kICADeviceMemoryAllocationErr
        The device module encountered a memory allocation error.
    @constant kICADeviceInternalErr
        The device module encountered an unspecifed error.
    @constant kICADeviceInvalidParamErr
        At least one of the parameters passed to the device module is invalid.
    @constant kICADeviceAlreadyOpenErr  
        The specified device is already open.
    @constant kICADeviceLocationIDNotFoundErr
        The specified USB Location ID is not found.
    @constant kICADeviceGUIDNotFoundErr
        The specified FireWire GUID is not found.
    @constant kICADeviceIOServicePathNotFoundErr
        The specified IOService path is not found.
    @constant kICAFrameworkInternalErr
        Image Capture Framework encountered an error.
    @constant kICAExtensionInternalErr
        Image Capture Extension encountered an error.
    @constant kICAInvalidSessionErr
        The specified session is not valid.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
const
	kICACommunicationErr = -9900;
	kICADeviceNotFoundErr = -9901;
	kICADeviceNotOpenErr = -9902;
	kICAFileCorruptedErr = -9903;
	kICAIOPendingErr = -9904;
	kICAInvalidObjectErr = -9905;
	kICAInvalidPropertyErr = -9906;
	kICAIndexOutOfRangeErr = -9907;
	kICAPropertyTypeNotFoundErr = -9908;
	kICACannotYieldDevice = -9909;
	kICADataTypeNotFoundErr = -9910;
	kICADeviceMemoryAllocationErr = -9911;
	kICADeviceInternalErr = -9912;
	kICADeviceInvalidParamErr = -9913;
	kICADeviceAlreadyOpenErr = -9914;
	kICADeviceLocationIDNotFoundErr = -9915;
	kICADeviceGUIDNotFoundErr = -9916;
	kICADeviceIOServicePathNotFoundErr = -9917;
	kICADeviceUnsupportedErr = -9918;
	kICAFrameworkInternalErr = -9919;
	kICAExtensionInternalErr = -9920;
	kICAInvalidSessionErr = -9921;

//------------------------------------------------------------------------------------------------- ICAObject types and subtypes
{!
    @enum ICAObject types and subtypes
    @discussion
        Definition of ICAObject types and subtypes
    @constant kICADevice
        Object is a device supported by Image Capture framework.
    @constant kICADeviceCamera
        Object is a camera.
    @constant kICADeviceScanner
        Object is a scanner.
    @constant kICADeviceMFP
        Object is a multi-function peripheral.
    @constant kICADevicePhone
        Object is a camera phone.
    @constant kICADevicePDA
        Object is a personal digital assistant.
    @constant kICADeviceOther
        Object is a device supported by Image Capture framework, but of unknown subtype.
    @constant kICAList
        Object is a device list.
    @constant kICADirectory
        Object is a directory.
    @constant kICAFile
        Object is a file.
    @constant kICAFileImage
        Object is an image file.
    @constant kICAFileMovie
        Object is a movie file.
    @constant kICAFileAudio
        Object is an audio file.
    @constant kICAFileFirmware
        Object is a firmware file.
    @constant kICAFileOther
        Object is a generic file.
}
const
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/cpstrnew
const
	kICACommunicationErr = -9900;
	kICADeviceNotFoundErr = -9901;
	kICADeviceNotOpenErr = -9902;
	kICAFileCorruptedErr = -9903;
	kICAIOPendingErr = -9904;
	kICAInvalidObjectErr = -9905;
	kICAInvalidPropertyErr = -9906;
	kICAIndexOutOfRangeErr = -9907;
	kICAPropertyTypeNotFoundErr = -9908;
	kICACannotYieldDevice = -9909;
	kICADataTypeNotFoundErr = -9910;
	kICADeviceMemoryAllocationErr = -9911;
	kICADeviceInternalErr = -9912;
	kICADeviceInvalidParamErr = -9913;
	kICADeviceAlreadyOpenErr = -9914;
	kICADeviceLocationIDNotFoundErr = -9915;
	kICADeviceGUIDNotFoundErr = -9916;
	kICADeviceIOServicePathNotFoundErr = -9917;
	kICADeviceUnsupportedErr = -9918;
	kICAFrameworkInternalErr = -9919;
	kICAExtensionInternalErr = -9920;
	kICAInvalidSessionErr = -9921;

//------------------------------------------------------------------------------------------------- ICAObject types and subtypes
{!
    @enum ICAObject types and subtypes
    @discussion
        Definition of ICAObject types and subtypes
    @constant kICADevice
        Object is a device supported by Image Capture framework.
    @constant kICADeviceCamera
        Object is a camera.
    @constant kICADeviceScanner
        Object is a scanner.
    @constant kICADeviceMFP
        Object is a multi-function peripheral.
    @constant kICADevicePhone
        Object is a camera phone.
    @constant kICADevicePDA
        Object is a personal digital assistant.
    @constant kICADeviceOther
        Object is a device supported by Image Capture framework, but of unknown subtype.
    @constant kICAList
        Object is a device list.
    @constant kICADirectory
        Object is a directory.
    @constant kICAFile
        Object is a file.
    @constant kICAFileImage
        Object is an image file.
    @constant kICAFileMovie
        Object is a movie file.
    @constant kICAFileAudio
        Object is an audio file.
    @constant kICAFileFirmware
        Object is a firmware file.
    @constant kICAFileOther
        Object is a generic file.
}
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kICADevice = FourCharCode('icdv');
	kICADeviceCamera = FourCharCode('cmra');
	kICADeviceScanner = FourCharCode('scan');
	kICADeviceMFP = FourCharCode('mfp ');
	kICADevicePhone = FourCharCode('phon');
	kICADevicePDA = FourCharCode('pda ');
	kICADeviceOther = FourCharCode('doth');
	kICAList = FourCharCode('objl');
	kICADirectory = FourCharCode('dire');
	kICAFile = FourCharCode('file');
	kICAFileImage = FourCharCode('imag');
	kICAFileMovie = FourCharCode('moov');
	kICAFileAudio = FourCharCode('audo');
	kICAFileFirmware = FourCharCode('firm');
	kICAFileOther = FourCharCode('othe');

//------------------------------------------------------------------------------------------------------------ ICAProperty types
{!
    @enum ICAProperty types
    @discussion
        Definition of ICAProperties
    @constant kICAProperty
        Generic property type; for images, refer to 'Digital Still Camera Image File Format Standard' Exif Version 2.1 section 2.6.4. and 2.6.5.
    @constant kICAPropertyImageWidth
        Image width. 
    @constant kICAPropertyImageHeight
        Image height.
    @constant kICAPropertyImageBitDepth
        Image bit-depth.
    @constant kICAPropertyImageDPI
        Image DPI.
    @constant kICAPropertyImageExposureTime
        Image exposure time.
    @constant kICAPropertyImageFNumber
        Image f-Number.
    @constant kICAPropertyImageDateOriginal
        Original date & time of an object; value associated with this property is a null-terminated string conforming to format "YYYY:MM:DD hh:mm:ss".
    @constant kICAPropertyImageDateDigitized
        Digitized date & time of an object; value associated with this property is a null-terminated string conforming to format "YYYY:MM:DD hh:mm:ss".
    @constant kICAPropertyImageShutterSpeed
        Shutter speed used to capture an image.
    @constant kICAPropertyImageAperture
        Aperture used to capture an image.
    @constant kICAPropertyImageFlash
        Indicates whether flash was used to capture an image.
    @constant kICAPropertyColorSpace
        Color space used to represent an image.
    @constant kICAPropertyImageFilename
        Filename of an image.
    @constant kICAPropertyImageSize
        Size of an image in bytes. 
    @constant kICAPropertyImageData
        Data of an image.
    @constant kICAPropertyImageThumbnail
        Thumbnail of an image.
    @constant kICAPropertyColorSyncProfile
        ColorSync profile associated with an image.
}  
const
	kICAProperty = FourCharCode('prop');
	kICAPropertyImageWidth = FourCharCode('0100');
	kICAPropertyImageHeight = FourCharCode('0101');
	kICAPropertyImageBitDepth = FourCharCode('0102');
	kICAPropertyImageDPI = FourCharCode('011A');
	kICAPropertyImageExposureTime = FourCharCode('829A');
	kICAPropertyImageFNumber = FourCharCode('829D');
	kICAPropertyImageDateOriginal = FourCharCode('9003');
	kICAPropertyImageDateDigitized = FourCharCode('9004');
	kICAPropertyImageShutterSpeed = FourCharCode('9201');
	kICAPropertyImageAperture = FourCharCode('9202');
	kICAPropertyImageFlash = FourCharCode('9209');
	kICAPropertyColorSpace = FourCharCode('A001');
	kICAPropertyImageFilename = FourCharCode('ifil');
	kICAPropertyImageSize = FourCharCode('isiz');
	kICAPropertyImageData = FourCharCode('idat');
	kICAPropertyImageThumbnail = FourCharCode('thum');
	kICAPropertyColorSyncProfile = FourCharCode('prof');

//------------------------------------------------------------------------------------------------------------------- Data types
{!
    @enum Data types
    @discussion
        Definition of data types; these are mapped to AppleEvent types.
    @constant kICATypeUInt8
        UInt8.
    @constant kICATypeUInt16
        UInt16.
    @constant kICATypeUInt32
        UInt32.
    @constant kICATypeUInt64
        UInt64.
    @constant kICATypeSInt16
        SInt16.
    @constant kICATypeSInt32
        SInt32.
    @constant kICATypeSInt64
        SInt64.
    @constant kICATypeFloat
        float.
    @constant kICATypeFixed
        IEEE 32-bit floating point.
    @constant kICATypeBoolean
        Boolean.
    @constant kICATypeString
        Char string.
    @constant kICATypeData
        void *.
    @constant kICATypeThumbnail
        ICAThumbnail.
}
const
	kICATypeUInt8 = FourCharCode('ui08');
	kICATypeUInt16 = FourCharCode('ui16');
	kICATypeUInt32 = FourCharCode('ui32');
	kICATypeUInt64 = FourCharCode('ui64');
	kICATypeSInt16 = FourCharCode('si16');
	kICATypeSInt32 = FourCharCode('si32');
	kICATypeSInt64 = FourCharCode('si64');
	kICATypeFloat = FourCharCode('floa');
	kICATypeFixed = FourCharCode('sing');
	kICATypeBoolean = FourCharCode('bool');
	kICATypeString = FourCharCode('TEXT');
	kICATypeData = FourCharCode('data');
	kICATypeThumbnail = FourCharCode('thum');

//----------------------------------------------------------------------------------------------------- PropertyInfo flag values
{!
    @enum PropertyInfo flag values
    @discussion
        Values for PropertyInfo flag.
    @constant kICAFlagReadWriteAccess
        Access for read and write.
    @constant kICAFlagReadAccess
        Access for read only.
}
const
	kICAFlagReadWriteAccess = 1 shl 0;
	kICAFlagReadAccess = 1 shl 1;

//----------------------------------------------------------------------------------------------------------------- Button types
{!
    @enum Button types
    @discussion
        Buttons types associated with buttons on a scanner.
    @constant kICAButtonScan
        Scan button.
    @constant kICAButtonCopy
        Copy button.
    @constant kICAButtonEMail
        Email button.
    @constant kICAButtonWeb
        Web button.
}
const
	kICAButtonScan = FourCharCode('scan');
	kICAButtonCopy = FourCharCode('copy');
	kICAButtonEMail = FourCharCode('mail');
	kICAButtonWeb = FourCharCode('web ');

//------------------------------------------------------------------------ Flags associated with Image Capture PassThru commands
{!
    @enum Flags associated with Image Capture PassThru commands.
    @discussion
        Flag values that can be used in ICAUploadFilePB parameter block.
    @constant kICACameraPassThruSend
        Use this constant when sending data to a device using a pass-through command.
    @constant kICACameraPassThruReceive
        Use this constant when receiving data from a device using a pass-through command.
    @constant kICACameraPassThruNotUsed
        Use this constant when using a pass-through command that doesn't involve sending or receiving data.
}
const
	kICACameraPassThruSend = 0;
	kICACameraPassThruReceive = 1;
	kICACameraPassThruNotUsed = 2;

//---------------------------------------------------------------------------------------------------------- ICAPTPPassThroughPB
{!
    @struct ICAPTPPassThroughPB
    @field commandCode
        PTP command code (including vendor specific) <--
    @field resultCode
        PTP response code -->
    @field numOfInputParams
        Number of valid parameters to be sent to device <--
    @field numOfOutputParams
        Number of valid parameters expected from device <--
    @field params
        PTP parameters (command specific / optional) <->
    @field dataUsageMode
        One of (kICACameraPassThruSend, kICACameraPassThruReceive, kICACameraPassThruNotUsed) <--
    @field flags
        Not used currently
    @field dataSize
        Size of data block <->
    @field data
        Data block <->
}
type
	ICAPTPPassThroughPBPtr = ^ICAPTPPassThroughPB;
	ICAPTPPassThroughPB = record
		commandCode: UInt32;
		resultCode: UInt32;
		numOfInputParams: UInt32;
		numOfOutputParams: UInt32;
		params: array [0..4-1] of UInt32;
		dataUsageMode: UInt32;
		flags: UInt32;
		dataSize: UInt32;
		data: array [0..1-1] of UInt8;
	end;

//----------------------------------------------------------------------------------------------------------- ICAPTPEventDataset
{!
    @struct ICAPTPEventDataset
    @field dataLength
        Data length in bytes
    @field containerType
        PTP container type
    @field eventCode
        PTP event code
    @field transactionID
        PTP transaction ID
    @field params
        PTP params. The number of params should be (dataLength - 12)/4
}
type
	ICAPTPEventDatasetPtr = ^ICAPTPEventDataset;
	ICAPTPEventDataset = record
		dataLength: UInt32;
		containerType: UInt16;		// should be 0x0004 for event
		eventCode: UInt16;
		transactionID: UInt32;
		params: array [0..3-1] of UInt32;			// up to 3 params. # of params = (dataLength - 12)/4
	end;
<<<<<<< HEAD

//------------------------------------------------------------------------------------- Keys used in object property dictionary
{ Keys used in object property dictionary }
//#pragma mark -
//#pragma mark TODO: document the data types of values for these keys

// Keys returned by ICACopyObjectDictionary() for deviceList object returned by ICAGetDeviceList()

var kICADevicesArrayKey: CFStringRef; external name '_kICADevicesArrayKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICAObjectKey: CFStringRef; external name '_kICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAObjectNameKey: CFStringRef; external name '_kICAObjectNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAUSBVendorIDKey: CFStringRef; external name '_kICAUSBVendorIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAUSBProductIDKey: CFStringRef; external name '_kICAUSBProductIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceTypeKey: CFStringRef; external name '_kICADeviceTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAExecutableArchitectureKey: CFStringRef; external name '_kICAExecutableArchitectureKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICARemoteDeviceKey: CFStringRef; external name '_kICARemoteDeviceKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceSharedKey: CFStringRef; external name '_kICADeviceSharedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceWebSharedKey: CFStringRef; external name '_kICADeviceWebSharedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceUsedKey: CFStringRef; external name '_kICADeviceUsedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICABonjourServiceTypeKey: CFStringRef; external name '_kICABonjourServiceTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICABonjourServiceNameKey: CFStringRef; external name '_kICABonjourServiceNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICABonjourTXTRecordKey: CFStringRef; external name '_kICABonjourTXTRecordKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceCapabilitiesKey: CFStringRef; external name '_kICADeviceCapabilitiesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICALockStatusKey: CFStringRef; external name '_kICALockStatusKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICADataPropertyKey: CFStringRef; external name '_kICADataPropertyKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADataTypeKey: CFStringRef; external name '_kICADataTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADataSizeKey: CFStringRef; external name '_kICADataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAThumbnailPropertyKey: CFStringRef; external name '_kICAThumbnailPropertyKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAThumbnailSizeKey: CFStringRef; external name '_kICAThumbnailSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICARawKey: CFStringRef; external name '_kICARawKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICAMediaHeightKey: CFStringRef; external name '_kICAMediaHeightKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// value is a number
var kICAMediaWidthKey: CFStringRef; external name '_kICAMediaWidthKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// value is a number
var kICACreationDateStringKey: CFStringRef; external name '_kICACreationDateStringKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAModificationDateStringKey: CFStringRef; external name '_kICAModificationDateStringKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kMetaDataDictionaryKey: CFStringRef; external name '_kMetaDataDictionaryKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAMediaDurationInSecondsKey: CFStringRef; external name '_kICAMediaDurationInSecondsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICADeviceTypeCamera: CFStringRef; external name '_kICADeviceTypeCamera'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceTypeScanner: CFStringRef; external name '_kICADeviceTypeScanner'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
    In addition to the above, the following keys may also be present in the object property dictionay:
    
        kICAUSBLocationIDKey
        kICAFireWireGUIDKey
}

{ Transport types }
var kICAUSBTransportType: CFStringRef; external name '_kICAUSBTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAFireWireTransportType: CFStringRef; external name '_kICAFireWireTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICABluetoothTransportType: CFStringRef; external name '_kICABluetoothTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICATCPIPTransportType: CFStringRef; external name '_kICATCPIPTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICASCSITransportType: CFStringRef; external name '_kICASCSITransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICATWAINTransportType: CFStringRef; external name '_kICATWAINTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Keys used for paramDictionary in ICALoadDeviceModulePB}
var kICADeviceBrowserDeviceRefKey: CFStringRef; external name '_kICADeviceBrowserDeviceRefKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceModulePathKey: CFStringRef; external name '_kICADeviceModulePathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICADeviceIconPathKey: CFStringRef; external name '_kICADeviceIconPathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICATransportTypeKey: CFStringRef; external name '_kICATransportTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICABluetoothAddressKey: CFStringRef; external name '_kICABluetoothAddressKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAUSBLocationIDKey: CFStringRef; external name '_kICAUSBLocationIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAFireWireGUIDKey: CFStringRef; external name '_kICAFireWireGUIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIOServicePathKey: CFStringRef; external name '_kICAIOServicePathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPAddressKey: CFStringRef; external name '_kICAIPAddressKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPPortKey: CFStringRef; external name '_kICAIPPortKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPNameKey: CFStringRef; external name '_kICAIPNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPGUIDKey: CFStringRef; external name '_kICAIPGUIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICATWAINDSPathKey: CFStringRef; external name '_kICATWAINDSPathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
    @const
        kICAUserAssignedDeviceNameKey
    @abstract 
        This key may be present in the property dictionary of a device if the device has a user-assigned name.
    @discussion
        Value is of type CFStringRef.
}
var kICAUserAssignedDeviceNameKey: CFStringRef; external name '_kICAUserAssignedDeviceNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

//-------------------------------------------------------------------------------------------------------------------- ICAHeader
{!
    @struct ICAHeader
    @discussion
        This is the first field in all parameter blocks used by APIs defined in ICAApplication.h.
        Type of parameter passed to a callback function used by APIs defined in ICAApplication.h.
        The parameter for the completion proc should to be casted to an appropriate type such as ICAGetChildCountPB* for it to be useful.
    @field err
        Error returned by an API. -->
    @field refcon
        An arbitrary refcon value passed to the callback. <--
}
type
	ICAHeaderPtr = ^ICAHeader;
	ICAHeader = record
		err: ICAError;
		refcon: UNSIGNEDLONG;
	end;

//--------------------------------------------------------------------------------------------------------------- Callback procs

type
	ICACompletion = procedure( var pb: ICAHeader );

type
	ICAImportFilterProc = function( imageInfo: CFDictionaryRef; refcon: UNSIGNEDLONG ): Boolean;
=======

//------------------------------------------------------------------------------------- Keys used in object property dictionary
{ Keys used in object property dictionary }
//#pragma mark -
//#pragma mark TODO: document the data types of values for these keys

// Keys returned by ICACopyObjectDictionary() for deviceList object returned by ICAGetDeviceList()

var kICADevicesArrayKey: CFStringRef; external name '_kICADevicesArrayKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICAObjectKey: CFStringRef; external name '_kICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAObjectNameKey: CFStringRef; external name '_kICAObjectNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAUSBVendorIDKey: CFStringRef; external name '_kICAUSBVendorIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAUSBProductIDKey: CFStringRef; external name '_kICAUSBProductIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceTypeKey: CFStringRef; external name '_kICADeviceTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAExecutableArchitectureKey: CFStringRef; external name '_kICAExecutableArchitectureKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICARemoteDeviceKey: CFStringRef; external name '_kICARemoteDeviceKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceSharedKey: CFStringRef; external name '_kICADeviceSharedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceWebSharedKey: CFStringRef; external name '_kICADeviceWebSharedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceUsedKey: CFStringRef; external name '_kICADeviceUsedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICABonjourServiceTypeKey: CFStringRef; external name '_kICABonjourServiceTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICABonjourServiceNameKey: CFStringRef; external name '_kICABonjourServiceNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICABonjourTXTRecordKey: CFStringRef; external name '_kICABonjourTXTRecordKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceCapabilitiesKey: CFStringRef; external name '_kICADeviceCapabilitiesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICALockStatusKey: CFStringRef; external name '_kICALockStatusKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICADataPropertyKey: CFStringRef; external name '_kICADataPropertyKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADataTypeKey: CFStringRef; external name '_kICADataTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADataSizeKey: CFStringRef; external name '_kICADataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAThumbnailPropertyKey: CFStringRef; external name '_kICAThumbnailPropertyKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAThumbnailSizeKey: CFStringRef; external name '_kICAThumbnailSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICARawKey: CFStringRef; external name '_kICARawKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICAMediaHeightKey: CFStringRef; external name '_kICAMediaHeightKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// value is a number
var kICAMediaWidthKey: CFStringRef; external name '_kICAMediaWidthKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// value is a number
var kICACreationDateStringKey: CFStringRef; external name '_kICACreationDateStringKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAModificationDateStringKey: CFStringRef; external name '_kICAModificationDateStringKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kMetaDataDictionaryKey: CFStringRef; external name '_kMetaDataDictionaryKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAMediaDurationInSecondsKey: CFStringRef; external name '_kICAMediaDurationInSecondsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICADeviceTypeCamera: CFStringRef; external name '_kICADeviceTypeCamera'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceTypeScanner: CFStringRef; external name '_kICADeviceTypeScanner'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
    In addition to the above, the following keys may also be present in the object property dictionay:
    
        kICAUSBLocationIDKey
        kICAFireWireGUIDKey
}
>>>>>>> origin/cpstrnew

{ Transport types }
var kICAUSBTransportType: CFStringRef; external name '_kICAUSBTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAFireWireTransportType: CFStringRef; external name '_kICAFireWireTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICABluetoothTransportType: CFStringRef; external name '_kICABluetoothTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICATCPIPTransportType: CFStringRef; external name '_kICATCPIPTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICASCSITransportType: CFStringRef; external name '_kICASCSITransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICATWAINTransportType: CFStringRef; external name '_kICATWAINTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Keys used for paramDictionary in ICALoadDeviceModulePB}
var kICADeviceBrowserDeviceRefKey: CFStringRef; external name '_kICADeviceBrowserDeviceRefKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceModulePathKey: CFStringRef; external name '_kICADeviceModulePathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICADeviceIconPathKey: CFStringRef; external name '_kICADeviceIconPathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICATransportTypeKey: CFStringRef; external name '_kICATransportTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICABluetoothAddressKey: CFStringRef; external name '_kICABluetoothAddressKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAUSBLocationIDKey: CFStringRef; external name '_kICAUSBLocationIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAFireWireGUIDKey: CFStringRef; external name '_kICAFireWireGUIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIOServicePathKey: CFStringRef; external name '_kICAIOServicePathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPAddressKey: CFStringRef; external name '_kICAIPAddressKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPPortKey: CFStringRef; external name '_kICAIPPortKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPNameKey: CFStringRef; external name '_kICAIPNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPGUIDKey: CFStringRef; external name '_kICAIPGUIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICATWAINDSPathKey: CFStringRef; external name '_kICATWAINDSPathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
    @const
        kICAUserAssignedDeviceNameKey
    @abstract 
        This key may be present in the property dictionary of a device if the device has a user-assigned name.
    @discussion
        Value is of type CFStringRef.
}
var kICAUserAssignedDeviceNameKey: CFStringRef; external name '_kICAUserAssignedDeviceNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

//-------------------------------------------------------------------------------------------------------------------- ICAHeader
{!
    @struct ICAHeader
    @discussion
        This is the first field in all parameter blocks used by APIs defined in ICAApplication.h.
        Type of parameter passed to a callback function used by APIs defined in ICAApplication.h.
        The parameter for the completion proc should to be casted to an appropriate type such as ICAGetChildCountPB* for it to be useful.
    @field err
        Error returned by an API. -->
    @field refcon
        An arbitrary refcon value passed to the callback. <--
}
type
<<<<<<< HEAD
	ICANotificationProc = procedure( notificationType: CFStringRef; notificationDictionary: CFDictionaryRef );

//------------------------------------------------------------------------------------------------------------------- Object IDs

type
	ICAObject = UInt32;
	ICAProperty = UInt32;
	ICAConnectionID = UInt32;
	ICASessionID = UInt32;
	ICAScannerSessionID = ICASessionID;
	ICAEventDataCookie = UInt32;

//#pragma mark -
//#pragma mark General APIs
//--------------------------------------------------------------------------------------------------------------- ICAImportImage
{!
    @struct ICAObjectInfo
    @field objectType
        An object type, e.g., kICAFile.
    @field objectSubtype
        An object subtype, e.g., kICAFileImage.
}
type
	ICAObjectInfoPtr = ^ICAObjectInfo;
	ICAObjectInfo = record
		objectType: OSType;
		objectSubtype: OSType;
	end;

{!
    @enum ImportImage flags.
    @discussion
        Flag values that can be used in ICAImportImagePB parameter block.
    @constant kICAAllowMultipleImages
        Use this constant to allow users to select multiple images in the Import Image dialog.
    @constant kICADownloadAndReturnPathArray
        Use this constant to download the images to a temporary location and return an array of paths to the downloaded images.
}
const
	kICAAllowMultipleImages = $00000001;
	kICADownloadAndReturnPathArray = $00000002;

{!
    @struct ICAImportImagePB
    @field header
        See description for ICAHeader.  <->
    @field deviceObject
        Object ID of a camera or scanner device. Set this to NULL to ge the default behavior: (a) if no device is connected, a panel saying that there’s no device connected is displayed, (b) if a single device is connected, an appropriate user interface to access that device will be displayed, (c) if several devices are connected, a device selector panel will be displayed. <--
    @field flags
        One or more flags (combined with an OR operator) defined in ImportImage flags enum. <--
    @field supportedFileTypes
        An array of file extension strings such as "jpg", "tif", etc., that are of interest to the calling application. Set to NULL to display all files. <--
    @field filterProc
        Specify a filter proc to that will be called for each file before it is displayed in the user interface. <--
    @field importedImages
        Returns an array of CFDataRefs for the imported images if the kICADownloadAndReturnPathArray flag is not specified. Otherwise returns an array of CFStringRefs holding the paths of the images that are downloaded. The caller should provide a pointer to a CFArrayRef object initialized to NULL. The caller is responsible for released the array returned by this function. -->
}
type
	ICAImportImagePBPtr = ^ICAImportImagePB;
	ICAImportImagePB = record
		header: ICAHeader;
		deviceObject: ICAObject;
		flags: UInt32;
		supportedFileTypes: CFArrayRef;
		filterProc: ICAImportFilterProc;
		importedImages: CFArrayRefPtr;
	end;

{!
    @function ICAImportImage
    @abstract
        This API displays a Common User Interface panel similar to the user interface of Image Capture Application. This allows the user to work a camera or a scanner.
    @discussion
        Use this API to add Image Capture support to an application.

<pre>
@textblock
        Example:
        
        void ImportImage()
        (
            OSErr             err;
            CFArrayRef        imagesArray = NULL;
            ICAImportImagePB  pb = ();

            pb.deviceObject       = 0;
            pb.flags              = 0;
            pb.supportedFileTypes = (CFArrayRef)[NSArray arrayWithObjects: @"tif", @"tiff", @"jpg", NULL];
            pb.importedImages     = &imagesArray;
            
            err = ICAImportImage(&pb, NULL);

            if ( noErr != err )
            (
                // handle error
            )
            else
            (
                // Process the importedImages array
                // pb.importedImages   // CFArrayRef *
            )
        )
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICAImportImagePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAImportImage</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAImportImage( var pb: ICAImportImagePB; completion: ICACompletion ): ICAError; external name '_ICAImportImage';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew

//--------------------------------------------------------------------------------------------------------- ICAShowDeviceBrowser
{!
    @function ICAShowDeviceBrowser
    @abstract
        Use this API to display a device browser user interface from any Image Capture client application.
    @discussion
        The device browser user interface allows the user to do the following:
          - enable and disable sharing of locally connected cameras and scanners.
          - connect to or disconnect from cameras and scanners shared by other computers.
          - configure WiFi capable cameras for use over the WiFi network.
    @param options
        Set options to NULL to display the device browser with default settings. <--
        This parameter is intended for future use.
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAShowDeviceBrowser( options: CFDictionaryRef ): ICAError; external name '_ICAShowDeviceBrowser';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew

//---------------------------------------------------------------------------------------------- ICARegisterForEventNotification
// Function prototype for an Image Capture notification callback proc
type
	ICANotification = procedure( notificationType: CFStringRef; notificationDictionary: CFDictionaryRef );

{ The Image Capture notification callabck function will be called with a notificationDictionary that may
   contain one or more key-value pairs as defined below:
   
    Key                                 Value Type        Comments
    
    kICANotificationICAObjectKey        CFNumberRef       An object associated with the notification.
    kICANotificationDeviceICAObjectKey  CFNumberRef       A device object associated with the notification.
    kICANotificationClassKey            CFStringRef       See below.
    kICANotificationTypeKey             CFStringRef       See below.
    kICANotificationRawEventKey         CFNumberRef       The unprocesssed event code sent by a device.
    kICANotificationDataKey             CFDataRef         Data associated with the event.
    kICANotificationDataSizeKey         CFNumberRef       Size of data associated with the event. This is used if the data is
                                                          not sent with the notification. [Needed for backward compatiblity with pre-Leopard device modules].
    kICANotificationDataCookieKey       CFNumberRef       A token identifying the data associated with this event.
                                                          This data can be retrieved by calling ICAObjectSendMessage with messageType set to kICAMessageGetEventData, dataType set to value of kICANotificationDataCookieKeyand dataSize set to value of kICANotificationDataSizeKey.
                                          
    The following keys are present if the value of kICANotificationDataKey represents image data. The values of these
    keys are CFNumbers representing the width, height, bytes per row, start row, and number of rows of the image:
    
    kICANotificationImageKey              CFDictionaryRef A dictionary that describes an Image associated
                                                          with the notification.
    kICANotificationImageDataKey          CFDataRef       Image data
    kICANotificationImageWidthKey         CFNumberRef     Image width in pixels
    kICANotificationImageHeightKey        CFNumberRef     Image height in pixels
    kICANotificationImageBytesPerRowKey   CFNumberRef     Bytes per row in image
    kICANotificationImageStartRowKey      CFNumberRef     Starting row number of the image.
    kICANotificationImageNumberOfRowsKey  CFNumberRef     Number of rows of image data sent in this notification.
}

// Possible values for kICANotificationTypeKey:

var kICANotificationTypeObjectAdded: CFStringRef; external name '_kICANotificationTypeObjectAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeObjectRemoved: CFStringRef; external name '_kICANotificationTypeObjectRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeObjectInfoChanged: CFStringRef; external name '_kICANotificationTypeObjectInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeStoreAdded: CFStringRef; external name '_kICANotificationTypeStoreAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreRemoved: CFStringRef; external name '_kICANotificationTypeStoreRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreFull: CFStringRef; external name '_kICANotificationTypeStoreFull'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreInfoChanged: CFStringRef; external name '_kICANotificationTypeStoreInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceAdded: CFStringRef; external name '_kICANotificationTypeDeviceAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceRemoved: CFStringRef; external name '_kICANotificationTypeDeviceRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceInfoChanged: CFStringRef; external name '_kICANotificationTypeDeviceInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDevicePropertyChanged: CFStringRef; external name '_kICANotificationTypeDevicePropertyChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceWasReset: CFStringRef; external name '_kICANotificationTypeDeviceWasReset'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceStatusInfo: CFStringRef; external name '_kICANotificationTypeDeviceStatusInfo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceStatusError: CFStringRef; external name '_kICANotificationTypeDeviceStatusError'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeCaptureComplete: CFStringRef; external name '_kICANotificationTypeCaptureComplete'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeRequestObjectTransfer: CFStringRef; external name '_kICANotificationTypeRequestObjectTransfer'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeTransactionCanceled: CFStringRef; external name '_kICANotificationTypeTransactionCanceled'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeUnreportedStatus: CFStringRef; external name '_kICANotificationTypeUnreportedStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeProprietary: CFStringRef; external name '_kICANotificationTypeProprietary'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceConnectionProgress: CFStringRef; external name '_kICANotificationTypeDeviceConnectionProgress'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDownloadProgressStatus: CFStringRef; external name '_kICANotificationTypeDownloadProgressStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScanProgressStatus: CFStringRef; external name '_kICANotificationTypeScanProgressStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerSessionClosed: CFStringRef; external name '_kICANotificationTypeScannerSessionClosed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerScanDone: CFStringRef; external name '_kICANotificationTypeScannerScanDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerPageDone: CFStringRef; external name '_kICANotificationTypeScannerPageDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerButtonPressed: CFStringRef; external name '_kICANotificationTypeScannerButtonPressed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeScannerOverviewOverlayAvailable: CFStringRef; external name '_kICANotificationTypeScannerOverviewOverlayAvailable'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

// Possible keys in the notification dictionary:

var kICAErrorKey: CFStringRef; external name '_kICAErrorKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICARefconKey: CFStringRef; external name '_kICARefconKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationICAObjectKey: CFStringRef; external name '_kICANotificationICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDeviceICAObjectKey: CFStringRef; external name '_kICANotificationDeviceICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDeviceListICAObjectKey: CFStringRef; external name '_kICANotificationDeviceListICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassKey: CFStringRef; external name '_kICANotificationClassKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeKey: CFStringRef; external name '_kICANotificationTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationRawEventKey: CFStringRef; external name '_kICANotificationRawEventKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataKey: CFStringRef; external name '_kICANotificationDataKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataSizeKey: CFStringRef; external name '_kICANotificationDataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataCookieKey: CFStringRef; external name '_kICANotificationDataCookieKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationImageKey: CFStringRef; external name '_kICANotificationImageKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageWidthKey: CFStringRef; external name '_kICANotificationImageWidthKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageHeightKey: CFStringRef; external name '_kICANotificationImageHeightKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageBytesPerRowKey: CFStringRef; external name '_kICANotificationImageBytesPerRowKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageStartRowKey: CFStringRef; external name '_kICANotificationImageStartRowKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageNumberOfRowsKey: CFStringRef; external name '_kICANotificationImageNumberOfRowsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageDataKey: CFStringRef; external name '_kICANotificationImageDataKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageDataSizeKey: CFStringRef; external name '_kICANotificationImageDataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataIsBigEndianKey: CFStringRef; external name '_kICANotificationDataIsBigEndianKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationScannerDocumentNameKey: CFStringRef; external name '_kICANotificationScannerDocumentNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationScannerButtonTypeKey: CFStringRef; external name '_kICANotificationScannerButtonTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationNumerOfImagesRemainingKey: CFStringRef; external name '_kICANotificationNumerOfImagesRemainingKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationPercentDownloadedKey: CFStringRef; external name '_kICANotificationPercentDownloadedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationSubTypeKey: CFStringRef; external name '_kICANotificationSubTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypeWarmUpStarted: CFStringRef; external name '_kICANotificationSubTypeWarmUpStarted'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypeWarmUpDone: CFStringRef; external name '_kICANotificationSubTypeWarmUpDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationVendorErrorCodeKey: CFStringRef; external name '_kICANotificationVendorErrorCodeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypePerformOverviewScan: CFStringRef; external name '_kICANotificationSubTypePerformOverviewScan'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
var kICANotificationSubTypeDocumentLoaded: CFStringRef; external name '_kICANotificationSubTypeDocumentLoaded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
var kICANotificationSubTypeDocumentNotLoaded: CFStringRef; external name '_kICANotificationSubTypeDocumentNotLoaded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

// Possible values in the notification dictionary:
// ...

// Possible values for kICANotificationClassKey
var kICANotificationClassPTPStandard: CFStringRef; external name '_kICANotificationClassPTPStandard'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassPTPVendor: CFStringRef; external name '_kICANotificationClassPTPVendor'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassProprietary: CFStringRef; external name '_kICANotificationClassProprietary'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

// Device Properties

var kICADevicePropUndefined: CFStringRef; external name '_kICADevicePropUndefined'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBatteryLevel: CFStringRef; external name '_kICADevicePropBatteryLevel'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFunctionalMode: CFStringRef; external name '_kICADevicePropFunctionalMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropImageSize: CFStringRef; external name '_kICADevicePropImageSize'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCompressionSetting: CFStringRef; external name '_kICADevicePropCompressionSetting'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropWhiteBalance: CFStringRef; external name '_kICADevicePropWhiteBalance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropRGBGain: CFStringRef; external name '_kICADevicePropRGBGain'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFNumber: CFStringRef; external name '_kICADevicePropFNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocalLength: CFStringRef; external name '_kICADevicePropFocalLength'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusDistance: CFStringRef; external name '_kICADevicePropFocusDistance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusMode: CFStringRef; external name '_kICADevicePropFocusMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureMeteringMode: CFStringRef; external name '_kICADevicePropExposureMeteringMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFlashMode: CFStringRef; external name '_kICADevicePropFlashMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureTime: CFStringRef; external name '_kICADevicePropExposureTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureProgramMode: CFStringRef; external name '_kICADevicePropExposureProgramMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureIndex: CFStringRef; external name '_kICADevicePropExposureIndex'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureBiasCompensation: CFStringRef; external name '_kICADevicePropExposureBiasCompensation'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropDateTime: CFStringRef; external name '_kICADevicePropDateTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCaptureDelay: CFStringRef; external name '_kICADevicePropCaptureDelay'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropStillCaptureMode: CFStringRef; external name '_kICADevicePropStillCaptureMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropContrast: CFStringRef; external name '_kICADevicePropContrast'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropSharpness: CFStringRef; external name '_kICADevicePropSharpness'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropDigitalZoom: CFStringRef; external name '_kICADevicePropDigitalZoom'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropEffectMode: CFStringRef; external name '_kICADevicePropEffectMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBurstNumber: CFStringRef; external name '_kICADevicePropBurstNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBurstInterval: CFStringRef; external name '_kICADevicePropBurstInterval'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropTimelapseNumber: CFStringRef; external name '_kICADevicePropTimelapseNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropTimelapseInterval: CFStringRef; external name '_kICADevicePropTimelapseInterval'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusMeteringMode: CFStringRef; external name '_kICADevicePropFocusMeteringMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropUploadURL: CFStringRef; external name '_kICADevicePropUploadURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropArtist: CFStringRef; external name '_kICADevicePropArtist'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCopyrightInfo: CFStringRef; external name '_kICADevicePropCopyrightInfo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{!
    @struct ICARegisterForEventNotificationPB
    @discussion
        Use this parameter structure to specify a set of events associated with an object
        about which notifications should be sent to the specified notification function.
    @field header
        See description for ICAHeader. <->
    @field objectOfInterest
        An object about which notifications are requested. <--
    @field eventsOfInterest
        An array of notification types of interest. <--
    @field notificationProc
        A callback function to receive the notifications. <--
    @field options
        Set options to NULL. This parameter is intended for future use. <--
}

type
	ICARegisterForEventNotificationPBPtr = ^ICARegisterForEventNotificationPB;
	ICARegisterForEventNotificationPB = record
		header: ICAHeader;
		objectOfInterest: ICAObject;
		eventsOfInterest: CFArrayRef;
		notificationProc: ICANotification;
		options: CFDictionaryRef;
	end;

{!
    @function ICARegisterForEventNotification
    @abstract
        Use this API to register with Image Capture framework to receive
        notification about events of interest.
    @param params
        A pointer to ICARegisterForEventNotificationPB struct <--
    @param completionProc
        A pointer to a completion routine that will be invoked at the completion of
        this function. Set this parameter to NULL to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICARegisterForEventNotification( var params: ICARegisterForEventNotificationPB; completionProc: ICACompletion ): ICAError; external name '_ICARegisterForEventNotification';
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew

=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

//---------------------------------------------------------------------------------------------- ICARegisterForEventNotification
// Function prototype for an Image Capture notification callback proc
type
	ICANotification = procedure( notificationType: CFStringRef; notificationDictionary: CFDictionaryRef );

{ The Image Capture notification callabck function will be called with a notificationDictionary that may
   contain one or more key-value pairs as defined below:
   
    Key                                 Value Type        Comments
    
    kICANotificationICAObjectKey        CFNumberRef       An object associated with the notification.
    kICANotificationDeviceICAObjectKey  CFNumberRef       A device object associated with the notification.
    kICANotificationClassKey            CFStringRef       See below.
    kICANotificationTypeKey             CFStringRef       See below.
    kICANotificationRawEventKey         CFNumberRef       The unprocesssed event code sent by a device.
    kICANotificationDataKey             CFDataRef         Data associated with the event.
    kICANotificationDataSizeKey         CFNumberRef       Size of data associated with the event. This is used if the data is
                                                          not sent with the notification. [Needed for backward compatiblity with pre-Leopard device modules].
    kICANotificationDataCookieKey       CFNumberRef       A token identifying the data associated with this event.
                                                          This data can be retrieved by calling ICAObjectSendMessage with messageType set to kICAMessageGetEventData, dataType set to value of kICANotificationDataCookieKeyand dataSize set to value of kICANotificationDataSizeKey.
                                          
    The following keys are present if the value of kICANotificationDataKey represents image data. The values of these
    keys are CFNumbers representing the width, height, bytes per row, start row, and number of rows of the image:
    
    kICANotificationImageKey              CFDictionaryRef A dictionary that describes an Image associated
                                                          with the notification.
    kICANotificationImageDataKey          CFDataRef       Image data
    kICANotificationImageWidthKey         CFNumberRef     Image width in pixels
    kICANotificationImageHeightKey        CFNumberRef     Image height in pixels
    kICANotificationImageBytesPerRowKey   CFNumberRef     Bytes per row in image
    kICANotificationImageStartRowKey      CFNumberRef     Starting row number of the image.
    kICANotificationImageNumberOfRowsKey  CFNumberRef     Number of rows of image data sent in this notification.
}

// Possible values for kICANotificationTypeKey:

var kICANotificationTypeObjectAdded: CFStringRef; external name '_kICANotificationTypeObjectAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeObjectRemoved: CFStringRef; external name '_kICANotificationTypeObjectRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeObjectInfoChanged: CFStringRef; external name '_kICANotificationTypeObjectInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeStoreAdded: CFStringRef; external name '_kICANotificationTypeStoreAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreRemoved: CFStringRef; external name '_kICANotificationTypeStoreRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreFull: CFStringRef; external name '_kICANotificationTypeStoreFull'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreInfoChanged: CFStringRef; external name '_kICANotificationTypeStoreInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceAdded: CFStringRef; external name '_kICANotificationTypeDeviceAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceRemoved: CFStringRef; external name '_kICANotificationTypeDeviceRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceInfoChanged: CFStringRef; external name '_kICANotificationTypeDeviceInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDevicePropertyChanged: CFStringRef; external name '_kICANotificationTypeDevicePropertyChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceWasReset: CFStringRef; external name '_kICANotificationTypeDeviceWasReset'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceStatusInfo: CFStringRef; external name '_kICANotificationTypeDeviceStatusInfo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceStatusError: CFStringRef; external name '_kICANotificationTypeDeviceStatusError'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeCaptureComplete: CFStringRef; external name '_kICANotificationTypeCaptureComplete'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeRequestObjectTransfer: CFStringRef; external name '_kICANotificationTypeRequestObjectTransfer'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeTransactionCanceled: CFStringRef; external name '_kICANotificationTypeTransactionCanceled'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeUnreportedStatus: CFStringRef; external name '_kICANotificationTypeUnreportedStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeProprietary: CFStringRef; external name '_kICANotificationTypeProprietary'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceConnectionProgress: CFStringRef; external name '_kICANotificationTypeDeviceConnectionProgress'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDownloadProgressStatus: CFStringRef; external name '_kICANotificationTypeDownloadProgressStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScanProgressStatus: CFStringRef; external name '_kICANotificationTypeScanProgressStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerSessionClosed: CFStringRef; external name '_kICANotificationTypeScannerSessionClosed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerScanDone: CFStringRef; external name '_kICANotificationTypeScannerScanDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerPageDone: CFStringRef; external name '_kICANotificationTypeScannerPageDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerButtonPressed: CFStringRef; external name '_kICANotificationTypeScannerButtonPressed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeScannerOverviewOverlayAvailable: CFStringRef; external name '_kICANotificationTypeScannerOverviewOverlayAvailable'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

// Possible keys in the notification dictionary:

var kICAErrorKey: CFStringRef; external name '_kICAErrorKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICARefconKey: CFStringRef; external name '_kICARefconKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationICAObjectKey: CFStringRef; external name '_kICANotificationICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDeviceICAObjectKey: CFStringRef; external name '_kICANotificationDeviceICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDeviceListICAObjectKey: CFStringRef; external name '_kICANotificationDeviceListICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassKey: CFStringRef; external name '_kICANotificationClassKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeKey: CFStringRef; external name '_kICANotificationTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationRawEventKey: CFStringRef; external name '_kICANotificationRawEventKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataKey: CFStringRef; external name '_kICANotificationDataKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataSizeKey: CFStringRef; external name '_kICANotificationDataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataCookieKey: CFStringRef; external name '_kICANotificationDataCookieKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationImageKey: CFStringRef; external name '_kICANotificationImageKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageWidthKey: CFStringRef; external name '_kICANotificationImageWidthKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageHeightKey: CFStringRef; external name '_kICANotificationImageHeightKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageBytesPerRowKey: CFStringRef; external name '_kICANotificationImageBytesPerRowKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageStartRowKey: CFStringRef; external name '_kICANotificationImageStartRowKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageNumberOfRowsKey: CFStringRef; external name '_kICANotificationImageNumberOfRowsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageDataKey: CFStringRef; external name '_kICANotificationImageDataKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageDataSizeKey: CFStringRef; external name '_kICANotificationImageDataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataIsBigEndianKey: CFStringRef; external name '_kICANotificationDataIsBigEndianKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationScannerDocumentNameKey: CFStringRef; external name '_kICANotificationScannerDocumentNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationScannerButtonTypeKey: CFStringRef; external name '_kICANotificationScannerButtonTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationNumerOfImagesRemainingKey: CFStringRef; external name '_kICANotificationNumerOfImagesRemainingKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationPercentDownloadedKey: CFStringRef; external name '_kICANotificationPercentDownloadedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationSubTypeKey: CFStringRef; external name '_kICANotificationSubTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypeWarmUpStarted: CFStringRef; external name '_kICANotificationSubTypeWarmUpStarted'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypeWarmUpDone: CFStringRef; external name '_kICANotificationSubTypeWarmUpDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationVendorErrorCodeKey: CFStringRef; external name '_kICANotificationVendorErrorCodeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypePerformOverviewScan: CFStringRef; external name '_kICANotificationSubTypePerformOverviewScan'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
var kICANotificationSubTypeDocumentLoaded: CFStringRef; external name '_kICANotificationSubTypeDocumentLoaded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
var kICANotificationSubTypeDocumentNotLoaded: CFStringRef; external name '_kICANotificationSubTypeDocumentNotLoaded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

// Possible values in the notification dictionary:
// ...

// Possible values for kICANotificationClassKey
var kICANotificationClassPTPStandard: CFStringRef; external name '_kICANotificationClassPTPStandard'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassPTPVendor: CFStringRef; external name '_kICANotificationClassPTPVendor'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassProprietary: CFStringRef; external name '_kICANotificationClassProprietary'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

// Device Properties

var kICADevicePropUndefined: CFStringRef; external name '_kICADevicePropUndefined'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBatteryLevel: CFStringRef; external name '_kICADevicePropBatteryLevel'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFunctionalMode: CFStringRef; external name '_kICADevicePropFunctionalMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropImageSize: CFStringRef; external name '_kICADevicePropImageSize'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCompressionSetting: CFStringRef; external name '_kICADevicePropCompressionSetting'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropWhiteBalance: CFStringRef; external name '_kICADevicePropWhiteBalance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropRGBGain: CFStringRef; external name '_kICADevicePropRGBGain'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFNumber: CFStringRef; external name '_kICADevicePropFNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocalLength: CFStringRef; external name '_kICADevicePropFocalLength'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusDistance: CFStringRef; external name '_kICADevicePropFocusDistance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusMode: CFStringRef; external name '_kICADevicePropFocusMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureMeteringMode: CFStringRef; external name '_kICADevicePropExposureMeteringMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFlashMode: CFStringRef; external name '_kICADevicePropFlashMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureTime: CFStringRef; external name '_kICADevicePropExposureTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureProgramMode: CFStringRef; external name '_kICADevicePropExposureProgramMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureIndex: CFStringRef; external name '_kICADevicePropExposureIndex'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureBiasCompensation: CFStringRef; external name '_kICADevicePropExposureBiasCompensation'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropDateTime: CFStringRef; external name '_kICADevicePropDateTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCaptureDelay: CFStringRef; external name '_kICADevicePropCaptureDelay'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropStillCaptureMode: CFStringRef; external name '_kICADevicePropStillCaptureMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropContrast: CFStringRef; external name '_kICADevicePropContrast'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropSharpness: CFStringRef; external name '_kICADevicePropSharpness'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropDigitalZoom: CFStringRef; external name '_kICADevicePropDigitalZoom'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropEffectMode: CFStringRef; external name '_kICADevicePropEffectMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBurstNumber: CFStringRef; external name '_kICADevicePropBurstNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBurstInterval: CFStringRef; external name '_kICADevicePropBurstInterval'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropTimelapseNumber: CFStringRef; external name '_kICADevicePropTimelapseNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropTimelapseInterval: CFStringRef; external name '_kICADevicePropTimelapseInterval'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusMeteringMode: CFStringRef; external name '_kICADevicePropFocusMeteringMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropUploadURL: CFStringRef; external name '_kICADevicePropUploadURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropArtist: CFStringRef; external name '_kICADevicePropArtist'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCopyrightInfo: CFStringRef; external name '_kICADevicePropCopyrightInfo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{!
    @struct ICARegisterForEventNotificationPB
    @discussion
        Use this parameter structure to specify a set of events associated with an object
        about which notifications should be sent to the specified notification function.
    @field header
        See description for ICAHeader. <->
    @field objectOfInterest
        An object about which notifications are requested. <--
    @field eventsOfInterest
        An array of notification types of interest. <--
    @field notificationProc
        A callback function to receive the notifications. <--
    @field options
        Set options to NULL. This parameter is intended for future use. <--
}

type
	ICARegisterForEventNotificationPBPtr = ^ICARegisterForEventNotificationPB;
	ICARegisterForEventNotificationPB = record
		header: ICAHeader;
		objectOfInterest: ICAObject;
		eventsOfInterest: CFArrayRef;
		notificationProc: ICANotification;
		options: CFDictionaryRef;
	end;

{!
    @function ICARegisterForEventNotification
    @abstract
        Use this API to register with Image Capture framework to receive
        notification about events of interest.
    @param params
        A pointer to ICARegisterForEventNotificationPB struct <--
    @param completionProc
        A pointer to a completion routine that will be invoked at the completion of
        this function. Set this parameter to NULL to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICARegisterForEventNotification( var params: ICARegisterForEventNotificationPB; completionProc: ICACompletion ): ICAError; external name '_ICARegisterForEventNotification';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
//---------------------------------------------------------------------------------------------------------- ICASendNotification
// This parameter block is used with 'ICDSendNotification' and 'ICDSendNotificationAndWaitForReply' APIs defined
// in ICADevices.framework

=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

//---------------------------------------------------------------------------------------------- ICARegisterForEventNotification
// Function prototype for an Image Capture notification callback proc
type
	ICANotification = procedure( notificationType: CFStringRef; notificationDictionary: CFDictionaryRef );

{ The Image Capture notification callabck function will be called with a notificationDictionary that may
   contain one or more key-value pairs as defined below:
   
    Key                                 Value Type        Comments
    
    kICANotificationICAObjectKey        CFNumberRef       An object associated with the notification.
    kICANotificationDeviceICAObjectKey  CFNumberRef       A device object associated with the notification.
    kICANotificationClassKey            CFStringRef       See below.
    kICANotificationTypeKey             CFStringRef       See below.
    kICANotificationRawEventKey         CFNumberRef       The unprocesssed event code sent by a device.
    kICANotificationDataKey             CFDataRef         Data associated with the event.
    kICANotificationDataSizeKey         CFNumberRef       Size of data associated with the event. This is used if the data is
                                                          not sent with the notification. [Needed for backward compatiblity with pre-Leopard device modules].
    kICANotificationDataCookieKey       CFNumberRef       A token identifying the data associated with this event.
                                                          This data can be retrieved by calling ICAObjectSendMessage with messageType set to kICAMessageGetEventData, dataType set to value of kICANotificationDataCookieKeyand dataSize set to value of kICANotificationDataSizeKey.
                                          
    The following keys are present if the value of kICANotificationDataKey represents image data. The values of these
    keys are CFNumbers representing the width, height, bytes per row, start row, and number of rows of the image:
    
    kICANotificationImageKey              CFDictionaryRef A dictionary that describes an Image associated
                                                          with the notification.
    kICANotificationImageDataKey          CFDataRef       Image data
    kICANotificationImageWidthKey         CFNumberRef     Image width in pixels
    kICANotificationImageHeightKey        CFNumberRef     Image height in pixels
    kICANotificationImageBytesPerRowKey   CFNumberRef     Bytes per row in image
    kICANotificationImageStartRowKey      CFNumberRef     Starting row number of the image.
    kICANotificationImageNumberOfRowsKey  CFNumberRef     Number of rows of image data sent in this notification.
}

// Possible values for kICANotificationTypeKey:

var kICANotificationTypeObjectAdded: CFStringRef; external name '_kICANotificationTypeObjectAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeObjectRemoved: CFStringRef; external name '_kICANotificationTypeObjectRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeObjectInfoChanged: CFStringRef; external name '_kICANotificationTypeObjectInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeStoreAdded: CFStringRef; external name '_kICANotificationTypeStoreAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreRemoved: CFStringRef; external name '_kICANotificationTypeStoreRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreFull: CFStringRef; external name '_kICANotificationTypeStoreFull'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreInfoChanged: CFStringRef; external name '_kICANotificationTypeStoreInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceAdded: CFStringRef; external name '_kICANotificationTypeDeviceAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceRemoved: CFStringRef; external name '_kICANotificationTypeDeviceRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceInfoChanged: CFStringRef; external name '_kICANotificationTypeDeviceInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDevicePropertyChanged: CFStringRef; external name '_kICANotificationTypeDevicePropertyChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceWasReset: CFStringRef; external name '_kICANotificationTypeDeviceWasReset'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceStatusInfo: CFStringRef; external name '_kICANotificationTypeDeviceStatusInfo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceStatusError: CFStringRef; external name '_kICANotificationTypeDeviceStatusError'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeCaptureComplete: CFStringRef; external name '_kICANotificationTypeCaptureComplete'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeRequestObjectTransfer: CFStringRef; external name '_kICANotificationTypeRequestObjectTransfer'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeTransactionCanceled: CFStringRef; external name '_kICANotificationTypeTransactionCanceled'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeUnreportedStatus: CFStringRef; external name '_kICANotificationTypeUnreportedStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeProprietary: CFStringRef; external name '_kICANotificationTypeProprietary'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceConnectionProgress: CFStringRef; external name '_kICANotificationTypeDeviceConnectionProgress'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDownloadProgressStatus: CFStringRef; external name '_kICANotificationTypeDownloadProgressStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScanProgressStatus: CFStringRef; external name '_kICANotificationTypeScanProgressStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerSessionClosed: CFStringRef; external name '_kICANotificationTypeScannerSessionClosed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerScanDone: CFStringRef; external name '_kICANotificationTypeScannerScanDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerPageDone: CFStringRef; external name '_kICANotificationTypeScannerPageDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerButtonPressed: CFStringRef; external name '_kICANotificationTypeScannerButtonPressed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeScannerOverviewOverlayAvailable: CFStringRef; external name '_kICANotificationTypeScannerOverviewOverlayAvailable'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

// Possible keys in the notification dictionary:

var kICAErrorKey: CFStringRef; external name '_kICAErrorKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICARefconKey: CFStringRef; external name '_kICARefconKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationICAObjectKey: CFStringRef; external name '_kICANotificationICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDeviceICAObjectKey: CFStringRef; external name '_kICANotificationDeviceICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDeviceListICAObjectKey: CFStringRef; external name '_kICANotificationDeviceListICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassKey: CFStringRef; external name '_kICANotificationClassKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeKey: CFStringRef; external name '_kICANotificationTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationRawEventKey: CFStringRef; external name '_kICANotificationRawEventKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataKey: CFStringRef; external name '_kICANotificationDataKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataSizeKey: CFStringRef; external name '_kICANotificationDataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataCookieKey: CFStringRef; external name '_kICANotificationDataCookieKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationImageKey: CFStringRef; external name '_kICANotificationImageKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageWidthKey: CFStringRef; external name '_kICANotificationImageWidthKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageHeightKey: CFStringRef; external name '_kICANotificationImageHeightKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageBytesPerRowKey: CFStringRef; external name '_kICANotificationImageBytesPerRowKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageStartRowKey: CFStringRef; external name '_kICANotificationImageStartRowKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageNumberOfRowsKey: CFStringRef; external name '_kICANotificationImageNumberOfRowsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageDataKey: CFStringRef; external name '_kICANotificationImageDataKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageDataSizeKey: CFStringRef; external name '_kICANotificationImageDataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataIsBigEndianKey: CFStringRef; external name '_kICANotificationDataIsBigEndianKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationScannerDocumentNameKey: CFStringRef; external name '_kICANotificationScannerDocumentNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationScannerButtonTypeKey: CFStringRef; external name '_kICANotificationScannerButtonTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationNumerOfImagesRemainingKey: CFStringRef; external name '_kICANotificationNumerOfImagesRemainingKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationPercentDownloadedKey: CFStringRef; external name '_kICANotificationPercentDownloadedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationSubTypeKey: CFStringRef; external name '_kICANotificationSubTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypeWarmUpStarted: CFStringRef; external name '_kICANotificationSubTypeWarmUpStarted'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypeWarmUpDone: CFStringRef; external name '_kICANotificationSubTypeWarmUpDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationVendorErrorCodeKey: CFStringRef; external name '_kICANotificationVendorErrorCodeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypePerformOverviewScan: CFStringRef; external name '_kICANotificationSubTypePerformOverviewScan'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
var kICANotificationSubTypeDocumentLoaded: CFStringRef; external name '_kICANotificationSubTypeDocumentLoaded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
var kICANotificationSubTypeDocumentNotLoaded: CFStringRef; external name '_kICANotificationSubTypeDocumentNotLoaded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

// Possible values in the notification dictionary:
// ...

// Possible values for kICANotificationClassKey
var kICANotificationClassPTPStandard: CFStringRef; external name '_kICANotificationClassPTPStandard'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassPTPVendor: CFStringRef; external name '_kICANotificationClassPTPVendor'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassProprietary: CFStringRef; external name '_kICANotificationClassProprietary'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

// Device Properties

var kICADevicePropUndefined: CFStringRef; external name '_kICADevicePropUndefined'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBatteryLevel: CFStringRef; external name '_kICADevicePropBatteryLevel'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFunctionalMode: CFStringRef; external name '_kICADevicePropFunctionalMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropImageSize: CFStringRef; external name '_kICADevicePropImageSize'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCompressionSetting: CFStringRef; external name '_kICADevicePropCompressionSetting'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropWhiteBalance: CFStringRef; external name '_kICADevicePropWhiteBalance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropRGBGain: CFStringRef; external name '_kICADevicePropRGBGain'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFNumber: CFStringRef; external name '_kICADevicePropFNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocalLength: CFStringRef; external name '_kICADevicePropFocalLength'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusDistance: CFStringRef; external name '_kICADevicePropFocusDistance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusMode: CFStringRef; external name '_kICADevicePropFocusMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureMeteringMode: CFStringRef; external name '_kICADevicePropExposureMeteringMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFlashMode: CFStringRef; external name '_kICADevicePropFlashMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureTime: CFStringRef; external name '_kICADevicePropExposureTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureProgramMode: CFStringRef; external name '_kICADevicePropExposureProgramMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureIndex: CFStringRef; external name '_kICADevicePropExposureIndex'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureBiasCompensation: CFStringRef; external name '_kICADevicePropExposureBiasCompensation'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropDateTime: CFStringRef; external name '_kICADevicePropDateTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCaptureDelay: CFStringRef; external name '_kICADevicePropCaptureDelay'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropStillCaptureMode: CFStringRef; external name '_kICADevicePropStillCaptureMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropContrast: CFStringRef; external name '_kICADevicePropContrast'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropSharpness: CFStringRef; external name '_kICADevicePropSharpness'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropDigitalZoom: CFStringRef; external name '_kICADevicePropDigitalZoom'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropEffectMode: CFStringRef; external name '_kICADevicePropEffectMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBurstNumber: CFStringRef; external name '_kICADevicePropBurstNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBurstInterval: CFStringRef; external name '_kICADevicePropBurstInterval'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropTimelapseNumber: CFStringRef; external name '_kICADevicePropTimelapseNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropTimelapseInterval: CFStringRef; external name '_kICADevicePropTimelapseInterval'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusMeteringMode: CFStringRef; external name '_kICADevicePropFocusMeteringMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropUploadURL: CFStringRef; external name '_kICADevicePropUploadURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropArtist: CFStringRef; external name '_kICADevicePropArtist'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCopyrightInfo: CFStringRef; external name '_kICADevicePropCopyrightInfo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{!
    @struct ICARegisterForEventNotificationPB
    @discussion
        Use this parameter structure to specify a set of events associated with an object
        about which notifications should be sent to the specified notification function.
    @field header
        See description for ICAHeader. <->
    @field objectOfInterest
        An object about which notifications are requested. <--
    @field eventsOfInterest
        An array of notification types of interest. <--
    @field notificationProc
        A callback function to receive the notifications. <--
    @field options
        Set options to NULL. This parameter is intended for future use. <--
}

type
	ICARegisterForEventNotificationPBPtr = ^ICARegisterForEventNotificationPB;
	ICARegisterForEventNotificationPB = record
		header: ICAHeader;
		objectOfInterest: ICAObject;
		eventsOfInterest: CFArrayRef;
		notificationProc: ICANotification;
		options: CFDictionaryRef;
	end;

{!
    @function ICARegisterForEventNotification
    @abstract
        Use this API to register with Image Capture framework to receive
        notification about events of interest.
    @param params
        A pointer to ICARegisterForEventNotificationPB struct <--
    @param completionProc
        A pointer to a completion routine that will be invoked at the completion of
        this function. Set this parameter to NULL to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICARegisterForEventNotification( var params: ICARegisterForEventNotificationPB; completionProc: ICACompletion ): ICAError; external name '_ICARegisterForEventNotification';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

//---------------------------------------------------------------------------------------------------------- ICASendNotification
// This parameter block is used with 'ICDSendNotification' and 'ICDSendNotificationAndWaitForReply' APIs defined
// in ICADevices.framework

>>>>>>> graemeg/cpstrnew
=======
	ICAHeaderPtr = ^ICAHeader;
	ICAHeader = record
		err: ICAError;
		refcon: UNSIGNEDLONG;
	end;

//--------------------------------------------------------------------------------------------------------------- Callback procs

type
	ICACompletion = procedure( var pb: ICAHeader );

type
	ICAImportFilterProc = function( imageInfo: CFDictionaryRef; refcon: UNSIGNEDLONG ): Boolean;

type
	ICANotificationProc = procedure( notificationType: CFStringRef; notificationDictionary: CFDictionaryRef );

//------------------------------------------------------------------------------------------------------------------- Object IDs

type
	ICAObject = UInt32;
	ICAProperty = UInt32;
	ICAConnectionID = UInt32;
	ICASessionID = UInt32;
	ICAScannerSessionID = ICASessionID;
	ICAEventDataCookie = UInt32;

//#pragma mark -
//#pragma mark General APIs
//--------------------------------------------------------------------------------------------------------------- ICAImportImage
{!
    @struct ICAObjectInfo
    @field objectType
        An object type, e.g., kICAFile.
    @field objectSubtype
        An object subtype, e.g., kICAFileImage.
}
type
	ICAObjectInfoPtr = ^ICAObjectInfo;
	ICAObjectInfo = record
		objectType: OSType;
		objectSubtype: OSType;
	end;

{!
    @enum ImportImage flags.
    @discussion
        Flag values that can be used in ICAImportImagePB parameter block.
    @constant kICAAllowMultipleImages
        Use this constant to allow users to select multiple images in the Import Image dialog.
    @constant kICADownloadAndReturnPathArray
        Use this constant to download the images to a temporary location and return an array of paths to the downloaded images.
}
const
	kICAAllowMultipleImages = $00000001;
	kICADownloadAndReturnPathArray = $00000002;

{!
    @struct ICAImportImagePB
    @field header
        See description for ICAHeader.  <->
    @field deviceObject
        Object ID of a camera or scanner device. Set this to NULL to ge the default behavior: (a) if no device is connected, a panel saying that there’s no device connected is displayed, (b) if a single device is connected, an appropriate user interface to access that device will be displayed, (c) if several devices are connected, a device selector panel will be displayed. <--
    @field flags
        One or more flags (combined with an OR operator) defined in ImportImage flags enum. <--
    @field supportedFileTypes
        An array of file extension strings such as "jpg", "tif", etc., that are of interest to the calling application. Set to NULL to display all files. <--
    @field filterProc
        Specify a filter proc to that will be called for each file before it is displayed in the user interface. <--
    @field importedImages
        Returns an array of CFDataRefs for the imported images if the kICADownloadAndReturnPathArray flag is not specified. Otherwise returns an array of CFStringRefs holding the paths of the images that are downloaded. The caller should provide a pointer to a CFArrayRef object initialized to NULL. The caller is responsible for released the array returned by this function. -->
}
type
	ICAImportImagePBPtr = ^ICAImportImagePB;
	ICAImportImagePB = record
		header: ICAHeader;
		deviceObject: ICAObject;
		flags: UInt32;
		supportedFileTypes: CFArrayRef;
		filterProc: ICAImportFilterProc;
		importedImages: CFArrayRefPtr;
	end;

{!
    @function ICAImportImage
    @abstract
        This API displays a Common User Interface panel similar to the user interface of Image Capture Application. This allows the user to work a camera or a scanner.
    @discussion
        Use this API to add Image Capture support to an application.

<pre>
@textblock
        Example:
        
        void ImportImage()
        (
            OSErr             err;
            CFArrayRef        imagesArray = NULL;
            ICAImportImagePB  pb = ();

            pb.deviceObject       = 0;
            pb.flags              = 0;
            pb.supportedFileTypes = (CFArrayRef)[NSArray arrayWithObjects: @"tif", @"tiff", @"jpg", NULL];
            pb.importedImages     = &imagesArray;
            
            err = ICAImportImage(&pb, NULL);

            if ( noErr != err )
            (
                // handle error
            )
            else
            (
                // Process the importedImages array
                // pb.importedImages   // CFArrayRef *
            )
        )
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICAImportImagePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAImportImage</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAImportImage( var pb: ICAImportImagePB; completion: ICACompletion ): ICAError; external name '_ICAImportImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

//--------------------------------------------------------------------------------------------------------- ICAShowDeviceBrowser
{!
    @function ICAShowDeviceBrowser
    @abstract
        Use this API to display a device browser user interface from any Image Capture client application.
    @discussion
        The device browser user interface allows the user to do the following:
          - enable and disable sharing of locally connected cameras and scanners.
          - connect to or disconnect from cameras and scanners shared by other computers.
          - configure WiFi capable cameras for use over the WiFi network.
    @param options
        Set options to NULL to display the device browser with default settings. <--
        This parameter is intended for future use.
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAShowDeviceBrowser( options: CFDictionaryRef ): ICAError; external name '_ICAShowDeviceBrowser';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

//---------------------------------------------------------------------------------------------- ICARegisterForEventNotification
// Function prototype for an Image Capture notification callback proc
type
	ICANotification = procedure( notificationType: CFStringRef; notificationDictionary: CFDictionaryRef );

{ The Image Capture notification callabck function will be called with a notificationDictionary that may
   contain one or more key-value pairs as defined below:
   
    Key                                 Value Type        Comments
    
    kICANotificationICAObjectKey        CFNumberRef       An object associated with the notification.
    kICANotificationDeviceICAObjectKey  CFNumberRef       A device object associated with the notification.
    kICANotificationClassKey            CFStringRef       See below.
    kICANotificationTypeKey             CFStringRef       See below.
    kICANotificationRawEventKey         CFNumberRef       The unprocesssed event code sent by a device.
    kICANotificationDataKey             CFDataRef         Data associated with the event.
    kICANotificationDataSizeKey         CFNumberRef       Size of data associated with the event. This is used if the data is
                                                          not sent with the notification. [Needed for backward compatiblity with pre-Leopard device modules].
    kICANotificationDataCookieKey       CFNumberRef       A token identifying the data associated with this event.
                                                          This data can be retrieved by calling ICAObjectSendMessage with messageType set to kICAMessageGetEventData, dataType set to value of kICANotificationDataCookieKeyand dataSize set to value of kICANotificationDataSizeKey.
                                          
    The following keys are present if the value of kICANotificationDataKey represents image data. The values of these
    keys are CFNumbers representing the width, height, bytes per row, start row, and number of rows of the image:
    
    kICANotificationImageKey              CFDictionaryRef A dictionary that describes an Image associated
                                                          with the notification.
    kICANotificationImageDataKey          CFDataRef       Image data
    kICANotificationImageWidthKey         CFNumberRef     Image width in pixels
    kICANotificationImageHeightKey        CFNumberRef     Image height in pixels
    kICANotificationImageBytesPerRowKey   CFNumberRef     Bytes per row in image
    kICANotificationImageStartRowKey      CFNumberRef     Starting row number of the image.
    kICANotificationImageNumberOfRowsKey  CFNumberRef     Number of rows of image data sent in this notification.
}

// Possible values for kICANotificationTypeKey:

var kICANotificationTypeObjectAdded: CFStringRef; external name '_kICANotificationTypeObjectAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeObjectRemoved: CFStringRef; external name '_kICANotificationTypeObjectRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeObjectInfoChanged: CFStringRef; external name '_kICANotificationTypeObjectInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeStoreAdded: CFStringRef; external name '_kICANotificationTypeStoreAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreRemoved: CFStringRef; external name '_kICANotificationTypeStoreRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreFull: CFStringRef; external name '_kICANotificationTypeStoreFull'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreInfoChanged: CFStringRef; external name '_kICANotificationTypeStoreInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceAdded: CFStringRef; external name '_kICANotificationTypeDeviceAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceRemoved: CFStringRef; external name '_kICANotificationTypeDeviceRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceInfoChanged: CFStringRef; external name '_kICANotificationTypeDeviceInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDevicePropertyChanged: CFStringRef; external name '_kICANotificationTypeDevicePropertyChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceWasReset: CFStringRef; external name '_kICANotificationTypeDeviceWasReset'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceStatusInfo: CFStringRef; external name '_kICANotificationTypeDeviceStatusInfo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceStatusError: CFStringRef; external name '_kICANotificationTypeDeviceStatusError'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeCaptureComplete: CFStringRef; external name '_kICANotificationTypeCaptureComplete'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeRequestObjectTransfer: CFStringRef; external name '_kICANotificationTypeRequestObjectTransfer'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeTransactionCanceled: CFStringRef; external name '_kICANotificationTypeTransactionCanceled'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeUnreportedStatus: CFStringRef; external name '_kICANotificationTypeUnreportedStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeProprietary: CFStringRef; external name '_kICANotificationTypeProprietary'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceConnectionProgress: CFStringRef; external name '_kICANotificationTypeDeviceConnectionProgress'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDownloadProgressStatus: CFStringRef; external name '_kICANotificationTypeDownloadProgressStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScanProgressStatus: CFStringRef; external name '_kICANotificationTypeScanProgressStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerSessionClosed: CFStringRef; external name '_kICANotificationTypeScannerSessionClosed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerScanDone: CFStringRef; external name '_kICANotificationTypeScannerScanDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerPageDone: CFStringRef; external name '_kICANotificationTypeScannerPageDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerButtonPressed: CFStringRef; external name '_kICANotificationTypeScannerButtonPressed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeScannerOverviewOverlayAvailable: CFStringRef; external name '_kICANotificationTypeScannerOverviewOverlayAvailable'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

// Possible keys in the notification dictionary:

var kICAErrorKey: CFStringRef; external name '_kICAErrorKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICARefconKey: CFStringRef; external name '_kICARefconKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationICAObjectKey: CFStringRef; external name '_kICANotificationICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDeviceICAObjectKey: CFStringRef; external name '_kICANotificationDeviceICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDeviceListICAObjectKey: CFStringRef; external name '_kICANotificationDeviceListICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassKey: CFStringRef; external name '_kICANotificationClassKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeKey: CFStringRef; external name '_kICANotificationTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationRawEventKey: CFStringRef; external name '_kICANotificationRawEventKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataKey: CFStringRef; external name '_kICANotificationDataKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataSizeKey: CFStringRef; external name '_kICANotificationDataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataCookieKey: CFStringRef; external name '_kICANotificationDataCookieKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationImageKey: CFStringRef; external name '_kICANotificationImageKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageWidthKey: CFStringRef; external name '_kICANotificationImageWidthKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageHeightKey: CFStringRef; external name '_kICANotificationImageHeightKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageBytesPerRowKey: CFStringRef; external name '_kICANotificationImageBytesPerRowKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageStartRowKey: CFStringRef; external name '_kICANotificationImageStartRowKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageNumberOfRowsKey: CFStringRef; external name '_kICANotificationImageNumberOfRowsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageDataKey: CFStringRef; external name '_kICANotificationImageDataKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageDataSizeKey: CFStringRef; external name '_kICANotificationImageDataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataIsBigEndianKey: CFStringRef; external name '_kICANotificationDataIsBigEndianKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationScannerDocumentNameKey: CFStringRef; external name '_kICANotificationScannerDocumentNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationScannerButtonTypeKey: CFStringRef; external name '_kICANotificationScannerButtonTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationNumerOfImagesRemainingKey: CFStringRef; external name '_kICANotificationNumerOfImagesRemainingKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationPercentDownloadedKey: CFStringRef; external name '_kICANotificationPercentDownloadedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationSubTypeKey: CFStringRef; external name '_kICANotificationSubTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypeWarmUpStarted: CFStringRef; external name '_kICANotificationSubTypeWarmUpStarted'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypeWarmUpDone: CFStringRef; external name '_kICANotificationSubTypeWarmUpDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationVendorErrorCodeKey: CFStringRef; external name '_kICANotificationVendorErrorCodeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypePerformOverviewScan: CFStringRef; external name '_kICANotificationSubTypePerformOverviewScan'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
var kICANotificationSubTypeDocumentLoaded: CFStringRef; external name '_kICANotificationSubTypeDocumentLoaded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
var kICANotificationSubTypeDocumentNotLoaded: CFStringRef; external name '_kICANotificationSubTypeDocumentNotLoaded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

// Possible values in the notification dictionary:
// ...

// Possible values for kICANotificationClassKey
var kICANotificationClassPTPStandard: CFStringRef; external name '_kICANotificationClassPTPStandard'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassPTPVendor: CFStringRef; external name '_kICANotificationClassPTPVendor'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassProprietary: CFStringRef; external name '_kICANotificationClassProprietary'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

// Device Properties

var kICADevicePropUndefined: CFStringRef; external name '_kICADevicePropUndefined'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBatteryLevel: CFStringRef; external name '_kICADevicePropBatteryLevel'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFunctionalMode: CFStringRef; external name '_kICADevicePropFunctionalMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropImageSize: CFStringRef; external name '_kICADevicePropImageSize'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCompressionSetting: CFStringRef; external name '_kICADevicePropCompressionSetting'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropWhiteBalance: CFStringRef; external name '_kICADevicePropWhiteBalance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropRGBGain: CFStringRef; external name '_kICADevicePropRGBGain'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFNumber: CFStringRef; external name '_kICADevicePropFNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocalLength: CFStringRef; external name '_kICADevicePropFocalLength'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusDistance: CFStringRef; external name '_kICADevicePropFocusDistance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusMode: CFStringRef; external name '_kICADevicePropFocusMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureMeteringMode: CFStringRef; external name '_kICADevicePropExposureMeteringMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFlashMode: CFStringRef; external name '_kICADevicePropFlashMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureTime: CFStringRef; external name '_kICADevicePropExposureTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureProgramMode: CFStringRef; external name '_kICADevicePropExposureProgramMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureIndex: CFStringRef; external name '_kICADevicePropExposureIndex'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureBiasCompensation: CFStringRef; external name '_kICADevicePropExposureBiasCompensation'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropDateTime: CFStringRef; external name '_kICADevicePropDateTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCaptureDelay: CFStringRef; external name '_kICADevicePropCaptureDelay'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropStillCaptureMode: CFStringRef; external name '_kICADevicePropStillCaptureMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropContrast: CFStringRef; external name '_kICADevicePropContrast'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropSharpness: CFStringRef; external name '_kICADevicePropSharpness'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropDigitalZoom: CFStringRef; external name '_kICADevicePropDigitalZoom'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropEffectMode: CFStringRef; external name '_kICADevicePropEffectMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBurstNumber: CFStringRef; external name '_kICADevicePropBurstNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBurstInterval: CFStringRef; external name '_kICADevicePropBurstInterval'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropTimelapseNumber: CFStringRef; external name '_kICADevicePropTimelapseNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropTimelapseInterval: CFStringRef; external name '_kICADevicePropTimelapseInterval'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusMeteringMode: CFStringRef; external name '_kICADevicePropFocusMeteringMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropUploadURL: CFStringRef; external name '_kICADevicePropUploadURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropArtist: CFStringRef; external name '_kICADevicePropArtist'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCopyrightInfo: CFStringRef; external name '_kICADevicePropCopyrightInfo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{!
    @struct ICARegisterForEventNotificationPB
    @discussion
        Use this parameter structure to specify a set of events associated with an object
        about which notifications should be sent to the specified notification function.
    @field header
        See description for ICAHeader. <->
    @field objectOfInterest
        An object about which notifications are requested. <--
    @field eventsOfInterest
        An array of notification types of interest. <--
    @field notificationProc
        A callback function to receive the notifications. <--
    @field options
        Set options to NULL. This parameter is intended for future use. <--
}

type
	ICARegisterForEventNotificationPBPtr = ^ICARegisterForEventNotificationPB;
	ICARegisterForEventNotificationPB = record
		header: ICAHeader;
		objectOfInterest: ICAObject;
		eventsOfInterest: CFArrayRef;
		notificationProc: ICANotification;
		options: CFDictionaryRef;
	end;

{!
    @function ICARegisterForEventNotification
    @abstract
        Use this API to register with Image Capture framework to receive
        notification about events of interest.
    @param params
        A pointer to ICARegisterForEventNotificationPB struct <--
    @param completionProc
        A pointer to a completion routine that will be invoked at the completion of
        this function. Set this parameter to NULL to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICARegisterForEventNotification( var params: ICARegisterForEventNotificationPB; completionProc: ICACompletion ): ICAError; external name '_ICARegisterForEventNotification';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

//---------------------------------------------------------------------------------------------------------- ICASendNotification
// This parameter block is used with 'ICDSendNotification' and 'ICDSendNotificationAndWaitForReply' APIs defined
// in ICADevices.framework

>>>>>>> origin/cpstrnew
type
	ICASendNotificationPBPtr = ^ICASendNotificationPB;
	ICASendNotificationPB = record
		header: ICAHeader;
		notificationDictionary: CFMutableDictionaryRef;
		replyCode: UInt32;
	end;

function ICASendNotification( var pb: ICASendNotificationPB ): ICAError; external name '_ICASendNotification';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
function ICASendNotificationAndWaitForReply( var pb: ICASendNotificationPB ): ICAError; external name '_ICASendNotificationAndWaitForReply';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
function ICASendNotificationAndWaitForReply( var pb: ICASendNotificationPB ): ICAError; external name '_ICASendNotificationAndWaitForReply';
>>>>>>> graemeg/cpstrnew
=======
function ICASendNotificationAndWaitForReply( var pb: ICASendNotificationPB ): ICAError; external name '_ICASendNotificationAndWaitForReply';
>>>>>>> graemeg/cpstrnew
=======
function ICASendNotificationAndWaitForReply( var pb: ICASendNotificationPB ): ICAError; external name '_ICASendNotificationAndWaitForReply';
>>>>>>> graemeg/cpstrnew
=======
function ICASendNotificationAndWaitForReply( var pb: ICASendNotificationPB ): ICAError; external name '_ICASendNotificationAndWaitForReply';
>>>>>>> origin/cpstrnew

//#pragma mark -
//#pragma mark Object related APIs
//------------------------------------------------------------------------------------------------------------- ICAGetDeviceList
{!
    @struct ICAGetDeviceListPB
    @field header
        See description for ICAHeader. <-->
    @field object
        The device list object, if ICAGetDeviceList returns successfully. -->
}
type
	ICAGetDeviceListPBPtr = ^ICAGetDeviceListPB;
	ICAGetDeviceListPB = record
		header: ICAHeader;
		objct: ICAObject;
	end;

{!
    @function ICAGetDeviceList
    @abstract
        Fetches the object at the top of the object heirarchy.
    @discussion
        Image Capture framework presents cameras and scanners, their contents and their capabilities as a heirarchy of objects and their properties. The device list object is at the top of the heirarchy of objects. The <b><code>ICAGetDeviceList</b></code> function fetches this object in the <code><b>object</b></code> field of parameter <code><b>pb</b></code>. Children of the device list object can be accessed by passing the device list object to functions <code><b>ICAGetChildCount()</b></code> and <code>ICAGetNthChild()</code>.
        
<pre>
@textblock
        Example:
        
        ICAObject GetDeviceList()
        (
            ICAGetDeviceListPB getDeviceListPB  = ();
            ICAObject          deviceList       = 0;
            OSErr              err;
            
            err = ICAGetDeviceList( &getDeviceListPB, nil );
            
            if ( noErr == err )
            (
                deviceList = getDeviceListPB.object;
            )
            
            return deviceList;
        )
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICAGetDeviceListPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAGetDeviceList</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAGetDeviceList( var pb: ICAGetDeviceListPB; completion: ICACompletion ): ICAError; external name '_ICAGetDeviceList';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

//---------------------------------------------------------------------------------------------- ICACopyObjectPropertyDictionary
{!
    @struct ICACopyObjectPropertyDictionaryPB
    @field header
        See description for ICAHeader. <->
    @field object
        An object whose properties are being requested. <--
    @field theDict
        A dictionary to hold the properties. This must be released by the caller. -->
}
type
	ICACopyObjectPropertyDictionaryPBPtr = ^ICACopyObjectPropertyDictionaryPB;
	ICACopyObjectPropertyDictionaryPB = record
		header: ICAHeader;
		objct: ICAObject;
		theDict: CFDictionaryRefPtr;
	end;

{!
    @function ICACopyObjectPropertyDictionary
    @abstract
        Use this API to get a CFDictionaryRef containing all the properties for an object specified in the object field of the ICACopyObjectPropertyDictionaryPB struct.
    @discussion
        This API is the preferred way to get to any ICAObject related property data.
        
<pre>
@textblock
        Example:
        
        void  CopyObjectPropertyDictionary()
        (
            OSErr                             err;
            ICACopyObjectPropertyDictionaryPB pb = ();

            pb.object = <#ICAObject object#>;
            err = ICACopyObjectPropertyDictionary( &pb, NULL );

            if ( noErr != err)
            (
                // handle error
            )
            else
            (
                // Make sure to release the returned dictionary
                // pb.theDict   // CFDictionaryRef *
            )
        )
        
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICACopyObjectPropertyDictionaryPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICACopyObjectPropertyDictionary</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICACopyObjectPropertyDictionary( var pb: ICACopyObjectPropertyDictionaryPB; completion: ICACompletion ): ICAError; external name '_ICACopyObjectPropertyDictionary';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/cpstrnew

//------------------------------------------------------------------------------------------------------- ICACopyObjectThumbnail
{!
    @enum Thumbnail formats.
    @discussion
        Format alues that can be used in ICACopyObjectThumbnailPB parameter block.
    @constant kICAThumbnailFormatJPEG
        Use this constant to receive a thumbnail in JPEG format.
    @constant kICAThumbnailFormatTIFF
        Use this constant to receive a thumbnail in TIFF format.
    @constant kICAThumbnailFormatPNG
        Use this constant to receive a thumbnail in PNG format.
}
const
	kICAThumbnailFormatJPEG = FourCharCode('jpeg');
	kICAThumbnailFormatTIFF = FourCharCode('tiff');
	kICAThumbnailFormatPNG = FourCharCode('png ');

{!
    @struct ICACopyObjectThumbnailPB
    @field header
        See description for ICAHeader. <->
    @field object
        An object whose thumbail is being requested. <--
    @field thumbnailFormat
        One of the format values defined above. <--
    @field thumbnailData
        A pointer to a CFDataRef holding the thumbnail data. The returned CFDataRef must be released by the caller. -->
}
type
	ICACopyObjectThumbnailPBPtr = ^ICACopyObjectThumbnailPB;
	ICACopyObjectThumbnailPB = record
		header: ICAHeader;
		objct: ICAObject;
		thumbnailFormat: OSType;
		thumbnailData: CFDataRefPtr;
	end;

{!
    @function ICACopyObjectThumbnail
    @abstract
        Use this API to get a thumbnail associated with an object.
    @discussion
        This is the recommended way to get the thumbnail of an object. Getting the thumbnail using ICAGetPropertyData is deprecaed in 10.5.
        
<pre>
@textblock
        Example:
        
        void CopyObjectThumbnail()
        (
            OSErr                     err;
            ICACopyObjectThumbnailPB  pb = ();

            pb.object          = <#ICAObject object#>;
            pb.thumbnailFormat = <#OSType thumbnailFormat#>;
            
            err = ICACopyObjectThumbnail( &pb, NULL );

            if ( noErr != err )
            (
                // handle error
            )
            else
            (
                // Make sure to release the thumbnailData
                // pb.thumbnailData   // CFDataRef *
            )
        )
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICACopyObjectThumbnailPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICACopyObjectThumbnail</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICACopyObjectThumbnail( var pb: ICACopyObjectThumbnailPB; completion: ICACompletion ): ICAError; external name '_ICACopyObjectThumbnail';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew

//------------------------------------------------------------------------------------------------------------ ICACopyObjectData
{!
    @struct ICACopyObjectDataPB
    @field header
        See description for ICAHeader.  <->
    @field object
        A file object.  <--
    @field startByte
        Starting byte offset of the data in the file object.  <--
    @field requestedSize
        Requested data size in bytes. <--
    @field data
        A pointer to CFDataRef in which the data will be returned. -->
        It is the responsibility fo the caller to release this object. 
}
type
	ICACopyObjectDataPBPtr = ^ICACopyObjectDataPB;
	ICACopyObjectDataPB = record
		header: ICAHeader;
		objct: ICAObject;
		startByte: size_t;
		requestedSize: size_t;
		data: CFDataRefPtr;
	end;

{!
    @function ICACopyObjectData
    @abstract
        Use this API to get a copy of data associated with a file object.
    @discussion
        Use this API to get a copy of data associated with a file object. This API should be used in place of ICAGetPropertyData.
    @param params
        A pointer to ICACopyObjectDataPB struct <--
    @param completionProc
        A pointer to a completion routine that will be invoked at the completion of
        this function. Set this parameter to NULL to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICACopyObjectData( var params: ICACopyObjectDataPB; completionProc: ICACompletion ): ICAError; external name '_ICACopyObjectData';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew

//--------------------------------------------------------------------------------------------------------- ICAObjectSendMessage
{!
    @struct ICAMessage
    @field messageType
        A message type. e.g., kICAMessageCameraCaptureNewImage. <--
    @field startByte
        Offset in dataPtr from where data access for read/write should occur. <--
    @field dataPtr
        A pointer to a data buffer. <--
    @field dataSize
        Size of data. <--
    @field dataType
        Type of data. <--
}
type
	ICAMessagePtr = ^ICAMessage;
	ICAMessage = record
		messageType: OSType;
		startByte: UInt32;
		dataPtr: UnivPtr;
		dataSize: UInt32;
		dataType: OSType;
	end;

{!
    @enum ICAMessage types
    @discussion
        Definition of ICAMessage types.
    @constant kICAMessageConnect
        Connect to device.
    @constant kICAMessageDisconnect
        Disconnect device.
    @constant kICAMessageReset
        Reset device.
    @constant kICAMessageCheckDevice
        Check device.
    @constant kICAMessageCameraReadClock
        Read clock from device.
    @constant kICAMessageGetLastButtonPressed
        Get last button pressed on the device (scanner).
    @constant kICAMessageGetEventData
        Get data associated with an event.
    @constant kICAMessageDeviceYield
        Yield device. Image Capture framework yields a device so that the sender of the message can directly communicate with the device.
}
const
	kICAMessageConnect = FourCharCode('open');
	kICAMessageDisconnect = FourCharCode('clos');
	kICAMessageReset = FourCharCode('rese');
	kICAMessageCheckDevice = FourCharCode('chkd');
	kICAMessageCameraReadClock = FourCharCode('rclk');
	kICAMessageGetLastButtonPressed = FourCharCode('btn?');
	kICAMessageGetEventData = FourCharCode('mged');
	kICAMessageDeviceYield = FourCharCode('yiel');
	kICAMessageCameraPassThrough = FourCharCode('pass');
	kICAMessageScannerOverviewSelectionChanged = FourCharCode('area');

{!
    @struct ICAObjectSendMessagePB
    @field header
        See description for ICAHeader. <-->
    @field object
        A target object for the message sent by ICAObjectSendMessage. <--
    @field message
        One of the messages define above. <--
    @field result
        A message specific result is returned here. -->
}
type
	ICAObjectSendMessagePBPtr = ^ICAObjectSendMessagePB;
	ICAObjectSendMessagePB = record
		header: ICAHeader;
		objct: ICAObject;
		message: ICAMessage;
		result: UInt32;
	end;

{!
    @function ICAObjectSendMessage
    @abstract
        Use this API to send a message to a device object.
    @discussion
        Use this API to send a message to a device object. All devices do not respond to all the messages defined above.
    @param pb
        A pointer to an <code><b>ICAObjectSendMessagePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAObjectSendMessage</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAObjectSendMessage( var pb: ICAObjectSendMessagePB; completion: ICACompletion ): ICAError; external name '_ICAObjectSendMessage';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

//-------------------------------------------------------------------------------------------------------------- ICADownloadFile
{!
    @enum Flag to use with ICADownloadFile
    @discussion
        Use any combination of these values when downloading a file.
    @constant kDeleteAfterDownload
        Delete file after a successful download.
    @constant kCreateCustomIcon
        Create a custom icon for Finder.
    @constant kAddMetaDataToFinderComment
        Add basic metadata to finder comment field.
    @constant kAdjustCreationDate
        Set creation date of the downloaded file same as the creation date for the file as reported by the device.
    @constant kSetFileTypeAndCreator
        Set 4-char file type and creator code.
    @constant kRotateImage
        Rotate the image.
    @constant kDontEmbedColorSyncProfile
        Embed ColorSync profile to the image if one was not already embedded.
}
const
	kDeleteAfterDownload = $00000001;
	kCreateCustomIcon = $00000002;
	kAddMetaDataToFinderComment = $00000004;
	kAdjustCreationDate = $00000008;
	kSetFileTypeAndCreator = $00000010;
    //kEmbedColorSyncProfile              = 0x00000020,
	kRotateImage = $00000040;
	kDontEmbedColorSyncProfile = $00000080;  

{!
    @struct ICADownloadFilePB
    @field header
        See description for ICAHeader. <->
    @field object
        The file object. <--
    @field dirFSRef
        FSRef of destination directiory. <--
    @field flags 
        Any combination of flag values defined above. <--
    @field fileType
        Four-char code indicating the type of file. <--
    @field fileCreator
        Four-char code indicating with the creator of the file. <--
    @field rotationAngle
        Rotation angle in steps of 90 degress. <--
    @field fileFSRef
        A pointer to FSRef struct to hold the FSRef of downloaded file. Set this to NULL if the FSRef of downloaded file is not of interest. --> 
}
type
	ICADownloadFilePBPtr = ^ICADownloadFilePB;
	ICADownloadFilePB = record
		header: ICAHeader;
		objct: ICAObject;
		dirFSRef: FSRefPtr;
		flags: UInt32;
		fileType: OSType;
		fileCreator: OSType;
		rotationAngle: Fixed;
		fileFSRef: FSRefPtr;
	end;

{!
    @function ICADownloadFile
    @abstract
        Use this API to download a file to disk. 
    @discussion
      This API is a convenient way to download a file to disk. To receive the image data in memory use ICACopyObjectData. Using ICAGetPropertyData is not recommend for this purpose since ICAGetPropertyData is Deprecated in 10.5.

<pre>
@textblock
        Example:
        
        void DownloadFile()
        (
            OSErr             err;
            ICADownloadFilePB pb = ();

            pb.flags         = <#UInt32 flags#>;
            pb.rotationAngle = <#Fixed rotationAngle#>;
            pb.object        = <#ICAObject object#>;
            pb.fileCreator   = <#OSType fileCreator#>;
            pb.dirFSRef      = <#FSRef * dirFSRef#>;
            pb.fileType      = <#OSType fileType#>;
            
            err = ICADownloadFile( &pb, NULL );

            if ( noErr != err )
            (
                // handle error
            )
            else
            (
                // pb.fileFSRef   // FSRef *
            )
        )
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICADownloadFilePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICADownloadFile</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICADownloadFile( var pb: ICADownloadFilePB; completion: ICACompletion ): ICAError; external name '_ICADownloadFile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/cpstrnew

//---------------------------------------------------------------------------------------------------------------- ICAUploadFile
{!
    @enum Upload file option flags.
    @discussion
        Flag values that can be used in ICAUploadFilePB parameter block.
    @constant kICAUploadFileAsIs
        Use this constant to upload a file as is.
    @constant kICAUploadFileScaleToFit
        Use this constant to upload a file after scaling to fit a specified bounding rect.
}
const
	kICAUploadFileAsIs = $00000000;
	kICAUploadFileScaleToFit = $00000001;  

{!
    @struct ICAUploadFilePB
    @field header
        See description for ICAHeader. <->
    @field parentObject <->
        An ICAObject corresponding to a folder on the device. The device will store the uploaded file inside this folder if possible.
    @field fileFSRef <--
        An FSRef for the file to be uploaded to the device.
    @field flags <--
        One of the flags defined above.
}
type
	ICAUploadFilePBPtr = ^ICAUploadFilePB;
	ICAUploadFilePB = record
		header: ICAHeader;
		parentObject: ICAObject;
		fileFSRef: FSRefPtr;
		flags: UInt32;
	end;

{!
    @function ICAUploadFile
    @abstract
        Use this API to upload a file to a device that supports this capability.
    @discussion
        The device choses an appropriate destination location for the uploaded image and sends a kICANotificationTypeObjectAdded notification.
        
<pre>
@textblock
        Example:
        
        void  UploadFile()
        (
            OSErr           err;
            ICAUploadFilePB pb = ();

            pb.fileFSRef    = <#FSRef * fileFSRef#>;
            pb.flags        = <#UInt32 flags#>;
            pb.parentObject = <#ICAObject parentObject#>;
            
            err = ICAUploadFile( &pb, NULL );

            if ( noErr != err )
            (
                // handle error
            )
            else
            (
                // no return value(s)
            )
        )
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICAUploadFilePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAUploadFile</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAUploadFile( var pb: ICAUploadFilePB; completion: ICACompletion ): ICAError; external name '_ICAUploadFile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew

//#pragma mark -
//#pragma mark Device related APIs
//---------------------------------------------------------------------------------------------------------- ICALoadDeviceModule
{ struct ICALoadDeviceModulePB
    Legal Key-Value pairs for populating paramDictionary:
            
    Key                             Value                               Comment
    
    kICADeviceModulePathKey         CFStringRef                         Path to the device module bundle that needs to be launched.
    kICATransportTypeKey            CFStringRef                         Should be one of the six predifined transport types.
    kICABluetoothAddressKey         CFStringRef                         Bluetooth device address string formatted as "00-11-22-33-44-55". 
    kICAUSBLocationIDKey            CFNumberRef (kCFNumberLongType)     32 bit USB location ID.
    kICAFireWireGUIDKey             CFNumberRef (kCFNumberLongLongType) 64 bit FireWire GUID.                                                               
    kICAIOServicePathKey            CFStringRef                         IO service path to the device obtained from the IO registry.
    kICAIPAddressKey                CFStringRef                         IP address of the device. This can be a host address ("camera.apple.com"),
                                                                        ipv4 address ('192.168.123.10") or ipv6 address ("3ff3:0000:0000:0000:0123:4567:89ab:cdef")
    kICAIPPortKey                   CFNumberRef (kCFNumberLongType)     IP port number of the device.
    kICAIPNameKey                   CFStringRef                         Human readable device name.
    kICAIPGUIDKey                   CFStringRef                         16 byte GUID string of the device formatted as "01234567-89ab-cdef-0123-456789abcdef".
    kICATWAINDSPathKey              CFStringRef                         Path to TWAIN DS bundle. }


{!
    @struct ICALoadDeviceModulePB
    @field header
        See description for ICAHeader. <->
    @field paramDictionary <--
        A parameter dictionary with sufficient key-value pairs to load a device module. This dictionary itself or the information provided in this dictionary will be sent to the device module.
}
type
	ICALoadDeviceModulePBPtr = ^ICALoadDeviceModulePB;
	ICALoadDeviceModulePB = record
		header: ICAHeader;
		paramDictionary: CFDictionaryRef;
	end;

{!
    @function ICALoadDeviceModule
    @abstract
        Use this API to load a device module.
    @discussion
        Typically, connecting a FireWire or an USB device will automatically load an appropriate device module. This API is needed only for loading a device module manually for devices that do not use a hot-plug interface, such as Bluetooth, SCSI, or TCP/IP.
    @param pb
        A pointer to an <code><b>ICALoadDeviceModulePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICALoadDeviceModule</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICALoadDeviceModule( var pb: ICALoadDeviceModulePB; completion: ICACompletion ): ICAError; external name '_ICALoadDeviceModule';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew

//--------------------------------------------------------------------------------------------------------- ICAUnloadDeviceModule
{!
    @struct ICAUnloadDeviceModulePB
    @field header
        See description for ICAHeader. <->
    @field deviceObject <--
        A device ICAObject.
}
type
	ICAUnloadDeviceModulePBPtr = ^ICAUnloadDeviceModulePB;
	ICAUnloadDeviceModulePB = record
		header: ICAHeader;
		deviceObject: ICAObject;
	end;

{!
    @function ICAUnloadDeviceModule
    @abstract
        Uset this API to unload a device module.
    @discussion
        The device module providing this object will be unloaded, if this is the last device object provided by the device module.
    @param pb
        A pointer to an <code><b>ICAUnloadDeviceModulePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAUnloadDeviceModule</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAUnloadDeviceModule( var pb: ICAUnloadDeviceModulePB; completion: ICACompletion ): ICAError; external name '_ICAUnloadDeviceModule';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew

//--------------------------------------------------------------------------------------------------------------- ICAOpenSession
{!
    @struct ICAOpenSessionPB
    @field header
        See description for ICAHeader. <->
    @field deviceObject
        A camera object. <--
    @field sessionID
        A session ID of the opened session. -->
}
type
	ICAOpenSessionPBPtr = ^ICAOpenSessionPB;
	ICAOpenSessionPB = record
		header: ICAHeader;
		deviceObject: ICAObject;
		sessionID: ICASessionID;
	end;

{!
    @function ICAOpenSession
    @abstract
        Use this API to open a session on a camera device. For a scanner device use the ICAScannerOpenSession API.
    @discussion
        This API gets a session ID for a open session on a camera device. Since access to cameras is generally not be session-based, this API generall will not fail. If the camera has open session, the device module controlling the camera will continue to control it during fast-user-switching.
    @param pb
        A pointer to an <code><b>ICAOpenSessionPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAOpenSession</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAOpenSession( var pb: ICAOpenSessionPB; completion: ICACompletion ): ICAError; external name '_ICAOpenSession';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew

//-------------------------------------------------------------------------------------------------------------- ICACloseSession
{!
    @struct ICACloseSessionPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the session to be closed. <--
}
type
	ICACloseSessionPBPtr = ^ICACloseSessionPB;
	ICACloseSessionPB = record
		header: ICAHeader;
		sessionID: ICASessionID;
	end;

{!
    @function ICACloseSession
    @abstract
        Use this API to close a session on a camera device. For a scanner device use the ICAScannerCloseSession API.
    @discussion
        This API closes an open session on a camera device. If the camera does not have any open sessions, the device module controlling the camera is free to give it up during fast-user-switching.
    @param pb
        A pointer to an <code><b>ICACloseSessionPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICACloseSession</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICACloseSession( var pb: ICACloseSessionPB; completion: ICACompletion ): ICAError; external name '_ICACloseSession';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew

//-------------------------------------------------------------------------------------------------------- ICAScannerOpenSession
{!
    @struct ICAScannerOpenSessionPB
    @field header
        See description for ICAHeader. <->
    @field object
        A scanner object. <--
    @field sessionID
        A session ID of the opened session. -->
}
type
	ICAScannerOpenSessionPBPtr = ^ICAScannerOpenSessionPB;
	ICAScannerOpenSessionPB = record
		header: ICAHeader;
		objct: ICAObject;
		sessionID: ICAScannerSessionID;
	end;

{!
    @function ICAScannerOpenSession
    @abstract
        Use this API to open a session on a scanner device. For a camera device use the ICAOpenSession API.
    @discussion
        For a given scanner, this API returns a unique session ID that allows you to work with the device. This API will fail, if a session is already open.
    @param pb
        A pointer to an <code><b>ICAScannerOpenSessionPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerOpenSession</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerOpenSession( var pb: ICAScannerOpenSessionPB; completion: ICACompletion ): ICAError; external name '_ICAScannerOpenSession';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew

//------------------------------------------------------------------------------------------------------- ICAScannerCloseSession
{!
    @struct ICAScannerCloseSessionPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the session to be closed. <--
}
type
	ICAScannerCloseSessionPBPtr = ^ICAScannerCloseSessionPB;
	ICAScannerCloseSessionPB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
	end;

{!
    @function ICAScannerCloseSession
    @abstract
        Use this API to close a session on a scanner device. For a camera device use the ICACloseSession API.
    @discussion
        This API closes an open session, allowing other clients to work with the scanner.
    @param pb
        A pointer to an <code><b>ICAScannerCloseSessionPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerCloseSession</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerCloseSession( var pb: ICAScannerCloseSessionPB; completion: ICACompletion ): ICAError; external name '_ICAScannerCloseSession';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew

//--------------------------------------------------------------------------------------------------------- ICAScannerInitialize
{!
    @struct ICAScannerInitializePB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the scanner to be initialized. <--
}
type
	ICAScannerInitializePBPtr = ^ICAScannerInitializePB;
	ICAScannerInitializePB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
	end;

{!
    @function ICAScannerInitialize
    @abstract
        Use this API to initialize a scanner device.
    @discussion
        After opening a session on a scanner device, use this API to set an initial state for the scanner.
    @param pb
        A pointer to an <code><b>ICAScannerInitializePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerInitialize</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerInitialize( var pb: ICAScannerInitializePB; completion: ICACompletion ): ICAError; external name '_ICAScannerInitialize';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew

//------------------------------------------------------------------------------------------------------ ICAScannerGetParameters
{!
    @struct ICAScannerGetParametersPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the scanner whose parameters are being fetched. <--
    @field theDict
        A dictionary containing the parameters. -->
}
type
	ICAScannerGetParametersPBPtr = ^ICAScannerGetParametersPB;
	ICAScannerGetParametersPB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
		theDict: CFMutableDictionaryRef;
	end;

{!
    @function ICAScannerGetParameters
    @abstract
        Use this API to get information about the scanner such as resolution, scanning area, etc.
    @discussion
        Use this API to get information about the scanner such as resolution, scanning area, etc.
    @param pb
        A pointer to an <code><b>ICAScannerGetParametersPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerGetParameters</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerGetParameters( var pb: ICAScannerGetParametersPB; completion: ICACompletion ): ICAError; external name '_ICAScannerGetParameters';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew

//------------------------------------------------------------------------------------------------------ ICAScannerSetParameters
{!
    @struct ICAScannerSetParametersPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the scanner whose parameters are being set. <--
    @field theDict
        A dictionary containing the parameters. <--
}
type
	ICAScannerSetParametersPBPtr = ^ICAScannerSetParametersPB;
	ICAScannerSetParametersPB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
		theDict: CFMutableDictionaryRef;
	end;

{!
    @function ICAScannerSetParameters
    @abstract
        Use this API to specify scan parameters that will be used when a scan is initiated via an ICAScannerStart.
    @discussion
        Use this API to specify scan parameters that will be used when a scan is initiated via an ICAScannerStart.
    @param pb
        A pointer to an <code><b>ICAScannerSetParametersPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerSetParameters</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerSetParameters( var pb: ICAScannerSetParametersPB; completion: ICACompletion ): ICAError; external name '_ICAScannerSetParameters';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew

//------------------------------------------------------------------------------------------------------------- ICAScannerStatus
{!
    @struct ICAScannerStatusPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the scanner whose status is being fetched. <--
    @field status
        A status value. -->
}
type
	ICAScannerStatusPBPtr = ^ICAScannerStatusPB;
	ICAScannerStatusPB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
		status: UInt32;
	end;

{!
    @function ICAScannerStatus
    @abstract
        Use this API to get information about the current status of the scanner.
    @discussion
        Use this API to get information about the current status of the scanner.
    @param pb
        A pointer to an <code><b>ICAScannerStatusPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerStatus</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerStatus( var pb: ICAScannerStatusPB; completion: ICACompletion ): ICAError; external name '_ICAScannerStatus';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew

//-------------------------------------------------------------------------------------------------------------- ICAScannerStart
{!
    @struct ICAScannerStartPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the scanner that should start scanning. <--
}
type
	ICAScannerStartPBPtr = ^ICAScannerStartPB;
	ICAScannerStartPB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
	end;

{!
    @function ICAScannerStart
    @abstract
        Use this API start a scan based on the parameters that were specified in a previous ICAScannerSetParameters call.
    @discussion
        Use this API start a scan based on the parameters that were specified in a previous ICAScannerSetParameters call.
    @param pb
        A pointer to an <code><b>ICAScannerStartPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerStart</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerStart( var pb: ICAScannerStartPB; completion: ICACompletion ): ICAError; external name '_ICAScannerStart';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew

//------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------------------

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,AEDataModel,Files,CFDictionary;
=======
uses MacTypes,AEDataModel,Files,CFBase,CFArray,CFData,CFDictionary;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

//------------------------------------------------------------------------------------------------------------------------------
{!
    @header
        ICAApplication.h
    @discussion
      ICAApplication.h defines structures and functions that are used by clients of Image Capture framework. 
}

//------------------------------------------------------------------------------------------------------------------------------

type
	ICAError = OSErr;

//------------------------------------------------------------------------------------------------------ Parameter block version
{!
    @enum Parameter block version
    @discussion
        Parameter block version.
    @constant kICAPBVersion
        Version 1 parameter block.
}
const
	kICAPBVersion = $00010000;

//------------------------------------------------------------------------------------------------------------------ Error codes
// Image Capture error code range = -9900 to -9949
{!
    @enum Error codes
    @discussion
        Definition of error codes returned by Image Capture framework
    @constant kICACommunicationErr
        An error occurred in communication between different components of Image Capture framework.
    @constant kICADeviceNotFoundErr
        The specified device is not found.
    @constant kICADeviceNotOpenErr
        The specified device is not open.
    @constant kICAFileCorruptedErr
        Encountered a corrupt file.
    @constant kICAIOPendingErr
        There is a pending I/O.
    @constant kICAInvalidObjectErr
        The specified object is invalid.
    @constant kICAInvalidPropertyErr
        The specified property is invalid.
    @constant kICAIndexOutOfRangeErr
        The specified index is out of range.
    @constant kICAPropertyTypeNotFoundErr
        A property with the specified property type is not found.
    @constant kICACannotYieldDevice
        The device module cannot yield the specified device to the requestor.
    @constant kICADataTypeNotFoundErr
        Data with the specified data type is not found.
    @constant kICADeviceMemoryAllocationErr
        The device module encountered a memory allocation error.
    @constant kICADeviceInternalErr
        The device module encountered an unspecifed error.
    @constant kICADeviceInvalidParamErr
        At least one of the parameters passed to the device module is invalid.
    @constant kICADeviceAlreadyOpenErr  
        The specified device is already open.
    @constant kICADeviceLocationIDNotFoundErr
        The specified USB Location ID is not found.
    @constant kICADeviceGUIDNotFoundErr
        The specified FireWire GUID is not found.
    @constant kICADeviceIOServicePathNotFoundErr
        The specified IOService path is not found.
    @constant kICAFrameworkInternalErr
        Image Capture Framework encountered an error.
    @constant kICAExtensionInternalErr
        Image Capture Extension encountered an error.
    @constant kICAInvalidSessionErr
        The specified session is not valid.
}
const
	kICACommunicationErr = -9900;
	kICADeviceNotFoundErr = -9901;
	kICADeviceNotOpenErr = -9902;
	kICAFileCorruptedErr = -9903;
	kICAIOPendingErr = -9904;
	kICAInvalidObjectErr = -9905;
	kICAInvalidPropertyErr = -9906;
	kICAIndexOutOfRangeErr = -9907;
	kICAPropertyTypeNotFoundErr = -9908;
	kICACannotYieldDevice = -9909;
	kICADataTypeNotFoundErr = -9910;
	kICADeviceMemoryAllocationErr = -9911;
	kICADeviceInternalErr = -9912;
	kICADeviceInvalidParamErr = -9913;
	kICADeviceAlreadyOpenErr = -9914;
	kICADeviceLocationIDNotFoundErr = -9915;
	kICADeviceGUIDNotFoundErr = -9916;
	kICADeviceIOServicePathNotFoundErr = -9917;
	kICADeviceUnsupportedErr = -9918;
	kICAFrameworkInternalErr = -9919;
	kICAExtensionInternalErr = -9920;
	kICAInvalidSessionErr = -9921;

//------------------------------------------------------------------------------------------------- ICAObject types and subtypes
{!
    @enum ICAObject types and subtypes
    @discussion
        Definition of ICAObject types and subtypes
    @constant kICADevice
        Object is a device supported by Image Capture framework.
    @constant kICADeviceCamera
        Object is a camera.
    @constant kICADeviceScanner
        Object is a scanner.
    @constant kICADeviceMFP
        Object is a multi-function peripheral.
    @constant kICADevicePhone
        Object is a camera phone.
    @constant kICADevicePDA
        Object is a personal digital assistant.
    @constant kICADeviceOther
        Object is a device supported by Image Capture framework, but of unknown subtype.
    @constant kICAList
        Object is a device list.
    @constant kICADirectory
        Object is a directory.
    @constant kICAFile
        Object is a file.
    @constant kICAFileImage
        Object is an image file.
    @constant kICAFileMovie
        Object is a movie file.
    @constant kICAFileAudio
        Object is an audio file.
    @constant kICAFileFirmware
        Object is a firmware file.
    @constant kICAFileOther
        Object is a generic file.
}
const
	kICADevice = FourCharCode('icdv');
	kICADeviceCamera = FourCharCode('cmra');
	kICADeviceScanner = FourCharCode('scan');
	kICADeviceMFP = FourCharCode('mfp ');
	kICADevicePhone = FourCharCode('phon');
	kICADevicePDA = FourCharCode('pda ');
	kICADeviceOther = FourCharCode('doth');
	kICAList = FourCharCode('objl');
	kICADirectory = FourCharCode('dire');
	kICAFile = FourCharCode('file');
	kICAFileImage = FourCharCode('imag');
	kICAFileMovie = FourCharCode('moov');
	kICAFileAudio = FourCharCode('audo');
	kICAFileFirmware = FourCharCode('firm');
	kICAFileOther = FourCharCode('othe');

//------------------------------------------------------------------------------------------------------------ ICAProperty types
{!
    @enum ICAProperty types
    @discussion
        Definition of ICAProperties
    @constant kICAProperty
        Generic property type; for images, refer to 'Digital Still Camera Image File Format Standard' Exif Version 2.1 section 2.6.4. and 2.6.5.
    @constant kICAPropertyImageWidth
        Image width. 
    @constant kICAPropertyImageHeight
        Image height.
    @constant kICAPropertyImageBitDepth
        Image bit-depth.
    @constant kICAPropertyImageDPI
        Image DPI.
    @constant kICAPropertyImageExposureTime
        Image exposure time.
    @constant kICAPropertyImageFNumber
        Image f-Number.
    @constant kICAPropertyImageDateOriginal
        Original date & time of an object; value associated with this property is a null-terminated string conforming to format "YYYY:MM:DD hh:mm:ss".
    @constant kICAPropertyImageDateDigitized
        Digitized date & time of an object; value associated with this property is a null-terminated string conforming to format "YYYY:MM:DD hh:mm:ss".
    @constant kICAPropertyImageShutterSpeed
        Shutter speed used to capture an image.
    @constant kICAPropertyImageAperture
        Aperture used to capture an image.
    @constant kICAPropertyImageFlash
        Indicates whether flash was used to capture an image.
    @constant kICAPropertyColorSpace
        Color space used to represent an image.
    @constant kICAPropertyImageFilename
        Filename of an image.
    @constant kICAPropertyImageSize
        Size of an image in bytes. 
    @constant kICAPropertyImageData
        Data of an image.
    @constant kICAPropertyImageThumbnail
        Thumbnail of an image.
    @constant kICAPropertyColorSyncProfile
        ColorSync profile associated with an image.
}  
const
	kICAProperty = FourCharCode('prop');
	kICAPropertyImageWidth = FourCharCode('0100');
	kICAPropertyImageHeight = FourCharCode('0101');
	kICAPropertyImageBitDepth = FourCharCode('0102');
	kICAPropertyImageDPI = FourCharCode('011A');
	kICAPropertyImageExposureTime = FourCharCode('829A');
	kICAPropertyImageFNumber = FourCharCode('829D');
	kICAPropertyImageDateOriginal = FourCharCode('9003');
	kICAPropertyImageDateDigitized = FourCharCode('9004');
	kICAPropertyImageShutterSpeed = FourCharCode('9201');
	kICAPropertyImageAperture = FourCharCode('9202');
	kICAPropertyImageFlash = FourCharCode('9209');
	kICAPropertyColorSpace = FourCharCode('A001');
	kICAPropertyImageFilename = FourCharCode('ifil');
	kICAPropertyImageSize = FourCharCode('isiz');
	kICAPropertyImageData = FourCharCode('idat');
	kICAPropertyImageThumbnail = FourCharCode('thum');
	kICAPropertyColorSyncProfile = FourCharCode('prof');

//------------------------------------------------------------------------------------------------------------------- Data types
{!
    @enum Data types
    @discussion
        Definition of data types; these are mapped to AppleEvent types.
    @constant kICATypeUInt8
        UInt8.
    @constant kICATypeUInt16
        UInt16.
    @constant kICATypeUInt32
        UInt32.
    @constant kICATypeUInt64
        UInt64.
    @constant kICATypeSInt16
        SInt16.
    @constant kICATypeSInt32
        SInt32.
    @constant kICATypeSInt64
        SInt64.
    @constant kICATypeFloat
        float.
    @constant kICATypeFixed
        IEEE 32-bit floating point.
    @constant kICATypeBoolean
        Boolean.
    @constant kICATypeString
        Char string.
    @constant kICATypeData
        void *.
    @constant kICATypeThumbnail
        ICAThumbnail.
}
const
	kICATypeUInt8 = FourCharCode('ui08');
	kICATypeUInt16 = FourCharCode('ui16');
	kICATypeUInt32 = FourCharCode('ui32');
	kICATypeUInt64 = FourCharCode('ui64');
	kICATypeSInt16 = FourCharCode('si16');
	kICATypeSInt32 = FourCharCode('si32');
	kICATypeSInt64 = FourCharCode('si64');
	kICATypeFloat = FourCharCode('floa');
	kICATypeFixed = FourCharCode('sing');
	kICATypeBoolean = FourCharCode('bool');
	kICATypeString = FourCharCode('TEXT');
	kICATypeData = FourCharCode('data');
	kICATypeThumbnail = FourCharCode('thum');

//----------------------------------------------------------------------------------------------------- PropertyInfo flag values
{!
    @enum PropertyInfo flag values
    @discussion
        Values for PropertyInfo flag.
    @constant kICAFlagReadWriteAccess
        Access for read and write.
    @constant kICAFlagReadAccess
        Access for read only.
}
const
	kICAFlagReadWriteAccess = 1 shl 0;
	kICAFlagReadAccess = 1 shl 1;

//----------------------------------------------------------------------------------------------------------------- Button types
{!
    @enum Button types
    @discussion
        Buttons types associated with buttons on a scanner.
    @constant kICAButtonScan
        Scan button.
    @constant kICAButtonCopy
        Copy button.
    @constant kICAButtonEMail
        Email button.
    @constant kICAButtonWeb
        Web button.
}
const
	kICAButtonScan = FourCharCode('scan');
	kICAButtonCopy = FourCharCode('copy');
	kICAButtonEMail = FourCharCode('mail');
	kICAButtonWeb = FourCharCode('web ');

//------------------------------------------------------------------------ Flags associated with Image Capture PassThru commands
{!
    @enum Flags associated with Image Capture PassThru commands.
    @discussion
        Flag values that can be used in ICAUploadFilePB parameter block.
    @constant kICACameraPassThruSend
        Use this constant when sending data to a device using a pass-through command.
    @constant kICACameraPassThruReceive
        Use this constant when receiving data from a device using a pass-through command.
    @constant kICACameraPassThruNotUsed
        Use this constant when using a pass-through command that doesn't involve sending or receiving data.
}
const
	kICACameraPassThruSend = 0;
	kICACameraPassThruReceive = 1;
	kICACameraPassThruNotUsed = 2;

//---------------------------------------------------------------------------------------------------------- ICAPTPPassThroughPB
{!
    @struct ICAPTPPassThroughPB
    @field commandCode
        PTP command code (including vendor specific) <--
    @field resultCode
        PTP response code -->
    @field numOfInputParams
        Number of valid parameters to be sent to device <--
    @field numOfOutputParams
        Number of valid parameters expected from device <--
    @field params
        PTP parameters (command specific / optional) <->
    @field dataUsageMode
        One of (kICACameraPassThruSend, kICACameraPassThruReceive, kICACameraPassThruNotUsed) <--
    @field flags
        Not used currently
    @field dataSize
        Size of data block <->
    @field data
        Data block <->
}
type
	ICAPTPPassThroughPBPtr = ^ICAPTPPassThroughPB;
	ICAPTPPassThroughPB = record
		commandCode: UInt32;
		resultCode: UInt32;
		numOfInputParams: UInt32;
		numOfOutputParams: UInt32;
		params: array [0..4-1] of UInt32;
		dataUsageMode: UInt32;
		flags: UInt32;
		dataSize: UInt32;
		data: array [0..1-1] of UInt8;
	end;

//----------------------------------------------------------------------------------------------------------- ICAPTPEventDataset
{!
    @struct ICAPTPEventDataset
    @field dataLength
        Data length in bytes
    @field containerType
        PTP container type
    @field eventCode
        PTP event code
    @field transactionID
        PTP transaction ID
    @field params
        PTP params. The number of params should be (dataLength - 12)/4
}
type
	ICAPTPEventDatasetPtr = ^ICAPTPEventDataset;
	ICAPTPEventDataset = record
		dataLength: UInt32;
		containerType: UInt16;		// should be 0x0004 for event
		eventCode: UInt16;
		transactionID: UInt32;
		params: array [0..3-1] of UInt32;			// up to 3 params. # of params = (dataLength - 12)/4
	end;

//------------------------------------------------------------------------------------- Keys used in object property dictionary
{ Keys used in object property dictionary }
//#pragma mark -
//#pragma mark TODO: document the data types of values for these keys

// Keys returned by ICACopyObjectDictionary() for deviceList object returned by ICAGetDeviceList()

var kICADevicesArrayKey: CFStringRef; external name '_kICADevicesArrayKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICAObjectKey: CFStringRef; external name '_kICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAObjectNameKey: CFStringRef; external name '_kICAObjectNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAUSBVendorIDKey: CFStringRef; external name '_kICAUSBVendorIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAUSBProductIDKey: CFStringRef; external name '_kICAUSBProductIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceTypeKey: CFStringRef; external name '_kICADeviceTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAExecutableArchitectureKey: CFStringRef; external name '_kICAExecutableArchitectureKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICARemoteDeviceKey: CFStringRef; external name '_kICARemoteDeviceKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceSharedKey: CFStringRef; external name '_kICADeviceSharedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceWebSharedKey: CFStringRef; external name '_kICADeviceWebSharedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceUsedKey: CFStringRef; external name '_kICADeviceUsedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICABonjourServiceTypeKey: CFStringRef; external name '_kICABonjourServiceTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICABonjourServiceNameKey: CFStringRef; external name '_kICABonjourServiceNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICABonjourTXTRecordKey: CFStringRef; external name '_kICABonjourTXTRecordKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceCapabilitiesKey: CFStringRef; external name '_kICADeviceCapabilitiesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICALockStatusKey: CFStringRef; external name '_kICALockStatusKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICADataPropertyKey: CFStringRef; external name '_kICADataPropertyKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADataTypeKey: CFStringRef; external name '_kICADataTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADataSizeKey: CFStringRef; external name '_kICADataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAThumbnailPropertyKey: CFStringRef; external name '_kICAThumbnailPropertyKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAThumbnailSizeKey: CFStringRef; external name '_kICAThumbnailSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICARawKey: CFStringRef; external name '_kICARawKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICAMediaHeightKey: CFStringRef; external name '_kICAMediaHeightKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// value is a number
var kICAMediaWidthKey: CFStringRef; external name '_kICAMediaWidthKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// value is a number
var kICACreationDateStringKey: CFStringRef; external name '_kICACreationDateStringKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAModificationDateStringKey: CFStringRef; external name '_kICAModificationDateStringKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kMetaDataDictionaryKey: CFStringRef; external name '_kMetaDataDictionaryKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICAMediaDurationInSecondsKey: CFStringRef; external name '_kICAMediaDurationInSecondsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICADeviceTypeCamera: CFStringRef; external name '_kICADeviceTypeCamera'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceTypeScanner: CFStringRef; external name '_kICADeviceTypeScanner'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
    In addition to the above, the following keys may also be present in the object property dictionay:
    
        kICAUSBLocationIDKey
        kICAFireWireGUIDKey
}

{ Transport types }
var kICAUSBTransportType: CFStringRef; external name '_kICAUSBTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAFireWireTransportType: CFStringRef; external name '_kICAFireWireTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICABluetoothTransportType: CFStringRef; external name '_kICABluetoothTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICATCPIPTransportType: CFStringRef; external name '_kICATCPIPTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICASCSITransportType: CFStringRef; external name '_kICASCSITransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICATWAINTransportType: CFStringRef; external name '_kICATWAINTransportType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Keys used for paramDictionary in ICALoadDeviceModulePB}
var kICADeviceBrowserDeviceRefKey: CFStringRef; external name '_kICADeviceBrowserDeviceRefKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADeviceModulePathKey: CFStringRef; external name '_kICADeviceModulePathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICADeviceIconPathKey: CFStringRef; external name '_kICADeviceIconPathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICATransportTypeKey: CFStringRef; external name '_kICATransportTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICABluetoothAddressKey: CFStringRef; external name '_kICABluetoothAddressKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAUSBLocationIDKey: CFStringRef; external name '_kICAUSBLocationIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAFireWireGUIDKey: CFStringRef; external name '_kICAFireWireGUIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIOServicePathKey: CFStringRef; external name '_kICAIOServicePathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPAddressKey: CFStringRef; external name '_kICAIPAddressKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPPortKey: CFStringRef; external name '_kICAIPPortKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPNameKey: CFStringRef; external name '_kICAIPNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICAIPGUIDKey: CFStringRef; external name '_kICAIPGUIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kICATWAINDSPathKey: CFStringRef; external name '_kICATWAINDSPathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
    @const
        kICAUserAssignedDeviceNameKey
    @abstract 
        This key may be present in the property dictionary of a device if the device has a user-assigned name.
    @discussion
        Value is of type CFStringRef.
}
var kICAUserAssignedDeviceNameKey: CFStringRef; external name '_kICAUserAssignedDeviceNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

//-------------------------------------------------------------------------------------------------------------------- ICAHeader
{!
    @struct ICAHeader
    @discussion
        This is the first field in all parameter blocks used by APIs defined in ICAApplication.h.
        Type of parameter passed to a callback function used by APIs defined in ICAApplication.h.
        The parameter for the completion proc should to be casted to an appropriate type such as ICAGetChildCountPB* for it to be useful.
    @field err
        Error returned by an API. -->
    @field refcon
        An arbitrary refcon value passed to the callback. <--
}
type
	ICAHeaderPtr = ^ICAHeader;
	ICAHeader = record
		err: ICAError;
		refcon: UNSIGNEDLONG;
	end;

//--------------------------------------------------------------------------------------------------------------- Callback procs

type
	ICACompletion = procedure( var pb: ICAHeader );

type
	ICAImportFilterProc = function( imageInfo: CFDictionaryRef; refcon: UNSIGNEDLONG ): Boolean;

type
	ICANotificationProc = procedure( notificationType: CFStringRef; notificationDictionary: CFDictionaryRef );

//------------------------------------------------------------------------------------------------------------------- Object IDs

type
	ICAObject = UInt32;
	ICAProperty = UInt32;
	ICAConnectionID = UInt32;
	ICASessionID = UInt32;
	ICAScannerSessionID = ICASessionID;
	ICAEventDataCookie = UInt32;

//#pragma mark -
//#pragma mark General APIs
//--------------------------------------------------------------------------------------------------------------- ICAImportImage
{!
    @struct ICAObjectInfo
    @field objectType
        An object type, e.g., kICAFile.
    @field objectSubtype
        An object subtype, e.g., kICAFileImage.
}
type
	ICAObjectInfoPtr = ^ICAObjectInfo;
	ICAObjectInfo = record
		objectType: OSType;
		objectSubtype: OSType;
	end;

{!
    @enum ImportImage flags.
    @discussion
        Flag values that can be used in ICAImportImagePB parameter block.
    @constant kICAAllowMultipleImages
        Use this constant to allow users to select multiple images in the Import Image dialog.
    @constant kICADownloadAndReturnPathArray
        Use this constant to download the images to a temporary location and return an array of paths to the downloaded images.
}
const
	kICAAllowMultipleImages = $00000001;
	kICADownloadAndReturnPathArray = $00000002;

{!
    @struct ICAImportImagePB
    @field header
        See description for ICAHeader.  <->
    @field deviceObject
        Object ID of a camera or scanner device. Set this to NULL to ge the default behavior: (a) if no device is connected, a panel saying that there’s no device connected is displayed, (b) if a single device is connected, an appropriate user interface to access that device will be displayed, (c) if several devices are connected, a device selector panel will be displayed. <--
    @field flags
        One or more flags (combined with an OR operator) defined in ImportImage flags enum. <--
    @field supportedFileTypes
        An array of file extension strings such as "jpg", "tif", etc., that are of interest to the calling application. Set to NULL to display all files. <--
    @field filterProc
        Specify a filter proc to that will be called for each file before it is displayed in the user interface. <--
    @field importedImages
        Returns an array of CFDataRefs for the imported images if the kICADownloadAndReturnPathArray flag is not specified. Otherwise returns an array of CFStringRefs holding the paths of the images that are downloaded. The caller should provide a pointer to a CFArrayRef object initialized to NULL. The caller is responsible for released the array returned by this function. -->
}
type
	ICAImportImagePBPtr = ^ICAImportImagePB;
	ICAImportImagePB = record
		header: ICAHeader;
		deviceObject: ICAObject;
		flags: UInt32;
		supportedFileTypes: CFArrayRef;
		filterProc: ICAImportFilterProc;
		importedImages: CFArrayRefPtr;
	end;

{!
    @function ICAImportImage
    @abstract
        This API displays a Common User Interface panel similar to the user interface of Image Capture Application. This allows the user to work a camera or a scanner.
    @discussion
        Use this API to add Image Capture support to an application.

<pre>
@textblock
        Example:
        
        void ImportImage()
        (
            OSErr             err;
            CFArrayRef        imagesArray = NULL;
            ICAImportImagePB  pb = ();

            pb.deviceObject       = 0;
            pb.flags              = 0;
            pb.supportedFileTypes = (CFArrayRef)[NSArray arrayWithObjects: @"tif", @"tiff", @"jpg", NULL];
            pb.importedImages     = &imagesArray;
            
            err = ICAImportImage(&pb, NULL);

            if ( noErr != err )
            (
                // handle error
            )
            else
            (
                // Process the importedImages array
                // pb.importedImages   // CFArrayRef *
            )
        )
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICAImportImagePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAImportImage</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAImportImage( var pb: ICAImportImagePB; completion: ICACompletion ): ICAError; external name '_ICAImportImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

//--------------------------------------------------------------------------------------------------------- ICAShowDeviceBrowser
{!
    @function ICAShowDeviceBrowser
    @abstract
        Use this API to display a device browser user interface from any Image Capture client application.
    @discussion
        The device browser user interface allows the user to do the following:
          - enable and disable sharing of locally connected cameras and scanners.
          - connect to or disconnect from cameras and scanners shared by other computers.
          - configure WiFi capable cameras for use over the WiFi network.
    @param options
        Set options to NULL to display the device browser with default settings. <--
        This parameter is intended for future use.
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAShowDeviceBrowser( options: CFDictionaryRef ): ICAError; external name '_ICAShowDeviceBrowser';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

//---------------------------------------------------------------------------------------------- ICARegisterForEventNotification
// Function prototype for an Image Capture notification callback proc
type
	ICANotification = procedure( notificationType: CFStringRef; notificationDictionary: CFDictionaryRef );

{ The Image Capture notification callabck function will be called with a notificationDictionary that may
   contain one or more key-value pairs as defined below:
   
    Key                                 Value Type        Comments
    
    kICANotificationICAObjectKey        CFNumberRef       An object associated with the notification.
    kICANotificationDeviceICAObjectKey  CFNumberRef       A device object associated with the notification.
    kICANotificationClassKey            CFStringRef       See below.
    kICANotificationTypeKey             CFStringRef       See below.
    kICANotificationRawEventKey         CFNumberRef       The unprocesssed event code sent by a device.
    kICANotificationDataKey             CFDataRef         Data associated with the event.
    kICANotificationDataSizeKey         CFNumberRef       Size of data associated with the event. This is used if the data is
                                                          not sent with the notification. [Needed for backward compatiblity with pre-Leopard device modules].
    kICANotificationDataCookieKey       CFNumberRef       A token identifying the data associated with this event.
                                                          This data can be retrieved by calling ICAObjectSendMessage with messageType set to kICAMessageGetEventData, dataType set to value of kICANotificationDataCookieKeyand dataSize set to value of kICANotificationDataSizeKey.
                                          
    The following keys are present if the value of kICANotificationDataKey represents image data. The values of these
    keys are CFNumbers representing the width, height, bytes per row, start row, and number of rows of the image:
    
    kICANotificationImageKey              CFDictionaryRef A dictionary that describes an Image associated
                                                          with the notification.
    kICANotificationImageDataKey          CFDataRef       Image data
    kICANotificationImageWidthKey         CFNumberRef     Image width in pixels
    kICANotificationImageHeightKey        CFNumberRef     Image height in pixels
    kICANotificationImageBytesPerRowKey   CFNumberRef     Bytes per row in image
    kICANotificationImageStartRowKey      CFNumberRef     Starting row number of the image.
    kICANotificationImageNumberOfRowsKey  CFNumberRef     Number of rows of image data sent in this notification.
}

// Possible values for kICANotificationTypeKey:

var kICANotificationTypeObjectAdded: CFStringRef; external name '_kICANotificationTypeObjectAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeObjectRemoved: CFStringRef; external name '_kICANotificationTypeObjectRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeObjectInfoChanged: CFStringRef; external name '_kICANotificationTypeObjectInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeStoreAdded: CFStringRef; external name '_kICANotificationTypeStoreAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreRemoved: CFStringRef; external name '_kICANotificationTypeStoreRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreFull: CFStringRef; external name '_kICANotificationTypeStoreFull'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeStoreInfoChanged: CFStringRef; external name '_kICANotificationTypeStoreInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceAdded: CFStringRef; external name '_kICANotificationTypeDeviceAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceRemoved: CFStringRef; external name '_kICANotificationTypeDeviceRemoved'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceInfoChanged: CFStringRef; external name '_kICANotificationTypeDeviceInfoChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDevicePropertyChanged: CFStringRef; external name '_kICANotificationTypeDevicePropertyChanged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceWasReset: CFStringRef; external name '_kICANotificationTypeDeviceWasReset'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceStatusInfo: CFStringRef; external name '_kICANotificationTypeDeviceStatusInfo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDeviceStatusError: CFStringRef; external name '_kICANotificationTypeDeviceStatusError'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeCaptureComplete: CFStringRef; external name '_kICANotificationTypeCaptureComplete'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeRequestObjectTransfer: CFStringRef; external name '_kICANotificationTypeRequestObjectTransfer'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeTransactionCanceled: CFStringRef; external name '_kICANotificationTypeTransactionCanceled'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeUnreportedStatus: CFStringRef; external name '_kICANotificationTypeUnreportedStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeProprietary: CFStringRef; external name '_kICANotificationTypeProprietary'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeDeviceConnectionProgress: CFStringRef; external name '_kICANotificationTypeDeviceConnectionProgress'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeDownloadProgressStatus: CFStringRef; external name '_kICANotificationTypeDownloadProgressStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScanProgressStatus: CFStringRef; external name '_kICANotificationTypeScanProgressStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerSessionClosed: CFStringRef; external name '_kICANotificationTypeScannerSessionClosed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerScanDone: CFStringRef; external name '_kICANotificationTypeScannerScanDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerPageDone: CFStringRef; external name '_kICANotificationTypeScannerPageDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeScannerButtonPressed: CFStringRef; external name '_kICANotificationTypeScannerButtonPressed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationTypeScannerOverviewOverlayAvailable: CFStringRef; external name '_kICANotificationTypeScannerOverviewOverlayAvailable'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

// Possible keys in the notification dictionary:

var kICAErrorKey: CFStringRef; external name '_kICAErrorKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICARefconKey: CFStringRef; external name '_kICARefconKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationICAObjectKey: CFStringRef; external name '_kICANotificationICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDeviceICAObjectKey: CFStringRef; external name '_kICANotificationDeviceICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDeviceListICAObjectKey: CFStringRef; external name '_kICANotificationDeviceListICAObjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassKey: CFStringRef; external name '_kICANotificationClassKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationTypeKey: CFStringRef; external name '_kICANotificationTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationRawEventKey: CFStringRef; external name '_kICANotificationRawEventKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataKey: CFStringRef; external name '_kICANotificationDataKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataSizeKey: CFStringRef; external name '_kICANotificationDataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataCookieKey: CFStringRef; external name '_kICANotificationDataCookieKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationImageKey: CFStringRef; external name '_kICANotificationImageKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageWidthKey: CFStringRef; external name '_kICANotificationImageWidthKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageHeightKey: CFStringRef; external name '_kICANotificationImageHeightKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageBytesPerRowKey: CFStringRef; external name '_kICANotificationImageBytesPerRowKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageStartRowKey: CFStringRef; external name '_kICANotificationImageStartRowKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageNumberOfRowsKey: CFStringRef; external name '_kICANotificationImageNumberOfRowsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageDataKey: CFStringRef; external name '_kICANotificationImageDataKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationImageDataSizeKey: CFStringRef; external name '_kICANotificationImageDataSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationDataIsBigEndianKey: CFStringRef; external name '_kICANotificationDataIsBigEndianKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationScannerDocumentNameKey: CFStringRef; external name '_kICANotificationScannerDocumentNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationScannerButtonTypeKey: CFStringRef; external name '_kICANotificationScannerButtonTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationNumerOfImagesRemainingKey: CFStringRef; external name '_kICANotificationNumerOfImagesRemainingKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationPercentDownloadedKey: CFStringRef; external name '_kICANotificationPercentDownloadedKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

var kICANotificationSubTypeKey: CFStringRef; external name '_kICANotificationSubTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypeWarmUpStarted: CFStringRef; external name '_kICANotificationSubTypeWarmUpStarted'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypeWarmUpDone: CFStringRef; external name '_kICANotificationSubTypeWarmUpDone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationVendorErrorCodeKey: CFStringRef; external name '_kICANotificationVendorErrorCodeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationSubTypePerformOverviewScan: CFStringRef; external name '_kICANotificationSubTypePerformOverviewScan'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
var kICANotificationSubTypeDocumentLoaded: CFStringRef; external name '_kICANotificationSubTypeDocumentLoaded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
var kICANotificationSubTypeDocumentNotLoaded: CFStringRef; external name '_kICANotificationSubTypeDocumentNotLoaded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

// Possible values in the notification dictionary:
// ...

// Possible values for kICANotificationClassKey
var kICANotificationClassPTPStandard: CFStringRef; external name '_kICANotificationClassPTPStandard'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassPTPVendor: CFStringRef; external name '_kICANotificationClassPTPVendor'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICANotificationClassProprietary: CFStringRef; external name '_kICANotificationClassProprietary'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

// Device Properties

var kICADevicePropUndefined: CFStringRef; external name '_kICADevicePropUndefined'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBatteryLevel: CFStringRef; external name '_kICADevicePropBatteryLevel'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFunctionalMode: CFStringRef; external name '_kICADevicePropFunctionalMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropImageSize: CFStringRef; external name '_kICADevicePropImageSize'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCompressionSetting: CFStringRef; external name '_kICADevicePropCompressionSetting'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropWhiteBalance: CFStringRef; external name '_kICADevicePropWhiteBalance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropRGBGain: CFStringRef; external name '_kICADevicePropRGBGain'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFNumber: CFStringRef; external name '_kICADevicePropFNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocalLength: CFStringRef; external name '_kICADevicePropFocalLength'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusDistance: CFStringRef; external name '_kICADevicePropFocusDistance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusMode: CFStringRef; external name '_kICADevicePropFocusMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureMeteringMode: CFStringRef; external name '_kICADevicePropExposureMeteringMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFlashMode: CFStringRef; external name '_kICADevicePropFlashMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureTime: CFStringRef; external name '_kICADevicePropExposureTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureProgramMode: CFStringRef; external name '_kICADevicePropExposureProgramMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureIndex: CFStringRef; external name '_kICADevicePropExposureIndex'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropExposureBiasCompensation: CFStringRef; external name '_kICADevicePropExposureBiasCompensation'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropDateTime: CFStringRef; external name '_kICADevicePropDateTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCaptureDelay: CFStringRef; external name '_kICADevicePropCaptureDelay'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropStillCaptureMode: CFStringRef; external name '_kICADevicePropStillCaptureMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropContrast: CFStringRef; external name '_kICADevicePropContrast'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropSharpness: CFStringRef; external name '_kICADevicePropSharpness'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropDigitalZoom: CFStringRef; external name '_kICADevicePropDigitalZoom'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropEffectMode: CFStringRef; external name '_kICADevicePropEffectMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBurstNumber: CFStringRef; external name '_kICADevicePropBurstNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropBurstInterval: CFStringRef; external name '_kICADevicePropBurstInterval'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropTimelapseNumber: CFStringRef; external name '_kICADevicePropTimelapseNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropTimelapseInterval: CFStringRef; external name '_kICADevicePropTimelapseInterval'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropFocusMeteringMode: CFStringRef; external name '_kICADevicePropFocusMeteringMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropUploadURL: CFStringRef; external name '_kICADevicePropUploadURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropArtist: CFStringRef; external name '_kICADevicePropArtist'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kICADevicePropCopyrightInfo: CFStringRef; external name '_kICADevicePropCopyrightInfo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{!
    @struct ICARegisterForEventNotificationPB
    @discussion
        Use this parameter structure to specify a set of events associated with an object
        about which notifications should be sent to the specified notification function.
    @field header
        See description for ICAHeader. <->
    @field objectOfInterest
        An object about which notifications are requested. <--
    @field eventsOfInterest
        An array of notification types of interest. <--
    @field notificationProc
        A callback function to receive the notifications. <--
    @field options
        Set options to NULL. This parameter is intended for future use. <--
}

type
	ICARegisterForEventNotificationPBPtr = ^ICARegisterForEventNotificationPB;
	ICARegisterForEventNotificationPB = record
		header: ICAHeader;
		objectOfInterest: ICAObject;
		eventsOfInterest: CFArrayRef;
		notificationProc: ICANotification;
		options: CFDictionaryRef;
	end;

{!
    @function ICARegisterForEventNotification
    @abstract
        Use this API to register with Image Capture framework to receive
        notification about events of interest.
    @param params
        A pointer to ICARegisterForEventNotificationPB struct <--
    @param completionProc
        A pointer to a completion routine that will be invoked at the completion of
        this function. Set this parameter to NULL to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICARegisterForEventNotification( var params: ICARegisterForEventNotificationPB; completionProc: ICACompletion ): ICAError; external name '_ICARegisterForEventNotification';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

//---------------------------------------------------------------------------------------------------------- ICASendNotification
// This parameter block is used with 'ICDSendNotification' and 'ICDSendNotificationAndWaitForReply' APIs defined
// in ICADevices.framework

type
	ICASendNotificationPBPtr = ^ICASendNotificationPB;
	ICASendNotificationPB = record
		header: ICAHeader;
		notificationDictionary: CFMutableDictionaryRef;
		replyCode: UInt32;
	end;

function ICASendNotification( var pb: ICASendNotificationPB ): ICAError; external name '_ICASendNotification';
function ICASendNotificationAndWaitForReply( var pb: ICASendNotificationPB ): ICAError; external name '_ICASendNotificationAndWaitForReply';

//#pragma mark -
//#pragma mark Object related APIs
//------------------------------------------------------------------------------------------------------------- ICAGetDeviceList
{!
    @struct ICAGetDeviceListPB
    @field header
        See description for ICAHeader. <-->
    @field object
        The device list object, if ICAGetDeviceList returns successfully. -->
}
type
	ICAGetDeviceListPBPtr = ^ICAGetDeviceListPB;
	ICAGetDeviceListPB = record
		header: ICAHeader;
		objct: ICAObject;
	end;

{!
    @function ICAGetDeviceList
    @abstract
        Fetches the object at the top of the object heirarchy.
    @discussion
        Image Capture framework presents cameras and scanners, their contents and their capabilities as a heirarchy of objects and their properties. The device list object is at the top of the heirarchy of objects. The <b><code>ICAGetDeviceList</b></code> function fetches this object in the <code><b>object</b></code> field of parameter <code><b>pb</b></code>. Children of the device list object can be accessed by passing the device list object to functions <code><b>ICAGetChildCount()</b></code> and <code>ICAGetNthChild()</code>.
        
<pre>
@textblock
        Example:
        
        ICAObject GetDeviceList()
        (
            ICAGetDeviceListPB getDeviceListPB  = ();
            ICAObject          deviceList       = 0;
            OSErr              err;
            
            err = ICAGetDeviceList( &getDeviceListPB, nil );
            
            if ( noErr == err )
            (
                deviceList = getDeviceListPB.object;
            )
            
            return deviceList;
        )
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICAGetDeviceListPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAGetDeviceList</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAGetDeviceList( var pb: ICAGetDeviceListPB; completion: ICACompletion ): ICAError; external name '_ICAGetDeviceList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

//---------------------------------------------------------------------------------------------- ICACopyObjectPropertyDictionary
{!
    @struct ICACopyObjectPropertyDictionaryPB
    @field header
        See description for ICAHeader. <->
    @field object
        An object whose properties are being requested. <--
    @field theDict
        A dictionary to hold the properties. This must be released by the caller. -->
}
type
	ICACopyObjectPropertyDictionaryPBPtr = ^ICACopyObjectPropertyDictionaryPB;
	ICACopyObjectPropertyDictionaryPB = record
		header: ICAHeader;
		objct: ICAObject;
		theDict: CFDictionaryRefPtr;
	end;

{!
    @function ICACopyObjectPropertyDictionary
    @abstract
        Use this API to get a CFDictionaryRef containing all the properties for an object specified in the object field of the ICACopyObjectPropertyDictionaryPB struct.
    @discussion
        This API is the preferred way to get to any ICAObject related property data.
        
<pre>
@textblock
        Example:
        
        void  CopyObjectPropertyDictionary()
        (
            OSErr                             err;
            ICACopyObjectPropertyDictionaryPB pb = ();

            pb.object = <#ICAObject object#>;
            err = ICACopyObjectPropertyDictionary( &pb, NULL );

            if ( noErr != err)
            (
                // handle error
            )
            else
            (
                // Make sure to release the returned dictionary
                // pb.theDict   // CFDictionaryRef *
            )
        )
        
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICACopyObjectPropertyDictionaryPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICACopyObjectPropertyDictionary</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICACopyObjectPropertyDictionary( var pb: ICACopyObjectPropertyDictionaryPB; completion: ICACompletion ): ICAError; external name '_ICACopyObjectPropertyDictionary';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

//------------------------------------------------------------------------------------------------------- ICACopyObjectThumbnail
{!
    @enum Thumbnail formats.
    @discussion
        Format alues that can be used in ICACopyObjectThumbnailPB parameter block.
    @constant kICAThumbnailFormatJPEG
        Use this constant to receive a thumbnail in JPEG format.
    @constant kICAThumbnailFormatTIFF
        Use this constant to receive a thumbnail in TIFF format.
    @constant kICAThumbnailFormatPNG
        Use this constant to receive a thumbnail in PNG format.
}
const
	kICAThumbnailFormatJPEG = FourCharCode('jpeg');
	kICAThumbnailFormatTIFF = FourCharCode('tiff');
	kICAThumbnailFormatPNG = FourCharCode('png ');

{!
    @struct ICACopyObjectThumbnailPB
    @field header
        See description for ICAHeader. <->
    @field object
        An object whose thumbail is being requested. <--
    @field thumbnailFormat
        One of the format values defined above. <--
    @field thumbnailData
        A pointer to a CFDataRef holding the thumbnail data. The returned CFDataRef must be released by the caller. -->
}
type
	ICACopyObjectThumbnailPBPtr = ^ICACopyObjectThumbnailPB;
	ICACopyObjectThumbnailPB = record
		header: ICAHeader;
		objct: ICAObject;
		thumbnailFormat: OSType;
		thumbnailData: CFDataRefPtr;
	end;

{!
    @function ICACopyObjectThumbnail
    @abstract
        Use this API to get a thumbnail associated with an object.
    @discussion
        This is the recommended way to get the thumbnail of an object. Getting the thumbnail using ICAGetPropertyData is deprecaed in 10.5.
        
<pre>
@textblock
        Example:
        
        void CopyObjectThumbnail()
        (
            OSErr                     err;
            ICACopyObjectThumbnailPB  pb = ();

            pb.object          = <#ICAObject object#>;
            pb.thumbnailFormat = <#OSType thumbnailFormat#>;
            
            err = ICACopyObjectThumbnail( &pb, NULL );

            if ( noErr != err )
            (
                // handle error
            )
            else
            (
                // Make sure to release the thumbnailData
                // pb.thumbnailData   // CFDataRef *
            )
        )
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICACopyObjectThumbnailPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICACopyObjectThumbnail</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICACopyObjectThumbnail( var pb: ICACopyObjectThumbnailPB; completion: ICACompletion ): ICAError; external name '_ICACopyObjectThumbnail';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

//------------------------------------------------------------------------------------------------------------ ICACopyObjectData
{!
    @struct ICACopyObjectDataPB
    @field header
        See description for ICAHeader.  <->
    @field object
        A file object.  <--
    @field startByte
        Starting byte offset of the data in the file object.  <--
    @field requestedSize
        Requested data size in bytes. <--
    @field data
        A pointer to CFDataRef in which the data will be returned. -->
        It is the responsibility fo the caller to release this object. 
}
type
	ICACopyObjectDataPBPtr = ^ICACopyObjectDataPB;
	ICACopyObjectDataPB = record
		header: ICAHeader;
		objct: ICAObject;
		startByte: size_t;
		requestedSize: size_t;
		data: CFDataRefPtr;
	end;

{!
    @function ICACopyObjectData
    @abstract
        Use this API to get a copy of data associated with a file object.
    @discussion
        Use this API to get a copy of data associated with a file object. This API should be used in place of ICAGetPropertyData.
    @param params
        A pointer to ICACopyObjectDataPB struct <--
    @param completionProc
        A pointer to a completion routine that will be invoked at the completion of
        this function. Set this parameter to NULL to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICACopyObjectData( var params: ICACopyObjectDataPB; completionProc: ICACompletion ): ICAError; external name '_ICACopyObjectData';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

//--------------------------------------------------------------------------------------------------------- ICAObjectSendMessage
{!
    @struct ICAMessage
    @field messageType
        A message type. e.g., kICAMessageCameraCaptureNewImage. <--
    @field startByte
        Offset in dataPtr from where data access for read/write should occur. <--
    @field dataPtr
        A pointer to a data buffer. <--
    @field dataSize
        Size of data. <--
    @field dataType
        Type of data. <--
}
type
	ICAMessagePtr = ^ICAMessage;
	ICAMessage = record
		messageType: OSType;
		startByte: UInt32;
		dataPtr: UnivPtr;
		dataSize: UInt32;
		dataType: OSType;
	end;

{!
    @enum ICAMessage types
    @discussion
        Definition of ICAMessage types.
    @constant kICAMessageConnect
        Connect to device.
    @constant kICAMessageDisconnect
        Disconnect device.
    @constant kICAMessageReset
        Reset device.
    @constant kICAMessageCheckDevice
        Check device.
    @constant kICAMessageCameraReadClock
        Read clock from device.
    @constant kICAMessageGetLastButtonPressed
        Get last button pressed on the device (scanner).
    @constant kICAMessageGetEventData
        Get data associated with an event.
    @constant kICAMessageDeviceYield
        Yield device. Image Capture framework yields a device so that the sender of the message can directly communicate with the device.
}
const
	kICAMessageConnect = FourCharCode('open');
	kICAMessageDisconnect = FourCharCode('clos');
	kICAMessageReset = FourCharCode('rese');
	kICAMessageCheckDevice = FourCharCode('chkd');
	kICAMessageCameraReadClock = FourCharCode('rclk');
	kICAMessageGetLastButtonPressed = FourCharCode('btn?');
	kICAMessageGetEventData = FourCharCode('mged');
	kICAMessageDeviceYield = FourCharCode('yiel');
	kICAMessageCameraPassThrough = FourCharCode('pass');
	kICAMessageScannerOverviewSelectionChanged = FourCharCode('area');

{!
    @struct ICAObjectSendMessagePB
    @field header
        See description for ICAHeader. <-->
    @field object
        A target object for the message sent by ICAObjectSendMessage. <--
    @field message
        One of the messages define above. <--
    @field result
        A message specific result is returned here. -->
}
type
	ICAObjectSendMessagePBPtr = ^ICAObjectSendMessagePB;
	ICAObjectSendMessagePB = record
		header: ICAHeader;
		objct: ICAObject;
		message: ICAMessage;
		result: UInt32;
	end;

{!
    @function ICAObjectSendMessage
    @abstract
        Use this API to send a message to a device object.
    @discussion
        Use this API to send a message to a device object. All devices do not respond to all the messages defined above.
    @param pb
        A pointer to an <code><b>ICAObjectSendMessagePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAObjectSendMessage</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAObjectSendMessage( var pb: ICAObjectSendMessagePB; completion: ICACompletion ): ICAError; external name '_ICAObjectSendMessage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

//-------------------------------------------------------------------------------------------------------------- ICADownloadFile
{!
    @enum Flag to use with ICADownloadFile
    @discussion
        Use any combination of these values when downloading a file.
    @constant kDeleteAfterDownload
        Delete file after a successful download.
    @constant kCreateCustomIcon
        Create a custom icon for Finder.
    @constant kAddMetaDataToFinderComment
        Add basic metadata to finder comment field.
    @constant kAdjustCreationDate
        Set creation date of the downloaded file same as the creation date for the file as reported by the device.
    @constant kSetFileTypeAndCreator
        Set 4-char file type and creator code.
    @constant kRotateImage
        Rotate the image.
    @constant kDontEmbedColorSyncProfile
        Embed ColorSync profile to the image if one was not already embedded.
}
const
	kDeleteAfterDownload = $00000001;
	kCreateCustomIcon = $00000002;
	kAddMetaDataToFinderComment = $00000004;
	kAdjustCreationDate = $00000008;
	kSetFileTypeAndCreator = $00000010;
    //kEmbedColorSyncProfile              = 0x00000020,
	kRotateImage = $00000040;
	kDontEmbedColorSyncProfile = $00000080;  

{!
    @struct ICADownloadFilePB
    @field header
        See description for ICAHeader. <->
    @field object
        The file object. <--
    @field dirFSRef
        FSRef of destination directiory. <--
    @field flags 
        Any combination of flag values defined above. <--
    @field fileType
        Four-char code indicating the type of file. <--
    @field fileCreator
        Four-char code indicating with the creator of the file. <--
    @field rotationAngle
        Rotation angle in steps of 90 degress. <--
    @field fileFSRef
        A pointer to FSRef struct to hold the FSRef of downloaded file. Set this to NULL if the FSRef of downloaded file is not of interest. --> 
}
type
	ICADownloadFilePBPtr = ^ICADownloadFilePB;
	ICADownloadFilePB = record
		header: ICAHeader;
		objct: ICAObject;
		dirFSRef: FSRefPtr;
		flags: UInt32;
		fileType: OSType;
		fileCreator: OSType;
		rotationAngle: Fixed;
		fileFSRef: FSRefPtr;
	end;

{!
    @function ICADownloadFile
    @abstract
        Use this API to download a file to disk. 
    @discussion
      This API is a convenient way to download a file to disk. To receive the image data in memory use ICACopyObjectData. Using ICAGetPropertyData is not recommend for this purpose since ICAGetPropertyData is Deprecated in 10.5.

<pre>
@textblock
        Example:
        
        void DownloadFile()
        (
            OSErr             err;
            ICADownloadFilePB pb = ();

            pb.flags         = <#UInt32 flags#>;
            pb.rotationAngle = <#Fixed rotationAngle#>;
            pb.object        = <#ICAObject object#>;
            pb.fileCreator   = <#OSType fileCreator#>;
            pb.dirFSRef      = <#FSRef * dirFSRef#>;
            pb.fileType      = <#OSType fileType#>;
            
            err = ICADownloadFile( &pb, NULL );

            if ( noErr != err )
            (
                // handle error
            )
            else
            (
                // pb.fileFSRef   // FSRef *
            )
        )
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICADownloadFilePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICADownloadFile</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICADownloadFile( var pb: ICADownloadFilePB; completion: ICACompletion ): ICAError; external name '_ICADownloadFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

//---------------------------------------------------------------------------------------------------------------- ICAUploadFile
{!
    @enum Upload file option flags.
    @discussion
        Flag values that can be used in ICAUploadFilePB parameter block.
    @constant kICAUploadFileAsIs
        Use this constant to upload a file as is.
    @constant kICAUploadFileScaleToFit
        Use this constant to upload a file after scaling to fit a specified bounding rect.
}
const
	kICAUploadFileAsIs = $00000000;
	kICAUploadFileScaleToFit = $00000001;  

{!
    @struct ICAUploadFilePB
    @field header
        See description for ICAHeader. <->
    @field parentObject <->
        An ICAObject corresponding to a folder on the device. The device will store the uploaded file inside this folder if possible.
    @field fileFSRef <--
        An FSRef for the file to be uploaded to the device.
    @field flags <--
        One of the flags defined above.
}
type
	ICAUploadFilePBPtr = ^ICAUploadFilePB;
	ICAUploadFilePB = record
		header: ICAHeader;
		parentObject: ICAObject;
		fileFSRef: FSRefPtr;
		flags: UInt32;
	end;

{!
    @function ICAUploadFile
    @abstract
        Use this API to upload a file to a device that supports this capability.
    @discussion
        The device choses an appropriate destination location for the uploaded image and sends a kICANotificationTypeObjectAdded notification.
        
<pre>
@textblock
        Example:
        
        void  UploadFile()
        (
            OSErr           err;
            ICAUploadFilePB pb = ();

            pb.fileFSRef    = <#FSRef * fileFSRef#>;
            pb.flags        = <#UInt32 flags#>;
            pb.parentObject = <#ICAObject parentObject#>;
            
            err = ICAUploadFile( &pb, NULL );

            if ( noErr != err )
            (
                // handle error
            )
            else
            (
                // no return value(s)
            )
        )
@/textblock
</pre>

    @param pb
        A pointer to an <code><b>ICAUploadFilePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAUploadFile</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAUploadFile( var pb: ICAUploadFilePB; completion: ICACompletion ): ICAError; external name '_ICAUploadFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

//#pragma mark -
//#pragma mark Device related APIs
//---------------------------------------------------------------------------------------------------------- ICALoadDeviceModule
{ struct ICALoadDeviceModulePB
    Legal Key-Value pairs for populating paramDictionary:
            
    Key                             Value                               Comment
    
    kICADeviceModulePathKey         CFStringRef                         Path to the device module bundle that needs to be launched.
    kICATransportTypeKey            CFStringRef                         Should be one of the six predifined transport types.
    kICABluetoothAddressKey         CFStringRef                         Bluetooth device address string formatted as "00-11-22-33-44-55". 
    kICAUSBLocationIDKey            CFNumberRef (kCFNumberLongType)     32 bit USB location ID.
    kICAFireWireGUIDKey             CFNumberRef (kCFNumberLongLongType) 64 bit FireWire GUID.                                                               
    kICAIOServicePathKey            CFStringRef                         IO service path to the device obtained from the IO registry.
    kICAIPAddressKey                CFStringRef                         IP address of the device. This can be a host address ("camera.apple.com"),
                                                                        ipv4 address ('192.168.123.10") or ipv6 address ("3ff3:0000:0000:0000:0123:4567:89ab:cdef")
    kICAIPPortKey                   CFNumberRef (kCFNumberLongType)     IP port number of the device.
    kICAIPNameKey                   CFStringRef                         Human readable device name.
    kICAIPGUIDKey                   CFStringRef                         16 byte GUID string of the device formatted as "01234567-89ab-cdef-0123-456789abcdef".
    kICATWAINDSPathKey              CFStringRef                         Path to TWAIN DS bundle. }


{!
    @struct ICALoadDeviceModulePB
    @field header
        See description for ICAHeader. <->
    @field paramDictionary <--
        A parameter dictionary with sufficient key-value pairs to load a device module. This dictionary itself or the information provided in this dictionary will be sent to the device module.
}
type
	ICALoadDeviceModulePBPtr = ^ICALoadDeviceModulePB;
	ICALoadDeviceModulePB = record
		header: ICAHeader;
		paramDictionary: CFDictionaryRef;
	end;

{!
    @function ICALoadDeviceModule
    @abstract
        Use this API to load a device module.
    @discussion
        Typically, connecting a FireWire or an USB device will automatically load an appropriate device module. This API is needed only for loading a device module manually for devices that do not use a hot-plug interface, such as Bluetooth, SCSI, or TCP/IP.
    @param pb
        A pointer to an <code><b>ICALoadDeviceModulePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICALoadDeviceModule</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICALoadDeviceModule( var pb: ICALoadDeviceModulePB; completion: ICACompletion ): ICAError; external name '_ICALoadDeviceModule';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

//--------------------------------------------------------------------------------------------------------- ICAUnloadDeviceModule
{!
    @struct ICAUnloadDeviceModulePB
    @field header
        See description for ICAHeader. <->
    @field deviceObject <--
        A device ICAObject.
}
type
	ICAUnloadDeviceModulePBPtr = ^ICAUnloadDeviceModulePB;
	ICAUnloadDeviceModulePB = record
		header: ICAHeader;
		deviceObject: ICAObject;
	end;

{!
    @function ICAUnloadDeviceModule
    @abstract
        Uset this API to unload a device module.
    @discussion
        The device module providing this object will be unloaded, if this is the last device object provided by the device module.
    @param pb
        A pointer to an <code><b>ICAUnloadDeviceModulePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAUnloadDeviceModule</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAUnloadDeviceModule( var pb: ICAUnloadDeviceModulePB; completion: ICACompletion ): ICAError; external name '_ICAUnloadDeviceModule';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

//--------------------------------------------------------------------------------------------------------------- ICAOpenSession
{!
    @struct ICAOpenSessionPB
    @field header
        See description for ICAHeader. <->
    @field deviceObject
        A camera object. <--
    @field sessionID
        A session ID of the opened session. -->
}
type
	ICAOpenSessionPBPtr = ^ICAOpenSessionPB;
	ICAOpenSessionPB = record
		header: ICAHeader;
		deviceObject: ICAObject;
		sessionID: ICASessionID;
	end;

{!
    @function ICAOpenSession
    @abstract
        Use this API to open a session on a camera device. For a scanner device use the ICAScannerOpenSession API.
    @discussion
        This API gets a session ID for a open session on a camera device. Since access to cameras is generally not be session-based, this API generall will not fail. If the camera has open session, the device module controlling the camera will continue to control it during fast-user-switching.
    @param pb
        A pointer to an <code><b>ICAOpenSessionPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAOpenSession</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAOpenSession( var pb: ICAOpenSessionPB; completion: ICACompletion ): ICAError; external name '_ICAOpenSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

//-------------------------------------------------------------------------------------------------------------- ICACloseSession
{!
    @struct ICACloseSessionPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the session to be closed. <--
}
type
	ICACloseSessionPBPtr = ^ICACloseSessionPB;
	ICACloseSessionPB = record
		header: ICAHeader;
		sessionID: ICASessionID;
	end;

{!
    @function ICACloseSession
    @abstract
        Use this API to close a session on a camera device. For a scanner device use the ICAScannerCloseSession API.
    @discussion
        This API closes an open session on a camera device. If the camera does not have any open sessions, the device module controlling the camera is free to give it up during fast-user-switching.
    @param pb
        A pointer to an <code><b>ICACloseSessionPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICACloseSession</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICACloseSession( var pb: ICACloseSessionPB; completion: ICACompletion ): ICAError; external name '_ICACloseSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

//-------------------------------------------------------------------------------------------------------- ICAScannerOpenSession
{!
    @struct ICAScannerOpenSessionPB
    @field header
        See description for ICAHeader. <->
    @field object
        A scanner object. <--
    @field sessionID
        A session ID of the opened session. -->
}
type
	ICAScannerOpenSessionPBPtr = ^ICAScannerOpenSessionPB;
	ICAScannerOpenSessionPB = record
		header: ICAHeader;
		objct: ICAObject;
		sessionID: ICAScannerSessionID;
	end;

{!
    @function ICAScannerOpenSession
    @abstract
        Use this API to open a session on a scanner device. For a camera device use the ICAOpenSession API.
    @discussion
        For a given scanner, this API returns a unique session ID that allows you to work with the device. This API will fail, if a session is already open.
    @param pb
        A pointer to an <code><b>ICAScannerOpenSessionPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerOpenSession</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerOpenSession( var pb: ICAScannerOpenSessionPB; completion: ICACompletion ): ICAError; external name '_ICAScannerOpenSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

//------------------------------------------------------------------------------------------------------- ICAScannerCloseSession
{!
    @struct ICAScannerCloseSessionPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the session to be closed. <--
}
type
	ICAScannerCloseSessionPBPtr = ^ICAScannerCloseSessionPB;
	ICAScannerCloseSessionPB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
	end;

{!
    @function ICAScannerCloseSession
    @abstract
        Use this API to close a session on a scanner device. For a camera device use the ICACloseSession API.
    @discussion
        This API closes an open session, allowing other clients to work with the scanner.
    @param pb
        A pointer to an <code><b>ICAScannerCloseSessionPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerCloseSession</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerCloseSession( var pb: ICAScannerCloseSessionPB; completion: ICACompletion ): ICAError; external name '_ICAScannerCloseSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

//--------------------------------------------------------------------------------------------------------- ICAScannerInitialize
{!
    @struct ICAScannerInitializePB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the scanner to be initialized. <--
}
type
	ICAScannerInitializePBPtr = ^ICAScannerInitializePB;
	ICAScannerInitializePB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
	end;

{!
    @function ICAScannerInitialize
    @abstract
        Use this API to initialize a scanner device.
    @discussion
        After opening a session on a scanner device, use this API to set an initial state for the scanner.
    @param pb
        A pointer to an <code><b>ICAScannerInitializePB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerInitialize</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerInitialize( var pb: ICAScannerInitializePB; completion: ICACompletion ): ICAError; external name '_ICAScannerInitialize';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

//------------------------------------------------------------------------------------------------------ ICAScannerGetParameters
{!
    @struct ICAScannerGetParametersPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the scanner whose parameters are being fetched. <--
    @field theDict
        A dictionary containing the parameters. -->
}
type
	ICAScannerGetParametersPBPtr = ^ICAScannerGetParametersPB;
	ICAScannerGetParametersPB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
		theDict: CFMutableDictionaryRef;
	end;

{!
    @function ICAScannerGetParameters
    @abstract
        Use this API to get information about the scanner such as resolution, scanning area, etc.
    @discussion
        Use this API to get information about the scanner such as resolution, scanning area, etc.
    @param pb
        A pointer to an <code><b>ICAScannerGetParametersPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerGetParameters</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerGetParameters( var pb: ICAScannerGetParametersPB; completion: ICACompletion ): ICAError; external name '_ICAScannerGetParameters';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

//------------------------------------------------------------------------------------------------------ ICAScannerSetParameters
{!
    @struct ICAScannerSetParametersPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the scanner whose parameters are being set. <--
    @field theDict
        A dictionary containing the parameters. <--
}
type
	ICAScannerSetParametersPBPtr = ^ICAScannerSetParametersPB;
	ICAScannerSetParametersPB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
		theDict: CFMutableDictionaryRef;
	end;

{!
    @function ICAScannerSetParameters
    @abstract
        Use this API to specify scan parameters that will be used when a scan is initiated via an ICAScannerStart.
    @discussion
        Use this API to specify scan parameters that will be used when a scan is initiated via an ICAScannerStart.
    @param pb
        A pointer to an <code><b>ICAScannerSetParametersPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerSetParameters</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerSetParameters( var pb: ICAScannerSetParametersPB; completion: ICACompletion ): ICAError; external name '_ICAScannerSetParameters';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

//------------------------------------------------------------------------------------------------------------- ICAScannerStatus
{!
    @struct ICAScannerStatusPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the scanner whose status is being fetched. <--
    @field status
        A status value. -->
}
type
	ICAScannerStatusPBPtr = ^ICAScannerStatusPB;
	ICAScannerStatusPB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
		status: UInt32;
	end;

{!
    @function ICAScannerStatus
    @abstract
        Use this API to get information about the current status of the scanner.
    @discussion
        Use this API to get information about the current status of the scanner.
    @param pb
        A pointer to an <code><b>ICAScannerStatusPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerStatus</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerStatus( var pb: ICAScannerStatusPB; completion: ICACompletion ): ICAError; external name '_ICAScannerStatus';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

//-------------------------------------------------------------------------------------------------------------- ICAScannerStart
{!
    @struct ICAScannerStartPB
    @field header
        See description for ICAHeader. <->
    @field sessionID
        A session ID of the scanner that should start scanning. <--
}
type
	ICAScannerStartPBPtr = ^ICAScannerStartPB;
	ICAScannerStartPB = record
		header: ICAHeader;
		sessionID: ICAScannerSessionID;
	end;

{!
    @function ICAScannerStart
    @abstract
        Use this API start a scan based on the parameters that were specified in a previous ICAScannerSetParameters call.
    @discussion
        Use this API start a scan based on the parameters that were specified in a previous ICAScannerSetParameters call.
    @param pb
        A pointer to an <code><b>ICAScannerStartPB</b></code> parameter block.
    @param completion
        A pointer to a completion routine that will be invoked at the completion of <code><b>ICAScannerStart</b></code> function. Set this parameter to <code><b>NULL</b></code> to invoke this API synchronously. 
    @result
        Returns an error code defined in ICAApplication.h
}
function ICAScannerStart( var pb: ICAScannerStartPB; completion: ICACompletion ): ICAError; external name '_ICAScannerStart';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

//------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------------------

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
