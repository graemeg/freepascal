<<<<<<< HEAD
<<<<<<< HEAD
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
{------------------------------------------------------------------------------------------------------------------------------
 *
 *  ImageCapture/ICADevice.h
 *
 *  Copyright (c) 2000-2006 Apple Computer, Inc. All rights reserved.
 *
 *  For bug reports, consult the following page onthe World Wide Web:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  http://bugs.freepascal.org
 *
 *----------------------------------------------------------------------------------------------------------------------------}
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  http://www.freepascal.org/bugs.html
 *
 *----------------------------------------------------------------------------------------------------------------------------}
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
     File:       ICADevice.p
 
     Contains:   Image Capture device definitions.  This file is included
 
     Version:    Technology: 1.0
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  © 2000-2002 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}

=======
>>>>>>> origin/fixes_2.4

{------------------------------------------------------------------------------------------------------------------------------
 *
 *  ImageCapture/ICADevice.h
 *
 *  Copyright (c) 2000-2006 Apple Computer, Inc. All rights reserved.
 *
 *  For bug reports, consult the following page onthe World Wide Web:
 *  http://www.freepascal.org/bugs.html
 *
 *----------------------------------------------------------------------------------------------------------------------------}
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

unit ICADevice;
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
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
<<<<<<< HEAD
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
=======
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
uses MacTypes,ICAApplication;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======

>>>>>>> graemeg/cpstrnew

=======


>>>>>>> graemeg/cpstrnew
{$ifc TARGET_OS_MAC}
=======


{$ifc TARGET_OS_MAC}
>>>>>>> origin/cpstrnew

{$ALIGN MAC68K}

//------------------------------------------------------------------------------------------------------------------------------
{!
    @header ICADevice.h
    @discussion
        ICADevice.h defines structures and functions that are used by native Image Capture device modules. 
}

//-------------------------------------------------------------------------------------------------------------------- ICDHeader
{!
    @struct ICDHeader
    @discussion
        This is the first field in all parameter blocks used by APIs defined in ICADevices.h.
        Type of parameter passed to a callback function used by APIs defined in ICADevices.h.
        The parameter for the completion proc should to be casted to an appropriate type such as ICD_NewObjectPB* for it to be useful.
    @field err
        Error returned by an API. -->
    @field refcon
        An arbitrary refcon value passed to the callback. <--
}
type
	ICDHeaderPtr = ^ICDHeader;
	ICDHeader = record
		err: ICAError;
		refcon: UNSIGNEDLONG;
	end;

//--------------------------------------------------------------------------------------------------------------- Callback procs

{!
    @typedef ICDCompletion
    @discussion
        Type of callback function used by APIs defined in ICADevices.h.
    @param pb
        The parameter pb is a pointer to the parameter block passed to the API.
}

type
	ICDCompletion = procedure( var pb: ICDHeader );

//----------------------------------------------------------------------------------------------------------------- ICDNewObject
{!
    @struct ICD_NewObjectPB
    @discussion
        Parameter block passed to function <code>ICDNewObject</code>.
    @field header
        The function returns error code in the <code>err</code> field of this structure. 
        The <code>refcon</code> field of this structure is used to pass a pointer to the callback function if <code>ICDNewObject</code> is called asynchronously.
    @field  parentObject
        Parent object of the new object.
    @field  objectInfo
        <code>ICAObjectInfo</code> struct filled with information about the new object.
    @field  object
        New object.
}
type
	ICD_NewObjectPBPtr = ^ICD_NewObjectPB;
	ICD_NewObjectPB = record
		header: ICDHeader;
		parentObject: ICAObject;
		objectInfo: ICAObjectInfo;
		objct: ICAObject;
	end;

{!
    @function ICDNewObject
    @abstract
        A function to create a new object.
    @discussion
        Call this function to create a new object.
    @param pb
        An <code>ICD_NewObjectPB</code> structure.
    @param completion
        A pointer to a callback function that conforms to the interface of <code>ICDCompletion</code>. Pass <code>NULL</code> to make a synchronous call. 
    @result
        Returns an error code. If the function is called asynchronously, it returns <code>0</code> if the the call is accepted for asynchronous
        processing and returns an error code in the header passed to the callback function.
}
function ICDNewObject( var pb: ICD_NewObjectPB; completion: ICDCompletion ): ICAError; external name '_ICDNewObject';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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

//------------------------------------------------------------------------------------------------------------- ICDDisposeObject
{!
    @struct ICD_DisposeObjectPB
    @discussion
        Parameter block passed to function <code>ICDDisposeObject</code>.
    @field header
        The function returns error code in the <code>err</code> field of this structure. 
        The <code>refcon</code> field of this structure is used to pass a pointer to the callback function if <code>ICDDisposeObject</code> is called asynchronously.
    @field object
        Object to be disposed.
}
type
	ICD_DisposeObjectPBPtr = ^ICD_DisposeObjectPB;
	ICD_DisposeObjectPB = record
		header: ICDHeader;
		objct: ICAObject;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{!
    @function ICDDisposeObject
    @abstract
        A function to dispose an object.
    @discussion
        Call this function to dispose an object.
    @param pb
        An <code>ICD_DisposeObjectPB</code> structure.
    @param completion
        A pointer to a callback function that conforms to the interface of <code>ICDCompletion</code>. Pass <code>NULL</code> to make a synchronous call. 
    @result
        Returns an error code. If the function is called asynchronously, it returns <code>0</code> if the the call is accepted for asynchronous 
        processing and returns an error code in the header passed to the callback function.
}
function ICDDisposeObject( var pb: ICD_DisposeObjectPB; completion: ICDCompletion ): ICAError; external name '_ICDDisposeObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)

//------------------------------------------------------------------------------------------------------------------------------


=======

{!
    @function ICDDisposeObject
    @abstract
        A function to dispose an object.
    @discussion
        Call this function to dispose an object.
    @param pb
        An <code>ICD_DisposeObjectPB</code> structure.
    @param completion
        A pointer to a callback function that conforms to the interface of <code>ICDCompletion</code>. Pass <code>NULL</code> to make a synchronous call. 
    @result
        Returns an error code. If the function is called asynchronously, it returns <code>0</code> if the the call is accepted for asynchronous 
        processing and returns an error code in the header passed to the callback function.
}
function ICDDisposeObject( var pb: ICD_DisposeObjectPB; completion: ICDCompletion ): ICAError; external name '_ICDDisposeObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======

{!
    @function ICDDisposeObject
    @abstract
        A function to dispose an object.
    @discussion
        Call this function to dispose an object.
    @param pb
        An <code>ICD_DisposeObjectPB</code> structure.
    @param completion
        A pointer to a callback function that conforms to the interface of <code>ICDCompletion</code>. Pass <code>NULL</code> to make a synchronous call. 
    @result
        Returns an error code. If the function is called asynchronously, it returns <code>0</code> if the the call is accepted for asynchronous 
        processing and returns an error code in the header passed to the callback function.
}
function ICDDisposeObject( var pb: ICD_DisposeObjectPB; completion: ICDCompletion ): ICAError; external name '_ICDDisposeObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
=======

{!
    @function ICDDisposeObject
    @abstract
        A function to dispose an object.
    @discussion
        Call this function to dispose an object.
    @param pb
        An <code>ICD_DisposeObjectPB</code> structure.
    @param completion
        A pointer to a callback function that conforms to the interface of <code>ICDCompletion</code>. Pass <code>NULL</code> to make a synchronous call. 
    @result
        Returns an error code. If the function is called asynchronously, it returns <code>0</code> if the the call is accepted for asynchronous 
        processing and returns an error code in the header passed to the callback function.
}
function ICDDisposeObject( var pb: ICD_DisposeObjectPB; completion: ICDCompletion ): ICAError; external name '_ICDDisposeObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

//------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------------------
>>>>>>> origin/cpstrnew

{!
    @function ICDDisposeObject
    @abstract
        A function to dispose an object.
    @discussion
        Call this function to dispose an object.
    @param pb
        An <code>ICD_DisposeObjectPB</code> structure.
    @param completion
        A pointer to a callback function that conforms to the interface of <code>ICDCompletion</code>. Pass <code>NULL</code> to make a synchronous call. 
    @result
        Returns an error code. If the function is called asynchronously, it returns <code>0</code> if the the call is accepted for asynchronous 
        processing and returns an error code in the header passed to the callback function.
}
function ICDDisposeObject( var pb: ICD_DisposeObjectPB; completion: ICDCompletion ): ICAError; external name '_ICDDisposeObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

//------------------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD


//------------------------------------------------------------------------------------------------------------------------------
>>>>>>> graemeg/cpstrnew

//------------------------------------------------------------------------------------------------------------------------------

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew

//------------------------------------------------------------------------------------------------------------------------------


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
//------------------------------------------------------------------------------------------------------------------------------


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
//------------------------------------------------------------------------------------------------------------------------------
=======
>>>>>>> origin/cpstrnew

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

//------------------------------------------------------------------------------------------------------------------------------
{!
    @header ICADevice.h
    @discussion
        ICADevice.h defines structures and functions that are used by native Image Capture device modules. 
}

//-------------------------------------------------------------------------------------------------------------------- ICDHeader
{!
    @struct ICDHeader
    @discussion
        This is the first field in all parameter blocks used by APIs defined in ICADevices.h.
        Type of parameter passed to a callback function used by APIs defined in ICADevices.h.
        The parameter for the completion proc should to be casted to an appropriate type such as ICD_NewObjectPB* for it to be useful.
    @field err
        Error returned by an API. -->
    @field refcon
        An arbitrary refcon value passed to the callback. <--
}
type
	ICDHeaderPtr = ^ICDHeader;
	ICDHeader = record
		err: ICAError;
		refcon: UNSIGNEDLONG;
	end;

//--------------------------------------------------------------------------------------------------------------- Callback procs

{!
    @typedef ICDCompletion
    @discussion
        Type of callback function used by APIs defined in ICADevices.h.
    @param pb
        The parameter pb is a pointer to the parameter block passed to the API.
}

type
	ICDCompletion = procedure( var pb: ICDHeader );

//----------------------------------------------------------------------------------------------------------------- ICDNewObject
{!
    @struct ICD_NewObjectPB
    @discussion
        Parameter block passed to function <code>ICDNewObject</code>.
    @field header
        The function returns error code in the <code>err</code> field of this structure. 
        The <code>refcon</code> field of this structure is used to pass a pointer to the callback function if <code>ICDNewObject</code> is called asynchronously.
    @field  parentObject
        Parent object of the new object.
    @field  objectInfo
        <code>ICAObjectInfo</code> struct filled with information about the new object.
    @field  object
        New object.
}
type
	ICD_NewObjectPBPtr = ^ICD_NewObjectPB;
	ICD_NewObjectPB = record
		header: ICDHeader;
		parentObject: ICAObject;
		objectInfo: ICAObjectInfo;
		objct: ICAObject;
	end;

{!
    @function ICDNewObject
    @abstract
        A function to create a new object.
    @discussion
        Call this function to create a new object.
    @param pb
        An <code>ICD_NewObjectPB</code> structure.
    @param completion
        A pointer to a callback function that conforms to the interface of <code>ICDCompletion</code>. Pass <code>NULL</code> to make a synchronous call. 
    @result
        Returns an error code. If the function is called asynchronously, it returns <code>0</code> if the the call is accepted for asynchronous
        processing and returns an error code in the header passed to the callback function.
}
function ICDNewObject( var pb: ICD_NewObjectPB; completion: ICDCompletion ): ICAError; external name '_ICDNewObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

//------------------------------------------------------------------------------------------------------------- ICDDisposeObject
{!
    @struct ICD_DisposeObjectPB
    @discussion
        Parameter block passed to function <code>ICDDisposeObject</code>.
    @field header
        The function returns error code in the <code>err</code> field of this structure. 
        The <code>refcon</code> field of this structure is used to pass a pointer to the callback function if <code>ICDDisposeObject</code> is called asynchronously.
    @field object
        Object to be disposed.
}
type
	ICD_DisposeObjectPBPtr = ^ICD_DisposeObjectPB;
	ICD_DisposeObjectPB = record
		header: ICDHeader;
		objct: ICAObject;
	end;

{!
    @function ICDDisposeObject
    @abstract
        A function to dispose an object.
    @discussion
        Call this function to dispose an object.
    @param pb
        An <code>ICD_DisposeObjectPB</code> structure.
    @param completion
        A pointer to a callback function that conforms to the interface of <code>ICDCompletion</code>. Pass <code>NULL</code> to make a synchronous call. 
    @result
        Returns an error code. If the function is called asynchronously, it returns <code>0</code> if the the call is accepted for asynchronous 
        processing and returns an error code in the header passed to the callback function.
}
function ICDDisposeObject( var pb: ICD_DisposeObjectPB; completion: ICDCompletion ): ICAError; external name '_ICDDisposeObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

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
