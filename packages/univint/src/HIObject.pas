{
     File:       HIToolbox/HIObject.h
 
     Contains:   Base object for HIToolbox
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    HIToolbox-624~3
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> origin/cpstrnew
 
     Copyright:  © 2001-2008 by Apple Computer, Inc., all rights reserved.
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 2001-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 2001-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    HIToolbox-437~1
 
     Copyright:  © 2001-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/fixes_2_2
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> origin/fixes_2_2
 
}
{
      Change History (most recent first):

         <4>      5/8/04    GRP     Updated comments to match changes in HIObject.h, version HIToolbox-145.33~1.
         <3>      4/8/04    PNL     Moved HIObjectxxxx type declarations to HIObjectCore to fix cyclic dependency problem.
         <2>    10/02/04    GRP     Added support for GPC as well as CodeWarrior Pascal.
         <1>      9/8/03    GRP     First Pascal translation of HIObject.h, version HIToolbox-123.6~10.
}
{This file was processed by Dan's Source Converter}
{version 1.3 (this version modified by Ingemar Ragnemalm)}
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
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

unit HIObject;
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
=======
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
>>>>>>> graemeg/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,CFBase,CFBundle,Events,CarbonEventsCore,AXUIElement,CFDictionary;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
<<<<<<< HEAD
=======
uses MacTypes,CFBase,CFBundle,Events,CarbonEvents,HIObjectCore,AXUIElement,CFDictionary,CarbonEventsCore;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,CFBundle,Events,CarbonEvents,HIObjectCore,AXUIElement,CFDictionary,CarbonEventsCore;
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{$ALIGN POWER}


{
 *  HIObject
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    HIObject is the HIToolbox's base class for various objects. In
 *    Mac OS X 10.2 and later, most common user interface objects
 *    (controls, windows, menus, and toolbars) are derived from
 *    HIObject. Code which is external to HIToolbox can also create its
 *    own subclasses of our objects using the routines contained in
 *    this file. There are also polymorphic functions one can use on
 *    any HIObject for getting the class ID, etc. 
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
 *    HIObject is the HIToolbox's base class for various objects. Over
 *    time, all of our common objects (controls, windows, menus, etc.)
 *    will be derived from HIObject. Code which is external to
 *    HIToolbox can also create its own subclasses of our objects using
 *    the routines contained in this file. There are also polymorphic
 *    functions one can use on any HIObject for getting the class ID,
 *    etc. 
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *    
 *    HIObjects are actually CF types under the hood. This means that
 *    they can be put into CF collections and retain/release can be
 *    called on them. 
 *    
 *    An HIObject is essentially a very basic building-block object
 *    which contains an event target. You can create these objects to
 *    use as your own Carbon Event receptors in your application, or
 *    you can subclass existing Toolbox object to suit your needs.
 *    
 *    
 *    You register your subclasses with HIObjectRegisterSubclass,
 *    passing your class ID, the parent class, and an event handler.
 *    You also pass a list of events the handler is interested in.
 *    
 *    
 *    To create an object of your subclass, you call HIObjectCreate,
 *    passing the class ref you registered, as well as an
 *    initialization event. 
 *    
 *    Construction is two-phase: first the basic construction of the
 *    object is done, then initialization is performed. The toolbox
 *    sends construction events bottom-up, as you would expect in C++
 *    or the like. Here is the list of what goes on to create an
 *    object: 
 *    
 *    1)  The Toolbox creates the base HIObject 
 *    
 *    2)  It then installs the event handler you specified when you
 *    registered your subclass. Your handler must listen for
 *    kEventHIObjectConstruct and kEventHIObjectDestruct events. If it
 *    does not, the class cannot be registered (you will get a
 *    paramErr). 
 *    
 *    3)  Next, the Toolbox _directly_ calls your handler with an
 *    kEventHIObjectConstruct event. When called like this, you are not
 *    really being called in the context of a handler stack, so you
 *    cannot do things like CallNextEventHandler. The userData
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    parameter is what you specified as the inConstructData parameter
 *    to HIObjectRegisterSubclass. Typically, during construction you
 *    will allocate memory yourself to store your own instance data;
 *    this allocation might be as simple as calling malloc or NewPtr,
 *    or it might involve creating your own C++ object. In the
 *    construct event, the base HIObject of the object being created is
 *    passed as the kEventParamHIObjectInstance parameter. Typically
 *    you would store this HIObjectRef in your own instance data for
 *    later use. When handling this construct event, you should be sure
 *    to use SetEventParameter to set the kEventParamHIObjectInstance
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
 *    parameter is what you specified when you registered the class.
 *    Typically, during construction you will allocate memory yourself
 *    to store your own instance data; this allocation might be as
 *    simple as calling malloc or NewPtr, or it might involve creating
 *    your own C++ object. In the construct event, you are passed the
 *    base HIObjectRef of the object being created. Typically you would
 *    store this HIObjectRef in your own instance data for later use.
 *    When handling this construct event, you should be sure to use
 *    SetEventParameter to set the kEventParamHIObjectInstance
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *    parameter in the construction event with your own instance data.
 *    You must use typeVoidPtr as the type. 
 *    
 *    4)  The Toolbox looks for your instance of typeVoidPtr after you
 *    handle the construct event. It then takes that data and stores it
 *    off with the object and also sets the user data of the event
 *    handler it installed to be this instance data. This means that
 *    following the construct event, all calls to your event handler
 *    will have the instance data you returned to us. 
 *    
 *    5)  Once construction has completed successfully, we will send
 *    your object the initialize event passed into HIObjectCreate. At
 *    this point, all events are now sent to your object using standard
 *    Carbon Events mechanisms (it is only the construct event which is
 *    special). When we send the initialization event to your subclass,
 *    you should pass the event to your superclass before proceeding.
 *    You do this with CallNextEventHandler. Once back from that call,
 *    you should verify that the result is noErr, indicating that the
 *    superclass did in fact initialize properly. If it did not, your
 *    should return the error that CallNextEventHandler returned from
 *    your handler as well. The object will be destroyed by the
 *    Toolbox. Your object should be able to be destroyed in a
 *    partially initialized state such as this. This stage is optional,
 *    i.e. an object does not need to respond to the initialize event
 *    unless it is expecting certain parameters to be passed to it at
 *    creation time. This is where those parameters can be fetched.
 *    
 *    
 *    6)  Once initialization is successful, the HIObjectRef is
 *    returned to the caller of HIObjectCreate. From there, you can
 *    have all sorts of cool fun. 
 *    
 *    When someone has called CFRelease enough such that the refcount
 *    of the object drops to zero, the object is destroyed. The Toolbox
 *    will send a kEventHIObjectDestruct event to your object. DO NOT
 *    CALL CALLNEXTEVENTHANDLER. You will be setting yourself up for
 *    some hurt. Just clean up and return from your handler.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	HIObjectClassRef = ^OpaqueHIObjectClassRef; { an opaque type }
	OpaqueHIObjectClassRef = record end;
	HIObjectClassRefPtr = ^HIObjectClassRef; { when a var xx:HIObjectClassRef parameter can be nil, it is changed to xx: HIObjectClassRefPtr }
	HIObjectRef = ^OpaqueHIObjectRef; { an opaque type }
	OpaqueHIObjectRef = record end;
=======
	HIObjectClassRef = ^SInt32; { an opaque type }
	HIObjectClassRefPtr = ^HIObjectClassRef; { when a var xx:HIObjectClassRef parameter can be nil, it is changed to xx: HIObjectClassRefPtr }
	HIObjectRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	HIObjectClassRef = ^SInt32; { an opaque type }
	HIObjectClassRefPtr = ^HIObjectClassRef; { when a var xx:HIObjectClassRef parameter can be nil, it is changed to xx: HIObjectClassRefPtr }
	HIObjectRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	HIObjectClassRef = ^SInt32; { an opaque type }
	HIObjectClassRefPtr = ^HIObjectClassRef; { when a var xx:HIObjectClassRef parameter can be nil, it is changed to xx: HIObjectClassRefPtr }
	HIObjectRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	HIObjectClassRef = ^SInt32; { an opaque type }
	HIObjectClassRefPtr = ^HIObjectClassRef; { when a var xx:HIObjectClassRef parameter can be nil, it is changed to xx: HIObjectClassRefPtr }
	HIObjectRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
type
	HIObjectClassRef = ^SInt32; { an opaque type }
	HIObjectClassRefPtr = ^HIObjectClassRef; { when a var xx:HIObjectClassRef parameter can be nil, it is changed to xx: HIObjectClassRefPtr }
	HIObjectRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
{--------------------------------------------------------------------------------------}
{  • Constants                                                                         }
{--------------------------------------------------------------------------------------}

<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{
	The declarations for HIObjectClassRef, HIObjectClassRefPtr, and HIObjectRef 
	have been moved to HIObjectCore. See HIObjectCore.p[.pas] for reason.
}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  Discussion:
 *    HIObject errors
 }
const
{
   * You are trying to register a class ID that already exists.
   }
	hiObjectClassExistsErr = -22080;

  {
   * You are trying to unregister a class which has instances which
   * still exist. You must destroy them first, before they destroy you!
   }
	hiObjectClassHasInstancesErr = -22081;
	hiObjectClassHasSubclassesErr = -22082;

  {
   * You are trying to create an HIObject class that is defined as
   * being abstract. You must subclass it instead. Oh yes. Don't make
   * us say it twice!
   }
	hiObjectClassIsAbstractErr = -22083;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
  {
   * You may not register a subclass of a class that has the
   * kHIClassOptionSingleton attribute.
   }
	hiObjectCannotSubclassSingletonErr = -22084;

  {
   * Indicates that HIObjectAddDelegate has been called with a delegate
   * that already exists at the specified position.
   }
	hiObjectDelegateAlreadyExistsErr = -22090;

  {
   * Indicates that HIObjectRemoveDelegate has been called with a
   * delegate that could not be found at the specified position.
   }
	hiObjectDelegateNotFoundErr = -22091;


{
 *  Summary:
 *    Options for HIObjectRegisterSubclass.
 }
const
{
   * Indicates that HIObjectCreate should always return the same
   * instance of this class. This instance is created and cached on the
   * first call to HIObjectCreate for this class ID. All other calls to
   * HIObjectCreate will return the same instance. Available in Mac OS
   * X 10.5 and later.
   }
	kHIClassOptionSingleton = 1 shl 0;

{--------------------------------------------------------------------------------------}
{  • kEventClassHIObject                                                               }
{--------------------------------------------------------------------------------------}
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

const
{
   * The event class for HIObject events.
   }
	kEventClassHIObject = FourCharCode('hiob');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

const
{
   * [typeHIObjectRef or typeVoidPtr] An HIObjectRef that identifies an
   * instance of an HIObject subclass. This parameter is included in
   * the kEventHIObjectInitialize event.
   }
	kEventParamHIObjectInstance = FourCharCode('hioi');

  {
   * [typeCFTypeRef] An HIArchiveRef for encoding or decoding an
   * HIObject. This parameter may be included in the
   * kEventParamInitialize event. It is optional and may not be found
   * in all instances of this event. This parameter is also included in
   * the kEventHIObjectEncode event.
   }
	kEventParamHIArchive = FourCharCode('hiac');

  {
   * [typeCFArrayRef]  An array of HIObjectRefs that will be added as
   * delegates in the Before position on an HIObject. This parameter
   * may be included in the kEventHIObjectInitialize event. It is
   * optional and may not be found in all instances of this event. This
   * parameter is supported on Mac OS X 10.5 and later.
   }
	kEventParamBeforeDelegates = FourCharCode('bdel');

  {
   * [typeCFArrayRef]  An array of HIObjectRefs that will be added as
   * delegates in the After position on an HIObject. This parameter may
   * be included in the kEventHIObjectInitialize event. It is optional
   * and may not be found in all instances of this event. This
   * parameter is supported on Mac OS X 10.5 and later.
   }
	kEventParamAfterDelegates = FourCharCode('adel');

  {
   * [typeBoolean]  Indicates whether an object is being created inside
   * an editing environment. An object may choose to change its
   * behavior in this case. This parameter is included in the
   * kEventHIObjectInitialize and kEventHIObjectCreatedFromArchive
   * events. This parameter is supported on Mac OS X 10.5 and later.
   }
	kEventParamDecodingForEditor = FourCharCode('defe');

  {
   * [typeCFMutableArrayRef]  A list of dictionaries describing init
   * event parameters supported by this object class. This parameter is
   * included in the kEventHIObjectGetInitParameters event. This
   * parameter is supported on Mac OS X 10.5 and later.
   }
	kEventParamInitParameters = FourCharCode('para');

  {
   * The type for an event parameter that contains an HIObjectRef.
   }
	typeHIObjectRef = FourCharCode('hiob');

{
    Dictionary keys for dictionaries included in the kEventParamInitParameters parameter
    of the kEventHIObjectGetInitParameters event.
}
{
 *  kHIObjectInitParamUserName
 *  
 *  Discussion:
 *    The human-readable name of this parameter, such as "Bounds". The
 *    data for this key is a CFStringRef.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectInitParamUserName: CFStringRef; external name '_kHIObjectInitParamUserName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{
 *  kHIObjectInitParamDescription
 *  
 *  Discussion:
 *    A human-readable description of a parameter, such as "Bounding
 *    box of the view". The data for this key is a CFStringRef.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectInitParamDescription: CFStringRef; external name '_kHIObjectInitParamDescription'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{
 *  kHIObjectInitParamEventName
 *  
 *  Discussion:
 *    The Carbon event parameter name used for this parameter, such as
 *    'boun'. The data for this key is a CFString, typically created
 *    with the CreateTypeStringWithOSType API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectInitParamEventName: CFStringRef; external name '_kHIObjectInitParamEventName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{
 *  kHIObjectInitParamEventType
 *  
 *  Discussion:
 *    The Carbon event parameter type used for this parameter, such as
 *    typeHIRect. The data for this key is a CFString, typically
 *    created with the CreateTypeStringWithOSType API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectInitParamEventType: CFStringRef; external name '_kHIObjectInitParamEventType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{
=======
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
const
{
   * [typeHIObjectRef or typeVoidPtr] An HIObjectRef that identifies an
   * instance of an HIObject subclass. This parameter is included in
   * the kEventHIObjectInitialize event.
   }
	kEventParamHIObjectInstance = FourCharCode('hioi');

  {
   * [typeCFTypeRef] An HIArchiveRef for encoding or decoding an
   * HIObject. This parameter may be included in the
   * kEventParamInitialize event. It is optional and may not be found
   * in all instances of this event. This parameter is also included in
   * the kEventHIObjectEncode event.
   }
	kEventParamHIArchive = FourCharCode('hiac');

  {
   * [typeCFArrayRef]  An array of HIObjectRefs that will be added as
   * delegates in the Before position on an HIObject. This parameter
   * may be included in the kEventHIObjectInitialize event. It is
   * optional and may not be found in all instances of this event. This
   * parameter is supported on Mac OS X 10.5 and later.
   }
	kEventParamBeforeDelegates = FourCharCode('bdel');

  {
   * [typeCFArrayRef]  An array of HIObjectRefs that will be added as
   * delegates in the After position on an HIObject. This parameter may
   * be included in the kEventHIObjectInitialize event. It is optional
   * and may not be found in all instances of this event. This
   * parameter is supported on Mac OS X 10.5 and later.
   }
	kEventParamAfterDelegates = FourCharCode('adel');

  {
   * [typeBoolean]  Indicates whether an object is being created inside
   * an editing environment. An object may choose to change its
   * behavior in this case. This parameter is included in the
   * kEventHIObjectInitialize and kEventHIObjectCreatedFromArchive
   * events. This parameter is supported on Mac OS X 10.5 and later.
   }
	kEventParamDecodingForEditor = FourCharCode('defe');

  {
   * [typeCFMutableArrayRef]  A list of dictionaries describing init
   * event parameters supported by this object class. This parameter is
   * included in the kEventHIObjectGetInitParameters event. This
   * parameter is supported on Mac OS X 10.5 and later.
   }
	kEventParamInitParameters = FourCharCode('para');

  {
   * The type for an event parameter that contains an HIObjectRef.
   }
	typeHIObjectRef = FourCharCode('hiob');

{
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    Dictionary keys for dictionaries included in the kEventParamInitParameters parameter
    of the kEventHIObjectGetInitParameters event.
}
{
 *  kHIObjectInitParamUserName
 *  
 *  Discussion:
 *    The human-readable name of this parameter, such as "Bounds". The
 *    data for this key is a CFStringRef.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectInitParamUserName: CFStringRef; external name '_kHIObjectInitParamUserName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{
 *  kHIObjectInitParamDescription
 *  
 *  Discussion:
 *    A human-readable description of a parameter, such as "Bounding
 *    box of the view". The data for this key is a CFStringRef.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectInitParamDescription: CFStringRef; external name '_kHIObjectInitParamDescription'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{
 *  kHIObjectInitParamEventName
 *  
 *  Discussion:
 *    The Carbon event parameter name used for this parameter, such as
 *    'boun'. The data for this key is a CFString, typically created
 *    with the CreateTypeStringWithOSType API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectInitParamEventName: CFStringRef; external name '_kHIObjectInitParamEventName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{
 *  kHIObjectInitParamEventType
 *  
 *  Discussion:
 *    The Carbon event parameter type used for this parameter, such as
 *    typeHIRect. The data for this key is a CFString, typically
 *    created with the CreateTypeStringWithOSType API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectInitParamEventType: CFStringRef; external name '_kHIObjectInitParamEventType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{
>>>>>>> origin/fixes_2.4
    kEventClassHIObject quick reference:
    
    These are the events for base class functionality of HIObjects. You should only need to be
    aware of these if you are implementing a subclass of HIObject.
    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
    kEventHIObjectConstruct             = 1,
    kEventHIObjectInitialize            = 2,
    kEventHIObjectDestruct              = 3,
    kEventHIObjectIsEqual               = 4,
    kEventHIObjectPrintDebugInfo        = 5,
    kEventHIObjectEncode                = 6,
    kEventHIObjectCreatedFromArchive    = 7,
    kEventHIObjectGetInitParameters     = 8
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
    kEventHIObjectConstruct         = 1,
    kEventHIObjectInitialize        = 2,
    kEventHIObjectDestruct          = 3,
    kEventHIObjectIsEqual           = 4,
    kEventHIObjectPrintDebugInfo    = 5,
    kEventHIObjectEncode            = 6
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
}
{
 *  kEventClassHIObject / kEventHIObjectConstruct
 *  
 *  Summary:
 *    Your object is being constructed. You should allocate instance
 *    data for your object.
 *  
 *  Discussion:
 *    When your event handler is called with this event, it is being
 *    called directly and not through the normal event dispatching
 *    mechanism. This means that the EventHandlerCallRef passed to your
 *    handler will be NULL and CallNextEventHandler will not work. You
 *    are passed the actual HIObjectRef of your base class for you to
 *    record in your instance data.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    <-> kEventParamHIObjectInstance (in/out, typeHIObjectRef)
 *          On entry, this parameter is typeHIObjectRef, and is the
 *          HIObjectRef of your instance’s base class. Typically you
 *          will read this parameter from the event and store it in
 *          your instance data so that when your instance needs to call
 *          HIObject APIs, it can use this HIObjectRef.
 *          
 *          On exit, this parameter is typeVoidPtr, and should be a
 *          pointer to your instance data that you have written into
 *          the event with SetEventParameter. After your event handler
 *          returns, the toolbox reads your instance data pointer from
 *          the event, installs the event handlers that were passed to
 *          HIObjectRegisterSubclass on the new object, and uses the
 *          instance data pointer as the refcon for those event
 *          handlers. This allows your event handlers to retrieve your
 *          instance data pointer from the refcon.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectConstruct = 1;

{
<<<<<<< HEAD
 *  kEventClassHIObject / kEventHIObjectInitialize
 *  
 *  Summary:
 *    Your object is being initialized. You should read initialization
 *    data from the event and store it into your instance data.
 *  
 *  Discussion:
 *    Your handler should pass this event on to the superclass first
 *    before handling this event. This is done by calling
 *    CallNextEventHandler with the event. When that function returns,
 *    you should make sure the result is noErr. If not, you should NOT
<<<<<<< HEAD
 *    continue to initialize your object.
=======
 *    continue to initialize your class.
>>>>>>> graemeg/fixes_2_2
 *    
 *    Assuming that CallNextEventHandler returned noErr, you may then
 *    proceed to read initialization data from the event parameters, if
 *    any. For example, you might be create an object that includes a
 *    string as part of its instance data. The caller of HIObjectCreate
 *    would create an EventRef and add a string to the event using a
 *    parameter name and type defined by your object. In your object’s
 *    kEventHIObjectInitialize event handler, you would read the string
 *    from the event parameter and store it into your instance
 *    data.
 *    
 *    The parameters of this event, therefore, contain the union of all
 *    parameters needed by all base classes of this object to properly
 *    construct themselves.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamHIArchive (in, typeCFTypeRef)
 *          The HIArchive reference from which the HIObject should be
 *          decoded. This parameter will only exist when the HIObject
<<<<<<< HEAD
 *          is requested to initialize itself from a decoded archive.
 *          Otherwise, the HIObject should initialize itself normally.
 *    
 *    --> kEventParamDecodingForEditor (in, typeBoolean)
 *          If the kEventParamHIArchive parameter is present, then this
 *          parameter will also be present on Mac OS X 10.5 and later.
 *          It indicates whether the client that is creating the object
 *          is doing so for the purposes of editing the archive
 *          contents.
=======
 *          is requested to initialize itself from a decoded archive. 
 *          Otherwise, the HIObject should initialize itself normally.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectInitialize = 2;

{
 *  kEventClassHIObject / kEventHIObjectDestruct
 *  
 *  Summary:
 *    Your object is being destroyed. This is your chance to dispose of
 *    anything you might have allocated for your object’s instance data.
 *  
 *  Discussion:
 *    Do NOT call through with CallNextEventHandler, as you will
 *    disrupt the fabric of space-time. An HIObject is destroyed in
 *    most-derived to least-derived order, and if you call through,
 *    your base class can be destroyed before you destroy your own
 *    data, which can cause undefined results.
 *    
 *    Note that the refcon of your event handler for this event will be
 *    the instance data pointer allocated and returned by your
 *    kEventHIObjectConstruct handler in the
 *    kEventParamHIObjectInstance parameter.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectDestruct = 3;

{
 *  kEventClassHIObject / kEventHIObjectIsEqual
 *  
 *  Summary:
 *    HIObjectIsEqual has been called, and you are being asked to
 *    determine if your object is equivalent to the one being passed to
 *    your handler.
 *  
 *  Discussion:
 *    The base HIObject class handles this event by comparing the
 *    HIObjectRef values for pointer equality. Your subclass may choose
 *    to also compare the contents of the HIObject instance data.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The object to which your object should be compared.
 *    
 *    <-- kEventParamResult (out, typeBoolean)
 *          If your object is equivalent to the direct object, you
 *          should return true in this parameter; otherwise, return
 *          false.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectIsEqual = 4;

{
 *  kEventClassHIObject / kEventHIObjectPrintDebugInfo
 *  
 *  Summary:
 *    HIObjectPrintDebugInfo has been called, and you are being asked
 *    to print your information to stdout.
 *  
 *  Discussion:
 *    This event is sent to all handlers registered for it. You should
 *    NOT call CallNextEventHandler.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectPrintDebugInfo = 5;

{
 *  kEventClassHIObject / kEventHIObjectEncode
 *  
 *  Summary:
 *    Your object is being requested to encode itself into an archive.
 *  
 *  Discussion:
 *    HIArchiveEncodeCFType has been called on your HIObject, and you
 *    are being asked to encode any relevant long-term instance data to
 *    the HIArchive provided. Your handler should pass this event on to
 *    the superclass first before handling this event, by calling
 *    CallNextEventHandler with the event. If CallNextEventHandler does
 *    not return noErr, you should not continue to encode your instance
 *    data.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamHIArchive (in, typeCFTypeRef)
 *          The HIArchive reference into which the HIObject should be
 *          stored.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectEncode = 6;

{
 *  kEventClassHIObject / kEventHIObjectCreatedFromArchive
 *  
 *  Summary:
 *    Your object has been recreated from an archive.
 *  
 *  Discussion:
 *    This event is a notification that your object has been decoded
 *    from an archive. This event is also sent by IBCarbonRuntime after
 *    recreating a window, control, or menu from a nib file. The event
 *    is provided so that your object can perform any necessary
 *    post-decoding initialization or setup. 
 *    
 *    At the time this event is sent, the object has been completely
 *    decoded and initialized using data from the archive. However, the
 *    object has not yet been returned to the caller of the decoding
 *    API (either HIArchiveCopyDecodedCFType, CreateWindowFromNib, or
 *    CreateMenuFromNib), and so the caller will not be able to install
 *    an event handler on the object for this event before the event is
 *    sent. Therefore, there are only two ways to handle this event: an
 *    object that is a custom subclass of HIObject or HIView can
 *    install a handler for this event in its kEventHIObjectInitialize
 *    handler, or an object (not necessarily a custom subclass) can
 *    have the class ID of a delegate HIObject specified in its
 *    archived data, and the delegate object can have a handler for
 *    this event. 
 *    
 *    This event is only sent to the object that has been decoded; it
 *    is not propagated past that object. It is sent to all handlers
 *    registered for it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamHIObjectInstance (in, typeHIObjectRef)
 *          The object that has been recreated from an archive.
 *    
 *    --> kEventParamDecodingForEditor (in, typeBoolean)
 *          Indicates whether the client that is decoding the archive
 *          is doing so for the purposes of editing the archive
 *          contents. Event handlers for this event may choose to
 *          ignore the event in this case, if appropriate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectCreatedFromArchive = 7;

{
 *  kEventClassHIObject / kEventHIObjectGetInitParameters
 *  
 *  Summary:
 *    Requests information about the init event parameters supported by
 *    this object.
 *  
 *  Discussion:
 *    This event is typically used by UI editing environments that are
 *    providing the ability to edit an instance of an object. The
 *    object should respond to this event by first calling
 *    CallNextEventHandler, so that superclasses can add their own
 *    event parameters, and then by adding CFDictionaries containing
 *    information about the init event parameters supported by this
 *    object class. Each dictionary should, at a minimum, contain data
 *    for two keys, kHIObjectInitParamEventName and
 *    kHIObjectInitParamEventType, which are both CFStringRefs. The
 *    contents of each string should be a four-character code,
 *    indicating (respectively) the OSType for the event parameter, and
 *    the OSType for the parameter type. The dictionary may optionally
 *    contain data for other keys, as described above.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamHIObjectInstance (in, typeHIObjectRef)
 *          The object whose parameters are being requested.
 *    
 *    --> kEventParamInitParameters (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFDictionaries containing data about each
 *          init event parameter supported by this HIObject class.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectGetInitParameters = 8;

{--------------------------------------------------------------------------------------}
{  • kEventClassDelegate                                                               }
{--------------------------------------------------------------------------------------}

const
{
   * The event class for events that are sent to HIObject delegate
   * objects. This event class is available in Mac OS X 10.5 and later.
   }
	kEventClassDelegate = FourCharCode('dele');

  {
   * [typeHIObjectRef]  The HIObject to which a delegate is attached.
   }
	kEventParamDelegateTarget = FourCharCode('delt');

  {
   * [typeCFMutableArrayRef]  A list of HIObject classes to which a
   * delegate can be attached.
   }
	kEventParamDelegateTargetClasses = FourCharCode('trgc');

  {
   * [typeCFMutableArrayRef]  A list of HIObject delegate classes that
   * is installed automatically by this delegate.
   }
	kEventParamDelegateGroupClasses = FourCharCode('grpc');

  {
   * [typeCFDictionaryRef] This parameter is used with
   * kEventHIObjectInitialize when that event is sent to a delegate
   * group object. The event data is a dictionary containing custom
   * archive data for the subdelegates of a delegate group. The keys
   * are subdelegate class IDs, and the values are CFDictionaryRefs in
   * the custom archive data format.
   }
	kEventParamDelegateGroupParameters = FourCharCode('grpp');

{
    kEventClassDelegate quick reference:
    
    These are the events that are sent to HIObjects that are serving as delegates to another object.
    You should only need to be aware of these if you are implementing a delegate object.
    
    kEventDelegateInstalled         = 1,
    kEventDelegateRemoved           = 2,
    kEventDelegateGetTargetClasses  = 3,
    kEventDelegateIsGroup           = 4,
    kEventDelegateGetGroupClasses   = 5
}
{
 *  kEventClassDelegate / kEventDelegateInstalled
 *  
 *  Summary:
 *    Notifies a delegate that it has been installed on another object.
 *  
 *  Discussion:
 *    Most delegate objects can ignore this event, but a delegate that
 *    implements a delegate group should handle this event so it can
 *    install its subdelegate objects. This event is sent to all
 *    handlers registered for it. This event is only sent to the
 *    delegate object, and is not propagated past it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTarget (in, typeHIObjectRef)
 *          The object on which the delegate is being installed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateInstalled = 1;

{
 *  kEventClassDelegate / kEventDelegateRemoved
 *  
 *  Summary:
 *    Notifies a delegate that it has been removed from another object.
 *  
 *  Discussion:
 *    Most delegate objects can ignore this event, but a delegate that
 *    implements a delegate group should handle this event so it can
 *    remove its subdelegate objects. This event is sent to all
 *    handlers registered for it. This event is only sent to the
 *    delegate object, and is not propagated past it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTarget (in, typeHIObjectRef)
 *          The object from which the delegate is being removed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateRemoved = 2;

{
 *  kEventClassDelegate / kEventDelegateGetTargetClasses
 *  
 *  Summary:
 *    Requests a list of HIObject classes to which a delegate can be
 *    attached.
 *  
 *  Discussion:
 *    This event is typically used by user interface editor
 *    applications to determine whether a delegate object can be
 *    attached to a particular HIObject. If this event is not handled,
 *    the event sender should assume that the object can be attached to
 *    any type of object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTargetClasses (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFStringRefs containing the HIObject class
 *          IDs to which this object can be attached.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateGetTargetClasses = 3;

{
 *  kEventClassDelegate / kEventDelegateIsGroup
 *  
 *  Summary:
 *    Indicates whether a delegate is a delegate group.
 *  
 *  Discussion:
 *    A delegate group is a delegate object that installs other
 *    delegates when it is itself installed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    <-- kEventParamResult (out, typeBoolean)
 *          On return, indicates whether the object is a delegate
 *          group. If not present, the event sender should assume that
 *          the object is not a group.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateIsGroup = 4;

{
 *  kEventClassDelegate / kEventDelegateGetGroupClasses
 *  
 *  Summary:
<<<<<<< HEAD
 *    Retrieves the HIObject class IDs of the delegates that are
 *    installed by a delegate group.
=======
 *    Your object is being requested to encode itself into an archive.
 *  
 *  Discussion:
 *    HIArchiveEncodeCFType has been called on your HIObject, and you
 *    are being asked to encode any relevant long-term instance data to
 *    the HIArchive provided. Your handler should pass this event on to
 *    the superclass first before handling this event, by calling
 *    CallNextEventHandler with the event. If CallNextEventHandler does
 *    not return noErr, you should not continue to encode your instance
 *    data.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateGroupClasses (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFStringRefs containing the HIObject class
 *          IDs which this object installs.
=======
 *    --> kEventParamHIArchive (in, typeCFTypeRef)
 *          The HIArchive reference into which the HIObject should be
 *          stored.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateGetGroupClasses = 5;

{--------------------------------------------------------------------------------------}
{  • API                                                                               }
{--------------------------------------------------------------------------------------}
{
 *  kEventClassHIObject / kEventHIObjectCreatedFromArchive
 *  
 *  Summary:
 *    Your object has been recreated from an archive.
 *  
 *  Discussion:
 *    This event is a notification that your object has been decoded
 *    from an archive. This event is also sent by IBCarbonRuntime after
 *    recreating a window, control, or menu from a nib file. The event
 *    is provided so that your object can perform any necessary
 *    post-decoding initialization or setup. 
 *    
 *    At the time this event is sent, the object has been completely
 *    decoded and initialized using data from the archive. However, the
 *    object has not yet been returned to the caller of the decoding
 *    API (either HIArchiveCopyDecodedCFType, CreateWindowFromNib, or
 *    CreateMenuFromNib), and so the caller will not be able to install
 *    an event handler on the object for this event before the event is
 *    sent. Therefore, there are only two ways to handle this event: an
 *    object that is a custom subclass of HIObject or HIView can
 *    install a handler for this event in its kEventHIObjectInitialize
 *    handler, or an object (not necessarily a custom subclass) can
 *    have the class ID of a delegate HIObject specified in its
 *    archived data, and the delegate object can have a handler for
 *    this event. 
 *    
 *    This event is only sent to the object that has been decoded; it
 *    is not propagated past that object. It is sent to all handlers
 *    registered for it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamHIObjectInstance (in, typeHIObjectRef)
 *          The object that has been recreated from an archive.
 *    
 *    --> kEventParamDecodingForEditor (in, typeBoolean)
 *          Indicates whether the client that is decoding the archive
 *          is doing so for the purposes of editing the archive
 *          contents. Event handlers for this event may choose to
 *          ignore the event in this case, if appropriate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectCreatedFromArchive = 7;

{
 *  kEventClassHIObject / kEventHIObjectGetInitParameters
 *  
 *  Summary:
 *    Requests information about the init event parameters supported by
 *    this object.
 *  
 *  Discussion:
 *    This event is typically used by UI editing environments that are
 *    providing the ability to edit an instance of an object. The
 *    object should respond to this event by first calling
 *    CallNextEventHandler, so that superclasses can add their own
 *    event parameters, and then by adding CFDictionaries containing
 *    information about the init event parameters supported by this
 *    object class. Each dictionary should, at a minimum, contain data
 *    for two keys, kHIObjectInitParamEventName and
 *    kHIObjectInitParamEventType, which are both CFStringRefs. The
 *    contents of each string should be a four-character code,
 *    indicating (respectively) the OSType for the event parameter, and
 *    the OSType for the parameter type. The dictionary may optionally
 *    contain data for other keys, as described above.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamHIObjectInstance (in, typeHIObjectRef)
 *          The object whose parameters are being requested.
 *    
 *    --> kEventParamInitParameters (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFDictionaries containing data about each
 *          init event parameter supported by this HIObject class.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectGetInitParameters = 8;

{--------------------------------------------------------------------------------------}
{  • kEventClassDelegate                                                               }
{--------------------------------------------------------------------------------------}

const
{
   * The event class for events that are sent to HIObject delegate
   * objects. This event class is available in Mac OS X 10.5 and later.
   }
	kEventClassDelegate = FourCharCode('dele');

  {
   * [typeHIObjectRef]  The HIObject to which a delegate is attached.
   }
	kEventParamDelegateTarget = FourCharCode('delt');

  {
   * [typeCFMutableArrayRef]  A list of HIObject classes to which a
   * delegate can be attached.
   }
	kEventParamDelegateTargetClasses = FourCharCode('trgc');

  {
   * [typeCFMutableArrayRef]  A list of HIObject delegate classes that
   * is installed automatically by this delegate.
   }
	kEventParamDelegateGroupClasses = FourCharCode('grpc');

  {
   * [typeCFDictionaryRef] This parameter is used with
   * kEventHIObjectInitialize when that event is sent to a delegate
   * group object. The event data is a dictionary containing custom
   * archive data for the subdelegates of a delegate group. The keys
   * are subdelegate class IDs, and the values are CFDictionaryRefs in
   * the custom archive data format.
   }
	kEventParamDelegateGroupParameters = FourCharCode('grpp');

{
    kEventClassDelegate quick reference:
    
    These are the events that are sent to HIObjects that are serving as delegates to another object.
    You should only need to be aware of these if you are implementing a delegate object.
    
    kEventDelegateInstalled         = 1,
    kEventDelegateRemoved           = 2,
    kEventDelegateGetTargetClasses  = 3,
    kEventDelegateIsGroup           = 4,
    kEventDelegateGetGroupClasses   = 5
}
{
 *  kEventClassDelegate / kEventDelegateInstalled
 *  
 *  Summary:
 *    Notifies a delegate that it has been installed on another object.
 *  
 *  Discussion:
 *    Most delegate objects can ignore this event, but a delegate that
 *    implements a delegate group should handle this event so it can
 *    install its subdelegate objects. This event is sent to all
 *    handlers registered for it. This event is only sent to the
 *    delegate object, and is not propagated past it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTarget (in, typeHIObjectRef)
 *          The object on which the delegate is being installed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateInstalled = 1;

{
 *  kEventClassDelegate / kEventDelegateRemoved
 *  
 *  Summary:
 *    Notifies a delegate that it has been removed from another object.
 *  
 *  Discussion:
 *    Most delegate objects can ignore this event, but a delegate that
 *    implements a delegate group should handle this event so it can
 *    remove its subdelegate objects. This event is sent to all
 *    handlers registered for it. This event is only sent to the
 *    delegate object, and is not propagated past it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTarget (in, typeHIObjectRef)
 *          The object from which the delegate is being removed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateRemoved = 2;

{
 *  kEventClassDelegate / kEventDelegateGetTargetClasses
 *  
 *  Summary:
 *    Requests a list of HIObject classes to which a delegate can be
 *    attached.
 *  
 *  Discussion:
 *    This event is typically used by user interface editor
 *    applications to determine whether a delegate object can be
 *    attached to a particular HIObject. If this event is not handled,
 *    the event sender should assume that the object can be attached to
 *    any type of object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTargetClasses (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFStringRefs containing the HIObject class
 *          IDs to which this object can be attached.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateGetTargetClasses = 3;

{
 *  kEventClassDelegate / kEventDelegateIsGroup
 *  
 *  Summary:
 *    Indicates whether a delegate is a delegate group.
 *  
 *  Discussion:
 *    A delegate group is a delegate object that installs other
 *    delegates when it is itself installed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    <-- kEventParamResult (out, typeBoolean)
 *          On return, indicates whether the object is a delegate
 *          group. If not present, the event sender should assume that
 *          the object is not a group.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateIsGroup = 4;

{
 *  kEventClassDelegate / kEventDelegateGetGroupClasses
 *  
 *  Summary:
<<<<<<< HEAD
 *    Retrieves the HIObject class IDs of the delegates that are
 *    installed by a delegate group.
=======
 *    Your object is being requested to encode itself into an archive.
 *  
 *  Discussion:
 *    HIArchiveEncodeCFType has been called on your HIObject, and you
 *    are being asked to encode any relevant long-term instance data to
 *    the HIArchive provided. Your handler should pass this event on to
 *    the superclass first before handling this event, by calling
 *    CallNextEventHandler with the event. If CallNextEventHandler does
 *    not return noErr, you should not continue to encode your instance
 *    data.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateGroupClasses (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFStringRefs containing the HIObject class
 *          IDs which this object installs.
=======
 *    --> kEventParamHIArchive (in, typeCFTypeRef)
 *          The HIArchive reference into which the HIObject should be
 *          stored.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateGetGroupClasses = 5;

{--------------------------------------------------------------------------------------}
{  • API                                                                               }
{--------------------------------------------------------------------------------------}
{
 *  kEventClassHIObject / kEventHIObjectCreatedFromArchive
 *  
 *  Summary:
 *    Your object has been recreated from an archive.
 *  
 *  Discussion:
 *    This event is a notification that your object has been decoded
 *    from an archive. This event is also sent by IBCarbonRuntime after
 *    recreating a window, control, or menu from a nib file. The event
 *    is provided so that your object can perform any necessary
 *    post-decoding initialization or setup. 
 *    
 *    At the time this event is sent, the object has been completely
 *    decoded and initialized using data from the archive. However, the
 *    object has not yet been returned to the caller of the decoding
 *    API (either HIArchiveCopyDecodedCFType, CreateWindowFromNib, or
 *    CreateMenuFromNib), and so the caller will not be able to install
 *    an event handler on the object for this event before the event is
 *    sent. Therefore, there are only two ways to handle this event: an
 *    object that is a custom subclass of HIObject or HIView can
 *    install a handler for this event in its kEventHIObjectInitialize
 *    handler, or an object (not necessarily a custom subclass) can
 *    have the class ID of a delegate HIObject specified in its
 *    archived data, and the delegate object can have a handler for
 *    this event. 
 *    
 *    This event is only sent to the object that has been decoded; it
 *    is not propagated past that object. It is sent to all handlers
 *    registered for it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamHIObjectInstance (in, typeHIObjectRef)
 *          The object that has been recreated from an archive.
 *    
 *    --> kEventParamDecodingForEditor (in, typeBoolean)
 *          Indicates whether the client that is decoding the archive
 *          is doing so for the purposes of editing the archive
 *          contents. Event handlers for this event may choose to
 *          ignore the event in this case, if appropriate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectCreatedFromArchive = 7;

{
 *  kEventClassHIObject / kEventHIObjectGetInitParameters
 *  
 *  Summary:
 *    Requests information about the init event parameters supported by
 *    this object.
 *  
 *  Discussion:
 *    This event is typically used by UI editing environments that are
 *    providing the ability to edit an instance of an object. The
 *    object should respond to this event by first calling
 *    CallNextEventHandler, so that superclasses can add their own
 *    event parameters, and then by adding CFDictionaries containing
 *    information about the init event parameters supported by this
 *    object class. Each dictionary should, at a minimum, contain data
 *    for two keys, kHIObjectInitParamEventName and
 *    kHIObjectInitParamEventType, which are both CFStringRefs. The
 *    contents of each string should be a four-character code,
 *    indicating (respectively) the OSType for the event parameter, and
 *    the OSType for the parameter type. The dictionary may optionally
 *    contain data for other keys, as described above.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamHIObjectInstance (in, typeHIObjectRef)
 *          The object whose parameters are being requested.
 *    
 *    --> kEventParamInitParameters (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFDictionaries containing data about each
 *          init event parameter supported by this HIObject class.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectGetInitParameters = 8;

{--------------------------------------------------------------------------------------}
{  • kEventClassDelegate                                                               }
{--------------------------------------------------------------------------------------}

const
{
   * The event class for events that are sent to HIObject delegate
   * objects. This event class is available in Mac OS X 10.5 and later.
   }
	kEventClassDelegate = FourCharCode('dele');

  {
   * [typeHIObjectRef]  The HIObject to which a delegate is attached.
   }
	kEventParamDelegateTarget = FourCharCode('delt');

  {
   * [typeCFMutableArrayRef]  A list of HIObject classes to which a
   * delegate can be attached.
   }
	kEventParamDelegateTargetClasses = FourCharCode('trgc');

  {
   * [typeCFMutableArrayRef]  A list of HIObject delegate classes that
   * is installed automatically by this delegate.
   }
	kEventParamDelegateGroupClasses = FourCharCode('grpc');

  {
   * [typeCFDictionaryRef] This parameter is used with
   * kEventHIObjectInitialize when that event is sent to a delegate
   * group object. The event data is a dictionary containing custom
   * archive data for the subdelegates of a delegate group. The keys
   * are subdelegate class IDs, and the values are CFDictionaryRefs in
   * the custom archive data format.
   }
	kEventParamDelegateGroupParameters = FourCharCode('grpp');

{
    kEventClassDelegate quick reference:
    
    These are the events that are sent to HIObjects that are serving as delegates to another object.
    You should only need to be aware of these if you are implementing a delegate object.
    
    kEventDelegateInstalled         = 1,
    kEventDelegateRemoved           = 2,
    kEventDelegateGetTargetClasses  = 3,
    kEventDelegateIsGroup           = 4,
    kEventDelegateGetGroupClasses   = 5
}
{
 *  kEventClassDelegate / kEventDelegateInstalled
 *  
 *  Summary:
 *    Notifies a delegate that it has been installed on another object.
 *  
 *  Discussion:
 *    Most delegate objects can ignore this event, but a delegate that
 *    implements a delegate group should handle this event so it can
 *    install its subdelegate objects. This event is sent to all
 *    handlers registered for it. This event is only sent to the
 *    delegate object, and is not propagated past it.
=======
 *  kEventClassHIObject / kEventHIObjectInitialize
 *  
 *  Summary:
 *    Your object is being initialized. You should read initialization
 *    data from the event and store it into your instance data.
 *  
 *  Discussion:
 *    Your handler should pass this event on to the superclass first
 *    before handling this event. This is done by calling
 *    CallNextEventHandler with the event. When that function returns,
 *    you should make sure the result is noErr. If not, you should NOT
 *    continue to initialize your object.
 *    
 *    Assuming that CallNextEventHandler returned noErr, you may then
 *    proceed to read initialization data from the event parameters, if
 *    any. For example, you might be create an object that includes a
 *    string as part of its instance data. The caller of HIObjectCreate
 *    would create an EventRef and add a string to the event using a
 *    parameter name and type defined by your object. In your object’s
 *    kEventHIObjectInitialize event handler, you would read the string
 *    from the event parameter and store it into your instance
 *    data.
 *    
 *    The parameters of this event, therefore, contain the union of all
 *    parameters needed by all base classes of this object to properly
 *    construct themselves.
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTarget (in, typeHIObjectRef)
 *          The object on which the delegate is being installed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateInstalled = 1;

{
 *  kEventClassDelegate / kEventDelegateRemoved
 *  
 *  Summary:
 *    Notifies a delegate that it has been removed from another object.
 *  
 *  Discussion:
 *    Most delegate objects can ignore this event, but a delegate that
 *    implements a delegate group should handle this event so it can
 *    remove its subdelegate objects. This event is sent to all
 *    handlers registered for it. This event is only sent to the
 *    delegate object, and is not propagated past it.
=======
 *    --> kEventParamHIArchive (in, typeCFTypeRef)
 *          The HIArchive reference from which the HIObject should be
 *          decoded. This parameter will only exist when the HIObject
 *          is requested to initialize itself from a decoded archive.
 *          Otherwise, the HIObject should initialize itself normally.
 *    
 *    --> kEventParamDecodingForEditor (in, typeBoolean)
 *          If the kEventParamHIArchive parameter is present, then this
 *          parameter will also be present on Mac OS X 10.5 and later.
 *          It indicates whether the client that is creating the object
 *          is doing so for the purposes of editing the archive
 *          contents.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectInitialize = 2;

{
 *  kEventClassHIObject / kEventHIObjectDestruct
 *  
 *  Summary:
 *    Your object is being destroyed. This is your chance to dispose of
 *    anything you might have allocated for your object’s instance data.
 *  
 *  Discussion:
 *    Do NOT call through with CallNextEventHandler, as you will
 *    disrupt the fabric of space-time. An HIObject is destroyed in
 *    most-derived to least-derived order, and if you call through,
 *    your base class can be destroyed before you destroy your own
 *    data, which can cause undefined results.
 *    
 *    Note that the refcon of your event handler for this event will be
 *    the instance data pointer allocated and returned by your
 *    kEventHIObjectConstruct handler in the
 *    kEventParamHIObjectInstance parameter.
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTarget (in, typeHIObjectRef)
 *          The object from which the delegate is being removed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateRemoved = 2;

{
 *  kEventClassDelegate / kEventDelegateGetTargetClasses
 *  
 *  Summary:
 *    Requests a list of HIObject classes to which a delegate can be
 *    attached.
 *  
 *  Discussion:
 *    This event is typically used by user interface editor
 *    applications to determine whether a delegate object can be
 *    attached to a particular HIObject. If this event is not handled,
 *    the event sender should assume that the object can be attached to
 *    any type of object.
=======
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectDestruct = 3;

{
 *  kEventClassHIObject / kEventHIObjectIsEqual
 *  
 *  Summary:
 *    HIObjectIsEqual has been called, and you are being asked to
 *    determine if your object is equivalent to the one being passed to
 *    your handler.
 *  
 *  Discussion:
 *    The base HIObject class handles this event by comparing the
 *    HIObjectRef values for pointer equality. Your subclass may choose
 *    to also compare the contents of the HIObject instance data.
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
<<<<<<< HEAD
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTargetClasses (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFStringRefs containing the HIObject class
 *          IDs to which this object can be attached.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateGetTargetClasses = 3;

{
 *  kEventClassDelegate / kEventDelegateIsGroup
 *  
 *  Summary:
 *    Indicates whether a delegate is a delegate group.
 *  
 *  Discussion:
 *    A delegate group is a delegate object that installs other
 *    delegates when it is itself installed.
=======
=======
 *          The object to which your object should be compared.
 *    
 *    <-- kEventParamResult (out, typeBoolean)
 *          If your object is equivalent to the direct object, you
 *          should return true in this parameter; otherwise, return
 *          false.
 *  
 *  Availability:
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectIsEqual = 4;

{
 *  kEventClassHIObject / kEventHIObjectPrintDebugInfo
 *  
 *  Summary:
 *    HIObjectPrintDebugInfo has been called, and you are being asked
 *    to print your information to stdout.
 *  
 *  Discussion:
 *    This event is sent to all handlers registered for it. You should
 *    NOT call CallNextEventHandler.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    <-- kEventParamResult (out, typeBoolean)
 *          On return, indicates whether the object is a delegate
 *          group. If not present, the event sender should assume that
 *          the object is not a group.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateIsGroup = 4;

{
 *  kEventClassDelegate / kEventDelegateGetGroupClasses
 *  
 *  Summary:
 *    Retrieves the HIObject class IDs of the delegates that are
 *    installed by a delegate group.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectPrintDebugInfo = 5;

{
 *  kEventClassHIObject / kEventHIObjectEncode
 *  
 *  Summary:
 *    Your object is being requested to encode itself into an archive.
 *  
 *  Discussion:
<<<<<<< HEAD
 *    Your handler should pass this event on to the superclass first
 *    before handling this event. This is done by calling
 *    CallNextEventHandler with the event. When that function returns,
 *    you should make sure the result is noErr. If not, you should NOT
 *    continue to initialize your class.
 *    
 *    HIArchiveEncodeHIObject has been called on your HIObject. At this
 *    point all relevant long term data should be encoded to the
 *    HIArchive provided.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    HIArchiveEncodeCFType has been called on your HIObject, and you
 *    are being asked to encode any relevant long-term instance data to
 *    the HIArchive provided. Your handler should pass this event on to
 *    the superclass first before handling this event, by calling
 *    CallNextEventHandler with the event. If CallNextEventHandler does
 *    not return noErr, you should not continue to encode your instance
 *    data.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateGroupClasses (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFStringRefs containing the HIObject class
 *          IDs which this object installs.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateGetGroupClasses = 5;

{--------------------------------------------------------------------------------------}
{  • API                                                                               }
{--------------------------------------------------------------------------------------}
=======
=======
>>>>>>> origin/fixes_2_2
 *    --> kEventParamHIArchive (in, typeCFTypeRef)
 *          The HIArchive reference into which the HIObject should be
 *          stored.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectEncode = 6;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{
 *  kEventClassHIObject / kEventHIObjectCreatedFromArchive
 *  
 *  Summary:
 *    Your object has been recreated from an archive.
 *  
 *  Discussion:
 *    This event is a notification that your object has been decoded
 *    from an archive. This event is also sent by IBCarbonRuntime after
 *    recreating a window, control, or menu from a nib file. The event
 *    is provided so that your object can perform any necessary
 *    post-decoding initialization or setup. 
 *    
 *    At the time this event is sent, the object has been completely
 *    decoded and initialized using data from the archive. However, the
 *    object has not yet been returned to the caller of the decoding
 *    API (either HIArchiveCopyDecodedCFType, CreateWindowFromNib, or
 *    CreateMenuFromNib), and so the caller will not be able to install
 *    an event handler on the object for this event before the event is
 *    sent. Therefore, there are only two ways to handle this event: an
 *    object that is a custom subclass of HIObject or HIView can
 *    install a handler for this event in its kEventHIObjectInitialize
 *    handler, or an object (not necessarily a custom subclass) can
 *    have the class ID of a delegate HIObject specified in its
 *    archived data, and the delegate object can have a handler for
 *    this event. 
 *    
 *    This event is only sent to the object that has been decoded; it
 *    is not propagated past that object. It is sent to all handlers
 *    registered for it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamHIObjectInstance (in, typeHIObjectRef)
 *          The object that has been recreated from an archive.
 *    
 *    --> kEventParamDecodingForEditor (in, typeBoolean)
 *          Indicates whether the client that is decoding the archive
 *          is doing so for the purposes of editing the archive
 *          contents. Event handlers for this event may choose to
 *          ignore the event in this case, if appropriate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectCreatedFromArchive = 7;

{
 *  kEventClassHIObject / kEventHIObjectGetInitParameters
 *  
 *  Summary:
 *    Requests information about the init event parameters supported by
 *    this object.
 *  
 *  Discussion:
 *    This event is typically used by UI editing environments that are
 *    providing the ability to edit an instance of an object. The
 *    object should respond to this event by first calling
 *    CallNextEventHandler, so that superclasses can add their own
 *    event parameters, and then by adding CFDictionaries containing
 *    information about the init event parameters supported by this
 *    object class. Each dictionary should, at a minimum, contain data
 *    for two keys, kHIObjectInitParamEventName and
 *    kHIObjectInitParamEventType, which are both CFStringRefs. The
 *    contents of each string should be a four-character code,
 *    indicating (respectively) the OSType for the event parameter, and
 *    the OSType for the parameter type. The dictionary may optionally
 *    contain data for other keys, as described above.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamHIObjectInstance (in, typeHIObjectRef)
 *          The object whose parameters are being requested.
 *    
 *    --> kEventParamInitParameters (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFDictionaries containing data about each
 *          init event parameter supported by this HIObject class.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventHIObjectGetInitParameters = 8;

{--------------------------------------------------------------------------------------}
{  • kEventClassDelegate                                                               }
{--------------------------------------------------------------------------------------}

const
{
   * The event class for events that are sent to HIObject delegate
   * objects. This event class is available in Mac OS X 10.5 and later.
   }
	kEventClassDelegate = FourCharCode('dele');

  {
   * [typeHIObjectRef]  The HIObject to which a delegate is attached.
   }
	kEventParamDelegateTarget = FourCharCode('delt');

  {
   * [typeCFMutableArrayRef]  A list of HIObject classes to which a
   * delegate can be attached.
   }
	kEventParamDelegateTargetClasses = FourCharCode('trgc');

  {
   * [typeCFMutableArrayRef]  A list of HIObject delegate classes that
   * is installed automatically by this delegate.
   }
	kEventParamDelegateGroupClasses = FourCharCode('grpc');

  {
   * [typeCFDictionaryRef] This parameter is used with
   * kEventHIObjectInitialize when that event is sent to a delegate
   * group object. The event data is a dictionary containing custom
   * archive data for the subdelegates of a delegate group. The keys
   * are subdelegate class IDs, and the values are CFDictionaryRefs in
   * the custom archive data format.
   }
	kEventParamDelegateGroupParameters = FourCharCode('grpp');

{
    kEventClassDelegate quick reference:
    
    These are the events that are sent to HIObjects that are serving as delegates to another object.
    You should only need to be aware of these if you are implementing a delegate object.
    
    kEventDelegateInstalled         = 1,
    kEventDelegateRemoved           = 2,
    kEventDelegateGetTargetClasses  = 3,
    kEventDelegateIsGroup           = 4,
    kEventDelegateGetGroupClasses   = 5
}
{
 *  kEventClassDelegate / kEventDelegateInstalled
 *  
 *  Summary:
 *    Notifies a delegate that it has been installed on another object.
 *  
 *  Discussion:
 *    Most delegate objects can ignore this event, but a delegate that
 *    implements a delegate group should handle this event so it can
 *    install its subdelegate objects. This event is sent to all
 *    handlers registered for it. This event is only sent to the
 *    delegate object, and is not propagated past it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTarget (in, typeHIObjectRef)
 *          The object on which the delegate is being installed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateInstalled = 1;

{
 *  kEventClassDelegate / kEventDelegateRemoved
 *  
 *  Summary:
 *    Notifies a delegate that it has been removed from another object.
 *  
 *  Discussion:
 *    Most delegate objects can ignore this event, but a delegate that
 *    implements a delegate group should handle this event so it can
 *    remove its subdelegate objects. This event is sent to all
 *    handlers registered for it. This event is only sent to the
 *    delegate object, and is not propagated past it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTarget (in, typeHIObjectRef)
 *          The object from which the delegate is being removed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateRemoved = 2;

{
 *  kEventClassDelegate / kEventDelegateGetTargetClasses
 *  
 *  Summary:
 *    Requests a list of HIObject classes to which a delegate can be
 *    attached.
 *  
 *  Discussion:
 *    This event is typically used by user interface editor
 *    applications to determine whether a delegate object can be
 *    attached to a particular HIObject. If this event is not handled,
 *    the event sender should assume that the object can be attached to
 *    any type of object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateTargetClasses (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFStringRefs containing the HIObject class
 *          IDs to which this object can be attached.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateGetTargetClasses = 3;

{
 *  kEventClassDelegate / kEventDelegateIsGroup
 *  
 *  Summary:
 *    Indicates whether a delegate is a delegate group.
 *  
 *  Discussion:
 *    A delegate group is a delegate object that installs other
 *    delegates when it is itself installed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    <-- kEventParamResult (out, typeBoolean)
 *          On return, indicates whether the object is a delegate
 *          group. If not present, the event sender should assume that
 *          the object is not a group.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateIsGroup = 4;

{
 *  kEventClassDelegate / kEventDelegateGetGroupClasses
 *  
 *  Summary:
 *    Retrieves the HIObject class IDs of the delegates that are
 *    installed by a delegate group.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeHIObjectRef)
 *          The delegate object.
 *    
 *    --> kEventParamDelegateGroupClasses (in, typeCFMutableArrayRef)
 *          On entry, contains a mutable CFArrayRef. A handler for this
 *          event should add CFStringRefs containing the HIObject class
 *          IDs which this object installs.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventDelegateGetGroupClasses = 5;

{--------------------------------------------------------------------------------------}
{  • API                                                                               }
{--------------------------------------------------------------------------------------}
{
 *  HIObjectRegisterSubclass()
 *  
 *  Discussion:
 *    Registers a class with the Toolbox for creation later.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inClassID:
 *      The class ID of your class. It should be unique. We recommend
 *      using Java-style com.company.foo naming conventions to avoid
 *      collisions.
 *    
 *    inBaseClassID:
 *      The class ID of the class you derive from. Passing NULL
 *      indicates you wish to subclass HIObject (the base class)
 *      directly.
 *    
 *    inOptions:
 *      Any special options for your class. Currently you must pass 0
 *      for this parameter.
 *    
 *    inConstructProc:
 *      The construction proc for this subclass. You pass the address
 *      of an event handler into this parameter. This handler is called
 *      directly, rather than through the normal event dispatching
 *      mechanism. This means that the EventHandlerCallRef passed in
 *      will be NULL, and you cannot use it for calls like
 *      CallNextEventHandler. Other than that, you should return a
 *      result as usual. After your object is constructed, this proc
 *      will be installed as the event handler for the remaining events
 *      specified in the inEventList parameter. On Mac OS X 10.4 and
 *      later, you may pass NULL to create an "abstract class" that
 *      cannot be instantiated, but can still be used as a base class
 *      for subclasses; if you pass NULL, HIObjectCreate on the class
 *      ID will return hiObjectClassIsAbstractErr.
 *    
 *    inNumEvents:
 *      The number of events you are installing.
 *    
 *    inEventList:
 *      The events your handler wishes to receive. If you are not
 *      creating an abstract class, then you must handle the
 *      kEventHIObjectConstruct and kEventHIObjectDestruct event. If
 *      these events are not specified, an error is returned. An
 *      abstract class may pass 0 for the inNumEvents parameter and
 *      NULL for the inEventList parameter.
 *    
 *    inConstructData:
 *      Pass any info you want passed into your event handler here. For
 *      a C++ hierarchy based on HIObjects, you might actually pass a
 *      static method to construct your object here, and the base class
 *      event handler to do construction as your event handler.
 *    
 *    outClassRef:
 *      The newly created class reference. Pass NULL if you don't care.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function HIObjectRegisterSubclass( inClassID: CFStringRef; inBaseClassID: CFStringRef; inOptions: OptionBits; inConstructProc: EventHandlerUPP { can be NULL }; inNumEvents: ItemCount; {const} inEventList: {variable-size-array} EventTypeSpecPtr; inConstructData: UnivPtr; outClassRef: HIObjectClassRefPtr { can be NULL } ): OSStatus; external name '_HIObjectRegisterSubclass';
=======
function HIObjectRegisterSubclass( inClassID: CFStringRef; inBaseClassID: CFStringRef; inOptions: OptionBits; inConstructProc: EventHandlerUPP { can be NULL }; inNumEvents: UInt32; {const} inEventList: {variable-size-array} EventTypeSpecPtr; inConstructData: UnivPtr; outClassRef: HIObjectClassRefPtr { can be NULL } ): OSStatus; external name '_HIObjectRegisterSubclass';
>>>>>>> graemeg/fixes_2_2
=======
function HIObjectRegisterSubclass( inClassID: CFStringRef; inBaseClassID: CFStringRef; inOptions: OptionBits; inConstructProc: EventHandlerUPP { can be NULL }; inNumEvents: UInt32; {const} inEventList: {variable-size-array} EventTypeSpecPtr; inConstructData: UnivPtr; outClassRef: HIObjectClassRefPtr { can be NULL } ): OSStatus; external name '_HIObjectRegisterSubclass';
>>>>>>> origin/fixes_2_2
=======
function HIObjectRegisterSubclass( inClassID: CFStringRef; inBaseClassID: CFStringRef; inOptions: OptionBits; inConstructProc: EventHandlerUPP { can be NULL }; inNumEvents: ItemCount; {const} inEventList: {variable-size-array} EventTypeSpecPtr; inConstructData: UnivPtr; outClassRef: HIObjectClassRefPtr { can be NULL } ): OSStatus; external name '_HIObjectRegisterSubclass';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIObjectUnregisterClass()
 *  
 *  Discussion:
 *    Unregisters a previously registered subclass of HIObject. You
 *    will receive an error if there are subclasses of your class or
 *    instances of it which still exist. All instances and subclasses
 *    must be disposed of and unregistered first.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inClassRef:
 *      The class ref of the class of object you wish to unregister.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectUnregisterClass( inClassRef: HIObjectClassRef ): OSStatus; external name '_HIObjectUnregisterClass';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIObjectCreate()
 *  
 *  Discussion:
 *    Creates an object derived from HIObject.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inClassID:
 *      The class ID of the class of object you wish to instantiate.
 *    
 *    inConstructData:
 *      If your class (or any class you derive from) accepts creation
 *      parameters, you need to pass an event into this parameter. The
 *      class must be kEventClassHIObject, and the kind should be
 *      kEventHIObjectInitialize. Any other parameters should be added
 *      as necessary. Specific subclasses of HIObject which require
 *      initialization parameters will specify those parameters in the
 *      appropriate headers.
 *    
 *    outObject:
 *      The instance of the object you create.
 *  
 *  Result:
 *    An operating system result code. A return value of
 *    hiObjectClassIsAbstractErr indicates that the inConstructProc
 *    parameter to HIObjectRegisterSubclass was NULL; instances of such
 *    a class may not be created, only subclassed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectCreate( inClassID: CFStringRef; inConstructData: EventRef; var outObject: HIObjectRef ): OSStatus; external name '_HIObjectCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIObjectGetEventTarget()
 *  
 *  Discussion:
 *    Returns the event target of an HIObjectRef.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object whose target you want.
 *  
 *  Result:
 *    An EventTargetRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectGetEventTarget( inObject: HIObjectRef ): EventTargetRef; external name '_HIObjectGetEventTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIObjectPrintDebugInfo()
 *  
 *  Discussion:
 *    Prints the internal information of an HIObject for debugging
 *    purposes. It outputs the info to stdout.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object to inspect.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
procedure HIObjectPrintDebugInfo( inObject: HIObjectRef ); external name '_HIObjectPrintDebugInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIObjectCopyClassID()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Returns the class ID of a given HIObject. 
 *    
 *    This API will crash if called from an kEventHIObjectConstruct
 *    handler in Mac OS X 10.4 and earlier. It successfully returns the
 *    object's class ID in Mac OS X 10.5 and later.
<<<<<<< HEAD
=======
 *    Returns the class ID of a given HIObject.
>>>>>>> graemeg/fixes_2_2
=======
 *    Returns the class ID of a given HIObject.
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object whose class ID you are interested in.
 *  
 *  Result:
 *    A CFStringRef containing the object's class ID.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectCopyClassID( inObject: HIObjectRef ): CFStringRef; external name '_HIObjectCopyClassID';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIObjectIsOfClass()
 *  
 *  Discussion:
 *    Returns whether or not an object is of a certain class. You can
 *    us this to see whether or not an object you have derives from an
 *    expected superclass.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object whose class ID you wish to check.
 *    
 *    inObjectClassID:
 *      The class ID in question.
 *  
 *  Result:
 *    A Boolean result indicating whether or not the object is of the
 *    class specified.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectIsOfClass( inObject: HIObjectRef; inObjectClassID: CFStringRef ): Boolean; external name '_HIObjectIsOfClass';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIObjectDynamicCast()
 *  
 *  Discussion:
 *    Returns the instance data for a specific class of an HIObject.
 *    The instance data returned is the same instance data the class's
 *    construction event handler returns in the instance data
 *    parameter. This is stored off with the class reference so that it
 *    can be fetched later for use by this function. It allows your
 *    subclass to easily get at the data it created, if your subclass
 *    needs that data outside of an event handler. (Inside an event
 *    handler, your subclass can get at its instance data via the
 *    userData parameter to the event handler.)
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object whose class ID you wish to check.
 *    
 *    inClassID:
 *      The class ID to get the instance data for.
 *  
 *  Result:
 *    A void * result which contains the instance data for the object,
 *    or NULL if the object is not an instance of the class.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectDynamicCast( inObject: HIObjectRef; inClassID: CFStringRef ): UnivPtr; external name '_HIObjectDynamicCast';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIObjectCreateFromBundle()
 *  
 *  Discussion:
 *    Returns the HIObject for the given bundle. A bundle can be
 *    designed to communicate with an app through an HIObject. The
 *    bundle must be designed to create an HIObject and have a defined
 *    suite of CarbonEvents that clients can use to communicate with
 *    the bundle's HIObject. Given a CFBundleRef, this API will tell
 *    the bundle to create the HIObject and return it to the caller.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBundle:
 *      The bundle that you wish to communicate with.
 *    
 *    outObject:
 *      The HIObject associated with the bundle.
 *  
 *  Result:
 *    An operating system result code. If the bundle's HIObject
 *    creation function cannot be found, cfragNoSymbolErr will be
 *    returned.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectCreateFromBundle( inBundle: CFBundleRef; var outObject: HIObjectRef ): OSStatus; external name '_HIObjectCreateFromBundle';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  HIObjectFromEventTarget()
 *  
 *  Summary:
 *    Returns the HIObjectRef that owns an event target.
=======
=======
>>>>>>> origin/fixes_2_2
 *  HIObjectIsAccessibilityIgnored()
 *  
 *  Discussion:
 *    Reports whether or not the given HIObject is marked as ignored
 *    for accessibility. 
 *    
 *    See the discussion of HIObjectSetAccessibilityIgnored for details
 *    on what it means to be accessibility ignored.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object whose accessibility ignored state you wish to query.
 *  
 *  Result:
 *    A Boolean value indicating whether or not the HIObject is ignored
 *    for accessibility.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectIsAccessibilityIgnored( inObject: HIObjectRef ): Boolean; external name '_HIObjectIsAccessibilityIgnored';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIObjectSetAccessibilityIgnored()
 *  
 *  Discussion:
 *    Marks an HIObject as ignored (or not) for the purposes of the
 *    accessibility APIs. 
 *    
 *    An HIObject that is ignored for accessibility will never be shown
 *    to an assistive application that uses the accessibility APIs to
 *    examine an interface. Your application's accessibility
 *    implementation can (and should) still report an ignored HIObject
 *    as usual. Carbon's accessibility engine will automatically prune
 *    any ignored HIObjects out of the data that is shown to an
 *    assistive application. 
 *    <BR>By default, an HIObject is *not* accessibility ignored.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object whose accessibility ignored state you wish to change.
 *    
 *    inIgnored:
 *      A Boolean value indicating whether or not to ignore the object.
 *  
 *  Result:
 *    An OSStatus signifying success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectSetAccessibilityIgnored( inObject: HIObjectRef; inIgnored: Boolean ): OSStatus; external name '_HIObjectSetAccessibilityIgnored';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIObjectSetAuxiliaryAccessibilityAttribute()
 *  
 *  Summary:
 *    Associates an additional accessibility attribute with a UIElement
 *    that is used to represent a given HIObject or a part thereof.
 *  
 *  Discussion:
 *    This routine lets your application provide the name of and data
 *    for an accessibility attribute that you want to add to the
 *    UIElement used to represent a given HIObject-identifier pair.
 *    Normally, accessibility attributes can only be supplied
 *    dynamically via Carbon Events, but this routine allows you to
 *    supply them statically. 
 *    
 *    When an accessibility attribute Carbon Event is handled by the
 *    HIObject with a given identifier, the toolbox automatically
 *    supplies the names and/or values of any auxiliary attributes
 *    associated with that HIObject- identifier pair. 
 *    
 *    This routine is particularly useful for supplying values for the
 *    kAXDescriptionAttribute, kAXTitleUIElementAttribute,
 *    kAXServesAsTitleUIElementAttribute, kAXLinkedUIElementsAttribute
 *    and other attributes whose value is specific to the layout and
 *    usage of your application. 
 *    
 *    This routine only allows you to associate attributes whose values
 *    never change. If you need to supply attributes whose values are
 *    determined dynamically or whose values are settable, you must
 *    install the normal accessibility Carbon Event handlers. 
 *    
 *    The auxiliary attribute store is consulted during the HIObject's
 *    default handling of the accessibility attribute Carbon Events.
 *    This means that any programmatic handling of a given
 *    accessibility attribute will have a chance to override or block
 *    the consultation of the store. The general rule is that if the
 *    toolbox or a Carbon Event handler can provide the attribute value
 *    some other way, the store will not be consulted.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
 *    inTarget:
 *      The event target whose owing HIObjectRef to return.
 *  
 *  Result:
 *    The HIObjectRef that owns the event target, or NULL if the target
 *    is invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIObjectFromEventTarget( inTarget: EventTargetRef ): HIObjectRef; external name '_HIObjectFromEventTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{--------------------------------------------------------------------------------------}
{  • Archiving                                                                         }
{--------------------------------------------------------------------------------------}
=======
=======
>>>>>>> origin/fixes_2_2
 *    inHIObject:
 *      The HIObjectRef part of the object-identifier pair to which the
 *      attribute data is associated.
 *    
 *    inIdentifier:
 *      The 64-bit identifier part of the object-identifier pair to
 *      which the attribute data is associated. When you want to
 *      associate the attribute data to the HIObject as a whole -- such
 *      as when you want to give a description attribute to a push
 *      button -- you should pass zero in this parameter.
 *    
 *    inAttributeName:
 *      A CFStringRef of the name of the attribute you wish to
 *      associate with the object-identifier pair. This string will be
 *      retained before adding it to the auxiliary attribute store.
 *    
 *    inAttributeData:
 *      A CFTypeRef with the data to be supplied as the attribute's
 *      value. This data will be retained before adding it to the
 *      auxiliary attribute store; you may release inAttributeData
 *      after you have called this routine. The inAttributeData
 *      parameter may also be NULL, which indicates that the named
 *      auxiliary attribute should no longer be associated with the
 *      object-identifier pair; any named attribute data previously
 *      associated with the object-identifier pair will be released.
 *  
 *  Result:
 *    An OSStatus result code. The function will return noErr if it was
 *    able to associate the attribute data with the HIObjectRef. If the
 *    HIObjectRef is invalid, paramErr will be returned. Other results
 *    may be returned in other situations.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectSetAuxiliaryAccessibilityAttribute( inHIObject: HIObjectRef; inIdentifier: UInt64; inAttributeName: CFStringRef; inAttributeData: CFTypeRef { can be NULL } ): OSStatus; external name '_HIObjectSetAuxiliaryAccessibilityAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  HIObjectOverrideAccessibilityContainment()
=======
 *  HIObjectFromEventTarget()
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Returns the HIObjectRef that owns an event target.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTarget:
 *      The event target whose owing HIObjectRef to return.
 *  
 *  Result:
 *    The HIObjectRef that owns the event target, or NULL if the target
 *    is invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIObjectFromEventTarget( inTarget: EventTargetRef ): HIObjectRef; external name '_HIObjectFromEventTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{--------------------------------------------------------------------------------------}
{  • Archiving                                                                         }
{--------------------------------------------------------------------------------------}
>>>>>>> origin/fixes_2.4
{
 *  HIObjectIsArchivingIgnored()
 *  
 *  Discussion:
 *    Reports whether or not the given HIObject is marked as ignored
 *    for archiving.
 *    See the discussion of HIObjectSetArchivingIgnored for details on
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    what it means to be archiving-ignored.
=======
 *    what it means to be archiving ignored.
>>>>>>> graemeg/fixes_2_2
=======
 *    what it means to be archiving ignored.
>>>>>>> origin/fixes_2_2
=======
 *    what it means to be archiving-ignored.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      The object whose archiving-ignored state you wish to query.
=======
 *      The object whose archiving ignored state you wish to query.
>>>>>>> graemeg/fixes_2_2
=======
 *      The object whose archiving ignored state you wish to query.
>>>>>>> origin/fixes_2_2
=======
 *      The object whose archiving-ignored state you wish to query.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    A Boolean value indicating whether or not the HIObject is ignored
 *    for archiving.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectIsArchivingIgnored( inObject: HIObjectRef ): Boolean; external name '_HIObjectIsArchivingIgnored';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  HIObjectSetArchivingIgnored()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Call this function to mark or unmark an HIObject as ignored for
 *    archiving. By default, HIObjects are marked as ignored for
 *    archiving. HIObject subclasses that support archiving with the
 *    kEventHIObjectInitialize and kEventHIObjectEncode events must set
 *    their archiving-ignored state to false in order to receive
 *    archiving requests from clients. Typically, an HIObject subclass
 *    will call HIObjectSetArchivingIgnored in its
 *    kEventHIObjectInitialize handler. A client may still reset the
 *    archiving-ignored state to true on a particular object after the
 *    object has been initialized. An HIObject marked as ignored for
 *    archiving will never be requested to encode itself into an
 *    archive.
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
 *    Marks an HIObject as ignored (or not) for the purposes of the
 *    archiving APIs.
 *    An HIObject that is ignored for archiving will never be requested
 *    to encode itself into an archive. This is a statement that the
 *    HIObject does not handle the archiving protocol.
 *    By default, an HIObject *is* archiving ignored.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      The object whose archiving-ignored state you wish to change.
=======
 *      The object whose archiving ignored state you wish to change.
>>>>>>> graemeg/fixes_2_2
=======
 *      The object whose archiving ignored state you wish to change.
>>>>>>> origin/fixes_2_2
=======
 *      The object whose archiving-ignored state you wish to change.
>>>>>>> origin/fixes_2.4
 *    
 *    inIgnored:
 *      A Boolean value indicating whether or not to ignore the object.
 *  
 *  Result:
 *    An OSStatus signifying success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectSetArchivingIgnored( inObject: HIObjectRef; inIgnored: Boolean ): OSStatus; external name '_HIObjectSetArchivingIgnored';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
   Standard custom archive data dictionary keys describing custom initialize event parameters.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   The name and type keys denote an array of OSTypes represented by CFStrings. Use
   UTCreateStringForOSType and UTGetOSTypeFromString in UTType.h for CFStringRef <-> OSType
   conversion. The value key denotes an array of CFType objects.
}
{$endc} {not TARGET_CPU_64}

=======
   Each keyed value is an array of OSTypes represented by CFStrings. Use UTCreateStringForOSType
   and UTGetOSTypeFromString in UTType.h for CFStringRef <-> OSType conversion.
}
>>>>>>> graemeg/fixes_2_2
=======
   Each keyed value is an array of OSTypes represented by CFStrings. Use UTCreateStringForOSType
   and UTGetOSTypeFromString in UTType.h for CFStringRef <-> OSType conversion.
}
>>>>>>> origin/fixes_2_2
=======
   The name and type keys denote an array of OSTypes represented by CFStrings. Use
   UTCreateStringForOSType and UTGetOSTypeFromString in UTType.h for CFStringRef <-> OSType
   conversion. The value key denotes an array of CFType objects.
}
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{
 *  kHIObjectCustomDataParameterNamesKey
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectCustomDataParameterNamesKey: CFStringRef; external name '_kHIObjectCustomDataParameterNamesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kHIObjectCustomDataParameterTypesKey
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectCustomDataParameterTypesKey: CFStringRef; external name '_kHIObjectCustomDataParameterTypesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kHIObjectCustomDataParameterValuesKey
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectCustomDataParameterValuesKey: CFStringRef; external name '_kHIObjectCustomDataParameterValuesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
   Standard custom archive data dictionary keys defining a class and superclass for clients who do
   not implement the object's true class. Each keyed value is a CFStringRef based HIObject class ID.
}
{
 *  kHIObjectCustomDataClassIDKey
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectCustomDataClassIDKey: CFStringRef; external name '_kHIObjectCustomDataClassIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kHIObjectCustomDataSuperClassIDKey
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectCustomDataSuperClassIDKey: CFStringRef; external name '_kHIObjectCustomDataSuperClassIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
 *  kHIObjectCustomDataCDEFProcIDKey
 *  
 *  Discussion:
 *    Standard custom archive data dictionary key for ProcPointer based
 *    CDEFs. The key value is a CFString based SInt16. Use
 *    CFStringGetIntValue in CFString.h for CFStringRef <-> SInt16
 *    conversion.
 *  
=======
=======
>>>>>>> origin/fixes_2_2

{
   Standard custom archive data dictionary key for ProcPointer based CDEFs.  The key value is a
   CFString based SInt16. Use CFStringGetIntValue in CFString.h for CFStringRef <-> SInt16 conversion.
}
{
 *  kHIObjectCustomDataCDEFProcIDKey
 *  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  kHIObjectCustomDataCDEFProcIDKey
 *  
 *  Discussion:
 *    Standard custom archive data dictionary key for ProcPointer based
 *    CDEFs. The key value is a CFString based SInt16. Use
 *    CFStringGetIntValue in CFString.h for CFStringRef <-> SInt16
 *    conversion.
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectCustomDataCDEFProcIDKey: CFStringRef; external name '_kHIObjectCustomDataCDEFProcIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  kHIObjectCustomDataDelegateGroupParametersKey
 *  
 *  Discussion:
 *    Custom archive data dictionary key identifying custom archive
 *    data for subdelegates of a delegate group. The value for this key
 *    is a dictionary, whose keys are the class IDs of a delegate
 *    group's subdelegates, and whose values are custom archive data
 *    dictionaries for the respective subdelegate objects.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIObjectCustomDataDelegateGroupParametersKey: CFStringRef; external name '_kHIObjectCustomDataDelegateGroupParametersKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$ifc not TARGET_CPU_64}
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  HIObjectCopyCustomArchiveData()
 *  
 *  Discussion:
 *    Copies the custom archive data associated with an HIObject that
 *    has been read from or will be written to an archive. Useful for
 *    an archive editor that has read a custom object from an archive
 *    and would like to edit its custom data.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object whose custom archive data you wish to retrieve.
 *    
 *    outCustomData:
 *      On return, a CFDictionaryRef containing the custom data. The
 *      client is responsible for releasing the dictionary. NULL will
 *      be returned if there is no custom archive data available.
 *  
 *  Result:
 *    An OSStatus signifying success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectCopyCustomArchiveData( inObject: HIObjectRef; var outCustomData: CFDictionaryRef ): OSStatus; external name '_HIObjectCopyCustomArchiveData';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  HIObjectSetCustomArchiveData()
 *  
 *  Discussion:
 *    Retrieves the custom archive data associated with an HIObject
 *    that has been read from or will be written to an archive. Useful
 *    for an archive editor that has edited a custom object's custom
 *    data and would like it to be written to an archive with the
 *    object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object whose custom archive data you wish to change.
 *    
 *    inCustomData:
 *      A CFDictionaryRef containing the custom archive data you would
 *      like to associate with the object. Setting custom data will
 *      replace any existing custom data. Passing NULL will clear the
 *      custom archive data. The dictionary's keys and values must use
 *      CFType callbacks for archiving purposes.
 *  
 *  Result:
 *    An OSStatus signifying success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectSetCustomArchiveData( inObject: HIObjectRef; inCustomData: CFDictionaryRef { can be NULL } ): OSStatus; external name '_HIObjectSetCustomArchiveData';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{--------------------------------------------------------------------------------------}
{  • Delegate API constants                                                            }
{--------------------------------------------------------------------------------------}
{$endc} {not TARGET_CPU_64}


{
 *  HIDelegatePosition
 *  
 *  Summary:
 *    Positions in the list of delegates of an HIObject where a new
 *    delegate will be added.
 *  
 *  Discussion:
 *    When an event is sent to an HIObject, its delegates are also
 *    given the opportunity to handle the event. Delegates installed
 *    with kHIDelegateBefore will be called before the object receives
 *    the event. Delegates installed with kHIDelegateAfter will be
 *    called after the object receives the event, if the object's
 *    handler (and any delegates installed before the object) have all
 *    returned eventNotHandledErr. 
 *    
 *    If the event is sent with the SendToAllHandlers option, it will
 *    be delivered to every handler registered for it on each delegate
 *    and on the object, regardless of the error codes returned from
 *    event handlers on the delegates and the object. 
 *    
 *    If the event is not sent with the SendToAllHandlers option,
 *    normal event-processing rules apply. If the event handler of any
 *    delegate installed before the object returns an error code other
 *    than eventNotHandledErr, event processing will stop and neither
 *    the object nor any other delegate will receive the event. If the
 *    event reaches the object handler but the object handler returns
 *    an error code other than eventNotHandledErr, no delegate
 *    installed after the object will receive the event.
 }
type
	HIDelegatePosition = UInt32;
const
{
   * This constant may only be used with HIObjectRemoveDelegate. It
   * indicates that a delegate should be removed from all positions in
   * which it is installed. Passing this value to HIObjectAddDelegate
   * will cause an error to be returned.
   }
	kHIDelegateAll = 0;

  {
   * This delegate will be added at the top of the delegate list, and
   * will receive events sent to the object first. However, other
   * delegates added later with the "Before" position will be
   * positioned before this delegate.
   }
	kHIDelegateBefore = 1;

  {
   * This delegate will be added at the end of the delegate list, and
   * will receive events sent to the object last. However, other
   * delegates added later with the "After" position will be positioned
   * after this delegate. Also, delegates installed at this position
   * will never receive the kEventHIObjectDestruct event; once the
   * object on which the delegate is installed has handled the Destruct
   * event, all delegates have been removed from the object.
   }
	kHIDelegateAfter = 2;

{
 *  kHIDelegateBeforeKey
 *  
 *  Discussion:
 *    In a dictionary returned by HICopyDelegates, the key for an array
 *    of HIObjects installed before an object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIDelegateBeforeKey: CFStringRef; external name '_kHIDelegateBeforeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{
 *  kHIDelegateAfterKey
 *  
 *  Discussion:
 *    In a dictionary returned by HICopyDelegates, the key for an array
 *    of HIObjects installed after an object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kHIDelegateAfterKey: CFStringRef; external name '_kHIDelegateAfterKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{--------------------------------------------------------------------------------------}
{  • Delegate APIs                                                                     }
{--------------------------------------------------------------------------------------}
{
 *  HIObjectAddDelegate()
 *  
 *  Summary:
 *    Adds a delegate to an object at the specified position.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object to which to add the delegate.
 *    
 *    inDelegate:
 *      The delegate object. The delegate object will be retained by
 *      the owning object. If the owning object is destroyed, the
 *      delegate object will be automatically removed and released.
 *    
 *    inPosition:
 *      The position in the delegate list where this object should be
 *      added, either kHIDelegateBefore or kHIDelegateAfter. A delegate
 *      can be installed once for each possible delegate position.
 *  
 *  Result:
 *    An operating system result code. hiObjectDelegateAlreadyExistsErr
 *    is returned if the delegate already exists in the specified
 *    section of the delegate list.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIObjectAddDelegate( inObject: HIObjectRef; inDelegate: HIObjectRef; inPosition: HIDelegatePosition ): OSStatus; external name '_HIObjectAddDelegate';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIObjectRemoveDelegate()
 *  
 *  Summary:
 *    Removes a delegate from an object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object from which to remove the delegate.
 *    
 *    inDelegate:
 *      The delegate to remove. The delegate object will be released
 *      after it is removed from the owning object.
 *    
 *    inPosition:
 *      The position in the delegate list from which this object should
 *      be removed. A delegate may be installed on an object in
 *      multiple positions; removing the delegate from a specified
 *      position does not remove it from any other position. You may
 *      pass kHIDelegateAll to remove the delegate from all positions
 *      on the object.
 *  
 *  Result:
 *    An operating system result code. hiObjectDelegateNotFoundErr is
 *    returned if the delegate is not found in the specified section of
 *    the delegate list.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIObjectRemoveDelegate( inObject: HIObjectRef; inDelegate: HIObjectRef; inPosition: HIDelegatePosition ): OSStatus; external name '_HIObjectRemoveDelegate';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIObjectCopyDelegates()
 *  
 *  Summary:
 *    Returns a dictionary containing a list of the delegates attached
 *    to an object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inObject:
 *      The object whose delegates to retrieve.
 *    
 *    outDelegates:
 *      On exit, contains a dictionary describing the delegates
 *      attached to the object. The dictionary keys are
 *      kHIDelegateBeforeKey and kHIDelegateAfterKey, and the values
 *      are CFArrays. The arrays will contain the HIObjects installed
 *      at the corresponding positions in the delegate list, in order
 *      that they would be called to handle an event. If the specific
 *      object has no delegates installed in a particular position, the
 *      key and value for that position will be missing from the
 *      dictionary. The dictionary itself will always be created,
 *      however, even if the object has no delegates at all. The caller
 *      should release the dictionary but should not release the arrays.
 *  
 *  Result:
 *    An operating system result code. paramErr is returned if the
 *    object is invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIObjectCopyDelegates( inObject: HIObjectRef; var outDelegates: CFDictionaryRef ): OSStatus; external name '_HIObjectCopyDelegates';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIObjectGetEventHandlerObject()
 *  
 *  Summary:
 *    Given an EventHandlerCallRef for an invocation of an event
 *    handler, returns the HIObject that owns the event target that is
 *    currently handling the event.
 *  
 *  Discussion:
 *    This API is typically used by a delegate handler when handling an
 *    event that does not contain a parameter indicating the object to
 *    which the event is sent. For example, a delegate handler for a
 *    kEventTextDidChange event might want to know the HIViewRef of the
 *    text field object that changed; it can get this object by calling
 *    HIObjectGetEventHandlerObject while the delegate handler is
 *    executing. 
 *    
 *    Note that this API does not return the delegate object; it
 *    returns the object to which the delegate is attached. 
 *    
 *    This API may also be useful whenever you have an event handler
 *    installed on an object's event target and you need to know which
 *    object is currently handling the event, especially if the event
 *    does not have a DirectObject parameter or other identifier. For
 *    example, a kEventScrollableGetInfo handler could use this API to
 *    determine the object to which the GetInfo event was sent, without
 *    needing to store the object in the event handler's refcon.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRef:
 *      The EventHandlerCallRef that was passed to the event handler.
 *  
 *  Result:
 *    The HIObjectRef that owns the event target that is currently
 *    handling the event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIObjectGetEventHandlerObject( inRef: EventHandlerCallRef ): HIObjectRef; external name '_HIObjectGetEventHandlerObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • ControlRef                                                                                        }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{
   This is a forward declaration for ControlRef and HIViewRef so that other
   HIToolbox headers can use these types without causing recursive includes.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ControlRef = ^OpaqueControlRef; { an opaque type }
	OpaqueControlRef = record end;
=======
	ControlRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ControlRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ControlRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ControlRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	ControlRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	ControlRefPtr = ^ControlRef;
{ ControlHandle is obsolete. Use ControlRef.}
type
	ControlHandle = ControlRef;
	HIViewRef = ControlRef;

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

<<<<<<< HEAD
end.
{$endc} {not MACOSALLINCLUDE}
=======
end.
>>>>>>> graemeg/fixes_2_2
=======
end.
>>>>>>> origin/fixes_2_2
=======
end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
