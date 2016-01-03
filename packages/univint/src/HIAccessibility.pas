{
     File:       HIToolbox/HIAccessibility.h
 
     Contains:   Accessibility Carbon events and API for HIToolbox
 
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
=======
     Version:    HIToolbox-437~1
>>>>>>> origin/fixes_2.4
 
     Copyright:  © 2005-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{       Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
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
{       Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit HIAccessibility;
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
uses MacTypes,CFBase,CarbonEvents,Events,Menus,Controls,MacWindows,HIObject,AXUIElement;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{--------------------------------------------------------------------------------------}
{  Accessibility Events                                                                }
{--------------------------------------------------------------------------------------}
{
    kEventClassAccessibility quick reference:
    
    kEventAccessibleGetChildAtPoint                     =   1,
    kEventAccessibleGetFocusedChild                     =   2,
    
    kEventAccessibleGetAllAttributeNames                =   21,
    kEventAccessibleGetAllParameterizedAttributeNames   =   25,
    kEventAccessibleGetNamedAttribute                   =   22,
    kEventAccessibleSetNamedAttribute                   =   23,
    kEventAccessibleIsNamedAttributeSettable            =   24,
    
    kEventAccessibleGetAllActionNames                   =   41,
    kEventAccessiblePerformNamedAction                  =   42,
    kEventAccessibleGetNamedActionDescription           =   44
}
{
 *  kEventClassAccessibility / kEventAccessibleGetChildAtPoint
 *  
 *  Summary:
 *    Finds the child of an accessible object at a given point.
 *  
 *  Discussion:
 *    The kEventParamMouseLocation parameter will contain a global
 *    point. Your handler for this event should find the child of
 *    yourself which is underneath that point and return it in the
 *    kEventParamAccessibleChild parameter.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamAccessibleObject (in, typeCFTypeRef)
 *          The accessible object, in the form of an AXUIElementRef.
 *    
 *    --> kEventParamMouseLocation (in, typeHIPoint)
 *          The location in global coordinates.
 *    
 *    <-- kEventParamAccessibleChild (out, typeCFTypeRef)
 *          On exit, contains the child of the accessible object at the
 *          specified point, in the form of an AXUIElementRef. If there
 *          is no child at the given point, you should still return
 *          noErr, but leave the parameter empty (do not call
 *          SetEventParameter). Only return immediate children; do not
 *          return grandchildren of yourself.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventAccessibleGetChildAtPoint = 1;

{
 *  kEventClassAccessibility / kEventAccessibleGetFocusedChild
 *  
 *  Summary:
 *    Finds the focused child of an accessible object.
 *  
 *  Discussion:
 *    Your handler for this event should find the child of itself which
 *    is part of the focus chain and return it in the
 *    kEventParamAccessibleChild parameter.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamAccessibleObject (in, typeCFTypeRef)
 *          The accessible object, in the form of an AXUIElementRef.
 *    
 *    <-- kEventParamAccessibleChild (out, typeCFTypeRef)
 *          On exit, contains the focused child of the accessible
 *          object, in the form of an AXUIElementRef. If there is no
 *          child in the focus chain, you should still return noErr,
 *          but leave the parameter empty (do not call
 *          SetEventParameter). Only return immediate children; do not
 *          return grandchildren of yourself.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventAccessibleGetFocusedChild = 2;

{
 *  kEventClassAccessibility / kEventAccessibleGetAllAttributeNames
 *  
 *  Summary:
 *    Returns the attributes supported by an accessible object. You
 *    must only return the names of your regular (non-parameterized)
 *    attributes via this event. If you support parameterized
 *    attributes, you must return them via the new
 *    kEventAccessibleGetAllParameterizedAttributeNames event.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamAccessibleObject (in, typeCFTypeRef)
 *          The accessible object, in the form of an AXUIElementRef.
 *    
 *    <-> kEventParamAccessibleAttributeNames (in/out, typeCFMutableArrayRef)
 *          Add each of the regular (non-parameterized) attribute names
 *          supported by the accessible object to this array, as
 *          CFStringRefs.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventAccessibleGetAllAttributeNames = 21;

{
 *  kEventClassAccessibility / kEventAccessibleGetAllParameterizedAttributeNames
 *  
 *  Summary:
 *    Returns the parameterized attributes supported by an accessible
 *    object. You must not return the names of your regular
 *    (non-parameterized) attributes via this event. If you support
 *    regular attributes, you must return them via the original
 *    kEventAccessibleGetAllAttributeNames event. Parameterized
 *    attributes are introduced in Mac OS X 10.3.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamAccessibleObject (in, typeCFTypeRef)
 *          The accessible object, in the form of an AXUIElementRef.
 *    
 *    <-> kEventParamAccessibleAttributeNames (in/out, typeCFMutableArrayRef)
 *          Add each of the parameterized attribute names supported by
 *          the accessible object to this array, as CFStringRefs.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventAccessibleGetAllParameterizedAttributeNames = 25;

{
 *  kEventClassAccessibility / kEventAccessibleGetNamedAttribute
 *  
 *  Summary:
 *    Returns the value of an attribute of an accessible object.
 *  
 *  Discussion:
 *    The kEventParamAccessibleAttributeName parameter will contain an
 *    attribute name in the form of a CFStringRef. If you support the
 *    named attribute, return the attributeÕs value in the
 *    kEventParamAccessibleAttributeValue parameter.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamAccessibleObject (in, typeCFTypeRef)
 *          The accessible object, in the form of an AXUIElementRef.
 *    
 *    --> kEventParamAccessibleAttributeName (in, typeCFStringRef)
 *          The name of the requested attribute.
 *    
 *    --> kEventParamAccessibleAttributeParameter (in, typeCFTypeRef)
 *          This parameter is optional and will only exist if your
 *          accessible object is being asked for the value of a
 *          parameterized attribute. When present, this event parameter
 *          will contain a CFTypeRef describing the parameters of the
 *          request. Parameterized attributes are introduced in Mac OS
 *          X 10.3.
 *    
 *    <-- kEventParamAccessibleAttributeValue (out, typeCFTypeRef)
 *          On exit, contains the attribute's value. The type of this
 *          parameter varies according to the attribute; it might
 *          typically be typeCFStringRef (for a textual attribute),
 *          typeBoolean (for a boolean attribute), or typeSInt32 (for
 *          an integer-valued attribute).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventAccessibleGetNamedAttribute = 22;

{
 *  kEventClassAccessibility / kEventAccessibleSetNamedAttribute
 *  
 *  Summary:
 *    Sets the value of an attribute of an accessible object.
 *  
 *  Discussion:
 *    The kEventParamAccessibleAttributeName parameter will contain an
 *    attribute name in the form of a CFStringRef. The
 *    kEventParamAccessibleAttributeValue parameter will contain data
 *    in an arbitrary format. If you support the named attribute, set
 *    the named attributeÕs value to the data provided in the event.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamAccessibleObject (in, typeCFTypeRef)
 *          The accessible object, in the form of an AXUIElementRef.
 *    
 *    --> kEventParamAccessibleAttributeName (in, typeCFStringRef)
 *          The name of the requested attribute.
 *    
 *    --> kEventParamAccessibleAttributeValue (in, typeCFTypeRef)
 *          The new value of the attribute. The type of this parameter
 *          varies according to the attribute; it might typically be
 *          typeCFStringRef (for a textual attribute), typeBoolean (for
 *          a boolean attribute), or typeSInt32 (for an integer-valued
 *          attribute).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventAccessibleSetNamedAttribute = 23;

{
 *  kEventClassAccessibility / kEventAccessibleIsNamedAttributeSettable
 *  
 *  Summary:
 *    Determines whether an attribute of an accessible object can be
 *    modified.
 *  
 *  Discussion:
 *    The kEventParamAccessibleAttributeName parameter will contain an
 *    attribute name in the form of a CFStringRef. If you support the
 *    named attribute, set the kEventParamAccessibleAttributeSettable
 *    parameter to a Boolean indicating whether the named attribute can
 *    have its value changed via the kEventAccessibleSetNamedAttribute
 *    event.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamAccessibleObject (in, typeCFTypeRef)
 *          The accessible object, in the form of an AXUIElementRef.
 *    
 *    --> kEventParamAccessibleAttributeName (in, typeCFStringRef)
 *          The name of the requested attribute.
 *    
 *    <-- kEventParamAccessibleAttributeSettable (out, typeBoolean)
 *          On exit, indicates whether the attribute may be modified.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventAccessibleIsNamedAttributeSettable = 24;

{
 *  kEventClassAccessibility / kEventAccessibleGetAllActionNames
 *  
 *  Summary:
 *    Returns the actions supported by an accessible object.
 *  
 *  Discussion:
 *    The kEventParamAccessibleActionNames parameter will contain a
 *    CFMutableArrayRef. Add each of the action names you support to
 *    this array in the form of a CFStringRef.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamAccessibleObject (in, typeCFTypeRef)
 *          The accessible object, in the form of an AXUIElementRef.
 *    
 *    <-> kEventParamAccessibleActionNames (in/out, typeCFMutableArrayRef)
 *          Add each of the actions supported by the accessible object
 *          to this array, as CFStringRefs.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventAccessibleGetAllActionNames = 41;

{
 *  kEventClassAccessibility / kEventAccessiblePerformNamedAction
 *  
 *  Summary:
 *    Requests that a specific action be performed by an accessible
 *    object.
 *  
 *  Discussion:
 *    The kEventParamAccessibleActionName parameter will contain an
 *    attribute name in the form of a CFStringRef. If you support the
 *    named action, perform the action. There are times, however, when
 *    performing an action causes an accessible object to call a
 *    routine which may not return immediately, such as StandardAlert
 *    or PopUpMenuSelect. You should only call such routines when you
 *    receive an action request that was queued; if you call such a
 *    routine when processing an event that was directly dispatched,
 *    you will probably cause the assistive app to receive a timeout
 *    error. On Mac OS X 10.3 and later, the
 *    kEventParamAccessibilityEventQueued parameter will indicate
 *    whether the event was queued. If so, process the request
 *    normally. If not, you can request that the event be posted to the
 *    queue and sent to you later by returning
 *    eventDeferAccessibilityEventErr from your handler. On Mac OS X
 *    10.2, the parameter will not exist, the event will always be
 *    directly dispatched, and there is no way to request that it be
 *    posted to the queue; in this case, you should perform the action
 *    even if it will cause the assistive app to receive a timeout
 *    error.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamAccessibleObject (in, typeCFTypeRef)
 *          The accessible object, in the form of an AXUIElementRef.
 *    
 *    --> kEventParamAccessibleActionName (in, typeCFStringRef)
 *          The name of the requested action.
 *    
 *    --> kEventParamAccessibilityEventQueued (in, typeBoolean)
 *          An indication of whether this event was delivered to you
 *          via the main event queue. This parameter only exists on Mac
 *          OS X 10.3 and later. See the discussion for more details.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventAccessiblePerformNamedAction = 42;

{
 *  kEventClassAccessibility / kEventAccessibleGetNamedActionDescription
 *  
 *  Summary:
 *    Returns a human-language description of an action supported by an
 *    accessible object.
 *  
 *  Discussion:
 *    The kEventParamAccessibleActionName parameter will contain an
 *    attribute name in the form of a CFStringRef. The
 *    kEventParamAccessibleActionDescription parameter will contain a
 *    CFMutableStringRef. If you support the named action, alter the
 *    mutable string to contain a textual description of the actionÕs
 *    significance.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamAccessibleObject (in, typeCFTypeRef)
 *          The accessible object, in the form of an AXUIElementRef.
 *    
 *    --> kEventParamAccessibleActionName (in, typeCFStringRef)
 *          The name of the requested action.
 *    
 *    <-> kEventParamAccessibleActionDescription (in/out, typeCFMutableStringRef)
 *          If you support the action, extract this parameter from the
 *          event and set the contents of the mutable string to contain
 *          a description of the action. Do not set this event
 *          parameter to a CFStringRef of your own creation; you must
 *          modify the preexisting mutable string stored in the event
 *          parameter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventAccessibleGetNamedActionDescription = 44;

{$ifc not TARGET_CPU_64}
{
 *  AXUIElementCreateWithHIObjectAndIdentifier()
 *  
 *  Discussion:
 *    This routine creates an AXUIElementRef to represent an accessible
 *    object for a Carbon application. A Carbon accessible object is
 *    comprised of an HIObjectRef and a 64-bit identifier. The
 *    resulting AXUIElementRef is a CFTypeRef, and must be managed as
 *    such. You can create a new AXUIElementRef every time you need
 *    one. Even though the actual hex values of two AXUIElementRefs
 *    might be different, they may represent the same accessible
 *    object; because AXUIElementRefs are Core Foundation objects, you
 *    can use CFEqual to compare them.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inHIObject:
 *      The HIObjectRef of the accessible object.
 *    
 *    inIdentifier:
 *      The 64-bit identifier of the accessible object.
 *  
 *  Result:
 *    An AXUIElementRef that represents the Carbon accessible object
 *    identified by the given HIObjectRef and 64-bit identifier. This
 *    follows CoreFoundation semantics in that it will return NULL for
 *    failure, and because it is a "Create" function you will need to
 *    CFRelease() this AXUIElementRef when it is no longer needed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function AXUIElementCreateWithHIObjectAndIdentifier( inHIObject: HIObjectRef; inIdentifier: UInt64 ): AXUIElementRef; external name '_AXUIElementCreateWithHIObjectAndIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  AXUIElementGetHIObject()
 *  
 *  Discussion:
 *    If the incoming AXUIElementRef is a Carbon accessible object,
 *    this routine will return the HIObjectRef of the accessible object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inUIElement:
 *      The AXUIElementRef of whom you'd like to get the HIObjectRef.
 *  
 *  Result:
 *    The HIObjectRef of the AXUIElementRef. If the incoming
 *    AXUIElementRef is not a Carbon accessible object, this routine
 *    will return NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function AXUIElementGetHIObject( inUIElement: AXUIElementRef ): HIObjectRef; external name '_AXUIElementGetHIObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  AXUIElementGetIdentifier()
 *  
 *  Discussion:
 *    If the incoming AXUIElementRef is a Carbon accessible object,
 *    this routine will pass back the 64-bit identifier of the
 *    accessible object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inUIElement:
 *      The AXUIElementRef of whom you'd like to get the 64-bit
 *      identifier.
 *    
 *    outIdentifier:
 *      The 64-bit identifier of the AXUIElementRef. If the incoming
 *      AXUIElementRef is not a Carbon accessible object, this routine
 *      will pass back zero. Note that zero is often a legal value for
 *      Carbon accessible object, so do not assume that the accessible
 *      object is not a Carbon accessible object just because you get a
 *      result of zero.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
procedure AXUIElementGetIdentifier( inUIElement: AXUIElementRef; var outIdentifier: UInt64 ); external name '_AXUIElementGetIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  AXNotificationHIObjectNotify()
 *  
 *  Discussion:
 *    Posts a notification for the given pseudo-AXUIElementRef. Though
 *    an actual AXUIElementRef is not actually passed in to this
 *    function, its component parts are. This saves the implementation
 *    the hassle of dismantling the AXUIElementRef into its component
 *    parts.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inNotification:
 *      The notification name string.
 *    
 *    inHIObject:
 *      The HIObjectRef component of the AXUIElementRef to whom the
 *      notification applies.
 *    
 *    inIdentifier:
 *      The 64-bit identifier component of the AXUIElementRef to whom
 *      the notification applies.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
procedure AXNotificationHIObjectNotify( inNotification: CFStringRef; inHIObject: HIObjectRef; inIdentifier: UInt64 ); external name '_AXNotificationHIObjectNotify';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HICopyAccessibilityRoleDescription()
 *  
 *  Summary:
 *    Returns the role description string for a standard role or a
 *    standard role-subrole pair.
 *  
 *  Discussion:
 *    This routine is useful if you are implementing an accessible
 *    object that has a standard role or role-subrole pair and you want
 *    to make sure your object provides the same role description
 *    string that the equivalent system-supplied object provides. In
 *    other words, if you are implementing an accessible object of role
 *    kAXButtonRole, you can use this routine to make sure it provides
 *    the same role description that the standard push button provides.
 *    
 *    This routine can provide role description strings for all roles
 *    and role-subrole pairs that are used in the standard/system
 *    accessible objects on Mac OS X 10.4 and later. Once this routine
 *    is able to provide a role description string for a role or
 *    role-subrole pair, it will continue to do so on subsequent system
 *    releases, even if the system no longer produces a standard
 *    accessible object with that role or role-subrole pair.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRole:
 *      The role CFStringRef for your accessible object. Callers
 *      typically pass one of the kAXFooRole constant strings from
 *      within the HIServices framework.
 *    
 *    inSubrole:
 *      The subrole CFStringRef for your accessible object. Callers
 *      typically pass one of the kAXFooSubrole constant strings from
 *      within the HIServices framework. Pass NULL if your accessible
 *      object does not have a subrole.
 *  
 *  Result:
 *    A CFStringRef with the standard role description for the role or
 *    role-subrole pair. You must release the role description when you
 *    are finished with it. If there is no standard role description
 *    for the role or role-subrole pair, this routine will return NULL.
 *    If you pass either an unknown role or an unknown subrole, this
 *    routine will return NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HICopyAccessibilityRoleDescription( inRole: CFStringRef; inSubrole: CFStringRef { can be NULL } ): CFStringRef; external name '_HICopyAccessibilityRoleDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  HICopyAccessibilityActionDescription()
 *  
 *  Summary:
 *    Returns the action description string for a standard
 *    accessibility action.
 *  
 *  Discussion:
 *    This routine is useful if you are implementing an accessible
 *    object that implements a standard action and you want to make
 *    sure your object provides the same role action string that the a
 *    system-supplied object provides. 
 *    This routine can provide action description strings for all
 *    actions that are used in the standard/system accessible objects
 *    on Mac OS X 10.4 and later. Once this routine is able to provide
 *    a description string for an action, it will continue to do so on
 *    subsequent system releases, even if the system no longer produces
 *    a standard accessible object that supports the action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inAction:
 *      The action CFStringRef for which you'd like to generate a
 *      description. Callers must pass one of the kAXFooAction constant
 *      strings from within the HIServices framework.
 *  
 *  Result:
 *    A CFStringRef with the standard description for the action. You
 *    must release the description when you are finished with it. If
 *    you pass an unsupported action to this routine, the behavior is
 *    undefined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HICopyAccessibilityActionDescription( inAction: CFStringRef ): CFStringRef; external name '_HICopyAccessibilityActionDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
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
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
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
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
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
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
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
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectSetAuxiliaryAccessibilityAttribute( inHIObject: HIObjectRef; inIdentifier: UInt64; inAttributeName: CFStringRef; inAttributeData: CFTypeRef { can be NULL } ): OSStatus; external name '_HIObjectSetAuxiliaryAccessibilityAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  HIObjectOverrideAccessibilityContainment()
 *  
 *  Summary:
 *    Allows you to override the AXUIElementRefs that an HIObject would
 *    normally supply as the values of its AXParent, AXWindow, and
 *    AXTopLevelUIElement attributes.
 *  
 *  Discussion:
 *    This routine allows you to change the parent that an HIObject
 *    would normally supply in the accessibility hierarchy. For
 *    instance, a popup control could call this routine on its menu so
 *    that the menu returned the popup control as the menu's parent;
 *    normally the menu would return the application as its parent.
 *    Optionally, this routine also allows you to change the window and
 *    top-level element that an HIObject would normally supply. 
 *     
 *    If the input HIObject is a standard toolbox construct like an
 *    HIView or a Menu, the input HIObject will not be added as an
 *    accessibility child of its normal parent. In all other cases, it
 *    is the client's responsibility to ensure that the input HIObject
 *    is not added as an accessibility child of its normal parent. 
 *     
 *    If the desired AXUIElementRef parent represents an HIView, a
 *    Menu, or a Window, the input HIObject will be automatically added
 *    as an accessibility child of the specified parent. In all other
 *    cases, it is the client's responsibility to manually add the
 *    input HIObject as a child of the specified parent. To represent
 *    an HIView, a Menu, or a Window, an AXUIElementRef must contain
 *    the appropriate HIObjectRef as well as an identifier value of
 *    zero. 
 *    
 *    Similar rules don't have to apply for the handling of the window
 *    and top-level element attributes because those attributes don't
 *    represent two-way relationships. 
 *    
 *    A containment override is not necessarily supported by every type
 *    of HIObject. Currently, it is supported by HIViews, Menus, and
 *    Windows.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inHIObject:
 *      The HIObjectRef whose parent attribute you want to override.
 *    
 *    inDesiredParent:
 *      The AXUIElementRef that you wish the HIObject to return as the
 *      value of its AXParent attribute. This routine makes a copy of
 *      the AXUIElementRef; you must release inDesiredParent after you
 *      have called this routine. Passing NULL indicates that you want
 *      the HIObject to revert to its normal, un-overridden behavior.
 *    
 *    inDesiredWindow:
 *      The AXUIElementRef that you wish the HIObject to return as the
 *      value of its AXWindow attribute. This routine makes a copy of
 *      the AXUIElementRef; you must release inDesiredWindow after you
 *      have called this routine. Passing NULL indicates that you want
 *      the HIObject to report its normal window, if any.
 *    
 *    inDesiredTopLevelUIElement:
 *      The AXUIElementRef that you wish the HIObject to return as the
 *      value of its AXTopLevelUIElement attribute. This routine makes
 *      a copy of the AXUIElementRef; you must release
 *      inDesiredTopLevelUIElement after you have called this routine.
 *      Passing NULL indicates that you want the HIObject to report its
 *      normal top-level element, if any.
 *  
 *  Result:
 *    An OSStatus result code. If the HIObjectRef is invalid, this
 *    routine will return paramErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIObjectOverrideAccessibilityContainment( inHIObject: HIObjectRef; inDesiredParent: AXUIElementRef; inDesiredWindow: AXUIElementRef; inDesiredTopLevelUIElement: AXUIElementRef ): OSStatus; external name '_HIObjectOverrideAccessibilityContainment';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
