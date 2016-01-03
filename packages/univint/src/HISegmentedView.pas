{
     File:       HIToolbox/HISegmentedView.h
 
     Contains:   API and type definitions related to HISegmentedView.
 
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
 
     Copyright:  © 2006-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit HISegmentedView;
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
uses MacTypes,HIGeometry,HIObject,HIView,CFBase,CGBase;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{==============================================================================}
{  HISegmentedView                                                             }
{  HISegmentedView is a new view available in Mac OS X 10.3 and later.         }
{  Examples of the segmented view are the Finder's icon/column/list view       }
{  switcher, and the back/forward buttons in Open panels.                      }
{  The segmented view operates as a group of buttons, each of which can be     }
{  configured with different behaviors and content.                            }
{  Accessibility Notes: Those of you who wish to customize the accessibility   }
{  information provided for individual segments of the segmented view -- by    }
{  handling various kEventClassAccessibility Carbon Events, by calling         }
{  HIObjectSetAuxiliaryAccessibilityAttribute, etc. -- need to know how to     }
{  interpret and/or build AXUIElementRefs that represent individual segments.  }
{  The AXUIElement representing an individual segment will/must be constructed }
{  using the segmented view's HIViewRef and the UInt64 identifier of the       }
{  one-based index of the segment the element refers to. As usual, a UInt64    }
{  identifier of zero represents the segmented view as a whole. You must       }
{  neither interpret nor create segmented view elements whose identifiers are  }
{  greater than the count of segments in the segmented view.                   }
{==============================================================================}
{ The HIObject class ID for the HISegmentedView class. }


{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHISegmentedViewClassID CFSTRP('com.apple.HISegmentedView')}
{$endc}
{ The HIViewKind for the HISegmentedView class. }
const
	kHISegmentedViewKind = FourCharCode('sgmt');

{$ifc not TARGET_CPU_64}
{
 *  HISegmentedViewCreate()
 *  
 *  Summary:
 *    Creates a segmented view. This is the type of view you would use
 *    to implement the icon/column/list view switcher as seen in the
 *    Finder. 
 *    
 *    After creating a segmented view, you need to set the number of
 *    segments via HISegmentedViewSetSegmentCount. Each segment can be
 *    configured independently using the other HISegmentedView APIs.
 *    
 *    
 *    Changing the number of segments and configuring each segment will
 *    change the appearance of the segmented view. After you configure
 *    the view, you may want to call HIViewGetOptimalBounds on the view
 *    and resize it so the content will fit optimally. 
 *    
 *    The value of the whole segmented view corresponds to the index of
 *    the currently selected segment with the radio behavior. If you
 *    set the value of the whole segmented view to n via
 *    HIViewSetValue, every radio-behavior segment will have its value
 *    set to zero except for the segment at index n; if segment n also
 *    has the radio behavior, it will have its value set to one. When a
 *    radio-behavior segment is clicked, the value of the whole
 *    segmented view will be set to the segment's index. 
 *    
 *    The segmented view works in both compositing and non-compositing
 *    modes.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
 *      The bounds of the view to be created. Can be NULL, in which
 *      case the view is created with CGRectZero bounds.
 *    
 *    outRef:
 *      A valid pointer to an HIViewRef. On successful completion of
 *      this routine, the destination HIViewRef will be filled with the
 *      HIViewRef of the newly created view. The view is initially
 *      invisible.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewCreate( {const} inBounds: HIRectPtr { can be NULL }; var outRef: HIViewRef ): OSStatus; external name '_HISegmentedViewCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewSetSegmentCount()
 *  
 *  Summary:
 *    Sets the number of segments for the segmented view. Any previous
 *    segments beyond the new count will have their content released.
 *    All new segments beyond the previous count be initialized with
 *    the most basic settings possible: momentary, no attributes, zero
 *    value, enabled, no command, no label, no content, and
 *    auto-calculated content width. You should configure any new
 *    segments to match your needs.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view for which the number of segments is to be
 *      set.
 *    
 *    inSegmentCount:
 *      A positive integer indicating how many segments the view is to
 *      have.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewSetSegmentCount( inSegmentedView: HIViewRef; inSegmentCount: UInt32 ): OSStatus; external name '_HISegmentedViewSetSegmentCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewGetSegmentCount()
 *  
 *  Summary:
 *    Gets the number of segments in the segmented view.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view for which the number of segments is to be
 *      retrieved.
 *  
 *  Result:
 *    A UInt32 indicating the number of segments in the segmented view.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewGetSegmentCount( inSegmentedView: HIViewRef ): UInt32; external name '_HISegmentedViewGetSegmentCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{$endc} {not TARGET_CPU_64}


{
 *  Summary:
 *    HISegmentBehavior constants
 }
const
{
   * Pops back up after being pressed, just like a push button.
   }
	kHISegmentBehaviorMomentary = 1;

  {
   * Stays pressed until another segment with the radio behavior is
   * pressed. This makes the segment behave like a radio button. After
   * this segment is clicked, the segmented view's value will be
   * changed to this segment's one-based index.
   }
	kHISegmentBehaviorRadio = 2;

  {
   * Like a check box. When clicked, it toggles back and forth between
   * checked and unchecked states. This behavior is not implemented in
   * any version of Mac OS X, and this constant is deprecated.
   }
	kHISegmentBehaviorToggles = 3;

  {
   * After being pressed, this type of segment stays pressed until you
   * programatically unpress it. This behavior is not implemented in
   * any version of Mac OS X, and this constant is deprecated.
   }
	kHISegmentBehaviorSticky = 4;

type
	HISegmentBehavior = UInt32;
{$ifc not TARGET_CPU_64}
{
 *  HISegmentedViewSetSegmentBehavior()
 *  
 *  Summary:
 *    Changes the behavior of an individual segment of a segmented
 *    view. By default, a segment has the kHISegmentBehaviorMomentary
 *    behavior.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment whose behavior you
 *      want to change.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment whose behavior you want to
 *      change. This must be a non-zero value that is no higher than
 *      the segmented view's current segment count.
 *    
 *    inBehavior:
 *      A HISegmentBehavior describing the behavior you wish the
 *      segment to have.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewSetSegmentBehavior( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32; inBehavior: HISegmentBehavior ): OSStatus; external name '_HISegmentedViewSetSegmentBehavior';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewGetSegmentBehavior()
 *  
 *  Summary:
 *    Returns the behavior of an individual segment of a segmented view.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment being queried.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment whose behavior you desire.
 *      This must be a non-zero value that is no higher than the
 *      segmented view's current segment count.
 *  
 *  Result:
 *    A HISegmentBehavior describing the behavior of the given segment.
 *    If you pass an illegal segment index, the result is undefined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewGetSegmentBehavior( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32 ): HISegmentBehavior; external name '_HISegmentedViewGetSegmentBehavior';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{$endc} {not TARGET_CPU_64}


{
 *  Summary:
 *    HISegmentedView segment attributes
 *  
 *  Discussion:
 *    These attribute bits are for use with
 *    HISegmentedViewChangeSegmentAttributes and
 *    HISegmentedViewGetSegmentAttributes.
 }
const
{
   * Pass this to indicate no attributes at all.
   }
	kHISegmentNoAttributes = 0;

  {
   * If this attribute bit is set, the command that gets sent out when
   * the segment is clicked will be directed at the user focus instead
   * of up the segmented view's containment hierarchy.
   }
	kHISegmentSendCmdToUserFocus = 1 shl 0;

{$ifc not TARGET_CPU_64}
{
 *  HISegmentedViewChangeSegmentAttributes()
 *  
 *  Summary:
 *    Changes the attributes of an individual segment of a segmented
 *    view. By default, a segment has no attribute bits set.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment whose attributes you
 *      want to change.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment whose attributes you want to
 *      change. This must be a non-zero value that is no higher than
 *      the segmented view's current segment count.
 *    
 *    inAttributesToSet:
 *      The attribute bits you wish to set for the segment.
 *    
 *    inAttributesToClear:
 *      The attribute bits you wish to clear for the segment.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewChangeSegmentAttributes( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32; inAttributesToSet: OptionBits; inAttributesToClear: OptionBits ): OSStatus; external name '_HISegmentedViewChangeSegmentAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewGetSegmentAttributes()
 *  
 *  Summary:
 *    Returns the attributes of an individual segment of a segmented
 *    view.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment being queried.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment whose attributes you desire.
 *      This must be a non-zero value that is no higher than the
 *      segmented view's current segment count.
 *  
 *  Result:
 *    The attribute bits that are set/enabled for the given segment. If
 *    you pass an illegal segment index, the result is undefined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewGetSegmentAttributes( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32 ): OptionBits; external name '_HISegmentedViewGetSegmentAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewSetSegmentValue()
 *  
 *  Summary:
 *    Changes the value of an individual segment of a segmented view.
 *    This is only meaningful for segments with the sticky, toggles, or
 *    radio behaviors. If you set the value of momentary segments to
 *    something other than zero, the behavior is undefined.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment whose value you want
 *      to change.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment whose value you want to
 *      change. This must be a non-zero value that is no higher than
 *      the segmented view's current segment count.
 *    
 *    inValue:
 *      The value you wish the segment to have. Zero means
 *      unpressed/unselected and one means pressed/selected. Other
 *      values will result in undefined behavior.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewSetSegmentValue( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32; inValue: SInt32 ): OSStatus; external name '_HISegmentedViewSetSegmentValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewGetSegmentValue()
 *  
 *  Summary:
 *    Determines the value of an individual segment of a segmented
 *    view. This is only meaningful for segments with the sticky,
 *    toggles, or radio behaviors. The value of a momentary segment is
 *    undefined.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment being queried.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment whose value you desire. This
 *      must be a non-zero value that is no higher than the segmented
 *      view's current segment count.
 *  
 *  Result:
 *    A SInt32 indicating the value of the given segment. If you pass
 *    an illegal segment index, the result is undefined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewGetSegmentValue( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32 ): SInt32; external name '_HISegmentedViewGetSegmentValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewSetSegmentEnabled()
 *  
 *  Summary:
 *    Enables or disables an individual segment of a segmented view.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment to enable or disable.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment to disable or enable. This
 *      must be a non-zero value that is no higher than the segmented
 *      view's current segment count.
 *    
 *    inEnabled:
 *      A Boolean indicating whether the segment is to become enabled
 *      or disabled.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewSetSegmentEnabled( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32; inEnabled: Boolean ): OSStatus; external name '_HISegmentedViewSetSegmentEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewIsSegmentEnabled()
 *  
 *  Summary:
 *    Tests an individual segment of a segmented view to see if it is
 *    enabled or disabled.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment being queried.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment to test. This must be a
 *      non-zero value that is no higher than the segmented view's
 *      current segment count.
 *  
 *  Result:
 *    True if the segment is enabled or false if the segment is
 *    disabled. If you pass an illegal segment index, the result is
 *    undefined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewIsSegmentEnabled( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32 ): Boolean; external name '_HISegmentedViewIsSegmentEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewSetSegmentCommand()
 *  
 *  Summary:
 *    Sets the command ID for the given segment. By default, the
 *    command is zero. If you set any non-zero command ID, the
 *    segmented view will send an HICommand whenever the segment is
 *    clicked. By default, the command is sent to the segmented view
 *    and up the containment hierarchy. You can request that the
 *    command start at the user focus instead by turning on the
 *    kHISegmentSendCmdToUserFocus attribute for the segment.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment whose command you wish
 *      to set.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment whose command you wish to
 *      set. This must be a non-zero value that is no higher than the
 *      segmented view's current segment count.
 *    
 *    inCommand:
 *      The command ID you wish to associate with the segment. A value
 *      of zero signifies "no command".
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewSetSegmentCommand( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32; inCommand: UInt32 ): OSStatus; external name '_HISegmentedViewSetSegmentCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewGetSegmentCommand()
 *  
 *  Summary:
 *    Gets the command ID associated with the given segment.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment being queried.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment to query. This must be a
 *      non-zero value that is no higher than the segmented view's
 *      current segment count.
 *  
 *  Result:
 *    Returns the command ID associated with the segment. If you pass
 *    an illegal segment index, the result is undefined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewGetSegmentCommand( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32 ): UInt32; external name '_HISegmentedViewGetSegmentCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewSetSegmentLabel()
 *  
 *  Summary:
 *    Sets the label string for the given segment. By default, a
 *    segment has no label string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment whose label you wish
 *      to set.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment whose label you wish to set.
 *      This must be a non-zero value that is no higher than the
 *      segmented view's current segment count.
 *    
 *    inLabel:
 *      A CFStringRef with the text of the label. The segmented view
 *      will copy the string passed in. You may pass NULL or an empty
 *      CFStringRef if you wish to eliminate the label from the segment.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewSetSegmentLabel( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32; inLabel: CFStringRef ): OSStatus; external name '_HISegmentedViewSetSegmentLabel';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewCopySegmentLabel()
 *  
 *  Summary:
 *    Gets the label associated with the given segment.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment being queried.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment to query. This must be a
 *      non-zero value that is no higher than the segmented view's
 *      current segment count.
 *    
 *    outLabel:
 *      On exit, outLabel will be a copy of the label associated with
 *      the segment; you must release this string. If there is no label
 *      associated with the segment, outLabel will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewCopySegmentLabel( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32; var outLabel: CFStringRef ): OSStatus; external name '_HISegmentedViewCopySegmentLabel';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewSetSegmentContentWidth()
 *  
 *  Summary:
 *    Sets whether you want the segment to automatically calculate its
 *    own width or whether you want to determine the segment's width
 *    manually. The content width is the horizontal area taken up by a
 *    segment's label and/or image.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment whose content width
 *      you wish to set.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment whose content width you wish
 *      to set. This must be a non-zero value that is no higher than
 *      the segmented view's current segment count.
 *    
 *    inAutoCalculateWidth:
 *      A Boolean indicating whether you want the segment to calculate
 *      its own width. If you pass true, the inWidth parameter is
 *      ignored.
 *    
 *    inWidth:
 *      If you passed false in inAutoCalculateWidth, this parameter
 *      specifies the width you want to manually associate with the
 *      segment. If you pass a negative width, the behavior is
 *      undefined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewSetSegmentContentWidth( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32; inAutoCalculateWidth: Boolean; inWidth: CGFloat ): OSStatus; external name '_HISegmentedViewSetSegmentContentWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewGetSegmentContentWidth()
 *  
 *  Summary:
 *    Gets the content width of the given segment. This also optionally
 *    passes back a Boolean indicating whether the width was
 *    automatically calculated. The content width is the horizontal
 *    area taken up by a segment's label and/or image.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment being queried.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment to query. This must be a
 *      non-zero value that is no higher than the segmented view's
 *      current segment count.
 *    
 *    outAutoCalculated:
 *      On exit, this is a Boolean indicating whether the width was
 *      automatically calculated. You may pass NULL if you don't need
 *      this information.
 *  
 *  Result:
 *    Returns the width of the content for the given segment. If you
 *    pass an illegal segment index, the result is undefined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewGetSegmentContentWidth( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32; outAutoCalculated: BooleanPtr ): CGFloat; external name '_HISegmentedViewGetSegmentContentWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewSetSegmentImage()
 *  
 *  Summary:
 *    Sets or clears the image associated with a given segment.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment whose image you wish
 *      to set.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment whose image you wish to set.
 *      This must be a non-zero value that is no higher than the
 *      segmented view's current segment count.
 *    
 *    inImage:
 *      An HIViewImageContentInfo structure with the image information
 *      for the given segment. Prior to Mac OS X 10.5, segments only
 *      support three types of image content: kHIViewContentNone (no
 *      image), kHIViewContentIconRef, and kHIViewContentCGImageRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewSetSegmentImage( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32; const (*var*) inImage: HIViewImageContentInfo ): OSStatus; external name '_HISegmentedViewSetSegmentImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewGetSegmentImageContentType()
 *  
 *  Summary:
 *    Gets the type of image content drawn by the given segment. You
 *    will need to call this before calling
 *    HISegmentedViewCopySegmentImage so you know what type of image
 *    content to request from the latter API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment being queried.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment to query. This must be a
 *      non-zero value that is no higher than the segmented view's
 *      current segment count.
 *  
 *  Result:
 *    Returns the image content type of the image drawn by the given
 *    segment. If you pass an illegal segment index, the result is
 *    undefined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewGetSegmentImageContentType( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32 ): HIViewImageContentType; external name '_HISegmentedViewGetSegmentImageContentType';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HISegmentedViewCopySegmentImage()
 *  
 *  Summary:
 *    Gives you a copy of the image (if any) drawn by the given
 *    segment. You are responsible for releasing any image passed back
 *    by this function. You request the image by asking for a
 *    particular type of image. If the segment isn't using the
 *    requested type of image, an error will be returned. If you wish
 *    to know the actual type of image displayed by the segment, you
 *    can call HISegmentedViewGetSegmentImageContentType.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSegmentedView:
 *      The segmented view that owns the segment being queried.
 *    
 *    inSegmentIndexOneBased:
 *      The one-based index of the segment to query. This must be a
 *      non-zero value that is no higher than the segmented view's
 *      current segment count.
 *    
 *    ioImage:
 *      On entry, you must fill out the contentType field of this
 *      structure with the type of image you desire. On exit, if that
 *      type of image is used by the segment, the appropriate field of
 *      the union will be filled in with a copy of the image. You are
 *      responsible for releasing the image.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HISegmentedViewCopySegmentImage( inSegmentedView: HIViewRef; inSegmentIndexOneBased: UInt32; var ioImage: HIViewImageContentInfo ): OSStatus; external name '_HISegmentedViewCopySegmentImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
