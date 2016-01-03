{
     File:       HIToolbox/HIImageViews.h
 
     Contains:   API and type definitions related to views that display image content.
 
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
 
     Copyright:  © 1999-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> origin/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> origin/fixes_2.4
 
}
{       Initial Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
<<<<<<< HEAD
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

unit HIImageViews;
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
uses MacTypes,HIView,Controls,IconsCore,Icons,QuickdrawTypes,CGBase,CGImage,HIObject;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{==============================================================================}
{  HIImageView                                                                 }
{ The image view is a new view starting in Mac OS 10.2. It can only be used    }
{ in a compositing window. Like all new HIFoo views, this view is initially    }
{ invisible. You must show the view after creation.                            }
{==============================================================================}
{ The HIObject class ID for the HIImageView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIImageViewClassID CFSTRP('com.apple.HIImageView')}
{$endc}

const
{
   * The control kind of the image view
   }
	kControlKindHIImageView = FourCharCode('imag');

{$ifc not TARGET_CPU_64}
{
 *  HIImageViewCreate()
 *  
 *  Discussion:
 *    Creates an image view. The view responds to the scrollable
 *    interface and can be used in a scrolling view. You can pass an
 *    image initially, or set one later.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inImage:
 *      An initial image, or NULL. You can set the image later via
 *      SetControlData.
 *    
 *    outView:
 *      The new image view.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIImageViewCreate( inImage: CGImageRef { can be NULL }; var outView: HIViewRef ): OSStatus; external name '_HIImageViewCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIImageViewSetOpaque()
 *  
 *  Discussion:
 *    Allows you to set whether an image view should be treated as
 *    opaque. If this is set to true, the image view can make certain
 *    optimizations for compositing and scrolling. The alpha-related
 *    image view APIs are rendered useless if opacity it set to true.
 *    An image view, when created, is transparent by default.
 *    
 *    NOTE: In Mac OS X 10.2, this view was documented as being opaque
 *    by default, but the implementation did not enforce that. So in
 *    Mac OS X 10.3 and beyond, the view is transparent by default, and
 *    you can make it opaque by calling HIImageViewSetOpaque.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inView:
 *      The image view to affect.
 *    
 *    inOpaque:
 *      The new opacity setting. Pass true to indicate you want the
 *      image to be treated as opaque.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIImageViewSetOpaque( inView: HIViewRef; inOpaque: Boolean ): OSStatus; external name '_HIImageViewSetOpaque';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIImageViewIsOpaque()
 *  
 *  Discussion:
 *    Allows you to determine whether an image view is opaque or not.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inView:
 *      The image view to query.
 *  
 *  Result:
 *    A boolean result, where true indicates the image view is opaque.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIImageViewIsOpaque( inView: HIViewRef ): Boolean; external name '_HIImageViewIsOpaque';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIImageViewSetAlpha()
 *  
 *  Discussion:
 *    Allows you to set the alpha for an image, making it more or less
 *    transparent. An alpha of 1.0 is fully opaque, and 0.0 is fully
 *    transparent. The default alpha for an image is 1.0.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inView:
 *      The image view to affect.
 *    
 *    inAlpha:
 *      The new alpha value.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIImageViewSetAlpha( inView: HIViewRef; inAlpha: CGFloat ): OSStatus; external name '_HIImageViewSetAlpha';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIImageViewGetAlpha()
 *  
 *  Discussion:
 *    Allows you to get the alpha for an image. An alpha of 1.0 is
 *    fully opaque, and 0.0 is fully transparent.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inView:
 *      The image view to query.
 *  
 *  Result:
 *    A floating point number representing the alpha from 0.0 through
 *    1.0.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIImageViewGetAlpha( inView: HIViewRef ): CGFloat; external name '_HIImageViewGetAlpha';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIImageViewSetScaleToFit()
 *  
 *  Discussion:
 *    Normally an image view will clip to the view's bounds. Using this
 *    API, you can instead tell the image view to size the image to fit
 *    into the view bounds specified.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inView:
 *      The image view to affect.
 *    
 *    inScaleToFit:
 *      A boolean indicating whether the image should be scaled to fit
 *      the view bounds (true) or merely clip to the view bounds
 *      (false).
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIImageViewSetScaleToFit( inView: HIViewRef; inScaleToFit: Boolean ): OSStatus; external name '_HIImageViewSetScaleToFit';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIImageViewGetScaleToFit()
 *  
 *  Discussion:
 *    Returns whether or not an image view will scale the image it
 *    displays to the view bounds or merely clip to the view bounds. A
 *    true result means it scales.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inView:
 *      The image view to query.
 *  
 *  Result:
 *    A boolean result.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIImageViewGetScaleToFit( inView: HIViewRef ): Boolean; external name '_HIImageViewGetScaleToFit';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIImageViewSetImage()
 *  
 *  Discussion:
 *    Sets the image to display in an image view. The image passed in
 *    is retained by the view, so you may release the image after
 *    calling this API if you no longer need to reference it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inView:
 *      The image view to affect.
 *    
 *    inImage:
 *      The image to set.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIImageViewSetImage( inView: HIViewRef; inImage: CGImageRef { can be NULL } ): OSStatus; external name '_HIImageViewSetImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIImageViewCopyImage()
 *  
 *  Discussion:
 *    Gets the image for an image view. If there is no image set on the
 *    view, or the view ref is invalid, NULL is returned. The image is
 *    retained, so you should take care to release it when you are
 *    finished with it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inView:
 *      The image view to query.
 *  
 *  Result:
 *    A CoreGraphics (Quartz) image ref.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIImageViewCopyImage( inView: HIViewRef ): CGImageRef; external name '_HIImageViewCopyImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{$endc} {not TARGET_CPU_64}


{
 *  HIImageViewAutoTransformOptions
 *  
 *  Summary:
 *    These are the available options for applying automatic transforms
 *    on image views and can be accessed with the
 *    HIImageViewSetAutoTransform and HIImageViewGetAutoTransform APIs.
 *  
 *  Discussion:
 *    Currently, the auto transforms for disabled or inactive image
 *    views is similar to the transform applied to the image of an
 *    image well or the image in a bevel button when those controls are
 *    disabled or inactive.
 }
type
	HIImageViewAutoTransformOptions = UInt32;
const
{
   * No transform is applied to the image.
   }
	kHIImageViewAutoTransformNone = 0;

  {
   * Transforms are applied to the image view's image when the view is
   * disabled.
   }
	kHIImageViewAutoTransformOnDisable = 1 shl 0;

  {
   * Transforms are applied to the image view's image when the view is
   * deactivated.
   }
	kHIImageViewAutoTransformOnDeactivate = 1 shl 1;

{$ifc not TARGET_CPU_64}
{
 *  HIImageViewSetAutoTransform()
 *  
 *  Discussion:
 *    Sets how an image view will automatically transform its image.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inView:
 *      The image view for which to set the auto transform options.
 *    
 *    inOptions:
 *      A set of HIImageViewAutoTransformOptions bits.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIImageViewSetAutoTransform( inView: HIViewRef; inOptions: HIImageViewAutoTransformOptions ): OSStatus; external name '_HIImageViewSetAutoTransform';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIImageViewGetAutoTransform()
 *  
 *  Discussion:
 *    Gets how an image view is set to automatically transform its
 *    image.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inView:
 *      The image view to query.
 *  
 *  Result:
 *    A value of HIImageViewAutoTransformOptions bits. The return value
 *    is undefined if the HIViewRef is invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIImageViewGetAutoTransform( inView: HIViewRef ): HIImageViewAutoTransformOptions; external name '_HIImageViewGetAutoTransform';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{$endc} {not TARGET_CPU_64}


const
{
   * This tag is used with GetControlData and SetControlData to get or
   * set the CGImageRef associated with an HIImageView. This constant
   * is deprecated; use the HIImageViewCopyImage and
   * HIImageViewSetImage APIs to get or set the image for an image view.
   }
	kHIImageViewImageTag = FourCharCode('imag');

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ ICON CONTROL (CDEF 20)                                                            }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Value parameter should contain the ID of the ICON or cicn you wish to display when  }
{  creating controls of this type. If you don't want the control tracked at all, use   }
{  the 'no track' variant.                                                             }
{ Icon control proc IDs }
const
	kControlIconProc = 320;
	kControlIconNoTrackProc = 321;  { immediately returns kControlIconPart}
	kControlIconSuiteProc = 322;
	kControlIconSuiteNoTrackProc = 323;   { immediately returns kControlIconPart}

const
{ icon ref controls may have either an icon, color icon, icon suite, or icon ref.}
                                        { for data other than icon, you must set the data by passing a}
                                        { ControlButtonContentInfo to SetControlData}
	kControlIconRefProc = 324;
	kControlIconRefNoTrackProc = 325;   { immediately returns kControlIconPart}

{ Control Kind Tag }
const
	kControlKindIcon = FourCharCode('icon');

{ The HIObject class ID for the HIIconView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIIconViewClassID CFSTRP('com.apple.HIIconView')}
{$endc}
{$ifc not TARGET_CPU_64}
{
 *  CreateIconControl()
 *  
 *  Summary:
 *    Creates an Icon control at a specific position in the specified
 *    window.
 *  
 *  Discussion:
 *    Icon controls display an icon that (optionally) hilites when
 *    clicked on. On Mac OS X, a root control will be created for the
 *    window if one does not already exist. If a root control exists
 *    for the window, the Icon control will be embedded into it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inWindow:
 *      The WindowRef into which the Icon control will be created. May
 *      be NULL on 10.3 and later.
 *    
 *    inBoundsRect:
 *      The desired position (in coordinates local to the window's
 *      port) for the Icon control.
 *    
 *    inIconContent:
 *      The descriptor for the icon you want the control to display.
 *      Mac OS X and CarbonLib 1.5 (and beyond) support all of the icon
 *      content types. Prior to CarbonLib 1.5, the only content types
 *      that are properly respected are kControlContentIconSuiteRes,
 *      kControlContentCIconRes, and kControlContentICONRes.
 *    
 *    inDontTrack:
 *      A Boolean value indicating whether the control should hilite
 *      when it is clicked on. False means hilite and track the mouse.
 *    
 *    outControl:
 *      On successful output, outControl will contain a reference to
 *      the Icon control.
 *  
 *  Result:
 *    An OSStatus code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateIconControl( inWindow: WindowRef { can be NULL }; const (*var*) inBoundsRect: Rect; const (*var*) inIconContent: ControlButtonContentInfo; inDontTrack: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateIconControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by icon controls }
{$endc} {not TARGET_CPU_64}

const
	kControlIconTransformTag = FourCharCode('trfm'); { IconTransformType}
	kControlIconAlignmentTag = FourCharCode('algn'); { IconAlignmentType}

{ Tags available with appearance 1.1 or later }
const
	kControlIconResourceIDTag = FourCharCode('ires'); { SInt16 resource ID of icon to use}
	kControlIconContentTag = kControlContentTag; { ControlImageContentInfo}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ IMAGE WELL (CDEF 11)                                                              }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Image Wells allow you to control the content type (pict/icon/etc.) shown in the     }
{  well.                                                                               }
{  This is made possible by overloading the Min and Value parameters for the control.  }
{  Parameter                   What Goes Here                                          }
{  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ         ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ      }
{  Min                         content type (see constants for bevel buttons)          }
{  Value                       Resource ID of content type, if resource-based.         }
{  Handle-based Content                                                                }
{  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ                                                                }
{  You can create your control and then set the content to an existing handle to an    }
{  icon suite, etc. using the macros below. Please keep in mind that resource-based    }
{  content is owned by the control, handle-based content is owned by you. The CDEF will}
{  not try to dispose of handle-based content. If you are changing the content type of }
{  the button on the fly, you must make sure that if you are replacing a handle-       }
{  based content with a resource-based content to properly dispose of the handle,      }
{  else a memory leak will ensue.                                                      }
{ Image Well proc IDs }
const
	kControlImageWellProc = 176;

{ Control Kind Tag }
const
	kControlKindImageWell = FourCharCode('well');

{ The HIObject class ID for the HIImageWell class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIImageWellClassID CFSTRP('com.apple.HIImageWell')}
{$endc}
{ Creation API: Carbon only }
{$ifc not TARGET_CPU_64}
{
 *  CreateImageWellControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateImageWellControl( window: WindowRef; const (*var*) boundsRect: Rect; const (*var*) info: ControlButtonContentInfo; var outControl: ControlRef ): OSStatus; external name '_CreateImageWellControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by image wells }
{$endc} {not TARGET_CPU_64}

const
	kControlImageWellContentTag = kControlContentTag; { ControlImageContentInfo}
	kControlImageWellTransformTag = FourCharCode('tran'); { IconTransformType}
	kControlImageWellIsDragDestinationTag = FourCharCode('drag'); { Boolean}

{ Helper routines are available only thru the shared library/glue. }
{$ifc not TARGET_CPU_64}
{
 *  GetImageWellContentInfo()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetImageWellContentInfo( inButton: ControlRef; outContent: ControlButtonContentInfoPtr ): OSErr; external name '_GetImageWellContentInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetImageWellContentInfo()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetImageWellContentInfo( inButton: ControlRef; inContent: ControlButtonContentInfoPtr ): OSErr; external name '_SetImageWellContentInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetImageWellTransform()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetImageWellTransform( inButton: ControlRef; inTransform: IconTransformType ): OSErr; external name '_SetImageWellTransform';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
