unit iPhoneAll;

{$mode delphi}
{$modeswitch objectivec1}
{$modeswitch cvar}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$packrecords c}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

interface

{$linkframework CoreFoundation}
{$linkframework CoreGraphics}
{$linkframework UIKit}

uses
  ctypes,
  CFBase,CFArray,CFBag,CFCharacterSet,CFData,CFDate,CFDictionary,CFNumber,CFPropertyList,CFSet,CFString,CFStringEncodingExt,CFTimeZone,CFTree,CFURL,CFXMLNode,CFXMLParser,CFMachPort,CFMessagePort,CFRunLoop,CFSocket,CFBinaryHeap,CFBitVector,CFBundle,CFByteOrders,CFPlugIn,CFPreferences,CFURLAccess,CFUUID,CFLocale,CFStream,CFDateFormatter,CFNumberFormatter,CFCalendar,CFUserNotification,CFNotificationCenter,CFAttributedString,CFNetworkErrorss,
  CGBase,CGAffineTransforms,CGBitmapContext,CGColor,CGColorSpace,CGContext,CGDataConsumer,CGDataProvider,CGDirectDisplay,CGDirectPalette,CGDisplayConfiguration,CGDisplayFades,CGErrors,CGEvent,CGEventSource,CGEventTypes,CGFont,CGFunction,CGGLContext,CGGeometry,CGImage,CGLayer,CGPDFArray,CGPDFContentStream,CGPDFContext,CGPDFDictionary,CGPDFDocument,CGPDFObject,CGPDFOperatorTable,CGPDFPage,CGPDFScanner,CGPDFStream,CGPDFString,CGPSConverter,CGPath,CGPattern,CGRemoteOperation,CGSession,CGShading,CGWindowLevels,
  MacTypes, SecBase, SecTrust,
  AnonClassDefinitionsUikit;

{ undefine to generate SDK 3.2 headers }
{$define IOS_SDK_4_2_OR_HIGHER}
  
{$define INTERFACE}

{$include UndefinedTypes.inc}

{$define HEADER}
{$include foundation/Foundation.inc}
{$include opengles/OpenGLES.inc}
{$include quartzcore/QuartzCore.inc}
{$include uikit/UIKit.inc}
{$undef HEADER}

{$define TYPES}
{$include foundation/Foundation.inc}
{$include opengles/OpenGLES.inc}
{$include quartzcore/QuartzCore.inc}
{$include uikit/UIKit.inc}
{$undef TYPES}

{$define RECORDS}
{$include foundation/Foundation.inc}
{$include opengles/OpenGLES.inc}
{$include quartzcore/QuartzCore.inc}
{$include uikit/UIKit.inc}
{$undef RECORDS}

type
{$define FORWARD}
{$include foundation/Foundation.inc}
{$include opengles/OpenGLES.inc}
{$include quartzcore/QuartzCore.inc}
{$include uikit/UIKit.inc}
{$undef FORWARD}

{$define PROTOCOLS}
{$include foundation/Foundation.inc}
{$include opengles/OpenGLES.inc}
{$include quartzcore/QuartzCore.inc}
{$include uikit/UIKit.inc}
{$undef PROTOCOLS}

{$define CLASSES}
{$include foundation/Foundation.inc}
{$include opengles/OpenGLES.inc}
{$include quartzcore/QuartzCore.inc}
{$include uikit/UIKit.inc}
{$undef CLASSES}
 
{$define FUNCTIONS}
{$include foundation/Foundation.inc}
{$include opengles/OpenGLES.inc}
{$include quartzcore/QuartzCore.inc}
{$include uikit/UIKit.inc}
{$undef FUNCTIONS}

{$define EXTERNAL_SYMBOLS}
{$include foundation/Foundation.inc}
{$include opengles/OpenGLES.inc}
{$include quartzcore/QuartzCore.inc}
{$include uikit/UIKit.inc}
{$undef EXTERNAL_SYMBOLS}

{$define USER_PATCHES}
{$include foundation/Foundation.inc}
{$include opengles/OpenGLES.inc}
{$include quartzcore/QuartzCore.inc}
{$include uikit/UIKit.inc}
{$undef USER_PATCHES}

{ Inline functions }
function NSSTR (inString: PChar): NSString;

{$undef INTERFACE}
implementation
{$define IMPLEMENTATION}

{$include InlineFunctions.inc}

{$define USER_PATCHES}
{$include foundation/Foundation.inc}
{$include opengles/OpenGLES.inc}
{$include quartzcore/QuartzCore.inc}
{$include uikit/UIKit.inc}
{$undef USER_PATCHES}

{$undef IMPLEMENTATION}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
end.
=======
end.
>>>>>>> graemeg/cpstrnew
=======
end.
>>>>>>> graemeg/cpstrnew
=======
end.
>>>>>>> graemeg/cpstrnew
=======
end.
>>>>>>> origin/cpstrnew
