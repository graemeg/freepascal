unit CocoaAll;

{$mode delphi}
{$modeswitch objectivec1}
{$modeswitch cvar} { for "external" after the semi-colon with external vars }
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

uses 
  ctypes,
  AEDataModel,IconsCore,
  CFBase, CFArray, CFBag , CFCharacterSet, CFData, CFDate, CFDictionary, CFNumber ,CFPropertyList, CFSet, CFString, CFStringEncodingExt, CFTimeZone, CFTree, CFURL, CFXMLNode, CFXMLParser, CFMachPort, CFMessagePort, CFRunLoop, CFSocket, CFBinaryHeap, CFBitVector, CFBundle, CFByteOrders, CFPlugIn, CFPreferences, CFURLAccess, CFUUID, CFLocale, CFStream, CFDateFormatter, CFNumberFormatter, CFCalendar, CFUserNotification, CFNotificationCenter, CFAttributedString, CFNetworkErrorss,
  CGBase, CGAffineTransforms, CGBitmapContext, CGColor, CGColorSpace, CGContext, CGDataConsumer, CGDataProvider, CGDirectDisplay, CGDirectPalette, CGDisplayConfiguration, CGDisplayFades, CGErrors, CGEvent, CGEventSource, CGEventTypes, CGFont, CGFunction, CGGLContext, CGGeometry, CGImage, CGLayer, CGPDFArray, CGPDFContentStream, CGPDFContext, CGPDFDictionary, CGPDFDocument, CGPDFObject, CGPDFOperatorTable, CGPDFPage, CGPDFScanner, CGPDFStream, CGPDFString, CGPSConverter, CGPath, CGPattern, CGRemoteOperation, CGSession, CGShading, CGWindowLevels,
  CVBase,CVImageBuffer,
  IOSurfaceAPI,
  SecBase, SecTrust,
  MacTypes, MacOSXPosix,
  macgl, CGLTypes,
  AnonClassDefinitionsQuartzcore;
  
{$linkframework Cocoa}
{$define INTERFACE}

{$include UndefinedTypes.inc}
// also includes the ones for foundation and appkit
{$include quartzcore/AnonIncludeClassDefinitionsQuartzcore.inc}

{$define HEADER}
{$include foundation/Foundation.inc}
{$include quartzcore/QuartzCore.inc}
{$include appkit/AppKit.inc}
{$undef HEADER}

{$define TYPES}
{$include foundation/Foundation.inc}
{$include quartzcore/QuartzCore.inc}
{$include appkit/AppKit.inc}
{$undef TYPES}

{$define RECORDS}
{$include foundation/Foundation.inc}
{$include quartzcore/QuartzCore.inc}
{$include appkit/AppKit.inc}
{$undef RECORDS}

type
{$define FORWARD}
{$include foundation/Foundation.inc}
{$include quartzcore/QuartzCore.inc}
{$include appkit/AppKit.inc}
{$undef FORWARD}

{$define PROTOCOLS}
{$include foundation/Foundation.inc}
{$include quartzcore/QuartzCore.inc}
{$include appkit/AppKit.inc}
{$undef PROTOCOLS}

{$define CLASSES}
{$include foundation/Foundation.inc}
{$include quartzcore/QuartzCore.inc}
{$include appkit/AppKit.inc}
{$undef CLASSES}
 
{$define FUNCTIONS}
{$include foundation/Foundation.inc}
{$include quartzcore/QuartzCore.inc}
{$include appkit/AppKit.inc}
{$undef FUNCTIONS}

{$define EXTERNAL_SYMBOLS}
{$include foundation/Foundation.inc}
{$include quartzcore/QuartzCore.inc}
{$include appkit/AppKit.inc}
{$undef EXTERNAL_SYMBOLS}

{$define USER_PATCHES}
{$include foundation/Foundation.inc}
{$include quartzcore/QuartzCore.inc}
{$include appkit/AppKit.inc}
{$undef USER_PATCHES}

{ Inline functions }
function NSSTR (inString: PChar): NSString;
function NSMakeRange (loc: NSUInteger; len: NSUInteger): NSRange;
function NSMaxRange (range: NSRange): NSUInteger;
function NSLocationInRange (loc: NSUInteger; range: NSRange): boolean;
function NSEqualRanges (range1, range2: NSRange): boolean;
function NSMakePoint (x: CGFloat; y: CGFloat): NSPoint;
function NSMakeSize(w: CGFloat; h: CGFloat): NSSize;
function NSMakeRect(x, y: CGFloat; w, h: CGFloat): NSRect;
function NSMaxX (aRect: NSRect): CGFloat;
function NSMaxY (aRect: NSRect): CGFloat;
function NSMidX (aRect: NSRect): CGFloat;
function NSMidY (aRect: NSRect): CGFloat;
function NSMinX (aRect: NSRect): CGFloat;
function NSMinY (aRect: NSRect): CGFloat;
function NSWidth (aRect: NSRect): CGFloat;
function NSHeight (aRect: NSRect): CGFloat;
function NSRectFromCGRect (aRect: CGRect): NSRect;
function NSRectToCGRect (aRect: NSRect): CGRect;
function NSPointFromCGPoint (aPoint: CGPoint): NSPoint;
function NSPointToCGPoint (aPoint: NSPoint): CGPoint;
function NSSizeFromCGSize(aSize: CGSize): NSSize;
function NSSizeToCGSize(aSize: NSSize): CGSize;

{$undef INTERFACE}

implementation
{$define IMPLEMENTATION}

{$include InlineFunctions.inc}

{$define USER_PATCHES}
{$include foundation/Foundation.inc}
{$include quartzcore/QuartzCore.inc}
{$include appkit/AppKit.inc}
{$undef USER_PATCHES}

{$undef IMPLEMENTATION}
end.
