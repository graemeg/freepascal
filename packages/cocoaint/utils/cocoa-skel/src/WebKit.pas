unit WebKit;

{$mode delphi}
{$modeswitch cvar}
{$modeswitch objectivec1}
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

{$linkframework WebKit}

uses
  ctypes, CocoaAll,
  CFBase, CFArray, CFBag , CFCharacterSet, CFData, CFDate, CFDictionary, CFNumber ,CFPropertyList, CFSet, CFString, CFStringEncodingExt, CFTimeZone, CFTree, CFURL, CFXMLNode, CFXMLParser, CFMachPort, CFMessagePort, CFRunLoop, CFSocket, CFBinaryHeap, CFBitVector, CFBundle, CFByteOrders, CFPlugIn, CFPreferences, CFURLAccess, CFUUID, CFLocale, CFStream, CFDateFormatter, CFNumberFormatter, CFCalendar, CFUserNotification, CFNotificationCenter, CFAttributedString,
  CGBase, CGAffineTransforms, CGBitmapContext, CGColor, CGColorSpace, CGContext, CGDataConsumer, CGDataProvider, CGDirectDisplay, CGDirectPalette, CGDisplayConfiguration, CGDisplayFades, CGErrors, CGEvent, CGEventSource, CGEventTypes, CGFont, CGFunction, CGGLContext, CGGeometry, CGImage, CGLayer, CGPDFArray, CGPDFContentStream, CGPDFContext, CGPDFDictionary, CGPDFDocument, CGPDFObject, CGPDFOperatorTable, CGPDFPage, CGPDFScanner, CGPDFStream, CGPDFString, CGPSConverter, CGPath, CGPattern, CGRemoteOperation, CGSession, CGShading, CGWindowLevels,
  MacTypes,
  AnonClassDefinitionsWebkit;
  
{$define INTERFACE}

{$include webkit/UndefinedTypes.inc}
{$include webkit/AnonIncludeClassDefinitionsWebkit.inc}

{$define HEADER}
{$include webkit/WebKit.inc}
{$undef HEADER}

{$define TYPES}
{$include webkit/WebKit.inc}
{$undef TYPES}

{$define RECORDS}
{$include webkit/WebKit.inc}
{$undef RECORDS}

type
{$define FORWARD}
{$include webkit/WebKit.inc}
{$undef FORWARD}

{$define PROTOCOLS}
{$include webkit/WebKit.inc}
{$undef PROTOCOLS}

{$define CLASSES}
{$include webkit/WebKit.inc}
{$undef CLASSES}
 
{$define FUNCTIONS}
{$include webkit/WebKit.inc}
{$undef FUNCTIONS}

{$define EXTERNAL_SYMBOLS}
{$include webkit/WebKit.inc}
{$undef EXTERNAL_SYMBOLS}

{$define USER_PATCHES}
{$include webkit/WebKit.inc}
{$undef USER_PATCHES}

{$undef INTERFACE}
implementation
{$define IMPLEMENTATION}

{$define USER_PATCHES}
{$include webkit/WebKit.inc}
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
