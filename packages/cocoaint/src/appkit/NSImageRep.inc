{ Parsed from Appkit.framework NSImageRep.h }

{$ifdef TYPES}
{$ifndef NSIMAGEREP_PAS_T}
{$define NSIMAGEREP_PAS_T}

{ Constants }

const
  NSImageRepMatchesDevice = 0;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSIMAGEREP_PAS_R}
{$define NSIMAGEREP_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSIMAGEREP_PAS_F}
{$define NSIMAGEREP_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSIMAGEREP_PAS_S}
{$define NSIMAGEREP_PAS_S}

{ External string constants }
var
  NSImageRepRegistryDidChangeNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSImageRep = objcclass;
  NSImageRepPointer = ^NSImageRep;
  NSImageRepPtr = NSImageRepPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSIMAGEREP_PAS_C}
{$define NSIMAGEREP_PAS_C}

{ NSImageRep }
  NSImageRep = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __repFlags: record
      case byte of
       0: (_anonbitfield_NSImageRep0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __repFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSImageRep0: cuint);
       1: (
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
        _hasAlpha: 0..1;
        _isOpaque: 0..1;
        cacheParamsComputed: 0..1 deprecated;
        cacheAlphaComputed: 0..1 deprecated;
        loadState: 0..((1 shl 2)-1);
        keepCacheWindow: 0..1 deprecated;
        reserved: 0..1;
        _bitsPerSample: 0..((1 shl 8)-1);
        gsaved: 0..((1 shl 16)-1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      );
      end;
    _colorSpaceName: NSString;
    _size: NSSize;
    _pixelsWide: cint;
    _pixelsHigh: cint;
    
  public
    function draw: Boolean; message 'draw';
    function drawAtPoint(point: NSPoint): Boolean; message 'drawAtPoint:';
    function drawInRect(rect: NSRect): Boolean; message 'drawInRect:';
    function drawInRect_fromRect_operation_fraction_respectFlipped_hints(dstSpacePortionRect: NSRect; srcSpacePortionRect: NSRect; op: NSCompositingOperation; requestedAlpha: CGFloat; respectContextIsFlipped: Boolean; hints: NSDictionary): Boolean; message 'drawInRect:fromRect:operation:fraction:respectFlipped:hints:';
    procedure setSize(aSize: NSSize); message 'setSize:';
    function size: NSSize; message 'size';
    procedure setAlpha(flag: Boolean); message 'setAlpha:';
    function hasAlpha: Boolean; message 'hasAlpha';
    procedure setOpaque(flag: Boolean); message 'setOpaque:';
    function isOpaque: Boolean; message 'isOpaque';
    procedure setColorSpaceName(string_: NSString); message 'setColorSpaceName:';
    function colorSpaceName: NSString; message 'colorSpaceName';
    procedure setBitsPerSample(anInt: NSInteger); message 'setBitsPerSample:';
    function bitsPerSample: NSInteger; message 'bitsPerSample';
    procedure setPixelsWide(anInt: NSInteger); message 'setPixelsWide:';
    function pixelsWide: NSInteger; message 'pixelsWide';
    procedure setPixelsHigh(anInt: NSInteger); message 'setPixelsHigh:';
    function pixelsHigh: NSInteger; message 'pixelsHigh';
    class procedure registerImageRepClass(imageRepClass: Pobjc_class); message 'registerImageRepClass:';
    class procedure unregisterImageRepClass(imageRepClass: Pobjc_class); message 'unregisterImageRepClass:';
    class function registeredImageRepClasses: NSArray; message 'registeredImageRepClasses';
    class function imageRepClassForFileType(type_: NSString): Pobjc_class; message 'imageRepClassForFileType:';
    class function imageRepClassForPasteboardType(type_: NSString): Pobjc_class; message 'imageRepClassForPasteboardType:';
    class function imageRepClassForType(type_: NSString): Pobjc_class; message 'imageRepClassForType:';
    class function imageRepClassForData(data: NSData): Pobjc_class; message 'imageRepClassForData:';
    class function canInitWithData(data: NSData): Boolean; message 'canInitWithData:';
    class function imageUnfilteredFileTypes: NSArray; message 'imageUnfilteredFileTypes';
    class function imageUnfilteredPasteboardTypes: NSArray; message 'imageUnfilteredPasteboardTypes';
    class function imageFileTypes: NSArray; message 'imageFileTypes';
    class function imagePasteboardTypes: NSArray; message 'imagePasteboardTypes';
    class function imageUnfilteredTypes: NSArray; message 'imageUnfilteredTypes';
    class function imageTypes: NSArray; message 'imageTypes';
    class function canInitWithPasteboard(pasteboard: NSPasteboard): Boolean; message 'canInitWithPasteboard:';
    class function imageRepsWithContentsOfFile(filename: NSString): NSArray; message 'imageRepsWithContentsOfFile:';
    class function imageRepWithContentsOfFile(filename: NSString): id; message 'imageRepWithContentsOfFile:';
    class function imageRepsWithContentsOfURL(url: NSURL): NSArray; message 'imageRepsWithContentsOfURL:';
    class function imageRepWithContentsOfURL(url: NSURL): id; message 'imageRepWithContentsOfURL:';
    class function imageRepsWithPasteboard(pasteboard: NSPasteboard): NSArray; message 'imageRepsWithPasteboard:';
    class function imageRepWithPasteboard(pasteboard: NSPasteboard): id; message 'imageRepWithPasteboard:';
    function CGImageForProposedRect_context_hints(proposedDestRect: NSRectPtr; context: NSGraphicsContext; hints: NSDictionary): CGImageRef; message 'CGImageForProposedRect:context:hints:';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
