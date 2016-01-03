{ Parsed from Appkit.framework NSBitmapImageRep.h }

{$ifdef TYPES}
{$ifndef NSBITMAPIMAGEREP_PAS_T}
{$define NSBITMAPIMAGEREP_PAS_T}

{ Constants }

const
  NSTIFFCompressionNone = 1;
  NSTIFFCompressionCCITTFAX3 = 3;
  NSTIFFCompressionCCITTFAX4 = 4;
  NSTIFFCompressionLZW = 5;
  NSTIFFCompressionJPEG = 6;
  NSTIFFCompressionNEXT = 32766;
  NSTIFFCompressionPackBits = 32773;
  NSTIFFCompressionOldJPEG = 32865;

const
  NSTIFFFileType = 0;
  NSBMPFileType = 1;
  NSGIFFileType = 2;
  NSJPEGFileType = 3;
  NSPNGFileType = 4;
  NSJPEG2000FileType = 5;

const
  NSImageRepLoadStatusUnknownType = -1;
  NSImageRepLoadStatusReadingHeader = -2;
  NSImageRepLoadStatusWillNeedAllData = -3;
  NSImageRepLoadStatusInvalidData = -4;
  NSImageRepLoadStatusUnexpectedEOF = -5;
  NSImageRepLoadStatusCompleted = -6;

const
  NSAlphaFirstBitmapFormat = 1 shl 0;
  NSAlphaNonpremultipliedBitmapFormat = 1 shl 1;
  NSFloatingPointSamplesBitmapFormat = 1 shl 2;

{ Types }
type
  NSTIFFCompression = NSUInteger;
  NSTIFFCompressionPtr = ^NSTIFFCompression;
  NSBitmapImageFileType = NSUInteger;
  NSBitmapImageFileTypePtr = ^NSBitmapImageFileType;
  NSImageRepLoadStatus = NSInteger;
  NSImageRepLoadStatusPtr = ^NSImageRepLoadStatus;
  NSBitmapFormat = NSUInteger;
  NSBitmapFormatPtr = ^NSBitmapFormat;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSBITMAPIMAGEREP_PAS_R}
{$define NSBITMAPIMAGEREP_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSBITMAPIMAGEREP_PAS_F}
{$define NSBITMAPIMAGEREP_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSBITMAPIMAGEREP_PAS_S}
{$define NSBITMAPIMAGEREP_PAS_S}

{ External string constants }
var
  NSImageCompressionMethod: NSString; cvar; external;
  NSImageCompressionFactor: NSString; cvar; external;
  NSImageDitherTransparency: NSString; cvar; external;
  NSImageRGBColorTable: NSString; cvar; external;
  NSImageInterlaced: NSString; cvar; external;
  NSImageColorSyncProfileData: NSString; cvar; external;
  NSImageFrameCount: NSString; cvar; external;
  NSImageCurrentFrame: NSString; cvar; external;
  NSImageCurrentFrameDuration: NSString; cvar; external;
  NSImageLoopCount: NSString; cvar; external;
  NSImageGamma: NSString; cvar; external;
  NSImageProgressive: NSString; cvar; external;
  NSImageEXIFData: NSString; cvar; external;
  NSImageFallbackBackgroundColor: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSBitmapImageRep = objcclass;
  NSBitmapImageRepPointer = ^NSBitmapImageRep;
  NSBitmapImageRepPtr = NSBitmapImageRepPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSBITMAPIMAGEREP_PAS_C}
{$define NSBITMAPIMAGEREP_PAS_C}

{ NSBitmapImageRep }
  NSBitmapImageRep = objcclass external (NSImageRep)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __moreRepFlags: record
      case byte of
       0: (_anonbitfield_NSBitmapImageRep0: cuint);
       1: (data: bitpacked record
        _bitsPerPixel: 0..((1 shl 8)-1);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __moreRepFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSBitmapImageRep0: cuint);
       1: (
        _bitsPerPixel: cuint;
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
        _isPlanar: 0..1;
        explicitPlanes: 0..1;
        imageSourceIsIndexed: 0..1;
        dataLoaded: 0..1;
        colorModel: 0..((1 shl 4)-1);
        tierTwoInfoIsLoaded: 0..1;
        respectO: 0..1;
        compressionFactor: 0..((1 shl 14)-1);
        imageNumber: 0..((1 shl 8)-1);
        _bitmapFormat: 0..((1 shl 3)-1);
        cgImageIsPrimary: 0..1;
        compression: 0..((1 shl 20)-1);
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
    _bytesPerRow: cuint;
    _dataObj: id; {garbage collector: volatile }
    _tiffData: NSData;
    _properties: id;
    
  public
    function initWithFocusedViewRect(rect: NSRect): id; message 'initWithFocusedViewRect:';
    function initWithBitmapDataPlanes_pixelsWide_pixelsHigh_bitsPerSample_samplesPerPixel_hasAlpha_isPlanar_colorSpaceName_bytesPerRow_bitsPerPixel(planes: PChar; width: NSInteger; height: NSInteger; bps: NSInteger; spp: NSInteger; alpha: Boolean; isPlanar_: Boolean; colorSpaceName_: NSString; rBytes: NSInteger; pBits: NSInteger): id; message 'initWithBitmapDataPlanes:pixelsWide:pixelsHigh:bitsPerSample:samplesPerPixel:hasAlpha:isPlanar:colorSpaceName:bytesPerRow:bitsPerPixel:';
    function initWithBitmapDataPlanes_pixelsWide_pixelsHigh_bitsPerSample_samplesPerPixel_hasAlpha_isPlanar_colorSpaceName_bitmapFormat_bytesPerRow_bitsPerPixel(planes: PChar; width: NSInteger; height: NSInteger; bps: NSInteger; spp: NSInteger; alpha: Boolean; isPlanar_: Boolean; colorSpaceName_: NSString; bitmapFormat_: NSBitmapFormat; rBytes: NSInteger; pBits: NSInteger): id; message 'initWithBitmapDataPlanes:pixelsWide:pixelsHigh:bitsPerSample:samplesPerPixel:hasAlpha:isPlanar:colorSpaceName:bitmapFormat:bytesPerRow:bitsPerPixel:';
    function initWithCGImage(cgImage_: CGImageRef): id; message 'initWithCGImage:';
    function initWithCIImage(ciImage_: CIImage): id; message 'initWithCIImage:';
    class function imageRepsWithData(data: NSData): NSArray; message 'imageRepsWithData:';
    class function imageRepWithData(data: NSData): id; message 'imageRepWithData:';
    function initWithData(data: NSData): id; message 'initWithData:';
    function bitmapData: PChar; message 'bitmapData';
    procedure getBitmapDataPlanes(data: PChar); message 'getBitmapDataPlanes:';
    function isPlanar: Boolean; message 'isPlanar';
    function samplesPerPixel: NSInteger; message 'samplesPerPixel';
    function bitsPerPixel: NSInteger; message 'bitsPerPixel';
    function bytesPerRow: NSInteger; message 'bytesPerRow';
    function bytesPerPlane: NSInteger; message 'bytesPerPlane';
    function numberOfPlanes: NSInteger; message 'numberOfPlanes';
    function bitmapFormat: NSBitmapFormat; message 'bitmapFormat';
    procedure getCompression_factor(compression: NSTIFFCompressionPtr; factor: psingle); message 'getCompression:factor:';
    procedure setCompression_factor(compression: NSTIFFCompression; factor: single); message 'setCompression:factor:';
    function TIFFRepresentation: NSData; message 'TIFFRepresentation';
    function TIFFRepresentationUsingCompression_factor(comp: NSTIFFCompression; factor: single): NSData; message 'TIFFRepresentationUsingCompression:factor:';
    class function TIFFRepresentationOfImageRepsInArray(array_: NSArray): NSData; message 'TIFFRepresentationOfImageRepsInArray:';
    class function TIFFRepresentationOfImageRepsInArray_usingCompression_factor(array_: NSArray; comp: NSTIFFCompression; factor: single): NSData; message 'TIFFRepresentationOfImageRepsInArray:usingCompression:factor:';
    class procedure getTIFFCompressionTypes_count(list: NSTIFFCompressionPtr; numTypes: NSIntegerPtr); message 'getTIFFCompressionTypes:count:';
    class function localizedNameForTIFFCompressionType(compression: NSTIFFCompression): NSString; message 'localizedNameForTIFFCompressionType:';
    function canBeCompressedUsing(compression: NSTIFFCompression): Boolean; message 'canBeCompressedUsing:';
    procedure colorizeByMappingGray_toColor_blackMapping_whiteMapping(midPoint: CGFloat; midPointColor: NSColor; shadowColor: NSColor; lightColor: NSColor); message 'colorizeByMappingGray:toColor:blackMapping:whiteMapping:';
    function initForIncrementalLoad: id; message 'initForIncrementalLoad';
    function incrementalLoadFromData_complete(data: NSData; complete: Boolean): NSInteger; message 'incrementalLoadFromData:complete:';
    procedure setColor_atX_y(color: NSColor; x: NSInteger; y: NSInteger); message 'setColor:atX:y:';
    function colorAtX_y(x: NSInteger; y: NSInteger): NSColor; message 'colorAtX:y:';
    procedure getPixel_atX_y(p: NSUInteger; x: NSInteger; y: NSInteger); message 'getPixel:atX:y:';
    procedure setPixel_atX_y(p: NSUInteger; x: NSInteger; y: NSInteger); message 'setPixel:atX:y:';
    function CGImage: CGImageRef; message 'CGImage';
    function colorSpace: NSColorSpace; message 'colorSpace';
    function bitmapImageRepByConvertingToColorSpace_renderingIntent(targetSpace: NSColorSpace; renderingIntent: NSColorRenderingIntent): NSBitmapImageRep; message 'bitmapImageRepByConvertingToColorSpace:renderingIntent:';
    function bitmapImageRepByRetaggingWithColorSpace(newSpace: NSColorSpace): NSBitmapImageRep; message 'bitmapImageRepByRetaggingWithColorSpace:';
  end;

{ NSBitmapImageFileTypeExtensionsCategory }
  NSBitmapImageFileTypeExtensionsCategory = objccategory external (NSBitmapImageRep)
    class function representationOfImageRepsInArray_usingType_properties(imageReps: NSArray; storageType: NSBitmapImageFileType; properties: NSDictionary): NSData; message 'representationOfImageRepsInArray:usingType:properties:';
    function representationUsingType_properties(storageType: NSBitmapImageFileType; properties: NSDictionary): NSData; message 'representationUsingType:properties:';
    procedure setProperty_withValue(property_: NSString; value: id); message 'setProperty:withValue:';
    function valueForProperty(property_: NSString): id; message 'valueForProperty:';
  end;

{$endif}
{$endif}
