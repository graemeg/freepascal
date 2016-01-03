{ Parsed from Foundation.framework NSData.h }

{$ifdef TYPES}
{$ifndef NSDATA_PAS_T}
{$define NSDATA_PAS_T}

{ Constants }

const
  NSDataReadingMapped = 1 shl 0;
  NSDataReadingUncached = 1 shl 1;

const
  NSDataWritingAtomic = 1 shl 0;

const
  NSMappedRead = NSDataReadingMapped;
  NSUncachedRead = NSDataReadingUncached;

const
  NSAtomicWrite = NSDataWritingAtomic;

const
  NSDataSearchBackwards = 1 shl 0;
  NSDataSearchAnchored = 1 shl 1;

{ Types }
type
  NSDataReadingOptions = NSUInteger;
  NSDataReadingOptionsPtr = ^NSDataReadingOptions;
  NSDataWritingOptions = NSUInteger;
  NSDataWritingOptionsPtr = ^NSDataWritingOptions;
  NSDataSearchOptions = NSUInteger;
  NSDataSearchOptionsPtr = ^NSDataSearchOptions;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSDATA_PAS_R}
{$define NSDATA_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSDATA_PAS_F}
{$define NSDATA_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSDATA_PAS_S}
{$define NSDATA_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSData = objcclass;
  NSDataPointer = ^NSData;
  NSDataPtr = NSDataPointer;
  NSMutableData = objcclass;
  NSMutableDataPointer = ^NSMutableData;
  NSMutableDataPtr = NSMutableDataPointer;
  NSPurgeableData = objcclass;
  NSPurgeableDataPointer = ^NSPurgeableData;
  NSPurgeableDataPtr = NSPurgeableDataPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSDATA_PAS_C}
{$define NSDATA_PAS_C}

{ NSData }
  NSData = objcclass external (NSObject, NSCopyingProtocol, NSMutableCopyingProtocol, NSCodingProtocol)
    
  public
    function length: NSUInteger; message 'length';
    function bytes: Pointer; message 'bytes';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    function mutableCopyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSMutableData }
  NSMutableData = objcclass external (NSData)
    
  public
    function mutableBytes: Pointer; message 'mutableBytes';
    procedure setLength(length_: NSUInteger); message 'setLength:';
  end;

{ NSPurgeableData }
  NSPurgeableData = objcclass external (NSMutableData, NSDiscardableContentProtocol)
  private
    _length: NSUInteger;
    _accessCount: cint32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _private: array[0..(32)-1] of cuint8;
=======
        _private: array[0..31] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..31] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..31] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..31] of cuint8;
>>>>>>> origin/cpstrnew
    _reserved: Pointer;
    
  public

    { Adopted Protocols }
    function beginContentAccess: Boolean;
    procedure endContentAccess;
    procedure discardContentIfPossible;
    function isContentDiscarded: Boolean;
  end;

{ NSExtendedDataCategory }
  NSExtendedDataCategory = objccategory external (NSData)
    function description: NSString; message 'description';
    procedure getBytes_length(buffer: Pointer; length_: NSUInteger); message 'getBytes:length:';
    procedure getBytes_range(buffer: Pointer; range: NSRange); message 'getBytes:range:';
    function isEqualToData(other: NSData): Boolean; message 'isEqualToData:';
    function subdataWithRange(range: NSRange): NSData; message 'subdataWithRange:';
    function writeToFile_atomically(path: NSString; useAuxiliaryFile: Boolean): Boolean; message 'writeToFile:atomically:';
    function writeToURL_atomically(url: NSURL; atomically: Boolean): Boolean; message 'writeToURL:atomically:';
    function writeToFile_options_error(path: NSString; writeOptionsMask: NSDataWritingOptions; errorPtr: NSErrorPointer): Boolean; message 'writeToFile:options:error:';
    function writeToURL_options_error(url: NSURL; writeOptionsMask: NSDataWritingOptions; errorPtr: NSErrorPointer): Boolean; message 'writeToURL:options:error:';
    function rangeOfData_options_range(dataToFind: NSData; mask: NSDataSearchOptions; searchRange: NSRange): NSRange; message 'rangeOfData:options:range:';
  end;

{ NSDataCreationCategory }
  NSDataCreationCategory = objccategory external (NSData)
    class function data: id; message 'data';
    class function dataWithBytes_length(bytes_: Pointer; length_: NSUInteger): id; message 'dataWithBytes:length:';
    class function dataWithBytesNoCopy_length(bytes_: Pointer; length_: NSUInteger): id; message 'dataWithBytesNoCopy:length:';
    class function dataWithBytesNoCopy_length_freeWhenDone(bytes_: Pointer; length_: NSUInteger; b: Boolean): id; message 'dataWithBytesNoCopy:length:freeWhenDone:';
    class function dataWithContentsOfFile_options_error(path: NSString; readOptionsMask: NSDataReadingOptions; errorPtr: NSErrorPointer): id; message 'dataWithContentsOfFile:options:error:';
    class function dataWithContentsOfURL_options_error(url: NSURL; readOptionsMask: NSDataReadingOptions; errorPtr: NSErrorPointer): id; message 'dataWithContentsOfURL:options:error:';
    class function dataWithContentsOfFile(path: NSString): id; message 'dataWithContentsOfFile:';
    class function dataWithContentsOfURL(url: NSURL): id; message 'dataWithContentsOfURL:';
    class function dataWithContentsOfMappedFile(path: NSString): id; message 'dataWithContentsOfMappedFile:';
    function initWithBytes_length(bytes_: Pointer; length_: NSUInteger): id; message 'initWithBytes:length:';
    function initWithBytesNoCopy_length(bytes_: Pointer; length_: NSUInteger): id; message 'initWithBytesNoCopy:length:';
    function initWithBytesNoCopy_length_freeWhenDone(bytes_: Pointer; length_: NSUInteger; b: Boolean): id; message 'initWithBytesNoCopy:length:freeWhenDone:';
    function initWithContentsOfFile_options_error(path: NSString; readOptionsMask: NSDataReadingOptions; errorPtr: NSErrorPointer): id; message 'initWithContentsOfFile:options:error:';
    function initWithContentsOfURL_options_error(url: NSURL; readOptionsMask: NSDataReadingOptions; errorPtr: NSErrorPointer): id; message 'initWithContentsOfURL:options:error:';
    function initWithContentsOfFile(path: NSString): id; message 'initWithContentsOfFile:';
    function initWithContentsOfURL(url: NSURL): id; message 'initWithContentsOfURL:';
    function initWithContentsOfMappedFile(path: NSString): id; message 'initWithContentsOfMappedFile:';
    function initWithData(data_: NSData): id; message 'initWithData:';
    class function dataWithData(data_: NSData): id; message 'dataWithData:';
  end;

{ NSDeprecated_NSDataCategory }
  NSDeprecated_NSDataCategory = objccategory external name 'NSDeprecated' (NSData)
    procedure getBytes(buffer: Pointer); message 'getBytes:';
  end;

{ NSExtendedMutableDataCategory }
  NSExtendedMutableDataCategory = objccategory external (NSMutableData)
    procedure appendBytes_length(bytes_: Pointer; length_: NSUInteger); message 'appendBytes:length:';
    procedure appendData(other: NSData); message 'appendData:';
    procedure increaseLengthBy(extraLength: NSUInteger); message 'increaseLengthBy:';
    procedure replaceBytesInRange_withBytes(range: NSRange; bytes_: Pointer); message 'replaceBytesInRange:withBytes:';
    procedure resetBytesInRange(range: NSRange); message 'resetBytesInRange:';
    procedure setData(data_: NSData); message 'setData:';
    procedure replaceBytesInRange_withBytes_length(range: NSRange; replacementBytes: Pointer; replacementLength: NSUInteger); message 'replaceBytesInRange:withBytes:length:';
  end;

{ NSMutableDataCreationCategory }
  NSMutableDataCreationCategory = objccategory external (NSMutableData)
    class function dataWithCapacity(aNumItems: NSUInteger): id; message 'dataWithCapacity:';
    class function dataWithLength(length_: NSUInteger): id; message 'dataWithLength:';
    function initWithCapacity(capacity: NSUInteger): id; message 'initWithCapacity:';
    function initWithLength(length_: NSUInteger): id; message 'initWithLength:';
  end;

{$endif}
{$endif}
