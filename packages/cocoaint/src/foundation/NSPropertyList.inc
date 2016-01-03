{ Parsed from Foundation.framework NSPropertyList.h }

{$ifdef TYPES}
{$ifndef NSPROPERTYLIST_PAS_T}
{$define NSPROPERTYLIST_PAS_T}

{ Constants }

const
  NSPropertyListImmutable = kCFPropertyListImmutable;
  NSPropertyListMutableContainers = kCFPropertyListMutableContainers;
  NSPropertyListMutableContainersAndLeaves = kCFPropertyListMutableContainersAndLeaves;

const
  NSPropertyListOpenStepFormat = kCFPropertyListOpenStepFormat;

{ Types }
type
  NSPropertyListMutabilityOptions = NSUInteger;
  NSPropertyListMutabilityOptionsPtr = ^NSPropertyListMutabilityOptions;
  NSPropertyListFormat = NSUInteger;
  NSPropertyListFormatPtr = ^NSPropertyListFormat;
  NSPropertyListReadOptions = NSUInteger;
  NSPropertyListReadOptionsPtr = ^NSPropertyListReadOptions;
  NSPropertyListWriteOptions = NSUInteger;
  NSPropertyListWriteOptionsPtr = ^NSPropertyListWriteOptions;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPROPERTYLIST_PAS_R}
{$define NSPROPERTYLIST_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPROPERTYLIST_PAS_F}
{$define NSPROPERTYLIST_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPROPERTYLIST_PAS_S}
{$define NSPROPERTYLIST_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPropertyListSerialization = objcclass;
  NSPropertyListSerializationPointer = ^NSPropertyListSerialization;
  NSPropertyListSerializationPtr = NSPropertyListSerializationPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPROPERTYLIST_PAS_C}
{$define NSPROPERTYLIST_PAS_C}

{ NSPropertyListSerialization }
  NSPropertyListSerialization = objcclass external (NSObject)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    reserved: array[0..(6)-1] of Pointer;
=======
        reserved: array[0..5] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        reserved: array[0..5] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        reserved: array[0..5] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        reserved: array[0..5] of Pointer;
>>>>>>> origin/cpstrnew
    
  public
    class function propertyList_isValidForFormat(plist: id; format: NSPropertyListFormat): Boolean; message 'propertyList:isValidForFormat:';
    class function dataWithPropertyList_format_options_error(plist: id; format: NSPropertyListFormat; opt: NSPropertyListWriteOptions; error: NSErrorPointer): NSData; message 'dataWithPropertyList:format:options:error:';
    class function writePropertyList_toStream_format_options_error(plist: id; stream: NSOutputStream; format: NSPropertyListFormat; opt: NSPropertyListWriteOptions; error: NSErrorPointer): NSInteger; message 'writePropertyList:toStream:format:options:error:';
    class function propertyListWithData_options_format_error(data: NSData; opt: NSPropertyListReadOptions; format: NSPropertyListFormatPtr; error: NSErrorPointer): id; message 'propertyListWithData:options:format:error:';
    class function propertyListWithStream_options_format_error(stream: NSInputStream; opt: NSPropertyListReadOptions; format: NSPropertyListFormatPtr; error: NSErrorPointer): id; message 'propertyListWithStream:options:format:error:';
    class function dataFromPropertyList_format_errorDescription(plist: id; format: NSPropertyListFormat; errorString: NSStringPointer): NSData; message 'dataFromPropertyList:format:errorDescription:';
    class function propertyListFromData_mutabilityOption_format_errorDescription(data: NSData; opt: NSPropertyListMutabilityOptions; format: NSPropertyListFormatPtr; errorString: NSStringPointer): id; message 'propertyListFromData:mutabilityOption:format:errorDescription:';
  end;

{$endif}
{$endif}
