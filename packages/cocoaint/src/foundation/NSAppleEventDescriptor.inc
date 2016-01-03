{ Parsed from Foundation.framework NSAppleEventDescriptor.h }

{$ifdef TYPES}
{$ifndef NSAPPLEEVENTDESCRIPTOR_PAS_T}
{$define NSAPPLEEVENTDESCRIPTOR_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSAPPLEEVENTDESCRIPTOR_PAS_R}
{$define NSAPPLEEVENTDESCRIPTOR_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSAPPLEEVENTDESCRIPTOR_PAS_F}
{$define NSAPPLEEVENTDESCRIPTOR_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSAPPLEEVENTDESCRIPTOR_PAS_S}
{$define NSAPPLEEVENTDESCRIPTOR_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSAppleEventDescriptor = objcclass;
  NSAppleEventDescriptorPointer = ^NSAppleEventDescriptor;
  NSAppleEventDescriptorPtr = NSAppleEventDescriptorPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSAPPLEEVENTDESCRIPTOR_PAS_C}
{$define NSAPPLEEVENTDESCRIPTOR_PAS_C}

{ NSAppleEventDescriptor }
  NSAppleEventDescriptor = objcclass external (NSObject, NSCopyingProtocol)
  private
    _desc: AEDesc;
    _hasValidDesc: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _padding: array[0..(3)-1] of char;
=======
        _padding: array[0..2] of char;
>>>>>>> graemeg/cpstrnew
=======
        _padding: array[0..2] of char;
>>>>>>> graemeg/cpstrnew
=======
        _padding: array[0..2] of char;
>>>>>>> graemeg/cpstrnew
=======
        _padding: array[0..2] of char;
>>>>>>> origin/cpstrnew
    
  public
    class function nullDescriptor: NSAppleEventDescriptor; message 'nullDescriptor';
    class function descriptorWithDescriptorType_bytes_length(descriptorType_: DescType; bytes: Pointer; byteCount: NSUInteger): NSAppleEventDescriptor; message 'descriptorWithDescriptorType:bytes:length:';
    class function descriptorWithDescriptorType_data(descriptorType_: DescType; data_: NSData): NSAppleEventDescriptor; message 'descriptorWithDescriptorType:data:';
    class function descriptorWithBoolean(boolean_: Boolean): NSAppleEventDescriptor; message 'descriptorWithBoolean:';
    class function descriptorWithEnumCode(enumerator: OSType): NSAppleEventDescriptor; message 'descriptorWithEnumCode:';
    class function descriptorWithInt32(signedInt: SInt32): NSAppleEventDescriptor; message 'descriptorWithInt32:';
    class function descriptorWithTypeCode(typeCode: OSType): NSAppleEventDescriptor; message 'descriptorWithTypeCode:';
    class function descriptorWithString(string_: NSString): NSAppleEventDescriptor; message 'descriptorWithString:';
    class function appleEventWithEventClass_eventID_targetDescriptor_returnID_transactionID(eventClass_: AEEventClass; eventID_: AEEventID; targetDescriptor: NSAppleEventDescriptor; returnID_: AEReturnID; transactionID_: AETransactionID): NSAppleEventDescriptor; message 'appleEventWithEventClass:eventID:targetDescriptor:returnID:transactionID:';
    class function listDescriptor: NSAppleEventDescriptor; message 'listDescriptor';
    class function recordDescriptor: NSAppleEventDescriptor; message 'recordDescriptor';
    function initWithAEDescNoCopy(aeDesc_: AEDescPtr): id; message 'initWithAEDescNoCopy:';
    function initWithDescriptorType_bytes_length(descriptorType_: DescType; bytes: Pointer; byteCount: NSUInteger): id; message 'initWithDescriptorType:bytes:length:';
    function initWithDescriptorType_data(descriptorType_: DescType; data_: NSData): id; message 'initWithDescriptorType:data:';
    function initWithEventClass_eventID_targetDescriptor_returnID_transactionID(eventClass_: AEEventClass; eventID_: AEEventID; targetDescriptor: NSAppleEventDescriptor; returnID_: AEReturnID; transactionID_: AETransactionID): id; message 'initWithEventClass:eventID:targetDescriptor:returnID:transactionID:';
    function initListDescriptor: id; message 'initListDescriptor';
    function initRecordDescriptor: id; message 'initRecordDescriptor';
    function aeDesc: AEDescPtr; message 'aeDesc';
    function descriptorType: DescType; message 'descriptorType';
    function data: NSData; message 'data';
    function booleanValue: Boolean; message 'booleanValue';
    function enumCodeValue: OSType; message 'enumCodeValue';
    function int32Value: SInt32; message 'int32Value';
    function typeCodeValue: OSType; message 'typeCodeValue';
    function stringValue: NSString; message 'stringValue';
    function eventClass: AEEventClass; message 'eventClass';
    function eventID: AEEventID; message 'eventID';
    function returnID: AEReturnID; message 'returnID';
    function transactionID: AETransactionID; message 'transactionID';
    procedure setParamDescriptor_forKeyword(descriptor: NSAppleEventDescriptor; keyword: AEKeyword); message 'setParamDescriptor:forKeyword:';
    function paramDescriptorForKeyword(keyword: AEKeyword): NSAppleEventDescriptor; message 'paramDescriptorForKeyword:';
    procedure removeParamDescriptorWithKeyword(keyword: AEKeyword); message 'removeParamDescriptorWithKeyword:';
    procedure setAttributeDescriptor_forKeyword(descriptor: NSAppleEventDescriptor; keyword: AEKeyword); message 'setAttributeDescriptor:forKeyword:';
    function attributeDescriptorForKeyword(keyword: AEKeyword): NSAppleEventDescriptor; message 'attributeDescriptorForKeyword:';
    function numberOfItems: NSInteger; message 'numberOfItems';
    procedure insertDescriptor_atIndex(descriptor: NSAppleEventDescriptor; index: NSInteger); message 'insertDescriptor:atIndex:';
    function descriptorAtIndex(index: NSInteger): NSAppleEventDescriptor; message 'descriptorAtIndex:';
    procedure removeDescriptorAtIndex(index: NSInteger); message 'removeDescriptorAtIndex:';
    procedure removeDecriptorAtIndex(index: NSInteger); message 'removeDecriptorAtIndex:';
    procedure setDescriptor_forKeyword(descriptor: NSAppleEventDescriptor; keyword: AEKeyword); message 'setDescriptor:forKeyword:';
    function descriptorForKeyword(keyword: AEKeyword): NSAppleEventDescriptor; message 'descriptorForKeyword:';
    procedure removeDescriptorWithKeyword(keyword: AEKeyword); message 'removeDescriptorWithKeyword:';
    function keywordForDescriptorAtIndex(index: NSInteger): AEKeyword; message 'keywordForDescriptorAtIndex:';
    function coerceToDescriptorType(descriptorType_: DescType): NSAppleEventDescriptor; message 'coerceToDescriptorType:';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
  end;

{$endif}
{$endif}
