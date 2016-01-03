{ Parsed from Foundation.framework NSMetadata.h }

{$ifdef TYPES}
{$ifndef NSMETADATA_PAS_T}
{$define NSMETADATA_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMETADATA_PAS_R}
{$define NSMETADATA_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMETADATA_PAS_F}
{$define NSMETADATA_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMETADATA_PAS_S}
{$define NSMETADATA_PAS_S}

{ External string constants }
var
  NSMetadataQueryDidStartGatheringNotification: NSString; cvar; external;
  NSMetadataQueryGatheringProgressNotification: NSString; cvar; external;
  NSMetadataQueryDidFinishGatheringNotification: NSString; cvar; external;
  NSMetadataQueryDidUpdateNotification: NSString; cvar; external;
  NSMetadataQueryResultContentRelevanceAttribute: NSString; cvar; external;
  NSMetadataQueryUserHomeScope: NSString; cvar; external;
  NSMetadataQueryLocalComputerScope: NSString; cvar; external;
  NSMetadataQueryNetworkScope: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSMetadataQueryDelegateProtocol = objcprotocol;
  NSMetadataQuery = objcclass;
  NSMetadataQueryPointer = ^NSMetadataQuery;
  NSMetadataQueryPtr = NSMetadataQueryPointer;
  NSMetadataItem = objcclass;
  NSMetadataItemPointer = ^NSMetadataItem;
  NSMetadataItemPtr = NSMetadataItemPointer;
  NSMetadataQueryAttributeValueTuple = objcclass;
  NSMetadataQueryAttributeValueTuplePointer = ^NSMetadataQueryAttributeValueTuple;
  NSMetadataQueryAttributeValueTuplePtr = NSMetadataQueryAttributeValueTuplePointer;
  NSMetadataQueryResultGroup = objcclass;
  NSMetadataQueryResultGroupPointer = ^NSMetadataQueryResultGroup;
  NSMetadataQueryResultGroupPtr = NSMetadataQueryResultGroupPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMETADATA_PAS_C}
{$define NSMETADATA_PAS_C}

{ NSMetadataQuery }
  NSMetadataQuery = objcclass external (NSObject)
  private
    _flags: NSUInteger;
    _interval: NSTimeInterval;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _private: array[0..(11)-1] of id;
=======
        _private: array[0..10] of id;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..10] of id;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..10] of id;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..10] of id;
>>>>>>> origin/cpstrnew
    _reserved: Pointer; {garbage collector: __strong }
    
  public
    function init: id; message 'init';
    function delegate: NSMetadataQueryDelegateProtocol; message 'delegate';
    procedure setDelegate(delegate_: NSMetadataQueryDelegateProtocol); message 'setDelegate:';
    function predicate: NSPredicate; message 'predicate';
    procedure setPredicate(predicate_: NSPredicate); message 'setPredicate:';
    function sortDescriptors: NSArray; message 'sortDescriptors';
    procedure setSortDescriptors(descriptors: NSArray); message 'setSortDescriptors:';
    function valueListAttributes: NSArray; message 'valueListAttributes';
    procedure setValueListAttributes(attrs: NSArray); message 'setValueListAttributes:';
    function groupingAttributes: NSArray; message 'groupingAttributes';
    procedure setGroupingAttributes(attrs: NSArray); message 'setGroupingAttributes:';
    function notificationBatchingInterval: NSTimeInterval; message 'notificationBatchingInterval';
    procedure setNotificationBatchingInterval(ti: NSTimeInterval); message 'setNotificationBatchingInterval:';
    function searchScopes: NSArray; message 'searchScopes';
    procedure setSearchScopes(scopes: NSArray); message 'setSearchScopes:';
    function startQuery: Boolean; message 'startQuery';
    procedure stopQuery; message 'stopQuery';
    function isStarted: Boolean; message 'isStarted';
    function isGathering: Boolean; message 'isGathering';
    function isStopped: Boolean; message 'isStopped';
    procedure disableUpdates; message 'disableUpdates';
    procedure enableUpdates; message 'enableUpdates';
    function resultCount: NSUInteger; message 'resultCount';
    function resultAtIndex(idx: NSUInteger): id; message 'resultAtIndex:';
    function results: NSArray; message 'results';
    function indexOfResult(result_: id): NSUInteger; message 'indexOfResult:';
    function valueLists: NSDictionary; message 'valueLists';
    function groupedResults: NSArray; message 'groupedResults';
    function valueOfAttribute_forResultAtIndex(attrName: NSString; idx: NSUInteger): id; message 'valueOfAttribute:forResultAtIndex:';
  end;

{ NSMetadataItem }
  NSMetadataItem = objcclass external (NSObject)
  private
    _item: id;
    _reserved: Pointer; {garbage collector: __strong }
    
  public
    function valueForAttribute(key: NSString): id; message 'valueForAttribute:';
    function valuesForAttributes(keys: NSArray): NSDictionary; message 'valuesForAttributes:';
    function attributes: NSArray; message 'attributes';
  end;

{ NSMetadataQueryAttributeValueTuple }
  NSMetadataQueryAttributeValueTuple = objcclass external (NSObject)
  private
    _attr: id;
    _value: id;
    _count: NSUInteger;
    _reserved: Pointer;
    
  public
    function attribute: NSString; message 'attribute';
    function value: id; message 'value';
    function count: NSUInteger; message 'count';
  end;

{ NSMetadataQueryResultGroup }
  NSMetadataQueryResultGroup = objcclass external (NSObject)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _private: array[0..(9)-1] of id;
    _private2: array[0..(1)-1] of NSUInteger;
=======
        _private: array[0..8] of id;
    _private2: NSUInteger;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..8] of id;
    _private2: NSUInteger;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..8] of id;
    _private2: NSUInteger;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..8] of id;
    _private2: NSUInteger;
>>>>>>> origin/cpstrnew
    _reserved: Pointer;
    
  public
    function attribute: NSString; message 'attribute';
    function value: id; message 'value';
    function subgroups: NSArray; message 'subgroups';
    function resultCount: NSUInteger; message 'resultCount';
    function resultAtIndex(idx: NSUInteger): id; message 'resultAtIndex:';
    function results: NSArray; message 'results';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSMETADATA_PAS_P}
{$define NSMETADATA_PAS_P}
  
{ NSMetadataQueryDelegate Protocol }
  NSMetadataQueryDelegateProtocol = objcprotocol external name 'NSMetadataQueryDelegate'
  optional
    function metadataQuery_replacementObjectForResultObject(query: NSMetadataQuery; result_: NSMetadataItem): id; message 'metadataQuery:replacementObjectForResultObject:';
    function metadataQuery_replacementValueForAttribute_value(query: NSMetadataQuery; attrName: NSString; attrValue: id): id; message 'metadataQuery:replacementValueForAttribute:value:';
  end;
{$endif}
{$endif}
