{ Parsed from Coredata.framework NSFetchRequest.h }

{$ifdef TYPES}
{$ifndef NSFETCHREQUEST_PAS_T}
{$define NSFETCHREQUEST_PAS_T}

{ Constants }

const
  NSManagedObjectResultType = $00;
  NSManagedObjectIDResultType = $01;
  NSDictionaryResultType = $02;

{ Types }
type
  NSFetchRequestResultType = NSUInteger;
  NSFetchRequestResultTypePtr = ^NSFetchRequestResultType;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSFETCHREQUEST_PAS_R}
{$define NSFETCHREQUEST_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSFETCHREQUEST_PAS_F}
{$define NSFETCHREQUEST_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSFETCHREQUEST_PAS_S}
{$define NSFETCHREQUEST_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSFetchRequest = objcclass;
  NSFetchRequestPointer = ^NSFetchRequest;
  NSFetchRequestPtr = NSFetchRequestPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSFETCHREQUEST_PAS_C}
{$define NSFETCHREQUEST_PAS_C}

{ NSFetchRequest }
  NSFetchRequest = objcclass external (NSObject, NSCodingProtocol, NSCopyingProtocol)
  private
    _reserved: Pointer;
    _reserved2: Pointer;
    _offset: NSUInteger;
    _valuesToFetch: NSArray;
    _entity: NSEntityDescription; {garbage collector: __weak }
    _predicate: NSPredicate;
    _sortDescriptors: NSArray;
    _batchSize: NSUInteger;
    _fetchLimit: culong;
    _affectedStores: NSArray;
    _relationshipKeyPathsForPrefetching: NSArray;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSFetchRequest0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __flags: bitpacked record
      case byte of
       0: (_anonbitfield_NSFetchRequest0: cuint);
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
        distinctValuesOnly: 0..1;
        _includesSubentities: 0..1;
        _includesPropertyValues: 0..1;
        _resultType: 0..((1 shl 3)-1);
        _returnsObjectsAsFaults: 0..1;
        excludePendingChanges: 0..1;
        isInUse: 0..1;
        __RESERVED: 0..((1 shl 23)-1);
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
    
  public
    function entity: NSEntityDescription; message 'entity';
    procedure setEntity(entity_: NSEntityDescription); message 'setEntity:';
    function predicate: NSPredicate; message 'predicate';
    procedure setPredicate(predicate_: NSPredicate); message 'setPredicate:';
    function sortDescriptors: NSArray; message 'sortDescriptors';
    procedure setSortDescriptors(sortDescriptors_: NSArray); message 'setSortDescriptors:';
    function fetchLimit: NSUInteger; message 'fetchLimit';
    procedure setFetchLimit(limit: NSUInteger); message 'setFetchLimit:';
    function affectedStores: NSArray; message 'affectedStores';
    procedure setAffectedStores(stores: NSArray); message 'setAffectedStores:';
    function resultType: NSFetchRequestResultType; message 'resultType';
    procedure setResultType(type_: NSFetchRequestResultType); message 'setResultType:';
    function includesSubentities: Boolean; message 'includesSubentities';
    procedure setIncludesSubentities(yesNo: Boolean); message 'setIncludesSubentities:';
    function includesPropertyValues: Boolean; message 'includesPropertyValues';
    procedure setIncludesPropertyValues(yesNo: Boolean); message 'setIncludesPropertyValues:';
    function returnsObjectsAsFaults: Boolean; message 'returnsObjectsAsFaults';
    procedure setReturnsObjectsAsFaults(yesNo: Boolean); message 'setReturnsObjectsAsFaults:';
    function relationshipKeyPathsForPrefetching: NSArray; message 'relationshipKeyPathsForPrefetching';
    procedure setRelationshipKeyPathsForPrefetching(keys: NSArray); message 'setRelationshipKeyPathsForPrefetching:';
    function includesPendingChanges: Boolean; message 'includesPendingChanges';
    procedure setIncludesPendingChanges(flag: Boolean); message 'setIncludesPendingChanges:';
    function returnsDistinctResults: Boolean; message 'returnsDistinctResults';
    procedure setReturnsDistinctResults(flag: Boolean); message 'setReturnsDistinctResults:';
    function propertiesToFetch: NSArray; message 'propertiesToFetch';
    procedure setPropertiesToFetch(values: NSArray); message 'setPropertiesToFetch:';
    function fetchOffset: NSUInteger; message 'fetchOffset';
    procedure setFetchOffset(offset: NSUInteger); message 'setFetchOffset:';
    function fetchBatchSize: NSUInteger; message 'fetchBatchSize';
    procedure setFetchBatchSize(bsize: NSUInteger); message 'setFetchBatchSize:';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
    function copyWithZone(zone_: NSZonePtr): id;
  end;

{$endif}
{$endif}
