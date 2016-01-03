{ Parsed from Coredata.framework NSManagedObjectContext.h }

{$ifdef TYPES}
{$ifndef NSMANAGEDOBJECTCONTEXT_PAS_T}
{$define NSMANAGEDOBJECTCONTEXT_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMANAGEDOBJECTCONTEXT_PAS_R}
{$define NSMANAGEDOBJECTCONTEXT_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMANAGEDOBJECTCONTEXT_PAS_F}
{$define NSMANAGEDOBJECTCONTEXT_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMANAGEDOBJECTCONTEXT_PAS_S}
{$define NSMANAGEDOBJECTCONTEXT_PAS_S}

{ External string constants }
var
  NSManagedObjectContextWillSaveNotification: NSString; cvar; external;
  NSManagedObjectContextDidSaveNotification: NSString; cvar; external;
  NSManagedObjectContextObjectsDidChangeNotification: NSString; cvar; external;
  NSInsertedObjectsKey: NSString; cvar; external;
  NSUpdatedObjectsKey: NSString; cvar; external;
  NSDeletedObjectsKey: NSString; cvar; external;
  NSRefreshedObjectsKey: NSString; cvar; external;
  NSInvalidatedObjectsKey: NSString; cvar; external;
  NSInvalidatedAllObjectsKey: NSString; cvar; external;

{ External symbols }
var
  NSErrorMergePolicy: id; cvar; external;
  NSMergeByPropertyStoreTrumpMergePolicy: id; cvar; external;
  NSMergeByPropertyObjectTrumpMergePolicy: id; cvar; external;
  NSOverwriteMergePolicy: id; cvar; external;
  NSRollbackMergePolicy: id; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSManagedObjectContext = objcclass;
  NSManagedObjectContextPointer = ^NSManagedObjectContext;
  NSManagedObjectContextPtr = NSManagedObjectContextPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMANAGEDOBJECTCONTEXT_PAS_C}
{$define NSMANAGEDOBJECTCONTEXT_PAS_C}

{ NSManagedObjectContext }
  NSManagedObjectContext = objcclass external (NSObject, NSCodingProtocol, NSLockingProtocol)
  private
    _spinLock: cint32;
    _objectStore: NSPersistentStoreCoordinator;
    _undoManager: NSUndoManager;
    _children: NSMutableArray;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSManagedObjectContext0: cuint);
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
       0: (_anonbitfield_NSManagedObjectContext0: cuint);
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
        _registeredForCallback: 0..1;
        _propagatesDeletesAtEndOfEvent: 0..1;
        _exhaustiveValidation: 0..1;
        _processingChanges: 0..1;
        _useCommittedSnapshot: 0..1;
        _registeredUndoTransactionID: 0..1;
        _retainsAllRegisteredObjects: 0..1;
        _savingInProgress: 0..1;
        _wasDisposed: 0..1;
        _unprocessedChangesPending: 0..1;
        _isDirty: 0..1;
        _ignoreUndoCheckpoints: 0..1;
        _propagatingDeletes: 0..1;
        _isNSEditorEditing: 0..1;
        _isMainThreadBlessed: 0..1;
        _reservedFlags: 0..((1 shl 17)-1);
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
    _unprocessedChanges: NSMutableSet;
    _unprocessedDeletes: NSMutableSet;
    _unprocessedInserts: NSMutableSet;
    _insertedObjects: NSMutableSet;
    _deletedObjects: NSMutableSet;
    _changedObjects: NSMutableSet;
    _lockedObjects: NSMutableSet;
    _refreshedObjects: NSMutableSet;
    _infoByGID: id;
    _cachedObsInfoByEntity: idPtr;
    _undoTransactionID: cshort;
    _lock: id;
    _lockCount: clong;
    _objectStoreLockCount: clong;
    _fetchTimestamp: NSTimeInterval;
    _delegate: id;
    _referenceQueue: id;
    _ignoreChangeNotification: clong;
    _mergePolicy: id;
    _cd_rc: cint32;
    _reserved3: cint32;
    _editors: id;
    _debuggingRecords: idPtr;
    _reserved1: Pointer;
    _reserved2: Pointer;
    
  public
    procedure setPersistentStoreCoordinator(coordinator: NSPersistentStoreCoordinator); message 'setPersistentStoreCoordinator:';
    function persistentStoreCoordinator: NSPersistentStoreCoordinator; message 'persistentStoreCoordinator';
    procedure setUndoManager(undoManager_: NSUndoManager); message 'setUndoManager:';
    function undoManager: NSUndoManager; message 'undoManager';
    function hasChanges: Boolean; message 'hasChanges';
    function objectRegisteredForID(objectID: NSManagedObjectID): NSManagedObject; message 'objectRegisteredForID:';
    function objectWithID(objectID: NSManagedObjectID): NSManagedObject; message 'objectWithID:';
    function existingObjectWithID_error(objectID: NSManagedObjectID; error: NSErrorPointer): NSManagedObject; message 'existingObjectWithID:error:';
    function executeFetchRequest_error(request: NSFetchRequest; error: NSErrorPointer): NSArray; message 'executeFetchRequest:error:';
    function countForFetchRequest_error(request: NSFetchRequest; error: NSErrorPointer): NSUInteger; message 'countForFetchRequest:error:';
    procedure insertObject(object_: NSManagedObject); message 'insertObject:';
    procedure deleteObject(object_: NSManagedObject); message 'deleteObject:';
    procedure refreshObject_mergeChanges(object_: NSManagedObject; flag: Boolean); message 'refreshObject:mergeChanges:';
    procedure detectConflictsForObject(object_: NSManagedObject); message 'detectConflictsForObject:';
    procedure observeValueForKeyPath_ofObject_change_context(keyPath: NSString; object_: id; change: NSDictionary; context: Pointer); message 'observeValueForKeyPath:ofObject:change:context:';
    procedure processPendingChanges; message 'processPendingChanges';
    procedure assignObject_toPersistentStore(object_: id; store: NSPersistentStore); message 'assignObject:toPersistentStore:';
    function insertedObjects: NSSet; message 'insertedObjects';
    function updatedObjects: NSSet; message 'updatedObjects';
    function deletedObjects: NSSet; message 'deletedObjects';
    function registeredObjects: NSSet; message 'registeredObjects';
    procedure undo; message 'undo';
    procedure redo; message 'redo';
    procedure reset; message 'reset';
    procedure rollback; message 'rollback';
    function save(error: NSErrorPointer): Boolean; message 'save:';
    procedure lock; message 'lock';
    procedure unlock; message 'unlock';
    function tryLock: Boolean; message 'tryLock';
    function propagatesDeletesAtEndOfEvent: Boolean; message 'propagatesDeletesAtEndOfEvent';
    procedure setPropagatesDeletesAtEndOfEvent(flag: Boolean); message 'setPropagatesDeletesAtEndOfEvent:';
    function retainsRegisteredObjects: Boolean; message 'retainsRegisteredObjects';
    procedure setRetainsRegisteredObjects(flag: Boolean); message 'setRetainsRegisteredObjects:';
    function stalenessInterval: NSTimeInterval; message 'stalenessInterval';
    procedure setStalenessInterval(expiration: NSTimeInterval); message 'setStalenessInterval:';
    procedure setMergePolicy(mergePolicy_: id); message 'setMergePolicy:';
    function mergePolicy: id; message 'mergePolicy';
    function obtainPermanentIDsForObjects_error(objects: NSArray; error: NSErrorPointer): Boolean; message 'obtainPermanentIDsForObjects:error:';
    procedure mergeChangesFromContextDidSaveNotification(notification: NSNotification); message 'mergeChangesFromContextDidSaveNotification:';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
