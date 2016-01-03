{ Parsed from Coredata.framework NSMigrationManager.h }

{$ifdef TYPES}
{$ifndef NSMIGRATIONMANAGER_PAS_T}
{$define NSMIGRATIONMANAGER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMIGRATIONMANAGER_PAS_R}
{$define NSMIGRATIONMANAGER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMIGRATIONMANAGER_PAS_F}
{$define NSMIGRATIONMANAGER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMIGRATIONMANAGER_PAS_S}
{$define NSMIGRATIONMANAGER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSMigrationManager = objcclass;
  NSMigrationManagerPointer = ^NSMigrationManager;
  NSMigrationManagerPtr = NSMigrationManagerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMIGRATIONMANAGER_PAS_C}
{$define NSMIGRATIONMANAGER_PAS_C}

{ NSMigrationManager }
  NSMigrationManager = objcclass external (NSObject)
  private
    _sourceModel: NSManagedObjectModel;
    _sourceEntitiesByVersionHash: NSDictionary;
    _destinationModel: NSManagedObjectModel;
    _destinationEntitiesByVersionHash: NSDictionary;
    _mappingModel: NSMappingModel;
    _sourceManagedObjectContext: NSManagedObjectContext;
    _destinationManagedObjectContext: NSManagedObjectContext;
    _migrationContext: NSMigrationContext;
    _userInfo: NSDictionary;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __migrationManagerFlags: record
      case byte of
       0: (_anonbitfield_NSMigrationManager0: cuint);
       1: (data: bitpacked record
        _migrationWasCancelled: 0..1;
        _reservedMigrationManager: 0..((1 shl 31)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __migrationManagerFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSMigrationManager0: cuint);
       1: (
        _migrationWasCancelled: 0..1;
        _reservedMigrationManager: 0..((1 shl 31)-1);
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
      );
      end;
    _migrationCancellationError: NSError;
    _reserved1: id;
    _reserved2: id;
    _reserved3: id;
    _reserved4: id;
    
  public
    function initWithSourceModel_destinationModel(sourceModel_: NSManagedObjectModel; destinationModel_: NSManagedObjectModel): id; message 'initWithSourceModel:destinationModel:';
    function migrateStoreFromURL_type_options_withMappingModel_toDestinationURL_destinationType_destinationOptions_error(sourceURL: NSURL; sStoreType: NSString; sOptions: NSDictionary; mappings: NSMappingModel; dURL: NSURL; dStoreType: NSString; dOptions: NSDictionary; error: NSErrorPointer): Boolean; message 'migrateStoreFromURL:type:options:withMappingModel:toDestinationURL:destinationType:destinationOptions:error:';
    procedure reset; message 'reset';
    function mappingModel: NSMappingModel; message 'mappingModel';
    function sourceModel: NSManagedObjectModel; message 'sourceModel';
    function destinationModel: NSManagedObjectModel; message 'destinationModel';
    function sourceContext: NSManagedObjectContext; message 'sourceContext';
    function destinationContext: NSManagedObjectContext; message 'destinationContext';
    function sourceEntityForEntityMapping(mEntity: NSEntityMapping): NSEntityDescription; message 'sourceEntityForEntityMapping:';
    function destinationEntityForEntityMapping(mEntity: NSEntityMapping): NSEntityDescription; message 'destinationEntityForEntityMapping:';
    procedure associateSourceInstance_withDestinationInstance_forEntityMapping(sourceInstance: NSManagedObject; destinationInstance: NSManagedObject; entityMapping: NSEntityMapping); message 'associateSourceInstance:withDestinationInstance:forEntityMapping:';
    function destinationInstancesForEntityMappingNamed_sourceInstances(mappingName: NSString; sourceInstances: NSArray): NSArray; message 'destinationInstancesForEntityMappingNamed:sourceInstances:';
    function sourceInstancesForEntityMappingNamed_destinationInstances(mappingName: NSString; destinationInstances: NSArray): NSArray; message 'sourceInstancesForEntityMappingNamed:destinationInstances:';
    function currentEntityMapping: NSEntityMapping; message 'currentEntityMapping';
    function migrationProgress: single; message 'migrationProgress';
    function userInfo: NSDictionary; message 'userInfo';
    procedure setUserInfo(dict: NSDictionary); message 'setUserInfo:';
    procedure cancelMigrationWithError(error: NSError); message 'cancelMigrationWithError:';
  end;

{$endif}
{$endif}
