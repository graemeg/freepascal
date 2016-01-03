{ Parsed from Coredata.framework NSPersistentStore.h }

{$ifdef TYPES}
{$ifndef NSPERSISTENTSTORE_PAS_T}
{$define NSPERSISTENTSTORE_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPERSISTENTSTORE_PAS_R}
{$define NSPERSISTENTSTORE_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPERSISTENTSTORE_PAS_F}
{$define NSPERSISTENTSTORE_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPERSISTENTSTORE_PAS_S}
{$define NSPERSISTENTSTORE_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPersistentStore = objcclass;
  NSPersistentStorePointer = ^NSPersistentStore;
  NSPersistentStorePtr = NSPersistentStorePointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPERSISTENTSTORE_PAS_C}
{$define NSPERSISTENTSTORE_PAS_C}

{ NSPersistentStore }
  NSPersistentStore = objcclass external (NSObject)
  private
    _coordinator: NSPersistentStoreCoordinator; {garbage collector: __weak }
    _configurationName: NSString;
    _url: NSURL;
    _options: NSDictionary;
    _oidFactories: NSMutableDictionary;
    _defaultFaultHandler: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSPersistentStore0: cuint);
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
       0: (_anonbitfield_NSPersistentStore0: cuint);
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
        _isReadOnly: 0..1;
        cleanOnRemove: 0..1;
        isMDDirty: 0..1;
        _RESERVED: 0..((1 shl 29)-1);
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
    _temporaryIDClass: Pointer;
    _reserved2: Pointer;
    _reserved3: Pointer;
    
  public
    class function metadataForPersistentStoreWithURL_error(url_: NSURL; error: NSErrorPointer): NSDictionary; message 'metadataForPersistentStoreWithURL:error:';
    class function setMetadata_forPersistentStoreWithURL_error(metadata_: NSDictionary; url_: NSURL; error: NSErrorPointer): Boolean; message 'setMetadata:forPersistentStoreWithURL:error:';
    class function migrationManagerClass: Pobjc_class; message 'migrationManagerClass';
    function initWithPersistentStoreCoordinator_configurationName_URL_options(root: NSPersistentStoreCoordinator; name: NSString; url_: NSURL; options_: NSDictionary): id; message 'initWithPersistentStoreCoordinator:configurationName:URL:options:';
    function loadMetadata(error: NSErrorPointer): Boolean; message 'loadMetadata:';
    function persistentStoreCoordinator: NSPersistentStoreCoordinator; message 'persistentStoreCoordinator';
    function configurationName: NSString; message 'configurationName';
    function options: NSDictionary; message 'options';
    function URL: NSURL; message 'URL';
    procedure setURL(url_: NSURL); message 'setURL:';
    function identifier: NSString; message 'identifier';
    procedure setIdentifier(identifier_: NSString); message 'setIdentifier:';
    function type_: NSString; message 'type';
    function isReadOnly: Boolean; message 'isReadOnly';
    procedure setReadOnly(flag: Boolean); message 'setReadOnly:';
    function metadata: NSDictionary; message 'metadata';
    procedure setMetadata(storeMetadata: NSDictionary); message 'setMetadata:';
    procedure didAddToPersistentStoreCoordinator(coordinator: NSPersistentStoreCoordinator); message 'didAddToPersistentStoreCoordinator:';
    procedure willRemoveFromPersistentStoreCoordinator(coordinator: NSPersistentStoreCoordinator); message 'willRemoveFromPersistentStoreCoordinator:';
  end;

{$endif}
{$endif}
