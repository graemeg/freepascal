{ Parsed from Appkit.framework NSObjectController.h }

{$ifdef TYPES}
{$ifndef NSOBJECTCONTROLLER_PAS_T}
{$define NSOBJECTCONTROLLER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSOBJECTCONTROLLER_PAS_R}
{$define NSOBJECTCONTROLLER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSOBJECTCONTROLLER_PAS_F}
{$define NSOBJECTCONTROLLER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSOBJECTCONTROLLER_PAS_S}
{$define NSOBJECTCONTROLLER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSObjectController = objcclass;
  NSObjectControllerPointer = ^NSObjectController;
  NSObjectControllerPtr = NSObjectControllerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSOBJECTCONTROLLER_PAS_C}
{$define NSOBJECTCONTROLLER_PAS_C}

{ NSObjectController }
  NSObjectController = objcclass external (NSController)
  private
    _reserved3: Pointer;
    _managedProxy: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __objectControllerFlags: record
      case byte of
       0: (_anonbitfield_NSObjectController0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __objectControllerFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSObjectController0: cuint);
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
        _editable: 0..1;
        _automaticallyPreparesContent: 0..1;
        _hasLoadedData: 0..1;
        _explicitlyCannotAdd: 0..1;
        _explicitlyCannotRemove: 0..1;
        _isUsingManagedProxy: 0..1;
        _hasFetched: 0..1;
        _batches: 0..1;
        _reservedObjectController: 0..((1 shl 24)-1);
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
    _objectClassName: NSString;
    _objectClass: Pobjc_class;
    _contentObjectArray: NSArray;
    _content: id;
    _objectHandler: id;
    
  public
    function initWithContent(content_: id): id; message 'initWithContent:';
    procedure setContent(content_: id); message 'setContent:';
    function content: id; message 'content';
    function selection: id; message 'selection';
    function selectedObjects: NSArray; message 'selectedObjects';
    procedure setAutomaticallyPreparesContent(flag: Boolean); message 'setAutomaticallyPreparesContent:';
    function automaticallyPreparesContent: Boolean; message 'automaticallyPreparesContent';
    procedure prepareContent; message 'prepareContent';
    procedure setObjectClass(objectClass_: Pobjc_class); message 'setObjectClass:';
    function objectClass: Pobjc_class; message 'objectClass';
    function newObject: id; message 'newObject';
    procedure addObject(object_: id); message 'addObject:';
    procedure removeObject(object_: id); message 'removeObject:';
    procedure setEditable(flag: Boolean); message 'setEditable:';
    function isEditable: Boolean; message 'isEditable';
    procedure add(sender: id); message 'add:';
    function canAdd: Boolean; message 'canAdd';
    procedure remove(sender: id); message 'remove:';
    function canRemove: Boolean; message 'canRemove';
    function validateUserInterfaceItem(item: NSValidatedUserInterfaceItemProtocol): Boolean; message 'validateUserInterfaceItem:';
  end;

{ NSManagedControllerCategory }
  NSManagedControllerCategory = objccategory external (NSObjectController)
    function managedObjectContext: NSManagedObjectContext; message 'managedObjectContext';
    procedure setManagedObjectContext(managedObjectContext_: NSManagedObjectContext); message 'setManagedObjectContext:';
    function entityName: NSString; message 'entityName';
    procedure setEntityName(entityName_: NSString); message 'setEntityName:';
    function fetchPredicate: NSPredicate; message 'fetchPredicate';
    procedure setFetchPredicate(predicate: NSPredicate); message 'setFetchPredicate:';
    function fetchWithRequest_merge_error(fetchRequest: NSFetchRequest; merge: Boolean; error: NSErrorPointer): Boolean; message 'fetchWithRequest:merge:error:';
    procedure fetch(sender: id); message 'fetch:';
    procedure setUsesLazyFetching(enabled: Boolean); message 'setUsesLazyFetching:';
    function usesLazyFetching: Boolean; message 'usesLazyFetching';
    function defaultFetchRequest: NSFetchRequest; message 'defaultFetchRequest';
  end;

{$endif}
{$endif}
