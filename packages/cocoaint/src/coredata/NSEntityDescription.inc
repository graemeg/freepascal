{ Parsed from Coredata.framework NSEntityDescription.h }

{$ifdef TYPES}
{$ifndef NSENTITYDESCRIPTION_PAS_T}
{$define NSENTITYDESCRIPTION_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSENTITYDESCRIPTION_PAS_R}
{$define NSENTITYDESCRIPTION_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSENTITYDESCRIPTION_PAS_F}
{$define NSENTITYDESCRIPTION_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSENTITYDESCRIPTION_PAS_S}
{$define NSENTITYDESCRIPTION_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSEntityDescription = objcclass;
  NSEntityDescriptionPointer = ^NSEntityDescription;
  NSEntityDescriptionPtr = NSEntityDescriptionPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSENTITYDESCRIPTION_PAS_C}
{$define NSENTITYDESCRIPTION_PAS_C}

{ NSEntityDescription }
  NSEntityDescription = objcclass external (NSObject, NSCodingProtocol, NSCopyingProtocol, NSFastEnumerationProtocol)
  private
    _cd_rc: cint32;
    _reserved1: id;
    _versionHashModifier: NSString;
    _versionHash: NSData;
    _model: NSManagedObjectModel; {garbage collector: __weak }
    _classNameForEntity: NSString;
    _instanceClass: Pobjc_class;
    _name: NSString;
    _rootentity: NSEntityDescription; {garbage collector: __weak }
    _superentity: NSEntityDescription; {garbage collector: __weak }
    _subentities: NSMutableDictionary;
    _properties: NSMutableDictionary;
    _propertyMapping: id;
    _propertyRanges: NSRangePtr; {garbage collector: __strong }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __entityDescriptionFlags: record
      case byte of
       0: (_anonbitfield_NSEntityDescription0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __entityDescriptionFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSEntityDescription0: cuint);
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
        _isAbstract: 0..1;
        _shouldValidateOnSave: 0..1;
        _isImmutable: 0..1;
        _isFlattened: 0..1;
        _skipValidation: 0..1;
        _hasPropertiesIndexedBySpotlight: 0..1;
        _hasPropertiesStoredInTruthFile: 0..1;
        _reservedEntityDescription: 0..((1 shl 25)-1);
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
    _extraIvars: Pointer; {garbage collector: __strong }
    _userInfo: NSMutableDictionary;
    _flattenedSubentities: id;
    _kvcPropertyAccessors: idPtrPtr;
    _modelsReferenceIDForEntity: clong;
    
  public
    class function entityForName_inManagedObjectContext(entityName: NSString; context: NSManagedObjectContext): NSEntityDescription; message 'entityForName:inManagedObjectContext:';
    class function insertNewObjectForEntityForName_inManagedObjectContext(entityName: NSString; context: NSManagedObjectContext): id; message 'insertNewObjectForEntityForName:inManagedObjectContext:';
    function managedObjectModel: NSManagedObjectModel; message 'managedObjectModel';
    function managedObjectClassName: NSString; message 'managedObjectClassName';
    procedure setManagedObjectClassName(name_: NSString); message 'setManagedObjectClassName:';
    function name: NSString; message 'name';
    procedure setName(name_: NSString); message 'setName:';
    function isAbstract: Boolean; message 'isAbstract';
    procedure setAbstract(flag: Boolean); message 'setAbstract:';
    function subentitiesByName: NSDictionary; message 'subentitiesByName';
    function subentities: NSArray; message 'subentities';
    procedure setSubentities(array_: NSArray); message 'setSubentities:';
    function superentity: NSEntityDescription; message 'superentity';
    function propertiesByName: NSDictionary; message 'propertiesByName';
    function properties: NSArray; message 'properties';
    procedure setProperties(properties_: NSArray); message 'setProperties:';
    function userInfo: NSDictionary; message 'userInfo';
    procedure setUserInfo(dictionary: NSDictionary); message 'setUserInfo:';
    function attributesByName: NSDictionary; message 'attributesByName';
    function relationshipsByName: NSDictionary; message 'relationshipsByName';
    function relationshipsWithDestinationEntity(entity: NSEntityDescription): NSArray; message 'relationshipsWithDestinationEntity:';
    function isKindOfEntity(entity: NSEntityDescription): Boolean; message 'isKindOfEntity:';
    function versionHash: NSData; message 'versionHash';
    function versionHashModifier: NSString; message 'versionHashModifier';
    procedure setVersionHashModifier(modifierString: NSString); message 'setVersionHashModifier:';
    function renamingIdentifier: NSString; message 'renamingIdentifier';
    procedure setRenamingIdentifier(value: NSString); message 'setRenamingIdentifier:';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
    function copyWithZone(zone_: NSZonePtr): id;
    function countByEnumeratingWithState_objects_count(state: NSFastEnumerationStatePtr; stackbuf: idPtr; len: NSUInteger): NSUInteger;
  end;

{$endif}
{$endif}
