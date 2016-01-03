{ Parsed from Coredata.framework NSManagedObjectModel.h }

{$ifdef TYPES}
{$ifndef NSMANAGEDOBJECTMODEL_PAS_T}
{$define NSMANAGEDOBJECTMODEL_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMANAGEDOBJECTMODEL_PAS_R}
{$define NSMANAGEDOBJECTMODEL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMANAGEDOBJECTMODEL_PAS_F}
{$define NSMANAGEDOBJECTMODEL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMANAGEDOBJECTMODEL_PAS_S}
{$define NSMANAGEDOBJECTMODEL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSManagedObjectModel = objcclass;
  NSManagedObjectModelPointer = ^NSManagedObjectModel;
  NSManagedObjectModelPtr = NSManagedObjectModelPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMANAGEDOBJECTMODEL_PAS_C}
{$define NSMANAGEDOBJECTMODEL_PAS_C}

{ NSManagedObjectModel }
  NSManagedObjectModel = objcclass external (NSObject, NSCodingProtocol, NSCopyingProtocol, NSFastEnumerationProtocol)
  private
    _reserved: Pointer;
    _reserved2: Pointer;
    _localizationPolicy: id;
    _entities: NSMutableDictionary;
    _configurations: NSMutableDictionary;
    _fetchRequestTemplates: NSMutableDictionary;
    _versionIdentifiers: NSSet;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __managedObjectModelFlags: record
      case byte of
       0: (_anonbitfield_NSManagedObjectModel0: cuint);
       1: (data: bitpacked record
        _isInUse: 0..1;
        _isImmutable: 0..1;
        _reservedEntityDescription: 0..((1 shl 30)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __managedObjectModelFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSManagedObjectModel0: cuint);
       1: (
        _isInUse: 0..1;
        _isImmutable: 0..1;
        _reservedEntityDescription: 0..((1 shl 30)-1);
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
    
  public
    class function mergedModelFromBundles(bundles: NSArray): NSManagedObjectModel; message 'mergedModelFromBundles:';
    class function modelByMergingModels(models: NSArray): NSManagedObjectModel; message 'modelByMergingModels:';
    function init: id; message 'init';
    function initWithContentsOfURL(url: NSURL): id; message 'initWithContentsOfURL:';
    function entitiesByName: NSDictionary; message 'entitiesByName';
    function entities: NSArray; message 'entities';
    procedure setEntities(entities_: NSArray); message 'setEntities:';
    function configurations: NSArray; message 'configurations';
    function entitiesForConfiguration(configuration: NSString): NSArray; message 'entitiesForConfiguration:';
    procedure setEntities_forConfiguration(entities_: NSArray; configuration: NSString); message 'setEntities:forConfiguration:';
    procedure setFetchRequestTemplate_forName(fetchRequestTemplate: NSFetchRequest; name: NSString); message 'setFetchRequestTemplate:forName:';
    function fetchRequestTemplateForName(name: NSString): NSFetchRequest; message 'fetchRequestTemplateForName:';
    function fetchRequestFromTemplateWithName_substitutionVariables(name: NSString; variables: NSDictionary): NSFetchRequest; message 'fetchRequestFromTemplateWithName:substitutionVariables:';
    function localizationDictionary: NSDictionary; message 'localizationDictionary';
    procedure setLocalizationDictionary(localizationDictionary_: NSDictionary); message 'setLocalizationDictionary:';
    class function mergedModelFromBundles_forStoreMetadata(bundles: NSArray; metadata: NSDictionary): NSManagedObjectModel; message 'mergedModelFromBundles:forStoreMetadata:';
    class function modelByMergingModels_forStoreMetadata(models: NSArray; metadata: NSDictionary): NSManagedObjectModel; message 'modelByMergingModels:forStoreMetadata:';
    function fetchRequestTemplatesByName: NSDictionary; message 'fetchRequestTemplatesByName';
    function versionIdentifiers: NSSet; message 'versionIdentifiers';
    procedure setVersionIdentifiers(identifiers: NSSet); message 'setVersionIdentifiers:';
    function isConfiguration_compatibleWithStoreMetadata(configuration: NSString; metadata: NSDictionary): Boolean; message 'isConfiguration:compatibleWithStoreMetadata:';
    function entityVersionHashesByName: NSDictionary; message 'entityVersionHashesByName';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
    function copyWithZone(zone_: NSZonePtr): id;
    function countByEnumeratingWithState_objects_count(state: NSFastEnumerationStatePtr; stackbuf: idPtr; len: NSUInteger): NSUInteger;
  end;

{$endif}
{$endif}
