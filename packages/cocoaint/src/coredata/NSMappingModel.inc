{ Parsed from Coredata.framework NSMappingModel.h }

{$ifdef TYPES}
{$ifndef NSMAPPINGMODEL_PAS_T}
{$define NSMAPPINGMODEL_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMAPPINGMODEL_PAS_R}
{$define NSMAPPINGMODEL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMAPPINGMODEL_PAS_F}
{$define NSMAPPINGMODEL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMAPPINGMODEL_PAS_S}
{$define NSMAPPINGMODEL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSMappingModel = objcclass;
  NSMappingModelPointer = ^NSMappingModel;
  NSMappingModelPtr = NSMappingModelPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMAPPINGMODEL_PAS_C}
{$define NSMAPPINGMODEL_PAS_C}

{ NSMappingModel }
  NSMappingModel = objcclass external (NSObject)
  private
    _reserved: Pointer;
    _reserved1: Pointer;
    _reserved2: Pointer;
    _entityMappings: NSMutableArray;
    _entityMappingsByName: NSMutableDictionary;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __modelMappingFlags: record
      case byte of
       0: (_anonbitfield_NSMappingModel0: cuint);
       1: (data: bitpacked record
        _isInUse: 0..1;
        _reservedModelMapping: 0..((1 shl 31)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __modelMappingFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSMappingModel0: cuint);
       1: (
        _isInUse: 0..1;
        _reservedModelMapping: 0..((1 shl 31)-1);
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
    class function mappingModelFromBundles_forSourceModel_destinationModel(bundles: NSArray; sourceModel: NSManagedObjectModel; destinationModel: NSManagedObjectModel): NSMappingModel; message 'mappingModelFromBundles:forSourceModel:destinationModel:';
    class function inferredMappingModelForSourceModel_destinationModel_error(sourceModel: NSManagedObjectModel; destinationModel: NSManagedObjectModel; error: NSErrorPointer): NSMappingModel; message 'inferredMappingModelForSourceModel:destinationModel:error:';
    function initWithContentsOfURL(url: NSURL): id; message 'initWithContentsOfURL:';
    function entityMappings: NSArray; message 'entityMappings';
    procedure setEntityMappings(mappings: NSArray); message 'setEntityMappings:';
    function entityMappingsByName: NSDictionary; message 'entityMappingsByName';
  end;

{$endif}
{$endif}
