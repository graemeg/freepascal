{ Parsed from Coredata.framework NSEntityMapping.h }

{$ifdef TYPES}
{$ifndef NSENTITYMAPPING_PAS_T}
{$define NSENTITYMAPPING_PAS_T}

{ Constants }

const
  NSUndefinedEntityMappingType = $00;
  NSCustomEntityMappingType = $01;
  NSAddEntityMappingType = $02;
  NSRemoveEntityMappingType = $03;
  NSCopyEntityMappingType = $04;
  NSTransformEntityMappingType = $05;

{ Types }
type
  NSEntityMappingType = NSUInteger;
  NSEntityMappingTypePtr = ^NSEntityMappingType;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSENTITYMAPPING_PAS_R}
{$define NSENTITYMAPPING_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSENTITYMAPPING_PAS_F}
{$define NSENTITYMAPPING_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSENTITYMAPPING_PAS_S}
{$define NSENTITYMAPPING_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSEntityMapping = objcclass;
  NSEntityMappingPointer = ^NSEntityMapping;
  NSEntityMappingPtr = NSEntityMappingPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSENTITYMAPPING_PAS_C}
{$define NSENTITYMAPPING_PAS_C}

{ NSEntityMapping }
  NSEntityMapping = objcclass external (NSObject)
  private
    _reserved: Pointer;
    _reserved1: Pointer;
    _reserved2: Pointer;
    _name: NSString;
    _mappingType: NSEntityMappingType;
    _sourceEntityName: NSString;
    _sourceEntityVersionHash: NSData;
    _destinationEntityName: NSString;
    _destinationEntityVersionHash: NSData;
    _sourceExpression: NSExpression;
    _userInfo: NSDictionary;
    _entityMigrationPolicyClassName: NSString;
    _attributeMappings: NSMutableArray;
    _relationshipMappings: NSMutableArray;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __entityMappingFlags: record
      case byte of
       0: (_anonbitfield_NSEntityMapping0: cuint);
       1: (data: bitpacked record
        _isInUse: 0..1;
        _reservedEntityMapping: 0..((1 shl 31)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __entityMappingFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSEntityMapping0: cuint);
       1: (
        _isInUse: 0..1;
        _reservedEntityMapping: 0..((1 shl 31)-1);
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
    function name: NSString; message 'name';
    procedure setName(name_: NSString); message 'setName:';
    function mappingType: NSEntityMappingType; message 'mappingType';
    procedure setMappingType(type_: NSEntityMappingType); message 'setMappingType:';
    function sourceEntityName: NSString; message 'sourceEntityName';
    procedure setSourceEntityName(name_: NSString); message 'setSourceEntityName:';
    function sourceEntityVersionHash: NSData; message 'sourceEntityVersionHash';
    procedure setSourceEntityVersionHash(vhash: NSData); message 'setSourceEntityVersionHash:';
    function destinationEntityName: NSString; message 'destinationEntityName';
    procedure setDestinationEntityName(name_: NSString); message 'setDestinationEntityName:';
    function destinationEntityVersionHash: NSData; message 'destinationEntityVersionHash';
    procedure setDestinationEntityVersionHash(vhash: NSData); message 'setDestinationEntityVersionHash:';
    function attributeMappings: NSArray; message 'attributeMappings';
    procedure setAttributeMappings(mappings: NSArray); message 'setAttributeMappings:';
    function relationshipMappings: NSArray; message 'relationshipMappings';
    procedure setRelationshipMappings(mappings: NSArray); message 'setRelationshipMappings:';
    function sourceExpression: NSExpression; message 'sourceExpression';
    procedure setSourceExpression(source: NSExpression); message 'setSourceExpression:';
    function userInfo: NSDictionary; message 'userInfo';
    procedure setUserInfo(dict: NSDictionary); message 'setUserInfo:';
    function entityMigrationPolicyClassName: NSString; message 'entityMigrationPolicyClassName';
    procedure setEntityMigrationPolicyClassName(name_: NSString); message 'setEntityMigrationPolicyClassName:';
  end;

{$endif}
{$endif}
