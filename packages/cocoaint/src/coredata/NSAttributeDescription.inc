{ Parsed from Coredata.framework NSAttributeDescription.h }

{$ifdef TYPES}
{$ifndef NSATTRIBUTEDESCRIPTION_PAS_T}
{$define NSATTRIBUTEDESCRIPTION_PAS_T}

{ Constants }

const
  NSUndefinedAttributeType = 0;
  NSInteger16AttributeType = 100;
  NSInteger32AttributeType = 200;
  NSInteger64AttributeType = 300;
  NSDecimalAttributeType = 400;
  NSDoubleAttributeType = 500;
  NSFloatAttributeType = 600;
  NSStringAttributeType = 700;
  NSBooleanAttributeType = 800;
  NSDateAttributeType = 900;
  NSBinaryDataAttributeType = 1000;
  NSTransformableAttributeType = 1800;
  NSObjectIDAttributeType = 2000;

{ Types }
type
  NSAttributeType = NSUInteger;
  NSAttributeTypePtr = ^NSAttributeType;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSATTRIBUTEDESCRIPTION_PAS_R}
{$define NSATTRIBUTEDESCRIPTION_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSATTRIBUTEDESCRIPTION_PAS_F}
{$define NSATTRIBUTEDESCRIPTION_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSATTRIBUTEDESCRIPTION_PAS_S}
{$define NSATTRIBUTEDESCRIPTION_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSAttributeDescription = objcclass;
  NSAttributeDescriptionPointer = ^NSAttributeDescription;
  NSAttributeDescriptionPtr = NSAttributeDescriptionPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSATTRIBUTEDESCRIPTION_PAS_C}
{$define NSATTRIBUTEDESCRIPTION_PAS_C}

{ NSAttributeDescription }
  NSAttributeDescription = objcclass external (NSPropertyDescription)
  private
    _attributeValueClass: Pobjc_class;
    _valueTransformerName: NSString;
    _type: NSAttributeType;
    _attributeValueClassName: NSString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __attributeDescriptionFlags: record
      case byte of
       0: (_anonbitfield_NSAttributeDescription0: cuint);
       1: (data: bitpacked record
        _useForLocking: 0..1;
        _reservedAttributeDescription: 0..((1 shl 31)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __attributeDescriptionFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSAttributeDescription0: cuint);
       1: (
        _useForLocking: 0..1;
        _reservedAttributeDescription: 0..((1 shl 31)-1);
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
    _defaultValue: id;
    
  public
    function attributeType: NSAttributeType; message 'attributeType';
    procedure setAttributeType(type_: NSAttributeType); message 'setAttributeType:';
    function attributeValueClassName: NSString; message 'attributeValueClassName';
    function defaultValue: id; message 'defaultValue';
    procedure setDefaultValue(value: id); message 'setDefaultValue:';
    procedure setAttributeValueClassName(className: NSString); message 'setAttributeValueClassName:';
    function versionHash: NSData; message 'versionHash';
    function valueTransformerName: NSString; message 'valueTransformerName';
    procedure setValueTransformerName(string_: NSString); message 'setValueTransformerName:';
  end;

{$endif}
{$endif}
