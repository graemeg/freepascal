{ Parsed from Coredata.framework NSPropertyMapping.h }

{$ifdef TYPES}
{$ifndef NSPROPERTYMAPPING_PAS_T}
{$define NSPROPERTYMAPPING_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPROPERTYMAPPING_PAS_R}
{$define NSPROPERTYMAPPING_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPROPERTYMAPPING_PAS_F}
{$define NSPROPERTYMAPPING_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPROPERTYMAPPING_PAS_S}
{$define NSPROPERTYMAPPING_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPropertyMapping = objcclass;
  NSPropertyMappingPointer = ^NSPropertyMapping;
  NSPropertyMappingPtr = NSPropertyMappingPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPROPERTYMAPPING_PAS_C}
{$define NSPROPERTYMAPPING_PAS_C}

{ NSPropertyMapping }
  NSPropertyMapping = objcclass external (NSObject)
  private
    _reserved: Pointer;
    _reserved1: Pointer;
    _propertyTransforms: NSArray;
    _name: NSString;
    _valueExpression: NSExpression;
    _userInfo: NSDictionary;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __propertyMappingFlags: record
      case byte of
       0: (_anonbitfield_NSPropertyMapping0: cuint);
       1: (data: bitpacked record
        _isInUse: 0..1;
        _reservedPropertyMapping: 0..((1 shl 31)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __propertyMappingFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSPropertyMapping0: cuint);
       1: (
        _isInUse: 0..1;
        _reservedPropertyMapping: 0..((1 shl 31)-1);
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
    function valueExpression: NSExpression; message 'valueExpression';
    procedure setValueExpression(expression: NSExpression); message 'setValueExpression:';
    function userInfo: NSDictionary; message 'userInfo';
    procedure setUserInfo(userInfo_: NSDictionary); message 'setUserInfo:';
  end;

{$endif}
{$endif}
