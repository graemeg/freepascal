{ Parsed from Appkit.framework NSColorList.h }

{$ifdef TYPES}
{$ifndef NSCOLORLIST_PAS_T}
{$define NSCOLORLIST_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSCOLORLIST_PAS_R}
{$define NSCOLORLIST_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSCOLORLIST_PAS_F}
{$define NSCOLORLIST_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSCOLORLIST_PAS_S}
{$define NSCOLORLIST_PAS_S}

{ External string constants }
var
  NSColorListDidChangeNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSColorList = objcclass;
  NSColorListPointer = ^NSColorList;
  NSColorListPtr = NSColorListPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSCOLORLIST_PAS_C}
{$define NSCOLORLIST_PAS_C}

{ NSColorList }
  NSColorList = objcclass external (NSObject, NSCodingProtocol)
  private
    _keyArray: NSMutableArray;
    _colorArray: NSMutableArray;
    _keyToIndexTable: CFMutableDictionaryRef;
    _name: NSString;
    _printerType: NSString;
    _fileName: NSString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSColorList2: cuint);
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
       0: (_anonbitfield_NSColorList2: cuint);
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
        colorsLoaded: 0..1;
        editable: 0..1;
        hasDeviceSpecificLists: 0..1;
        dirty: 0..1;
        hasFrozen: 0..1;
        notificationsDisabled: 0..1;
        hasAttemptedLoadingBundleForDirectory: 0..1;
        isProfileBased: 0..1;
        _anonbitfield_NSColorList0: 0..((1 shl 24)-1);
        {$ifdef cpu64}
        _anonbitfield_NSColorList1: 0..((1 shl 32)-1);
        {$endif}
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
    _clAuxiliaryStorage: id;
    
  public
    class function availableColorLists: NSArray; message 'availableColorLists';
    class function colorListNamed(name_: NSString): NSColorList; message 'colorListNamed:';
    function initWithName(name_: NSString): id; message 'initWithName:';
    function initWithName_fromFile(name_: NSString; path: NSString): id; message 'initWithName:fromFile:';
    function name: NSString; message 'name';
    procedure setColor_forKey(color: NSColor; key: NSString); message 'setColor:forKey:';
    procedure insertColor_key_atIndex(color: NSColor; key: NSString; loc: NSUInteger); message 'insertColor:key:atIndex:';
    procedure removeColorWithKey(key: NSString); message 'removeColorWithKey:';
    function colorWithKey(key: NSString): NSColor; message 'colorWithKey:';
    function allKeys: NSArray; message 'allKeys';
    function isEditable: Boolean; message 'isEditable';
    function writeToFile(path: NSString): Boolean; message 'writeToFile:';
    procedure removeFile; message 'removeFile';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
