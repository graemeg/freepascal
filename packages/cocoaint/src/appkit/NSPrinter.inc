{ Parsed from Appkit.framework NSPrinter.h }

{$ifdef TYPES}
{$ifndef NSPRINTER_PAS_T}
{$define NSPRINTER_PAS_T}

{ Constants }

const
  NSPrinterTableOK = 0;
  NSPrinterTableNotFound = 1;
  NSPrinterTableError = 2;

{ Types }
type
  NSPrinterTableStatus = NSUInteger;
  NSPrinterTableStatusPtr = ^NSPrinterTableStatus;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPRINTER_PAS_R}
{$define NSPRINTER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPRINTER_PAS_F}
{$define NSPRINTER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPRINTER_PAS_S}
{$define NSPRINTER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPrinter = objcclass;
  NSPrinterPointer = ^NSPrinter;
  NSPrinterPtr = NSPrinterPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPRINTER_PAS_C}
{$define NSPRINTER_PAS_C}

{ NSPrinter }
  NSPrinter = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol)
  private
    _printerName: NSString;
    _printer: Pointer;
    _cachedDeviceDescription: NSDictionary;
    _ppdCreationNum: cint;
    _ppdNodes: Pointer;
    _ppdPriv: Pointer;
    {$ifdef cpu64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(3)-1] of id;
    {$else}
    _compatibilityPadding: array[0..(20)-1] of char;
=======
        _reserved: array[0..2] of id;
    {$else}
        _compatibilityPadding: array[0..19] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..2] of id;
    {$else}
        _compatibilityPadding: array[0..19] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..2] of id;
    {$else}
        _compatibilityPadding: array[0..19] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..2] of id;
    {$else}
        _compatibilityPadding: array[0..19] of char;
>>>>>>> origin/cpstrnew
    {$endif}
    
  public
    class function printerNames: NSArray; message 'printerNames';
    class function printerTypes: NSArray; message 'printerTypes';
    class function printerWithName(name_: NSString): NSPrinter; message 'printerWithName:';
    class function printerWithType(type__: NSString): NSPrinter; message 'printerWithType:';
    function name: NSString; message 'name';
    function type_: NSString; message 'type';
    function languageLevel: NSInteger; message 'languageLevel';
    function pageSizeForPaper(paperName: NSString): NSSize; message 'pageSizeForPaper:';
    function statusForTable(tableName: NSString): NSPrinterTableStatus; message 'statusForTable:';
    function isKey_inTable(key: NSString; table: NSString): Boolean; message 'isKey:inTable:';
    function booleanForKey_inTable(key: NSString; table: NSString): Boolean; message 'booleanForKey:inTable:';
    function floatForKey_inTable(key: NSString; table: NSString): single; message 'floatForKey:inTable:';
    function intForKey_inTable(key: NSString; table: NSString): cint; message 'intForKey:inTable:';
    function rectForKey_inTable(key: NSString; table: NSString): NSRect; message 'rectForKey:inTable:';
    function sizeForKey_inTable(key: NSString; table: NSString): NSSize; message 'sizeForKey:inTable:';
    function stringForKey_inTable(key: NSString; table: NSString): NSString; message 'stringForKey:inTable:';
    function stringListForKey_inTable(key: NSString; table: NSString): NSArray; message 'stringListForKey:inTable:';
    function deviceDescription: NSDictionary; message 'deviceDescription';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSDeprecated_NSPrinterCategory }
  NSDeprecated_NSPrinterCategory = objccategory external name 'NSDeprecated' (NSPrinter)
    function imageRectForPaper(paperName: NSString): NSRect; message 'imageRectForPaper:'; deprecated 'in Mac OS X 10.2 and later';
    function acceptsBinary: Boolean; message 'acceptsBinary'; deprecated 'in Mac OS X 10.2 and later';
    function isColor: Boolean; message 'isColor'; deprecated 'in Mac OS X 10.2 and later';
    function isFontAvailable(faceName: NSString): Boolean; message 'isFontAvailable:'; deprecated 'in Mac OS X 10.2 and later';
    function isOutputStackInReverseOrder: Boolean; message 'isOutputStackInReverseOrder'; deprecated 'in Mac OS X 10.2 and later';
    class function printerWithName_domain_includeUnavailable(name_: NSString; domain_: NSString; flag: Boolean): NSPrinter; message 'printerWithName:domain:includeUnavailable:'; deprecated 'in Mac OS X 10.2 and later';
    function domain: NSString; message 'domain'; deprecated 'in Mac OS X 10.2 and later';
    function host: NSString; message 'host'; deprecated 'in Mac OS X 10.2 and later';
    function note: NSString; message 'note'; deprecated 'in Mac OS X 10.2 and later';
  end;

{$endif}
{$endif}
