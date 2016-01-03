{ Parsed from Appkit.framework NSDictionaryController.h }

{$ifdef TYPES}
{$ifndef NSDICTIONARYCONTROLLER_PAS_T}
{$define NSDICTIONARYCONTROLLER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSDICTIONARYCONTROLLER_PAS_R}
{$define NSDICTIONARYCONTROLLER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSDICTIONARYCONTROLLER_PAS_F}
{$define NSDICTIONARYCONTROLLER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSDICTIONARYCONTROLLER_PAS_S}
{$define NSDICTIONARYCONTROLLER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSDictionaryController = objcclass;
  NSDictionaryControllerPointer = ^NSDictionaryController;
  NSDictionaryControllerPtr = NSDictionaryControllerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSDICTIONARYCONTROLLER_PAS_C}
{$define NSDICTIONARYCONTROLLER_PAS_C}

{ NSDictionaryController }
  NSDictionaryController = objcclass external (NSArrayController)
  private
    _reserved5: Pointer;
    _reserved6: Pointer;
    _reserved7: Pointer;
    _contentDictionary: id;
    _initialKey: NSString;
    _initialValue: id;
    _minimumInsertionKeyIndex: NSUInteger;
    _localizedKeyStringsFileName: NSString;
    _localizedKeyForKeyDictionary: NSDictionary;
    _keyForLocalizedKeyDictionary: NSDictionary;
    _includedKeys: NSArray;
    _excludedKeys: NSArray;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __dictionaryControllerFlags: record
      case byte of
       0: (_anonbitfield_NSDictionaryController0: cuint);
       1: (data: bitpacked record
        _deepCopiesValues: 0..1;
        _suppressBuildingDictionary: 0..1;
        _reservedDictionaryController: 0..((1 shl 30)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __dictionaryControllerFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSDictionaryController0: cuint);
       1: (
        _deepCopiesValues: 0..1;
        _suppressBuildingDictionary: 0..1;
        _reservedDictionaryController: 0..((1 shl 30)-1);
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
    function newObject: id; message 'newObject';
    procedure setInitialKey(key: NSString); message 'setInitialKey:';
    function initialKey: NSString; message 'initialKey';
    procedure setInitialValue(value: id); message 'setInitialValue:';
    function initialValue: id; message 'initialValue';
    procedure setIncludedKeys(keys: NSArray); message 'setIncludedKeys:';
    function includedKeys: NSArray; message 'includedKeys';
    procedure setExcludedKeys(keys: NSArray); message 'setExcludedKeys:';
    function excludedKeys: NSArray; message 'excludedKeys';
    procedure setLocalizedKeyDictionary(dictionary: NSDictionary); message 'setLocalizedKeyDictionary:';
    function localizedKeyDictionary: NSDictionary; message 'localizedKeyDictionary';
    procedure setLocalizedKeyTable(stringsFileName: NSString); message 'setLocalizedKeyTable:';
    function localizedKeyTable: NSString; message 'localizedKeyTable';
  end;

{ NSDictionaryControllerKeyValuePairCategory }
  NSDictionaryControllerKeyValuePairCategory = objccategory external (NSObject)
    procedure setLocalizedKey(localizedKey: NSString); message 'setLocalizedKey:';
    function localizedKey: NSString; message 'localizedKey';
    procedure setKey(key: NSString); message 'setKey:';
    function key: NSString; message 'key';
    procedure setValue(value: id); message 'setValue:';
    function value: id; message 'value';
    function isExplicitlyIncluded: Boolean; message 'isExplicitlyIncluded';
  end;

{$endif}
{$endif}
