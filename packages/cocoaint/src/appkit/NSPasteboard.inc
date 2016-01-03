{ Parsed from Appkit.framework NSPasteboard.h }

{$ifdef TYPES}
{$ifndef NSPASTEBOARD_PAS_T}
{$define NSPASTEBOARD_PAS_T}

{ Constants }

const
  NSPasteboardWritingPromised = 1 shl 9;

const
  NSPasteboardReadingAsData = 0;
  NSPasteboardReadingAsString = 1 shl 0;
  NSPasteboardReadingAsPropertyList = 1 shl 1;
  NSPasteboardReadingAsKeyedArchive = 1 shl 2;

{ Types }
type
  NSPasteboardWritingOptions = NSUInteger;
  NSPasteboardWritingOptionsPtr = ^NSPasteboardWritingOptions;
  NSPasteboardReadingOptions = NSUInteger;
  NSPasteboardReadingOptionsPtr = ^NSPasteboardReadingOptions;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPASTEBOARD_PAS_R}
{$define NSPASTEBOARD_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPASTEBOARD_PAS_F}
{$define NSPASTEBOARD_PAS_F}

{ Functions }
function NSCreateFilenamePboardType(fileType: NSString): NSString; cdecl; external;
function NSCreateFileContentsPboardType(fileType: NSString): NSString; cdecl; external;
function NSGetFileType(pboardType: NSString): NSString; cdecl; external;
function NSGetFileTypes(pboardTypes: NSArray): NSArray; cdecl; external;

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPASTEBOARD_PAS_S}
{$define NSPASTEBOARD_PAS_S}

{ External string constants }
var
  NSPasteboardTypeString: NSString; cvar; external;
  NSPasteboardTypePDF: NSString; cvar; external;
  NSPasteboardTypeTIFF: NSString; cvar; external;
  NSPasteboardTypePNG: NSString; cvar; external;
  NSPasteboardTypeRTF: NSString; cvar; external;
  NSPasteboardTypeRTFD: NSString; cvar; external;
  NSPasteboardTypeHTML: NSString; cvar; external;
  NSPasteboardTypeTabularText: NSString; cvar; external;
  NSPasteboardTypeFont: NSString; cvar; external;
  NSPasteboardTypeRuler: NSString; cvar; external;
  NSPasteboardTypeColor: NSString; cvar; external;
  NSPasteboardTypeSound: NSString; cvar; external;
  NSPasteboardTypeMultipleTextSelection: NSString; cvar; external;
  NSPasteboardTypeFindPanelSearchOptions: NSString; cvar; external;
  NSGeneralPboard: NSString; cvar; external;
  NSFontPboard: NSString; cvar; external;
  NSRulerPboard: NSString; cvar; external;
  NSFindPboard: NSString; cvar; external;
  NSDragPboard: NSString; cvar; external;
  NSPasteboardURLReadingFileURLsOnlyKey: NSString; cvar; external;
  NSPasteboardURLReadingContentsConformToTypesKey: NSString; cvar; external;
  NSStringPboardType: NSString; cvar; external;
  NSFilenamesPboardType: NSString; cvar; external;
  NSTIFFPboardType: NSString; cvar; external;
  NSRTFPboardType: NSString; cvar; external;
  NSTabularTextPboardType: NSString; cvar; external;
  NSFontPboardType: NSString; cvar; external;
  NSRulerPboardType: NSString; cvar; external;
  NSColorPboardType: NSString; cvar; external;
  NSRTFDPboardType: NSString; cvar; external;
  NSHTMLPboardType: NSString; cvar; external;
  NSURLPboardType: NSString; cvar; external;
  NSPDFPboardType: NSString; cvar; external;
  NSMultipleTextSelectionPboardType: NSString; cvar; external;
  NSPostScriptPboardType: NSString; cvar; external;
  NSVCardPboardType: NSString; cvar; external;
  NSInkTextPboardType: NSString; cvar; external;
  NSFilesPromisePboardType: NSString; cvar; external;
  NSFileContentsPboardType: NSString; cvar; external;
  NSPICTPboardType: NSString deprecated 'in Mac OS X 10.6 and later'; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPasteboardWritingProtocol = objcprotocol;
  NSPasteboardReadingProtocol = objcprotocol;
  NSPasteboard = objcclass;
  NSPasteboardPointer = ^NSPasteboard;
  NSPasteboardPtr = NSPasteboardPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPASTEBOARD_PAS_C}
{$define NSPASTEBOARD_PAS_C}

{ NSPasteboard }
  NSPasteboard = objcclass external (NSObject)
  private
    _pboard: id;
    _gen: cint;
    _owners: id;
    _cachedTypeNameChangeCount: CFIndex;
    _cachedTypeNames: id;
    _promiseTypeNamesByIdentifier: NSMutableDictionary;
    _support: id;
    _pasteboardItems: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(3)-1] of Pointer;
=======
        _reserved: array[0..2] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..2] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..2] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..2] of Pointer;
>>>>>>> origin/cpstrnew
    
  public
    class function generalPasteboard: NSPasteboard; message 'generalPasteboard';
    class function pasteboardWithName(name_: NSString): NSPasteboard; message 'pasteboardWithName:';
    class function pasteboardWithUniqueName: NSPasteboard; message 'pasteboardWithUniqueName';
    function name: NSString; message 'name';
    function changeCount: NSInteger; message 'changeCount';
    procedure releaseGlobally; message 'releaseGlobally';
    function clearContents: NSInteger; message 'clearContents';
    function writeObjects(objects: NSArray): Boolean; message 'writeObjects:';
    function readObjectsForClasses_options(classArray: NSArray; options: NSDictionary): NSArray; message 'readObjectsForClasses:options:';
    function pasteboardItems: NSArray; message 'pasteboardItems';
    function indexOfPasteboardItem(pasteboardItem: NSPasteboardItem): NSUInteger; message 'indexOfPasteboardItem:';
    function canReadItemWithDataConformingToTypes(types_: NSArray): Boolean; message 'canReadItemWithDataConformingToTypes:';
    function canReadObjectForClasses_options(classArray: NSArray; options: NSDictionary): Boolean; message 'canReadObjectForClasses:options:';
    function declareTypes_owner(newTypes: NSArray; newOwner: id): NSInteger; message 'declareTypes:owner:';
    function addTypes_owner(newTypes: NSArray; newOwner: id): NSInteger; message 'addTypes:owner:';
    function types: NSArray; message 'types';
    function availableTypeFromArray(types_: NSArray): NSString; message 'availableTypeFromArray:';
    function setData_forType(data: NSData; dataType: NSString): Boolean; message 'setData:forType:';
    function setPropertyList_forType(plist: id; dataType: NSString): Boolean; message 'setPropertyList:forType:';
    function setString_forType(string_: NSString; dataType: NSString): Boolean; message 'setString:forType:';
    function dataForType(dataType: NSString): NSData; message 'dataForType:';
    function propertyListForType(dataType: NSString): id; message 'propertyListForType:';
    function stringForType(dataType: NSString): NSString; message 'stringForType:';
  end;

{ FilterServicesCategory }
  FilterServicesCategory = objccategory external (NSPasteboard)
    class function typesFilterableTo(type_: NSString): NSArray; message 'typesFilterableTo:';
    class function pasteboardByFilteringFile(filename: NSString): NSPasteboard; message 'pasteboardByFilteringFile:';
    class function pasteboardByFilteringData_ofType(data: NSData; type_: NSString): NSPasteboard; message 'pasteboardByFilteringData:ofType:';
    class function pasteboardByFilteringTypesInPasteboard(pboard: NSPasteboard): NSPasteboard; message 'pasteboardByFilteringTypesInPasteboard:';
  end;

{ NSPasteboardOwnerCategory }
  NSPasteboardOwnerCategory = objccategory external (NSObject)
    procedure pasteboard_provideDataForType(sender: NSPasteboard; type_: NSString); message 'pasteboard:provideDataForType:';
    procedure pasteboardChangedOwner(sender: NSPasteboard); message 'pasteboardChangedOwner:';
  end;

{ NSPasteboardSupportCategory }
  NSPasteboardSupportCategory = objccategory external (NSURL)
    class function URLFromPasteboard(pasteBoard: NSPasteboard): NSURL; message 'URLFromPasteboard:';
    procedure writeToPasteboard(pasteBoard: NSPasteboard); message 'writeToPasteboard:';
  end;

{ NSPasteboardSupport_NSStringCategory }
  NSPasteboardSupport_NSStringCategory = objccategory external name 'NSPasteboardSupport' (NSString)
  end;

{ NSFileContentsCategory }
  NSFileContentsCategory = objccategory external (NSPasteboard)
    function writeFileContents(filename: NSString): Boolean; message 'writeFileContents:';
    function readFileContentsType_toFile(type_: NSString; filename: NSString): NSString; message 'readFileContentsType:toFile:';
    function writeFileWrapper(wrapper: NSFileWrapper): Boolean; message 'writeFileWrapper:';
    function readFileWrapper: NSFileWrapper; message 'readFileWrapper';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSPASTEBOARD_PAS_P}
{$define NSPASTEBOARD_PAS_P}
  
{ NSPasteboardWriting Protocol }
  NSPasteboardWritingProtocol = objcprotocol external name 'NSPasteboardWriting'
  required
    function writableTypesForPasteboard(pasteboard: NSPasteboard): NSArray; message 'writableTypesForPasteboard:';
  optional
    function writingOptionsForType_pasteboard(type_: NSString; pasteboard: NSPasteboard): NSPasteboardWritingOptions; message 'writingOptionsForType:pasteboard:';
  required
    function pasteboardPropertyListForType(type_: NSString): id; message 'pasteboardPropertyListForType:';
  end;
  
{ NSPasteboardReading Protocol }
  NSPasteboardReadingProtocol = objcprotocol external name 'NSPasteboardReading'
  required
    class function readableTypesForPasteboard(pasteboard: NSPasteboard): NSArray; message 'readableTypesForPasteboard:';
  optional
    class function readingOptionsForType_pasteboard(type_: NSString; pasteboard: NSPasteboard): NSPasteboardReadingOptions; message 'readingOptionsForType:pasteboard:';
    function initWithPasteboardPropertyList_ofType(propertyList: id; type_: NSString): id; message 'initWithPasteboardPropertyList:ofType:';
  end;
{$endif}
{$endif}
