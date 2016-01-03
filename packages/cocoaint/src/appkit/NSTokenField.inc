{ Parsed from Appkit.framework NSTokenField.h }

{$ifdef TYPES}
{$ifndef NSTOKENFIELD_PAS_T}
{$define NSTOKENFIELD_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTOKENFIELD_PAS_R}
{$define NSTOKENFIELD_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTOKENFIELD_PAS_F}
{$define NSTOKENFIELD_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTOKENFIELD_PAS_S}
{$define NSTOKENFIELD_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTokenFieldDelegateProtocol = objcprotocol;
  NSTokenField = objcclass;
  NSTokenFieldPointer = ^NSTokenField;
  NSTokenFieldPtr = NSTokenFieldPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTOKENFIELD_PAS_C}
{$define NSTOKENFIELD_PAS_C}

{ NSTokenField }
  NSTokenField = objcclass external (NSTextField)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  private
    _reserved1: Boolean;
    _reserved2: Boolean;
    _reserved3: Boolean;
    _reserved4: Boolean;
    _trackingRectTag: NSTrackingRectTag;
    _reserved5: id;
    _reserved6: id;
    _reserved7: id;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    
  public
    procedure setDelegate(anObject: NSTokenFieldDelegateProtocol); message 'setDelegate:';
    function delegate: NSTokenFieldDelegateProtocol; message 'delegate';
    procedure setTokenStyle(style: NSTokenStyle); message 'setTokenStyle:';
    function tokenStyle: NSTokenStyle; message 'tokenStyle';
    procedure setCompletionDelay(delay: NSTimeInterval); message 'setCompletionDelay:';
    function completionDelay: NSTimeInterval; message 'completionDelay';
    class function defaultCompletionDelay: NSTimeInterval; message 'defaultCompletionDelay';
    procedure setTokenizingCharacterSet(characterSet: NSCharacterSet); message 'setTokenizingCharacterSet:';
    function tokenizingCharacterSet: NSCharacterSet; message 'tokenizingCharacterSet';
    class function defaultTokenizingCharacterSet: NSCharacterSet; message 'defaultTokenizingCharacterSet';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSTOKENFIELD_PAS_P}
{$define NSTOKENFIELD_PAS_P}
  
{ NSTokenFieldDelegate Protocol }
  NSTokenFieldDelegateProtocol = objcprotocol external name 'NSTokenFieldDelegate'
  optional
    function tokenField_completionsForSubstring_indexOfToken_indexOfSelectedItem(tokenField: NSTokenField; substring: NSString; tokenIndex: NSInteger; selectedIndex: NSIntegerPtr): NSArray; message 'tokenField:completionsForSubstring:indexOfToken:indexOfSelectedItem:';
    function tokenField_shouldAddObjects_atIndex(tokenField: NSTokenField; tokens: NSArray; index: NSUInteger): NSArray; message 'tokenField:shouldAddObjects:atIndex:';
    function tokenField_displayStringForRepresentedObject(tokenField: NSTokenField; representedObject: id): NSString; message 'tokenField:displayStringForRepresentedObject:';
    function tokenField_editingStringForRepresentedObject(tokenField: NSTokenField; representedObject: id): NSString; message 'tokenField:editingStringForRepresentedObject:';
    function tokenField_representedObjectForEditingString(tokenField: NSTokenField; editingString: NSString): id; message 'tokenField:representedObjectForEditingString:';
    function tokenField_writeRepresentedObjects_toPasteboard(tokenField: NSTokenField; objects: NSArray; pboard: NSPasteboard): Boolean; message 'tokenField:writeRepresentedObjects:toPasteboard:';
    function tokenField_readFromPasteboard(tokenField: NSTokenField; pboard: NSPasteboard): NSArray; message 'tokenField:readFromPasteboard:';
    function tokenField_menuForRepresentedObject(tokenField: NSTokenField; representedObject: id): NSMenu; message 'tokenField:menuForRepresentedObject:';
    function tokenField_hasMenuForRepresentedObject(tokenField: NSTokenField; representedObject: id): Boolean; message 'tokenField:hasMenuForRepresentedObject:';
    function tokenField_styleForRepresentedObject(tokenField: NSTokenField; representedObject: id): NSTokenStyle; message 'tokenField:styleForRepresentedObject:';
  end;
{$endif}
{$endif}
