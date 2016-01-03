{ Parsed from Appkit.framework NSTokenFieldCell.h }

{$ifdef TYPES}
{$ifndef NSTOKENFIELDCELL_PAS_T}
{$define NSTOKENFIELDCELL_PAS_T}

{ Constants }

const
  NSDefaultTokenStyle = 0;
  NSPlainTextTokenStyle = 1;
  NSRoundedTokenStyle = 2;

{ Types }
type
  NSTokenStyle = NSUInteger;
  NSTokenStylePtr = ^NSTokenStyle;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTOKENFIELDCELL_PAS_R}
{$define NSTOKENFIELDCELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTOKENFIELDCELL_PAS_F}
{$define NSTOKENFIELDCELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTOKENFIELDCELL_PAS_S}
{$define NSTOKENFIELDCELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTokenFieldCellDelegateProtocol = objcprotocol;
  NSTokenFieldCell = objcclass;
  NSTokenFieldCellPointer = ^NSTokenFieldCell;
  NSTokenFieldCellPtr = NSTokenFieldCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTOKENFIELDCELL_PAS_C}
{$define NSTOKENFIELDCELL_PAS_C}

{ NSTokenFieldCell }
  NSTokenFieldCell = objcclass external (NSTextFieldCell)
  private
    _tokenizingCharacterSet: NSCharacterSet;
    _delegate: id;
    _completionDelay: NSTimeInterval;
    _cache: id;
    _defaultTerminator: id;
    _trackingArea: id;
    _lastCell: id;
    _lastCellFrame: NSRect;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(7)-1] of id;
    __tfcFlags: record
      case byte of
       0: (_anonbitfield_NSTokenFieldCell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        _reserved: array[0..6] of id;
    __tfcFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSTokenFieldCell0: cuint);
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
        _style: 0..((1 shl 4)-1);
        
        _invalidCache: 0..1;
        _inDidChange: 0..1;
        _validationDisabled: 0..1;
        _pendingComplete: 0..1;
        _autoCompleteMode: 0..((1 shl 2)-1);
        _inValidateEditing: 0..1;
        _performingDrop: 0..1;
        
        __reserved: 0..((1 shl 20)-1);
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
    
  public
    procedure setTokenStyle(style: NSTokenStyle); message 'setTokenStyle:';
    function tokenStyle: NSTokenStyle; message 'tokenStyle';
    procedure setCompletionDelay(delay: NSTimeInterval); message 'setCompletionDelay:';
    function completionDelay: NSTimeInterval; message 'completionDelay';
    class function defaultCompletionDelay: NSTimeInterval; message 'defaultCompletionDelay';
    procedure setTokenizingCharacterSet(characterSet: NSCharacterSet); message 'setTokenizingCharacterSet:';
    function tokenizingCharacterSet: NSCharacterSet; message 'tokenizingCharacterSet';
    class function defaultTokenizingCharacterSet: NSCharacterSet; message 'defaultTokenizingCharacterSet';
    procedure setDelegate(anObject: NSTokenFieldCellDelegateProtocol); message 'setDelegate:';
    function delegate: NSTokenFieldCellDelegateProtocol; message 'delegate';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSTOKENFIELDCELL_PAS_P}
{$define NSTOKENFIELDCELL_PAS_P}
  
{ NSTokenFieldCellDelegate Protocol }
  NSTokenFieldCellDelegateProtocol = objcprotocol external name 'NSTokenFieldCellDelegate'
  optional
    function tokenFieldCell_completionsForSubstring_indexOfToken_indexOfSelectedItem(tokenFieldCell: NSTokenFieldCell; substring: NSString; tokenIndex: NSInteger; selectedIndex: NSIntegerPtr): NSArray; message 'tokenFieldCell:completionsForSubstring:indexOfToken:indexOfSelectedItem:';
    function tokenFieldCell_shouldAddObjects_atIndex(tokenFieldCell: NSTokenFieldCell; tokens: NSArray; index: NSUInteger): NSArray; message 'tokenFieldCell:shouldAddObjects:atIndex:';
    function tokenFieldCell_displayStringForRepresentedObject(tokenFieldCell: NSTokenFieldCell; representedObject: id): NSString; message 'tokenFieldCell:displayStringForRepresentedObject:';
    function tokenFieldCell_editingStringForRepresentedObject(tokenFieldCell: NSTokenFieldCell; representedObject: id): NSString; message 'tokenFieldCell:editingStringForRepresentedObject:';
    function tokenFieldCell_representedObjectForEditingString(tokenFieldCell: NSTokenFieldCell; editingString: NSString): id; message 'tokenFieldCell:representedObjectForEditingString:';
    function tokenFieldCell_writeRepresentedObjects_toPasteboard(tokenFieldCell: NSTokenFieldCell; objects: NSArray; pboard: NSPasteboard): Boolean; message 'tokenFieldCell:writeRepresentedObjects:toPasteboard:';
    function tokenFieldCell_readFromPasteboard(tokenFieldCell: NSTokenFieldCell; pboard: NSPasteboard): NSArray; message 'tokenFieldCell:readFromPasteboard:';
    function tokenFieldCell_menuForRepresentedObject(tokenFieldCell: NSTokenFieldCell; representedObject: id): NSMenu; message 'tokenFieldCell:menuForRepresentedObject:';
    function tokenFieldCell_hasMenuForRepresentedObject(tokenFieldCell: NSTokenFieldCell; representedObject: id): Boolean; message 'tokenFieldCell:hasMenuForRepresentedObject:';
    function tokenFieldCell_styleForRepresentedObject(tokenFieldCell: NSTokenFieldCell; representedObject: id): NSTokenStyle; message 'tokenFieldCell:styleForRepresentedObject:';
  end;
{$endif}
{$endif}
