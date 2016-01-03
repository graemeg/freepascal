{ Parsed from Appkit.framework NSSearchFieldCell.h }

{$ifdef TYPES}
{$ifndef NSSEARCHFIELDCELL_PAS_T}
{$define NSSEARCHFIELDCELL_PAS_T}

{ Defines }
const
  NSSearchFieldRecentsTitleMenuItemTag = 1000;
  NSSearchFieldRecentsMenuItemTag = 1001;
  NSSearchFieldClearRecentsMenuItemTag = 1002;
  NSSearchFieldNoRecentsMenuItemTag = 1003;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSSEARCHFIELDCELL_PAS_R}
{$define NSSEARCHFIELDCELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSSEARCHFIELDCELL_PAS_F}
{$define NSSEARCHFIELDCELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSSEARCHFIELDCELL_PAS_S}
{$define NSSEARCHFIELDCELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSSearchFieldCell = objcclass;
  NSSearchFieldCellPointer = ^NSSearchFieldCell;
  NSSearchFieldCellPtr = NSSearchFieldCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSSEARCHFIELDCELL_PAS_C}
{$define NSSEARCHFIELDCELL_PAS_C}

{ NSSearchFieldCell }
  NSSearchFieldCell = objcclass external (NSTextFieldCell)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __sfFlags: record
      case byte of
       0: (_anonbitfield_NSSearchFieldCell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __sfFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSSearchFieldCell0: cuint);
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
        _sendsWholeSearchString: 0..1;
        _maximumRecents: 0..((1 shl 8)-1);
        cancelVisible: 0..1;
        reserved2: 0..((1 shl 2)-1);
        disableText: 0..1;
        menuTracking: 0..1;
        deferredUpdate: 0..1;
        sendsImmediately: 0..1;
        
        reserved: 0..((1 shl 16)-1);
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
    _searchButtonCell: NSButtonCell;
    _cancelButtonCell: NSButtonCell;
    _searchMenuTemplate: NSMenu;
    _recentsAutosaveName: NSString;
    _recentSearches: NSMutableArray;
    _searchMenu: NSMenu;
    _partialStringTimer: NSTimer;
    _reserved1: cuint;
    _reserved2: cuint;
    _reserved3: cuint;
    _reserved4: cuint;
    
  public
    function searchButtonCell: NSButtonCell; message 'searchButtonCell';
    procedure setSearchButtonCell(cell: NSButtonCell); message 'setSearchButtonCell:';
    function cancelButtonCell: NSButtonCell; message 'cancelButtonCell';
    procedure setCancelButtonCell(cell: NSButtonCell); message 'setCancelButtonCell:';
    procedure resetSearchButtonCell; message 'resetSearchButtonCell';
    procedure resetCancelButtonCell; message 'resetCancelButtonCell';
    function searchTextRectForBounds(rect: NSRect): NSRect; message 'searchTextRectForBounds:';
    function searchButtonRectForBounds(rect: NSRect): NSRect; message 'searchButtonRectForBounds:';
    function cancelButtonRectForBounds(rect: NSRect): NSRect; message 'cancelButtonRectForBounds:';
    procedure setSearchMenuTemplate(menu_: NSMenu); message 'setSearchMenuTemplate:';
    function searchMenuTemplate: NSMenu; message 'searchMenuTemplate';
    procedure setSendsWholeSearchString(flag: Boolean); message 'setSendsWholeSearchString:';
    function sendsWholeSearchString: Boolean; message 'sendsWholeSearchString';
    procedure setMaximumRecents(maxRecents: NSInteger); message 'setMaximumRecents:';
    function maximumRecents: NSInteger; message 'maximumRecents';
    procedure setRecentSearches(searches: NSArray); message 'setRecentSearches:';
    function recentSearches: NSArray; message 'recentSearches';
    procedure setRecentsAutosaveName(string_: NSString); message 'setRecentsAutosaveName:';
    function recentsAutosaveName: NSString; message 'recentsAutosaveName';
    function sendsSearchStringImmediately: Boolean; message 'sendsSearchStringImmediately';
    procedure setSendsSearchStringImmediately(flag: Boolean); message 'setSendsSearchStringImmediately:';
  end;

{$endif}
{$endif}
