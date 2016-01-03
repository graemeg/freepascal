{ Parsed from Appkit.framework NSPopUpButtonCell.h }

{$ifdef TYPES}
{$ifndef NSPOPUPBUTTONCELL_PAS_T}
{$define NSPOPUPBUTTONCELL_PAS_T}

{ Constants }

const
  NSPopUpNoArrow = 0;
  NSPopUpArrowAtCenter = 1;
  NSPopUpArrowAtBottom = 2;

{ Types }
type
  NSPopUpArrowPosition = NSUInteger;
  NSPopUpArrowPositionPtr = ^NSPopUpArrowPosition;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPOPUPBUTTONCELL_PAS_R}
{$define NSPOPUPBUTTONCELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPOPUPBUTTONCELL_PAS_F}
{$define NSPOPUPBUTTONCELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPOPUPBUTTONCELL_PAS_S}
{$define NSPOPUPBUTTONCELL_PAS_S}

{ External string constants }
var
  NSPopUpButtonCellWillPopUpNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPopUpButtonCell = objcclass;
  NSPopUpButtonCellPointer = ^NSPopUpButtonCell;
  NSPopUpButtonCellPtr = NSPopUpButtonCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPOPUPBUTTONCELL_PAS_C}
{$define NSPOPUPBUTTONCELL_PAS_C}

{ NSPopUpButtonCell }
  NSPopUpButtonCell = objcclass external (NSMenuItemCell)
  private
    _menu: NSMenu;
    _selectedIndex: cint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __pbcFlags: record
      case byte of
       0: (_anonbitfield_NSPopUpButtonCell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __pbcFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSPopUpButtonCell0: cuint);
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
        _pullsDown: 0..1;
        _preferredEdge: 0..((1 shl 3)-1);
        menuIsAttached: 0..1;
        _usesItemFromMenu: 0..1;
        _altersStateOfSelectedItem: 0..1;
        decoding: 0..1;
        _arrowPosition: 0..((1 shl 2)-1);
        ignoreMenuLayout: 0..1;
        drawing: 0..1;
        menuShouldBeUniquedAgainstMain: 0..1;
        RESERVED: 0..((1 shl 19)-1);
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
    {$ifdef cpu64}
    _popupReserved: id;
    {$endif}
    
  public
    function initTextCell_pullsDown(stringValue_: NSString; pullDown: Boolean): id; message 'initTextCell:pullsDown:';
    procedure setMenu(menu_: NSMenu); message 'setMenu:';
    function menu: NSMenu; message 'menu';
    procedure setPullsDown(flag: Boolean); message 'setPullsDown:';
    function pullsDown: Boolean; message 'pullsDown';
    procedure setAutoenablesItems(flag: Boolean); message 'setAutoenablesItems:';
    function autoenablesItems: Boolean; message 'autoenablesItems';
    procedure setPreferredEdge(edge: NSRectEdge); message 'setPreferredEdge:';
    function preferredEdge: NSRectEdge; message 'preferredEdge';
    procedure setUsesItemFromMenu(flag: Boolean); message 'setUsesItemFromMenu:';
    function usesItemFromMenu: Boolean; message 'usesItemFromMenu';
    procedure setAltersStateOfSelectedItem(flag: Boolean); message 'setAltersStateOfSelectedItem:';
    function altersStateOfSelectedItem: Boolean; message 'altersStateOfSelectedItem';
    procedure addItemWithTitle(title_: NSString); message 'addItemWithTitle:';
    procedure addItemsWithTitles(itemTitles_: NSArray); message 'addItemsWithTitles:';
    procedure insertItemWithTitle_atIndex(title_: NSString; index: NSInteger); message 'insertItemWithTitle:atIndex:';
    procedure removeItemWithTitle(title_: NSString); message 'removeItemWithTitle:';
    procedure removeItemAtIndex(index: NSInteger); message 'removeItemAtIndex:';
    procedure removeAllItems; message 'removeAllItems';
    function itemArray: NSArray; message 'itemArray';
    function numberOfItems: NSInteger; message 'numberOfItems';
    function indexOfItem(item: NSMenuItem): NSInteger; message 'indexOfItem:';
    function indexOfItemWithTitle(title_: NSString): NSInteger; message 'indexOfItemWithTitle:';
    function indexOfItemWithTag(tag_: NSInteger): NSInteger; message 'indexOfItemWithTag:';
    function indexOfItemWithRepresentedObject(obj: id): NSInteger; message 'indexOfItemWithRepresentedObject:';
    function indexOfItemWithTarget_andAction(target_: id; actionSelector: SEL): NSInteger; message 'indexOfItemWithTarget:andAction:';
    function itemAtIndex(index: NSInteger): NSMenuItem; message 'itemAtIndex:';
    function itemWithTitle(title_: NSString): NSMenuItem; message 'itemWithTitle:';
    function lastItem: NSMenuItem; message 'lastItem';
    procedure selectItem(item: NSMenuItem); message 'selectItem:';
    procedure selectItemAtIndex(index: NSInteger); message 'selectItemAtIndex:';
    procedure selectItemWithTitle(title_: NSString); message 'selectItemWithTitle:';
    function selectItemWithTag(tag_: NSInteger): Boolean; message 'selectItemWithTag:';
    procedure setTitle(aString: NSString); message 'setTitle:';
    function selectedItem: NSMenuItem; message 'selectedItem';
    function indexOfSelectedItem: NSInteger; message 'indexOfSelectedItem';
    procedure synchronizeTitleAndSelectedItem; message 'synchronizeTitleAndSelectedItem';
    function itemTitleAtIndex(index: NSInteger): NSString; message 'itemTitleAtIndex:';
    function itemTitles: NSArray; message 'itemTitles';
    function titleOfSelectedItem: NSString; message 'titleOfSelectedItem';
    procedure attachPopUpWithFrame_inView(cellFrame: NSRect; controlView_: NSView); message 'attachPopUpWithFrame:inView:';
    procedure dismissPopUp; message 'dismissPopUp';
    procedure performClickWithFrame_inView(frame: NSRect; controlView_: NSView); message 'performClickWithFrame:inView:';
    function arrowPosition: NSPopUpArrowPosition; message 'arrowPosition';
    procedure setArrowPosition(position: NSPopUpArrowPosition); message 'setArrowPosition:';
    function objectValue: id; message 'objectValue';
    procedure setObjectValue(obj: id); message 'setObjectValue:';
  end;

{$endif}
{$endif}
