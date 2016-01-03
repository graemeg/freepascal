{ Parsed from Appkit.framework NSComboBoxCell.h }

{$ifdef TYPES}
{$ifndef NSCOMBOBOXCELL_PAS_T}
{$define NSCOMBOBOXCELL_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSCOMBOBOXCELL_PAS_R}
{$define NSCOMBOBOXCELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSCOMBOBOXCELL_PAS_F}
{$define NSCOMBOBOXCELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSCOMBOBOXCELL_PAS_S}
{$define NSCOMBOBOXCELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSComboBoxCellDataSourceProtocol = objcprotocol;
  NSComboBoxCell = objcclass;
  NSComboBoxCellPointer = ^NSComboBoxCell;
  NSComboBoxCellPtr = NSComboBoxCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSCOMBOBOXCELL_PAS_C}
{$define NSCOMBOBOXCELL_PAS_C}

{ NSComboBoxCell }
  NSComboBoxCell = objcclass external (NSTextFieldCell)
  private
    _delegate: id;
    _dataSource: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __cbcFlags: record
      case byte of
       0: (_anonbitfield_NSComboBoxCell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __cbcFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSComboBoxCell0: cuint);
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
        _usesDataSource: 0..1;
        _completes: 0..1;
        buttonBordered: 0..1;
        popUpIsUp: 0..1;
        filteringEvents: 0..1;
        drawing: 0..1;
        synchronizingSelection: 0..1;
        reserved: 0..((1 shl 9)-1);
        visibleItems: 0..((1 shl 16)-1);
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
    _buttonCell: NSButtonCell;
    _tableView: NSTableView;
    _scrollView: NSScrollView;
    _popUp: NSWindow;
    _popUpList: NSMutableArray;
    _cellFrame: NSRectPtr; {garbage collector: __strong }
    _reserved: Pointer;
    
  public
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function hasVerticalScroller: Boolean; message 'hasVerticalScroller';
    procedure setHasVerticalScroller(flag: Boolean); message 'setHasVerticalScroller:';
    function intercellSpacing: NSSize; message 'intercellSpacing';
    procedure setIntercellSpacing(aSize: NSSize); message 'setIntercellSpacing:';
    function itemHeight: CGFloat; message 'itemHeight';
    procedure setItemHeight(itemHeight_: CGFloat); message 'setItemHeight:';
    function numberOfVisibleItems: NSInteger; message 'numberOfVisibleItems';
    procedure setNumberOfVisibleItems(visibleItems: NSInteger); message 'setNumberOfVisibleItems:';
    procedure setButtonBordered(flag: Boolean); message 'setButtonBordered:';
    function isButtonBordered: Boolean; message 'isButtonBordered';
    procedure reloadData; message 'reloadData';
    procedure noteNumberOfItemsChanged; message 'noteNumberOfItemsChanged';
    procedure setUsesDataSource(flag: Boolean); message 'setUsesDataSource:';
    function usesDataSource: Boolean; message 'usesDataSource';
    procedure scrollItemAtIndexToTop(index: NSInteger); message 'scrollItemAtIndexToTop:';
    procedure scrollItemAtIndexToVisible(index: NSInteger); message 'scrollItemAtIndexToVisible:';
    procedure selectItemAtIndex(index: NSInteger); message 'selectItemAtIndex:';
    procedure deselectItemAtIndex(index: NSInteger); message 'deselectItemAtIndex:';
    function indexOfSelectedItem: NSInteger; message 'indexOfSelectedItem';
    function numberOfItems: NSInteger; message 'numberOfItems';
    function completes: Boolean; message 'completes';
    procedure setCompletes(completes_: Boolean); message 'setCompletes:';
    function completedString(string_: NSString): NSString; message 'completedString:';
    function dataSource: NSComboBoxCellDataSourceProtocol; message 'dataSource';
    procedure setDataSource(aSource: NSComboBoxCellDataSourceProtocol); message 'setDataSource:';
    procedure addItemWithObjectValue(object_: id); message 'addItemWithObjectValue:';
    procedure addItemsWithObjectValues(objects: NSArray); message 'addItemsWithObjectValues:';
    procedure insertItemWithObjectValue_atIndex(object_: id; index: NSInteger); message 'insertItemWithObjectValue:atIndex:';
    procedure removeItemWithObjectValue(object_: id); message 'removeItemWithObjectValue:';
    procedure removeItemAtIndex(index: NSInteger); message 'removeItemAtIndex:';
    procedure removeAllItems; message 'removeAllItems';
    procedure selectItemWithObjectValue(object_: id); message 'selectItemWithObjectValue:';
    function itemObjectValueAtIndex(index: NSInteger): id; message 'itemObjectValueAtIndex:';
    function objectValueOfSelectedItem: id; message 'objectValueOfSelectedItem';
    function indexOfItemWithObjectValue(object_: id): NSInteger; message 'indexOfItemWithObjectValue:';
    function objectValues: NSArray; message 'objectValues';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSCOMBOBOXCELL_PAS_P}
{$define NSCOMBOBOXCELL_PAS_P}
  
{ NSComboBoxCellDataSource Protocol }
  NSComboBoxCellDataSourceProtocol = objcprotocol external name 'NSComboBoxCellDataSource'
  optional
    function numberOfItemsInComboBoxCell(comboBoxCell: NSComboBoxCell): NSInteger; message 'numberOfItemsInComboBoxCell:';
    function comboBoxCell_objectValueForItemAtIndex(aComboBoxCell: NSComboBoxCell; index: NSInteger): id; message 'comboBoxCell:objectValueForItemAtIndex:';
    function comboBoxCell_indexOfItemWithStringValue(aComboBoxCell: NSComboBoxCell; string_: NSString): NSUInteger; message 'comboBoxCell:indexOfItemWithStringValue:';
    function comboBoxCell_completedString(aComboBoxCell: NSComboBoxCell; uncompletedString: NSString): NSString; message 'comboBoxCell:completedString:';
  end;
{$endif}
{$endif}
