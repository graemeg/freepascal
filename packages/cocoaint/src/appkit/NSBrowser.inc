{ Parsed from Appkit.framework NSBrowser.h }

{$ifdef TYPES}
{$ifndef NSBROWSER_PAS_T}
{$define NSBROWSER_PAS_T}

{ Defines }
const
  NSAppKitVersionNumberWithContinuousScrollingBrowser = 680.0;
  NSAppKitVersionNumberWithColumnResizingBrowser = 685.0;

{ Constants }

const
  NSBrowserNoColumnResizing = 0;
  NSBrowserAutoColumnResizing = 1;
  NSBrowserUserColumnResizing = 2;

const
  NSBrowserDropOn = 0;
  NSBrowserDropAbove = 1;

{ Types }
type
  NSBrowserColumnResizingType = NSUInteger;
  NSBrowserColumnResizingTypePtr = ^NSBrowserColumnResizingType;
  NSBrowserDropOperation = NSUInteger;
  NSBrowserDropOperationPtr = ^NSBrowserDropOperation;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSBROWSER_PAS_R}
{$define NSBROWSER_PAS_R}

{ Records }
type
  __Brflags = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    case byte of
    0: (_anonbitfield_NSBrowser0: cuint);
    1: (data: bitpacked record
{$ifdef fpc_big_endian}
          allowsMultipleSelection: 0..1;
          allowsBranchSelection: 0..1;
          reuseColumns: 0..1;
          isTitled: 0..1;
          titleFromPrevious: 0..1;
          separateColumns: 0..1;
          delegateImplementsWillDisplayCell: 0..1;
          delegateSetsTitles: 0..1;
          delegateSelectsCellsByString: 0..1;
          delegateDoesNotCreateRowsInMatrix: 0..1;
          delegateValidatesColumns: 0..1;
          acceptArrowKeys: 0..1;
          dontDrawTitles: 0..1;
          sendActionOnArrowKeys: 0..1;
          prohibitEmptySel: 0..1;
          hasHorizontalScroller: 0..1;
          time: 0..1;
          allowsIncrementalSearching: 0..1;
          delegateSelectsCellsByRow: 0..1;
          disableCompositing: 0..((1 shl 6)-1);
          refusesFirstResponder: 0..1;
          acceptsFirstMouse: 0..1;
          actionNeedsToBeSent: 0..1;
          usesSmallSizeTitleFont: 0..1;
          usesSmallScrollers: 0..1;
          prefersAllColumnUserResizing: 0..1;
          firstVisibleCalculationDisabled: 0..1;
{$else}
          firstVisibleCalculationDisabled: 0..1;
          prefersAllColumnUserResizing: 0..1;
          usesSmallScrollers: 0..1;
          usesSmallSizeTitleFont: 0..1;
          actionNeedsToBeSent: 0..1;
          acceptsFirstMouse: 0..1;
          refusesFirstResponder: 0..1;
          disableCompositing: 0..((1 shl 6)-1);
          delegateSelectsCellsByRow: 0..1;
          allowsIncrementalSearching: 0..1;
          time: 0..1;
          hasHorizontalScroller: 0..1;
          prohibitEmptySel: 0..1;
          sendActionOnArrowKeys: 0..1;
          dontDrawTitles: 0..1;
          acceptArrowKeys: 0..1;
          delegateValidatesColumns: 0..1;
          delegateDoesNotCreateRowsInMatrix: 0..1;
          delegateSelectsCellsByString: 0..1;
          delegateSetsTitles: 0..1;
          delegateImplementsWillDisplayCell: 0..1;
          separateColumns: 0..1;
          titleFromPrevious: 0..1;
          isTitled: 0..1;
          reuseColumns: 0..1;
          allowsBranchSelection: 0..1;
          allowsMultipleSelection: 0..1;
{$endif}
         end;
       );
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef fpc_big_endian}
    allowsMultipleSelection: 0..1;
    allowsBranchSelection: 0..1;
    reuseColumns: 0..1;
    isTitled: 0..1;
    titleFromPrevious: 0..1;
    separateColumns: 0..1;
    delegateImplementsWillDisplayCell: 0..1;
    delegateSetsTitles: 0..1;
    delegateSelectsCellsByString: 0..1;
    delegateDoesNotCreateRowsInMatrix: 0..1;
    delegateValidatesColumns: 0..1;
    acceptArrowKeys: 0..1;
    dontDrawTitles: 0..1;
    sendActionOnArrowKeys: 0..1;
    prohibitEmptySel: 0..1;
    hasHorizontalScroller: 0..1;
    time: 0..1;
    allowsIncrementalSearching: 0..1;
    delegateSelectsCellsByRow: 0..1;
    disableCompositing: 0..((1 shl 6)-1);
    refusesFirstResponder: 0..1;
    acceptsFirstMouse: 0..1;
    actionNeedsToBeSent: 0..1;
    usesSmallSizeTitleFont: 0..1;
    usesSmallScrollers: 0..1;
    prefersAllColumnUserResizing: 0..1;
    firstVisibleCalculationDisabled: 0..1;
{$else}
    firstVisibleCalculationDisabled: 0..1;
    prefersAllColumnUserResizing: 0..1;
    usesSmallScrollers: 0..1;
    usesSmallSizeTitleFont: 0..1;
    actionNeedsToBeSent: 0..1;
    acceptsFirstMouse: 0..1;
    refusesFirstResponder: 0..1;
    disableCompositing: 0..((1 shl 6)-1);
    delegateSelectsCellsByRow: 0..1;
    allowsIncrementalSearching: 0..1;
    time: 0..1;
    hasHorizontalScroller: 0..1;
    prohibitEmptySel: 0..1;
    sendActionOnArrowKeys: 0..1;
    dontDrawTitles: 0..1;
    acceptArrowKeys: 0..1;
    delegateValidatesColumns: 0..1;
    delegateDoesNotCreateRowsInMatrix: 0..1;
    delegateSelectsCellsByString: 0..1;
    delegateSetsTitles: 0..1;
    delegateImplementsWillDisplayCell: 0..1;
    separateColumns: 0..1;
    titleFromPrevious: 0..1;
    isTitled: 0..1;
    reuseColumns: 0..1;
    allowsBranchSelection: 0..1;
    allowsMultipleSelection: 0..1;
{$endif}
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
  end;
_Brflags = __Brflags;
__BrflagsPtr = ^__Brflags;


{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSBROWSER_PAS_F}
{$define NSBROWSER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSBROWSER_PAS_S}
{$define NSBROWSER_PAS_S}

{ External string constants }
var
  NSBrowserColumnConfigurationDidChangeNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSBrowserDelegateProtocol = objcprotocol;
  NSBrowser = objcclass;
  NSBrowserPointer = ^NSBrowser;
  NSBrowserPtr = NSBrowserPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSBROWSER_PAS_C}
{$define NSBROWSER_PAS_C}

{ NSBrowser }
  NSBrowser = objcclass external (NSControl)
  private
    _target: id;
    _action: SEL;
    _delegate: id;
    _doubleAction: SEL;
    _matrixClass: Pobjc_class;
    _cellPrototype: id;
    _columnSize: NSSize;
    _numberOfVisibleColumns: cshort;
    _minColumnWidth: cshort;
    _firstVisibleColumn: cshort;
    _maxVisibleColumns: cshort;
    _titles: NSMutableArray;
    _pathSeparator: NSString;
    _columns: NSMutableArray;
    _brAuxiliaryStorage: id;
    _firstColumnTitle: NSString;
    _scroller: NSScroller;
    _brflags: _Brflags;
    
  public
    class function cellClass: Pobjc_class; message 'cellClass';
    procedure loadColumnZero; message 'loadColumnZero';
    function isLoaded: Boolean; message 'isLoaded';
    procedure setDoubleAction(aSelector: SEL); message 'setDoubleAction:';
    function doubleAction: SEL; message 'doubleAction';
    procedure setMatrixClass(factoryId: Pobjc_class); message 'setMatrixClass:';
    function matrixClass: Pobjc_class; message 'matrixClass';
    procedure setCellClass(factoryId: Pobjc_class); message 'setCellClass:';
    procedure setCellPrototype(aCell: NSCell); message 'setCellPrototype:';
    function cellPrototype: id; message 'cellPrototype';
    procedure setDelegate(anObject: NSBrowserDelegateProtocol); message 'setDelegate:';
    function delegate: NSBrowserDelegateProtocol; message 'delegate';
    procedure setReusesColumns(flag: Boolean); message 'setReusesColumns:';
    function reusesColumns: Boolean; message 'reusesColumns';
    procedure setHasHorizontalScroller(flag: Boolean); message 'setHasHorizontalScroller:';
    function hasHorizontalScroller: Boolean; message 'hasHorizontalScroller';
    procedure setAutohidesScroller(flag: Boolean); message 'setAutohidesScroller:';
    function autohidesScroller: Boolean; message 'autohidesScroller';
    procedure setSeparatesColumns(flag: Boolean); message 'setSeparatesColumns:';
    function separatesColumns: Boolean; message 'separatesColumns';
    procedure setTitled(flag: Boolean); message 'setTitled:';
    function isTitled: Boolean; message 'isTitled';
    procedure setMinColumnWidth(columnWidth: CGFloat); message 'setMinColumnWidth:';
    function minColumnWidth: CGFloat; message 'minColumnWidth';
    procedure setMaxVisibleColumns(columnCount: NSInteger); message 'setMaxVisibleColumns:';
    function maxVisibleColumns: NSInteger; message 'maxVisibleColumns';
    procedure setAllowsMultipleSelection(flag: Boolean); message 'setAllowsMultipleSelection:';
    function allowsMultipleSelection: Boolean; message 'allowsMultipleSelection';
    procedure setAllowsBranchSelection(flag: Boolean); message 'setAllowsBranchSelection:';
    function allowsBranchSelection: Boolean; message 'allowsBranchSelection';
    procedure setAllowsEmptySelection(flag: Boolean); message 'setAllowsEmptySelection:';
    function allowsEmptySelection: Boolean; message 'allowsEmptySelection';
    procedure setTakesTitleFromPreviousColumn(flag: Boolean); message 'setTakesTitleFromPreviousColumn:';
    function takesTitleFromPreviousColumn: Boolean; message 'takesTitleFromPreviousColumn';
    procedure setSendsActionOnArrowKeys(flag: Boolean); message 'setSendsActionOnArrowKeys:';
    function sendsActionOnArrowKeys: Boolean; message 'sendsActionOnArrowKeys';
    function itemAtIndexPath(indexPath: NSIndexPath): id; message 'itemAtIndexPath:';
    function itemAtRow_inColumn(row: NSInteger; column: NSInteger): id; message 'itemAtRow:inColumn:';
    function indexPathForColumn(column: NSInteger): NSIndexPath; message 'indexPathForColumn:';
    function isLeafItem(item: id): Boolean; message 'isLeafItem:';
    procedure reloadDataForRowIndexes_inColumn(rowIndexes: NSIndexSet; column: NSInteger); message 'reloadDataForRowIndexes:inColumn:';
    function parentForItemsInColumn(column: NSInteger): id; message 'parentForItemsInColumn:';
    procedure scrollRowToVisible_inColumn(row: NSInteger; column: NSInteger); message 'scrollRowToVisible:inColumn:';
    procedure setTitle_ofColumn(aString: NSString; column: NSInteger); message 'setTitle:ofColumn:';
    function titleOfColumn(column: NSInteger): NSString; message 'titleOfColumn:';
    procedure setPathSeparator(newString: NSString); message 'setPathSeparator:';
    function pathSeparator: NSString; message 'pathSeparator';
    function setPath(path_: NSString): Boolean; message 'setPath:';
    function path: NSString; message 'path';
    function pathToColumn(column: NSInteger): NSString; message 'pathToColumn:';
    function clickedColumn: NSInteger; message 'clickedColumn';
    function clickedRow: NSInteger; message 'clickedRow';
    function selectedColumn: NSInteger; message 'selectedColumn';
    function selectedCell: id; message 'selectedCell';
    function selectedCellInColumn(column: NSInteger): id; message 'selectedCellInColumn:';
    function selectedCells: NSArray; message 'selectedCells';
    procedure selectRow_inColumn(row: NSInteger; column: NSInteger); message 'selectRow:inColumn:';
    function selectedRowInColumn(column: NSInteger): NSInteger; message 'selectedRowInColumn:';
    function selectionIndexPath: NSIndexPath; message 'selectionIndexPath';
    procedure setSelectionIndexPath(path_: NSIndexPath); message 'setSelectionIndexPath:';
    function selectionIndexPaths: NSArray; message 'selectionIndexPaths';
    procedure setSelectionIndexPaths(paths: NSArray); message 'setSelectionIndexPaths:';
    procedure selectRowIndexes_inColumn(indexes: NSIndexSet; column: NSInteger); message 'selectRowIndexes:inColumn:';
    function selectedRowIndexesInColumn(column: NSInteger): NSIndexSet; message 'selectedRowIndexesInColumn:';
    procedure reloadColumn(column: NSInteger); message 'reloadColumn:';
    procedure validateVisibleColumns; message 'validateVisibleColumns';
    procedure scrollColumnsRightBy(shiftAmount: NSInteger); message 'scrollColumnsRightBy:';
    procedure scrollColumnsLeftBy(shiftAmount: NSInteger); message 'scrollColumnsLeftBy:';
    procedure scrollColumnToVisible(column: NSInteger); message 'scrollColumnToVisible:';
    function lastColumn: NSInteger; message 'lastColumn';
    procedure setLastColumn(column: NSInteger); message 'setLastColumn:';
    procedure addColumn; message 'addColumn';
    function numberOfVisibleColumns: NSInteger; message 'numberOfVisibleColumns';
    function firstVisibleColumn: NSInteger; message 'firstVisibleColumn';
    function lastVisibleColumn: NSInteger; message 'lastVisibleColumn';
    function columnOfMatrix(matrix: NSMatrix): NSInteger; message 'columnOfMatrix:';
    function matrixInColumn(column: NSInteger): NSMatrix; message 'matrixInColumn:';
    function loadedCellAtRow_column(row: NSInteger; col: NSInteger): id; message 'loadedCellAtRow:column:';
    procedure selectAll(sender: id); message 'selectAll:';
    procedure tile; message 'tile';
    procedure doClick(sender: id); message 'doClick:';
    procedure doDoubleClick(sender: id); message 'doDoubleClick:';
    function sendAction: Boolean; message 'sendAction';
    function titleFrameOfColumn(column: NSInteger): NSRect; message 'titleFrameOfColumn:';
    procedure drawTitleOfColumn_inRect(column: NSInteger; aRect: NSRect); message 'drawTitleOfColumn:inRect:';
    function titleHeight: CGFloat; message 'titleHeight';
    function frameOfColumn(column: NSInteger): NSRect; message 'frameOfColumn:';
    function frameOfInsideOfColumn(column: NSInteger): NSRect; message 'frameOfInsideOfColumn:';
    function frameOfRow_inColumn(row: NSInteger; column: NSInteger): NSRect; message 'frameOfRow:inColumn:';
    function getRow_column_forPoint(row: NSIntegerPtr; column: NSIntegerPtr; point: NSPoint): Boolean; message 'getRow:column:forPoint:';
    function columnWidthForColumnContentWidth(columnContentWidth: CGFloat): CGFloat; message 'columnWidthForColumnContentWidth:';
    function columnContentWidthForColumnWidth(columnWidth: CGFloat): CGFloat; message 'columnContentWidthForColumnWidth:';
    procedure setColumnResizingType(columnResizingType_: NSBrowserColumnResizingType); message 'setColumnResizingType:';
    function columnResizingType: NSBrowserColumnResizingType; message 'columnResizingType';
    procedure setPrefersAllColumnUserResizing(prefersAllColumnResizing: Boolean); message 'setPrefersAllColumnUserResizing:';
    function prefersAllColumnUserResizing: Boolean; message 'prefersAllColumnUserResizing';
    procedure setWidth_ofColumn(columnWidth: CGFloat; columnIndex: NSInteger); message 'setWidth:ofColumn:';
    function widthOfColumn(column: NSInteger): CGFloat; message 'widthOfColumn:';
    procedure setRowHeight(height: CGFloat); message 'setRowHeight:';
    function rowHeight: CGFloat; message 'rowHeight';
    procedure noteHeightOfRowsWithIndexesChanged_inColumn(indexSet: NSIndexSet; columnIndex: NSInteger); message 'noteHeightOfRowsWithIndexesChanged:inColumn:';
    procedure setDefaultColumnWidth(columnWidth: CGFloat); message 'setDefaultColumnWidth:';
    function defaultColumnWidth: CGFloat; message 'defaultColumnWidth';
    procedure setColumnsAutosaveName(name: NSString); message 'setColumnsAutosaveName:';
    function columnsAutosaveName: NSString; message 'columnsAutosaveName';
    class procedure removeSavedColumnsWithAutosaveName(name: NSString); message 'removeSavedColumnsWithAutosaveName:';
    function canDragRowsWithIndexes_inColumn_withEvent(rowIndexes: NSIndexSet; column: NSInteger; event: NSEvent): Boolean; message 'canDragRowsWithIndexes:inColumn:withEvent:';
    function draggingImageForRowsWithIndexes_inColumn_withEvent_offset(rowIndexes: NSIndexSet; column: NSInteger; event: NSEvent; dragImageOffset: NSPointPointer): NSImage; message 'draggingImageForRowsWithIndexes:inColumn:withEvent:offset:';
    procedure setDraggingSourceOperationMask_forLocal(mask: NSDragOperation; isLocal: Boolean); message 'setDraggingSourceOperationMask:forLocal:';
    function allowsTypeSelect: Boolean; message 'allowsTypeSelect';
    procedure setAllowsTypeSelect(value: Boolean); message 'setAllowsTypeSelect:';
    procedure setBackgroundColor(color: NSColor); message 'setBackgroundColor:';
    function backgroundColor: NSColor; message 'backgroundColor';
    procedure editItemAtIndexPath_withEvent_select(indexPath: NSIndexPath; theEvent: NSEvent; select: Boolean); message 'editItemAtIndexPath:withEvent:select:';
  end;

{ NSDeprecated_NSBrowserCategory }
  NSDeprecated_NSBrowserCategory = objccategory external name 'NSDeprecated' (NSBrowser)
    procedure setAcceptsArrowKeys(flag: Boolean); message 'setAcceptsArrowKeys:'; deprecated 'in Mac OS X 10.6 and later';
    function acceptsArrowKeys: Boolean; message 'acceptsArrowKeys'; deprecated 'in Mac OS X 10.6 and later';
    procedure displayColumn(column: NSInteger); message 'displayColumn:'; deprecated 'in Mac OS X 10.3 and later';
    procedure displayAllColumns; message 'displayAllColumns'; deprecated 'in Mac OS X 10.3 and later';
    procedure scrollViaScroller(sender: NSScroller); message 'scrollViaScroller:'; deprecated 'in Mac OS X 10.3 and later';
    procedure updateScroller; message 'updateScroller'; deprecated 'in Mac OS X 10.3 and later';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSBROWSER_PAS_P}
{$define NSBROWSER_PAS_P}
  
{ NSBrowserDelegate Protocol }
  NSBrowserDelegateProtocol = objcprotocol external name 'NSBrowserDelegate'
  optional
    function browser_numberOfRowsInColumn(sender: NSBrowser; column: NSInteger): NSInteger; message 'browser:numberOfRowsInColumn:';
    procedure browser_createRowsForColumn_inMatrix(sender: NSBrowser; column: NSInteger; matrix: NSMatrix); message 'browser:createRowsForColumn:inMatrix:';
    function browser_numberOfChildrenOfItem(browser: NSBrowser; item: id): NSInteger; message 'browser:numberOfChildrenOfItem:';
    function browser_child_ofItem(browser: NSBrowser; index: NSInteger; item: id): id; message 'browser:child:ofItem:';
    function browser_isLeafItem(browser: NSBrowser; item: id): Boolean; message 'browser:isLeafItem:';
    function browser_objectValueForItem(browser: NSBrowser; item: id): id; message 'browser:objectValueForItem:';
    function browser_heightOfRow_inColumn(browser: NSBrowser; row: NSInteger; columnIndex: NSInteger): CGFloat; message 'browser:heightOfRow:inColumn:';
    function rootItemForBrowser(browser: NSBrowser): id; message 'rootItemForBrowser:';
    procedure browser_setObjectValue_forItem(browser: NSBrowser; object_: id; item: id); message 'browser:setObjectValue:forItem:';
    function browser_shouldEditItem(browser: NSBrowser; item: id): Boolean; message 'browser:shouldEditItem:';
    procedure browser_willDisplayCell_atRow_column(sender: NSBrowser; cell: id; row: NSInteger; column: NSInteger); message 'browser:willDisplayCell:atRow:column:';
    function browser_titleOfColumn(sender: NSBrowser; column: NSInteger): NSString; message 'browser:titleOfColumn:';
    function browser_selectCellWithString_inColumn(sender: NSBrowser; title: NSString; column: NSInteger): Boolean; message 'browser:selectCellWithString:inColumn:';
    function browser_selectRow_inColumn(sender: NSBrowser; row: NSInteger; column: NSInteger): Boolean; message 'browser:selectRow:inColumn:';
    function browser_isColumnValid(sender: NSBrowser; column: NSInteger): Boolean; message 'browser:isColumnValid:';
    procedure browserWillScroll(sender: NSBrowser); message 'browserWillScroll:';
    procedure browserDidScroll(sender: NSBrowser); message 'browserDidScroll:';
    function browser_shouldSizeColumn_forUserResize_toWidth(browser: NSBrowser; columnIndex: NSInteger; forUserResize: Boolean; suggestedWidth: CGFloat): CGFloat; message 'browser:shouldSizeColumn:forUserResize:toWidth:';
    function browser_sizeToFitWidthOfColumn(browser: NSBrowser; columnIndex: NSInteger): CGFloat; message 'browser:sizeToFitWidthOfColumn:';
    procedure browserColumnConfigurationDidChange(notification: NSNotification); message 'browserColumnConfigurationDidChange:';
    function browser_shouldShowCellExpansionForRow_column(browser: NSBrowser; row: NSInteger; column: NSInteger): Boolean; message 'browser:shouldShowCellExpansionForRow:column:';
    function browser_writeRowsWithIndexes_inColumn_toPasteboard(browser: NSBrowser; rowIndexes: NSIndexSet; column: NSInteger; pasteboard: NSPasteboard): Boolean; message 'browser:writeRowsWithIndexes:inColumn:toPasteboard:';
    function browser_namesOfPromisedFilesDroppedAtDestination_forDraggedRowsWithIndexes_inColumn(browser: NSBrowser; dropDestination: NSURL; rowIndexes: NSIndexSet; column: NSInteger): NSArray; message 'browser:namesOfPromisedFilesDroppedAtDestination:forDraggedRowsWithIndexes:inColumn:';
    function browser_canDragRowsWithIndexes_inColumn_withEvent(browser: NSBrowser; rowIndexes: NSIndexSet; column: NSInteger; event: NSEvent): Boolean; message 'browser:canDragRowsWithIndexes:inColumn:withEvent:';
    function browser_draggingImageForRowsWithIndexes_inColumn_withEvent_offset(browser: NSBrowser; rowIndexes: NSIndexSet; column: NSInteger; event: NSEvent; dragImageOffset: NSPointPointer): NSImage; message 'browser:draggingImageForRowsWithIndexes:inColumn:withEvent:offset:';
    function browser_validateDrop_proposedRow_column_dropOperation(browser: NSBrowser; info: NSDraggingInfoProtocol; row: NSIntegerPtr; column: NSIntegerPtr; dropOperation: NSBrowserDropOperationPtr): NSDragOperation; message 'browser:validateDrop:proposedRow:column:dropOperation:';
    function browser_acceptDrop_atRow_column_dropOperation(browser: NSBrowser; info: NSDraggingInfoProtocol; row: NSInteger; column: NSInteger; dropOperation: NSBrowserDropOperation): Boolean; message 'browser:acceptDrop:atRow:column:dropOperation:';
    function browser_typeSelectStringForRow_inColumn(browser: NSBrowser; row: NSInteger; column: NSInteger): NSString; message 'browser:typeSelectStringForRow:inColumn:';
    function browser_shouldTypeSelectForEvent_withCurrentSearchString(browser: NSBrowser; event: NSEvent; searchString: NSString): Boolean; message 'browser:shouldTypeSelectForEvent:withCurrentSearchString:';
    function browser_nextTypeSelectMatchFromRow_toRow_inColumn_forString(browser: NSBrowser; startRow: NSInteger; endRow: NSInteger; column: NSInteger; searchString: NSString): NSInteger; message 'browser:nextTypeSelectMatchFromRow:toRow:inColumn:forString:';
    function browser_previewViewControllerForLeafItem(browser: NSBrowser; item: id): NSViewController; message 'browser:previewViewControllerForLeafItem:';
    function browser_headerViewControllerForItem(browser: NSBrowser; item: id): NSViewController; message 'browser:headerViewControllerForItem:';
    procedure browser_didChangeLastColumn_toColumn(browser: NSBrowser; oldLastColumn: NSInteger; column: NSInteger); message 'browser:didChangeLastColumn:toColumn:';
    function browser_selectionIndexesForProposedSelection_inColumn(browser: NSBrowser; proposedSelectionIndexes: NSIndexSet; column: NSInteger): NSIndexSet; message 'browser:selectionIndexesForProposedSelection:inColumn:';
  end;
{$endif}
{$endif}
