{ Parsed from Appkit.framework NSTableView.h }

{$ifdef TYPES}
{$ifndef NSTABLEVIEW_PAS_T}
{$define NSTABLEVIEW_PAS_T}

{ Constants }

const
  NSTableViewDropOn = 0;
  NSTableViewDropAbove = 1;

const
  NSTableViewNoColumnAutoresizing = 0;
  NSTableViewUniformColumnAutoresizingStyle = 1;
  NSTableViewSequentialColumnAutoresizingStyle = 2;
  NSTableViewReverseSequentialColumnAutoresizingStyle = 3;
  NSTableViewLastColumnOnlyAutoresizingStyle = 4;
  NSTableViewFirstColumnOnlyAutoresizingStyle = 5;

const
  NSTableViewGridNone = 0;
  NSTableViewSolidVerticalGridLineMask = 1 shl 0;
  NSTableViewSolidHorizontalGridLineMask = 1 shl 1;

const
  NSTableViewSelectionHighlightStyleNone = -1;
  NSTableViewSelectionHighlightStyleRegular = 0;
  NSTableViewSelectionHighlightStyleSourceList = 1;

const
  NSTableViewDraggingDestinationFeedbackStyleNone = -1;
  NSTableViewDraggingDestinationFeedbackStyleRegular = 0;
  NSTableViewDraggingDestinationFeedbackStyleSourceList = 1;

{ Types }
type
  NSTableViewDropOperation = NSUInteger;
  NSTableViewDropOperationPtr = ^NSTableViewDropOperation;
  NSTableViewColumnAutoresizingStyle = NSUInteger;
  NSTableViewColumnAutoresizingStylePtr = ^NSTableViewColumnAutoresizingStyle;
  NSTableViewSelectionHighlightStyle = NSInteger;
  NSTableViewSelectionHighlightStylePtr = ^NSTableViewSelectionHighlightStyle;
  NSTableViewDraggingDestinationFeedbackStyle = NSInteger;
  NSTableViewDraggingDestinationFeedbackStylePtr = ^NSTableViewDraggingDestinationFeedbackStyle;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTABLEVIEW_PAS_R}
{$define NSTABLEVIEW_PAS_R}

{ Records }
type
  __TvFlags = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    case byte of
    0: (_anonbitfield_NSTableView0: cuint);
    1: (data: bitpacked record
{$ifdef fpc_big_endian}
          allowsColumnReordering: 0..1;
          allowsColumnResizing: 0..1;
          oldDrawsGridFlag: 0..1;
          allowsEmptySelection: 0..1;
          allowsMultipleSelection: 0..1;
          allowsColumnSelection: 0..1;
          selectionType: 0..((1 shl 2)-1);
          changingLayout: 0..1;
          compareWidthWithSuperview: 0..((1 shl 2)-1);
          delegateWillDisplayCell: 0..1;
          delegateShouldEditTableColumn: 0..1;
          delegateShouldSelectRow: 0..1;
          delegateShouldSelectTableColumn: 0..1;
          delegateSelectionShouldChangeInTableView: 0..1;
          oldAutoresizesAllColumnsToFit: 0..1;
          dataSourceSetObjectValue: 0..1;
          selectionPostingDisableCount: 0..((1 shl 7)-1);
          movedPostingDisableCount: 0..((1 shl 6)-1);
          refusesFirstResponder: 0..1;
{$else}
          refusesFirstResponder: 0..1;
          movedPostingDisableCount: 0..((1 shl 6)-1);
          selectionPostingDisableCount: 0..((1 shl 7)-1);
          dataSourceSetObjectValue: 0..1;
          oldAutoresizesAllColumnsToFit: 0..1;
          delegateSelectionShouldChangeInTableView: 0..1;
          delegateShouldSelectTableColumn: 0..1;
          delegateShouldSelectRow: 0..1;
          delegateShouldEditTableColumn: 0..1;
          delegateWillDisplayCell: 0..1;
          compareWidthWithSuperview: 0..((1 shl 2)-1);
          changingLayout: 0..1;
          selectionType: 0..((1 shl 2)-1);
          allowsColumnSelection: 0..1;
          allowsMultipleSelection: 0..1;
          allowsEmptySelection: 0..1;
          oldDrawsGridFlag: 0..1;
          allowsColumnResizing: 0..1;
          allowsColumnReordering: 0..1;
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
    allowsColumnReordering: 0..1;
    allowsColumnResizing: 0..1;
    oldDrawsGridFlag: 0..1;
    allowsEmptySelection: 0..1;
    allowsMultipleSelection: 0..1;
    allowsColumnSelection: 0..1;
    selectionType: 0..((1 shl 2)-1);
    changingLayout: 0..1;
    compareWidthWithSuperview: 0..((1 shl 2)-1);
    delegateWillDisplayCell: 0..1;
    delegateShouldEditTableColumn: 0..1;
    delegateShouldSelectRow: 0..1;
    delegateShouldSelectTableColumn: 0..1;
    delegateSelectionShouldChangeInTableView: 0..1;
    oldAutoresizesAllColumnsToFit: 0..1;
    dataSourceSetObjectValue: 0..1;
    selectionPostingDisableCount: 0..((1 shl 7)-1);
    movedPostingDisableCount: 0..((1 shl 6)-1);
    refusesFirstResponder: 0..1;
{$else}
    refusesFirstResponder: 0..1;
    movedPostingDisableCount: 0..((1 shl 6)-1);
    selectionPostingDisableCount: 0..((1 shl 7)-1);
    dataSourceSetObjectValue: 0..1;
    oldAutoresizesAllColumnsToFit: 0..1;
    delegateSelectionShouldChangeInTableView: 0..1;
    delegateShouldSelectTableColumn: 0..1;
    delegateShouldSelectRow: 0..1;
    delegateShouldEditTableColumn: 0..1;
    delegateWillDisplayCell: 0..1;
    compareWidthWithSuperview: 0..((1 shl 2)-1);
    changingLayout: 0..1;
    selectionType: 0..((1 shl 2)-1);
    allowsColumnSelection: 0..1;
    allowsMultipleSelection: 0..1;
    allowsEmptySelection: 0..1;
    oldDrawsGridFlag: 0..1;
    allowsColumnResizing: 0..1;
    allowsColumnReordering: 0..1;
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
_TvFlags = __TvFlags;
__TvFlagsPtr = ^__TvFlags;


{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTABLEVIEW_PAS_F}
{$define NSTABLEVIEW_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTABLEVIEW_PAS_S}
{$define NSTABLEVIEW_PAS_S}

{ External string constants }
var
  NSTableViewSelectionDidChangeNotification: NSString; cvar; external;
  NSTableViewColumnDidMoveNotification: NSString; cvar; external;
  NSTableViewColumnDidResizeNotification: NSString; cvar; external;
  NSTableViewSelectionIsChangingNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTableViewDelegateProtocol = objcprotocol;
  NSTableViewDataSourceProtocol = objcprotocol;
  NSTableView = objcclass;
  NSTableViewPointer = ^NSTableView;
  NSTableViewPtr = NSTableViewPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTABLEVIEW_PAS_C}
{$define NSTABLEVIEW_PAS_C}

{ NSTableView }
  NSTableView = objcclass external (NSControl, NSUserInterfaceValidationsProtocol, NSTextViewDelegateProtocol)
  private
    _headerView: NSTableHeaderView;
    _cornerView: NSView;
    _tableColumns: NSMutableArray;
    _editingCell: NSCell;
    _delegate: id;
    _dataSource: id;
    _intercellSpacing: NSSize;
    _rowHeight: CGFloat;
    _lastSelectedColumn: NSInteger;
    _lastSelectedRow: NSInteger;
    _editingRow: NSInteger;
    _editingColumn: NSInteger;
    _selectedColumns: NSMutableIndexSet;
    _selectedRows: NSMutableIndexSet;
    _bodyDragImage: NSImage;
    _backgroundColor: NSColor;
    _gridColor: NSColor;
    _dragYPos: CGFloat;
    _target: id;
    _action: SEL;
    _doubleAction: SEL;
    _rectOfLastColumn: NSRect;
    _lastCachedRectColumn: NSInteger;
    _rectOfLastRow: NSRect;
    _lastCachedRectRow: NSInteger;
    _tvFlags: _TvFlags;
    _reserved: id;
    
  public
    procedure setDataSource(aSource: NSTableViewDataSourceProtocol); message 'setDataSource:';
    function dataSource: NSTableViewDataSourceProtocol; message 'dataSource';
    procedure setDelegate(delegate_: NSTableViewDelegateProtocol); message 'setDelegate:';
    function delegate: NSTableViewDelegateProtocol; message 'delegate';
    procedure setHeaderView(headerView_: NSTableHeaderView); message 'setHeaderView:';
    function headerView: NSTableHeaderView; message 'headerView';
    procedure setCornerView(cornerView_: NSView); message 'setCornerView:';
    function cornerView: NSView; message 'cornerView';
    procedure setAllowsColumnReordering(flag: Boolean); message 'setAllowsColumnReordering:';
    function allowsColumnReordering: Boolean; message 'allowsColumnReordering';
    procedure setAllowsColumnResizing(flag: Boolean); message 'setAllowsColumnResizing:';
    function allowsColumnResizing: Boolean; message 'allowsColumnResizing';
    procedure setColumnAutoresizingStyle(style: NSTableViewColumnAutoresizingStyle); message 'setColumnAutoresizingStyle:';
    function columnAutoresizingStyle: NSTableViewColumnAutoresizingStyle; message 'columnAutoresizingStyle';
    procedure setGridStyleMask(gridType: NSUInteger); message 'setGridStyleMask:';
    function gridStyleMask: NSUInteger; message 'gridStyleMask';
    procedure setIntercellSpacing(aSize: NSSize); message 'setIntercellSpacing:';
    function intercellSpacing: NSSize; message 'intercellSpacing';
    procedure setUsesAlternatingRowBackgroundColors(useAlternatingRowColors: Boolean); message 'setUsesAlternatingRowBackgroundColors:';
    function usesAlternatingRowBackgroundColors: Boolean; message 'usesAlternatingRowBackgroundColors';
    procedure setBackgroundColor(color: NSColor); message 'setBackgroundColor:';
    function backgroundColor: NSColor; message 'backgroundColor';
    procedure setGridColor(color: NSColor); message 'setGridColor:';
    function gridColor: NSColor; message 'gridColor';
    procedure setRowHeight(rowHeight_: CGFloat); message 'setRowHeight:';
    function rowHeight: CGFloat; message 'rowHeight';
    procedure noteHeightOfRowsWithIndexesChanged(indexSet: NSIndexSet); message 'noteHeightOfRowsWithIndexesChanged:';
    function tableColumns: NSArray; message 'tableColumns';
    function numberOfColumns: NSInteger; message 'numberOfColumns';
    function numberOfRows: NSInteger; message 'numberOfRows';
    procedure addTableColumn(tableColumn: NSTableColumn); message 'addTableColumn:';
    procedure removeTableColumn(tableColumn: NSTableColumn); message 'removeTableColumn:';
    procedure moveColumn_toColumn(oldIndex: NSInteger; newIndex: NSInteger); message 'moveColumn:toColumn:';
    function columnWithIdentifier(identifier: id): NSInteger; message 'columnWithIdentifier:';
    function tableColumnWithIdentifier(identifier: id): NSTableColumn; message 'tableColumnWithIdentifier:';
    procedure tile; message 'tile';
    procedure sizeToFit; message 'sizeToFit';
    procedure sizeLastColumnToFit; message 'sizeLastColumnToFit';
    procedure scrollRowToVisible(row: NSInteger); message 'scrollRowToVisible:';
    procedure scrollColumnToVisible(column: NSInteger); message 'scrollColumnToVisible:';
    procedure reloadData; message 'reloadData';
    procedure noteNumberOfRowsChanged; message 'noteNumberOfRowsChanged';
    procedure reloadDataForRowIndexes_columnIndexes(rowIndexes: NSIndexSet; columnIndexes: NSIndexSet); message 'reloadDataForRowIndexes:columnIndexes:';
    function editedColumn: NSInteger; message 'editedColumn';
    function editedRow: NSInteger; message 'editedRow';
    function clickedColumn: NSInteger; message 'clickedColumn';
    function clickedRow: NSInteger; message 'clickedRow';
    procedure setDoubleAction(aSelector: SEL); message 'setDoubleAction:';
    function doubleAction: SEL; message 'doubleAction';
    procedure setSortDescriptors(array_: NSArray); message 'setSortDescriptors:';
    function sortDescriptors: NSArray; message 'sortDescriptors';
    procedure setIndicatorImage_inTableColumn(anImage: NSImage; tableColumn: NSTableColumn); message 'setIndicatorImage:inTableColumn:';
    function indicatorImageInTableColumn(tableColumn: NSTableColumn): NSImage; message 'indicatorImageInTableColumn:';
    procedure setHighlightedTableColumn(tableColumn: NSTableColumn); message 'setHighlightedTableColumn:';
    function highlightedTableColumn: NSTableColumn; message 'highlightedTableColumn';
    procedure setVerticalMotionCanBeginDrag(flag: Boolean); message 'setVerticalMotionCanBeginDrag:';
    function verticalMotionCanBeginDrag: Boolean; message 'verticalMotionCanBeginDrag';
    function canDragRowsWithIndexes_atPoint(rowIndexes: NSIndexSet; mouseDownPoint: NSPoint): Boolean; message 'canDragRowsWithIndexes:atPoint:';
    function dragImageForRowsWithIndexes_tableColumns_event_offset(dragRows: NSIndexSet; tableColumns_: NSArray; dragEvent: NSEvent; dragImageOffset: NSPointPointer): NSImage; message 'dragImageForRowsWithIndexes:tableColumns:event:offset:';
    procedure setDraggingSourceOperationMask_forLocal(mask: NSDragOperation; isLocal: Boolean); message 'setDraggingSourceOperationMask:forLocal:';
    procedure setDropRow_dropOperation(row: NSInteger; dropOperation: NSTableViewDropOperation); message 'setDropRow:dropOperation:';
    procedure setAllowsMultipleSelection(flag: Boolean); message 'setAllowsMultipleSelection:';
    function allowsMultipleSelection: Boolean; message 'allowsMultipleSelection';
    procedure setAllowsEmptySelection(flag: Boolean); message 'setAllowsEmptySelection:';
    function allowsEmptySelection: Boolean; message 'allowsEmptySelection';
    procedure setAllowsColumnSelection(flag: Boolean); message 'setAllowsColumnSelection:';
    function allowsColumnSelection: Boolean; message 'allowsColumnSelection';
    procedure selectAll(sender: id); message 'selectAll:';
    procedure deselectAll(sender: id); message 'deselectAll:';
    procedure selectColumnIndexes_byExtendingSelection(indexes: NSIndexSet; extend: Boolean); message 'selectColumnIndexes:byExtendingSelection:';
    procedure selectRowIndexes_byExtendingSelection(indexes: NSIndexSet; extend: Boolean); message 'selectRowIndexes:byExtendingSelection:';
    function selectedColumnIndexes: NSIndexSet; message 'selectedColumnIndexes';
    function selectedRowIndexes: NSIndexSet; message 'selectedRowIndexes';
    procedure deselectColumn(column: NSInteger); message 'deselectColumn:';
    procedure deselectRow(row: NSInteger); message 'deselectRow:';
    function selectedColumn: NSInteger; message 'selectedColumn';
    function selectedRow: NSInteger; message 'selectedRow';
    function isColumnSelected(column: NSInteger): Boolean; message 'isColumnSelected:';
    function isRowSelected(row: NSInteger): Boolean; message 'isRowSelected:';
    function numberOfSelectedColumns: NSInteger; message 'numberOfSelectedColumns';
    function numberOfSelectedRows: NSInteger; message 'numberOfSelectedRows';
    function allowsTypeSelect: Boolean; message 'allowsTypeSelect';
    procedure setAllowsTypeSelect(value: Boolean); message 'setAllowsTypeSelect:';
    function selectionHighlightStyle: NSTableViewSelectionHighlightStyle; message 'selectionHighlightStyle';
    procedure setSelectionHighlightStyle(selectionHighlightStyle_: NSTableViewSelectionHighlightStyle); message 'setSelectionHighlightStyle:';
    procedure setDraggingDestinationFeedbackStyle(style: NSTableViewDraggingDestinationFeedbackStyle); message 'setDraggingDestinationFeedbackStyle:';
    function draggingDestinationFeedbackStyle: NSTableViewDraggingDestinationFeedbackStyle; message 'draggingDestinationFeedbackStyle';
    function rectOfColumn(column: NSInteger): NSRect; message 'rectOfColumn:';
    function rectOfRow(row: NSInteger): NSRect; message 'rectOfRow:';
    function columnIndexesInRect(rect: NSRect): NSIndexSet; message 'columnIndexesInRect:';
    function rowsInRect(rect: NSRect): NSRange; message 'rowsInRect:';
    function columnAtPoint(point: NSPoint): NSInteger; message 'columnAtPoint:';
    function rowAtPoint(point: NSPoint): NSInteger; message 'rowAtPoint:';
    function frameOfCellAtColumn_row(column: NSInteger; row: NSInteger): NSRect; message 'frameOfCellAtColumn:row:';
    function preparedCellAtColumn_row(column: NSInteger; row: NSInteger): NSCell; message 'preparedCellAtColumn:row:';
    function textShouldBeginEditing(textObject: NSText): Boolean; message 'textShouldBeginEditing:';
    function textShouldEndEditing(textObject: NSText): Boolean; message 'textShouldEndEditing:';
    procedure textDidBeginEditing(notification: NSNotification); message 'textDidBeginEditing:';
    procedure textDidEndEditing(notification: NSNotification); message 'textDidEndEditing:';
    procedure textDidChange(notification: NSNotification); message 'textDidChange:';
    procedure setAutosaveName(name: NSString); message 'setAutosaveName:';
    function autosaveName: NSString; message 'autosaveName';
    procedure setAutosaveTableColumns(save: Boolean); message 'setAutosaveTableColumns:';
    function autosaveTableColumns: Boolean; message 'autosaveTableColumns';
    function shouldFocusCell_atColumn_row(cell_: NSCell; column: NSInteger; row: NSInteger): Boolean; message 'shouldFocusCell:atColumn:row:';
    function focusedColumn: NSInteger; message 'focusedColumn';
    procedure setFocusedColumn(focusedColumn_: NSInteger); message 'setFocusedColumn:';
    procedure performClickOnCellAtColumn_row(column: NSInteger; row: NSInteger); message 'performClickOnCellAtColumn:row:';
    procedure editColumn_row_withEvent_select(column: NSInteger; row: NSInteger; theEvent: NSEvent; select: Boolean); message 'editColumn:row:withEvent:select:';
    procedure drawRow_clipRect(row: NSInteger; clipRect: NSRect); message 'drawRow:clipRect:';
    procedure highlightSelectionInClipRect(clipRect: NSRect); message 'highlightSelectionInClipRect:';
    procedure drawGridInClipRect(clipRect: NSRect); message 'drawGridInClipRect:';
    procedure drawBackgroundInClipRect(clipRect: NSRect); message 'drawBackgroundInClipRect:';

    { Adopted Protocols }
    function validateUserInterfaceItem(anItem: NSValidatedUserInterfaceItemProtocol): Boolean;
    function textView_clickedOnLink_atIndex(textView: NSTextView; link: id; charIndex: NSUInteger): Boolean;
    procedure textView_clickedOnCell_inRect_atIndex(textView: NSTextView; cell: NSTextAttachmentCellProtocol; cellFrame: NSRect; charIndex: NSUInteger);
    procedure textView_doubleClickedOnCell_inRect_atIndex(textView: NSTextView; cell: NSTextAttachmentCellProtocol; cellFrame: NSRect; charIndex: NSUInteger);
    procedure textView_draggedCell_inRect_event_atIndex(view: NSTextView; cell: NSTextAttachmentCellProtocol; rect: NSRect; event: NSEvent; charIndex: NSUInteger);
    function textView_writablePasteboardTypesForCell_atIndex(view: NSTextView; cell: NSTextAttachmentCellProtocol; charIndex: NSUInteger): NSArray;
    function textView_writeCell_atIndex_toPasteboard_type(view: NSTextView; cell: NSTextAttachmentCellProtocol; charIndex: NSUInteger; pboard: NSPasteboard; type_: NSString): Boolean;
    function textView_willChangeSelectionFromCharacterRange_toCharacterRange(textView: NSTextView; oldSelectedCharRange: NSRange; newSelectedCharRange: NSRange): NSRange;
    function textView_willChangeSelectionFromCharacterRanges_toCharacterRanges(textView: NSTextView; oldSelectedCharRanges: NSArray; newSelectedCharRanges: NSArray): NSArray;
    function textView_shouldChangeTextInRanges_replacementStrings(textView: NSTextView; affectedRanges: NSArray; replacementStrings: NSArray): Boolean;
    function textView_shouldChangeTypingAttributes_toAttributes(textView: NSTextView; oldTypingAttributes: NSDictionary; newTypingAttributes: NSDictionary): NSDictionary;
    procedure textViewDidChangeSelection(notification: NSNotification);
    procedure textViewDidChangeTypingAttributes(notification: NSNotification);
    function textView_willDisplayToolTip_forCharacterAtIndex(textView: NSTextView; tooltip: NSString; characterIndex: NSUInteger): NSString;
    function textView_completions_forPartialWordRange_indexOfSelectedItem(textView: NSTextView; words: NSArray; charRange: NSRange; index: NSIntegerPtr): NSArray;
    function textView_shouldChangeTextInRange_replacementString(textView: NSTextView; affectedCharRange: NSRange; replacementString: NSString): Boolean;
    function textView_doCommandBySelector(textView: NSTextView; commandSelector: SEL): Boolean;
    function textView_shouldSetSpellingState_range(textView: NSTextView; value: NSInteger; affectedCharRange: NSRange): NSInteger;
    function textView_menu_forEvent_atIndex(view: NSTextView; menu: NSMenu; event: NSEvent; charIndex: NSUInteger): NSMenu;
    function textView_willCheckTextInRange_options_types(view: NSTextView; range: NSRange; options: NSDictionary; checkingTypes: NSTextCheckingTypesPtr): NSDictionary;
    function textView_didCheckTextInRange_types_options_results_orthography_wordCount(view: NSTextView; range: NSRange; checkingTypes: NSTextCheckingTypes; options: NSDictionary; results: NSArray; orthography: NSOrthography; wordCount: NSInteger): NSArray;
    function textView_clickedOnLink(textView: NSTextView; link: id): Boolean;
    procedure textView_clickedOnCell_inRect(textView: NSTextView; cell: NSTextAttachmentCellProtocol; cellFrame: NSRect);
    procedure textView_doubleClickedOnCell_inRect(textView: NSTextView; cell: NSTextAttachmentCellProtocol; cellFrame: NSRect);
    procedure textView_draggedCell_inRect_event(view: NSTextView; cell: NSTextAttachmentCellProtocol; rect: NSRect; event: NSEvent);
    function undoManagerForTextView(view: NSTextView): NSUndoManager;
  end;

{ NSTableViewDataSourceDeprecatedCategory }
  NSTableViewDataSourceDeprecatedCategory = objccategory external (NSObject)
    function tableView_writeRows_toPasteboard(tableView: NSTableView; rows: NSArray; pboard: NSPasteboard): Boolean; message 'tableView:writeRows:toPasteboard:'; deprecated 'in Mac OS X 10.4 and later';
  end;

{ NSDeprecated_NSTableViewCategory }
  NSDeprecated_NSTableViewCategory = objccategory external name 'NSDeprecated' (NSTableView)
    procedure setDrawsGrid(flag: Boolean); message 'setDrawsGrid:'; deprecated 'in Mac OS X 10.3 and later';
    function drawsGrid: Boolean; message 'drawsGrid'; deprecated 'in Mac OS X 10.3 and later';
    procedure selectColumn_byExtendingSelection(column: NSInteger; extend: Boolean); message 'selectColumn:byExtendingSelection:'; deprecated 'in Mac OS X 10.3 and later';
    procedure selectRow_byExtendingSelection(row: NSInteger; extend: Boolean); message 'selectRow:byExtendingSelection:'; deprecated 'in Mac OS X 10.3 and later';
    function selectedColumnEnumerator: NSEnumerator; message 'selectedColumnEnumerator'; deprecated 'in Mac OS X 10.3 and later';
    function selectedRowEnumerator: NSEnumerator; message 'selectedRowEnumerator'; deprecated 'in Mac OS X 10.3 and later';
    function dragImageForRows_event_dragImageOffset(dragRows: NSArray; dragEvent: NSEvent; dragImageOffset: NSPointPointer): NSImage; message 'dragImageForRows:event:dragImageOffset:'; deprecated 'in Mac OS X 10.4 and later';
    procedure setAutoresizesAllColumnsToFit(flag: Boolean); message 'setAutoresizesAllColumnsToFit:'; deprecated 'in Mac OS X 10.4 and later';
    function autoresizesAllColumnsToFit: Boolean; message 'autoresizesAllColumnsToFit'; deprecated 'in Mac OS X 10.4 and later';
    function columnsInRect(rect: NSRect): NSRange; message 'columnsInRect:'; deprecated 'in Mac OS X 10.5 and later';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSTABLEVIEW_PAS_P}
{$define NSTABLEVIEW_PAS_P}
  
{ NSTableViewDelegate Protocol }
  NSTableViewDelegateProtocol = objcprotocol external name 'NSTableViewDelegate'
  optional
    procedure tableView_willDisplayCell_forTableColumn_row(tableView: NSTableView; cell: id; tableColumn: NSTableColumn; row: NSInteger); message 'tableView:willDisplayCell:forTableColumn:row:';
    function tableView_shouldEditTableColumn_row(tableView: NSTableView; tableColumn: NSTableColumn; row: NSInteger): Boolean; message 'tableView:shouldEditTableColumn:row:';
    function selectionShouldChangeInTableView(tableView: NSTableView): Boolean; message 'selectionShouldChangeInTableView:';
    function tableView_shouldSelectRow(tableView: NSTableView; row: NSInteger): Boolean; message 'tableView:shouldSelectRow:';
    function tableView_selectionIndexesForProposedSelection(tableView: NSTableView; proposedSelectionIndexes: NSIndexSet): NSIndexSet; message 'tableView:selectionIndexesForProposedSelection:';
    function tableView_shouldSelectTableColumn(tableView: NSTableView; tableColumn: NSTableColumn): Boolean; message 'tableView:shouldSelectTableColumn:';
    procedure tableView_mouseDownInHeaderOfTableColumn(tableView: NSTableView; tableColumn: NSTableColumn); message 'tableView:mouseDownInHeaderOfTableColumn:';
    procedure tableView_didClickTableColumn(tableView: NSTableView; tableColumn: NSTableColumn); message 'tableView:didClickTableColumn:';
    procedure tableView_didDragTableColumn(tableView: NSTableView; tableColumn: NSTableColumn); message 'tableView:didDragTableColumn:';
    function tableView_toolTipForCell_rect_tableColumn_row_mouseLocation(tableView: NSTableView; cell: NSCell; rect: NSRectPointer; tableColumn: NSTableColumn; row: NSInteger; mouseLocation: NSPoint): NSString; message 'tableView:toolTipForCell:rect:tableColumn:row:mouseLocation:';
    function tableView_heightOfRow(tableView: NSTableView; row: NSInteger): CGFloat; message 'tableView:heightOfRow:';
    function tableView_typeSelectStringForTableColumn_row(tableView: NSTableView; tableColumn: NSTableColumn; row: NSInteger): NSString; message 'tableView:typeSelectStringForTableColumn:row:';
    function tableView_nextTypeSelectMatchFromRow_toRow_forString(tableView: NSTableView; startRow: NSInteger; endRow: NSInteger; searchString: NSString): NSInteger; message 'tableView:nextTypeSelectMatchFromRow:toRow:forString:';
    function tableView_shouldTypeSelectForEvent_withCurrentSearchString(tableView: NSTableView; event: NSEvent; searchString: NSString): Boolean; message 'tableView:shouldTypeSelectForEvent:withCurrentSearchString:';
    function tableView_shouldShowCellExpansionForTableColumn_row(tableView: NSTableView; tableColumn: NSTableColumn; row: NSInteger): Boolean; message 'tableView:shouldShowCellExpansionForTableColumn:row:';
    function tableView_shouldTrackCell_forTableColumn_row(tableView: NSTableView; cell: NSCell; tableColumn: NSTableColumn; row: NSInteger): Boolean; message 'tableView:shouldTrackCell:forTableColumn:row:';
    function tableView_dataCellForTableColumn_row(tableView: NSTableView; tableColumn: NSTableColumn; row: NSInteger): NSCell; message 'tableView:dataCellForTableColumn:row:';
    function tableView_isGroupRow(tableView: NSTableView; row: NSInteger): Boolean; message 'tableView:isGroupRow:';
    function tableView_sizeToFitWidthOfColumn(tableView: NSTableView; column: NSInteger): CGFloat; message 'tableView:sizeToFitWidthOfColumn:';
    function tableView_shouldReorderColumn_toColumn(tableView: NSTableView; columnIndex: NSInteger; newColumnIndex: NSInteger): Boolean; message 'tableView:shouldReorderColumn:toColumn:';
    procedure tableViewSelectionDidChange(notification: NSNotification); message 'tableViewSelectionDidChange:';
    procedure tableViewColumnDidMove(notification: NSNotification); message 'tableViewColumnDidMove:';
    procedure tableViewColumnDidResize(notification: NSNotification); message 'tableViewColumnDidResize:';
    procedure tableViewSelectionIsChanging(notification: NSNotification); message 'tableViewSelectionIsChanging:';
  end;
  
{ NSTableViewDataSource Protocol }
  NSTableViewDataSourceProtocol = objcprotocol external name 'NSTableViewDataSource'
  optional
    function numberOfRowsInTableView(tableView: NSTableView): NSInteger; message 'numberOfRowsInTableView:';
    function tableView_objectValueForTableColumn_row(tableView: NSTableView; tableColumn: NSTableColumn; row: NSInteger): id; message 'tableView:objectValueForTableColumn:row:';
    procedure tableView_setObjectValue_forTableColumn_row(tableView: NSTableView; object_: id; tableColumn: NSTableColumn; row: NSInteger); message 'tableView:setObjectValue:forTableColumn:row:';
    procedure tableView_sortDescriptorsDidChange(tableView: NSTableView; oldDescriptors: NSArray); message 'tableView:sortDescriptorsDidChange:';
    function tableView_writeRowsWithIndexes_toPasteboard(tableView: NSTableView; rowIndexes: NSIndexSet; pboard: NSPasteboard): Boolean; message 'tableView:writeRowsWithIndexes:toPasteboard:';
    function tableView_validateDrop_proposedRow_proposedDropOperation(tableView: NSTableView; info: NSDraggingInfoProtocol; row: NSInteger; dropOperation: NSTableViewDropOperation): NSDragOperation; message 'tableView:validateDrop:proposedRow:proposedDropOperation:';
    function tableView_acceptDrop_row_dropOperation(tableView: NSTableView; info: NSDraggingInfoProtocol; row: NSInteger; dropOperation: NSTableViewDropOperation): Boolean; message 'tableView:acceptDrop:row:dropOperation:';
    function tableView_namesOfPromisedFilesDroppedAtDestination_forDraggedRowsWithIndexes(tableView: NSTableView; dropDestination: NSURL; indexSet: NSIndexSet): NSArray; message 'tableView:namesOfPromisedFilesDroppedAtDestination:forDraggedRowsWithIndexes:';
  end;
{$endif}
{$endif}
