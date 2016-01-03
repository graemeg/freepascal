{ Parsed from Appkit.framework NSMatrix.h }

{$ifdef TYPES}
{$ifndef NSMATRIX_PAS_T}
{$define NSMATRIX_PAS_T}

{ Constants }

const
  NSRadioModeMatrix = 0;
  NSHighlightModeMatrix = 1;
  NSListModeMatrix = 2;
  NSTrackModeMatrix = 3;

{ Types }
type
  NSMatrixMode = NSUInteger;
  NSMatrixModePtr = ^NSMatrixMode;

{ Callbacks }
type
  NSMatrixCompare = function (param1: id; param2: id; param3: Pointer): NSInteger; cdecl;


{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMATRIX_PAS_R}
{$define NSMATRIX_PAS_R}

{ Records }
type
  __MFlags = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    case byte of
    0: (_anonbitfield_NSMatrix0: cuint);
    1: (data: bitpacked record
{$ifdef fpc_big_endian}
          highlightMode: 0..1;
          radioMode: 0..1;
          listMode: 0..1;
          allowEmptySel: 0..1;
          autoscroll: 0..1;
          selectionByRect: 0..1;
          drawsCellBackground: 0..1;
          drawsBackground: 0..1;
          autosizeCells: 0..1;
          drawingAncestor: 0..1;
          tabKeyTraversesCells: 0..1;
          tabKeyTraversesCellsExplicitlySet: 0..1;
          allowsIncrementalSearching: 0..1;
          currentlySelectingCell: 0..1;
          onlySetKeyCell: 0..1;
          changingSelectionWithKeyboard: 0..1;
          dontScroll: 0..1;
          refusesFirstResponder: 0..1;
          useSimpleTrackingMode: 0..1;
          checkForSimpleTrackingMode: 0..1;
          liveResizeImageCacheingEnabled: 0..1;
          hasCachedSubclassIsSafeForLiveResize: 0..1;
          subclassIsSafeForLiveResize: 0..1;
          tmpAllowNonVisibleCellsToBecomeFirstResponder: 0..1;
          needsRedrawBeforeFirstLiveResizeCache: 0..1;
          browserOptimizationsEnabled: 0..1;
          drawingContextMenuHighlight: 0..1;
          drawingContextMenuHighlightOnAllSelectedRows: 0..1;
          reservedMatrix: 0..((1 shl 4)-1);
{$else}
          reservedMatrix: 0..((1 shl 4)-1);
          drawingContextMenuHighlightOnAllSelectedRows: 0..1;
          drawingContextMenuHighlight: 0..1;
          browserOptimizationsEnabled: 0..1;
          needsRedrawBeforeFirstLiveResizeCache: 0..1;
          tmpAllowNonVisibleCellsToBecomeFirstResponder: 0..1;
          subclassIsSafeForLiveResize: 0..1;
          hasCachedSubclassIsSafeForLiveResize: 0..1;
          liveResizeImageCacheingEnabled: 0..1;
          checkForSimpleTrackingMode: 0..1;
          useSimpleTrackingMode: 0..1;
          refusesFirstResponder: 0..1;
          dontScroll: 0..1;
          changingSelectionWithKeyboard: 0..1;
          onlySetKeyCell: 0..1;
          currentlySelectingCell: 0..1;
          allowsIncrementalSearching: 0..1;
          tabKeyTraversesCellsExplicitlySet: 0..1;
          tabKeyTraversesCells: 0..1;
          drawingAncestor: 0..1;
          autosizeCells: 0..1;
          drawsBackground: 0..1;
          drawsCellBackground: 0..1;
          selectionByRect: 0..1;
          autoscroll: 0..1;
          allowEmptySel: 0..1;
          listMode: 0..1;
          radioMode: 0..1;
          highlightMode: 0..1;
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
    highlightMode: 0..1;
    radioMode: 0..1;
    listMode: 0..1;
    allowEmptySel: 0..1;
    autoscroll: 0..1;
    selectionByRect: 0..1;
    drawsCellBackground: 0..1;
    drawsBackground: 0..1;
    autosizeCells: 0..1;
    drawingAncestor: 0..1;
    tabKeyTraversesCells: 0..1;
    tabKeyTraversesCellsExplicitlySet: 0..1;
    allowsIncrementalSearching: 0..1;
    currentlySelectingCell: 0..1;
    onlySetKeyCell: 0..1;
    changingSelectionWithKeyboard: 0..1;
    dontScroll: 0..1;
    refusesFirstResponder: 0..1;
    useSimpleTrackingMode: 0..1;
    checkForSimpleTrackingMode: 0..1;
    liveResizeImageCacheingEnabled: 0..1;
    hasCachedSubclassIsSafeForLiveResize: 0..1;
    subclassIsSafeForLiveResize: 0..1;
    tmpAllowNonVisibleCellsToBecomeFirstResponder: 0..1;
    needsRedrawBeforeFirstLiveResizeCache: 0..1;
    browserOptimizationsEnabled: 0..1;
    drawingContextMenuHighlight: 0..1;
    drawingContextMenuHighlightOnAllSelectedRows: 0..1;
    reservedMatrix: 0..((1 shl 4)-1);
{$else}
    reservedMatrix: 0..((1 shl 4)-1);
    drawingContextMenuHighlightOnAllSelectedRows: 0..1;
    drawingContextMenuHighlight: 0..1;
    browserOptimizationsEnabled: 0..1;
    needsRedrawBeforeFirstLiveResizeCache: 0..1;
    tmpAllowNonVisibleCellsToBecomeFirstResponder: 0..1;
    subclassIsSafeForLiveResize: 0..1;
    hasCachedSubclassIsSafeForLiveResize: 0..1;
    liveResizeImageCacheingEnabled: 0..1;
    checkForSimpleTrackingMode: 0..1;
    useSimpleTrackingMode: 0..1;
    refusesFirstResponder: 0..1;
    dontScroll: 0..1;
    changingSelectionWithKeyboard: 0..1;
    onlySetKeyCell: 0..1;
    currentlySelectingCell: 0..1;
    allowsIncrementalSearching: 0..1;
    tabKeyTraversesCellsExplicitlySet: 0..1;
    tabKeyTraversesCells: 0..1;
    drawingAncestor: 0..1;
    autosizeCells: 0..1;
    drawsBackground: 0..1;
    drawsCellBackground: 0..1;
    selectionByRect: 0..1;
    autoscroll: 0..1;
    allowEmptySel: 0..1;
    listMode: 0..1;
    radioMode: 0..1;
    highlightMode: 0..1;
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
_MFlags = __MFlags;
__MFlagsPtr = ^__MFlags;


{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMATRIX_PAS_F}
{$define NSMATRIX_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMATRIX_PAS_S}
{$define NSMATRIX_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSMatrixDelegateProtocol = objcprotocol;
  NSMatrix = objcclass;
  NSMatrixPointer = ^NSMatrix;
  NSMatrixPtr = NSMatrixPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMATRIX_PAS_C}
{$define NSMATRIX_PAS_C}

{ NSMatrix }
  NSMatrix = objcclass external (NSControl, NSUserInterfaceValidationsProtocol)
  private
    _target: id;
    _action: SEL;
    _doubleAction: SEL;
    _errorAction: SEL;
    _delegate: id;
    _selectedCell: id;
    _selectedRow: NSInteger;
    _selectedCol: NSInteger;
    _numRows: NSInteger;
    _numCols: NSInteger;
    _cellSize: NSSize;
    _intercell: NSSize;
    _font: id;
    _protoCell: id;
    _cellClass: id;
    _backgroundColor: NSColor;
    _private: id;
    _cells: NSMutableArray;
    _mFlags: _MFlags;
    
  public
    function initWithFrame(frameRect: NSRect): id; message 'initWithFrame:';
    function initWithFrame_mode_prototype_numberOfRows_numberOfColumns(frameRect: NSRect; aMode: NSMatrixMode; aCell: NSCell; rowsHigh: NSInteger; colsWide: NSInteger): id; message 'initWithFrame:mode:prototype:numberOfRows:numberOfColumns:';
    function initWithFrame_mode_cellClass_numberOfRows_numberOfColumns(frameRect: NSRect; aMode: NSMatrixMode; factoryId: Pobjc_class; rowsHigh: NSInteger; colsWide: NSInteger): id; message 'initWithFrame:mode:cellClass:numberOfRows:numberOfColumns:';
    procedure setCellClass(factoryId: Pobjc_class); message 'setCellClass:';
    function cellClass: Pobjc_class; message 'cellClass';
    function prototype: id; message 'prototype';
    procedure setPrototype(aCell: NSCell); message 'setPrototype:';
    function makeCellAtRow_column(row: NSInteger; col: NSInteger): NSCell; message 'makeCellAtRow:column:';
    function mode: NSMatrixMode; message 'mode';
    procedure setMode(aMode: NSMatrixMode); message 'setMode:';
    procedure setAllowsEmptySelection(flag: Boolean); message 'setAllowsEmptySelection:';
    function allowsEmptySelection: Boolean; message 'allowsEmptySelection';
    procedure sendAction_to_forAllCells(aSelector: SEL; anObject: id; flag: Boolean); message 'sendAction:to:forAllCells:';
    function cells: NSArray; message 'cells';
    procedure sortUsingSelector(comparator: SEL); message 'sortUsingSelector:';
    procedure sortUsingFunction_context(compare: NSMatrixCompare; context: Pointer); message 'sortUsingFunction:context:';
    function selectedCell: id; message 'selectedCell';
    function selectedCells: NSArray; message 'selectedCells';
    function selectedRow: NSInteger; message 'selectedRow';
    function selectedColumn: NSInteger; message 'selectedColumn';
    procedure setSelectionByRect(flag: Boolean); message 'setSelectionByRect:';
    function isSelectionByRect: Boolean; message 'isSelectionByRect';
    procedure setSelectionFrom_to_anchor_highlight(startPos: NSInteger; endPos: NSInteger; anchorPos: NSInteger; lit: Boolean); message 'setSelectionFrom:to:anchor:highlight:';
    procedure deselectSelectedCell; message 'deselectSelectedCell';
    procedure deselectAllCells; message 'deselectAllCells';
    procedure selectCellAtRow_column(row: NSInteger; col: NSInteger); message 'selectCellAtRow:column:';
    procedure selectAll(sender: id); message 'selectAll:';
    function selectCellWithTag(anInt: NSInteger): Boolean; message 'selectCellWithTag:';
    function cellSize: NSSize; message 'cellSize';
    procedure setCellSize(aSize: NSSize); message 'setCellSize:';
    function intercellSpacing: NSSize; message 'intercellSpacing';
    procedure setIntercellSpacing(aSize: NSSize); message 'setIntercellSpacing:';
    procedure setScrollable(flag: Boolean); message 'setScrollable:';
    procedure setBackgroundColor(color: NSColor); message 'setBackgroundColor:';
    function backgroundColor: NSColor; message 'backgroundColor';
    procedure setCellBackgroundColor(color: NSColor); message 'setCellBackgroundColor:';
    function cellBackgroundColor: NSColor; message 'cellBackgroundColor';
    procedure setDrawsCellBackground(flag: Boolean); message 'setDrawsCellBackground:';
    function drawsCellBackground: Boolean; message 'drawsCellBackground';
    procedure setDrawsBackground(flag: Boolean); message 'setDrawsBackground:';
    function drawsBackground: Boolean; message 'drawsBackground';
    procedure setState_atRow_column(value: NSInteger; row: NSInteger; col: NSInteger); message 'setState:atRow:column:';
    procedure getNumberOfRows_columns(rowCount: NSIntegerPtr; colCount: NSIntegerPtr); message 'getNumberOfRows:columns:';
    function numberOfRows: NSInteger; message 'numberOfRows';
    function numberOfColumns: NSInteger; message 'numberOfColumns';
    function cellAtRow_column(row: NSInteger; col: NSInteger): id; message 'cellAtRow:column:';
    function cellFrameAtRow_column(row: NSInteger; col: NSInteger): NSRect; message 'cellFrameAtRow:column:';
    function getRow_column_ofCell(row: NSIntegerPtr; col: NSIntegerPtr; aCell: NSCell): Boolean; message 'getRow:column:ofCell:';
    function getRow_column_forPoint(row: NSIntegerPtr; col: NSIntegerPtr; aPoint: NSPoint): Boolean; message 'getRow:column:forPoint:';
    procedure renewRows_columns(newRows: NSInteger; newCols: NSInteger); message 'renewRows:columns:';
    procedure putCell_atRow_column(newCell: NSCell; row: NSInteger; col: NSInteger); message 'putCell:atRow:column:';
    procedure addRow; message 'addRow';
    procedure addRowWithCells(newCells: NSArray); message 'addRowWithCells:';
    procedure insertRow(row: NSInteger); message 'insertRow:';
    procedure insertRow_withCells(row: NSInteger; newCells: NSArray); message 'insertRow:withCells:';
    procedure removeRow(row: NSInteger); message 'removeRow:';
    procedure addColumn; message 'addColumn';
    procedure addColumnWithCells(newCells: NSArray); message 'addColumnWithCells:';
    procedure insertColumn(column: NSInteger); message 'insertColumn:';
    procedure insertColumn_withCells(column: NSInteger; newCells: NSArray); message 'insertColumn:withCells:';
    procedure removeColumn(col: NSInteger); message 'removeColumn:';
    function cellWithTag(anInt: NSInteger): id; message 'cellWithTag:';
    function doubleAction: SEL; message 'doubleAction';
    procedure setDoubleAction(aSelector: SEL); message 'setDoubleAction:';
    procedure setAutosizesCells(flag: Boolean); message 'setAutosizesCells:';
    function autosizesCells: Boolean; message 'autosizesCells';
    procedure sizeToCells; message 'sizeToCells';
    procedure setValidateSize(flag: Boolean); message 'setValidateSize:';
    procedure drawCellAtRow_column(row: NSInteger; col: NSInteger); message 'drawCellAtRow:column:';
    procedure highlightCell_atRow_column(flag: Boolean; row: NSInteger; col: NSInteger); message 'highlightCell:atRow:column:';
    procedure setAutoscroll(flag: Boolean); message 'setAutoscroll:';
    function isAutoscroll: Boolean; message 'isAutoscroll';
    procedure scrollCellToVisibleAtRow_column(row: NSInteger; col: NSInteger); message 'scrollCellToVisibleAtRow:column:';
    function mouseDownFlags: NSInteger; message 'mouseDownFlags';
    procedure mouseDown(theEvent: NSEvent); message 'mouseDown:';
    function performKeyEquivalent(theEvent: NSEvent): Boolean; message 'performKeyEquivalent:';
    function sendAction: Boolean; message 'sendAction';
    procedure sendDoubleAction; message 'sendDoubleAction';
    function delegate: NSMatrixDelegateProtocol; message 'delegate';
    procedure setDelegate(anObject: NSMatrixDelegateProtocol); message 'setDelegate:';
    function textShouldBeginEditing(textObject: NSText): Boolean; message 'textShouldBeginEditing:';
    function textShouldEndEditing(textObject: NSText): Boolean; message 'textShouldEndEditing:';
    procedure textDidBeginEditing(notification: NSNotification); message 'textDidBeginEditing:';
    procedure textDidEndEditing(notification: NSNotification); message 'textDidEndEditing:';
    procedure textDidChange(notification: NSNotification); message 'textDidChange:';
    procedure selectText(sender: id); message 'selectText:';
    function selectTextAtRow_column(row: NSInteger; col: NSInteger): id; message 'selectTextAtRow:column:';
    function acceptsFirstMouse(theEvent: NSEvent): Boolean; message 'acceptsFirstMouse:';
    procedure resetCursorRects; message 'resetCursorRects';
    procedure setToolTip_forCell(toolTipString: NSString; cell_: NSCell); message 'setToolTip:forCell:';
    function toolTipForCell(cell_: NSCell): NSString; message 'toolTipForCell:';

    { Adopted Protocols }
    function validateUserInterfaceItem(anItem: NSValidatedUserInterfaceItemProtocol): Boolean;
  end;

{ NSKeyboardUI_NSMatrixCategory }
  NSKeyboardUI_NSMatrixCategory = objccategory external name 'NSKeyboardUI' (NSMatrix)
    procedure setTabKeyTraversesCells(flag: Boolean); message 'setTabKeyTraversesCells:';
    function tabKeyTraversesCells: Boolean; message 'tabKeyTraversesCells';
    procedure setKeyCell(keyCell_: NSCell); message 'setKeyCell:';
    function keyCell: id; message 'keyCell';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSMATRIX_PAS_P}
{$define NSMATRIX_PAS_P}
  
{ NSMatrixDelegate Protocol }
  NSMatrixDelegateProtocol = objcprotocol external name 'NSMatrixDelegate'
  end;
{$endif}
{$endif}
