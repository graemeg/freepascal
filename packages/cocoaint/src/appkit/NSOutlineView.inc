{ Parsed from Appkit.framework NSOutlineView.h }

{$ifdef TYPES}
{$ifndef NSOUTLINEVIEW_PAS_T}
{$define NSOUTLINEVIEW_PAS_T}

{ Constants }

const
  NSOutlineViewDropOnItemIndex = -1;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSOUTLINEVIEW_PAS_R}
{$define NSOUTLINEVIEW_PAS_R}

{ Records }
type
  __OvFlags = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    case byte of
    0: (_anonbitfield_NSOutlineView0: cuint);
    1: (data: bitpacked record
{$ifdef fpc_big_endian}
          delegateWillDisplayCell: 0..1;
          delegateShouldEditTableColumn: 0..1;
          delegateShouldSelectItem: 0..1;
          delegateShouldSelectTableColumn: 0..1;
          delegateSelectionShouldChangeInOutlineView: 0..1;
          delegateShouldCollapseItem: 0..1;
          delegateShouldExpandItem: 0..1;
          autoresizesOutlineColumn: 0..1;
          autoSaveExpandItems: 0..1;
          enableExpandNotifications: 0..1;
          delegateWillDisplayOutlineCell: 0..1;
          removeChildInProgress: 0..1;
          selectionAdjustmentDisabled: 0..1;
          autoExpandFlashState: 0..1;
          _unused: 0..1;
          delegateHeightOfRowByItem: 0..1;
          delayRowEntryFreeDisabled: 0..1;
          numberOfRowsDataExpandEntered: 0..1;
          validDataSourceMethods: 0..1;
          reloadingData: 0..1;
          dataSourceDraggingEndedAt: 0..1;
          dataSourceDraggedImageMovedTo: 0..1;
          delegateShouldShowOutlineCellForItem: 0..1;
          delegateAutoExpandItem: 0..1;
          delegateAutoCollapseItem: 0..1;
          delegateShouldAutoExpandItem: 0..1;
          flashingEntireRow: 0..1;
          flashingOutlineCell: 0..1;
          dontRedisplayOnFrameChange: 0..1;
          __reserved: 0..((1 shl 3)-1);
{$else}
          __reserved: 0..((1 shl 3)-1);
          dontRedisplayOnFrameChange: 0..1;
          flashingOutlineCell: 0..1;
          flashingEntireRow: 0..1;
          delegateShouldAutoExpandItem: 0..1;
          delegateAutoCollapseItem: 0..1;
          delegateAutoExpandItem: 0..1;
          delegateShouldShowOutlineCellForItem: 0..1;
          dataSourceDraggedImageMovedTo: 0..1;
          dataSourceDraggingEndedAt: 0..1;
          reloadingData: 0..1;
          validDataSourceMethods: 0..1;
          numberOfRowsDataExpandEntered: 0..1;
          delayRowEntryFreeDisabled: 0..1;
          delegateHeightOfRowByItem: 0..1;
          _unused: 0..1;
          autoExpandFlashState: 0..1;
          selectionAdjustmentDisabled: 0..1;
          removeChildInProgress: 0..1;
          delegateWillDisplayOutlineCell: 0..1;
          enableExpandNotifications: 0..1;
          autoSaveExpandItems: 0..1;
          autoresizesOutlineColumn: 0..1;
          delegateShouldExpandItem: 0..1;
          delegateShouldCollapseItem: 0..1;
          delegateSelectionShouldChangeInOutlineView: 0..1;
          delegateShouldSelectTableColumn: 0..1;
          delegateShouldSelectItem: 0..1;
          delegateShouldEditTableColumn: 0..1;
          delegateWillDisplayCell: 0..1;
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
    delegateWillDisplayCell: 0..1;
    delegateShouldEditTableColumn: 0..1;
    delegateShouldSelectItem: 0..1;
    delegateShouldSelectTableColumn: 0..1;
    delegateSelectionShouldChangeInOutlineView: 0..1;
    delegateShouldCollapseItem: 0..1;
    delegateShouldExpandItem: 0..1;
    autoresizesOutlineColumn: 0..1;
    autoSaveExpandItems: 0..1;
    enableExpandNotifications: 0..1;
    delegateWillDisplayOutlineCell: 0..1;
    removeChildInProgress: 0..1;
    selectionAdjustmentDisabled: 0..1;
    autoExpandFlashState: 0..1;
    _unused: 0..1;
    delegateHeightOfRowByItem: 0..1;
    delayRowEntryFreeDisabled: 0..1;
    numberOfRowsDataExpandEntered: 0..1;
    validDataSourceMethods: 0..1;
    reloadingData: 0..1;
    dataSourceDraggingEndedAt: 0..1;
    dataSourceDraggedImageMovedTo: 0..1;
    delegateShouldShowOutlineCellForItem: 0..1;
    delegateAutoExpandItem: 0..1;
    delegateAutoCollapseItem: 0..1;
    delegateShouldAutoExpandItem: 0..1;
    flashingEntireRow: 0..1;
    flashingOutlineCell: 0..1;
    dontRedisplayOnFrameChange: 0..1;
    __reserved: 0..((1 shl 3)-1);
{$else}
    __reserved: 0..((1 shl 3)-1);
    dontRedisplayOnFrameChange: 0..1;
    flashingOutlineCell: 0..1;
    flashingEntireRow: 0..1;
    delegateShouldAutoExpandItem: 0..1;
    delegateAutoCollapseItem: 0..1;
    delegateAutoExpandItem: 0..1;
    delegateShouldShowOutlineCellForItem: 0..1;
    dataSourceDraggedImageMovedTo: 0..1;
    dataSourceDraggingEndedAt: 0..1;
    reloadingData: 0..1;
    validDataSourceMethods: 0..1;
    numberOfRowsDataExpandEntered: 0..1;
    delayRowEntryFreeDisabled: 0..1;
    delegateHeightOfRowByItem: 0..1;
    _unused: 0..1;
    autoExpandFlashState: 0..1;
    selectionAdjustmentDisabled: 0..1;
    removeChildInProgress: 0..1;
    delegateWillDisplayOutlineCell: 0..1;
    enableExpandNotifications: 0..1;
    autoSaveExpandItems: 0..1;
    autoresizesOutlineColumn: 0..1;
    delegateShouldExpandItem: 0..1;
    delegateShouldCollapseItem: 0..1;
    delegateSelectionShouldChangeInOutlineView: 0..1;
    delegateShouldSelectTableColumn: 0..1;
    delegateShouldSelectItem: 0..1;
    delegateShouldEditTableColumn: 0..1;
    delegateWillDisplayCell: 0..1;
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
_OVFlags = __OvFlags;
__OvFlagsPtr = ^__OvFlags;


{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSOUTLINEVIEW_PAS_F}
{$define NSOUTLINEVIEW_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSOUTLINEVIEW_PAS_S}
{$define NSOUTLINEVIEW_PAS_S}

{ External string constants }
var
  NSOutlineViewSelectionDidChangeNotification: NSString; cvar; external;
  NSOutlineViewColumnDidMoveNotification: NSString; cvar; external;
  NSOutlineViewColumnDidResizeNotification: NSString; cvar; external;
  NSOutlineViewSelectionIsChangingNotification: NSString; cvar; external;
  NSOutlineViewItemWillExpandNotification: NSString; cvar; external;
  NSOutlineViewItemDidExpandNotification: NSString; cvar; external;
  NSOutlineViewItemWillCollapseNotification: NSString; cvar; external;
  NSOutlineViewItemDidCollapseNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSOutlineViewDataSourceProtocol = objcprotocol;
  NSOutlineViewDelegateProtocol = objcprotocol;
  NSOutlineView = objcclass;
  NSOutlineViewPointer = ^NSOutlineView;
  NSOutlineViewPtr = NSOutlineViewPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSOUTLINEVIEW_PAS_C}
{$define NSOUTLINEVIEW_PAS_C}

{ NSOutlineView }
  NSOutlineView = objcclass external (NSTableView)
  private
    _numberOfRows: NSInteger;
    __rowEntryTree: Pointer;
    _itemToEntryMap: CFMutableDictionaryRef;
    _unused2: NSInteger;
    _unused3: NSInteger;
    _unused1: NSInteger;
    _outlineTableColumn: NSTableColumn;
    _initedRows: Boolean;
    _indentationMarkerInCell: Boolean;
    _indentationPerLevel: NSInteger;
    _outlineCell: NSButtonCell;
    _trackingOutlineFrame: NSRect;
    _tracker: NSMouseTracker;
    _unused4: id;
    _ovFlags: _OVFlags;
    _ovLock: id;
    _indentArray: pclong; {garbage collector: __strong }
    _originalWidth: clong;
    _expandSet: id;
    _expandSetToExpandItemsInto: id;
    _indentArraySize: clong;
    _trackingOutlineCell: NSButtonCell;
    _trackingRow: NSInteger;
    _ovReserved: id;
    
  public
    procedure setDelegate(anObject: NSOutlineViewDelegateProtocol); message 'setDelegate:';
    function delegate: NSOutlineViewDelegateProtocol; message 'delegate';
    procedure setDataSource(aSource: NSOutlineViewDataSourceProtocol); message 'setDataSource:';
    function dataSource: NSOutlineViewDataSourceProtocol; message 'dataSource';
    procedure setOutlineTableColumn(outlineTableColumn_: NSTableColumn); message 'setOutlineTableColumn:';
    function outlineTableColumn: NSTableColumn; message 'outlineTableColumn';
    function isExpandable(item: id): Boolean; message 'isExpandable:';
    procedure expandItem_expandChildren(item: id; expandChildren: Boolean); message 'expandItem:expandChildren:';
    procedure expandItem(item: id); message 'expandItem:';
    procedure collapseItem_collapseChildren(item: id; collapseChildren: Boolean); message 'collapseItem:collapseChildren:';
    procedure collapseItem(item: id); message 'collapseItem:';
    procedure reloadItem_reloadChildren(item: id; reloadChildren: Boolean); message 'reloadItem:reloadChildren:';
    procedure reloadItem(item: id); message 'reloadItem:';
    function parentForItem(item: id): id; message 'parentForItem:';
    function itemAtRow(row: NSInteger): id; message 'itemAtRow:';
    function rowForItem(item: id): NSInteger; message 'rowForItem:';
    function levelForItem(item: id): NSInteger; message 'levelForItem:';
    function levelForRow(row: NSInteger): NSInteger; message 'levelForRow:';
    function isItemExpanded(item: id): Boolean; message 'isItemExpanded:';
    procedure setIndentationPerLevel(indentationPerLevel_: CGFloat); message 'setIndentationPerLevel:';
    function indentationPerLevel: CGFloat; message 'indentationPerLevel';
    procedure setIndentationMarkerFollowsCell(drawInCell: Boolean); message 'setIndentationMarkerFollowsCell:';
    function indentationMarkerFollowsCell: Boolean; message 'indentationMarkerFollowsCell';
    procedure setAutoresizesOutlineColumn(resize: Boolean); message 'setAutoresizesOutlineColumn:';
    function autoresizesOutlineColumn: Boolean; message 'autoresizesOutlineColumn';
    function frameOfOutlineCellAtRow(row: NSInteger): NSRect; message 'frameOfOutlineCellAtRow:';
    procedure setDropItem_dropChildIndex(item: id; index: NSInteger); message 'setDropItem:dropChildIndex:';
    function shouldCollapseAutoExpandedItemsForDeposited(deposited: Boolean): Boolean; message 'shouldCollapseAutoExpandedItemsForDeposited:';
    function autosaveExpandedItems: Boolean; message 'autosaveExpandedItems';
    procedure setAutosaveExpandedItems(save: Boolean); message 'setAutosaveExpandedItems:';
  end;

{ NSOutlineViewNotificationsCategory }
  NSOutlineViewNotificationsCategory = objccategory external (NSObject)
    procedure outlineViewSelectionDidChange(notification: NSNotification); message 'outlineViewSelectionDidChange:';
    procedure outlineViewColumnDidMove(notification: NSNotification); message 'outlineViewColumnDidMove:';
    procedure outlineViewColumnDidResize(notification: NSNotification); message 'outlineViewColumnDidResize:';
    procedure outlineViewSelectionIsChanging(notification: NSNotification); message 'outlineViewSelectionIsChanging:';
    procedure outlineViewItemWillExpand(notification: NSNotification); message 'outlineViewItemWillExpand:';
    procedure outlineViewItemDidExpand(notification: NSNotification); message 'outlineViewItemDidExpand:';
    procedure outlineViewItemWillCollapse(notification: NSNotification); message 'outlineViewItemWillCollapse:';
    procedure outlineViewItemDidCollapse(notification: NSNotification); message 'outlineViewItemDidCollapse:';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSOUTLINEVIEW_PAS_P}
{$define NSOUTLINEVIEW_PAS_P}
  
{ NSOutlineViewDataSource Protocol }
  NSOutlineViewDataSourceProtocol = objcprotocol external name 'NSOutlineViewDataSource'
  optional
    function outlineView_child_ofItem(outlineView: NSOutlineView; index: NSInteger; item: id): id; message 'outlineView:child:ofItem:';
    function outlineView_isItemExpandable(outlineView: NSOutlineView; item: id): Boolean; message 'outlineView:isItemExpandable:';
    function outlineView_numberOfChildrenOfItem(outlineView: NSOutlineView; item: id): NSInteger; message 'outlineView:numberOfChildrenOfItem:';
    function outlineView_objectValueForTableColumn_byItem(outlineView: NSOutlineView; tableColumn: NSTableColumn; item: id): id; message 'outlineView:objectValueForTableColumn:byItem:';
    procedure outlineView_setObjectValue_forTableColumn_byItem(outlineView: NSOutlineView; object_: id; tableColumn: NSTableColumn; item: id); message 'outlineView:setObjectValue:forTableColumn:byItem:';
    function outlineView_itemForPersistentObject(outlineView: NSOutlineView; object_: id): id; message 'outlineView:itemForPersistentObject:';
    function outlineView_persistentObjectForItem(outlineView: NSOutlineView; item: id): id; message 'outlineView:persistentObjectForItem:';
    procedure outlineView_sortDescriptorsDidChange(outlineView: NSOutlineView; oldDescriptors: NSArray); message 'outlineView:sortDescriptorsDidChange:';
    function outlineView_writeItems_toPasteboard(outlineView: NSOutlineView; items: NSArray; pasteboard: NSPasteboard): Boolean; message 'outlineView:writeItems:toPasteboard:';
    function outlineView_validateDrop_proposedItem_proposedChildIndex(outlineView: NSOutlineView; info: NSDraggingInfoProtocol; item: id; index: NSInteger): NSDragOperation; message 'outlineView:validateDrop:proposedItem:proposedChildIndex:';
    function outlineView_acceptDrop_item_childIndex(outlineView: NSOutlineView; info: NSDraggingInfoProtocol; item: id; index: NSInteger): Boolean; message 'outlineView:acceptDrop:item:childIndex:';
    function outlineView_namesOfPromisedFilesDroppedAtDestination_forDraggedItems(outlineView: NSOutlineView; dropDestination: NSURL; items: NSArray): NSArray; message 'outlineView:namesOfPromisedFilesDroppedAtDestination:forDraggedItems:';
  end;
  
{ NSOutlineViewDelegate Protocol }
  NSOutlineViewDelegateProtocol = objcprotocol external name 'NSOutlineViewDelegate'
  optional
    procedure outlineView_willDisplayCell_forTableColumn_item(outlineView: NSOutlineView; cell: id; tableColumn: NSTableColumn; item: id); message 'outlineView:willDisplayCell:forTableColumn:item:';
    function outlineView_shouldEditTableColumn_item(outlineView: NSOutlineView; tableColumn: NSTableColumn; item: id): Boolean; message 'outlineView:shouldEditTableColumn:item:';
    function selectionShouldChangeInOutlineView(outlineView: NSOutlineView): Boolean; message 'selectionShouldChangeInOutlineView:';
    function outlineView_shouldSelectItem(outlineView: NSOutlineView; item: id): Boolean; message 'outlineView:shouldSelectItem:';
    function outlineView_selectionIndexesForProposedSelection(outlineView: NSOutlineView; proposedSelectionIndexes: NSIndexSet): NSIndexSet; message 'outlineView:selectionIndexesForProposedSelection:';
    function outlineView_shouldSelectTableColumn(outlineView: NSOutlineView; tableColumn: NSTableColumn): Boolean; message 'outlineView:shouldSelectTableColumn:';
    procedure outlineView_mouseDownInHeaderOfTableColumn(outlineView: NSOutlineView; tableColumn: NSTableColumn); message 'outlineView:mouseDownInHeaderOfTableColumn:';
    procedure outlineView_didClickTableColumn(outlineView: NSOutlineView; tableColumn: NSTableColumn); message 'outlineView:didClickTableColumn:';
    procedure outlineView_didDragTableColumn(outlineView: NSOutlineView; tableColumn: NSTableColumn); message 'outlineView:didDragTableColumn:';
    function outlineView_toolTipForCell_rect_tableColumn_item_mouseLocation(outlineView: NSOutlineView; cell: NSCell; rect: NSRectPointer; tableColumn: NSTableColumn; item: id; mouseLocation: NSPoint): NSString; message 'outlineView:toolTipForCell:rect:tableColumn:item:mouseLocation:';
    function outlineView_heightOfRowByItem(outlineView: NSOutlineView; item: id): CGFloat; message 'outlineView:heightOfRowByItem:';
    function outlineView_typeSelectStringForTableColumn_item(outlineView: NSOutlineView; tableColumn: NSTableColumn; item: id): NSString; message 'outlineView:typeSelectStringForTableColumn:item:';
    function outlineView_nextTypeSelectMatchFromItem_toItem_forString(outlineView: NSOutlineView; startItem: id; endItem: id; searchString: NSString): id; message 'outlineView:nextTypeSelectMatchFromItem:toItem:forString:';
    function outlineView_shouldTypeSelectForEvent_withCurrentSearchString(outlineView: NSOutlineView; event: NSEvent; searchString: NSString): Boolean; message 'outlineView:shouldTypeSelectForEvent:withCurrentSearchString:';
    function outlineView_shouldShowCellExpansionForTableColumn_item(outlineView: NSOutlineView; tableColumn: NSTableColumn; item: id): Boolean; message 'outlineView:shouldShowCellExpansionForTableColumn:item:';
    function outlineView_shouldTrackCell_forTableColumn_item(outlineView: NSOutlineView; cell: NSCell; tableColumn: NSTableColumn; item: id): Boolean; message 'outlineView:shouldTrackCell:forTableColumn:item:';
    function outlineView_dataCellForTableColumn_item(outlineView: NSOutlineView; tableColumn: NSTableColumn; item: id): NSCell; message 'outlineView:dataCellForTableColumn:item:';
    function outlineView_isGroupItem(outlineView: NSOutlineView; item: id): Boolean; message 'outlineView:isGroupItem:';
    function outlineView_shouldExpandItem(outlineView: NSOutlineView; item: id): Boolean; message 'outlineView:shouldExpandItem:';
    function outlineView_shouldCollapseItem(outlineView: NSOutlineView; item: id): Boolean; message 'outlineView:shouldCollapseItem:';
    procedure outlineView_willDisplayOutlineCell_forTableColumn_item(outlineView: NSOutlineView; cell: id; tableColumn: NSTableColumn; item: id); message 'outlineView:willDisplayOutlineCell:forTableColumn:item:';
    function outlineView_sizeToFitWidthOfColumn(outlineView: NSOutlineView; column: NSInteger): CGFloat; message 'outlineView:sizeToFitWidthOfColumn:';
    function outlineView_shouldReorderColumn_toColumn(outlineView: NSOutlineView; columnIndex: NSInteger; newColumnIndex: NSInteger): Boolean; message 'outlineView:shouldReorderColumn:toColumn:';
    function outlineView_shouldShowOutlineCellForItem(outlineView: NSOutlineView; item: id): Boolean; message 'outlineView:shouldShowOutlineCellForItem:';
  end;
{$endif}
{$endif}
