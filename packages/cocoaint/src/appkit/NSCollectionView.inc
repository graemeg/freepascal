{ Parsed from Appkit.framework NSCollectionView.h }

{$ifdef TYPES}
{$ifndef NSCOLLECTIONVIEW_PAS_T}
{$define NSCOLLECTIONVIEW_PAS_T}

{ Types }
type
  NSCollectionViewDropOperation = NSInteger;
  NSCollectionViewDropOperationPtr = ^NSCollectionViewDropOperation;

{ Constants }

const
  NSCollectionViewDropOn = 0;
  NSCollectionViewDropBefore = 1;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSCOLLECTIONVIEW_PAS_R}
{$define NSCOLLECTIONVIEW_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSCOLLECTIONVIEW_PAS_F}
{$define NSCOLLECTIONVIEW_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSCOLLECTIONVIEW_PAS_S}
{$define NSCOLLECTIONVIEW_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSCollectionViewDelegateProtocol = objcprotocol;
  NSCollectionViewItem = objcclass;
  NSCollectionViewItemPointer = ^NSCollectionViewItem;
  NSCollectionViewItemPtr = NSCollectionViewItemPointer;
  NSCollectionView = objcclass;
  NSCollectionViewPointer = ^NSCollectionView;
  NSCollectionViewPtr = NSCollectionViewPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSCOLLECTIONVIEW_PAS_C}
{$define NSCOLLECTIONVIEW_PAS_C}

{ NSCollectionViewItem }
  NSCollectionViewItem = objcclass external (NSViewController, NSCopyingProtocol)
  private
    _collectionView: NSCollectionView;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __cviFlags: record
      case byte of
       0: (_anonbitfield_NSCollectionView0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __cviFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSCollectionView0: cuint);
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
        _isSelected: 0..1;
        suppressSelectionChangedNotification: 0..1;
        connectionsCopied: 0..1;
        reserved: 0..((1 shl 29)-1);
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
    _cachedArchive: NSData;
    _reserved2: Pointer;
    
  public
    function collectionView: NSCollectionView; message 'collectionView';
    procedure setSelected(flag: Boolean); message 'setSelected:';
    function isSelected: Boolean; message 'isSelected';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
  end;

{ NSCollectionView }
  NSCollectionView = objcclass external (NSView)
  private
    _content: NSArray;
    _selectionIndexes: NSMutableIndexSet;
    _itemPrototype: NSCollectionViewItem;
    _minItemSize: NSSize;
    _maxItemSize: NSSize;
    _maxGridRows: NSUInteger;
    _maxGridColumns: NSUInteger;
    _backgroundColors: NSArray;
    _displayedItems: NSMutableArray;
    _animationDuration: NSTimeInterval;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __cvFlags: record
      case byte of
       0: (_anonbitfield_NSCollectionView1: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __cvFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSCollectionView1: cuint);
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
        _isFirstResponder: 0..1;
        invalidateItemViews: 0..1;
        selectable: 0..1;
        _allowsMultipleSelection: 0..1;
        avoidsEmptySelection: 0..1;
        superviewIsClipView: 0..1;
        needsUpdateGrid: 0..1;
        needsUpdateBackground: 0..1;
        gridSettingsNeedUpdate: 0..1;
        guardSetFrameSize: 0..1;
        canDisplayItems: 0..1;
        animateForDrag: 0..1;
        unarchiving: 0..1;
        reserved: 0..((1 shl 19)-1);
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
    _delegate: id;
    _backgroundLayers: NSMutableArray;
    _storedFrameSize: NSSize;
    _cachedNib: NSNib;
    _animTimer: NSTimer;
    _removedItems: NSArray;
    _resizeTimer: NSTimer;
    _draggedIndexes: NSIndexSet;
    _draggingSourceOperationMaskForLocal: NSDragOperation;
    _draggingSourceOperationMaskForNonLocal: NSDragOperation;
    _currentDropIndex: NSInteger;
    _currentDropSpaceIndex: NSInteger;
    _shiftRow: NSInteger;
    _currentDragOperation: NSDragOperation;
    _currentDropOperation: NSCollectionViewDropOperation;
    _dragTimer: NSTimer;
    _private: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(16)-1] of Pointer;
=======
        _reserved: array[0..15] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..15] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..15] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..15] of Pointer;
>>>>>>> origin/cpstrnew
    
  public
    procedure setDelegate(aDelegate: NSCollectionViewDelegateProtocol); message 'setDelegate:';
    function delegate: NSCollectionViewDelegateProtocol; message 'delegate';
    function isFirstResponder: Boolean; message 'isFirstResponder';
    procedure setContent(content_: NSArray); message 'setContent:';
    function content: NSArray; message 'content';
    procedure setSelectable(flag: Boolean); message 'setSelectable:';
    function isSelectable: Boolean; message 'isSelectable';
    procedure setAllowsMultipleSelection(flag: Boolean); message 'setAllowsMultipleSelection:';
    function allowsMultipleSelection: Boolean; message 'allowsMultipleSelection';
    procedure setSelectionIndexes(indexes: NSIndexSet); message 'setSelectionIndexes:';
    function selectionIndexes: NSIndexSet; message 'selectionIndexes';
    function newItemForRepresentedObject(object_: id): NSCollectionViewItem; message 'newItemForRepresentedObject:';
    procedure setItemPrototype(prototype: NSCollectionViewItem); message 'setItemPrototype:';
    function itemPrototype: NSCollectionViewItem; message 'itemPrototype';
    procedure setMaxNumberOfRows(number: NSUInteger); message 'setMaxNumberOfRows:';
    function maxNumberOfRows: NSUInteger; message 'maxNumberOfRows';
    procedure setMaxNumberOfColumns(number: NSUInteger); message 'setMaxNumberOfColumns:';
    function maxNumberOfColumns: NSUInteger; message 'maxNumberOfColumns';
    procedure setMinItemSize(size: NSSize); message 'setMinItemSize:';
    function minItemSize: NSSize; message 'minItemSize';
    procedure setMaxItemSize(size: NSSize); message 'setMaxItemSize:';
    function maxItemSize: NSSize; message 'maxItemSize';
    procedure setBackgroundColors(colors: NSArray); message 'setBackgroundColors:';
    function backgroundColors: NSArray; message 'backgroundColors';
    function itemAtIndex(index: NSUInteger): NSCollectionViewItem; message 'itemAtIndex:';
    function frameForItemAtIndex(index: NSUInteger): NSRect; message 'frameForItemAtIndex:';
    procedure setDraggingSourceOperationMask_forLocal(dragOperationMask: NSDragOperation; localDestination: Boolean); message 'setDraggingSourceOperationMask:forLocal:';
    function draggingImageForItemsAtIndexes_withEvent_offset(indexes: NSIndexSet; event: NSEvent; dragImageOffset: NSPointPointer): NSImage; message 'draggingImageForItemsAtIndexes:withEvent:offset:';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSCOLLECTIONVIEW_PAS_P}
{$define NSCOLLECTIONVIEW_PAS_P}
  
{ NSCollectionViewDelegate Protocol }
  NSCollectionViewDelegateProtocol = objcprotocol external name 'NSCollectionViewDelegate'
  optional
    function collectionView_canDragItemsAtIndexes_withEvent(collectionView: NSCollectionView; indexes: NSIndexSet; event: NSEvent): Boolean; message 'collectionView:canDragItemsAtIndexes:withEvent:';
    function collectionView_writeItemsAtIndexes_toPasteboard(collectionView: NSCollectionView; indexes: NSIndexSet; pasteboard: NSPasteboard): Boolean; message 'collectionView:writeItemsAtIndexes:toPasteboard:';
    function collectionView_namesOfPromisedFilesDroppedAtDestination_forDraggedItemsAtIndexes(collectionView: NSCollectionView; dropURL: NSURL; indexes: NSIndexSet): NSArray; message 'collectionView:namesOfPromisedFilesDroppedAtDestination:forDraggedItemsAtIndexes:';
    function collectionView_draggingImageForItemsAtIndexes_withEvent_offset(collectionView: NSCollectionView; indexes: NSIndexSet; event: NSEvent; dragImageOffset: NSPointPointer): NSImage; message 'collectionView:draggingImageForItemsAtIndexes:withEvent:offset:';
    function collectionView_validateDrop_proposedIndex_dropOperation(collectionView: NSCollectionView; draggingInfo: NSDraggingInfoProtocol; proposedDropIndex: NSIntegerPtr; proposedDropOperation: NSCollectionViewDropOperationPtr): NSDragOperation; message 'collectionView:validateDrop:proposedIndex:dropOperation:';
    function collectionView_acceptDrop_index_dropOperation(collectionView: NSCollectionView; draggingInfo: NSDraggingInfoProtocol; index: NSInteger; dropOperation: NSCollectionViewDropOperation): Boolean; message 'collectionView:acceptDrop:index:dropOperation:';
  end;
{$endif}
{$endif}
