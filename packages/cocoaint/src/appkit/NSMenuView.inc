{ Parsed from Appkit.framework NSMenuView.h }

{$ifdef TYPES}
{$ifndef NSMENUVIEW_PAS_T}
{$define NSMENUVIEW_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMENUVIEW_PAS_R}
{$define NSMENUVIEW_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMENUVIEW_PAS_F}
{$define NSMENUVIEW_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMENUVIEW_PAS_S}
{$define NSMENUVIEW_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSMenuView = objcclass;
  NSMenuViewPointer = ^NSMenuView;
  NSMenuViewPtr = NSMenuViewPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMENUVIEW_PAS_C}
{$define NSMENUVIEW_PAS_C}

{ NSMenuView }
  NSMenuView = objcclass external (NSView)
  private
    _menu: NSMenu;
    _cells: NSMutableArray;
    _stateImageWidth: CGFloat;
    _imageAndTitleWidth: CGFloat;
    _keyEquivalentWidth: CGFloat;
    _extents: CGFloatPtr;
    _extentsCapacity: cuint;
    _highlightedItemIndex: cint;
    _submenuPopupTimer: Pointer;
    _attachedMenuView: NSMenuView;
    _scrollArrowHeight: CGFloat;
    _maxWinHeight: CGFloat;
    _font: NSFont;
    _minSize: NSSize;
    _horizontalEdgePad: CGFloat;
    trackingState: Pointer;
    _scrollingView: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __mvFlags: record
      case byte of
       0: (_anonbitfield_NSMenuView0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __mvFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSMenuView0: cuint);
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
        _needsSizing: 0..1;
        releasingWindow: 0..1;
        _isHorizontal: 0..1;
        disableSizing: 0..1;
        attachedSubmenuWhileMouseWasUp: 0..1;
        needsToCreateCells: 0..1;
        allowsTearOffs: 0..1;
        isTearOff: 0..1;
        keyEquivGlyphWidth: 0..((1 shl 16)-1);
        disableResize: 0..1;
        savedHODState: 0..1;
        drawCenter: 0..1;
        RESERVED: 0..((1 shl 5)-1);
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
    class function menuBarHeight: CGFloat; message 'menuBarHeight';
    function initWithFrame(frame_: NSRect): id; message 'initWithFrame:';
    function initAsTearOff: id; message 'initAsTearOff';
    procedure setMenu(menu_: NSMenu); message 'setMenu:';
    function menu: NSMenu; message 'menu';
    procedure itemChanged(notification: NSNotification); message 'itemChanged:';
    procedure itemAdded(notification: NSNotification); message 'itemAdded:';
    procedure itemRemoved(notification: NSNotification); message 'itemRemoved:';
    procedure update; message 'update';
    procedure setHorizontal(flag: Boolean); message 'setHorizontal:';
    function isHorizontal: Boolean; message 'isHorizontal';
    procedure setFont(font_: NSFont); message 'setFont:';
    function font: NSFont; message 'font';
    function innerRect: NSRect; message 'innerRect';
    function rectOfItemAtIndex(index: NSInteger): NSRect; message 'rectOfItemAtIndex:';
    function indexOfItemAtPoint(point: NSPoint): NSInteger; message 'indexOfItemAtPoint:';
    procedure setNeedsDisplayForItemAtIndex(index: NSInteger); message 'setNeedsDisplayForItemAtIndex:';
    procedure setHighlightedItemIndex(index: NSInteger); message 'setHighlightedItemIndex:';
    function highlightedItemIndex: NSInteger; message 'highlightedItemIndex';
    function stateImageOffset: CGFloat; message 'stateImageOffset';
    function stateImageWidth: CGFloat; message 'stateImageWidth';
    function imageAndTitleOffset: CGFloat; message 'imageAndTitleOffset';
    function imageAndTitleWidth: CGFloat; message 'imageAndTitleWidth';
    function keyEquivalentOffset: CGFloat; message 'keyEquivalentOffset';
    function keyEquivalentWidth: CGFloat; message 'keyEquivalentWidth';
    procedure setMenuItemCell_forItemAtIndex(cell: NSMenuItemCell; index: NSInteger); message 'setMenuItemCell:forItemAtIndex:';
    function menuItemCellForItemAtIndex(index: NSInteger): NSMenuItemCell; message 'menuItemCellForItemAtIndex:';
    function attachedMenuView: NSMenuView; message 'attachedMenuView';
    procedure setNeedsSizing(flag: Boolean); message 'setNeedsSizing:';
    function needsSizing: Boolean; message 'needsSizing';
    procedure sizeToFit; message 'sizeToFit';
    function attachedMenu: NSMenu; message 'attachedMenu';
    function isAttached: Boolean; message 'isAttached';
    function isTornOff: Boolean; message 'isTornOff';
    function locationForSubmenu(aSubmenu: NSMenu): NSPoint; message 'locationForSubmenu:';
    procedure setWindowFrameForAttachingToRect_onScreen_preferredEdge_popUpSelectedItem(screenRect: NSRect; screen: NSScreen; edge: NSRectEdge; selectedItemIndex: NSInteger); message 'setWindowFrameForAttachingToRect:onScreen:preferredEdge:popUpSelectedItem:';
    procedure detachSubmenu; message 'detachSubmenu';
    procedure attachSubmenuForItemAtIndex(index: NSInteger); message 'attachSubmenuForItemAtIndex:';
    procedure performActionWithHighlightingForItemAtIndex(index: NSInteger); message 'performActionWithHighlightingForItemAtIndex:';
    function trackWithEvent(event: NSEvent): Boolean; message 'trackWithEvent:';
    function horizontalEdgePadding: CGFloat; message 'horizontalEdgePadding';
    procedure setHorizontalEdgePadding(pad: CGFloat); message 'setHorizontalEdgePadding:';
  end;

{$endif}
{$endif}
