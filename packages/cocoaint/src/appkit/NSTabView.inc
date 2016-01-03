{ Parsed from Appkit.framework NSTabView.h }

{$ifdef TYPES}
{$ifndef NSTABVIEW_PAS_T}
{$define NSTABVIEW_PAS_T}

{ Defines }
const
  NSAppKitVersionNumberWithDirectionalTabs = 631.0;

{ Constants }

const
  NSTopTabsBezelBorder = 0;
  NSLeftTabsBezelBorder = 1;
  NSBottomTabsBezelBorder = 2;
  NSRightTabsBezelBorder = 3;
  NSNoTabsBezelBorder = 4;
  NSNoTabsLineBorder = 5;
  NSNoTabsNoBorder = 6;

{ Types }
type
  NSTabViewType = NSUInteger;
  NSTabViewTypePtr = ^NSTabViewType;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTABVIEW_PAS_R}
{$define NSTABVIEW_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTABVIEW_PAS_F}
{$define NSTABVIEW_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTABVIEW_PAS_S}
{$define NSTABVIEW_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTabViewDelegateProtocol = objcprotocol;
  NSTabView = objcclass;
  NSTabViewPointer = ^NSTabView;
  NSTabViewPtr = NSTabViewPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTABVIEW_PAS_C}
{$define NSTABVIEW_PAS_C}

{ NSTabView }
  NSTabView = objcclass external (NSView)
  private
    _tabViewItems: id;
    _selectedTabViewItem: NSTabViewItem;
    _font: NSFont;
    _tabViewType: NSTabViewType;
    _allowTruncatedLabels: Boolean;
    _delegate: id;
    _tabViewUnusedBOOL1: Boolean;
    _drawsBackground: Boolean;
    _pressedTabViewItem: NSTabViewItem;
    _endTabWidth: NSInteger;
    _maxOverlap: NSInteger;
    _tabHeight: CGFloat;
    _tabViewItemWithKeyView: NSTabViewItem;
    _originalNextKeyView: NSView;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __delegateRespondTo: record
      case byte of
       0: (_anonbitfield_NSTabView0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __delegateRespondTo: bitpacked record
      case byte of
       0: (_anonbitfield_NSTabView0: cuint);
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
        shouldSelectTabViewItem: 0..1;
        willSelectTabViewItem: 0..1;
        didSelectTabViewItem: 0..1;
        didChangeNumberOfTabViewItems: 0..1;
        reserved: 0..((1 shl 28)-1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       end;
      );
      end;
    __flags: record
      case byte of
       0: (_anonbitfield_NSTabView1: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      );
      end;
    __flags: bitpacked record
      case byte of
       0: (_anonbitfield_NSTabView1: cuint);
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
        needsLayout: 0..1;
        _controlTint: 0..((1 shl 3)-1);
        _controlSize: 0..((1 shl 2)-1);
        wiringNibConnections: 0..1;
        wiringInteriorLastKeyView: 0..1;
        originalNextKeyViewChanged: 0..1;
        liveResizeSkippedResetToolTips: 0..1;
        reserved: 0..((1 shl 22)-1);
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
    _focusedTabViewItem: NSTabViewItem;
    _tabViewUnused2: Pointer;
    
  public
    procedure selectTabViewItem(tabViewItem: NSTabViewItem); message 'selectTabViewItem:';
    procedure selectTabViewItemAtIndex(index: NSInteger); message 'selectTabViewItemAtIndex:';
    procedure selectTabViewItemWithIdentifier(identifier: id); message 'selectTabViewItemWithIdentifier:';
    procedure takeSelectedTabViewItemFromSender(sender: id); message 'takeSelectedTabViewItemFromSender:';
    procedure selectFirstTabViewItem(sender: id); message 'selectFirstTabViewItem:';
    procedure selectLastTabViewItem(sender: id); message 'selectLastTabViewItem:';
    procedure selectNextTabViewItem(sender: id); message 'selectNextTabViewItem:';
    procedure selectPreviousTabViewItem(sender: id); message 'selectPreviousTabViewItem:';
    function selectedTabViewItem: NSTabViewItem; message 'selectedTabViewItem';
    function font: NSFont; message 'font';
    function tabViewType: NSTabViewType; message 'tabViewType';
    function tabViewItems: NSArray; message 'tabViewItems';
    function allowsTruncatedLabels: Boolean; message 'allowsTruncatedLabels';
    function minimumSize: NSSize; message 'minimumSize';
    function drawsBackground: Boolean; message 'drawsBackground';
    function controlTint: NSControlTint; message 'controlTint';
    function controlSize: NSControlSize; message 'controlSize';
    procedure setFont(font_: NSFont); message 'setFont:';
    procedure setTabViewType(tabViewType_: NSTabViewType); message 'setTabViewType:';
    procedure setAllowsTruncatedLabels(allowTruncatedLabels: Boolean); message 'setAllowsTruncatedLabels:';
    procedure setDrawsBackground(flag: Boolean); message 'setDrawsBackground:';
    procedure setControlTint(controlTint_: NSControlTint); message 'setControlTint:';
    procedure setControlSize(controlSize_: NSControlSize); message 'setControlSize:';
    procedure addTabViewItem(tabViewItem: NSTabViewItem); message 'addTabViewItem:';
    procedure insertTabViewItem_atIndex(tabViewItem: NSTabViewItem; index: NSInteger); message 'insertTabViewItem:atIndex:';
    procedure removeTabViewItem(tabViewItem: NSTabViewItem); message 'removeTabViewItem:';
    procedure setDelegate(anObject: NSTabViewDelegateProtocol); message 'setDelegate:';
    function delegate: NSTabViewDelegateProtocol; message 'delegate';
    function tabViewItemAtPoint(point: NSPoint): NSTabViewItem; message 'tabViewItemAtPoint:';
    function contentRect: NSRect; message 'contentRect';
    function numberOfTabViewItems: NSInteger; message 'numberOfTabViewItems';
    function indexOfTabViewItem(tabViewItem: NSTabViewItem): NSInteger; message 'indexOfTabViewItem:';
    function tabViewItemAtIndex(index: NSInteger): NSTabViewItem; message 'tabViewItemAtIndex:';
    function indexOfTabViewItemWithIdentifier(identifier: id): NSInteger; message 'indexOfTabViewItemWithIdentifier:';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSTABVIEW_PAS_P}
{$define NSTABVIEW_PAS_P}
  
{ NSTabViewDelegate Protocol }
  NSTabViewDelegateProtocol = objcprotocol external name 'NSTabViewDelegate'
  optional
    function tabView_shouldSelectTabViewItem(tabView: NSTabView; tabViewItem: NSTabViewItem): Boolean; message 'tabView:shouldSelectTabViewItem:';
    procedure tabView_willSelectTabViewItem(tabView: NSTabView; tabViewItem: NSTabViewItem); message 'tabView:willSelectTabViewItem:';
    procedure tabView_didSelectTabViewItem(tabView: NSTabView; tabViewItem: NSTabViewItem); message 'tabView:didSelectTabViewItem:';
    procedure tabViewDidChangeNumberOfTabViewItems(TabView: NSTabView); message 'tabViewDidChangeNumberOfTabViewItems:';
  end;
{$endif}
{$endif}
