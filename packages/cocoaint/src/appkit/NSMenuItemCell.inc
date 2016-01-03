{ Parsed from Appkit.framework NSMenuItemCell.h }

{$ifdef TYPES}
{$ifndef NSMENUITEMCELL_PAS_T}
{$define NSMENUITEMCELL_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMENUITEMCELL_PAS_R}
{$define NSMENUITEMCELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMENUITEMCELL_PAS_F}
{$define NSMENUITEMCELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMENUITEMCELL_PAS_S}
{$define NSMENUITEMCELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSMenuItemCell = objcclass;
  NSMenuItemCellPointer = ^NSMenuItemCell;
  NSMenuItemCellPtr = NSMenuItemCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMENUITEMCELL_PAS_C}
{$define NSMENUITEMCELL_PAS_C}

{ NSMenuItemCell }
  NSMenuItemCell = objcclass external (NSButtonCell)
  private
    _extraData: id;
    _stateImageSize: NSSize;
    _imageSize: NSSize;
    _titleSize: NSSize;
    _keyEquivalentSize: NSSize;
    _size: NSSize;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __micFlags: record
      case byte of
       0: (_anonbitfield_NSMenuItemCell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __micFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSMenuItemCell0: cuint);
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
        reserved: 0..1;
        _needsDisplay: 0..1;
        keyEquivGlyphWidth: 0..((1 shl 16)-1);
        uniqueAgainstMain: 0..1;
        _RESERVED: 0..((1 shl 12)-1);
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
    procedure setMenuItem(item: NSMenuItem); message 'setMenuItem:';
    function menuItem: NSMenuItem; message 'menuItem';
    procedure setMenuView(menuView_: NSMenuView); message 'setMenuView:';
    function menuView: NSMenuView; message 'menuView';
    procedure setNeedsSizing(flag: Boolean); message 'setNeedsSizing:';
    function needsSizing: Boolean; message 'needsSizing';
    procedure calcSize; message 'calcSize';
    procedure setNeedsDisplay_(flag: Boolean); message 'setNeedsDisplay:';
    function needsDisplay: Boolean; message 'needsDisplay';
    function stateImageWidth: CGFloat; message 'stateImageWidth';
    function imageWidth: CGFloat; message 'imageWidth';
    function titleWidth: CGFloat; message 'titleWidth';
    function keyEquivalentWidth: CGFloat; message 'keyEquivalentWidth';
    function stateImageRectForBounds(cellFrame: NSRect): NSRect; message 'stateImageRectForBounds:';
    function titleRectForBounds(cellFrame: NSRect): NSRect; message 'titleRectForBounds:';
    function keyEquivalentRectForBounds(cellFrame: NSRect): NSRect; message 'keyEquivalentRectForBounds:';
    procedure drawSeparatorItemWithFrame_inView(cellFrame: NSRect; controlView_: NSView); message 'drawSeparatorItemWithFrame:inView:';
    procedure drawStateImageWithFrame_inView(cellFrame: NSRect; controlView_: NSView); message 'drawStateImageWithFrame:inView:';
    procedure drawImageWithFrame_inView(cellFrame: NSRect; controlView_: NSView); message 'drawImageWithFrame:inView:';
    procedure drawTitleWithFrame_inView(cellFrame: NSRect; controlView_: NSView); message 'drawTitleWithFrame:inView:';
    procedure drawKeyEquivalentWithFrame_inView(cellFrame: NSRect; controlView_: NSView); message 'drawKeyEquivalentWithFrame:inView:';
    procedure drawBorderAndBackgroundWithFrame_inView(cellFrame: NSRect; controlView_: NSView); message 'drawBorderAndBackgroundWithFrame:inView:';
    function tag: NSInteger; message 'tag';
  end;

{$endif}
{$endif}
