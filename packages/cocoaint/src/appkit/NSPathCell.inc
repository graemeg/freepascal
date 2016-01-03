{ Parsed from Appkit.framework NSPathCell.h }

{$ifdef TYPES}
{$ifndef NSPATHCELL_PAS_T}
{$define NSPATHCELL_PAS_T}

{ Constants }

const
  NSPathStyleStandard = 0;
  NSPathStyleNavigationBar = 1;
  NSPathStylePopUp = 2;

{ Types }
type
  NSPathStyle = NSInteger;
  NSPathStylePtr = ^NSPathStyle;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPATHCELL_PAS_R}
{$define NSPATHCELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPATHCELL_PAS_F}
{$define NSPATHCELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPATHCELL_PAS_S}
{$define NSPATHCELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPathCellDelegateProtocol = objcprotocol;
  NSPathCell = objcclass;
  NSPathCellPointer = ^NSPathCell;
  NSPathCellPtr = NSPathCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPATHCELL_PAS_C}
{$define NSPATHCELL_PAS_C}

{ NSPathCell }
  NSPathCell = objcclass external (NSActionCell, NSOpenSavePanelDelegateProtocol)
  private
    _backgroundColor: NSColor;
    _borderColors: NSMutableArray;
    _cells: NSMutableArray;
    _clickedCell: NSPathComponentCell;
    _hoveredCell: NSPathComponentCell;
    _popUpButtonCell: NSPopUpButtonCell;
    _currentRect: NSRect;
    _animation: NSAnimation;
    _allowedTypes: NSArray;
    _doubleAction: SEL;
    _delegate: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __piFlags: record
      case byte of
       0: (_anonbitfield_NSPathCell0: cuint);
       1: (data: bitpacked record
        reserved: 0..((1 shl 32)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __piFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSPathCell0: cuint);
       1: (
        reserved: 0..((1 shl 32)-1);
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
      );
      end;
    _pathStyle: NSPathStyle;
    _aux: id;
    
  public
    function pathStyle: NSPathStyle; message 'pathStyle';
    procedure setPathStyle(style: NSPathStyle); message 'setPathStyle:';
    function URL: NSURL; message 'URL';
    procedure setURL(url_: NSURL); message 'setURL:';
    procedure setObjectValue(obj: NSCopyingProtocol); message 'setObjectValue:';
    function allowedTypes: NSArray; message 'allowedTypes';
    procedure setAllowedTypes(allowedTypes_: NSArray); message 'setAllowedTypes:';
    function delegate: NSPathCellDelegateProtocol; message 'delegate';
    procedure setDelegate(value: NSPathCellDelegateProtocol); message 'setDelegate:';
    class function pathComponentCellClass: Pobjc_class; message 'pathComponentCellClass';
    function pathComponentCells: NSArray; message 'pathComponentCells';
    procedure setPathComponentCells(cells: NSArray); message 'setPathComponentCells:';
    function rectOfPathComponentCell_withFrame_inView(cell: NSPathComponentCell; frame: NSRect; view: NSView): NSRect; message 'rectOfPathComponentCell:withFrame:inView:';
    function pathComponentCellAtPoint_withFrame_inView(point: NSPoint; frame: NSRect; view: NSView): NSPathComponentCell; message 'pathComponentCellAtPoint:withFrame:inView:';
    function clickedPathComponentCell: NSPathComponentCell; message 'clickedPathComponentCell';
    procedure mouseEntered_withFrame_inView(event: NSEvent; frame: NSRect; view: NSView); message 'mouseEntered:withFrame:inView:';
    procedure mouseExited_withFrame_inView(event: NSEvent; frame: NSRect; view: NSView); message 'mouseExited:withFrame:inView:';
    function doubleAction: SEL; message 'doubleAction';
    procedure setDoubleAction(action_: SEL); message 'setDoubleAction:';
    procedure setBackgroundColor(color: NSColor); message 'setBackgroundColor:';
    function backgroundColor: NSColor; message 'backgroundColor';
    procedure setPlaceholderString(string_: NSString); message 'setPlaceholderString:';
    function placeholderString: NSString; message 'placeholderString';
    procedure setPlaceholderAttributedString(string_: NSAttributedString); message 'setPlaceholderAttributedString:';
    function placeholderAttributedString: NSAttributedString; message 'placeholderAttributedString';
    procedure setControlSize(size: NSControlSize); message 'setControlSize:';

    { Adopted Protocols }
    function panel_shouldEnableURL(sender: id; url: NSURL): Boolean;
    function panel_validateURL_error(sender: id; url: NSURL; outError: NSErrorPointer): Boolean;
    procedure panel_didChangeToDirectoryURL(sender: id; url: NSURL);
    function panel_userEnteredFilename_confirmed(sender: id; filename: NSString; okFlag: Boolean): NSString;
    procedure panel_willExpand(sender: id; expanding: Boolean);
    procedure panelSelectionDidChange(sender: id);
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSPATHCELL_PAS_P}
{$define NSPATHCELL_PAS_P}
  
{ NSPathCellDelegate Protocol }
  NSPathCellDelegateProtocol = objcprotocol external name 'NSPathCellDelegate'
  optional
    procedure pathCell_willDisplayOpenPanel(pathCell: NSPathCell; openPanel: NSOpenPanel); message 'pathCell:willDisplayOpenPanel:';
    procedure pathCell_willPopUpMenu(pathCell: NSPathCell; menu: NSMenu); message 'pathCell:willPopUpMenu:';
  end;
{$endif}
{$endif}
