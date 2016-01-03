{ Parsed from Appkit.framework NSStatusItem.h }

{$ifdef TYPES}
{$ifndef NSSTATUSITEM_PAS_T}
{$define NSSTATUSITEM_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSSTATUSITEM_PAS_R}
{$define NSSTATUSITEM_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSSTATUSITEM_PAS_F}
{$define NSSTATUSITEM_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSSTATUSITEM_PAS_S}
{$define NSSTATUSITEM_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSStatusItem = objcclass;
  NSStatusItemPointer = ^NSStatusItem;
  NSStatusItemPtr = NSStatusItemPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSSTATUSITEM_PAS_C}
{$define NSSTATUSITEM_PAS_C}

{ NSStatusItem }
  NSStatusItem = objcclass external (NSObject)
  private
    _fStatusBar: NSStatusBar;
    _fLength: CGFloat;
    _fWindow: NSWindow;
    _fView: NSView;
    _fPriority: cint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __fFlags: record
      case byte of
       0: (_anonbitfield_NSStatusItem0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __fFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSStatusItem0: cuint);
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
        customView: 0..1;
        _highlightMode: 0..1;
        hasAlternateImage: 0..1;
        hidden: 0..1;
        backgroundStyle: 0..((1 shl 4)-1);
        inAdjustLength: 0..1;
        reserved: 0..((1 shl 23)-1);
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
    _statusItemMenu: id;
    _fReserved2: id;
    _fReserved3: id;
    _fReserved4: id;
    
  public
    function statusBar: NSStatusBar; message 'statusBar';
    function length: CGFloat; message 'length';
    procedure setLength(length_: CGFloat); message 'setLength:';
  end;

{ NSStatusItemCommonCategory }
  NSStatusItemCommonCategory = objccategory external (NSStatusItem)
    function action: SEL; message 'action';
    procedure setAction(action_: SEL); message 'setAction:';
    function doubleAction: SEL; message 'doubleAction';
    procedure setDoubleAction(action_: SEL); message 'setDoubleAction:';
    function target: id; message 'target';
    procedure setTarget(target_: id); message 'setTarget:';
    function title: NSString; message 'title';
    procedure setTitle(title_: NSString); message 'setTitle:';
    function attributedTitle: NSAttributedString; message 'attributedTitle';
    procedure setAttributedTitle(title_: NSAttributedString); message 'setAttributedTitle:';
    function image: NSImage; message 'image';
    procedure setImage(image_: NSImage); message 'setImage:';
    function alternateImage: NSImage; message 'alternateImage';
    procedure setAlternateImage(image_: NSImage); message 'setAlternateImage:';
    function menu: NSMenu; message 'menu';
    procedure setMenu(menu_: NSMenu); message 'setMenu:';
    function isEnabled: Boolean; message 'isEnabled';
    procedure setEnabled(enabled: Boolean); message 'setEnabled:';
    function toolTip: NSString; message 'toolTip';
    procedure setToolTip(toolTip_: NSString); message 'setToolTip:';
    procedure setHighlightMode(highlightMode_: Boolean); message 'setHighlightMode:';
    function highlightMode: Boolean; message 'highlightMode';
    function sendActionOn(mask: NSInteger): NSInteger; message 'sendActionOn:';
    procedure popUpStatusItemMenu(menu_: NSMenu); message 'popUpStatusItemMenu:';
    procedure drawStatusBarBackgroundInRect_withHighlight(rect: NSRect; highlight: Boolean); message 'drawStatusBarBackgroundInRect:withHighlight:';
  end;

{ NSStatusItemViewCategory }
  NSStatusItemViewCategory = objccategory external (NSStatusItem)
    function view: NSView; message 'view';
    procedure setView(view_: NSView); message 'setView:';
  end;

{$endif}
{$endif}
