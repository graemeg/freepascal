{ Parsed from Appkit.framework NSBox.h }

{$ifdef TYPES}
{$ifndef NSBOX_PAS_T}
{$define NSBOX_PAS_T}

{ Constants }

const
  NSNoTitle = 0;
  NSAboveTop = 1;
  NSAtTop = 2;
  NSBelowTop = 3;
  NSAboveBottom = 4;
  NSAtBottom = 5;
  NSBelowBottom = 6;

const
  NSBoxPrimary = 0;
  NSBoxSecondary = 1;
  NSBoxSeparator = 2;
  NSBoxOldStyle = 3;
  NSBoxCustom = 4;

{ Types }
type
  NSTitlePosition = NSUInteger;
  NSTitlePositionPtr = ^NSTitlePosition;
  NSBoxType = NSUInteger;
  NSBoxTypePtr = ^NSBoxType;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSBOX_PAS_R}
{$define NSBOX_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSBOX_PAS_F}
{$define NSBOX_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSBOX_PAS_S}
{$define NSBOX_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSBox = objcclass;
  NSBoxPointer = ^NSBox;
  NSBoxPtr = NSBoxPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSBOX_PAS_C}
{$define NSBOX_PAS_C}

{ NSBox }
  NSBox = objcclass external (NSView)
  private
    _titleCell: id;
    _contentView: id;
    _offsets: NSSize;
    _borderRect: NSRect;
    _titleRect: NSRect;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __bFlags: record
      case byte of
       0: (_anonbitfield_NSBox0: NSBorderType);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __bFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSBox0: NSBorderType);
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
        _borderType: 0..((1 shl 2)-1);
        _titlePosition: 0..((1 shl 3)-1);
        backgroundTransparent: 0..1;
        reserved: 0..((1 shl 2)-1);
        needsTile: 0..1;
        transparent: 0..1;
        colorAltInterpretation: 0..1;
        _boxType: 0..((1 shl 3)-1);
        _RESERVED: 0..((1 shl 18)-1);
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
    _unused: id;
    
  public
    function borderType: NSBorderType; message 'borderType';
    function titlePosition: NSTitlePosition; message 'titlePosition';
    procedure setBorderType(aType: NSBorderType); message 'setBorderType:';
    procedure setBoxType(boxType_: NSBoxType); message 'setBoxType:';
    function boxType: NSBoxType; message 'boxType';
    procedure setTitlePosition(aPosition: NSTitlePosition); message 'setTitlePosition:';
    function title: NSString; message 'title';
    procedure setTitle(aString: NSString); message 'setTitle:';
    function titleFont: NSFont; message 'titleFont';
    procedure setTitleFont(fontObj: NSFont); message 'setTitleFont:';
    function borderRect: NSRect; message 'borderRect';
    function titleRect: NSRect; message 'titleRect';
    function titleCell: id; message 'titleCell';
    procedure sizeToFit; message 'sizeToFit';
    function contentViewMargins: NSSize; message 'contentViewMargins';
    procedure setContentViewMargins(offsetSize: NSSize); message 'setContentViewMargins:';
    procedure setFrameFromContentFrame(contentFrame: NSRect); message 'setFrameFromContentFrame:';
    function contentView: id; message 'contentView';
    procedure setContentView(aView: NSView); message 'setContentView:';
    function isTransparent: Boolean; message 'isTransparent';
    procedure setTransparent(flag: Boolean); message 'setTransparent:';
  end;

{ NSKeyboardUI_NSBoxCategory }
  NSKeyboardUI_NSBoxCategory = objccategory external name 'NSKeyboardUI' (NSBox)
    procedure setTitleWithMnemonic(stringWithAmpersand: NSString); message 'setTitleWithMnemonic:';
  end;

{ NSCustomBoxTypePropertiesCategory }
  NSCustomBoxTypePropertiesCategory = objccategory external (NSBox)
    function borderWidth: CGFloat; message 'borderWidth';
    procedure setBorderWidth(borderWidth_: CGFloat); message 'setBorderWidth:';
    function cornerRadius: CGFloat; message 'cornerRadius';
    procedure setCornerRadius(cornerRadius_: CGFloat); message 'setCornerRadius:';
    function borderColor: NSColor; message 'borderColor';
    procedure setBorderColor(borderColor_: NSColor); message 'setBorderColor:';
    function fillColor: NSColor; message 'fillColor';
    procedure setFillColor(fillColor_: NSColor); message 'setFillColor:';
  end;

{$endif}
{$endif}
