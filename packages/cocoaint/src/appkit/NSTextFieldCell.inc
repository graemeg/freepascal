{ Parsed from Appkit.framework NSTextFieldCell.h }

{$ifdef TYPES}
{$ifndef NSTEXTFIELDCELL_PAS_T}
{$define NSTEXTFIELDCELL_PAS_T}

{ Constants }

const
  NSTextFieldSquareBezel = 0;
  NSTextFieldRoundedBezel = 1;

{ Types }
type
  NSTextFieldBezelStyle = NSUInteger;
  NSTextFieldBezelStylePtr = ^NSTextFieldBezelStyle;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTEXTFIELDCELL_PAS_R}
{$define NSTEXTFIELDCELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTEXTFIELDCELL_PAS_F}
{$define NSTEXTFIELDCELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTEXTFIELDCELL_PAS_S}
{$define NSTEXTFIELDCELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTextFieldCell = objcclass;
  NSTextFieldCellPointer = ^NSTextFieldCell;
  NSTextFieldCellPtr = NSTextFieldCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTEXTFIELDCELL_PAS_C}
{$define NSTEXTFIELDCELL_PAS_C}

{ NSTextFieldCell }
  NSTextFieldCell = objcclass external (NSActionCell)
  private
    _backgroundColor: NSColor;
    _textColor: NSColor;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __tfFlags: record
      case byte of
       0: (_anonbitfield_NSTextFieldCell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __tfFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSTextFieldCell0: cuint);
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
        _drawsBackground: 0..1;
        _bezelStyle: 0..((1 shl 3)-1);
        thcSortDirection: 0..((1 shl 2)-1);
        thcSortPriority: 0..((1 shl 4)-1);
        mini: 0..1;
        textColorIgnoresNormalDisableFlag: 0..1;
        textColorDisableFlag: 0..1;
        thcForceHighlightForSort: 0..1;
        invalidTextColor: 0..1;
        notificationForMarkedText: 0..1;
        reservedTextFieldCell: 0..((1 shl 16)-1);
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
    procedure setBackgroundColor(color: NSColor); message 'setBackgroundColor:';
    function backgroundColor: NSColor; message 'backgroundColor';
    procedure setDrawsBackground(flag: Boolean); message 'setDrawsBackground:';
    function drawsBackground: Boolean; message 'drawsBackground';
    procedure setTextColor(color: NSColor); message 'setTextColor:';
    function textColor: NSColor; message 'textColor';
    function setUpFieldEditorAttributes(textObj: NSText): NSText; message 'setUpFieldEditorAttributes:';
    procedure setBezelStyle(style: NSTextFieldBezelStyle); message 'setBezelStyle:';
    function bezelStyle: NSTextFieldBezelStyle; message 'bezelStyle';
    procedure setPlaceholderString(string_: NSString); message 'setPlaceholderString:';
    function placeholderString: NSString; message 'placeholderString';
    procedure setPlaceholderAttributedString(string_: NSAttributedString); message 'setPlaceholderAttributedString:';
    function placeholderAttributedString: NSAttributedString; message 'placeholderAttributedString';
    procedure setWantsNotificationForMarkedText(flag: Boolean); message 'setWantsNotificationForMarkedText:';
    function allowedInputSourceLocales: NSArray; message 'allowedInputSourceLocales';
    procedure setAllowedInputSourceLocales(localeIdentifiers: NSArray); message 'setAllowedInputSourceLocales:';
  end;

{$endif}
{$endif}
