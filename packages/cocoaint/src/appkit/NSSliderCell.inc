{ Parsed from Appkit.framework NSSliderCell.h }

{$ifdef TYPES}
{$ifndef NSSLIDERCELL_PAS_T}
{$define NSSLIDERCELL_PAS_T}

{ Constants }

const
  NSTickMarkBelow = 0;
  NSTickMarkAbove = 1;
  NSTickMarkLeft = NSTickMarkAbove;
  NSTickMarkRight = NSTickMarkBelow;

const
  NSLinearSlider = 0;
  NSCircularSlider = 1;

{ Types }
type
  NSTickMarkPosition = NSUInteger;
  NSTickMarkPositionPtr = ^NSTickMarkPosition;
  NSSliderType = NSUInteger;
  NSSliderTypePtr = ^NSSliderType;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSSLIDERCELL_PAS_R}
{$define NSSLIDERCELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSSLIDERCELL_PAS_F}
{$define NSSLIDERCELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSSLIDERCELL_PAS_S}
{$define NSSLIDERCELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSSliderCell = objcclass;
  NSSliderCellPointer = ^NSSliderCell;
  NSSliderCellPtr = NSSliderCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSSLIDERCELL_PAS_C}
{$define NSSLIDERCELL_PAS_C}

{ NSSliderCell }
  NSSliderCell = objcclass external (NSActionCell)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(3)-1] of cint;
=======
        _reserved: array[0..2] of cint;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..2] of cint;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..2] of cint;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..2] of cint;
>>>>>>> origin/cpstrnew
    _numberOfTickMarks: cint;
    _altIncValue: double;
    _value: double;
    _maxValue: double;
    _minValue: double;
    _trackRect: NSRect;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __scFlags: record
      case byte of
       0: (_anonbitfield_NSSliderCell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __scFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSSliderCell0: cuint);
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
        weAreVertical: 0..1;
        weAreVerticalSet: 0..1;
        reserved1: 0..1;
        isPressed: 0..1;
        _allowsTickMarkValuesOnly: 0..1;
        _tickMarkPosition: 0..1;
        _sliderType: 0..((1 shl 2)-1);
        drawing: 0..1;
        reserved2: 0..((1 shl 23)-1);
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
    class function prefersTrackingUntilMouseUp: Boolean; message 'prefersTrackingUntilMouseUp';
    function minValue: double; message 'minValue';
    procedure setMinValue(aDouble: double); message 'setMinValue:';
    function maxValue: double; message 'maxValue';
    procedure setMaxValue(aDouble: double); message 'setMaxValue:';
    procedure setAltIncrementValue(incValue: double); message 'setAltIncrementValue:';
    function altIncrementValue: double; message 'altIncrementValue';
    function isVertical: NSInteger; message 'isVertical';
    procedure setTitleColor(newColor: NSColor); message 'setTitleColor:';
    function titleColor: NSColor; message 'titleColor';
    procedure setTitleFont(fontObj: NSFont); message 'setTitleFont:';
    function titleFont: NSFont; message 'titleFont';
    function title: NSString; message 'title';
    procedure setTitle(aString: NSString); message 'setTitle:';
    procedure setTitleCell(aCell: NSCell); message 'setTitleCell:';
    function titleCell: id; message 'titleCell';
    procedure setKnobThickness(aFloat: CGFloat); message 'setKnobThickness:';
    function knobThickness: CGFloat; message 'knobThickness';
    function knobRectFlipped(flipped: Boolean): NSRect; message 'knobRectFlipped:';
    procedure drawKnob(knobRect: NSRect); message 'drawKnob:';
    procedure drawBarInside_flipped(aRect: NSRect; flipped: Boolean); message 'drawBarInside:flipped:';
    function trackRect: NSRect; message 'trackRect';
    procedure setSliderType(sliderType_: NSSliderType); message 'setSliderType:';
    function sliderType: NSSliderType; message 'sliderType';
  end;

{ NSTickMarkSupportCategory }
  NSTickMarkSupportCategory = objccategory external (NSSliderCell)
    procedure setNumberOfTickMarks(count: NSInteger); message 'setNumberOfTickMarks:';
    function numberOfTickMarks: NSInteger; message 'numberOfTickMarks';
    procedure setTickMarkPosition(position: NSTickMarkPosition); message 'setTickMarkPosition:';
    function tickMarkPosition: NSTickMarkPosition; message 'tickMarkPosition';
    procedure setAllowsTickMarkValuesOnly(yorn: Boolean); message 'setAllowsTickMarkValuesOnly:';
    function allowsTickMarkValuesOnly: Boolean; message 'allowsTickMarkValuesOnly';
    function tickMarkValueAtIndex(index: NSInteger): double; message 'tickMarkValueAtIndex:';
    function rectOfTickMarkAtIndex(index: NSInteger): NSRect; message 'rectOfTickMarkAtIndex:';
    function indexOfTickMarkAtPoint(point: NSPoint): NSInteger; message 'indexOfTickMarkAtPoint:';
    function closestTickMarkValueToValue(value: double): double; message 'closestTickMarkValueToValue:';
  end;

{$endif}
{$endif}
