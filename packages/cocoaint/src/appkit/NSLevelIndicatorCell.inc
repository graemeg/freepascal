{ Parsed from Appkit.framework NSLevelIndicatorCell.h }

{$ifdef TYPES}
{$ifndef NSLEVELINDICATORCELL_PAS_T}
{$define NSLEVELINDICATORCELL_PAS_T}

{ Constants }

const
  NSRelevancyLevelIndicatorStyle = 0;
  NSContinuousCapacityLevelIndicatorStyle = 1;
  NSDiscreteCapacityLevelIndicatorStyle = 2;
  NSRatingLevelIndicatorStyle = 3;

{ Types }
type
  NSLevelIndicatorStyle = NSUInteger;
  NSLevelIndicatorStylePtr = ^NSLevelIndicatorStyle;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSLEVELINDICATORCELL_PAS_R}
{$define NSLEVELINDICATORCELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSLEVELINDICATORCELL_PAS_F}
{$define NSLEVELINDICATORCELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSLEVELINDICATORCELL_PAS_S}
{$define NSLEVELINDICATORCELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSLevelIndicatorCell = objcclass;
  NSLevelIndicatorCellPointer = ^NSLevelIndicatorCell;
  NSLevelIndicatorCellPtr = NSLevelIndicatorCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSLEVELINDICATORCELL_PAS_C}
{$define NSLEVELINDICATORCELL_PAS_C}

{ NSLevelIndicatorCell }
  NSLevelIndicatorCell = objcclass external (NSActionCell)
  private
    _value: double;
    _minValue: double;
    _maxValue: double;
    _warningValue: double;
    _criticalValue: double;
    _numberOfTickMarks: cint;
    _numberOfMajorTickMarks: cint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __liFlags: record
      case byte of
       0: (_anonbitfield_NSLevelIndicatorCell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __liFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSLevelIndicatorCell0: cuint);
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
        indicatorStyle: 0..((1 shl 4)-1);
        _tickMarkPosition: 0..1;
        selectable: 0..1;
        reserved: 0..((1 shl 26)-1);
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
    _cellFrame: NSRect;
    _reserved1: cint;
    _reserved2: cint;
    _reserved3: cint;
    _reserved4: cint;
    
  public
    function initWithLevelIndicatorStyle(levelIndicatorStyle_: NSLevelIndicatorStyle): id; message 'initWithLevelIndicatorStyle:';
    function levelIndicatorStyle: NSLevelIndicatorStyle; message 'levelIndicatorStyle';
    procedure setLevelIndicatorStyle(levelIndicatorStyle_: NSLevelIndicatorStyle); message 'setLevelIndicatorStyle:';
    function minValue: double; message 'minValue';
    procedure setMinValue(minValue_: double); message 'setMinValue:';
    function maxValue: double; message 'maxValue';
    procedure setMaxValue(maxValue_: double); message 'setMaxValue:';
    function warningValue: double; message 'warningValue';
    procedure setWarningValue(warningValue_: double); message 'setWarningValue:';
    function criticalValue: double; message 'criticalValue';
    procedure setCriticalValue(criticalValue_: double); message 'setCriticalValue:';
    procedure setTickMarkPosition(position: NSTickMarkPosition); message 'setTickMarkPosition:';
    function tickMarkPosition: NSTickMarkPosition; message 'tickMarkPosition';
    procedure setNumberOfTickMarks(count: NSInteger); message 'setNumberOfTickMarks:';
    function numberOfTickMarks: NSInteger; message 'numberOfTickMarks';
    procedure setNumberOfMajorTickMarks(count: NSInteger); message 'setNumberOfMajorTickMarks:';
    function numberOfMajorTickMarks: NSInteger; message 'numberOfMajorTickMarks';
    function rectOfTickMarkAtIndex(index: NSInteger): NSRect; message 'rectOfTickMarkAtIndex:';
    function tickMarkValueAtIndex(index: NSInteger): double; message 'tickMarkValueAtIndex:';
    procedure setImage(image_: NSImage); message 'setImage:';
  end;

{$endif}
{$endif}
