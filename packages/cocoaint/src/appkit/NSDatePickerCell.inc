{ Parsed from Appkit.framework NSDatePickerCell.h }

{$ifdef TYPES}
{$ifndef NSDATEPICKERCELL_PAS_T}
{$define NSDATEPICKERCELL_PAS_T}

{ Constants }

const
  NSTextFieldAndStepperDatePickerStyle = 0;
  NSClockAndCalendarDatePickerStyle = 1;
  NSTextFieldDatePickerStyle = 2;

const
  NSSingleDateMode = 0;
  NSRangeDateMode = 1;

const
  NSHourMinuteDatePickerElementFlag = $000c;
  NSHourMinuteSecondDatePickerElementFlag = $000e;
  NSTimeZoneDatePickerElementFlag = $0010;
  NSYearMonthDatePickerElementFlag = $00c0;
  NSYearMonthDayDatePickerElementFlag = $00e0;
  NSEraDatePickerElementFlag = $0100;

{ Types }
type
  NSDatePickerStyle = NSUInteger;
  NSDatePickerStylePtr = ^NSDatePickerStyle;
  NSDatePickerMode = NSUInteger;
  NSDatePickerModePtr = ^NSDatePickerMode;
  NSDatePickerElementFlags = NSUInteger;
  NSDatePickerElementFlagsPtr = ^NSDatePickerElementFlags;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSDATEPICKERCELL_PAS_R}
{$define NSDATEPICKERCELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSDATEPICKERCELL_PAS_F}
{$define NSDATEPICKERCELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSDATEPICKERCELL_PAS_S}
{$define NSDATEPICKERCELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSDatePickerCellDelegateProtocol = objcprotocol;
  NSDatePickerCell = objcclass;
  NSDatePickerCellPointer = ^NSDatePickerCell;
  NSDatePickerCellPtr = NSDatePickerCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSDATEPICKERCELL_PAS_C}
{$define NSDATEPICKERCELL_PAS_C}

{ NSDatePickerCell }
  NSDatePickerCell = objcclass external (NSActionCell)
  private
    _timeInterval: NSTimeInterval;
    _minDate: NSDate;
    _maxDate: NSDate;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __dcFlags: record
      case byte of
       0: (_anonbitfield_NSDatePickerCell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __dcFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSDatePickerCell0: cuint);
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
        elements: 0..((1 shl 16)-1);
        controlStyle: 0..((1 shl 3)-1);
        controlMode: 0..((1 shl 2)-1);
        trackingHand: 0..((1 shl 2)-1);
        reserved2: 0..((1 shl 4)-1);
        _drawsBackground: 0..1;
        digitsEntered: 0..((1 shl 2)-1);
        forcesLeadingZeroes: 0..1;
        wrapsDateComponentArithmetic: 0..1;
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
    _calendar: NSCalendar;
    _locale: NSLocale;
    _timeZone: NSTimeZone;
    _backgroundColor: NSColor;
    _textColor: NSColor;
    _indexOfSelectedSubfield: cint;
    _reserved0: cint;
    _reserved1: id;
    _reserved2: id;
    _reserved3: id;
    _reserved4: id;
    
  public
    function datePickerStyle: NSDatePickerStyle; message 'datePickerStyle';
    procedure setDatePickerStyle(newStyle: NSDatePickerStyle); message 'setDatePickerStyle:';
    function drawsBackground: Boolean; message 'drawsBackground';
    procedure setDrawsBackground(flag: Boolean); message 'setDrawsBackground:';
    function backgroundColor: NSColor; message 'backgroundColor';
    procedure setBackgroundColor(color: NSColor); message 'setBackgroundColor:';
    function textColor: NSColor; message 'textColor';
    procedure setTextColor(color: NSColor); message 'setTextColor:';
    function datePickerMode: NSDatePickerMode; message 'datePickerMode';
    procedure setDatePickerMode(newMode: NSDatePickerMode); message 'setDatePickerMode:';
    function datePickerElements: NSDatePickerElementFlags; message 'datePickerElements';
    procedure setDatePickerElements(elementFlags: NSDatePickerElementFlags); message 'setDatePickerElements:';
    function calendar: NSCalendar; message 'calendar';
    procedure setCalendar(newCalendar: NSCalendar); message 'setCalendar:';
    function locale: NSLocale; message 'locale';
    procedure setLocale(newLocale: NSLocale); message 'setLocale:';
    function timeZone: NSTimeZone; message 'timeZone';
    procedure setTimeZone(newTimeZone: NSTimeZone); message 'setTimeZone:';
    function dateValue: NSDate; message 'dateValue';
    procedure setDateValue(newStartDate: NSDate); message 'setDateValue:';
    function timeInterval: NSTimeInterval; message 'timeInterval';
    procedure setTimeInterval(newTimeInterval: NSTimeInterval); message 'setTimeInterval:';
    function minDate: NSDate; message 'minDate';
    procedure setMinDate(date: NSDate); message 'setMinDate:';
    function maxDate: NSDate; message 'maxDate';
    procedure setMaxDate(date: NSDate); message 'setMaxDate:';
    function delegate: NSDatePickerCellDelegateProtocol; message 'delegate';
    procedure setDelegate(anObject: NSDatePickerCellDelegateProtocol); message 'setDelegate:';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSDATEPICKERCELL_PAS_P}
{$define NSDATEPICKERCELL_PAS_P}
  
{ NSDatePickerCellDelegate Protocol }
  NSDatePickerCellDelegateProtocol = objcprotocol external name 'NSDatePickerCellDelegate'
  optional
    procedure datePickerCell_validateProposedDateValue_timeInterval(aDatePickerCell: NSDatePickerCell; proposedDateValue: NSDatePointer; proposedTimeInterval: NSTimeIntervalPtr); message 'datePickerCell:validateProposedDateValue:timeInterval:';
  end;
{$endif}
{$endif}
