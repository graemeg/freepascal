{ Parsed from Appkit.framework NSScroller.h }

{$ifdef TYPES}
{$ifndef NSSCROLLER_PAS_T}
{$define NSSCROLLER_PAS_T}

{ Constants }

const
  NSScrollerArrowsMaxEnd = 0;
  NSScrollerArrowsMinEnd = 1;
  NSScrollerArrowsDefaultSetting = 0;
  NSScrollerArrowsNone = 2;

const
  NSNoScrollerParts = 0;
  NSOnlyScrollerArrows = 1;
  NSAllScrollerParts = 2;

const
  NSScrollerNoPart = 0;
  NSScrollerDecrementPage = 1;
  NSScrollerKnob = 2;
  NSScrollerIncrementPage = 3;
  NSScrollerDecrementLine = 4;
  NSScrollerIncrementLine = 5;
  NSScrollerKnobSlot = 6;

const
  NSScrollerIncrementArrow = 0;
  NSScrollerDecrementArrow = 1;

{ Types }
type
  NSScrollArrowPosition = NSUInteger;
  NSScrollArrowPositionPtr = ^NSScrollArrowPosition;
  NSUsableScrollerParts = NSUInteger;
  NSUsableScrollerPartsPtr = ^NSUsableScrollerParts;
  NSScrollerPart = NSUInteger;
  NSScrollerPartPtr = ^NSScrollerPart;
  NSScrollerArrow = NSUInteger;
  NSScrollerArrowPtr = ^NSScrollerArrow;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSSCROLLER_PAS_R}
{$define NSSCROLLER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSSCROLLER_PAS_F}
{$define NSSCROLLER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSSCROLLER_PAS_S}
{$define NSSCROLLER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSScroller = objcclass;
  NSScrollerPointer = ^NSScroller;
  NSScrollerPtr = NSScrollerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSSCROLLER_PAS_C}
{$define NSSCROLLER_PAS_C}

{ NSScroller }
  NSScroller = objcclass external (NSControl)
  private
    _curValue: CGFloat;
    _percent: CGFloat;
    _knobSize: CGFloat;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __sFlags2: record
      case byte of
       0: (_anonbitfield_NSScroller0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __sFlags2: bitpacked record
      case byte of
       0: (_anonbitfield_NSScroller0: cuint);
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
        _hitPart: 0..((1 shl 4)-1);
        _controlSize: 0..((1 shl 2)-1);
        inMaxEnd: 0..1;
        setFloatValueOverridden: 0..1;
        setFloatValueKnobProportionOverridden: 0..1;
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
    _target: id;
    _action: SEL;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _sFlags: record
      case byte of
       0: (_anonbitfield_NSScroller1: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    _sFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSScroller1: cuint);
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
        isHoriz: 0..1;
        arrowsLoc: 0..((1 shl 2)-1);
        partsUsable: 0..((1 shl 2)-1);
        fine: 0..1;
        needsEnableFlush: 0..1;
        thumbing: 0..1;
        slotDrawn: 0..1;
        knobDrawn: 0..1;
        lit: 0..1;
        knobLit: 0..1;
        reserved: 0..1;
        _controlTint: 0..((1 shl 3)-1);
        repeatCount: 0..((1 shl 16)-1);
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
    class function scrollerWidth: CGFloat; message 'scrollerWidth';
    class function scrollerWidthForControlSize(controlSize_: NSControlSize): CGFloat; message 'scrollerWidthForControlSize:';
    procedure drawParts; message 'drawParts';
    function rectForPart(partCode: NSScrollerPart): NSRect; message 'rectForPart:';
    procedure checkSpaceForParts; message 'checkSpaceForParts';
    function usableParts: NSUsableScrollerParts; message 'usableParts';
    procedure setArrowsPosition(where: NSScrollArrowPosition); message 'setArrowsPosition:';
    function arrowsPosition: NSScrollArrowPosition; message 'arrowsPosition';
    procedure setControlTint(controlTint_: NSControlTint); message 'setControlTint:';
    function controlTint: NSControlTint; message 'controlTint';
    procedure setControlSize(controlSize_: NSControlSize); message 'setControlSize:';
    function controlSize: NSControlSize; message 'controlSize';
    procedure drawArrow_highlight(whichArrow: NSScrollerArrow; flag: Boolean); message 'drawArrow:highlight:';
    procedure drawKnob; message 'drawKnob';
    procedure drawKnobSlotInRect_highlight(slotRect: NSRect; flag: Boolean); message 'drawKnobSlotInRect:highlight:';
    procedure highlight(flag: Boolean); message 'highlight:';
    function testPart(thePoint: NSPoint): NSScrollerPart; message 'testPart:';
    procedure trackKnob(theEvent: NSEvent); message 'trackKnob:';
    procedure trackScrollButtons(theEvent: NSEvent); message 'trackScrollButtons:';
    function hitPart: NSScrollerPart; message 'hitPart';
    function knobProportion: CGFloat; message 'knobProportion';
    procedure setKnobProportion(proportion: CGFloat); message 'setKnobProportion:';
  end;

{ NSDeprecated_NSScrollerCategory }
  NSDeprecated_NSScrollerCategory = objccategory external name 'NSDeprecated' (NSScroller)
    procedure setFloatValue_knobProportion(aFloat: single; proportion: CGFloat); message 'setFloatValue:knobProportion:'; deprecated 'in Mac OS X 10.5 and later';
  end;

{$endif}
{$endif}
