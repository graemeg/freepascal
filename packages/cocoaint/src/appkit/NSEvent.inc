{ Parsed from Appkit.framework NSEvent.h }

{$ifdef TYPES}
{$ifndef NSEVENT_PAS_T}
{$define NSEVENT_PAS_T}

{ Constants }

const
  NSLeftMouseDown = 1;
  NSLeftMouseUp = 2;
  NSRightMouseDown = 3;
  NSRightMouseUp = 4;
  NSMouseMoved = 5;
  NSLeftMouseDragged = 6;
  NSRightMouseDragged = 7;
  NSMouseEntered = 8;
  NSMouseExited = 9;
  NSKeyDown = 10;
  NSKeyUp = 11;
  NSFlagsChanged = 12;
  NSAppKitDefined = 13;
  NSSystemDefined = 14;
  NSApplicationDefined = 15;
  NSPeriodic = 16;
  NSCursorUpdate = 17;
  NSScrollWheel = 22;
  NSTabletPoint = 23;
  NSTabletProximity = 24;
  NSOtherMouseDown = 25;
  NSOtherMouseUp = 26;
  NSOtherMouseDragged = 27;
  NSEventTypeGesture = 29;
  NSEventTypeMagnify = 30;
  NSEventTypeSwipe = 31;
  NSEventTypeRotate = 18;
  NSEventTypeBeginGesture = 19;
  NSEventTypeEndGesture = 20;

const
  NSLeftMouseDownMask = 1 shl NSLeftMouseDown;
  NSLeftMouseUpMask = 1 shl NSLeftMouseUp;
  NSRightMouseDownMask = 1 shl NSRightMouseDown;
  NSRightMouseUpMask = 1 shl NSRightMouseUp;
  NSMouseMovedMask = 1 shl NSMouseMoved;
  NSLeftMouseDraggedMask = 1 shl NSLeftMouseDragged;
  NSRightMouseDraggedMask = 1 shl NSRightMouseDragged;
  NSMouseEnteredMask = 1 shl NSMouseEntered;
  NSMouseExitedMask = 1 shl NSMouseExited;
  NSKeyDownMask = 1 shl NSKeyDown;
  NSKeyUpMask = 1 shl NSKeyUp;
  NSFlagsChangedMask = 1 shl NSFlagsChanged;
  NSAppKitDefinedMask = 1 shl NSAppKitDefined;
  NSSystemDefinedMask = 1 shl NSSystemDefined;
  NSApplicationDefinedMask = 1 shl NSApplicationDefined;
  NSPeriodicMask = 1 shl NSPeriodic;
  NSCursorUpdateMask = 1 shl NSCursorUpdate;
  NSScrollWheelMask = 1 shl NSScrollWheel;
  NSTabletPointMask = 1 shl NSTabletPoint;
  NSTabletProximityMask = 1 shl NSTabletProximity;
  NSOtherMouseDownMask = 1 shl NSOtherMouseDown;
  NSOtherMouseUpMask = 1 shl NSOtherMouseUp;
  NSOtherMouseDraggedMask = 1 shl NSOtherMouseDragged;
  NSEventMaskGesture = 1 shl NSEventTypeGesture;
  NSEventMaskMagnify = 1 shl NSEventTypeMagnify;
  NSEventMaskSwipe = 1 shl NSEventTypeSwipe;
  NSEventMaskRotate = 1 shl NSEventTypeRotate;
  NSEventMaskBeginGesture = 1 shl NSEventTypeBeginGesture;
  NSEventMaskEndGesture = 1 shl NSEventTypeEndGesture;
  NSAnyEventMask = NSUIntegerMax;

const
  NSAlphaShiftKeyMask = 1 shl 16;
  NSShiftKeyMask = 1 shl 17;
  NSControlKeyMask = 1 shl 18;
  NSAlternateKeyMask = 1 shl 19;
  NSCommandKeyMask = 1 shl 20;
  NSNumericPadKeyMask = 1 shl 21;
  NSHelpKeyMask = 1 shl 22;
  NSFunctionKeyMask = 1 shl 23;
  NSDeviceIndependentModifierFlagsMask = $ffff0000;

const
  NSUnknownPointingDevice = NX_TABLET_POINTER_UNKNOWN;
  NSPenPointingDevice = NX_TABLET_POINTER_PEN;
  NSCursorPointingDevice = NX_TABLET_POINTER_CURSOR;
  NSEraserPointingDevice = NX_TABLET_POINTER_ERASER;

const
  NSPenTipMask = NX_TABLET_BUTTON_PENTIPMASK;
  NSPenLowerSideMask = NX_TABLET_BUTTON_PENLOWERSIDEMASK;
  NSPenUpperSideMask = NX_TABLET_BUTTON_PENUPPERSIDEMASK;

const
  NSUpArrowFunctionKey = $F700;
  NSDownArrowFunctionKey = $F701;
  NSLeftArrowFunctionKey = $F702;
  NSRightArrowFunctionKey = $F703;
  NSF1FunctionKey = $F704;
  NSF2FunctionKey = $F705;
  NSF3FunctionKey = $F706;
  NSF4FunctionKey = $F707;
  NSF5FunctionKey = $F708;
  NSF6FunctionKey = $F709;
  NSF7FunctionKey = $F70A;
  NSF8FunctionKey = $F70B;
  NSF9FunctionKey = $F70C;
  NSF10FunctionKey = $F70D;
  NSF11FunctionKey = $F70E;
  NSF12FunctionKey = $F70F;
  NSF13FunctionKey = $F710;
  NSF14FunctionKey = $F711;
  NSF15FunctionKey = $F712;
  NSF16FunctionKey = $F713;
  NSF17FunctionKey = $F714;
  NSF18FunctionKey = $F715;
  NSF19FunctionKey = $F716;
  NSF20FunctionKey = $F717;
  NSF21FunctionKey = $F718;
  NSF22FunctionKey = $F719;
  NSF23FunctionKey = $F71A;
  NSF24FunctionKey = $F71B;
  NSF25FunctionKey = $F71C;
  NSF26FunctionKey = $F71D;
  NSF27FunctionKey = $F71E;
  NSF28FunctionKey = $F71F;
  NSF29FunctionKey = $F720;
  NSF30FunctionKey = $F721;
  NSF31FunctionKey = $F722;
  NSF32FunctionKey = $F723;
  NSF33FunctionKey = $F724;
  NSF34FunctionKey = $F725;
  NSF35FunctionKey = $F726;
  NSInsertFunctionKey = $F727;
  NSDeleteFunctionKey = $F728;
  NSHomeFunctionKey = $F729;
  NSBeginFunctionKey = $F72A;
  NSEndFunctionKey = $F72B;
  NSPageUpFunctionKey = $F72C;
  NSPageDownFunctionKey = $F72D;
  NSPrintScreenFunctionKey = $F72E;
  NSScrollLockFunctionKey = $F72F;
  NSPauseFunctionKey = $F730;
  NSSysReqFunctionKey = $F731;
  NSBreakFunctionKey = $F732;
  NSResetFunctionKey = $F733;
  NSStopFunctionKey = $F734;
  NSMenuFunctionKey = $F735;
  NSUserFunctionKey = $F736;
  NSSystemFunctionKey = $F737;
  NSPrintFunctionKey = $F738;
  NSClearLineFunctionKey = $F739;
  NSClearDisplayFunctionKey = $F73A;
  NSInsertLineFunctionKey = $F73B;
  NSDeleteLineFunctionKey = $F73C;
  NSInsertCharFunctionKey = $F73D;
  NSDeleteCharFunctionKey = $F73E;
  NSPrevFunctionKey = $F73F;
  NSNextFunctionKey = $F740;
  NSSelectFunctionKey = $F741;
  NSExecuteFunctionKey = $F742;
  NSUndoFunctionKey = $F743;
  NSRedoFunctionKey = $F744;
  NSFindFunctionKey = $F745;
  NSHelpFunctionKey = $F746;
  NSModeSwitchFunctionKey = $F747;

const
  NSWindowExposedEventType = 0;
  NSApplicationActivatedEventType = 1;
  NSApplicationDeactivatedEventType = 2;
  NSWindowMovedEventType = 4;
  NSScreenChangedEventType = 8;
  NSAWTEventType = 16;

const
  NSPowerOffEventType = 1;

const
  NSMouseEventSubtype = NX_SUBTYPE_DEFAULT;
  NSTabletPointEventSubtype = NX_SUBTYPE_TABLET_POINT;
  NSTabletProximityEventSubtype = NX_SUBTYPE_TABLET_PROXIMITY;
  NSTouchEventSubtype = NX_SUBTYPE_MOUSE_TOUCH;

{ Types }
type
  NSEventType = NSUInteger;
  NSEventTypePtr = ^NSEventType;
  NSPointingDeviceType = NSUInteger;
  NSPointingDeviceTypePtr = ^NSPointingDeviceType;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSEVENT_PAS_R}
{$define NSEVENT_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSEVENT_PAS_F}
{$define NSEVENT_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSEVENT_PAS_S}
{$define NSEVENT_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSEvent = objcclass;
  NSEventPointer = ^NSEvent;
  NSEventPtr = NSEventPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSEVENT_PAS_C}
{$define NSEVENT_PAS_C}

{ NSEvent }
  NSEvent = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol)
  private
    _type: NSEventType;
    _location: NSPoint;
    _modifierFlags: cuint;
    _WSTimestamp: id;
    _timestamp: NSTimeInterval;
    _windowNumber: NSInteger;
    _window: NSWindow;
    _context: NSGraphicsContext;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _data: record
      case byte of
        1: (
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    _mouse: record
        _eventNumber: cint;
        _clickCount: cint;
        _pressure: single;
        {$ifdef cpu64}
        _deltaX: CGFloat;
        _deltaY: CGFloat;
        _subtype: cint;
        _buttonNumber: cshort;
        reserved1: cshort;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        reserved2: array[0..(3)-1] of cint;
        {$endif}
      end;
           );
        2: (
=======
            reserved2: array[0..2] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved2: array[0..2] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved2: array[0..2] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved2: array[0..2] of cint;
        {$endif}
      end;
>>>>>>> origin/cpstrnew
    _key: record
        keys: NSString;
        unmodKeys: NSString;
        _keyCode: cushort;
        _isARepeat: Boolean;
        {$ifdef cpu64}
        eventFlags: cint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        reserved: array[0..(5)-1] of cint;
        {$endif}
      end;
           );
        3: (
=======
            reserved: array[0..4] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved: array[0..4] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved: array[0..4] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved: array[0..4] of cint;
        {$endif}
      end;
>>>>>>> origin/cpstrnew
    _tracking: record
        _eventNumber: cint;
        _trackingNumber: NSInteger;
        _userData: Pointer;
        {$ifdef cpu64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        reserved: array[0..(6)-1] of cint;
        {$endif}
      end;
           );
        4: (
=======
            reserved: array[0..5] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved: array[0..5] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved: array[0..5] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved: array[0..5] of cint;
        {$endif}
      end;
>>>>>>> origin/cpstrnew
    _scrollWheel: record
        _deltaX: CGFloat;
        _deltaY: CGFloat;
        _deltaZ: CGFloat;
        {$ifdef cpu64}
        _subtype: cshort;
        reserved1: cshort;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        reserved2: array[0..(6)-1] of cint;
        {$endif}
      end;
           );
        5: (
=======
            reserved2: array[0..5] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved2: array[0..5] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved2: array[0..5] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved2: array[0..5] of cint;
        {$endif}
      end;
>>>>>>> origin/cpstrnew
    _axisGesture: record
        _deltaX: CGFloat;
        _deltaY: CGFloat;
        _deltaZ: CGFloat;
        {$ifdef cpu64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        reserved: array[0..(7)-1] of cint;
        {$endif}
      end;
           );
        6: (
=======
            reserved: array[0..6] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved: array[0..6] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved: array[0..6] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved: array[0..6] of cint;
        {$endif}
      end;
>>>>>>> origin/cpstrnew
    _miscGesture: record
        _subtype: cshort;
        gestureEnded: Boolean;
        reserved: Boolean;
        value: cint;
        percentage: single;
        {$ifdef cpu64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        reserved2: array[0..(7)-1] of cint;
        {$endif}
      end;
          );
       7: (
=======
            reserved2: array[0..6] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved2: array[0..6] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved2: array[0..6] of cint;
        {$endif}
      end;
>>>>>>> graemeg/cpstrnew
=======
            reserved2: array[0..6] of cint;
        {$endif}
      end;
>>>>>>> origin/cpstrnew
    _misc: record
        _subtype: cint;
        _data1: NSInteger;
        _data2: NSInteger;
        {$ifdef cpu64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        reserved: array[0..(6)-1] of cint;
        {$endif}
      end;
          );
    {$ifdef cpu64}
       8: (
    tabletPointData: array[0..(14)-1] of cint;
          );
       9: (
    tabletProximityData: array[0..(14)-1] of cint;
          );
    {$endif}
    end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            reserved: array[0..5] of cint;
        {$endif}
      end;
    {$ifdef cpu64}
        tabletPointData: array[0..13] of cint;
        tabletProximityData: array[0..13] of cint;
    {$endif}
    __data: record
      end;
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
    _eventRef: Pointer;
    {$ifdef cpu64}
    reserved1: Pointer;
    reserved2: Pointer;
    {$endif}
    
  public
    function type_: NSEventType; message 'type';
    function modifierFlags: NSUInteger; message 'modifierFlags';
    function timestamp: NSTimeInterval; message 'timestamp';
    function window: NSWindow; message 'window';
    function windowNumber: NSInteger; message 'windowNumber';
    function context: NSGraphicsContext; message 'context';
    function clickCount: NSInteger; message 'clickCount';
    function buttonNumber: NSInteger; message 'buttonNumber';
    function eventNumber: NSInteger; message 'eventNumber';
    function pressure: single; message 'pressure';
    function locationInWindow: NSPoint; message 'locationInWindow';
    function deltaX: CGFloat; message 'deltaX';
    function deltaY: CGFloat; message 'deltaY';
    function deltaZ: CGFloat; message 'deltaZ';
    function characters: NSString; message 'characters';
    function charactersIgnoringModifiers: NSString; message 'charactersIgnoringModifiers';
    function isARepeat: Boolean; message 'isARepeat';
    function keyCode: cushort; message 'keyCode';
    function trackingNumber: NSInteger; message 'trackingNumber';
    function userData: Pointer; message 'userData';
    function trackingArea: NSTrackingArea; message 'trackingArea';
    function subtype: cshort; message 'subtype';
    function data1: NSInteger; message 'data1';
    function data2: NSInteger; message 'data2';
    function eventRef: Pointer; message 'eventRef';
    class function eventWithEventRef(eventRef_: Pointer): NSEvent; message 'eventWithEventRef:';
    function CGEvent: CGEventRef; message 'CGEvent';
    class function eventWithCGEvent(cgEvent_: CGEventRef): NSEvent; message 'eventWithCGEvent:';
    class procedure setMouseCoalescingEnabled(flag: Boolean); message 'setMouseCoalescingEnabled:';
    class function isMouseCoalescingEnabled: Boolean; message 'isMouseCoalescingEnabled';
    function magnification: CGFloat; message 'magnification';
    function deviceID: NSUInteger; message 'deviceID';
    function rotation: single; message 'rotation';
    function absoluteX: NSInteger; message 'absoluteX';
    function absoluteY: NSInteger; message 'absoluteY';
    function absoluteZ: NSInteger; message 'absoluteZ';
    function buttonMask: NSUInteger; message 'buttonMask';
    function tilt: NSPoint; message 'tilt';
    function tangentialPressure: single; message 'tangentialPressure';
    function vendorDefined: id; message 'vendorDefined';
    function vendorID: NSUInteger; message 'vendorID';
    function tabletID: NSUInteger; message 'tabletID';
    function pointingDeviceID: NSUInteger; message 'pointingDeviceID';
    function systemTabletID: NSUInteger; message 'systemTabletID';
    function vendorPointingDeviceType: NSUInteger; message 'vendorPointingDeviceType';
    function pointingDeviceSerialNumber: NSUInteger; message 'pointingDeviceSerialNumber';
    function uniqueID: culonglong; message 'uniqueID';
    function capabilityMask: NSUInteger; message 'capabilityMask';
    function pointingDeviceType: NSPointingDeviceType; message 'pointingDeviceType';
    function isEnteringProximity: Boolean; message 'isEnteringProximity';
    function touchesMatchingPhase_inView(phase: NSTouchPhase; view: NSView): NSSet; message 'touchesMatchingPhase:inView:';
    class procedure startPeriodicEventsAfterDelay_withPeriod(delay: NSTimeInterval; period: NSTimeInterval); message 'startPeriodicEventsAfterDelay:withPeriod:';
    class procedure stopPeriodicEvents; message 'stopPeriodicEvents';
    class function mouseEventWithType_location_modifierFlags_timestamp_windowNumber_context_eventNumber_clickCount_pressure(type__: NSEventType; location: NSPoint; flags: NSUInteger; time: NSTimeInterval; wNum: NSInteger; context_: NSGraphicsContext; eNum: NSInteger; cNum: NSInteger; pressure_: single): NSEvent; message 'mouseEventWithType:location:modifierFlags:timestamp:windowNumber:context:eventNumber:clickCount:pressure:';
    class function keyEventWithType_location_modifierFlags_timestamp_windowNumber_context_characters_charactersIgnoringModifiers_isARepeat_keyCode(type__: NSEventType; location: NSPoint; flags: NSUInteger; time: NSTimeInterval; wNum: NSInteger; context_: NSGraphicsContext; keys: NSString; ukeys: NSString; flag: Boolean; code: cushort): NSEvent; message 'keyEventWithType:location:modifierFlags:timestamp:windowNumber:context:characters:charactersIgnoringModifiers:isARepeat:keyCode:';
    class function enterExitEventWithType_location_modifierFlags_timestamp_windowNumber_context_eventNumber_trackingNumber_userData(type__: NSEventType; location: NSPoint; flags: NSUInteger; time: NSTimeInterval; wNum: NSInteger; context_: NSGraphicsContext; eNum: NSInteger; tNum: NSInteger; data: Pointer): NSEvent; message 'enterExitEventWithType:location:modifierFlags:timestamp:windowNumber:context:eventNumber:trackingNumber:userData:';
    class function otherEventWithType_location_modifierFlags_timestamp_windowNumber_context_subtype_data1_data2(type__: NSEventType; location: NSPoint; flags: NSUInteger; time: NSTimeInterval; wNum: NSInteger; context_: NSGraphicsContext; subtype_: cshort; d: NSInteger; d1: NSInteger): NSEvent; message 'otherEventWithType:location:modifierFlags:timestamp:windowNumber:context:subtype:data1:data2:';
    class function mouseLocation: NSPoint; message 'mouseLocation';
    class function pressedMouseButtons: NSUInteger; message 'pressedMouseButtons';
    class function doubleClickInterval: NSTimeInterval; message 'doubleClickInterval';
    class function keyRepeatDelay: NSTimeInterval; message 'keyRepeatDelay';
    class function keyRepeatInterval: NSTimeInterval; message 'keyRepeatInterval';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
