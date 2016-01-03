{ Parsed from Appkit.framework NSCell.h }

{$ifdef TYPES}
{$ifndef NSCELL_PAS_T}
{$define NSCELL_PAS_T}

{ Constants }

const
  NSAnyType = 0;
  NSIntType = 1;
  NSPositiveIntType = 2;
  NSFloatType = 3;
  NSPositiveFloatType = 4;
  NSDoubleType = 6;
  NSPositiveDoubleType = 7;

const
  NSNullCellType = 0;
  NSTextCellType = 1;
  NSImageCellType = 2;

const
  NSCellDisabled = 0;
  NSCellState = 1;
  NSPushInCell = 2;
  NSCellEditable = 3;
  NSChangeGrayCell = 4;
  NSCellHighlighted = 5;
  NSCellLightsByContents = 6;
  NSCellLightsByGray = 7;
  NSChangeBackgroundCell = 8;
  NSCellLightsByBackground = 9;
  NSCellIsBordered = 10;
  NSCellHasOverlappingImage = 11;
  NSCellHasImageHorizontal = 12;
  NSCellHasImageOnLeftOrBottom = 13;
  NSCellChangesContents = 14;
  NSCellIsInsetButton = 15;
  NSCellAllowsMixedState = 16;

const
  NSNoImage = 0;
  NSImageOnly = 1;
  NSImageLeft = 2;
  NSImageRight = 3;
  NSImageBelow = 4;
  NSImageAbove = 5;
  NSImageOverlaps = 6;

const
  NSImageScaleProportionallyDown = 0;
  NSImageScaleAxesIndependently = 1;
  NSImageScaleNone = 2;
  NSImageScaleProportionallyUpOrDown = 3;

const
  NSMixedState = -1;
  NSOffState = 0;
  NSOnState = 1;

const
  NSNoCellMask = 0;
  NSContentsCellMask = 1;
  NSPushInCellMask = 2;
  NSChangeGrayCellMask = 4;
  NSChangeBackgroundCellMask = 8;

const
  NSDefaultControlTint = 0;
  NSBlueControlTint = 1;
  NSGraphiteControlTint = 6;
  NSClearControlTint = 7;

const
  NSRegularControlSize = 0;
  NSSmallControlSize = 1;
  NSMiniControlSize = 2;

const
  NSCellHitNone = 0;
  NSCellHitContentArea = 1 shl 0;
  NSCellHitEditableTextArea = 1 shl 1;
  NSCellHitTrackableArea = 1 shl 2;

const
  NSBackgroundStyleLight = 0;
  NSBackgroundStyleDark = 1;
  NSBackgroundStyleRaised = 2;
  NSBackgroundStyleLowered = 3;

const
  NSScaleProportionally = 0;
  NSScaleToFit = 1;
  NSScaleNone = 2;

{ Types }
type
  NSCellType = NSUInteger;
  NSCellTypePtr = ^NSCellType;
  NSCellAttribute = NSUInteger;
  NSCellAttributePtr = ^NSCellAttribute;
  NSCellImagePosition = NSUInteger;
  NSCellImagePositionPtr = ^NSCellImagePosition;
  NSImageScaling = NSUInteger;
  NSImageScalingPtr = ^NSImageScaling;
  NSCellStateValue = NSInteger;
  NSCellStateValuePtr = ^NSCellStateValue;
  NSControlTint = NSUInteger;
  NSControlTintPtr = ^NSControlTint;
  NSControlSize = NSUInteger;
  NSControlSizePtr = ^NSControlSize;
  NSBackgroundStyle = NSInteger;
  NSBackgroundStylePtr = ^NSBackgroundStyle;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSCELL_PAS_R}
{$define NSCELL_PAS_R}

{ Records }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  __CFlags = record
    case byte of
    0: (_anonbitfield_NSCell0: cuint);
    1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  __CFlags = bitpacked record
    case byte of
    0: (_anonbitfield_NSCell0: cuint);
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
          state: 0..1;
          highlighted: 0..1;
          disabled: 0..1;
          editable: 0..1;
          _type: 0..((1 shl 2)-1);
          vCentered: 0..1;
          hCentered: 0..1;
          bordered: 0..1;
          bezeled: 0..1;
          selectable: 0..1;
          scrollable: 0..1;
          continuous: 0..1;
          actOnMouseDown: 0..1;
          isLeaf: 0..1;
          invalidObjectValue: 0..1;
          invalidFont: 0..1;
          lineBreakMode: 0..((1 shl 3)-1);
          cellReserved1: 0..((1 shl 2)-1);
          singleLineMode: 0..1;
          actOnMouseDragged: 0..1;
          isLoaded: 0..1;
          truncateLastLine: 0..1;
          dontActOnMouseUp: 0..1;
          isWhite: 0..1;
          useUserKeyEquivalent: 0..1;
          showsFirstResponder: 0..1;
          focusRingType: 0..((1 shl 2)-1);
          wasSelectable: 0..1;
          hasInvalidObject: 0..1;
          allowsEditingTextAttributes: 0..1;
          importsGraphics: 0..1;
          alignment: 0..((1 shl 3)-1);
          layoutDirectionRTL: 0..1;
          backgroundStyle: 0..((1 shl 3)-1);
          cellReserved2: 0..((1 shl 4)-1);
          refusesFirstResponder: 0..1;
          needsHighlightedText: 0..1;
          dontAllowsUndo: 0..1;
          currentlyEditing: 0..1;
          allowsMixedState: 0..1;
          inMixedState: 0..1;
          sendsActionOnEndEditing: 0..1;
          inSendAction: 0..1;
          menuWasSet: 0..1;
          controlTint: 0..((1 shl 3)-1);
          controlSize: 0..((1 shl 2)-1);
          branchImageDisabled: 0..1;
          drawingInRevealover: 0..1;
          needsHighlightedTextHint: 0..1;
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
_CFlags = __CFlags;
__CFlagsPtr = ^__CFlags;


{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSCELL_PAS_F}
{$define NSCELL_PAS_F}

{ Functions }
procedure NSDrawThreePartImage(frame: NSRect; startCap: NSImage; centerFill: NSImage; endCap: NSImage; vertical: Boolean; op: NSCompositingOperation; alphaFraction: CGFloat; flipped: Boolean); cdecl; external;
procedure NSDrawNinePartImage(frame: NSRect; topLeftCorner: NSImage; topEdgeFill: NSImage; topRightCorner: NSImage; leftEdgeFill: NSImage; centerFill: NSImage; rightEdgeFill: NSImage; bottomLeftCorner: NSImage; bottomEdgeFill: NSImage; bottomRightCorner: NSImage; op: NSCompositingOperation; alphaFraction: CGFloat; flipped: Boolean); cdecl; external;

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSCELL_PAS_S}
{$define NSCELL_PAS_S}

{ External string constants }
var
  NSControlTintDidChangeNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSCell = objcclass;
  NSCellPointer = ^NSCell;
  NSCellPtr = NSCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSCELL_PAS_C}
{$define NSCELL_PAS_C}

{ NSCell }
  NSCell = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol)
  private
    _contents: id;
    _cFlags: _CFlags;
    _support: id;
    
  public
    class function prefersTrackingUntilMouseUp: Boolean; message 'prefersTrackingUntilMouseUp';
    function initTextCell(aString: NSString): id; message 'initTextCell:';
    function initImageCell(image_: NSImage): id; message 'initImageCell:';
    function controlView: NSView; message 'controlView';
    procedure setControlView(view: NSView); message 'setControlView:';
    function type_: NSCellType; message 'type';
    procedure setType(aType: NSCellType); message 'setType:';
    function state: NSInteger; message 'state';
    procedure setState(value: NSInteger); message 'setState:';
    function target: id; message 'target';
    procedure setTarget(anObject: id); message 'setTarget:';
    function action: SEL; message 'action';
    procedure setAction(aSelector: SEL); message 'setAction:';
    function tag: NSInteger; message 'tag';
    procedure setTag(anInt: NSInteger); message 'setTag:';
    function title: NSString; message 'title';
    procedure setTitle(aString: NSString); message 'setTitle:';
    function isOpaque: Boolean; message 'isOpaque';
    function isEnabled: Boolean; message 'isEnabled';
    procedure setEnabled(flag: Boolean); message 'setEnabled:';
    function sendActionOn(mask: NSInteger): NSInteger; message 'sendActionOn:';
    function isContinuous: Boolean; message 'isContinuous';
    procedure setContinuous(flag: Boolean); message 'setContinuous:';
    function isEditable: Boolean; message 'isEditable';
    procedure setEditable(flag: Boolean); message 'setEditable:';
    function isSelectable: Boolean; message 'isSelectable';
    procedure setSelectable(flag: Boolean); message 'setSelectable:';
    function isBordered: Boolean; message 'isBordered';
    procedure setBordered(flag: Boolean); message 'setBordered:';
    function isBezeled: Boolean; message 'isBezeled';
    procedure setBezeled(flag: Boolean); message 'setBezeled:';
    function isScrollable: Boolean; message 'isScrollable';
    procedure setScrollable(flag: Boolean); message 'setScrollable:';
    function isHighlighted: Boolean; message 'isHighlighted';
    procedure setHighlighted(flag: Boolean); message 'setHighlighted:';
    function alignment: NSTextAlignment; message 'alignment';
    procedure setAlignment(mode: NSTextAlignment); message 'setAlignment:';
    function wraps: Boolean; message 'wraps';
    procedure setWraps(flag: Boolean); message 'setWraps:';
    function font: NSFont; message 'font';
    procedure setFont(fontObj: NSFont); message 'setFont:';
    function isEntryAcceptable(aString: NSString): Boolean; message 'isEntryAcceptable:';
    function keyEquivalent: NSString; message 'keyEquivalent';
    procedure setFormatter(newFormatter: NSFormatter); message 'setFormatter:';
    function formatter: id; message 'formatter';
    function objectValue: id; message 'objectValue';
    procedure setObjectValue(obj: NSCopyingProtocol); message 'setObjectValue:';
    function hasValidObjectValue: Boolean; message 'hasValidObjectValue';
    function stringValue: NSString; message 'stringValue';
    procedure setStringValue(aString: NSString); message 'setStringValue:';
    function compare(otherCell: id): NSComparisonResult; message 'compare:';
    function intValue: cint; message 'intValue';
    procedure setIntValue(anInt: cint); message 'setIntValue:';
    function floatValue: single; message 'floatValue';
    procedure setFloatValue(aFloat: single); message 'setFloatValue:';
    function doubleValue: double; message 'doubleValue';
    procedure setDoubleValue(aDouble: double); message 'setDoubleValue:';
    procedure takeIntValueFrom(sender: id); message 'takeIntValueFrom:';
    procedure takeFloatValueFrom(sender: id); message 'takeFloatValueFrom:';
    procedure takeDoubleValueFrom(sender: id); message 'takeDoubleValueFrom:';
    procedure takeStringValueFrom(sender: id); message 'takeStringValueFrom:';
    procedure takeObjectValueFrom(sender: id); message 'takeObjectValueFrom:';
    function image: NSImage; message 'image';
    procedure setImage(image_: NSImage); message 'setImage:';
    procedure setControlTint(controlTint_: NSControlTint); message 'setControlTint:';
    function controlTint: NSControlTint; message 'controlTint';
    procedure setControlSize(size: NSControlSize); message 'setControlSize:';
    function controlSize: NSControlSize; message 'controlSize';
    function representedObject: id; message 'representedObject';
    procedure setRepresentedObject(anObject: id); message 'setRepresentedObject:';
    function cellAttribute(aParameter: NSCellAttribute): NSInteger; message 'cellAttribute:';
    procedure setCellAttribute_to(aParameter: NSCellAttribute; value: NSInteger); message 'setCellAttribute:to:';
    function imageRectForBounds(theRect: NSRect): NSRect; message 'imageRectForBounds:';
    function titleRectForBounds(theRect: NSRect): NSRect; message 'titleRectForBounds:';
    function drawingRectForBounds(theRect: NSRect): NSRect; message 'drawingRectForBounds:';
    function cellSize: NSSize; message 'cellSize';
    function cellSizeForBounds(aRect: NSRect): NSSize; message 'cellSizeForBounds:';
    function highlightColorWithFrame_inView(cellFrame: NSRect; controlView_: NSView): NSColor; message 'highlightColorWithFrame:inView:';
    procedure calcDrawInfo(aRect: NSRect); message 'calcDrawInfo:';
    function setUpFieldEditorAttributes(textObj: NSText): NSText; message 'setUpFieldEditorAttributes:';
    procedure drawInteriorWithFrame_inView(cellFrame: NSRect; controlView_: NSView); message 'drawInteriorWithFrame:inView:';
    procedure drawWithFrame_inView(cellFrame: NSRect; controlView_: NSView); message 'drawWithFrame:inView:';
    procedure highlight_withFrame_inView(flag: Boolean; cellFrame: NSRect; controlView_: NSView); message 'highlight:withFrame:inView:';
    function mouseDownFlags: NSInteger; message 'mouseDownFlags';
    procedure getPeriodicDelay_interval(delay: psingle; interval: psingle); message 'getPeriodicDelay:interval:';
    function startTrackingAt_inView(startPoint: NSPoint; controlView_: NSView): Boolean; message 'startTrackingAt:inView:';
    function continueTracking_at_inView(lastPoint: NSPoint; currentPoint: NSPoint; controlView_: NSView): Boolean; message 'continueTracking:at:inView:';
    procedure stopTracking_at_inView_mouseIsUp(lastPoint: NSPoint; stopPoint: NSPoint; controlView_: NSView; flag: Boolean); message 'stopTracking:at:inView:mouseIsUp:';
    function trackMouse_inRect_ofView_untilMouseUp(theEvent: NSEvent; cellFrame: NSRect; controlView_: NSView; flag: Boolean): Boolean; message 'trackMouse:inRect:ofView:untilMouseUp:';
    procedure editWithFrame_inView_editor_delegate_event(aRect: NSRect; controlView_: NSView; textObj: NSText; anObject: id; theEvent: NSEvent); message 'editWithFrame:inView:editor:delegate:event:';
    procedure selectWithFrame_inView_editor_delegate_start_length(aRect: NSRect; controlView_: NSView; textObj: NSText; anObject: id; selStart: NSInteger; selLength: NSInteger); message 'selectWithFrame:inView:editor:delegate:start:length:';
    procedure endEditing(textObj: NSText); message 'endEditing:';
    procedure resetCursorRect_inView(cellFrame: NSRect; controlView_: NSView); message 'resetCursorRect:inView:';
    procedure setMenu(aMenu: NSMenu); message 'setMenu:';
    function menu: NSMenu; message 'menu';
    function menuForEvent_inRect_ofView(event: NSEvent; cellFrame: NSRect; view: NSView): NSMenu; message 'menuForEvent:inRect:ofView:';
    class function defaultMenu: NSMenu; message 'defaultMenu';
    procedure setSendsActionOnEndEditing(flag: Boolean); message 'setSendsActionOnEndEditing:';
    function sendsActionOnEndEditing: Boolean; message 'sendsActionOnEndEditing';
    function baseWritingDirection: NSWritingDirection; message 'baseWritingDirection';
    procedure setBaseWritingDirection(writingDirection: NSWritingDirection); message 'setBaseWritingDirection:';
    procedure setLineBreakMode(mode: NSLineBreakMode); message 'setLineBreakMode:';
    function lineBreakMode: NSLineBreakMode; message 'lineBreakMode';
    procedure setAllowsUndo(allowsUndo_: Boolean); message 'setAllowsUndo:';
    function allowsUndo: Boolean; message 'allowsUndo';
    function integerValue: NSInteger; message 'integerValue';
    procedure setIntegerValue(anInteger: NSInteger); message 'setIntegerValue:';
    procedure takeIntegerValueFrom(sender: id); message 'takeIntegerValueFrom:';
    function truncatesLastVisibleLine: Boolean; message 'truncatesLastVisibleLine';
    procedure setTruncatesLastVisibleLine(flag: Boolean); message 'setTruncatesLastVisibleLine:';
    function userInterfaceLayoutDirection: NSUserInterfaceLayoutDirection; message 'userInterfaceLayoutDirection';
    procedure setUserInterfaceLayoutDirection(layoutDirection: NSUserInterfaceLayoutDirection); message 'setUserInterfaceLayoutDirection:';
    function fieldEditorForView(aControlView: NSView): NSTextView; message 'fieldEditorForView:';
    function usesSingleLineMode: Boolean; message 'usesSingleLineMode';
    procedure setUsesSingleLineMode(flag: Boolean); message 'setUsesSingleLineMode:';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSKeyboardUI_NSCellCategory }
  NSKeyboardUI_NSCellCategory = objccategory external name 'NSKeyboardUI' (NSCell)
    procedure setRefusesFirstResponder(flag: Boolean); message 'setRefusesFirstResponder:';
    function refusesFirstResponder: Boolean; message 'refusesFirstResponder';
    function acceptsFirstResponder: Boolean; message 'acceptsFirstResponder';
    procedure setShowsFirstResponder(showFR: Boolean); message 'setShowsFirstResponder:';
    function showsFirstResponder: Boolean; message 'showsFirstResponder';
    procedure setMnemonicLocation(location: NSUInteger); message 'setMnemonicLocation:';
    function mnemonicLocation: NSUInteger; message 'mnemonicLocation';
    function mnemonic: NSString; message 'mnemonic';
    procedure setTitleWithMnemonic(stringWithAmpersand: NSString); message 'setTitleWithMnemonic:';
    procedure performClick(sender: id); message 'performClick:';
    procedure setFocusRingType(focusRingType_: NSFocusRingType); message 'setFocusRingType:';
    function focusRingType: NSFocusRingType; message 'focusRingType';
    class function defaultFocusRingType: NSFocusRingType; message 'defaultFocusRingType';
    function wantsNotificationForMarkedText: Boolean; message 'wantsNotificationForMarkedText';
  end;

{ NSCellAttributedStringMethodsCategory }
  NSCellAttributedStringMethodsCategory = objccategory external (NSCell)
    function attributedStringValue: NSAttributedString; message 'attributedStringValue';
    procedure setAttributedStringValue(obj: NSAttributedString); message 'setAttributedStringValue:';
    function allowsEditingTextAttributes: Boolean; message 'allowsEditingTextAttributes';
    procedure setAllowsEditingTextAttributes(flag: Boolean); message 'setAllowsEditingTextAttributes:';
    function importsGraphics: Boolean; message 'importsGraphics';
    procedure setImportsGraphics(flag: Boolean); message 'setImportsGraphics:';
  end;

{ NSCellMixedStateCategory }
  NSCellMixedStateCategory = objccategory external (NSCell)
    procedure setAllowsMixedState(flag: Boolean); message 'setAllowsMixedState:';
    function allowsMixedState: Boolean; message 'allowsMixedState';
    function nextState: NSInteger; message 'nextState';
    procedure setNextState; message 'setNextState';
  end;

{ NSCellHitTestCategory }
  NSCellHitTestCategory = objccategory external (NSCell)
    function hitTestForEvent_inRect_ofView(event: NSEvent; cellFrame: NSRect; controlView_: NSView): NSUInteger; message 'hitTestForEvent:inRect:ofView:';
  end;

{ NSCellExpansionCategory }
  NSCellExpansionCategory = objccategory external (NSCell)
    function expansionFrameWithFrame_inView(cellFrame: NSRect; view: NSView): NSRect; message 'expansionFrameWithFrame:inView:';
    procedure drawWithExpansionFrame_inView(cellFrame: NSRect; view: NSView); message 'drawWithExpansionFrame:inView:';
  end;

{ NSCellBackgroundStyleCategory }
  NSCellBackgroundStyleCategory = objccategory external (NSCell)
    function backgroundStyle: NSBackgroundStyle; message 'backgroundStyle';
    procedure setBackgroundStyle(style: NSBackgroundStyle); message 'setBackgroundStyle:';
    function interiorBackgroundStyle: NSBackgroundStyle; message 'interiorBackgroundStyle';
  end;

{ NSDeprecated_NSCellCategory }
  NSDeprecated_NSCellCategory = objccategory external name 'NSDeprecated' (NSCell)
    function entryType: NSInteger; message 'entryType'; deprecated 'in Mac OS X 10 and later';
    procedure setEntryType(aType: NSInteger); message 'setEntryType:'; deprecated 'in Mac OS X 10 and later';
    procedure setFloatingPointFormat_left_right(autoRange: Boolean; leftDigits: NSUInteger; rightDigits: NSUInteger); message 'setFloatingPointFormat:left:right:'; deprecated 'in Mac OS X 10 and later';
  end;

{$endif}
{$endif}
