{ Parsed from Appkit.framework NSCursor.h }

{$ifdef TYPES}
{$ifndef NSCURSOR_PAS_T}
{$define NSCURSOR_PAS_T}

{ Defines }
const
  NSAppKitVersionNumberWithCursorSizeSupport = 682.0;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSCURSOR_PAS_R}
{$define NSCURSOR_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSCURSOR_PAS_F}
{$define NSCURSOR_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSCURSOR_PAS_S}
{$define NSCURSOR_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSCursor = objcclass;
  NSCursorPointer = ^NSCursor;
  NSCursorPtr = NSCursorPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSCURSOR_PAS_C}
{$define NSCURSOR_PAS_C}

{ NSCursor }
  NSCursor = objcclass external (NSObject, NSCodingProtocol)
  private
    _hotSpot: NSPoint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSCursor1: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __flags: bitpacked record
      case byte of
       0: (_anonbitfield_NSCursor1: cuint);
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
        onMouseExited: 0..1;
        onMouseEntered: 0..1;
        cursorType: 0..((1 shl 8)-1);
        _anonbitfield_NSCursor0: 0..((1 shl 22)-1);
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
    _image: id;
    
  public
    class function currentCursor: NSCursor; message 'currentCursor';
    class function currentSystemCursor: NSCursor; message 'currentSystemCursor';
    class function arrowCursor: NSCursor; message 'arrowCursor';
    class function IBeamCursor: NSCursor; message 'IBeamCursor';
    class function pointingHandCursor: NSCursor; message 'pointingHandCursor';
    class function closedHandCursor: NSCursor; message 'closedHandCursor';
    class function openHandCursor: NSCursor; message 'openHandCursor';
    class function resizeLeftCursor: NSCursor; message 'resizeLeftCursor';
    class function resizeRightCursor: NSCursor; message 'resizeRightCursor';
    class function resizeLeftRightCursor: NSCursor; message 'resizeLeftRightCursor';
    class function resizeUpCursor: NSCursor; message 'resizeUpCursor';
    class function resizeDownCursor: NSCursor; message 'resizeDownCursor';
    class function resizeUpDownCursor: NSCursor; message 'resizeUpDownCursor';
    class function crosshairCursor: NSCursor; message 'crosshairCursor';
    class function disappearingItemCursor: NSCursor; message 'disappearingItemCursor';
    class function operationNotAllowedCursor: NSCursor; message 'operationNotAllowedCursor';
    class function dragLinkCursor: NSCursor; message 'dragLinkCursor';
    class function dragCopyCursor: NSCursor; message 'dragCopyCursor';
    class function contextualMenuCursor: NSCursor; message 'contextualMenuCursor';
    function initWithImage_hotSpot(newImage: NSImage; aPoint: NSPoint): id; message 'initWithImage:hotSpot:';
    function initWithImage_foregroundColorHint_backgroundColorHint_hotSpot(newImage: NSImage; fg: NSColor; bg: NSColor; hotSpot_: NSPoint): id; message 'initWithImage:foregroundColorHint:backgroundColorHint:hotSpot:';
    class procedure hide; message 'hide';
    class procedure unhide; message 'unhide';
    class procedure setHiddenUntilMouseMoves(flag: Boolean); message 'setHiddenUntilMouseMoves:';
    class procedure pop; message 'pop';
    function image: NSImage; message 'image';
    function hotSpot: NSPoint; message 'hotSpot';
    procedure push; message 'push';
    procedure set_; message 'set';
    procedure setOnMouseExited(flag: Boolean); message 'setOnMouseExited:';
    procedure setOnMouseEntered(flag: Boolean); message 'setOnMouseEntered:';
    function isSetOnMouseExited: Boolean; message 'isSetOnMouseExited';
    function isSetOnMouseEntered: Boolean; message 'isSetOnMouseEntered';
    procedure mouseEntered(theEvent: NSEvent); message 'mouseEntered:';
    procedure mouseExited(theEvent: NSEvent); message 'mouseExited:';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
