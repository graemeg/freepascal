{ Parsed from Appkit.framework NSMovieView.h }

{$ifdef TYPES}
{$ifndef NSMOVIEVIEW_PAS_T}
{$define NSMOVIEVIEW_PAS_T}

{ Constants }

const
  NSQTMovieNormalPlayback = 0;
  NSQTMovieLoopingPlayback = 1;
  NSQTMovieLoopingBackAndForthPlayback = 2;

{ Types }
type
  NSQTMovieLoopMode = NSUInteger;
  NSQTMovieLoopModePtr = ^NSQTMovieLoopMode;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMOVIEVIEW_PAS_R}
{$define NSMOVIEVIEW_PAS_R}

{ Records }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  __MVFlags = record
    case byte of
    0: (_anonbitfield_NSMovieView0: cuint);
    1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  __MVFlags = bitpacked record
    case byte of
    0: (_anonbitfield_NSMovieView0: cuint);
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
          editable: 0..1;
          loopMode: 0..((1 shl 3)-1);
          playsEveryFrame: 0..1;
          playsSelectionOnly: 0..1;
          controllerVisible: 0..1;
          reserved: 0..((1 shl 25)-1);
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
_MVFlags = __MVFlags;
__MVFlagsPtr = ^__MVFlags;


{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMOVIEVIEW_PAS_F}
{$define NSMOVIEVIEW_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMOVIEVIEW_PAS_S}
{$define NSMOVIEVIEW_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSMovieView = objcclass;
  NSMovieViewPointer = ^NSMovieView;
  NSMovieViewPtr = NSMovieViewPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMOVIEVIEW_PAS_C}
{$define NSMOVIEVIEW_PAS_C}

{ NSMovieView }
  NSMovieView = objcclass external (NSView, NSUserInterfaceValidationsProtocol)
  private
    _fMovie: NSMovie;
    _fRate: single;
    _fVolume: single;
    _fFlags: _MVFlags;
    _fAuxData: Pointer;
    _fReserved1: culong;
    _fReserved2: culong;
    _fReserved3: culong;
    
  public
    procedure setMovie(movie_: NSMovie); message 'setMovie:'; deprecated 'in Mac OS X 10.5 and later';
    function movie: NSMovie; message 'movie'; deprecated 'in Mac OS X 10.5 and later';
    function movieController: Pointer; message 'movieController'; deprecated 'in Mac OS X 10.5 and later';
    function movieRect: NSRect; message 'movieRect'; deprecated 'in Mac OS X 10.5 and later';
    procedure start(sender: id); message 'start:'; deprecated 'in Mac OS X 10.5 and later';
    procedure stop(sender: id); message 'stop:'; deprecated 'in Mac OS X 10.5 and later';
    function isPlaying: Boolean; message 'isPlaying'; deprecated 'in Mac OS X 10.5 and later';
    procedure gotoPosterFrame(sender: id); message 'gotoPosterFrame:'; deprecated 'in Mac OS X 10.5 and later';
    procedure gotoBeginning(sender: id); message 'gotoBeginning:'; deprecated 'in Mac OS X 10.5 and later';
    procedure gotoEnd(sender: id); message 'gotoEnd:'; deprecated 'in Mac OS X 10.5 and later';
    procedure stepForward(sender: id); message 'stepForward:'; deprecated 'in Mac OS X 10.5 and later';
    procedure stepBack(sender: id); message 'stepBack:'; deprecated 'in Mac OS X 10.5 and later';
    procedure setRate(rate_: single); message 'setRate:'; deprecated 'in Mac OS X 10.5 and later';
    function rate: single; message 'rate'; deprecated 'in Mac OS X 10.5 and later';
    procedure setVolume(volume_: single); message 'setVolume:'; deprecated 'in Mac OS X 10.5 and later';
    function volume: single; message 'volume'; deprecated 'in Mac OS X 10.5 and later';
    procedure setMuted(mute: Boolean); message 'setMuted:'; deprecated 'in Mac OS X 10.5 and later';
    function isMuted: Boolean; message 'isMuted'; deprecated 'in Mac OS X 10.5 and later';
    procedure setLoopMode(mode: NSQTMovieLoopMode); message 'setLoopMode:'; deprecated 'in Mac OS X 10.5 and later';
    function loopMode: NSQTMovieLoopMode; message 'loopMode'; deprecated 'in Mac OS X 10.5 and later';
    procedure setPlaysSelectionOnly(flag: Boolean); message 'setPlaysSelectionOnly:'; deprecated 'in Mac OS X 10.5 and later';
    function playsSelectionOnly: Boolean; message 'playsSelectionOnly'; deprecated 'in Mac OS X 10.5 and later';
    procedure setPlaysEveryFrame(flag: Boolean); message 'setPlaysEveryFrame:'; deprecated 'in Mac OS X 10.5 and later';
    function playsEveryFrame: Boolean; message 'playsEveryFrame'; deprecated 'in Mac OS X 10.5 and later';
    procedure showController_adjustingSize(show: Boolean; adjustSize: Boolean); message 'showController:adjustingSize:'; deprecated 'in Mac OS X 10.5 and later';
    function isControllerVisible: Boolean; message 'isControllerVisible'; deprecated 'in Mac OS X 10.5 and later';
    procedure resizeWithMagnification(magnification: CGFloat); message 'resizeWithMagnification:'; deprecated 'in Mac OS X 10.5 and later';
    function sizeForMagnification(magnification: CGFloat): NSSize; message 'sizeForMagnification:'; deprecated 'in Mac OS X 10.5 and later';
    procedure setEditable(editable: Boolean); message 'setEditable:'; deprecated 'in Mac OS X 10.5 and later';
    function isEditable: Boolean; message 'isEditable'; deprecated 'in Mac OS X 10.5 and later';
    procedure cut(sender: id); message 'cut:'; deprecated 'in Mac OS X 10.5 and later';
    procedure copy_(sender: id); message 'copy:'; deprecated 'in Mac OS X 10.5 and later';
    procedure paste(sender: id); message 'paste:'; deprecated 'in Mac OS X 10.5 and later';
    procedure delete(sender: id); message 'delete:'; deprecated 'in Mac OS X 10.5 and later';
    procedure selectAll(sender: id); message 'selectAll:'; deprecated 'in Mac OS X 10.5 and later';
    procedure clear(sender: id); message 'clear:'; deprecated 'in Mac OS X 10.5 and later';

    { Adopted Protocols }
    function validateUserInterfaceItem(anItem: NSValidatedUserInterfaceItemProtocol): Boolean;
  end;

{$endif}
{$endif}
