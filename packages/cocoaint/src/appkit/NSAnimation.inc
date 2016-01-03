{ Parsed from Appkit.framework NSAnimation.h }

{$ifdef TYPES}
{$ifndef NSANIMATION_PAS_T}
{$define NSANIMATION_PAS_T}

{ Constants }

const
  NSAnimationEaseInOut = 0;
  NSAnimationEaseIn = 1;
  NSAnimationEaseOut = 2;
  NSAnimationLinear = 3;

const
  NSAnimationBlocking = 0;
  NSAnimationNonblocking = 1;
  NSAnimationNonblockingThreaded = 2;

{ Types }
type
  NSAnimationCurve = NSUInteger;
  NSAnimationCurvePtr = ^NSAnimationCurve;
  NSAnimationBlockingMode = NSUInteger;
  NSAnimationBlockingModePtr = ^NSAnimationBlockingMode;
  NSAnimationProgress = single;
  NSAnimationProgressPtr = ^NSAnimationProgress;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSANIMATION_PAS_R}
{$define NSANIMATION_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSANIMATION_PAS_F}
{$define NSANIMATION_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSANIMATION_PAS_S}
{$define NSANIMATION_PAS_S}

{ External string constants }
var
  NSAnimationProgressMarkNotification: NSString; cvar; external;
  NSAnimationProgressMark: NSString; cvar; external;
  NSViewAnimationTargetKey: NSString; cvar; external;
  NSViewAnimationStartFrameKey: NSString; cvar; external;
  NSViewAnimationEndFrameKey: NSString; cvar; external;
  NSViewAnimationEffectKey: NSString; cvar; external;
  NSViewAnimationFadeInEffect: NSString; cvar; external;
  NSViewAnimationFadeOutEffect: NSString; cvar; external;
  NSAnimationTriggerOrderIn: NSString; cvar; external;
  NSAnimationTriggerOrderOut: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSAnimationDelegateProtocol = objcprotocol;
  NSAnimatablePropertyContainerProtocol = objcprotocol;
  NSAnimation = objcclass;
  NSAnimationPointer = ^NSAnimation;
  NSAnimationPtr = NSAnimationPointer;
  NSViewAnimation = objcclass;
  NSViewAnimationPointer = ^NSViewAnimation;
  NSViewAnimationPtr = NSViewAnimationPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSANIMATION_PAS_C}
{$define NSANIMATION_PAS_C}

{ NSAnimation }
  NSAnimation = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol)
  private
    _duration: NSTimeInterval;
    _currentProgress: NSAnimationProgress;
    _framesPerSecond: single;
    _delegate: id;
    _timer: NSTimer;
    _startTime: NSTimeInterval;
    _progressMarks: NSMutableArray;
    _startAnimation: NSAnimation;
    _stopAnimation: NSAnimation;
    _nextProgressMark: cint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __aFlags: record
      case byte of
       0: (_anonbitfield_NSAnimation0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __aFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSAnimation0: cuint);
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
        delegateAnimationShouldStart: 0..1;
        delegateAnimationDidStop: 0..1;
        delegateAnimationDidEnd: 0..1;
        delegateAnimationValueForProgress: 0..1;
        delegateAnimationDidReachProgressMark: 0..1;
        animating: 0..1;
        blocking: 0..1;
        reserved: 0..((1 shl 25)-1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       end;
      );
      end;
    __aSettings: record
      case byte of
       0: (_anonbitfield_NSAnimation1: cuint);
       1: (data: bitpacked record
        _animationCurve: 0..((1 shl 8)-1);
        _animationBlockingMode: 0..((1 shl 2)-1);
        reserved: 0..((1 shl 22)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      );
      end;
    __aSettings: bitpacked record
      case byte of
       0: (_anonbitfield_NSAnimation1: cuint);
       1: (
        _animationCurve: 0..((1 shl 8)-1);
        _animationBlockingMode: 0..((1 shl 2)-1);
        reserved: 0..((1 shl 22)-1);
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
      );
      end;
    _scheduledRunLoop: NSRunLoop;
    _reserved2: NSInteger;
    _reserved3: NSInteger;
    _reserved4: NSInteger;
    
  public
    function initWithDuration_animationCurve(duration_: NSTimeInterval; animationCurve_: NSAnimationCurve): id; message 'initWithDuration:animationCurve:';
    procedure startAnimation; message 'startAnimation';
    procedure stopAnimation; message 'stopAnimation';
    function isAnimating: Boolean; message 'isAnimating';
    function currentProgress: NSAnimationProgress; message 'currentProgress';
    procedure setCurrentProgress(progress: NSAnimationProgress); message 'setCurrentProgress:';
    procedure setDuration(duration_: NSTimeInterval); message 'setDuration:';
    function duration: NSTimeInterval; message 'duration';
    function animationBlockingMode: NSAnimationBlockingMode; message 'animationBlockingMode';
    procedure setAnimationBlockingMode(animationBlockingMode_: NSAnimationBlockingMode); message 'setAnimationBlockingMode:';
    procedure setFrameRate(framesPerSecond: single); message 'setFrameRate:';
    function frameRate: single; message 'frameRate';
    procedure setAnimationCurve(curve: NSAnimationCurve); message 'setAnimationCurve:';
    function animationCurve: NSAnimationCurve; message 'animationCurve';
    function currentValue: single; message 'currentValue';
    procedure setDelegate(delegate_: NSAnimationDelegateProtocol); message 'setDelegate:';
    function delegate: NSAnimationDelegateProtocol; message 'delegate';
    function progressMarks: NSArray; message 'progressMarks';
    procedure setProgressMarks(progressMarks_: NSArray); message 'setProgressMarks:';
    procedure addProgressMark(progressMark: NSAnimationProgress); message 'addProgressMark:';
    procedure removeProgressMark(progressMark: NSAnimationProgress); message 'removeProgressMark:';
    procedure startWhenAnimation_reachesProgress(animation: NSAnimation; startProgress: NSAnimationProgress); message 'startWhenAnimation:reachesProgress:';
    procedure stopWhenAnimation_reachesProgress(animation: NSAnimation; stopProgress: NSAnimationProgress); message 'stopWhenAnimation:reachesProgress:';
    procedure clearStartAnimation; message 'clearStartAnimation';
    procedure clearStopAnimation; message 'clearStopAnimation';
    function runLoopModesForAnimating: NSArray; message 'runLoopModesForAnimating';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSViewAnimation }
  NSViewAnimation = objcclass external (NSAnimation)
  private
    _viewAnimations: NSArray;
    _viewAnimationInfo: CFMutableDictionaryRef;
    _windowAnimationInfo: CFMutableDictionaryRef;
    _reserved4a: NSUInteger;
    _reserved4b: NSUInteger;
    _reserved4c: NSUInteger;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __vaFlags: record
      case byte of
       0: (_anonbitfield_NSAnimation2: cuint);
       1: (data: bitpacked record
        reserved: 0..((1 shl 32)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __vaFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSAnimation2: cuint);
       1: (
        reserved: 0..((1 shl 32)-1);
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
      );
      end;
    _reserved5: NSUInteger;
    _reserved6: NSUInteger;
    _reserved7: NSUInteger;
    _reserved8: NSUInteger;
    
  public
    function initWithViewAnimations(viewAnimations_: NSArray): id; message 'initWithViewAnimations:';
    function viewAnimations: NSArray; message 'viewAnimations';
    procedure setViewAnimations(viewAnimations_: NSArray); message 'setViewAnimations:';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSANIMATION_PAS_P}
{$define NSANIMATION_PAS_P}
  
{ NSAnimationDelegate Protocol }
  NSAnimationDelegateProtocol = objcprotocol external name 'NSAnimationDelegate'
  optional
    function animationShouldStart(animation: NSAnimation): Boolean; message 'animationShouldStart:';
    procedure animationDidStop(animation: NSAnimation); message 'animationDidStop:';
    procedure animationDidEnd(animation: NSAnimation); message 'animationDidEnd:';
    function animation_valueForProgress(animation: NSAnimation; progress: NSAnimationProgress): single; message 'animation:valueForProgress:';
    procedure animation_didReachProgressMark(animation: NSAnimation; progress: NSAnimationProgress); message 'animation:didReachProgressMark:';
  end;
  
{ NSAnimatablePropertyContainer Protocol }
  NSAnimatablePropertyContainerProtocol = objcprotocol external name 'NSAnimatablePropertyContainer'
  optional
    function animator: id; message 'animator';
    function animations: NSDictionary; message 'animations';
    procedure setAnimations(dict: NSDictionary); message 'setAnimations:';
    function animationForKey(key: NSString): id; message 'animationForKey:';
  end;
{$endif}
{$endif}
