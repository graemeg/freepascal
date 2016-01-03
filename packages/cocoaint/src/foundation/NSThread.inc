{ Parsed from Foundation.framework NSThread.h }

{$ifdef TYPES}
{$ifndef NSTHREAD_PAS_T}
{$define NSTHREAD_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTHREAD_PAS_R}
{$define NSTHREAD_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTHREAD_PAS_F}
{$define NSTHREAD_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTHREAD_PAS_S}
{$define NSTHREAD_PAS_S}

{ External string constants }
var
  NSWillBecomeMultiThreadedNotification: NSString; cvar; external;
  NSDidBecomeSingleThreadedNotification: NSString; cvar; external;
  NSThreadWillExitNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSThread = objcclass;
  NSThreadPointer = ^NSThread;
  NSThreadPtr = NSThreadPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTHREAD_PAS_C}
{$define NSTHREAD_PAS_C}

{ NSThread }
  NSThread = objcclass external (NSObject)
  private
    _private: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _bytes: array[0..(44)-1] of cuint8;
=======
        _bytes: array[0..43] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _bytes: array[0..43] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _bytes: array[0..43] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _bytes: array[0..43] of cuint8;
>>>>>>> origin/cpstrnew
    
  public
    class function currentThread: NSThread; message 'currentThread';
    class procedure detachNewThreadSelector_toTarget_withObject(selector: SEL; target: id; argument: id); message 'detachNewThreadSelector:toTarget:withObject:';
    class function isMultiThreaded: Boolean; message 'isMultiThreaded';
    function threadDictionary: NSMutableDictionary; message 'threadDictionary';
    class procedure sleepUntilDate(date: NSDate); message 'sleepUntilDate:';
    class procedure sleepForTimeInterval(ti: NSTimeInterval); message 'sleepForTimeInterval:';
    class procedure exit; message 'exit';
    class function threadPriority: double; message 'threadPriority';
    class function setThreadPriority(p: double): Boolean; message 'setThreadPriority:';
    class function callStackReturnAddresses: NSArray; message 'callStackReturnAddresses';
    class function callStackSymbols: NSArray; message 'callStackSymbols';
    procedure setName(n: NSString); message 'setName:';
    function name: NSString; message 'name';
    function stackSize: NSUInteger; message 'stackSize';
    procedure setStackSize(s: NSUInteger); message 'setStackSize:';
    function isMainThread: Boolean; message 'isMainThread';
    class function mainThread: NSThread; message 'mainThread';
    function init: id; message 'init';
    function initWithTarget_selector_object(target: id; selector: SEL; argument: id): id; message 'initWithTarget:selector:object:';
    function isExecuting: Boolean; message 'isExecuting';
    function isFinished: Boolean; message 'isFinished';
    function isCancelled: Boolean; message 'isCancelled';
    procedure cancel; message 'cancel';
    procedure start; message 'start';
    procedure main; message 'main';
  end;

{ NSThreadPerformAdditionsCategory }
  NSThreadPerformAdditionsCategory = objccategory external (NSObject)
    procedure performSelectorOnMainThread_withObject_waitUntilDone_modes(aSelector: SEL; arg: id; wait: Boolean; array_: NSArray); message 'performSelectorOnMainThread:withObject:waitUntilDone:modes:';
    procedure performSelectorOnMainThread_withObject_waitUntilDone(aSelector: SEL; arg: id; wait: Boolean); message 'performSelectorOnMainThread:withObject:waitUntilDone:';
    procedure performSelector_onThread_withObject_waitUntilDone_modes(aSelector: SEL; thr: NSThread; arg: id; wait: Boolean; array_: NSArray); message 'performSelector:onThread:withObject:waitUntilDone:modes:';
    procedure performSelector_onThread_withObject_waitUntilDone(aSelector: SEL; thr: NSThread; arg: id; wait: Boolean); message 'performSelector:onThread:withObject:waitUntilDone:';
    procedure performSelectorInBackground_withObject(aSelector: SEL; arg: id); message 'performSelectorInBackground:withObject:';
  end;

{$endif}
{$endif}
