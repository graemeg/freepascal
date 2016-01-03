{ Parsed from Foundation.framework NSAppleEventManager.h }

{$ifdef TYPES}
{$ifndef NSAPPLEEVENTMANAGER_PAS_T}
{$define NSAPPLEEVENTMANAGER_PAS_T}

{ Types }
type
  NSAppleEventManagerSuspensionID = __NSAppleEventManagerSuspensionPtr;
  NSAppleEventManagerSuspensionIDPtr = ^NSAppleEventManagerSuspensionID;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSAPPLEEVENTMANAGER_PAS_R}
{$define NSAPPLEEVENTMANAGER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSAPPLEEVENTMANAGER_PAS_F}
{$define NSAPPLEEVENTMANAGER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSAPPLEEVENTMANAGER_PAS_S}
{$define NSAPPLEEVENTMANAGER_PAS_S}

{ External symbols }
var
  NSAppleEventTimeOutDefault: double; cvar; external;
  NSAppleEventTimeOutNone: double; cvar; external;

{ External string constants }
var
  NSAppleEventManagerWillProcessFirstEventNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSAppleEventManager = objcclass;
  NSAppleEventManagerPointer = ^NSAppleEventManager;
  NSAppleEventManagerPtr = NSAppleEventManagerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSAPPLEEVENTMANAGER_PAS_C}
{$define NSAPPLEEVENTMANAGER_PAS_C}

{ NSAppleEventManager }
  NSAppleEventManager = objcclass external (NSObject)
  private
    _isPreparedForDispatch: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _padding: array[0..(3)-1] of char;
=======
        _padding: array[0..2] of char;
>>>>>>> graemeg/cpstrnew
=======
        _padding: array[0..2] of char;
>>>>>>> graemeg/cpstrnew
=======
        _padding: array[0..2] of char;
>>>>>>> graemeg/cpstrnew
=======
        _padding: array[0..2] of char;
>>>>>>> origin/cpstrnew
    
  public
    class function sharedAppleEventManager: NSAppleEventManager; message 'sharedAppleEventManager';
    procedure setEventHandler_andSelector_forEventClass_andEventID(handler: id; handleEventSelector: SEL; eventClass: AEEventClass; eventID: AEEventID); message 'setEventHandler:andSelector:forEventClass:andEventID:';
    procedure removeEventHandlerForEventClass_andEventID(eventClass: AEEventClass; eventID: AEEventID); message 'removeEventHandlerForEventClass:andEventID:';
    function dispatchRawAppleEvent_withRawReply_handlerRefCon(theAppleEvent: AppleEventPtr; theReply: AppleEventPtr; handlerRefCon: SRefCon): OSErr; message 'dispatchRawAppleEvent:withRawReply:handlerRefCon:';
    function currentAppleEvent: NSAppleEventDescriptor; message 'currentAppleEvent';
    function currentReplyAppleEvent: NSAppleEventDescriptor; message 'currentReplyAppleEvent';
    function suspendCurrentAppleEvent: NSAppleEventManagerSuspensionID; message 'suspendCurrentAppleEvent';
    function appleEventForSuspensionID(suspensionID: NSAppleEventManagerSuspensionID): NSAppleEventDescriptor; message 'appleEventForSuspensionID:';
    function replyAppleEventForSuspensionID(suspensionID: NSAppleEventManagerSuspensionID): NSAppleEventDescriptor; message 'replyAppleEventForSuspensionID:';
    procedure setCurrentAppleEventAndReplyEventWithSuspensionID(suspensionID: NSAppleEventManagerSuspensionID); message 'setCurrentAppleEventAndReplyEventWithSuspensionID:';
    procedure resumeWithSuspensionID(suspensionID: NSAppleEventManagerSuspensionID); message 'resumeWithSuspensionID:';
  end;

{$endif}
{$endif}
