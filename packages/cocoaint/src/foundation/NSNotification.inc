{ Parsed from Foundation.framework NSNotification.h }

{$ifdef TYPES}
{$ifndef NSNOTIFICATION_PAS_T}
{$define NSNOTIFICATION_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSNOTIFICATION_PAS_R}
{$define NSNOTIFICATION_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSNOTIFICATION_PAS_F}
{$define NSNOTIFICATION_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSNOTIFICATION_PAS_S}
{$define NSNOTIFICATION_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSNotification = objcclass;
  NSNotificationPointer = ^NSNotification;
  NSNotificationPtr = NSNotificationPointer;
  NSNotificationCenter = objcclass;
  NSNotificationCenterPointer = ^NSNotificationCenter;
  NSNotificationCenterPtr = NSNotificationCenterPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSNOTIFICATION_PAS_C}
{$define NSNOTIFICATION_PAS_C}

{ NSNotification }
  NSNotification = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol)
    
  public
    function name: NSString; message 'name';
    function object_: id; message 'object';
    function userInfo: NSDictionary; message 'userInfo';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSNotificationCenter }
  NSNotificationCenter = objcclass external (NSObject)
  private
    _impl: Pointer; {garbage collector: __strong }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _callback_block: array[0..(4)-1] of Pointer;
    _pad: array[0..(8)-1] of Pointer;
=======
        _callback_block: array[0..3] of Pointer;
        _pad: array[0..7] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _callback_block: array[0..3] of Pointer;
        _pad: array[0..7] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _callback_block: array[0..3] of Pointer;
        _pad: array[0..7] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _callback_block: array[0..3] of Pointer;
        _pad: array[0..7] of Pointer;
>>>>>>> origin/cpstrnew
    
  public
    class function defaultCenter: id; message 'defaultCenter';
    procedure addObserver_selector_name_object(observer: id; aSelector: SEL; aName: NSString; anObject: id); message 'addObserver:selector:name:object:';
    procedure postNotification(notification: NSNotification); message 'postNotification:';
    procedure postNotificationName_object(aName: NSString; anObject: id); message 'postNotificationName:object:';
    procedure postNotificationName_object_userInfo(aName: NSString; anObject: id; aUserInfo: NSDictionary); message 'postNotificationName:object:userInfo:';
    procedure removeObserver(observer: id); message 'removeObserver:';
    procedure removeObserver_name_object(observer: id; aName: NSString; anObject: id); message 'removeObserver:name:object:';
  end;

{ NSNotificationCreationCategory }
  NSNotificationCreationCategory = objccategory external (NSNotification)
    class function notificationWithName_object(aName: NSString; anObject: id): id; message 'notificationWithName:object:';
    class function notificationWithName_object_userInfo(aName: NSString; anObject: id; aUserInfo: NSDictionary): id; message 'notificationWithName:object:userInfo:';
  end;

{$endif}
{$endif}
