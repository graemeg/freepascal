{ Parsed from Foundation.framework NSUndoManager.h }

{$ifdef TYPES}
{$ifndef NSUNDOMANAGER_PAS_T}
{$define NSUNDOMANAGER_PAS_T}

{ Constants }

const
  NSUndoCloseGroupingRunLoopOrdering = 350000;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSUNDOMANAGER_PAS_R}
{$define NSUNDOMANAGER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSUNDOMANAGER_PAS_F}
{$define NSUNDOMANAGER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSUNDOMANAGER_PAS_S}
{$define NSUNDOMANAGER_PAS_S}

{ External string constants }
var
  NSUndoManagerCheckpointNotification: NSString; cvar; external;
  NSUndoManagerWillUndoChangeNotification: NSString; cvar; external;
  NSUndoManagerWillRedoChangeNotification: NSString; cvar; external;
  NSUndoManagerDidUndoChangeNotification: NSString; cvar; external;
  NSUndoManagerDidRedoChangeNotification: NSString; cvar; external;
  NSUndoManagerDidOpenUndoGroupNotification: NSString; cvar; external;
  NSUndoManagerWillCloseUndoGroupNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSUndoManager = objcclass;
  NSUndoManagerPointer = ^NSUndoManager;
  NSUndoManagerPtr = NSUndoManagerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSUNDOMANAGER_PAS_C}
{$define NSUNDOMANAGER_PAS_C}

{ NSUndoManager }
  NSUndoManager = objcclass external (NSObject)
  private
    _undoStack: id;
    _redoStack: id;
    _runLoopModes: NSArray;
    _disabled: NSInteger;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSUndoManager0: cuint);
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
       0: (_anonbitfield_NSUndoManager0: cuint);
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
        undoing: 0..1;
        redoing: 0..1;
        registeredForCallback: 0..1;
        postingCheckpointNotification: 0..1;
        _groupsByEvent: 0..1;
        reserved: 0..((1 shl 27)-1);
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
    _proxy: id;
    _NSUndoManagerReserved1: Pointer;
    _NSUndoManagerReserved2: Pointer;
    
  public
    procedure beginUndoGrouping; message 'beginUndoGrouping';
    procedure endUndoGrouping; message 'endUndoGrouping';
    function groupingLevel: NSInteger; message 'groupingLevel';
    procedure disableUndoRegistration; message 'disableUndoRegistration';
    procedure enableUndoRegistration; message 'enableUndoRegistration';
    function isUndoRegistrationEnabled: Boolean; message 'isUndoRegistrationEnabled';
    function groupsByEvent: Boolean; message 'groupsByEvent';
    procedure setGroupsByEvent(groupsByEvent_: Boolean); message 'setGroupsByEvent:';
    procedure setLevelsOfUndo(levels: NSUInteger); message 'setLevelsOfUndo:';
    function levelsOfUndo: NSUInteger; message 'levelsOfUndo';
    procedure setRunLoopModes(runLoopModes_: NSArray); message 'setRunLoopModes:';
    function runLoopModes: NSArray; message 'runLoopModes';
    procedure undo; message 'undo';
    procedure redo; message 'redo';
    procedure undoNestedGroup; message 'undoNestedGroup';
    function canUndo: Boolean; message 'canUndo';
    function canRedo: Boolean; message 'canRedo';
    function isUndoing: Boolean; message 'isUndoing';
    function isRedoing: Boolean; message 'isRedoing';
    procedure removeAllActions; message 'removeAllActions';
    procedure removeAllActionsWithTarget(target: id); message 'removeAllActionsWithTarget:';
    procedure registerUndoWithTarget_selector_object(target: id; selector: SEL; anObject: id); message 'registerUndoWithTarget:selector:object:';
    function prepareWithInvocationTarget(target: id): id; message 'prepareWithInvocationTarget:';
    function undoActionName: NSString; message 'undoActionName';
    function redoActionName: NSString; message 'redoActionName';
    procedure setActionName(actionName: NSString); message 'setActionName:';
    function undoMenuItemTitle: NSString; message 'undoMenuItemTitle';
    function redoMenuItemTitle: NSString; message 'redoMenuItemTitle';
    function undoMenuTitleForUndoActionName(actionName: NSString): NSString; message 'undoMenuTitleForUndoActionName:';
    function redoMenuTitleForUndoActionName(actionName: NSString): NSString; message 'redoMenuTitleForUndoActionName:';
  end;

{$endif}
{$endif}
