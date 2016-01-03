{ Parsed from Appkit.framework NSWindowController.h }

{$ifdef TYPES}
{$ifndef NSWINDOWCONTROLLER_PAS_T}
{$define NSWINDOWCONTROLLER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSWINDOWCONTROLLER_PAS_R}
{$define NSWINDOWCONTROLLER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSWINDOWCONTROLLER_PAS_F}
{$define NSWINDOWCONTROLLER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSWINDOWCONTROLLER_PAS_S}
{$define NSWINDOWCONTROLLER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSWindowController = objcclass;
  NSWindowControllerPointer = ^NSWindowController;
  NSWindowControllerPtr = NSWindowControllerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSWINDOWCONTROLLER_PAS_C}
{$define NSWINDOWCONTROLLER_PAS_C}

{ NSWindowController }
  NSWindowController = objcclass external (NSResponder, NSCodingProtocol)
  private
    _window: NSWindow;
    _windowNibName: NSString;
    _document: NSDocument;
    _topLevelObjects: NSArray;
    _owner: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __wcFlags: record
      case byte of
       0: (_anonbitfield_NSWindowController0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __wcFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSWindowController0: cuint);
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
        _shouldCloseDocument: 0..1;
        shouldCascade: 0..1;
        nibIsLoaded: 0..1;
        nibNameIsPath: 0..1;
        RESERVED: 0..((1 shl 28)-1);
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
    _frameAutosaveName: NSString;
    _moreVars: id;
    
  public
    function initWithWindow(window_: NSWindow): id; message 'initWithWindow:';
    function initWithWindowNibName(windowNibName_: NSString): id; message 'initWithWindowNibName:';
    function initWithWindowNibName_owner(windowNibName_: NSString; owner_: id): id; message 'initWithWindowNibName:owner:';
    function initWithWindowNibPath_owner(windowNibPath_: NSString; owner_: id): id; message 'initWithWindowNibPath:owner:';
    function windowNibName: NSString; message 'windowNibName';
    function windowNibPath: NSString; message 'windowNibPath';
    function owner: id; message 'owner';
    procedure setWindowFrameAutosaveName(name: NSString); message 'setWindowFrameAutosaveName:';
    function windowFrameAutosaveName: NSString; message 'windowFrameAutosaveName';
    procedure setShouldCascadeWindows(flag: Boolean); message 'setShouldCascadeWindows:';
    function shouldCascadeWindows: Boolean; message 'shouldCascadeWindows';
    function document: id; message 'document';
    procedure setDocument(document_: NSDocument); message 'setDocument:';
    procedure setDocumentEdited(dirtyFlag: Boolean); message 'setDocumentEdited:';
    procedure setShouldCloseDocument(flag: Boolean); message 'setShouldCloseDocument:';
    function shouldCloseDocument: Boolean; message 'shouldCloseDocument';
    procedure setWindow(window_: NSWindow); message 'setWindow:';
    function window: NSWindow; message 'window';
    procedure synchronizeWindowTitleWithDocumentName; message 'synchronizeWindowTitleWithDocumentName';
    function windowTitleForDocumentDisplayName(displayName: NSString): NSString; message 'windowTitleForDocumentDisplayName:';
    procedure close; message 'close';
    procedure showWindow(sender: id); message 'showWindow:';
    function isWindowLoaded: Boolean; message 'isWindowLoaded';
    procedure windowWillLoad; message 'windowWillLoad';
    procedure windowDidLoad; message 'windowDidLoad';
    procedure loadWindow; message 'loadWindow';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
