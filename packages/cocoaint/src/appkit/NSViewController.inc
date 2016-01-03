{ Parsed from Appkit.framework NSViewController.h }

{$ifdef TYPES}
{$ifndef NSVIEWCONTROLLER_PAS_T}
{$define NSVIEWCONTROLLER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSVIEWCONTROLLER_PAS_R}
{$define NSVIEWCONTROLLER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSVIEWCONTROLLER_PAS_F}
{$define NSVIEWCONTROLLER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSVIEWCONTROLLER_PAS_S}
{$define NSVIEWCONTROLLER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSViewController = objcclass;
  NSViewControllerPointer = ^NSViewController;
  NSViewControllerPtr = NSViewControllerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSVIEWCONTROLLER_PAS_C}
{$define NSVIEWCONTROLLER_PAS_C}

{ NSViewController }
  NSViewController = objcclass external (NSResponder)
  private
    _nibName: NSString;
    _nibBundle: NSBundle;
    _representedObject: id;
    _title: NSString;
    _view: NSView;
    _topLevelObjects: NSArray;
    _editors: NSPointerArray;
    _autounbinder: id;
    _designNibBundleIdentifier: NSString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(2)-1] of id;
=======
        _reserved: array[0..1] of id;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..1] of id;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..1] of id;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..1] of id;
>>>>>>> origin/cpstrnew
    
  public
    function initWithNibName_bundle(nibNameOrNil: NSString; nibBundleOrNil: NSBundle): id; message 'initWithNibName:bundle:';
    procedure setRepresentedObject(representedObject_: id); message 'setRepresentedObject:';
    function representedObject: id; message 'representedObject';
    procedure setTitle(title_: NSString); message 'setTitle:';
    function title: NSString; message 'title';
    function view: NSView; message 'view';
    procedure loadView; message 'loadView';
    function nibName: NSString; message 'nibName';
    function nibBundle: NSBundle; message 'nibBundle';
    procedure setView(view_: NSView); message 'setView:';
    procedure commitEditingWithDelegate_didCommitSelector_contextInfo(delegate: id; didCommitSelector: SEL; contextInfo: Pointer); message 'commitEditingWithDelegate:didCommitSelector:contextInfo:';
    function commitEditing: Boolean; message 'commitEditing';
    procedure discardEditing; message 'discardEditing';
  end;

{$endif}
{$endif}
