{ Parsed from Appkit.framework NSPageLayout.h }

{$ifdef TYPES}
{$ifndef NSPAGELAYOUT_PAS_T}
{$define NSPAGELAYOUT_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPAGELAYOUT_PAS_R}
{$define NSPAGELAYOUT_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPAGELAYOUT_PAS_F}
{$define NSPAGELAYOUT_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPAGELAYOUT_PAS_S}
{$define NSPAGELAYOUT_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPageLayout = objcclass;
  NSPageLayoutPointer = ^NSPageLayout;
  NSPageLayoutPtr = NSPageLayoutPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPAGELAYOUT_PAS_C}
{$define NSPAGELAYOUT_PAS_C}

{ NSPageLayout }
  NSPageLayout = objcclass external (NSObject)
  private
    _accessoryControllers: NSMutableArray;
    _originalPrintInfo: NSPrintInfo;
    _delegate: id;
    _didEndSelector: SEL;
    _contextInfo: Pointer;
    _presentedPrintInfo: NSPrintInfo;
    _windowController: NSWindowController;
    {$ifdef cpu64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(4)-1] of id;
    {$else}
    _compatibilityPadding: array[0..(156)-1] of char;
=======
        _reserved: array[0..3] of id;
    {$else}
        _compatibilityPadding: array[0..155] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..3] of id;
    {$else}
        _compatibilityPadding: array[0..155] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..3] of id;
    {$else}
        _compatibilityPadding: array[0..155] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..3] of id;
    {$else}
        _compatibilityPadding: array[0..155] of char;
>>>>>>> origin/cpstrnew
    {$endif}
    
  public
    class function pageLayout: NSPageLayout; message 'pageLayout';
    procedure addAccessoryController(accessoryController: NSViewController); message 'addAccessoryController:';
    procedure removeAccessoryController(accessoryController: NSViewController); message 'removeAccessoryController:';
    function accessoryControllers: NSArray; message 'accessoryControllers';
    procedure beginSheetWithPrintInfo_modalForWindow_delegate_didEndSelector_contextInfo(printInfo_: NSPrintInfo; docWindow: NSWindow; delegate: id; didEndSelector: SEL; contextInfo: Pointer); message 'beginSheetWithPrintInfo:modalForWindow:delegate:didEndSelector:contextInfo:';
    function runModalWithPrintInfo(printInfo_: NSPrintInfo): NSInteger; message 'runModalWithPrintInfo:';
    function runModal: NSInteger; message 'runModal';
    function printInfo: NSPrintInfo; message 'printInfo';
  end;

{ NSDeprecated_NSPageLayoutCategory }
  NSDeprecated_NSPageLayoutCategory = objccategory external name 'NSDeprecated' (NSPageLayout)
    procedure setAccessoryView(accessoryView: NSView); message 'setAccessoryView:'; deprecated 'in Mac OS X 10.5 and later';
    function accessoryView: NSView; message 'accessoryView'; deprecated 'in Mac OS X 10.5 and later';
    procedure readPrintInfo; message 'readPrintInfo'; deprecated 'in Mac OS X 10.5 and later';
    procedure writePrintInfo; message 'writePrintInfo'; deprecated 'in Mac OS X 10.5 and later';
  end;

{ NSPageLayoutPanelCategory }
  NSPageLayoutPanelCategory = objccategory external (NSApplication)
    procedure runPageLayout(sender: id); message 'runPageLayout:';
  end;

{$endif}
{$endif}
