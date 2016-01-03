{ Parsed from Appkit.framework NSPrintPanel.h }

{$ifdef TYPES}
{$ifndef NSPRINTPANEL_PAS_T}
{$define NSPRINTPANEL_PAS_T}

{ Constants }

const
  NSPrintPanelShowsCopies = 1 shl 0;
  NSPrintPanelShowsPageRange = 1 shl 1;
  NSPrintPanelShowsPaperSize = 1 shl 2;
  NSPrintPanelShowsOrientation = 1 shl 3;
  NSPrintPanelShowsScaling = 1 shl 4;
  NSPrintPanelShowsPrintSelection = 1 shl 5;
  NSPrintPanelShowsPageSetupAccessory = 1 shl 8;
  NSPrintPanelShowsPreview = 1 shl 17;

{ Types }
type
  NSPrintPanelOptions = NSInteger;
  NSPrintPanelOptionsPtr = ^NSPrintPanelOptions;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPRINTPANEL_PAS_R}
{$define NSPRINTPANEL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPRINTPANEL_PAS_F}
{$define NSPRINTPANEL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPRINTPANEL_PAS_S}
{$define NSPRINTPANEL_PAS_S}

{ External string constants }
var
  NSPrintPhotoJobStyleHint: NSString; cvar; external;
  NSPrintAllPresetsJobStyleHint: NSString; cvar; external;
  NSPrintNoPresetsJobStyleHint: NSString; cvar; external;
  NSPrintPanelAccessorySummaryItemNameKey: NSString; cvar; external;
  NSPrintPanelAccessorySummaryItemDescriptionKey: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPrintPanelAccessorizingProtocol = objcprotocol;
  NSPrintPanel = objcclass;
  NSPrintPanelPointer = ^NSPrintPanel;
  NSPrintPanelPtr = NSPrintPanelPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPRINTPANEL_PAS_C}
{$define NSPRINTPANEL_PAS_C}

{ NSPrintPanel }
  NSPrintPanel = objcclass external (NSObject)
  private
    _accessoryControllers: NSMutableArray;
    _previewController: id;
    _thumbnailView: NSView;
    _options: NSInteger;
    _defaultButtonTitle: NSString;
    _helpAnchor: NSString;
    _jobStyleHint: NSString;
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
    _reserved: array[0..(2)-1] of id;
    {$else}
    _compatibilityPadding: array[0..(192)-1] of char;
=======
        _reserved: array[0..1] of id;
    {$else}
        _compatibilityPadding: array[0..191] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..1] of id;
    {$else}
        _compatibilityPadding: array[0..191] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..1] of id;
    {$else}
        _compatibilityPadding: array[0..191] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..1] of id;
    {$else}
        _compatibilityPadding: array[0..191] of char;
>>>>>>> origin/cpstrnew
    {$endif}
    
  public
    class function printPanel: NSPrintPanel; message 'printPanel';
    procedure addAccessoryController(accessoryController: NSViewController); message 'addAccessoryController:';
    procedure removeAccessoryController(accessoryController: NSViewController); message 'removeAccessoryController:';
    function accessoryControllers: NSArray; message 'accessoryControllers';
    procedure setOptions(options_: NSPrintPanelOptions); message 'setOptions:';
    function options: NSPrintPanelOptions; message 'options';
    procedure setDefaultButtonTitle(defaultButtonTitle_: NSString); message 'setDefaultButtonTitle:';
    function defaultButtonTitle: NSString; message 'defaultButtonTitle';
    procedure setHelpAnchor(helpAnchor_: NSString); message 'setHelpAnchor:';
    function helpAnchor: NSString; message 'helpAnchor';
    procedure setJobStyleHint(hint: NSString); message 'setJobStyleHint:';
    function jobStyleHint: NSString; message 'jobStyleHint';
    procedure beginSheetWithPrintInfo_modalForWindow_delegate_didEndSelector_contextInfo(printInfo_: NSPrintInfo; docWindow: NSWindow; delegate: id; didEndSelector: SEL; contextInfo: Pointer); message 'beginSheetWithPrintInfo:modalForWindow:delegate:didEndSelector:contextInfo:';
    function runModalWithPrintInfo(printInfo_: NSPrintInfo): NSInteger; message 'runModalWithPrintInfo:';
    function runModal: NSInteger; message 'runModal';
    function printInfo: NSPrintInfo; message 'printInfo';
  end;

{ NSDeprecated_NSPrintPanelCategory }
  NSDeprecated_NSPrintPanelCategory = objccategory external name 'NSDeprecated' (NSPrintPanel)
    procedure setAccessoryView(accessoryView_: NSView); message 'setAccessoryView:'; deprecated 'in Mac OS X 10.5 and later';
    function accessoryView: NSView; message 'accessoryView'; deprecated 'in Mac OS X 10.5 and later';
    procedure updateFromPrintInfo; message 'updateFromPrintInfo'; deprecated 'in Mac OS X 10.5 and later';
    procedure finalWritePrintInfo; message 'finalWritePrintInfo'; deprecated 'in Mac OS X 10.5 and later';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSPRINTPANEL_PAS_P}
{$define NSPRINTPANEL_PAS_P}
  
{ NSPrintPanelAccessorizing Protocol }
  NSPrintPanelAccessorizingProtocol = objcprotocol external name 'NSPrintPanelAccessorizing'
    function localizedSummaryItems: NSArray; message 'localizedSummaryItems';
  optional
    function keyPathsForValuesAffectingPreview: NSSet; message 'keyPathsForValuesAffectingPreview';
  end;
{$endif}
{$endif}
