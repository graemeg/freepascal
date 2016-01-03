{ Parsed from Appkit.framework NSToolbar.h }

{$ifdef TYPES}
{$ifndef NSTOOLBAR_PAS_T}
{$define NSTOOLBAR_PAS_T}

{ Constants }

const
  NSToolbarDisplayModeDefault = 0;
  NSToolbarDisplayModeIconAndLabel = 1;
  NSToolbarDisplayModeIconOnly = 2;
  NSToolbarDisplayModeLabelOnly = 3;

const
  NSToolbarSizeModeDefault = 0;
  NSToolbarSizeModeRegular = 1;
  NSToolbarSizeModeSmall = 2;

{ Types }
type
  NSToolbarDisplayMode = NSUInteger;
  NSToolbarDisplayModePtr = ^NSToolbarDisplayMode;
  NSToolbarSizeMode = NSUInteger;
  NSToolbarSizeModePtr = ^NSToolbarSizeMode;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTOOLBAR_PAS_R}
{$define NSTOOLBAR_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTOOLBAR_PAS_F}
{$define NSTOOLBAR_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTOOLBAR_PAS_S}
{$define NSTOOLBAR_PAS_S}

{ External string constants }
var
  NSToolbarWillAddItemNotification: NSString; cvar; external;
  NSToolbarDidRemoveItemNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSToolbarDelegateProtocol = objcprotocol;
  NSToolbar = objcclass;
  NSToolbarPointer = ^NSToolbar;
  NSToolbarPtr = NSToolbarPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTOOLBAR_PAS_C}
{$define NSTOOLBAR_PAS_C}

{ NSToolbar }
  NSToolbar = objcclass external (NSObject)
  private
    _toolbarIdentifier: NSString;
    _currentItems: NSMutableArray;
    _currentItemIdentifiers: NSMutableArray;
    _initPListDatabase: NSDictionary;
    _initPListTarget: id;
    _selectedItemIdentifier: NSString;
    _metrics: Pointer; {garbage collector: __strong }
    _delegate: id;
    _window: NSWindow;
    _configPalette: id;
    _toolbarView: id;
    _syncPostEnabledCount: NSInteger;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __tbFlags: record
      case byte of
       0: (_anonbitfield_NSToolbar0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __tbFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSToolbar0: cuint);
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
        _allowsUserCustomization: 0..1;
        autosavesUsingIdentifier: 0..1;
        initialConfigurationDone: 0..1;
        shouldHideAfterCustomization: 0..1;
        delegateDefaultItemIdentifiers: 0..1;
        delegateAllowedItemIdentifiers: 0..1;
        delegateItemWithItemIdentifier: 0..1;
        delegateNotificationsEnabled: 0..1;
        prefersToBeShown: 0..1;
        loadItemsImmediately: 0..1;
        currentItemsContainsPlaceholder: 0..1;
        customizationPanelIsRunning: 0..1;
        usesCustomSheetWidth: 0..1;
        clickAndDragPerformsCustomization: 0..1;
        showsNoContextMenu: 0..1;
        currentlyLoadingPlaceholders: 0..1;
        delegateItemWithItemIdentifier2: 0..1;
        reserved: 0..((1 shl 4)-1);
        keyboardLoopNeedsUpdating: 0..1;
        showHideDuringConfigurationChangeDisabled: 0..1;
        _displayMode: 0..((1 shl 2)-1);
        _sizeMode: 0..((1 shl 2)-1);
        doNotShowBaselineSeparator: 0..1;
        hideWithoutResizingWindowHint: 0..1;
        autovalidatesItemsDisabled: 0..1;
        inAutovalidation: 0..1;
        loadedMetrics: 0..1;
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
    _customizationSheetWidth: NSInteger;
    _tbReserved: id;
    
  public
    function initWithIdentifier(identifier_: NSString): id; message 'initWithIdentifier:';
    procedure insertItemWithItemIdentifier_atIndex(itemIdentifier: NSString; index: NSInteger); message 'insertItemWithItemIdentifier:atIndex:';
    procedure removeItemAtIndex(index: NSInteger); message 'removeItemAtIndex:';
    procedure setDelegate(delegate_: NSToolbarDelegateProtocol); message 'setDelegate:';
    function delegate: NSToolbarDelegateProtocol; message 'delegate';
    procedure setVisible(shown: Boolean); message 'setVisible:';
    function isVisible: Boolean; message 'isVisible';
    procedure runCustomizationPalette(sender: id); message 'runCustomizationPalette:';
    function customizationPaletteIsRunning: Boolean; message 'customizationPaletteIsRunning';
    procedure setDisplayMode(displayMode_: NSToolbarDisplayMode); message 'setDisplayMode:';
    function displayMode: NSToolbarDisplayMode; message 'displayMode';
    procedure setSelectedItemIdentifier(itemIdentifier: NSString); message 'setSelectedItemIdentifier:';
    function selectedItemIdentifier: NSString; message 'selectedItemIdentifier';
    procedure setSizeMode(sizeMode_: NSToolbarSizeMode); message 'setSizeMode:';
    function sizeMode: NSToolbarSizeMode; message 'sizeMode';
    procedure setShowsBaselineSeparator(flag: Boolean); message 'setShowsBaselineSeparator:';
    function showsBaselineSeparator: Boolean; message 'showsBaselineSeparator';
    procedure setAllowsUserCustomization(allowCustomization: Boolean); message 'setAllowsUserCustomization:';
    function allowsUserCustomization: Boolean; message 'allowsUserCustomization';
    function identifier: NSString; message 'identifier';
    function items: NSArray; message 'items';
    function visibleItems: NSArray; message 'visibleItems';
    procedure setAutosavesConfiguration(flag: Boolean); message 'setAutosavesConfiguration:';
    function autosavesConfiguration: Boolean; message 'autosavesConfiguration';
    procedure setConfigurationFromDictionary(configDict: NSDictionary); message 'setConfigurationFromDictionary:';
    function configurationDictionary: NSDictionary; message 'configurationDictionary';
    procedure validateVisibleItems; message 'validateVisibleItems';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSTOOLBAR_PAS_P}
{$define NSTOOLBAR_PAS_P}
  
{ NSToolbarDelegate Protocol }
  NSToolbarDelegateProtocol = objcprotocol external name 'NSToolbarDelegate'
  optional
    function toolbar_itemForItemIdentifier_willBeInsertedIntoToolbar(toolbar: NSToolbar; itemIdentifier: NSString; flag: Boolean): NSToolbarItem; message 'toolbar:itemForItemIdentifier:willBeInsertedIntoToolbar:';
    function toolbarDefaultItemIdentifiers(toolbar: NSToolbar): NSArray; message 'toolbarDefaultItemIdentifiers:';
    function toolbarAllowedItemIdentifiers(toolbar: NSToolbar): NSArray; message 'toolbarAllowedItemIdentifiers:';
    function toolbarSelectableItemIdentifiers(toolbar: NSToolbar): NSArray; message 'toolbarSelectableItemIdentifiers:';
    procedure toolbarWillAddItem(notification: NSNotification); message 'toolbarWillAddItem:';
    procedure toolbarDidRemoveItem(notification: NSNotification); message 'toolbarDidRemoveItem:';
  end;
{$endif}
{$endif}
