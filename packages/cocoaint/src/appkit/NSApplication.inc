{ Parsed from Appkit.framework NSApplication.h }

{$ifdef TYPES}
{$ifndef NSAPPLICATION_PAS_T}
{$define NSAPPLICATION_PAS_T}

{ Defines }
const
  NSAppKitVersionNumber10_0 = 577;
  NSAppKitVersionNumber10_1 = 620;
  NSAppKitVersionNumber10_2 = 663;
  NSAppKitVersionNumber10_2_3 = 663.6;
  NSAppKitVersionNumber10_3 = 743;
  NSAppKitVersionNumber10_3_2 = 743.14;
  NSAppKitVersionNumber10_3_3 = 743.2;
  NSAppKitVersionNumber10_3_5 = 743.24;
  NSAppKitVersionNumber10_3_7 = 743.33;
  NSAppKitVersionNumber10_3_9 = 743.36;
  NSAppKitVersionNumber10_4 = 824;
  NSAppKitVersionNumber10_4_1 = 824.1;
  NSAppKitVersionNumber10_4_3 = 824.23;
  NSAppKitVersionNumber10_4_4 = 824.33;
  NSAppKitVersionNumber10_4_7 = 824.41;
  NSAppKitVersionNumber10_5 = 949;
  NSAppKitVersionNumber10_5_2 = 949.27;
  NSAppKitVersionNumber10_5_3 = 949.33;

{ Constants }

const
  NSRunStoppedResponse = -1000;
  NSRunAbortedResponse = -1001;
  NSRunContinuesResponse = -1002;

const
  NSUpdateWindowsRunLoopOrdering = 500000;

const
  NSApplicationPresentationDefault = 0;
  NSApplicationPresentationAutoHideDock = 1 shl  0;
  NSApplicationPresentationHideDock = 1 shl  1;
  NSApplicationPresentationAutoHideMenuBar = 1 shl  2;
  NSApplicationPresentationHideMenuBar = 1 shl  3;
  NSApplicationPresentationDisableAppleMenu = 1 shl  4;
  NSApplicationPresentationDisableProcessSwitching = 1 shl  5;
  NSApplicationPresentationDisableForceQuit = 1 shl  6;
  NSApplicationPresentationDisableSessionTermination = 1 shl  7;
  NSApplicationPresentationDisableHideApplication = 1 shl  8;
  NSApplicationPresentationDisableMenuBarTransparency = 1 shl  9;

const
  NSCriticalRequest = 0;
  NSInformationalRequest = 10;

const
  NSApplicationDelegateReplySuccess = 0;
  NSApplicationDelegateReplyCancel = 1;
  NSApplicationDelegateReplyFailure = 2;

const
  NSTerminateCancel = 0;
  NSTerminateNow = 1;
  NSTerminateLater = 2;

const
  NSPrintingCancelled = 0;
  NSPrintingSuccess = 1;
  NSPrintingFailure = 3;
  NSPrintingReplyLater = 2;

const
  NSUserInterfaceLayoutDirectionLeftToRight = 0;
  NSUserInterfaceLayoutDirectionRightToLeft = 1;

{ Types }
type
  NSApplicationPresentationOptions = NSUInteger;
  NSApplicationPresentationOptionsPtr = ^NSApplicationPresentationOptions;
  NSModalSession = Pointer;
  NSModalSessionPtr = ^NSModalSession;
  NSThreadPrivate = record end;
  _NSThreadPrivate = NSThreadPrivate;
  _NSThreadPrivatePtr = ^_NSThreadPrivate;
  NSRequestUserAttentionType = NSUInteger;
  NSRequestUserAttentionTypePtr = ^NSRequestUserAttentionType;
  NSApplicationDelegateReply = NSUInteger;
  NSApplicationDelegateReplyPtr = ^NSApplicationDelegateReply;
  NSApplicationTerminateReply = NSUInteger;
  NSApplicationTerminateReplyPtr = ^NSApplicationTerminateReply;
  NSApplicationPrintReply = NSUInteger;
  NSApplicationPrintReplyPtr = ^NSApplicationPrintReply;
  NSUserInterfaceLayoutDirection = NSInteger;
  NSUserInterfaceLayoutDirectionPtr = ^NSUserInterfaceLayoutDirection;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSAPPLICATION_PAS_R}
{$define NSAPPLICATION_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSAPPLICATION_PAS_F}
{$define NSAPPLICATION_PAS_F}

{ Functions }
function NSApplicationMain(argc: cint; argv: PPChar {array of PChar}): cint; cdecl; external;
function NSApplicationLoad: Boolean; cdecl; external;
function NSShowsServicesMenuItem(itemName: NSString): Boolean; cdecl; external;
function NSSetShowsServicesMenuItem(itemName: NSString; enabled: Boolean): NSInteger; cdecl; external;
procedure NSUpdateDynamicServices; cdecl; external;
function NSPerformService(itemName: NSString; pboard: NSPasteboard): Boolean; cdecl; external;
procedure NSRegisterServicesProvider(provider: id; name: NSString); cdecl; external;
procedure NSUnregisterServicesProvider(name: NSString); cdecl; external;

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSAPPLICATION_PAS_S}
{$define NSAPPLICATION_PAS_S}

{ External symbols }
var
  NSAppKitVersionNumber: double; cvar; external;
  NSApp: id; cvar; external;

{ External string constants }
var
  NSModalPanelRunLoopMode: NSString; cvar; external;
  NSEventTrackingRunLoopMode: NSString; cvar; external;
  NSApplicationDidBecomeActiveNotification: NSString; cvar; external;
  NSApplicationDidHideNotification: NSString; cvar; external;
  NSApplicationDidFinishLaunchingNotification: NSString; cvar; external;
  NSApplicationDidResignActiveNotification: NSString; cvar; external;
  NSApplicationDidUnhideNotification: NSString; cvar; external;
  NSApplicationDidUpdateNotification: NSString; cvar; external;
  NSApplicationWillBecomeActiveNotification: NSString; cvar; external;
  NSApplicationWillHideNotification: NSString; cvar; external;
  NSApplicationWillFinishLaunchingNotification: NSString; cvar; external;
  NSApplicationWillResignActiveNotification: NSString; cvar; external;
  NSApplicationWillUnhideNotification: NSString; cvar; external;
  NSApplicationWillUpdateNotification: NSString; cvar; external;
  NSApplicationWillTerminateNotification: NSString; cvar; external;
  NSApplicationDidChangeScreenParametersNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSApplicationDelegateProtocol = objcprotocol;
  NSApplication = objcclass;
  NSApplicationPointer = ^NSApplication;
  NSApplicationPtr = NSApplicationPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSAPPLICATION_PAS_C}
{$define NSAPPLICATION_PAS_C}

{ NSApplication }
  NSApplication = objcclass external (NSResponder, NSUserInterfaceValidationsProtocol)
  private
    _currentEvent: NSEvent;
    _windowList: id;
    _keyWindow: id;
    _mainWindow: id;
    _delegate: id;
    _hiddenList: idPtr;
    _hiddenCount: cint;
    _context: NSInteger;
    _appleEventSuspensionID: Pointer;
    _previousKeyWindow: id; {garbage collector: __weak }
    _unusedApp: cshort;
    _running: cshort;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __appFlags: record
      case byte of
       0: (_anonbitfield_NSApplication0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __appFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSApplication0: cuint);
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
        _hidden: 0..1;
        _RESERVED1: 0..1;
        _active: 0..1;
        _hasBeenRun: 0..1;
        _doingUnhide: 0..1;
        _delegateReturnsValidRequestor: 0..1;
        _deactPending: 0..1;
        _invalidState: 0..1;
        _invalidEvent: 0..1;
        _postedWindowsNeedUpdateNote: 0..1;
        _wantsToActivate: 0..1;
        _doingHide: 0..1;
        _dontSendShouldTerminate: 0..1;
        _skipWin32DelayedRestoreKeyWindowAfterHide: 0..1;
        _finishedLaunching: 0..1;
        _hasEventDelegate: 0..1;
        _appDying: 0..1;
        _didNSOpenOrPrint: 0..1;
        _inDealloc: 0..1;
        _pendingDidFinish: 0..1;
        _hasKeyFocus: 0..1;
        _panelsNonactivating: 0..1;
        _hiddenOnLaunch: 0..1;
        _openStatus: 0..((1 shl 2)-1);
        _batchOrdering: 0..1;
        _waitingForTerminationReply: 0..1;
        _windowMoveDisabled: 0..1;
        _reserved: 0..((1 shl 4)-1);
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
    _mainMenu: id;
    _appIcon: id;
    _nameTable: id;
    _eventDelegate: id;
    _threadingSupport: _NSThreadPrivatePtr;
    
  public
    class function sharedApplication: NSApplication; message 'sharedApplication';
    procedure setDelegate(anObject: NSApplicationDelegateProtocol); message 'setDelegate:';
    function delegate: NSApplicationDelegateProtocol; message 'delegate';
    function context: NSGraphicsContext; message 'context';
    procedure hide(sender: id); message 'hide:';
    procedure unhide(sender: id); message 'unhide:';
    procedure unhideWithoutActivation; message 'unhideWithoutActivation';
    function windowWithWindowNumber(windowNum: NSInteger): NSWindow; message 'windowWithWindowNumber:';
    function mainWindow: NSWindow; message 'mainWindow';
    function keyWindow: NSWindow; message 'keyWindow';
    function isActive: Boolean; message 'isActive';
    function isHidden: Boolean; message 'isHidden';
    function isRunning: Boolean; message 'isRunning';
    procedure deactivate; message 'deactivate';
    procedure activateIgnoringOtherApps(flag: Boolean); message 'activateIgnoringOtherApps:';
    procedure hideOtherApplications(sender: id); message 'hideOtherApplications:';
    procedure unhideAllApplications(sender: id); message 'unhideAllApplications:';
    procedure finishLaunching; message 'finishLaunching';
    procedure run; message 'run';
    function runModalForWindow(theWindow: NSWindow): NSInteger; message 'runModalForWindow:';
    procedure stop(sender: id); message 'stop:';
    procedure stopModal; message 'stopModal';
    procedure stopModalWithCode(returnCode: NSInteger); message 'stopModalWithCode:';
    procedure abortModal; message 'abortModal';
    function modalWindow: NSWindow; message 'modalWindow';
    function beginModalSessionForWindow(theWindow: NSWindow): NSModalSession; message 'beginModalSessionForWindow:';
    function runModalSession(session: NSModalSession): NSInteger; message 'runModalSession:';
    procedure endModalSession(session: NSModalSession); message 'endModalSession:';
    procedure terminate(sender: id); message 'terminate:';
    function requestUserAttention(requestType: NSRequestUserAttentionType): NSInteger; message 'requestUserAttention:';
    procedure cancelUserAttentionRequest(request: NSInteger); message 'cancelUserAttentionRequest:';
    procedure beginSheet_modalForWindow_modalDelegate_didEndSelector_contextInfo(sheet: NSWindow; docWindow: NSWindow; modalDelegate: id; didEndSelector: SEL; contextInfo: Pointer); message 'beginSheet:modalForWindow:modalDelegate:didEndSelector:contextInfo:';
    procedure endSheet(sheet: NSWindow); message 'endSheet:';
    procedure endSheet_returnCode(sheet: NSWindow; returnCode: NSInteger); message 'endSheet:returnCode:';
    function nextEventMatchingMask_untilDate_inMode_dequeue(mask: NSUInteger; expiration: NSDate; mode: NSString; deqFlag: Boolean): NSEvent; message 'nextEventMatchingMask:untilDate:inMode:dequeue:';
    procedure discardEventsMatchingMask_beforeEvent(mask: NSUInteger; lastEvent: NSEvent); message 'discardEventsMatchingMask:beforeEvent:';
    procedure postEvent_atStart(event: NSEvent; flag: Boolean); message 'postEvent:atStart:';
    function currentEvent: NSEvent; message 'currentEvent';
    procedure sendEvent(theEvent: NSEvent); message 'sendEvent:';
    procedure preventWindowOrdering; message 'preventWindowOrdering';
    function makeWindowsPerform_inOrder(aSelector: SEL; flag: Boolean): NSWindow; message 'makeWindowsPerform:inOrder:';
    function windows: NSArray; message 'windows';
    procedure setWindowsNeedUpdate(needUpdate: Boolean); message 'setWindowsNeedUpdate:';
    procedure updateWindows; message 'updateWindows';
    procedure setMainMenu(aMenu: NSMenu); message 'setMainMenu:';
    function mainMenu: NSMenu; message 'mainMenu';
    procedure setHelpMenu(helpMenu_: NSMenu); message 'setHelpMenu:';
    function helpMenu: NSMenu; message 'helpMenu';
    procedure setApplicationIconImage(image: NSImage); message 'setApplicationIconImage:';
    function applicationIconImage: NSImage; message 'applicationIconImage';
    function activationPolicy: NSApplicationActivationPolicy; message 'activationPolicy';
    function setActivationPolicy(activationPolicy_: NSApplicationActivationPolicy): Boolean; message 'setActivationPolicy:';
    function dockTile: NSDockTile; message 'dockTile';
    function sendAction_to_from(theAction: SEL; theTarget: id; sender: id): Boolean; message 'sendAction:to:from:';
    function targetForAction(theAction: SEL): id; message 'targetForAction:';
    function targetForAction_to_from(theAction: SEL; theTarget: id; sender: id): id; message 'targetForAction:to:from:';
    function tryToPerform_with(anAction: SEL; anObject: id): Boolean; message 'tryToPerform:with:';
    function validRequestorForSendType_returnType(sendType: NSString; returnType: NSString): id; message 'validRequestorForSendType:returnType:';
    procedure reportException(theException: NSException); message 'reportException:';
    class procedure detachDrawingThread_toTarget_withObject(selector: SEL; target: id; argument: id); message 'detachDrawingThread:toTarget:withObject:';
    procedure replyToApplicationShouldTerminate(shouldTerminate: Boolean); message 'replyToApplicationShouldTerminate:';
    procedure replyToOpenOrPrint(reply: NSApplicationDelegateReply); message 'replyToOpenOrPrint:';
    procedure orderFrontCharacterPalette(sender: id); message 'orderFrontCharacterPalette:';
    function presentationOptions: NSApplicationPresentationOptions; message 'presentationOptions';
    procedure setPresentationOptions(newOptions: NSApplicationPresentationOptions); message 'setPresentationOptions:';
    function currentSystemPresentationOptions: NSApplicationPresentationOptions; message 'currentSystemPresentationOptions';

    { Adopted Protocols }
    function validateUserInterfaceItem(anItem: NSValidatedUserInterfaceItemProtocol): Boolean;
  end;

{ NSWindowsMenuCategory }
  NSWindowsMenuCategory = objccategory external (NSApplication)
    procedure setWindowsMenu(aMenu: NSMenu); message 'setWindowsMenu:';
    function windowsMenu: NSMenu; message 'windowsMenu';
    procedure arrangeInFront(sender: id); message 'arrangeInFront:';
    procedure removeWindowsItem(win: NSWindow); message 'removeWindowsItem:';
    procedure addWindowsItem_title_filename(win: NSWindow; aString: NSString; isFilename: Boolean); message 'addWindowsItem:title:filename:';
    procedure changeWindowsItem_title_filename(win: NSWindow; aString: NSString; isFilename: Boolean); message 'changeWindowsItem:title:filename:';
    procedure updateWindowsItem(win: NSWindow); message 'updateWindowsItem:';
    procedure miniaturizeAll(sender: id); message 'miniaturizeAll:';
  end;

{ NSFullKeyboardAccessCategory }
  NSFullKeyboardAccessCategory = objccategory external (NSApplication)
    function isFullKeyboardAccessEnabled: Boolean; message 'isFullKeyboardAccessEnabled';
  end;

{ NSServicesMenuCategory }
  NSServicesMenuCategory = objccategory external (NSApplication)
    procedure setServicesMenu(aMenu: NSMenu); message 'setServicesMenu:';
    function servicesMenu: NSMenu; message 'servicesMenu';
    procedure registerServicesMenuSendTypes_returnTypes(sendTypes: NSArray; returnTypes: NSArray); message 'registerServicesMenuSendTypes:returnTypes:';
  end;

{ NSServicesRequestsCategory }
  NSServicesRequestsCategory = objccategory external (NSObject)
    function writeSelectionToPasteboard_types(pboard: NSPasteboard; types: NSArray): Boolean; message 'writeSelectionToPasteboard:types:';
    function readSelectionFromPasteboard(pboard: NSPasteboard): Boolean; message 'readSelectionFromPasteboard:';
  end;

{ NSServicesHandlingCategory }
  NSServicesHandlingCategory = objccategory external (NSApplication)
    procedure setServicesProvider(provider: id); message 'setServicesProvider:';
    function servicesProvider: id; message 'servicesProvider';
  end;

{ NSStandardAboutPanelCategory }
  NSStandardAboutPanelCategory = objccategory external (NSApplication)
    procedure orderFrontStandardAboutPanel(sender: id); message 'orderFrontStandardAboutPanel:';
    procedure orderFrontStandardAboutPanelWithOptions(optionsDictionary: NSDictionary); message 'orderFrontStandardAboutPanelWithOptions:';
  end;

{ NSApplicationLayoutDirectionCategory }
  NSApplicationLayoutDirectionCategory = objccategory external (NSApplication)
    function userInterfaceLayoutDirection: NSUserInterfaceLayoutDirection; message 'userInterfaceLayoutDirection';
  end;

{ NSDeprecated_NSApplicationCategory }
  NSDeprecated_NSApplicationCategory = objccategory external name 'NSDeprecated' (NSApplication)
    function runModalForWindow_relativeToWindow(theWindow: NSWindow; docWindow: NSWindow): NSInteger; message 'runModalForWindow:relativeToWindow:'; deprecated 'in Mac OS X 10 and later';
    function beginModalSessionForWindow_relativeToWindow(theWindow: NSWindow; docWindow: NSWindow): NSModalSession; message 'beginModalSessionForWindow:relativeToWindow:'; deprecated 'in Mac OS X 10 and later';
    procedure application_printFiles(sender: NSApplication; filenames: NSArray); message 'application:printFiles:'; deprecated 'in Mac OS X 10.4 and later';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSAPPLICATION_PAS_P}
{$define NSAPPLICATION_PAS_P}
  
{ NSApplicationDelegate Protocol }
  NSApplicationDelegateProtocol = objcprotocol external name 'NSApplicationDelegate'
  optional
    function applicationShouldTerminate(sender: NSApplication): NSApplicationTerminateReply; message 'applicationShouldTerminate:';
    function application_openFile(sender: NSApplication; filename: NSString): Boolean; message 'application:openFile:';
    procedure application_openFiles(sender: NSApplication; filenames: NSArray); message 'application:openFiles:';
    function application_openTempFile(sender: NSApplication; filename: NSString): Boolean; message 'application:openTempFile:';
    function applicationShouldOpenUntitledFile(sender: NSApplication): Boolean; message 'applicationShouldOpenUntitledFile:';
    function applicationOpenUntitledFile(sender: NSApplication): Boolean; message 'applicationOpenUntitledFile:';
    function application_openFileWithoutUI(sender: id; filename: NSString): Boolean; message 'application:openFileWithoutUI:';
    function application_printFile(sender: NSApplication; filename: NSString): Boolean; message 'application:printFile:';
    function application_printFiles_withSettings_showPrintPanels(application: NSApplication; fileNames: NSArray; printSettings: NSDictionary; showPrintPanels: Boolean): NSApplicationPrintReply; message 'application:printFiles:withSettings:showPrintPanels:';
    function applicationShouldTerminateAfterLastWindowClosed(sender: NSApplication): Boolean; message 'applicationShouldTerminateAfterLastWindowClosed:';
    function applicationShouldHandleReopen_hasVisibleWindows(sender: NSApplication; flag: Boolean): Boolean; message 'applicationShouldHandleReopen:hasVisibleWindows:';
    function applicationDockMenu(sender: NSApplication): NSMenu; message 'applicationDockMenu:';
    function application_willPresentError(application: NSApplication; error: NSError): NSError; message 'application:willPresentError:';
    procedure applicationWillFinishLaunching(notification: NSNotification); message 'applicationWillFinishLaunching:';
    procedure applicationDidFinishLaunching(notification: NSNotification); message 'applicationDidFinishLaunching:';
    procedure applicationWillHide(notification: NSNotification); message 'applicationWillHide:';
    procedure applicationDidHide(notification: NSNotification); message 'applicationDidHide:';
    procedure applicationWillUnhide(notification: NSNotification); message 'applicationWillUnhide:';
    procedure applicationDidUnhide(notification: NSNotification); message 'applicationDidUnhide:';
    procedure applicationWillBecomeActive(notification: NSNotification); message 'applicationWillBecomeActive:';
    procedure applicationDidBecomeActive(notification: NSNotification); message 'applicationDidBecomeActive:';
    procedure applicationWillResignActive(notification: NSNotification); message 'applicationWillResignActive:';
    procedure applicationDidResignActive(notification: NSNotification); message 'applicationDidResignActive:';
    procedure applicationWillUpdate(notification: NSNotification); message 'applicationWillUpdate:';
    procedure applicationDidUpdate(notification: NSNotification); message 'applicationDidUpdate:';
    procedure applicationWillTerminate(notification: NSNotification); message 'applicationWillTerminate:';
    procedure applicationDidChangeScreenParameters(notification: NSNotification); message 'applicationDidChangeScreenParameters:';
  end;
{$endif}
{$endif}
