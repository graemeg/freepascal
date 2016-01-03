{ Parsed from Appkit.framework NSDocument.h }

{$ifdef TYPES}
{$ifndef NSDOCUMENT_PAS_T}
{$define NSDOCUMENT_PAS_T}

{ Constants }

const
  NSChangeDone = 0;
  NSChangeUndone = 1;
  NSChangeCleared = 2;
  NSChangeRedone = 5;
  NSChangeReadOtherContents = 3;
  NSChangeAutosaved = 4;

const
  NSSaveOperation = 0;
  NSSaveAsOperation = 1;
  NSSaveToOperation = 2;
  NSAutosaveOperation = 3;

{ Types }
type
  NSDocumentChangeType = NSUInteger;
  NSDocumentChangeTypePtr = ^NSDocumentChangeType;
  NSSaveOperationType = NSUInteger;
  NSSaveOperationTypePtr = ^NSSaveOperationType;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSDOCUMENT_PAS_R}
{$define NSDOCUMENT_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSDOCUMENT_PAS_F}
{$define NSDOCUMENT_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSDOCUMENT_PAS_S}
{$define NSDOCUMENT_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSDocument = objcclass;
  NSDocumentPointer = ^NSDocument;
  NSDocumentPtr = NSDocumentPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSDOCUMENT_PAS_C}
{$define NSDOCUMENT_PAS_C}

{ NSDocument }
  NSDocument = objcclass external (NSObject, NSUserInterfaceValidationsProtocol)
  private
    _window: NSWindow;
    _windowControllers: id;
    _fileURL: NSURL;
    _fileType: NSString;
    _printInfo: NSPrintInfo;
    _changeCount: clong;
    savePanelAccessory: NSView;
    _displayName: id;
    _privateData: id;
    _undoManager: NSUndoManager;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __docFlags: record
      case byte of
       0: (_anonbitfield_NSDocument0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __docFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSDocument0: cuint);
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
        inClose: 0..1;
        _hasUndoManager: 0..1;
        isShowingPageLayout: 0..1;
        isRunningPrintOperation: 0..1;
        savePanelNameExtensionHidden: 0..1;
        reconciledToFileName: 0..1;
        checkingDisplayName: 0..1;
        definitelyHasUnsavedChanges: 0..1;
        definitelyHasUnautosavedChanges: 0..1;
        RESERVED: 0..((1 shl 23)-1);
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
    _savePanelSaveType: NSString;
    
  public
    function init: id; message 'init';
    function initWithType_error(typeName: NSString; outError: NSErrorPointer): id; message 'initWithType:error:';
    class function canConcurrentlyReadDocumentsOfType(typeName: NSString): Boolean; message 'canConcurrentlyReadDocumentsOfType:';
    function initWithContentsOfURL_ofType_error(absoluteURL: NSURL; typeName: NSString; outError: NSErrorPointer): id; message 'initWithContentsOfURL:ofType:error:';
    function initForURL_withContentsOfURL_ofType_error(absoluteDocumentURL: NSURL; absoluteDocumentContentsURL: NSURL; typeName: NSString; outError: NSErrorPointer): id; message 'initForURL:withContentsOfURL:ofType:error:';
    procedure setFileType(typeName: NSString); message 'setFileType:';
    function fileType: NSString; message 'fileType';
    procedure setFileURL(absoluteURL: NSURL); message 'setFileURL:';
    function fileURL: NSURL; message 'fileURL';
    procedure setFileModificationDate(modificationDate: NSDate); message 'setFileModificationDate:';
    function fileModificationDate: NSDate; message 'fileModificationDate';
    procedure revertDocumentToSaved(sender: id); message 'revertDocumentToSaved:';
    function revertToContentsOfURL_ofType_error(absoluteURL: NSURL; typeName: NSString; outError: NSErrorPointer): Boolean; message 'revertToContentsOfURL:ofType:error:';
    function readFromURL_ofType_error(absoluteURL: NSURL; typeName: NSString; outError: NSErrorPointer): Boolean; message 'readFromURL:ofType:error:';
    function readFromFileWrapper_ofType_error(fileWrapper: NSFileWrapper; typeName: NSString; outError: NSErrorPointer): Boolean; message 'readFromFileWrapper:ofType:error:';
    function readFromData_ofType_error(data: NSData; typeName: NSString; outError: NSErrorPointer): Boolean; message 'readFromData:ofType:error:';
    function writeToURL_ofType_error(absoluteURL: NSURL; typeName: NSString; outError: NSErrorPointer): Boolean; message 'writeToURL:ofType:error:';
    function fileWrapperOfType_error(typeName: NSString; outError: NSErrorPointer): NSFileWrapper; message 'fileWrapperOfType:error:';
    function dataOfType_error(typeName: NSString; outError: NSErrorPointer): NSData; message 'dataOfType:error:';
    function writeSafelyToURL_ofType_forSaveOperation_error(absoluteURL: NSURL; typeName: NSString; saveOperation: NSSaveOperationType; outError: NSErrorPointer): Boolean; message 'writeSafelyToURL:ofType:forSaveOperation:error:';
    function writeToURL_ofType_forSaveOperation_originalContentsURL_error(absoluteURL: NSURL; typeName: NSString; saveOperation: NSSaveOperationType; absoluteOriginalContentsURL: NSURL; outError: NSErrorPointer): Boolean; message 'writeToURL:ofType:forSaveOperation:originalContentsURL:error:';
    function fileAttributesToWriteToURL_ofType_forSaveOperation_originalContentsURL_error(absoluteURL: NSURL; typeName: NSString; saveOperation: NSSaveOperationType; absoluteOriginalContentsURL: NSURL; outError: NSErrorPointer): NSDictionary; message 'fileAttributesToWriteToURL:ofType:forSaveOperation:originalContentsURL:error:';
    function keepBackupFile: Boolean; message 'keepBackupFile';
    procedure saveDocument(sender: id); message 'saveDocument:';
    procedure saveDocumentAs(sender: id); message 'saveDocumentAs:';
    procedure saveDocumentTo(sender: id); message 'saveDocumentTo:';
    procedure saveDocumentWithDelegate_didSaveSelector_contextInfo(delegate: id; didSaveSelector: SEL; contextInfo: Pointer); message 'saveDocumentWithDelegate:didSaveSelector:contextInfo:';
    procedure runModalSavePanelForSaveOperation_delegate_didSaveSelector_contextInfo(saveOperation: NSSaveOperationType; delegate: id; didSaveSelector: SEL; contextInfo: Pointer); message 'runModalSavePanelForSaveOperation:delegate:didSaveSelector:contextInfo:';
    function shouldRunSavePanelWithAccessoryView: Boolean; message 'shouldRunSavePanelWithAccessoryView';
    function prepareSavePanel(savePanel: NSSavePanel): Boolean; message 'prepareSavePanel:';
    function fileNameExtensionWasHiddenInLastRunSavePanel: Boolean; message 'fileNameExtensionWasHiddenInLastRunSavePanel';
    function fileTypeFromLastRunSavePanel: NSString; message 'fileTypeFromLastRunSavePanel';
    procedure saveToURL_ofType_forSaveOperation_delegate_didSaveSelector_contextInfo(absoluteURL: NSURL; typeName: NSString; saveOperation: NSSaveOperationType; delegate: id; didSaveSelector: SEL; contextInfo: Pointer); message 'saveToURL:ofType:forSaveOperation:delegate:didSaveSelector:contextInfo:';
    function saveToURL_ofType_forSaveOperation_error(absoluteURL: NSURL; typeName: NSString; saveOperation: NSSaveOperationType; outError: NSErrorPointer): Boolean; message 'saveToURL:ofType:forSaveOperation:error:';
    function hasUnautosavedChanges: Boolean; message 'hasUnautosavedChanges';
    procedure autosaveDocumentWithDelegate_didAutosaveSelector_contextInfo(delegate: id; didAutosaveSelector: SEL; contextInfo: Pointer); message 'autosaveDocumentWithDelegate:didAutosaveSelector:contextInfo:';
    function autosavingFileType: NSString; message 'autosavingFileType';
    procedure setAutosavedContentsFileURL(absoluteURL: NSURL); message 'setAutosavedContentsFileURL:';
    function autosavedContentsFileURL: NSURL; message 'autosavedContentsFileURL';
    procedure canCloseDocumentWithDelegate_shouldCloseSelector_contextInfo(delegate: id; shouldCloseSelector: SEL; contextInfo: Pointer); message 'canCloseDocumentWithDelegate:shouldCloseSelector:contextInfo:';
    procedure close; message 'close';
    procedure runPageLayout(sender: id); message 'runPageLayout:';
    procedure runModalPageLayoutWithPrintInfo_delegate_didRunSelector_contextInfo(printInfo_: NSPrintInfo; delegate: id; didRunSelector: SEL; contextInfo: Pointer); message 'runModalPageLayoutWithPrintInfo:delegate:didRunSelector:contextInfo:';
    function preparePageLayout(pageLayout: NSPageLayout): Boolean; message 'preparePageLayout:';
    function shouldChangePrintInfo(newPrintInfo: NSPrintInfo): Boolean; message 'shouldChangePrintInfo:';
    procedure setPrintInfo(printInfo_: NSPrintInfo); message 'setPrintInfo:';
    function printInfo: NSPrintInfo; message 'printInfo';
    procedure printDocument(sender: id); message 'printDocument:';
    procedure printDocumentWithSettings_showPrintPanel_delegate_didPrintSelector_contextInfo(printSettings: NSDictionary; showPrintPanel: Boolean; delegate: id; didPrintSelector: SEL; contextInfo: Pointer); message 'printDocumentWithSettings:showPrintPanel:delegate:didPrintSelector:contextInfo:';
    function printOperationWithSettings_error(printSettings: NSDictionary; outError: NSErrorPointer): NSPrintOperation; message 'printOperationWithSettings:error:';
    procedure runModalPrintOperation_delegate_didRunSelector_contextInfo(printOperation: NSPrintOperation; delegate: id; didRunSelector: SEL; contextInfo: Pointer); message 'runModalPrintOperation:delegate:didRunSelector:contextInfo:';
    function isDocumentEdited: Boolean; message 'isDocumentEdited';
    procedure updateChangeCount(change: NSDocumentChangeType); message 'updateChangeCount:';
    procedure setUndoManager(undoManager_: NSUndoManager); message 'setUndoManager:';
    function undoManager: NSUndoManager; message 'undoManager';
    procedure setHasUndoManager(hasUndoManager_: Boolean); message 'setHasUndoManager:';
    function hasUndoManager: Boolean; message 'hasUndoManager';
    procedure presentError_modalForWindow_delegate_didPresentSelector_contextInfo(error: NSError; window: NSWindow; delegate: id; didPresentSelector: SEL; contextInfo: Pointer); message 'presentError:modalForWindow:delegate:didPresentSelector:contextInfo:';
    function presentError(error: NSError): Boolean; message 'presentError:';
    function willPresentError(error: NSError): NSError; message 'willPresentError:';
    procedure makeWindowControllers; message 'makeWindowControllers';
    function windowNibName: NSString; message 'windowNibName';
    procedure windowControllerWillLoadNib(windowController: NSWindowController); message 'windowControllerWillLoadNib:';
    procedure windowControllerDidLoadNib(windowController: NSWindowController); message 'windowControllerDidLoadNib:';
    procedure setWindow(window: NSWindow); message 'setWindow:';
    procedure addWindowController(windowController: NSWindowController); message 'addWindowController:';
    procedure removeWindowController(windowController: NSWindowController); message 'removeWindowController:';
    procedure showWindows; message 'showWindows';
    function windowControllers: NSArray; message 'windowControllers';
    procedure shouldCloseWindowController_delegate_shouldCloseSelector_contextInfo(windowController: NSWindowController; delegate: id; shouldCloseSelector: SEL; contextInfo: Pointer); message 'shouldCloseWindowController:delegate:shouldCloseSelector:contextInfo:';
    function displayName: NSString; message 'displayName';
    function windowForSheet: NSWindow; message 'windowForSheet';
    class function readableTypes: NSArray; message 'readableTypes';
    class function writableTypes: NSArray; message 'writableTypes';
    class function isNativeType(type_: NSString): Boolean; message 'isNativeType:';
    function writableTypesForSaveOperation(saveOperation: NSSaveOperationType): NSArray; message 'writableTypesForSaveOperation:';
    function fileNameExtensionForType_saveOperation(typeName: NSString; saveOperation: NSSaveOperationType): NSString; message 'fileNameExtensionForType:saveOperation:';
    function validateUserInterfaceItem(anItem: NSValidatedUserInterfaceItemProtocol): Boolean; message 'validateUserInterfaceItem:';

    { Adopted Protocols }
  end;

{ NSDeprecated_NSDocumentCategory }
  NSDeprecated_NSDocumentCategory = objccategory external name 'NSDeprecated' (NSDocument)
    function dataRepresentationOfType(type_: NSString): NSData; message 'dataRepresentationOfType:'; deprecated 'in Mac OS X 10.4 and later';
    function fileAttributesToWriteToFile_ofType_saveOperation(fullDocumentPath: NSString; documentTypeName: NSString; saveOperationType: NSSaveOperationType): NSDictionary; message 'fileAttributesToWriteToFile:ofType:saveOperation:'; deprecated 'in Mac OS X 10.4 and later';
    function fileName: NSString; message 'fileName'; deprecated 'in Mac OS X 10.4 and later';
    function fileWrapperRepresentationOfType(type_: NSString): NSFileWrapper; message 'fileWrapperRepresentationOfType:'; deprecated 'in Mac OS X 10.4 and later';
    function initWithContentsOfFile_ofType(absolutePath: NSString; typeName: NSString): id; message 'initWithContentsOfFile:ofType:'; deprecated 'in Mac OS X 10.4 and later';
    function initWithContentsOfURL_ofType(absoluteURL: NSURL; typeName: NSString): id; message 'initWithContentsOfURL:ofType:'; deprecated 'in Mac OS X 10.4 and later';
    function loadDataRepresentation_ofType(data: NSData; type_: NSString): Boolean; message 'loadDataRepresentation:ofType:'; deprecated 'in Mac OS X 10.4 and later';
    function loadFileWrapperRepresentation_ofType(wrapper: NSFileWrapper; type_: NSString): Boolean; message 'loadFileWrapperRepresentation:ofType:'; deprecated 'in Mac OS X 10.4 and later';
    procedure printShowingPrintPanel(flag: Boolean); message 'printShowingPrintPanel:'; deprecated 'in Mac OS X 10.4 and later';
    function readFromFile_ofType(fileName_: NSString; type_: NSString): Boolean; message 'readFromFile:ofType:'; deprecated 'in Mac OS X 10.4 and later';
    function readFromURL_ofType(url: NSURL; type_: NSString): Boolean; message 'readFromURL:ofType:'; deprecated 'in Mac OS X 10.4 and later';
    function revertToSavedFromFile_ofType(fileName_: NSString; type_: NSString): Boolean; message 'revertToSavedFromFile:ofType:'; deprecated 'in Mac OS X 10.4 and later';
    function revertToSavedFromURL_ofType(url: NSURL; type_: NSString): Boolean; message 'revertToSavedFromURL:ofType:'; deprecated 'in Mac OS X 10.4 and later';
    function runModalPageLayoutWithPrintInfo(printInfo_: NSPrintInfo): NSInteger; message 'runModalPageLayoutWithPrintInfo:'; deprecated 'in Mac OS X 10.4 and later';
    procedure saveToFile_saveOperation_delegate_didSaveSelector_contextInfo(fileName_: NSString; saveOperation: NSSaveOperationType; delegate: id; didSaveSelector: SEL; contextInfo: Pointer); message 'saveToFile:saveOperation:delegate:didSaveSelector:contextInfo:'; deprecated 'in Mac OS X 10.4 and later';
    procedure setFileName(fileName_: NSString); message 'setFileName:'; deprecated 'in Mac OS X 10.4 and later';
    function writeToFile_ofType(fileName_: NSString; type_: NSString): Boolean; message 'writeToFile:ofType:'; deprecated 'in Mac OS X 10.4 and later';
    function writeToFile_ofType_originalFile_saveOperation(fullDocumentPath: NSString; documentTypeName: NSString; fullOriginalDocumentPath: NSString; saveOperationType: NSSaveOperationType): Boolean; message 'writeToFile:ofType:originalFile:saveOperation:'; deprecated 'in Mac OS X 10.4 and later';
    function writeToURL_ofType(url: NSURL; type_: NSString): Boolean; message 'writeToURL:ofType:'; deprecated 'in Mac OS X 10.4 and later';
    function writeWithBackupToFile_ofType_saveOperation(fullDocumentPath: NSString; documentTypeName: NSString; saveOperationType: NSSaveOperationType): Boolean; message 'writeWithBackupToFile:ofType:saveOperation:'; deprecated 'in Mac OS X 10.4 and later';
  end;

{$endif}
{$endif}
