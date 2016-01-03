{ Parsed from Appkit.framework NSSavePanel.h }

{$ifdef TYPES}
{$ifndef NSSAVEPANEL_PAS_T}
{$define NSSAVEPANEL_PAS_T}

{ Constants }

const
  NSFileHandlingPanelCancelButton = NSCancelButton;
  NSFileHandlingPanelOKButton = NSOKButton;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSSAVEPANEL_PAS_R}
{$define NSSAVEPANEL_PAS_R}

{ Records }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  __SPFlags = record
    case byte of
    0: (_anonbitfield_NSSavePanel0: cuint);
    1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  __SPFlags = bitpacked record
    case byte of
    0: (_anonbitfield_NSSavePanel0: cuint);
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
          saveMode: 0..1;
          isExpanded: 0..1;
          allowsOtherFileTypes: 0..1;
          canCreateDirectories: 0..1;
          canSelectedHiddenExtension: 0..1;
          inConfigureForDirectory: 0..1;
          delegate_shouldShowFilename: 0..1;
          delegate_compareFilename: 0..1;
          delegate_shouldEnableURL: 0..1;
          delegate_validateURL: 0..1;
          delegate_didChangeToDirectoryURL: 0..1;
          changingFrameSize: 0..1;
          movingAccessoryView: 0..1;
          userAccessoryViewFrameChange: 0..1;
          reserved: 0..((1 shl 18)-1);
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
_SPFlags = __SPFlags;
__SPFlagsPtr = ^__SPFlags;


{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSSAVEPANEL_PAS_F}
{$define NSSAVEPANEL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSSAVEPANEL_PAS_S}
{$define NSSAVEPANEL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSOpenSavePanelDelegateProtocol = objcprotocol;
  NSSavePanel = objcclass;
  NSSavePanelPointer = ^NSSavePanel;
  NSSavePanelPtr = NSSavePanelPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSSAVEPANEL_PAS_C}
{$define NSSAVEPANEL_PAS_C}

{ NSSavePanel }
  NSSavePanel = objcclass external (NSPanel)
  private
    _navView: NSNavView;
    _accessoryView: NSView;
    _allowedFileTypes: NSArray;
    _validatedPosixName: NSString;
    _hiddenExtension: NSString;
    _messageTextField: NSTextField;
    _savePane: NSView;
    _saveNavSeparatorBox: NSBox;
    _savePaneTopPartsContainer: NSView;
    _nameField: NSTextField;
    _nameFieldLabel: NSTextField;
    _expansionButton: NSButton;
    _directoryPopUpContainer: NSView;
    _directoryPopUp: id;
    _directoryPopUpLabel: NSTextField;
    _navViewContainer: NSBox;
    _accessoryViewContainer: NSBox;
    _bottomControlsContainer: NSView;
    _hideExtensionButton: NSButton;
    _newFolderButton: NSButton;
    _cancelButton: NSButton;
    _okButton: NSButton;
    _filepathInputController: id;
    _newFolderController: id;
    _spFlags: _SPFlags;
    _spAuxiliaryStorage: NSSavePanelAuxiliary;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(5)-1] of char;
=======
        _reserved: array[0..4] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..4] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..4] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..4] of char;
>>>>>>> origin/cpstrnew
    _openProgressIndicator: NSProgressIndicator;
    
  public
    class function savePanel: NSSavePanel; message 'savePanel';
    function URL: NSURL; message 'URL';
    function directoryURL: NSURL; message 'directoryURL';
    procedure setDirectoryURL(url_: NSURL); message 'setDirectoryURL:';
    function allowedFileTypes: NSArray; message 'allowedFileTypes';
    procedure setAllowedFileTypes(types: NSArray); message 'setAllowedFileTypes:';
    function allowsOtherFileTypes: Boolean; message 'allowsOtherFileTypes';
    procedure setAllowsOtherFileTypes(flag: Boolean); message 'setAllowsOtherFileTypes:';
    function accessoryView: NSView; message 'accessoryView';
    procedure setAccessoryView(view: NSView); message 'setAccessoryView:';
    function delegate: NSOpenSavePanelDelegateProtocol; message 'delegate';
    procedure setDelegate(delegate_: NSOpenSavePanelDelegateProtocol); message 'setDelegate:';
    function isExpanded: Boolean; message 'isExpanded';
    function canCreateDirectories: Boolean; message 'canCreateDirectories';
    procedure setCanCreateDirectories(flag: Boolean); message 'setCanCreateDirectories:';
    function canSelectHiddenExtension: Boolean; message 'canSelectHiddenExtension';
    procedure setCanSelectHiddenExtension(flag: Boolean); message 'setCanSelectHiddenExtension:';
    function isExtensionHidden: Boolean; message 'isExtensionHidden';
    procedure setExtensionHidden(flag: Boolean); message 'setExtensionHidden:';
    function treatsFilePackagesAsDirectories: Boolean; message 'treatsFilePackagesAsDirectories';
    procedure setTreatsFilePackagesAsDirectories(flag: Boolean); message 'setTreatsFilePackagesAsDirectories:';
    function prompt: NSString; message 'prompt';
    procedure setPrompt(prompt_: NSString); message 'setPrompt:';
    function title: NSString; message 'title';
    procedure setTitle(title_: NSString); message 'setTitle:';
    function nameFieldLabel: NSString; message 'nameFieldLabel';
    procedure setNameFieldLabel(label_: NSString); message 'setNameFieldLabel:';
    function nameFieldStringValue: NSString; message 'nameFieldStringValue';
    procedure setNameFieldStringValue(value: NSString); message 'setNameFieldStringValue:';
    function message: NSString; message 'message';
    procedure setMessage(message_: NSString); message 'setMessage:';
    procedure validateVisibleColumns; message 'validateVisibleColumns';
    function showsHiddenFiles: Boolean; message 'showsHiddenFiles';
    procedure setShowsHiddenFiles(flag: Boolean); message 'setShowsHiddenFiles:';
    procedure ok(sender: id); message 'ok:';
    procedure cancel(sender: id); message 'cancel:';
    function runModal: NSInteger; message 'runModal';
  end;

{ NSSavePanelDelegateDeprecatedCategory }
  NSSavePanelDelegateDeprecatedCategory = objccategory external (NSObject)
    function panel_isValidFilename(sender: id; filename: NSString): Boolean; message 'panel:isValidFilename:';
    procedure panel_directoryDidChange(sender: id; path: NSString); message 'panel:directoryDidChange:';
    function panel_shouldShowFilename(sender: id; filename: NSString): Boolean; message 'panel:shouldShowFilename:';
  end;

{ NSDeprecated_NSSavePanelCategory }
  NSDeprecated_NSSavePanelCategory = objccategory external name 'NSDeprecated' (NSSavePanel)
    function filename: NSString; message 'filename';
    function directory: NSString; message 'directory';
    procedure setDirectory(path: NSString); message 'setDirectory:';
    function requiredFileType: NSString; message 'requiredFileType';
    procedure setRequiredFileType(type_: NSString); message 'setRequiredFileType:';
    procedure beginSheetForDirectory_file_modalForWindow_modalDelegate_didEndSelector_contextInfo(path: NSString; name: NSString; docWindow: NSWindow; delegate_: id; didEndSelector: SEL; contextInfo: Pointer); message 'beginSheetForDirectory:file:modalForWindow:modalDelegate:didEndSelector:contextInfo:';
    function runModalForDirectory_file(path: NSString; name: NSString): NSInteger; message 'runModalForDirectory:file:';
    procedure selectText(sender: id); message 'selectText:'; deprecated 'in Mac OS X 10.3 and later';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSSAVEPANEL_PAS_P}
{$define NSSAVEPANEL_PAS_P}
  
{ NSOpenSavePanelDelegate Protocol }
  NSOpenSavePanelDelegateProtocol = objcprotocol external name 'NSOpenSavePanelDelegate'
  optional
    function panel_shouldEnableURL(sender: id; url: NSURL): Boolean; message 'panel:shouldEnableURL:';
    function panel_validateURL_error(sender: id; url: NSURL; outError: NSErrorPointer): Boolean; message 'panel:validateURL:error:';
    procedure panel_didChangeToDirectoryURL(sender: id; url: NSURL); message 'panel:didChangeToDirectoryURL:';
    function panel_userEnteredFilename_confirmed(sender: id; filename: NSString; okFlag: Boolean): NSString; message 'panel:userEnteredFilename:confirmed:';
    procedure panel_willExpand(sender: id; expanding: Boolean); message 'panel:willExpand:';
    procedure panelSelectionDidChange(sender: id); message 'panelSelectionDidChange:';
  end;
{$endif}
{$endif}
