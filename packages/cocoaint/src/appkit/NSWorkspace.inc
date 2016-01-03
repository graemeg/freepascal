{ Parsed from Appkit.framework NSWorkspace.h }

{$ifdef TYPES}
{$ifndef NSWORKSPACE_PAS_T}
{$define NSWORKSPACE_PAS_T}

{ Types }
type
  NSWorkspaceLaunchOptions = NSUInteger;
  NSWorkspaceLaunchOptionsPtr = ^NSWorkspaceLaunchOptions;
  NSWorkspaceIconCreationOptions = NSUInteger;
  NSWorkspaceIconCreationOptionsPtr = ^NSWorkspaceIconCreationOptions;

{ Constants }

const
  NSWorkspaceLaunchAndPrint = $00000002;
  NSWorkspaceLaunchInhibitingBackgroundOnly = $00000080;
  NSWorkspaceLaunchWithoutAddingToRecents = $00000100;
  NSWorkspaceLaunchWithoutActivation = $00000200;
  NSWorkspaceLaunchAsync = $00010000;
  NSWorkspaceLaunchAllowingClassicStartup = $00020000;
  NSWorkspaceLaunchPreferringClassic = $00040000;
  NSWorkspaceLaunchNewInstance = $00080000;
  NSWorkspaceLaunchAndHide = $00100000;
  NSWorkspaceLaunchAndHideOthers = $00200000;
  NSWorkspaceLaunchDefault = NSWorkspaceLaunchAsync or NSWorkspaceLaunchAllowingClassicStartup;

const
  NSExcludeQuickDrawElementsIconCreationOption = 1 shl 1;
  NSExclude10_4ElementsIconCreationOption = 1 shl 2;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSWORKSPACE_PAS_R}
{$define NSWORKSPACE_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSWORKSPACE_PAS_F}
{$define NSWORKSPACE_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSWORKSPACE_PAS_S}
{$define NSWORKSPACE_PAS_S}

{ External string constants }
var
  NSWorkspaceDesktopImageScalingKey: NSString; cvar; external;
  NSWorkspaceDesktopImageAllowClippingKey: NSString; cvar; external;
  NSWorkspaceDesktopImageFillColorKey: NSString; cvar; external;
  NSWorkspaceApplicationKey: NSString; cvar; external;
  NSWorkspaceWillLaunchApplicationNotification: NSString; cvar; external;
  NSWorkspaceDidLaunchApplicationNotification: NSString; cvar; external;
  NSWorkspaceDidTerminateApplicationNotification: NSString; cvar; external;
  NSWorkspaceDidHideApplicationNotification: NSString; cvar; external;
  NSWorkspaceDidUnhideApplicationNotification: NSString; cvar; external;
  NSWorkspaceDidActivateApplicationNotification: NSString; cvar; external;
  NSWorkspaceDidDeactivateApplicationNotification: NSString; cvar; external;
  NSWorkspaceVolumeLocalizedNameKey: NSString; cvar; external;
  NSWorkspaceVolumeURLKey: NSString; cvar; external;
  NSWorkspaceVolumeOldLocalizedNameKey: NSString; cvar; external;
  NSWorkspaceVolumeOldURLKey: NSString; cvar; external;
  NSWorkspaceDidMountNotification: NSString; cvar; external;
  NSWorkspaceDidUnmountNotification: NSString; cvar; external;
  NSWorkspaceWillUnmountNotification: NSString; cvar; external;
  NSWorkspaceDidRenameVolumeNotification: NSString; cvar; external;
  NSWorkspaceWillPowerOffNotification: NSString; cvar; external;
  NSWorkspaceWillSleepNotification: NSString; cvar; external;
  NSWorkspaceDidWakeNotification: NSString; cvar; external;
  NSWorkspaceScreensDidSleepNotification: NSString; cvar; external;
  NSWorkspaceScreensDidWakeNotification: NSString; cvar; external;
  NSWorkspaceSessionDidBecomeActiveNotification: NSString; cvar; external;
  NSWorkspaceSessionDidResignActiveNotification: NSString; cvar; external;
  NSWorkspaceDidPerformFileOperationNotification: NSString; cvar; external;
  NSWorkspaceDidChangeFileLabelsNotification: NSString; cvar; external;
  NSWorkspaceActiveSpaceDidChangeNotification: NSString; cvar; external;
  NSWorkspaceLaunchConfigurationAppleEvent: NSString; cvar; external;
  NSWorkspaceLaunchConfigurationArguments: NSString; cvar; external;
  NSWorkspaceLaunchConfigurationEnvironment: NSString; cvar; external;
  NSWorkspaceLaunchConfigurationArchitecture: NSString; cvar; external;
  NSWorkspaceMoveOperation: NSString; cvar; external;
  NSWorkspaceCopyOperation: NSString; cvar; external;
  NSWorkspaceLinkOperation: NSString; cvar; external;
  NSWorkspaceCompressOperation: NSString; cvar; external;
  NSWorkspaceDecompressOperation: NSString; cvar; external;
  NSWorkspaceEncryptOperation: NSString; cvar; external;
  NSWorkspaceDecryptOperation: NSString; cvar; external;
  NSWorkspaceDestroyOperation: NSString; cvar; external;
  NSWorkspaceRecycleOperation: NSString; cvar; external;
  NSWorkspaceDuplicateOperation: NSString; cvar; external;
  NSPlainFileType: NSString deprecated 'in Mac OS X 10.6 and later'; cvar; external;
  NSDirectoryFileType: NSString deprecated 'in Mac OS X 10.6 and later'; cvar; external;
  NSApplicationFileType: NSString deprecated 'in Mac OS X 10.6 and later'; cvar; external;
  NSFilesystemFileType: NSString deprecated 'in Mac OS X 10.6 and later'; cvar; external;
  NSShellCommandFileType: NSString deprecated 'in Mac OS X 10.6 and later'; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSWorkspace = objcclass;
  NSWorkspacePointer = ^NSWorkspace;
  NSWorkspacePtr = NSWorkspacePointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSWORKSPACE_PAS_C}
{$define NSWORKSPACE_PAS_C}

{ NSWorkspace }
  NSWorkspace = objcclass external (NSObject)
  private
    _notificationCenter: NSNotificationCenter;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef cpu64}
    deviceStatusCount: cint;
    applicationStatusCount: cint;
{$endif cpu64}
=======
    deviceStatusCount: cint;
    applicationStatusCount: cint;
>>>>>>> graemeg/cpstrnew
=======
    deviceStatusCount: cint;
    applicationStatusCount: cint;
>>>>>>> graemeg/cpstrnew
=======
    deviceStatusCount: cint;
    applicationStatusCount: cint;
>>>>>>> graemeg/cpstrnew
=======
    deviceStatusCount: cint;
    applicationStatusCount: cint;
>>>>>>> origin/cpstrnew
    _reservedWorkspace1: id;
    
  public
    class function sharedWorkspace: NSWorkspace; message 'sharedWorkspace';
    function notificationCenter: NSNotificationCenter; message 'notificationCenter';
    function openFile(fullPath: NSString): Boolean; message 'openFile:';
    function openFile_withApplication(fullPath: NSString; appName: NSString): Boolean; message 'openFile:withApplication:';
    function openFile_withApplication_andDeactivate(fullPath: NSString; appName: NSString; flag: Boolean): Boolean; message 'openFile:withApplication:andDeactivate:';
    function openFile_fromImage_at_inView(fullPath: NSString; anImage: NSImage; point: NSPoint; aView: NSView): Boolean; message 'openFile:fromImage:at:inView:';
    function openURL(url: NSURL): Boolean; message 'openURL:';
    function launchApplication(appName: NSString): Boolean; message 'launchApplication:';
    function launchApplicationAtURL_options_configuration_error(url: NSURL; options: NSWorkspaceLaunchOptions; configuration: NSDictionary; error: NSErrorPointer): NSRunningApplication; message 'launchApplicationAtURL:options:configuration:error:';
    function launchApplication_showIcon_autolaunch(appName: NSString; showIcon: Boolean; autolaunch: Boolean): Boolean; message 'launchApplication:showIcon:autolaunch:';
    function fullPathForApplication(appName: NSString): NSString; message 'fullPathForApplication:';
    function selectFile_inFileViewerRootedAtPath(fullPath: NSString; rootFullPath: NSString): Boolean; message 'selectFile:inFileViewerRootedAtPath:';
    procedure activateFileViewerSelectingURLs(fileURLs: NSArray); message 'activateFileViewerSelectingURLs:';
    function showSearchResultsForQueryString(queryString: NSString): Boolean; message 'showSearchResultsForQueryString:';
    procedure noteFileSystemChanged(path: NSString); message 'noteFileSystemChanged:';
    function getInfoForFile_application_type(fullPath: NSString; appName: NSStringPointer; type_: NSStringPointer): Boolean; message 'getInfoForFile:application:type:';
    function isFilePackageAtPath(fullPath: NSString): Boolean; message 'isFilePackageAtPath:';
    function iconForFile(fullPath: NSString): NSImage; message 'iconForFile:';
    function iconForFiles(fullPaths: NSArray): NSImage; message 'iconForFiles:';
    function iconForFileType(fileType: NSString): NSImage; message 'iconForFileType:';
    function setIcon_forFile_options(image: NSImage; fullPath: NSString; options: NSWorkspaceIconCreationOptions): Boolean; message 'setIcon:forFile:options:';
    function fileLabels: NSArray; message 'fileLabels';
    function fileLabelColors: NSArray; message 'fileLabelColors';
    function getFileSystemInfoForPath_isRemovable_isWritable_isUnmountable_description_type(fullPath: NSString; removableFlag: pboolean; writableFlag: pboolean; unmountableFlag: pboolean; description_: NSStringPointer; fileSystemType: NSStringPointer): Boolean; message 'getFileSystemInfoForPath:isRemovable:isWritable:isUnmountable:description:type:';
    function performFileOperation_source_destination_files_tag(operation: NSString; source: NSString; destination: NSString; files: NSArray; tag: NSIntegerPtr): Boolean; message 'performFileOperation:source:destination:files:tag:';
    function unmountAndEjectDeviceAtPath(path: NSString): Boolean; message 'unmountAndEjectDeviceAtPath:';
    function unmountAndEjectDeviceAtURL_error(url: NSURL; error: NSErrorPointer): Boolean; message 'unmountAndEjectDeviceAtURL:error:';
    function extendPowerOffBy(requested: NSInteger): NSInteger; message 'extendPowerOffBy:';
    procedure hideOtherApplications; message 'hideOtherApplications';
    function mountedLocalVolumePaths: NSArray; message 'mountedLocalVolumePaths';
    function mountedRemovableMedia: NSArray; message 'mountedRemovableMedia';
    function URLForApplicationWithBundleIdentifier(bundleIdentifier: NSString): NSURL; message 'URLForApplicationWithBundleIdentifier:';
    function URLForApplicationToOpenURL(url: NSURL): NSURL; message 'URLForApplicationToOpenURL:';
    function absolutePathForAppBundleWithIdentifier(bundleIdentifier: NSString): NSString; message 'absolutePathForAppBundleWithIdentifier:';
    function launchAppWithBundleIdentifier_options_additionalEventParamDescriptor_launchIdentifier(bundleIdentifier: NSString; options: NSWorkspaceLaunchOptions; descriptor: NSAppleEventDescriptor; identifier: NSNumberPointer): Boolean; message 'launchAppWithBundleIdentifier:options:additionalEventParamDescriptor:launchIdentifier:';
    function openURLs_withAppBundleIdentifier_options_additionalEventParamDescriptor_launchIdentifiers(urls: NSArray; bundleIdentifier: NSString; options: NSWorkspaceLaunchOptions; descriptor: NSAppleEventDescriptor; identifiers: NSArrayPointer): Boolean; message 'openURLs:withAppBundleIdentifier:options:additionalEventParamDescriptor:launchIdentifiers:';
    function launchedApplications: NSArray; message 'launchedApplications';
    function activeApplication: NSDictionary; message 'activeApplication';
    function typeOfFile_error(absoluteFilePath: NSString; outError: NSErrorPointer): NSString; message 'typeOfFile:error:';
    function localizedDescriptionForType(typeName: NSString): NSString; message 'localizedDescriptionForType:';
    function preferredFilenameExtensionForType(typeName: NSString): NSString; message 'preferredFilenameExtensionForType:';
    function filenameExtension_isValidForType(filenameExtension: NSString; typeName: NSString): Boolean; message 'filenameExtension:isValidForType:';
    function type_conformsToType(firstTypeName: NSString; secondTypeName: NSString): Boolean; message 'type:conformsToType:';
  end;

{ NSDesktopImagesCategory }
  NSDesktopImagesCategory = objccategory external (NSWorkspace)
    function setDesktopImageURL_forScreen_options_error(url: NSURL; screen: NSScreen; options: NSDictionary; error: NSErrorPointer): Boolean; message 'setDesktopImageURL:forScreen:options:error:';
    function desktopImageURLForScreen(screen: NSScreen): NSURL; message 'desktopImageURLForScreen:';
    function desktopImageOptionsForScreen(screen: NSScreen): NSDictionary; message 'desktopImageOptionsForScreen:';
  end;

{ NSDeprecated_NSWorkspaceCategory }
  NSDeprecated_NSWorkspaceCategory = objccategory external name 'NSDeprecated' (NSWorkspace)
    function openTempFile(fullPath: NSString): Boolean; message 'openTempFile:'; deprecated 'in Mac OS X 10.6 and later';
    procedure findApplications; message 'findApplications'; deprecated 'in Mac OS X 10.6 and later';
    procedure noteUserDefaultsChanged; message 'noteUserDefaultsChanged'; deprecated 'in Mac OS X 10.6 and later';
    procedure slideImage_from_to(image: NSImage; fromPoint: NSPoint; toPoint: NSPoint); message 'slideImage:from:to:'; deprecated 'in Mac OS X 10.6 and later';
    procedure checkForRemovableMedia; message 'checkForRemovableMedia'; deprecated 'in Mac OS X 10.6 and later';
    function fileSystemChanged: Boolean; message 'fileSystemChanged'; deprecated 'in Mac OS X 10.6 and later';
    function userDefaultsChanged: Boolean; message 'userDefaultsChanged'; deprecated 'in Mac OS X 10.6 and later';
    function mountNewRemovableMedia: NSArray; message 'mountNewRemovableMedia'; deprecated 'in Mac OS X 10.6 and later';
  end;

{$endif}
{$endif}
