{ Version FrameworkParser: 1.3. PasCocoa 0.3, Objective-P 0.2 - Tue Sep 8 9:10:40 ICT 2009 }

unit NSDelegatesAll;
interface

{ Copy and paste these delegate methods into your real classes. }

type
  NSAlertDelegate = objccategory
    function alertShowHelp(alert: NSAlert): Boolean; message 'alertShowHelp:';
  end;

type
  NSAnimationDelegate = objccategory
    procedure animation_didReachProgressMark(animation: NSAnimation; progress: NSAnimationProgress); message 'animation:didReachProgressMark:';
    function animation_valueForProgress(animation: NSAnimation; progress: NSAnimationProgress): single; message 'animation:valueForProgress:';
    procedure animationDidEnd(animation: NSAnimation); message 'animationDidEnd:';
    procedure animationDidStop(animation: NSAnimation); message 'animationDidStop:';
    function animationShouldStart(animation: NSAnimation): Boolean; message 'animationShouldStart:';
  end;

type
  NSApplicationDelegate = objccategory
    function application_openFile(sender: NSApplication; filename: NSString): Boolean; message 'application:openFile:';
    function application_openFileWithoutUI(sender: id; filename: NSString): Boolean; message 'application:openFileWithoutUI:';
    procedure application_openFiles(sender: NSApplication; filenames: NSArray); message 'application:openFiles:';
    function application_openTempFile(sender: NSApplication; filename: NSString): Boolean; message 'application:openTempFile:';
    function application_printFile(sender: NSApplication; filename: NSString): Boolean; message 'application:printFile:';
    procedure application_printFiles(sender: NSApplication; filenames: NSArray); message 'application:printFiles:';
    function application_printFiles_withSettings_showPrintPanels(application: NSApplication; fileNames: NSArray; printSettings: NSDictionary; showPrintPanels: Boolean): NSApplicationPrintReply; message 'application:printFiles:withSettings:showPrintPanels:';
    function application_willPresentError(application: NSApplication; error: NSError): NSError; message 'application:willPresentError:';
    function applicationDockMenu(sender: NSApplication): NSMenu; message 'applicationDockMenu:';
    function applicationOpenUntitledFile(sender: NSApplication): Boolean; message 'applicationOpenUntitledFile:';
    function applicationShouldHandleReopen_hasVisibleWindows(sender: NSApplication; flag: Boolean): Boolean; message 'applicationShouldHandleReopen:hasVisibleWindows:';
    function applicationShouldOpenUntitledFile(sender: NSApplication): Boolean; message 'applicationShouldOpenUntitledFile:';
    function applicationShouldTerminate(sender: NSApplication): NSApplicationTerminateReply; message 'applicationShouldTerminate:';
    function applicationShouldTerminateAfterLastWindowClosed(sender: NSApplication): Boolean; message 'applicationShouldTerminateAfterLastWindowClosed:';
  end;

type
  NSApplicationNotifications = objccategory
    procedure applicationDidBecomeActive(notification: NSNotification); message 'applicationDidBecomeActive:';
    procedure applicationDidChangeScreenParameters(notification: NSNotification); message 'applicationDidChangeScreenParameters:';
    procedure applicationDidFinishLaunching(notification: NSNotification); message 'applicationDidFinishLaunching:';
    procedure applicationDidHide(notification: NSNotification); message 'applicationDidHide:';
    procedure applicationDidResignActive(notification: NSNotification); message 'applicationDidResignActive:';
    procedure applicationDidUnhide(notification: NSNotification); message 'applicationDidUnhide:';
    procedure applicationDidUpdate(notification: NSNotification); message 'applicationDidUpdate:';
    procedure applicationWillBecomeActive(notification: NSNotification); message 'applicationWillBecomeActive:';
    procedure applicationWillFinishLaunching(notification: NSNotification); message 'applicationWillFinishLaunching:';
    procedure applicationWillHide(notification: NSNotification); message 'applicationWillHide:';
    procedure applicationWillResignActive(notification: NSNotification); message 'applicationWillResignActive:';
    procedure applicationWillTerminate(notification: NSNotification); message 'applicationWillTerminate:';
    procedure applicationWillUnhide(notification: NSNotification); message 'applicationWillUnhide:';
    procedure applicationWillUpdate(notification: NSNotification); message 'applicationWillUpdate:';
  end;

type
  NSApplicationScriptingDelegation = objccategory
    function application_delegateHandlesKey(sender: NSApplication; key: NSString): Boolean; message 'application:delegateHandlesKey:';
  end;

type
  NSBrowserDelegate = objccategory
    function browser_acceptDrop_atRow_column_dropOperation(browser: NSBrowser; info: id; row: clong; column: clong; dropOperation: NSBrowserDropOperation): Boolean; message 'browser:acceptDrop:atRow:column:dropOperation:';
    function browser_canDragRowsWithIndexes_inColumn_withEvent(browser: NSBrowser; rowIndexes: NSIndexSet; column: clong; event: NSEvent): Boolean; message 'browser:canDragRowsWithIndexes:inColumn:withEvent:';
    procedure browser_createRowsForColumn_inMatrix(sender: NSBrowser; column: clong; matrix: NSMatrix); message 'browser:createRowsForColumn:inMatrix:';
    function browser_draggingImageForRowsWithIndexes_inColumn_withEvent_offset(browser: NSBrowser; rowIndexes: NSIndexSet; column: clong; event: NSEvent; dragImageOffset: NSPointPointer): NSImage; message 'browser:draggingImageForRowsWithIndexes:inColumn:withEvent:offset:';
    function browser_isColumnValid(sender: NSBrowser; column: clong): Boolean; message 'browser:isColumnValid:';
    function browser_namesOfPromisedFilesDroppedAtDestination_forDraggedRowsWithIndexes_inColumn(browser: NSBrowser; dropDestination: NSURL; rowIndexes: NSIndexSet; column: clong): NSArray; message 'browser:namesOfPromisedFilesDroppedAtDestination:forDraggedRowsWithIndexes:inColumn:';
    function browser_nextTypeSelectMatchFromRow_toRow_inColumn_forString(browser: NSBrowser; startRow: clong; endRow: clong; column: clong; searchString: NSString): clong; message 'browser:nextTypeSelectMatchFromRow:toRow:inColumn:forString:';
    function browser_numberOfRowsInColumn(sender: NSBrowser; column: clong): clong; message 'browser:numberOfRowsInColumn:';
    function browser_selectCellWithString_inColumn(sender: NSBrowser; title: NSString; column: clong): Boolean; message 'browser:selectCellWithString:inColumn:';
    function browser_selectRow_inColumn(sender: NSBrowser; row: clong; column: clong): Boolean; message 'browser:selectRow:inColumn:';
    function browser_shouldShowCellExpansionForRow_column(browser: NSBrowser; row: clong; column: clong): Boolean; message 'browser:shouldShowCellExpansionForRow:column:';
    function browser_shouldSizeColumn_forUserResize_toWidth(browser: NSBrowser; columnIndex: clong; forUserResize: Boolean; suggestedWidth: CGFloat): CGFloat; message 'browser:shouldSizeColumn:forUserResize:toWidth:';
    function browser_shouldTypeSelectForEvent_withCurrentSearchString(browser: NSBrowser; event: NSEvent; searchString: NSString): Boolean; message 'browser:shouldTypeSelectForEvent:withCurrentSearchString:';
    function browser_sizeToFitWidthOfColumn(browser: NSBrowser; columnIndex: clong): CGFloat; message 'browser:sizeToFitWidthOfColumn:';
    function browser_titleOfColumn(sender: NSBrowser; column: clong): NSString; message 'browser:titleOfColumn:';
    function browser_typeSelectStringForRow_inColumn(browser: NSBrowser; row: clong; column: clong): NSString; message 'browser:typeSelectStringForRow:inColumn:';
    function browser_validateDrop_proposedRow_column_dropOperation(browser: NSBrowser; info: id; row: clong; column: clong; dropOperation: NSBrowserDropOperation): NSDragOperation; message 'browser:validateDrop:proposedRow:column:dropOperation:';
    procedure browser_willDisplayCell_atRow_column(sender: NSBrowser; cell_: id; row: clong; column: clong); message 'browser:willDisplayCell:atRow:column:';
    function browser_writeRowsWithIndexes_inColumn_toPasteboard(browser: NSBrowser; rowIndexes: NSIndexSet; column: clong; pasteboard: NSPasteboard): Boolean; message 'browser:writeRowsWithIndexes:inColumn:toPasteboard:';
    procedure browserColumnConfigurationDidChange(notification: NSNotification); message 'browserColumnConfigurationDidChange:';
    procedure browserDidScroll(sender: NSBrowser); message 'browserDidScroll:';
    procedure browserWillScroll(sender: NSBrowser); message 'browserWillScroll:';
  end;

type
  NSComboBoxCellDataSource = objccategory
    function comboBoxCell_completedString(aComboBoxCell: NSComboBoxCell; uncompletedString: NSString): NSString; message 'comboBoxCell:completedString:';
    function comboBoxCell_indexOfItemWithStringValue(aComboBoxCell: NSComboBoxCell; string_: NSString): culong; message 'comboBoxCell:indexOfItemWithStringValue:';
    function comboBoxCell_objectValueForItemAtIndex(aComboBoxCell: NSComboBoxCell; index: clong): id; message 'comboBoxCell:objectValueForItemAtIndex:';
    function numberOfItemsInComboBoxCell(comboBoxCell: NSComboBoxCell): clong; message 'numberOfItemsInComboBoxCell:';
  end;

type
  NSComboBoxDataSource = objccategory
    function comboBox_completedString(aComboBox: NSComboBox; string_: NSString): NSString; message 'comboBox:completedString:';
    function comboBox_indexOfItemWithStringValue(aComboBox: NSComboBox; string_: NSString): culong; message 'comboBox:indexOfItemWithStringValue:';
    function comboBox_objectValueForItemAtIndex(aComboBox: NSComboBox; index: clong): id; message 'comboBox:objectValueForItemAtIndex:';
    function numberOfItemsInComboBox(aComboBox: NSComboBox): clong; message 'numberOfItemsInComboBox:';
  end;

type
  NSComboBoxNotifications = objccategory
    procedure comboBoxSelectionDidChange(notification: NSNotification); message 'comboBoxSelectionDidChange:';
    procedure comboBoxSelectionIsChanging(notification: NSNotification); message 'comboBoxSelectionIsChanging:';
    procedure comboBoxWillDismiss(notification: NSNotification); message 'comboBoxWillDismiss:';
    procedure comboBoxWillPopUp(notification: NSNotification); message 'comboBoxWillPopUp:';
  end;

type
  NSConnectionDelegateMethods = objccategory
    function authenticateComponents_withData(components: NSArray; signature: NSData): Boolean; message 'authenticateComponents:withData:';
    function authenticationDataForComponents(components: NSArray): NSData; message 'authenticationDataForComponents:';
    function connection_shouldMakeNewConnection(ancestor: NSConnection; conn: NSConnection): Boolean; message 'connection:shouldMakeNewConnection:';
    function createConversationForConnection(conn: NSConnection): id; message 'createConversationForConnection:';
    function makeNewConnection_sender(conn: NSConnection; ancestor: NSConnection): Boolean; message 'makeNewConnection:sender:';
  end;

type
  NSControlSubclassDelegate = objccategory
    function control_didFailToFormatString_errorDescription(control: NSControl; string_: NSString; error: NSString): Boolean; message 'control:didFailToFormatString:errorDescription:';
    procedure control_didFailToValidatePartialString_errorDescription(control: NSControl; string_: NSString; error: NSString); message 'control:didFailToValidatePartialString:errorDescription:';
    function control_isValidObject(control: NSControl; obj: id): Boolean; message 'control:isValidObject:';
    function control_textShouldBeginEditing(control: NSControl; fieldEditor: NSText): Boolean; message 'control:textShouldBeginEditing:';
    function control_textShouldEndEditing(control: NSControl; fieldEditor: NSText): Boolean; message 'control:textShouldEndEditing:';
    function control_textView_completions_forPartialWordRange_indexOfSelectedItem(control: NSControl; textView: NSTextView; words: NSArray; charRange: NSRange; index: clong): NSArray; message 'control:textView:completions:forPartialWordRange:indexOfSelectedItem:';
    function control_textView_doCommandBySelector(control: NSControl; textView: NSTextView; commandSelector: SEL): Boolean; message 'control:textView:doCommandBySelector:';
  end;

type
  NSControlSubclassNotifications = objccategory
    procedure controlTextDidBeginEditing(obj: NSNotification); message 'controlTextDidBeginEditing:';
    procedure controlTextDidChange(obj: NSNotification); message 'controlTextDidChange:';
    procedure controlTextDidEndEditing(obj: NSNotification); message 'controlTextDidEndEditing:';
  end;

type
  NSCopyLinkMoveHandler = objccategory
    function fileManager_shouldProceedAfterError(fm: NSFileManager; errorInfo: NSDictionary): Boolean; message 'fileManager:shouldProceedAfterError:';
    procedure fileManager_willProcessPath(fm: NSFileManager; path: NSString); message 'fileManager:willProcessPath:';
  end;

type
  NSDatePickerCellDelegate = objccategory
    procedure datePickerCell_validateProposedDateValue_timeInterval(aDatePickerCell: NSDatePickerCell; proposedDateValue: NSDate; proposedTimeInterval: NSTimeInterval); message 'datePickerCell:validateProposedDateValue:timeInterval:';
  end;

type
  NSDistantObjectRequestMethods = objccategory
    function connection_handleRequest(connection_: NSConnection; doreq: NSDistantObjectRequest): Boolean; message 'connection:handleRequest:';
  end;

type
  NSDraggingDestination = objccategory
    procedure concludeDragOperation(sender: id); message 'concludeDragOperation:';
    procedure draggingEnded(sender: id); message 'draggingEnded:';
    function draggingEntered(sender: id): NSDragOperation; message 'draggingEntered:';
    procedure draggingExited(sender: id); message 'draggingExited:';
    function draggingUpdated(sender: id): NSDragOperation; message 'draggingUpdated:';
    function performDragOperation(sender: id): Boolean; message 'performDragOperation:';
    function prepareForDragOperation(sender: id): Boolean; message 'prepareForDragOperation:';
    function wantsPeriodicDraggingUpdates: Boolean; message 'wantsPeriodicDraggingUpdates';
  end;

type
  NSDraggingSource = objccategory
    procedure draggedImage_beganAt(image: NSImage; screenPoint: NSPoint); message 'draggedImage:beganAt:';
    procedure draggedImage_endedAt_deposited(image: NSImage; screenPoint: NSPoint; flag: Boolean); message 'draggedImage:endedAt:deposited:';
    procedure draggedImage_endedAt_operation(image: NSImage; screenPoint: NSPoint; operation: NSDragOperation); message 'draggedImage:endedAt:operation:';
    procedure draggedImage_movedTo(image: NSImage; screenPoint: NSPoint); message 'draggedImage:movedTo:';
    function draggingSourceOperationMaskForLocal(flag: Boolean): NSDragOperation; message 'draggingSourceOperationMaskForLocal:';
    function ignoreModifierKeysWhileDragging: Boolean; message 'ignoreModifierKeysWhileDragging';
    function namesOfPromisedFilesDroppedAtDestination(dropDestination: NSURL): NSArray; message 'namesOfPromisedFilesDroppedAtDestination:';
  end;

type
  NSDrawerDelegate = objccategory
    function drawerShouldClose(sender: NSDrawer): Boolean; message 'drawerShouldClose:';
    function drawerShouldOpen(sender: NSDrawer): Boolean; message 'drawerShouldOpen:';
    function drawerWillResizeContents_toSize(sender: NSDrawer; contentSize: NSSize): NSSize; message 'drawerWillResizeContents:toSize:';
  end;

type
  NSDrawerNotifications = objccategory
    procedure drawerDidClose(notification: NSNotification); message 'drawerDidClose:';
    procedure drawerDidOpen(notification: NSNotification); message 'drawerDidOpen:';
    procedure drawerWillClose(notification: NSNotification); message 'drawerWillClose:';
    procedure drawerWillOpen(notification: NSNotification); message 'drawerWillOpen:';
  end;

type
  NSEditorRegistration = objccategory
    procedure objectDidBeginEditing(editor: id); message 'objectDidBeginEditing:';
    procedure objectDidEndEditing(editor: id); message 'objectDidEndEditing:';
  end;

type
  NSFileManagerFileOperationAdditions = objccategory
    function fileManager_shouldCopyItemAtPath_toPath(fileManager: NSFileManager; srcPath: NSString; dstPath: NSString): Boolean; message 'fileManager:shouldCopyItemAtPath:toPath:';
    function fileManager_shouldLinkItemAtPath_toPath(fileManager: NSFileManager; srcPath: NSString; dstPath: NSString): Boolean; message 'fileManager:shouldLinkItemAtPath:toPath:';
    function fileManager_shouldMoveItemAtPath_toPath(fileManager: NSFileManager; srcPath: NSString; dstPath: NSString): Boolean; message 'fileManager:shouldMoveItemAtPath:toPath:';
    function fileManager_shouldProceedAfterError_copyingItemAtPath_toPath(fileManager: NSFileManager; error: NSError; srcPath: NSString; dstPath: NSString): Boolean; message 'fileManager:shouldProceedAfterError:copyingItemAtPath:toPath:';
    function fileManager_shouldProceedAfterError_linkingItemAtPath_toPath(fileManager: NSFileManager; error: NSError; srcPath: NSString; dstPath: NSString): Boolean; message 'fileManager:shouldProceedAfterError:linkingItemAtPath:toPath:';
    function fileManager_shouldProceedAfterError_movingItemAtPath_toPath(fileManager: NSFileManager; error: NSError; srcPath: NSString; dstPath: NSString): Boolean; message 'fileManager:shouldProceedAfterError:movingItemAtPath:toPath:';
    function fileManager_shouldProceedAfterError_removingItemAtPath(fileManager: NSFileManager; error: NSError; path: NSString): Boolean; message 'fileManager:shouldProceedAfterError:removingItemAtPath:';
    function fileManager_shouldRemoveItemAtPath(fileManager: NSFileManager; path: NSString): Boolean; message 'fileManager:shouldRemoveItemAtPath:';
  end;

type
  NSFontManagerDelegate = objccategory
    function fontManager_willIncludeFont(sender: id; fontName: NSString): Boolean; message 'fontManager:willIncludeFont:';
  end;

type
  NSImageDelegate = objccategory
    procedure image_didLoadPartOfRepresentation_withValidRows(image: NSImage; rep: NSImageRep; rows: clong); message 'image:didLoadPartOfRepresentation:withValidRows:';
    procedure image_didLoadRepresentation_withStatus(image: NSImage; rep: NSImageRep; status: NSImageLoadStatus); message 'image:didLoadRepresentation:withStatus:';
    procedure image_didLoadRepresentationHeader(image: NSImage; rep: NSImageRep); message 'image:didLoadRepresentationHeader:';
    procedure image_willLoadRepresentation(image: NSImage; rep: NSImageRep); message 'image:willLoadRepresentation:';
    function imageDidNotDraw_inRect(sender: id; aRect: NSRect): NSImage; message 'imageDidNotDraw:inRect:';
  end;

type
  NSKeyValueObserverNotification = objccategory
    procedure didChange_valuesAtIndexes_forKey(changeKind: NSKeyValueChange; indexes: NSIndexSet; key: NSString); message 'didChange:valuesAtIndexes:forKey:';
    procedure didChangeValueForKey(key: NSString); message 'didChangeValueForKey:';
    procedure didChangeValueForKey_withSetMutation_usingObjects(key: NSString; mutationKind: NSKeyValueSetMutationKind; objects: NSSet); message 'didChangeValueForKey:withSetMutation:usingObjects:';
    procedure willChange_valuesAtIndexes_forKey(changeKind: NSKeyValueChange; indexes: NSIndexSet; key: NSString); message 'willChange:valuesAtIndexes:forKey:';
    procedure willChangeValueForKey(key: NSString); message 'willChangeValueForKey:';
    procedure willChangeValueForKey_withSetMutation_usingObjects(key: NSString; mutationKind: NSKeyValueSetMutationKind; objects: NSSet); message 'willChangeValueForKey:withSetMutation:usingObjects:';
  end;

type
  NSKeyedArchiverDelegate = objccategory
    procedure archiver_didEncodeObject(archiver: NSKeyedArchiver; object_: id); message 'archiver:didEncodeObject:';
    function archiver_willEncodeObject(archiver: NSKeyedArchiver; object_: id): id; message 'archiver:willEncodeObject:';
    procedure archiver_willReplaceObject_withObject(archiver: NSKeyedArchiver; object_: id; newObject: id); message 'archiver:willReplaceObject:withObject:';
    procedure archiverDidFinish(archiver: NSKeyedArchiver); message 'archiverDidFinish:';
    procedure archiverWillFinish(archiver: NSKeyedArchiver); message 'archiverWillFinish:';
  end;

type
  NSKeyedUnarchiverDelegate = objccategory
    function unarchiver_cannotDecodeObjectOfClassName_originalClasses(unarchiver: NSKeyedUnarchiver; name: NSString; classNames: NSArray): Pobjc_class; message 'unarchiver:cannotDecodeObjectOfClassName:originalClasses:';
    function unarchiver_didDecodeObject(unarchiver: NSKeyedUnarchiver; object_: id): id; message 'unarchiver:didDecodeObject:';
    procedure unarchiver_willReplaceObject_withObject(unarchiver: NSKeyedUnarchiver; object_: id; newObject: id); message 'unarchiver:willReplaceObject:withObject:';
    procedure unarchiverDidFinish(unarchiver: NSKeyedUnarchiver); message 'unarchiverDidFinish:';
    procedure unarchiverWillFinish(unarchiver: NSKeyedUnarchiver); message 'unarchiverWillFinish:';
  end;

type
  NSLayoutManagerDelegate = objccategory
    procedure layoutManager_didCompleteLayoutForTextContainer_atEnd(layoutManager: NSLayoutManager; textContainer: NSTextContainer; layoutFinishedFlag: Boolean); message 'layoutManager:didCompleteLayoutForTextContainer:atEnd:';
    function layoutManager_shouldUseTemporaryAttributes_forDrawingToScreen_atCharacterIndex_effectiveRange(layoutManager: NSLayoutManager; attrs: NSDictionary; toScreen: Boolean; charIndex: culong; effectiveCharRange: NSRangePointer): NSDictionary; message 'layoutManager:shouldUseTemporaryAttributes:forDrawingToScreen:atCharacterIndex:effectiveRange:';
    procedure layoutManagerDidInvalidateLayout(sender: NSLayoutManager); message 'layoutManagerDidInvalidateLayout:';
  end;

type
  NSMachPortDelegateMethods = objccategory
    procedure handleMachMessage(msg: Pointer); message 'handleMachMessage:';
  end;

type
  NSMenuDelegate = objccategory
    function menu_updateItem_atIndex_shouldCancel(menu: NSMenu; item: NSMenuItem; index: clong; shouldCancel: Boolean): Boolean; message 'menu:updateItem:atIndex:shouldCancel:';
    procedure menu_willHighlightItem(menu: NSMenu; item: NSMenuItem); message 'menu:willHighlightItem:';
    procedure menuDidClose(menu: NSMenu); message 'menuDidClose:';
    function menuHasKeyEquivalent_forEvent_target_action(menu: NSMenu; event: NSEvent; target: id; action: SEL): Boolean; message 'menuHasKeyEquivalent:forEvent:target:action:';
    procedure menuNeedsUpdate(menu: NSMenu); message 'menuNeedsUpdate:';
    procedure menuWillOpen(menu: NSMenu); message 'menuWillOpen:';
    function numberOfItemsInMenu(menu: NSMenu): clong; message 'numberOfItemsInMenu:';
  end;

type
  NSMetadataQueryDelegate = objccategory
    function metadataQuery_replacementObjectForResultObject(query: NSMetadataQuery; result_: NSMetadataItem): id; message 'metadataQuery:replacementObjectForResultObject:';
    function metadataQuery_replacementValueForAttribute_value(query: NSMetadataQuery; attrName: NSString; attrValue: id): id; message 'metadataQuery:replacementValueForAttribute:value:';
  end;

type
  NSNetServiceBrowserDelegateMethods = objccategory
    procedure netServiceBrowser_didFindDomain_moreComing(aNetServiceBrowser: NSNetServiceBrowser; domainString: NSString; moreComing: Boolean); message 'netServiceBrowser:didFindDomain:moreComing:';
    procedure netServiceBrowser_didFindService_moreComing(aNetServiceBrowser: NSNetServiceBrowser; aNetService: NSNetService; moreComing: Boolean); message 'netServiceBrowser:didFindService:moreComing:';
    procedure netServiceBrowser_didNotSearch(aNetServiceBrowser: NSNetServiceBrowser; errorDict: NSDictionary); message 'netServiceBrowser:didNotSearch:';
    procedure netServiceBrowser_didRemoveDomain_moreComing(aNetServiceBrowser: NSNetServiceBrowser; domainString: NSString; moreComing: Boolean); message 'netServiceBrowser:didRemoveDomain:moreComing:';
    procedure netServiceBrowser_didRemoveService_moreComing(aNetServiceBrowser: NSNetServiceBrowser; aNetService: NSNetService; moreComing: Boolean); message 'netServiceBrowser:didRemoveService:moreComing:';
    procedure netServiceBrowserDidStopSearch(aNetServiceBrowser: NSNetServiceBrowser); message 'netServiceBrowserDidStopSearch:';
    procedure netServiceBrowserWillSearch(aNetServiceBrowser: NSNetServiceBrowser); message 'netServiceBrowserWillSearch:';
  end;

type
  NSNetServiceDelegateMethods = objccategory
    procedure netService_didNotPublish(sender: NSNetService; errorDict: NSDictionary); message 'netService:didNotPublish:';
    procedure netService_didNotResolve(sender: NSNetService; errorDict: NSDictionary); message 'netService:didNotResolve:';
    procedure netService_didUpdateTXTRecordData(sender: NSNetService; data: NSData); message 'netService:didUpdateTXTRecordData:';
    procedure netServiceDidPublish(sender: NSNetService); message 'netServiceDidPublish:';
    procedure netServiceDidResolveAddress(sender: NSNetService); message 'netServiceDidResolveAddress:';
    procedure netServiceDidStop(sender: NSNetService); message 'netServiceDidStop:';
    procedure netServiceWillPublish(sender: NSNetService); message 'netServiceWillPublish:';
    procedure netServiceWillResolve(sender: NSNetService); message 'netServiceWillResolve:';
  end;

type
  NSOutlineViewDataSource = objccategory
    function outlineView_acceptDrop_item_childIndex(outlineView: NSOutlineView; info: id; item: id; index: clong): Boolean; message 'outlineView:acceptDrop:item:childIndex:';
    function outlineView_child_ofItem(outlineView: NSOutlineView; index: clong; item: id): id; message 'outlineView:child:ofItem:';
    function outlineView_isItemExpandable(outlineView: NSOutlineView; item: id): Boolean; message 'outlineView:isItemExpandable:';
    function outlineView_itemForPersistentObject(outlineView: NSOutlineView; object_: id): id; message 'outlineView:itemForPersistentObject:';
    function outlineView_namesOfPromisedFilesDroppedAtDestination_forDraggedItems(outlineView: NSOutlineView; dropDestination: NSURL; items: NSArray): NSArray; message 'outlineView:namesOfPromisedFilesDroppedAtDestination:forDraggedItems:';
    function outlineView_numberOfChildrenOfItem(outlineView: NSOutlineView; item: id): clong; message 'outlineView:numberOfChildrenOfItem:';
    function outlineView_objectValueForTableColumn_byItem(outlineView: NSOutlineView; tableColumn: NSTableColumn; item: id): id; message 'outlineView:objectValueForTableColumn:byItem:';
    function outlineView_persistentObjectForItem(outlineView: NSOutlineView; item: id): id; message 'outlineView:persistentObjectForItem:';
    procedure outlineView_setObjectValue_forTableColumn_byItem(outlineView: NSOutlineView; object_: id; tableColumn: NSTableColumn; item: id); message 'outlineView:setObjectValue:forTableColumn:byItem:';
    procedure outlineView_sortDescriptorsDidChange(outlineView: NSOutlineView; oldDescriptors: NSArray); message 'outlineView:sortDescriptorsDidChange:';
    function outlineView_validateDrop_proposedItem_proposedChildIndex(outlineView: NSOutlineView; info: id; item: id; index: clong): NSDragOperation; message 'outlineView:validateDrop:proposedItem:proposedChildIndex:';
    function outlineView_writeItems_toPasteboard(outlineView: NSOutlineView; items: NSArray; pasteboard: NSPasteboard): Boolean; message 'outlineView:writeItems:toPasteboard:';
  end;

type
  NSOutlineViewDelegate = objccategory
    function outlineView_dataCellForTableColumn_item(outlineView: NSOutlineView; tableColumn: NSTableColumn; item: id): NSCell; message 'outlineView:dataCellForTableColumn:item:';
    procedure outlineView_didClickTableColumn(outlineView: NSOutlineView; tableColumn: NSTableColumn); message 'outlineView:didClickTableColumn:';
    procedure outlineView_didDragTableColumn(outlineView: NSOutlineView; tableColumn: NSTableColumn); message 'outlineView:didDragTableColumn:';
    function outlineView_heightOfRowByItem(outlineView: NSOutlineView; item: id): CGFloat; message 'outlineView:heightOfRowByItem:';
    function outlineView_isGroupItem(outlineView: NSOutlineView; item: id): Boolean; message 'outlineView:isGroupItem:';
    procedure outlineView_mouseDownInHeaderOfTableColumn(outlineView: NSOutlineView; tableColumn: NSTableColumn); message 'outlineView:mouseDownInHeaderOfTableColumn:';
    function outlineView_nextTypeSelectMatchFromItem_toItem_forString(outlineView: NSOutlineView; startItem: id; endItem: id; searchString: NSString): id; message 'outlineView:nextTypeSelectMatchFromItem:toItem:forString:';
    function outlineView_selectionIndexesForProposedSelection(outlineView: NSOutlineView; proposedSelectionIndexes: NSIndexSet): NSIndexSet; message 'outlineView:selectionIndexesForProposedSelection:';
    function outlineView_shouldCollapseItem(outlineView: NSOutlineView; item: id): Boolean; message 'outlineView:shouldCollapseItem:';
    function outlineView_shouldEditTableColumn_item(outlineView: NSOutlineView; tableColumn: NSTableColumn; item: id): Boolean; message 'outlineView:shouldEditTableColumn:item:';
    function outlineView_shouldExpandItem(outlineView: NSOutlineView; item: id): Boolean; message 'outlineView:shouldExpandItem:';
    function outlineView_shouldSelectItem(outlineView: NSOutlineView; item: id): Boolean; message 'outlineView:shouldSelectItem:';
    function outlineView_shouldSelectTableColumn(outlineView: NSOutlineView; tableColumn: NSTableColumn): Boolean; message 'outlineView:shouldSelectTableColumn:';
    function outlineView_shouldShowCellExpansionForTableColumn_item(outlineView: NSOutlineView; tableColumn: NSTableColumn; item: id): Boolean; message 'outlineView:shouldShowCellExpansionForTableColumn:item:';
    function outlineView_shouldTrackCell_forTableColumn_item(outlineView: NSOutlineView; cell_: NSCell; tableColumn: NSTableColumn; item: id): Boolean; message 'outlineView:shouldTrackCell:forTableColumn:item:';
    function outlineView_shouldTypeSelectForEvent_withCurrentSearchString(outlineView: NSOutlineView; event: NSEvent; searchString: NSString): Boolean; message 'outlineView:shouldTypeSelectForEvent:withCurrentSearchString:';
    function outlineView_toolTipForCell_rect_tableColumn_item_mouseLocation(outlineView: NSOutlineView; cell_: NSCell; rect: NSRectPointer; tableColumn: NSTableColumn; item: id; mouseLocation: NSPoint): NSString; message 'outlineView:toolTipForCell:rect:tableColumn:item:mouseLocation:';
    function outlineView_typeSelectStringForTableColumn_item(outlineView: NSOutlineView; tableColumn: NSTableColumn; item: id): NSString; message 'outlineView:typeSelectStringForTableColumn:item:';
    procedure outlineView_willDisplayCell_forTableColumn_item(outlineView: NSOutlineView; cell_: id; tableColumn: NSTableColumn; item: id); message 'outlineView:willDisplayCell:forTableColumn:item:';
    procedure outlineView_willDisplayOutlineCell_forTableColumn_item(outlineView: NSOutlineView; cell_: id; tableColumn: NSTableColumn; item: id); message 'outlineView:willDisplayOutlineCell:forTableColumn:item:';
    function selectionShouldChangeInOutlineView(outlineView: NSOutlineView): Boolean; message 'selectionShouldChangeInOutlineView:';
  end;

type
  NSOutlineViewNotifications = objccategory
    procedure outlineViewColumnDidMove(notification: NSNotification); message 'outlineViewColumnDidMove:';
    procedure outlineViewColumnDidResize(notification: NSNotification); message 'outlineViewColumnDidResize:';
    procedure outlineViewItemDidCollapse(notification: NSNotification); message 'outlineViewItemDidCollapse:';
    procedure outlineViewItemDidExpand(notification: NSNotification); message 'outlineViewItemDidExpand:';
    procedure outlineViewItemWillCollapse(notification: NSNotification); message 'outlineViewItemWillCollapse:';
    procedure outlineViewItemWillExpand(notification: NSNotification); message 'outlineViewItemWillExpand:';
    procedure outlineViewSelectionDidChange(notification: NSNotification); message 'outlineViewSelectionDidChange:';
    procedure outlineViewSelectionIsChanging(notification: NSNotification); message 'outlineViewSelectionIsChanging:';
  end;

type
  NSPasteboardOwner = objccategory
    procedure pasteboard_provideDataForType(sender: NSPasteboard; type_: NSString); message 'pasteboard:provideDataForType:';
    procedure pasteboardChangedOwner(sender: NSPasteboard); message 'pasteboardChangedOwner:';
  end;

type
  NSPortDelegateMethods = objccategory
    procedure handlePortMessage(message: NSPortMessage); message 'handlePortMessage:';
  end;

type
  NSRuleEditorDelegateMethods = objccategory
    function ruleEditor_child_forCriterion_withRowType(editor: NSRuleEditor; index: clong; criterion: id; rowType: NSRuleEditorRowType): id; message 'ruleEditor:child:forCriterion:withRowType:';
    function ruleEditor_displayValueForCriterion_inRow(editor: NSRuleEditor; criterion: id; row: clong): id; message 'ruleEditor:displayValueForCriterion:inRow:';
    function ruleEditor_numberOfChildrenForCriterion_withRowType(editor: NSRuleEditor; criterion: id; rowType: NSRuleEditorRowType): clong; message 'ruleEditor:numberOfChildrenForCriterion:withRowType:';
    function ruleEditor_predicatePartsForCriterion_withDisplayValue_inRow(editor: NSRuleEditor; criterion: id; value: id; row: clong): NSDictionary; message 'ruleEditor:predicatePartsForCriterion:withDisplayValue:inRow:';
    procedure ruleEditorRowsDidChange(notification: NSNotification); message 'ruleEditorRowsDidChange:';
  end;

type
  NSSavePanelDelegate = objccategory
    function panel_compareFilename_with_caseSensitive(sender: id; name: NSString; name1: NSString; caseSensitive: Boolean): NSComparisonResult; message 'panel:compareFilename:with:caseSensitive:';
    procedure panel_directoryDidChange(sender: id; path: NSString); message 'panel:directoryDidChange:';
    function panel_isValidFilename(sender: id; filename_: NSString): Boolean; message 'panel:isValidFilename:';
    function panel_shouldShowFilename(sender: id; filename_: NSString): Boolean; message 'panel:shouldShowFilename:';
    function panel_userEnteredFilename_confirmed(sender: id; filename_: NSString; okFlag: Boolean): NSString; message 'panel:userEnteredFilename:confirmed:';
    procedure panel_willExpand(sender: id; expanding: Boolean); message 'panel:willExpand:';
    procedure panelSelectionDidChange(sender: id); message 'panelSelectionDidChange:';
  end;

type
  NSSoundDelegateMethods = objccategory
    procedure sound_didFinishPlaying(sound: NSSound; aBool: Boolean); message 'sound:didFinishPlaying:';
  end;

type
  NSSpeechRecognizerDelegate = objccategory
    procedure speechRecognizer_didRecognizeCommand(sender: NSSpeechRecognizer; command: id); message 'speechRecognizer:didRecognizeCommand:';
  end;

type
  NSSpeechSynthesizerDelegate = objccategory
    procedure speechSynthesizer_didEncounterErrorAtIndex_ofString_message(sender: NSSpeechSynthesizer; characterIndex: culong; string_: NSString; message: NSString); message 'speechSynthesizer:didEncounterErrorAtIndex:ofString:message:';
    procedure speechSynthesizer_didEncounterSyncMessage(sender: NSSpeechSynthesizer; message: NSString); message 'speechSynthesizer:didEncounterSyncMessage:';
    procedure speechSynthesizer_didFinishSpeaking(sender: NSSpeechSynthesizer; finishedSpeaking: Boolean); message 'speechSynthesizer:didFinishSpeaking:';
    procedure speechSynthesizer_willSpeakPhoneme(sender: NSSpeechSynthesizer; phonemeOpcode: cshort); message 'speechSynthesizer:willSpeakPhoneme:';
    procedure speechSynthesizer_willSpeakWord_ofString(sender: NSSpeechSynthesizer; characterRange: NSRange; string_: NSString); message 'speechSynthesizer:willSpeakWord:ofString:';
  end;

type
  NSSpellServerDelegate = objccategory
    function spellServer_checkGrammarInString_language_details(sender: NSSpellServer; stringToCheck: NSString; language: NSString; details: NSArray): NSRange; message 'spellServer:checkGrammarInString:language:details:';
    procedure spellServer_didForgetWord_inLanguage(sender: NSSpellServer; word: NSString; language: NSString); message 'spellServer:didForgetWord:inLanguage:';
    procedure spellServer_didLearnWord_inLanguage(sender: NSSpellServer; word: NSString; language: NSString); message 'spellServer:didLearnWord:inLanguage:';
    function spellServer_findMisspelledWordInString_language_wordCount_countOnly(sender: NSSpellServer; stringToCheck: NSString; language: NSString; wordCount: clong; countOnly: Boolean): NSRange; message 'spellServer:findMisspelledWordInString:language:wordCount:countOnly:';
    function spellServer_suggestCompletionsForPartialWordRange_inString_language(sender: NSSpellServer; range: NSRange; string_: NSString; language: NSString): NSArray; message 'spellServer:suggestCompletionsForPartialWordRange:inString:language:';
    function spellServer_suggestGuessesForWord_inLanguage(sender: NSSpellServer; word: NSString; language: NSString): NSArray; message 'spellServer:suggestGuessesForWord:inLanguage:';
  end;

type
  NSSplitViewDelegate = objccategory
    function splitView_additionalEffectiveRectOfDividerAtIndex(splitView: NSSplitView; dividerIndex: clong): NSRect; message 'splitView:additionalEffectiveRectOfDividerAtIndex:';
    function splitView_canCollapseSubview(splitView: NSSplitView; subview: NSView): Boolean; message 'splitView:canCollapseSubview:';
    function splitView_constrainMaxCoordinate_ofSubviewAt(splitView: NSSplitView; proposedMaximumPosition: CGFloat; dividerIndex: clong): CGFloat; message 'splitView:constrainMaxCoordinate:ofSubviewAt:';
    function splitView_constrainMinCoordinate_ofSubviewAt(splitView: NSSplitView; proposedMinimumPosition: CGFloat; dividerIndex: clong): CGFloat; message 'splitView:constrainMinCoordinate:ofSubviewAt:';
    function splitView_constrainSplitPosition_ofSubviewAt(splitView: NSSplitView; proposedPosition: CGFloat; dividerIndex: clong): CGFloat; message 'splitView:constrainSplitPosition:ofSubviewAt:';
    function splitView_effectiveRect_forDrawnRect_ofDividerAtIndex(splitView: NSSplitView; proposedEffectiveRect: NSRect; drawnRect: NSRect; dividerIndex: clong): NSRect; message 'splitView:effectiveRect:forDrawnRect:ofDividerAtIndex:';
    procedure splitView_resizeSubviewsWithOldSize(splitView: NSSplitView; oldSize: NSSize); message 'splitView:resizeSubviewsWithOldSize:';
    function splitView_shouldCollapseSubview_forDoubleClickOnDividerAtIndex(splitView: NSSplitView; subview: NSView; dividerIndex: clong): Boolean; message 'splitView:shouldCollapseSubview:forDoubleClickOnDividerAtIndex:';
    function splitView_shouldHideDividerAtIndex(splitView: NSSplitView; dividerIndex: clong): Boolean; message 'splitView:shouldHideDividerAtIndex:';
    procedure splitViewDidResizeSubviews(notification: NSNotification); message 'splitViewDidResizeSubviews:';
    procedure splitViewWillResizeSubviews(notification: NSNotification); message 'splitViewWillResizeSubviews:';
  end;

type
  NSStreamDelegateEventExtensions = objccategory
    procedure stream_handleEvent(aStream: NSStream; eventCode: NSStreamEvent); message 'stream:handleEvent:';
  end;

type
  NSTabViewDelegate = objccategory
    procedure tabView_didSelectTabViewItem(tabView: NSTabView; tabViewItem: NSTabViewItem); message 'tabView:didSelectTabViewItem:';
    function tabView_shouldSelectTabViewItem(tabView: NSTabView; tabViewItem: NSTabViewItem): Boolean; message 'tabView:shouldSelectTabViewItem:';
    procedure tabView_willSelectTabViewItem(tabView: NSTabView; tabViewItem: NSTabViewItem); message 'tabView:willSelectTabViewItem:';
    procedure tabViewDidChangeNumberOfTabViewItems(TabView: NSTabView); message 'tabViewDidChangeNumberOfTabViewItems:';
  end;

type
  NSTableDataSource = objccategory
    function numberOfRowsInTableView(tableView: NSTableView): clong; message 'numberOfRowsInTableView:';
    function tableView_acceptDrop_row_dropOperation(tableView: NSTableView; info: id; row: clong; dropOperation: NSTableViewDropOperation): Boolean; message 'tableView:acceptDrop:row:dropOperation:';
    function tableView_namesOfPromisedFilesDroppedAtDestination_forDraggedRowsWithIndexes(tableView: NSTableView; dropDestination: NSURL; indexSet: NSIndexSet): NSArray; message 'tableView:namesOfPromisedFilesDroppedAtDestination:forDraggedRowsWithIndexes:';
    function tableView_objectValueForTableColumn_row(tableView: NSTableView; tableColumn: NSTableColumn; row: clong): id; message 'tableView:objectValueForTableColumn:row:';
    procedure tableView_setObjectValue_forTableColumn_row(tableView: NSTableView; object_: id; tableColumn: NSTableColumn; row: clong); message 'tableView:setObjectValue:forTableColumn:row:';
    procedure tableView_sortDescriptorsDidChange(tableView: NSTableView; oldDescriptors: NSArray); message 'tableView:sortDescriptorsDidChange:';
    function tableView_validateDrop_proposedRow_proposedDropOperation(tableView: NSTableView; info: id; row: clong; dropOperation: NSTableViewDropOperation): NSDragOperation; message 'tableView:validateDrop:proposedRow:proposedDropOperation:';
    function tableView_writeRows_toPasteboard(tableView: NSTableView; rows: NSArray; pboard: NSPasteboard): Boolean; message 'tableView:writeRows:toPasteboard:';
    function tableView_writeRowsWithIndexes_toPasteboard(tableView: NSTableView; rowIndexes: NSIndexSet; pboard: NSPasteboard): Boolean; message 'tableView:writeRowsWithIndexes:toPasteboard:';
  end;

type
  NSTableViewDelegate = objccategory
    function selectionShouldChangeInTableView(tableView: NSTableView): Boolean; message 'selectionShouldChangeInTableView:';
    function tableView_dataCellForTableColumn_row(tableView: NSTableView; tableColumn: NSTableColumn; row: clong): NSCell; message 'tableView:dataCellForTableColumn:row:';
    procedure tableView_didClickTableColumn(tableView: NSTableView; tableColumn: NSTableColumn); message 'tableView:didClickTableColumn:';
    procedure tableView_didDragTableColumn(tableView: NSTableView; tableColumn: NSTableColumn); message 'tableView:didDragTableColumn:';
    function tableView_heightOfRow(tableView: NSTableView; row: clong): CGFloat; message 'tableView:heightOfRow:';
    function tableView_isGroupRow(tableView: NSTableView; row: clong): Boolean; message 'tableView:isGroupRow:';
    procedure tableView_mouseDownInHeaderOfTableColumn(tableView: NSTableView; tableColumn: NSTableColumn); message 'tableView:mouseDownInHeaderOfTableColumn:';
    function tableView_nextTypeSelectMatchFromRow_toRow_forString(tableView: NSTableView; startRow: clong; endRow: clong; searchString: NSString): clong; message 'tableView:nextTypeSelectMatchFromRow:toRow:forString:';
    function tableView_selectionIndexesForProposedSelection(tableView: NSTableView; proposedSelectionIndexes: NSIndexSet): NSIndexSet; message 'tableView:selectionIndexesForProposedSelection:';
    function tableView_shouldEditTableColumn_row(tableView: NSTableView; tableColumn: NSTableColumn; row: clong): Boolean; message 'tableView:shouldEditTableColumn:row:';
    function tableView_shouldSelectRow(tableView: NSTableView; row: clong): Boolean; message 'tableView:shouldSelectRow:';
    function tableView_shouldSelectTableColumn(tableView: NSTableView; tableColumn: NSTableColumn): Boolean; message 'tableView:shouldSelectTableColumn:';
    function tableView_shouldShowCellExpansionForTableColumn_row(tableView: NSTableView; tableColumn: NSTableColumn; row: clong): Boolean; message 'tableView:shouldShowCellExpansionForTableColumn:row:';
    function tableView_shouldTrackCell_forTableColumn_row(tableView: NSTableView; cell_: NSCell; tableColumn: NSTableColumn; row: clong): Boolean; message 'tableView:shouldTrackCell:forTableColumn:row:';
    function tableView_shouldTypeSelectForEvent_withCurrentSearchString(tableView: NSTableView; event: NSEvent; searchString: NSString): Boolean; message 'tableView:shouldTypeSelectForEvent:withCurrentSearchString:';
    function tableView_toolTipForCell_rect_tableColumn_row_mouseLocation(tableView: NSTableView; cell_: NSCell; rect: NSRectPointer; tableColumn: NSTableColumn; row: clong; mouseLocation: NSPoint): NSString; message 'tableView:toolTipForCell:rect:tableColumn:row:mouseLocation:';
    function tableView_typeSelectStringForTableColumn_row(tableView: NSTableView; tableColumn: NSTableColumn; row: clong): NSString; message 'tableView:typeSelectStringForTableColumn:row:';
    procedure tableView_willDisplayCell_forTableColumn_row(tableView: NSTableView; cell_: id; tableColumn: NSTableColumn; row: clong); message 'tableView:willDisplayCell:forTableColumn:row:';
  end;

type
  NSTableViewNotifications = objccategory
    procedure tableViewColumnDidMove(notification: NSNotification); message 'tableViewColumnDidMove:';
    procedure tableViewColumnDidResize(notification: NSNotification); message 'tableViewColumnDidResize:';
    procedure tableViewSelectionDidChange(notification: NSNotification); message 'tableViewSelectionDidChange:';
    procedure tableViewSelectionIsChanging(notification: NSNotification); message 'tableViewSelectionIsChanging:';
  end;

type
  NSTextDelegate = objccategory
    procedure textDidBeginEditing(notification: NSNotification); message 'textDidBeginEditing:';
    procedure textDidChange(notification: NSNotification); message 'textDidChange:';
    procedure textDidEndEditing(notification: NSNotification); message 'textDidEndEditing:';
    function textShouldBeginEditing(textObject: NSText): Boolean; message 'textShouldBeginEditing:';
    function textShouldEndEditing(textObject: NSText): Boolean; message 'textShouldEndEditing:';
  end;

type
  NSTextStorageDelegate = objccategory
    procedure textStorageDidProcessEditing(notification: NSNotification); message 'textStorageDidProcessEditing:';
    procedure textStorageWillProcessEditing(notification: NSNotification); message 'textStorageWillProcessEditing:';
  end;

type
  NSTextViewDelegate = objccategory
    procedure textView_clickedOnCell_inRect(textView: NSTextView; cell: id; cellFrame: NSRect); message 'textView:clickedOnCell:inRect:';
    procedure textView_clickedOnCell_inRect_atIndex(textView: NSTextView; cell: id; cellFrame: NSRect; charIndex: culong); message 'textView:clickedOnCell:inRect:atIndex:';
    function textView_clickedOnLink(textView: NSTextView; link: id): Boolean; message 'textView:clickedOnLink:';
    function textView_clickedOnLink_atIndex(textView: NSTextView; link: id; charIndex: culong): Boolean; message 'textView:clickedOnLink:atIndex:';
    function textView_completions_forPartialWordRange_indexOfSelectedItem(textView: NSTextView; words: NSArray; charRange: NSRange; index: clong): NSArray; message 'textView:completions:forPartialWordRange:indexOfSelectedItem:';
    function textView_doCommandBySelector(textView: NSTextView; commandSelector: SEL): Boolean; message 'textView:doCommandBySelector:';
    procedure textView_doubleClickedOnCell_inRect(textView: NSTextView; cell: id; cellFrame: NSRect); message 'textView:doubleClickedOnCell:inRect:';
    procedure textView_doubleClickedOnCell_inRect_atIndex(textView: NSTextView; cell: id; cellFrame: NSRect; charIndex: culong); message 'textView:doubleClickedOnCell:inRect:atIndex:';
    procedure textView_draggedCell_inRect_event(view: NSTextView; cell: id; rect: NSRect; event: NSEvent); message 'textView:draggedCell:inRect:event:';
    procedure textView_draggedCell_inRect_event_atIndex(view: NSTextView; cell: id; rect: NSRect; event: NSEvent; charIndex: culong); message 'textView:draggedCell:inRect:event:atIndex:';
    function textView_menu_forEvent_atIndex(view: NSTextView; menu_: NSMenu; event: NSEvent; charIndex: culong): NSMenu; message 'textView:menu:forEvent:atIndex:';
    function textView_shouldChangeTextInRange_replacementString(textView: NSTextView; affectedCharRange: NSRange; replacementString: NSString): Boolean; message 'textView:shouldChangeTextInRange:replacementString:';
    function textView_shouldChangeTextInRanges_replacementStrings(textView: NSTextView; affectedRanges: NSArray; replacementStrings: NSArray): Boolean; message 'textView:shouldChangeTextInRanges:replacementStrings:';
    function textView_shouldChangeTypingAttributes_toAttributes(textView: NSTextView; oldTypingAttributes: NSDictionary; newTypingAttributes: NSDictionary): NSDictionary; message 'textView:shouldChangeTypingAttributes:toAttributes:';
    function textView_shouldSetSpellingState_range(textView: NSTextView; value: clong; affectedCharRange: NSRange): clong; message 'textView:shouldSetSpellingState:range:';
    function textView_willChangeSelectionFromCharacterRange_toCharacterRange(textView: NSTextView; oldSelectedCharRange: NSRange; newSelectedCharRange: NSRange): NSRange; message 'textView:willChangeSelectionFromCharacterRange:toCharacterRange:';
    function textView_willChangeSelectionFromCharacterRanges_toCharacterRanges(textView: NSTextView; oldSelectedCharRanges: NSArray; newSelectedCharRanges: NSArray): NSArray; message 'textView:willChangeSelectionFromCharacterRanges:toCharacterRanges:';
    function textView_willDisplayToolTip_forCharacterAtIndex(textView: NSTextView; toolTip_: NSString; characterIndex: culong): NSString; message 'textView:willDisplayToolTip:forCharacterAtIndex:';
    function textView_writablePasteboardTypesForCell_atIndex(view: NSTextView; cell: id; charIndex: culong): NSArray; message 'textView:writablePasteboardTypesForCell:atIndex:';
    function textView_writeCell_atIndex_toPasteboard_type(view: NSTextView; cell: id; charIndex: culong; pboard: NSPasteboard; type_: NSString): Boolean; message 'textView:writeCell:atIndex:toPasteboard:type:';
    procedure textViewDidChangeSelection(notification: NSNotification); message 'textViewDidChangeSelection:';
    procedure textViewDidChangeTypingAttributes(notification: NSNotification); message 'textViewDidChangeTypingAttributes:';
    function undoManagerForTextView(view: NSTextView): NSUndoManager; message 'undoManagerForTextView:';
  end;

type
  NSTokenFieldCellDelegate = objccategory
    function tokenFieldCell_completionsForSubstring_indexOfToken_indexOfSelectedItem(tokenFieldCell: NSTokenFieldCell; substring: NSString; tokenIndex: clong; selectedIndex: clong): NSArray; message 'tokenFieldCell:completionsForSubstring:indexOfToken:indexOfSelectedItem:';
    function tokenFieldCell_displayStringForRepresentedObject(tokenFieldCell: NSTokenFieldCell; representedObject_: id): NSString; message 'tokenFieldCell:displayStringForRepresentedObject:';
    function tokenFieldCell_editingStringForRepresentedObject(tokenFieldCell: NSTokenFieldCell; representedObject_: id): NSString; message 'tokenFieldCell:editingStringForRepresentedObject:';
    function tokenFieldCell_hasMenuForRepresentedObject(tokenFieldCell: NSTokenFieldCell; representedObject_: id): Boolean; message 'tokenFieldCell:hasMenuForRepresentedObject:';
    function tokenFieldCell_menuForRepresentedObject(tokenFieldCell: NSTokenFieldCell; representedObject_: id): NSMenu; message 'tokenFieldCell:menuForRepresentedObject:';
    function tokenFieldCell_readFromPasteboard(tokenFieldCell: NSTokenFieldCell; pboard: NSPasteboard): NSArray; message 'tokenFieldCell:readFromPasteboard:';
    function tokenFieldCell_representedObjectForEditingString(tokenFieldCell: NSTokenFieldCell; editingString: NSString): id; message 'tokenFieldCell:representedObjectForEditingString:';
    function tokenFieldCell_shouldAddObjects_atIndex(tokenFieldCell: NSTokenFieldCell; tokens: NSArray; index: culong): NSArray; message 'tokenFieldCell:shouldAddObjects:atIndex:';
    function tokenFieldCell_styleForRepresentedObject(tokenFieldCell: NSTokenFieldCell; representedObject_: id): NSTokenStyle; message 'tokenFieldCell:styleForRepresentedObject:';
    function tokenFieldCell_writeRepresentedObjects_toPasteboard(tokenFieldCell: NSTokenFieldCell; objects: NSArray; pboard: NSPasteboard): Boolean; message 'tokenFieldCell:writeRepresentedObjects:toPasteboard:';
  end;

type
  NSTokenFieldDelegate = objccategory
    function tokenField_completionsForSubstring_indexOfToken_indexOfSelectedItem(tokenField: NSTokenField; substring: NSString; tokenIndex: clong; selectedIndex: clong): NSArray; message 'tokenField:completionsForSubstring:indexOfToken:indexOfSelectedItem:';
    function tokenField_displayStringForRepresentedObject(tokenField: NSTokenField; representedObject: id): NSString; message 'tokenField:displayStringForRepresentedObject:';
    function tokenField_editingStringForRepresentedObject(tokenField: NSTokenField; representedObject: id): NSString; message 'tokenField:editingStringForRepresentedObject:';
    function tokenField_hasMenuForRepresentedObject(tokenField: NSTokenField; representedObject: id): Boolean; message 'tokenField:hasMenuForRepresentedObject:';
    function tokenField_menuForRepresentedObject(tokenField: NSTokenField; representedObject: id): NSMenu; message 'tokenField:menuForRepresentedObject:';
    function tokenField_readFromPasteboard(tokenField: NSTokenField; pboard: NSPasteboard): NSArray; message 'tokenField:readFromPasteboard:';
    function tokenField_representedObjectForEditingString(tokenField: NSTokenField; editingString: NSString): id; message 'tokenField:representedObjectForEditingString:';
    function tokenField_shouldAddObjects_atIndex(tokenField: NSTokenField; tokens: NSArray; index: culong): NSArray; message 'tokenField:shouldAddObjects:atIndex:';
    function tokenField_styleForRepresentedObject(tokenField: NSTokenField; representedObject: id): NSTokenStyle; message 'tokenField:styleForRepresentedObject:';
    function tokenField_writeRepresentedObjects_toPasteboard(tokenField: NSTokenField; objects: NSArray; pboard: NSPasteboard): Boolean; message 'tokenField:writeRepresentedObjects:toPasteboard:';
  end;

type
  NSToolbarDelegate = objccategory
    function toolbar_itemForItemIdentifier_willBeInsertedIntoToolbar(toolbar: NSToolbar; itemIdentifier: NSString; flag: Boolean): NSToolbarItem; message 'toolbar:itemForItemIdentifier:willBeInsertedIntoToolbar:';
    function toolbarAllowedItemIdentifiers(toolbar: NSToolbar): NSArray; message 'toolbarAllowedItemIdentifiers:';
    function toolbarDefaultItemIdentifiers(toolbar: NSToolbar): NSArray; message 'toolbarDefaultItemIdentifiers:';
    function toolbarSelectableItemIdentifiers(toolbar: NSToolbar): NSArray; message 'toolbarSelectableItemIdentifiers:';
  end;

type
  NSToolbarNotifications = objccategory
    procedure toolbarDidRemoveItem(notification: NSNotification); message 'toolbarDidRemoveItem:';
    procedure toolbarWillAddItem(notification: NSNotification); message 'toolbarWillAddItem:';
  end;

type
  NSURLConnectionDelegate = objccategory
    procedure connection_didCancelAuthenticationChallenge(connection: NSURLConnection; challenge: NSURLAuthenticationChallenge); message 'connection:didCancelAuthenticationChallenge:';
    procedure connection_didFailWithError(connection: NSURLConnection; error: NSError); message 'connection:didFailWithError:';
    procedure connection_didReceiveAuthenticationChallenge(connection: NSURLConnection; challenge: NSURLAuthenticationChallenge); message 'connection:didReceiveAuthenticationChallenge:';
    procedure connection_didReceiveData(connection: NSURLConnection; data: NSData); message 'connection:didReceiveData:';
    procedure connection_didReceiveResponse(connection: NSURLConnection; response: NSURLResponse); message 'connection:didReceiveResponse:';
    function connection_willCacheResponse(connection: NSURLConnection; cachedResponse: NSCachedURLResponse): NSCachedURLResponse; message 'connection:willCacheResponse:';
    function connection_willSendRequest_redirectResponse(connection: NSURLConnection; request: NSURLRequest; response: NSURLResponse): NSURLRequest; message 'connection:willSendRequest:redirectResponse:';
    procedure connectionDidFinishLoading(connection: NSURLConnection); message 'connectionDidFinishLoading:';
  end;

type
  NSURLDownloadDelegate = objccategory
    procedure download_decideDestinationWithSuggestedFilename(download: NSURLDownload; filename: NSString); message 'download:decideDestinationWithSuggestedFilename:';
    procedure download_didCancelAuthenticationChallenge(download: NSURLDownload; challenge: NSURLAuthenticationChallenge); message 'download:didCancelAuthenticationChallenge:';
    procedure download_didCreateDestination(download: NSURLDownload; path: NSString); message 'download:didCreateDestination:';
    procedure download_didFailWithError(download: NSURLDownload; error: NSError); message 'download:didFailWithError:';
    procedure download_didReceiveAuthenticationChallenge(download: NSURLDownload; challenge: NSURLAuthenticationChallenge); message 'download:didReceiveAuthenticationChallenge:';
    procedure download_didReceiveDataOfLength(download: NSURLDownload; length: culong); message 'download:didReceiveDataOfLength:';
    procedure download_didReceiveResponse(download: NSURLDownload; response: NSURLResponse); message 'download:didReceiveResponse:';
    function download_shouldDecodeSourceDataOfMIMEType(download: NSURLDownload; encodingType: NSString): Boolean; message 'download:shouldDecodeSourceDataOfMIMEType:';
    procedure download_willResumeWithResponse_fromByte(download: NSURLDownload; response: NSURLResponse; startingByte: clonglong); message 'download:willResumeWithResponse:fromByte:';
    function download_willSendRequest_redirectResponse(download: NSURLDownload; request_: NSURLRequest; redirectResponse: NSURLResponse): NSURLRequest; message 'download:willSendRequest:redirectResponse:';
    procedure downloadDidBegin(download: NSURLDownload); message 'downloadDidBegin:';
    procedure downloadDidFinish(download: NSURLDownload); message 'downloadDidFinish:';
  end;

type
  NSWindowDelegate = objccategory
    function window_shouldDragDocumentWithEvent_from_withPasteboard(window: NSWindow; event: NSEvent; dragImageLocation: NSPoint; pasteboard: NSPasteboard): Boolean; message 'window:shouldDragDocumentWithEvent:from:withPasteboard:';
    function window_shouldPopUpDocumentPathMenu(window: NSWindow; menu_: NSMenu): Boolean; message 'window:shouldPopUpDocumentPathMenu:';
    function window_willPositionSheet_usingRect(window: NSWindow; sheet: NSWindow; rect: NSRect): NSRect; message 'window:willPositionSheet:usingRect:';
    function windowShouldClose(sender: id): Boolean; message 'windowShouldClose:';
    function windowShouldZoom_toFrame(window: NSWindow; newFrame: NSRect): Boolean; message 'windowShouldZoom:toFrame:';
    function windowWillResize_toSize(sender: NSWindow; frameSize: NSSize): NSSize; message 'windowWillResize:toSize:';
    function windowWillReturnFieldEditor_toObject(sender: NSWindow; client: id): id; message 'windowWillReturnFieldEditor:toObject:';
    function windowWillReturnUndoManager(window: NSWindow): NSUndoManager; message 'windowWillReturnUndoManager:';
    function windowWillUseStandardFrame_defaultFrame(window: NSWindow; newFrame: NSRect): NSRect; message 'windowWillUseStandardFrame:defaultFrame:';
  end;

type
  NSWindowNotifications = objccategory
    procedure windowDidBecomeKey(notification: NSNotification); message 'windowDidBecomeKey:';
    procedure windowDidBecomeMain(notification: NSNotification); message 'windowDidBecomeMain:';
    procedure windowDidChangeScreen(notification: NSNotification); message 'windowDidChangeScreen:';
    procedure windowDidChangeScreenProfile(notification: NSNotification); message 'windowDidChangeScreenProfile:';
    procedure windowDidDeminiaturize(notification: NSNotification); message 'windowDidDeminiaturize:';
    procedure windowDidEndSheet(notification: NSNotification); message 'windowDidEndSheet:';
    procedure windowDidExpose(notification: NSNotification); message 'windowDidExpose:';
    procedure windowDidMiniaturize(notification: NSNotification); message 'windowDidMiniaturize:';
    procedure windowDidMove(notification: NSNotification); message 'windowDidMove:';
    procedure windowDidResignKey(notification: NSNotification); message 'windowDidResignKey:';
    procedure windowDidResignMain(notification: NSNotification); message 'windowDidResignMain:';
    procedure windowDidResize(notification: NSNotification); message 'windowDidResize:';
    procedure windowDidUpdate(notification: NSNotification); message 'windowDidUpdate:';
    procedure windowWillBeginSheet(notification: NSNotification); message 'windowWillBeginSheet:';
    procedure windowWillClose(notification: NSNotification); message 'windowWillClose:';
    procedure windowWillMiniaturize(notification: NSNotification); message 'windowWillMiniaturize:';
    procedure windowWillMove(notification: NSNotification); message 'windowWillMove:';
  end;

type
  NSXMLParserDelegateEventAdditions = objccategory
    procedure parser_didEndElement_namespaceURI_qualifiedName(parser: NSXMLParser; elementName: NSString; namespaceURI: NSString; qName: NSString); message 'parser:didEndElement:namespaceURI:qualifiedName:';
    procedure parser_didEndMappingPrefix(parser: NSXMLParser; prefix: NSString); message 'parser:didEndMappingPrefix:';
    procedure parser_didStartElement_namespaceURI_qualifiedName_attributes(parser: NSXMLParser; elementName: NSString; namespaceURI: NSString; qName: NSString; attributeDict: NSDictionary); message 'parser:didStartElement:namespaceURI:qualifiedName:attributes:';
    procedure parser_didStartMappingPrefix_toURI(parser: NSXMLParser; prefix: NSString; namespaceURI: NSString); message 'parser:didStartMappingPrefix:toURI:';
    procedure parser_foundAttributeDeclarationWithName_forElement_type_defaultValue(parser: NSXMLParser; attributeName: NSString; elementName: NSString; type_: NSString; defaultValue: NSString); message 'parser:foundAttributeDeclarationWithName:forElement:type:defaultValue:';
    procedure parser_foundCDATA(parser: NSXMLParser; CDATABlock: NSData); message 'parser:foundCDATA:';
    procedure parser_foundCharacters(parser: NSXMLParser; string_: NSString); message 'parser:foundCharacters:';
    procedure parser_foundComment(parser: NSXMLParser; comment: NSString); message 'parser:foundComment:';
    procedure parser_foundElementDeclarationWithName_model(parser: NSXMLParser; elementName: NSString; model: NSString); message 'parser:foundElementDeclarationWithName:model:';
    procedure parser_foundExternalEntityDeclarationWithName_publicID_systemID(parser: NSXMLParser; name: NSString; publicID_: NSString; systemID_: NSString); message 'parser:foundExternalEntityDeclarationWithName:publicID:systemID:';
    procedure parser_foundIgnorableWhitespace(parser: NSXMLParser; whitespaceString: NSString); message 'parser:foundIgnorableWhitespace:';
    procedure parser_foundInternalEntityDeclarationWithName_value(parser: NSXMLParser; name: NSString; value: NSString); message 'parser:foundInternalEntityDeclarationWithName:value:';
    procedure parser_foundNotationDeclarationWithName_publicID_systemID(parser: NSXMLParser; name: NSString; publicID_: NSString; systemID_: NSString); message 'parser:foundNotationDeclarationWithName:publicID:systemID:';
    procedure parser_foundProcessingInstructionWithTarget_data(parser: NSXMLParser; target: NSString; data: NSString); message 'parser:foundProcessingInstructionWithTarget:data:';
    procedure parser_foundUnparsedEntityDeclarationWithName_publicID_systemID_notationName(parser: NSXMLParser; name: NSString; publicID_: NSString; systemID_: NSString; notationName: NSString); message 'parser:foundUnparsedEntityDeclarationWithName:publicID:systemID:notationName:';
    procedure parser_parseErrorOccurred(parser: NSXMLParser; parseError: NSError); message 'parser:parseErrorOccurred:';
    function parser_resolveExternalEntityName_systemID(parser: NSXMLParser; name: NSString; systemID_: NSString): NSData; message 'parser:resolveExternalEntityName:systemID:';
    procedure parser_validationErrorOccurred(parser: NSXMLParser; validationError: NSError); message 'parser:validationErrorOccurred:';
    procedure parserDidEndDocument(parser: NSXMLParser); message 'parserDidEndDocument:';
    procedure parserDidStartDocument(parser: NSXMLParser); message 'parserDidStartDocument:';
  end;
