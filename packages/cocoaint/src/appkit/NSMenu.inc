{ Parsed from Appkit.framework NSMenu.h }

{$ifdef TYPES}
{$ifndef NSMENU_PAS_T}
{$define NSMENU_PAS_T}

{ Constants }

const
  NSMenuPropertyItemTitle = 1 shl 0;
  NSMenuPropertyItemAttributedTitle = 1 shl 1;
  NSMenuPropertyItemKeyEquivalent = 1 shl 2;
  NSMenuPropertyItemImage = 1 shl 3;
  NSMenuPropertyItemEnabled = 1 shl 4;
  NSMenuPropertyItemAccessibilityDescription = 1 shl 5;

{ Types }
type
  NSMenuProperties = NSUInteger;
  NSMenuPropertiesPtr = ^NSMenuProperties;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMENU_PAS_R}
{$define NSMENU_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMENU_PAS_F}
{$define NSMENU_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMENU_PAS_S}
{$define NSMENU_PAS_S}

{ External string constants }
var
  NSMenuWillSendActionNotification: NSString; cvar; external;
  NSMenuDidSendActionNotification: NSString; cvar; external;
  NSMenuDidAddItemNotification: NSString; cvar; external;
  NSMenuDidRemoveItemNotification: NSString; cvar; external;
  NSMenuDidChangeItemNotification: NSString; cvar; external;
  NSMenuDidBeginTrackingNotification: NSString; cvar; external;
  NSMenuDidEndTrackingNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSMenuDelegateProtocol = objcprotocol;
  NSMenu = objcclass;
  NSMenuPointer = ^NSMenu;
  NSMenuPtr = NSMenuPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMENU_PAS_C}
{$define NSMENU_PAS_C}

{ NSMenu }
  NSMenu = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol)
  private
    _supermenu: NSMenu;
    _title: NSString;
    _itemArray: id;
    _extra: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __mFlags: record
      case byte of
       0: (_anonbitfield_NSMenu0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __mFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSMenu0: cuint);
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
        noAutoenable: 0..1;
        inMain: 0..1;
        internalPerformAction: 0..1;
        condenseSeparators: 0..1;
        disabled: 0..1;
        ownedByPopUp: 0..1;
        delegateNeedsUpdate: 0..1;
        delegateUpdateItem: 0..1;
        delegateHasKeyEquiv: 0..1;
        delegateHasAltKeyEquiv: 0..1;
        excludeMarkColumn: 0..1;
        isContextualMenu: 0..1;
        cmPluginMode: 0..((1 shl 2)-1);
        invertedCMPluginTypes: 0..((1 shl 2)-1);
        allowsDifferentSelection: 0..1;
        RESERVED: 0..((1 shl 15)-1);
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
    _name: NSString;
    
  public
    function initWithTitle(aTitle: NSString): id; message 'initWithTitle:';
    procedure setTitle(aString: NSString); message 'setTitle:';
    function title: NSString; message 'title';
    class procedure popUpContextMenu_withEvent_forView(menu: NSMenu; event: NSEvent; view: NSView); message 'popUpContextMenu:withEvent:forView:';
    class procedure popUpContextMenu_withEvent_forView_withFont(menu: NSMenu; event: NSEvent; view: NSView; font_: NSFont); message 'popUpContextMenu:withEvent:forView:withFont:';
    function popUpMenuPositioningItem_atLocation_inView(item: NSMenuItem; location: NSPoint; view: NSView): Boolean; message 'popUpMenuPositioningItem:atLocation:inView:';
    class procedure setMenuBarVisible(visible: Boolean); message 'setMenuBarVisible:';
    class function menuBarVisible: Boolean; message 'menuBarVisible';
    function supermenu: NSMenu; message 'supermenu';
    procedure insertItem_atIndex(newItem: NSMenuItem; index: NSInteger); message 'insertItem:atIndex:';
    procedure addItem(newItem: NSMenuItem); message 'addItem:';
    function insertItemWithTitle_action_keyEquivalent_atIndex(aString: NSString; aSelector: SEL; charCode: NSString; index: NSInteger): NSMenuItem; message 'insertItemWithTitle:action:keyEquivalent:atIndex:';
    function addItemWithTitle_action_keyEquivalent(aString: NSString; aSelector: SEL; charCode: NSString): NSMenuItem; message 'addItemWithTitle:action:keyEquivalent:';
    procedure removeItemAtIndex(index: NSInteger); message 'removeItemAtIndex:';
    procedure removeItem(item: NSMenuItem); message 'removeItem:';
    procedure setSubmenu_forItem(aMenu: NSMenu; anItem: NSMenuItem); message 'setSubmenu:forItem:';
    procedure removeAllItems; message 'removeAllItems';
    function itemArray: NSArray; message 'itemArray';
    function numberOfItems: NSInteger; message 'numberOfItems';
    function indexOfItem(index: NSMenuItem): NSInteger; message 'indexOfItem:';
    function indexOfItemWithTitle(aTitle: NSString): NSInteger; message 'indexOfItemWithTitle:';
    function indexOfItemWithTag(aTag: NSInteger): NSInteger; message 'indexOfItemWithTag:';
    function indexOfItemWithRepresentedObject(object_: id): NSInteger; message 'indexOfItemWithRepresentedObject:';
    function indexOfItemWithSubmenu(submenu: NSMenu): NSInteger; message 'indexOfItemWithSubmenu:';
    function indexOfItemWithTarget_andAction(target: id; actionSelector: SEL): NSInteger; message 'indexOfItemWithTarget:andAction:';
    function itemWithTitle(aTitle: NSString): NSMenuItem; message 'itemWithTitle:';
    function itemWithTag(tag: NSInteger): NSMenuItem; message 'itemWithTag:';
    procedure setAutoenablesItems(flag: Boolean); message 'setAutoenablesItems:';
    function autoenablesItems: Boolean; message 'autoenablesItems';
    procedure update; message 'update';
    function performKeyEquivalent(theEvent: NSEvent): Boolean; message 'performKeyEquivalent:';
    procedure itemChanged(item: NSMenuItem); message 'itemChanged:';
    procedure performActionForItemAtIndex(index: NSInteger); message 'performActionForItemAtIndex:';
    procedure setDelegate(anObject: NSMenuDelegateProtocol); message 'setDelegate:';
    function delegate: NSMenuDelegateProtocol; message 'delegate';
    function menuBarHeight: CGFloat; message 'menuBarHeight';
    procedure cancelTracking; message 'cancelTracking';
    procedure cancelTrackingWithoutAnimation; message 'cancelTrackingWithoutAnimation';
    function highlightedItem: NSMenuItem; message 'highlightedItem';
    function minimumWidth: CGFloat; message 'minimumWidth';
    procedure setMinimumWidth(width: CGFloat); message 'setMinimumWidth:';
    function size: NSSize; message 'size';
    function font: NSFont; message 'font';
    procedure setFont(font_: NSFont); message 'setFont:';
    function allowsContextMenuPlugIns: Boolean; message 'allowsContextMenuPlugIns';
    procedure setAllowsContextMenuPlugIns(allows: Boolean); message 'setAllowsContextMenuPlugIns:';
    procedure setShowsStateColumn(showsState: Boolean); message 'setShowsStateColumn:';
    function showsStateColumn: Boolean; message 'showsStateColumn';
    class function menuZone: NSZonePtr; message 'menuZone';
    procedure setMenuChangedMessagesEnabled(flag: Boolean); message 'setMenuChangedMessagesEnabled:';
    function menuChangedMessagesEnabled: Boolean; message 'menuChangedMessagesEnabled';
    procedure helpRequested(eventPtr: NSEvent); message 'helpRequested:';
    function isTornOff: Boolean; message 'isTornOff';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSSubmenuActionCategory }
  NSSubmenuActionCategory = objccategory external (NSMenu)
    procedure submenuAction(sender: id); message 'submenuAction:';
  end;

{ NSMenuValidationCategory }
  NSMenuValidationCategory = objccategory external (NSObject)
    function validateMenuItem(menuItem: NSMenuItem): Boolean; message 'validateMenuItem:';
  end;

{ NSMenuPropertiesToUpdateCategory }
  NSMenuPropertiesToUpdateCategory = objccategory external (NSMenu)
    function propertiesToUpdate: NSMenuProperties; message 'propertiesToUpdate';
  end;

{ NSDeprecated_NSMenuCategory }
  NSDeprecated_NSMenuCategory = objccategory external name 'NSDeprecated' (NSMenu)
    procedure setMenuRepresentation(menuRep: id); message 'setMenuRepresentation:'; deprecated 'in Mac OS X 10.2 and later';
    function menuRepresentation: id; message 'menuRepresentation'; deprecated 'in Mac OS X 10.2 and later';
    procedure setContextMenuRepresentation(menuRep: id); message 'setContextMenuRepresentation:'; deprecated 'in Mac OS X 10.2 and later';
    function contextMenuRepresentation: id; message 'contextMenuRepresentation'; deprecated 'in Mac OS X 10.2 and later';
    procedure setTearOffMenuRepresentation(menuRep: id); message 'setTearOffMenuRepresentation:'; deprecated 'in Mac OS X 10.2 and later';
    function tearOffMenuRepresentation: id; message 'tearOffMenuRepresentation'; deprecated 'in Mac OS X 10.2 and later';
    class procedure setMenuZone(aZone: NSZonePtr); message 'setMenuZone:'; deprecated 'in Mac OS X 10.2 and later';
    function attachedMenu: NSMenu; message 'attachedMenu'; deprecated 'in Mac OS X 10.2 and later';
    function isAttached: Boolean; message 'isAttached'; deprecated 'in Mac OS X 10.2 and later';
    procedure sizeToFit; message 'sizeToFit'; deprecated 'in Mac OS X 10.2 and later';
    function locationForSubmenu(aSubmenu: NSMenu): NSPoint; message 'locationForSubmenu:'; deprecated 'in Mac OS X 10.2 and later';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSMENU_PAS_P}
{$define NSMENU_PAS_P}
  
{ NSMenuDelegate Protocol }
  NSMenuDelegateProtocol = objcprotocol external name 'NSMenuDelegate'
  optional
    procedure menuNeedsUpdate(menu: NSMenu); message 'menuNeedsUpdate:';
    function numberOfItemsInMenu(menu: NSMenu): NSInteger; message 'numberOfItemsInMenu:';
    function menu_updateItem_atIndex_shouldCancel(menu: NSMenu; item: NSMenuItem; index: NSInteger; shouldCancel: Boolean): Boolean; message 'menu:updateItem:atIndex:shouldCancel:';
    function menuHasKeyEquivalent_forEvent_target_action(menu: NSMenu; event: NSEvent; target: idPtr; action: SELPtr): Boolean; message 'menuHasKeyEquivalent:forEvent:target:action:';
    procedure menuWillOpen(menu: NSMenu); message 'menuWillOpen:';
    procedure menuDidClose(menu: NSMenu); message 'menuDidClose:';
    procedure menu_willHighlightItem(menu: NSMenu; item: NSMenuItem); message 'menu:willHighlightItem:';
    function confinementRectForMenu_onScreen(menu: NSMenu; screen: NSScreen): NSRect; message 'confinementRectForMenu:onScreen:';
  end;
{$endif}
{$endif}
