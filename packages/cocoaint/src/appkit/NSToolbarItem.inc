{ Parsed from Appkit.framework NSToolbarItem.h }

{$ifdef TYPES}
{$ifndef NSTOOLBARITEM_PAS_T}
{$define NSTOOLBARITEM_PAS_T}

{ Constants }

const
  NSToolbarItemVisibilityPriorityStandard = 0;
  NSToolbarItemVisibilityPriorityLow = -1000;
  NSToolbarItemVisibilityPriorityHigh = 1000;
  NSToolbarItemVisibilityPriorityUser = 2000;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTOOLBARITEM_PAS_R}
{$define NSTOOLBARITEM_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTOOLBARITEM_PAS_F}
{$define NSTOOLBARITEM_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTOOLBARITEM_PAS_S}
{$define NSTOOLBARITEM_PAS_S}

{ External string constants }
var
  NSToolbarSeparatorItemIdentifier: NSString; cvar; external;
  NSToolbarSpaceItemIdentifier: NSString; cvar; external;
  NSToolbarFlexibleSpaceItemIdentifier: NSString; cvar; external;
  NSToolbarShowColorsItemIdentifier: NSString; cvar; external;
  NSToolbarShowFontsItemIdentifier: NSString; cvar; external;
  NSToolbarCustomizeToolbarItemIdentifier: NSString; cvar; external;
  NSToolbarPrintItemIdentifier: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSToolbarItem = objcclass;
  NSToolbarItemPointer = ^NSToolbarItem;
  NSToolbarItemPtr = NSToolbarItemPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTOOLBARITEM_PAS_C}
{$define NSTOOLBARITEM_PAS_C}

{ NSToolbarItem }
  NSToolbarItem = objcclass external (NSObject, NSCopyingProtocol, NSValidatedUserInterfaceItemProtocol)
  private
    _toolbar: NSToolbar;
    _image: NSImage;
    _itemIdentifier: NSString;
    _label: NSString;
    _labelAlignment: NSTextAlignment;
    _paletteLabel: NSString;
    _toolTip: NSString;
    _menuItemRep: NSMenuItem;
    _tag: NSInteger;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __tbiFlags: record
      case byte of
       0: (_anonbitfield_NSToolbarItem0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __tbiFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSToolbarItem0: cuint);
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
        viewRespondsToIsEnabled: 0..1;
        viewRespondsToSetEnabled: 0..1;
        viewRespondsToTag: 0..1;
        viewRespondsToSetTag: 0..1;
        viewRespondsToAction: 0..1;
        viewRespondsToSetAction: 0..1;
        viewRespondsToTarget: 0..1;
        viewRespondsToSetTarget: 0..1;
        viewRespondsToImage: 0..1;
        viewRespondsToSetImage: 0..1;
        _isEnabled: 0..1;
        isUserRemovable: 0..1;
        menuHasBeenSet: 0..1;
        menuRepIsDefault: 0..1;
        viewHasBeenLoaded: 0..1;
        drawingForDragImage: 0..1;
        isCustomItemType: 0..1;
        hasValidatedAutoModeConfiguration: 0..1;
        useAutoModeConfiguration: 0..1;
        RESERVED2: 0..1;
        autovalidationDisabled: 0..1;
        tagHasBeenSet: 0..1;
        sizeHasBeenSet: 0..1;
        stateWasDisabledBeforeSheet: 0..1;
        RESERVED: 0..((1 shl 8)-1);
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
    _tbiReserved: id;
    _itemViewer: id;
    _view: NSView;
    _minSize: NSSize;
    _maxSize: NSSize;
    {$ifdef cpu64}
    _toolbarItemReserved: id;
    {$endif}
    
  public
    function initWithItemIdentifier(itemIdentifier_: NSString): id; message 'initWithItemIdentifier:';
    function itemIdentifier: NSString; message 'itemIdentifier';
    function toolbar: NSToolbar; message 'toolbar';
    procedure setLabel(label__: NSString); message 'setLabel:';
    function label_: NSString; message 'label';
    procedure setPaletteLabel(paletteLabel_: NSString); message 'setPaletteLabel:';
    function paletteLabel: NSString; message 'paletteLabel';
    procedure setToolTip(toolTip_: NSString); message 'setToolTip:';
    function toolTip: NSString; message 'toolTip';
    procedure setMenuFormRepresentation(menuItem: NSMenuItem); message 'setMenuFormRepresentation:';
    function menuFormRepresentation: NSMenuItem; message 'menuFormRepresentation';
    procedure setTag(tag_: NSInteger); message 'setTag:';
    function tag: NSInteger; message 'tag';
    procedure setTarget(target_: id); message 'setTarget:';
    function target: id; message 'target';
    procedure setAction(action_: SEL); message 'setAction:';
    function action: SEL; message 'action';
    procedure setEnabled(enabled: Boolean); message 'setEnabled:';
    function isEnabled: Boolean; message 'isEnabled';
    procedure setImage(image_: NSImage); message 'setImage:';
    function image: NSImage; message 'image';
    procedure setView(view_: NSView); message 'setView:';
    function view: NSView; message 'view';
    procedure setMinSize(size: NSSize); message 'setMinSize:';
    function minSize: NSSize; message 'minSize';
    procedure setMaxSize(size: NSSize); message 'setMaxSize:';
    function maxSize: NSSize; message 'maxSize';
    procedure setVisibilityPriority(visibilityPriority_: NSInteger); message 'setVisibilityPriority:';
    function visibilityPriority: NSInteger; message 'visibilityPriority';
    procedure validate; message 'validate';
    procedure setAutovalidates(resistance: Boolean); message 'setAutovalidates:';
    function autovalidates: Boolean; message 'autovalidates';
    function allowsDuplicatesInToolbar: Boolean; message 'allowsDuplicatesInToolbar';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
  end;

{ NSToolbarItemValidationCategory }
  NSToolbarItemValidationCategory = objccategory external (NSObject)
    function validateToolbarItem(theItem: NSToolbarItem): Boolean; message 'validateToolbarItem:';
  end;

{$endif}
{$endif}
