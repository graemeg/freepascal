{ Parsed from Appkit.framework NSMenuItem.h }

{$ifdef TYPES}
{$ifndef NSMENUITEM_PAS_T}
{$define NSMENUITEM_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMENUITEM_PAS_R}
{$define NSMENUITEM_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMENUITEM_PAS_F}
{$define NSMENUITEM_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMENUITEM_PAS_S}
{$define NSMENUITEM_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSMenuItem = objcclass;
  NSMenuItemPointer = ^NSMenuItem;
  NSMenuItemPtr = NSMenuItemPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMENUITEM_PAS_C}
{$define NSMENUITEM_PAS_C}

{ NSMenuItem }
  NSMenuItem = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol, NSValidatedUserInterfaceItemProtocol)
  private
    _menu: NSMenu;
    _title: NSString;
    _keyEquivalent: NSString;
    _keyEquivalentModifierMask: NSUInteger;
    _userKEGenerationCount: NSInteger;
    _state: NSInteger;
    _image: NSImage;
    _submenu: NSMenu;
    _uiid: NSString;
    _repObject: id;
    _target: id;
    _action: SEL;
    _tag: NSInteger;
    _extraData: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __miFlags: record
      case byte of
       0: (_anonbitfield_NSMenuItem0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __miFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSMenuItem0: cuint);
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
        disabled: 0..1;
        isSeparator: 0..1;
        hidden: 0..1;
        alternate: 0..1;
        indent: 0..((1 shl 4)-1);
        keShareMode: 0..((1 shl 3)-1);
        RESERVED1: 0..((1 shl 13)-1);
        highlighted: 0..1;
        limitedView: 0..1;
        nextItemIsAlternate: 0..1;
        blockKE: 0..1;
        ignoredForAccessibility: 0..1;
        hiddenActiveKE: 0..1;
        noRepeatKEs: 0..1;
        RESERVED: 0..1;
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
    
  public
    class procedure setUsesUserKeyEquivalents(flag: Boolean); message 'setUsesUserKeyEquivalents:';
    class function usesUserKeyEquivalents: Boolean; message 'usesUserKeyEquivalents';
    class function separatorItem: NSMenuItem; message 'separatorItem';
    function initWithTitle_action_keyEquivalent(aString: NSString; aSelector: SEL; charCode: NSString): id; message 'initWithTitle:action:keyEquivalent:';
    procedure setMenu(menu_: NSMenu); message 'setMenu:';
    function menu: NSMenu; message 'menu';
    function hasSubmenu: Boolean; message 'hasSubmenu';
    procedure setSubmenu(submenu_: NSMenu); message 'setSubmenu:';
    function submenu: NSMenu; message 'submenu';
    function parentItem: NSMenuItem; message 'parentItem';
    procedure setTitle(aString: NSString); message 'setTitle:';
    function title: NSString; message 'title';
    procedure setAttributedTitle(string_: NSAttributedString); message 'setAttributedTitle:';
    function attributedTitle: NSAttributedString; message 'attributedTitle';
    function isSeparatorItem: Boolean; message 'isSeparatorItem';
    procedure setKeyEquivalent(aKeyEquivalent: NSString); message 'setKeyEquivalent:';
    function keyEquivalent: NSString; message 'keyEquivalent';
    procedure setKeyEquivalentModifierMask(mask: NSUInteger); message 'setKeyEquivalentModifierMask:';
    function keyEquivalentModifierMask: NSUInteger; message 'keyEquivalentModifierMask';
    function userKeyEquivalent: NSString; message 'userKeyEquivalent';
    procedure setTitleWithMnemonic(stringWithAmpersand: NSString); message 'setTitleWithMnemonic:';
    procedure setImage(menuImage: NSImage); message 'setImage:';
    function image: NSImage; message 'image';
    procedure setState(state_: NSInteger); message 'setState:';
    function state: NSInteger; message 'state';
    procedure setOnStateImage(image_: NSImage); message 'setOnStateImage:';
    function onStateImage: NSImage; message 'onStateImage';
    procedure setOffStateImage(image_: NSImage); message 'setOffStateImage:';
    function offStateImage: NSImage; message 'offStateImage';
    procedure setMixedStateImage(image_: NSImage); message 'setMixedStateImage:';
    function mixedStateImage: NSImage; message 'mixedStateImage';
    procedure setEnabled(flag: Boolean); message 'setEnabled:';
    function isEnabled: Boolean; message 'isEnabled';
    procedure setAlternate(isAlternate_: Boolean); message 'setAlternate:';
    function isAlternate: Boolean; message 'isAlternate';
    procedure setIndentationLevel(indentationLevel_: NSInteger); message 'setIndentationLevel:';
    function indentationLevel: NSInteger; message 'indentationLevel';
    procedure setTarget(anObject: id); message 'setTarget:';
    function target: id; message 'target';
    procedure setAction(aSelector: SEL); message 'setAction:';
    function action: SEL; message 'action';
    procedure setTag(anInt: NSInteger); message 'setTag:';
    function tag: NSInteger; message 'tag';
    procedure setRepresentedObject(anObject: id); message 'setRepresentedObject:';
    function representedObject: id; message 'representedObject';
    procedure setView(view_: NSView); message 'setView:';
    function view: NSView; message 'view';
    function isHighlighted: Boolean; message 'isHighlighted';
    procedure setHidden(hidden: Boolean); message 'setHidden:';
    function isHidden: Boolean; message 'isHidden';
    function isHiddenOrHasHiddenAncestor: Boolean; message 'isHiddenOrHasHiddenAncestor';
    procedure setToolTip(toolTip_: NSString); message 'setToolTip:';
    function toolTip: NSString; message 'toolTip';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSViewEnclosingMenuItemCategory }
  NSViewEnclosingMenuItemCategory = objccategory external (NSView)
    function enclosingMenuItem: NSMenuItem; message 'enclosingMenuItem';
  end;

{ NSDeprecated_NSMenuItemCategory }
  NSDeprecated_NSMenuItemCategory = objccategory external name 'NSDeprecated' (NSMenuItem)
    procedure setMnemonicLocation(location: NSUInteger); message 'setMnemonicLocation:'; deprecated 'in Mac OS X 10.6 and later';
    function mnemonicLocation: NSUInteger; message 'mnemonicLocation'; deprecated 'in Mac OS X 10.6 and later';
    function mnemonic: NSString; message 'mnemonic'; deprecated 'in Mac OS X 10.6 and later';
  end;

{$endif}
{$endif}
