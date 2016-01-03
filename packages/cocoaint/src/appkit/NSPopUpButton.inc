{ Parsed from Appkit.framework NSPopUpButton.h }

{$ifdef TYPES}
{$ifndef NSPOPUPBUTTON_PAS_T}
{$define NSPOPUPBUTTON_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPOPUPBUTTON_PAS_R}
{$define NSPOPUPBUTTON_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPOPUPBUTTON_PAS_F}
{$define NSPOPUPBUTTON_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPOPUPBUTTON_PAS_S}
{$define NSPOPUPBUTTON_PAS_S}

{ External string constants }
var
  NSPopUpButtonWillPopUpNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPopUpButton = objcclass;
  NSPopUpButtonPointer = ^NSPopUpButton;
  NSPopUpButtonPtr = NSPopUpButtonPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPOPUPBUTTON_PAS_C}
{$define NSPOPUPBUTTON_PAS_C}

{ NSPopUpButton }
  NSPopUpButton = objcclass external (NSButton)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __pbFlags: record
      case byte of
       0: (_anonbitfield_NSPopUpButton0: cuint);
       1: (data: bitpacked record
        needsPullsDownFromTemplate: 0..1;
        RESERVED: 0..((1 shl 31)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __pbFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSPopUpButton0: cuint);
       1: (
        needsPullsDownFromTemplate: 0..1;
        RESERVED: 0..((1 shl 31)-1);
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
      );
      end;
    {$ifdef cpu64}
    _popupReserved: id;
    {$endif}
    
  public
    function initWithFrame_pullsDown(buttonFrame: NSRect; flag: Boolean): id; message 'initWithFrame:pullsDown:';
    procedure setMenu(menu_: NSMenu); message 'setMenu:';
    function menu: NSMenu; message 'menu';
    procedure setPullsDown(flag: Boolean); message 'setPullsDown:';
    function pullsDown: Boolean; message 'pullsDown';
    procedure setAutoenablesItems(flag: Boolean); message 'setAutoenablesItems:';
    function autoenablesItems: Boolean; message 'autoenablesItems';
    procedure setPreferredEdge(edge: NSRectEdge); message 'setPreferredEdge:';
    function preferredEdge: NSRectEdge; message 'preferredEdge';
    procedure addItemWithTitle(title_: NSString); message 'addItemWithTitle:';
    procedure addItemsWithTitles(itemTitles_: NSArray); message 'addItemsWithTitles:';
    procedure insertItemWithTitle_atIndex(title_: NSString; index: NSInteger); message 'insertItemWithTitle:atIndex:';
    procedure removeItemWithTitle(title_: NSString); message 'removeItemWithTitle:';
    procedure removeItemAtIndex(index: NSInteger); message 'removeItemAtIndex:';
    procedure removeAllItems; message 'removeAllItems';
    function itemArray: NSArray; message 'itemArray';
    function numberOfItems: NSInteger; message 'numberOfItems';
    function indexOfItem(item: NSMenuItem): NSInteger; message 'indexOfItem:';
    function indexOfItemWithTitle(title_: NSString): NSInteger; message 'indexOfItemWithTitle:';
    function indexOfItemWithTag(tag_: NSInteger): NSInteger; message 'indexOfItemWithTag:';
    function indexOfItemWithRepresentedObject(obj: id): NSInteger; message 'indexOfItemWithRepresentedObject:';
    function indexOfItemWithTarget_andAction(target_: id; actionSelector: SEL): NSInteger; message 'indexOfItemWithTarget:andAction:';
    function itemAtIndex(index: NSInteger): NSMenuItem; message 'itemAtIndex:';
    function itemWithTitle(title_: NSString): NSMenuItem; message 'itemWithTitle:';
    function lastItem: NSMenuItem; message 'lastItem';
    procedure selectItem(item: NSMenuItem); message 'selectItem:';
    procedure selectItemAtIndex(index: NSInteger); message 'selectItemAtIndex:';
    procedure selectItemWithTitle(title_: NSString); message 'selectItemWithTitle:';
    function selectItemWithTag(tag_: NSInteger): Boolean; message 'selectItemWithTag:';
    procedure setTitle(aString: NSString); message 'setTitle:';
    function selectedItem: NSMenuItem; message 'selectedItem';
    function indexOfSelectedItem: NSInteger; message 'indexOfSelectedItem';
    procedure synchronizeTitleAndSelectedItem; message 'synchronizeTitleAndSelectedItem';
    function itemTitleAtIndex(index: NSInteger): NSString; message 'itemTitleAtIndex:';
    function itemTitles: NSArray; message 'itemTitles';
    function titleOfSelectedItem: NSString; message 'titleOfSelectedItem';
  end;

{$endif}
{$endif}
