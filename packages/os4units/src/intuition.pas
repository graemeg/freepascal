{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2016 by Free Pascal development team

    intuition.library functions for Amiga OS 4.x

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$PACKRECORDS 2}

unit intuition;

interface

uses
  exec, agraphics, utility, inputevent, timer, layers;

//***** User visible handles on objects, classes, messages
type
  Object_ = LongWord;
  PObject_ = ^Object_;
  PPObject_ = ^PObject_;
  ClassID = STRPTR;

// you can use this type to point to a 'generic' message, in the object-oriented programming parlance.
// Based on the value of 'MethodID', you dispatch to processing for the various message types.
// The meaningful parameter packet structure definitions are defined below.

  PMsg = ^TMsg;
  TMsg = record
    MethodID: LongWord;
    // method-specific data follows, some examples below
  end;

//**** IntuiText
// IntuiText is a series of strings that start with a screen location
// (always relative to the upper-left corner of something) and then the text of the string.  The text is null-terminated.
type
  PIntuiText = ^TIntuiText;
  TIntuiText = record
    FrontPen: Byte;
    BackPen: Byte;         // the pen numbers for the rendering
    DrawMode: Byte;        // the mode for rendering the text
    LeftEdge: SmallInt;    // relative start location for the text
    TopEdge: SmallInt;     // relative start location for the text
    ITextFont: PTextAttr;  // if NULL, you accept the default
    IText: STRPTR;         // pointer to null-terminated text
    NextText: PIntuiText;  // continuation to TxWrite another text
  end;

//**** Border
// Data type Border, used for drawing a series of lines which is intended for use as a border drawing, but which may, in fact, be used to render any
//  arbitrary vector shape. The routine DrawBorder sets up the RastPort with the appropriate variables, then does a Move to the first coordinate, then does Draws
// to the subsequent coordinates. After all the Draws are done, if NextBorder is non-zero we call DrawBorder recursively
type
  PBorder = ^TBorder;
  TBorder = record
    LeftEdge: SmallInt;
    TopEdge: SmallInt;     // initial offsets from the origin
    FrontPen: Byte;
    BackPen: Byte;       // pens numbers for rendering
    DrawMode: Byte;      // mode for rendering
    Count: Shortint;     // number of XY pairs
    XY: Pointer;         // vector coordinate pairs rel to LeftTop
    NextBorder: PBorder; // pointer to any other Border too
  end;

//**** MenuItem
type
  PMenuItem = ^TMenuItem;
  TMenuItem = record
    NextItem: PMenuItem;    // pointer to next in chained list
    LeftEdge: SmallInt;
    TopEdge: SmallInt;      // position of the select box
    Width: SmallInt;
    Height: SmallInt;       // dimensions of the select box
    Flags: Word;            // see the defines below
    MutualExclude: LongInt; // set bits mean this item excludes that
    ItemFill: APTR;         // points to Image, IntuiText, or nil
    // when this item is pointed to by the cursor and the items highlight mode HIGHIMAGE is selected, this alternate image will be displayed
    SelectFill: APTR;       // points to Image, IntuiText, or nil
    Command: Char;          // only if appliprog sets the COMMSEQ flag
    SubItem: PMenuItem;     // if non-zero, DrawMenu shows "->"
    // The NextSelect field represents the menu number of next selected item (when user has drag-selected several items)
    NextSelect: Word;
  end;

const
// FLAGS SET BY THE APPLIPROG
  CHECKIT     = $0001; // whether to check this item if selected
  ITEMTEXT    = $0002; // set if textual, clear if graphical item
  COMMSEQ     = $0004; // set if there's an command sequence
  MENUTOGGLE  = $0008; // set to toggle the check of a menu item
  ITEMENABLED = $0010; // set if this item is enabled
  SUBMENU     = $0200; // set to get standard submenu indicator (V50)

// these are the SPECIAL HIGHLIGHT FLAG state meanings
  HIGHFLAGS   = $00C0; // see definitions below for these bits
  HIGHIMAGE   = $0000; // use the user's "select image"
  HIGHCOMP    = $0040; // highlight by complementing the selectbox
  HIGHBOX     = $0080; // highlight by "boxing" the selectbox
  HIGHNONE    = $00C0; // don't highlight

// FLAGS SET BY BOTH APPLIPROG AND INTUITION
  CHECKED     = $0100; // if CHECKIT, then set this when selected

// FLAGS SET BY INTUITION
  BOOPSIMENU  = $0400; // item is a BOOPSI object, hence a black box
  ISDRAWN     = $1000; // this item's subs are currently drawn
  HIGHITEM    = $2000; // this item is currently highlighted
  MENUTOGGLED = $4000; // this item was already toggled

//**** Menu
type
  PMenu = ^TMenu;
  TMenu = record
    NextMenu: PMenu;      // same level
    LeftEdge: SmallInt;
    TopEdge: SmallInt;    // position of the select box
    Width: SmallInt;
    Height: SmallInt;     // dimensions of the select box
    Flags: Word;          // see flag definitions below
    MenuName: STRPTR;     // text for this Menu Header
    FirstItem: PMenuItem; // pointer to first in chain
    // these mysteriously-named variables are for internal use only
    JazzX, JazzY, BeatX, BeatY: SmallInt;
  end;

const
// FLAGS SET BY BOTH THE APPLIPROG AND INTUITION }
  MENUENABLED = $0001; // whether or not this menu is enabled

// Menu class attributes
  MA_Dummy     = TAG_USER + $440000;
  MA_Type      = MA_Dummy + 1;  // (LongWord) Type of a menuclass object. Defaults to T_ROOT.
  MA_Label     = MA_Dummy + 2;  // (STRPTR) Label of the object, which can be either a menu title or a menu (sub-)item text, depending on the object's type.
  MA_Level     = MA_Dummy + 3;  // (LongInt) How deep the object is in the menu tree.
  MA_Separator = MA_Dummy + 4;  // (BOOL) If TRUE, declares the object as a separator bar between two items.
  MA_ID        = MA_Dummy + 5;  // (LongWord) Menu item ID assigned by the application.
  MA_Key       = MA_Dummy + 6;  // (STRPTR) Single-character keyboard shortcut or full command string for the menu item object.
  MA_Image     = MA_Dummy + 7;  // (PImage) An Intuition image, either traditional or BOOPSI, to be displayed at the left side of (or in place of) the menu item label.
  MA_Disabled  = MA_Dummy + 8;  // (BOOL) If TRUE, the menu title or item will appear in a disabled look and won't be selectable.
  MA_Toggle    = MA_Dummy + 9;  // (BOOL) If TRUE, the item represents an option which can be switched on or off.
  MA_MX        = MA_Dummy + 10; // (LongWord) If non-zero, the item represents an option which is mutually exclusive with certain other options
  MA_Selected  = MA_Dummy + 11; // (BOOL) The state of a toggle or mutual exclude item; if TRUE, the item is checked (on), otherwise it is unchecked (off).
  MA_UserData  = MA_Dummy + 12; // (LongWord) Application-specific user data which can be associated to a menu or menu item object and read back at any time. Defaults to zero.
  MA_Parent    = MA_Dummy + 13; // (PObject_) The parent of an object in the menu tree; this will be the menu root for a menu title,
  MA_EvenSize  = MA_Dummy + 14; // (BOOL) If TRUE, all children (items or sub-items) of the object will have the same height
  MA_Hidden    = MA_Dummy + 15; // (BOOL) If TRUE, this object will not appear at menu display time.
  MA_MenuHelpID= MA_Dummy + 16; // (LongWord) ID number of the menu item or menu title for which the user requested help during the last menu session.
  MA_LastSelected = MA_Dummy + 17; // (LongWord) Index of the most recently selected child (item or sub-item) of the object.
  MA_MenuPosX  = MA_Dummy + 18; // (LongInt) Left edge of the main panel of a context menu. Ignored if the menu tree is not used as a context menu.
  MA_MenuPosY  = MA_Dummy + 19; // (LongInt) Top edge of the main panel of a context menu. Ignored if the menu tree is not used as a context menu.
  MA_AddChild  = MA_Dummy + 50; // (PObject_) An object which is to become a child of the current object.
  MA_RemoveChild = MA_Dummy + 51; // (PObject_) An object which is to be removed from the list of children of the current object.
  MA_StringHook= MA_Dummy + 80; // (PHook) A hook used for dynamic localization of menu strings.
  MA_Catalog   = MA_Dummy + 81; // (PCatalog) A catalog used for dynamic localization of menu strings. This is ignored if you don't also set MA_StringHook.
  MA_MinStringID = MA_Dummy + 82; // (LongWord) The lower bound of the range within which a value passed via MA_Label or MA_Key must lie to be considered a string ID (used for dynamic localization) rather than an actual string.
  MA_MaxStringID = MA_Dummy + 83; // (LongWord) The upper bound of the range within which a value passed via MA_Label or MA_Key must lie to be considered a string ID (used for dynamic localization) rather than an actual string.
  MA_CharSet   = MA_Dummy + 84; // (LongWord) The charset (character set) for the label and the keyboard shortcut, if any, of the current menu or menu item object.
  MA_TextAttr  = MA_Dummy + 85; // (PTextAttr) The font for the item's label (and shortcut).
  MA_EmbeddedKey = MA_Dummy + 86; // (BOOL) If TRUE, menu item labels can have a single letter prepended to them, separated with a #0 character, which will be used as the item's keyboard shortcut.
  MA_FreeImage = MA_Dummy + 87; // (BOOL) If TRUE, a BOOPSI image passed via MA_Image is automatically disposed of when the object itself is.
  MA_PickHook  = MA_Dummy + 88; // (PHook) A custom hook which is invoked during the processing of menu pick events to handle the selection of this menu item object.
  MA_HelpHook  = MA_Dummy + 89; // (PHook) A custom hook which is invoked during the processing of menu help events to handle a help request on this menu object.
  MA_ErrorCode = MA_Dummy + 98; // (PLongInt) Pointer to a longword variable to store error codes in.
  MA_ErrorTagItem = MA_Dummy + 99; // (PPTagItem) When an error occurs whilst processing the tag list passed to OM_NEW, OM_SET or MM_NEWMENU, you can have a pointer to the item that caused the error passed back via the MA_ErrorTagItem tag.
// Possible values for MA_Type.
   T_ROOT  = -1; // The menu tree root
   T_MENU  = 0;  // A menu
   T_ITEM  = 1;  // A menu item (or sub-item)
  // MA_Label should be a text string, or the special constant ML_SEPARATOR, to get a separator bar.
  // The latter is equivalent to omitting MA_Label and passing MA_Separator, TRUE instead.
  ML_SEPARATOR = STRPTR(not 0);
  // This means "no menu selection" and must never be passed as a value for the MA_ID attribute.
  NO_MENU_ID = 0;
  // You can OR these to the value of MA_MenuPosX or MA_MenuPosY (if it is non-negative) to alter the way it gets interpreted by Intuition.
  CMENU_RIGHT  = $10000000; // Value is right edge, not left edge
  CMENU_BOTTOM = $10000000; // Value is bottom edge, not top edge
  CMENU_CENTER = $20000000; // Value is center, not left/top edge
  // Special value for the MA_MenuPosX and MA_MenuPosY attributes, meaning "centered relative to mouse pointer".
  CMENU_CENTER_MOUSE = $2FFFFFFF;
// Menu method identifiers
  MM_FINDID     = 4001; // Return address of menu object in tree with given ID
  MM_SCAN       = 4002; // Do a recursive scan of menu tree with custom hook
  MM_NEWMENU    = 4003; // Build a whole menu (sub-)tree from a tag list
  MM_DELETEMENU = 4004; // Free menu (sub-)tree built by MM_NEWMENU
  MM_NEXTCHILD  = 4005; // Get address of next child of a menu object
  MM_SETSTATE   = 4006; // Change checked/disabled attributes of item with given ID
  MM_GETSTATE   = 4007; // Read checked/disabled attributes of item with given ID
  MM_NEXTSELECT = 4009; // Get next ID in menu item selection chain
  MM_HANDLEPICK = 4011; // Browse through menu selections, invoking pick hooks
  MM_HANDLEHELP = 4012; // Return menu help ID or invoke help hook if non-nil
  MM_SETUP      = 4013; // Prepare a menu (sub-)tree for display; normally not needed
  MM_CLEANUP    = 4014; // Release all resources allocated during menu setup

type
// Parameter "messages" passed to menu class methods
// MM_FINDID - Return the address of the menuclass object having the specified ID, if it exists in the menu tree, otherwise nil.
  PmpFindID = ^TmpFindID;
  TmpFindID = record
    MethodID: LongWord;
    mpfi_Reserved: LongWord;
    mpfi_ID: LongWord;
  end;
// MM_SCAN - Do a recursive scan of the menu tree. For each object in the tree, the specified hook is
// invoked on it and is passed a MenuScanMessage structure (see below) as message.
  PmpScan = ^TmpScan;
  TmpScan = record
    MethodID: LongWord;
    mps_Reserved: LongWord;
    mps_Hook: PHook;
    mps_Args: array[0..3] of LongWord
  end;
// MM_NEWMENU - Build a whole menu (sub-)tree from a compact tag-based description, with less typing,
// more readability, greater efficiency and better error checking than doing so through a series of nested NewObject() calls.
  PmpNewMenu = ^TmpNewMenu;
  TmpNewMenu = record
    MethodID: LongWord;
    mpnm_Reserved: LongWord;
    mpnm_AttrList: PTagItem;
  end;
// MM_DELETEMENU - Free a whole menu (sub-)tree that was built via MM_NEWMENU.
  PmpDeleteMenu = ^TmpDeleteMenu;
  TmpDeleteMenu = record
    MethodID: LongWord;
    mpdm_Reserved: LongWord;
  end;
// MM_NEXTCHILD - Return the object which comes next after the passed one in the children list of the menuclass object it's invoked on.
// If nil is passed for mpnc_Current, the first child in the list is returned; if mpnc_Current is the last child in the list, NULL is returned.
// This method can be used in a loop to browse through all children of a given object.
  PmpNextChild = ^TmpNextChild;
  TmpNextChild = record
    MethodID: LongWord;
    mpnc_Reserved: LongWord;
    mpnc_Current: PObject_;
  end;
// MM_SETSTATE - Change the state of the "checked" and/or "disabled" attribute of the menuclass object having the specified ID. The apply mask
// defines which attributes are to be changed, and the state mask what they should be changed to; they can be a combination of the MS_CHECKED and MS_DISABLED bits.
  PmpSetState = ^TmpSetState;
  TmpSetState = record
    MethodID: LongWord;
    mpss_Reserved: LongWord;
    mpss_ID: LongWord;
    mpss_ApplyMask: LongWord;
    mpss_StateMask: LongWord;
  end;
// MM_GETSTATE - Examine the state of the "checked" and "disabled" attributes of the menuclass object
// having the specified ID. The result is a bit mask with a combination of MS_CHECKED and MS_DISABLED.
  PmpGetState = ^TmpGetState;
  TmpGetState = record
    MethodID: LongWord;
    mpgs_Reserved: LongWord;
    mpgs_ID: LongWord;
  end;
// MM_NEXTSELECT - Get the ID of the next menu item picked by the user during the most recent menu selection operation.
  PmpNextSelect = ^TmpNextSelect;
  TmpNextSelect = record
    MethodID: LongWord;
    mpns_Reserved: LongWord;
    mpns_CurrentID: LongWord;
  end;
// MM_HANDLEPICK - Browse through the menu selection list and invoke the pick hook of each selected item that has one.
// Every time an item with a nil pick hook is encountered in the list, stop and return its ID.
  PmpHandleEvent = ^TmpHandleEvent;
  TmpHandleEvent = record
    MethodID: LongWord;
    mphe_Reserved: LongWord;
    mphe_CurrentID: LongWord;
    mphe_Window: Pointer;
    mphe_UserData: APTR;
  end;
// MM_SETUP - Prepare a menu (sub-)tree for display by allocating any needed resources and computing geometric properties.
  PmpSetup = ^TmpSetup;
  TmpSetup = record
    MethodID: LongWord;
    mps_Reserved: LongWord;
    mps_Initial: LongWord;
  end;

const
// Special control tags for MM_NEWMENU
  NM_Dummy    = TAG_USER + $450000;
  NM_Menu     = NM_Dummy + 1; // (STRPTR) Used to add a new menu object to the menu root.
  NM_Item     = NM_Dummy + 2; // (STRPTR) Used to add a new menu item object to the current menu.
  NM_SubItems = NM_Dummy + 3; // (LongWord) Used to begin and end a list of sub-items of the current menu item.
// Values for NM_SubItems
  SI_BEGIN = 1; // This begins a sub-item list
  SI_END   = 0; // This ends a sub-item list
// Possible state flags for MM_SETSTATE/MM_GETSTATE
  MS_CHECKED  = 1;
  MS_DISABLED = 2;
type
// The string hook specified via MA_StringHook receives this message.
  PMenuStringMessage = ^TMenuStringMessage;
  TMenuStringMessage = record
    StructSize: LongWord; // For future expansion
    StringID: LongWord;   // The string ID number
    Catalog: Pointer;     // (PCatalog) Catalog pointer, may be nil
    CharSet: LongWord;    // Charset number, may be zero
  end;
// The menu scan hook passed to MM_SCAN receives this message. The four Args variables hold the same values which
// were passed upon method invocation; they can be used to feed your custom arguments to the hook.
  PMenuScanMessage = ^TMenuScanMessage;
  TMenuScanMessage = record
    StructSize: LongWord;          // For future expansion
    Level: LongInt;                // How deep we are in the menu tree
    Args: array[0..3] of LongWord; // Custom arguments
  end;

// Pick hooks and help hooks specified via MA_PickHook and MA_HelpHook receive this message. The UserData field contains any
// context information that was passed by the application upon MM_HANDLEPICK or MM_HANDLEHELP method invocation.
  PMenuEventMessage = ^TMenuEventMessage;
  TMenuEventMessage = record
    StructSize: LongWord; // For future expansion
    EventType: LongWord;  // ET_MENUPICK or ET_MENUHELP
    Window: Pointer;      // (PWindow) Event window pointer
    UserData: APTR;       // Custom data pointer
  end;
const
// Values for MenuEventMessage.EventType
  ET_MENUPICK = 0;
  ET_MENUHELP = 1;
// Menu-specific error codes MM_NEWMENU can return (via MA_ErrorCode) in addition to standard DOS error codes.
  ERROR_TITLE_OUT_OF_PLACE       = 4001; // A menu title was found at the wrong position in the tree.
  ERROR_SUBITEMS_WITHOUT_PARENT  = 4002; // A sub-item list was started without having a parent item first.
  ERROR_ITEM_WITHOUT_ID          = 4003; // No ID was set for a menu item or sub-item which needs one.
  ERROR_SEPARATOR_AT_TITLE_LEVEL = 4004; // A separator was inserted before the first menu title.
  ERROR_BAD_ACTION_ON_ROOT       = 4005; // An unsupported attribute was specified for a menu root object.
  ERROR_BAD_ACTION_ON_MENU       = 4006; // An unsupported attribute was specified for a menu object.
  ERROR_BAD_ACTION_ON_ITEM       = 4007; // An unsupported attribute was specified for a menu item object.
  ERROR_BAD_ACTION_ON_SEPARATOR  = 4008; // An unsupported attribute was specified for a separator.
  ERROR_ITEM_ID_IS_ZERO          = 4009; // An ID of zero was specified for a menu item or sub-item.
  ERROR_COMM_KEY_AT_TITLE_LEVEL  = 4010; // A command key was specified for a menu title.

  SNA_PubName  = TAG_USER + $01; // public screen to watch, nil for all screens
  SNA_Notify   = TAG_USER + $02; // see below
  SNA_UserData = TAG_USER + $03; // for your use
  SNA_SigTask  = TAG_USER + $04; // task to signal
  SNA_SigBit   = TAG_USER + $05; // signal bit
  SNA_MsgPort  = TAG_USER + $06; // send message to this port
  SNA_Priority = TAG_USER + $07; // priority of your request
  SNA_Hook     = TAG_USER + $08;
// SNA_Notify (all unassigned bits are reserved for system use)
  SNOTIFY_AFTER_OPENSCREEN   = 1 shl 0;  // screen has been opened
  SNOTIFY_BEFORE_CLOSESCREEN = 1 shl 1;  // going to close screen
  SNOTIFY_AFTER_OPENWB       = 1 shl 2;  // Workbench is open
  SNOTIFY_BEFORE_CLOSEWB     = 1 shl 3;  // Workbench is going to be closed
  SNOTIFY_AFTER_OPENWINDOW   = 1 shl 4;  // new window
  SNOTIFY_BEFORE_CLOSEWINDOW = 1 shl 5;  // window is going to be closed
  SNOTIFY_PUBSCREENSTATE     = 1 shl 6;  // PubScreenState()
  SNOTIFY_LOCKPUBSCREEN      = 1 shl 7;  // LockPubScreen()
  SNOTIFY_SCREENDEPTH        = 1 shl 8;  // ScreenDepth()
  SNOTIFY_AFTER_CLOSESCREEN  = 1 shl 9;  // notify after CloseScreen()
  SNOTIFY_AFTER_CLOSEWINDOW  = 1 shl 10; // dto. CloseWindow()
  SNOTIFY_BEFORE_OPENSCREEN  = 1 shl 11; // notify before OpenScreen()
  SNOTIFY_BEFORE_OPENWINDOW  = 1 shl 12; // dto. OpenWindow()
  SNOTIFY_BEFORE_OPENWB      = 1 shl 13; // like OPENSCREEN
  SNOTIFY_AFTER_CLOSEWB      = 1 shl 14; // like CLOSESCREEN
  SNOTIFY_WAIT_REPLY         = 1 shl 15; // wait for reply before  taking action
  SNOTIFY_UNLOCKPUBSCREEN    = 1 shl 16; // UnlockPubScreen()
type
  PScreenNotifyMessage = ^TScreenNotifyMessage;
  TScreenNotifyMessage = record
    snm_Message: TMessage; // embedded message
    snm_Class: LongWord;   // see above
    snm_Code: LongWord;
    snm_Object: APTR;      // either a pointer to struct Window or struct Screen (READ-ONLY). For SNRF_#?PUBSCREEN this the name of the public screen
    snm_UserData: APTR;    // SNA_UserData
    snm_Request: APTR;     // pointer returned by StartScreenNotify()
    snm_Reserved: array[0..4] of LongWord; // don't touch!
  end;

// *** The Intuition plugin interface
  PGUIPlugin = ^TGUIPlugin;
  TGUIPlugin = record
    Node: TNode;        // Reserved, don't use
    Version: LongWord;  // Version of the plugin
    Type_: LongWord;    // Type of plugin
    Attrs: LongWord;    // Type-specific attributes
    Flags: LongWord;    // Additional information
    AttrList: PTagItem; // Optional list of GUI attributes
    Reserved: array[0..3] of LongWord; // For future expansion
    // Plugin-specific fields follow here
  end;

const
// Plugin attributes (flags)
  PA_INTERNAL = $10000000; // Plugin is implemented internally by Intuition

// *** Rendering hooks: common structure and definitions
// Possible return values from a rendering hook
  RCB_OK      = 0; // Hook understands this message type
  RCB_UNKNOWN = 1; // Hook does not understand this message
type
// Structure of messages for rendering hooks: the object is context-specific.
  PRenderMsg = ^TRenderMsg;
  TRenderMsg = record
    rm_MethodID: LongWord;  // Type of rendering to perform
    rm_RastPort: PRastPort; // Where to render to
    rm_DrawInfo: Pointer;   // (PDrawInfo) Context information
    rm_Bounds: TRectangle;  // Limits of where to render
    rm_State: LongWord;     // How to render
    rm_IAddress: APTR;      // Subsystem-specific data
    rm_Flags: LongWord;     // Subsystem-specific flags
    rm_TagList: PTagItem;   // Additional information
  end;

//**** Gadget
  PGadget = ^TGadget;
  TGadget = record
    NextGadget: PGadget;         // next gadget in the list
    LeftEdge, TopEdge: SmallInt; // "hit box" of gadget
    Width, Height: SmallInt;     // "hit box" of gadget
    Flags: Word;      // see below for list of defines
    Activation: Word; // see below for list of defines
    GadgetType: Word; // see below for defines
    // appliprog can specify that the Gadget be rendered as either as Border or an Image.
    // This variable points to which (or equals nil if there's nothing to be rendered about this Gadget)
    GadgetRender: APTR;
    // appliprog can specify "highlighted" imagery rather than algorithmic this can point to either Border or Image data
    SelectRender: APTR;
    GadgetText: PIntuiText;   // text for this gadget

    // by using the MutualExclude word, the appliprog can describe which gadgets mutually-exclude which other ones.  The bits
    // in MutualExclude correspond to the gadgets in object containing the gadget list.  If this gadget is selected and a bit is set
    // in this gadget's MutualExclude and the gadget corresponding to that bit is currently selected (e.g. bit 2 set and gadget 2
    // is currently selected) that gadget must be unselected. Intuition does the visual unselecting (with checkmarks) and
    // leaves it up to the program to unselect internally
    MutualExclude: LongInt;   // Obsolete

    // pointer to a structure of special data required by Proportional, String and LongInt Gadgets
    SpecialInfo: APTR;
    GadgetID: Word;           // user-definable ID field
    UserData: APTR;           // ptr to general purpose User data (ignored by In)
  end;

  PExtGadget = ^TExtGadget;
  TExtGadget = record
    // The first fields match struct Gadget exactly
    NextGadget: PExtGadget;  // Matches struct Gadget
    LeftEdge, TopEdge,       // Matches struct Gadget
    Width, Height: SmallInt; // Matches struct Gadget
    Flags,                   // Matches struct Gadget
    Activation,              // Matches struct Gadget
    GadgetType: Word;        // Matches struct Gadget
    GadgetRender,            // Matches struct Gadget
    SelectRender: APTR;      // Matches struct Gadget
    GadgetText: PIntuiText;  // Matches struct Gadget
    MutualExclude: LongInt;  // Matches struct Gadget
    SpecialInfo: APTR;       // Matches struct Gadget
    GadgetID: Word;          // Matches struct Gadget
    UserData: APTR;          // Matches struct Gadget

    // These fields only exist under V39 and only if GFLG_EXTENDED is set
    MoreFlags: LongWord;     // see GMORE_ flags below
    BoundsLeftEdge,          // Bounding extent for gadget, valid
    BoundsTopEdge,           // only if GMORE_BOUNDS is set.  The
    BoundsWidth,             // GFLG_RELxxx flags affect these
    BoundsHeight: SmallInt;  // coordinates as well.
 end;

const
// --- Gadget.Flags values
// combinations in these bits describe the highlight technique to be used
  GFLG_GADGHIGHBITS = $0003;
  GFLG_GADGHCOMP    = $0000; // Complement the select box
  GFLG_GADGHBOX     = $0001; // Draw a box around the image
  GFLG_GADGHIMAGE   = $0002; // Blast in this alternate image
  GFLG_GADGHNONE    = $0003; // don't highlight

  GFLG_GADGIMAGE    = $0004; // set IF GadgetRender AND SelectRender point to an Image structure, clear if they point to Border structures

// combinations in these next two bits specify to which corner the gadget's Left & Top coordinates are relative.  If relative to Top/Left,
// these are "normal" coordinates (everything is relative to something in this universe).
//
// Gadget positions and dimensions are relative to the window or requester which contains the gadget
  GFLG_RELBOTTOM = $0008; // vert. pos. is relative to bottom edge
  GFLG_RELRIGHT  = $0010; // horiz. pos. is relative to right edge
  GFLG_RELWIDTH  = $0020; // width is relative to req/window
  GFLG_RELHEIGHT = $0040; // height is relative to req/window

// New for V39: GFLG_RELSPECIAL allows custom gadget implementors to make gadgets whose position and size depend in an arbitrary way
// on their window's dimensions.  The GM_LAYOUT method will be invoked for such a gadget (or any other GREL_xxx gadget) at suitable times,
// such as when the window opens or the window's size changes.
  GFLG_RELSPECIAL = $4000; // custom gadget has special relativity. Gadget box values are absolutes, but can be changed via the GM_LAYOUT method.
  GFLG_SELECTED   = $0080; // you may initialize AND look at this

// the GFLG_DISABLED flag is initialized by you and later set by Intuition according to your calls to On/OffGadget().  It specifies whether or not
// this Gadget is currently disabled from being selected
  GFLG_DISABLED    = $0100;

// These flags specify the type of text field that Gadget.GadgetText points to.  In all normal (pre-V36) gadgets which you initialize
// this field should always be zero.  Some types of gadget objects created from classes will use these fields to keep track of
// types of labels/contents that different from IntuiText, but are stashed in GadgetText.
  GFLG_LABELMASK   = $3000;
  GFLG_LABELITEXT  = $0000; // GadgetText points to IntuiText
  GFLG_LABELSTRING = $1000; // GadgetText points to (PByte)
  GFLG_LABELIMAGE  = $2000; // GadgetText points to Image (object)

// New for V37: GFLG_TABCYCLE
  GFLG_TABCYCLE    = $0200; //(string OR custom) gadget participates in cycling activation with Tab or Shift-Tab

// New for V37: GFLG_STRINGEXTEND.  We discovered that V34 doesn't properly ignore the value we had chosen for the Gadget->Activation flag
// GACT_STRINGEXTEND.  NEVER SET THAT FLAG WHEN RUNNING UNDER V34. The Gadget->Flags bit GFLG_STRINGEXTEND is provided as a synonym which is
// safe under V34, and equivalent to GACT_STRINGEXTEND under V37. (Note that the two flags are not numerically equal)
  GFLG_STRINGEXTEND = $0400; // this String Gadget has StringExtend

// New for V39: GFLG_IMAGEDISABLE.  This flag is automatically set if the custom image of this gadget knows how to do disabled rendering
// (more specifically, if its IA_SupportsDisable attribute is TRUE). Intuition uses this to defer the ghosting to the image-class,
// instead of doing it itself (the old compatible way). Do not set this flag yourself - Intuition will do it for you.
 GFLG_IMAGEDISABLE = $0800; // Gadget's image knows how to do disabled rendering

// New for V39:  If set, this bit means that the Gadget is actually a struct ExtGadget, with new fields and flags.  All V39 boopsi
// gadgets are ExtGadgets.  Never ever attempt to read the extended fields of a gadget if this flag is not set.
  GFLG_EXTENDED    = $8000; // Gadget is extended

// ---  Gadget.Activation flag values
// Set GACT_RELVERIFY if you want to verify that the pointer was still over the gadget when the select button was released.  Will cause
// an IDCMP_GADGETUP message to be sent if so.
  GACT_RELVERIFY    = $0001;

// the flag GACT_IMMEDIATE, when set, informs the caller that the gadget was activated when it was activated.  This flag works in conjunction with
//   the GACT_RELVERIFY flag
  GACT_IMMEDIATE    = $0002;

// the flag GACT_ENDGADGET, when set, tells the system that this gadget, when selected, causes the Requester to be ended.  Requesters
// that are ended are erased and unlinked from the system.
  GACT_ENDGADGET    = $0004;

// the GACT_FOLLOWMOUSE flag, when set, specifies that you want to receive reports on mouse movements while this gadget is active.
// You probably want to set the GACT_IMMEDIATE flag when using GACT_FOLLOWMOUSE, since that's the only reasonable way you have of
// learning why Intuition is suddenly sending you a stream of mouse movement events.  If you don't set GACT_RELVERIFY, you'll get at
// least one Mouse Position event.
  GACT_FOLLOWMOUSE = $0008;

// if any of the BORDER flags are set in a Gadget that's included in the Gadget list when a Window is opened, the corresponding Border will
// be adjusted to make room for the Gadget
  GACT_RIGHTBORDER = $0010;
  GACT_LEFTBORDER  = $0020;
  GACT_TOPBORDER   = $0040;
  GACT_BOTTOMBORDER= $0080;
  GACT_BORDERSNIFF = $8000; // neither set nor rely on this bit

  GACT_TOGGLESELECT= $0100; // this bit for toggle-select mode
  GACT_BOOLEXTEND  = $2000; // this Boolean Gadget has a BoolInfo

// should properly be in StringInfo, but aren't
  GACT_STRINGLEFT  = $0000; // NOTE WELL: that this has value zero
  GACT_STRINGCENTER= $0200;
  GACT_STRINGRIGHT = $0400;
  GACT_LONGINT     = $0800; // this String Gadget is for Long Ints
  GACT_ALTKEYMAP   = $1000; // this String has an alternate keymap
  GACT_STRINGEXTEND= $2000; // this String Gadget has StringExtend
  // NOTE: NEVER SET GACT_STRINGEXTEND IF YOU
  //  ARE RUNNING ON LESS THAN V36!  SEE GFLG_STRINGEXTEND (ABOVE) INSTEAD

  GACT_ACTIVEGADGET = $4000; // this gadget is "active".  This flag is maintained by Intuition, and you
                             // cannot count on its value persisting while you do something on your program's
                             // task.  It can only be trusted by people implementing custom gadgets

// note $8000 is used above (GACT_BORDERSNIFF); all Activation flags defined

// --- GADGET TYPES
// These are the Gadget Type definitions for the variable GadgetType gadget number type MUST start from one.
// NO TYPES OF ZERO ALLOWED. first comes the mask for Gadget flags reserved for Gadget typing
  GTYP_GADGETTYPE = $FC00; // all Gadget Global Type flags (padded)
  GTYP_SYSGADGET  = $8000; // 1 = Allocated by the system, 0 = by app.
  GTYP_SCRGADGET  = $4000; // 1 = ScreenGadget, 0 = WindowGadget
  GTYP_GZZGADGET  = $2000; // 1 = for WFLG_GIMMEZEROZERO borders
  GTYP_REQGADGET  = $1000; // 1 = this is a Requester Gadget
// GTYP_SYSGADGET means that Intuition ALLOCATED the gadget.
// GTYP_SYSTYPEMASK is the mask you can apply to tell what type of system-gadget it is.  The possible types follow.
  GTYP_SYSTYPEMASK = $00F0;
// These definitions describe system gadgets in V36 and higher:
  GTYP_SIZING    = $0010; // Window sizing gadget
  GTYP_WDRAGGING = $0020; // Window drag bar
  GTYP_SDRAGGING = $0030; // Screen drag bar
  GTYP_WDEPTH    = $0040; // Window depth gadget
  GTYP_SDEPTH    = $0050; // Screen depth gadget
  GTYP_WZOOM     = $0060; // Window zoom gadget
  GTYP_SUNUSED   = $0070; // Unused screen gadget
  GTYP_CLOSE     = $0080; // Window close gadget
  GTYP_WUPFRONT  = GTYP_WDEPTH;  // Window to-front gadget
  GTYP_SUPFRONT  = GTYP_SDEPTH;  // Screen to-front gadget
  GTYP_WDOWNBACK = GTYP_WZOOM;   // Window to-back gadget
  GTYP_SDOWNBACK = GTYP_SUNUSED; // Screen to-back gadget
// GTYP_GTYPEMASK is a mask you can apply to tell what class of gadget this is.  The possible classes follow.
  GTYP_GTYPEMASK    = $0007;
  GTYP_BOOLGADGET   = $0001;
  GTYP_GADGET0002   = $0002;
  GTYP_PROPGadget   = $0003;
  GTYP_STRGADGET    = $0004;
  GTYP_CUSTOMGADGET = $0005;

  GTYP_TBARGADGET   = $0200;

// New for V39.  Gadgets which have the GFLG_EXTENDED flag set are actually ExtGadgets, which have more flags.  The GMORE_xxx
// identifiers describe those flags.  For GMORE_SCROLLRASTER, see important information in the ScrollWindowRaster() autodoc.
// NB: GMORE_SCROLLRASTER must be set before the gadget is added to a window.
  GMORE_BOUNDS           = $00000001; // ExtGadget has valid Bounds
  GMORE_GADGETHELP       = $00000002; // This gadget responds to gadget help
  GMORE_SCROLLRASTER     = $00000004; // This (custom) gadget uses ScrollRaster
  GMORE_NOFILTERMENUKEYS = $00000008; // Set this flag to make menu shortcuts work even when the gadget is active. V50.
  GMORE_HIDDEN           = $00000010; // This gadgets is hidden/can't be drawn. V50.
  GMORE_PRIVATE          = $00000020; // Reserved for system use. V50.
  GMORE_PRIVATE2         = $00000040; // Reserved for system use. V51.
  GMORE_NOFILTERWHEEL    = $00000080; // Your window will receive mouse wheel events even when the gadget is active. V51.
  GMORE_NOCMENUOFFLOAD   = $00000100; // Reserved for system use. V54.

// Domain types recognized by GadgetBox()
  GBD_WINDOW    = 0; // Domain is a Window
  GBD_BOX       = 1; // Domain is an IBox
  GBD_RECTANGLE = 2; // Domain is a Rectangle
  GBD_GINFO     = 3; // Domain is a GadgetInfo (V51)

// Flags for GadgetBox()
  GBF_BOUNDS   = $00000001; // Return bounding box, not hit box
  GBF_MAKERECT = $00000002; // Return a Rectangle, not an IBox

//**** BoolInfo

// This is the special data needed by an Extended Boolean Gadget Typically this structure will be pointed to by the Gadget field SpecialInfo
type
  PBoolInfo = ^TBoolInfo;
  TBoolInfo = record
    Flags: Word; // defined below
    Mask: PWord; // bit mask for highlighting and selecting mask must follow the same rules as an Image
                 // plane.  It's width and height are determined by the width and height of the gadget's
                 // select box. (i.e. Gadget.Width and .Height).
    Reserved: LongWord; // set to 0
  end;

const
// set BoolInfo.Flags to this flag bit. in the future, additional bits might mean more stuff hanging off of BoolInfo.Reserved.
  BOOLMASK    = $0001; // extension is for masked gadget

//**** PropInfo
// this is the special data required by the proportional Gadget typically, this data will be pointed to by the Gadget variable SpecialInfo
type
  PPropInfo = ^TPropInfo;
  TPropInfo = record
    Flags: Word; // general purpose flag bits (see defines below)
      // You initialize the Pot variables before the Gadget is added to the system.  Then you can look here for the current settings
      // any time, even while User is playing with this Gadget.  To adjust these after the Gadget is added to the System, use
      // ModifyProp();  The Pots are the actual proportional settings, where a value of zero means zero and a value of MAXPOT means
      // that the Gadget is set to its maximum setting.
    HorizPot: Word; // 16-bit FixedPoint horizontal quantity percentage
    VertPot: Word;  // 16-bit FixedPoint vertical quantity percentage
      // the 16-bit FixedPoint Body variables describe what percentage of the entire body of stuff referred to by this Gadget is actually
      // shown at one time.  This is used with the AUTOKNOB routines, to adjust the size of the AUTOKNOB according to how much of
      // the data can be seen.  This is also used to decide how far to advance the Pots when User hits the Container of the Gadget.
      // For instance, if you were controlling the display of a 5-line Window of text with this Gadget, and there was a total of 15
      // lines that could be displayed, you would set the VertBody value to (MAXBODY / (TotalLines / DisplayLines)) = MAXBODY / 3.
      // Therefore, the AUTOKNOB would fill 1/3 of the container, and if User hits the Cotainer outside of the knob, the pot would
      // advance 1/3 (plus or minus) If there's no body to show, or the total amount of displayable info is less than the display area,
      // set the Body variables to the MAX.  To adjust these after the Gadget is added to the System, use ModifyProp();
    HorizBody: Word;  // horizontal Body
    VertBody: Word;   // vertical Body
      // these are the variables that Intuition sets and maintains
    CWidth: Word;     // Container width (with any relativity absoluted)
    CHeight: Word;    // Container height (with any relativity absoluted)
    HPotRes: Word;
    VPotRes: Word;    // pot increments
    LeftBorder: Word; // Container borders
    TopBorder: Word;  // Container borders
  end;

const
// --- FLAG BITS
  AUTOKNOB     =   $0001;  // this flag sez:  gimme that old auto-knob
// NOTE: if you do not use an AUTOKNOB for a proportional gadget, you are currently limited to using a single Image of your own
// design: Intuition won't handle a linked list of images as a proportional gadget knob.
  FREEHORIZ      =  $0002; // IF set, the knob can move horizontally
  FREEVERT       =  $0004; // IF set, the knob can move vertically
  PROPBORDERLESS =  $0008; // IF set, no border will be rendered
  KNOBHIT        =  $0100; // set when this Knob is hit
  PROPNEWLOOK    =  $0010; // set this IF you want to get the new V36 look

  KNOBHMIN      =  6;     // minimum horizontal size of the Knob
  KNOBVMIN      =  4;     // minimum vertical size of the Knob
  MAXBODY       =  $FFFF; // maximum body value
  MAXPOT        =  $FFFF; // maximum pot value

//**** StringInfo
// this is the special data required by the string Gadget typically, this data will be pointed to by the Gadget variable SpecialInfo
type
  PStringInfo = ^TStringInfo;
  TStringInfo = record
      // you initialize these variables, and then Intuition maintains them
    Buffer: STRPTR;      // the buffer containing the start and final string
    UndoBuffer: STRPTR;  // optional buffer for undoing current entry
    BufferPos: SmallInt; // character position in Buffer
    MaxChars: SmallInt;  // max number of chars in Buffer (including nil)
    DispPos: SmallInt;   // Buffer position of first displayed character
      // Intuition initializes and maintains these variables for you
    UndoPos: SmallInt;   // character position in the undo buffer
    NumChars: SmallInt;  // number of characters currently in Buffer
    DispCount: SmallInt; // number of whole characters visible in Container
    CLeft: SmallInt;     //
    CTop: SmallInt;      // topleft offset of the container
      // you can initialize this variable before the gadget is submitted to Intuition, and then examine it later to discover what LongInt
      // the user has entered (if the user never plays with the gadget, the value will be unchanged from your initial setting)
    Extension: Pointer;
    _LongInt: LongInt;
      // If you want this Gadget to use your own Console keymapping, you set the ALTKEYMAP bit in the Activation flags of the Gadget, and then
      // set this variable to point to your keymap.  If you don't set the ALTKEYMAP, you'll get the standard ASCII keymapping.
    AltKeyMap: Pointer;
  end;

//**** Requester
type
  PRequester = ^TRequester;
  TRequester = record
    OlderRequest: PRequester;
    LeftEdge, TopEdge: SmallInt; // dimensions of the entire box
    Width, Height: SmallInt;     // dimensions of the entire box
    RelLeft, RelTop: SmallInt;   // for Pointer relativity offsets

    ReqGadget: PGadget;  // pointer to a list of Gadgets
    ReqBorder: PBorder;  // the box's border
    ReqText: PIntuiText; // the box's text
    Flags: Word;         // see definitions below

    // pen number for back-plane fill before draws
    BackFill: Byte;
    // Layer in place of clip rect
    ReqLayer: PLayer;
    ReqPad1: array[0..31] of Byte;

   { If the BitMap plane pointers are non-zero, this tells the system that the image comes pre-drawn (if the appliprog wants to define
     it's own box, in any shape or size it wants!);  this is OK by Intuition as long as there's a good correspondence between the image and the specified Gadgets}
    ImageBMap: PBitMap; // points to the BitMap of PREDRAWN imagery
    RWindow: Pointer;   // PWindow added.  points back to Window
    ReqImage: Pointer;  // PImage new for V36: drawn if USEREQIMAGE set
    ReqPad2: array[0..31] of Byte;
  end;

const
// FLAGS SET BY THE APPLIPROG
  POINTREL      = $0001; // if POINTREL set, TopLeft is relative to pointer for DMRequester, relative to window center for Request().
  PREDRAWN      = $0002; // set if Requester.ImageBMap points to predrawn Requester imagery
  NOISYREQ      = $0004; // if you don't want requester to filter input
  SIMPLEREQ     = $0010; // to use SIMPLEREFRESH layer (recommended)
  // New for V36
  USEREQIMAGE   = $0020; // render linked list ReqImage after BackFill but before gadgets and text
  NOREQBACKFILL = $0040; // don't bother filling requester with Requester.BackFill pen

{ FLAGS SET BY INTUITION }
  REQOFFWINDOW  = $1000; // part of one of the Gadgets was offwindow
  REQACTIVE     = $2000; // this requester is active
  SYSREQUEST    = $4000; // this requester caused by system
  DEFERREFRESH  = $8000; // this Requester stops a Refresh broadcast

// **** Image
// This is a brief image structure for very simple transfers of image data to a RastPort
type
  PImage = ^TImage;
  TImage = record
    LeftEdge: SmallInt; // starting offset relative to some origin
    TopEdge: SmallInt;  // starting offsets relative to some origin
    Width: SmallInt;    // pixel size (though data is word-aligned)
    Height: SmallInt;
    Depth: SmallInt;    // pixel sizes
    ImageData: Pointer; // pointer to the actual word-aligned bits

    // the PlanePick and PlaneOnOff variables work much the same way as the equivalent GELS Bob variables.  It's a space-saving
    // mechanism for image data.  Rather than defining the image data for every plane of the RastPort, you need define data only
    // for the planes that are not entirely zero or one.  As you define your Imagery, you will often find that most of the planes
    // ARE just as color selectors.  For instance, if you're designing a two-color Gadget to use colors two and three, and the Gadget
    // will reside in a five-plane display, bit plane zero of your imagery would be all ones, bit plane one would have data that
    // describes the imagery, and bit planes two through four would be all zeroes.  Using these flags allows you to avoid wasting all
    // that memory in this way:  first, you specify which planes you want your data to appear in using the PlanePick variable.  For
    // each bit set in the variable, the next "plane" of your image data is blitted to the display.  For each bit clear in this
    // variable, the corresponding bit in PlaneOnOff is examined. If that bit is clear, a "plane" of zeroes will be used.
    // If the bit is set, ones will go out instead.  So, for our example:
    //   Gadget.PlanePick = $02;
    //   Gadget.PlaneOnOff = $01;
    // Note that this also allows for generic Gadgets, like the System Gadgets, which will work in any number of bit planes.
    // Note also that if you want an Image that is only a filled rectangle, you can get this by setting PlanePick to zero
    // (pick no planes of data) and set PlaneOnOff to describe the pen color of the rectangle.
    PlanePick: Byte;
    PlaneOnOff: Byte;
    // if the NextImage variable is not NULL, Intuition presumes that it points to another Image structure with another Image to be rendered
    NextImage: PImage;
  end;

//**** IntuiMessage
type
  PIntuiMessage = ^TIntuiMessage;
  TIntuiMessage = record
    ExecMessage: TMessage;
    IClass: LongWord;      // the Class bits correspond directly with the IDCMP Flags, except for the special bit LONELYMESSAGE (defined below)
    Code: Word;            // the Code field is for special values like MENU number
    Qualifier: Word;       // the Qualifier field is a copy of the current InputEvent's Qualifier
    IAddress: APTR;        // IAddress contains particular addresses for Intuition functions, like the pointer to the Gadget or the Screen

    MouseX,                // when getting mouse movement reports, any event you get will have the the mouse coordinates in these variables.  the coordinates are relative
    MouseY: SmallInt;      // to the upper-left corner of your Window (GIMMEZEROZERO notwithstanding)
    Seconds,               // the time values are copies of the current system clock time.
    Micros: LongWord;      // Micros are in units of microseconds, Seconds in seconds.
    IDCMPWindow: Pointer;  // the IDCMPWindow variable will always have the Pointer of the Window of this IDCMP
    // system-use variable
    SpecialLink: PIntuiMessage;
  end;

const
// IntuiMessage tags start here.
  IMTAG_Dummy = TAG_USER + $414000;

// New for V51:
// The IAddress field of IDCMP_EXTENDEDMOUSE messages points to the following structure. Always check the Code field of the IntuiMessage
// against IMSGCODE_INTUIWHEELDATA, future versions of Intuition may introduce additional structures!
type
  PIntuiWheelData = ^TIntuiWheelData;
  TIntuiWheelData = record
    Version: Word;    // version of this structure (see below)
    Reserved: Word;   // always 0, reserved for future use
    WheelX: SmallInt; // horizontal wheel movement delta
    WheelY: SmallInt; // vertical wheel movement delta
  end;
const
  INTUIWHEELDATA_VERSION = 1; // current version of the structure above
type
// New for V51:
// The IAddress field of IDCMP_EXTENDEDKEYBOARD messages points to the following structure. Always check the Code field of the IntuiMessage
// against IMSGCODE_RAWKEYDATA, future versions of Intuition may introduce additional structures!
  PIntuiRawKeyData = ^TIntuiRawKeyData;
  TIntuiRawKeyData = record
    Version: Word;  // version of this structure (see below)
    Reserved: Word; // always 0, reserved for future use
    Class_: Word;    // copy of ie_SubClass (see IECLASS_EXTENDED_RAWKEY)
    Code: Word;     // rawkey code
    DeadKeys: TExtendedDeadKey; // deadkey information
  end;
const
  INTUIRAWKEYDATA_VERSION  = 1; // current version of the structure above

  IMSGCODE_INTUIWHEELDATA  = 1 shl 15;
  IMSGCODE_INTUIRAWKEYDATA = 1 shl 14;

  IMTAG_MenuType    = IMTAG_Dummy + 1;
  IMTAG_MenuContext = IMTAG_Dummy + 2;
  // Values for IMTAG_MenuType
  IMT_DEFAULT            = 0;
  IMT_CONTEXT_WINDOW     = 1;
  IMT_CONTEXT_GADGET_APP = 2;
  IMT_CONTEXT_GADGET_OBJ = 3;


// **** IDCMP Classes
// Please refer to the Autodoc for OpenWindow() and to the Rom Kernel Manual for full details on the IDCMP classes.
  IDCMP_SIZEVERIFY      =  $00000001;
  IDCMP_NEWSIZE         =  $00000002;
  IDCMP_REFRESHWINDOW   =  $00000004;
  IDCMP_MOUSEBUTTONS    =  $00000008;
  IDCMP_MOUSEMOVE       =  $00000010;
  IDCMP_GADGETDOWN      =  $00000020;
  IDCMP_GADGETUP        =  $00000040;
  IDCMP_REQSET          =  $00000080;
  IDCMP_MENUPICK        =  $00000100;
  IDCMP_CLOSEWINDOW     =  $00000200;
  IDCMP_RAWKEY          =  $00000400;
  IDCMP_REQVERIFY       =  $00000800;
  IDCMP_REQCLEAR        =  $00001000;
  IDCMP_MENUVERIFY      =  $00002000;
  IDCMP_NEWPREFS        =  $00004000;
  IDCMP_DISKINSERTED    =  $00008000;
  IDCMP_DISKREMOVED     =  $00010000;
  IDCMP_WBENCHMESSAGE   =  $00020000; // System use only
  IDCMP_ACTIVEWINDOW    =  $00040000;
  IDCMP_INACTIVEWINDOW  =  $00080000;
  IDCMP_DELTAMOVE       =  $00100000;
  IDCMP_VANILLAKEY      =  $00200000;
  IDCMP_INTUITICKS      =  $00400000;
  //  for notifications from "boopsi" gadgets
  IDCMP_IDCMPUPDATE     =  $00800000; // new for V36
  // for getting help key report during menu session
  IDCMP_MENUHELP        =  $01000000; // new for V36
  // for notification of any move/size/zoom/change window
  IDCMP_CHANGEWINDOW     = $02000000; // new for V36
  IDCMP_GADGETHELP       = $04000000; // new for V39
  IDCMP_EXTENDEDMOUSE    = $08000000; // new for V51
  IDCMP_EXTENDEDKEYBOARD = $10000000; // new for V51
  IDCMP_RESERVED1        = $20000000; // reserved for IDCMP extension scheme
  IDCMP_RESERVED2        = $40000000; //* reserved for IDCMP extension scheme
// NOTEZ-BIEN:            $80000000 is reserved for internal use

// the IDCMP Flags do not use this special bit, which is cleared when Intuition sends its special message to the Task, and set when Intuition
// gets its Message back from the Task.  Therefore, I can check here to find out fast whether or not this Message is available for me to send
  IDCMP_LONELYMESSAGE   =  $80000000;

//--- IDCMP Codes
// This group of codes is for the IDCMP_CHANGEWINDOW message
  CWCODE_MOVESIZE = $0000; // Window was moved and/or sized
  CWCODE_DEPTH    = $0001; // Window was depth-arranged (new for V39)
  // New for V51: these codes are for the IDCMP_(IN)ACTIVEWINDOW messages
  AWCODE_NORMAL   = $0000; // Window did actually change its activation
  AWCODE_INTERIM  = $0001; // Window state changed due to toolbox usage
  // These codes are for IDCMP_NEWSIZE messages (V53.43)
  NSCODE_FINAL    = $0000; // Final window size change
  NSCODE_INTERIM  = $0001; // Interim window size change

// This group of codes is for the IDCMP_MENUVERIFY function
  MENUHOT       =  $0001; // IntuiWants verification OR MENUCANCEL
  MENUCANCEL    =  $0002; // HOT Reply of this cancels Menu operation
  MENUWAITING   =  $0003; // Intuition simply wants a ReplyMsg() ASAP

// These are internal tokens to represent state of verification attempts shown here as a clue.
  OKOK          =  MENUHOT;    // guy didn't care
  OKABORT       =  $0004;      // window rendered question moot
  OKCANCEL      =  MENUCANCEL; // window sent cancel reply

// This group of codes is for the IDCMP_WBENCHMESSAGE messages
  WBENCHOPEN    =  $0001;
  WBENCHCLOSE   =  $0002;

// A data structure common in V36 Intuition processing
type
  PIBox = ^TIBox;
  TIBox = record
    Left: SmallInt;
    Top: SmallInt;
    Width: SmallInt;
    Height: SmallInt;
  end;

//**** Window
  PScreen = ^TScreen;
  PWindow = ^TWindow;
  TWindow = record
    NextWindow: PWindow;    // for the linked list in a screen
    LeftEdge,
    TopEdge: SmallInt;      // screen dimensions of window
    Width,
    Height: SmallInt;       // screen dimensions of window
    MouseY,
    MouseX: SmallInt;       // relative to upper-left of window
    MinWidth,
    MinHeight: SmallInt;    // minimum sizes
    MaxWidth,
    MaxHeight: Word;          // maximum sizes
    Flags: LongWord;          // see below for defines
    MenuStrip: PMenu;         // the strip of Menu headers
    Title: STRPTR;            // the title text for this window
    FirstRequest: PRequester; // all active Requesters
    DMRequest: PRequester;    // double-click Requester
    ReqCount: SmallInt;       // count of reqs blocking Window
    WScreen: PScreen;         // this Window's Screen
    RPort: PRastPort;         // this Window's very own RastPort

    // the border variables describe the window border.   If you specify GIMMEZEROZERO when you open the window, then the upper-left of the
    // ClipRect for this window will be upper-left of the BitMap (with correct offsets when in SuperBitMap mode; you MUST select GIMMEZEROZERO when
    // using SuperBitMap).  If you don't specify ZeroZero, then you save memory (no allocation of RastPort, Layer, ClipRect and associated
    // Bitmaps), but you also must offset all your writes by BorderTop, BorderLeft and do your own mini-clipping to prevent writing over the system gadgets
    BorderLeft,
    BorderTop,
    BorderRight,
    BorderBottom: Shortint;
    BorderRPort: PRastPort;
    // You supply a linked-list of Gadgets for your Window. This list DOES NOT include system gadgets.  You get the standard
    // window system gadgets by setting flag-bits in the variable Flags (see the bit definitions below)
    FirstGadget : PGadget;
    // these are for opening/closing the windows
    Parent,
    Descendant: PWindow;
    // sprite data information for your own Pointer set these AFTER you Open the Window by calling SetPointer()
    _Pointer: PWord;           // sprite data
    PtrHeight: Shortint;       // sprite height (not including sprite padding)
    PtrWidth: Shortint;        // sprite width (must be less than or equal to 16)
    XOffset,
    YOffset: Shortint;         // sprite offsets
    // the IDCMP Flags and User's and Intuition's Message Ports
    IDCMPFlags: LongWord;      // User-selected flags
    UserPort,
    WindowPort: PMsgPort;
    MessageKey: PIntuiMessage;

    DetailPen,
    BlockPen: Byte;  // for bar/border/gadget rendering
    // the CheckMark is a pointer to the imagery that will be used when rendering MenuItems of this
    // Window that want to be checkmarked if this is equal to NULL, you'll get the default imagery
    CheckMark: PImage;
    ScreenTitle: STRPTR;  // if non-nil, Screen title when Window is active
    // These variables have the mouse coordinates relative to the inner-Window of GIMMEZEROZERO Windows.  This is compared with the
    // MouseX and MouseY variables, which contain the mouse coordinates relative to the upper-left corner of the Window, GIMMEZEROZERO notwithstanding
    GZZMouseX: SmallInt;
    GZZMouseY: SmallInt;
    // these variables contain the width and height of the inner-Window of GIMMEZEROZERO Windows
    GZZWidth: SmallInt;
    GZZHeight: SmallInt;
    ExtData: PByte;
    UserData: PSmallInt;  // general-purpose pointer to User data extension
    // 11/18/85: this pointer keeps a duplicate of what Window.RPort->Layer is _supposed_ to be pointing at
    WLayer: PLayer;
    // NEW 1.2: need to keep track of the font that OpenWindow opened, in case user SetFont's into RastPort
    IFont: PTextFont;
    // (V36) another flag word (the Flags field is used up). At present, all flag values are system private. Until further notice, you may not change nor use this field.
    MoreFlags: LongWord;
    //**** Data beyond this point are Intuition Private.  DO NOT USE ****
  end;

// === Screen
  TScreen = record
    NextScreen: PScreen;         // linked list of screens
    FirstWindow: PWindow;        // linked list Screen's Windows

    LeftEdge, TopEdge: SmallInt; // parameters of the screen
    Width, Height: SmallInt;     // parameters of the screen

    MouseY, MouseX: SmallInt;    // position relative to upper-left

    Flags: Word;                 // see definitions below

    Title: STRPTR;               // null-terminated Title text
    DefaultTitle: STRPTR;        // for Windows without ScreenTitle

    // Bar sizes for this Screen and all Window's in this Screen Note that BarHeight is one less than the actual menu bar
    // height.  We're going to keep this in V36 for compatibility, although V36 artwork might use that extra pixel
    // Also, the title bar height of a window is calculated from the screen's WBorTop field, plus the font height, plus one.
    BarHeight, BarVBorder, BarHBorder, MenuVBorder, MenuHBorder: Shortint;
    WBorTop, WBorLeft, WBorRight, WBorBottom: Shortint;

    Font: PTextAttr; // this screen's default font
    // the display data structures for this Screen (note the prefix S)
    ViewPort: TViewPort;    // describing the Screen's display
    RastPort: TRastPort;    // describing Screen rendering
    BitMap: TBitMap;        // extra copy of RastPort BitMap
    LayerInfo: TLayer_Info; // each screen gets a LayerInfo
    // You supply a linked-list of Gadgets for your Screen. This list DOES NOT include system Gadgets. You get the standard system Screen Gadgets by default
    FirstGadget: PGadget;

    DetailPen, BlockPen: Byte; // for bar/border/gadget rendering
    // the following variable(s) are maintained by Intuition to support the DisplayBeep() color flashing technique
    SaveColor0: Word;
    // This layer is for the Screen and Menu bars
    BarLayer: PLayer;
    ExtData: PByte;
    UserData: PByte; // general-purpose pointer to User data extension
    //*** Data below this point are SYSTEM PRIVATE
  end;

const
// --- Flags requested at OpenWindow() time by the application ---------
  WFLG_SIZEGADGET  = $00000001; // include sizing system-gadget?
  WFLG_DRAGBAR     = $00000002; // include dragging system-gadget?
  WFLG_DEPTHGADGET = $00000004; // include depth arrangement gadget?
  WFLG_CLOSEGADGET = $00000008; // include close-box system-gadget?
  WFLG_SIZEBRIGHT  = $00000010; // size gadget uses right border
  WFLG_SIZEBBOTTOM = $00000020; // size gadget uses bottom border
// --- refresh modes ---------------------------------------------------
// combinations of the WFLG_REFRESHBITS select the refresh type
  WFLG_REFRESHBITS    = $000000C0;
  WFLG_SMART_REFRESH  = $00000000;
  WFLG_SIMPLE_REFRESH = $00000040;
  WFLG_SUPER_BITMAP   = $00000080;
  WFLG_OTHER_REFRESH  = $000000C0;
  WFLG_BACKDROP       = $00000100; // this is a backdrop window
  WFLG_REPORTMOUSE    = $00000200; // to hear about every mouse move
  WFLG_GIMMEZEROZERO  = $00000400; // a GimmeZeroZero window
  WFLG_BORDERLESS     = $00000800; // to get a Window sans border
  WFLG_ACTIVATE       = $00001000; // when Window opens, it's Active
// --- Other User Flags ------------------------------------------------
  WFLG_RMBTRAP       = $00010000; // Catch RMB events for your own
  WFLG_NOCAREREFRESH = $00020000; // not to be bothered with REFRESH
// - V36 new Flags which the programmer may specify in TNewWindow.Flags
  WFLG_NW_EXTENDED   = $00040000; // extension data provided  see TExtNewWindow
// - V39 new Flags which the programmer may specify in TNewWindow.Flags
  WFLG_NEWLOOKMENUS  = $00200000; // window has NewLook menus
// These flags are set only by Intuition.  YOU MAY NOT SET THEM YOURSELF!
  WFLG_WINDOWACTIVE  = $00002000; // this window is the active one
  WFLG_INREQUEST     = $00004000; // this window is in request mode
  WFLG_MENUSTATE     = $00008000; // Window is active with Menus on
  WFLG_WINDOWREFRESH = $01000000; // Window is currently refreshing
  WFLG_WBENCHWINDOW  = $02000000; // WorkBench tool ONLY Window
  WFLG_WINDOWTICKED  = $04000000; // only one timer tick at a time
  WFLG_VISITOR       = $08000000; // visitor window
  WFLG_ZOOMED        = $10000000; // identifies "zoom state"
  WFLG_HASZOOM       = $20000000; // windowhas a zoom gadget
// --- Other Window Values ---------------------------------------------
  DEFAULTMOUSEQUEUE  = 5; // no more mouse messages
// --- see TIntuiMessage for the IDCMP Flag definitions -------------


// === NewWindow
type

  PNewWindow = ^TNewWindow;
  TNewWindow = record
    LeftEdge, TopEdge: SmallInt; // screen dimensions of window
    Width, Height: SmallInt;     // screen dimensions of window
    DetailPen, BlockPen: Byte;   // for bar/border/gadget rendering
    IDCMPFlags: LongWord;        // User-selected IDCMP flags
    Flags: LongWord;             // see Window struct for defines
    // You supply a linked-list of Gadgets for your Window.
    //    This list DOES NOT include system Gadgets.  You get the standard
    //    system Window Gadgets by setting flag-bits in the variable Flags
    //    (see the bit definitions under the Window structure definition)
    FirstGadget: PGadget;
    // the CheckMark is a pointer to the imagery that will be used when rendering MenuItems of this Window
    // that want to be checkmarked if this is equal to nil, you'll get the default imagery
    CheckMark: PImage;
    Title: STRPTR;                // the title text for this window
    // the Screen pointer is used only if you've defined a CUSTOMSCREEN and want this Window to open in it. If so, you pass
    // the Pointer of the Custom Screen structure in this variable.  Otherwise, this variable is ignored and doesn't have to be initialized.
    Screen: PScreen;
    // WFLG_SUPER_BITMAP Window?  If so, put the address of your BitMap structure in this variable.
    // If not, this variable is ignored and doesn't have to be initialized
    BitMap: PBitMap;
    { the values describe the minimum and maximum sizes of your Windows. these matter only if you've chosen the WINDOWSIZING Gadget option,
      which means that you want to let the User to change the size of this Window.  You describe the minimum and maximum sizes that the
      Window can grow by setting these variables.  You can initialize any one these to zero, which will mean that you want to duplicate
      the setting for that dimension (if MinWidth == 0, MinWidth will be set to the opening Width of the Window).
      You can change these settings later using SetWindowLimits(). If you haven't asked for a SIZING Gadget, you don't have to initialize any of these variables.}
    MinWidth, MinHeight: SmallInt; // minimums
    MaxWidth, MaxHeight: Word;     // maximums
    // the type variable describes the Screen in which you want this Window to open.  The type value can either be CUSTOMSCREEN or one of the
    // system standard Screen Types such as WBENCHSCREEN.  See the type definitions under the Screen structure
    WType: Word;
  end;

// The following structure is the future NewWindow.  Compatibility issues require that the size of NewWindow not change.
//  Data in the common part (NewWindow) indicates the the extension fields are being used.
//  NOTE WELL: This structure may be subject to future extension. Writing code depending on its size is not allowed.
  PExtNewWindow = ^TExtNewWindow;
  TExtNewWindow = record
    LeftEdge, TopEdge: SmallInt;
    Width, Height: SmallInt;
    DetailPen, BlockPen: Byte;
    IDCMPFlags: LongWord;
    Flags: LongWord;
    FirstGadget: PGadget;
    CheckMark: PImage;
    Title: STRPTR;
    Screen: PScreen;
    BitMap: PBitMap;
    MinWidth, MinHeight: SmallInt;
    MaxWidth, MaxHeight: Word;
    // the type variable describes the Screen in which you want this Window to open.  The type value can either be CUSTOMSCREEN or one of the
    // system standard Screen Types such as WBENCHSCREEN.  See the type definitions under the Screen structure.
    // A new possible value for this field is PUBLICSCREEN, which defines the window as a 'visitor' window.  See below for additional information provided.
    Type_: Word;
    // extensions for V36 if the NewWindow Flag value WFLG_NW_EXTENDED is set, then this field is assumed to point to an array ( or chain of arrays)
    // of TagItem structures.  See also ExtNewScreen for another use of TagItems to pass optional data. see below for tag values and the corresponding data.
    Extension: PTagItem;
  end;

// The TagItem ID's (ti_Tag values) for OpenWindowTagList() follow. They are values in a TagItem array passed as extension/replacement
// values for the data in NewWindow.  OpenWindowTagList() can actually work well with a NULL NewWindow pointer.
const
  WA_Dummy     =   (TAG_USER + 99); // $80000063
// these tags simply override TNewWindow parameters
  WA_Left      = WA_Dummy + $01;
  WA_Top       = WA_Dummy + $02;
  WA_Width     = WA_Dummy + $03;
  WA_Height    = WA_Dummy + $04;
  WA_DetailPen = WA_Dummy + $05;
  WA_BlockPen  = WA_Dummy + $06;
  WA_IDCMP     = WA_Dummy + $07;
  // "bulk" initialization of TNewWindow.Flags
  WA_Flags     = WA_Dummy + $08;
  WA_Gadgets   = WA_Dummy + $09;
  WA_Checkmark = WA_Dummy + $0A;
  WA_Title     = WA_Dummy + $0B;
  // means you don't have to call SetWindowTitles after you open your window
  WA_ScreenTitle  = WA_Dummy + $0C;
  WA_CustomScreen = WA_Dummy + $0D;
  WA_SuperBitMap  = WA_Dummy + $0E;
  // also implies WFLG_SUPER_BITMAP property
  WA_MinWidth  = WA_Dummy + $0F;
  WA_MinHeight = WA_Dummy + $10;
  WA_MaxWidth  = WA_Dummy + $11;
  WA_MaxHeight = WA_Dummy + $12;
  // The following are specifications for new features
  WA_InnerWidth    = WA_Dummy + $13; // You can specify the dimensions of the interior region of your window, independent of what the border widths will be.  You probably want
  WA_InnerHeight   = WA_Dummy + $14; // to also specify WA_AutoAdjust to allow Intuition to move your window or even shrink it so that it is completely on screen.
  WA_PubScreenName = WA_Dummy + $15; // declares that you want the window to open as a visitor on the public screen whose name is pointed to by PByte ti_Data
  WA_PubScreen     = WA_Dummy + $16; // open as a visitor window on the public screen whose Pointer is in PScreen ti_Data.
                                     // To ensure that this screen remains open, you should either be the screen's owner, have a window open on the screen, or use LockPubScreen().
  WA_PubScreenFallBack = WA_Dummy + $17; // A Boolean, specifies whether a visitor window should "fall back" to the default public screen
                                           // (or Workbench) if the named public screen isn't available
  WA_WindowName = WA_Dummy + $18; // optional name for your window, useful for input helpers and diagnostic tools to identify the window
                                  // when it doesn't have a proper title. (V51)
  WA_Colors     = WA_Dummy + $19; // a ColorSpec array for colors to be set when this window is active.  This is not
                                  // implemented, and may not be, since the default values to restore would be hard to track.
                                  // We'd like to at least support per-window colors for the mouse pointer sprite.
  WA_Zoom       = WA_Dummy + $1A; // ti_Data points to an array of four SmallInt's, the initial Left/Top/Width/Height values of
                                  // the "alternate" zoom position/dimensions. It also specifies that you want a Zoom gadget
                                  // for your window, whether or not you have a sizing gadget.
  WA_MouseQueue = WA_Dummy + $1B; // ti_Data contains initial value for the mouse message backlog limit for this window.
  WA_BackFill   = WA_Dummy + $1C; // provides a "backfill hook" for your window's Layer. See layers.library/CreateUpfrontHookLayer().
  WA_RptQueue   = WA_Dummy + $1D; // initial value of repeat key backlog limit
  // These Boolean tag items are alternatives to the TNewWindow.Flags boolean flags with similar names.
  WA_SizeGadget    = WA_Dummy + $1E;
  WA_DragBar       = WA_Dummy + $1F;
  WA_DepthGadget   = WA_Dummy + $20;
  WA_CloseGadget   = WA_Dummy + $21;
  WA_Backdrop      = WA_Dummy + $22;
  WA_ReportMouse   = WA_Dummy + $23;
  WA_NoCareRefresh = WA_Dummy + $24;
  WA_Borderless    = WA_Dummy + $25;
  WA_Activate      = WA_Dummy + $26;
  WA_RMBTrap       = WA_Dummy + $27;
  WA_WBenchWindow  = WA_Dummy + $28; // PRIVATE!!
  WA_SimpleRefresh = WA_Dummy + $29; // only specify if TRUE
  WA_SmartRefresh  = WA_Dummy + $2A; // only specify if TRUE
  WA_SizeBRight    = WA_Dummy + $2B;
  WA_SizeBBottom   = WA_Dummy + $2C;
  // New Boolean properties
  WA_AutoAdjust    = WA_Dummy + $2D; // shift or squeeze the window's position and dimensions to fit it on screen.
  WA_GimmeZeroZero = WA_Dummy + $2E; // equiv. to TNewWindow.Flags WFLG_GIMMEZEROZERO
  // New for V37: WA_MenuHelp (ignored by V36)
  WA_MenuHelp      = WA_Dummy + $2F; // Enables IDCMP_MENUHELP:  Pressing HELP during menus will return IDCMP_MENUHELP message.
  // New for V39: (ignored by V37 and earlier)
  WA_NewLookMenus  = WA_Dummy + $30; // Set to TRUE if you want NewLook menus
  WA_AmigaKey      = WA_Dummy + $31; // Pointer to image for Amiga-key equiv in menus
  WA_NotifyDepth   = WA_Dummy + $32; // Requests IDCMP_CHANGEWINDOW message when window is depth arranged (imsg^.Code = CWCODE_DEPTH)
  // WA_Dummy + $33 is obsolete
  WA_Pointer       = WA_Dummy + $34; // Allows you to specify a custom pointer for your window.  ti_Data points to a
                                     // pointer object you obtained via "pointerclass". nil signifies the default pointer.
                                     // This tag may be passed to OpenWindowTags() or SetWindowPointer().
  WA_BusyPointer   = WA_Dummy + $35; // ti_Data is boolean.  Set to TRUE to request the standard busy pointer.
                                     // This tag may be passed to OpenWindowTags() or SetWindowPointer().
  WA_PointerDelay  = WA_Dummy + $36; // ti_Data is boolean.  Set to TRUE to request that the changing of the pointer be slightly delayed.  The change
                                     // will be called off if you call NewSetPointer() before the delay expires.  This allows
                                     // you to post a busy-pointer even if you think the busy-time may be very Word, without fear of a flashing pointer.
                                     // This tag may be passed to OpenWindowTags() or SetWindowPointer().
  WA_TabletMessages= WA_Dummy + $37; // ti_Data is a boolean.  Set to TRUE to request that tablet information be included in IntuiMessages sent to your window.
                                     // Requires that something (i.e. a tablet driver) feed IESUBCLASS_NEWTABLET InputEvents into
                                     // the system.  For a pointer to the TabletData, examine the ExtIntuiMessage->eim_TabletData
                                     // field.  It is UNSAFE to check this field when running on pre-V39 systems.  It's always
                                     // safe to check this field under V39 and up, though it may be nil.
  WA_HelpGroup     = WA_Dummy + $38; // When the active window has gadget help enabled, other windows of the same HelpGroup number
                                     // will also get GadgetHelp.  This allows GadgetHelp to work for multi-windowed applications.
                                     // Use GetGroupID() to get an ID number.  Pass this number as ti_Data to all your windows. See also the HelpControl() function.
  WA_HelpGroupWindow=WA_Dummy + $39; // When the active window has gadget help enabled, other windows of the same HelpGroup will also get
                                     // GadgetHelp.  This allows GadgetHelp to work for multi-windowed applications.  As an alternative
                                     // to WA_HelpGroup, you can pass a pointer to any other window of the same group to join its help
                                     // group.  Defaults to NULL, which has no effect. See also the HelpControl() function.
// New for V50:  (ignored by V40 and earlier)
  WA_UserPort   = WA_Dummy + $3a; // (PMsgPort) A shared idcmp port, Intuition will not attempt to delete this port in ModifyIDCMP() or
                                  // CloseWindow(). The CloseWindow() routine also takes care of stripping off all intuimessages that belong to the given window. V50.
  WA_WindowBox  = WA_Dummy + $3b; // (PIBox) An alternative way of specifying the window position and size. V50.
  WA_Hidden     = WA_Dummy + $3c; // (BOOL) if TRUE, the window will open in hidden state. V50.
  WA_ToolBox    = WA_Dummy + $3d; // (BOOL) if TRUE, the window can't be activated. ToolBox windows send gadget messages. V50.
  WA_Reserved1  = WA_Dummy + $3e; // Reserved for system use. V50.
  WA_MenuHook   = WA_Dummy + $3f; // (PHook) Hook to call when user requests the menu of the window. V50.
  WA_AutoAdjustDClip = WA_Dummy + $40; // (BOOL) Like WA_AutoAdjust but moves the window onto the DClip. V50.
  WA_ShapeRegion= WA_Dummy + $41; // (PRegion) Region describing the shape of the window. See layers.library. V50.
  WA_ShapeHook  = WA_Dummy + $42; // (PHook) Hook providing the shape of the window on the fly. V50.
  WA_InFrontOf  = WA_Dummy + $43; // (PWindow) Open the window in front of the given window. V50.
  WA_GrabFocus  = WA_Dummy + $44; // (LongWord) When the window is active, limit mousepointer movements to the window area. The number given is the
                                  // time (in intuiticks) this restriction will apply. This is so to give back full control to the user even when
                                  // your application has crashed. The SetWindowAttrs() function allows you to set this feature at any time.
                                  // A value of 0 disables it. The maximum value is 100.
  WA_StayTop    = WA_Dummy + $45; // (BOOL) Make this window to always stay in front of all others. V50.
  WA_MouseLimits= WA_Dummy + $46; // (PIBox) This tag works together with WA_GrabFocus and allows you to specify the working area of the mouse
                                  // pointer. The coordinates are relative to the upper left corner of the window.
  WA_NoMenuKeyVerify=WA_Dummy+$47;// (BOOL) Restrict usage of menu verification for this window to mouse menu events only, letting keyboard shortcuts pass
                                  // through. Ignored if IDCMP_MENUVERIFY is not set. V51.
  WA_Reserved2  = WA_Dummy + $48; // Reserved for system use. V51.
  WA_AlphaClips = WA_Dummy + $49; // (PClipRect) ClipRect list describing the alpha map of the window. See layers.library. V53.
  WA_AlphaHook  = WA_Dummy + $4A; // (PHook) Hook providing the alpha map of the window on the fly. V53.
  WA_Opaqueness = WA_Dummy + $4B; // (LongWord) Overall opaqueness of the window. V53.
  WA_FadeTime   = WA_Dummy + $4C; // (LongWord) Duration of an opaqueness transition for the window, expressed in microseconds; 0 means "immediate". V53.
  WA_OverrideOpaqueness = WA_Dummy + $4D; // (BOOL) If TRUE, the window opaqueness set via WA_Opaqueness
                                          //  will never get affected by any global opaqueness level from user preferences. V53.
  WA_NoHitThreshold = WA_Dummy + $4E; // (LongInt) Any pixel of the window whose opaqueness is less than or equal to the specified amount will be treated as if it
  // were intangible, i.e. it will let mouse clicks pass through to windows behind it. Possible values range from zero (only
  // fully transparent pixels are intangible) to 255 (no pixel in the window can ever be hit). Additionally, -1 will make the
  // whole window tangible, even where it is fully transparent. Use with care, as making opaque areas not hittable (or vice  versa) may easily confuse the user. Defaults to 16. V53.
  WA_DropShadows = WA_Dummy + $4F; // (LongInt) Allow drop shadows on window sides, if requested. Set to FALSE to prevent shadows from ever being drawn for
  // this window, to TRUE to always allow shadows, and to -1 to only allow them if this window has a visible border.
  // This tag defaults to -1, and is ignored if compositing is not enabled for this window's screen. V53.
  WA_PointerType = WA_Dummy + $50; // Allows you to set one of Intuition's built-in pointers for your window. Zero signifies the default pointer.
                                   // This tag may be passed to OpenWindowTags() or SetWindowPointer(). V53.
  WA_MenuStrip   = WA_Dummy + $51; // (PMenu) A menu strip for the window. This can be either a linked chain of traditional Menu structures, or a BOOPSI menu object tree from "menuclass". V54.
  WA_ContextMenuHook = WA_Dummy + $52; // (struct Hook *) This hook will be invoked when Intuition is about to bring up the menus of a window

type
// Definitions for WA_ContextMenuHook (window context menu hook feature).
  PContextMenuMsg = ^TContextMenuMsg;
  TContextMenuMsg = record
    State: LongWord; // CM_QUERY
    // Set the following fields on CM_QUERY
    Menu: APTR;     // A context menu, or nil for the window menu
    Context: APTR;  // The application-specific element the menu is tied to
  end;
const
  CM_QUERY = 0;  // Menus are about to open, please return a context menu
// Special codes for ShowWindow() and WA_InFrontOf: Give this as target window where to move your window to.
  WINDOW_BACKMOST  = PWindow(nil);
  WINDOW_FRONTMOST = PWindow(1);
// HelpControl() flags: HC_GADGETHELP - Set this flag to enable Gadget-Help for one or more windows.
  HC_GADGETHELP  = 1;

// IntuitionControlA() tags:
  ICTRL_Dummy = TAG_USER + $1C000;
  // No public tags defined so far

{ New for V39, Intuition supports the IESUBCLASS_NEWTABLET subclass of the IECLASS_NEWPOINTERPOS event.
  The ie_EventAddress of such an event points to a TabletData structure (see below).

  The TabletData structure contains certain elements including a taglist. The taglist can be used for special tablet parameters.  A tablet driver
  should include only those tag-items the tablet supports.  An application can listen for any tag-items that interest it.  Note: an application
  must set the WA_TabletMessages attribute to TRUE to receive this extended information in its IntuiMessages.

  The definitions given here MUST be followed.  Pay careful attention to normalization and the interpretation of signs.

  Note: a stylus that supports tilt should use the TABLETA_AngleX and TABLETA_AngleY attributes.  Tilting the stylus so the tip
  points towards increasing or decreasing X is actually a rotation around the Y-axis.  Thus, if the stylus tip points towards
  positive X, then that tilt is represented as a negative TABLETA_AngleY.  Likewise, if the stylus tip points towards positive Y, that tilt is represented by positive TABLETA_AngleX.}
const
 TABLETA_Dummy       = TAG_USER + $3A000;
 TABLETA_TabletZ     = TABLETA_Dummy + $01; // the current value of the tablet in the Z direction. This unsigned value should typically be in the natural units of the tablet. You should also provide TABLETA_RangeZ.
 TABLETA_RangeZ      = TABLETA_Dummy + $02; // the maximum value of the tablet in the Z direction. Normally specified along with TABLETA_TabletZ, this allows the application to scale the actual Z value across its range.
 TABLETA_AngleX      = TABLETA_Dummy + $03; // the angle of rotation or tilt about the X-axis.  This number should be normalized to fill a signed long LongInt.  Positive values imply a clockwise rotation about the X-axis when viewing from +X towards the origin.
 TABLETA_AngleY      = TABLETA_Dummy + $04; // the angle of rotation or tilt about the Y-axis.  This number should be normalized to fill a signed long LongInt.  Positive values imply a clockwise rotation about the Y-axis when viewing from +Y towards the origin.
 TABLETA_AngleZ      = TABLETA_Dummy + $05; // the angle of rotation or tilt about the Z axis.  This number should be normalized to fill a signed long LongInt.  Positive values imply a clockwise rotation about the Z-axis when viewing from +Z towards the origin.
 TABLETA_Pressure    = TABLETA_Dummy + $06; // the pressure reading of the stylus.  The pressure should be normalized to fill a signed long LongInt.  Typical devices
                                            // won't generate negative pressure, but the possibility is not precluded. The pressure threshold which is considered to cause a button-click is
                                            // expected to be set in a Preferences program supplied by the tablet vendor.  The tablet driver would send IECODE_LBUTTON-type events as the pressure crossed that threshold.
 TABLETA_ButtonBits  = TABLETA_Dummy + $07; // ti_Data is a long LongInt whose bits are to be interpreted at the state of the first 32 buttons of the tablet.
 TABLETA_InProximity = TABLETA_Dummy + $08; // ti_Data is a boolean.  For tablets that support proximity, they should send the (TABLETA_InProximity,FALSE) tag item
                                            // when the stylus is out of proximity.  One possible use we can forsee is a mouse-blanking commodity which keys off this to blank the
                                            // mouse.  When this tag is absent, the stylus is assumed to be in proximity.
 TABLETA_ResolutionX = TABLETA_Dummy + $09; // ti_Data is an unsigned long LongInt which is the x-axis resolution in dots per inch.
 TABLETA_ResolutionY = TABLETA_Dummy + $0A; // ti_Data is an unsigned long LongInt which is the y-axis resolution in dots per inch.

// If your window sets WA_TabletMessages to TRUE, then it will receive extended IntuiMessages (struct ExtIntuiMessage) whose eim_TabletData
// field points at a TabletData structure.  This structure contains additional information about the input event.
type
  PTabletData = ^TTabletData;
  TTabletData = record
    td_XFraction, td_YFraction: Word; // Sub-pixel position of tablet, in screen coordinates, scaled to fill a word fraction
    td_TabletX, td_TabletY: LongWord; // Current tablet coordinates along each axis
    td_RangeX, td_RangeY: LongWord;   // Tablet range along each axis.  For example, if td_TabletX can take values 0-999, td_RangeX should be 1000.
    td_TagList: PTagItem;             // Pointer to tag-list of additional tablet attributes.
 end;

// If a tablet driver supplies a hook for ient_CallBack, it will be invoked in the standard hook manner.  A0 will point to the Hook
// itself, A2 will point to the InputEvent that was sent, and A1 will point to a TabletHookData structure.  The InputEvent's
// ie_EventAddress field points at the IENewTablet structure that the driver supplied.
// Based on the thd_Screen, thd_Width, and thd_Height fields, the driver should scale the ient_TabletX and ient_TabletY fields and store the
// result in ient_ScaledX, ient_ScaledY, ient_ScaledXFraction, and ient_ScaledYFraction. The tablet hook must currently return nil.
// This is the only acceptable return-value under V39.
  PTabletHookData = ^TTabletHookData;
  TTabletHookData = record
    thd_Screen: PScreen; // Pointer to the active screen: Note: if there are no open screens, thd_Screen will be NULL. thd_Width and thd_Height will then describe an NTSC 640x400 screen.  Please scale accordingly.
    thd_Width, thd_Height: LongWord; // The width and height (measured in pixels of the active screen) that your are to scale to:
    thd_ScreenChanged  : LongInt; // Non-zero if the screen or something about the screen changed since the last time you were invoked:
 end;
// New for V39:
// All IntuiMessages are now slightly extended.  The ExtIntuiMessage structure has an additional field for tablet data, which is usually
// nil.  If a tablet driver which is sending IESUBCLASS_NEWTABLET events is installed in the system, windows with the WA_TabletMessages
// property set will find that eim_TabletData points to the TabletData structure.  Applications must first check that this field is non-NULL;
// it will be NULL for certain kinds of message, including mouse activity generated from other than the tablet (i.e. the keyboard equivalents or the mouse itself).
// NEVER EVER examine any extended fields when running under pre-V39!
// NOTE: This structure is subject to grow in the future. Making assumptions about its size is A BAD IDEA.
  PExtIntuiMessage = ^TExtIntuiMessage;
  TExtIntuiMessage = record
    eim_IntuiMessage: TIntuiMessage;
    eim_TabletData: PTabletData;
  end;

// === DoScrollHook() specifications (V50)
// Hook function with extra data; invoked when one of the scrolling operations are to be performed.
  PScrollHook = ^TScrollHook;
  TScrollHook = record
    sh_Hook: PHook;
    sh_Top: LongInt;     // This is the number of the first visible item.
    sh_Total: LongInt;   // This is the total number of items available for display.
    sh_Visible: LongInt; // This is how many entries can be visible at a time.
    sh_Size: LongInt;    // This is how many pixels high/wide each item is.
  end;
// The message passed to your scrolling function. The action to perform is in the sm_Action field.
  PScrollMsg = ^TScrollMsg;
  TScrollMsg = record
    sm_Action: LongWord;   // The action to perform
    sm_NumItems: LongInt;  // Number of items to render
    sm_NumPixels: LongInt; // Number of pixels to scroll
    sm_TopItem: LongInt;   // The first visible item
    sm_TopPixel: LongInt;  // The first visible pixel
  end;
const
// Scrolling actions to perform.
  SMA_GetScrollerTop = 0; // Return the number of the first visible line.
  SMA_RedrawAll      = 1; // Redraw the entire display.
  SMA_Scroll         = 2; // Scroll the display; the number of pixels to scroll is stored in the sm_NumPixels field. Return whether
                          // the scrolling operation produced layer damage which needs to be repaired.
  SMA_Draw           = 3; // Draw a portion of the display. The number of lines to print is in the sm_NumItems field. The pixel position
                          // to start drawing at is in the sm_TopPixel field. The first line to print is in the sm_TopItem field.
  SMA_RepairDamage   = 4; // Same as SMA_Draw except that the drawing operations must be performed between BeginRefresh() and EndRefresh().
// How to perform the scrolling operation (this is the scroll mode parameter of the DoScrollHook() function.
  DSHM_Smooth = 0; // Scroll smoothly
  DSHM_Jump   = 1; // Jump straight to the requested display position.

// GUI element base types
  BT_BACKGROUND  = $00000000; // Generic background
  BT_DRIPEN      = $00100000; // A base DrawInfo pen
  BT_WINBORDER   = $00200000; // Window border
  BT_GADGET      = $00300000; // Generic gadget
  BT_PROPKNOB    = $00400000; // Proportional gadget knob
  BT_PROPBG      = $00500000; // Proportional gadget background
  BT_WINPROPKNOB = $00600000; // Prop. gadget knob in window border
  BT_WINPROPBG   = $00700000; // Prop. gadget backgr. in window border
  BT_MENU        = $00800000; // Menu panel background
  BT_SCREENBAR   = $00900000; // Screen titlebar
  BT_WINDOWBAR   = $00A00000; // Window titlebar
  BT_WINDOWBG    = $00B00000; // Window background
  BT_REQUESTERBG = $00C00000; // Requester background
  BT_TABPAGE     = $00D00000; // Tab page background

  BT_FLAGSMASK   = $000F0000; // Flags valid for ALL base types
  BT_OFFSETRECT  = $00010000; // Left, top contain backfill offsets
  BT_DIRECTSHADE = $00020000; // Flag for ShadeRectA(), see autodoc
// Shading levels
  LEVEL_BRIGHT     = 0; // Full-intensity bright details
  LEVEL_HALFBRIGHT = 1; // Half-intensity bright details
  LEVEL_NORMAL     = 2; // Normal imagery
  LEVEL_HALFDARK   = 3; // Half-intensity dark details
  LEVEL_DARK       = 4; // Full-intensity dark details
// Contrast levels and flags
  STD_DISABLE_CONTRAST = $1000; // For disabled imagery
  CNTR_CORRECT         = $0100; // Correct contrast of bright details
  CNTR_ABSOLUTE        = $0200; // Absolute (non-percentage) shading
// ShadeRectA() tags
  SRA_Dummy = $03800000;
  SRA_OffsetX     = SRA_Dummy + 1; // (Word) Logical rendering offset on the X axis.
  SRA_OffsetY     = SRA_Dummy + 2; // (Word) Logical rendering offset on the Y axis.
  SRA_DeltaX      = SRA_Dummy + 3; // (Word) Delta increment for SRA_OffsetX.
  SRA_DeltaY      = SRA_Dummy + 4; // (Word) Delta increment for SRA_OffsetY.
  SRA_Domain      = SRA_Dummy + 5; // (PRectangle) Logical rendering container.
  SRA_DirectShade = SRA_Dummy + 6; // (BOOL) Force on-the-fly shading
  SRA_RefreshBG   = SRA_Dummy + 7; // (BOOL) Redraw background first to avoid overshading
// DisableTemplate() data types (V51)
  TT_BITPLANE = 0; // Template is a 16-bit-aligned bitplane
  TT_ALPHA8   = 1; // Template is an alpha map (modulo = 1)
  TT_ALPHA16  = 2; // Template is an alpha map (modulo = 2)
  TT_ALPHA24  = 3; // Template is an alpha map (modulo = 3)
  TT_ALPHA32  = 4; // Template is an alpha map (modulo = 4)
// GUI attributes
  GUIA_Dummy  = TAG_USER + $4400000;
  //******* Window-, screen- and global-level GUI attributes
  GUIA_DefaultStyle        = GUIA_Dummy + 1;  // (APTR) Default style plugin, may be overridden (G/IG/SG).
  GUIA_WindowBorderStyle   = GUIA_Dummy + 2;  // (APTR) Style plugin to be used to render window borders (G/G/G).
  GUIA_WindowGadgetStyle   = GUIA_Dummy + 3;  // (APTR) Style plugin to be used to render border gadgets (G/G/G).
  GUIA_SizeGadgetWidth     = GUIA_Dummy + 4;  // (ShortInt) Width of window sizing gadget; -1 = automatic (G/IG/SG).
  GUIA_SizeGadgetHeight    = GUIA_Dummy + 5;  // (ShortInt) Height of window sizing gadget; -1 = automatic (G/IG/SG).
  GUIA_ArrowGadgetWidth    = GUIA_Dummy + 6;  // (ShortInt) Width of border arrow gadgets; -1 = automatic (G/IG/SG).
  GUIA_ArrowGadgetHeight   = GUIA_Dummy + 7;  // (ShortInt) Height of border arrow gadgets; -1 = automatic (G/IG/SG).
  GUIA_WindowBorderLeft    = GUIA_Dummy + 8;  // (ShortInt) Thickness of left window border, without gadgets (G/IG/SG).
  GUIA_WindowBorderTop     = GUIA_Dummy + 9;  // (ShortInt) Thickness of top window border, without gadgets (G/IG/SG).
  GUIA_WindowBorderRight   = GUIA_Dummy + 10; // (ShortInt) Thickness of right window border, without gadgets (G/IG/SG).
  GUIA_WindowBorderBottom  = GUIA_Dummy + 11; // (ShortInt) Thickness of bottom window border, without gadgets (G/IG/SG).
  GUIA_WindowTitlePosition = GUIA_Dummy + 12; // (Byte) Position of titles in window titlebars (G/IG/SG).
  GUIA_WindowBarPadding    = GUIA_Dummy + 13; // (Byte) Added to the font height to set titlebar height (G/IG/SG).
  GUIA_WindowReserved      = GUIA_Dummy + 14; // (BOOL) Reserved for now, don't use.
  GUIA_WindowPropKnobHandle= GUIA_Dummy + 15; // (Word) Type of handle for window border scroller knobs (G/ISG/SG).
  GUIA_WindowPropKnobColor = GUIA_Dummy + 16; // (BOOL) Do border scroller knobs use window border color? (G/ISG/SG).
  GUIA_WindowPropBackground= GUIA_Dummy + 17; // (Word) Type of background for window border scrollers (G/ISG/SG).
  GUIA_PropBackground      = GUIA_Dummy + 18; // (Word) Type of background for proportional gadgets (G/ISG/SG).
  GUIA_WindowPropKnobBackfill      = GUIA_Dummy + 19; // (PHook) Backfill hook for border scroller knobs (G/ISG/SG).
  GUIA_PropKnobBackfill            = GUIA_Dummy + 20; // (PHook) Backfill hook for prop gadget knobs (G/ISG/SG).
  GUIA_WindowPropSelKnobBackfill   = GUIA_Dummy + 21; // (PHook) Backfill hook for selected border knobs (G/ISG/SG).
  GUIA_PropSelKnobBackfill         = GUIA_Dummy + 22; // (PHook) Backfill hook for selected prop knobs (G/ISG/SG).
  GUIA_WindowPropInactKnobBackfill = GUIA_Dummy + 23; // (PHook) Backfill hook for inactive border knobs (G/ISG/SG).
  GUIA_WindowPropBackfill          = GUIA_Dummy + 24; // (PHook) Backfill hook for border scrollers (G/ISG/SG).
  GUIA_WindowPropInactBackfill     = GUIA_Dummy + 25; // (PHook) Backfill hook for inactive border scrollers (G/ISG/SG).
  GUIA_PropBackfill                = GUIA_Dummy + 26; // (PHook) Backfill hook for proportional gadgets (G/ISG/SG).
  GUIA_SysImageFrameType   = GUIA_Dummy + 27; // (Word) Type of frame for screen and window system gadgets (G/IG/SG).
  GUIA_WindowPropBorder    = GUIA_Dummy + 28; // (BOOL) Do window border scrollers have a double border? (G/ISG/SG).
  GUIA_PropBorder          = GUIA_Dummy + 29; // (BOOL) Do GT/RA scrollers have a double border? (G/ISG/SG).
  GUIA_WindowPropSpacing   = GUIA_Dummy + 30; // (Byte) Thickness of window scrollers borders (G/ISG/SG).
  GUIA_PropSpacing         = GUIA_Dummy + 31; // (Byte) Thickness of GT/RA scrollers borders (G/ISG/SG).
  GUIA_PropKnobHandle      = GUIA_Dummy + 32; // (Word) Type of handle for scroller knobs (G/ISG/SG).
  GUIA_PropKnobColor       = GUIA_Dummy + 33; // (BOOL) Do scroller knobs use FILLPEN as their color? (G/ISG/SG).
  GUIA_WindowSizeBorder    = GUIA_Dummy + 34; // (Word) Preferred border(s) to place the sizing gadget in (G/IG/SG).
  GUIA_WindowGadgetFrameStyle = GUIA_Dummy + 35; // (APTR) Style plugin to be used to render border gadget frames (G/G/G).
  GUIA_GadgetStyle         = GUIA_Dummy + 36; // (APTR) Style plugin to be used to render GT/RA gadgets (G/G/G).
  GUIA_DefaultGeometry     = GUIA_Dummy + 37; // (APTR) Style plugin defining geometry of GUI elements (G/IG/SG).
  GUIA_WindowGadgetGeometry= GUIA_Dummy + 38; // (APTR) Style plugin defining geometry of border gadgets (G/G/G).
  GUIA_GadgetGeometry      = GUIA_Dummy + 39; // (APTR) Style plugin defining geometry of GT/RA gadgets (G/G/G).
  GUIA_ScreenBarGadgetSize = GUIA_Dummy + 40; // (APTR) Size of gadgets in the screen titlebar (G/IG/SG).
  GUIA_WindowBarGadgetSize = GUIA_Dummy + 41; // (APTR) Size of gadgets in the window titlebar (G/ISG/SG).
  GUIA_GadgetArrowType     = GUIA_Dummy + 42; // (LongWord) Type of arrow symbols to be used in arrow buttons (G/IG/SG).
  GUIA_GadgetCycleType     = GUIA_Dummy + 43; // (LongWord) Type of cycle symbol to be used in cycle gadgets (G/IG/SG).
  GUIA_GadgetArrowStyle    = GUIA_Dummy + 44; // (APTR) Style plugin to be used to render GT/RA arrow buttons (G/G/G).
  GUIA_GadgetCycleStyle    = GUIA_Dummy + 45; // (APTR) Style plugin to be used to render GT/RA cycle gadgets (G/G/G).
  GUIA_SysImageSymbolType  = GUIA_Dummy + 46; // (Word) Type of symbols for screen and window system gadgets (G/IG/SG).
  GUIA_SysIGeometryType    = GUIA_Dummy + 47; // (Word) Type of geometry for screen and window system gadgets (G/IG/SG).
  GUIA_WindowBarBackfill   = GUIA_Dummy + 48; // (PHook) Backfill hook for active window titlebars (G/ISG/SG).
  GUIA_WindowBarInactBackfill = GUIA_Dummy + 49; // (PHook) Backfill hook for inactive window titlebars (G/ISG/SG).
  GUIA_WindowBorderBackfill= GUIA_Dummy + 50; // (PHook) Backfill hook for active window borders (G/ISG/SG).
  GUIA_WindowBorderInactBackfill = GUIA_Dummy + 51; // (PHook) Backfill hook for inactive window borders (G/ISG/SG).
  GUIA_WindowPropLook      = GUIA_Dummy + 52; // (BOOL) Use the GUIA_FramePropLook also for border scrollers? (G/ISG/SG).
  GUIA_EvenRequesterButtons= GUIA_Dummy + 53; // (BOOL) Make all buttons of system requesters equally wide? (G/ISG/SG).
  GUIA_RequestersPosition  = GUIA_Dummy + 54; // (Word) Opening position of system requesters (G/ISG/SG).
  GUIA_WindowBorderLook    = GUIA_Dummy + 55; // (Word) Type of window border (3D, framed, flat...) (G/ISG/SG).
  GUIA_Reserved1           = GUIA_Dummy + 56;
  GUIA_Reserved2           = GUIA_Dummy + 57;
  GUIA_Reserved3           = GUIA_Dummy + 58; // (APTR) System reserved attributes, don't use.
  GUIA_WindowBarFrameThickness= GUIA_Dummy + 59; // (Word) Thickness of 3D edges of window titlebar frames (1 or 2) (G/ISG/SG).
  GUIA_WindowFrameThickness= GUIA_Dummy + 60; // (Word) Thickness of 3D edges of window frames (1 or 2) (G/ISG/SG).
  GUIA_PropOuterSpacing    = GUIA_Dummy + 61; // (Word) Make scrollers detached from lists and/or buttons? (G/ISG/SG).
  GUIA_SliderDisplayMode   = GUIA_Dummy + 62; // (Word) Preferred position of slider level display (G/ISG/SG).
  GUIA_CycleLabelJustify   = GUIA_Dummy + 63; // (Word) Justification of cycle/chooser labels (G/ISG/SG).
  GUIA_ClickTabLook        = GUIA_Dummy + 64; // (LongWord) Various flags affecting the look of clicktabs (G/ISG/SG).
  GUIA_GadgetHorizPadding  = GUIA_Dummy + 65;
  GUIA_GadgetVertPadding   = GUIA_Dummy + 66; // (Word) Horizontal and vertical padding for ReAction gadgets (G/ISG/SG).
  GUIA_WindowBackfill      = GUIA_Dummy + 67; // (PHook) Backfill hook for window background (G/ISG/SG).
  GUIA_RequesterBackfill   = GUIA_Dummy + 68; // (PHook) Backfill hook for requester background (G/ISG/SG).
  GUIA_TabTitleBackfill    = GUIA_Dummy + 69; // (PHook) Backfill hook for active tab header (G/ISG/SG).
  GUIA_TabActiveBackfill   = GUIA_Dummy + 70; // (PHook) Backfill hook for active tab page (G/ISG/SG).
  GUIA_TabInactiveBackfill = GUIA_Dummy + 71; // (PHook) Backfill hook for inactive tabs (G/ISG/SG).
  GUIA_LayoutHorizSpacing  = GUIA_Dummy + 72;
  GUIA_LayoutVertSpacing   = GUIA_Dummy + 73; // (Word) Horizontal and vertical spacing in ReAction layouts (G/ISG/SG).
  GUIA_GroupLabelPlace     = GUIA_Dummy + 74; // (LongWord) Position of group labels (left/center/right) (G/ISG/SG).
  GUIA_GroupLabelLook      = GUIA_Dummy + 75; // (LongWord) Appearance of group labels (plain, 3D, bold...) (G/ISG/SG).
  GUIA_GroupLabelTextAttr  = GUIA_Dummy + 76; // (PTextAttr) Font specification for group labels (G/ISG/SG).
  GUIA_FallbackTextAttr    = GUIA_Dummy + 77; // (PTextAttr) Font specification for fallback layout (G/ISG/SG).
  GUIA_GadgetTextAttr      = GUIA_Dummy + 78; // (PTextAttr) Font specification for RA gadgets (G/ISG/SG).
  GUIA_LabelTextAttr       = GUIA_Dummy + 79; // (PTextAttr) Font specification for RA labels (G/ISG/SG).
  GUIA_WindowRefresh       = GUIA_Dummy + 80; // (LongWord) Preferred window refresh type (1 = simple, 0 = smart) (G/ISG/SG).
  GUIA_GroupLabelVertAlign = GUIA_Dummy + 81; // (LongWord) Vertical alignment of group labels (center, baseline...) (G/ISG/SG).
  GUIA_ListHierNodeStyle   = GUIA_Dummy + 82; // (Word) Style of hierarchical list nodes (+/- boxes, arrows) (G/ISG/SG).
  GUIA_ListHierConnectType = GUIA_Dummy + 83; // (Word) How to connect hierarchical list nodes (lines, nothing) (G/ISG/SG).
  GUIA_Reserved4           = GUIA_Dummy + 84; // (APTR) System reserved attribute, don't use.
  GUIA_WindowOuterFlatThickness = GUIA_Dummy + 85; // (Word) Thickness of outer flat borders for framed/flat window look (1 or 2) (G/ISG/SG).
  GUIA_WindowInnerFlatThickness = GUIA_Dummy + 86;  // (Word) Thickness of inner flat borders for framed/flat window look (1 or 2) (G/ISG/SG).
  GUIA_CycleLook           = GUIA_Dummy + 87; // (LongWord) Various flags affecting the look of cycle/chooser gadgets (G/ISG/SG).
  GUIA_WindowGaugeLook     = GUIA_Dummy + 88; // (LongWord) Various flags affecting the look of window fill gauge gadgets (G/ISG/SG).
  GUIA_GaugeLook           = GUIA_Dummy + 89; // (LongWord) Various flags affecting the look of fill gauge gadgets (G/ISG/SG).
  GUIA_ClearBackground     = GUIA_Dummy + 90; // (BOOL) Always erase background before redrawing GUI elements? (G/ISG/SG).
  GUIA_Reserved5           = GUIA_Dummy + 91; // (APTR) System reserved attribute, don't use.
  GUIA_WindowAlpha         = GUIA_Dummy + 92; // (Byte) Opaqueness of a normal window (G/ISG/SG).
  GUIA_WindowInactAlpha    = GUIA_Dummy + 93; // (Byte) Opaqueness of an inactive window (G/ISG/SG).
  GUIA_WindowDragAlpha     = GUIA_Dummy + 94; // (Byte) Opaqueness of a dragged window (G/ISG/SG).
  GUIA_WindowOpenFadeTime  = GUIA_Dummy + 95; // (LongWord) Duration of window fade-in at open time (G/ISG/SG).
  GUIA_WindowCloseFadeTime = GUIA_Dummy + 96; // (LongWord) Duration of window fade-out at close time (G/ISG/SG).
  GUIA_WindowGoActiveFadeTime   = GUIA_Dummy + 97; // (LongWord) Duration of window fade when going active (G/ISG/SG).
  GUIA_WindowGoInactiveFadeTime = GUIA_Dummy + 98; // (LongWord) Duration of window fade when going inactive (G/ISG/SG).
  GUIA_FrameAlphaStatus    = GUIA_Dummy + 99; // (LongWord) Potential transparency of various frame types (G/G/G).
  GUIA_WindowShadowSizes   = GUIA_Dummy + 100; // (LongWord) Sizes of drop shadows on the four window sides; this is four signed bytes packed as a single longword (G/G/G).
  GUIA_WindowShadowStrength= GUIA_Dummy + 101; // (LongWord) Intensity of drop shadows for various window types (0..255); this is four unsigned bytes packed as a single longword (G/ISG/SG).
  GUIA_WindowShadowType    = GUIA_Dummy + 102; // (LongWord) Type of built-in window drop shadow (G/ISG/SG).
  GUIA_WindowShadowColor   = GUIA_Dummy + 103; // (LongWord) Color of window drop shadows in 00R8G8B8 form (G/ISG/SG).
  GUIA_WindowShadowDisplacement = GUIA_Dummy + 104; // (LongWord) X/Y displacement of drop shadows relative to the window; this is four signed bytes packed as a single longword, two bytes for the active window and two for inactive windows (G/ISG/SG).
  GUIA_WindowShadowSmoothness   = GUIA_Dummy + 105; // (Word) Smoothness level of window drop shadows (0..20) (G/ISG/SG).
  //************ Screen- and global-level GUI attributes
  GUIA_ScreenTitlePosition = GUIA_Dummy + 1001; // (Byte) Position of titles in screen titlebars (G/IG/SG).
  GUIA_ScreenBarPadding    = GUIA_Dummy + 1002; // (Byte) Added to the font height to set titlebar height (G/IG/SG).
  GUIA_MenuType            = GUIA_Dummy + 1003; // (Word) Type of menu: MT_PULLDOWN, MT_POPUP or MT_RELATIVE (G/ISG/SG).
  GUIA_MenuFlags           = GUIA_Dummy + 1004; // (LongWord) Menu flags (see definitions below) (G/ISG/SG).
  GUIA_MenuDropShadows     = GUIA_Dummy + 1005; // (BOOL) Do menus cast drop shadows? (G/ISG/SG).
  GUIA_MenuTransparency    = GUIA_Dummy + 1006; // (BOOL) Do menus have transparency? (G/ISG/SG).
  GUIA_MenuRenderHook      = GUIA_Dummy + 1007; // (PHook) Rendering hook for menus (G/ISG/SG).
  GUIA_MenuBackfill        = GUIA_Dummy + 1008; // (PHook) Backfill hook for menus (G/ISG/SG).
  GUIA_MenuStyle           = GUIA_Dummy + 1009; // (APTR) Style plugin to import menu rendering hook from (G/ISG/SG).
  GUIA_BrightContrast      = GUIA_Dummy + 1010;
  GUIA_DarkContrast        = GUIA_Dummy + 1011; // (Byte) Default contrast for edges brightening/darkening (G/IG/SG).
  GUIA_FillBrightContrast  = GUIA_Dummy + 1012;
  GUIA_FillDarkContrast    = GUIA_Dummy + 1013; // (Byte) Contrast of FILLSHINEPEN/FILLSHADOWPEN (G/IG/SG).
  GUIA_InactiveFillBrightContrast = GUIA_Dummy + 1014;
  GUIA_InactiveFillDarkContrast   = GUIA_Dummy + 1015; // (Byte) Contrast of INACTIVEFILLSHINEPEN/INACTIVEFILLSHADOWPEN (G/IG/SG).
  GUIA_MenuBrightContrast  = GUIA_Dummy + 1016;
  GUIA_MenuDarkContrast    = GUIA_Dummy + 1017; // (Byte) Contrast of MENUSHINEPEN/MENUSHADOWPEN (G/IG/SG).
  GUIA_SelectBrightContrast= GUIA_Dummy + 1018;
  GUIA_SelectDarkContrast  = GUIA_Dummy + 1019; // (Byte) Contrast of SELECTSHINEPEN/SELECTSHADOWPEN (G/IG/SG).
  GUIA_BarBlockBrightContrast = GUIA_Dummy + 1020;
  GUIA_BarBlockDarkContrast   = GUIA_Dummy + 1021; // (Byte) Contrast of BARCONTOURPEN/BARTRIMPEN (G/IG/SG).
  GUIA_DisabledBrightContrast = GUIA_Dummy + 1022;
  GUIA_DisabledDarkContrast   = GUIA_Dummy + 1023; // (Byte) Contrast of DISABLEDSHINEPEN/DISABLEDSHADOWPEN (G/IG/SG).
  GUIA_ForeBrightContrast    = GUIA_Dummy + 1024;
  GUIA_ForeDarkContrast      = GUIA_Dummy + 1025; // (Byte) Contrast of FORESHINEPEN/FORESHADOWPEN (G/IG/SG).
  GUIA_AutomaticEdgesContrast= GUIA_Dummy + 1026; // (BOOL) Enable/disable automatic computation of shine/shadow pens (G/IG/SG).
  GUIA_BrightCurve         = GUIA_Dummy + 1028;
  GUIA_DarkCurve           = GUIA_Dummy + 1029; // (Byte) Default curve for shine/shadow colors gradients (G/IG/SG).
  GUIA_FillBrightCurve     = GUIA_Dummy + 1030;
  GUIA_FillDarkCurve       = GUIA_Dummy + 1031; // (Byte) Curve for FILLSHINEPEN/FILLSHADOWPEN gradients (G/IG/SG).
  GUIA_InactiveFillBrightCurve = GUIA_Dummy + 1032;
  GUIA_InactiveFillDarkCurve   = GUIA_Dummy + 1033; // (Byte) Curve for INACTIVEFILLSHINEPEN/INACTIVEFILLSHADOWPEN gradients (G/IG/SG).
  GUIA_MenuBrightCurve     = GUIA_Dummy + 1034;
  GUIA_MenuDarkCurve       = GUIA_Dummy + 1035; // (Byte) Curve for MENUSHINEPEN/MENUSHADOWPEN gradients (G/IG/SG).
  GUIA_SelectBrightCurve   = GUIA_Dummy + 1036;
  GUIA_SelectDarkCurve     = GUIA_Dummy + 1037; // (Byte) Curve for SELECTSHINEPEN/SELECTSHADOWPEN gradients (G/IG/SG).
  GUIA_BarBlockBrightCurve = GUIA_Dummy + 1038;
  GUIA_BarBlockDarkCurve   = GUIA_Dummy + 1039; // (Byte) Curve for BARCONTOURPEN/BARTRIMPEN gradients (G/IG/SG).
  GUIA_DisabledBrightCurve = GUIA_Dummy + 1040;
  GUIA_DisabledDarkCurve   = GUIA_Dummy + 1041; // (Byte) Curve for DISABLEDSHINEPEN/DISABLEDSHADOWPEN gradients (G/IG/SG).
  GUIA_ForeBrightCurve     = GUIA_Dummy + 1042;
  GUIA_ForeDarkCurve       = GUIA_Dummy + 1043; // (Byte) Curve for FORESHINEPEN/FORESHADOWPEN gradients (G/IG/SG).
  GUIA_PropKnobHighlight   = GUIA_Dummy + 1044; // (Byte) Selected knob look: 0 = raised, 1 = recessed (G/ISG/SG).
  GUIA_PaletteRGBTable     = GUIA_Dummy + 1045; // (PByte) Initial 256-color palette for the screens (G/IG/SG).
  GUIA_PaletteLockTable    = GUIA_Dummy + 1046; // (PByte) Array of locking information for each palette entry (G/IG/SG).
  GUIA_DRIPens             = GUIA_Dummy + 1047; // (PWord) Array of DrawInfo pens for the screen (G/IG/SG).
  GUIA_FrameForceThinEdges = GUIA_Dummy + 1048; // (Byte) Flag mask, see definitions below (G/ISG/SG).
  GUIA_FrameSelectedColor  = GUIA_Dummy + 1049; // (Byte) Fill pen for selected buttons, see values below (G/ISG/SG).
  GUIA_FrameDisabledColors = GUIA_Dummy + 1050; // (Byte) Pens for disabled frame edges, see values below (G/ISG/SG).
  GUIA_FrameButtonLook     = GUIA_Dummy + 1051; // (Byte) Look of button frames, see values below (G/ISG/SG).
  GUIA_FramePressedLook    = GUIA_Dummy + 1052; // (Byte) Look of pressed (selected) buttons, see values below (G/ISG/SG).
  GUIA_FrameCornersType    = GUIA_Dummy + 1053; // (Byte) Type of frame corners (0 = square, 1 = rounded) (G/ISG/SG).
  GUIA_FrameBackfill       = GUIA_Dummy + 1054; // (PHook) Backfill hook for unselected frames (G/ISG/SG).
  GUIA_FrameSelBackfill    = GUIA_Dummy + 1055; // (PHook) Backfill hook for selected frames (G/ISG/SG).
  GUIA_FrameStringLook     = GUIA_Dummy + 1056; // (Byte) Look of string gadget frames, see values below (G/ISG/SG).
  GUIA_FramePropLook       = GUIA_Dummy + 1057; // (Byte) Look of prop gadget frames, see values below (G/ISG/SG).
  GUIA_FrameDisplayLook    = GUIA_Dummy + 1058; // (Byte) Look of display gadget frames, see values below (G/ISG/SG).
  GUIA_WindowPropKnobHighlight = GUIA_Dummy + 1059; // (Byte) Selected window knob look: 0 = raised, 1 = recessed (G/ISG/SG).
  GUIA_OffScreenDragging   = GUIA_Dummy + 1060; // (BOOL) Is off-screen window dragging allowed on this screen? (G/ISG/SG).
  GUIA_FrameDisBackfill    = GUIA_Dummy + 1061; // (PHook) Backfill hook for disabled frames (G/ISG/SG).
  GUIA_RealShading         = GUIA_Dummy + 1062; // (BOOL) Realistic bitmap shading on hicolor/truecolor screens?(G/ISG/SG).
  GUIA_ScreenBarBackfill   = GUIA_Dummy + 1063; // (PHook) Backfill hook for screen titlebars (G/ISG/SG).
  GUIA_MenuSelItemFrame    = GUIA_Dummy + 1064; // (Byte) Type of frame for selected menu items (G/ISG/SG).
  GUIA_OffScreenSizing     = GUIA_Dummy + 1065; // (BOOL) Is off-screen window sizing allowed on this screen? (G/ISG/SG).
  GUIA_OffScreenResistance = GUIA_Dummy + 1066; // (Word) Amount of resistance screen edges offer to window crossing them (G/ISG/SG).
  GUIA_MenuTransparencyLevel = GUIA_Dummy + 1067; // (Byte) Level of menu transparency (if transparency is turned on) (G/ISG/SG).
  GUIA_SpecialEffects      = GUIA_Dummy + 1068; // (BOOL) Enable fancy compositing-based special effects? (G/ISG/SG).
  GUIA_ShareComposeBitMap  = GUIA_Dummy + 1069; // (BOOL) Try to save video memory by sharing temporary off-screen bitmap for layer composition among different screens? (G/ISG/SG).
  GUIA_ScreenBarShadowStrength = GUIA_Dummy + 1070; // (Word) Intensity of screen titlebar drop shadow (0..255) (G/ISG/SG).
  GUIA_ScreenBarShadowColor    = GUIA_Dummy + 1071; // (ULONG) Color of screen titlebar drop shadow in 00R8G8B8 form (G/ISG/SG).
  GUIA_ScreenBarShadowDisplacement = GUIA_Dummy + 1072; // (Word) Vertical displacement of drop shadow relative to the screen titlebar (G/ISG/SG).
  GUIA_ScreenBarShadowSmoothness   = GUIA_Dummy + 1073; // (Word) Smoothness level of screen titlebar drop shadow (0..20) (G/ISG/SG).
  GUIA_MenuOpenDelay       = GUIA_Dummy + 1074; // (Word) Duration of panel open delay for menus and items (0..8).
                                                // Packed as two bytes in a word, low byte for menus and high byte for items (G/ISG/SG).
  GUIA_MenuCloseDelay      = GUIA_Dummy + 1075; // (Word) Duration of panel close delay for menus and items (0..8).
                                                //  Packed as two bytes in a word, low byte for menus and high byte for items (G/ISG/SG).
  GUIA_VSyncCompose        = GUIA_Dummy + 1076; // (BOOL) Synchronize display updates of layer composition with vertical refresh rate of monitor when possible? (G/ISG/SG).
  //**** Global GUI attributes
  GUIA_GlobalFlags         = GUIA_Dummy + 5001; // (LongWord) Global GUI flags (see definitions below) (G/G/SG).
  GUIA_ScreenDragging      = GUIA_Dummy + 5002; // (BOOL) Enable/disable screen dragging (defaults to TRUE) (G/G/SG).
  GUIA_DefaultDRIPens      = GUIA_Dummy + 5003; // (PWord) Default four-color DrawInfo pen array (G/G/G).

// Global GUI flags
  GGPF_SCREENDRAGGING = $00000001; // Enable screen dragging

  // Values for GUIA_WindowBorderLook
  FRAMEWINDOW_3D     = 0; // Standard 3D bevelled frame
  FRAMEWINDOW_FRAMED = 1; // Dark frame with 3D inner frame
  FRAMEWINDOW_FLAT   = 2; // Thin or thick dark frame

// Flags for GUIA_FrameForceThinEdges
  FRAMETHIN_FILLEDRAISED      = $01;
  FRAMETHIN_FILLEDRECESSED    = $02;
  FRAMETHIN_EDGESONLYRAISED   = $04;
  FRAMETHIN_EDGESONLYRECESSED = $08;

// Values for GUIA_FrameSelectedColor
  FRAMESELCOL_FILL   = 0; // Fill pen
  FRAMESELCOL_SELECT = 1; // Select pen
  FRAMESELCOL_FOREG  = 2; // Foreground pen
  FRAMESELCOL_BACKG  = 3; // Background pen

// Values for GUIA_FrameDisabledColors
  FRAMEDISCOL_FOREG = 0; // Foreground shine/shadow pens
  FRAMEDISCOL_BACKG = 1; // Background shine/shadow pens
  FRAMEDISCOL_DISAB = 2; // Disabled shine/shadow pens

// Values for GUIA_FrameButtonLook
  FRAMEBUTTON_3D     = 0;  // Standard 3D bevelled frame
  FRAMEBUTTON_FRAMED = 1;  // Dark frame with 3D inner frame
  FRAMEBUTTON_FLAT   = 2;  // Thin or thick dark frame

// Values for GUIA_FrameStringLook
  FRAMESTRING_3D      = 0;  // Standard 3D ridge
  FRAMESTRING_FRAMED  = 1;  // Dark frame with inner recessed frame
  FRAMESTRING_FLAT    = 2;  // Thin or thick dark frame
  FRAMESTRING_FRAMED2 = 3;  // Recessed frame with inner dark frame

// Values for GUIA_FramePropLook
  FRAMEPROP_3D     = 0;  // Standard 3D bevelled frame
  FRAMEPROP_FRAMED = 1;  // Dark frame
  FRAMEPROP_FLAT   = 2;  // Thin or thick dark frame
  FRAMEPROP_NONE   = 3;  // No frame (only applies to container)

// Values for GUIA_FrameDisplayLook
  FRAMEDISPLAY_3D      = 0;  // Standard 3D recessed frame
  FRAMEDISPLAY_FRAMED  = 1;  // Dark frame with inner recessed frame
  FRAMEDISPLAY_FLAT    = 2;  // Thin or thick dark frame
  FRAMEDISPLAY_FRAMED2 = 3;  // Recessed frame with inner dark frame

// Values for GUIA_FramePressedLook
  FRAMEPRESSED_PUSHED   = 0;  // Pushed in
  FRAMEPRESSED_INVERTED = 1;  // Inverted edges colors

// Values for GUIA_ClickTabLook
  CLICKTAB_ACTIVEBOLD       = $00000001;
  CLICKTAB_EVENWIDTH        = $00000002;
  CLICKTAB_INACTIVEDARK     = $00000004;
  CLICKTAB_ACTIVEFILLGRAD   = $00000008;
  CLICKTAB_ACTIVEPENMASK    = $000000F0;
  CLICKTAB_ACTIVEPENTEXT    = $00000000;
  CLICKTAB_ACTIVEPENHLTEXT  = $00000010;
  CLICKTAB_ACTIVEPENTITLE   = $00000020;
  CLICKTAB_ACTIVEFILLMASK   = $00000F00;
  CLICKTAB_ACTIVEFILLFILL   = $00000000;
  CLICKTAB_ACTIVEFILLSHINE  = $00000100;
  CLICKTAB_ACTIVEFILLSELECT = $00000200;
  CLICKTAB_ACTIVEFILLNONE   = $00000F00;
  CLICKTAB_ACTIVEBRIGHT     = $00001000;

// Values for GUIA_GaugeLook and GUIA_WindowGaugeLook
  GAUGE_FILL_STYLEMASK  = $00000007;
  GAUGE_FILL_BORDERLESS = $00000000;
  GAUGE_FILL_3D         = $00000001;
  GAUGE_FILL_FRAMED     = $00000002;
  GAUGE_FILL_FLAT       = $00000003;
  GAUGE_FILL_AUTOSTYLE  = $00000007;
  GAUGE_FILL_GRADIENT   = $00000010;
  GAUGE_CONT_STYLEMASK  = $00000700;
  GAUGE_CONT_BORDERLESS = $00000000;
  GAUGE_CONT_3D         = $00000100;
  GAUGE_CONT_FRAMED     = $00000200;
  GAUGE_CONT_FLAT       = $00000300;
  GAUGE_CONT_FRAMED2    = $00000400;
  GAUGE_CONT_AUTOSTYLE  = $00000700;
  GAUGE_CONT_GRADIENT   = $00001000;
  GAUGE_SPACINGMASK     = $00030000;
  GAUGE_INNERTICKS      = $00040000;
  GAUGE_UNUSED          = $00080000;
  GAUGE_BORDERPENS      = $00100000;
  GAUGE_RESERVED        = $80000000;

// Special value for GUIA_PaletteRGBTable
  PALETTERGB_IGNORE = not 0;

// Special value for GUIA_PaletteLockTable
  PALETTELOCK_IGNORE = not 0;

// Special value for GUIA_DRIPens
  DRIPENS_IGNORE = not 0;

// Window title positions
  WTPOS_LEFT      = 0;  // Left side (default)
  WTPOS_CENTERREL = 1;  // Centered in dragbar
  WTPOS_CENTERABS = 2;  // Centered in whole titlebar

// Menu types
  MT_PULLDOWN = 0;  // Always pulldown (default)
  MT_POPUP    = 1;  // Always pop-up
  MT_RELATIVE = 2;  // Pulldown or pop-up depending on position

/// Menu flags
  MENUTRANSP   = $00000001;  // Menus have transparency
  MENUSHADOW   = $00000002;  // Menus have drop shadows
  MENUALTERN   = $00000010;  // Alternative look (rounded corners)
  MENUFRAMED   = $00000020;  // Framed style for menu edges
  MENUFLAT     = $00000040;  // Never use embossed effects
  MENUTRACK    = $00000100;  // Remember last selections
  MENUSTICKYPD = $00000200;  // Use sticky pulldown menu panels
  MENUSTICKYPU = $00000400;  // Use sticky pop-up menu panels
  MENUCMCORNER = $00000800;  // Alt. position for context menu panels
  MENUNBPD     = $00001000;  // Use non-blocking pulldown menu panels
  MENUNBPU     = $00002000;  // Use non-blocking pop-up menu panels
  MENUPDTOPDEL = $00010000;  // Use open delay with pulldown titles
  MENUPDTCLDEL = $00020000;  // Use close delay with pulldown titles

// === Remember
// this structure is used for remembering what memory has been allocated to date by a given routine,
// so that a premature abort or systematic exit can deallocate memory cleanly, easily, and completely
type
  PRemember = ^TRemember;
  TRemember = record
    NextRemember: PRemember;
    RememberSize: LongWord;
    Memory: PByte;
  end;

// === Color Spec
// How to tell Intuition about RGB values for a color table entry.
  PColorSpec = ^TColorSpec;
  TColorSpec = record
    ColorIndex: SmallInt; // -1 terminates an array of ColorSpec
    Red: Word;            // only the _bottom_ 4 bits recognized
    Green: Word;          // only the _bottom_ 4 bits recognized
    Blue: Word;           // only the _bottom_ 4 bits recognized
  end;

// === Easy Requester Specification
// see also autodocs for EasyRequest and BuildEasyRequest
// NOTE: This structure may grow in size in the future
// NOTE: This structure actually grew in size for V50
  PEasyStruct = ^TEasyStruct;
  TEasyStruct = record
    es_StructSize: LongWord; // should be SizeOf(TEasyStruct)
    es_Flags: LongWord;      // should be 0 for now
    es_Title: STRPTR;        // title of requester window
    es_TextFormat: STRPTR;   // 'printf' style formatting string
    es_GadgetFormat: STRPTR; // 'printf' style formatting string
    es_Screen: PScreen;      // screen to open on (new for V50)
    es_TagList: PTagItem;    // additional information (new for V50)
  end;
const
// EasyRequester flags
  ESF_SCREEN   = $00000001; // Open on the screen specified in es_Screen
  ESF_TAGGED   = $00000002; // Apply attributes specified in es_TagList
  ESF_EVENSIZE = $00000004; // Make all requester buttons equally wide
  ESF_INACTIVE = $00000008; // Open requester window in inactive state
// EasyRequester tags
  ESA_Dummy      = $00340000;
  ESA_Position   = ESA_Dummy + 1; // Where the requester will open
  ESA_Underscore = ESA_Dummy + 2; // Underscore character
// Values for ESA_Position
  REQPOS_DEFAULT      = 0; // Place according to user's GUI preferences
  REQPOS_CORNER       = 1; // Place at top left corner
  REQPOS_BELOWBAR     = 2; // Place at top left corner, below screen bar
  REQPOS_CENTERMOUSE  = 3; // Center under mouse pointer
  REQPOS_CENTERSCREEN = 4; // Center in visible screen clip
  REQPOS_CENTERWINDOW = 5; // Center in reference window

type
// === Gradient Specification
// This is the structure you pass to DrawGradient(). The Direction field tells Intuition the orientation of the gradient; the Mode field is a
// combination of gradient type and flags (see definitions below). Depending on the type, either the Rel or Abs specifications are used;
// Rel is used for the SIMPLE and SHADE types, Abs for the COLOR type. In the case of Rel, the Contrast values indicate the dark and bright
// shading percentages for the opposite ends of the gradient, to be applied to the BasePen color (SIMPLE) or to the background (SHADE).
// Contrast values of 0, 0 produce no gradient at all, whereas contrast values of 255, 255 produce a black-to-white gradient.
  PGradientSpec = ^TGradientSpec;
  TGradientSpec = record
    Direction: LongWord; // X:Y ratio as returned by DirectionVector(angle)
    Mode: Word;          // See definitions below
    case longint of
      0:(
        Rel: record
          BasePen: Word;                 // Base DrawInfo pen to be shaded
          Contrast: array[0..1] of Byte; // Shade levels for gradient ends
        end;
        );
      1:(
         Abs: record
          RGBStart: array[0..2] of Byte; // Starting color of gradient
          RGBEnd: array[0..2] of Byte;   // Ending color of gradient
          Reserved: array[0..3] of Byte; // For future use, leave to zero
        end;
        );
  end;
const
// Gradient modes (type and flags packed in a single word)
  GRADMODE_TYPEMASK  = $000F; // Mask for type extraction
  GRADMODE_NONE      = $0000; // Don't draw any gradient
  GRADMODE_SIMPLE    = $0001; // Shades of BasePen, from Contrast[0] to Contrast[1]
  GRADMODE_COLOR     = $0002; // Shades of color, from RGBStart to RGBEnd
  GRADMODE_SHADE     = $0003; // Shades of background, from Contrast[0] to Contrast[1]
  GRADMODE_IMPOSE    = $0100; // Superimpose gradient to existing background
  GRADMODE_KEEPRATIO = $0200; // Don't scale gradient to domain aspect ratio
  GRADMODE_PALETTE   = $0400; // Rel.BasePen is a palette index, not a DrawInfo pen index
  GRADMODE_INVERT1   = $1000; // Invert the meaning of Contrast[0] (brighten rather than darken)
  GRADMODE_INVERT2   = $2000; // Invert the meaning of Contrast[1] (darken rather than brighten)

// === Miscellaneous
const
// = MENU STUFF
  NOMENU   = $001F;
  NOITEM   = $003F;
  NOSUB    = $001F;
  MENUNULL = -1;

// these defines are for the COMMSEQ and CHECKIT menu stuff.  If CHECKIT, I'll use a generic
// Width (for all resolutions) for the CheckMark. If COMMSEQ, likewise I'll use this generic stuff
  CHECKWIDTH    = 19;
  COMMWIDTH     = 27;
  LOWCHECKWIDTH = 13;
  LOWCOMMWIDTH  = 16;
// these are the AlertNumber defines.  if you are calling DisplayAlert() the AlertNumber you supply
// must have the ALERT_TYPE bits set to one of these patterns
  ALERT_TYPE     = $80000000;
  RECOVERY_ALERT = $00000000; // the system can recover from this
  DEADEND_ALERT  = $80000000; // no recovery possible, this is it
// When you're defining IntuiText for the Positive and Negative Gadgets created by a call to AutoRequest(), these defines will get you
// reasonable-looking text.  The only field without a define is the IText field; you decide what text goes with the Gadget
  AUTOFRONTPEN  = 0;
  AUTOBACKPEN   = 1;
  AUTODRAWMODE  = JAM2;
  AUTOLEFTEDGE  = 6;
  AUTOTOPEDGE   = 3;
  AUTOITEXTFONT = nil;
  AUTONEXTTEXT  = nil;
// --- RAWMOUSE Codes and Qualifiers (Console OR IDCMP)
  SELECTUP   = IECODE_LBUTTON + IECODE_UP_PREFIX;
  SELECTDOWN = IECODE_LBUTTON;
  MENUUP     = IECODE_RBUTTON + IECODE_UP_PREFIX;
  MENUDOWN   = IECODE_RBUTTON;
  MIDDLEUP   = IECODE_MBUTTON + IECODE_UP_PREFIX;
  MIDDLEDOWN = IECODE_MBUTTON;
  ALTLEFT    = IEQUALIFIER_LALT;
  ALTRIGHT   = IEQUALIFIER_RALT;
  AMIGALEFT  = IEQUALIFIER_LCOMMAND;
  AMIGARIGHT = IEQUALIFIER_RCOMMAND;
  AMIGAKEYS  = AMIGALEFT + AMIGARIGHT;

  CURSORUP        = $4C; // Same as RAWKEY_CRSRUP
  CURSORLEFT      = $4F; // Same as RAWKEY_CRSRLEFT
  CURSORRIGHT     = $4E; // Same as RAWKEY_CRSRRIGHT
  CURSORDOWN      = $4D; // Same as RAWKEY_CRSRDOWN
  // WARNING: The following codes are for the US keyboard only
  KEYCODE_Q       = $10;
  KEYCODE_X       = $32;
  KEYCODE_N       = $36;
  KEYCODE_M       = $37;
  KEYCODE_V       = $34;
  KEYCODE_B       = $35;
  KEYCODE_LESS    = $38;
  KEYCODE_GREATER = $39;

// === DrawInfo
const
  // DRI_VERSION =
  //   1 corresponds to V37 release.
  //   2 corresponds to V39, and includes three new pens and the dri_CheckMark and dri_AmigaKey fields.
  //   3 corresponds to V50, and includes 33 new pens and the dri_Screen and the dri_Prefs fields.
  DRI_VERSION = 3;
type
  PDrawInfo = ^TDrawInfo;
  TDrawInfo = record
    dri_Version: Word;     // will be  DRI_VERSION
    dri_NumPens: Word;     // guaranteed to be >= numDrIPens
    dri_Pens: PWord;       // pointer to pen array
    dri_Font: PTextFont;   // screen default font
    dri_Depth: Word;       // (initial) depth of screen bitmap
    dri_Resolution: record // from DisplayInfo database for initial display mode
      x: word;
      y: word;
    end;
    dri_Flags: LongWord;   // defined below
    // New for V39: dri_CheckMark, dri_AmigaKey.
    dri_CheckMark: PImage; // pointer to scaled checkmark image Will be nil if DRI_VERSION < 2
    dri_AmigaKey: PImage;  // pointer to scaled Amiga-key image Will be nil if DRI_VERSION < 2
    // New for V50: dri_Screen, dri_Prefs.
    dri_Screen: PScreen;   // pointer to associated screen Will be nil if DRI_VERSION < 3
    dri_Prefs: APTR;       // opaque handle to GUI settings Will be NULL if DRI_VERSION < 3
    dri_Reserved: array[0..2] of LongWord; // avoid recompilation ;^)
  end;

const
  DRIF_NEWLOOK    = $00000001; // specified SA_Pens, full treatment
  DRIF_NEWDISABLE = $00000002; // fancy look for disabled items (V50)
  DRIF_REALSHADE  = $00000004; // realistic bitmap shading (V50)

// rendering pen number indexes into DrawInfo.dri_Pens[]
  DETAILPEN = $0000;        // compatible Intuition rendering pens
  BLOCKPEN = $0001;         // compatible Intuition rendering pens
  TEXTPEN = $0002;          // text on background
  SHINEPEN = $0003;         // bright edge on 3D objects
  SHADOWPEN = $0004;        // dark edge on 3D objects
  FILLPEN = $0005;          // active-window/selected-gadget fill
  FILLTEXTPEN = $0006;      // text over FILLPEN
  BACKGROUNDPEN = $0007;    // always color 0
  HIGHLIGHTTEXTPEN = $0008; // special color text, on background
  // New for V39, only present if DRI_VERSION >= 2:
  BARDETAILPEN = $0009;     // text/detail in screen-bar/menus
  BARBLOCKPEN = $000A;      // screen-bar/menus fill
  BARTRIMPEN = $000B;       // trim under screen-bar
// New for V50, only present if DRI_VERSION >= 3:
  BARCONTOURPEN         = $000C; // contour above screen-bar
  FOREGROUNDPEN         = $000D; // inside of unselected gadgets
  FORESHINEPEN          = $000E; // bright edges of unselected gadgets
  FORESHADOWPEN         = $000F; // dark edges of unselected gadgets
  FILLSHINEPEN          = $0010; // bright edges for FILLPEN
  FILLSHADOWPEN         = $0011; // dark edges for FILLPEN
  INACTIVEFILLPEN       = $0012; // inactive window borders fill
  INACTIVEFILLTEXTPEN   = $0013; // text over INACTIVEFILLPEN
  INACTIVEFILLSHINEPEN  = $0014; // bright edges for INACTIVEFILLPEN
  INACTIVEFILLSHADOWPEN = $0015; // dark edges for INACTIVEFILLPEN
  DISABLEDPEN           = $0016; // background of disabled elements
  DISABLEDTEXTPEN       = $0017; // text of disabled string gadgets
  DISABLEDSHINEPEN      = $0018; // bright edges of disabled elements
  DISABLEDSHADOWPEN     = $0019; // dark edges of disabled elements
  MENUBACKGROUNDPEN     = $001A; // background of menus
  MENUTEXTPEN           = $001B; // normal text in menus
  MENUSHINEPEN          = $001C; // bright edges of menus
  MENUSHADOWPEN         = $001D; // dark edges of menus
  SELECTPEN             = $001E; // background of selected items
  SELECTTEXTPEN         = $001F; // text of selected items
  SELECTSHINEPEN        = $0020; // bright edges of selected items
  SELECTSHADOWPEN       = $0021; // dark edges of selected items
  GLYPHPEN              = $0022; // system gadget glyphs, outlines
  GLYPHFILLPEN          = $0023; // system gadget glyphs, colored areas
  INACTIVEGLYPHPEN      = $0024; // system gadget glyphs, inact. windows
  RESERVEDPEN           = $0025; // reserved - don't use
  GADGETPEN             = $0026; // gadget symbols (arrows, cycle, etc.)
  TITLEPEN              = $0027; // title of gadget groups
  HALFSHINEPEN          = $0028; // half-bright edge on 3D objects
  HALFSHADOWPEN         = $0029; // half-dark edge on 3D objects
  FLATBORDERPEN         = $002A; // flat (non-3D) borders and frames
  FILLFLATPEN           = $002B; // flat outlines of active windows
  INACTIVEFILLFLATPEN   = $002C; // flat outlines of inactive windows

  NUMDRIPENS = $002D;

// New for V39:  It is sometimes useful to specify that a pen value is to be the complement of color zero to three.  The "magic" numbers serve that purpose:
  PEN_C3 =  $FEFC; // Complement of color 3
  PEN_C2 =  $FEFD; // Complement of color 2
  PEN_C1 =  $FEFE; // Complement of color 1
  PEN_C0 =  $FEFF; // Complement of color 0

const
// --- FLAGS SET BY INTUITION
// The SCREENTYPE bits are reserved for describing various Screen types available under Intuition.
  SCREENTYPE_F   = $000F; // all the screens types available
// --- the definitions for the Screen Type
  WBENCHSCREEN_F = $0001; // Ta Da!  The Workbench
  PUBLICSCREEN_F = $0002; // Public shared (custom) screen
  CUSTOMSCREEN_F = $000F; // for that special look
  SHOWTITLE_F    = $0010; // this gets set by a call to ShowTitle()
  BEEPING_F      = $0020; // set when Screen is beeping
  CUSTOMBITMAP_F = $0040; // if you are supplying your own BitMap
  SCREENBEHIND_F = $0080; // if you want your screen to open behind already open screens
  SCREENQUIET_F  = $0100; // if you do not want Intuition to render into your screen (gadgets, title)
  SCREENHIRES    = $0200; // do no use lowres gadgets (private)
  NS_EXTENDED    = $1000; // TExtNewScreen.Extension is valid
  // V36 applications can use OpenScreenTagList() instead of NS_EXTENDED
  AUTOSCROLL     = $4000; // screen is to autoscoll
  // New for V39:
  PENSHARED      = $0400; // Screen opener set (SA_SharePens,TRUE)

  STDSCREENHEIGHT = -1; // supply in NewScreen.Height
  STDSCREENWIDTH  = -1; // supply in NewScreen.Width

// Screen attribute tag ID's.  These are used in the ti_Tag field of TagItem arrays passed to OpenScreenTagList() (or in the ExtNewScreen.Extension field).
  SA_Dummy    =    TAG_USER + 32;
// these items specify items equivalent to fields in TNewScreen
  SA_Left     = SA_Dummy + $0001;
  SA_Top      = SA_Dummy + $0002;
  SA_Width    = SA_Dummy + $0003;
  SA_Height   = SA_Dummy + $0004; // traditional screen positions and dimensions
  SA_Depth    = SA_Dummy + $0005; // screen bitmap depth
  SA_DetailPen= SA_Dummy + $0006; // serves as default for windows, too
  SA_BlockPen = SA_Dummy + $0007;
  SA_Title    = SA_Dummy + $0008; // default screen title
  SA_Colors   = SA_Dummy + $0009; // ti_Data is an array of struct ColorSpec, terminated by ColorIndex = -1.  Specifies initial screen palette colors.
  SA_ErrorCode= SA_Dummy + $000A; // ti_Data points to LONG error code (values below)
  SA_Font     = SA_Dummy + $000B; // equiv. to NewScreen.Font
  SA_SysFont  = SA_Dummy + $000C; // Selects one of the preferences system fonts: 0 - old DefaultFont, fixed-width 1 - WB Screen preferred font
  SA_Type     = SA_Dummy + $000D; // equiv. to NewScreen.Type
  SA_BitMap   = SA_Dummy + $000E; // ti_Data is pointer to custom BitMap.  This implies type of CUSTOMBITMAP
  SA_PubName  = SA_Dummy + $000F; // presence of this tag means that the screen is to be a public screen.  Please specify BEFORE the two tags below
  SA_PubSig   = SA_Dummy + $0010;
  SA_PubTask  = SA_Dummy + $0011; // Task ID and signal for being notified that the last window has closed on a public screen.
  SA_DisplayID= SA_Dummy + $0012; // ti_Data is new extended display ID
  SA_DClip    = SA_Dummy + $0013; // ti_Data points to a rectangle which defines screen display clip region
  SA_Overscan = SA_Dummy + $0014; // was S_STDDCLIP.  Set to one of the OSCAN_ specifiers below to get a system standard
                                 // overscan region for your display clip, screen dimensions (unless otherwise specified),
                                 // and automatically centered position (partial support only so far). If you use this, you shouldn't specify
                                 // SA_DClip.  SA_Overscan is for "standard" overscan dimensions, SA_DClip is for your custom numeric specifications.
  SA_Obsolete1  = SA_Dummy + $0015; // obsolete S_MONITORNAME
  // booleans
  SA_ShowTitle  = SA_Dummy + $0016; // boolean equivalent to flag SHOWTITLE
  SA_Behind     = SA_Dummy + $0017; // boolean equivalent to flag SCREENBEHIND
  SA_Quiet      = SA_Dummy + $0018; // boolean equivalent to flag SCREENQUIET
  SA_AutoScroll = SA_Dummy + $0019; // boolean equivalent to flag AUTOSCROLL
  SA_Pens       = SA_Dummy + $001A; //  pointer to ~0 terminated UWORD array, as found in struct DrawInfo
  SA_FullPalette= SA_Dummy + $001B; // boolean: initialize color table to entire preferences palette (32 for V36), rather
                                    //   than compatible pens 0-3, 17-19, with remaining palette as returned by GetColorMap()
  SA_ColorMapEntries = SA_Dummy + $001C; // New for V39: Allows you to override the number of entries in the ColorMap for your screen.  Intuition
                                         // normally allocates (1<<depth) or 32, whichever is more, but you may require even more if you
                                         // use certain V39 graphics.library features (eg. palette-banking).
  SA_Parent      = SA_Dummy + $001D; // New for V39: ti_Data is a pointer to a "parent" screen to
                                     // attach this one to.  Attached screens slide and depth-arrange together.
  SA_Draggable   = SA_Dummy + $001E; // New for V39: Boolean tag allowing non-draggable screens.
                                     // Do not use without good reason! (Defaults to TRUE).
  SA_Exclusive   = SA_Dummy + $001F; // New for V39: Boolean tag allowing screens that won't share the display.
                                     // Use sparingly!  Starting with 3.01, attached screens may be SA_Exclusive.
                                     // Setting SA_Exclusive for each screen will produce an exclusive family. (Defaults to FALSE).
  SA_SharePens   = SA_Dummy + $0020; // New for V39: For those pens in the screen's PDrawInfo^.dri_Pens, Intuition obtains them in shared mode (see
                                     // graphics.library/ObtainPen()). For compatibility, Intuition obtains the other pens of a public
                                     // screen as PEN_EXCLUSIVE.  Screens that wish to manage the pens themselves should generally set
                                     // this tag to TRUE.  This instructs Intuition to leave the other pens unallocated.
  SA_BackFill    = SA_Dummy + $0021; // New for V39: provides a "backfill hook" for your screen's
                                     // Layer_Info. See layers.library/InstallLayerInfoHook()
  SA_Interleaved = SA_Dummy + $0022; // New for V39: Boolean tag requesting that the bitmap allocated for you be interleaved. (Defaults to FALSE).
  SA_Colors32    = SA_Dummy + $0023; // New for V39: Tag to set the screen's initial palette colors at 32 bits-per-gun.  ti_Data is a pointer
                                     // to a table to be passed to the graphics.library/LoadRGB32() function. This format supports both runs of color
                                     // registers and sparse registers.  See the autodoc for that function for full details.
                                     // Any color set here has precedence over the same register set by SA_Colors.
  SA_VideoControl = SA_Dummy + $0024;// New for V39: ti_Data is a pointer to a taglist that Intuition
                                     // will pass to graphics.library/VideoControl(), upon opening the screen.
  SA_FrontChild  = SA_Dummy + $0025; // New for V39: ti_Data is a pointer to an already open screen that is to be the child of the screen being
                                     // opened.  The child screen will be moved to the front of its family.
  SA_BackChild   = SA_Dummy + $0026; // New for V39: ti_Data is a pointer to an already open screen that is to be the child of the screen being
                                     // opened.  The child screen will be moved to the back of its family.
  SA_LikeWorkbench = SA_Dummy + $0027; // New for V39: Set ti_Data to 1 to request a screen which is just like the Workbench.  This gives
                                       // you the same screen mode, depth, size, colors, etc., as the Workbench screen.
  SA_Reserved    = SA_Dummy + $0028; // Reserved for private Intuition use
  SA_MinimizeISG = SA_Dummy + $0029; // New for V40: For compatibility, Intuition always ensures that the inter-screen gap is at least three
                                     // non-interlaced lines.  If your application would look best with the smallest possible
                                     // inter-screen gap, set ti_Data to TRUE. If you use the new graphics VideoControl()
                                     // VC_NoColorPaletteLoad tag for your screen's ViewPort, you should also set this tag.
  SA_OffScreenDragging = SA_Dummy + $002a; // New for V50: When TRUE, windows can be dragged off the screen.
  SA_Reserved2         = SA_Dummy + $002b; // Reserved for private Intuition use. V50.
  SA_ActiveWindow      = SA_Dummy + $002c; // the active window of the screen. V50.
  SA_MaxWindowBox      = SA_Dummy + $002d; // (PIBox). V50. PRIVATE
  SA_Reserved3         = SA_Dummy + $002e; // Reserved for private Intuition use. V51.
  SA_Compositing       = SA_Dummy + $002f; // When TRUE, compositing mode is enabled for layers of this screen. Set to FALSE to force
                                           // traditional layer handling. Set to (not 0) (or don't pass this tag at all) to respect user
                                           // preferences for this attribute. V53.
  SA_WindowDropShadows = SA_Dummy + $0030; // Allow drop shadows on window sides, if requested. Set to FALSE to prevent shadows from ever being
                                           // drawn. This tag is ignored if compositing is not enabled for the screen. Defaults to TRUE. V53. */

// this is an obsolete tag included only for compatibility with V35 interim release for the A2024 and Viking monitors
  NSTAG_EXT_VPMODE = TAG_USER + 1;


// OpenScreen error codes, which are returned in the (optional) LongInt pointed to by ti_Data for the SA_ErrorCode tag item
  OSERR_NOMONITOR    = 1;  // named monitor spec not available
  OSERR_NOCHIPS      = 2;  // you need newer custom chips
  OSERR_NOMEM        = 3;  // couldn't get normal memory
  OSERR_NOCHIPMEM    = 4;  // couldn't get chipmem
  OSERR_PUBNOTUNIQUE = 5;  // public screen name already used
  OSERR_UNKNOWNMODE  = 6;  // don't recognize mode asked for
  OSERR_TOODEEP      = 7;  // Screen deeper than HW supports
  OSERR_ATTACHFAIL   = 8;  // Failed to attach screens
  OSERR_NOTAVAILABLE = 9;  // Mode not available for other reason
  OSERR_BADBITMAP    = 10; // Custom bitmap not displayable (V51)

// === NewScreen
type
  PNewScreen = ^TNewScreen;
  TNewScreen = record
    LeftEdge, TopEdge, Width, Height, Depth: SmallInt; // screen dimensions
    DetailPen, BlockPen: Byte; // for bar/border/gadget rendering
    ViewModes: Word;           // the Modes for the ViewPort (and View)
    Type_: Word;               // the Screen type (see defines above)
    Font: PTextAttr;           // this Screen's default text attributes
    DefaultTitle: STRPTR;      // the default title for this Screen
    Gadgets: PGadget;          // your own Gadgets for this Screen
    // if you are opening a CUSTOMSCREEN and already have a BitMap that you want used for your Screen, you set the flags CUSTOMBITMAP in
    // the Type field and you set this variable to point to your BitMap structure.  The structure will be copied into your Screen structure,
    // after which you may discard your own BitMap if you want
    CustomBitMap: PBitMap;
  end;

// For compatibility reasons, we need a new structure for extending NewScreen.  Use this structure is you need to use the new Extension field.
// NOTE: V36-specific applications should use the OpenScreenTagList( newscreen, tags ) version of OpenScreen().
// Applications that want to be V34-compatible as well may safely use the ExtNewScreen structure.  Its tags will be ignored by V34 Intuition.
  PExtNewScreen = ^TExtNewScreen;
  TExtNewScreen = record
    LeftEdge, TopEdge, Width, Height, Depth: SmallInt;
    DetailPen, BlockPen: Byte;
    ViewModes: Word;
    Type_: Word;
    Font: PTextAttr;
    DefaultTitle: STRPTR;
    Gadgets: PGadget;
    CustomBitMap: PBitMap;
    Extension: PTagItem;
  end;

const
// === Overscan Types
  OSCAN_TEXT     = 1; // entirely visible
  OSCAN_STANDARD = 2; // just past edges
  OSCAN_MAX      = 3; // as much as possible
  OSCAN_VIDEO    = 4; // even more than is possible

// === Public Shared Screen Node
// This is the representative of a public shared screen. This is an internal data structure, but some functions may
// present a copy of it to the calling application.  In that case, be aware that the screen pointer of the structure can NOT be
// used safely, since there is no guarantee that the referenced screen will remain open and a valid data structure. Never change one of these.
type
  PPubScreenNode = ^TPubScreenNode;
  TPubScreenNode = record
    psn_Node: TNode;            // ln_Name is screen name
    psn_Screen: PScreen;
    psn_Flags: Word;            // below
    psn_Size: SmallInt;         // includes name buffer
    psn_VisitorCount: SmallInt; // how many visitor windows
    psn_SigTask: PTask;         // who to signal when visitors gone
    psn_SigBit: Byte;           // which signal
  end;

const
  PSNF_PRIVATE  = $0001;
// NOTE: Due to a bug in NextPubScreen(), make sure your buffer actually has MAXPUBSCREENNAME+1 characters in it!
  MAXPUBSCREENNAME = 139; // names no longer, please
// pub screen modes
  SHANGHAI     = $0001; // put workbench windows on pub screen
  POPPUBSCREEN = $0002; // pop pub screen to front when visitor opens

// New for V39:  Intuition has new screen depth-arrangement and movement functions called ScreenDepth() and ScreenPosition() respectively.
// These functions permit the old behavior of ScreenToFront(), ScreenToBack(), and MoveScreen().  ScreenDepth() also allows
// independent depth control of attached screens.  ScreenPosition() optionally allows positioning screens even though they were opened (SA_Draggable,FALSE).

// For ScreenDepth(), specify one of SDEPTH_TOFRONT or SDEPTH_TOBACK, and optionally also SDEPTH_INFAMILY.
// NOTE: ONLY THE OWNER OF THE SCREEN should ever specify SDEPTH_INFAMILY.  Commodities, "input helper" programs, or any other program that did not open a screen should never
// use that flag.  (Note that this is a style-behavior requirement;  there is no technical requirement that the
// task calling this function need be the task which opened the screen).
  SDEPTH_TOFRONT  = 0; // Bring screen to front
  SDEPTH_TOBACK   = 1; // Send screen to back
  SDEPTH_INFAMILY = 2; // Move an attached screen with respect to other screens of its family

  // Here's an obsolete name equivalent to SDEPTH_INFAMILY:
  SDEPTH_CHILDONLY      =  SDEPTH_INFAMILY;

// For ScreenPosition() the kind of screen positioning you wish to perform
  SPOS_RELATIVE    = 0; // The x1 and y1 parameters to ScreenPosition() describe the offset in coordinates you wish to move the screen by.
  SPOS_ABSOLUTE    = 1; // The x1 and y1 parameters to ScreenPosition() describe the absolute coordinates you wish to move the screen to.
  SPOS_MAKEVISIBLE = 2; // (x1,y1)-(x2,y2) describes a rectangle on the screen which you would like autoscrolled into view.
  SPOS_FORCEDRAG   = 4; // Move non-draggable screen

// New for V39: Intuition supports double-buffering in screens, with friendly interaction with menus and certain gadgets.
// For each buffer, you need to get one of these structures from the AllocScreenBuffer() call.  Never allocate your  own ScreenBuffer structures!
// The sb_DBufInfo field is for your use.  See the graphics.library AllocDBufInfo() autodoc for details.

type
  PScreenBuffer = ^TScreenBuffer;
  TScreenBuffer = record
    sb_BitMap: PBitMap;     // BitMap of this buffer
    sb_DBufInfo: PDBufInfo; // DBufInfo for this buffer
  end;

const
// These are the flags that may be passed to AllocScreenBuffer().
  SB_SCREEN_BITMAP      =  1;
  SB_COPY_BITMAP        =  2;

// === Preferences
const
// these are the definitions for the printer configurations
  FILENAME_SIZE = 30; // Filename size
  DEVNAME_SIZE  = 16; // Device-name size

  POINTERSIZE   = (1 + 16 + 1) * 2; // Size of Pointer data buffer

// These defines are for the default font size.   These actually describe the height of the defaults fonts.  The default font type is the topaz
// font, which is a fixed width font that can be used in either eighty-column or sixty-column mode.  The Preferences structure reflects
// which is currently selected by the value found in the variable FontSize, which may have either of the values defined below.  These values actually
// are used to select the height of the default font.  By changing the height, the resolution of the font changes as well.
  TOPAZ_EIGHTY = 8;
  TOPAZ_SIXTY  = 9;

type
  PPreferences = ^TPreferences;
  TPreferences = record
    // the default font height
    FontHeight: ShortInt;  // height for system default font
    // constant describing what's hooked up to the port
    PrinterPort: Byte;     // printer port connection
    // the baud rate of the port
    BaudRate: Word;        // baud rate for the serial port
    // various timing rates
    KeyRptSpeed: TTimeVal; // repeat speed for keyboard
    KeyRptDelay: TTimeVal; // Delay before keys repeat
    DoubleClick: TTimeVal; // Interval allowed between clicks
    // Intuition Pointer data
    PointerMatrix: array[0..POINTERSIZE - 1] of Word; // Definition of pointer sprite
    XOffset: Shortint;  // X-Offset for active 'bit'
    YOffset: Shortint;  // Y-Offset for active 'bit'
    color17: Word;      //*********************************
    color18: Word;      // Colours for sprite pointer
    color19: Word;      //*********************************
    PointerTicks: Word; // Sensitivity of the pointer
    // Workbench Screen colors
    color0: Word;       //*********************************
    color1: Word;       //   Standard default colours
    color2: Word;       //   Used in the Workbench
    color3: Word;       //*********************************
    // positioning data for the Intuition View
    ViewXOffset: Shortint; // Offset for top lefthand corner
    ViewYOffset: Shortint; // X and Y dimensions
    ViewInitX: SmallInt;
    ViewInitY: SmallInt;   // View initial offset values
    EnableCLI: WordBool;   // CLI availability switch
    // printer configurations
    PrinterType: Word;     // printer type
    PrinterFilename: array[0..FILENAME_SIZE-1] of Char; // file for printer
    // print format and quality configurations
    PrintPitch: Word;         // print pitch
    PrintQuality: Word;       // print quality
    PrintSpacing: Word;       // number of lines per inch
    PrintLeftMargin: Word;    // left margin in characters
    PrintRightMargin: Word;   // right margin in characters
    PrintImage: Word;         // positive or negative
    PrintAspect: Word;        // horizontal or vertical
    PrintShade: Word;         // b&w, half-tone, or color
    PrintThreshold: SmallInt; // darkness ctrl for b/w dumps
    // print paper descriptors
    PaperSize: Word;   // paper size
    PaperLength: Word; // paper length in number of lines
    PaperType: Word;   // continuous or single sheet
    // Serial device settings: These are six nibble-fields in three bytes
    // (these look a little strange so the defaults will map out to zero)
    SerRWBits: Byte;  // upper nibble = (8-number of read bits)
                      // lower nibble = (8-number of write bits)
    SerStopBuf: Byte; // upper nibble = (number of stop bits - 1)
                      // lower nibble = (table value for BufSize)
    SerParShk: Byte;  // upper nibble = (value for Parity setting)
                      // lower nibble = (value for Handshake mode)
    LaceWB: Byte;     // if workbench is to be interlaced
    Pad: array[0..FILENAME_SIZE - 1] of Char; // This was UBYTE WorkName[FILENAME_SIZE]; (temp file for printer) in old versions.
    PrinterDevPrivateFlags: Byte;    // system private (V51)
    PrtDevOpenDeviceFlags: LongWord; // flags for device below (V51)
    PrtDevName: array[0..DEVNAME_SIZE - 1] of Char; // device used by printer.device (omit the ".device")
    DefaultPrtUnit: Byte;      // default unit opened by printer.device
    DefaultSerUnit: Byte;      // default serial unit
    RowSizeChange: SmallInt;   // affect NormalDisplayRows/Columns
    ColumnSizeChange: SmallInt;

    PrintFlags: Word;     // user preference flags
    PrintMaxWidth: Word;  // max width of printed picture in 10ths/inch
    PrintMaxHeight: Word; // max height of printed picture in 10ths/inch
    PrintDensity: Byte;   // print density
    PrintXOffset: Byte;   // offset of printed picture in 10ths/inch

    wb_Width: Word;  // override default workbench width
    wb_Height: Word; // override default workbench height
    wb_Depth: Byte;  // override default workbench depth

    ext_size: Byte;  // extension information -- do not touch! extension size in blocks of 64 bytes
  end;

const
// Workbench Interlace (use one bit)
  LACEWB      = 1 shl 0;
  LW_RESERVED = 1;       // internal use only
// Enable_CLI
  SCREEN_DRAG  = 1 shl 14;
  MOUSE_ACCEL  = 1 shl 15;
// PrinterPort
  PARALLEL_PRINTER = $00;
  SERIAL_PRINTER   = $01;
// BaudRate
  BAUD_110   = $00;
  BAUD_300   = $01;
  BAUD_1200  = $02;
  BAUD_2400  = $03;
  BAUD_4800  = $04;
  BAUD_9600  = $05;
  BAUD_19200 = $06;
  BAUD_MIDI  = $07;
// PaperType
  FANFOLD_PT = $00;
  SINGLE_PT  = $80;
// PrintPitch
  PICA  = $000;
  ELITE = $400;
  FINE  = $800;
// PrintQuality
  DRAFT  = $000;
  LETTER = $100;
// PrintSpacing
  SIX_LPI   = $000;
  EIGHT_LPI = $200;
// Print Image
  IMAGE_POSITIVE = $00;
  IMAGE_NEGATIVE = $01;
// PrintAspect
  ASPECT_HORIZ = $00;
  ASPECT_VERT  = $01;
// PrintShade
  SHADE_BW        = $00;
  SHADE_GREYSCALE = $01;
  SHADE_COLOR     = $02;
// PaperSize (all paper sizes have a zero in the lowest nybble)
  US_LETTER    = $00;
  US_LEGAL     = $10;
  N_TRACTOR    = $20;
  W_TRACTOR    = $30;
  CUSTOM_PAPER = $40;
// New PaperSizes for V36:
  EURO_A0 = $50; // European size A0: 841 x 1189
  EURO_A1 = $60; // European size A1: 594 x 841
  EURO_A2 = $70; // European size A2: 420 x 594
  EURO_A3 = $80; // European size A3: 297 x 420
  EURO_A4 = $90; // European size A4: 210 x 297
  EURO_A5 = $A0; // European size A5: 148 x 210
  EURO_A6 = $B0; // European size A6: 105 x 148
  EURO_A7 = $C0; // European size A7: 74 x 105
  EURO_A8 = $D0; // European size A8: 52 x 74
// PrinterType
  CUSTOM_NAME         = $00;
  ALPHA_P_101         = $01;
  BROTHER_15XL        = $02;
  CBM_MPS1000         = $03;
  DIAB_630            = $04;
  DIAB_ADV_D25        = $05;
  DIAB_C_150          = $06;
  EPSON               = $07;
  EPSON_JX_80         = $08;
  OKIMATE_20          = $09;
  QUME_LP_20          = $0A;
// new printer entries, 3 October 1985
  HP_LASERJET         = $0B;
  HP_LASERJET_PLUS    = $0C;
// Serial Input Buffer Sizes
  SBUF_512            = $00;
  SBUF_1024           = $01;
  SBUF_2048           = $02;
  SBUF_4096           = $03;
  SBUF_8000           = $04;
  SBUF_16000          = $05;
// Serial Bit Masks
  SREAD_BITS          = $F0; // for SerRWBits
  SWRITE_BITS         = $0F;
  SSTOP_BITS          = $F0; // for SerStopBuf
  SBUFSIZE_BITS       = $0F;
  SPARITY_BITS        = $F0; // for SerParShk
  SHSHAKE_BITS        = $0F;
// Serial Parity (upper nibble, after being shifted by macro SPARNUM() )
  SPARITY_NONE        = 0;
  SPARITY_EVEN        = 1;
  SPARITY_ODD         = 2;
// Serial Handshake Mode (lower nibble, after masking using macro SHANKNUM() )
  SHSHAKE_XON         = 0;
  SHSHAKE_RTS         = 1;
  SHSHAKE_NONE        = 2;
// new defines for PrintFlags
  CORRECT_RED         = $0001; // color correct red shades
  CORRECT_GREEN       = $0002; // color correct green shades
  CORRECT_BLUE        = $0004; // color correct blue shades
                               //
  CENTER_IMAGE        = $0008; // center image on paper
                               //
  IGNORE_DIMENSIONS   = $0000; // ignore max width/height settings
  BOUNDED_DIMENSIONS  = $0010; // use max width/height as boundaries
  ABSOLUTE_DIMENSIONS = $0020; // use max width/height as absolutes
  PIXEL_DIMENSIONS    = $0040; // use max width/height as prt pixels
  MULTIPLY_DIMENSIONS = $0080; // use max width/height as multipliers
                               //
  INTEGER_SCALING     = $0100; // force integer scaling
                               //
  ORDERED_DITHERING   = $0000; // ordered dithering
  HALFTONE_DITHERING  = $0200; // halftone dithering
  FLOYD_DITHERING     = $0400; // Floyd-Steinberg dithering
                               //
  ANTI_ALIAS          = $0800; // anti-alias image
  GREY_SCALE2         = $1000; // for use with hi-res monitor
// masks used for checking bits
  CORRECT_RGB_MASK = CORRECT_RED + CORRECT_GREEN + CORRECT_BLUE;
  DIMENSIONS_MASK  = BOUNDED_DIMENSIONS + ABSOLUTE_DIMENSIONS + PIXEL_DIMENSIONS + MULTIPLY_DIMENSIONS;
  DITHERING_MASK   = HALFTONE_DITHERING + FLOYD_DITHERING;

// these are the display modes for which we have corresponding parameter settings in the config arrays
const
  DMODECOUNT = $0002; // how many modes there are
  HIRESPICK  = $0000;
  LOWRESPICK = $0001;

  EVENTMAX = 10; // size of event array
// these are the system Gadget defines
  RESCOUNT       = 2;
  HIRESGADGET    = 0;
  LOWRESGADGET   = 1;

  GADGETCOUNT    = 8;
  UPFRONTGADGET  = 0;
  DOWNBACKGADGET = 1;
  SIZEGADGET     = 2;
  CLOSEGADGET    = 3;
  DRAGGADGET     = 4;
  SUPFRONTGADGET = 5;
  SDOWNBACKGADGET= 6;
  SDRAGGADGET    = 7;

// === IntuitionBase
// Be sure to protect yourself against someone modifying these data as you look at them.  This is done by calling:
// lock := LockIBase(0), which returns an Integer.  When done call UnlockIBase(lock) where lock is what LockIBase() returned.
type
  PIntuitionBase = ^TIntuitionBase;
  TIntuitionBase = record
    LibNode: TLibrary;
    ViewLord: TView;

    ActiveWindow: PWindow;
    ActiveScreen: PScreen;
    // the FirstScreen variable points to the frontmost Screen.   Screens are
    // then maintained in a front to back order using Screen.NextScreen
    FirstScreen: PScreen;   // for linked list of all screens
    Flags: LongWord;        // see definitions below
    MouseY: SmallInt;
    MouseX: SmallInt;       // mouse position relative to View

    Seconds: LongWord;      // timestamp of most current input event
    Micros: LongWord;       // timestamp of most current input event
    // I told you this was private. The data beyond this point has changed, is changing, and will continue to change.
  end;

// Package of information passed to custom and 'boopsi' gadget 'hook' functions.  This structure is READ ONLY.
type
  PGadgetInfo = ^TGadgetInfo;
  TGadgetInfo = record
    gi_Screen: PScreen;
    gi_Window: PWindow;       // nil for screen gadgets
    gi_Requester: PRequester; // nil IF not GTYP_REQGADGET
    gi_RastPort: PRastPort;   // rendering information: don't use these without cloning/locking. Official way is to call ObtainRPort()
    gi_Layer: PLayer;
    gi_Domain: TIBox;         { copy of dimensions of screen/window/g00/req(/group)
                                that gadget resides in.  Left/Top of this box is
                                offset from window mouse coordinates to gadget coordinates
                                         screen gadgets:         0,0 (from screen coords)
                                 window gadgets (no g00):        0,0
                                 GTYP_GZZGADGETs (borderlayer):  0,0
                                 GZZ innerlayer gadget:          borderleft, bordertop
                                 Requester gadgets:              reqleft, reqtop }
    gi_Pens: record           // these are the pens for the window or screen
      DetailPen: Byte;
      BlockPen : Byte;
    end;
    gi_DrInfo: pDrawInfo;     // the Detail and Block pens in gi_DrInfo->dri_Pens[] are for the screen.  Use the above for window-sensitive colors.
    gi_Gadget: PGadget;       // gadget backpointer. New for V50.
    gi_Reserved: array[0..4] of LongWord; // reserved space: this structure is extensible anyway, but using these saves some recompilation
  end;

//** system private data structure for now **
// prop gadget extra info
  PPGX = ^TPGX;
  TPGX = record
    pgx_Container: TIBox;
    pgx_NewKnob: TIBox;
  end;

{
 * Class id strings for Intuition classes.
 * There's no real reason to use the uppercase constants
 * over the lowercase strings, but this makes a good place
 * to list the names of the built-in classes.
 }
const
  ROOTCLASS      : PChar = 'rootclass';
  IMAGECLASS     : PChar = 'imageclass';
  FRAMEICLASS    : PChar = 'frameiclass';
  SYSICLASS      : PChar = 'sysiclass';
  FILLRECTCLASS  : PChar = 'fillrectclass';
  GADGETCLASS    : PChar = 'gadgetclass';
  PROPGCLASS     : PChar = 'propgclass';
  STRGCLASS      : PChar = 'strgclass';
  BUTTONGCLASS   : PChar = 'buttongclass';
  FRBUTTONCLASS  : PChar = 'frbuttonclass';
  GROUPGCLASS    : PChar = 'groupgclass';
  SCROLLERGCLASS : PChar = 'scrollergclass'; // V50
  ICCLASS        : PChar = 'icclass';
  MODELCLASS     : PChar = 'modelclass';
  ITEXTICLASS    : PChar = 'itexticlass';
  POINTERCLASS   : PChar = 'pointerclass';


// Dispatched method ID's
// NOTE: Applications should use Intuition entry points, not direct DoMethod() calls, for NewObject, DisposeObject, SetAttrs,  SetGadgetAttrs, and GetAttr.

  OM_Dummy       = $100;
  OM_NEW         = $101; // 'object' parameter is 'true class'
  OM_DISPOSE     = $102; // delete self (no parameters)
  OM_SET         = $103; // set attributes (in tag list)
  OM_GET         = $104; // return single attribute value
  OM_ADDTAIL     = $105; // add self to a List (let root do it)
  OM_REMOVE      = $106; // remove self from list
  OM_NOTIFY      = $107; // send to self: notify dependents
  OM_UPDATE      = $108; // notification message from somebody
  OM_ADDMEMBER   = $109; // used by various classes with lists
  OM_REMMEMBER   = $10A; // used by various classes with lists

// Parameter 'Messages' passed to methods
// OM_NEW and OM_SET
type
  PopSet = ^TopSet;
  TopSet = record
    MethodID: LongWord;
    ops_AttrList: PTagItem;  // new attributes
    ops_GInfo: PGadgetInfo;  //  always there for gadgets, when SetGadgetAttrs() is used, but will be nil for OM_NEW                                         }
  end;

// OM_NOTIFY, and OM_UPDATE
  PopUpdate = ^TopUpdate;
  TopUpdate = record
    MethodID: LongWord;
    opu_AttrList: PTagItem; // new attributes
    opu_GInfo: PGadgetInfo; // non-NULL when SetGadgetAttrs OR notification resulting from gadget input occurs.
    opu_Flags: LongWord;    // defined below
  end;

{ this flag means that the update message is being issued from
  something like an active gadget, a la GACT_FOLLOWMOUSE.  When
  the gadget goes inactive, it will issue a final update
  message with this bit cleared.  Examples of use are for
  GACT_FOLLOWMOUSE equivalents for propgadclass, and repeat strobes
  for buttons. }
const
  OPUF_INTERIM = 1 shl 1;

// OM_GET
type
  PopGet = ^TopGet;
  TopGet = record
    MethodID: LongWord;
    opg_AttrID: LongWord;
    opg_Storage: PLongWord; // may be other types, but 'integer' types are all LongWord
  end;

// OM_ADDTAIL
  PopAddTail = ^TopAddTail;
  TopAddTail = record
    MethodID: LongWord;
    opat_List: PList;
  end;

// OM_ADDMEMBER, OM_REMMEMBER
type
  PopMember = ^TopMember;
  TopMember = record
    MethodID: LongWord;
    opam_Object: PObject_;
  end;

//***** 'White box' access to struct IClass
// This structure is READ-ONLY, and allocated only by Intuition
type
  PIClass = ^TIClass;
  TIClass = record
    cl_Dispatcher: THook;       // Class dispatcher
    cl_Reserved: LongWord;      // must be 0
    cl_Super: PIClass;          // Pointer to superclass
    cl_ID: ClassID;             // Class ID

    cl_InstOffset: Word;        // Offset of instance data
    cl_InstSize: Word;          // Size of instance data

    cl_UserData: LongWord;      // Class global data
    cl_SubclassCount: LongWord; // Number of subclasses
    cl_ObjectCount: LongWord;   // Number of objects
    cl_Flags: LongWord;
  end;

const
  CLF_INLIST =  $00000001; // class is in public class list




//**** 'White box' access to struct _Object

{ We have this, the instance data of the root class, PRECEDING  the 'object'.  This is so that Gadget objects are Gadget pointers,
  and so on.  If this structure grows, it will always have o_Class at the end, so the macro OCLASS(o) will always have the same
  offset back from the pointer returned from NewObject(). This data structure is subject to change.  Do not use the o_Node
  embedded structure.
 }
type
  P_Object = ^T_Object;
  T_Object = record
    o_Node: TMinNode;
    o_Class: PIClass;
  end;

// BOOPSI class libraries should use this structure as the base for their library data.
// This allows developers to obtain the class pointer for performing object-less inquiries.

  PClassLibrary = ^TClassLibrary;
  TClassLibrary = record
    cl_Lib: TLibrary;  // Embedded library
    cl_Pad: Word;      // Align the structure
    cl_Class: PIClass; // Class pointer
  end;

// Gadget Class attributes
CONST
  GA_Dummy           = TAG_USER + $30000;
  GA_Left            = GA_Dummy + 1;  // (LongInt) Left edge of the gadget relative to the left edge of the window
  GA_RelRight        = GA_Dummy + 2;  // (LongInt) Left edge of the gadget relative to the right edge of the window
  GA_Top             = GA_Dummy + 3;  // (LongInt) Top edge of the gadget relative to the top edge of the window
  GA_RelBottom       = GA_Dummy + 4;  // (LongInt) Top edge of the gadget relative to the bottom edge of the window
  GA_Width           = GA_Dummy + 5;  // (LongInt) Width of the gadget
  GA_RelWidth        = GA_Dummy + 6;  // (LongInt) Width of the gadget relative to the width of the window
  GA_Height          = GA_Dummy + 7;  // (LongInt) Height of the gadget
  GA_RelHeight       = GA_Dummy + 8;  // (LongInt) Height of the gadget relative to the height of the window
  GA_Text            = GA_Dummy + 9;  // (STRPTR) Gadget imagry is #0 terminated string
  GA_Image           = GA_Dummy + 10; // (PImage) Gadget imagry is an image
  GA_Border          = GA_Dummy + 11; // (PBorder) Gadget imagry is a border
  GA_SelectRender    = GA_Dummy + 12; // (PImage) Selected gadget imagry
  GA_Highlight       = GA_Dummy + 13; // (Word) One of GFLG_GADGHNONE, GFLG_GADGHBOX, GFLG_GADGHCOMP, or GFLG_GADGHIMAGE
  GA_Disabled        = GA_Dummy + 14; // (Boolean) Indicate whether gadget is disabled or not. Defaults to FALSE.
  GA_GZZGadget       = GA_Dummy + 15; // (BOOL) Indicate whether the gadget is for WFLG_GIMMEZEROZERO window borders or not. Defaults to FALSE.
  GA_ID              = GA_Dummy + 16; // (Word) Gadget ID assigned by the application
  GA_UserData        = GA_Dummy + 17; // (APTR) Application specific data
  GA_SpecialInfo     = GA_Dummy + 18; // (APTR) Gadget specific data
  GA_Selected        = GA_Dummy + 19; // (BOOL) Indicate whether the gadget is selected or not. Defaults to FALSE
  GA_EndGadget       = GA_Dummy + 20; // (BOOL) When set tells the system that when this gadget is selected causes the requester that it is in to be ended.  Defaults to FALSE.
  GA_Immediate       = GA_Dummy + 21; // (BOOL) When set indicates that the gadget is to notify the application when it becomes active.  Defaults to FALSE.
  GA_RelVerify       = GA_Dummy + 22; // (BOOL) When set indicates that the application wants toverify that the pointer was still over the gadget when
                                       //    the select button is released.  Defaults to FALSE. }
  GA_FollowMouse     = GA_Dummy + 23; // (BOOL) When set indicates that the application wants to be notified of mouse movements while the gadget is active.
                                       //   It is recommmended that GA_Immediate and GA_RelVerify are also used so that the active gadget can be tracked by the
                                       //   application. Defaults to FALSE.
  GA_RightBorder     = GA_Dummy + 24; // (BOOL) Indicate whether the gadget is in the right border or not. Defaults to FALSE.
  GA_LeftBorder      = GA_Dummy + 25; // (BOOL) Indicate whether the gadget is in the left border or not. Defaults to FALSE.
  GA_TopBorder       = GA_Dummy + 26; // (BOOL) Indicate whether the gadget is in the top border or not. Defaults to FALSE.
  GA_BottomBorder    = GA_Dummy + 27; // (BOOL) Indicate whether the gadget is in the bottom border or not. Defaults to FALSE.
  GA_ToggleSelect    = GA_Dummy + 28; // (BOOL) Indicate whether the gadget is toggle-selected or not. Defaults to FALSE.
  GA_SysGadget       = GA_Dummy + 29; // (BOOL) Reserved for system use to indicate that the gadget belongs to the system. Defaults to FALSE.
  GA_SysGType        = GA_Dummy + 30; // (Word) Reserved for system use to indicate the gadget type.
  GA_Previous        = GA_Dummy + 31; // (PGadget) Previous gadget in the linked list.
                                      //   NOTE: This attribute CANNOT be used to link new gadgets into the gadget list of an open window or requester.  You must use AddGList().
  GA_Next            = GA_Dummy + 32; // (PGadget) Next gadget in the linked list.
  GA_DrawInfo        = GA_Dummy + 33; // (PDrawInfo) Some gadgets need a DrawInfo at creation time
  // You should use at most ONE of GA_Text, GA_IntuiText, and GA_LabelImage
  GA_IntuiText       = GA_Dummy + 34; // (PIntuiText) Label is an IntuiText.
  GA_LabelImage      = GA_Dummy + 35; // (PObject_) Label is an image object.
  // New for V37:
  GA_TabCycle        = GA_Dummy + 36; // (BOOL) Indicate whether gadget is part of TAB/SHIFT-TAB cycle activation. Defaults to FALSE
  // New for V39:
  GA_GadgetHelp      = GA_Dummy + 37; // (BOOL) Indicate whether gadget is to send IDCMP_GADGETHELP. Defaults to FALSE
  GA_Bounds          = GA_Dummy + 38; // (PIBox) Copied into the extended gadget's bounds.
  GA_RelSpecial      = GA_Dummy + 39; // (BOOL) Indicate whether gadget has special relativity. Defaults to FALSE
  // New for V42:
  GA_TextAttr       = GA_Dummy + 40; // (PTextAttr) Indicate the font to use for the gadget.
  GA_ReadOnly       = GA_Dummy + 41; // (BOOL) Indicate that the gadget is read-only (non-selectable). Defaults to FALSE
  // New for V44:
  GA_Underscore     = GA_Dummy + 42; // (UBYTE) Underscore/escape character for keyboard shortcuts. Defaults to '_'
  GA_ActivateKey    = GA_Dummy + 43; // (STRPTR) Set/Get the gadgets shortcut/activation key(s) Defaults to nil
  GA_BackFill       = GA_Dummy + 44; // (PHook) Backfill pattern hook. Defaults to nil
  GA_GadgetHelpText = GA_Dummy + 45; // (STRPTR)   RESERVERD/PRIVATE DO NOT USE Defaults to nil
  GA_UserInput      = GA_Dummy + 46; // (BOOL) Notification tag indicates this notification is from the activite
                                     //   gadget receiving user input - an attempt to make IDCMPUPDATE more efficient. Defaults to FALSE
  // New for V50:
  GA_DoLayout       = GA_Dummy + 51; // (BOOL) Set this to TRUE if the gadget should recompute its  position and size when it receives a GM_LAYOUT message. For
                                     //   gadgets which are part of a layout group this attribute should be FALSE; the layout takes care of placing and sizing
                                     //   its members and the resulting geometry must not be changed. The default is usually FALSE for ReAction gadgets, while for
                                     //   others it is defined on a class-by-class basis
  GA_NoFilterMenuKeys = GA_Dummy+52; // (BOOL) Corresponds to GMORE_NOFILTERMENUKEYS
  GA_Titlebar       = GA_Dummy + 53; // (BOOL) Set this to TRUE if you want the gadget to be automatically positioned by Intuition in the window titlebar (at the left or at
                                     // the right side, depending on the state of GFLG_RELRIGHT). The layout is done according to the current style and geometry settings for
                                     // window border gadgets. Please use an appropriate sysiclass instance as imagery for a titlebar gadget, such as ICONIFYIMAGE or TBFRAMEIMAGE. Note: This property implies [GA_TopBorder, TRUE].
  GA_Hidden         = GA_Dummy + 54; // (BOOL) Corresponds to GMORE_HIDDEN
  GA_NoFilterWheel  = GA_Dummy + 55; // (BOOL) Corresponds to GMORE_NOFILTERWHEEL
  GA_HintInfo       = GA_Dummy + 56; // (STRPTR) The default HintInfo string for this gadget. See window.class documentation for more information
  GA_ContextMenu    = GA_Dummy + 57; // (APTR) The default context menu for this gadget. It can be either a traditional menu or a BOOPSI menu tree from menuclass

// PROPGCLASS attributes
  PGA_Dummy      = TAG_USER + $31000;
  PGA_Freedom    = PGA_Dummy + $0001;
  // only one of PGA_FREEVERT or PGA_FREEHORIZ
  PGA_Borderless = PGA_Dummy + $0002;
  PGA_HorizPot   = PGA_Dummy + $0003;
  PGA_HorizBody  = PGA_Dummy + $0004;
  PGA_VertPot    = PGA_Dummy + $0005;
  PGA_VertBody   = PGA_Dummy + $0006;
  PGA_Total      = PGA_Dummy + $0007;
  PGA_Visible    = PGA_Dummy + $0008;
  PGA_Top        = PGA_Dummy + $0009;
  // New for V37:
  PGA_NewLook    = PGA_Dummy + $000A;
  // New for V50: scrollergclass attributes
  PGA_ArrowDelta = PGA_Dummy + $000B;
  PGA_ArrowDown  = PGA_Dummy + $000C;

// STRGCLASS attributes
  STRINGA_Dummy      = TAG_USER + $32000;
  STRINGA_MaxChars   = STRINGA_Dummy + $0001;
  // Note:  There is a minor problem with Intuition when using boopsi integer gadgets (which are requested by using STRINGA_LongVal).  Such gadgets
  // must not have a STRINGA_MaxChars to be bigger than 15. Setting STRINGA_MaxChars for a boopsi integer gadget will cause a mismatched  FreeMem() to occur.
  STRINGA_Buffer     = STRINGA_Dummy + $0002;
  STRINGA_UndoBuffer = STRINGA_Dummy + $0003;
  STRINGA_WorkBuffer = STRINGA_Dummy + $0004;
  STRINGA_BufferPos  = STRINGA_Dummy + $0005;
  STRINGA_DispPos    = STRINGA_Dummy + $0006;
  STRINGA_AltKeyMap  = STRINGA_Dummy + $0007;
  STRINGA_Font       = STRINGA_Dummy + $0008;
  STRINGA_Pens       = STRINGA_Dummy + $0009;
  STRINGA_ActivePens = STRINGA_Dummy + $000A;
  STRINGA_EditHook   = STRINGA_Dummy + $000B;
  STRINGA_EditModes  = STRINGA_Dummy + $000C;

// booleans
  STRINGA_ReplaceMode    = STRINGA_Dummy + $000D;
  STRINGA_FixedFieldMode = STRINGA_Dummy + $000E;
  STRINGA_NoFilterMode   = STRINGA_Dummy + $000F;

  STRINGA_Justification  = STRINGA_Dummy + $0010;
  // GACT_STRINGCENTER, GACT_STRINGLEFT, GACT_STRINGRIGHT
  STRINGA_LongVal        = STRINGA_Dummy + $0011;
  STRINGA_TextVal        = STRINGA_Dummy + $0012;

  STRINGA_ExitHelp       = STRINGA_Dummy + $0013; // STRINGA_ExitHelp is new for V37, and ignored by V36. Set this if you want the gadget to exit when Help is
                                                  // pressed.  Look for a code of $5F, the rawkey code for Help
  // New in V50:
  STRINGA_MarkedBlock    = STRINGA_Dummy + $0014; // Sets/gets the marked block of a string.gadget. The hi-word contains the start position (first marked char) and the
                                                  // lo-word the end position (last marked char).  If both are -1 nothing is marked.
  SG_DEFAULTMAXCHARS = 128;

// Gadget Layout related attributes
  LAYOUTA_Dummy       = TAG_USER  + $38000;
  LAYOUTA_LayoutObj   = LAYOUTA_Dummy + $0001;
  LAYOUTA_Spacing     = LAYOUTA_Dummy + $0002;
  LAYOUTA_Orientation = LAYOUTA_Dummy + $0003;
  // New For V42:
  LAYOUTA_ChildMaxWidth  = LAYOUTA_Dummy + $0004; // (BOOL) Child objects are of equal width.  Should default to TRUE for gadgets with a horizontal orientation.
  LAYOUTA_ChildMaxHeight = LAYOUTA_Dummy + $0005; // (BOOL) Child objects are of equal height.  Should default to TRUE for gadgets with a vertical orientation.
  // orientation values
  LORIENT_NONE  = 0;
  LORIENT_HORIZ = 1;
  LORIENT_VERT  = 2;
// Gadget Method ID's
  GM_Dummy         = -1; // not used for anything
  GM_HITTEST       = 0;  // return GMR_GADGETHIT IF you are clicked on (whether or not you are disabled).
  GM_RENDER        = 1;  // draw yourself, in the appropriate state
  GM_GOACTIVE      = 2;  // you are now going to be fed input
  GM_HANDLEINPUT   = 3;  // handle that input
  GM_GOINACTIVE    = 4;  // whether or not by choice, you are done
  GM_HELPTEST      = 5;  // Will you send gadget help if the mouse is at the specified coordinates?  See below for possible GMR_ values.
  GM_LAYOUT        = 6;  // re-evaluate your size based on the GadgetInfo Domain.  Do NOT re-render yourself yet, you will be called when it is time...
  GM_DOMAIN        = 7;  // Used to obtain the sizing requirements of an object.  Does not require an object.
  GM_KEYTEST       = 8;  // Return GMR_GADGETHIT if your activation key matches (whether or not you are disabled).
  GM_KEYGOACTIVE   = 9;  // You are now going to be fed raw key events (V53)
  GM_KEYGOINACTIVE = 10; // You are done receiving raw key events (V53)
  GM_PRERENDER     = 11; // Private (V50)
  GM_POSTRENDER    = 12; // Private (V50)
  GM_EXTENT        = 13; // Let Intuition know what pixels your GM_RENDER method will fill (V51)
  GM_HANDLESCROLL  = 14; // Handle a mouse wheel event. (V52)
  GM_QUERY         = 15; // You're being queried about some special information. (V53)
  GM_MENUPICK      = 16; //  The user selected an item from your context menu. (V54)
  GM_MENUHELP      = 17; // The user asked for help on an item from your context menu. (V54)

type
// Parameter "Messages" passed to gadget class methods  }
// GM_HITTEST and GM_HELPTEST send this message. For GM_HITTEST, gpht_Mouse are coordinates relative to the gadget
// select box.  For GM_HELPTEST, the coordinates are relative to the gadget bounding box (which defaults to the select box).
  PgpHitTest = ^TgpHitTest;
  TgpHitTest = record
    MethodID: LongWord;
    gpht_GInfo: PGadgetInfo;
    gpht_Mouse: record
      x: SmallInt;
      y: SmallInt;
    end;
  end;

const
// For GM_HITTEST, return GMR_GADGETHIT if you were indeed hit, otherwise return zero.
//
// For GM_HELPTEST, return GMR_NOHELPHIT (zero) if you were not hit. Typically, return GMR_HELPHIT if you were hit.
// It is possible to pass a UWORD to the application via the Code field of the IDCMP_GADGETHELP message.  Return GMR_HELPCODE or'd with
// the UWORD-sized result you wish to return.
//
// GMR_HELPHIT yields a Code value of (LongWord(not 0)), which should mean "nothing particular" to the application.

  GMR_GADGETHIT = $00000004; // GM_HITTEST hit
  GMR_NOHELPHIT = $00000000; // GM_HELPTEST didn't hit
  GMR_HELPHIT   = $FFFFFFFF; // GM_HELPTEST hit, return code = not 0
  GMR_HELPCODE  = $00010000; // GM_HELPTEST hit, return low word as code


// GM_RENDER    }
type
  PgpRender = ^TgpRender;
  TgpRender = record
    MethodID: LongWord;
    gpr_GInfo: PGadgetInfo; // gadget context
    gpr_RPort: PRastPort;   // all ready for use
    gpr_Redraw: LongInt;    // might be a "highlight pass"
  end;

// values of gpr_Redraw
const
  GREDRAW_UPDATE = 2; // incremental update, e.g. prop slider
  GREDRAW_REDRAW = 1; // redraw gadget
  GREDRAW_TOGGLE = 0; // toggle highlight, IF applicable

// GM_GOACTIVE, GM_HANDLEINPUT
type
  PgpInput = ^TgpInput;
  TgpInput = record
    MethodID: LongWord;
    gpi_GInfo: PGadgetInfo;
    gpi_IEvent: PInputEvent;
    gpi_Termination: PLongInt;
    gpi_Mouse: record
      x: SmallInt;
      y: SmallInt;
    end;
    { (V39) Pointer to TabletData structure, if this event originated from a tablet which sends IESUBCLASS_NEWTABLET events, or nil if      not.
      DO NOT ATTEMPT TO READ THIS FIELD UNDER INTUITION PRIOR TO V39! IT WILL BE INVALID!}
    gpi_TabletData : pTabletData;
  end;

// GM_HANDLEINPUT and GM_GOACTIVE  return code flags
// return GMR_MEACTIVE (0) alone if you want more input. Otherwise, return ONE of GMR_NOREUSE and GMR_REUSE, and optionally GMR_VERIFY.
const
  GMR_MEACTIVE = 0;
  GMR_NOREUSE  = 1 shl 1;
  GMR_REUSE    = 1 shl 2;
  GMR_VERIFY   = 1 shl 3; // you MUST set cgp_Termination

// New for V37:
// You can end activation with one of GMR_NEXTACTIVE and GMR_PREVACTIVE, which instructs
// Intuition to activate the next or previous gadget that has GFLG_TABCYCLE set.
  GMR_NEXTACTIVE = 1 shl 5;
  GMR_PREVACTIVE = 1 shl 6;

// GM_GOINACTIVE }
type
  PgpGoInactive = ^TgpGoInactive;
  TgpGoInactive = record
    MethodID: LongWord;
    gpgi_GInfo: PGadgetInfo;
    // V37 field only!  DO NOT attempt to read under V36! }
    gpgi_Abort: LongWord; // gpgi_Abort=1 IF gadget was aborted by Intuition and 0 if gadget went inactive at its own request
  end;

{ New for V39: Intuition sends GM_LAYOUT to any GREL_ gadget when the gadget is added to the window (or when the window opens, if
  the gadget was part of the NewWindow.FirstGadget or the WA_Gadgets list), or when the window is resized.  Your gadget can set the
  GA_RelSpecial property to get GM_LAYOUT events without Intuition changing the interpretation of your gadget select box.  This
  allows for completely arbitrary resizing/repositioning based on window size.}
// GM_LAYOUT
  PgpLayout = ^TgpLayout;
  TgpLayout = record
    MethodID: LongWord;
    gpl_GInfo: PGadgetInfo;
    gpl_Initial: LongWord; // non-zero if this method was invoked during AddGList() or OpenWindow()
  end;                     // time.  zero if this method was invoked during window resizing.


// The GM_DOMAIN method is used to obtain the sizing requirements of an object for a class before ever creating an object.
// GM_DOMAIN
  PgpDomain = ^TgpDomain;
  TgpDomain = record
      MethodID: LongWord;
      gpd_GInfo: PGadgetInfo;
      gpd_RPort: PRastPort;   // RastPort to layout for
      gpd_Which: LongInt;
      gpd_Domain: TIBox;      // Resulting domain
      gpd_Attrs: PTagItem;    // Additional attributes
   end;

const
  GDOMAIN_MINIMUM = 0; // Minimum size
  GDOMAIN_NOMINAL = 1; // Nominal size
  GDOMAIN_MAXIMUM = 2; // Maximum size

// The GM_KEYTEST method is used to determin if a key press matches an object's activation key(s).
// GM_KEYTEST send this message.
type
  PgpKeyTest = ^TgpKeyTest;
  TgpKeyTest = record
    MethodID: LongWord;
    gpkt_GInfo: PGadgetInfo;
    gpkt_IMsg: PIntuiMessage;
    gpkt_VanillaKey: LongWord;
  end;


// The GM_KEYGOACTIVE method is called to "simulate" a gadget going down. A gadget should render itself in a selected state when receiving
//  this message. If the class supports this method, it must return GMR_KEYACTIVE.

// If a gadget returns zero for this method, it will subsequently be activated via ActivateGadget() with a NULL IEvent.
// GM_KEYGOACTIVE
  PgpKeyInput = ^TgpKeyInput;
  TgpKeyInput = record
    MethodID: LongWord;
    gpk_GInfo: PGadgetInfo;
    gpk_IEvent: PInputEvent;
    gpk_Termination: PLongInt;  // Used with GMR_KEYVERIFY
  end;

const
  GMR_KEYACTIVATE = $00000000; // Throw away event and call ActivateGadget() (default)
  GMR_KEYACTIVE   = $00000010; // Method supported and send more key events
  GMR_KEYVERIFY   = $00000020; // Stop sending key events. You MUST set gpk_Termination.

// The GM_KEYGOINACTIVE method is called to simulate the gadget release.
// Upon receiving this message, the gadget should do everything a normal gadget release would do.
type
  PgpKeyGoInactive = ^tgpKeyGoInactive;
  tgpKeyGoInactive = record
    MethodID: LongWord;       // GM_KEYGOINACTIVE
    gpki_GInfo: PGadgetInfo;
    gpki_Abort: LongWord;     // TRUE if input was aborted
  end;

// New for V51: Intuition may send GM_EXTENT to a gadget to ask it what
// pixels (at least) it will fully redraw when its GM_RENDER method is invoked in the same context.
// GM_EXTENT
  PgpExtent = ^TgpExtent;
  TgpExtent = record
    MethodID: LongWord;
    gpe_GInfo: PGadgetInfo;
    gpe_RPort: PRastPort;   // nil if masking not supported
    gpe_Region: PRegion;    // nil if clipping not supported
    gpe_Action: LongWord;   // Requested operation
    gpe_Flags: LongWord;    // Control flags, see below
    gpe_Attrs: PTagItem;    // Additional attributes
  end;
const
  // Possible operations requested by GM_EXTENT
  GEXTENT_REMOVE = 0; // You should CLEAR shapes from region/mask
  GEXTENT_ADD    = 1; // You should OR shapes into region/mask
  GEXTENT_INVERT = 2; // You should XOR shapes into region/mask
  GEXTENT_SECT   = 3; // You should AND shapes into region/mask
  // Control flags defined for GM_EXTENT
  GPEF_ALLOWSUPER = $00000001; // Allow superclass to handle the method
  // Possible return codes for GM_EXTENT
  GMR_INVALID  = $00000000; // Couldn't provide any information
  GMR_FULLHBOX = $00000010; // I fill every pixel within my hit box
  GMR_FULLBBOX = $00000020; // I fill every pixel within my bounding box
  GMR_CLIPDONE = $00000040; // Added to the region all areas I fully redraw
  GMR_MASKDONE = $00000080; // Wrote into the mask all pixels I fully redraw

// The GM_QUERY method is new for V53, and is currently invoked by Intuition and window.class.
// GM_QUERY
type
  PgpQuery = ^TgpQuery;
  TgpQuery = record
   MethodID: LongWord;      // always GM_QUERY
   gpq_GInfo: PGadgetInfo;  // filled in by Intuition
   gpq_IEvent: PInputEvent; // positional data is here
   gpq_Type: LongWord;      // see below
   gpq_Data: LongInt;       // the data being returned
  end;
const
// values for gpq_Type
  GMQ_HINTINFO    = 1;
  GMQ_WHICHMEMBER = 2;
  GMQ_CONTEXTMENU = 3;

type
// New for V54: the GM_MENUPICK and GM_MENUHELP methods are used to let a  custom gadget hear context menu selections or help requests by the user.
// GM_MENUPICK, GM_MENUHELP
  PgpMenuEvent = ^TgpMenuEvent;
  TgpMenuEvent = record
    MethodID: LongWord;
    gpme_GInfo: PGadgetInfo;   // Always nil
    gpme_MenuAddress: APTR;    // Originating context menu
    gpme_MenuNumber: LongWord; // First item selected
    gpme_Window: PWindow;      // The gadget's window
  end;

const
// Gadget/object interconnection classes
  ICM_Dummy      = $0401; // used for nothing
  ICM_SETLOOP    = $0402; // set/increment loop counter
  ICM_CLEARLOOP  = $0403; // clear/decrement loop counter
  ICM_CHECKLOOP  = $0404; // set/increment loop
// no parameters for ICM_SETLOOP, ICM_CLEARLOOP, ICM_CHECKLOOP

// interconnection attributes used by icclass, modelclass, and gadgetclass
  ICA_Dummy      = TAG_USER + $40000;
  ICA_TARGET     = ICA_Dummy + 1; // interconnection target
  ICA_MAP        = ICA_Dummy + 2; // interconnection map tagitem list
  ICSPECIAL_CODE = ICA_Dummy + 3; // a "pseudo-attribute", see below.

{ Normally, the value for ICA_TARGET is some object pointer, but if you specify the special value ICTARGET_IDCMP, notification
  will be send as an IDCMP_IDCMPUPDATE message to the appropriate window's IDCMP port.  See the definition of IDCMP_IDCMPUPDATE.

  When you specify ICTARGET_IDCMP for ICA_TARGET, the map you specify will be applied to derive the attribute list that is
  sent with the IDCMP_IDCMPUPDATE message.  If you specify a map list which results in the attribute tag id ICSPECIAL_CODE, the
  lower sixteen bits of the corresponding ti_Data value will be copied into the Code field of the IDCMP_IDCMPUPDATE IntuiMessage.}
  ICTARGET_IDCMP = not 0;

 // system image classes
  CUSTOMIMAGEDEPTH = -1; // if image.Depth is this, it's a new Image class object

  IA_Dummy      = TAG_USER + $20000;
  IA_Left       = IA_Dummy + $01;
  IA_Top        = IA_Dummy + $02;
  IA_Width      = IA_Dummy + $03;
  IA_Height     = IA_Dummy + $04;
  IA_FGPen      = IA_Dummy + $05; // IA_FGPen also means "PlanePick"
  IA_BGPen      = IA_Dummy + $06; // IA_BGPen also means "PlaneOnOff" }
  IA_Data       = IA_Dummy + $07; // bitplanes, for classic image, other image classes may use it for other things
  IA_LineWidth  = IA_Dummy + $08;
  IA_Pens       = IA_Dummy + $0E; // pointer to word pens[], ala DrawInfo.Pens, MUST be terminated by ~0.  Some classes can
                                 // choose to have this, or SYSIA_DrawInfo, or both.
  IA_Resolution = IA_Dummy + $0F; // packed uwords for x/y resolution into a longword ala DrawInfo.Resolution

// see class documentation to learn which classes recognize these
  IA_APattern     = IA_Dummy + $10;
  IA_APatSize     = IA_Dummy + $11;
  IA_Mode         = IA_Dummy + $12;
  IA_Font         = IA_Dummy + $13;
  IA_Outline      = IA_Dummy + $14;
  IA_Recessed     = IA_Dummy + $15;
  IA_DoubleEmboss = IA_Dummy + $16;
  IA_EdgesOnly    = IA_Dummy + $17;

 // "sysiclass" attributes
  SYSIA_Size          = IA_Dummy + $0B; // const's below
  SYSIA_Depth         = IA_Dummy + $0C; // this is unused by Intuition.  SYSIA_DrawInfo is used instead for V36
  SYSIA_Which         = IA_Dummy + $0D; // see 's below
  SYSIA_DrawInfo      = IA_Dummy + $18; // pass to sysiclass, please
  // obsolete: don't use these, use IA_Pens
  SYSIA_Pens          = IA_Pens;
  IA_ShadowPen        = IA_Dummy + $09;
  IA_HighlightPen     = IA_Dummy + $0A;
  // New for V39:
  SYSIA_ReferenceFont = IA_Dummy + $19; // Font to use as reference for scaling certain sysiclass images
  IA_SupportsDisable  = IA_Dummy + $1a; //  By default, Intuition ghosts gadgets itself, instead of relying on IDS_DISABLED or
                                        // IDS_SELECTEDDISABLED.  An imageclass that supports these states should return this attribute
                                        // as TRUE.  You cannot set or clear this attribute, however.
  IA_FrameType   = IA_Dummy + $1b; // Starting with V39, FrameIClass recognizes  several standard types of frame.  Use one
                                        // of the FRAME_ specifiers below.  Defaults to FRAME_DEFAULT.
  IA_Underscore  = IA_Dummy + $1c; // V44, Indicate underscore keyboard shortcut for image labels. (Char) Defaults to '_'
  IA_Scalable    = IA_Dummy + $1d; // V44, Attribute indicates this image is allowed to/can scale its rendering. (BOOL) Defaults to FALSE.
  IA_ActivateKey = IA_Dummy + $1e; // V44, Used to get an underscored label shortcut. Useful for labels attached to string gadgets. (Char) Defaults to #0.
  IA_Screen      = IA_Dummy + $1f; // V44 Screen pointer, may be useful/required by certain classes. (PScreen)
  IA_Precision   = IA_Dummy + $20; // V44 Precision value, typically pen precision but may be used for similar custom purposes. (LongWord)
  // New For V50:
  SYSIA_FullFrame   = IA_Dummy + $21; // Specifies whether the sysiclass image must have an inner frame, if the current style normally has one.
  SYSIA_ContentsBox = IA_Dummy + $22; // position and size of the inner area of a TBFRAMEIMAGE image (PIBox)
  IA_Orientation    = IA_Dummy + $23; // Defines orientation, for images needing this kind of information
  IA_Simple         = IA_Dummy + $24; // image should be drawn with the most plain graphic appearance it supports
  SYSIA_VectorInfo  = IA_Dummy + $25; // Custom vector definition for a sysiclass image.
  SYSIA_RenderHook  = IA_Dummy + $26; // Custom rendering hook for a sysiclass image.
  SYSIA_BitMapCache = IA_Dummy + $27; // Level of bitmap caching for sysiclass images (0 = none, 1 = deferred, 2 = immediate). Defaults to 2.
  IA_Label          = IA_Dummy + $28; // Pointer to a string to be used as the image's text label, if it supports one.
  SYSIA_Label       = IA_Label;

  IA_BackFill         = IA_Dummy + $29; // Pointer to a backfill hook the image can use to fill
  IA_SelectedBackFill = IA_Dummy + $2A; // its background, if it supports this attribute.
  IA_DisabledBackFill = IA_Dummy + $2B;
  IA_InactiveBackFill = IA_Dummy + $2C;
  // New For V51:
  IA_EraseBackground = IA_Dummy + $2D; // Erase the background before rendering the image?
  IA_InBorder        = IA_Dummy + $2E; // Is this image to be drawn inside a window border? Defaults to FALSE.

//**** "gaugeiclass" attributes
  GAUGEIA_Min        = IA_Dummy + $2F; // Minimum level of a gaugeiclass image. (V51)
  GAUGEIA_Max        = IA_Dummy + $30; // Maximum level of a gaugeiclass image. (V51)
  GAUGEIA_Level      = IA_Dummy + $31; // Current level of a gaugeiclass image. (V51)
  GAUGEIA_Ticks      = IA_Dummy + $32; // Number of tick marks to be drawn for a gaugeiclass image. Defaults to 0 (no tick marks). (V51)
  GAUGEIA_TickSize   = IA_Dummy + $33; // Length of tick marks for a gaugeiclass image, if they are present. Defaults to 4. (V51)
  GAUGEIA_ShortTicks = IA_Dummy + $34; // Should half-length tick marks be drawn in-between the normal tick marks of a gaugeiclass image? Defaults to False. (V51)
  GAUGEIA_InnerTicks = IA_Dummy + $35; // Should tick marks be drawn inside the gauge, rather than alongside it? Defaults to FALSE. (V51)
  GAUGEIA_ScaleHook  = IA_Dummy + $36; // Custom hook for calculating the size of the filled part of the gauge, relative to the container's size;
                                       // object is the image, message is a pointer to an array of 3 uint32s. The scale hook must calculate
                                       // (number[0] * number[1]) / number[2] and return the result, or zero if it can't perform the calculation.
                                       // Passing NULL means an internal scaling routine will be used. Defaults to nil. (V51)
  IA_Justification   = IA_Dummy + $37; // Justification of the image's text label, if it supports one. Use one of the IJ_ constants below.
                                       // Defaults to IJ_LEFT, unless otherwise specified in the documentation for the image class. (V51)
  IA_LabelPen        = IA_Dummy + $38; // Color of the image's text label, if it supports one. Defaults to the TEXTPEN color. (V51)

// data values for SYSIA_Size
  SYSISIZE_MEDRES = 0;
  SYSISIZE_LOWRES = 1;
  SYSISIZE_HIRES  = 2;

// SYSIA_Which tag data values: Specifies which system gadget you want an image for. Some numbers correspond to internal Intuition
  DEPTHIMAGE     = $00; // Window depth gadget image
  ZOOMIMAGE      = $01; // Window zoom gadget image
  SIZEIMAGE      = $02; // Window sizing gadget image
  CLOSEIMAGE     = $03; // Window close gadget image
  SDEPTHIMAGE    = $05; // screen depth gadget
  LEFTIMAGE      = $0A; // Window left-arrow gadget image
  UPIMAGE        = $0B; // Window up-arrow gadget image
  RIGHTIMAGE     = $0C; // Window right-arrow gadget image
  DOWNIMAGE      = $0D; // Window down-arrow gadget image
  CHECKIMAGE     = $0E; // GT/RA checkbox image
  MXIMAGE        = $0F; // GT/RA mutual exclude "button" image
  // New for V39:
  MENUCHECK      = $10; // Menu checkmark image
  AMIGAKEY       = $11; // Menu Amiga-key image
  // New for V50:
  SBARLOGO       = $12; // Screen titlebar logo
  POPUPIMAGE     = $13; // Window pop-up gadget image
  SETTINGSIMAGE  = $14; // Window settings gadget image
  SNAPSHOTIMAGE  = $15; // Window snapshot gadget image
  ICONIFYIMAGE   = $16; // Window iconify gadget image
  PADLOCKIMAGE   = $17; // Window padlock gadget image
  TBFRAMEIMAGE   = $18; // Window titlebar frame image
  HKHANDLEIMAGE  = $19; // Window horizontal knob handle symbol
  VKHANDLEIMAGE  = $1A; // Window vertical knob handle symbol
  MENUMX         = $1B; // Menu mutualexclude image
  MENUSUB        = $1C; // Menu sub-panel indicator
  CYCLEIMAGE     = $1D; // GT/RA cycle symbol
  CHECKMARKIMAGE = $1E; // GT/RA checkmark symbol
  GLEFTIMAGE     = $1F; // GT/RA left-arrow symbol
  GUPIMAGE       = $20; // GT/RA up-arrow symbol
  GRIGHTIMAGE    = $21; // GT/RA right-arrow symbol
  GDOWNIMAGE     = $22; // GT/RA down-arrow symbol
  GHKHANDLEIMAGE = $23; // GT/RA horizontal knob handle symbol
  GVKHANDLEIMAGE = $24; // GT/RA vertical knob handle symbol
  // New for V51:
  SCLOSEIMAGE    = $25; // Screen close gadget image
  SCREENSIMAGE   = $26; // Window screen-jump gadget image
  // New for V53.25:
  SORTASCIMAGE   = $27; // Sort ascending image
  SORTDESIMAGE   = $28; // Sort descending image
// Additional system image types recognized by DrawSysImageA() (V50)
  SI_HPROPBACKGROUND = $1001; // Background of horizontal scroller
  SI_VPROPBACKGROUND = $1002; // Background of vertical scroller

// Data values for IA_FrameType (recognized by FrameIClass)
  FRAME_DEFAULT     = 0; //The standard V37-type frame, which has thin edges.
  FRAME_BUTTON      = 1; // Standard button gadget frames, having thicker sides and nicely edged corners.
  FRAME_RIDGE       = 2; // A ridge such as used by standard string gadgets. You can recess the ridge to get a groove image.
  FRAME_ICONDROPBOX = 3; // A broad ridge which is the standard imagery for areas in AppWindows where icons may be dropped.
  // New for V50:
  FRAME_PROPBORDER  = 4; // A frame suitable for use as border of a proportional gadget container.
  FRAME_PROPKNOB    = 5; // A frame suitable for use as knob of a proportional gadget.
  FRAME_DISPLAY     = 6; // A recessed frame for display elements, such as read-only text or number gadgets.

// IA_Justification tag data values
  IJ_LEFT   = 0;
  IJ_CENTER = 1;
  IJ_RIGHT  = 2;

// image message id's
  IM_DRAW     = $202; // draw yourself, with "state"
  IM_HITTEST  = $203; // return TRUE IF click hits image
  IM_ERASE    = $204; // erase yourself
  IM_MOVE     = $205; // draw new AND erase old, smoothly

  IM_DRAWFRAME   = $206; // draw with specified dimensions
  IM_FRAMEBOX    = $207; // get recommended frame around some box
  IM_HITFRAME    = $208; // hittest with dimensions
  IM_ERASEFRAME  = $209; // hittest with dimensions

  IM_DOMAINFRAME = $20A; // query image for its domain info (V44)

  IM_EXTENT      = $20B; // inquire about rendering extent (V51)
  IM_EXTENTFRAME = $20C; // as IM_EXTENT, with dimensions (V51)

// image draw states or styles, for IM_DRAW
// Note that they have no bitwise meanings (unfortunately)
  IDS_NORMAL           = 0;
  IDS_SELECTED         = 1; // for selected gadgets
  IDS_DISABLED         = 2; // for disabled gadgets
  IDS_BUSY             = 3; // for future functionality
  IDS_INDETERMINATE    = 4; // for future functionality
  IDS_INACTIVENORMAL   = 5; // normal, in inactive window border
  IDS_INACTIVESELECTED = 6; // selected, in inactive border
  IDS_INACTIVEDISABLED = 7; // disabled, in inactive border
  IDS_SELECTEDDISABLED = 8; // disabled and selected

// oops, please forgive spelling error by jimm
  IDS_INDETERMINANT = IDS_INDETERMINATE;

// IM_FRAMEBOX
type
  PimpFrameBox = ^TimpFrameBox;
  TimpFrameBox = record
    MethodID: LongWord;
    imp_ContentsBox: PIBox;   // input: relative box of contents
    imp_FrameBox: PIBox;      // output: rel. box of encl frame
    imp_DrInfo: PDrawInfo;    // NB: May be nil
    imp_FrameFlags: LongWord;
  end;

const
  FRAMEF_SPECIFY = 1 shl 0; // Make do with the dimensions of FrameBox provided.

// IM_DRAW, IM_DRAWFRAME
type
  PimpDraw = ^TimpDraw;
  TimpDraw = record
    MethodID: LongWord;
    imp_RPort: PRastPort;
    imp_Offset: record
      x: SmallInt;
      y: SmallInt;
    end;
    imp_State: LongWord;
    imp_DrInfo: PDrawInfo; // NB: May be nil
    // these parameters only valid for IM_DRAWFRAME
    imp_Dimensions: record
      Width: SmallInt;
      Height: SmallInt;
    end;
  end;

// IM_ERASE, IM_ERASEFRAME
// NOTE: This is a subset of impDraw
  PimpErase = ^TimpErase;
  TimpErase = record
    MethodID: LongWord;
    imp_RPort: PRastPort;
    imp_Offset: record
      x: SmallInt;
      y: SmallInt;
    end;
    // these parameters only valid for IM_ERASEFRAME
    imp_Dimensions: record
      Width: SmallInt;
      Height: SmallInt;
    end;
  end;

// IM_HITTEST, IM_HITFRAME
  PimpHitTest = ^TimpHitTest;
  TimpHitTest = record
    MethodID: LongWord;
    imp_Point: record
      x: SmallInt;
      y: SmallInt;
    end;
    // these parameters only valid for IM_HITFRAME
    imp_Dimensions: record
      Width: SmallInt;
      Height: SmallInt;
    end;
  end;

// The IM_DOMAINFRAME method is used to obtain the sizing requirements of an image object within a layout group.
// IM_DOMAINFRAME
  PimpDomainFrame = ^TimpDomainFrame;
  TimpDomainFrame = record
    MethodID: LongWord;
    imp_DrInfo: PDrawInfo; // DrawInfo
    imp_RPort: PRastPort;  // RastPort to layout for
    imp_Which: LongInt;    // what size - min/nominal/max
    imp_Domain: TIBox;     // Resulting domain
    imp_Attrs: PTagItem;   // Additional attributes
  end;

// Accepted vales for imp_Which.
const
  IDOMAIN_MINIMUM = 0;
  IDOMAIN_NOMINAL = 1;
  IDOMAIN_MAXIMUM = 2;

// The IM_EXTENT/IM_EXTENTFRAME method is used to ask the image what pixels or areas (at least) it will fully
// overwrite in its IM_DRAW/IM_DRAWFRAME method.
// See the description of gadgetclass/GM_EXTENT for more detailed information about this topic.
// IM_EXTENT, IM_EXTENTFRAME
type
  PimpExtent = ^TimpExtent;
  TimpExtent = record
    MethodID: LongWord;
    imp_DrInfo: PDrawInfo; // May be nil
    imp_RPort: PRastPort;  // NULL if masking not supported
    imp_Region: PRegion;   // NULL if clipping not supported
    imp_Action: LongWord;  // Requested operation
    imp_Flags: LongWord;   // Control flags, see below
    imp_Attrs: PTagItem;   // Additional attributes
    imp_State: LongWord;   // Rendering state
    imp_Frame: TIBox;      // Sizes only valid for IM_EXTENTFRAME
  end;

const
// Possible operations requested by IM_EXTENT/IM_EXTENTFRAME
  IEXTENT_REMOVE = 0; // You should CLEAR shapes from region/mask
  IEXTENT_ADD    = 1; // You should OR shapes into region/mask
  IEXTENT_INVERT = 2; // You should XOR shapes into region/mask
  IEXTENT_SECT   = 3; // You should AND shapes into region/mask
// Control flags defined for IM_EXTENT/IM_EXTENTFRAME
  IMPF_ALLOWSUPER = $01; // Allow superclass to handle the method
// Possible return codes for IM_EXTENT/IM_EXTENTFRAME
  IMR_INVALID  = $00; // Couldn't provide any information
  IMR_CLIPDONE = $40; // Added to the region all areas I fully cover
  IMR_MASKDONE = $80; // Wrote into the mask all pixels I fully cover

 { **  'boopsi' pointer class interface }

const
  // The following tags are recognized at NewObject() time by pointerclass:
  POINTERA_Dummy = TAG_USER + $39000;
  POINTERA_BitMap      = POINTERA_Dummy + $01; // (PBitMap) - Pointer to bitmap to get pointer imagery from.  Bitplane data need not be in chip RAM.
  POINTERA_XOffset     = POINTERA_Dummy + $02; // (LongInt) - X-offset of the pointer hotspot.
  POINTERA_YOffset     = POINTERA_Dummy + $03; // (LongInt) - X-offset of the pointer hotspot.
  POINTERA_WordWidth   = POINTERA_Dummy + $04; // (LongWord) - designed width of the pointer in words
  POINTERA_XResolution = POINTERA_Dummy + $05; // (LongWord) - one of the POINTERXRESN_ flags below
  POINTERA_YResolution = POINTERA_Dummy + $06; // (LongWord) - one of the POINTERYRESN_ flags below
  POINTERA_ImageData   = POINTERA_Dummy + $07; // V52
  POINTERA_Width       = POINTERA_Dummy + $08; // V52
  POINTERA_Height      = POINTERA_Dummy + $09; // V52

// These are the choices for the POINTERA_XResolution attribute which will determine what
// resolution pixels are used for this pointer.
  POINTERXRESN_DEFAULT   = 0; // (ECS-compatible pointer width) = 70 ns if SUPERHIRES-type mode, 140 ns if not
  POINTERXRESN_140NS     = 1; // (pointer always in 140 ns pixels) = 140 ns always
  POINTERXRESN_70NS      = 2; // (pointer always in 70 ns pixels) = 70 ns always
  POINTERXRESN_35NS      = 3; // (pointer always in 35 ns pixels) = 35 ns always

  POINTERXRESN_SCREENRES = 4; // Same as pixel speed of screen
  POINTERXRESN_LORES     = 5; // (pointer always in lores-like pixels) = 140 ns in 15kHz modes, 70 ns in 31kHz modes
  POINTERXRESN_HIRES     = 6; // (pointer always in hires-like pixels) = 70 ns in 15kHz modes, 35 ns in 31kHz modes

// These are the choices for the POINTERA_YResolution attribute which
//  will determine what vertical resolution is used for this pointer.
  POINTERYRESN_DEFAULT          =  0; // In 15 kHz modes, the pointer resolution will be the same as a non-interlaced screen.  In 31 kHz modes, the pointer
                                     // will be doubled vertically.  This means there will be about 200-256 pointer lines per screen.
  POINTERYRESN_HIGH             =  2; // Where the hardware/software supports it, the pointer resolution will be high.  This means there will be about 400-480 pointer lines per screen.  POINTERYRESN_HIGHASPECT also means that
  POINTERYRESN_HIGHASPECT       =  3; //   when the pointer comes out double-height due to hardware/software restrictions, its width would be doubled as well, if possible(to preserve aspect).
  POINTERYRESN_SCREENRES        =  4; // Will attempt to match the vertical resolution of the pointer to the screen's vertical resolution.  POINTERYRESN_SCREENASPECT also means that when the pointer comes out double-height due to
  POINTERYRESN_SCREENRESASPECT  =  5; // hardware/software restrictions, its width would be doubled as well, if possible (to preserve aspect).

// Pointer type definitions for WA_PointerType (OpenWindowTags() and SetWindowPointer()) (V53.37)
  POINTERTYPE_NORMAL                   = 0;  // the well known normal pointer
  POINTERTYPE_BUSY                     = 1;  // the well known busy pointer
  POINTERTYPE_ALIAS                    = 2;  // use when dragging an item with the intent of creating a reference to or virtual copy of it
  POINTERTYPE_CELL                     = 3;  // use to indicate that the pointer is hovering on top of a table cell
  POINTERTYPE_COLUMNRESIZE             = 4;  // use to indicate that a column can be resized or is being resized
  POINTERTYPE_CONTEXTMENU              = 5;  // use to indicate that a menu or context menu can be opened at the current location
  POINTERTYPE_COPY                     = 6;  // use to indicate that something is in a copy buffer or that a drag&drop operation will result in a copy the the dragged item
  POINTERTYPE_CROSS                    = 7;  // use to indicate a precise selection, i.e. a precise area is being selected or while drawing pixels in a paint program
  POINTERTYPE_DRAGANDDROP              = 8;  // use to indicate that an item is currently being dragged around
  POINTERTYPE_EASTRESIZE               = 9;  // use to indicate that the item can be resized on its left border, i.e. a resize indication at the left border of a window
  POINTERTYPE_EASTWESTRESIZE           = 10; // use to indicate that the selected item is currently being resized either from its left or right border, i.e. the current window is being resized in its left border
  POINTERTYPE_HAND                     = 11; // use to indicate that the current item is drag&droppable or moveable
  POINTERTYPE_HELP                     = 12; // use to indicate that there is help available for the item the pointer is hovering over
  POINTERTYPE_LINK                     = 13; // use to indicate that the current context will be replaced, i.e. the pointer is hovering over a hyperlink or a window tab
  POINTERTYPE_MENU                     = 14; // use to indicate that a menu or context menu is active
  POINTERTYPE_NODROP                   = 15; // use to indicate that a dragged item cannot be dropped at the current location
  POINTERTYPE_NONE                     = 16; // use to indicate that the pointer is deactivated
  POINTERTYPE_NORTHEASTRESIZE          = 17; // use to indicate that the item can be resized on its top left corner, i.e. a resize indication at the top left corner of a window
  POINTERTYPE_NORTHEASTSOUTHWESTRESIZE = 18; // use to indicate that the selected item is currently being resized either from its top left or bottom right corner, i.e. the current window is being resized in its top left corner
  POINTERTYPE_NORTHRESIZE              = 19; // use to indicate that the item can be resized on its top border, i.e. a resize indication at the top border of a window
  POINTERTYPE_NORTHSOUTHRESIZE         = 20; // use to indicate that the selected item is currently being resized from either its top or bottom border, i.e. the current window is being resized in its bottom border
  POINTERTYPE_NORTHWESTRESIZE          = 21; // use to indicate that the selected item is currently being resized from its top left corner, i.e. the current window is being resized in its top left corner
  POINTERTYPE_NORTHWESTSOUTHEASTRESIZE = 22; // use to indicate that the selected item is currently being resized either from its top right or bottom left corner, i.e. the current window is being resized in its top right corner
  POINTERTYPE_NOTALLOWED               = 23; // use to indicate that the current operation is not allowed, i.e. the pointer is hovering over a disabled button
  POINTERTYPE_PROGRESS                 = 24; // use to indicate that the application is doing some background work but still accepts user input, i.e. a web page is currently loading in a browser application
  POINTERTYPE_ROWRESIZE                = 25; // use to indicate that a row can be resized or is being resized
  POINTERTYPE_SCROLLALL                = 26; // use to indicate that the item can be or is being moved, i.e. a window is being moved while a hotkey for window move is pressed
  POINTERTYPE_SOUTHEASTRESIZE          = 27; // use to indicate that the selected item is currently being resized from its bottom right corner, i.e. the current window is being resized in its bottom right corner
  POINTERTYPE_SOUTHRESIZE              = 28; // use to indicate that the item can be resized on its bottom border, i.e. a resize indication at the bottom border of a window
  POINTERTYPE_SOUTHWESTRESIZE          = 29; // use to indicate that the selected item is currently being resized from its bottom left corner, i.e. the current window is being resized in its bottom left corner
  POINTERTYPE_TEXT                     = 30; // use to indicate that horizontal text can be selected at the current position
  POINTERTYPE_VERTICALTEXT             = 31; // use to indicate that vertical text can be selected at the current position
  POINTERTYPE_WESTRESIZE               = 32; // use to indicate that the item can be resized on its right border, i.e. a resize indication at the right border of a window
  POINTERTYPE_ZOOMIN                   = 33; // use to indicate that a left click on the current item will result in a zoom in
  POINTERTYPE_ZOOMOUT                  = 34; // use to indicate that a left click on the current item will result in a zoom out
  POINTERTYPE_PEN                      = 35; // use to indicate a drawing action
  POINTERTYPE_ROTATE                   = 36; // use to indicate something can be rotated at the current position
  POINTERTYPE_RUBBER                   = 37; // use to indicate an erasing action
  POINTERTYPE_SELECT                   = 38; // use to indicate that something can be marked for selection at the current position
  POINTERTYPE_SMUDGE                   = 39; // use to indicate that something can be smudged/smeared at the current position
  POINTERTYPE_COUNT                    = 40; // just a counter for the number of available standard pointers

type
  PStringExtend = ^TStringExtend;
  TStringExtend = record
      // display specifications
    Font: PTextFont;                 // must be an open Font (not TextAttr)
    Pens: array[0..1] of Byte;       // color of text/backgroun
    ActivePens: array[0..1] of Byte; // colors when gadget is active
      // edit specifications
    InitialModes: LongWord; // initial mode flags, below
    EditHook: PHook;        // IF non-NULL, must supply WorkBuffer
    WorkBuffer: STRPTR;     // must be as large as StringInfo.Buffer
    Reserved: array[0..3] of LongWord; // set to 0
  end;

  PSGWork = ^TSGWork;
  TSGWork = record
     // set up when gadget is first activated
    Gadget: PGadget;         // the contestant itself
    StringInfo: pStringInfo; // easy access to sinfo
    WorkBuffer: STRPTR;      // intuition's planned result
    PrevBuffer: STRPTR;      // what was there before
    Modes: LongWord;         // current mode
      // modified for each input event
    IEvent: PInputEvent;     // actual event: do not change
    Code: Word;              // character code, IF one byte
    BufferPos: SmallInt;     // cursor position
    NumChars: SmallInt;
    Actions: LongWord;       // what Intuition will do
    LongInt_: LongInt;       // temp storage for LongInt
    GadgetInfo: PGadgetInfo;
    EditOp: Word;            // from constants below
  end;

// SGWork.EditOp - These values indicate what basic type of operation the global editing hook has performed on the string before your gadget's custom
// editing hook gets called.  You do not have to be concerned with the value your custom hook leaves in the EditOp field, only if you write a global editing hook.
// For most of these general edit operations, you'll want to compare the BufferPos and NumChars of the StringInfo (before global editing) and SGWork (after global editing).
const
  EO_NOOP       =  $0001; // did nothing
  EO_DELBACKWARD=  $0002; // deleted some chars maybe 0.
  EO_DELFORWARD =  $0003; // deleted some characters under and in front of the cursor
  EO_MOVECURSOR =  $0004; // moved the cursor
  EO_ENTER      =  $0005; // "enter" or "return" key, terminate
  EO_RESET      =  $0006; // current Intuition-style undo
  EO_REPLACECHAR=  $0007; // replaced one character and maybe advanced cursor
  EO_INSERTCHAR =  $0008; // inserted one char into string or added one at end
  EO_BADFORMAT  =  $0009; // didn't like the text data, e.g., Bad LongInt
  EO_BIGCHANGE  =  $000A; // unused by Intuition   // complete or major change to the text, e.g. new string
  EO_UNDO       =  $000B; // unused by Intuition   // some other style of undo
  EO_CLEAR      =  $000C; // clear the string
  EO_SPECIAL    =  $000D; // unused by Intuition   // some operation that doesn't fit into the categories here

  // Mode Flags definitions ONLY first group allowed as InitialModes
  SGM_REPLACE   =  1 shl 0; // replace mode // please initialize StringInfo with in-range value of BufferPos if you are using SGM_REPLACE mode.
  SGM_FIXEDFIELD = 1 shl 1; // fixed length buffer          // always set SGM_REPLACE, too
  SGM_NOFILTER   = 1 shl 2; // don't filter control chars
  // SGM_EXITHELP is new for V37, and ignored by V36:
  SGM_EXITHELP   = 1 shl 7; // exit with code = $5F IF HELP hit
  // These Mode Flags are for internal use only
  SGM_NOCHANGE   = 1 shl 3; // no edit changes yet
  SGM_NOWORKB    = 1 shl 4; // Buffer == PrevBuffer
  SGM_CONTROL    = 1 shl 5; // control char escape mode
  SGM_LONGINT    = 1 shl 6; // an intuition LongInt gadget
  // String Gadget Action Flags put in SGWork.Actions by EditHook
  SGA_USE        = $1;  // use contents of SGWork
  SGA_END        = $2;  // terminate gadget, code in Code field
  SGA_BEEP       = $4;  // flash the screen for the user
  SGA_REUSE      = $8;  // reuse input event
  SGA_REDISPLAY  = $10; // gadget visuals changed
  // New for V37:
  SGA_NEXTACTIVE = $20; // Make next possible gadget active.
  SGA_PREVACTIVE = $40; // Make previous possible gadget active.
  // function id for only existing custom string gadget edit hook
  SGH_KEY        = 1;    // process editing keystroke
  SGH_CLICK      = 2;    // process mouse click cursor position

const
 // Tags for ObtainBitMapSourceA()
  BMS_Dummy     = TAG_USER + $01547300;
  BMS_NoDiskIO  = BMS_Dummy + 1;      // Only open if already in memory
  BMS_DoMask    = BMS_Dummy + 2;      // Generate a mask, if possible
  BMS_DoOutline = BMS_Dummy + 3;      // Load outline mask, if present
  BMS_DoShade   = BMS_Dummy + 4;      // Load shading maps, if present

// Tags for ObtainBitMapInstanceA()
  BMI_Dummy         = TAG_USER + $01547400;
  BMI_Exclusive     = BMI_Dummy + 1;  // Is this bitmap exclusive?
  BMI_DoLevels      = BMI_Dummy + 2;  // Generate brighter/darker variants?
  BMI_Offsets       = BMI_Dummy + 3;  // Packed backfill offsets
  BMI_LayerInfo     = BMI_Dummy + 4;  // Is this a LayerInfo backfill?
  BMI_ReferencePen  = BMI_Dummy + 5;  // Base pen this bitmap is associated to
  BMI_GradientSpec  = BMI_Dummy + 6;  // Gradient specification for backfill
  BMI_CacheGradient = BMI_Dummy + 7;  // Packed W:H to rasterize gradient
  BMI_GradientAxis  = BMI_Dummy + 8;  // Tile rasterized gradient on this axis
  BMI_IgnoreDomain  = BMI_Dummy + 9;  // Always use whole RastPort as domain
  BMI_TileLeft      = BMI_Dummy + 10; // Left offset for backfill tile (V51)
  BMI_TileTop       = BMI_Dummy + 11; // Top offset for backfill tile (V51)
  BMI_TileWidth     = BMI_Dummy + 12; // Width of backfill tile (V51)
  BMI_TileHeight    = BMI_Dummy + 13; // Height of backfill tile (V51)

// Tags for BitMapInstanceControlA()
  BMICTRL_Dummy = TAG_USER + $01547600;

  BMICTRL_GetBitMap           = BMICTRL_Dummy + 1;  // (struct BitMap **) Get actual BitMap address.
  BMICTRL_GetHook             = BMICTRL_Dummy + 2;  // (struct Hook **) Get backfill hook address.
  BMICTRL_GetOffsets          = BMICTRL_Dummy + 3;  // (ULONG *) Get backfill offsets, packed in a single longword.
  BMICTRL_SetOffsets          = BMICTRL_Dummy + 4;  // (ULONG) Set backfill offsets, packed in a single longword;
                                                    //    this ONLY applies to instances allocated in exclusive mode.
  BMICTRL_GetWidth            = BMICTRL_Dummy + 5;  // (ULONG *) Get width of BitMap.
  BMICTRL_GetHeight           = BMICTRL_Dummy + 6;  // (ULONG *) Get height of BitMap.
  BMICTRL_GetBrightBitMap     = BMICTRL_Dummy + 7;  // (struct BitMap **) Get address of bright-level BitMap (if any).
  BMICTRL_GetHalfBrightBitMap = BMICTRL_Dummy + 8;  // (struct BitMap **) Get address of half-bright-level BitMap (if any).
  BMICTRL_GetHalfDarkBitMap   = BMICTRL_Dummy + 9;  // (struct BitMap **) Get address of half-dark-level BitMap (if any).
  BMICTRL_GetDarkBitMap       = BMICTRL_Dummy + 10; // (struct BitMap **) Get address of dark-level BitMap (if any).
  BMICTRL_GetMaskBitMap       = BMICTRL_Dummy + 11; // (struct BitMap **) Get address of single-plane mask bitmap (if any).
  BMICTRL_GetOutlineBitMap    = BMICTRL_Dummy + 12; // (struct BitMap **) Get address of single-plane outline bitmap (if any)
  BMICTRL_GetShineMap         = BMICTRL_Dummy + 13; // (UBYTE **) Get address of bright shading alpha map array (if any).
  BMICTRL_GetShadowMap        = BMICTRL_Dummy + 14; // (UBYTE **) Get address of dark shading alpha map array (if any).
  BMICTRL_GetAlphaMap         = BMICTRL_Dummy + 15; // (UBYTE **) Get address of alpha blending map array (if any).
  BMICTRL_GetShadeMaskBitMap  = BMICTRL_Dummy + 16; // (struct BitMap **) Get address of single-plane shade mask bitmap (if any).
  BMICTRL_GetScreen           = BMICTRL_Dummy + 17; // (struct Screen **) Get address of the reference screen (may be NULL).
  BMICTRL_GetBitMapSource     = BMICTRL_Dummy + 18; // (APTR *) Get address of the BitMapSource this instance was obtained from.
  BMICTRL_GetGradientSpec     = BMICTRL_Dummy + 19; // (GRADSPEC **) Get address of gradient specification (if any).
  BMICTRL_SetGradientSpec     = BMICTRL_Dummy + 20; // (GRADSPEC *) Set gradient specification; this ONLY applies to
                                                    //   instances allocated in exclusive mode.
  BMICTRL_GetReferencePen     = BMICTRL_Dummy + 21; // (ULONG *) Get reference pen index.
  BMICTRL_GetOutlineMap       = BMICTRL_Dummy + 22; // (UBYTE **) Get address of outline alpha map array (if any).
  BMICTRL_GetTileLeft         = BMICTRL_Dummy + 23; // (ULONG *) Get left offset of backfill tile. (V51)
  BMICTRL_GetTileTop          = BMICTRL_Dummy + 24; // (ULONG *) Get top offset of backfill tile. (V51)
  BMICTRL_GetTileWidth        = BMICTRL_Dummy + 25; // (ULONG *) Get width of backfill tile. (V51)
  BMICTRL_GetTileHeight       = BMICTRL_Dummy + 26; // (ULONG *) Get height of backfill tile. (V51)
  BMICTRL_GetTileBox          = BMICTRL_Dummy + 27; // (struct IBox *) Get backfill tile box in one go. (V51)
  BMICTRL_SetTileLeft         = BMICTRL_Dummy + 28; // (UWORD) Set left offset of backfill tile; this ONLY applies to
                                                    //   instances allocated in exclusive mode. (V51)
  BMICTRL_SetTileTop          = BMICTRL_Dummy + 29; // (UWORD) Set top offset of backfill tile; this ONLY applies to
                                                    //   instances allocated in exclusive mode. (V51)
  BMICTRL_SetTileWidth        = BMICTRL_Dummy + 30; // (UWORD) Set width of backfill tile; this ONLY applies to
                                                    // instances allocated in exclusive mode. (V51)
  BMICTRL_SetTileHeight       = BMICTRL_Dummy + 31; // (UWORD) Set height of backfill tile; this ONLY applies to
                                                    //   instances allocated in exclusive mode. (V51)
  BMICTRL_SetReferencePen     = BMICTRL_Dummy + 32; // (ULONG) Set reference pen index; this ONLY applies to
                                                    //   instances allocated in exclusive mode. (V51)

// Useful type definitions
type
  TBitMapSource = APTR;
  TBitMapInstance = APTR;

// *** Common structures for simple vector drawing
  PVector = ^TVector;
  TVector = record
    Operation: Byte;  // Type of rendering operation
    Type_: Byte;      // Type of environment (monochrome/color)
    DRIPen: Word;     // DrawInfo pen for this rendering
    States: LongWord; // States this rendering applies to
    X, Y: SmallInt;   // Offsets for rendering
    Data: PWord;      // Rendering specifications
  end;

  PVectorInfo = ^TVectorInfo;
  TVectorInfo = record
   VectorID: LongWord; // To identify the image, if needed
   Vectors: PVector;   // VO_END-terminated array of vectors
   DesignWidth: Word;  // Reference width for scaling
   DesignHeight: Word; // Reference height for scaling
   Flags: LongWord;    // Additional information
   States: LongWord;   // Supported states
  end;
const
// Values for Vector.Operation
  VO_LINE = $00; // Outline only
  VO_FILL = $01; // Filled, not outline
  VO_RECT = $00; // Rectangle
  VO_POLY = $02; // Polygon
  VO_ELPS = $04; // Ellipse

  VO_LINERECT = VO_LINE or VO_RECT;
  VO_FILLRECT = VO_FILL or VO_RECT;
  VO_LINEPOLY = VO_LINE or VO_POLY;
  VO_FILLPOLY = VO_FILL or VO_POLY;
  VO_LINEELPS = VO_LINE or VO_ELPS;
  VO_FILLELPS = VO_FILL or VO_ELPS;

  VO_END      = $FF;  // End marker for Vector array

// Values for TVector.Type
  VT_MONO  = $01; // Vector is used for monochrome rendering
  VT_COLOR = $02; // Vector is used for color rendering
  VT_BOTH  = $03; // Vector is used for both types of rendering

// Special values for TVector.DRIPen
  PEN_BRIGHT      = $1000; // Bright shade of the background
  PEN_HALFBRIGHT  = $1001; // Half-bright shade of the background
  PEN_HALFDARK    = $1003; // Half-dark shade of the background
  PEN_DARK        = $1004; // Dark shade of the background
  PEN_ALPHASHINE  = $2000; // OR with 0..255 for bright alpha shading
  PEN_ALPHASHADOW = $3000; // OR with 0..255 for dark alpha shading
  PEN_ALPHA       = $4000; // OR with 0..255 for alpha blending

// State flags. Sysiclass doesn't support INACTIVEDISABLED for now.
  IDSF_NORMAL           = 1 shl IDS_NORMAL;
  IDSF_SELECTED         = 1 shl IDS_SELECTED;
  IDSF_DISABLED         = 1 shl IDS_DISABLED;
  IDSF_INACTIVENORMAL   = 1 shl IDS_INACTIVENORMAL;
  IDSF_INACTIVESELECTED = 1 shl IDS_INACTIVESELECTED;
  IDSF_SELECTEDDISABLED = 1 shl IDS_SELECTEDDISABLED;

  IDSF_MASK             = $10000000; // Use for image mask
  IDSF_DISABLEDMASK     = $20000000; // Use for normal disabled mask
  IDSF_SELDISABLEDMASK  = $40000000; // Use for selected disabled mask
  IDSF_ERASEMASK        = $80000000; // Draw mask with color zero

// Values for TVectorInfo.Flags
  VIF_WMULTIPLY      = $0000000F; // Multiplier for width = VIF_REFFONT;
  VIF_REFFONT        = $00000010; // Allows reference font to be specified
  VIF_IGNOREWIDTH    = $00000020; // Ignore IA_Width
  VIF_IGNOREHEIGHT   = $00000040; // Ignore IA_Height
  VIF_ROUNDEDSCALING = $00000100; // Needs rounded scaling
  VIF_KEEPASPECT     = $00000200; // Preserve aspect ratio of reference size
  VIF_3DDISABLE      = $00000400; // Uses 3D = embossed; disable effects
  VIF_NOBUFFER       = $00000800; // Render without using off-screen bitmaps
  VIF_PENSHADE       = $00001000; // Uses pen shading, needs a map
  VIF_ALPHASHINE     = $00002000; // Uses bright alpha shading, needs a map
  VIF_ALPHASHADOW    = $00004000; // Uses dark alpha shading, needs a map
  VIF_ALPHASHADE     = $00006000; // Uses both bright and dark alpha shading
  VIF_ALPHABLEND     = $00008000; // Uses alpha blending, needs a map
  VIF_BORDERMASK     = $00FF0000; // Specifications for image borders
  VIF_TBLEFT         = $00010000; // Image is for left side of titlebar
  VIF_TBRIGHT        = $00020000; // Image is for right side of titlebar
  VIF_VERTARROW      = $00040000; // Image is for right window border
  VIF_HORIZARROW     = $00080000; // Image is for bottom window border
  VIF_BRCORN         = $00100000; // Image is for bottom-right  window corner
  VIF_SCREENBAR      = $00200000; // Image is for screen titlebar
  VIF_MENU           = $00400000; // Image is for menu panel
  VIF_BUTTON         = $00800000; // Image must have a button-like  border

// *** The sysiclass rendering hook interface
  SR_DRAW           = 801; // Render symbol imagery for this state
  SR_DRAWMASK       = 802; // Render symbol shape mask for this state
  SR_DRAWDISMASK    = 803; // Render outline mask for disabling, normal
  SR_DRAWSELDISMASK = 804; // Render outline mask for disabling, selected
  SR_DRAWLEVELMAP   = 805; // Render pen shade level map (if any)
  SR_DRAWSHINEMAP   = 806; // Render bright alpha shading map (if any)
  SR_DRAWSHADOWMAP  = 807; // Render dark alpha shading map (if any)
  SR_DRAWALPHAMAP   = 808; // Render alpha blending map (if any)
  SR_DRAWIMAGEFRAME = 809; // Render frame for this image, if any

// These are the sysiclass-specific values for RenderMsg.rm_Flags

  SRF_MONOCHROME     = $00000001; // Render as monochrome
  SRF_ISBITMAP       = $00000002; // A bitmap will later be applied
  SRF_HSCROLLERFRAME = $00000004; // Render frame for horizontal scroller
  SRF_VSCROLLERFRAME = $00000008; // Render frame for vertical scroller

///** The sysiclass plugin interface
// Warning: for style plugin implementors only!
  SC_GLOBALSTYLE      = 0;  // Default (fallback) graphic style
  SC_GLOBALGEOMETRY   = 1;  // Default (fallback) geometry
  SC_WINBORDERSTYLE   = 2;  // Style for window borders
  SC_WINBFRAMESTYLE   = 3;  // Style for border gadget frames
  SC_WINBGADSTYLE     = 4;  // Style for border gadget symbols
  SC_GADGETSTYLE      = 5;  // Style for generic gadget symbols
  SC_ARROWGSTYLE      = 6;  // Style for arrow gadget symbols
  SC_CYCLEGSTYLE      = 7;  // Style for cycle gadget symbols
  SC_KHANDLESTYLE     = 8;  // Style for knob handle symbols
  SC_MENUSTYLE        = 9;  // Style for menu images
  SC_WINBGADGEOMETRY  = 10; // Geometry for border gadgets
  SC_GADGETGEOMETRY   = 11; // Geometry for inner gadgets

type
  PSubStyle = ^TSubStyle;
  TSubStyle = record
   Name: STRPTR;   // Name of the sub-style
   Category: Word; // Category it belongs to
   ID: Word;       // Unique style ID for this plugin
  end;
  PSysIPlugin = ^TSysIPlugin;
  TSysIPlugin = record
   Node: TNode;          // Reserved, don't use
   Version: LongWord;    // Version of the plugin
   Type_:LongWord;       // PLUGIN_SYSI
   Attrs: LongWord;      // Type-specific attributes (see below)
   Flags: LongWord;      // Additional information
   AttrList: PTagItem;   // Optional list of GUI attributes
   Reserved: array[0..3] of LongWord; // For future expansion
   RenderHook: PHook;    // Optional rendering hook
   VIArray: PVectorInfo; // Optional array of struct VectorInfo
   Reserved1: Word;      // For future expansion
   GeometryHook: PHook;  // Optional geometry hook (see below)
   SubStyles: PSubStyle; // Table of available sub-styles
   Reserved2: array[0..3] of LongWord; // For future expansion
  end;
const
// Plugin type
  PLUGIN_SYSI = 1; // Identifies a system imagery plugin
// Plugin attributes (flags)
  SPA_WINBORDERSTYLE  = $00000004; // NOT SUPPORTED YET
  SPA_WINBFRAMESTYLE  = $00000008; // Plugin exports style for border gadget frames
  SPA_WINBGADSTYLE    = $00000010; // Plugin exports style for border gadget symbols
  SPA_GADGETSTYLE     = $00000020; // Plugin exports style for normal gadget symbols
  SPA_MENUSTYLE       = $00000200; // Plugin exports style for menu symbols
  SPA_WINBGADGEOMETRY = $00000400; // Plugin exports geometry for border gadgets
  SPA_GADGETGEOMETRY  = $00000800; // Plugin exports geometry for normal gadgets

// *** The geometry hook interface
  SG_IMAGEBOX  = 901; // Compute box for this (sysiclass) image
  SG_GADGETBOX = 902; // Compute box for this gadget

// Possible return values from a geometry hook

  GCB_OK      = 0; // Hook understands this message type
  GCB_UNKNOWN = 1; // Hook does not understand this message

type
  PImageGeometryMsg = ^TImageGeometryMsg;
  TImageGeometryMsg = record
    igm_MethodID: LongWord;  // One of the SG_ definitions above
    igm_RastPort: PRastPort; // Reference RastPort (for the font)
    igm_DrawInfo: PDrawInfo; // Context information
    igm_ImageBox: TIBox;     // (Output) Box computed by the hook
    igm_RenderBox: TIBox;    // (Output) Box computed by the hook
    igm_Reserved: LongWord;  // Reserved for future use
    igm_Flags: LongWord;     // More information
  end;

// Values for igm_Flags
const
  IGF_WBARLEFT   = $00000001; // Image for left side of window titlebar
  IGF_WBARRIGHT  = $00000002; // Image for right side of window titlebar
  IGF_SBARLEFT   = $00000004; // Image for left side of screen titlebar
  IGF_SBARRIGHT  = $00000008; // Image for right side of screen titlebar
  IGF_VERTARROW  = $00000010; // Image for right window border
  IGF_HORIZARROW = $00000020; // Image for bottom window border
  IGF_BRCORN     = $00000040; // Image for bottom-right window corner
  IGF_KEEPLEFT   = $00001000; // Use the supplied igm_ImageBox.Left
  IGF_KEEPTOP    = $00002000; // Use the supplied igm_ImageBox.Top
  IGF_KEEPWIDTH  = $00004000; // Use the supplied igm_ImageBox.Width
  IGF_KEEPHEIGHT = $00008000; // Use the supplied igm_ImageBox.Height

type
  PGadgetGeometryMsg = ^TGadgetGeometryMsg;
  TGadgetGeometryMsg = record
    ggm_MethodID: LongWord;  // One of the SG_ definitions above
    ggm_Window: PWindow;     // Reference window (may be nil)
    ggm_DrawInfo: PDrawInfo; // Context information
    ggm_BoundingBox: TIBox;  // (Output) Box computed by the hook
    ggm_HitBox: TIBox;       // (Output) Box computed by the hook
    ggm_IAddress: APTR;      // Gadget-specific data (see below)
    ggm_Flags: LongWord;     // More information
  end;

// Values for ggm_Flags
const
  GGF_INITIAL       = $00000001; // We are at OpenWindow() time
  GGF_CALCHSCROLLER = $00000002; // Compute scroller hit box from bounding box
  GGF_CALCVSCROLLER = $00000004; // Compute scroller hit box from bounding box
  GGF_CALCHSOVERLAP = $00000008; // Compute scroller/button overlap (in borders)
  GGF_CALCVSOVERLAP = $00000010; // Compute scroller/button overlap (in borders)
  GGF_BOUNDED       = $00000020; // Container size is passed in ggm_BoundingBox

// Specific data for GTYP_TBARGADGET gadgets (pointed to by ggm_IAddress)
//
// The geometry hook will use this information to determine the correct position of the gadget passed as object. Both the gadgets already in
// the window (or screen) and those in the separate list (which always contains at least the object itself) contribute to the calculation.
// For this kind of gadgets the geometry hook shouldn't depend on the ggm_Window address, since it could be invoked in situations where no
// window information is available and therefore the address is nil.
// New for V51: if GGF_BOUNDED is set in ggm_Flags, your hook can read a "nominal" window size (or screen titlebar size, for GTYP_SCRGADGET
// gadgets) from ggm_BoundingBox. Note that the hook is still expected to initialize ggm_BoundingBox after having read that initial information!
type
  PTBGadgetData = ^TTBGadgetData;
  TTBGadgetData = record
   DomainGadgetList: PGadget; // Gadgets already attached to window/screen Note: this may be nil
   GadgetGadgetList: PGadget; // The gadget list containing the gadget we receive as object (this cannot be nil)
  end;


const
  INTUITIONNAME: PChar = 'intuition.library';

var
  IntuitionBase: PIntuitionBase;
  IIntuition: PInterface = nil;

function IntuitionObtain(): LongWord; syscall IIntuition 60;
function IntuitionRelease(): LongWord; syscall IIntuition 64;
procedure IntuitionExpunge(); syscall IIntuition 68;
function IntuitionClone(): PInterface; syscall IIntuition 72;
procedure OpenIntuition(); syscall IIntuition 76;
procedure Intuition_(iEvent: PInputEvent); syscall IIntuition 80;
function AddGadget(Window: PWindow; Gadget: PGadget; Position: LongWord): LongWord; syscall IIntuition 84;
function ClearDMRequest(Eindow: PWindow): LongBool; syscall IIntuition 88;
procedure ClearMenuStrip(Window: PWindow); syscall IIntuition 92;
procedure ClearPointer(Window:PWindow); syscall IIntuition 96;
function CloseScreen(Screen: PScreen): LongBool; syscall IIntuition 100;
procedure CloseWindow(Window: PWindow); syscall IIntuition 104;
function CloseWorkBench: LongBool; syscall IIntuition 108;
procedure CurrentTime(var Seconds: LongWord; var Micros: LongWord); syscall IIntuition 112;
function DisplayAlert(AlertNumber: LongWord; const String_: STRPTR; Height: LongWord): LongBool; syscall IIntuition 116;
procedure DisplayBeep(Screen: PScreen); syscall IIntuition 116;
function DoubleClick(SSeconds, SMicros, CSeconds, CMicros: LongWord): LongBool; syscall IIntuition 124;
procedure DrawBorder(Rp: PRastPort ;const Border: PBorder; LeftOffset, TopOffset: LongInt); syscall IIntuition 128;
procedure DrawImage(Rp: PRastPort; Image: PImage; LeftOffset, TopOffset: LongInt); syscall IIntuition 132;
procedure EndRequest(Requester: PRequester; Window: PWindow); syscall IIntuition 136;
function GetDefPrefs(Preferences: PPreferences; Size: LongInt): PPreferences; syscall IIntuition 140;
function GetPrefs(Preferences: PPreferences; Size: LongInt): PPreferences; syscall IIntuition 144;
procedure InitRequester(Requester: PRequester); syscall IIntuition 148;
function ItemAddress(const MenuStrip: PMenu; MenuNumber: LongWord): PMenuItem; syscall IIntuition 152;
function ModifyIDCMP(Window: PWindow; Flags: LongWord): LongBool; syscall IIntuition 156;
procedure ModifyProp(Gadget: PGadget; Window: PWindow; Requester: PRequester; Flags, HorizPot, VertPot, HorizBody, VertBody: LongWord); syscall IIntuition 160;
procedure MoveScreen(Screen: PScreen; Dx, Dy: LongInt); syscall IIntuition 164;
procedure MoveWindow(Window: PWindow; Dx, Dy: LongInt); syscall IIntuition 168;
procedure OffGadget(Gadget: PGadget; Window: PWindow; Requester: PRequester); syscall IIntuition 172;
procedure OffMenu(Window: PWindow; MenuNumber: LongWord); syscall IIntuition 176;
procedure OnGadget(Gadget: PGadget; Window: PWindow; Requester: PRequester); syscall IIntuition 180;
procedure OnMenu(Window: PWindow; MenuNumber: LongWord); syscall IIntuition 184;
function OpenScreen(const NewScreen: PNewScreen): PScreen; syscall IIntuition 188;
function OpenWindow(const NewWindow: PNewWindow): PWindow; syscall IIntuition 192;
function OpenWorkBench: LongWord; syscall IIntuition 196;
procedure PrintIText(Rp: PRastPort; const IText: PIntuiText; Left, Top: LongInt); syscall IIntuition 200;
procedure RefreshGadgets(Gadgets: PGadget; Window: PWindow; Requester: PRequester); syscall IIntuition 204;
function RemoveGadget(Window: PWindow; Gadget: PGadget): LongWord; syscall IIntuition 208;
procedure ReportMouse(Flag: LongInt; Window: PWindow); syscall IIntuition 212;
procedure ReportMouse1(Window: PWindow; Flag: LongInt); syscall IIntuition 216;
function Request(Requester: PRequester; Window: PWindow): LongBool; syscall IIntuition 220;
procedure ScreenToBack(Screen: PScreen); syscall IIntuition 224;
procedure ScreenToFront(Screen: PScreen); syscall IIntuition 228;
function SetDMRequest(Window: PWindow; Requester: PRequester): LongBool; syscall IIntuition 232;
function SetMenuStrip(Window: PWindow; Menu: PMenu): LongBool; syscall IIntuition 236;
procedure SetPointer(Window: PWindow; Pointer_: PWord; Height, Width, XOffset, yOffset: LongInt); syscall IIntuition 240;
procedure SetWindowTitles(Window: PWindow; const WindowTitle: STRPTR; const ScreenTitle: STRPTR); syscall IIntuition 244;
procedure ShowTitle(Screen: PScreen; ShowIt: LongInt); syscall IIntuition 248;
procedure SizeWindow(Window: PWindow; Dx, Dy: LongInt); syscall IIntuition 252;
function ViewAddress: PView; syscall IIntuition 256;
function ViewPortAddress(const Window: PWindow): PViewPort; syscall IIntuition 260;
procedure WindowToBack(Window: PWindow); syscall IIntuition 264;
procedure WindowToFront(Window: PWindow); syscall IIntuition 268;
function WindowLimits(Window: PWindow; WidthMin, HeightMin: LongInt; WidthMax, HeightMax: LongWord): LongBool; syscall IIntuition 272;
function SetPrefs(const Preferences: PPreferences; Size: LongInt; Inform: LongInt): PPreferences; syscall IIntuition 276;
function IntuiTextLength(const IText: PIntuiText): LongInt; syscall IIntuition 280;
function WBenchToBack: LongBool; syscall IIntuition 284;
function WBenchToFront: LongBool; syscall IIntuition 288;
function AutoRequest(Window: PWindow; const Body: PIntuiText; const PosText: PIntuiText; const NegText: PIntuiText; PFlag, nFlag: LongWord; Width, Height: LongWord): LongBool; syscall IIntuition 292;
procedure BeginRefresh(Window: PWindow); syscall IIntuition 296;
function BuildSysRequest(Window: PWindow; const Body: PIntuiText; const PosText: PIntuiText; const NegText: PIntuiText; Flags: LongWord; Width, Height: LongWord): PWindow; syscall IIntuition 300;
procedure EndRefresh(Window: PWindow; Complete: LongBool); syscall IIntuition 304;
procedure FreeSysRequest(Window: PWindow); syscall IIntuition 308;
function MakeScreen(Screen: PScreen): LongInt; syscall IIntuition 312;
function RemakeDisplay: LongInt; syscall IIntuition 316;
function RethinkDisplay: LongInt; syscall IIntuition 320;
function AllocRemember(var RememberKey: PRemember; Size: LongWord; Flags: LongWord): APTR syscall IIntuition 324;
procedure AlohaWorkbench(WBPort: LongInt); syscall IIntuition 328;
procedure FreeRemember(var RememberKey: PRemember; ReallyForget: LongInt); syscall IIntuition 332;
function LockIBase(DontKnow: LongWord): LongWord; syscall IIntuition 336;
procedure UnlockIBase(IBLock: LongWord); syscall IIntuition 340;
function GetScreenData(Buffer: APTR; Size, Type_: LongWord; const Screen: PScreen): LongBool; syscall IIntuition 344;
procedure RefreshGList(Gadgets: PGadget; Window: PWindow; Requester: PRequester; NumGad: LongWord); syscall IIntuition 348;
function AddGList(Window: PWindow; Gadget: PGadget; Position: LongWord; NumGad: LongInt; Requester: PRequester): LongWord; syscall IIntuition 352;
function RemoveGList(RemPtr: PWindow; Gadget: PGadget; NumGad: LongInt): LongWord; syscall IIntuition 356;
procedure ActivateWindow(Window: PWindow); syscall IIntuition 360;
procedure RefreshWindowFrame(Window: PWindow); syscall IIntuition 364;
function ActivateGadget(Gadgets: PGadget; Window: PWindow; Requester: PRequester): LongBool syscall IIntuition 368;
procedure NewModifyProp(Gadget: PGadget; Window: PWindow; Requester: PRequester; Flags, HorizPot, VertPot, HorizBody, VertBody: LongWord; NumGad: LongInt); syscall IIntuition 372;
function QueryOverscan(DisplayID: LongWord; Rect: PRectangle; OScanType: LongInt): LongInt; syscall IIntuition 376;
procedure MoveWindowInFrontOf(Window: PWindow; BehindWindow: PWindow); syscall IIntuition 380;
procedure ChangeWindowBox(Window: PWindow; Left, Top, Width, Height: LongInt); syscall IIntuition 384;
function SetEditHook(Hook: PHook): PHook; syscall IIntuition 388;
function SetMouseQueue(Window: PWindow; QueueLength: LongWord): LongInt; syscall IIntuition 392;
procedure ZipWindow(Window: PWindow); syscall IIntuition 396;
function LockPubScreen(const Name: STRPTR): PScreen; syscall IIntuition 400;
procedure UnlockPubScreen(const Name: STRPTR; Screen: PScreen); syscall IIntuition 404;
function LockPubScreenList: PList; syscall IIntuition 408;
procedure UnlockPubScreenList; syscall IIntuition 412;
function NextPubScreen(const Screen: PScreen; NameBuf: STRPTR): STRPTR; syscall IIntuition 416;
procedure SetDefaultPubScreen(const Name: STRPTR); syscall IIntuition 420;
function SetPubScreenModes(Modes: LongWord): LongWord; syscall IIntuition 424;
function PubScreenStatus(Screen: PScreen; StatusFlags: LongWord): LongWord; syscall IIntuition 428;
function ObtainGIRPort(GInfo: PGadgetInfo): PRastPort; syscall IIntuition 432;
procedure ReleaseGIRPort(Rp: PRastPort); syscall IIntuition 436;
procedure GadgetMouse(Gadget: PGadget; GInfo: PGadgetInfo; MousePoint: PSmallInt); syscall IIntuition 440;
function SetIPrefs(Ptr: APTR; Size, Type_: LongInt): APTR; syscall IIntuition 444;
procedure GetDefaultPubScreen(NameBuffer: STRPTR); syscall IIntuition 448;
function EasyRequestArgs(Window: PWindow; const EasyStruct: PEasyStruct; IDCMPPtr: PLongWord; const Args: APTR): LongInt; syscall IIntuition 452;
// 456 EasyRequest
function BuildEasyRequestArgs(Window: PWindow; const EasyStruct: PEasyStruct; IDCMP: LongWord; const Args: APTR): PWindow; syscall IIntuition 460;
// 464 BuildEasyRequest
function SysReqHandler(Window: PWindow; IDCMPPtr: PLongWord; WaitInput: LongInt): LongInt; syscall IIntuition 468;
function OpenWindowTagList(const NewWindow: PNewWindow; const TagList: PTagItem): PWindow; syscall IIntuition 472;
// 476 OpenWindowTags
function OpenScreenTagList(const NewScreen: PNewScreen; const TagList: PTagItem): PScreen; syscall IIntuition 480;
// 484 OpenScreenTags
procedure DrawImageState(Rp: PRastPort; Image: PImage; LeftOffset, TopOffset: LongInt; State: LongWord; const DrawInfo: PDrawInfo); syscall IIntuition 488;
function PointInImage(Point: LongWord; Image: PImage): LongBool; syscall IIntuition 492;
procedure EraseImage(Rp: PRastPort; Image: PImage; LeftOffset, TopOffset: LongInt); syscall IIntuition 496;
function NewObjectA(ClassPtr: PIClass; const ClassID: ClassID; const TagList: PTagItem): PObject_; syscall IIntuition 500;
// 504 NewObject
procedure DisposeObject(Obj: PObject_); syscall IIntuition 508;
function SetAttrsA(obj: PObject_; const TagList: PTagItem): LongWord; syscall IIntuition 512;
// 516 SetAttrs
function GetAttr(AttrID: LongWord; obj: PObject_; StoragePtr: PLongWord): LongWord; overload; syscall IIntuition 520;
function GetAttr(AttrID: LongWord; obj: PObject_; var Storage: LongWord): LongWord; overload; syscall IIntuition 520;
function SetGadgetAttrsA(Gadget: PGadget; Window: PWindow; Requester: PRequester; const TagList: PTagItem): LongWord; syscall IIntuition 524;
// 528 SetGadgetAttrs
function NextObject(ObjectPtrPtr: PPObject_): PObject_; syscall IIntuition 532;
function FindClass(ClassID: ClassID): PIClass; syscall IIntuition 536;
function MakeClass(const ClassID: ClassID; const SuperClassID: ClassID; const SuperClassPtr: PIClass; InstanceSize: LongWord; Flags: LongWord): PIClass; syscall IIntuition 540;
procedure AddClass(ClassPtr: PIClass); syscall IIntuition 544;
function GetScreenDrawInfo(Screen: PScreen): PDrawInfo; syscall IIntuition 548;
procedure FreeScreenDrawInfo(Screen: PScreen; DrawInfo: PDrawInfo); syscall IIntuition 552;
function ResetMenuStrip(Window: PWindow; Menu: PMenu): LongBool; syscall IIntuition 556;
procedure RemoveClass(ClassPtr: PIClass); syscall IIntuition 560;
function FreeClass(ClassPtr: PIClass): LongBool; syscall IIntuition 564;
function LockClassList(): PList; syscall IIntuition 568;
procedure UnlockClassList(); syscall IIntuition 572;
// 576-596 reserved
function AllocScreenBuffer(Sc: PScreen; Bm: PBitMap; Flags: LongWord): PScreenBuffer; syscall IIntuition 600;
procedure FreeScreenBuffer(Sc: PScreen; Sb: PScreenBuffer); syscall IIntuition 604;
function ChangeScreenBuffer(Sc: PScreen; Sb: PScreenBuffer): LongWord; syscall IIntuition 608;
procedure ScreenDepth(Screen: PScreen; Flags: LongWord; Reserved: APTR); syscall IIntuition 612;
procedure ScreenPosition(Screen: PScreen; Flags: LongWord; X1, Y1, X2, Y2: LongInt); syscall IIntuition 616;
procedure ScrollWindowRaster(Win: PWindow; Dx, Dy, XMin, YMin, XMax, YMax: LongInt); syscall IIntuition 620;
procedure LendMenus(FromWindow: PWindow; ToWindow: PWindow); syscall IIntuition 624;
function DoGadgetMethodA(Gad: PGadget; Win: PWindow; Req: PRequester; Message_: TMsg): LongWord; syscall IIntuition 628;
// 632 DoGadgetMethod
procedure SetWindowPointerA(Win: PWindow; const Taglist: PTagItem); syscall IIntuition 636;
// 640 SetWindowPointer
function TimedDisplayAlert(AlertNumber: LongWord; const String_: STRPTR; Height: LongWord; Time: LongWord): LongBool; syscall IIntuition 644;
procedure HelpControl(Win: PWindow; Flags: LongWord); syscall IIntuition 648;
function ShowWindow(Window: PWindow; Other: PWindow): LongBool; syscall IIntuition 652;
function HideWindow(Window: PWindow): LongBool; syscall IIntuition 656;
function GetAttrsA(Object_: PObject_; TagList: PTagItem): LongWord; syscall IIntuition 660;
// 664 GetAttrs
function LockGUIPrefs(Reserved: LongWord): APTR; syscall IIntuition 668;
procedure UnlockGUIPrefs(Lock: APTR); syscall IIntuition 672;
function SetGUIAttrsA(Reserved: APTR; DrawInfo: PDrawInfo; TagList: PTagItem): LongWord; syscall IIntuition 676;
// 680 SetGUIAttrs
function GetGUIAttrsA(Reserved: APTR; DrawInfo: PDrawInfo; TagList: PTagItem): LongWord; syscall IIntuition 684;
// 684 GetGUIAttrs
function GetHalfPens(DrawInfo: PDrawInfo; BasePen: LongWord; HalfShinePtr: PWord; HalfShadowPtr: PWord): LongWord; syscall IIntuition 692;
function GadgetBox(Gadget: PGadget; Domain: APTR; DomainType, Flags: LongWord; Box: APTR): LongWord; syscall IIntuition 696;
procedure RefreshSetGadgetAttrsA(Gadget: PGadget; Window: PWindow; Requester: PRequester; TagList: PTagItem); syscall IIntuition 700;
// 704 RefreshSetGadgetAttrs
function IDoSuperMethodA(Cl: PIClass; Object_: PObject_; Msg: TMsg): LongWord; syscall IIntuition 708;
// 712 IDoSuperMethod
function ISetSuperAttrsA(Cl: PIClass; Object_: PObject_; TagList: PTagItem): LongWord; syscall IIntuition 716;
// 720 ISetSuperAttrs
function ICoerceMethodA(Cl: PIClass; Object_: PObject_; Msg: TMsg): LongWord; syscall IIntuition 724;
// 728 ICoerceMethod
function IDoMethodA(Object_: PObject_; Msg: TMsg): LongWord; syscall IIntuition 732;
// 736 IDoMethod
function OpenClass(const Name: STRPTR; Version: LongWord; var ClassPtr: PIClass): LongWord; syscall IIntuition 740;
procedure CloseClass(Cl: PIClass); syscall IIntuition 744;
function SetDisplayBeepHook(Hook: PHook): PHook; syscall IIntuition 748;
function LockScreen(Screen: PScreen; Micros: LongWord): LongBool; syscall IIntuition 752;
procedure UnlockScreen(Screen: PScreen); syscall IIntuition 756;
function GetWindowAttrsA(Win: PWindow; TagList: PTagItem): LongInt; syscall IIntuition 760;
// 764 GetWindowAttrs
function SetWindowAttrsA(Win: PWindow; TagList: PTagItem): LongInt; syscall IIntuition 768;
// 772 SetWindowAttrs
function GetWindowAttr(Win: PWindow; Attr: LongWord; Data: APTR; Size: LongWord): LongInt; syscall IIntuition 776;
function SetWindowAttr(Win: PWindow; Attr: LongWord; Data: APTR; Size: LongWord): LongInt; syscall IIntuition 780;
procedure StripIntuiMessages(Port: PMsgPort; Win: PWindow); syscall IIntuition 784;
// 788 Reserved
// 792 Reserved
function GetScreenAttrsA(Screen: PScreen; TagList: PTagItem): LongInt; syscall IIntuition 796;
// 800 GetScreenAttrs
function SetScreenAttrsA(Screen: PScreen; TagList: PTagItem): LongInt; syscall IIntuition 804;
// 808 SetScreenAttrs
function GetScreenAttr(Screen: PScreen; Attr: LongWord; Data: APTR; Size: LongWord): LongInt; syscall IIntuition 812;
function SetScreenAttr(Screen: PScreen; Attr: LongWord; Data: APTR; Size: LongWord): LongInt; syscall IIntuition 816;
function LockScreenList(): PScreen; syscall IIntuition 820;
procedure UnlockScreenList(); syscall IIntuition 824;
function LockScreenGI(gi: PGadgetInfo; Micros: LongWord): PScreen; syscall IIntuition 828;
procedure UnlockScreenGI(gi: PGadgetInfo; Rp: PRastPort); syscall IIntuition 832;
function GetMarkedBlock(Sgw: PSGWork): LongWord; syscall IIntuition 836;
procedure SetMarkedBlock(Sgw: PSGWork; Block: LongWord); syscall IIntuition 840;
function ObtainBitMapSourceA(const Name: STRPTR; TagList: PTagItem): APTR; syscall IIntuition 844;
// 848 ObtainBitMapSource
procedure ReleaseBitMapSource(BitmapSource: APTR); syscall IIntuition 852;
function ObtainBitMapInstanceA(BitmapSource: APTR; Screen: PScreen; TagList: PTagItem): APTR; syscall IIntuition 856;
// 860 ObtainBitMapInstance
procedure ReleaseBitMapInstance(BitmapInstance: APTR); syscall IIntuition 864;
procedure EmbossDisableRect(Rp: PRastPort; MinX, MinY, MaxX, MaxY: LongInt; BackType, Contrast: LongWord; Dri: PDrawInfo); syscall IIntuition 868;
procedure EmbossDisableText(Rp: PRastPort; const Text: STRPTR; Len, BackType, Contrast: LongWord; Dri: PDrawInfo); syscall IIntuition 872;
procedure PrintEmbossedDisabledIText(Rp: PRastPort; IText: PIntuiText; Left, Top: LongInt; BackType, Contrast: LongWord; Dri: PDrawInfo); syscall IIntuition 876;
function IntuiTextExtent(Rp: PRastPort; IText: PIntuiText; TextExtent: PTextExtent): LongWord; syscall IIntuition 880;
function ShadeRectOld(Rp: PRastPort; MinX, MinY, MaxX, MaxY: LongInt; ShadeLevel, BackType, State: LongWord; Dri: PDrawInfo): LongWord; syscall IIntuition 884;
procedure DisableTemplateRGB(Rp: PRastPort; Left, Top, Width, Height: LongInt; TemplatePtr: TPlanePtr; BrightLevel, DarkLevel: LongWord); syscall IIntuition 888;
function SetScreenBitMapInstance(Scr: PScreen; ID: LongWord; Source: APTR): LongWord; syscall IIntuition 892;
function FindMenuKey(Menu: PMenu; Code: LongInt): LongWord; syscall IIntuition 896;
function BitMapInstanceControlA(BitmapInstance: APTR; TagList: PTagItem): LongWord; syscall IIntuition 900;
// 904 BitMapInstanceControl
function ObtainIPluginList(Type_, AttrMask, ApplyMask: LongWord): PList; syscall IIntuition 908;
procedure ReleaseIPluginList(List: PList); syscall IIntuition 912;
function OpenGUIPlugin(Name: STRPTR; Version, Type_, AttrMask, ApplyMask: LongWord): PGUIPlugin; syscall IIntuition 916;
procedure CloseGUIPlugin(Plugin: PGUIPlugin); syscall IIntuition 920;
function DrawSysImageA(Rp: PRastPort; Left, Top, Width, Height: LongInt; Which, BackType, State: LongWord; Dri: PDrawInfo; TagList: PTagItem): LongWord; syscall IIntuition 924;
// 928 DrawSysImage
function DoRender(O: PObject_; Gi: PGadgetInfo; Flags: LongWord): LongWord; syscall IIntuition 932;
function SetRenderDomain(Rp: PRastPort; Domain: PRectangle): LongWord; syscall IIntuition 936;
function GetRenderDomain(Rp: PRastPort; Domain: PRectangle): LongWord; syscall IIntuition 940;
function DrawGradient(Rp: PRastPort; Left, Top, Width, Height: LongInt; Domain: PIBox; Reserved: LongWord; GradientSpec: PGradientSpec; Dri: PDrawInfo): LongWord; syscall IIntuition 944;
function DirectionVector(Degrees: LongWord): LongWord; syscall IIntuition 948;
function ShadeRectA(Rp: PRastPort; MinX, MinY, MaxX, MaxY: LongInt; ShadeLevel, BackType, State: LongWord; Dri: PDrawInfo; TagList: PTagItem): LongWord; syscall IIntuition 952;
// 956 ShadeRect
procedure DoScrollHook(ScrollHook: PScrollHook; ScrollMode: LongInt); syscall IIntuition 960;
function ObtainIBackFill(Dri: PDrawInfo; Element, State, Flags: LongWord): PHook; syscall IIntuition 964;
procedure ReleaseIBackFill(Hook: PHook); syscall IIntuition 968;
function IntuitionControlA(Object_: APTR; TagList: PTagItem): LongWord; syscall IIntuition 972;
// 976 IntuitionControl
function StartScreenNotifyTagList(TagList: PTagItem): APTR; syscall IIntuition 980;
// 984 StartScreenNotifyTags
function EndScreenNotify(Request: APTR): LongBool; syscall IIntuition 988;
procedure DisableTemplate(Rp: PRastPort; Left, Top, Width, Height: LongInt; TemplatePtr: APTR; OffX, OffY: LongInt; TemplateType, BytesPerRow, BackType: LongWord; Dri: PDrawInfo); syscall IIntuition 992;

function OpenScreenTags(newScreen: PNewScreen; const TagList: array of PtrUInt): PScreen;
function OpenWindowTags(newWindow: PNewWindow; const TagList: array of PtrUInt): PWindow;
function NewObject(ClassPtr: PIClass; ClassID: PChar; const argv: array of PtrUInt): Pointer;
function SetAttrs(obj: Pointer; const Tags: array of PtrUInt): LongWord;
function SetGadgetAttrs(gadget: PGadget; Window: PWindow; Requester: PRequester; const argv: array of PtrUInt): LongWord;
function EasyRequest(Window: PWindow; const EasyStruct: PEasyStruct; IDCMPPtr: PLongWord; const args: array of PtrUInt): LongInt;
procedure SetWindowPointer(Win: PWindow; const Tags: array of PtrUInt);
function DoMethodA(Obj: PObject_; Message: APTR): LongWord;
function DoMethod(Obj: PObject_; const Args: array of PtrUInt): LongWord;
function CoerceMethodA(cl: PIClass; Obj: PObject_; Message: APTR): LongWord;
function CoerceMethod(cl: PIClass; Obj: PObject_; const Args: array of PtrUInt): LongWord;
function DoSuperMethodA(cl: PIClass; Obj: PObject_; Message: APTR): LongWord;
function DoSuperMethod(cl: PIClass; Obj: PObject_; const Args: array of PtrUInt): LongWord;


{ Intuition macros }
function INST_DATA (cl: PIClass; o: P_Object): Pointer;
function SIZEOF_INSTANCE (cl: PIClass): LongInt;
function BASEOBJECT (o: P_Object): Pointer;
function _OBJ(o: P_Object): P_Object; inline;
function __OBJECT(o: Pointer): P_Object; inline;
function OCLASS(o: Pointer): PIClass; inline;
function SHIFTITEM(n: SmallInt): Word;
function SHIFTMENU(n: SmallInt): Word;
function SHIFTSUB(n: SmallInt): Word;
function FULLMENUNUM (menu, item, sub: SmallInt): Word;
function IM_BGPEN(im: PImage): Byte;
function IM_BOX(im: PImage): PIBox;
function IM_FGPEN(im: PImage): Byte;
function GADGET_BOX(g: PGadget): PIBox;
function CUSTOM_HOOK(gadget: PGadget): PHook;
function ITEMNUM(n: Word): Word;
function MENUNUM(n: Word): Word;
function SUBNUM(n: Word): Word;

IMPLEMENTATION

function OpenScreenTags(NewScreen: PNewScreen; const TagList: array of PtrUInt): PScreen;
begin
  OpenScreenTags := OpenScreenTagList(NewScreen, @TagList);
end;

function OpenWindowTags(NewWindow: PNewWindow; const TagList: array of PtrUInt): PWindow;
begin
  OpenWindowTags := OpenWindowTagList(NewWindow, @TagList);
end;

function NewObject(ClassPtr: PIClass; ClassID: PChar; const argv: array of PtrUInt): Pointer;
begin
  NewObject := NewObjectA(ClassPtr, ClassID, @argv);
end;

function SetAttrs(Obj: Pointer; const Tags: array of PtrUInt): LongWord;
begin
  SetAttrs := SetAttrsA(Obj, @Tags);
end;

function SetGadgetAttrs(Gadget: PGadget; Window: PWindow; Requester: PRequester; const argv: array of PtrUInt): LongWord;
begin
  SetGadgetAttrs := SetGadgetAttrsA(Gadget, Window, Requester, @argv);
end;

function EasyRequest(Window: PWindow; const EasyStruct: PEasyStruct; IDCMPPtr: PLongWord; const args: array of PtrUInt): LongInt;
begin
  EasyRequest := EasyRequestArgs(Window, Easystruct, IDCMPPtr, @args);
end;

procedure SetWindowPointer(Win: PWindow; const Tags: array of PtrUInt);
begin
  SetWindowPointerA(Win, @Tags);
end;

// Functions wrapper

function DoMethodA(Obj: PObject_; Message: APTR): LongWord; inline;
begin
  DoMethodA := 0;
  if Obj = nil then
    Exit;
  DoMethodA := CallHookPkt(PHook(OCLASS(Obj)), Obj, Message);
end;

function DoMethod(Obj: PObject_; const Args: array of PtrUInt): LongWord; inline;
begin
  DoMethod := 0;
  if obj = nil then
    Exit;
  DoMethod := CallHookPkt(PHook(OCLASS(Obj)), Obj, @Args);
end;

function DoSuperMethodA(cl: PIClass; Obj: PObject_; Message: APTR): LongWord; inline;
begin
  DoSuperMethodA := 0;
  if (cl = nil) or (obj = nil) then
    Exit;
  DoSuperMethodA := CallHookPkt(PHook(cl^.cl_Super), Obj, Message);
end;

function DoSuperMethod(cl: PIClass; Obj: PObject_; const Args: array of PtrUInt): LongWord; inline;
begin
  DoSuperMethod := 0;
  if (cl = nil) or (obj = nil) then
    Exit;
  DoSuperMethod := CallHookPkt(PHook(cl^.cl_Super), Obj, @Args);
end;

function CoerceMethodA(cl: PIClass; Obj: PObject_; Message: APTR): LongWord; inline;
begin
  CoerceMethodA := 0;
  if (cl = nil) or (obj = nil) then
    Exit;
  CoerceMethodA := CallHookPkt(PHook(cl), Obj, Message);
end;

function CoerceMethod(cl: PIClass; Obj: PObject_; const Args: array of PtrUInt): LongWord; inline;
begin
  CoerceMethod := CoerceMethodA(cl, Obj, @Args);
end;

function INST_DATA(cl: PIClass; o: P_Object): Pointer; inline;
begin
  INST_DATA := Pointer(PtrUInt(o) + cl^.cl_InstOffset);
end;

function SIZEOF_INSTANCE(cl: PIClass): LongInt; inline;
begin
  SIZEOF_INSTANCE := cl^.cl_InstOffset + cl^.cl_InstSize + SizeOf(T_Object);
end;

function BASEOBJECT(o: P_Object): Pointer; inline;
begin
  BASEOBJECT := Pointer(PtrUInt(o) + SizeOf(T_Object));
end;

function _OBJ(o: P_Object): P_Object; inline;
begin
   _OBJ := P_Object(o);
end;

function __OBJECT(o: Pointer): P_Object; inline;
begin
  __OBJECT := P_Object(PtrUInt(PtrUInt(o) - SizeOf(T_Object)));
end;

function OCLASS (o: Pointer): PIClass; inline;
begin
  OCLASS := P_Object(o - SizeOf(T_Object))^.o_Class;
end;

function SHIFTITEM (n: SmallInt): word; inline;
begin
  SHIFTITEM := (n and $3f) shl 5
end;

function SHIFTMENU (n: SmallInt): word; inline;
begin
  SHIFTMENU := n and $1f
end;

function SHIFTSUB (n: SmallInt): word; inline;
begin
  SHIFTSUB := (n and $1f) shl 11
end;

function FULLMENUNUM (menu, item, sub: smallint): word; inline;
begin
  FULLMENUNUM := ((sub and $1f) shl 11) or ((item and $3f) shl 5) or (menu and $1f)
end;



{ The next functons _BGPEN AND _FGPEN aren't a full replacement of the
  C macros because the C preprocessor makes it possible to set the
  A/BPen values of the image class objects as well. This can't work
  in pascal, of course! }

function IM_BGPEN (im: PImage): byte; inline;
begin
  IM_BGPEN := im^.PlaneOnOff;
end;

function IM_BOX (im: PImage): pIBox; inline;
begin
  IM_BOX := pIBox(@im^.LeftEdge);
end;

function IM_FGPEN(im: PImage): byte; inline;
begin
  IM_FGPEN := im^.PlanePick;
end;

function GADGET_BOX(g: PGadget): pIBox; inline;
begin
  GADGET_BOX := pIBox(@g^.LeftEdge);
end;

function CUSTOM_HOOK(gadget: PGadget): pHook; inline;
begin
  CUSTOM_HOOK := pHook(gadget^.MutualExclude);
end;

function ITEMNUM(n: Word): Word; inline;
begin
  ITEMNUM := (n shr 5) and $3F
end;

function MENUNUM(n: Word): Word; inline;
begin
  MENUNUM := n and $1f
end;

function SUBNUM(n: Word): Word; inline;
begin
  SUBNUM := (n shr 11) and $1f
end;


const
  // Change VERSION to proper values
  LIBVERSION: LongWord = 0;

initialization
  IntuitionBase := PIntuitionBase(OpenLibrary(INTUITIONNAME, LIBVERSION));
  if Assigned(IntuitionBase) then
    IIntuition := GetInterface(PLibrary(IntuitionBase), 'main', 1, nil);
finalization
  if Assigned(IIntuition) then
    DropInterface(IIntuition);
  if Assigned(IntuitionBase) then
    CloseLibrary(PLibrary(IntuitionBase));
end.


















