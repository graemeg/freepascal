{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2004 Karoly Balogh for Genesi S.a.r.l. <www.genesi.lu>

    intuition.library interface unit for MorphOS/PowerPC

    Based on work of Nils Sjoholm member of the Amiga RTL
    development team.

    MorphOS port was done on a free Pegasos II/G4 machine
    provided by Genesi S.a.r.l. <www.genesi.lu>

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$PACKRECORDS 2}
{$INLINE ON}
unit intuition;

interface

uses
  exec, agraphics, utility, inputevent, timer, layers;

{$define INTUI_V36_NAMES_ONLY}

{
 * NOTE:  intuition/iobsolete.h is included at the END of this file!
 }


type
{ IntuiText is a series of strings that start with a screen location
  (always relative to the upper-left corner of something) and then the
  text of the string.  The text is null-terminated.}
  PIntuiText = ^TIntuiText;
  TIntuiText = record
    FrontPen,
    BackPen: Byte;         // the pen numbers for the rendering
    DrawMode: Byte;        // the mode for rendering the text
    LeftEdge: SmallInt;    // relative start location for the text
    TopEdge: SmallInt;     // relative start location for the text
    ITextFont: PTextAttr;  // if nil, you accept the default
    IText: PChar;          // pointer to null-terminated text
    NextText: PIntuiText;  // continuation to TxWrite another text
  end;



{ Data type Border, used for drawing a series of lines which is intended for use as a border drawing, but which may, in fact, be used to render any
  arbitrary vector shape. The routine DrawBorder sets up the RastPort with the appropriate variables, then does a Move to the first coordinate, then does Draws
  to the subsequent coordinates. After all the Draws are done, if NextBorder is non-zero we call DrawBorder recursively}
type
  PBorder = ^TBorder;
  TBorder = record
    LeftEdge,
    TopEdge: SmallInt;   // initial offsets from the origin
    FrontPen,
    BackPen: Byte;       // pens numbers for rendering
    DrawMode: Byte;      // mode for rendering
    Count: ShortInt;     // number of XY pairs
    XY: PSmallInt;       // vector coordinate pairs rel to LeftTop
    NextBorder: PBorder; // pointer to any other Border too
  end;

// MenuItem
type
  PMenuItem = ^TMenuItem;
  TMenuItem = record
    NextItem: PMenuItem;    // pointer to next in chained list
    LeftEdge,
    TopEdge: SmallInt;      // position of the select box
    Width,
    Height: SmallInt;       // dimensions of the select box
    Flags: Word;            // see the defines below
    MutualExclude: LongInt; // set bits mean this item excludes that
    ItemFill: APTR;         // points to Image, IntuiText, or nil
    SelectFill: APTR;       // points to Image, IntuiText, or nil when this item is pointed to by the cursor and the items highlight mode HIGHIMAGE is selected, this alternate image will be displayed
    Command: Char;          // only if appliprog sets the COMMSEQ flag
    SubItem: PMenuItem;     // if non-nil, DrawMenu shows "->"
    NextSelect: Word;       // The NextSelect field represents the menu number of next selected item (when user has drag-selected several items)
  end;

const
// FLAGS SET BY THE APPLIPROG
  CHECKIT     = $0001;        // whether to check this item if selected
  ITEMTEXT    = $0002;        // set if textual, clear if graphical item
  COMMSEQ     = $0004;        // set if there's an command sequence
  MENUTOGGLE  = $0008;        // set to toggle the check of a menu item
  ITEMENABLED = $0010;        // set if this item is enabled
// these are the SPECIAL HIGHLIGHT FLAG state meanings
  HIGHFLAGS   = $00C0;        // see definitions below for these bits
  HIGHIMAGE   = $0000;        // use the user's "select image"
  HIGHCOMP    = $0040;        // highlight by complementing the selectbox
  HIGHBOX     = $0080;        // highlight by "boxing" the selectbox
  HIGHNONE    = $00C0;        // don't highlight
// FLAGS SET BY BOTH APPLIPROG AND INTUITION
  CHECKED     = $0100;        // if CHECKIT, then set this when selected
// FLAGS SET BY INTUITION
  ISDRAWN     = $1000;        // this item's subs are currently drawn
  HIGHITEM    = $2000;        // this item is currently highlighted
  MENUTOGGLED = $4000;        // this item was already toggled

// Menu
type
  PMenu = ^TMenu;
  TMenu = record
    NextMenu: PMenu;      // same level
    LeftEdge,
    TopEdge: SmallInt;    // position of the select box
    Width,
    Height: SmallInt;     // dimensions of the select box
    Flags: Word;          // see flag definitions below (MENUENABLED, MIDRAWN)
    MenuName: PChar;      // text for this Menu Header
    FirstItem: PMenuItem; // pointer to first in chain
    JazzX,                // these mysteriously-named variables are for internal use only
    JazzY,
    BeatX,
    BeatY: SmallInt;
  end;

const
// FLAGS SET BY BOTH THE APPLIPROG AND INTUITION
  MENUENABLED = $0001; // whether or not this menu is enabled
// FLAGS SET BY INTUITION
  MIDRAWN     = $0100; // this menu's items are currently drawn

// Gadget
type
  PGadget = ^TGadget;
  TGadget = record
    NextGadget: PGadget;  // next gadget in the list

    LeftEdge,
    TopEdge: SmallInt;    // "hit box" of gadget
    Width,
    Height: SmallInt;     // "hit box" of gadget

    Flags: Word;          // see below for list of defines GFLG_*
    Activation: Word;     // see below for list of defines GACT_*
    GadgetType: Word;     // see below for defines GTYP_*

    GadgetRender: APTR;     // appliprog can specify that the Gadget be rendered as either as Border
                            // or an Image.  This variable points to which (or equals nil if there's nothing to be rendered about this Gadget)
    SelectRender: APTR;     // appliprog can specify "highlighted" imagery rather than algorithmic this can point to either Border or Image data
    GadgetText: PIntuiText; // text for this gadget

    MutualExclude: LongInt; // obsolete

    SpecialInfo: APTR;      // pointer to a structure of special data required by Proportional, String and LongInt Gadgets
    GadgetID: Word;         // user-definable ID field
    UserData: APTR;         // ptr to general purpose User data (ignored by In)
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

    // These fields only exist if GFLG_EXTENDED is set
    MoreFlags: LongWord;     // see GMORE_* flags below
    BoundsLeftEdge,          // Bounding extent for gadget, valid
    BoundsTopEdge,           // only if GMORE_BOUNDS is set.  The
    BoundsWidth,             // GFLG_RELxxx flags affect these
    BoundsHeight: SmallInt;  // coordinates as well.
  end;


CONST
// Gadget.Flags values
  // combinations in these bits describe the highlight technique to be used
  GFLG_GADGHIGHBITS  = $0003;
  GFLG_GADGHCOMP     = $0000; // Complement the select box
  GFLG_GADGHBOX      = $0001; // Draw a box around the image
  GFLG_GADGHIMAGE    = $0002; // Blast in this alternate image
  GFLG_GADGHNONE     = $0003; // don't highlight }
  GFLG_GADGIMAGE     = $0004; // set IF GadgetRender AND SelectRender point to an Image structure, clear if they point to Border structures

  { combinations in these next two bits specify to which corner the gadget's
    Left & Top coordinates are relative.  If relative to Top/Left,
    these are "normal" coordinates (everything is relative to something in
    this universe).
    Gadget positions and dimensions are relative to the window or
    requester which contains the gadget}
  GFLG_RELBOTTOM = $0008; // vert. pos. is relative to bottom edge
  GFLG_RELRIGHT  = $0010; // horiz. pos. is relative to right edge
  GFLG_RELWIDTH  = $0020; // width is relative to req/window
  GFLG_RELHEIGHT = $0040; // height is relative to req/window

  { New for V39: GFLG_RELSPECIAL allows custom gadget implementors to
    make gadgets whose position and size depend in an arbitrary way
    on their window's dimensions.  The GM_LAYOUT method will be invoked
    for such a gadget (or any other GREL_xxx gadget) at suitable times,
    such as when the window opens or the window's size changes. }
  GFLG_RELSPECIAL  = $4000; // custom gadget has special relativity. Gadget box values are absolutes, but can be changed via the GM_LAYOUT method.
  GFLG_SELECTED    = $0080; // you may initialize AND look at this
  { the GFLG_DISABLED flag is initialized by you and later set by Intuition
    according to your calls to On/OffGadget().  It specifies whether or not
    this Gadget is currently disabled from being selected }
  GFLG_DISABLED    = $0100;

  { These flags specify the type of text field that Gadget.GadgetText
    points to.  In all normal (pre-V36) gadgets which you initialize
    this field should always be zero.  Some types of gadget objects
    created from classes will use these fields to keep track of
    types of labels/contents that different from IntuiText, but are
    stashed in GadgetText. }
  GFLG_LABELMASK   = $3000;
  GFLG_LABELITEXT  = $0000; // GadgetText points to IntuiText
  GFLG_LABELSTRING = $1000; // GadgetText points to (PChar)
  GFLG_LABELIMAGE  = $2000; // GadgetText points to Image (object)

  // New for V37: GFLG_TABCYCLE
  GFLG_TABCYCLE    = $0200;  // (string OR custom) gadget participates in cycling activation with Tab or Shift-Tab
  { New for V37: GFLG_STRINGEXTEND.  We discovered that V34 doesn't properly
    ignore the value we had chosen for the Gadget->Activation flag
    GACT_STRINGEXTEND.  NEVER SET THAT FLAG WHEN RUNNING UNDER V34.
    The Gadget->Flags bit GFLG_STRINGEXTEND is provided as a synonym which is
    safe under V34, and equivalent to GACT_STRINGEXTEND under V37.
    (Note that the two flags are not numerically equal)}
  GFLG_STRINGEXTEND = $0400; // this String Gadget has StringExtend
  { New for V39: GFLG_IMAGEDISABLE.  This flag is automatically set if
    the custom image of this gadget knows how to do disabled rendering
    (more specifically, if its IA_SupportsDisable attribute is TRUE).
    Intuition uses this to defer the ghosting to the image-class,
    instead of doing it itself (the old compatible way).
    Do not set this flag yourself - Intuition will do it for you.}
  GFLG_IMAGEDISABLE = $0800;  // Gadget's image knows how to do disabled rendering

  { New for V39:  If set, this bit means that the Gadget is actually
    a struct ExtGadget, with new fields and flags.  All V39 boopsi
    gadgets are ExtGadgets.  Never ever attempt to read the extended
    fields of a gadget if this flag is not set. }
  GFLG_EXTENDED    = $8000;  // Gadget is extended

// ---  Gadget.Activation flag values
  { Set GACT_RELVERIFY if you want to verify that the pointer was still over
    the gadget when the select button was released.  Will cause
    an IDCMP_GADGETUP message to be sent if so.}
  GACT_RELVERIFY    = $0001;
  { the flag GACT_IMMEDIATE, when set, informs the caller that the gadget
     was activated when it was activated.  This flag works in conjunction with
     the GACT_RELVERIFY flag }
  GACT_IMMEDIATE    = $0002;
  { the flag GACT_ENDGADGET, when set, tells the system that this gadget,
    when selected, causes the Requester to be ended.  Requesters
    that are ended are erased and unlinked from the system.}
  GACT_ENDGADGET    = $0004;
  { the GACT_FOLLOWMOUSE flag, when set, specifies that you want to receive
    reports on mouse movements while this gadget is active.
    You probably want to set the GACT_IMMEDIATE flag when using
    GACT_FOLLOWMOUSE, since that's the only reasonable way you have of
    learning why Intuition is suddenly sending you a stream of mouse
    movement events.  If you don't set GACT_RELVERIFY, you'll get at
    least one Mouse Position event. }
  GACT_FOLLOWMOUSE = $0008;

  { if any of the BORDER flags are set in a Gadget that's included in the
    Gadget list when a Window is opened, the corresponding Border will
    be adjusted to make room for the Gadget}
  GACT_RIGHTBORDER  = $0010;
  GACT_LEFTBORDER   = $0020;
  GACT_TOPBORDER    = $0040;
  GACT_BOTTOMBORDER = $0080;
  GACT_BORDERSNIFF  = $8000; // neither set nor rely on this bit

  GACT_TOGGLESELECT = $0100; // this bit for toggle-select mode
  GACT_BOOLEXTEND   = $2000; // this Boolean Gadget has a BoolInfo

  // should properly be in StringInfo, but aren't
  GACT_STRINGLEFT   = $0000; // NOTE WELL: that this has value zero
  GACT_STRINGCENTER = $0200;
  GACT_STRINGRIGHT  = $0400;
  GACT_LONGINT      = $0800; // this String Gadget is for Long Ints
  GACT_ALTKEYMAP    = $1000; // this String has an alternate keymap
  GACT_STRINGEXTEND = $2000; // this String Gadget has StringExtend
  GACT_ACTIVEGADGET = $4000; // this gadget is "active".  This flag is maintained by Intuition, and you
                             // cannot count on its value persisting while you do something on your program's
                             // task.  It can only be trusted by people implementing custom gadgets
  // note $8000 is used above (GACT_BORDERSNIFF) all Activation flags defined

// --- GADGET TYPES
  { These are the Gadget Type definitions for the variable GadgetType
    gadget number type MUST start from one.  NO TYPES OF ZERO ALLOWED.
    first comes the mask for Gadget flags reserved for Gadget typing}
  GTYP_GADGETTYPE = $FC00; // all Gadget Global Type flags (padded)
  GTYP_SYSGADGET  = $8000; // 1 = Allocated by the system, 0 = by app.
  GTYP_SCRGADGET  = $4000; // 1 = ScreenGadget, 0 = WindowGadget
  GTYP_GZZGADGET  = $2000; // 1 = for WFLG_GIMMEZEROZERO borders
  GTYP_REQGADGET  = $1000; // 1 = this is a Requester Gadget
  { system gadgets }
  GTYP_SIZING     = $0010;
  GTYP_WDRAGGING  = $0020;
  GTYP_SDRAGGING  = $0030;
  GTYP_WUPFRONT   = $0040;
  GTYP_SUPFRONT   = $0050;
  GTYP_WDOWNBACK  = $0060;
  GTYP_SDOWNBACK  = $0070;
  GTYP_CLOSE      = $0080;
  // V50 SYSGADGET types
  GTYP_WDRAGGING2 = $0090; // GTYP_WDRAGGING2 gadgets only send IDCMP_GADGETDOWN, this one also sends IDCMP_GADGETUP

  GTYP_WDEPTH = GTYP_WUPFRONT;
  GTYP_SDEPTH = GTYP_SUPFRONT;
  GTYP_WZOOM = GTYP_WDOWNBACK;
  GTYP_SUNUSED = GTYP_SDOWNBACK;

  GTYP_GTYPEMASK = $0007; // GTYP_GTYPEMASK is a mask you can apply to tell what class of gadget this is.  The possible classes follow.

  // application gadgets
  GTYP_BOOLGADGET   = $0001;
  GTYP_GADGET0002   = $0002;
  GTYP_PROPGADGET   = $0003;
  GTYP_STRGADGET    = $0004;
  GTYP_CUSTOMGADGET = $0005;

{ New for V39.  Gadgets which have the GFLG_EXTENDED flag set are
  actually ExtGadgets, which have more flags.  The GMORE_xxx
  identifiers describe those flags.  For GMORE_SCROLLRASTER, see
  important information in the ScrollWindowRaster() autodoc.
  NB: GMORE_SCROLLRASTER must be set before the gadget is
  added to a window.}
 GMORE_BOUNDS       = $00000001; // ExtGadget has valid Bounds
 GMORE_GADGETHELP   = $00000002; // This gadget responds to gadget help
 GMORE_SCROLLRASTER = $00000004; // This (custom) gadget uses ScrollRaster

// BoolInfo
//   This is the special data needed by an Extended Boolean Gadget Typically this structure will be pointed to by the Gadget field SpecialInfo
Type
  PBoolInfo = ^TBoolInfo;
  TBoolInfo = record
    Flags: Word; // defined below
    Mask: PWord; // bit mask for highlighting and selecting mask must follow the same rules as an Image
                 // plane.  It's width and height are determined by the width and height of the gadget's select box. (i.e. Gadget.Width and .Height).
    Reserved: LongWord; // set to 0
  end;

const
// set BoolInfo.Flags to this flag bit. in the future, additional bits might mean more stuff hanging off of BoolInfo.Reserved.
  BOOLMASK = $0001; // extension is for masked gadget

// PropInfo
// this is the special data required by the proportional Gadget typically, this data will be pointed to by the Gadget variable SpecialInfo
type
  PPropInfo = ^TPropInfo;
  TPropInfo = record
    Flags: Word; // general purpose flag bits (see defines below)

  { You initialize the Pot variables before the Gadget is added to
  * the system.  Then you can look here for the current settings
  * any time, even while User is playing with this Gadget.  To
  * adjust these after the Gadget is added to the System, use
  * ModifyProp();  The Pots are the actual proportional settings,
  * where a value of zero means zero and a value of MAXPOT means
  * that the Gadget is set to its maximum setting.
  }

    HorizPot: Word; // 16-bit FixedPoint horizontal quantity percentage
    VertPot: Word;  // 16-bit FixedPoint vertical quantity percentage

  { the 16-bit FixedPoint Body variables describe what percentage of
  * the entire body of stuff referred to by this Gadget is actually
  * shown at one time.  This is used with the AUTOKNOB routines,
  * to adjust the size of the AUTOKNOB according to how much of
  * the data can be seen.  This is also used to decide how far
  * to advance the Pots when User hits the Container of the Gadget.
  * For instance, if you were controlling the display of a 5-line
  * Window of text with this Gadget, and there was a total of 15
  * lines that could be displayed, you would set the VertBody value to
  *     (MAXBODY / (TotalLines / DisplayLines)) = MAXBODY / 3.
  * Therefore, the AUTOKNOB would fill 1/3 of the container, and
  * if User hits the Cotainer outside of the knob, the pot would
  * advance 1/3 (plus or minus) If there's no body to show, or
  * the total amount of displayable info is less than the display area,
  * set the Body variables to the MAX.  To adjust these after the
  * Gadget is added to the System, use ModifyProp();
  }

    HorizBody: Word; // horizontal Body
    VertBody: Word;  // vertical Body

  // these are the variables that Intuition sets and maintains

    CWidth: Word;     // Container width (with any relativity absoluted)
    CHeight: Word;    // Container height (with any relativity absoluted)
    HPotRes,
    VPotRes: Word;    // pot increments
    LeftBorder: Word; // Container borders
    TopBorder: Word;  // Container borders
  end;

const
// --- FLAG BITS
  AUTOKNOB       = $0001;  // this flag set: gimme that old auto-knob
  FREEHORIZ      = $0002;  // IF set, the knob can move horizontally
  FREEVERT       = $0004;  // IF set, the knob can move vertically
  PROPBORDERLESS = $0008;  // IF set, no border will be rendered
  KNOBHIT        = $0100;  // set when this Knob is hit
  PROPNEWLOOK    = $0010;  // set this IF you want to get the new

  KNOBHMIN = 6;     // minimum horizontal size of the Knob
  KNOBVMIN = 4;     // minimum vertical size of the Knob
  MAXBODY  = $FFFF; // maximum body value
  MAXPOT   = $FFFF; // maximum pot value

// StringInfo
//   this is the special data required by the string Gadget typically, this data will be pointed to by the Gadget variable SpecialInfo
type
  PStringInfo = ^TStringInfo;
  TStringInfo = record
    // you initialize these variables, and then Intuition maintains them
    Buffer: PChar;       // the buffer containing the start and final string
    UndoBuffer: PChar;   // optional buffer for undoing current entry
    BufferPos: SmallInt; // character position in Buffer
    MaxChars: SmallInt;  // max number of chars in Buffer (including #0)
    DispPos: SmallInt;   // Buffer position of first displayed character

    // Intuition initializes and maintains these variables for you

    UndoPos: SmallInt;   // character position in the undo buffer
    NumChars: SmallInt;  // number of characters currently in Buffer
    DispCount: SmallInt; // number of whole characters visible in Container
    CLeft,
    CTop: SmallInt;      // topleft offset of the container

    { you can initialize this variable before the gadget is submitted to
      Intuition, and then examine it later to discover what Longint
      the user has entered (if the user never plays with the gadget,
      the value will be unchanged from your initial setting) }
    Extension: Pointer;
    _LongInt: Longint;

    { If you want this Gadget to use your own Console keymapping, you
      set the ALTKEYMAP bit in the Activation flags of the Gadget, and then
      set this variable to point to your keymap.  If you don't set the
      ALTKEYMAP, you'll get the standard ASCII keymapping.}
    AltKeyMap: Pointer;
  end;

const
  // FLAGS SET BY THE APPLIPROG
  POINTREL            = $0001; // if POINTREL set, TopLeft is relative to pointer
  PREDRAWN            = $0002; // if ReqBMap points to predrawn Requester imagery
  NOISYREQ            = $0004; // if you don't want requester to filter input
  SIMPLEREQ           = $0010; // to use SIMPLEREFRESH layer (recommended)
  // New for V36
  USEREQIMAGE         = $0020; //  render linked list ReqImage after BackFill but before gadgets and text
  NOREQBACKFILL       = $0040; // don't bother filling requester with Requester.BackFill pen
  // FLAGS SET BY INTUITION
  REQOFFWINDOW        = $1000; // part of one of the Gadgets was offwindow
  REQACTIVE           = $2000; // this requester is active
  SYSREQUEST          = $4000; // this requester caused by system
  DEFERREFRESH        = $8000; // this Requester stops a Refresh broadcast

// Requester
type
  PImage = ^TImage;
  PScreen = ^TScreen;
  PWindow = ^TWindow;
  PTabletData = ^TTabletData;
  // Requesters The following struct is used for standard intuition requesters (not to be mixed up with asl or easy requesters). See intuition.library/Request() for more information.
  PRequester = ^TRequester;
  TRequester = record
    // the ClipRect and BitMap and used for rendering the requester
    OlderRequest: PRequester;
    LeftEdge,
    TopEdge: SmallInt;        // dimensions of the entire box
    Width,
    Height: SmallInt;         // dimensions of the entire box
    RelLeft,
    RelTop: SmallInt;         // for Pointer relativity offsets

    ReqGadget: PGadget;       // First gadget of the requester
    ReqBorder: PBorder;       // First border of the requester
    ReqText: PIntuiText;      // First intuitext of the requester

    Flags: Word;              // see definitions below
    BackFill: Byte;           // pen number for back-plane fill before draws

    ReqLayer: PLayer;         // Layer in place of clip rect

    ReqPad1: array [0..31] of Byte;  // Private

    { If the BitMap plane pointers are non-zero, this tells the system
      that the image comes pre-drawn (if the appliprog wants to define
      it's own box, in any shape or size it wants!);  this is OK by
      Intuition as long as there's a good correspondence between
      the image and the specified Gadgets}
    ImageBMap: PBitMap;    // you may use this to fill the requester with your own image
    RWindow: PWindow;      // window, which the requester belongs to
    ReqImage: PImage;      // corresponds to USEREQIMAGE (see below)
    ReqPad2: array[0..31] of ShortInt; // PRIVATE
  end;

{ The Timage.PlanePick and PlaneOnOff variables work much the same way as the equivalent GELS Bob variables.  It's a space-saving
  mechanism for image data.  Rather than defining the image data for every plane of the RastPort, you need define data only
  for the planes that are not entirely zero or one.  As you define your Imagery, you will often find that most of the planes
  ARE just as color selectors.  For instance, if you're designing a two-color Gadget to use colors two and three, and the Gadget
  will reside in a five-plane display, bit plane zero of your imagery would be all ones, bit plane one would have data that
  describes the imagery, and bit planes two through four would be all zeroes.  Using these flags allows you to avoid wasting all
  that memory in this way:  first, you specify which planes you want your data to appear in using the PlanePick variable.  For
  each bit set in the variable, the next "plane" of your image data is blitted to the display.  For each bit clear in this
  variable, the corresponding bit in PlaneOnOff is examined. If that bit is clear, a "plane" of zeroes will be used.
  If the bit is set, ones will go out instead.  So, for our example:
    Gadget.PlanePick = $02;
    Gadget.PlaneOnOff = $01;
  Note that this also allows for generic Gadgets, like the System Gadgets, which will work in any number of bit planes.
  Note also that if you want an Image that is only a filled rectangle, you can get this by setting PlanePick to zero
  (pick no planes of data) and set PlaneOnOff to describe the pen color of the rectangle.}

// This is a brief image structure for very simple transfers of image data to a RastPort
  TImage = record
    LeftEdge: SmallInt; // starting offset relative to some origin
    TopEdge: SmallInt;  // starting offsets relative to some origin
    Width: SmallInt;    // pixel size (though data is Word-aligned)
    Height: SmallInt;
    Depth: SmallInt;    // pixel sizes
    ImageData: PWord;   // pointer to the actual Word-aligned bits
    PlanePick,
    PlaneOnOff: Byte;
    NextImage: PImage;   // if not nil, Intuition presumes that it points to another Image structure with another Image to be rendered
  end;

{ If your window sets WA_TabletMessages to TRUE, then it will receive extended IntuiMessages (struct ExtIntuiMessage) whose eim_TabletData
  field points at a TabletData structure.  This structure contains additional information about the input event.}
  TTabletData = record
    td_XFraction,         // Sub-pixel position of tablet, in screen coordinates, scaled to fill a Word fraction:
    td_YFraction: Word;
    td_TabletX,
    td_TabletY: LongWord; // Current tablet coordinates along each axis
    td_RangeX,
    td_RangeY: LongWord;  // Tablet range along each axis.  For example, if td_TabletX can take values 0-999, td_RangeX should be 1000.
    td_TagList: PTagItem; // Pointer to tag-list of additional tablet attributes. TABLETA_*
  end;

{ If a tablet driver supplies a hook for ient_CallBack, it will be invoked in the standard hook manner.  A0 will point to the Hook
  itself, A2 will point to the InputEvent that was sent, and A1 will point to a TabletHookData structure.  The InputEvent's
  ie_EventAddress field points at the IENewTablet structure that the driver supplied.
  Based on the thd_Screen, thd_Width, and thd_Height fields, the driver should scale the ient_TabletX and ient_TabletY fields and store the
  result in ient_ScaledX, ient_ScaledY, ient_ScaledXFraction, and ient_ScaledYFraction.
  The tablet hook must currently return NULL.  This is the only acceptable return-value.}
  PTabletHookData = ^TTabletHookData;
  TTabletHookData = record
    thd_Screen: PScreen;        // Pointer to the active screen: Note: if there are no open screens, thd_Screen will be nil.
    thd_Width,                  // thd_Width and thd_Height will then describe an NTSC 64$400 screen.  Please scale accordingly.
    thd_Height: LongWord;       // The width and height (measured in pixels of the active screen)that your are to scale to:
    thd_ScreenChanged: LongInt; // Non-zero if the screen or something about the screen  changed since the last time you were invoked:
  end;

  PIntuiMessage = ^TIntuiMessage;
  TIntuiMessage = record
    ExecMessage: TMessage;
    IClass: LongWord;      // the Class bits correspond directly with the IDCMP Flags, except for the special bit LONELYMESSAGE (defined below)
    Code: Word;            // the Code field is for special values like MENU number
    Qualifier: Word;       // the Qualifier field is a copy of the current InputEvent's Qualifier
    IAddress: APTR;        // IAddress contains particular addresses for Intuition functions, like the pointer to the Gadget or the Screen

    MouseX,            // when getting mouse movement reports, any event you get will have the the mouse coordinates in these variables.
    MouseY: SmallInt;  // the coordinates are relative to the upper-left corner of your Window (GIMMEZEROZERO notwithstanding)
    Seconds,           // the time values are copies of the current system clock time.
    Micros: LongWord;  // Micros are in units of microseconds, Seconds in seconds.

    IDCMPWindow: PWindow; // the IDCMPWindow variable will always have the Pointer of the Window of this IDCMP
    SpecialLink: PIntuiMessage; // system-use variable
  end;

{ All IntuiMessages are now slightly extended.  The ExtIntuiMessage
  structure has an additional field for tablet data, which is usually
  nil.  If a tablet driver which is sending IESUBCLASS_NEWTABLET
  events is installed in the system, windows with the WA_TabletMessages
  property set will find that eim_TabletData points to the TabletData
  structure.  Applications must first check that this field is non-NULL;
  it will be nil for certain kinds of message, including mouse activity
  generated from other than the tablet (i.e. the keyboard equivalents
  or the mouse itself).
  NOTE: This structure is subject to grow in the future.  Making
  assumptions about its size is A BAD IDEA.}

  PExtIntuiMessage = ^TExtIntuiMessage;
  TExtIntuiMessage = record
    eim_IntuiMessage: TIntuiMessage;
    eim_TabletData: PTabletData;
  end;

  // A data structure common in Intuition processing
  PIBox = ^TIBox;
  TIBox = record
    Left,
    Top,
    Width,
    Height: SmallInt;
  end;


// Window
  TWindow = record
    NextWindow: PWindow; // for the linked list in a screen

    LeftEdge,
    TopEdge: SmallInt;   // screen dimensions of window
    Width,
    Height: SmallInt;    // screen dimensions of window

    MouseY,
    MouseX: SmallInt;    // relative to upper-left of window

    MinWidth,
    MinHeight: SmallInt; // minimum sizes
    MaxWidth,
    MaxHeight: SmallInt; // maximum sizes

    Flags: LongWord;     // see below for defines

    MenuStrip: PMenu;    // the strip of Menu headers

    Title: PChar;        // the title text for this window

    FirstRequest: PRequester; // all active Requesters

    DMRequest: PRequester;    // double-click Requester

    ReqCount: SmallInt;       // count of reqs blocking Window

    WScreen: PScreen;         // this Window's Screen
    RPort: PRastPort;         // this Window's very own RastPort

    { the border variables describe the window border.   If you specify
      GIMMEZEROZERO when you open the window, then the upper-left of the
      ClipRect for this window will be upper-left of the BitMap (with correct
      offsets when in SuperBitMap mode; you MUST select GIMMEZEROZERO when
      using SuperBitMap).  If you don't specify ZeroZero, then you save
      memory (no allocation of RastPort, Layer, ClipRect and associated
      Bitmaps), but you also must offset all your writes by BorderTop,
      BorderLeft and do your own mini-clipping to prevent writing over the
      system gadgets}
    BorderLeft,
    BorderTop,
    BorderRight,
    BorderBottom: ShortInt;
    BorderRPort: PRastPort;

    { You supply a linked-list of Gadgets for your Window.
      This list DOES NOT include system gadgets.  You get the standard
      window system gadgets by setting flag-bits in the variable Flags (see
      the bit definitions below)}
    FirstGadget: PGadget;
    // these are for opening/closing the windows
    Parent,
    Descendant: PWindow;
    // sprite data information for your own Pointer set these AFTER you Open the Window by calling SetPointer()
    _Pointer: Pointer;   // sprite data
    PtrHeight: ShortInt; // sprite height (not including sprite padding)
    PtrWidth: ShortInt;  // sprite width (must be less than or equal to 16)
    XOffset,
    YOffset: ShortInt;   // sprite offsets
    // the IDCMP Flags and User's and Intuition's Message Ports
    IDCMPFlags: LongWord; // User-selected flags
    UserPort,
    WindowPort: PMsgPort;
    MessageKey: PIntuiMessage;

    DetailPen,
    BlockPen: Byte; // for bar/border/gadget rendering
    { the CheckMark is a pointer to the imagery that will be used when
     rendering MenuItems of this Window that want to be checkmarked
     if this is equal to NULL, you'll get the default imagery}
    CheckMark: PImage;
    ScreenTitle: PChar; // if non-nil, Screen title when Window is active
    { These variables have the mouse coordinates relative to the
      inner-Window of GIMMEZEROZERO Windows.  This is compared with the
      MouseX and MouseY variables, which contain the mouse coordinates
      relative to the upper-left corner of the Window, GIMMEZEROZERO
      notwithstanding}
    GZZMouseX: SmallInt;
    GZZMouseY: SmallInt;
    // these variables contain the width and height of the inner-Window of GIMMEZEROZERO Windows
    GZZWidth: SmallInt;
    GZZHeight: SmallInt;

    ExtData: Pointer;

    UserData: Pointer; // general-purpose pointer to User data extension

    WLayer: PLayer;

    IFont: PTextFont;
    MoreFlags: LongWord;
    {**** Data beyond this point are Intuition Private.  DO NOT USE ****}
  end;

  TScreen = record
    NextScreen: PScreen;  // pointer to the next screen, can only be accessed between LockIBase()/UnlockIBase()
    FirstWindow: PWindow; // pointer to the first opened window on screen, can only be accessed between LockIBase()/UnlockIBase()

    LeftEdge,
    TopEdge: SmallInt;    // used for screen movement if the width/height is greater that display width/height, can be modified with MoveScreen() function
    Width,
    Height: SmallInt;     // dimensions of the screen

    MouseY,
    MouseX: SmallInt;     // mouse position on screen

    Flags: Word;          // one of the flags defined below
    Title: PChar;         // null-terminated Title text
    DefaultTitle: PChar;  // for Windows without ScreenTitle

    // BarHeight can still be read if you need to know the actual size of the bar, even if it's not
    // visible due to user configuration. Remember to add 1 to get real height
    BarHeight,
    BarVBorder,
    BarHBorder,
    MenuVBorder,
    MenuHBorder: ShortInt;
    WBorTop,
    WBorLeft,
    WBorRight,
    WBorBottom: ShortInt;

    Font: PTextAttr;       // default font for this screen

    // the display data structures for this Screen (note the prefix S)
    ViewPort: TViewPort;    // structure used to describe screen's display mode & state, should not be accessed
    RastPort: TRastPort;    // main RastPort structure of the screen, should never be used unless the screen is a custom screen with no windows on it.
    BitMap: TBitMap;        // OBSOLETE, always use screen^.RastPort.BitMap
    LayerInfo: TLayer_Info; // main layer structure of the screen, should only be used for layer lock calls

    // pointer to the first gadget attached to the screen, a private field
    FirstGadget: PGadget;

    DetailPen,
    BlockPen: Byte;         // for bar/border/gadget rendering, obsolete

  // the following variable(s) are maintained by Intuition to support the DisplayBeep() color flashing technique
    SaveColor0: Word;       // obsolete
    BarLayer: PLayer;       // This layer is for the Screen and Menu bars
    ExtData: Pointer;
    UserData: Pointer;
  { general-purpose pointer to User data extension
    **** Data below this point are SYSTEM PRIVATE ****}
  end;

{ New for V39, Intuition supports the IESUBCLASS_NEWTABLET subclass
 * of the IECLASS_NEWPOINTERPOS event.  The ie_EventAddress of such
 * an event points to a TabletData structure (see below).
 *
 * The TabletData structure contains certain elements including a taglist.
 * The taglist can be used for special tablet parameters.  A tablet driver
 * should include only those tag-items the tablet supports.  An application
 * can listen for any tag-items that interest it.  Note: an application
 * must set the WA_TabletMessages attribute to TRUE to receive this
 * extended information in its IntuiMessages.
 *
 * The definitions given here MUST be followed.  Pay careful attention
 * to normalization and the interpretation of signs.
 *
 * TABLETA_TabletZ:  the current value of the tablet in the Z direction.
 * This unsigned value should typically be in the natural units of the
 * tablet.  You should also provide TABLETA_RangeZ.
 *
 * TABLETA_RangeZ:  the maximum value of the tablet in the Z direction.
 * Normally specified along with TABLETA_TabletZ, this allows the
 * application to scale the actual Z value across its range.
 *
 * TABLETA_AngleX:  the angle of rotation or tilt about the X-axis.  This
 * number should be normalized to fill a signed long Longint.  Positive
 * values imply a clockwise rotation about the X-axis when viewing
 * from +X towards the origin.
 *
 * TABLETA_AngleY:  the angle of rotation or tilt about the Y-axis.  This
 * number should be normalized to fill a signed long Longint.  Positive
 * values imply a clockwise rotation about the Y-axis when viewing
 * from +Y towards the origin.
 *
 * TABLETA_AngleZ:  the angle of rotation or tilt about the Z axis.  This
 * number should be normalized to fill a signed long Longint.  Positive
 * values imply a clockwise rotation about the Z-axis when viewing
 * from +Z towards the origin.
 *
 *      Note: a stylus that supports tilt should use the TABLETA_AngleX
 *      and TABLETA_AngleY attributes.  Tilting the stylus so the tip
 *      points towards increasing or decreasing X is actually a rotation
 *      around the Y-axis.  Thus, if the stylus tip points towards
 *      positive X, then that tilt is represented as a negative
 *      TABLETA_AngleY.  Likewise, if the stylus tip points towards
 *      positive Y, that tilt is represented by positive TABLETA_AngleX.
 *
 * TABLETA_Pressure:  the pressure reading of the stylus.  The pressure
 * should be normalized to fill a signed long Longint.  Typical devices
 * won't generate negative pressure, but the possibility is not precluded.
 * The pressure threshold which is considered to cause a button-click is
 * expected to be set in a Preferences program supplied by the tablet
 * vendor.  The tablet driver would send IECODE_LBUTTON-type events as
 * the pressure crossed that threshold.
 *
 * TABLETA_ButtonBits:  ti_Data is a long Longint whose bits are to
 * be interpreted at the state of the first 32 buttons of the tablet.
 *
 * TABLETA_InProximity:  ti_Data is a boolean.  For tablets that support
 * proximity, they should send the (TABLETA_InProximity,FALSE) tag item
 * when the stylus is out of proximity.  One possible use we can forsee
 * is a mouse-blanking commodity which keys off this to blank the
 * mouse.  When this tag is absent, the stylus is assumed to be
 * in proximity.
 *
 * TABLETA_ResolutionX:  ti_Data is an unsigned long Longint which
 * is the x-axis resolution in dots per inch.
 *
 * TABLETA_ResolutionY:  ti_Data is an unsigned long Longint which
 * is the y-axis resolution in dots per inch.
 }

const
  TABLETA_Dummy       = TAG_USER + $3A000;
  TABLETA_TabletZ     = TABLETA_Dummy + 1;
  TABLETA_RangeZ      = TABLETA_Dummy + 2;
  TABLETA_AngleX      = TABLETA_Dummy + 3;
  TABLETA_AngleY      = TABLETA_Dummy + 4;
  TABLETA_AngleZ      = TABLETA_Dummy + 5;
  TABLETA_Pressure    = TABLETA_Dummy + 6;
  TABLETA_ButtonBits  = TABLETA_Dummy + 7;
  TABLETA_InProximity = TABLETA_Dummy + 8;
  TABLETA_ResolutionX = TABLETA_Dummy + 9;
  TABLETA_ResolutionY = TABLETA_Dummy + 10;

{ If your window sets WA_TabletMessages to TRUE, then it will receive
 * extended IntuiMessages (struct ExtIntuiMessage) whose eim_TabletData
 * field points at a TabletData structure.  This structure contains
 * additional information about the input event.
 }

const
// IDCMP Classes
//   Please refer to the Autodoc for OpenWindow() and to the Rom Kernel Manual for full details on the IDCMP classes.
  IDCMP_SIZEVERIFY     = $00000001;
  IDCMP_NEWSIZE        = $00000002;
  IDCMP_REFRESHWINDOW  = $00000004;
  IDCMP_MOUSEBUTTONS   = $00000008;
  IDCMP_MOUSEMOVE      = $00000010;
  IDCMP_GADGETDOWN     = $00000020;
  IDCMP_GADGETUP       = $00000040;
  IDCMP_REQSET         = $00000080;
  IDCMP_MENUPICK       = $00000100;
  IDCMP_CLOSEWINDOW    = $00000200;
  IDCMP_RAWKEY         = $00000400;
  IDCMP_REQVERIFY      = $00000800;
  IDCMP_REQCLEAR       = $00001000;
  IDCMP_MENUVERIFY     = $00002000;
  IDCMP_NEWPREFS       = $00004000;
  IDCMP_DISKINSERTED   = $00008000;
  IDCMP_DISKREMOVED    = $00010000;
  IDCMP_WBENCHMESSAGE  = $00020000; //  System use only
  IDCMP_ACTIVEWINDOW   = $00040000;
  IDCMP_INACTIVEWINDOW = $00080000;
  IDCMP_DELTAMOVE      = $00100000;
  IDCMP_VANILLAKEY     = $00200000;
  IDCMP_INTUITICKS     = $00400000;
  //  for notifications from "boopsi" gadgets
  IDCMP_IDCMPUPDATE    = $00800000; // new for V36
  // for getting help key report during menu session
  IDCMP_MENUHELP       = $01000000; // new for V36
  // for notification of any move/size/zoom/change window
  IDCMP_CHANGEWINDOW   = $02000000; // new for V36
  IDCMP_GADGETHELP     = $04000000; // new for V39
  IDCMP_MOUSEHOVER     = $08000000; // v50
  IDCMP_MOUSEOBJECTMUI = $40000000; // special idcmp message created by MUI */
  // the IDCMP Flags do not use this special bit, which is cleared when
  //  Intuition sends its special message to the Task, and set when Intuition
  // gets its Message back from the Task.  Therefore, I can check here to
  // find out fast whether or not this Message is available for me to send
  IDCMP_LONELYMESSAGE = $80000000;

// --- IDCMP Codes
  // This group of codes is for the IDCMP_CHANGEWINDOW message
  CWCODE_MOVESIZE = $0000; // Window was moved and/or sized
  CWCODE_DEPTH    = $0001; // Window was depth-arranged (new for V39)
  // This group of codes is for the IDCMP_MENUVERIFY function
  MENUHOT     = $0001; // IntuiWants verification OR MENUCANCEL
  MENUCANCEL  = $0002; // HOT Reply of this cancels Menu operation
  MENUWAITING = $0003; // Intuition simply wants a ReplyMsg() ASAP
  // These are internal tokens to represent state of verification attempts shown here as a clue.
  OKOK     = MENUHOT;    // guy didn't care
  OKABORT  = $0004;      // window rendered question moot
  OKCANCEL = MENUCANCEL; // window sent cancel reply
  // This group of codes is for the IDCMP_WBENCHMESSAGE messages
  WBENCHOPEN  = $0001;
  WBENCHCLOSE = $0002;
  // This group of codes is for the IDCMP_MOUSEHOVER messages
  HOVERSTART = $0001; // v50
  HOVERSTOP  = $0002; // v50

const
// --- Flags requested at OpenWindow() time by the application
  WFLG_SIZEGADGET  = $00000001; // include sizing system-gadget?
  WFLG_DRAGBAR     = $00000002; // include dragging system-gadget?
  WFLG_DEPTHGADGET = $00000004; // include depth arrangement gadget?
  WFLG_CLOSEGADGET = $00000008; // include close-box system-gadget?

  WFLG_SIZEBRIGHT  = $00000010; // size gadget uses right border
  WFLG_SIZEBBOTTOM = $00000020; // size gadget uses bottom border
// --- refresh modes
  // combinations of the WFLG_REFRESHBITS select the refresh type
  WFLG_REFRESHBITS    = $000000C0;
  WFLG_SMART_REFRESH  = $00000000;
  WFLG_SIMPLE_REFRESH = $00000040;
  WFLG_SUPER_BITMAP   = $00000080;
  WFLG_OTHER_REFRESH  = $000000C0;

  WFLG_BACKDROP      = $00000100; // this is a backdrop window
  WFLG_REPORTMOUSE   = $00000200; // to hear about every mouse move
  WFLG_GIMMEZEROZERO = $00000400; // a GimmeZeroZero window
  WFLG_BORDERLESS    = $00000800; // to get a Window sans border
  WFLG_ACTIVATE      = $00001000; // when Window opens, it's Active
// --- Other User Flags
  WFLG_RMBTRAP       = $00010000; // Catch RMB events for your own
  WFLG_NOCAREREFRESH = $00020000; // not to be bothered with REFRESH
// V36 new Flags which the programmer may specify in NewWindow.Flags
  WFLG_NW_EXTENDED   = $00040000; // extension data provided
// V39 new Flags which the programmer may specify in NewWindow.Flags
  WFLG_NEWLOOKMENUS  = $00200000; // window has NewLook menus
// These flags are set only by Intuition.  YOU MAY NOT SET THEM YOURSELF!
  WFLG_WINDOWACTIVE  = $00002000; // this window is the active one
  WFLG_INREQUEST     = $00004000; // this window is in request mode
  WFLG_MENUSTATE     = $00008000; // Window is active with Menus on
  WFLG_WINDOWREFRESH = $01000000; // Window is currently refreshing
  WFLG_WBENCHWINDOW  = $02000000; // WorkBench tool ONLY Window
  WFLG_WINDOWTICKED  = $04000000; // only one timer tick at a time
// --- V36 Flags to be set only by Intuition
  WFLG_VISITOR       = $08000000; // visitor window
  WFLG_ZOOMED        = $10000000; // identifies "zoom state"
  WFLG_HASZOOM       = $20000000; // windowhas a zoom gadget
// --- Other Window Values
  DEFAULTMOUSEQUEUE  = 5; // no more mouse messages

type
  PNewWindow = ^TNewWindow;
  TNewWindow = record
    LeftEdge,
    TopEdge: SmallInt;        // screen dimensions of window
    Width,
    Height: SmallInt;         // screen dimensions of window

    DetailPen,
    BlockPen: Byte;           // for bar/border/gadget rendering

    IDCMPFlags: LongWord;     // User-selected IDCMP flags
    Flags: LongWord;          // see Window struct for defines

    FirstGadget: PGadget; { You supply a linked-list of Gadgets for your Window.
                             This list DOES NOT include system Gadgets.  You get the standard
                             system Window Gadgets by setting flag-bits in the variable Flags (see
                             the bit definitions under the Window structure definition)}
    CheckMark: PImage;    { the CheckMark is a pointer to the imagery that will be used when
                             rendering MenuItems of this Window that want to be checkmarked
                             if this is equal to NULL, you'll get the default imagery}
    Title: PChar;         // the title text for this window
    Screen: PScreen;      { the Screen pointer is used only if you've defined a CUSTOMSCREEN and
                             want this Window to open in it.  If so, you pass the Pointer of the
                             Custom Screen structure in this variable.  Otherwise, this variable
                             is ignored and doesn't have to be initialized.}
    BitMap: PBitMap;      { SUPER_BITMAP Window?  If so, put the Pointer of your BitMap structure
                             in this variable.  If not, this variable is ignored and doesn't have
                             to be initialized}
    { the values describe the minimum and maximum sizes of your Windows.
      these matter only if you've chosen the WINDOWSIZING Gadget option,
      which means that you want to let the User to change the size of
      this Window.  You describe the minimum and maximum sizes that the
      Window can grow by setting these variables.  You can initialize
      any one these to zero, which will mean that you want to duplicate
      the setting for that dimension (if MinWidth == 0, MinWidth will be
      set to the opening Width of the Window).
      You can change these settings later using SetWindowLimits().
      If you haven't asked for a SIZING Gadget, you don't have to
      initialize any of these variables.}
    MinWidth,
    MinHeight: SmallInt;   // minimums
    MaxWidth,
    MaxHeight: Word;       // maximums

    WType: Word;           { the type variable describes the Screen in which you want this Window to
                              open.  The type value can either be CUSTOMSCREEN or one of the
                              system standard Screen Types such as WBENCHSCREEN.  See the
                              type definitions under the Screen structure}
  end;


{ The following structure is the future NewWindow.  Compatibility
  issues require that the size of NewWindow not change.
  Data in the common part (NewWindow) indicates the the extension
  fields are being used.
  NOTE WELL: This structure may be subject to future extension.
  Writing code depending on its size is not allowed.
 }
   PExtNewWindow = ^TExtNewWindow;
   TExtNewWindow = record
    LeftEdge,
    TopEdge: SmallInt;
    Width,
    Height: SmallInt;

    DetailPen,
    BlockPen: Byte;

    IDCMPFlags: LongWord;
    Flags: LongWord;

    FirstGadget: PGadget;
    CheckMark: PImage;
    Title: PChar;
    WScreen: PScreen;
    WBitMap: PBitMap;

    MinWidth,
    MinHeight: SmallInt;
    MaxWidth,
    MaxHeight: Word;

    { the type variable describes the Screen in which you want this Window to
      open.  The type value can either be CUSTOMSCREEN or one of the
      system standard Screen Types such as WBENCHSCREEN.  See the
      type definitions under the Screen structure.
      A new possible value for this field is PUBLICSCREEN, which
      defines the window as a 'visitor' window.  See below for
      additional information provided.}
    WType: Word;

    { if the NewWindow Flag value WFLG_NW_EXTENDED is set, then
       this field is assumed to point to an array ( or chain of arrays)
       of TagItem structures.  See also ExtNewScreen for another
       use of TagItems to pass optional data.
       see below for tag values and the corresponding data.}
    Extension: PTagItem;
  end;

{ The TagItem ID's (ti_Tag values) for OpenWindowTagList() follow.
  They are values in a TagItem array passed as extension/replacement
  values for the data in NewWindow.  OpenWindowTagList() can actually
  work well with a NULL NewWindow pointer.}
const
  WA_Dummy     =   (TAG_USER + 99); // $80000063
  // these tags simply override NewWindow parameters
  WA_Left         = WA_Dummy + 1;  // [ISG]
  WA_Top          = WA_Dummy + 2;  // [ISG]
  WA_Width        = WA_Dummy + 3;  // [ISG]
  WA_Height       = WA_Dummy + 4;  // [ISG]
  WA_DetailPen    = WA_Dummy + 5;  // [I..]
  WA_BlockPen     = WA_Dummy + 6;  // [I..]
  WA_IDCMP        = WA_Dummy + 7;  // [ISG]
  WA_Flags        = WA_Dummy + 8;  // [I..]
  WA_Gadgets      = WA_Dummy + 9;  // [I..]
  WA_Checkmark    = WA_Dummy + 10; // [I..]
  WA_Title        = WA_Dummy + 11; // [I..] means you don't have to call SetWindowTitles after you open your window
  WA_ScreenTitle  = WA_Dummy + 12; // [I..]
  WA_CustomScreen = WA_Dummy + 13; // [I..]
  WA_SuperBitMap  = WA_Dummy + 14; // [I..] also implies WFLG_SUPER_BITMAP property
  WA_MinWidth     = WA_Dummy + 15; // [ISG]
  WA_MinHeight    = WA_Dummy + 16; // [ISG]
  WA_MaxWidth     = WA_Dummy + 17; // [ISG]
  WA_MaxHeight    = WA_Dummy + 18; // [ISG]
  // The following are specifications for new features
  WA_InnerWidth   = WA_Dummy + 19; // [ISG]
  WA_InnerHeight  = WA_Dummy + 20; // [ISG]
    // You can specify the dimensions of the interior region of your window, independent of what
    // the border widths will be.  You probably want to also specify WA_AutoAdjust to allow
    // Intuition to move your window or even shrink it so that it is completely on screen.
  WA_PubScreenName = WA_Dummy + 21; // [I..] declares that you want the window to open as a visitor on the public screen whose name is pointed to by (PChar) ti_Data
  WA_PubScreen     = WA_Dummy + 22; // [I..] open as a visitor window on the public screen whose Pointer is in (struct Screen *) ti_Data.
    // To ensure that this screen remains open, you should either be the screen's owner, have a window open on the screen, or use LockPubScreen().
  WA_PubScreenFallBack = WA_Dummy + 23;  // [I..] A Boolean, specifies whether a visitor window should "fall back" to the default public screen
    // (or Workbench) if the named public screen isn't available
  WA_WindowName    = WA_Dummy + 24; // [I..] not implemented
  WA_Colors        = WA_Dummy + 25; // [I..]
    { a ColorSpec array for colors to be set when this window is active.  This is not implemented, and may not be, since the default
      values to restore would be hard to track. We'd like to at least support per-window colors for the mouse pointer sprite.}
  WA_Zoom          = WA_Dummy + 26; // [I.G]
    { ti_Data points to an array of four WORD's, the initial Left/Top/Width/Height values of the "alternate" zoom position/dimensions.
      It also specifies that you want a Zoom gadget for your window, whether or not you have a sizing gadget.}
  WA_MouseQueue    = WA_Dummy + 27; // [I..] ti_Data contains initial value for the mouse message backlog limit for this window.
  WA_BackFill      = WA_Dummy + 28; // [I..] unimplemented at present: provides a "backfill hook" for your window's layer.
  WA_RptQueue      = WA_Dummy + 29; // [I..] initial value of repeat key backlog limit

   // These Boolean tag items are alternatives to the NewWindow.Flags boolean flags with similar names.
  WA_SizeGadget    = WA_Dummy + 30; // [I..]
  WA_DragBar       = WA_Dummy + 31; // [I..]
  WA_DepthGadget   = WA_Dummy + 32; // [I..]
  WA_CloseGadget   = WA_Dummy + 33; // [I..]
  WA_Backdrop      = WA_Dummy + 34; // [I..]
  WA_ReportMouse   = WA_Dummy + 35; // [I..]
  WA_NoCareRefresh = WA_Dummy + 36; // [I..]
  WA_Borderless    = WA_Dummy + 37; // [I..]
  WA_Activate      = WA_Dummy + 38; // [I..]
  WA_RMBTrap       = WA_Dummy + 39; // [I..]
  WA_WBenchWindow  = WA_Dummy + 40; // [I..] PRIVATE!!
  WA_SimpleRefresh = WA_Dummy + 41; // [I..] only specify if TRUE
  WA_SmartRefresh  = WA_Dummy + 42; // [I..] only specify if TRUE
  WA_SizeBRight    = WA_Dummy + 43; // [I..]
  WA_SizeBBottom   = WA_Dummy + 44; // [I..]

  // New Boolean properties
  WA_AutoAdjust    = WA_Dummy + 45; // [I..] shift or squeeze the window's position and dimensions to fit it on screen.
  WA_GimmeZeroZero = WA_Dummy + 46; // [I..] equiv. to NewWindow.Flags WFLG_GIMMEZEROZERO

  // New for V37: WA_MenuHelp (ignored by V36) }
  WA_MenuHelp      = WA_Dummy + 47; // [I..] Enables IDCMP_MENUHELP:  Pressing HELP during menus will return IDCMP_MENUHELP message.
  // New for V39:  (ignored by V37 and earlier)
  WA_NewLookMenus  = WA_Dummy + 48; // [I..] Set to TRUE if you want NewLook menus
  WA_AmigaKey      = WA_Dummy + 49; // [I..] Pointer to image for Amiga-key equiv in menus
  WA_NotifyDepth   = WA_Dummy + 50; // [I..] Requests IDCMP_CHANGEWINDOW message when window is depth arranged (imsg^.Code = CWCODE_DEPTH)
  // WA_Dummy + 51 is obsolete
  WA_Pointer       = WA_Dummy + 52; // [IS.] Allows you to specify a custom pointer for your window.  ti_Data points to a pointer object you obtained via
    // "pointerclass". Nil signifies the default pointer. This tag may be passed to OpenWindowTags() or SetWindowPointer().
  WA_BusyPointer   = WA_Dummy + 53; // [ISG] ti_Data is boolean.  Set to TRUE to request the standard busy pointer.
    // This tag may be passed to OpenWindowTags() or SetWindowPointer().
  WA_PointerDelay  = WA_Dummy + 54; // [IS.] ti_Data is boolean.  Set to TRUE to request that the changing of the
    // pointer be slightly delayed.  The change will be called off if you call NewSetPointer() before the delay expires.  This allows
    // you to post a busy-pointer even if you think the busy-time may be very Word, without fear of a flashing pointer.
    // This tag may be passed to OpenWindowTags() or SetWindowPointer().
  WA_TabletMessages = WA_Dummy + 55; // [I..] ti_Data is a boolean.  Set to TRUE to request that tablet information be included
    // in IntuiMessages sent to your window. Requires that something (i.e. a tablet driver) feed IESUBCLASS_NEWTABLET InputEvents into
    // the system.  For a pointer to the TabletData, examine the ExtIntuiMessage->eim_TabletData field.  It is UNSAFE to check this field
    // when running on pre-V39 systems.  It's always safe to check this field under V39 and up, though it may be NULL.
  WA_HelpGroup      = WA_Dummy + 56; // [I..] When the active window has gadget help enabled, other windows of the same HelpGroup number
    // will also get GadgetHelp.  This allows GadgetHelp to work for multi-windowed applications. Use GetGroupID() to get an ID number.  Pass
    // this number as ti_Data to all your windows. See also the HelpControl() function.
  WA_HelpGroupWindow = WA_Dummy + 57; // [I..] When the active window has gadget help enabled, other windows of the same HelpGroup will also get
    // GadgetHelp.  This allows GadgetHelp to work for multi-windowed applications.  As an alternative to WA_HelpGroup, you can pass a pointer to any
    // other window of the same group to join its help group.  Defaults to NULL, which has no effect. See also the HelpControl() function.

// new OpenWindowTags tags (added some padding there)
// V50
  WA_ExtraTitlebarGadgets = WA_Dummy + 151; // [I..] LongWord flag field to indicate window titlebar gadgets your app wants to use. Those are built-in in sysiclass
    // and use intuition skins system. You'll be notified with normal IDCMP_GADGETUP when one of those gadgets get pressed.
  WA_ExtraGadgetsStartID  = WA_Dummy + 152; // [I..] All the extra gadgets have the Gadget ID's set to ETI_Dummy + gadget id value (defined below). Set this tag
    // if you want to change ETI_Dummy value for your gadgets. (for example when those ID's are already in use)
  // instead of using WA_ExtraTitlebarGadgets...
  WA_ExtraGadget_Iconify  = WA_Dummy + 153; // [IS.]
  WA_ExtraGadget_Lock     = WA_Dummy + 154; // [IS.]
  WA_ExtraGadget_MUI      = WA_Dummy + 155; // [IS.]
  WA_ExtraGadget_PopUp    = WA_Dummy + 156; // [IS.]
  WA_ExtraGadget_Snapshot = WA_Dummy + 157; // [IS.]
  WA_ExtraGadget_Jump     = WA_Dummy + 158; // [IS.]

  WA_SkinInfo             = WA_Dummy + 159; // [I..]

{ Intuition skins system usualy enchances window size when SIZEIMAGE width/height forces non-std border sizes.
  If your app already knows about the border sizes (GetSkinInfo) please add this tag to your OpenWindow call. This will switch
  off window size adjustment. ti_Data should point to SkinInfo struct allocated by GetSkinInfo.
  IMPORTANT: passing WA_SkinInfo tag to OpenWindowTags means that your app IS Skin compilant. Expect windows with non
   standard titlebar height, etc when you pass it (also with nil tag^.ti_Data!)}
  WA_TransparentRegion     = WA_Dummy + 160; // [I..]
  { Installs the provided region as a transparent region in window's layer. Best solution for fixed size windows. Setting WA_TransparentRegion clears
    previously set WA_TransparentRegionHook! For more information please refer to intuition/TransparencyControl() autodoc.}
  WA_TransparentRegionHook = WA_Dummy + 161; // [I..]
  { Installs the provided transparent region hook. The hook is called whenever window's layer needs updating (usualy on resize). The hook is called with
    window pointer in A2 and struct TransparencyMessage * in A1 registers. Setting this tag clears previously set WA_TransparentRegion!
    For more information please refer to intuition/TransparencyControl() autodoc.}
  WA_UserPort              = WA_Dummy + 162; // [I.G]
  { Please note that ModifyIDCMP(win,NULL) WILL NOT FREE userport when you use WA_UserPort!!! It will also NOT create a new msg
    port later!!! Keep in mind that ModifyIDCMP(win,NULL) will clear win->UserPort, but NOT free it - you need to store it and
    free manually! CloseWindow() doesn't free the port as well. IMPORTANT: remember that you need to reply all messages before the msg port was detached from all your windows!}
// V51
  WA_ToolbarWindow         = WA_Dummy + 163; // [I..]
  { Toolbar windows are windows that cannot be activated. They react fine on IDCMP_MOUSEBUTTONS, IDCMP_MOUSEMOVE, IDCMP_INTUITICKS, but
    only on those. The one and only supported intuition gadget is a GTYP_WDRAGGING(2) gadget, rest will be ignored. Toolbar windows are _always_ borderless.}
  WA_PointerType           = WA_Dummy + 164; // [ISG] Use one of intuition's built-in pointers in your window. There's basicly everything an app should need there - please avoid using custom pointers when possible.
  WA_FrontWindow           = WA_Dummy + 165; // [I..] Window stays always on front of other windows.
  WA_Parent                = WA_Dummy + 166; // [I.G]
  { PWindow. Makes the new window a child window of the one passed in ti_Data. Useful for popup windows - you can set child window position relatively to parent top/leftedge, child
    windows are also depth arranged with their parent windows, but NOT dragged (you need to care about this yourself).}
  WA_Opacity               = WA_Dummy + 168; // [ISG] LongWord. A 32bit opacity value. Use 0xFFFFFFFF for full visibility. NOTES: GZZ windows are not supported
  WA_HasAlpha              = WA_Dummy + 169; // [ISG] LongBool. Set to TRUE to make the window use the alpha data of it's buffer as window's opacity level NOTES: GZZ windows are not supported

  WA_SizeNumerator   = WA_Dummy + 171; // [ISG] LongWord. The four attributes define how the window should act when the user resizes it. This allows to define an aspect ratio in
  WA_SizeDenominator = WA_Dummy + 172; // [ISG] which the window will be resized. ExtraWidth/Height attributes specify the total size of the area which the aspect ratio resize
  WA_SizeExtraWidth  = WA_Dummy + 173; // [ISG] should not include (toolbars, window borders, etc). To disable aspect resizing, set the Numerator and Denominator to 1.
  WA_SizeExtraHeight = WA_Dummy + 174; // [ISG] Setting aspect resize doesn't resize the window or change it's size limits - you have to do it yourself.

  WA_HitLevel = WA_Dummy + 175; // [ISG]
  { LongWord. Defines the maximum opacity value to which the window will not be clickable. Unlike WA_Opacity, the value is in the
    0-255 range. WA_HitLevel,0 will make the window clickable if it's opacity is != 0. Use 255 to make a fully visible window ignore mouse clicks, hovering, etc}
  WA_ShadowTop    = WA_Dummy + 176; // [ISG] LongBool Some skins might support window shadows in certain display modes (depending on the hardware). In such case all windows with
  WA_ShadowLeft   = WA_Dummy + 177; // [ISG] a window border will be given a shadow. You can query if the shadow is on with those attributes (to disable your own fake shadows,
  WA_ShadowRight  = WA_Dummy + 178; // [ISG] etc). Set any of the tags above to false to disable a certain
  WA_ShadowBottom = WA_Dummy + 179; // [ISG] part of the shadow. Set any of the tags above to true to force shadows in a borderless window

  WA_VisibleOnMaximize = WA_Dummy + 180; // [ISG]
  { LongBool. When maximizing windows, intuition will take the windows with this tag on into the account and substract them from the
    area the maximized window will cover. Do note that intuition will not take windows that are not touching any screen border into the
    account. You should generally let user decide if he wants this functionality or not}

// window methods
  WM_Dummy = WA_Dummy;
  WM_OpenMenu     = WM_Dummy + 1; // Makes intuition open a menu for the window the method was called on. Does nothing if there is no menu or the system is busy. Will fail silently if the menu did not open */
  WM_ObtainEvents = WM_Dummy + 2; // LongBool. Obtains the events for the screen the window is at. This is an equivalent of installing your own, high priority
  {  inputhandler to capture input events. All IDCMPs will be sent to your window. Clicking outside of your window
     will not send events to other windows or cause their activation. Intuition menus will not be opened. The mouse pointer will use
     whatever your window has set, even if your window was not active when you obtained the pointer. Use this solely for the
     purpose of letting user pick a window, mark some area for snapshoting, etc. Will return FALSE if another window
     currently owns the events or your window already owns them. If the method returns true, you are the pointer owner and
     MUST match the call with WM_ReleaseEvents after you are done. For security reasons, if your application crashes or refuses
     to reply to IDCMP messages, the event ownership might be silently revoked. If a new window opens or gets activated
     while you have the ownership, it will be revoked and you will receive an IDCMP_INACTIVEWINDOW idcmp (even if your window
     is still active or was inactive before obtaining the pointer). The IDCMPS that your window will actually capture are:
     IDCMP_MOUSEMOVE, IDCMP_MOUSEBUTTON, IDCMP_RAWKEY, IDCMP_VANILLAKEY, IDCMP_INTUITICKS}
  WM_ReleaseEvents = WM_Dummy + 3; // Releases the events obtained with WM_ObtainEvents. You MUST always call it after WM_ObtainEvents returned TRUE. It is safe to call it in case your events ownership was revoked by the system

// HelpControl() flags: HC_GADGETHELP -
  HC_GADGETHELP  = 1; // Set this flag to enable Gadget-Help for one or more windows.

type
  // this structure is used for remembering what memory has been allocated to date by a given routine,
  // so that a premature abort or systematic exit can deallocate memory cleanly, easily, and completely
  PRemember = ^TRemember;
  TRemember = record
    NextRemember: PRemember;
    RememberSize: LongWord;
    Memory: PByte;
  end;

// How to tell Intuition about RGB values for a color table entry. }
  PColorSpec = ^TColorSpec;
  TColorSpec = record
    ColorIndex: SmallInt;   // -1 terminates an array of ColorSpec
    Red: Word;              // only the _bottom_ 4 bits recognized
    Green: Word;            // only the _bottom_ 4 bits recognized
    Blue: Word;             // only the _bottom_ 4 bits recognized
  end;

// Easy Requester Specification see also autodocs for EasyRequest and BuildEasyRequest NOTE: This structure may grow in size in the future
  PEasyStruct = ^TEasyStruct;
  TEasyStruct = record
    es_StructSize: LongWord; // should be sizeof (TEasyStruct) Note that this size may change, if you update the includes! Do not use absolute values as the size of pointers may vary on different platforms!
    es_Flags: LongWord;      // should be 0 for now
    es_Title: PChar;         // title of requester window
    es_TextFormat: PChar;    // 'printf' style formatting string
    es_GadgetFormat: PChar;  // Text of the gadgets, separated by |'s
  end;

const
// = MENU STUFF
  NOMENU   = $001F;
  NOITEM   = $003F;
  NOSUB    = $001F;
  MENUNULL = -1;

// = =RJ='s peculiarities

// these defines are for the COMMSEQ and CHECKIT menu stuff.  If CHECKIT,
// I'll use a generic Width (for all resolutions) for the CheckMark.
// If COMMSEQ, likewise I'll use this generic stuff
  CHECKWIDTH    = 19;
  COMMWIDTH     = 27;
  LOWCHECKWIDTH = 13;
  LOWCOMMWIDTH  = 16;

// these are the AlertNumber defines.  if you are calling DisplayAlert()
// the AlertNumber you supply must have the ALERT_TYPE bits set to one
// of these patterns
  ALERT_TYPE     = $80000000;
  RECOVERY_ALERT = $00000000;    { the system can recover from this }
  DEADEND_ALERT  = $80000000;    { no recovery possible, this is it }


{ When you're defining IntuiText for the Positive and Negative Gadgets
  created by a call to AutoRequest(), these defines will get you
  reasonable-looking text.  The only field without a define is the IText
  field; you decide what text goes with the Gadget}
  AUTOFRONTPEN        = 0;
  AUTOBACKPEN         = 1;
  AUTODRAWMODE        = JAM2;
  AUTOLEFTEDGE        = 6;
  AUTOTOPEDGE         = 3;
  AUTOITEXTFONT       = nil;
  AUTONEXTTEXT        = nil;

// --- RAWMOUSE Codes and Qualifiers (Console OR IDCMP)
  SELECTUP   = IECODE_LBUTTON or IECODE_UP_PREFIX;
  SELECTDOWN = IECODE_LBUTTON;
  MENUUP     = IECODE_RBUTTON or IECODE_UP_PREFIX;
  MENUDOWN   = IECODE_RBUTTON;
  MIDDLEUP   = IECODE_MBUTTON or IECODE_UP_PREFIX;
  MIDDLEDOWN = IECODE_MBUTTON;
  ALTLEFT    = IEQUALIFIER_LALT;
  ALTRIGHT   = IEQUALIFIER_RALT;
  AMIGALEFT  = IEQUALIFIER_LCOMMAND;
  AMIGARIGHT = IEQUALIFIER_RCOMMAND;
  AMIGAKEYS  = AMIGALEFT or AMIGARIGHT;

  CURSORUP        = $4C;
  CURSORLEFT      = $4F;
  CURSORRIGHT     = $4E;
  CURSORDOWN      = $4D;
  KEYCODE_Q       = $10;
  KEYCODE_X       = $32;
  KEYCODE_N       = $36;
  KEYCODE_M       = $37;
  KEYCODE_V       = $34;
  KEYCODE_B       = $35;
  KEYCODE_LESS    = $38;
  KEYCODE_GREATER = $39;

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

  GADGETCOUNT     = 8;
  UPFRONTGADGET   = 0;
  DOWNBACKGADGET  = 1;
  SIZEGADGET      = 2;
  CLOSEGADGET     = 3;
  DRAGGADGET      = 4;
  SUPFRONTGADGET  = 5;
  SDOWNBACKGADGET = 6;
  SDRAGGADGET     = 7;


// DrawInfo

// This is a packet of information for graphics rendering.  It originates with a Screen, and is gotten using GetScreenDrawInfo( screen );
// If you find dri_Version >= DRI_VERSION, you know this structure has at least the fields defined in this version of the include file
CONST
 RI_VERSION  = 1;     { obsolete, will be removed            }
 DRI_VERSION = 3;

type
  // This is a packet of information for graphics rendering.  It originates with a Screen, and is gotten using GetScreenDrawInfo( screen );
  PDrawInfo = ^TDrawInfo;
  TDrawInfo = record
    dri_Version: Word;   // will be  DRI_VERSION
    dri_NumPens: Word;   // guaranteed to be >= numDrIPens
    dri_Pens: PWord;     // pointer to pen array
    dri_Font: PTextFont; // screen default font
    dri_Depth: Word;     // (initial) depth of screen bitmap

    dri_Resolution: record
      x: Word;           // from DisplayInfo database for initial display mode
      y: Word;
    end;

    dri_Flags: LongWord; // defined below (DIRF_*)

    dri_CheckMark: PImage; // pointer to scaled checkmark image Will be nil if DRI_VERSION < 2
    dri_AmigaKey: PImage;  // pointer to scaled Amiga-key image Will be NULL if DRI_VERSION < 2

    dri_Reserved: array[0..4] of LongWord; // avoid recompilation ;^)
  end;

const
  // dri_Flags
  DRIF_NEWLOOK     = 1 shl 0; // specified SA_Pens, full treatment
  DRIF_DIRECTCOLOR = 1 shl 1;
  // rendering pen number indexes into DrawInfo.dri_Pens[]
  DETAILPEN        = 0;  // compatible Intuition rendering pens
  BLOCKPEN         = 1;  // compatible Intuition rendering pens
  TEXTPEN          = 2;  // text on background
  SHINEPEN         = 3;  // bright edge on 3D objects
  SHADOWPEN        = 4;  // dark edge on 3D objects
  FILLPEN          = 5;  // active-window/selected-gadget fill
  FILLTEXTPEN      = 6;  // text over FILLPEN
  BACKGROUNDPEN    = 7;  // always color 0
  HIGHLIGHTTEXTPEN = 8;  // special color text, on background
  // Only present if DRI_VERSION >= 2
  BARDETAILPEN     = 9;  // text/detail in screen-bar/menus
  BARBLOCKPEN      = 10; // screen-bar/menus fill
  BARTRIMPEN       = 11; // trim under screen-bar
  NUMDRIPENS       = 12;

//It is sometimes useful to specify that a pen value is to be the complement of color zero to three.  The "magic" numbers serve that purpose:
  PEN_C3 = $FEFC; // Complement of color 3
  PEN_C2 = $FEFD; // Complement of color 2
  PEN_C1 = $FEFE; // Complement of color 1
  PEN_C0 = $FEFF; // Complement of color 0

const
  // The SCREENTYPE bits are reserved for describing various Screen types available under Intuition.
  // The screen flags have the suffix "_f" added to avoid conflicts with routine names.
  // Screen^.Flags and (Ext)NewScreen^.Type
  WBENCHSCREEN_f = 1 shl 0;   // Ta Da!  The Workbench
  PUBLICSCREEN_f = 1 shl 1;
  CUSTOMSCREEN_f = $000F;     // for that special look
  SCREENTYPE_f   = $000F;     // all the screens types available
  // Screen^.Flags
  SHOWTITLE_f    = 1 shl 4;   // this gets set by a call to ShowTitle()
  BEEPING_f      = 1 shl 5;   // set when Screen is beeping
  CUSTOMBITMAP_f = 1 shl 6;   // if you are supplying your own BitMap
  SCREENBEHIND_f = 1 shl 7;   // if you want your screen to open behind already open screens
  SCREENQUIET_f  = 1 shl 8;   // if you do not want Intuition to render into your screen (gadgets, title)
  SCREENHIRES    = 1 shl 9;   // do no use lowres gadgets (private)
  PENSHARED      = 1 shl 10;  // Screen opener set (SA_SharePens,True)
  NS_EXTENDED    = 1 shl 12;  // ExtNewScreen.Extension is valid
  AUTOSCROLL     = 1 shl 14;  // screen is to autoscoll

  STDSCREENHEIGHT = -1; // supply in NewScreen.Height
  STDSCREENWIDTH  = -1; // supply in NewScreen.Width

const
  // Screen attribute tag ID's.  These are used in the ti_Tag field of TagItem arrays passed to OpenScreenTagList() (or in the ExtNewScreen.Extension field).
  SA_Dummy       = TAG_USER + 32;
  // these items specify items equivalent to fields in NewScreen
  SA_Left        = SA_Dummy + 1;
  SA_Top         = SA_Dummy + 2;
  SA_Width       = SA_Dummy + 3;
  SA_Height      = SA_Dummy + 4;  // traditional screen positions and dimensions
  SA_Depth       = SA_Dummy + 5;  // screen bitmap depth
  SA_DetailPen   = SA_Dummy + 6;  //serves as default for windows, too
  SA_BlockPen    = SA_Dummy + 7;
  SA_Title       = SA_Dummy + 8;  // default screen title
  SA_Colors      = SA_Dummy + 9;  // ti_Data is an array of struct ColorSpec, terminated by ColorIndex = -1.  Specifies initial screen palette colors.
  SA_ErrorCode   = SA_Dummy + 10; // ti_Data points to LONG error code (values below)
  SA_Font        = SA_Dummy + 11; // equiv. to NewScreen.Font
  SA_SysFont     = SA_Dummy + 12; // Selects one of the preferences system fonts: 0 - old DefaultFont, fixed-width 1 - WB Screen preferred font
  SA_Type        = SA_Dummy + 13; // equiv. to NewScreen.type
  SA_BitMap      = SA_Dummy + 14; // ti_Data is pointer to custom BitMap.  This implies type of CUSTOMBITMAP
  SA_PubName     = SA_Dummy + 15; // presence of this tag means that the screen is to be a public screen. Please specify BEFORE the two tags below
  SA_PubSig      = SA_Dummy + 16;
  SA_PubTask     = SA_Dummy + 17; // Task ID and signal for being notified that the last window has closed on a public screen.
  SA_DisplayID   = SA_Dummy + 18; // ti_Data is new extended display ID
  SA_DClip       = SA_Dummy + 19; // ti_Data points to a rectangle which defines screen display clip region
  SA_Overscan    = SA_Dummy + 20; { was S_STDDCLIP.  Set to one of the OSCAN_* specifiers below to get a system standard
                                       overscan region for your display clip, screen dimensions (unless otherwise specified),
                                       and automatically centered position (partial support only so far).
                                       If you use this, you shouldn't specify SA_DClip.  SA_Overscan is for "standard"
                                       overscan dimensions, SA_DClip is for your custom numeric specifications.}
  SA_ShowTitle     = SA_Dummy + 22; // boolean equivalent to flag SHOWTITLE
  SA_Behind        = SA_Dummy + 23; // boolean equivalent to flag SCREENBEHIND
  SA_Quiet         = SA_Dummy + 24; // boolean equivalent to flag SCREENQUIET
  SA_AutoScroll    = SA_Dummy + 25; // boolean equivalent to flag AUTOSCROLL
  SA_Pens          = SA_Dummy + 26; // pointer to ~0 terminated UWORD array, as found in struct DrawInfo
  SA_FullPalette   = SA_Dummy + 27; // boolean: initialize color table to entire preferences palette, rather than compatible pens 0-3, 17-19, with remaining palette as returned by GetColorMap()
  SA_ColorMapEntries = SA_Dummy + 28; {Allows you to override the number of entries in the ColorMap for your screen.  Intuition
                                            normally allocates (1 shl depth) or 32, whichever is more, but you may require even more if you
                                            use certain graphics.library features (eg. palette-banking).}
  SA_Parent        = SA_Dummy + 29; // Pointer to a "parent" screen to attach this one to.  Attached screens slide and depth-arrange together.
  SA_Draggable     = SA_Dummy + 30; // Boolean tag allowing non-draggable screens. Do not use without good reason! (Defaults to True).
  SA_Exclusive     = SA_Dummy + 31; { Boolean tag allowing screens that won't share the display.  Use sparingly!  Starting with 3.01,
                                         attached screens may be SA_Exclusive.  Setting SA_Exclusive for each screen will produce an
                                          exclusive family.   (Defaults to FALSE).}
  SA_SharePens     = SA_Dummy + 32; { For those pens in the screen's DrawInfo^.dri_Pens, Intuition obtains them in shared mode (see
                                         graphics.library/ObtainPen()).  For compatibility, Intuition obtains the other pens of a public
                                         screen as PEN_EXCLUSIVE.  Screens that wish to manage the pens themselves should generally set
                                         this tag to True.  This instructs Intuition to leave the other pens unallocated.}
  SA_BackFill      = SA_Dummy + 33; // provides a "backfill hook" for your screen's Layer_Info. See layers.library/InstallLayerInfoHook()
  SA_Interleaved   = SA_Dummy + 34; // Boolean tag requesting that the bitmap allocated for you be interleaved. (Defaults to False).
  SA_Colors32      = SA_Dummy + 35; { Tag to set the screen's initial palette colors at 32 bits-per-gun.  ti_Data is a pointer
                                         to a table to be passed to the graphics.library/LoadRGB32() function. This format supports both runs of color
                                         registers and sparse registers.  See the autodoc for that function for full details. Any color set here has
                                         precedence over the same register set by SA_Colors.}
  SA_VideoControl  = SA_Dummy + 36; // Pointer to a taglist that Intuition will pass to graphics.library/VideoControl(), upon opening the screen.
  SA_FrontChild    = SA_Dummy + 37; // Pointer to an already open screen that is to be the child of the screen being opened. The child screen will be moved to the front of its family.
  SA_BackChild     = SA_Dummy + 38; // Pointer to an already open screen that is to be the child of the screen being opened. The child screen will be moved to the back of its family.
  SA_LikeWorkbench = SA_Dummy + 39; // 1  = request a screen which is just like the Workbench.  This gives you the same screen mode, depth, size, colors, etc., as the Workbench screen.
  SA_Reserved      = SA_Dummy + 40; // Reserved for private Intuition use

  SA_MinimizeISG   = SA_Dummy + 41; { For compatibility, Intuition always ensures that the inter-screen gap is at least three non-interlaced lines.  If your application
                                           would look best with the smallest possible inter-screen gap, set ti_Data to True. If you use the new graphics VideoControl()
                                           VC_NoColorPaletteLoad tag for your screen's ViewPort, you should also set this tag.}
  // V51
  SA_Displayed     = SA_Dummy + 101; // [..G] LongBool. Check whether the screen is currently displayed on one of system's gfx cards.
  SA_MonitorName   = SA_Dummy + 102; // [I.G] PChar. Should be used to determine on which monitor (gfx card) the screen should be opened. Please use this instead of storing ModeIDs
    // in prefs files! Overrides SA_DisplayID and will be used to get the best available mode together with SA_Depth or SA_Width/Height. Will open a screen on other gfx card if the selected one is not
    // found. Please note that .monitor prefix is included in the name. For autoscrolling and other non-typical screens please read SA_DisplayWidth/Height notes.
  SA_MonitorObject = SA_Dummy + 103; // [..G] PObject_. Returns the MONITORCLASS object associated with the screen. Use it in order to get more info about the gfx card used to display the screen, monitor alignment, etc.

  SA_TopLeftScreen      = SA_Dummy + 112; // [..G] PScreen, Returns a pointer to the displayed screen on a monitor positioned relatively to the monitor your screen is
  SA_TopMiddleScreen    = SA_Dummy + 113; // [..G]   displayed on. Returns NULL if no screen can be found or the fronmost screen is not a public one. You MUST lock the
  SA_TopRightScreen     = SA_Dummy + 114; // [..G]   public screens lists if you wish to use the pointer: this example will position a new window on a monitor positioned
  SA_MiddleLeftScreen   = SA_Dummy + 115; // [..G]   on a left side of the one you have a window on.
  SA_MiddleRightScreen  = SA_Dummy + 117; // [..G]     LockPubScreenList();  //lock the screens list
  SA_BottomLeftScreen   = SA_Dummy + 118; // [..G]     GetAttr(SA_MiddleLeftScreen,APTR(mywindow^.WScreen), @newscreen);
  SA_BottomMiddleScreen = SA_Dummy + 119; // [..G]     if Assigned(newscreen) then OpenNewScreenOnNewScreen(newscreen);
  SA_BottomRightScreen  = SA_Dummy + 120; // [..G]     UnLockPubScreenList();  //unlock the screens list

  SA_StopBlanker  = SA_Dummy + 121; // [.S.] // BOOL. Setting this tag to TRUE will stop the intuition's screensaver/DPMS features. Please note that the call nests,
    // you have to set it to FALSE as many times as you set it to TRUE to really enable blanking again.
    // IMPORTANT: please remember that you're supposed to set SA_StopBlanker to FALSE for example when the user presses Pause in your movie player  and set it to TRUE again once playback is restarted.}
  SA_ShowPointer  = SA_Dummy + 122; // [ISG] LongBool. Setting this tag to FALSE will hide the mouse pointer on your screen. Please note that this tag is ONLY supported for CUSTOM
    //screens. On public screens you're only supposed to modify the pointer of your own window(s). Defaults to TRUE.
  SA_GammaControl = SA_Dummy + 123; // [I..] LongBool. Set to true if you wish to have control of screen's gamma settings. This tag enables the use of SA_GammaRed/Blue/Green tags.

  SA_GammaRed   = SA_Dummy + 124; // [ISG] PByte, Pointer to a 256 byte array containing gamma values for all color levels. Setting this tag to NULL will cause intuition to
  SA_GammaBlue  = SA_Dummy + 125; // [ISG]   use the default values defined for screen's monitor. The array is
  SA_GammaGreen = SA_Dummy + 126; // [ISG]   NOT copied! Setting the pointer to the current value will cause a gamma refresh.

  SA_3DSupport  = SA_Dummy + 127; // [I..] LongBool. Set this tag to TRUE to make intuition pick a screen mode with 3d support. Intuition might pick up a different depth for you if the one you requested doesn't have 3d support at all.

  SA_AdaptSize  = SA_Dummy + 128; // [I..] LongBool. Will adapt screen's width & height to screen's display width/height (note: this is determined by the available modes
    // so if someone has wrong modes for his display, it may match that). EXAMPLE: SA_Width,1280,SA_Height,960,SA_AdaptSize,TRUE on a 1280x1024 LCD will increase the screen height to 1024.

  SA_DisplayWidth  = SA_Dummy + 129; // [I.G] LongWord. Will be used to search for the displayID instead of SA_Width/Height. Use this tag if you want to open a screen
  SA_DisplayHeight = SA_Dummy + 130; // [I.G]   larger or smaller than the display size (for example an autoscrolling screen). The new screen will use dimensions passed with SA_Width/Height.

  SA_OpacitySupport     = SA_Dummy + 131; // [..G]
  SA_SourceAlphaSupport = SA_Dummy + 132; // [..G]

  SA_PixelFormat        = SA_Dummy + 133; // [..G]

  SA_ScreenbarTextYPos  = SA_Dummy + 134; // [..G] LongInt. Text Y pos the screenbar plugins should use. Consistent with title and clocks embedded plugins. Includes the baseline of the font
    // returned with SA_ScreenbarTextFont! You should use this position rather than compute it yourself, since it's SkinConfig dependant
  SA_ScreenbarTextPen   = SA_Dummy + 135; // [..G] LongWord. Text pen color used for rendering text on the screenbar
  SA_ScreenbarTextFont  = SA_Dummy + 136; // [..G] PTextFont. Font used by screenbar
  SA_ScreenbarSignal    = SA_Dummy + 137; // [..G] LongWord. Returns a common signal bit num (not mask!) that sbars may use. Mind that the signal is not yours to free

  SA_ExactMatchMonitorName = SA_Dummy + 138; // [I..] LongBool. If True, the screen will either be opened on the monitor matched by SA_MonitorName or won't open at all
  SA_CompositingLayers     = SA_Dummy + 139; // [I.G] LongBool. Set this to true to request a compositing engine to handle layers on your screen. Get this attr to see if this succceeded. From v51.30


// OpenScreen error codes, which are returned in the (optional) LongInt pointed to by ti_Data for the SA_ErrorCode tag item
  OSERR_NOMONITOR    = 1; // named monitor spec not available
  OSERR_NOCHIPS      = 2; // you need newer custom chips
  OSERR_NOMEM        = 3; // couldn't get normal memory
  OSERR_NOCHIPMEM    = 4; // couldn't get chipmem
  OSERR_PUBNOTUNIQUE = 5; // public screen name already used
  OSERR_UNKNOWNMODE  = 6; // don't recognize mode asked for
  OSERR_TOODEEP      = 7;
  OSERR_ATTACHFAIL   = 8;
  OSERR_NOTAVAILABLE = 9;

// NewScreen
type
  PNewScreen = ^TNewScreen;
  TNewScreen = record
    LeftEdge,
    TopEdge,
    Width,
    Height,
    Depth: SmallInt;        // screen dimensions

    DetailPen,
    BlockPen: Byte;         // for bar/border/gadget rendering

    ViewModes: Word;        // the Modes for the ViewPort (and View)
    SType: Word;            // the Screen type (see defines above) (Type in C-Include)

    Font: PTextAttr;        // this Screen's default text attributes
    DefaultTitle: PChar;    // the default title for this Screen
    Gadgets: PGadget;       // your own Gadgets for this Screen

  { if you are opening a CUSTOMSCREEN and already have a BitMap
    that you want used for your Screen, you set the flags CUSTOMBITMAP in
    the type field and you set this variable to point to your BitMap
    structure.  The structure will be copied into your Screen structure,
    after which you may discard your own BitMap if you want}
    CustomBitMap: PBitMap;
  end;

  PExtNewScreen = ^TExtNewScreen;
  TExtNewScreen = record
    LeftEdge,
    TopEdge,
    Width,
    Height,
    Depth: SmallInt;
    DetailPen,
    BlockPen: Byte;
    ViewModes: Word;
    ens_Type: Word;     { type in C-Includes }
    Font: PTextAttr;
    DefaultTitle: PChar;
    Gadgets: PGadget;
    CustomBitMap: PBitMap;
    Extension: PTagItem; // ExtNewScreen specific extension SA_*
  end;


const
  // === Overscan Types
  OSCAN_TEXT     = 1; // entirely visible
  OSCAN_STANDARD = 2; // just past edges
  OSCAN_MAX      = 3; // as much as possible
  OSCAN_VIDEO    = 4; // even more than is possible

type
// Public Shared Screen Node
{ This is the representative of a public shared screen.
  This is an internal data structure, but some functions may
  present a copy of it to the calling application.  In that case,
  be aware that the screen pointer of the structure can NOT be
  used safely, since there is no guarantee that the referenced
  screen will remain open and a valid data structure.
  Never change one of these.}
  PPubScreenNode = ^TPubScreenNode;
  TPubScreenNode = record
    psn_Node: TNode;            // ln_Name is screen name
    psn_Screen: PScreen;
    psn_Flags: Word;            // below (PSNF_*)
    psn_Size: SmallInt;         // includes name buffer
    psn_VisitorCount: SmallInt; // how many visitor windows
    psn_SigTask: PTask;         // who to signal when visitors gone
    psn_SigBit: Byte;           // which signal
  end;

const
  PSNF_PRIVATE = $0001;

  MAXPUBSCREENNAME = 139; // names no longer, please

  // pub screen modes
  SHANGHAI     = $0001; // put workbench windows on pub screen
  POPPUBSCREEN = $0002; // pop pub screen to front when visitor opens

  // ScreenDepth() flags, it's generaly easier to use ScreenToFront()/ScreenToBack() calls
  SDEPTH_TOFRONT = 0; // Bring screen to front
  SDEPTH_TOBACK  = 1; // Send screen to back

  SPOS_RELATIVE    = 0; // The x1 and y1 parameters to ScreenPosition() describe the offset in coordinates you wish to move the screen by.
  SPOS_ABSOLUTE    = 1; // The x1 and y1 parameters to ScreenPosition() describe the absolute coordinates you wish to move the screen to.
  SPOS_MAKEVISIBLE = 2; // (x1,y1)-(x2,y2) describes a rectangle on the screen which you would like autoscrolled into view.
  SPOS_FORCEDRAG   = 4; // Move non-draggable screen

{ Intuition supports double-buffering in screens, with friendly interaction with menus and certain gadgets.
  For each buffer, you need to get one of these structures from the AllocScreenBuffer() call.  Never allocate your own ScreenBuffer structures!
  The sb_DBufInfo field is for your use.  See the graphics.library AllocDBufInfo() autodoc for details.}
type
  PScreenBuffer = ^TScreenBuffer;
  TScreenBuffer = record
    sb_BitMap: PBitMap;      // BitMap of this buffer
    sb_DBufInfo: PDBufInfo;  // DBufInfo for this buffer
  end;

const
  // These are the flags that may be passed to AllocScreenBuffer().
  SB_SCREEN_BITMAP = 1;
  SB_COPY_BITMAP   = 2;

  // Flags for screen's layer opacity + source opacity support
  SAOS_OpacitySupport_None     = 0; // opacity setting unsupported
  SAOS_OpacitySupport_OnOff    = 1; // only fully visible/fully invisible supported
  SAOS_OpacitySupport_CPU      = 2; // opacity is done by the CPU - slow
  SAOS_OpacitySupport_HW       = 3; // opacity is hardware accelerated

  SASA_SourceAlphaSupport_None = 0; // source alpha not supported
  SASA_SourceAlphaSupport_CPU  = 1; // source alpha done by the CPU - slow!!!
  SASA_SourceAlphaSupport_HW   = 2; // source alpha hardware accelerated

const
  // these are the definitions for the printer configurations
  FILENAME_SIZE = 30; // Filename size
  DEVNAME_SIZE  = 16;
  POINTERSIZE = (1 + 16 + 1) * 2; // Size of Pointer data buffer

{ These defines are for the default font size.   These actually describe the
  height of the defaults fonts.  The default font type is the topaz
  font, which is a fixed width font that can be used in either
  eighty-column or sixty-column mode.  The Preferences structure reflects
  which is currently selected by the value found in the variable FontSize,
  which may have either of the values defined below.  These values actually
  are used to select the height of the default font.  By changing the
  height, the resolution of the font changes as well.}
  TOPAZ_EIGHTY = 8;
  TOPAZ_SIXTY = 9;

type
  PPreferences = ^TPreferences;
  TPreferences = record
    FontHeight: ShortInt;      // height for system default font
    PrinterPort: Byte;         // printer port connection constant describing what's hooked up to the port
    BaudRate: Word;            // baud rate for the serial port

  // various timing rates
    KeyRptSpeed: Ttimeval;     // repeat speed for keyboard
    KeyRptDelay: Ttimeval;     // Delay before keys repeat
    DoubleClick: Ttimeval;     // Interval allowed between clicks

  // Intuition Mouse-Pointer data
    PointerMatrix: array[0..POINTERSIZE - 1] of Word; // Definition of pointer sprite
    XOffset: ShortInt;   // X-Offset for active 'bit'
    YOffset: ShortInt;   // Y-Offset for active 'bit'
    color17: Word;       //
    color18: Word;       // Colours for sprite pointer
    color19: Word;       //
    PointerTicks: Word;  // Sensitivity of the pointer

   // Workbench Screen colors
    color0: Word;        //
    color1: Word;        // Standard default colours
    color2: Word;        // Used in the Workbench
    color3: Word;        //

   // positioning data for the Intuition View
    ViewXOffset: ShortInt; // Offset for top lefthand corner
    ViewYOffset: ShortInt; // X and Y dimensions
    ViewInitX,
    ViewInitY: SmallInt;   // View initial offset values

    EnableCLI: WordBool;   // CLI availability switch

  // printer configurations
    PrinterType: Word;     // printer type
    PrinterFilename: array[0..FILENAME_SIZE - 1] of char; // file for printer

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
    PaperSize: Word;          // paper size
    PaperLength: Word;        // paper length in number of lines
    PaperType: Word;          // continuous or single sheet

  // Serial device settings: These are six nibble-fields in three bytes (these look a little strange so the defaults will map out to zero)
    SerRWBits: Byte;  // upper nibble = (8-number of read bits), lower nibble = (8-number of write bits)
    SerStopBuf: Byte; // upper nibble = (number of stop bits - 1), lower nibble = (table value for BufSize)
    SerParShk: Byte;  // upper nibble = (value for Parity setting), lower nibble = (value for Handshake mode)
    LaceWB: Byte;     // if workbench is to be interlaced

  // temp file for printer
    Pad: array[0..11] of Byte;
    PrtDevName: array [0..DEVNAME_SIZE - 1] of char; // Device used by printer.device (leave out the ".device")

    DefaultPrtUnit: Byte; // Default unit opened by printer.device
    DefaultSerUnit: Byte; // Default serial unit

    RowSizeChange: ShortInt;    // Affect NormalDisplayRows/Columns
    ColumnSizeChange: ShortInt;

    PrintFlags: Word;     // user preference flags
    PrintMaxWidth: Word;  // max width of printed picture in 10ths/inch
    PrintMaxHeight: Word; // max height of printed picture in 10ths/inch
    PrintDensity: Byte;   // print density
    PrintXOffset: Byte;   // offset of printed picture in 10ths/inch

    wb_Width: Word;       // override default workbench width
    wb_Height: Word;      // override default workbench height
    wb_Depth: Byte;       // override default workbench depth

    ext_size: Byte;       // extension information -- do not touch!
    // extension size in blocks of 64 bytes DO NOT TOUCH
  end;

const
  // Workbench Interlace (use one bit)
  LACEWB      = 1 shl 0;
  LW_RESERVED = 1;        // internal use only
  SCREEN_DRAG = 1 shl 14;
  MOUSE_ACCEL = 1 shl 15;
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
  Single_PT  = $80;
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
  // PaperSize
  US_LETTER    = $00;
  US_LEGAL     = $10;
  N_TRACTOR    = $20;
  W_TRACTOR    = $30;
  CUSTOM_PAPER = $40;
  // European sizes
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
  CUSTOM_NAME      = $00;
  ALPHA_P_101      = $01;
  BROTHER_15XL     = $02;
  CBM_MPS1000      = $03;
  DIAB_630         = $04;
  DIAB_ADV_D25     = $05;
  DIAB_C_150       = $06;
  EPSON            = $07;
  EPSON_JX_80      = $08;
  OKIMATE_20       = $09;
  QUME_LP_20       = $0A;
  HP_LASERJET      = $0B;
  HP_LASERJET_PLUS = $0C;
  // Serial Input Buffer Sizes
  SBUF_512   = $00;
  SBUF_1024  = $01;
  SBUF_2048  = $02;
  SBUF_4096  = $03;
  SBUF_8000  = $04;
  SBUF_16000 = $05;
  // Serial Bit Masks
  SREAD_BITS  = $F0; // for SerRWBits
  SWRITE_BITS = $0F;

  SSTOP_BITS    = $F0; // for SerStopBuf
  SBUFSIZE_BITS = $0F;

  SPARITY_BITS = $F0; // for SerParShk
  SHSHAKE_BITS = $0F;
  // Serial Parity (upper nibble, after being shifted by macro SPARNUM())
  SPARITY_NONE  = 0;
  SPARITY_EVEN  = 1;
  SPARITY_ODD   = 2;
  SPARITY_MARK  = 3;
  SPARITY_SPACE = 4;
  // Serial Handshake Mode (lower nibble, after masking using macro SHANKNUM())
  SHSHAKE_XON  = 0;
  SHSHAKE_RTS  = 1;
  SHSHAKE_NONE = 2;
  // new defines for PrintFlags
  CORRECT_RED   = $0001; // color correct red shades
  CORRECT_GREEN = $0002; // color correct green shades
  CORRECT_BLUE  = $0004; // color correct blue shades

  CENTER_IMAGE = $0008; // center image on paper

  IGNORE_DIMENSIONS   = $0000; // ignore max width/height settings
  BOUNDED_DIMENSIONS  = $0010; // use max width/height as boundaries
  ABSOLUTE_DIMENSIONS = $0020; // use max width/height as absolutes
  PIXEL_DIMENSIONS    = $0040; // use max width/height as prt pixels
  MULTIPLY_DIMENSIONS = $0080; // use max width/height as multipliers

  INTEGER_SCALING = $0100; // force integer scaling

  ORDERED_DITHERING  = $0000; // ordered dithering
  HALFTONE_DITHERING = $0200; // halftone dithering
  FLOYD_DITHERING    = $0400; // Floyd-Steinberg dithering

  ANTI_ALIAS  = $0800; // anti-alias image
  GREY_SCALE2 = $1000; // for use with hi-res monitor

  // masks used for checking bits
  CORRECT_RGB_MASK = CORRECT_RED or CORRECT_GREEN or CORRECT_BLUE;
  DIMENSIONS_MASK = BOUNDED_DIMENSIONS or ABSOLUTE_DIMENSIONS or PIXEL_DIMENSIONS or MULTIPLY_DIMENSIONS;
  DITHERING_MASK = HALFTONE_DITHERING or FLOYD_DITHERING;

{ Be sure to protect yourself against someone modifying these data as you look at them.  This is done by calling:
  lock = LockIBase(0), which returns an Integer.  When done call UnlockIBase(lock) where lock is what LockIBase() returned.}
type
  PIntuitionBase = ^TIntuitionBase;
  TIntuitionBase = record
    // IntuitionBase should never be directly modified by programs
    // even a little bit, guys/gals; do you hear me?
    LibNode: TLibrary;

    ViewLord: TView;

    ActiveWindow: PWindow;
    ActiveScreen: PScreen;
    // the FirstScreen variable points to the frontmost Screen.   Screens are then maintained in a front to back order using Screen.NextScreen
    FirstScreen: PScreen; // for linked list of all screens

    Flags: LongWord;      // see definitions below
    MouseY,
    MouseX: SmallInt;     // mouse position relative to View

    Seconds: LongWord;    // timestamp of most current input event
    Micros: LongWord;     // timestamp of most current input event
    // I told you this was private. The data beyond this point has changed, is changing, and will continue to change.
  end;

{ Package of information passed to custom and 'boopsi'
  gadget 'hook' functions.  This structure is READ ONLY.}
type
  PGadgetInfo = ^TGadgetInfo;

  TGadgetInfo = record
    gi_Screen: PScreen;       // ScreenPtr
    gi_Window: PWindow;       // nil for screen gadgets
    gi_Requester: PRequester; // nil IF not GTYP_REQGADGET
    gi_RastPort: PRastPort;   // rendering information: don't use these without cloning/locking. Official way is to call ObtainRPort()
    gi_Layer: PLayer;         // LayerPtr
    { copy of dimensions of screen/window/g00/req(/group)
      that gadget resides in.  Left/Top of this box is
      offset from window mouse coordinates to gadget coordinates
               screen gadgets:                 0,0 (from screen coords)
       window gadgets (no g00):                0,0
       GTYP_GZZGADGETs (borderlayer):          0,0
       GZZ innerlayer gadget:          borderleft, bordertop
       Requester gadgets:              reqleft, reqtop}
    gi_Domain: TIBox;
    gi_Pens: record
      DetailPen: Byte;
      BlockPen: Byte;
    end;
    gi_DrInfo: PDrawInfo;                 // the Detail and Block pens in gi_DrInfo^.dri_Pens[] are for the screen.  Use the above for window-sensitive colors.
    gi_Reserved: array[0..5] of LongWord; // reserved space: this structure is extensible anyway, but using these saves some recompilation
  end;

{ system private data structure for now
   prop gadget extra info}
  PPGX = ^TPGX;
  TPGX = record
    pgx_Container: TIBox;
    pgx_NewKnob: TIBox;
  end;

{ this casts MutualExclude for easy assignment of a hook
 * pointer to the unused MutualExclude field of a custom gadget
 }

{** User visible handles on objects, classes, messages **}
Type
  Object_ = LongWord;
  PObject_ = ^Object_;
  PPObject_ = ^PObject_;
  ClassID = ^Byte;

{ you can use this type to point to a 'generic' message,
  in the object-oriented programming parlance.  Based on
  the value of 'MethodID', you dispatch to processing
  for the various message types.  The meaningful parameter
  packet structure definitions are defined below.}

  PMsg = ^TMsg;
  TMsg = record
    MethodID: LongWord;
  end;

{ Class id strings for Intuition classes.
  There's no real reason to use the uppercase constants
  over the lowercase strings, but this makes a good place
  to list the names of the built-in classes.}
const
  ROOTCLASS: PChar     = 'rootclass';
  IMAGECLASS: PChar    = 'imageclass';
  FRAMEICLASS: PChar   = 'frameiclass';
  SYSICLASS: PChar     = 'sysiclass';
  FILLRECTCLASS: PChar = 'fillrectclass';
  GADGETCLASS: PChar   = 'gadgetclass';
  PROPGCLASS: PChar    = 'propgclass';
  STRGCLASS: PChar     = 'strgclass';
  BUTTONGCLASS: PChar  = 'buttongclass';
  FRBUTTONCLASS: PChar = 'frbuttonclass';
  GROUPGCLASS: PChar   = 'groupgclass';
  ICCLASS: PChar       = 'icclass';
  MODELCLASS: PChar    = 'modelclass';
  ITEXTICLASS: PChar   = 'itexticlass';
  POINTERCLASS: PChar  = 'pointerclass';

{ Dispatched method ID's
    NOTE: Applications should use Intuition entry points, not direct
    DoMethod() calls, for NewObject, DisposeObject, SetAttrs,
    SetGadgetAttrs, and GetAttr}

  OM_Dummy     = $100;
  OM_NEW       = OM_Dummy + 1;  // 'object' parameter is 'true class'
  OM_DISPOSE   = OM_Dummy + 2;  // delete self (no parameters)
  OM_SET       = OM_Dummy + 3;  // set attributes (in tag list)
  OM_GET       = OM_Dummy + 4;  // return single attribute value
  OM_ADDTAIL   = OM_Dummy + 5;  // add self to a List (let root do it)
  OM_REMOVE    = OM_Dummy + 6;  // remove self from list
  OM_NOTIFY    = OM_Dummy + 7;  // send to self: notify dependents
  OM_UPDATE    = OM_Dummy + 8;  // notification message from somebody
  OM_ADDMEMBER = OM_Dummy + 9;  // used by various classes with lists
  OM_REMMEMBER = OM_Dummy + 10; // used by various classes with lists

// Parameter 'Messages' passed to methods
type
  // OM_NEW and OM_SET
  PopSet = ^TopSet;
  TopSet = record
    MethodID: LongWord;
    ops_AttrList: PTagItem; // new attributes
    ops_GInfo: PGadgetInfo; // always there for gadgets, when SetGadgetAttrs() is used, but will be nil for OM_NEW
  end;

  // OM_GET
  PopGet = ^TopGet;
  TopGet = record
    MethodID,
    opg_AttrID: LongWord;
    opg_Storage: Pointer;   // may be other types, but 'int' types are all LongWord
  end;

  // OM_NOTIFY, and OM_UPDATE
  PopUpdate = ^TopUpdate;
  TopUpdate = record
    MethodID: LongWord;
    opu_AttrList: PTagItem; // new attributes
    opu_GInfo: PGadgetInfo; // non-nil when SetGadgetAttrs OR notification resulting from gadget input occurs.
    opu_Flags: LongWord;    // defined below (OPUF_*)
  end;

  // OM_ADDTAIL
  PopAddTail = ^TopAddTail;
  TopAddTail = record
    MethodID: LongWord;
    opat_List: PList;
  end;

  // OM_ADDMEMBER, OM_REMMEMBER
  PopMember = ^TopMember;
  TopMember = record
    MethodID: LongWord;
    opam_Object: PObject_;
  end;
  TopAddMember = TopMember;
  PopAddMember = ^TopAddMember;

{ this flag means that the update message is being issued from
  something like an active gadget, a la GACT_FOLLOWMOUSE.  When
  the gadget goes inactive, it will issue a final update
  message with this bit cleared.  Examples of use are for
  GACT_FOLLOWMOUSE equivalents for propgadclass, and repeat strobes
  for buttons.}
const
  OPUF_INTERIM = 1; // opu_Flags

{*****************************************}
{** 'White box' access to struct IClass **}
{*****************************************}

{ This structure is READ-ONLY, and allocated only by Intuition }
type
  PIClass = ^TIClass;
  TIClass = record          // also used as Class
    cl_Dispatcher: THook;       // pointer to the class' Dispatcher
    cl_Reserved: LongWord;      // must be 0
    cl_Super: PIClass;          // pointer to the class's Super class, that is the class it inherits from
    cl_ID: ClassID;
    cl_InstOffset: Word;        // offset and size of the instance data for this class
    cl_InstSize: Word;

    cl_UserData: IPTR;          // class' private data of choice
    cl_SubclassCount: LongWord; // number of direct subclasses
    cl_ObjectCount: LongWord;   // number of instances
    cl_Flags: LongWord;         // Flags (CLF_INLIST)
  end;

const
  CLF_INLIST = $00000001; // class is in public class list

// Instance data of the root class, preceeding the object. Might grow from
// the beginning, so the o_Class offset will always stay the same relatively to
// the pointer returned by NewObject()
type
  P_Object = ^T_Object;
  T_Object = record
    o_Node: TMinNode; // do NOT use
    o_Class: PIClass;
  end;

// BOOPSI class libraries should use this structure as the base for their
// library data.  This allows developers to obtain the class pointer for
// performing object-less inquiries.
  PClassLibrary = ^TClassLibrary;
  TClassLibrary = record
    cl_Lib: TLibrary;  // Embedded library
    cl_Pad: Word;      // Align the structure
    cl_Class: PIClass; // Class pointer
  end;

{
 * NOTE:  <intuition/iobsolete.h> is included at the END of this file!
 }

// Gadget Class attributes
const
  GA_Dummy         = TAG_USER + $30000;
  GA_Left          = GA_Dummy + 1;  // (LongInt) Left edge of the gadget relative to the left edge of the window
  GA_RelRight      = GA_Dummy + 2;  // (LongInt) Left edge of the gadget relative to the right edge of the window
  GA_Top           = GA_Dummy + 3;  // (LongInt) Top edge of the gadget relative to the top edge of the window
  GA_RelBottom     = GA_Dummy + 4;  // (LongInt) Top edge of the gadget relative to the bottom edge of the window
  GA_Width         = GA_Dummy + 5;  // (LongInt) Width of the gadget
  GA_RelWidth      = GA_Dummy + 6;  // (LongInt) Width of the gadget relative to the width of the window
  GA_Height        = GA_Dummy + 7;  // (LONG) Height of the gadget
  GA_RelHeight     = GA_Dummy + 8;  // (LONG) Height of the gadget relative to the height of the window
  GA_Text          = GA_Dummy + 9;  // (PChar) Gadget imagry is #0 terminated string
  GA_Image         = GA_Dummy + 10; // (PImage) Gadget imagry is an image
  GA_Border        = GA_Dummy + 11; // (PBorder) Gadget imagry is a border
  GA_SelectRender  = GA_Dummy + 12; // (PImage) Selected gadget imagry
  GA_Highlight     = GA_Dummy + 13; // (Word) One of GFLG_GADGHNONE, GFLG_GADGHBOX, GFLG_GADGHCOMP, or GFLG_GADGHIMAGE
  GA_Disabled      = GA_Dummy + 14; // (LongBool) Indicate whether gadget is disabled or not. Defaults to False.
  GA_GZZGadget     = GA_Dummy + 15; // (LongBool) Indicate whether the gadget is for WFLG_GIMMEZEROZERO window borders or not.  Defaults to False.
  GA_ID            = GA_Dummy + 16; // (Word) Gadget ID assigned by the application
  GA_UserData      = GA_Dummy + 17; // (APTR) Application specific data
  GA_SpecialInfo   = GA_Dummy + 18; // (APTR) Gadget specific data
  GA_Selected      = GA_Dummy + 19; // (LongBool) Indicate whether the gadget is selected or not. Defaults to False
  GA_EndGadget     = GA_Dummy + 20; // (LongBool) When set tells the system that when this gadget is selected causes the requester that it is in to be ended. Defaults to False.
  GA_Immediate     = GA_Dummy + 21; // (LongBool) When set indicates that the gadget is to notify the application when it becomes active.  Defaults to False.
  GA_RelVerify     = GA_Dummy + 22; // (LongBool) When set indicates that the application wants to verify that the pointer was still over the gadget when the select button is released.  Defaults to False
  GA_FollowMouse   = GA_Dummy + 23; // (LongBool) When set indicates that the application wants to  be notified of mouse movements while the gadget is active.
                                    //   It is recommmended that GA_Immediate and GA_RelVerify are also used so that the active gadget can be tracked by the application.  Defaults to FALSE. }
  GA_RightBorder   = GA_Dummy + 24; // (LongBool) Indicate whether the gadget is in the right border or not.  Defaults to False
  GA_LeftBorder    = GA_Dummy + 25; // (LongBool) Indicate whether the gadget is in the left border or not.  Defaults to False.
  GA_TopBorder     = GA_Dummy + 26; // (LongBool) Indicate whether the gadget is in the top border or not.  Defaults to False.
  GA_BottomBorder  = GA_Dummy + 27; // (LongBool) Indicate whether the gadget is in the bottom border or not.  Defaults to False
  GA_ToggleSelect  = GA_Dummy + 28; // (LongBool) Indicate whether the gadget is toggle-selected or not.  Defaults to False.
  GA_SysGadget     = GA_Dummy + 29; // (LongBool) Reserved for system use to indicate that the  gadget belongs to the system.  Defaults to False.
  GA_SysGType      = GA_Dummy + 30; // (Word) Reserved for system use to indicate the gadget type.
  GA_Previous      = GA_Dummy + 31; // (PGadget) Previous gadget in the linked list. NOTE: This attribute CANNOT be used to link new gadgets
                                    //  into the gadget list of an open window or requester. You must use AddGList().
  GA_Next          = GA_Dummy + 32; // (PGadget) Next gadget in the linked list.
  GA_DrawInfo      = GA_Dummy + 33; // (PDrawInfo) Some gadgets need a DrawInfo at creation time
  // You should use at most ONE of GA_Text, GA_IntuiText, and GA_LabelImage
  GA_IntuiText     = GA_Dummy + 34; // (PIntuiText) Label is an IntuiText.
  GA_LabelImage    = GA_Dummy + 35; // (PObject_) Label is an image object.
  // V37:
  GA_TabCycle      = GA_Dummy + 36; // (LongBool) indicates that this gadget is to participate in cycling activation with Tab or Shift-Tab.
  // V39:
  GA_GadgetHelp    = GA_Dummy + 37; // (LongBool) indicates that this gadget sends gadget-help
  GA_Bounds        = GA_Dummy + 38; // (PIBox) is to be copied into the extended gadget's bounds.
  GA_RelSpecial    = GA_Dummy + 39; // (LongBool) indicates that this gadget has the "special relativity"
                                    //   property, which is useful for certain fancy relativity operations through the GM_LAYOUT method.
  // V42
  GA_TextAttr       = GA_Dummy + 40; // (PTextAttr) Indicate the font to use for the gadget.
  GA_ReadOnly       = GA_Dummy + 41; // (LongBool) Indicate that the gadget is read-only (non-selectable). Defaults to False
  // V44
  GA_Underscore     = GA_Dummy + 42; // (Char) Underscore/escape character for keyboard shortcuts. Defaults to '_' .
  GA_ActivateKey    = GA_Dummy + 43; // (PChar) Set/Get the gadgets shortcut/activation key(s) Defaults to nil
  GA_BackFill       = GA_Dummy + 44; // (PHook) Backfill pattern hook. Defaults to nil.
  GA_GadgetHelpText = GA_Dummy + 45; // (PChar) RESERVERD/PRIVATE DO NOT USE Defaults to nil.
  GA_UserInput      = GA_Dummy + 46; // (LongBool) Notification tag indicates this notification is from the activite
                                     //   gadget receiving user input - an attempt to make IDCMPUPDATE more efficient. Defaults to False
  // V50
  GA_LabelPlace     = GA_Dummy + 100; // [I..] (LongInt) Choose the placing of the label. GadgetClass does not support
                                      //   this directly. Its subclasses have to take care of that. For possible values see below.
  GV_LabelPlace_In    = 1;
  GV_LabelPlace_Left  = 2;
  GV_LabelPlace_Right = 3;
  GV_LabelPlace_Above = 4;
  GV_LabelPlace_Below = 5;

  // PROPGCLASS attributes
  PGA_Dummy      = TAG_USER + $31000;
  PGA_Freedom    = PGA_Dummy + 1;
  // only one of FREEVERT or FREEHORIZ }
  PGA_Borderless = PGA_Dummy + 2;
  PGA_HorizPot   = PGA_Dummy + 3;
  PGA_HorizBody  = PGA_Dummy + 4;
  PGA_VertPot    = PGA_Dummy + 5;
  PGA_VertBody   = PGA_Dummy + 6;
  PGA_Total      = PGA_Dummy + 7;
  PGA_Visible    = PGA_Dummy + 8;
  PGA_Top        = PGA_Dummy + 9;
  // V37:
  PGA_NewLook    = PGA_Dummy + 10;
  // V50:
  PGA_NotifyBehaviour = PGA_Dummy + 30; // [I..] (Word) If set to PG_BEHAVIOUR_NICE OM_NOTIFY messages are sent
                                        //   also during OM_SET/OM_UPDATE, not just when user drags the knob, which is the default behaviour (PG_BEHAVIOUR_COMPATIBLE)
  PGA_RenderBehaviour = PGA_Dummy + 31; // [I..] (Word) If set to PG_BEHAVIOUR_NICE the gadget is re-rendered
                                        //   during OM_SET/OM_UPDATE even when being a subclass of propgclass. The default behaviour (PG_BEHAVIOUR_COMPATIBLE) is that subclasses
                                        //   of propgclass don't render in OM_SET/OM_UPDATE
  PG_BEHAVIOUR_COMPATIBLE = 0;
  PG_BEHAVIOUR_NICE       = 1;

// STRGCLASS attributes
  STRINGA_Dummy      = TAG_USER + $32000;
  STRINGA_MaxChars   = STRINGA_Dummy + 1;
  STRINGA_Buffer     = STRINGA_Dummy + 2;
  STRINGA_UndoBuffer = STRINGA_Dummy + 3;
  STRINGA_WorkBuffer = STRINGA_Dummy + 4;
  STRINGA_BufferPos  = STRINGA_Dummy + 5;
  STRINGA_DispPos    = STRINGA_Dummy + 6;
  STRINGA_AltKeyMap  = STRINGA_Dummy + 7;
  STRINGA_Font       = STRINGA_Dummy + 8;
  STRINGA_Pens       = STRINGA_Dummy + 9;
  STRINGA_ActivePens = STRINGA_Dummy + 10;
  STRINGA_EditHook   = STRINGA_Dummy + 11;
  STRINGA_EditModes  = STRINGA_Dummy + 12;
  // booleans
  STRINGA_ReplaceMode    = STRINGA_Dummy + 13;
  STRINGA_FixedFieldMode = STRINGA_Dummy + 14;
  STRINGA_NoFilterMode   = STRINGA_Dummy + 15;
  STRINGA_Justification  = STRINGA_Dummy + 16; // GACT_STRINGCENTER, GACT_STRINGLEFT, GACT_STRINGRIGHT
  STRINGA_LongVal        = STRINGA_Dummy + 17;
  STRINGA_TextVal        = STRINGA_Dummy + 18;
  STRINGA_ExitHelp       = STRINGA_Dummy + 19; // Set this if you want the gadget to exit when Help is pressed.  Look for a code of $5F, the rawkey code for Help

  SG_DEFAULTMAXCHARS = 128;

// Gadget Layout related attributes
  LAYOUTA_Dummy       = TAG_USER  + $38000;
  LAYOUTA_LayoutObj   = LAYOUTA_Dummy + 1;
  LAYOUTA_Spacing     = LAYOUTA_Dummy + 2;
  LAYOUTA_Orientation = LAYOUTA_Dummy + 3;
  // V42
  LAYOUTA_ChildMaxWidth  = LAYOUTA_Dummy + 4; // (LongBool) Child objects are of equal width. Should default to True for gadgets with a horizontal orientation.
  LAYOUTA_ChildMaxHeight = LAYOUTA_Dummy + 5; // (LongBool) Child objects are of equal height. Should default to True for gadgets with a vertical orientation.
  // orientation values
  LORIENT_NONE  = 0;
  LORIENT_HORIZ = 1;
  LORIENT_VERT  = 2;

// Gadget Method ID's
  GM_Dummy       = -1; // not used for anything
  GM_HITTEST     = 0;  //return GMR_GADGETHIT IF you are clicked on  (whether or not you are disabled).
  GM_RENDER      = 1;  // draw yourself, in the appropriate state
  GM_GOACTIVE    = 2;  // you are now going to be fed input
  GM_HANDLEINPUT = 3;  // handle that input
  GM_GOINACTIVE  = 4;  // whether or not by choice, you are done
  GM_HELPTEST    = 5;  // Will you send gadget help if the mouse is at the specified coordinates?  See below for possible GMR_ values.
  GM_LAYOUT      = 6;  // re-evaluate your size based on the GadgetInfo Domain.  Do NOT re-render yourself yet, you will be called when it is time...

// Parameter "Messages" passed to gadget class methods
// GM_HITTEST
type
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
{ For GM_HITTEST, return GMR_GADGETHIT if you were indeed hit, otherwise return zero.
  For GM_HELPTEST, return GMR_NOHELPHIT (zero) if you were not hit. Typically, return GMR_HELPHIT if you were hit.
  It is possible to pass a UWORD to the application via the Code field of the IDCMP_GADGETHELP message.  Return GMR_HELPCODE or'd with
  the Word-sized result you wish to return.
  GMR_HELPHIT yields a Code value of ((Word) not 0), which should mean "nothing particular" to the application.}

  GMR_GADGETHIT = $00000004; // GM_HITTEST hit
  GMR_NOHELPHIT = $00000000; // GM_HELPTEST didn't hit
  GMR_HELPHIT   = $FFFFFFFF; // GM_HELPTEST hit, return code = not 0
  GMR_HELPCODE  = $00010000; // GM_HELPTEST hit, return low word as code

// GM_RENDER
type
  PgpRender = ^TgpRender;
  TgpRender = record
    MethodID: LongWord;
    gpr_GInfo: PGadgetInfo; // gadget context
    gpr_RPort: PRastPort;   // all ready for use
    gpr_Redraw: Longint;    // might be a "highlight pass"
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
    //(V39) Pointer to TabletData structure, if this event originated
    // from a tablet which sends IESUBCLASS_NEWTABLET events, or nil if not.
    gpi_TabletData: PTabletData;
  end;

// GM_HANDLEINPUT and GM_GOACTIVE  return code flags
// return GMR_MEACTIVE (0) alone if you want more input. Otherwise, return ONE of GMR_NOREUSE and GMR_REUSE, and optionally GMR_VERIFY.
const
  GMR_MEACTIVE = 0;
  GMR_NOREUSE  = 1 shl 1;
  GMR_REUSE    = 1 shl 2;
  GMR_VERIFY   = 1 shl 3; // you MUST set cgp_Termination
  // New for V37:
  // You can end activation with one of GMR_NEXTACTIVE and GMR_PREVACTIVE, which instructs Intuition to activate the next or previous gadget that has GFLG_TABCYCLE set.
  GMR_NEXTACTIVE = 1 shl 4;
  GMR_PREVACTIVE = 1 shl 5;

// GM_GOINACTIVE
type
  PgpGoInactive = ^TgpGoInactive;
  TgpGoInactive = record
    MethodID: LongWord;
    gpgi_GInfo: PGadgetInfo;
    // V37 field only
    gpgi_Abort: LongWord; // gpgi_Abort=1 IF gadget was aborted by Intuition and 0 if gadget went inactive at its own request
  end;

{ New for V39: Intuition sends GM_LAYOUT to any GREL_ gadget when
  the gadget is added to the window (or when the window opens, if
  the gadget was part of the NewWindow.FirstGadget or the WA_Gadgets
  list), or when the window is resized.  Your gadget can set the
  GA_RelSpecial property to get GM_LAYOUT events without Intuition
  changing the interpretation of your gadget select box.  This
  allows for completely arbitrary resizing/repositioning based on
  window size. }
// GM_LAYOUT
type
  PgpLayout = ^TgpLayout;
  TgpLayout = record
    MethodID: LongWord;
    gpl_GInfo: PGadgetInfo;
    gpl_Initial: LongWord; // non-zero if this method was invoked during AddGList() or OpenWindow()
                           // time.  zero if this method was invoked during window resizing.
  end;

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
// GM_KEYTEST send this message
type
  PgpKeyTest = ^TgpKeyTest;
  TgpKeyTest = record
    MethodID: LongWord;
    gpkt_GInfo: PGadgetInfo;
    gpkt_IMsg: PIntuiMessage;  // The IntuiMessage that triggered this
    gpkt_VanillaKey: LongWord;
  end;

{ The GM_KEYGOACTIVE method is called to "simulate" a gadget going down. A gadget should render itself in a selected state when receiving
  this message. If the class supports this method, it must return GMR_KEYACTIVE.
  If a gadget returns zero for this method, it will subsequently be activated via ActivateGadget() with a nil IEvent.}
  PgpKeyInput = ^tgpKeyInput;
  tgpKeyInput = record
    MethodID: Cardinal;        // GM_KEYGOACTIVE
    gpk_GInfo: PGadgetInfo;
    gpk_IEvent: PInputEvent;
    gpk_Termination: PLongInt;
  end;

const
  GMR_KEYACTIVE = 1 shl 4;
  // you MUST set gpk_Termination
  GMR_KEYVERIFY = 1 shl 5;

// The GM_KEYGOINACTIVE method is called to simulate the gadget release. Upon receiving this message, the gadget should do everything a normal gadget release would do.
type
  PgpKeyGoInactive = ^TgpKeyGoInactive;
  TgpKeyGoInactive = record
    MethodID: LongWord;
    gpki_GInfo: PGadgetInfo;
    gpki_Abort: LongWord;
  end;

const
  ICM_Dummy     = $0401;         // used for nothing
  ICM_SETLOOP   = ICM_Dummy + 1; // set/increment loop counter
  ICM_CLEARLOOP = ICM_Dummy + 2; // clear/decrement loop counter
  ICM_CHECKLOOP = ICM_Dummy + 3; // set/increment loop

// no parameters for ICM_SETLOOP, ICM_CLEARLOOP, ICM_CHECKLOOP
// interconnection attributes used by icclass, modelclass, and gadgetclass
  ICA_Dummy      = TAG_USER + $40000;
  ICA_TARGET     = ICA_Dummy + 1; // interconnection target
  ICA_MAP        = ICA_Dummy + 2; // interconnection map tagitem list
  ICSPECIAL_CODE = ICA_Dummy + 3; // a "pseudo-attribute", see below.

{ Normally, the value for ICA_TARGET is some object pointer,
  but if you specify the special value ICTARGET_IDCMP, notification
  will be send as an IDCMP_IDCMPUPDATE message to the appropriate window's
  IDCMP port.  See the definition of IDCMP_IDCMPUPDATE.

  When you specify ICTARGET_IDCMP for ICA_TARGET, the map you
  specify will be applied to derive the attribute list that is
  sent with the IDCMP_IDCMPUPDATE message.  If you specify a map list
  which results in the attribute tag id ICSPECIAL_CODE, the
  lower sixteen bits of the corresponding ti_Data value will
  be copied into the Code field of the IDCMP_IDCMPUPDATE IntuiMessage.}
  ICTARGET_IDCMP: LongWord = not 0;

const
  CUSTOMIMAGEDEPTH = -1; // if image.Depth is this, it's a new Image class object

const
  IA_Dummy       = TAG_USER + $20000;
  IA_Left        = IA_Dummy + 1;
  IA_Top         = IA_Dummy + 2;
  IA_Width       = IA_Dummy + 3;
  IA_Height      = IA_Dummy + 4;
  IA_FGPen       = IA_Dummy + 5; // IA_FGPen also means "PlanePick"  }
  IA_BGPen       = IA_Dummy + 6; // IA_BGPen also means "PlaneOnOff" }
  IA_Data        = IA_Dummy + 7; // bitplanes, for classic image, other image classes may use it for other things
  IA_LineWidth   = IA_Dummy + 8;

  SYSIA_Size     = IA_Dummy + 11;
  SYSIA_Depth    = IA_Dummy + 12; // this is unused by Intuition.  SYSIA_DrawInfo is used instead for V36
  SYSIA_Which    = IA_Dummy + 13;

  IA_Pens        = IA_Dummy + 14; // pointer to UWORD pens[], ala DrawInfo.Pens, MUST be terminated by ~0.  Some classes can choose to have this, or SYSIA_DrawInfo, or both.
  IA_Resolution  = IA_Dummy + 15; // packed uwords for x/y resolution into a longword ala DrawInfo.Resolution

  IA_APattern      = IA_Dummy + 16;
  IA_APatSize      = IA_Dummy + 17;
  IA_Mode          = IA_Dummy + 18;
  IA_Font          = IA_Dummy + 19;
  IA_Outline       = IA_Dummy + 20;
  IA_Recessed      = IA_Dummy + 21;
  IA_DoubleEmboss  = IA_Dummy + 22;
  IA_EdgesOnly     = IA_Dummy + 23;

  SYSIA_DrawInfo   = IA_Dummy + 24; // pass to sysiclass, please

  SYSIA_ReferenceFont = IA_Dummy + 25; // Font to use as reference for scaling certain sysiclass images
  // V39:
  IA_SupportsDisable = IA_Dummy + 26; // By default, Intuition ghosts gadgets itself, instead of relying on IDS_DISABLED or
                                      // IDS_SELECTEDDISABLED.  An imageclass that supports these states should return this attribute
                                      // as TRUE.  You cannot set or clear this attribute, however.
  IA_FrameType = IA_Dummy + 27; // Starting with V39, FrameIClass recognizes several standard types of frame.  Use one
                                // of the FRAME_ specifiers below.  Defaults to FRAME_DEFAULT.
  // V44:
  IA_Underscore  = IA_Dummy + 28; // Indicate underscore keyboard shortcut for image labels. (Char) Defaults to '_'
  IA_Scalable    = IA_Dummy + 29; // Attribute indicates this image is allowed to/can scale its rendering. (LongBool) Defaults to False.
  IA_ActivateKey = IA_Dummy + 30; // Used to get an underscored label shortcut. Useful for labels attached to string gadgets. (Byte) Defaults to 0.
  IA_Screen      = IA_Dummy + 31; // Screen pointer, may be useful/required by certain classes. (PScreen)
  IA_Precision   = IA_Dummy + 32; // Precision value, typically pen precision but may be used for similar custom purposes. (Cardinal)
  // V50:
  SYSIA_WithBorder = IA_FGPen;
  SYSIA_Style      = IA_BGPen;

  SYSISTYLE_NORMAL   = 0;
  SYSISTYLE_GADTOOLS = 1;
// obsolete
  SYSIA_Pens      = IA_Pens;
  IA_ShadowPen    = IA_Dummy + 9;
  IA_HighlightPen = IA_Dummy + 10;

// data values for SYSIA_Size
  SYSISIZE_MEDRES = 0;
  SYSISIZE_LOWRES = 1;
  SYSISIZE_HIRES  = 2;

// SYSIA_Which tag data values: Specifies which system gadget you want an image for. Some numbers correspond to internal Intuition s
  DEPTHIMAGE  = $00;
  ZOOMIMAGE   = $01;
  SIZEIMAGE   = $02;
  CLOSEIMAGE  = $03;
  SDEPTHIMAGE = $05; // screen depth gadget
  LEFTIMAGE   = $0A;
  UPIMAGE     = $0B;
  RIGHTIMAGE  = $0C;
  DOWNIMAGE   = $0D;
  CHECKIMAGE  = $0E;
  MXIMAGE     = $0F; // mutual exclude "button"
  // V39:
  MENUCHECK   = $10; // Menu checkmark image
  AMIGAKEY    = $11; // Menu Amiga-key image

// Data values for IA_FrameType (recognized by FrameIClass)
  FRAME_DEFAULT     = 0; // The standard V37-type frame, which has thin edges.
  FRAME_BUTTON      = 1; // Standard button gadget frames, having thicker sides and nicely edged corners.
  FRAME_RIDGE       = 2; // A ridge such as used by standard string gadgets. You can recess the ridge to get a groove image.
  FRAME_ICONDROPBOX = 3; // A broad ridge which is the standard imagery for areas in AppWindows where icons may be dropped.

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

// image draw states or styles, for IM_DRAW
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
    imp_ContentsBox: PIBox; // input: relative box of contents
    imp_FrameBox: PIBox;    // output: rel. box of encl frame
    imp_DrInfo: PDrawInfo;
    imp_FrameFlags: LongWord;
  end;

const
  FRAMEF_SPECIFY = 1 shl 0;  // Make do with the dimensions of FrameBox provided.

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
    imp_DrInfo: PDrawInfo;
    // these parameters only valid for IM_DRAWFRAME
    imp_Dimensions : record
      Width: SmallInt;
      Height: SmallInt;
    end;
  end;

{ IM_ERASE, IM_ERASEFRAME      }
{ NOTE: This is a subset of impDraw    }

  PimpErase = ^TimpErase;
  TimpErase = record
    MethodID: LongWord;
    imp_RPort: PRastPort;
    imp_Offset: record
     x : SmallInt;
     y : SmallInt;
    end;
    { these parameters only valid for IM_ERASEFRAME }
    imp_Dimensions : record
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
    imp_Dimensions : record
      Width: SmallInt;
      Height: SmallInt;
    end;
  end;


// The IM_DOMAINFRAME method is used to obtain the sizing requirements of an image object within a layout group.
// IM_DOMAINFRAME
  PimpDomainFrame = ^TimpDomainFrame;
  TimpDomainFrame = record
    MethodID: LongWord;
    imp_DrInfo: PDrawInfo;
    imp_RPort: PRastPort;
    imp_Which: LongInt;
    imp_Domain: TIBox;
    imp_Attrs: PTagItem;
  end;
// Accepted vales for imp_Which.
const
  IDOMAIN_MINIMUM = 0;
  IDOMAIN_NOMINAL = 1;
  IDOMAIN_MAXIMUM = 2;

// 'boopsi' pointer class interface
const
  // The following tags are recognized at NewObject() time by pointerclass
  POINTERA_Dummy       = TAG_USER + $39000;
  POINTERA_BitMap      = POINTERA_Dummy + $01; // (PBitmap) Pointer to bitmap to get pointer imagery from.  Bitplane data need not be in chip RAM.
  POINTERA_XOffset     = POINTERA_Dummy + $02; // (LongInt) - X-offset of the pointer hotspot.
  POINTERA_YOffset     = POINTERA_Dummy + $03; // (LongInt) - Y-offset of the pointer hotspot.
  POINTERA_WordWidth   = POINTERA_Dummy + $04; // (LongWord) - designed width of the pointer in words
  POINTERA_XResolution = POINTERA_Dummy + $05; // (LongWord) - one of the POINTERXRESN_ flags below
  POINTERA_YResolution = POINTERA_Dummy + $06; // (LongWord) - one of the POINTERYRESN_ flags below

  // These are the choices for the POINTERA_XResolution attribute which
  // will determine what resolution pixels are used for this pointer.
  POINTERXRESN_DEFAULT   = 0; // (ECS-compatible pointer width) = 70 ns if SUPERHIRES-type mode, 140 ns if not
  POINTERXRESN_140NS     = 1; // (pointer always in 140 ns pixels) = 140 ns always
  POINTERXRESN_70NS      = 2; // (pointer always in 70 ns pixels) = 70 ns always
  POINTERXRESN_35NS      = 3; // (pointer always in 35 ns pixels) = 35 ns always
  POINTERXRESN_SCREENRES = 4; // Same as pixel speed of screen
  POINTERXRESN_LORES     = 5; // (pointer always in lores-like pixels) = 140 ns in 15kHz modes, 70 ns in 31kHz modes
  POINTERXRESN_HIRES     = 6; // (pointer always in hires-like pixels) = 70 ns in 15kHz modes, 35 ns in 31kHz modes

{ These are the choices for the POINTERA_YResolution attribute which
  will determine what vertical resolution is used for this pointer.

  POINTERYRESN_DEFAULT
       = In 15 kHz modes, the pointer resolution will be the same
         as a non-interlaced screen.  In 31 kHz modes, the pointer
         will be doubled vertically.  This means there will be about
         200-256 pointer lines per screen.

  POINTERYRESN_HIGH
  POINTERYRESN_HIGHASPECT
       = Where the hardware/software supports it, the pointer resolution
         will be high.  This means there will be about 400-480 pointer
         lines per screen.  POINTERYRESN_HIGHASPECT also means that
         when the pointer comes out double-height due to hardware/software
         restrictions, its width would be doubled as well, if possible
         (to preserve aspect).

  POINTERYRESN_SCREENRES
  POINTERYRESN_SCREENRESASPECT
       = Will attempt to match the vertical resolution of the pointer
         to the screen's vertical resolution.  POINTERYRESN_SCREENASPECT also
         means that when the pointer comes out double-height due to
         hardware/software restrictions, its width would be doubled as well,
         if possible (to preserve aspect).}

  POINTERYRESN_DEFAULT         = 0;
  POINTERYRESN_HIGH            = 2;
  POINTERYRESN_HIGHASPECT      = 3;
  POINTERYRESN_SCREENRES       = 4;
  POINTERYRESN_SCREENRESASPECT = 5;

// pointer definitions for use with WA_PointerType (OpenWindow() and SetWindowPointer()) (V51)
  POINTERTYPE_NORMAL            = 0;
  POINTERTYPE_BUSY              = 1;
  POINTERTYPE_INVISIBLE         = 2;  // pointer still there, but completly transparent
  POINTERTYPE_DOT               = 3;  // single dot
  POINTERTYPE_WORKING           = 4;  // non blocking pointer signaling app is doing something
  POINTERTYPE_HELP              = 5;  // use when user will get help after clicking in window objects
  POINTERTYPE_AIMING            = 6;  // use for precise operations, such as painting
  POINTERTYPE_SELECTTEXT        = 7;  // use when selecting text
  POINTERTYPE_HANDWRITING       = 8;
  POINTERTYPE_NOTAVAILABLE      = 9;
  POINTERTYPE_VERTICALRESIZE    = 10; // resize operations
  POINTERTYPE_HORIZONTALRESIZE  = 11;
  POINTERTYPE_DIAGONALRESIZE1   = 12;
  POINTERTYPE_DIAGONALRESIZE2   = 13;
  POINTERTYPE_MOVE              = 14; // window move operations, etc
  POINTERTYPE_ALTERNATIVECHOICE = 15;
  POINTERTYPE_SELECTLINK        = 16; // for hyperlinks
  POINTERTYPE_NUMTYPES          = 17;

{ Compatibility note:
 *
 * The AA chipset supports variable sprite width and resolution, but
 * the setting of width and resolution is global for all sprites.
 * When no other sprites are in use, Intuition controls the sprite
 * width and sprite resolution for correctness based on pointerclass
 * attributes specified by the creator of the pointer.  Intuition
 * controls sprite resolution with the VTAG_DEFSPRITERESN_SET tag
 * to VideoControl().  Applications can override this on a per-viewport
 * basis with the VTAG_SPRITERESN_SET tag to VideoControl().
 *
 * If an application uses a sprite other than the pointer sprite,
 * Intuition will automatically regenerate the pointer sprite's image in
 * a compatible width.  This might involve BitMap scaling of the imagery
 * you supply.
 *
 * If any sprites other than the pointer sprite were obtained with the
 * old GetSprite() call, Intuition assumes that the owner of those
 * sprites is unaware of sprite resolution, hence Intuition will set the
 * default sprite resolution (VTAG_DEFSPRITERESN_SET) to ECS-compatible,
 * instead of as requested by the various pointerclass attributes.
 *
 * No resolution fallback occurs when applications use ExtSprites.
 * Such applications are expected to use VTAG_SPRITERESN_SET tag if
 * necessary.
 *
 * NB:  Under release V39, only sprite width compatibility is implemented.
 * Sprite resolution compatibility was added for V40.
 }

type
  PStringExtend = ^TStringExtend;
  TStringExtend = record
    // display specifications
    Font: PTextFont;                   // must be an open Font (not TextAttr)
    Pens: array[0..1] of Byte;         // color of text/background
    ActivePens: array[0..1] of Byte;   // colors when gadget is active
    // edit specifications
    InitialModes: LongWord;            // initial mode flags, below
    EditHook: PHook;                   // IF non-nil, must supply WorkBuffer
    WorkBuffer: STRPTR;                // must be as large as StringInfo.Buffer
    Reserved: array[0..3] of LongWord; // set to 0
  end;

  PSGWork = ^TSGWork;
  TSGWork = record
    // set up when gadget is first activated    }
    Gadget: PGadget;         // the contestant itself
    StringInfo: PStringInfo; // easy access to sinfo
    WorkBuffer: STRPTR;      // intuition's planned result
    PrevBuffer: STRPTR;      // what was there before
    Modes: LongWord;         // current mode
    // modified for each input event
    IEvent: PInputEvent;     // actual event: do not change
    Code: Word;              // character code, IF one Byte
    BufferPos: SmallInt;     // cursor position
    NumChars: SmallInt;
    Actions: LongWord;       // what Intuition will do
    LongInt_: LongInt;       // temp storage for LongInt
    GadgetInfo: PGadgetInfo;
    EditOp: Word;            // from constants below
  end;

{ TSGWork.EditOp - These values indicate what basic type of operation the global editing hook has performed on the string before your gadget's custom
  editing hook gets called.  You do not have to be concerned with the value your custom hook leaves in the EditOp field, only if you write a global editing hook.
  For most of these general edit operations, you'll want to compare the BufferPos and NumChars of the StringInfo (before global editing) and SGWork (after global editing).}
const
  EO_NOOP        = $0001; // did nothing
  EO_DELBACKWARD = $0002; // deleted some chars (maybe 0).
  EO_DELFORWARD  = $0003; // deleted some characters under and in front of the cursor
  EO_MOVECURSOR  = $0004; // moved the cursor
  EO_ENTER       = $0005; // "enter" or "return" key, terminate
  EO_RESET       = $0006; // current Intuition-style undo
  EO_REPLACECHAR = $0007; // replaced one character and (maybe) advanced cursor
  EO_INSERTCHAR  = $0008; // inserted one char into string or added one at end
  EO_BADFORMAT   = $0009; // didn't like the text data, e.g., Bad LONGINT
  EO_BIGCHANGE   = $000A; // unused by Intuition complete or major change to the text, e.g. new string
  EO_UNDO        = $000B; // unused by Intuition  some other style of undo
  EO_CLEAR       = $000C; // clear the string
  EO_SPECIAL     = $000D; // unused by Intuition some operation that doesn't fit into the categories here

  // Mode Flags definitions (ONLY first group allowed as InitialModes)
  SGM_REPLACE    = 1 shl 0; // replace mode please initialize StringInfo with in-range value of BufferPos if you are using SGM_REPLACE mode.
  SGM_FIXEDFIELD = 1 shl 1; // fixed length buffer always set SGM_REPLACE, too
  SGM_NOFILTER   = 1 shl 2; // don't filter control chars
  SGM_EXITHELP   = 1 shl 7; // exit with code = $5F IF HELP hit
  // These Mode Flags are for internal use only
  SGM_NOCHANGE   = 1 shl 3; // no edit changes yet
  SGM_NOWORKB    = 1 shl 4; // Buffer = PrevBuffer
  SGM_CONTROL    = 1 shl 5; // control char escape mode
  SGM_LONGINT    = 1 shl 6; // an intuition LongInt gadget
  // String Gadget Action Flags (put in SGWork.Actions by EditHook)
  SGA_USE        = $1;  // use contents of SGWork
  SGA_END        = $2;  // terminate gadget, code in Code field
  SGA_BEEP       = $4;  // flash the screen for the user
  SGA_REUSE      = $8;  // reuse input event
  SGA_REDISPLAY  = $10; // gadget visuals changed
  SGA_NEXTACTIVE = $20; // Make next possible gadget active.
  SGA_PREVACTIVE = $40; // Make previous possible gadget active.
  // function id for only existing custom string gadget edit hook
  SGH_KEY   = 1; // process editing keystroke
  SGH_CLICK = 2; // process mouse click cursor position

{ Here's a brief summary of how the custom string gadget edit hook works:
 *      You provide a hook in StringInfo.Extension.EditHook.
 *      The hook is called in the standard way with the 'object'
 *      a pointer to SGWork, and the 'message' a pointer to a command
 *      block, starting either with (longword) SGH_KEY, SGH_CLICK,
 *      or something new.
 *
 *      You return 0 if you don't understand the command (SGH_KEY is
 *      required and assumed).  Return non-zero if you implement the
 *      command.
 *
 *   SGH_KEY:
 *      There are no parameters following the command longword.
 *
 *      Intuition will put its idea of proper values in the SGWork
 *      before calling you, and if you leave SGA_USE set in the
 *      SGWork.Actions field, Intuition will use the values
 *      found in SGWork fields WorkBuffer, NumChars, BufferPos,
 *      and LongInt, copying the WorkBuffer back to the StringInfo
 *      Buffer.
 *
 *      NOTE WELL: You may NOT change other SGWork fields.
 *
 *      If you clear SGA_USE, the string gadget will be unchanged.
 *
 *      If you set SGA_END, Intuition will terminate the activation
 *      of the string gadget.  If you also set SGA_REUSE, Intuition
 *      will reuse the input event after it deactivates your gadget.
 *
 *      In this case, Intuition will put the value found in SGWork.Code
 *      into the IntuiMessage.Code field of the IDCMP_GADGETUP message it
 *      sends to the application.
 *
 *      If you set SGA_BEEP, Intuition will call DisplayBeep(); use
 *      this if the user has typed in error, or buffer is full.
 *
 *      Set SGA_REDISPLAY if the changes to the gadget warrant a
 *      gadget redisplay.  Note: cursor movement requires a redisplay.
 *
 *      Starting in V37, you may set SGA_PREVACTIVE or SGA_NEXTACTIVE
 *      when you set SGA_END.  This tells Intuition that you want
 *      the next or previous gadget with GFLG_TABCYCLE to be activated.
 *
 *   SGH_CLICK:
 *      This hook command is called when Intuition wants to position
 *      the cursor in response to a mouse click in the string gadget.
 *
 *      Again, here are no parameters following the command longword.
 *
 *      This time, Intuition has already calculated the mouse position
 *      character cell and put it in SGWork.BufferPos.  The previous
 *      BufferPos value remains in the SGWork.StringInfo.BufferPos.
 *
 *      Intuition will again use the SGWork fields listed above for
 *      SGH_KEY.  One restriction is that you are NOT allowed to set
 *      SGA_END or SGA_REUSE for this command.  Intuition will not
 *      stand for a gadget which goes inactive when you click in it.
 *
 *      You should always leave the SGA_REDISPLAY flag set, since Intuition
 *      uses this processing when activating a string gadget.
 }


// intuition blanker api definitions
const
  BTDPT_BOOLEAN = 1;
  BTDPT_INTEGER = 2;
  BTDPT_CYCLE   = 3;
  BTDPT_FILE    = 4;
  BTDPT_STRING  = 5;
  BTDPT_FONT    = 6;

  BTDMAXSTRLEN = 128;

type
  // this is the generic head for param structure
  PBTDNode = ^TBTDNode;
  PPBTDNode = ^PBTDNode;
  TBTDNode = record
    BN_Tag: Tag;    // ID tag for parameter
    BN_Name: PChar; // parameter name
    BN_Type: LongWord;
  end;

  // struct for boolean parameters
  PBTDBoolean = ^TBTDBoolean;
  TBTDBoolean = record
    BB_Node: TBTDNode;
    BB_Value: LongInt;  // boolean value
  end;

  PBTDInteger = ^TBTDInteger;
  TBTDInteger = record
    BI_Node: TBTDNode;
    BI_Value: LongInt;  // actual value
    BI_Min: LongInt;    // minimum value
    BI_Max: LongInt;    // maximum value
    BI_Slider: LongInt; // set to True to get a slider
    BI_Format: STRPTR;  // optional format template, e.g. "%ld %%"
    BI_Hook: PHook;     // optional 'Stringify' Hook
  end;
  // The 'Stringify' Hook can be used in case BI_Format is not enough. A STRPTR
  // to a buffer is passed in A2 and an ULONG * to the current slider value in A1.
  // The hook must return a STRPTR on success or NULL. If NULL is returned
  // then default formating is used. The available buffer size can be read from
  // value[1], but its guranteed to be at least 512 bytes.

  PBTDCycle = ^TBTDCycle;
  TBTDCycle = record
    BC_Node: TBTDNode;
    BC_Value: LongInt; // selected item
    BC_Labels: PPChar; // #0 terminated array with labels
  end;

  PBTDString = ^TBTDString;
  TBTDString = record
    BS_Node: TBTDNode;
    BS_String: array[0..BTDMAXSTRLEN - 1] of char;
  end;

  PBTDFont = ^TBTDFont;
  TBTDFont = record
    BF_Node: TBTDNode;
    BF_Font: array[0..BTDMAXSTRLEN - 1] of char;
  end;

  PBTDFile = ^TBTDFile;
  TBTDFile = record
    BF_Node: TBTDNode;
    BF_Font: array[0..BTDMAXSTRLEN - 1] of char;
  end;

// struct returned by QueryBlanker()
  PBTDInfo = ^TBTDInfo;
  TBTDInfo = record
    BI_Revision: LongWord; // put BTDI_Revision here
    BI_ID: LongWord;       // 4 char ID for your blanker
    BI_Name: PChar;        // blanker name
    BI_Description: PChar; // blanker description
    BI_Author: PChar;      // blanker author(s)
    BI_Flags: LongWord;    // blanker requirements
    BI_Params: PPBTDNode;  // #0 terminated array with params
  end;

const
  BTDI_Revision = 7; // you must ALWAYS use this symbol!

  BTDIF_3DBlanker = 1 shl 0; // set this if your blanker needs a screen format supported by 3d drivers
  BTDIF_Fast3D    = 1 shl 1; // set this if your blanker needs 3d hw accelerated driver
  BTDIF_DoNotWait = 1 shl 2; // set this if your blanker will control fps on it's own (by calling WaitBOVP, etc)

{ NOTE: intuition will always try to pick the best screen mode for your blanker depending on requirements,
  that doesn't mean you will always get a 3d supported screen, in that case your blanker must return
  BTDERR_Display error code
  Double/Triple Buffering
  In case you want double or triple buffered display, please ensure that BDI_Screen is set and
  use standard intuition screen buffering routines as described in AllocScreenBuffer.
  Be sure to free all screen buffers in your ExitBlanker implementation.}

type
  PBTDDrawInfo = ^TBTDDrawInfo;
  TBTDDrawInfo = record
    BDI_RPort: PRastPort; // RastPort with or without layer, the bitmap will never be a CLUT one!
    BDI_Screen: PScreen;  // In case your renderer needs to know the gfx card, call WaitBOVP. might be nil!
    BDI_Left: LongInt;    // position and dimmensions of your draw area
    BDI_Top: LongInt;     // NOTE: starting with BTDI_Revision 7 this will always be 0,0,screenw,screenh
    BDI_Width: LongInt;
    BDI_Height: LongInt;
  end;

const
  BTDERR_Size    = 1; // BTD_Error code: allocated draw space too small
  BTDERR_Memory  = 2; // BTD_Error code: not enough memory
  BTDERR_Display = 3; // BTD_Error code: unsupported display mode

  BTD_Dummy       = TAG_USER + $425444; // 'BTD'
  BTD_DrawInfo    = BTD_Dummy + 1;
  BTD_Error       = BTD_Dummy + 2;
  BTD_PreviewMode = BTD_Dummy + 4; // True if ran from Blanker.mprefs

  BTD_Client      = BTD_Dummy + 100;

  // intuition getdrawinfoattr definitions (V50)
const
  GDIA_Color       = $00100000;
  GDIA_Pen         = $00200000;
  GDIA_Version     = $00300000;
  GDIA_DirectColor = $00400000;
  GDIA_NumPens     = $00500000;
  GDIA_Font        = $00600000;
  GDIA_Depth       = $00700000;
  GDIA_ResolutionX = $00800000;
  GDIA_ResolutionY = $00900000;
  GDIA_CheckMark   = $00A00000;
  GDIA_MenuKey     = $00B00000;


  DRIPEN_DETAIL        = DETAILPEN;
  DRIPEN_BLOCK         = BLOCKPEN;
  DRIPEN_TEXT          = TEXTPEN;
  DRIPEN_SHINE         = SHINEPEN;
  DRIPEN_SHADOW        = SHADOWPEN;
  DRIPEN_FILL          = FILLPEN;
  DRIPEN_FILLTEXT      = FILLTEXTPEN;
  DRIPEN_BACKGROUND    = BACKGROUNDPEN;
  DRIPEN_HIGHLIGHTTEXT = HIGHLIGHTTEXTPEN;
  DRIPEN_BARDETAIL     = BARDETAILPEN;
  DRIPEN_BARBLOCK      = BARBLOCKPEN;
  DRIPEN_BARTRIM       = BARTRIMPEN;

  DRIPEN_HALFSHINE  = $000C;
  DRIPEN_HALFSHADOW = $000D;
  DRIPEN_NUMDRIPENS = $000E;

// intuition v50 definitions

  // new SYSIA_Which values (added some padding there)
  ICONIFYIMAGE    = $12;
  LOCKIMAGE       = $13;
  MUIIMAGE        = $14;
  POPUPIMAGE      = $15;
  SNAPSHOTIMAGE   = $16;
  JUMPIMAGE       = $17;
  MENUTOGGLEIMAGE = $19;
  SUBMENUIMAGE    = $1A;

  // Flags for ExtraTitlebarGadgets!!!
  ETG_ICONIFY  = $1;  // MUI iconify gadget
  ETG_LOCK     = $2;  // lock gadget from Magellan
  ETG_MUI      = $4;  // MUI prefs gadget
  ETG_POPUP    = $8;  // popup menu gadget
  ETG_SNAPSHOT = $10; // MUI snapshot gadget
  ETG_JUMP     = $20; // MUI screen jump gadget

  // Extra gadget ID's
  ETI_Dummy    = $FFD0; // you can change this base with WA_ExtraGadgetsStartID!
  ETI_Iconify  = ETI_Dummy;
  ETI_Lock     = ETI_Dummy + 1;
  ETI_MUI      = ETI_Dummy + 2;
  ETI_PopUp    = ETI_Dummy + 3;
  ETI_Snapshot = ETI_Dummy + 4;
  ETI_Jump     = ETI_Dummy + 5;

  // for use with custom ETI_Dummy base...
  ETD_Iconify  = 0;
  ETD_Lock     = 1;
  ETD_MUI      = 2;
  ETD_PopUp    = 3;
  ETD_Snapshot = 4;
  ETD_Jump     = 5;

  // GetSkinInfo defines
  SI_Dummy = $8000000;
  // Window border size
  SI_BorderTop            = SI_Dummy + 1;
  SI_BorderTopTitle       = SI_Dummy + 2; // when you want a window with title/titlebar gadgets
  SI_BorderLeft           = SI_Dummy + 3;
  SI_BorderRight          = SI_Dummy + 4; // std size of window border
  SI_BorderRightSize      = SI_Dummy + 5; // border with size gadgets
  SI_BorderBottom         = SI_Dummy + 6;
  SI_BorderBottomSize     = SI_Dummy + 7;
  SI_ScreenTitlebarHeight = SI_Dummy + 8; // real height of screen titlebar = no need to add 1 pixel there!;
                                          //   please use this instead of reading from struct Screen
                                          //   do note that this returns 0 for invisible or disappearing
                                          //   titlebars so it may not be what you need */
  // Titlebar gadgets positions/sizes
  SI_RightPropWidth       = SI_Dummy + 10;
  SI_BottomPropHeight     = SI_Dummy + 11;
  SI_RightArrowBox        = SI_Dummy + 12; // space used by arrows on right titlebar
  SI_BottomArrowBox       = SI_Dummy + 13;
  // Other Information
  SI_Shadows              = SI_Dummy + 14; // returns True if the skin supports window shadows

  // window action methods, see the autodoc for detailed descriptions
  WAC_DUMMY               = $0001;
  WAC_HIDEWINDOW          = WAC_DUMMY;
  WAC_SHOWWINDOW          = WAC_DUMMY + 1;
  WAC_SENDIDCMPCLOSE      = WAC_DUMMY + 2;
  WAC_MOVEWINDOW          = WAC_DUMMY + 3;
  WAC_SIZEWINDOW          = WAC_DUMMY + 4;
  WAC_CHANGEWINDOWBOX     = WAC_DUMMY + 5;
  WAC_WINDOWTOFRONT       = WAC_DUMMY + 6;
  WAC_WINDOWTOBACK        = WAC_DUMMY + 7;
  WAC_ZIPWINDOW           = WAC_DUMMY + 8;
  WAC_MOVEWINDOWINFRONTOF = WAC_DUMMY + 9;
  WAC_ACTIVATEWINDOW      = WAC_DUMMY + 10;
  // V51
  WAC_MAXIMIZEWINDOW        = WAC_DUMMY + 11;
  WAC_MINIMIZEWINDOW        = WAC_DUMMY + 12;
  WAC_RESTOREINITIALSIZEPOS = WAC_DUMMY + 13;
  WAC_OPENMENU              = WAC_DUMMY + 14;
  WAC_FAMILYTOFRONT         = WAC_DUMMY + 15;
  WAC_FAMILYTOBACK          = WAC_DUMMY + 16;

  // window action tags
  WAT_DUMMY             = TAG_USER;
  WAT_MOVEWINDOWX       = WAT_DUMMY + 1;
  WAT_MOVEWINDOWY       = WAT_DUMMY + 2;
  WAT_SIZEWINDOWX       = WAT_DUMMY + 3;
  WAT_SIZEWINDOWY       = WAT_DUMMY + 4;
  WAT_WINDOWBOXLEFT     = WAT_DUMMY + 5;
  WAT_WINDOWBOXTOP      = WAT_DUMMY + 6;
  WAT_WINDOWBOXWIDTH    = WAT_DUMMY + 7;
  WAT_WINDOWBOXHEIGHT   = WAT_DUMMY + 8;
  WAT_MOVEWBEHINDWINDOW = WAT_DUMMY + 9;

type
  PTransparencyMessage = ^TTransparencyMessage;
  TTransparencyMessage = record
    Layer: PLayer;         // the layer you're asked to provide transparency for
    Region: PRegion;       // create transparency in this region
    NewBounds: PRectangle; // current layer boundaries
    OldBounds: PRectangle; // old layer boundaries, useful after layer resize
  end;

  // TransparencyControl tags and methods
const
  TRANSPCONTROLMETHOD_INSTALLREGION      = $1; // Installs a new region, requires that you pass TRANSPCONTROL_REGION in tags. Setting this tag
                                               // to nil or not passing it at all removes currently installed. Passing TRANSPCONTROL_OLDREGION
                                               // will write old region address to storagePtr passed in tag^.ti_Data. Installing a region removes regionhook!
  TRANSPCONTROLMETHOD_INSTALLREGIONHOOK  = $2; // Similar to TRANSPCONTROLMETHOD_INSTALLREGION
  TRANSPCONTROLMETHOD_UPDATETRANSPARENCY = $3; // Calls your transparency hook to allow you to change the transparency whenever you want.
                                               // This has no effect with transparency region installed.
  TRANSPCONTROL_DUMMY         = TAG_USER;
  TRANSPCONTROL_REGION        = TRANSPCONTROL_DUMMY + 1;
  TRANSPCONTROL_REGIONHOOK    = TRANSPCONTROL_DUMMY + 2;
  TRANSPCONTROL_OLDREGION     = TRANSPCONTROL_DUMMY + 3;
  TRANSPCONTROL_OLDREGIONHOOK = TRANSPCONTROL_DUMMY + 4;
// requester extensions

  { Please pass this structure to EasyRequestArgs or BuildEasyRequest
    Note that the extended structure is recognised by es_StructSize field!
    You MUST set it to sizeof(struct ExtEasyStruct) in order to use the tag
    field, setting size to NULL or sizeof(struct EasyStruct) disables the use
    of this extension!
    please also note that no logos are displayed on CLUT screens (<=8 bitplanes)}
type
  PExtEasyStruct = ^TExtEasyStruct;
  TExtEasyStruct = record
    es_StructSize: LongWord;
    es_Flags: LongWord;
    es_Title: PChar;
    es_TextFormat: PChar;
    es_GadgetFormat: PChar;
    es_Tags: PTagItem;
  end;
const
  EES_Dummy        = TAG_USER;
  EES_ActiveButton = EES_Dummy + 1; // the button which will be hilighted when requester pops up, uses 1,2,..,n,0 numbering!
  EES_DosLogo      = EES_Dummy + 2;
  EES_DiskBased    = EES_Dummy + 3; // if True, the requesters may use diskbased components. if FALSE, no I/O will be done when displaying the requester. defaults to True

  DOSLOGO_DISKINSERT       = 1;
  DOSLOGO_DISKNOTVALIDATED = 2;
  DOSLOGO_DISKPROTECTED    = 3;
  DOSLOGO_DISKFULL         = 4;
  DOSLOGO_NOTADOSDISK      = 5;
  DOSLOGO_NODISK           = 6;
  DOSLOGO_DISKBUSY         = 7;
  DOSLOGO_DISKERROR        = 8;
  LOGO_CRASH               = 9;

  // GetMonitorList tags
  GMLA_Dummy = TAG_USER + $4000;
  GMLA_DisplayID = GMLA_Dummy + 1; // LongWord, finds the monitor related to the given display ID, only the
                                   // monitor(s) matching the provided DisplayIDs will be added to the list

// intuition gadget class definitions (V51)

//  Introduced in intuition.library 51. All attributes are *G*. Pointers valid
// until the message is replied.
// Every PIntuiMessage can be treated as an PObject_
  IMSGA_Dummy             = TAG_USER + $60000;
  IMSGA_Class             = IMSGA_Dummy + 1; // ULONG
  IMSGA_Code              = IMSGA_Dummy + 3; // ULONG
  IMSGA_Qualifier         = IMSGA_Dummy + 4; // ULONG
  IMSGA_IAddress          = IMSGA_Dummy + 5; // ULONG
  IMSGA_MouseX            = IMSGA_Dummy + 6; // LONG
  IMSGA_MouseY            = IMSGA_Dummy + 7; // LONG
  IMSGA_Seconds           = IMSGA_Dummy + 8; // ULONG
  IMSGA_Micros            = IMSGA_Dummy + 9; // ULONG
  IMSGA_IDCMPWindow       = IMSGA_Dummy + 10; // PWindow
  IMSGA_RawMouseX         = IMSGA_Dummy + 11; // LONG, raw, unaccelerated delta
  IMSGA_RawMouseY         = IMSGA_Dummy + 12; // LONG
  IMSGA_UCS4              = IMSGA_Dummy + 13; // ULONG, UCS4

  IMSGM_NewCopy           = $600;
// Allocates a new copy of the IntuiMessage in question. You may use the
// object after the original object has been replied. Delete the copy with
// the DisposeObject call. Returns PObject_

{$ifndef INTUI_V36_NAMES_ONLY}

//* Gadget Type names: */

  GTYPEMASK    = GTYP_GTYPEMASK;
  CUSTOMGADGET = GTYP_CUSTOMGADGET;
  STRGADGET    = GTYP_STRGADGET;
  PROPGADGET   = GTYP_PROPGADGET;
  GADGET0002   = GTYP_GADGET0002;
  BOOLGADGET   = GTYP_BOOLGADGET;
  IntuiCLOSE   = GTYP_CLOSE;
  SDRAGGING    = GTYP_SDRAGGING;
  WDRAGGING    = GTYP_WDRAGGING;
  SIZING       = GTYP_SIZING;
  REQGADGET    = GTYP_REQGADGET;
  GZZGADGET    = GTYP_GZZGADGET;
  SCRGADGET    = GTYP_SCRGADGET;
  SYSGADGET    = GTYP_SYSGADGET;
  GADGETTYPE   = GTYP_GADGETTYPE;
// Gadget Flags names:
  LABELIMAGE   = GFLG_LABELIMAGE;
  LABELSTRING  = GFLG_LABELSTRING;
  LABELITEXT   = GFLG_LABELITEXT;
  LABELMASK    = GFLG_LABELMASK;
  GADGDISABLED = GFLG_DISABLED;
  SELECTED     = GFLG_SELECTED;
  GRELHEIGHT   = GFLG_RELHEIGHT;
  GRELWIDTH    = GFLG_RELWIDTH;
  GRELRIGHT    = GFLG_RELRIGHT;
  GRELBOTTOM   = GFLG_RELBOTTOM;
  GADGIMAGE    = GFLG_GADGIMAGE;
  GADGHNONE    = GFLG_GADGHNONE;
  GADGHIMAGE   = GFLG_GADGHIMAGE;
  GADGHBOX     = GFLG_GADGHBOX;
  GADGHCOMP    = GFLG_GADGHCOMP;
  GADGHIGHBITS = GFLG_GADGHIGHBITS;
// Gadget Activation flag names:
  ACTIVEGADGET  = GACT_ACTIVEGADGET;
  STRINGEXTEND  = GACT_STRINGEXTEND;
  ALTKEYMAP     = GACT_ALTKEYMAP;
  IntuiLONGINT  = GACT_LONGINT;
  STRINGRIGHT   = GACT_STRINGRIGHT;
  STRINGCENTER  = GACT_STRINGCENTER;
  STRINGLEFT    = GACT_STRINGLEFT;
  BOOLEXTEND    = GACT_BOOLEXTEND;
  TOGGLESELECT  = GACT_TOGGLESELECT;
  BORDERSNIFF   = GACT_BORDERSNIFF;
  BOTTOMBORDER  = GACT_BOTTOMBORDER;
  TOPBORDER     = GACT_TOPBORDER;
  LEFTBORDER    = GACT_LEFTBORDER;
  RIGHTBORDER   = GACT_RIGHTBORDER;
  FOLLOWMOUSE   = GACT_FOLLOWMOUSE;
  ENDGADGET     = GACT_ENDGADGET;
  GADGIMMEDIATE = GACT_IMMEDIATE;
  RELVERIFY     = GACT_RELVERIFY;
// Window Flags names:
  HASZOOM       = WFLG_HASZOOM;
  ZOOMED        = WFLG_ZOOMED;
  VISITOR       = WFLG_VISITOR;
  NW_EXTENDED   = WFLG_NW_EXTENDED;
  WINDOWTICKED  = WFLG_WINDOWTICKED;
  WBENCHWINDOW  = WFLG_WBENCHWINDOW;
  WINDOWREFRESH = WFLG_WINDOWREFRESH;
  NOCAREREFRESH = WFLG_NOCAREREFRESH;
  RMBTRAP       = WFLG_RMBTRAP;
  MENUSTATE     = WFLG_MENUSTATE;
  INREQUEST     = WFLG_INREQUEST;
  WINDOWACTIVE  = WFLG_WINDOWACTIVE;
  ACTIVATE      = WFLG_ACTIVATE;
  BORDERLESS    = WFLG_BORDERLESS;
  GIMMEZEROZERO = WFLG_GIMMEZEROZERO;
  IntuiREPORTMOUSE   = WFLG_REPORTMOUSE;
  BACKDROP      = WFLG_BACKDROP;
  OTHER_REFRESH = WFLG_OTHER_REFRESH;
  SUPER_BITMAP  = WFLG_SUPER_BITMAP;
  SIMPLE_REFRESH= WFLG_SIMPLE_REFRESH;
  SMART_REFRESH = WFLG_SMART_REFRESH;
  REFRESHBITS   = WFLG_REFRESHBITS;
  SIZEBBOTTOM   = WFLG_SIZEBBOTTOM;
  SIZEBRIGHT    = WFLG_SIZEBRIGHT;
  WINDOWCLOSE   = WFLG_CLOSEGADGET;
  WINDOWDEPTH   = WFLG_DEPTHGADGET;
  WINDOWDRAG    = WFLG_DRAGBAR;
  WINDOWSIZING  = WFLG_SIZEGADGET;
// IDCMP class names:
  LONELYMESSAGE  = IDCMP_LONELYMESSAGE;
  CHANGEWINDOW   = IDCMP_CHANGEWINDOW;
  MENUHELP       = IDCMP_MENUHELP;
  IDCMPUPDATE    = IDCMP_IDCMPUPDATE;
  INTUITICKS     = IDCMP_INTUITICKS;
  VANILLAKEY     = IDCMP_VANILLAKEY;
  DELTAMOVE      = IDCMP_DELTAMOVE;
  INACTIVEWINDOW = IDCMP_INACTIVEWINDOW;
  ACTIVEWINDOW   = IDCMP_ACTIVEWINDOW;
  WBENCHMESSAGE  = IDCMP_WBENCHMESSAGE;
  DISKREMOVED    = IDCMP_DISKREMOVED;
  DISKINSERTED   = IDCMP_DISKINSERTED;
  NEWPREFS       = IDCMP_NEWPREFS;
  MENUVERIFY     = IDCMP_MENUVERIFY;
  REQCLEAR       = IDCMP_REQCLEAR;
  REQVERIFY      = IDCMP_REQVERIFY;
  RAWKEY         = IDCMP_RAWKEY;
  IntuiCLOSEWINDOW = IDCMP_CLOSEWINDOW;
  MENUPICK       = IDCMP_MENUPICK;
  REQSET         = IDCMP_REQSET;
  GADGETUP       = IDCMP_GADGETUP;
  GADGETDOWN     = IDCMP_GADGETDOWN;
  MOUSEMOVE      = IDCMP_MOUSEMOVE;
  MOUSEBUTTONS   = IDCMP_MOUSEBUTTONS;
  REFRESHWINDOW  = IDCMP_REFRESHWINDOW;
  NEWSIZE        = IDCMP_NEWSIZE;
  SIZEVERIFY     = IDCMP_SIZEVERIFY;
{$endif} // not INTUI_V36_NAMES_ONLY

const
  // Length of array returned by MA_PixelFormats
  MONITOR_MAXPIXELFORMATS = 14;

  // Attributes
  MA_Dummy               = TAG_USER;
  MA_MonitorName         = MA_Dummy + 1;  // [..G] STRPTR Monitor name
  MA_Manufacturer        = MA_Dummy + 2;  // [..G] STRPTR Hardware manufacturer string
  MA_ManufacturerID      = MA_Dummy + 3;  // [..G] LongWord
  MA_ProductID           = MA_Dummy + 4;  // [..G] LongWord
  MA_MemorySize          = MA_Dummy + 5;  // [..G] LongWord Video card memory size
  MA_PixelFormats        = MA_Dummy + 6;  // [..G] PLongWord Pixelformat support flags
  MA_TopLeftMonitor      = MA_Dummy + 7;  // [.SG] PObject_  Monitor placed in a position relative to the current one
  MA_TopMiddleMonitor    = MA_Dummy + 8;  // [.SG] PObject_
  MA_TopRightMonitor     = MA_Dummy + 9;  // [.SG] PObject_
  MA_MiddleLeftMonitor   = MA_Dummy + 10; // [.SG] PObject_
  MA_MiddleRightMonitor  = MA_Dummy + 11; // [.SG] PObject_
  MA_BottomLeftMonitor   = MA_Dummy + 12; // [.SG] PObject_
  MA_BottomMiddleMonitor = MA_Dummy + 13; // [.SG] PObject_
  MA_BottomRightMonitor  = MA_Dummy + 14; // [.SG] PObject_
  MA_GammaControl        = MA_Dummy + 15; // [..G] LongBool Whether gamma control is supported
  MA_PointerType         = MA_Dummy + 16; // [..G] LongWord Supported pointer types
  MA_DriverName          = MA_Dummy + 17; // [..G] STRPTR Driver name
  MA_MemoryClock         = MA_Dummy + 18; // [..G] LongWord Video memory clock in Hz, 0 if unknown
  MA_BacklightControl    = MA_Dummy + 19; //       LongBool. Returns TRUE if lcd backlight level control is available
  MA_BacklightLevelSteps = MA_Dummy + 20; //       LongWord. Returns the number of steps MA_BacklightLevel supports
  MA_BacklightLevel      = MA_Dummy + 21; //       LongWord. Changes the backlight level. Allowed values are from 0 (backlight disabled) up to MA_BacklightLevelSteps - 1

  //* Pointer type flags */
  PointerType_3Plus1 = $0001; // color 0 transparent, 1-3 visible
  PointerType_2Plus1 = $0002; // color 0 transparent, 2-3 visible, 1 undefined/clear/inverse
  PointerType_ARGB   = $0004; // Direct color alpha-blended bitmap pointer

  // Methods
  MM_GetRootBitMap         = $401; // Reserved
  MM_Query3DSupport        = $402; // Ask for 3D acceleration support for given pixelformat
  MM_GetDefaultGammaTables = $403; // Get default gamma correction table
  MM_GetDefaultPixelFormat = $404; // Ask for preferred pixelformat for given depth (-1 = unsupported depth)
  MM_GetPointerBounds      = $405; // Ask for maximum supported mouse pointer size
  MM_RunBlanker            = $406; // Start screensaver for this monitor
  MM_EnterPowerSaveMode    = $407; // Start power saving mode
  MM_ExitBlanker           = $408; // Stop screensaver or power saving mode
  MM_Authorization         = $409; // Opens the user password authorization screen, added in 51.68

type
  PmsGetRootBitMap = ^TmsGetRootBitMap;
  TmsGetRootBitMap = record
    MethodID: LongWord;
    PixelFormat: LongWord;
    Store: ^PBitMap;
  end;

  PmsQuery3DSupport = ^TmsQuery3DSupport;
  TmsQuery3DSupport = record
    MethodID: LongWord;
    PixelFormat: LongWord;
    Store: PLongWord;
  end;
const
  MSQUERY3D_UNKNOWN  = 0; // Unsupported pixelformat or other error
  MSQUERY3D_NODRIVER = 1; // No 3D support available
  MSQUERY3D_SWDRIVER = 2; // Software 3D support available
  MSQUERY3D_HWDRIVER = 3; // Hardware accelerated 3D available
type
  PmsGetDefaultGammaTables = ^TmsGetDefaultGammaTables;
  TmsGetDefaultGammaTables = record
    MethodID: LongWord;
    Red: PByte;         // Optional pointers to 256-byte arrays to fill in
    Green: PByte;
    Blue: PByte;
  end;

  PmsGetDefaultPixelFormat = ^TmsGetDefaultPixelFormat;
  TmsGetDefaultPixelFormat = record
    MethodID: LongWord;
    Depth: LongWord;
    Store: PLongWord;
  end;

  PmsGetPointerBounds = ^TmsGetPointerBounds;
  TmsGetPointerBounds = record
    MethodID: LongWord;
    PointerType: LongWord;
    Width: PLongWord;
    Height: PLongWord;
  end;

  PmsSetDefaultGammaTables = PmsGetDefaultGammaTables;
  TmsSetDefaultGammaTables = TmsGetDefaultGammaTables;

// intuition screenbar plugin definitions
const
  MUISERIALNO_INTUITION = $FECF;
  TAGBASE_SCREENBAR     = (TAG_USER or (MUISERIALNO_INTUITION shl 16)) + 3000;

  // redefinition from mui.pas
  MUIM_UpdateConfig = $8042b0a9;

  MUIA_Screenbar_DisplayedName  = TAGBASE_SCREENBAR + 2; // Returns a CONST_STRPTR containing a displayable name of the sbar. This is optional. If missing, the class name will be used. You should generally localise this string. The result will be copied.
  MUIA_Screenbar_DisplayedImage = TAGBASE_SCREENBAR + 3; // Returns an image object that will be used to draw the image in the Settings window. This is exactly
    // the same as implementing the MCC_Query(2) call in your MCC. The object will be disposed when it's no longer needed, no later than the last instance of the class is disposed
  // In order to support user preferences in your screenbar plugin, implement all of the following methods:
  MUIM_Screenbar_BuildSettingsPanel = TAGBASE_SCREENBAR + 20; // Builds a settings panel, inheriting from MUIC_Mccprefs and returns the pointer to the prefs object. All classes must implement this, even if it's just to hold the (C) info.
  MUIM_Screenbar_KnowsConfigItem    = TAGBASE_SCREENBAR + 21; // Return True if msg^.cfgid is one of your prefs CFGIDs, otherwise False
  MUIM_Screenbar_DefaultConfigItem  = TAGBASE_SCREENBAR + 22; // Return a default value for a cfgid
  MUIM_Screenbar_UpdateConfigItem   = MUIM_UpdateConfig;      // This method will be called whenever preferences are updated
  MUIM_Screenbar_Lock               = TAGBASE_SCREENBAR + 24; // Locks the screenbar so that it does not disappear when you need it you should generally always lock it if you pop a context menu up, open a popup window, etc.
  MUIM_Screenbar_Unlock             = TAGBASE_SCREENBAR + 25;
  MUIM_Screenbar_Signal             = TAGBASE_SCREENBAR + 26; // Implement this if you need to handle signals in your sbar plugin. The method is called on all sbar instances when a common signal
    // arrives to the application controlling the screenbars. Check screens.h to find out how to get the common signal bit.
type
  PMUIP_Screenbar_BuildSettingsPanel = ^TMUIP_Screenbar_BuildSettingsPanel;
  TMUIP_Screenbar_BuildSettingsPanel = record
    ID: LongWord;
  end;

  PMUIP_Screenbar_KnowsConfigItem = ^TMUIP_Screenbar_KnowsConfigItem;
  TMUIP_Screenbar_KnowsConfigItem = record
    id: LongWord;
    CfgID: LongWord;
  end;

  PMUIP_Screenbar_DefaultConfigItem = ^TMUIP_Screenbar_DefaultConfigItem;
  TMUIP_Screenbar_DefaultConfigItem = record
    id: LongWord;
    CfgID: LongWord;
  end;

  PMUIP_UpdateConfig = PMUIP_Screenbar_DefaultConfigItem;
  TMUIP_UpdateConfig = TMUIP_Screenbar_DefaultConfigItem;

  PMUIP_Screenbar_Lock = ^TMUIP_Screenbar_Lock;
  TMUIP_Screenbar_Lock = record
    ID: LongWord;
  end;

  PMUIP_Screenbar_Unlock = ^TMUIP_Screenbar_Unlock;
  TMUIP_Screenbar_Unlock = record
    ID: LongWord;
  end;

  PMUIP_Screenbar_Signal = ^TMUIP_Screenbar_Signal;
  TMUIP_Screenbar_Signal = record
    ID: LongWord;
  end;

const
  // ScreenbarControl tags
  SBCT_Dummy           = TAG_USER + $60500;
  SBCT_InstallPlugin   = SBCT_Dummy + 1; // PMUI_CustomClass,mcc_Class^.cl_ID must contain a valid name with ascii letters only
  SBCT_UninstallPlugin = SBCT_Dummy + 2; // PMUI_CustomClass

const
  INTUITIONNAME: PChar = 'intuition.library';

var
  Intuitionbase: PIntuitionBase = nil;

procedure OpenIntuition; SysCall IntuitionBase 030;
procedure Intuition(IEvent: PInputEvent location 'a0'); SysCall IntuitionBase 036;
function AddGadget(Window: PWindow location 'a0'; Gadget: PGadget location 'a1'; Position: LongWord location 'd0'): Word; SysCall IntuitionBase 042;
function ClearDMRequest(Window: PWindow location 'a0'): LongBool; SysCall IntuitionBase 048;
procedure ClearMenuStrip(Window: PWindow location 'a0'); SysCall IntuitionBase 054;
procedure ClearPointer(window: PWindow location 'a0'); SysCall IntuitionBase 060;
function CloseScreen(screen: PScreen location 'a0'): LongBool; SysCall IntuitionBase 066;
procedure CloseWindow(window: PWindow location 'a0'); SysCall IntuitionBase 072;
function CloseWorkBench: LongInt; SysCall IntuitionBase 078;
procedure CurrentTime(var seconds: LongWord location 'a0'; var micros: LongWord location 'a1'); SysCall IntuitionBase 084;
function DisplayAlert(AlertNumber: LongWord location 'd0'; String1: PChar location 'a0'; Height: LongWord location 'd1'): LongBool; SysCall IntuitionBase 090;
procedure DisplayBeep(Screen: PScreen location 'a0'); SysCall IntuitionBase 096;
function DoubleClick(SSeconds: LongWord location 'd0'; SMicros: LongWord location 'd1'; CSeconds: LongWord location 'd2'; CMicros: LongWord location 'd3'): LongBool; SysCall IntuitionBase 102;
procedure DrawBorder(Rp: PRastPort location 'a0'; Border: PBorder location 'a1'; LeftOffset: LongInt location 'd0'; TopOffset: LongInt location 'd1'); SysCall IntuitionBase 108;
procedure DrawImage(Rp: PRastPort location 'a0'; Image: PImage location 'a1'; LeftOffset: LongInt location 'd0'; TopOffset: LongInt location 'd1'); SysCall IntuitionBase 114;
procedure EndRequest(Requester: PRequester location 'a0'; Window: PWindow location 'a1'); SysCall IntuitionBase 120;
function GetDefPrefs(Preferences: PPreferences location 'a0'; size: LongInt location 'd0'): PPreferences; SysCall IntuitionBase 126;
function GetPrefs(Preferences: PPreferences location 'a0'; Size: LongInt location 'd0'): PPreferences; SysCall IntuitionBase 132;
procedure InitRequester(Requester: PRequester location 'a0'); SysCall IntuitionBase 138;
function ItemAddress(MenuStrip: PMenu location 'a0'; MenuNumber: LongWord location 'd0'): PMenuItem; SysCall IntuitionBase 144;
function ModifyIDCMP(Window: PWindow location 'a0'; Flags: LongWord location 'd0'): LongBool; SysCall IntuitionBase 150;
procedure ModifyProp(Gadget: PGadget location 'a0'; Window: PWindow location 'a1'; Requester: PRequester location 'a2'; Flags: LongWord location 'd0'; HorizPot: LongWord location 'd1'; VertPot: LongWord location 'd2'; HorizBody: LongWord location 'd3'; VertBody: LongWord location 'd4'); SysCall IntuitionBase 156;
procedure MoveScreen(Screen: PScreen location 'a0'; Dx: LongInt location 'd0'; Dy: LongInt location 'd1'); SysCall IntuitionBase 162;
procedure MoveWindow(Window: PWindow location 'a0'; Dx: LongInt location 'd0'; Dy: LongInt location 'd1'); SysCall IntuitionBase 168;
procedure OffGadget(Gadget: pGadget location 'a0'; Window: PWindow location 'a1'; Requester: pRequester location 'a2'); SysCall IntuitionBase 174;
procedure OffMenu(Window: PWindow location 'a0'; MenuNumber: LongWord location 'd0'); SysCall IntuitionBase 180;
procedure OnGadget(Gadget: PGadget location 'a0'; Window: PWindow location 'a1'; Requester: PRequester location 'a2'); SysCall IntuitionBase 186;
procedure OnMenu(Window: PWindow location 'a0'; MenuNumber: LongWord location 'd0'); SysCall IntuitionBase 192;
function OpenScreen(NewScreen: PNewScreen location 'a0'): PScreen; SysCall IntuitionBase 198;
function OpenWindow(NewWindow: PNewWindow location 'a0'): PWindow; SysCall IntuitionBase 204;
function OpenWorkBench: LongWord; SysCall IntuitionBase 210;
procedure PrintIText(Rp: PRastPort location 'a0'; IText: PIntuiText location 'a1'; Left: LongInt location 'd0'; Top: LongInt location 'd1'); SysCall IntuitionBase 216;
procedure RefreshGadgets(Gadgets: PGadget location 'a0'; Window: PWindow location 'a1'; Requester: PRequester location 'a2'); SysCall IntuitionBase 222;
function RemoveGadget(Window: PWindow location 'a0'; Gadget: PGadget location 'a1'): Word; SysCall IntuitionBase 228;

procedure ReportMouse(Flag: LongInt location 'd0'; Window: PWindow location 'a0'); SysCall IntuitionBase 234;
function Request(Requester: PRequester location 'a0'; Window: PWindow location 'a1'): LongBool; SysCall IntuitionBase 240;
procedure ScreenToBack(Screen: PScreen location 'a0'); SysCall IntuitionBase 246;
procedure ScreenToFront(Screen: PScreen location 'a0'); SysCall IntuitionBase 252;
function SetDMRequest(Window: PWindow location 'a0'; Requester: PRequester location 'a1'): LongBool; SysCall IntuitionBase 258;
function SetMenuStrip(Window: PWindow location 'a0'; Menu: PMenu location 'a1'): LongBool; SysCall IntuitionBase 264;
procedure SetPointer(Window: PWindow location 'a0'; Pointer: PWord location 'a1'; Height: LongInt location 'd0'; Width: LongInt location 'd1'; XOffset: LongInt location 'd2'; YOffset: LongInt location 'd3'); SysCall IntuitionBase 270;
procedure SetWindowTitles(Window: PWindow location 'a0'; WindowTitle: PChar location 'a1'; ScreenTitle: PChar location 'a2'); SysCall IntuitionBase 276;
procedure ShowTitle(Screen: PScreen location 'a0'; ShowIt: LongInt location 'd0'); SysCall IntuitionBase 282;
procedure SizeWindow(Window: PWindow location 'a0'; Dx: LongInt location 'd0'; Dy: LongInt location 'd1'); SysCall IntuitionBase 288;
function ViewAddress: PView; SysCall IntuitionBase 294;
function ViewPortAddress(Window: PWindow location 'a0'): PViewPort; SysCall IntuitionBase 300;
procedure WindowToBack(Window: PWindow location 'a0'); SysCall IntuitionBase 306;
procedure WindowToFront(Window: PWindow location 'a0'); SysCall IntuitionBase 312;
function WindowLimits(Window: PWindow location 'a0'; WidthMin: LongInt location 'd0'; HeightMin: LongInt location 'd1'; WidthMax: LongWord location 'd2'; HeightMax: LongWord location 'd3'): LongBool; SysCall IntuitionBase 318;

function SetPrefs(Preferences: PPreferences location 'a0'; Size: LongInt location 'd0'; Inform: LongInt location 'd1'): PPreferences; SysCall IntuitionBase 324;

function IntuiTextLength(IText: PIntuiText location 'a0'): LongInt; SysCall IntuitionBase 330;
function WBenchToBack: LongBool; SysCall IntuitionBase 336;
function WBenchToFront: LongBool; SysCall IntuitionBase 342;

function AutoRequest(Window: PWindow location 'a0'; Body: PIntuiText location 'a1'; PosText: PIntuiText location 'a2'; NegText: PIntuiText location 'a3'; PFlag: LongWord location 'd0'; NFlag: LongWord location 'd1'; Width: LongWord location 'd2'; Height: LongWord location 'd3'): LongBool; SysCall IntuitionBase 348;
procedure BeginRefresh(Window: PWindow location 'a0'); SysCall IntuitionBase 354;
function BuildSysRequest(Window: PWindow location 'a0'; Body: PIntuiText location 'a1'; PosText: PIntuiText location 'a2'; NegText: PIntuiText location 'a3'; Flags: LongWord location 'd0'; Width: LongWord location 'd1'; Height: LongWord location 'd2'): PWindow; SysCall IntuitionBase 360;
procedure EndRefresh(Window: PWindow location 'a0'; Complete: LongBool location 'd0'); SysCall IntuitionBase 366;
procedure FreeSysRequest(Window: PWindow location 'a0'); SysCall IntuitionBase 372;

function MakeScreen(Screen: PScreen location 'a0'): LongInt; SysCall IntuitionBase 378;
function RemakeDisplay: LongInt; SysCall IntuitionBase 384;
function RethinkDisplay: LongInt; SysCall IntuitionBase 390;

function AllocRemember(var RememberKey: PRemember location 'a0'; Size: LongWord location 'd0'; Flags: LongWord location 'd1'): APTR; SysCall IntuitionBase 396;
procedure FreeRemember(var RememberKey: PRemember location 'a0'; ReallyForget: LongInt location 'd0'); SysCall IntuitionBase 408;

function LockIBase(DontKnow: LongWord location 'd0'): LongWord; SysCall IntuitionBase 414;
procedure UnlockIBase(IbLock: LongWord location 'a0'); SysCall IntuitionBase 420;

function GetScreenData(Buffer: APTR location 'a0'; Size: LongWord location 'd0'; Type1: LongWord location 'd1'; Screen: PScreen location 'a1'): LongInt; SysCall IntuitionBase 426;
procedure RefreshGList(Gadgets: PGadget location 'a0'; Window: PWindow location 'a1'; Requester: PRequester location 'a2'; NumGad: LongInt location 'd0'); SysCall IntuitionBase 432;
function AddGList(Window: PWindow location 'a0'; Gadget: PGadget location 'a1'; Position: LongWord location 'd0'; NumGad: LongInt location 'd1'; Requester: PRequester location 'a2'): Word; SysCall IntuitionBase 438;
function RemoveGList(RemPtr: PWindow location 'a0'; Gadget: PGadget location 'a1'; NumGad: LongInt location 'd0'): Word; SysCall IntuitionBase 444;
procedure ActivateWindow(Window: PWindow location 'a0'); SysCall IntuitionBase 450;
procedure RefreshWindowFrame(Window: PWindow location 'a0'); SysCall IntuitionBase 456;
function ActivateGadget(Gadgets: PGadget location 'a0'; Window: PWindow location 'a1'; Requester: PRequester location 'a2'): LongBool; SysCall IntuitionBase 462;
procedure NewModifyProp(Gadget: PGadget location 'a0'; Window: PWindow location 'a1'; Requester: pRequester location 'a2'; Flags: LongWord location 'd0'; HorizPot: LongWord location 'd1'; VertPot: LongWord location 'd2'; HorizBody: LongWord location 'd3'; VertBody: LongWord location 'd4'; NumGad: LongInt location 'd5'); SysCall IntuitionBase 468;

function QueryOverscan(DisplayID: LongWord location 'a0'; Rect: PRectangle location 'a1'; OScanType: LongInt location 'd0'): LongInt; SysCall IntuitionBase 474;
procedure MoveWindowInFrontOf(Window: PWindow location 'a0'; BehindWindow: PWindow location 'a1'); SysCall IntuitionBase 480;
procedure ChangeWindowBox(Window: PWindow location 'a0'; Left: LongInt location 'd0'; Top: LongInt location 'd1'; Width: LongInt location 'd2'; Height: LongInt location 'd3'); SysCall IntuitionBase 486;
function SetEditHook(Hook: PHook location 'a0'): PHook; SysCall IntuitionBase 492;
function SetMouseQueue(Window: PWindow location 'a0'; QueueLength: LongWord location 'd0'): LongInt; SysCall IntuitionBase 498;
procedure ZipWindow(Window: PWindow location 'a0'); SysCall IntuitionBase 504;

function LockPubScreen(Name: PChar location 'a0'): PScreen; SysCall IntuitionBase 510;
procedure UnlockPubScreen(Name: PChar location 'a0'; Screen: PScreen location 'a1'); SysCall IntuitionBase 516;
function LockPubScreenList: PList; SysCall IntuitionBase 522;
procedure UnlockPubScreenList; SysCall IntuitionBase 528;
function NextPubScreen(Screen: PScreen location 'a0'; NameBuf: PChar location 'a1'): PChar; SysCall IntuitionBase 534;
procedure SetDefaultPubScreen(Name: PChar location 'a0'); SysCall IntuitionBase 540;
function SetPubScreenModes(Modes: LongWord location 'd0'): Word; SysCall IntuitionBase 546;
function PubScreenStatus(Screen: PScreen location 'a0'; StatusFlags: LongWord location 'd0'): Word; SysCall IntuitionBase 552;

function ObtainGIRPort(GInfo: PGadgetInfo location 'a0'): PRastPort; SysCall IntuitionBase 558;
procedure ReleaseGIRPort(Rp: PRastPort location 'a0'); SysCall IntuitionBase 564;
procedure GadgetMouse(Gadget: PGadget location 'a0'; GInfo: PGadgetInfo location 'a1'; var MousePoint: SmallInt location 'a2'); SysCall IntuitionBase 570;
procedure GetDefaultPubScreen(NameBuffer: PChar location 'a0'); SysCall IntuitionBase 582;
function EasyRequestArgs(Window: PWindow location 'a0'; EasyStruct: pEasyStruct location 'a1'; IDCMPPtr: PLongWord location 'a2'; Args: APTR location 'a3'): LongInt; SysCall IntuitionBase 588;
function BuildEasyRequestArgs(Window: PWindow location 'a0'; EasyStruct: PEasyStruct location 'a1'; IDCMP: LongWord location 'd0'; Args: APTR location 'a3'): PWindow; SysCall IntuitionBase 594;
function SysReqHandler(Window: PWindow location 'a0'; IDCMPPtr: PLongWord location 'a1'; WaitInput: LongInt location 'd0'): LongInt; SysCall IntuitionBase 600;
function OpenWindowTagList(NewWindow: PNewWindow location 'a0'; TagList: PTagItem location 'a1'): PWindow; SysCall IntuitionBase 606;
function OpenScreenTagList(NewScreen: PNewScreen location 'a0'; TagList: PTagItem location 'a1'): PScreen; SysCall IntuitionBase 612;

procedure DrawImageState(Rp: PRastPort location 'a0'; Image: PImage location 'a1'; LeftOffset: LongInt location 'd0'; TopOffset: LongInt location 'd1'; State: LongWord location 'd2'; DrawInfo: PDrawInfo location 'a2'); SysCall IntuitionBase 618;
function PointInImage(Point: LongWord location 'd0'; Image: PImage location 'a0'): LongBool; SysCall IntuitionBase 624;
procedure EraseImage(Rp: PRastPort location 'a0'; Image: PImage location 'a1'; LeftOffset: LongInt location 'd0'; TopOffset: LongInt location 'd1'); SysCall IntuitionBase 630;

function NewObjectA(ClassPtr: PIClass location 'a0'; ClassID: PChar location 'a1'; TagList: PTagItem location 'a2'): APTR; SysCall IntuitionBase 636;

procedure DisposeObject(Object1: APTR location 'a0'); SysCall IntuitionBase 642;
function SetAttrsA(Object1: APTR location 'a0'; TagList: PTagItem location 'a1'): LongWord; SysCall IntuitionBase 648;

function GetAttr(AttrID: LongWord location 'd0'; Object1: APTR location 'a0'; StoragePtr: PLongWord location 'a1'): LongWord; overload; SysCall IntuitionBase 654;
function GetAttr(AttrID: LongWord location 'd0'; Object1: APTR location 'a0'; var Storage: LongWord location 'a1'): LongWord; overload; SysCall IntuitionBase 654;

function SetGadgetAttrsA(Gadget: PGadget location 'a0'; Window: PWindow location 'a1'; Requester: PRequester location 'a2'; TagList: PTagItem location 'a3'): LongWord; SysCall IntuitionBase 660;

function NextObject(ObjectPtrPtr: APTR location 'a0'): APTR; SysCall IntuitionBase 666;
function MakeClass(ClassID: PChar location 'a0'; SuperClassID: PChar location 'a1'; SuperClassPtr: PIClass location 'a2'; InstanceSize: LongWord location 'd0'; Flags: LongWord location 'd1'): PIClass; SysCall IntuitionBase 678;
procedure AddClass(ClassPtr: PIClass location 'a0'); SysCall IntuitionBase 684;

function GetScreenDrawInfo(Screen: PScreen location 'a0'): PDrawInfo; SysCall IntuitionBase 690;
procedure FreeScreenDrawInfo(Screen: PScreen location 'a0'; DrawInfo: PDrawInfo location 'a1'); SysCall IntuitionBase 696;

function ResetMenuStrip(Window: PWindow location 'a0'; Menu: PMenu location 'a1'): LongBool; SysCall IntuitionBase 702;
procedure RemoveClass(ClassPtr: PIClass location 'a0'); SysCall IntuitionBase 708;
function FreeClass(ClassPtr: PIClass location 'a0'): LongBool; SysCall IntuitionBase 714;

function AllocScreenBuffer(Sc: PScreen location 'a0'; Bm: PBitMap location 'a1'; Flags: LongWord location 'd0'): PScreenBuffer; SysCall IntuitionBase 768;
procedure FreeScreenBuffer(Sc: PScreen location 'a0'; Sb: PScreenBuffer location 'a1'); SysCall IntuitionBase 774;
function ChangeScreenBuffer(Sc: PScreen location 'a0'; Sb: PScreenBuffer location 'a1'): LongWord; SysCall IntuitionBase 780;
procedure ScreenDepth(Screen: PScreen location 'a0'; Flags: LongWord location 'd0'; Reserved: APTR location 'a1'); SysCall IntuitionBase 786;
procedure ScreenPosition(Screen: PScreen location 'a0'; Flags: LongWord location 'd0'; X1: LongInt location 'd1'; Y1: LongInt location 'd2'; X2: LongInt location 'd3'; Y2: LongInt location 'd4'); SysCall IntuitionBase 792;
procedure ScrollWindowRaster(Win: PWindow location 'a1'; Dx: LongInt location 'd0'; Dy: LongInt location 'd1'; XMin: LongInt location 'd2'; YMin: LongInt location 'd3'; XMax: LongInt location 'd4'; YMax: LongInt location 'd5'); SysCall IntuitionBase 798;
procedure LendMenus(Fromwindow: PWindow location 'a0'; ToWindow: PWindow location 'a1'); SysCall IntuitionBase 804;
function DoGadgetMethodA(Gad: PGadget location 'a0'; Win: PWindow location 'a1'; Req: PRequester location 'a2'; Message: PLongWord location 'a3'): LongWord; SysCall IntuitionBase 810;
procedure SetWindowPointerA(Win: PWindow location 'a0'; Taglist: PTagItem location 'a1'); SysCall IntuitionBase 816;
function TimedDisplayAlert(AlertNumber: LongWord location 'd0'; String1: PChar location 'a0'; Height: LongWord location 'd1'; Time: LongWord location 'a1'): LongBool; SysCall IntuitionBase 822;
procedure HelpControl(Win: PWindow location 'a0'; Flags: LongWord location 'd0'); SysCall IntuitionBase 828;

// V50
procedure ShowWindow(win: PWindow location 'a0'); SysCall IntuitionBase 840;
procedure HideWindow(win: PWindow location 'a0'); SysCall IntuitionBase 846;

function GetSkinInfoAttrA(Drawinfo: PDrawInfo location 'a0'; Attr: LongWord location 'd0'; Taglist: PTagItem location 'a1'): LongWord; SysCall IntuitionBase 918;
function GetDrawInfoAttr(Drawinfo: PDrawInfo location 'a0'; Attr: LongWord location 'd0'; var ErrorPtr: LongWord location 'a1'): LongWord; SysCall IntuitionBase 936;
procedure WindowAction(Window: PWindow location 'a0'; Action: LongWord location 'd0'; Tags: PTagItem location 'a1'); SysCall IntuitionBase 942;
function TransparencyControl(Window: PWindow location 'a0'; Method: LongWord location 'd0'; Tags: PTagItem location 'a1'): LongBool; SysCall IntuitionBase 948;
procedure ScrollWindowRasterNoFill(Win: PWindow location 'a1'; Dx: LongInt location 'd0'; Dy: LongInt location 'd1'; XMin: LongInt location 'd2'; YMin: LongInt location 'd3'; XMax: LongInt location 'd4'; YMax: LongInt location 'd5'); SysCall IntuitionBase 954;
function GetMonitorList(Tags: PTagItem location 'a1'): PPObject_; SysCall IntuitionBase 966;
procedure FreeMonitorList(List: PPObject_ location 'a1'); SysCall IntuitionBase 972;
function ScreenbarControlA(Tags: PTagItem location 'a1'): LongWord; SysCall IntuitionBase 978;

{ Intuition macros }
function INST_DATA(Cl: PIClass; O: P_Object): Pointer; inline;
function SIZEOF_INSTANCE(Cl: PIClass): Longint; inline;
function BASEOBJECT(O: P_Object): Pointer; inline;
function _OBJ(O: P_Object): P_Object; inline;
function __OBJECT(O: Pointer): P_Object; inline;
function OCLASS(O: Pointer): PIClass; inline;
function SHIFTITEM(N: SmallInt): Word;
function SHIFTMENU(N: SmallInt): Word;
function SHIFTSUB(N: SmallInt): Word;
function FULLMENUNUM(Menu, Item, Sub: SmallInt): Word;
function IM_BGPEN(Im: PImage): Byte;
function IM_BOX(Im: PImage): PIBox;
function IM_FGPEN(Im: PImage): Byte;
function GADGET_BOX(G: PGadget): PIBox;
function CUSTOM_HOOK(Gadget: PGadget): PHook;
function ITEMNUM(N : Word): Word;
function MENUNUM(N : Word): Word;
function SUBNUM(N : Word): Word;

function EasyRequest(Window: PWindow; const EasyStruct: PEasyStruct; IDCMPPtr: PLongWord; const Args: array of PtrUInt): LongInt; inline;
function BuildEasyRequest(Window: PWindow; EasyStruct: PEasyStruct; IDCMP: LongWord; const Args: array of PtrUInt): PWindow; inline;
function OpenWindowTags(NewWindow: PNewWindow; const TagList: array of PtrUInt): PWindow; Inline;
function OpenScreenTags(NewScreen: PNewScreen; const TagList: array of PtrUInt): PScreen; Inline;
function NewObject(ClassPtr: PIClass; ClassID: PChar; const Tags: array of PtrUInt): Pointer; inline;
function SetAttrs(Object1: APTR; const Tags: array of PtrUInt): LongWord; inline;
function DoGadgetMethod(Gad: PGadget; Win: PWindow; Req: PRequester; const Args: array of PtrUInt): LongWord; inline;
function SetGadgetAttrs(Gadget: PGadget; Window: PWindow; Requester: PRequester; const Tags: array of PtrUInt): LongWord; inline;
procedure SetWindowPointer(Win: PWindow; const Tags: array of PtrUInt); inline;
function GetSkinInfoAttr(Drawinfo: PDrawInfo; Attr: LongWord; const Taglist: array of PtrUInt): LongWord; inline;
function ScreenbarControl(const Tags: array of PtrUInt): LongWord; inline;

{ Helper calls }
function InitIntuitionLibrary : boolean;

function DoMethodA(Obj: PObject_; Msg: Pointer): PtrUInt;
function DoMethod(obj: PObject_; const Msg: array of PtrUInt): PtrUInt; inline;

function DoSuperMethodA(Cl: PIClass; Obj: PObject_; Message: Pointer): PtrUInt; inline;
function DoSuperMethod(Cl: PIClass; Obj: PObject_; const Msg: array of PtrUInt): PtrUInt;

function DoSuperNew(Cl: PIClass; Obj: PObject_; const Tags: array of PtrUInt): PtrUInt;

implementation

function EasyRequest(Window: PWindow; const EasyStruct: PEasyStruct; IDCMPPtr: PLongWord; const Args: array of PtrUInt): LongInt; inline;
begin
  EasyRequest := EasyRequestArgs(Window, EasyStruct, IDCMPPtr, @Args);
end;

function BuildEasyRequest(Window: PWindow; EasyStruct: PEasyStruct; IDCMP: LongWord; const Args: array of PtrUInt): PWindow; inline;
begin
  BuildEasyRequest := BuildEasyRequestArgs(Window, EasyStruct, IDCMP, @Args);
end;

function OpenWindowTags(NewWindow: PNewWindow; const TagList: array of PtrUInt): PWindow; Inline;
begin
  OpenWindowTags := OpenWindowTagList(newWindow,@tagList);
end;

function OpenScreenTags(NewScreen: PNewScreen; const TagList: array of PtrUInt): PScreen; Inline;
begin
  OpenScreenTags := OpenScreenTagList(NewScreen, @TagList);
end;

function NewObject(ClassPtr: PIClass; ClassID: PChar; const Tags: array of PtrUInt): APTR; inline;
begin
  NewObject := NewObjectA(ClassPtr, ClassID, @Tags);
end;

function SetAttrs(Object1: APTR; const Tags: array of PtrUInt): LongWord; inline;
begin
  SetAttrs := SetAttrsA(Object1, @Tags);
end;

function SetGadgetAttrs(Gadget: PGadget; Window: PWindow; Requester: PRequester; const Tags: array of PtrUInt): LongWord; inline;
begin
  SetGadgetAttrs := SetGadgetAttrsA(Gadget, Window, Requester, @Tags);
end;

function DoGadgetMethod(Gad: PGadget; Win: PWindow; Req: PRequester; const Args: array of PtrUInt): LongWord; inline;
begin
  DoGadgetMethod := DoGadgetMethodA(Gad, Win, Req, @Args);
end;

procedure SetWindowPointer(Win: PWindow; const Tags: array of PtrUInt); inline;
begin
  SetWindowPointerA(Win, @Tags);
end;

function GetSkinInfoAttr(Drawinfo: PDrawInfo; Attr: LongWord; const Taglist: array of PtrUInt): LongWord; inline;
begin
  GetSkinInfoAttr := GetSkinInfoAttrA(DrawInfo, Attr, @TagList);
end;

function ScreenbarControl(const Tags: array of PtrUInt): LongWord; inline;
begin
  ScreenbarControl := ScreenbarControlA(@Tags);
end;

function INST_DATA(Cl: PIClass; O: P_Object): Pointer;
begin
  INST_DATA := Pointer(PtrUInt(O) + Cl^.cl_InstOffset);
end;

function SIZEOF_INSTANCE(Cl: PIClass): Longint;
begin
  SIZEOF_INSTANCE := Cl^.cl_InstOffset + Cl^.cl_InstSize + SizeOf(T_Object);
end;

function BASEOBJECT(O: P_Object): Pointer;
begin
  BASEOBJECT := Pointer(PtrUInt(O) + SizeOf(T_Object));
end;

function _OBJ(O: P_Object): P_Object;
begin
  _OBJ := P_Object(O);
end;

function __OBJECT(O: Pointer): P_Object;
begin
  __OBJECT := P_Object(PtrUInt(O) - SizeOf(T_Object))
end;

function OCLASS(O: Pointer): PIClass; inline;
begin
  OCLASS := P_Object(O - SizeOf(T_Object))^.o_Class;
end;

function SHIFTITEM(N: SmallInt): Word;
begin
  SHIFTITEM := (N and $3f) shl 5;
end;

function SHIFTMENU(N: SmallInt): Word;
begin
  SHIFTMENU := N and $1f;
end;

function SHIFTSUB(N: SmallInt): Word;
begin
  SHIFTSUB := (N and $1f) shl 11;
end;

function FULLMENUNUM(Menu, Item, Sub: SmallInt): Word;
begin
  FULLMENUNUM := ((Sub and $1f) shl 11) or ((Item and $3f) shl 5) or (Menu and $1f);
end;

{ The next functons _BGPEN AND _FGPEN aren't a full replacement of the
  C macros because the C preprocessor makes it possible to set the
  A/BPen values of the image class objects as well. This can't work
  in pascal, of course! }

function IM_BGPEN(Im: PImage): Byte;
begin
  IM_BGPEN := Im^.PlaneOnOff;
end;

function IM_BOX(Im: PImage): PIBox;
begin
  IM_BOX := PIBox(@Im^.LeftEdge);
end;

function IM_FGPEN(Im: PImage): Byte;
begin
  IM_FGPEN := Im^.PlanePick;
end;

function GADGET_BOX(G: PGadget): PIBox;
begin
  GADGET_BOX := PIBox(@G^.LeftEdge);
end;

function CUSTOM_HOOK(Gadget: PGadget): PHook;
begin
  CUSTOM_HOOK := PHook(Gadget^.MutualExclude);
end;

function ITEMNUM(N: Word): Word;
begin
  ITEMNUM := (N shr 5) and $3F
end;

function MENUNUM(N: Word): Word;
begin
  MENUNUM := N and $1f
end;

function SUBNUM(N: Word): Word;
begin
  SUBNUM := (N shr 11) and $1f
end;

function DoMethodA(Obj: PObject_; Msg: Pointer): PtrUInt;
var
  Hook: PHook;
begin
  Hook := @THook(OCLASS(P_Object(Obj))^.cl_Dispatcher);
  with GetEmulHandle^ do
  begin
    reg[regA0] := PtrUInt(Hook);
    reg[regA1] := PtrUInt(Msg);
    reg[regA2] := PtrUInt(Obj);
    { This is magic, but it essentially calls the class Dispatcher Hook entry point }
    DoMethodA := EmulCallDirect68k(hook^.h_Entry);
  end;
end;
{
// the old assembler implementation which trashes r31, kept for reference
asm
  mflr r31

  lwz r9,-4(r3)
  stw r9,32(r2)
  stw r4,36(r2)
  stw r3,40(r2)

  lwz r11,104(r2)
  lwz r3,8(r9)
  mtlr r11
  blrl

  mtlr r31
end ['R31'];
}

function DoMethod(obj: PObject_; const Msg: array of PtrUInt): PtrUInt; inline;
begin
  DoMethod := DoMethodA(obj, @Msg);
end;

function DoSuperMethodA(Cl: PIClass; Obj: PObject_; Message: Pointer): PtrUInt; inline;
var
  Hook: PHook;
begin
  Hook := @PIClass(Cl)^.cl_Super^.cl_Dispatcher;
  with GetEmulHandle^ do
  begin
    reg[regA0] := LongWord(Hook);
    reg[regA1] := LongWord(Message);
    reg[regA2] := LongWord(Obj);
    { This is magic, but it calls the superclass Dispatcher hook entry point }
    DoSuperMethodA := EmulCallDirect68k(hook^.h_Entry);
  end;
end;
{
// the old assembler implementation which trashes r31, kept for reference
asm
  mflr r31

  lwz r9,24(r3)
  stw r9,32(r2)
  stw r5,36(r2)
  stw r4,40(r2)

  lwz r11,104(r2)
  lwz r3,8(r9)
  mtlr r11
  blrl

  mtlr r31
end ['R31'];
}

function DoSuperMethod(Cl: PIClass; Obj: PObject_; const Msg: array of PtrUInt): PtrUInt;
begin
  DoSuperMethod:=DoSuperMethodA(Cl, obj, @msg);
end;

function DoSuperNew(Cl: PIClass; Obj: PObject_; const Tags: array of PtrUInt): PtrUInt;
var
  OpSet: TopSet;
begin
  OpSet.MethodID := OM_NEW;
  OpSet.ops_AttrList := @Tags;
  OpSet.ops_GInfo := nil;
  DoSuperNew := DoSuperMethodA(Cl, Obj, @OpSet);
end;


const
  { Change VERSION and LIBVERSION to proper values }
  VERSION : string[2] = '50';
  LIBVERSION : longword = 50;

function InitIntuitionLibrary: boolean;
begin
  InitIntuitionLibrary := Assigned(IntuitionBase);
end;

initialization
  IntuitionBase := OpenLibrary(INTUITIONNAME, LIBVERSION);
finalization
  if Assigned(IntuitionBase) then
    CloseLibrary(PLibrary(IntuitionBase));
end.

