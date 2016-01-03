{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{
   History:
   Changed tNewWindow.MaxHeigth and tNewWindow.MaxWidth
   from Word to Integer.
   30 May 2000.

   Added overlay functions for Pchar->Strings, functions
   and procedures.
   14 Jul 2000.

   Added functions and procedures with array of const.
   For use with fpc 1.0.7 They are in systemvartags.
   05 Nov 2002.

   Removed amigaoverlays, use smartlink instead.
   05 Nov 2002.

   Added the define use_amiga_smartlink.
   13 Jan 2003.

   Update for AmigaOS 3.9.
   New consts and new records.
   04 Feb 2003.

   Changed integer > smallint.
   09 Feb 2003.

   nils.sjoholm@mailbox.swipnet.se Nils Sjoholm

}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit intuition;

INTERFACE


uses exec, graphics, utility, inputevent, timer, layers;

{
 * NOTE:  intuition/iobsolete.h is included at the END of this file!
 }

{ ======================================================================== }
{ === IntuiText ========================================================== }
{ ================================= ======================================= }
{ IntuiText is a series of strings that start with a screen location
 *  (always relative to the upper-left corner of something) and then the
 *  text of the string.  The text is null-terminated.
 }
Type
    pIntuiText = ^tIntuiText;
    tIntuiText = record
        FrontPen,
        BackPen         : Byte;         { the pen numbers for the rendering }
        DrawMode        : Byte;         { the mode for rendering the text }
        LeftEdge        : smallint;        { relative start location for the text }
        TopEdge         : smallint;        { relative start location for the text }
        ITextFont       : pTextAttr;  { if NULL, you accept the default }
        IText           : STRPTR;       { pointer to null-terminated text }
        NextText        : pIntuiText;   { continuation to TxWrite another text }
    end;



{ ======================================================================== }
{ === Border ============================================================= }
{ ======================================================================== }
{ Data type Border, used for drawing a series of lines which is intended for
 *  use as a border drawing, but which may, in fact, be used to render any
 *  arbitrary vector shape.
 *  The routine DrawBorder sets up the RastPort with the appropriate
 *  variables, then does a Move to the first coordinate, then does Draws
 *  to the subsequent coordinates.
 *  After all the Draws are done, if NextBorder is non-zero we call DrawBorder
 *  recursively
 }
Type
    pBorder = ^tBorder;
    tBorder = record
        LeftEdge,
        TopEdge         : smallint;        { initial offsets from the origin }
        FrontPen,
        BackPen         : Byte;         { pens numbers for rendering }
        DrawMode        : Byte;         { mode for rendering }
        Count           : Shortint;         { number of XY pairs }
        XY              : Pointer;      { vector coordinate pairs rel to LeftTop}
        NextBorder      : pBorder;      { pointer to any other Border too }
    end;

{ ======================================================================== }
{ === MenuItem =========================================================== }
{ ======================================================================== }

Type

    pMenuItem = ^tMenuItem;
    tMenuItem = record
        NextItem        : pMenuItem;    { pointer to next in chained list }
        LeftEdge,
        TopEdge         : smallint;        { position of the select box }
        Width,
        Height          : smallint;        { dimensions of the select box }
        Flags           : Word;        { see the defines below }

        MutualExclude   : Longint;      { set bits mean this item excludes that }

        ItemFill        : Pointer;      { points to Image, IntuiText, or NULL }

    { when this item is pointed to by the cursor and the items highlight
     *  mode HIGHIMAGE is selected, this alternate image will be displayed
     }

        SelectFill      : Pointer;      { points to Image, IntuiText, or NULL }

        Command         : Char;         { only if appliprog sets the COMMSEQ flag }

        SubItem         : pMenuItem;    { if non-zero, DrawMenu shows "->" }

    { The NextSelect field represents the menu number of next selected
     *  item (when user has drag-selected several items)
     }

        NextSelect      : Word;
    end;


Const

{ FLAGS SET BY THE APPLIPROG }
    CHECKIT     = $0001;        { whether to check this item if selected }
    ITEMTEXT    = $0002;        { set if textual, clear if graphical item }
    COMMSEQ     = $0004;        { set if there's an command sequence }
    MENUTOGGLE  = $0008;        { set to toggle the check of a menu item }
    ITEMENABLED = $0010;        { set if this item is enabled }

{ these are the SPECIAL HIGHLIGHT FLAG state meanings }
    HIGHFLAGS   = $00C0;        { see definitions below for these bits }
    HIGHIMAGE   = $0000;        { use the user's "select image" }
    HIGHCOMP    = $0040;        { highlight by complementing the selectbox }
    HIGHBOX     = $0080;        { highlight by "boxing" the selectbox }
    HIGHNONE    = $00C0;        { don't highlight }

{ FLAGS SET BY BOTH APPLIPROG AND INTUITION }
    CHECKED     = $0100;        { if CHECKIT, then set this when selected }

{ FLAGS SET BY INTUITION }
    ISDRAWN     = $1000;        { this item's subs are currently drawn }
    HIGHITEM    = $2000;        { this item is currently highlighted }
    MENUTOGGLED = $4000;        { this item was already toggled }


{ ======================================================================== }
{ === Menu =============================================================== }
{ ======================================================================== }
Type

    pMenu = ^tMenu;
    tMenu = record
        NextMenu        : pMenu;        { same level }
        LeftEdge,
        TopEdge         : smallint;        { position of the select box }
        Width,
        Height          : smallint;        { dimensions of the select box }
        Flags           : Word;        { see flag definitions below }
        MenuName        : STRPTR;       { text for this Menu Header }
        FirstItem       : pMenuItem;  { pointer to first in chain }

    { these mysteriously-named variables are for internal use only }

        JazzX,
        JazzY,
        BeatX,
        BeatY           : smallint;
    end;

CONST
{ FLAGS SET BY BOTH THE APPLIPROG AND INTUITION }
    MENUENABLED = $0001;        { whether or not this menu is enabled }

{ FLAGS SET BY INTUITION }
    MIDRAWN     = $0100;        { this menu's items are currently drawn }




{ ======================================================================== }
{ === Gadget ============================================================= }
{ ======================================================================== }

Type

    pGadget = ^tGadget;
    tGadget = record
        NextGadget      : pGadget;      { next gadget in the list }

        LeftEdge,
        TopEdge         : smallint;        { "hit box" of gadget }
        Width,
        Height          : smallint;        { "hit box" of gadget }

        Flags           : Word;        { see below for list of defines }

        Activation      : Word;        { see below for list of defines }

        GadgetType      : Word;        { see below for defines }

    { appliprog can specify that the Gadget be rendered as either as Border
     * or an Image.  This variable points to which (or equals NULL if there's
     * nothing to be rendered about this Gadget)
     }

        GadgetRender    : Pointer;

    { appliprog can specify "highlighted" imagery rather than algorithmic
     * this can point to either Border or Image data
     }

        SelectRender    : Pointer;

        GadgetText      : pIntuiText; { text for this gadget }

    { by using the MutualExclude word, the appliprog can describe
     * which gadgets mutually-exclude which other ones.  The bits
     * in MutualExclude correspond to the gadgets in object containing
     * the gadget list.  If this gadget is selected and a bit is set
     * in this gadget's MutualExclude and the gadget corresponding to
     * that bit is currently selected (e.g. bit 2 set and gadget 2
     * is currently selected) that gadget must be unselected.
     * Intuition does the visual unselecting (with checkmarks) and
     * leaves it up to the program to unselect internally
     }

        MutualExclude   : Longint;      { set bits mean this gadget excludes that gadget }

    { pointer to a structure of special data required by Proportional,
     * String and Longint Gadgets
     }

        SpecialInfo     : Pointer;

        GadgetID        : Word;        { user-definable ID field }
        UserData        : Pointer;      { ptr to general purpose User data (ignored by In) }
    end;

 pExtGadget = ^tExtGadget;
 tExtGadget = record
    { The first fields match struct Gadget exactly }
    NextGadget     : pExtGadget;  { Matches struct Gadget }
    LeftEdge, TopEdge,            { Matches struct Gadget }
    Width, Height  : smallint;     { Matches struct Gadget }
    Flags,                        { Matches struct Gadget }
    Activation,                   { Matches struct Gadget }
    GadgetType     : WORD;        { Matches struct Gadget }
    GadgetRender,                 { Matches struct Gadget }
    SelectRender   : Pointer;     { Matches struct Gadget }
    GadgetText     : pIntuiText;  { Matches struct Gadget }
    MutualExclude  : Longint;     { Matches struct Gadget }
    SpecialInfo    : Pointer;     { Matches struct Gadget }
    GadgetID       : WORD;        { Matches struct Gadget }
    UserData       : Pointer;     { Matches struct Gadget }

    { These fields only exist under V39 and only if GFLG_EXTENDED is set }
    MoreFlags      : ULONG;     { see GMORE_ flags below }
    BoundsLeftEdge,             { Bounding extent for gadget, valid   }
    BoundsTopEdge,              { only if GMORE_BOUNDS is set.  The   }
    BoundsWidth,                { GFLG_RELxxx flags affect these      }
    BoundsHeight   : smallint;      { coordinates as well.        }
 end;


CONST
{ --- Gadget.Flags values      --- }
{ combinations in these bits describe the highlight technique to be used }
 GFLG_GADGHIGHBITS  = $0003;
 GFLG_GADGHCOMP     = $0000;  { Complement the select box }
 GFLG_GADGHBOX      = $0001;  { Draw a box around the image }
 GFLG_GADGHIMAGE    = $0002;  { Blast in this alternate image }
 GFLG_GADGHNONE     = $0003;  { don't highlight }

 GFLG_GADGIMAGE     = $0004;  { set IF GadgetRender AND SelectRender
                                   * point to an Image structure, clear
                                   * if they point to Border structures
                                   }

{ combinations in these next two bits specify to which corner the gadget's
 *  Left & Top coordinates are relative.  If relative to Top/Left,
 *  these are "normal" coordinates (everything is relative to something in
 *  this universe).
 *
 * Gadget positions and dimensions are relative to the window or
 * requester which contains the gadget
 }
 GFLG_RELBOTTOM   = $0008;  { vert. pos. is relative to bottom edge }
 GFLG_RELRIGHT    = $0010;  { horiz. pos. is relative to right edge }
 GFLG_RELWIDTH    = $0020;  { width is relative to req/window    }
 GFLG_RELHEIGHT   = $0040;  { height is relative to req/window   }

{ New for V39: GFLG_RELSPECIAL allows custom gadget implementors to
 * make gadgets whose position and size depend in an arbitrary way
 * on their window's dimensions.  The GM_LAYOUT method will be invoked
 * for such a gadget (or any other GREL_xxx gadget) at suitable times,
 * such as when the window opens or the window's size changes.
 }
 GFLG_RELSPECIAL  = $4000;  { custom gadget has special relativity.
                                   * Gadget box values are absolutes, but
                                   * can be changed via the GM_LAYOUT method.
                                   }

 GFLG_SELECTED    = $0080;  { you may initialize AND look at this        }

{ the GFLG_DISABLED flag is initialized by you and later set by Intuition
 * according to your calls to On/OffGadget().  It specifies whether or not
 * this Gadget is currently disabled from being selected
 }
 GFLG_DISABLED    = $0100;

{ These flags specify the type of text field that Gadget.GadgetText
 * points to.  In all normal (pre-V36) gadgets which you initialize
 * this field should always be zero.  Some types of gadget objects
 * created from classes will use these fields to keep track of
 * types of labels/contents that different from IntuiText, but are
 * stashed in GadgetText.
 }

 GFLG_LABELMASK   = $3000;
 GFLG_LABELITEXT  = $0000;  { GadgetText points to IntuiText     }
 GFLG_LABELSTRING = $1000;  { GadgetText points to (UBYTE *)     }
 GFLG_LABELIMAGE  = $2000;  { GadgetText points to Image (object)        }

{ New for V37: GFLG_TABCYCLE }
 GFLG_TABCYCLE    = $0200;  { (string OR custom) gadget participates in
                                   * cycling activation with Tab or Shift-Tab
                                   }
{ New for V37: GFLG_STRINGEXTEND.  We discovered that V34 doesn't properly
 * ignore the value we had chosen for the Gadget->Activation flag
 * GACT_STRINGEXTEND.  NEVER SET THAT FLAG WHEN RUNNING UNDER V34.
 * The Gadget->Flags bit GFLG_STRINGEXTEND is provided as a synonym which is
 * safe under V34, and equivalent to GACT_STRINGEXTEND under V37.
 * (Note that the two flags are not numerically equal)
 }
 GFLG_STRINGEXTEND = $0400;  { this String Gadget has StringExtend        }

{ New for V39: GFLG_IMAGEDISABLE.  This flag is automatically set if
 * the custom image of this gadget knows how to do disabled rendering
 * (more specifically, if its IA_SupportsDisable attribute is TRUE).
 * Intuition uses this to defer the ghosting to the image-class,
 * instead of doing it itself (the old compatible way).
 * Do not set this flag yourself - Intuition will do it for you.
 }

 GFLG_IMAGEDISABLE = $0800;  { Gadget's image knows how to do disabled
                                   * rendering
                                   }

{ New for V39:  If set, this bit means that the Gadget is actually
 * a struct ExtGadget, with new fields and flags.  All V39 boopsi
 * gadgets are ExtGadgets.  Never ever attempt to read the extended
 * fields of a gadget if this flag is not set.
 }
 GFLG_EXTENDED    = $8000;  { Gadget is extended }

{ ---  Gadget.Activation flag values   --- }
{ Set GACT_RELVERIFY if you want to verify that the pointer was still over
 * the gadget when the select button was released.  Will cause
 * an IDCMP_GADGETUP message to be sent if so.
 }
 GACT_RELVERIFY    = $0001;

{ the flag GACT_IMMEDIATE, when set, informs the caller that the gadget
 *  was activated when it was activated.  This flag works in conjunction with
 *  the GACT_RELVERIFY flag
 }
 GACT_IMMEDIATE    = $0002;

{ the flag GACT_ENDGADGET, when set, tells the system that this gadget,
 * when selected, causes the Requester to be ended.  Requesters
 * that are ended are erased and unlinked from the system.
 }
 GACT_ENDGADGET    = $0004;

{ the GACT_FOLLOWMOUSE flag, when set, specifies that you want to receive
 * reports on mouse movements while this gadget is active.
 * You probably want to set the GACT_IMMEDIATE flag when using
 * GACT_FOLLOWMOUSE, since that's the only reasonable way you have of
 * learning why Intuition is suddenly sending you a stream of mouse
 * movement events.  If you don't set GACT_RELVERIFY, you'll get at
 * least one Mouse Position event.
 }
 GACT_FOLLOWMOUSE = $0008;

{ if any of the BORDER flags are set in a Gadget that's included in the
 * Gadget list when a Window is opened, the corresponding Border will
 * be adjusted to make room for the Gadget
 }
 GACT_RIGHTBORDER = $0010;
 GACT_LEFTBORDER  = $0020;
 GACT_TOPBORDER   = $0040;
 GACT_BOTTOMBORDER= $0080;
 GACT_BORDERSNIFF = $8000;  { neither set nor rely on this bit   }

 GACT_TOGGLESELECT= $0100;  { this bit for toggle-select mode }
 GACT_BOOLEXTEND  = $2000;  { this Boolean Gadget has a BoolInfo }

{ should properly be in StringInfo, but aren't }
 GACT_STRINGLEFT  = $0000;  { NOTE WELL: that this has value zero        }
 GACT_STRINGCENTER= $0200;
 GACT_STRINGRIGHT = $0400;
 GACT_LONGINT     = $0800;  { this String Gadget is for Long Ints        }
 GACT_ALTKEYMAP   = $1000;  { this String has an alternate keymap        }
 GACT_STRINGEXTEND= $2000;  { this String Gadget has StringExtend        }
                                  { NOTE: NEVER SET GACT_STRINGEXTEND IF YOU
                                   * ARE RUNNING ON LESS THAN V36!  SEE
                                   * GFLG_STRINGEXTEND (ABOVE) INSTEAD
                                   }

 GACT_ACTIVEGADGET = $4000;  { this gadget is "active".  This flag
                                   * is maintained by Intuition, and you
                                   * cannot count on its value persisting
                                   * while you do something on your program's
                                   * task.  It can only be trusted by
                                   * people implementing custom gadgets
                                   }

{ note $8000 is used above (GACT_BORDERSNIFF);
 * all Activation flags defined }

{ --- GADGET TYPES ------------------------------------------------------- }
{ These are the Gadget Type definitions for the variable GadgetType
 * gadget number type MUST start from one.  NO TYPES OF ZERO ALLOWED.
 * first comes the mask for Gadget flags reserved for Gadget typing
 }
 GTYP_GADGETTYPE = $FC00;  { all Gadget Global Type flags (padded) }
 GTYP_SYSGADGET  = $8000;  { 1 = Allocated by the system, 0 = by app. }
 GTYP_SCRGADGET  = $4000;  { 1 = ScreenGadget, 0 = WindowGadget }
 GTYP_GZZGADGET  = $2000;  { 1 = for WFLG_GIMMEZEROZERO borders }
 GTYP_REQGADGET  = $1000;  { 1 = this is a Requester Gadget }
{ system gadgets }
 GTYP_SIZING     = $0010;
 GTYP_WDRAGGING  = $0020;
 GTYP_SDRAGGING  = $0030;
 GTYP_WUPFRONT   = $0040;
 GTYP_SUPFRONT   = $0050;
 GTYP_WDOWNBACK  = $0060;
 GTYP_SDOWNBACK  = $0070;
 GTYP_CLOSE      = $0080;
{ application gadgets }
 GTYP_BOOLGADGET = $0001;
 GTYP_GADGET0002 = $0002;
 GTYP_PROPGADGET = $0003;
 GTYP_STRGADGET  = $0004;
 GTYP_CUSTOMGADGET    =   $0005;


{* GTYP_GTYPEMASK is a mask you can apply to tell what class
 * of gadget this is.  The possible classes follow.
 *}
 GTYP_GTYPEMASK        =  $0007;

{ This bit in GadgetType is reserved for undocumented internal use
 * by the Gadget Toolkit, and cannot be used nor relied on by
 * applications:        $0100;
 }

{ New for V39.  Gadgets which have the GFLG_EXTENDED flag set are
 * actually ExtGadgets, which have more flags.  The GMORE_xxx
 * identifiers describe those flags.  For GMORE_SCROLLRASTER, see
 * important information in the ScrollWindowRaster() autodoc.
 * NB: GMORE_SCROLLRASTER must be set before the gadget is
 * added to a window.
 }
 GMORE_BOUNDS       = $00000001; { ExtGadget has valid Bounds }
 GMORE_GADGETHELP   = $00000002; { This gadget responds to gadget help }
 GMORE_SCROLLRASTER = $00000004; { This (custom) gadget uses ScrollRaster }

{ ======================================================================== }
{ === BoolInfo======================================================= }
{ ======================================================================== }
{ This is the special data needed by an Extended Boolean Gadget
 * Typically this structure will be pointed to by the Gadget field SpecialInfo
 }
Type
    pBoolInfo = ^tBoolInfo;
    tBoolInfo = record
        Flags   : Word;        { defined below }
        Mask    : Pointer; { bit mask for highlighting and selecting
                         * mask must follow the same rules as an Image
                         * plane.  It's width and height are determined
                         * by the width and height of the gadget's
                         * select box. (i.e. Gadget.Width and .Height).
                         }
        Reserved : ULONG;     { set to 0      }
    end;

Const

{ set BoolInfo.Flags to this flag bit.
 * in the future, additional bits might mean more stuff hanging
 * off of BoolInfo.Reserved.
}
    BOOLMASK    = $0001;        { extension is for masked gadget }

{ ======================================================================== }
{ === PropInfo =========================================================== }
{ ======================================================================== }
{ this is the special data required by the proportional Gadget
 * typically, this data will be pointed to by the Gadget variable SpecialInfo
 }

Type

    pPropInfo = ^tPropInfo;
    tPropInfo = record
        Flags   : Word;        { general purpose flag bits (see defines below) }

    { You initialize the Pot variables before the Gadget is added to
     * the system.  Then you can look here for the current settings
     * any time, even while User is playing with this Gadget.  To
     * adjust these after the Gadget is added to the System, use
     * ModifyProp();  The Pots are the actual proportional settings,
     * where a value of zero means zero and a value of MAXPOT means
     * that the Gadget is set to its maximum setting.
     }

        HorizPot        : WORD; { 16-bit FixedPoint horizontal quantity percentage }
        VertPot         : WORD; { 16-bit FixedPoint vertical quantity percentage }

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

        HorizBody       : Word;        { horizontal Body }
        VertBody        : Word;        { vertical Body }

    { these are the variables that Intuition sets and maintains }

        CWidth          : Word;        { Container width (with any relativity absoluted) }
        CHeight         : Word;        { Container height (with any relativity absoluted) }
        HPotRes,
        VPotRes         : Word;        { pot increments }
        LeftBorder      : Word;        { Container borders }
        TopBorder       : Word;        { Container borders }
    end;

CONST
{ --- FLAG BITS ---------------------------------------------------------- }
 AUTOKNOB     =   $0001;  { this flag sez:  gimme that old auto-knob }
{ NOTE: if you do not use an AUTOKNOB for a proportional gadget,
 * you are currently limited to using a single Image of your own
 * design: Intuition won't handle a linked list of images as
 * a proportional gadget knob.
 }

 FREEHORIZ     =  $0002;  { IF set, the knob can move horizontally }
 FREEVERT      =  $0004;  { IF set, the knob can move vertically }
 PROPBORDERLESS =  $0008;  { IF set, no border will be rendered }
 KNOBHIT       =  $0100;  { set when this Knob is hit }
 PROPNEWLOOK   =  $0010;  { set this IF you want to get the new
                                 * V36 look
                                 }

 KNOBHMIN      =  6;       { minimum horizontal size of the Knob }
 KNOBVMIN      =  4;       { minimum vertical size of the Knob }
 MAXBODY       =  $FFFF;  { maximum body value }
 MAXPOT        =  $FFFF;  { maximum pot value }

{ ======================================================================== }
{ === StringInfo ========================================================= }
{ ======================================================================== }
{ this is the special data required by the string Gadget
 * typically, this data will be pointed to by the Gadget variable SpecialInfo
 }

Type

    pStringInfo = ^tStringInfo;
    tStringInfo = record
    { you initialize these variables, and then Intuition maintains them }
        Buffer          : STRPTR;       { the buffer containing the start and final string }
        UndoBuffer      : STRPTR;       { optional buffer for undoing current entry }
        BufferPos       : smallint;        { character position in Buffer }
        MaxChars        : smallint;        { max number of chars in Buffer (including NULL) }
        DispPos         : smallint;        { Buffer position of first displayed character }

    { Intuition initializes and maintains these variables for you }

        UndoPos         : smallint;        { character position in the undo buffer }
        NumChars        : smallint;        { number of characters currently in Buffer }
        DispCount       : smallint;        { number of whole characters visible in Container }
        CLeft,
        CTop            : smallint;        { topleft offset of the container }

    { you can initialize this variable before the gadget is submitted to
     * Intuition, and then examine it later to discover what Longint
     * the user has entered (if the user never plays with the gadget,
     * the value will be unchanged from your initial setting)
     }
        Extension       : Pointer;
        _LongInt         : Longint;

    { If you want this Gadget to use your own Console keymapping, you
     * set the ALTKEYMAP bit in the Activation flags of the Gadget, and then
     * set this variable to point to your keymap.  If you don't set the
     * ALTKEYMAP, you'll get the standard ASCII keymapping.
     }

        AltKeyMap       : Pointer;
    end;


{ ======================================================================== }
{ === Requester ========================================================== }
{ ======================================================================== }

Type

    pRequester = ^tRequester;
    tRequester = record
    { the ClipRect and BitMap and used for rendering the requester }
        OlderRequest    : pRequester;
        LeftEdge,
        TopEdge         : smallint;        { dimensions of the entire box }
        Width,
        Height          : smallint;        { dimensions of the entire box }
        RelLeft,
        RelTop          : smallint;        { for Pointer relativity offsets }

        ReqGadget       : pGadget;    { pointer to a list of Gadgets }
        ReqBorder       : pBorder;    { the box's border }
        ReqText         : pIntuiText; { the box's text }
        Flags           : Word;        { see definitions below }

    { pen number for back-plane fill before draws }

        BackFill        : Byte;

    { Layer in place of clip rect       }

        ReqLayer        : pLayer;

        ReqPad1         : Array [0..31] of Byte;

    { If the BitMap plane pointers are non-zero, this tells the system
     * that the image comes pre-drawn (if the appliprog wants to define
     * it's own box, in any shape or size it wants!);  this is OK by
     * Intuition as long as there's a good correspondence between
     * the image and the specified Gadgets
     }

        ImageBMap       : pBitMap;    { points to the BitMap of PREDRAWN imagery }
        RWindow         : Pointer;      { added.  points back to Window }
        ReqImage        : Pointer;
        ReqPad2         : Array [0..31] of Shortint;
    end;


Const

{ FLAGS SET BY THE APPLIPROG }
    POINTREL            = $0001;    { if POINTREL set, TopLeft is relative to pointer}
    PREDRAWN            = $0002;    { if ReqBMap points to predrawn Requester imagery }
    NOISYREQ            = $0004;    { if you don't want requester to filter input          }

    SIMPLEREQ           = $0010;
        { to use SIMPLEREFRESH layer (recommended)     }

    { New for V36          }
    USEREQIMAGE         = $0020;
         {  render linked list ReqImage after BackFill
         * but before gadgets and text
         }
    NOREQBACKFILL       = $0040;
        { don't bother filling requester with Requester.BackFill pen   }


{ FLAGS SET BY INTUITION }
    REQOFFWINDOW        = $1000;        { part of one of the Gadgets was offwindow }
    REQACTIVE           = $2000;        { this requester is active }
    SYSREQUEST          = $4000;        { this requester caused by system }
    DEFERREFRESH        = $8000;        { this Requester stops a Refresh broadcast }




{ ======================================================================== }
{ === Image ============================================================== }
{ ======================================================================== }
{ This is a brief image structure for very simple transfers of
 * image data to a RastPort
 }

Type
    pImage = ^tImage;
    tImage = record
        LeftEdge        : smallint;        { starting offset relative to some origin }
        TopEdge         : smallint;        { starting offsets relative to some origin }
        Width           : smallint;        { pixel size (though data is word-aligned) }
        Height,
        Depth           : smallint;        { pixel sizes }
        ImageData       : Pointer;      { pointer to the actual word-aligned bits }

    { the PlanePick and PlaneOnOff variables work much the same way as the
     * equivalent GELS Bob variables.  It's a space-saving
     * mechanism for image data.  Rather than defining the image data
     * for every plane of the RastPort, you need define data only
     * for the planes that are not entirely zero or one.  As you
     * define your Imagery, you will often find that most of the planes
     * ARE just as color selectors.  For instance, if you're designing
     * a two-color Gadget to use colors two and three, and the Gadget
     * will reside in a five-plane display, bit plane zero of your
     * imagery would be all ones, bit plane one would have data that
     * describes the imagery, and bit planes two through four would be
     * all zeroes.  Using these flags allows you to avoid wasting all
     * that memory in this way:  first, you specify which planes you
     * want your data to appear in using the PlanePick variable.  For
     * each bit set in the variable, the next "plane" of your image
     * data is blitted to the display.  For each bit clear in this
     * variable, the corresponding bit in PlaneOnOff is examined.
     * If that bit is clear, a "plane" of zeroes will be used.
     * If the bit is set, ones will go out instead.  So, for our example:
     *   Gadget.PlanePick = $02;
     *   Gadget.PlaneOnOff = $01;
     * Note that this also allows for generic Gadgets, like the
     * System Gadgets, which will work in any number of bit planes.
     * Note also that if you want an Image that is only a filled
     * rectangle, you can get this by setting PlanePick to zero
     * (pick no planes of data) and set PlaneOnOff to describe the pen
     * color of the rectangle.
     }

        PlanePick,
        PlaneOnOff      : Byte;

    { if the NextImage variable is not NULL, Intuition presumes that
     * it points to another Image structure with another Image to be
     * rendered
     }

        NextImage       : pImage;
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
 TABLETA_Dummy          = (TAG_USER + $3A000)  ;
 TABLETA_TabletZ        = (TABLETA_Dummy + $01);
 TABLETA_RangeZ         = (TABLETA_Dummy + $02);
 TABLETA_AngleX         = (TABLETA_Dummy + $03);
 TABLETA_AngleY         = (TABLETA_Dummy + $04);
 TABLETA_AngleZ         = (TABLETA_Dummy + $05);
 TABLETA_Pressure       = (TABLETA_Dummy + $06);
 TABLETA_ButtonBits     = (TABLETA_Dummy + $07);
 TABLETA_InProximity    = (TABLETA_Dummy + $08);
 TABLETA_ResolutionX    = (TABLETA_Dummy + $09);
 TABLETA_ResolutionY    = (TABLETA_Dummy + $0A);

{ If your window sets WA_TabletMessages to TRUE, then it will receive
 * extended IntuiMessages (struct ExtIntuiMessage) whose eim_TabletData
 * field points at a TabletData structure.  This structure contains
 * additional information about the input event.
 }

Type
 pTabletData = ^tTabletData;
 tTabletData = record
    { Sub-pixel position of tablet, in screen coordinates,
     * scaled to fill a UWORD fraction:
     }
    td_XFraction, td_YFraction  : WORD;

    { Current tablet coordinates along each axis: }
    td_TabletX, td_TabletY      : ULONG;

    { Tablet range along each axis.  For example, if td_TabletX
     * can take values 0-999, td_RangeX should be 1000.
     }
    td_RangeX, td_RangeY        : ULONG;

    { Pointer to tag-list of additional tablet attributes.
     * See <intuition/intuition.h> for the tag values.
     }
    td_TagList                  : pTagItem;
 end;

{ If a tablet driver supplies a hook for ient_CallBack, it will be
 * invoked in the standard hook manner.  A0 will point to the Hook
 * itself, A2 will point to the InputEvent that was sent, and
 * A1 will point to a TabletHookData structure.  The InputEvent's
 * ie_EventAddress field points at the IENewTablet structure that
 * the driver supplied.
 *
 * Based on the thd_Screen, thd_Width, and thd_Height fields, the driver
 * should scale the ient_TabletX and ient_TabletY fields and store the
 * result in ient_ScaledX, ient_ScaledY, ient_ScaledXFraction, and
 * ient_ScaledYFraction.
 *
 * The tablet hook must currently return NULL.  This is the only
 * acceptable return-value under V39.
 }

 pTabletHookData = ^tTabletHookData;
 tTabletHookData = record
    { Pointer to the active screen:
     * Note: if there are no open screens, thd_Screen will be NULL.
     * thd_Width and thd_Height will then describe an NTSC 64$400
     * screen.  Please scale accordingly.
     }
    thd_Screen      : Pointer;

    { The width and height (measured in pixels of the active screen)
     * that your are to scale to:
     }
    thd_Width,
    thd_Height      : ULONG;

    { Non-zero if the screen or something about the screen
     * changed since the last time you were invoked:
     }
    thd_ScreenChanged   : Longint;
 end;


{ ======================================================================== }
{ === IntuiMessage ======================================================= }
{ ======================================================================== }

Type

    pIntuiMessage = ^tIntuiMessage;
    tIntuiMessage = record
        ExecMessage     : tMessage;

    { the Class bits correspond directly with the IDCMP Flags, except for the
     * special bit LONELYMESSAGE (defined below)
     }

        IClass           : ULONG;

    { the Code field is for special values like MENU number }

        Code            : Word;

    { the Qualifier field is a copy of the current InputEvent's Qualifier }

        Qualifier       : Word;

    { IAddress contains particular addresses for Intuition functions, like
     * the pointer to the Gadget or the Screen
     }

        IAddress        : Pointer;

    { when getting mouse movement reports, any event you get will have the
     * the mouse coordinates in these variables.  the coordinates are relative
     * to the upper-left corner of your Window (GIMMEZEROZERO notwithstanding)
     }

        MouseX,
        MouseY          : smallint;

    { the time values are copies of the current system clock time.  Micros
     * are in units of microseconds, Seconds in seconds.
     }

        Seconds,
        Micros          : ULONG;

    { the IDCMPWindow variable will always have the Pointer of the Window of
     * this IDCMP
     }

        IDCMPWindow     : Pointer;

    { system-use variable }

        SpecialLink     : pIntuiMessage;
    end;

{ New for V39:
 * All IntuiMessages are now slightly extended.  The ExtIntuiMessage
 * structure has an additional field for tablet data, which is usually
 * NULL.  If a tablet driver which is sending IESUBCLASS_NEWTABLET
 * events is installed in the system, windows with the WA_TabletMessages
 * property set will find that eim_TabletData points to the TabletData
 * structure.  Applications must first check that this field is non-NULL;
 * it will be NULL for certain kinds of message, including mouse activity
 * generated from other than the tablet (i.e. the keyboard equivalents
 * or the mouse itself).
 *
 * NEVER EVER examine any extended fields when running under pre-V39!
 *
 * NOTE: This structure is subject to grow in the future.  Making
 * assumptions about its size is A BAD IDEA.
 }

 pExtIntuiMessage = ^tExtIntuiMessage;
 tExtIntuiMessage = record
    eim_IntuiMessage  : tIntuiMessage;
    eim_TabletData    : pTabletData;
 end;


CONST

{ --- IDCMP Classes ------------------------------------------------------ }
{ Please refer to the Autodoc for OpenWindow() and to the Rom Kernel
 * Manual for full details on the IDCMP classes.
 }
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
 IDCMP_WBENCHMESSAGE   =  $00020000;  {  System use only         }
 IDCMP_ACTIVEWINDOW    =  $00040000;
 IDCMP_INACTIVEWINDOW  =  $00080000;
 IDCMP_DELTAMOVE       =  $00100000;
 IDCMP_VANILLAKEY      =  $00200000;
 IDCMP_INTUITICKS      =  $00400000;
{  for notifications from "boopsi" gadgets               }
 IDCMP_IDCMPUPDATE     =  $00800000;  { new for V36      }
{ for getting help key report during menu session        }
 IDCMP_MENUHELP        =  $01000000;  { new for V36      }
{ for notification of any move/size/zoom/change window   }
 IDCMP_CHANGEWINDOW    =  $02000000;  { new for V36      }
 IDCMP_GADGETHELP      =  $04000000;  { new for V39      }

{ NOTEZ-BIEN:                          $80000000 is reserved for internal use   }

{ the IDCMP Flags do not use this special bit, which is cleared when
 * Intuition sends its special message to the Task, and set when Intuition
 * gets its Message back from the Task.  Therefore, I can check here to
 * find out fast whether or not this Message is available for me to send
 }
 IDCMP_LONELYMESSAGE   =  $80000000;


{ --- IDCMP Codes -------------------------------------------------------- }
{ This group of codes is for the IDCMP_CHANGEWINDOW message }
 CWCODE_MOVESIZE = $0000;  { Window was moved and/or sized }
 CWCODE_DEPTH    = $0001;  { Window was depth-arranged (new for V39) }

{ This group of codes is for the IDCMP_MENUVERIFY function }
 MENUHOT       =  $0001;  { IntuiWants verification OR MENUCANCEL    }
 MENUCANCEL    =  $0002;  { HOT Reply of this cancels Menu operation }
 MENUWAITING   =  $0003;  { Intuition simply wants a ReplyMsg() ASAP }

{ These are internal tokens to represent state of verification attempts
 * shown here as a clue.
 }
 OKOK          =  MENUHOT; { guy didn't care                      }
 OKABORT       =  $0004;  { window rendered question moot        }
 OKCANCEL      =  MENUCANCEL; { window sent cancel reply          }

{ This group of codes is for the IDCMP_WBENCHMESSAGE messages }
 WBENCHOPEN    =  $0001;
 WBENCHCLOSE   =  $0002;


{ A data structure common in V36 Intuition processing  }
Type
   pIBox = ^tIBox;
   tIBox = record
    Left,
    Top,
    Width,
    Height : smallint;
   END;


{ ======================================================================== }
{ === Window ============================================================= }
{ ======================================================================== }

Type

    pWindow = ^tWindow;
    tWindow = record
        NextWindow      : pWindow;      { for the linked list in a screen }

        LeftEdge,
        TopEdge         : smallint;        { screen dimensions of window }
        Width,
        Height          : smallint;        { screen dimensions of window }

        MouseY,
        MouseX          : smallint;        { relative to upper-left of window }

        MinWidth,
        MinHeight       : smallint;        { minimum sizes }
        MaxWidth,
        MaxHeight       : smallint;        { maximum sizes }

        Flags           : ULONG;      { see below for defines }

        MenuStrip       : pMenu;      { the strip of Menu headers }

        Title           : STRPTR;       { the title text for this window }

        FirstRequest    : pRequester; { all active Requesters }

        DMRequest       : pRequester; { double-click Requester }

        ReqCount        : smallint;        { count of reqs blocking Window }

        WScreen         : Pointer;      { this Window's Screen }
        RPort           : pRastPort;  { this Window's very own RastPort }

    { the border variables describe the window border.   If you specify
     * GIMMEZEROZERO when you open the window, then the upper-left of the
     * ClipRect for this window will be upper-left of the BitMap (with correct
     * offsets when in SuperBitMap mode; you MUST select GIMMEZEROZERO when
     * using SuperBitMap).  If you don't specify ZeroZero, then you save
     * memory (no allocation of RastPort, Layer, ClipRect and associated
     * Bitmaps), but you also must offset all your writes by BorderTop,
     * BorderLeft and do your own mini-clipping to prevent writing over the
     * system gadgets
     }

        BorderLeft,
        BorderTop,
        BorderRight,
        BorderBottom    : Shortint;
        BorderRPort     : pRastPort;


    { You supply a linked-list of Gadgets for your Window.
     * This list DOES NOT include system gadgets.  You get the standard
     * window system gadgets by setting flag-bits in the variable Flags (see
     * the bit definitions below)
     }

        FirstGadget     : pGadget;

    { these are for opening/closing the windows }

        Parent,
        Descendant      : pWindow;

    { sprite data information for your own Pointer
     * set these AFTER you Open the Window by calling SetPointer()
     }

        _Pointer         : Pointer;      { sprite data }
        PtrHeight       : Shortint;         { sprite height (not including sprite padding) }
        PtrWidth        : Shortint;         { sprite width (must be less than or equal to 16) }
        XOffset,
        YOffset         : Shortint;         { sprite offsets }

    { the IDCMP Flags and User's and Intuition's Message Ports }
        IDCMPFlags      : ULONG;      { User-selected flags }
        UserPort,
        WindowPort      : pMsgPort;
        MessageKey      : pIntuiMessage;

        DetailPen,
        BlockPen        : Byte; { for bar/border/gadget rendering }

    { the CheckMark is a pointer to the imagery that will be used when
     * rendering MenuItems of this Window that want to be checkmarked
     * if this is equal to NULL, you'll get the default imagery
     }

        CheckMark       : pImage;

        ScreenTitle     : STRPTR; { if non-null, Screen title when Window is active }

    { These variables have the mouse coordinates relative to the
     * inner-Window of GIMMEZEROZERO Windows.  This is compared with the
     * MouseX and MouseY variables, which contain the mouse coordinates
     * relative to the upper-left corner of the Window, GIMMEZEROZERO
     * notwithstanding
     }

        GZZMouseX       : smallint;
        GZZMouseY       : smallint;

    { these variables contain the width and height of the inner-Window of
     * GIMMEZEROZERO Windows
     }

        GZZWidth        : smallint;
        GZZHeight       : smallint;

        ExtData         : Pointer;

        UserData        : Pointer;      { general-purpose pointer to User data extension }

    {* jimm: NEW: 11/18/85: this pointer keeps a duplicate of what
     * Window.RPort->Layer is _supposed_ to be pointing at
     }

        WLayer          : pLayer;

    { jimm: NEW 1.2: need to keep track of the font that
     * OpenWindow opened, in case user SetFont's into RastPort
     }

        IFont           : pTextFont;
    {* (V36) another flag word (the Flags field is used up).
     * At present, all flag values are system private.
     * Until further notice, you may not change nor use this field.
     *}
        MoreFlags       : ULONG;

    {**** Data beyond this point are Intuition Private.  DO NOT USE ****}

    end;

CONST
{ --- Flags requested at OpenWindow() time by the application --------- }
 WFLG_SIZEGADGET   =  $00000001;  { include sizing system-gadget? }
 WFLG_DRAGBAR      =  $00000002;  { include dragging system-gadget? }
 WFLG_DEPTHGADGET  =  $00000004;  { include depth arrangement gadget? }
 WFLG_CLOSEGADGET  =  $00000008;  { include close-box system-gadget? }

 WFLG_SIZEBRIGHT   =  $00000010;  { size gadget uses right border }
 WFLG_SIZEBBOTTOM  =  $00000020;  { size gadget uses bottom border }

{ --- refresh modes ------------------------------------------------------ }
{ combinations of the WFLG_REFRESHBITS select the refresh type }
 WFLG_REFRESHBITS   = $000000C0;
 WFLG_SMART_REFRESH = $00000000;
 WFLG_SIMPLE_REFRESH= $00000040;
 WFLG_SUPER_BITMAP  = $00000080;
 WFLG_OTHER_REFRESH = $000000C0;

 WFLG_BACKDROP      = $00000100;  { this is a backdrop window }

 WFLG_REPORTMOUSE   = $00000200;  { to hear about every mouse move }

 WFLG_GIMMEZEROZERO = $00000400;  { a GimmeZeroZero window       }

 WFLG_BORDERLESS    = $00000800;  { to get a Window sans border }

 WFLG_ACTIVATE      = $00001000;  { when Window opens, it's Active }


{ --- Other User Flags --------------------------------------------------- }
 WFLG_RMBTRAP       = $00010000;  { Catch RMB events for your own }
 WFLG_NOCAREREFRESH = $00020000;  { not to be bothered with REFRESH }

{ - V36 new Flags which the programmer may specify in NewWindow.Flags  }
 WFLG_NW_EXTENDED   = $00040000;  { extension data provided      }
                                        { see struct ExtNewWindow      }

{ - V39 new Flags which the programmer may specify in NewWindow.Flags  }
 WFLG_NEWLOOKMENUS  = $00200000;  { window has NewLook menus     }

{ These flags are set only by Intuition.  YOU MAY NOT SET THEM YOURSELF! }
 WFLG_WINDOWACTIVE  = $00002000;  { this window is the active one }
 WFLG_INREQUEST     = $00004000;  { this window is in request mode }
 WFLG_MENUSTATE     = $00008000;  { Window is active with Menus on }
 WFLG_WINDOWREFRESH = $01000000;  { Window is currently refreshing }
 WFLG_WBENCHWINDOW  = $02000000;  { WorkBench tool ONLY Window }
 WFLG_WINDOWTICKED  = $04000000;  { only one timer tick at a time }

{ --- V36 Flags to be set only by Intuition -------------------------  }
 WFLG_VISITOR       = $08000000;  { visitor window               }
 WFLG_ZOOMED        = $10000000;  { identifies "zoom state"      }
 WFLG_HASZOOM       = $20000000;  { windowhas a zoom gadget      }

{ --- Other Window Values ---------------------------------------------- }
 DEFAULTMOUSEQUEUE  =     (5);     { no more mouse messages       }

{ --- see struct IntuiMessage for the IDCMP Flag definitions ------------- }


{ ======================================================================== }
{ === NewWindow ========================================================== }
{ ======================================================================== }

Type

    pNewWindow = ^tNewWindow;
    tNewWindow = record
        LeftEdge,
        TopEdge         : smallint;        { screen dimensions of window }
        Width,
        Height          : smallint;        { screen dimensions of window }

        DetailPen,
        BlockPen        : Byte;         { for bar/border/gadget rendering }

        IDCMPFlags      : ULONG;      { User-selected IDCMP flags }

        Flags           : ULONG;      { see Window struct for defines }

    { You supply a linked-list of Gadgets for your Window.
     *  This list DOES NOT include system Gadgets.  You get the standard
     *  system Window Gadgets by setting flag-bits in the variable Flags (see
     *  the bit definitions under the Window structure definition)
     }

        FirstGadget     : pGadget;

    { the CheckMark is a pointer to the imagery that will be used when
     * rendering MenuItems of this Window that want to be checkmarked
     * if this is equal to NULL, you'll get the default imagery
     }

        CheckMark       : pImage;

        Title           : STRPTR;  { the title text for this window }

    { the Screen pointer is used only if you've defined a CUSTOMSCREEN and
     * want this Window to open in it.  If so, you pass the Pointer of the
     * Custom Screen structure in this variable.  Otherwise, this variable
     * is ignored and doesn't have to be initialized.
     }

        Screen          : Pointer;

    { SUPER_BITMAP Window?  If so, put the Pointer of your BitMap structure
     * in this variable.  If not, this variable is ignored and doesn't have
     * to be initialized
     }

        BitMap          : pBitMap;

    { the values describe the minimum and maximum sizes of your Windows.
     * these matter only if you've chosen the WINDOWSIZING Gadget option,
     * which means that you want to let the User to change the size of
     * this Window.  You describe the minimum and maximum sizes that the
     * Window can grow by setting these variables.  You can initialize
     * any one these to zero, which will mean that you want to duplicate
     * the setting for that dimension (if MinWidth == 0, MinWidth will be
     * set to the opening Width of the Window).
     * You can change these settings later using SetWindowLimits().
     * If you haven't asked for a SIZING Gadget, you don't have to
     * initialize any of these variables.
     }

        MinWidth,
        MinHeight       : smallint;        { minimums }
        MaxWidth,
        MaxHeight       : smallint;        { maximums }

    { the type variable describes the Screen in which you want this Window to
     * open.  The type value can either be CUSTOMSCREEN or one of the
     * system standard Screen Types such as WBENCHSCREEN.  See the
     * type definitions under the Screen structure
     }

        WType           : Word;        { is "Type" in C includes }
    end;


{ The following structure is the future NewWindow.  Compatibility
 * issues require that the size of NewWindow not change.
 * Data in the common part (NewWindow) indicates the the extension
 * fields are being used.
 * NOTE WELL: This structure may be subject to future extension.
 * Writing code depending on its size is not allowed.
 }
   pExtNewWindow = ^tExtNewWindow;
   tExtNewWindow = record
    LeftEdge, TopEdge : smallint;
    Width, Height : smallint;

    DetailPen, BlockPen : Byte;
    IDCMPFlags    : ULONG;
    Flags         : ULONG;
    FirstGadget   : pGadget;

    CheckMark     : pImage;

    Title         : STRPTR;
    WScreen       : Pointer;
    WBitMap       : pBitMap;

    MinWidth, MinHeight : smallint;
    MaxWidth, MaxHeight : Word;

    { the type variable describes the Screen in which you want this Window to
     * open.  The type value can either be CUSTOMSCREEN or one of the
     * system standard Screen Types such as WBENCHSCREEN.  See the
     * type definitions under the Screen structure.
     * A new possible value for this field is PUBLICSCREEN, which
     * defines the window as a 'visitor' window.  See below for
     * additional information provided.
     }
    WType  : Word;

    { ------------------------------------------------------- *
     * extensions for V36
     * if the NewWindow Flag value WFLG_NW_EXTENDED is set, then
     * this field is assumed to point to an array ( or chain of arrays)
     * of TagItem structures.  See also ExtNewScreen for another
     * use of TagItems to pass optional data.
     *
     * see below for tag values and the corresponding data.
     }
    Extension : pTagItem;
  END;

{
 * The TagItem ID's (ti_Tag values) for OpenWindowTagList() follow.
 * They are values in a TagItem array passed as extension/replacement
 * values for the data in NewWindow.  OpenWindowTagList() can actually
 * work well with a NULL NewWindow pointer.
 }
CONST
 WA_Dummy     =   (TAG_USER + 99); { $80000063   }

{ these tags simply override NewWindow parameters }
 WA_Left               =  (WA_Dummy + $01);
 WA_Top                =  (WA_Dummy + $02);
 WA_Width              =  (WA_Dummy + $03);
 WA_Height             =  (WA_Dummy + $04);
 WA_DetailPen          =  (WA_Dummy + $05);
 WA_BlockPen           =  (WA_Dummy + $06);
 WA_IDCMP              =  (WA_Dummy + $07);
                        { "bulk" initialization of NewWindow.Flags }
 WA_Flags              =  (WA_Dummy + $08);
 WA_Gadgets            =  (WA_Dummy + $09);
 WA_Checkmark          =  (WA_Dummy + $0A);
 WA_Title              =  (WA_Dummy + $0B);
                        { means you don't have to call SetWindowTitles
                         * after you open your window
                         }
 WA_ScreenTitle        =  (WA_Dummy + $0C);
 WA_CustomScreen       =  (WA_Dummy + $0D);
 WA_SuperBitMap        =  (WA_Dummy + $0E);
                        { also implies WFLG_SUPER_BITMAP property      }
 WA_MinWidth           =  (WA_Dummy + $0F);
 WA_MinHeight          =  (WA_Dummy + $10);
 WA_MaxWidth           =  (WA_Dummy + $11);
 WA_MaxHeight          =  (WA_Dummy + $12);

{ The following are specifications for new features    }

 WA_InnerWidth         =  (WA_Dummy + $13);
 WA_InnerHeight        =  (WA_Dummy + $14);
                        { You can specify the dimensions of the interior
                         * region of your window, independent of what
                         * the border widths will be.  You probably want
                         * to also specify WA_AutoAdjust to allow
                         * Intuition to move your window or even
                         * shrink it so that it is completely on screen.
                         }

 WA_PubScreenName      =  (WA_Dummy + $15);
                        { declares that you want the window to open as
                         * a visitor on the public screen whose name is
                         * pointed to by (UBYTE *) ti_Data
                         }
 WA_PubScreen          =  (WA_Dummy + $16);
                        { open as a visitor window on the public screen
                         * whose Pointer is in (struct Screen *) ti_Data.
                         * To ensure that this screen remains open, you
                         * should either be the screen's owner, have a
                         * window open on the screen, or use LockPubScreen().
                         }
 WA_PubScreenFallBack  =  (WA_Dummy + $17);
                        { A Boolean, specifies whether a visitor window
                         * should "fall back" to the default public screen
                         * (or Workbench) if the named public screen isn't
                         * available
                         }
 WA_WindowName         =  (WA_Dummy + $18);
                        { not implemented      }
 WA_Colors             =  (WA_Dummy + $19);
                        { a ColorSpec array for colors to be set
                         * when this window is active.  This is not
                         * implemented, and may not be, since the default
                         * values to restore would be hard to track.
                         * We'd like to at least support per-window colors
                         * for the mouse pointer sprite.
                         }
 WA_Zoom       =  (WA_Dummy + $1A);
                        { ti_Data points to an array of four WORD's,
                         * the initial Left/Top/Width/Height values of
                         * the "alternate" zoom position/dimensions.
                         * It also specifies that you want a Zoom gadget
                         * for your window, whether or not you have a
                         * sizing gadget.
                         }
 WA_MouseQueue         =  (WA_Dummy + $1B);
                        { ti_Data contains initial value for the mouse
                         * message backlog limit for this window.
                         }
 WA_BackFill           =  (WA_Dummy + $1C);
                        { unimplemented at present: provides a "backfill
                         * hook" for your window's layer.
                         }
 WA_RptQueue           =  (WA_Dummy + $1D);
                        { initial value of repeat key backlog limit    }

    { These Boolean tag items are alternatives to the NewWindow.Flags
     * boolean flags with similar names.
     }
 WA_SizeGadget         =  (WA_Dummy + $1E);
 WA_DragBar            =  (WA_Dummy + $1F);
 WA_DepthGadget        =  (WA_Dummy + $20);
 WA_CloseGadget        =  (WA_Dummy + $21);
 WA_Backdrop           =  (WA_Dummy + $22);
 WA_ReportMouse        =  (WA_Dummy + $23);
 WA_NoCareRefresh      =  (WA_Dummy + $24);
 WA_Borderless         =  (WA_Dummy + $25);
 WA_Activate           =  (WA_Dummy + $26);
 WA_RMBTrap            =  (WA_Dummy + $27);
 WA_WBenchWindow       =  (WA_Dummy + $28);       { PRIVATE!! }
 WA_SimpleRefresh      =  (WA_Dummy + $29);
                        { only specify if TRUE }
 WA_SmartRefresh       =  (WA_Dummy + $2A);
                        { only specify if TRUE }
 WA_SizeBRight         =  (WA_Dummy + $2B);
 WA_SizeBBottom        =  (WA_Dummy + $2C);

    { New Boolean properties   }
 WA_AutoAdjust         =  (WA_Dummy + $2D);
                        { shift or squeeze the window's position and
                         * dimensions to fit it on screen.
                         }

 WA_GimmeZeroZero      =  (WA_Dummy + $2E);
                        { equiv. to NewWindow.Flags WFLG_GIMMEZEROZERO }

{ New for V37: WA_MenuHelp (ignored by V36) }
 WA_MenuHelp           =  (WA_Dummy + $2F);
                        { Enables IDCMP_MENUHELP:  Pressing HELP during menus
                         * will return IDCMP_MENUHELP message.
                         }

{ New for V39:  (ignored by V37 and earlier) }
 WA_NewLookMenus       =  (WA_Dummy + $30);
                        { Set to TRUE if you want NewLook menus }
 WA_AmigaKey           =  (WA_Dummy + $31);
                        { Pointer to image for Amiga-key equiv in menus }
 WA_NotifyDepth        =  (WA_Dummy + $32);
                        { Requests IDCMP_CHANGEWINDOW message when
                         * window is depth arranged
                         * (imsg->Code = CWCODE_DEPTH)
                         }

{ WA_Dummy + $33 is obsolete }

 WA_Pointer            =  (WA_Dummy + $34);
                        { Allows you to specify a custom pointer
                         * for your window.  ti_Data points to a
                         * pointer object you obtained via
                         * "pointerclass". NULL signifies the
                         * default pointer.
                         * This tag may be passed to OpenWindowTags()
                         * or SetWindowPointer().
                         }

 WA_BusyPointer        =  (WA_Dummy + $35);
                        { ti_Data is boolean.  Set to TRUE to
                         * request the standard busy pointer.
                         * This tag may be passed to OpenWindowTags()
                         * or SetWindowPointer().
                         }

 WA_PointerDelay       =  (WA_Dummy + $36);
                        { ti_Data is boolean.  Set to TRUE to
                         * request that the changing of the
                         * pointer be slightly delayed.  The change
                         * will be called off if you call NewSetPointer()
                         * before the delay expires.  This allows
                         * you to post a busy-pointer even if you think
                         * the busy-time may be very Word, without
                         * fear of a flashing pointer.
                         * This tag may be passed to OpenWindowTags()
                         * or SetWindowPointer().
                         }

 WA_TabletMessages     =  (WA_Dummy + $37);
                        { ti_Data is a boolean.  Set to TRUE to
                         * request that tablet information be included
                         * in IntuiMessages sent to your window.
                         * Requires that something (i.e. a tablet driver)
                         * feed IESUBCLASS_NEWTABLET InputEvents into
                         * the system.  For a pointer to the TabletData,
                         * examine the ExtIntuiMessage->eim_TabletData
                         * field.  It is UNSAFE to check this field
                         * when running on pre-V39 systems.  It's always
                         * safe to check this field under V39 and up,
                         * though it may be NULL.
                         }

 WA_HelpGroup          =  (WA_Dummy + $38);
                        { When the active window has gadget help enabled,
                         * other windows of the same HelpGroup number
                         * will also get GadgetHelp.  This allows GadgetHelp
                         * to work for multi-windowed applications.
                         * Use GetGroupID() to get an ID number.  Pass
                         * this number as ti_Data to all your windows.
                         * See also the HelpControl() function.
                         }

 WA_HelpGroupWindow    =  (WA_Dummy + $39);
                        { When the active window has gadget help enabled,
                         * other windows of the same HelpGroup will also get
                         * GadgetHelp.  This allows GadgetHelp to work
                         * for multi-windowed applications.  As an alternative
                         * to WA_HelpGroup, you can pass a pointer to any
                         * other window of the same group to join its help
                         * group.  Defaults to NULL, which has no effect.
                         * See also the HelpControl() function.
                         }


{ HelpControl() flags:
 *
 * HC_GADGETHELP - Set this flag to enable Gadget-Help for one or more
 * windows.
 }

 HC_GADGETHELP  = 1;


{ ======================================================================== }
{ === Remember =========================================================== }
{ ======================================================================== }
{ this structure is used for remembering what memory has been allocated to
 * date by a given routine, so that a premature abort or systematic exit
 * can deallocate memory cleanly, easily, and completely
 }

Type

    pRemember = ^tRemember;
    tRemember = record
        NextRemember    : pRemember;
        RememberSize    : ULONG;
        Memory          : Pointer;
    end;


{ === Color Spec ====================================================== }
{ How to tell Intuition about RGB values for a color table entry. }

  pColorSpec = ^tColorSpec;
  tColorSpec = record
    ColorIndex  : smallint;     { -1 terminates an array of ColorSpec  }
    Red         : Word;     { only the _bottom_ 4 bits recognized }
    Green       : Word;     { only the _bottom_ 4 bits recognized }
    Blue        : Word;     { only the _bottom_ 4 bits recognized }
  END;

{ === Easy Requester Specification ======================================= }
{ see also autodocs for EasyRequest and BuildEasyRequest       }
{ NOTE: This structure may grow in size in the future          }

   pEasyStruct = ^tEasyStruct;
   tEasyStruct = record
    es_StructSize   : ULONG;  { should be sizeof (struct EasyStruct )}
    es_Flags        : ULONG;  { should be 0 for now                  }
    es_Title        : STRPTR;   { title of requester window            }
    es_TextFormat   : STRPTR;   { 'printf' style formatting string     }
    es_GadgetFormat : STRPTR;   { 'printf' style formatting string   }
   END;



{ ======================================================================== }
{ === Miscellaneous ====================================================== }
{ ======================================================================== }
CONST
{ = MENU STUFF =========================================================== }
    NOMENU      = $001F;
    NOITEM      = $003F;
    NOSUB       = $001F;
    MENUNULL    = -1;


{ = =RJ='s peculiarities ================================================= }

{ these defines are for the COMMSEQ and CHECKIT menu stuff.  If CHECKIT,
 * I'll use a generic Width (for all resolutions) for the CheckMark.
 * If COMMSEQ, likewise I'll use this generic stuff
 }

    CHECKWIDTH          = 19;
    COMMWIDTH           = 27;
    LOWCHECKWIDTH       = 13;
    LOWCOMMWIDTH        = 16;

{ these are the AlertNumber defines.  if you are calling DisplayAlert()
 * the AlertNumber you supply must have the ALERT_TYPE bits set to one
 * of these patterns
 }

    ALERT_TYPE          = $80000000;
    RECOVERY_ALERT      = $00000000;    { the system can recover from this }
    DEADEND_ALERT       = $80000000;    { no recovery possible, this is it }


{ When you're defining IntuiText for the Positive and Negative Gadgets
 * created by a call to AutoRequest(), these defines will get you
 * reasonable-looking text.  The only field without a define is the IText
 * field; you decide what text goes with the Gadget
 }

    AUTOFRONTPEN        = 0;
    AUTOBACKPEN         = 1;
    AUTODRAWMODE        = JAM2;
    AUTOLEFTEDGE        = 6;
    AUTOTOPEDGE         = 3;

{ -
    AUTOITEXTFONT       = Nil;
    AUTONEXTTEXT        = Nil;
- }


{ --- RAWMOUSE Codes and Qualifiers (Console OR IDCMP) ------------------- }


    SELECTUP            = IECODE_LBUTTON + IECODE_UP_PREFIX;
    SELECTDOWN          = IECODE_LBUTTON;
    MENUUP              = IECODE_RBUTTON + IECODE_UP_PREFIX;
    MENUDOWN            = IECODE_RBUTTON;
    ALTLEFT             = IEQUALIFIER_LALT;
    ALTRIGHT            = IEQUALIFIER_RALT;
    AMIGALEFT           = IEQUALIFIER_LCOMMAND;
    AMIGARIGHT          = IEQUALIFIER_RCOMMAND;
    AMIGAKEYS           = AMIGALEFT + AMIGARIGHT;

    CURSORUP            = $4C;
    CURSORLEFT          = $4F;
    CURSORRIGHT         = $4E;
    CURSORDOWN          = $4D;
    KEYCODE_Q           = $10;
    KEYCODE_X           = $32;
    KEYCODE_N           = $36;
    KEYCODE_M           = $37;
    KEYCODE_V           = $34;
    KEYCODE_B           = $35;
    KEYCODE_LESS        = $38;
    KEYCODE_GREATER     = $39;

{ these are the display modes for which we have corresponding parameter
 *  settings in the config arrays
 }
CONST
 DMODECOUNT    =  $0002;  { how many modes there are }
 HIRESPICK     =  $0000;
 LOWRESPICK    =  $0001;

 EVENTMAX = 10;             { size of event array }

{ these are the system Gadget defines }
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


{ ======================================================================== }
{ === DrawInfo ========================================================= }
{ ======================================================================== }

{ This is a packet of information for graphics rendering.  It originates
 * with a Screen, and is gotten using GetScreenDrawInfo( screen );
 }

{ If you find dri_Version >= DRI_VERSION, you know this structure
 * has at least the fields defined in this version of the include file
 }
CONST
 RI_VERSION  =    (1);     { obsolete, will be removed            }
 DRI_VERSION =    (1);

Type

 pDrawInfo = ^tDrawInfo;
 tDrawInfo = record
    dri_Version : Word;    { will be  DRI_VERSION                 }
    dri_NumPens : Word;    { guaranteed to be >= numDrIPens       }
    dri_Pens    : Pointer;  { pointer to pen array                 }

    dri_Font    : pTextFont;      { screen default font          }
    dri_Depth   : Word;            { (initial) depth of screen bitmap     }

    dri_Resolution : record      { from DisplayInfo database for initial display mode }
                   x : word;
                   y : word;
                   end;

    dri_Flags : ULONG;              { defined below                }
{ New for V39: dri_CheckMark, dri_AmigaKey. }
    dri_CheckMark : pImage; { ImagePtr }         { pointer to scaled checkmark image
                                                  * Will be NULL if DRI_VERSION < 2
                                                  }
    dri_AmigaKey  : pImage; { ImagePtr }    { pointer to scaled Amiga-key image
                                             * Will be NULL if DRI_VERSION < 2
                                             }

    dri_Reserved : Array[0..4] of ULONG;        { avoid recompilation ;^)      }
 END;

CONST
 DRIF_NEWLOOK =   $00000001;      { specified SA_Pens, full treatment }

{ rendering pen number indexes into DrawInfo.dri_Pens[]        }
 DETAILPEN    =    ($0000);       { compatible Intuition rendering pens  }
 BLOCKPEN     =    ($0001);       { compatible Intuition rendering pens  }
 TEXTPEN      =    ($0002);       { text on background                   }
 SHINEPEN     =    ($0003);       { bright edge on 3D objects            }
 SHADOWPEN    =    ($0004);       { dark edge on 3D objects              }
 FILLPEN      =    ($0005);       { active-window/selected-gadget fill   }
 FILLTEXTPEN  =    ($0006);       { text over FILLPEN                    }
 BACKGROUNDPEN =   ($0007);       { always color 0                       }
 HIGHLIGHTTEXTPEN = ($0008);       { special color text, on background    }
{ New for V39, only present if DRI_VERSION >= 2: }
 BARDETAILPEN   =  ($0009);       { text/detail in screen-bar/menus }
 BARBLOCKPEN    =  ($000A);       { screen-bar/menus fill }
 BARTRIMPEN     =  ($000B);       { trim under screen-bar }

 NUMDRIPENS   =    ($0009);

{ New for V39:  It is sometimes useful to specify that a pen value
 * is to be the complement of color zero to three.  The "magic" numbers
 * serve that purpose:
 }
 PEN_C3        =  $FEFC;          { Complement of color 3 }
 PEN_C2        =  $FEFD;          { Complement of color 2 }
 PEN_C1        =  $FEFE;          { Complement of color 1 }
 PEN_C0        =  $FEFF;          { Complement of color 0 }

{ ======================================================================== }
{ === Screen ============================================================= }
{ ======================================================================== }

Type

    pScreen = ^tScreen;
    tScreen = record
        NextScreen      : pScreen;      { linked list of screens }
        FirstWindow     : pWindow;      { linked list Screen's Windows }

        LeftEdge,
        TopEdge         : smallint;        { parameters of the screen }
        Width,
        Height          : smallint;        { parameters of the screen }

        MouseY,
        MouseX          : smallint;        { position relative to upper-left }

        Flags           : Word;        { see definitions below }

        Title           : STRPTR;       { null-terminated Title text }
        DefaultTitle    : STRPTR;       { for Windows without ScreenTitle }

    { Bar sizes for this Screen and all Window's in this Screen }
        BarHeight,
        BarVBorder,
        BarHBorder,
        MenuVBorder,
        MenuHBorder     : Shortint;
        WBorTop,
        WBorLeft,
        WBorRight,
        WBorBottom      : Shortint;

        Font            : pTextAttr;  { this screen's default font       }

    { the display data structures for this Screen (note the prefix S)}
        ViewPort        : tViewPort;     { describing the Screen's display }
        RastPort        : tRastPort;     { describing Screen rendering      }
        BitMap          : tBitMap;       { extra copy of RastPort BitMap   }
        LayerInfo       : tLayer_Info;   { each screen gets a LayerInfo     }

    { You supply a linked-list of Gadgets for your Screen.
     *  This list DOES NOT include system Gadgets.  You get the standard
     *  system Screen Gadgets by default
     }

        FirstGadget     : pGadget;

        DetailPen,
        BlockPen        : Byte;         { for bar/border/gadget rendering }

    { the following variable(s) are maintained by Intuition to support the
     * DisplayBeep() color flashing technique
     }
        SaveColor0      : Word;

    { This layer is for the Screen and Menu bars }
        BarLayer        : pLayer;

        ExtData         : Pointer;
        UserData        : Pointer;
                        { general-purpose pointer to User data extension }
    {**** Data below this point are SYSTEM PRIVATE ****}

    end;

Const

{ The screen flags have the suffix "_f" added to avoid conflicts with
  routine names. }

{ --- FLAGS SET BY INTUITION --------------------------------------------- }
{ The SCREENTYPE bits are reserved for describing various Screen types
 * available under Intuition.
 }
    SCREENTYPE_f        = $000F;        { all the screens types available       }
{ --- the definitions for the Screen Type ------------------------------- }
    WBENCHSCREEN_f      = $0001;        { Ta Da!  The Workbench         }
    CUSTOMSCREEN_f      = $000F;        { for that special look         }

    SHOWTITLE_f         = $0010;        { this gets set by a call to ShowTitle() }

    BEEPING_f           = $0020;        { set when Screen is beeping            }

    CUSTOMBITMAP_f      = $0040;        { if you are supplying your own BitMap }

    SCREENBEHIND_f      = $0080;        { if you want your screen to open behind
                                         * already open screens
                                         }
    SCREENQUIET_f       = $0100;        { if you do not want Intuition to render
                                         * into your screen (gadgets, title)     }
    SCREENHIRES         = $0200;        { do no use lowres gadgets (private)       }

    NS_EXTENDED         = $1000;          { ExtNewScreen.Extension is valid      }
    { V36 applications can use OpenScreenTagList() instead of NS_EXTENDED  }

{ New for V39: }
    PENSHARED           = $0400;  { Screen opener set (SA_SharePens,TRUE) }


    AUTOSCROLL          = $4000;  { screen is to autoscoll               }

    STDSCREENHEIGHT     = -1;           { supply in NewScreen.Height            }
    STDSCREENWIDTH      = -1;           { supply in NewScreen.Width             }



{
 * Screen attribute tag ID's.  These are used in the ti_Tag field of
 * TagItem arrays passed to OpenScreenTagList() (or in the
 * ExtNewScreen.Extension field).
 }

{ Screen attribute tags.  Please use these versions, not those in
 * iobsolete.h.
 }
CONST
  SA_Dummy    =    (TAG_USER + 32);
{
 * these items specify items equivalent to fields in NewScreen
 }
 SA_Left     =    (SA_Dummy + $0001);
 SA_Top      =    (SA_Dummy + $0002);
 SA_Width    =    (SA_Dummy + $0003);
 SA_Height   =    (SA_Dummy + $0004);
                        { traditional screen positions and dimensions  }
 SA_Depth    =    (SA_Dummy + $0005);
                        { screen bitmap depth                          }
 SA_DetailPen=    (SA_Dummy + $0006);
                        { serves as default for windows, too           }
 SA_BlockPen =    (SA_Dummy + $0007);
 SA_Title    =    (SA_Dummy + $0008);
                        { default screen title                         }
 SA_Colors   =    (SA_Dummy + $0009);
                        { ti_Data is an array of struct ColorSpec,
                         * terminated by ColorIndex = -1.  Specifies
                         * initial screen palette colors.
                         }
 SA_ErrorCode=    (SA_Dummy + $000A);
                        { ti_Data points to LONG error code (values below)}
 SA_Font     =    (SA_Dummy + $000B);
                        { equiv. to NewScreen.Font                     }
 SA_SysFont  =    (SA_Dummy + $000C);
                        { Selects one of the preferences system fonts:
                         *      0 - old DefaultFont, fixed-width
                         *      1 - WB Screen preferred font
                         }
 SA_Type     =    (SA_Dummy + $000D);
                        { equiv. to NewScreen.Type                     }
 SA_BitMap   =    (SA_Dummy + $000E);
                        { ti_Data is pointer to custom BitMap.  This
                         * implies type of CUSTOMBITMAP
                         }
 SA_PubName  =    (SA_Dummy + $000F);
                        { presence of this tag means that the screen
                         * is to be a public screen.  Please specify
                         * BEFORE the two tags below
                         }
 SA_PubSig   =    (SA_Dummy + $0010);
 SA_PubTask  =    (SA_Dummy + $0011);
                        { Task ID and signal for being notified that
                         * the last window has closed on a public screen.
                         }
 SA_DisplayID=    (SA_Dummy + $0012);
                        { ti_Data is new extended display ID from
                         * <graphics/displayinfo.h>.
                         }
 SA_DClip    =    (SA_Dummy + $0013);
                        { ti_Data points to a rectangle which defines
                         * screen display clip region
                         }
 SA_Overscan =    (SA_Dummy + $0014);
                        { was S_STDDCLIP.  Set to one of the OSCAN_
                         * specifiers below to get a system standard
                         * overscan region for your display clip,
                         * screen dimensions (unless otherwise specified),
                         * and automatically centered position (partial
                         * support only so far).
                         * If you use this, you shouldn't specify
                         * SA_DClip.  SA_Overscan is for "standard"
                         * overscan dimensions, SA_DClip is for
                         * your custom numeric specifications.
                         }
 SA_Obsolete1=    (SA_Dummy + $0015);
                        { obsolete S_MONITORNAME                       }

{* booleans *}
 SA_ShowTitle  =  (SA_Dummy + $0016);
                        { boolean equivalent to flag SHOWTITLE         }
 SA_Behind     =  (SA_Dummy + $0017);
                        { boolean equivalent to flag SCREENBEHIND      }
 SA_Quiet      =  (SA_Dummy + $0018);
                        { boolean equivalent to flag SCREENQUIET       }
 SA_AutoScroll =  (SA_Dummy + $0019);
                        { boolean equivalent to flag AUTOSCROLL        }
 SA_Pens       =  (SA_Dummy + $001A);
                        { pointer to ~0 terminated UWORD array, as
                         * found in struct DrawInfo
                         }
 SA_FullPalette=  (SA_Dummy + $001B);
                        { boolean: initialize color table to entire
                         *  preferences palette (32 for V36), rather
                         * than compatible pens 0-3, 17-19, with
                         * remaining palette as returned by GetColorMap()
                         }

 SA_ColorMapEntries = (SA_Dummy + $001C);
                        { New for V39:
                         * Allows you to override the number of entries
                         * in the ColorMap for your screen.  Intuition
                         * normally allocates (1<<depth) or 32, whichever
                         * is more, but you may require even more if you
                         * use certain V39 graphics.library features
                         * (eg. palette-banking).
                         }

 SA_Parent      = (SA_Dummy + $001D);
                        { New for V39:
                         * ti_Data is a pointer to a "parent" screen to
                         * attach this one to.  Attached screens slide
                         * and depth-arrange together.
                         }

 SA_Draggable   = (SA_Dummy + $001E);
                        { New for V39:
                         * Boolean tag allowing non-draggable screens.
                         * Do not use without good reason!
                         * (Defaults to TRUE).
                         }

 SA_Exclusive   = (SA_Dummy + $001F);
                        { New for V39:
                         * Boolean tag allowing screens that won't share
                         * the display.  Use sparingly!  Starting with 3.01,
                         * attached screens may be SA_Exclusive.  Setting
                         * SA_Exclusive for each screen will produce an
                         * exclusive family.   (Defaults to FALSE).
                         }

 SA_SharePens   = (SA_Dummy + $0020);
                        { New for V39:
                         * For those pens in the screen's DrawInfo->dri_Pens,
                         * Intuition obtains them in shared mode (see
                         * graphics.library/ObtainPen()).  For compatibility,
                         * Intuition obtains the other pens of a public
                         * screen as PEN_EXCLUSIVE.  Screens that wish to
                         * manage the pens themselves should generally set
                         * this tag to TRUE.  This instructs Intuition to
                         * leave the other pens unallocated.
                         }

 SA_BackFill    = (SA_Dummy + $0021);
                        { New for V39:
                         * provides a "backfill hook" for your screen's
                         * Layer_Info.
                         * See layers.library/InstallLayerInfoHook()
                         }

 SA_Interleaved = (SA_Dummy + $0022);
                        { New for V39:
                         * Boolean tag requesting that the bitmap
                         * allocated for you be interleaved.
                         * (Defaults to FALSE).
                         }

 SA_Colors32    = (SA_Dummy + $0023);
                        { New for V39:
                         * Tag to set the screen's initial palette colors
                         * at 32 bits-per-gun.  ti_Data is a pointer
                         * to a table to be passed to the
                         * graphics.library/LoadRGB32() function.
                         * This format supports both runs of color
                         * registers and sparse registers.  See the
                         * autodoc for that function for full details.
                         * Any color set here has precedence over
                         * the same register set by SA_Colors.
                         }

 SA_VideoControl = (SA_Dummy + $0024);
                        { New for V39:
                         * ti_Data is a pointer to a taglist that Intuition
                         * will pass to graphics.library/VideoControl(),
                         * upon opening the screen.
                         }

 SA_FrontChild  = (SA_Dummy + $0025);
                        { New for V39:
                         * ti_Data is a pointer to an already open screen
                         * that is to be the child of the screen being
                         * opened.  The child screen will be moved to the
                         * front of its family.
                         }

 SA_BackChild   = (SA_Dummy + $0026);
                        { New for V39:
                         * ti_Data is a pointer to an already open screen
                         * that is to be the child of the screen being
                         * opened.  The child screen will be moved to the
                         * back of its family.
                         }

 SA_LikeWorkbench     =   (SA_Dummy + $0027);
                        { New for V39:
                         * Set ti_Data to 1 to request a screen which
                         * is just like the Workbench.  This gives
                         * you the same screen mode, depth, size,
                         * colors, etc., as the Workbench screen.
                         }

 SA_Reserved          =   (SA_Dummy + $0028);
                        { Reserved for private Intuition use }

 SA_MinimizeISG       =   (SA_Dummy + $0029);
                        { New for V40:
                         * For compatibility, Intuition always ensures
                         * that the inter-screen gap is at least three
                         * non-interlaced lines.  If your application
                         * would look best with the smallest possible
                         * inter-screen gap, set ti_Data to TRUE.
                         * If you use the new graphics VideoControl()
                         * VC_NoColorPaletteLoad tag for your screen's
                         * ViewPort, you should also set this tag.
                          }


{ this is an obsolete tag included only for compatibility with V35
 * interim release for the A2024 and Viking monitors
 }
 NSTAG_EXT_VPMODE = (TAG_USER + 1);


{ OpenScreen error codes, which are returned in the (optional) LONG
 * pointed to by ti_Data for the SA_ErrorCode tag item
 }
 OSERR_NOMONITOR   = (1);     { named monitor spec not available     }
 OSERR_NOCHIPS     = (2);     { you need newer custom chips          }
 OSERR_NOMEM       = (3);     { couldn't get normal memory           }
 OSERR_NOCHIPMEM   = (4);     { couldn't get chipmem                 }
 OSERR_PUBNOTUNIQUE= (5);     { public screen name already used      }
 OSERR_UNKNOWNMODE = (6);     { don't recognize mode asked for       }

{ ======================================================================== }
{ === NewScreen ========================================================== }
{ ======================================================================== }

Type

    pNewScreen = ^tNewScreen;
    tNewScreen = record
        LeftEdge,
        TopEdge,
        Width,
        Height,
        Depth           : smallint;        { screen dimensions }

        DetailPen,
        BlockPen        : Byte;         { for bar/border/gadget rendering }

        ViewModes       : Word;        { the Modes for the ViewPort (and View) }

        SType           : Word;        { the Screen type (see defines above) }

        Font            : pTextAttr;  { this Screen's default text attributes }

        DefaultTitle    : STRPTR;       { the default title for this Screen }

        Gadgets         : pGadget;      { your own Gadgets for this Screen }

    { if you are opening a CUSTOMSCREEN and already have a BitMap
     * that you want used for your Screen, you set the flags CUSTOMBITMAP in
     * the Type field and you set this variable to point to your BitMap
     * structure.  The structure will be copied into your Screen structure,
     * after which you may discard your own BitMap if you want
     }

        CustomBitMap    : pBitMap;
    end;


type

 pExtNewScreen = ^tExtNewScreen;
 tExtNewScreen = record
  LeftEdge, TopEdge, Width, Height, Depth : smallint;
  DetailPen, BlockPen : Byte;
  ViewModes : Word;
  ens_Type : Word;     { Type in C-Includes }
  Font : pTextAttr;
  DefaultTitle : STRPTR;
  Gadgets : pGadget;
  CustomBitMap : pBitMap;
  Extension : pTagItem;
 END;


CONST
{ === Overscan Types ===       }
 OSCAN_TEXT     = (1);     { entirely visible     }
 OSCAN_STANDARD = (2);     { just past edges      }
 OSCAN_MAX      = (3);     { as much as possible  }
 OSCAN_VIDEO    = (4);     { even more than is possible   }


{ === Public Shared Screen Node ===    }

{ This is the representative of a public shared screen.
 * This is an internal data structure, but some functions may
 * present a copy of it to the calling application.  In that case,
 * be aware that the screen pointer of the structure can NOT be
 * used safely, since there is no guarantee that the referenced
 * screen will remain open and a valid data structure.
 *
 * Never change one of these.
 }

Type
   pPubScreenNode = ^tPubScreenNode;
   tPubScreenNode = record
    psn_Node    : tNode;       { ln_Name is screen name }
    psn_Screen  : pScreen;
    psn_Flags   : Word;      { below                }
    psn_Size    : smallint;      { includes name buffer }
    psn_VisitorCount  : smallint; { how many visitor windows }
    psn_SigTask : pTask;    { who to signal when visitors gone }
    psn_SigBit  : Byte;     { which signal }
   END;

CONST
 PSNF_PRIVATE  =  ($0001);

 MAXPUBSCREENNAME  =      (139);   { names no longer, please      }

{ pub screen modes     }
 SHANGHAI      =  $0001;  { put workbench windows on pub screen }
 POPPUBSCREEN  =  $0002;  { pop pub screen to front when visitor opens }

{ New for V39:  Intuition has new screen depth-arrangement and movement
 * functions called ScreenDepth() and ScreenPosition() respectively.
 * These functions permit the old behavior of ScreenToFront(),
 * ScreenToBack(), and MoveScreen().  ScreenDepth() also allows
 * independent depth control of attached screens.  ScreenPosition()
 * optionally allows positioning screens even though they were opened
 * (SA_Draggable,FALSE).
 }

{ For ScreenDepth(), specify one of SDEPTH_TOFRONT or SDEPTH_TOBACK,
 * and optionally also SDEPTH_INFAMILY.
 *
 * NOTE: ONLY THE OWNER OF THE SCREEN should ever specify
 * SDEPTH_INFAMILY.  Commodities, "input helper" programs,
 * or any other program that did not open a screen should never
 * use that flag.  (Note that this is a style-behavior
 * requirement;  there is no technical requirement that the
 * task calling this function need be the task which opened
 * the screen).
 }

 SDEPTH_TOFRONT        =  (0);     { Bring screen to front }
 SDEPTH_TOBACK         =  (1);     { Send screen to back }
 SDEPTH_INFAMILY       =  (2);     { Move an attached screen with
                                         * respect to other screens of
                                         * its family
                                         }

{ Here's an obsolete name equivalent to SDEPTH_INFAMILY: }
 SDEPTH_CHILDONLY      =  SDEPTH_INFAMILY;


{ For ScreenPosition(), specify one of SPOS_RELATIVE, SPOS_ABSOLUTE,
 * or SPOS_MAKEVISIBLE to describe the kind of screen positioning you
 * wish to perform:
 *
 * SPOS_RELATIVE: The x1 and y1 parameters to ScreenPosition() describe
 *      the offset in coordinates you wish to move the screen by.
 * SPOS_ABSOLUTE: The x1 and y1 parameters to ScreenPosition() describe
 *      the absolute coordinates you wish to move the screen to.
 * SPOS_MAKEVISIBLE: (x1,y1)-(x2,y2) describes a rectangle on the
 *      screen which you would like autoscrolled into view.
 *
 * You may additionally set SPOS_FORCEDRAG along with any of the
 * above.  Set this if you wish to reposition an (SA_Draggable,FALSE)
 * screen that you opened.
 *
 * NOTE: ONLY THE OWNER OF THE SCREEN should ever specify
 * SPOS_FORCEDRAG.  Commodities, "input helper" programs,
 * or any other program that did not open a screen should never
 * use that flag.
 }

 SPOS_RELATIVE         =  (0);     { Coordinates are relative }

 SPOS_ABSOLUTE         =  (1);     { Coordinates are expressed as
                                         * absolutes, not relatives.
                                         }

 SPOS_MAKEVISIBLE      =  (2);     { Coordinates describe a box on
                                         * the screen you wish to be
                                         * made visible by autoscrolling
                                         }

 SPOS_FORCEDRAG        =  (4);     { Move non-draggable screen }

{ New for V39: Intuition supports double-buffering in screens,
 * with friendly interaction with menus and certain gadgets.
 * For each buffer, you need to get one of these structures
 * from the AllocScreenBuffer() call.  Never allocate your
 * own ScreenBuffer structures!
 *
 * The sb_DBufInfo field is for your use.  See the graphics.library
 * AllocDBufInfo() autodoc for details.
 }
Type

 pScreenBuffer = ^tScreenBuffer;
 tScreenBuffer = record
    sb_BitMap  : pBitMap;           { BitMap of this buffer }
    sb_DBufInfo : pDBufInfo;       { DBufInfo for this buffer }
 end;

const
{ These are the flags that may be passed to AllocScreenBuffer().
 }
 SB_SCREEN_BITMAP      =  1;
 SB_COPY_BITMAP        =  2;


{ ======================================================================== }
{ === Preferences ======================================================== }
{ ======================================================================== }

Const

{ these are the definitions for the printer configurations }
    FILENAME_SIZE       = 30;           { Filename size }

    POINTERSIZE         = (1 + 16 + 1) * 2;     { Size of Pointer data buffer }

{ These defines are for the default font size.   These actually describe the
 * height of the defaults fonts.  The default font type is the topaz
 * font, which is a fixed width font that can be used in either
 * eighty-column or sixty-column mode.  The Preferences structure reflects
 * which is currently selected by the value found in the variable FontSize,
 * which may have either of the values defined below.  These values actually
 * are used to select the height of the default font.  By changing the
 * height, the resolution of the font changes as well.
 }
    TOPAZ_EIGHTY        = 8;
    TOPAZ_SIXTY         = 9;

Type

    pPreferences = ^tPreferences;
    tPreferences = record
    { the default font height }
        FontHeight      : Shortint;         { height for system default font  }

    { constant describing what's hooked up to the port }
        PrinterPort     : Byte;         { printer port connection     }

    { the baud rate of the port }
        BaudRate        : Word;        { baud rate for the serial port   }

    { various timing rates }
        KeyRptSpeed     : ttimeval;      { repeat speed for keyboard       }
        KeyRptDelay     : ttimeval;      { Delay before keys repeat             }
        DoubleClick     : ttimeval;      { Interval allowed between clicks }

    { Intuition Pointer data }
        PointerMatrix   : Array [0..POINTERSIZE-1] of Word;
                                        { Definition of pointer sprite     }
        XOffset         : Shortint;         { X-Offset for active 'bit'       }
        YOffset         : Shortint;         { Y-Offset for active 'bit'       }
        color17         : Word;        {*********************************}
        color18         : Word;        { Colours for sprite pointer      }
        color19         : Word;        {*********************************}
        PointerTicks    : Word;        { Sensitivity of the pointer       }

    { Workbench Screen colors }
        color0          : Word;        {*********************************}
        color1          : Word;        {   Standard default colours      }
        color2          : Word;        {   Used in the Workbench         }
        color3          : Word;        {*********************************}

    { positioning data for the Intuition View }
        ViewXOffset     : Shortint;         { Offset for top lefthand corner  }
        ViewYOffset     : Shortint;         { X and Y dimensions           }
        ViewInitX,
        ViewInitY       : smallint;        { View initial offset values      }

        EnableCLI       : Boolean;      { CLI availability switch }

    { printer configurations }
        PrinterType     : Word;        { printer type                     }
        PrinterFilename : Array [0..FILENAME_SIZE-1] of Char;
                                        { file for printer         }

    { print format and quality configurations }
        PrintPitch      : Word;        { print pitch              }
        PrintQuality    : Word;        { print quality    }
        PrintSpacing    : Word;        { number of lines per inch    }
        PrintLeftMargin : Word;        { left margin in characters        }
        PrintRightMargin : Word;       { right margin in characters       }
        PrintImage      : Word;        { positive or negative         }
        PrintAspect     : Word;        { horizontal or vertical      }
        PrintShade      : Word;        { b&w, half-tone, or color    }
        PrintThreshold  : smallint;        { darkness ctrl for b/w dumps      }

    { print paper descriptors }
        PaperSize       : Word;        { paper size               }
        PaperLength     : Word;        { paper length in number of lines }
        PaperType       : Word;        { continuous or single sheet       }

    { Serial device settings: These are six nibble-fields in three bytes }
    { (these look a little strange so the defaults will map out to zero) }
        SerRWBits       : Byte;
                             { upper nibble = (8-number of read bits)     }
                             { lower nibble = (8-number of write bits)    }
        SerStopBuf      : Byte;
                             { upper nibble = (number of stop bits - 1)  }
                             { lower nibble = (table value for BufSize)  }
        SerParShk       : Byte;
                             { upper nibble = (value for Parity setting) }
                             { lower nibble = (value for Handshake mode) }
        LaceWB          : Byte;         { if workbench is to be interlaced      }

        WorkName        : Array [0..FILENAME_SIZE-1] of Char;
                                        { temp file for printer         }

        RowSizeChange   : Shortint;
        ColumnSizeChange : Shortint;

        PrintFlags      : Word;        { user preference flags }
        PrintMaxWidth   : Word;        { max width of printed picture in 10ths/inch }
        PrintMaxHeight  : Word;        { max height of printed picture in 10ths/inch }
        PrintDensity    : Byte;         { print density }
        PrintXOffset    : Byte;         { offset of printed picture in 10ths/inch }

        wb_Width        : Word;        { override default workbench width       }
        wb_Height       : Word;        { override default workbench height }
        wb_Depth        : Byte;         { override default workbench depth       }

        ext_size        : Byte;         { extension information -- do not touch! }
                            { extension size in blocks of 64 bytes }
    end;

Const

{ Workbench Interlace (use one bit) }
    LACEWB              = $01;
    LW_RESERVED         = 1;     { internal use only }

{ PrinterPort }
    PARALLEL_PRINTER    = $00;
    SERIAL_PRINTER      = $01;

{ BaudRate }
    BAUD_110            = $00;
    BAUD_300            = $01;
    BAUD_1200           = $02;
    BAUD_2400           = $03;
    BAUD_4800           = $04;
    BAUD_9600           = $05;
    BAUD_19200          = $06;
    BAUD_MIDI           = $07;

{ PaperType }
    FANFOLD             = $00;
    SINGLE              = $80;

{ PrintPitch }
    PICA                = $000;
    ELITE               = $400;
    FINE                = $800;

{ PrintQuality }
    DRAFT               = $000;
    LETTER              = $100;

{ PrintSpacing }
    SIX_LPI             = $000;
    EIGHT_LPI           = $200;

{ Print Image }
    IMAGE_POSITIVE      = $00;
    IMAGE_NEGATIVE      = $01;

{ PrintAspect }
    ASPECT_HORIZ        = $00;
    ASPECT_VERT         = $01;

{ PrintShade }
    SHADE_BW            = $00;
    SHADE_GREYSCALE     = $01;
    SHADE_COLOR         = $02;

{ PaperSize }
    US_LETTER           = $00;
    US_LEGAL            = $10;
    N_TRACTOR           = $20;
    W_TRACTOR           = $30;
    CUSTOM_PAPER        = $40;

{ New PaperSizes for V36: }
 EURO_A0 = $50;            { European size A0: 841 x 1189 }
 EURO_A1 = $60;            { European size A1: 594 x 841 }
 EURO_A2 = $70;            { European size A2: 420 x 594 }
 EURO_A3 = $80;            { European size A3: 297 x 420 }
 EURO_A4 = $90;            { European size A4: 210 x 297 }
 EURO_A5 = $A0;            { European size A5: 148 x 210 }
 EURO_A6 = $B0;            { European size A6: 105 x 148 }
 EURO_A7 = $C0;            { European size A7: 74 x 105 }
 EURO_A8 = $D0;            { European size A8: 52 x 74 }

{ PrinterType }
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
{ new printer entries, 3 October 1985 }
    HP_LASERJET         = $0B;
    HP_LASERJET_PLUS    = $0C;

{ Serial Input Buffer Sizes }
    SBUF_512            = $00;
    SBUF_1024           = $01;
    SBUF_2048           = $02;
    SBUF_4096           = $03;
    SBUF_8000           = $04;
    SBUF_16000          = $05;

{ Serial Bit Masks }
    SREAD_BITS          = $F0;          { for SerRWBits   }
    SWRITE_BITS         = $0F;

    SSTOP_BITS          = $F0;          { for SerStopBuf  }
    SBUFSIZE_BITS       = $0F;

    SPARITY_BITS        = $F0;          { for SerParShk }
    SHSHAKE_BITS        = $0F;

{ Serial Parity (upper nibble, after being shifted by
 * macro SPARNUM() )
 }
    SPARITY_NONE        = 0;
    SPARITY_EVEN        = 1;
    SPARITY_ODD         = 2;

{ Serial Handshake Mode (lower nibble, after masking using
 * macro SHANKNUM() )
 }
    SHSHAKE_XON         = 0;
    SHSHAKE_RTS         = 1;
    SHSHAKE_NONE        = 2;

{ new defines for PrintFlags }

    CORRECT_RED         = $0001;        { color correct red shades }
    CORRECT_GREEN       = $0002;        { color correct green shades }
    CORRECT_BLUE        = $0004;        { color correct blue shades }

    CENTER_IMAGE        = $0008;        { center image on paper }

    IGNORE_DIMENSIONS   = $0000;        { ignore max width/height settings }
    BOUNDED_DIMENSIONS  = $0010;        { use max width/height as boundaries }
    ABSOLUTE_DIMENSIONS = $0020;        { use max width/height as absolutes }
    PIXEL_DIMENSIONS    = $0040;        { use max width/height as prt pixels }
    MULTIPLY_DIMENSIONS = $0080;        { use max width/height as multipliers }

    INTEGER_SCALING     = $0100;        { force integer scaling }

    ORDERED_DITHERING   = $0000;        { ordered dithering }
    HALFTONE_DITHERING  = $0200;        { halftone dithering }
    FLOYD_DITHERING     = $0400;        { Floyd-Steinberg dithering }

    ANTI_ALIAS          = $0800;        { anti-alias image }
    GREY_SCALE2         = $1000;        { for use with hi-res monitor }

{ masks used for checking bits }

    CORRECT_RGB_MASK    = CORRECT_RED + CORRECT_GREEN + CORRECT_BLUE;
    DIMENSIONS_MASK     = BOUNDED_DIMENSIONS + ABSOLUTE_DIMENSIONS +
                                PIXEL_DIMENSIONS + MULTIPLY_DIMENSIONS;
    DITHERING_MASK      = HALFTONE_DITHERING + FLOYD_DITHERING;






{ ======================================================================== }
{ === IntuitionBase ====================================================== }
{ ======================================================================== }
{
 * Be sure to protect yourself against someone modifying these data as
 * you look at them.  This is done by calling:
 *
 * lock = LockIBase(0), which returns an Integer.  When done call
 * UnlockIBase(lock) where lock is what LockIBase() returned.
 }

Type

    pIntuitionBase = ^tIntuitionBase;
    tIntuitionBase = record
{ IntuitionBase should never be directly modified by programs   }
{ even a little bit, guys/gals; do you hear me? }

        LibNode         : tLibrary;

        ViewLord        : tView;

        ActiveWindow    : pWindow;
        ActiveScreen    : pScreen;

    { the FirstScreen variable points to the frontmost Screen.   Screens are
     * then maintained in a front to back order using Screen.NextScreen
     }

        FirstScreen     : pScreen;    { for linked list of all screens }

        Flags           : ULONG;      { see definitions below }
        MouseY,
        MouseX          : smallint;        { mouse position relative to View }

        Seconds         : ULONG;      { timestamp of most current input event }
        Micros          : ULONG;      { timestamp of most current input event }

    { I told you this was private.
     * The data beyond this point has changed, is changing, and
     * will continue to change.
     }

    end;


{
 * Package of information passed to custom and 'boopsi'
 * gadget 'hook' functions.  This structure is READ ONLY.
 }
Type

   pGadgetInfo = ^tGadgetInfo;
   tGadgetInfo = record
    gi_Screen                   : pScreen;       { ScreenPtr }
    gi_Window                   : pWindow;       { null for screen gadgets }    { WindowPtr }
    gi_Requester                : pRequester;    { null IF not GTYP_REQGADGET } { RequesterPtr }

    { rendering information:
     * don't use these without cloning/locking.
     * Official way is to call ObtainRPort()
     }
    gi_RastPort                 : pRastPort;     { RastPortPtr }
    gi_Layer                    : pLayer;        { LayerPtr }

    { copy of dimensions of screen/window/g00/req(/group)
     * that gadget resides in.  Left/Top of this box is
     * offset from window mouse coordinates to gadget coordinates
     *          screen gadgets:                 0,0 (from screen coords)
     *  window gadgets (no g00):        0,0
     *  GTYP_GZZGADGETs (borderlayer):          0,0
     *  GZZ innerlayer gadget:          borderleft, bordertop
     *  Requester gadgets:              reqleft, reqtop
     }
    gi_Domain                   : tIBox;

    gi_Pens                     : record
                      DetailPen : Byte;
                      BlockPen  : Byte;
                                  end;

    { the Detail and Block pens in gi_DrInfo->dri_Pens[] are
     * for the screen.  Use the above for window-sensitive
     * colors.
     }
    gi_DrInfo                   : pDrawInfo;  { DrawInfoPtr }

    { reserved space: this structure is extensible
     * anyway, but using these saves some recompilation
     }
    gi_Reserved                 : Array[0..5] of ULONG;
   END;

{** system private data structure for now **}
{ prop gadget extra info       }

   pPGX = ^tPGX;
   tPGX = record
    pgx_Container : tIBox;
    pgx_NewKnob   : tIBox;
   END;

{ this casts MutualExclude for easy assignment of a hook
 * pointer to the unused MutualExclude field of a custom gadget
 }

{** User visible handles on objects, classes, messages **}
Type
 Object_ = ULONG;
 pObject_ = ^Object_;
 ClassID = ^Byte;

{
 you can use this type to point to a 'generic' message,
 * in the object-oriented programming parlance.  Based on
 * the value of 'MethodID', you dispatch to processing
 * for the various message types.  The meaningful parameter
 * packet structure definitions are defined below.

typedef struct
    ULONG MethodID;
     method-specific data follows, some examples below
               *Msg; }

  pMsg = ^tMsg;
  tMsg = record
     MethodID : ULONG;
  end;

{
 * Class id strings for Intuition classes.
 * There's no real reason to use the uppercase constants
 * over the lowercase strings, but this makes a good place
 * to list the names of the built-in classes.
 }
CONST
 ROOTCLASS      : PChar = 'rootclass'    ;         { classusr.h   }
 IMAGECLASS     : PChar = 'imageclass'   ;         { imageclass.h }
 FRAMEICLASS    : PChar = 'frameiclass'  ;
 SYSICLASS      : PChar = 'sysiclass'    ;
 FILLRECTCLASS  : PChar = 'fillrectclass';
 GADGETCLASS    : PChar = 'gadgetclass'  ;         { gadgetclass.h }
 PROPGCLASS     : PChar = 'propgclass'   ;
 STRGCLASS      : PChar = 'strgclass'    ;
 BUTTONGCLASS   : PChar = 'buttongclass' ;
 FRBUTTONCLASS  : PChar = 'frbuttonclass';
 GROUPGCLASS    : PChar = 'groupgclass'  ;
 ICCLASS        : PChar = 'icclass'      ;         { icclass.h    }
 MODELCLASS     : PChar = 'modelclass'   ;
 ITEXTICLASS    : PChar = 'itexticlass'  ;
 POINTERCLASS   : PChar = 'pointerclass' ;         { pointerclass.h }


{ Dispatched method ID's
 * NOTE: Applications should use Intuition entry points, not direct
 * DoMethod() calls, for NewObject, DisposeObject, SetAttrs,
 * SetGadgetAttrs, and GetAttr.
 }

 OM_Dummy       = ($100);
 OM_NEW         = ($101); { 'object' parameter is 'true class'   }
 OM_DISPOSE     = ($102); { delete self (no parameters)          }
 OM_SET         = ($103); { set attributes (in tag list)         }
 OM_GET         = ($104); { return single attribute value        }
 OM_ADDTAIL     = ($105); { add self to a List (let root do it)  }
 OM_REMOVE      = ($106); { remove self from list                }
 OM_NOTIFY      = ($107); { send to self: notify dependents      }
 OM_UPDATE      = ($108); { notification message from somebody   }
 OM_ADDMEMBER   = ($109); { used by various classes with lists   }
 OM_REMMEMBER   = ($10A); { used by various classes with lists   }

{ Parameter 'Messages' passed to methods       }

{ OM_NEW and OM_SET    }
Type
   popSet = ^topSet;
   topSet = record
    MethodID            : ULONG;
    ops_AttrList        : pTagItem;    { new attributes       }
    ops_GInfo           : pGadgetInfo; { always there for gadgets,
                                         * when SetGadgetAttrs() is used,
                                         * but will be NULL for OM_NEW
                                         }
   END;

{ OM_NOTIFY, and OM_UPDATE     }

  popUpdate = ^topUpdate;
  topUpdate = record
    MethodID            : ULONG;
    opu_AttrList        : pTagItem;     { new attributes       }
    opu_GInfo           : pGadgetInfo;  { non-NULL when SetGadgetAttrs OR
                                         * notification resulting from gadget
                                         * input occurs.
                                         }
    opu_Flags           : ULONG;        { defined below        }
  END;

{ this flag means that the update message is being issued from
 * something like an active gadget, a la GACT_FOLLOWMOUSE.  When
 * the gadget goes inactive, it will issue a final update
 * message with this bit cleared.  Examples of use are for
 * GACT_FOLLOWMOUSE equivalents for propgadclass, and repeat strobes
 * for buttons.
 }
CONST
 OPUF_INTERIM   = 1;

{ OM_GET       }
Type

  popGet = ^topGet;
  topGet = record
    MethodID,
    opg_AttrID          : ULONG;
    opg_Storage         : Pointer;   { may be other types, but 'int'
                                         * types are all ULONG
                                         }
  END;

{ OM_ADDTAIL   }

  popAddTail = ^topAddTail;
  topAddTail = record
    MethodID  : ULONG;
    opat_List : pList;
  END;

{ OM_ADDMEMBER, OM_REMMEMBER   }
Type

   popMember = ^topMember;
   topMember = record
    MethodID   : ULONG;
    opam_Object : pObject_;
   END;



{*****************************************}
{** 'White box' access to struct IClass **}
{*****************************************}

{ This structure is READ-ONLY, and allocated only by Intuition }
TYPE

   pIClass = ^tIClass;
   tIClass = record
    cl_Dispatcher       : tHook;
    cl_Reserved         : ULONG;    { must be 0  }
    cl_Super            : pIClass;
    cl_ID               : ClassID;

    { where within an object is the instance data for this class? }
    cl_InstOffset       : Word;
    cl_InstSize         : Word;

    cl_UserData         : ULONG;    { per-class data of your choice }
    cl_SubclassCount    : ULONG;
                                        { how many direct subclasses?  }
    cl_ObjectCount      : ULONG;
                                { how many objects created of this class? }
    cl_Flags            : ULONG;
   END;

CONST
 CLF_INLIST    =  $00000001;      { class is in public class list }



{************************************************}
{** 'White box' access to struct _Object       **}
{************************************************}

{
 * We have this, the instance data of the root class, PRECEDING
 * the 'object'.  This is so that Gadget objects are Gadget pointers,
 * and so on.  If this structure grows, it will always have o_Class
 * at the end, so the macro OCLASS(o) will always have the same
 * offset back from the pointer returned from NewObject().
 *
 * This data structure is subject to change.  Do not use the o_Node
 * embedded structure.
 }
Type
  p_Object = ^t_Object;
  t_Object = record
    o_Node    : tMinNode;
    o_Class   : pIClass;
  END;

{ BOOPSI class libraries should use this structure as the base for their
 * library data.  This allows developers to obtain the class pointer for
 * performing object-less inquiries. }


       PClassLibrary = ^tClassLibrary;
       tClassLibrary = record
            cl_Lib : tLibrary;    { Embedded library }
            cl_Pad : UWORD;       { Align the structure }
            cl_Class : PIClass;   { Class pointer }
         end;

{
 * NOTE:  <intuition/iobsolete.h> is included at the END of this file!
 }

{ Gadget Class attributes      }
CONST
    GA_Dummy           =  (TAG_USER +$30000);

    { (LONG) Left edge of the gadget relative to the left edge of
     * the window }
    GA_Left            =  (GA_Dummy + $0001);

    { (LONG) Left edge of the gadget relative to the right edge of
     * the window }
    GA_RelRight        =  (GA_Dummy + $0002);

    { (LONG) Top edge of the gadget relative to the top edge of
     * the window }
    GA_Top             =  (GA_Dummy + $0003);

    { (LONG) Top edge of the gadget relative to the bottom edge
     * of the window }
    GA_RelBottom       =  (GA_Dummy + $0004);

    { (LONG) Width of the gadget }
    GA_Width           =  (GA_Dummy + $0005);

    { (LONG) Width of the gadget relative to the width of the
     * window }
    GA_RelWidth        =  (GA_Dummy + $0006);

    { (LONG) Height of the gadget }
    GA_Height          =  (GA_Dummy + $0007);

    { (LONG) Height of the gadget relative to the height of
     * the window }
    GA_RelHeight       =  (GA_Dummy + $0008);

    { (STRPTR) Gadget imagry is NULL terminated string }
    GA_Text            =  (GA_Dummy + $0009); { ti_Data is (UBYTE *) }

    { (struct Image *) Gadget imagry is an image }
    GA_Image           =  (GA_Dummy + $000A);

    { (struct Border *) Gadget imagry is a border }
    GA_Border          =  (GA_Dummy + $000B);

    { (struct Image *) Selected gadget imagry }
    GA_SelectRender    =  (GA_Dummy + $000C);

    { (UWORD) One of GFLG_GADGHNONE, GFLG_GADGHBOX, GFLG_GADGHCOMP,
     * or GFLG_GADGHIMAGE }
    GA_Highlight       =  (GA_Dummy + $000D);

    { (BOOL) Indicate whether gadget is disabled or not.
     * Defaults to FALSE. }
    GA_Disabled        =  (GA_Dummy + $000E);

    { (BOOL) Indicate whether the gadget is for
     * WFLG_GIMMEZEROZERO window borders or not.  Defaults
     * to FALSE. }
    GA_GZZGadget       =  (GA_Dummy + $000F);

    { (UWORD) Gadget ID assigned by the application }
    GA_ID              =  (GA_Dummy + $0010);

    { (APTR) Application specific data }
    GA_UserData        =  (GA_Dummy + $0011);

    { (APTR) Gadget specific data }
    GA_SpecialInfo     =  (GA_Dummy + $0012);

    { (BOOL) Indicate whether the gadget is selected or not.
     * Defaults to FALSE }
    GA_Selected        =  (GA_Dummy + $0013);

    { (BOOL) When set tells the system that when this gadget
     * is selected causes the requester that it is in to be
     * ended.  Defaults to FALSE. }
    GA_EndGadget       =  (GA_Dummy + $0014);

    { (BOOL) When set indicates that the gadget is to
     * notify the application when it becomes active.  Defaults
     * to FALSE. }
    GA_Immediate       =  (GA_Dummy + $0015);

    { (BOOL) When set indicates that the application wants to
     * verify that the pointer was still over the gadget when
     * the select button is released.  Defaults to FALSE. }
    GA_RelVerify       =  (GA_Dummy + $0016);

    { (BOOL) When set indicates that the application wants to
     * be notified of mouse movements while the gadget is active.
     * It is recommmended that GA_Immediate and GA_RelVerify are
     * also used so that the active gadget can be tracked by the
     * application.  Defaults to FALSE. }
    GA_FollowMouse     =  (GA_Dummy + $0017);

    { (BOOL) Indicate whether the gadget is in the right border
     * or not.  Defaults to FALSE. }
    GA_RightBorder     =  (GA_Dummy + $0018);

    { (BOOL) Indicate whether the gadget is in the left border
     * or not.  Defaults to FALSE. }
    GA_LeftBorder      =  (GA_Dummy + $0019);

    { (BOOL) Indicate whether the gadget is in the top border
     * or not.  Defaults to FALSE. }
    GA_TopBorder       =  (GA_Dummy + $001A);

    { (BOOL) Indicate whether the gadget is in the bottom border
     * or not.  Defaults to FALSE. }
    GA_BottomBorder    =  (GA_Dummy + $001B);

    { (BOOL) Indicate whether the gadget is toggle-selected
     * or not.  Defaults to FALSE. }
    GA_ToggleSelect    =  (GA_Dummy + $001C);

    { (BOOL) Reserved for system use to indicate that the
     * gadget belongs to the system.  Defaults to FALSE. }
    GA_SysGadget       =  (GA_Dummy + $001D);

    { (UWORD) Reserved for system use to indicate the
     * gadget type. }
    GA_SysGType        =  (GA_Dummy + $001E);

    { (struct Gadget *) Previous gadget in the linked list.
     * NOTE: This attribute CANNOT be used to link new gadgets
     * into the gadget list of an open window or requester.
     * You must use AddGList(). }
    GA_Previous        =  (GA_Dummy + $001F);

    { (struct Gadget *) Next gadget in the linked list. }
    GA_Next            =  (GA_Dummy + $0020);

    { (struct DrawInfo *) Some gadgets need a DrawInfo at creation time }
    GA_DrawInfo        =  (GA_Dummy + $0021);

    { You should use at most ONE of GA_Text, GA_IntuiText, and GA_LabelImage }
    { (struct IntuiText *) Label is an IntuiText. }
    GA_IntuiText          =  (GA_Dummy + $0022);

    { (Object *) Label is an image object. }
    GA_LabelImage         =  (GA_Dummy + $0023);

    { New for V37:
         * Boolean indicates that this gadget is to participate in
         * cycling activation with Tab or Shift-Tab.
    }
    GA_TabCycle           =  (GA_Dummy + $0024);

    { New for V39:
         * Boolean indicates that this gadget sends gadget-help
    }
    GA_GadgetHelp         =  (GA_Dummy + $0025);

    { New for V39:
         * ti_Data is a pointer to an IBox structure which is
         * to be copied into the extended gadget's bounds.
    }
    GA_Bounds             =  (GA_Dummy + $0026);

    { New for V39:
         * Boolean indicates that this gadget has the "special relativity"
         * property, which is useful for certain fancy relativity
         * operations through the GM_LAYOUT method.
    }
    GA_RelSpecial         =  (GA_Dummy + $0027);


     GA_TextAttr = GA_Dummy + 40;
  { (struct TextAttr  ) Indicate the font to use for the gadget.
         New for V42.  }

     GA_ReadOnly = GA_Dummy + 41;
  { (BOOL) Indicate that the gadget is read-only (non-selectable).
         Defaults to FALSE. New for V42.  }

     GA_Underscore = GA_Dummy + 42;
  { (UBYTE) Underscore/escape character for keyboard shortcuts.
         Defaults to '_' . New for V44.  }

     GA_ActivateKey = GA_Dummy + 43;
  { (STRPTR) Set/Get the gadgets shortcut/activation key(s)
         Defaults to NULL. New for V44.  }

     GA_BackFill = GA_Dummy + 44;
  { (struct Hook  ) Backfill pattern hook.
         Defaults to NULL. New for V44.  }

     GA_GadgetHelpText = GA_Dummy + 45;
  { (STRPTR)   RESERVERD/PRIVATE DO NOT USE
         Defaults to NULL. New for V44.  }

     GA_UserInput = GA_Dummy + 46;
  { (BOOL) Notification tag indicates this notification is from the activite
           gadget receiving user input - an attempt to make IDCMPUPDATE more efficient.
         Defaults to FALSE. New for V44.  }
{ PROPGCLASS attributes }

 PGA_Dummy      = (TAG_USER + $31000);
 PGA_Freedom    = (PGA_Dummy + $0001);
        { only one of FREEVERT or FREEHORIZ }
 PGA_Borderless = (PGA_Dummy + $0002);
 PGA_HorizPot   = (PGA_Dummy + $0003);
 PGA_HorizBody  = (PGA_Dummy + $0004);
 PGA_VertPot    = (PGA_Dummy + $0005);
 PGA_VertBody   = (PGA_Dummy + $0006);
 PGA_Total      = (PGA_Dummy + $0007);
 PGA_Visible    = (PGA_Dummy + $0008);
 PGA_Top        = (PGA_Dummy + $0009);
{ New for V37: }
 PGA_NewLook    = (PGA_Dummy + $000A);

{ STRGCLASS attributes }

 STRINGA_Dummy         =  (TAG_USER      +$32000);
 STRINGA_MaxChars      =  (STRINGA_Dummy + $0001);
 STRINGA_Buffer        =  (STRINGA_Dummy + $0002);
 STRINGA_UndoBuffer    =  (STRINGA_Dummy + $0003);
 STRINGA_WorkBuffer    =  (STRINGA_Dummy + $0004);
 STRINGA_BufferPos     =  (STRINGA_Dummy + $0005);
 STRINGA_DispPos       =  (STRINGA_Dummy + $0006);
 STRINGA_AltKeyMap     =  (STRINGA_Dummy + $0007);
 STRINGA_Font          =  (STRINGA_Dummy + $0008);
 STRINGA_Pens          =  (STRINGA_Dummy + $0009);
 STRINGA_ActivePens    =  (STRINGA_Dummy + $000A);
 STRINGA_EditHook      =  (STRINGA_Dummy + $000B);
 STRINGA_EditModes     =  (STRINGA_Dummy + $000C);

{ booleans }
 STRINGA_ReplaceMode    = (STRINGA_Dummy + $000D);
 STRINGA_FixedFieldMode = (STRINGA_Dummy + $000E);
 STRINGA_NoFilterMode   = (STRINGA_Dummy + $000F);

 STRINGA_Justification  = (STRINGA_Dummy + $0010);
        { GACT_STRINGCENTER, GACT_STRINGLEFT, GACT_STRINGRIGHT }
 STRINGA_LongVal        = (STRINGA_Dummy + $0011);
 STRINGA_TextVal        = (STRINGA_Dummy + $0012);

 STRINGA_ExitHelp       = (STRINGA_Dummy + $0013);
        { STRINGA_ExitHelp is new for V37, and ignored by V36.
         * Set this if you want the gadget to exit when Help is
         * pressed.  Look for a code of $5F, the rawkey code for Help
         }

 SG_DEFAULTMAXCHARS     = (128);

{ Gadget Layout related attributes     }

 LAYOUTA_Dummy          = (TAG_USER  + $38000);
 LAYOUTA_LayoutObj      = (LAYOUTA_Dummy + $0001);
 LAYOUTA_Spacing        = (LAYOUTA_Dummy + $0002);
 LAYOUTA_Orientation    = (LAYOUTA_Dummy + $0003);

 LAYOUTA_ChildMaxWidth = LAYOUTA_Dummy + $0004;
  { (BOOL) Child objects are of equal width.  Should default to TRUE for
         gadgets with a horizontal orientation.  New for V42.  }

 LAYOUTA_ChildMaxHeight = LAYOUTA_Dummy + $0005;
  { (BOOL) Child objects are of equal height.  Should default to TRUE for
         gadgets with a vertical orientation.  New for V42.  }

{ orientation values   }
 LORIENT_NONE   = 0;
 LORIENT_HORIZ  = 1;
 LORIENT_VERT   = 2;


{ Gadget Method ID's   }

 GM_Dummy      =  (-1);    { not used for anything                }
 GM_HITTEST    =  (0);     { return GMR_GADGETHIT IF you are clicked on
                                 * (whether or not you are disabled).
                                 }
 GM_RENDER      = (1);     { draw yourself, in the appropriate state }
 GM_GOACTIVE    = (2);     { you are now going to be fed input    }
 GM_HANDLEINPUT = (3);     { handle that input                    }
 GM_GOINACTIVE  = (4);     { whether or not by choice, you are done  }
 GM_HELPTEST    = (5);     { Will you send gadget help if the mouse is
                                 * at the specified coordinates?  See below
                                 * for possible GMR_ values.
                                 }
 GM_LAYOUT      = (6);     { re-evaluate your size based on the GadgetInfo
                                 * Domain.  Do NOT re-render yourself yet, you
                                 * will be called when it is time...
                                 }

{ Parameter "Messages" passed to gadget class methods  }

{ GM_HITTEST   }
type

  pgpHitTest = ^tgpHitTest;
  tgpHitTest = record
    MethodID  : ULONG;
    gpht_GInfo : pGadgetInfo;
    gpht_Mouse : record
             x : smallint;
             y : smallint;
                 end;
  END;

const
{ For GM_HITTEST, return GMR_GADGETHIT if you were indeed hit,
 * otherwise return zero.
 *
 * For GM_HELPTEST, return GMR_NOHELPHIT (zero) if you were not hit.
 * Typically, return GMR_HELPHIT if you were hit.
 * It is possible to pass a UWORD to the application via the Code field
 * of the IDCMP_GADGETHELP message.  Return GMR_HELPCODE or'd with
 * the UWORD-sized result you wish to return.
 *
 * GMR_HELPHIT yields a Code value of ((UWORD) ~0), which should
 * mean "nothing particular" to the application.
 }

 GMR_GADGETHIT  = ($00000004);    { GM_HITTEST hit }

 GMR_NOHELPHIT  = ($00000000);    { GM_HELPTEST didn't hit }
 GMR_HELPHIT    = ($FFFFFFFF);    { GM_HELPTEST hit, return code = ~0 }
 GMR_HELPCODE   = ($00010000);    { GM_HELPTEST hit, return low word as code }


{ GM_RENDER    }
Type
   pgpRender = ^tgpRender;
   tgpRender = record
    MethodID  : ULONG;
    gpr_GInfo : pGadgetInfo;      { gadget context               }
    gpr_RPort : pRastPort;        { all ready for use            }
    gpr_Redraw : Longint;           { might be a "highlight pass"  }
   END;

{ values of gpr_Redraw }
CONST
 GREDRAW_UPDATE = (2);     { incremental update, e.g. prop slider }
 GREDRAW_REDRAW = (1);     { redraw gadget        }
 GREDRAW_TOGGLE = (0);     { toggle highlight, IF applicable      }

{ GM_GOACTIVE, GM_HANDLEINPUT  }
Type

  pgpInput = ^tgpInput;
  tgpInput = record
    MethodID : ULONG;
    gpi_GInfo : pGadgetInfo;
    gpi_IEvent : pInputEvent;
    gpi_Termination : Pointer;
    gpi_Mouse : record
            x : smallint;
            y : smallint;
                end;
    {* (V39) Pointer to TabletData structure, if this event originated
     * from a tablet which sends IESUBCLASS_NEWTABLET events, or NULL if
     * not.
     *
     * DO NOT ATTEMPT TO READ THIS FIELD UNDER INTUITION PRIOR TO V39!
     * IT WILL BE INVALID!
     *}
   gpi_TabletData  : pTabletData;
  END;

{ GM_HANDLEINPUT and GM_GOACTIVE  return code flags    }
{ return GMR_MEACTIVE (0) alone if you want more input.
 * Otherwise, return ONE of GMR_NOREUSE and GMR_REUSE, and optionally
 * GMR_VERIFY.
 }
CONST
 GMR_MEACTIVE  =  (0);
 GMR_NOREUSE   =  (2);
 GMR_REUSE     =  (4);
 GMR_VERIFY    =  (8);        { you MUST set cgp_Termination }

{ New for V37:
 * You can end activation with one of GMR_NEXTACTIVE and GMR_PREVACTIVE,
 * which instructs Intuition to activate the next or previous gadget
 * that has GFLG_TABCYCLE set.
 }
 GMR_NEXTACTIVE = (16);
 GMR_PREVACTIVE = (32);

{ GM_GOINACTIVE }
Type

   pgpGoInactive = ^tgpGoInactive;
   tgpGoInactive = record
    MethodID    : ULONG;
    gpgi_GInfo  : pGadgetInfo;

    { V37 field only!  DO NOT attempt to read under V36! }
    gpgi_Abort  : ULONG;               { gpgi_Abort=1 IF gadget was aborted
                                         * by Intuition and 0 if gadget went
                                         * inactive at its own request
                                         }
   END;

{* New for V39: Intuition sends GM_LAYOUT to any GREL_ gadget when
 * the gadget is added to the window (or when the window opens, if
 * the gadget was part of the NewWindow.FirstGadget or the WA_Gadgets
 * list), or when the window is resized.  Your gadget can set the
 * GA_RelSpecial property to get GM_LAYOUT events without Intuition
 * changing the interpretation of your gadget select box.  This
 * allows for completely arbitrary resizing/repositioning based on
 * window size.
 *}
{* GM_LAYOUT *}
Type

 pgpLayout = ^tgpLayout;
 tgpLayout = record
    MethodID            : ULONG;
    gpl_GInfo           : pGadgetInfo;
    gpl_Initial         : ULONG;      {* non-zero if this method was invoked
                                         * during AddGList() or OpenWindow()
                                         * time.  zero if this method was invoked
                                         * during window resizing.
                                         *}
 end;

{***************************************************************************}

{ The GM_DOMAIN method is used to obtain the sizing requirements of an
 * object for a class before ever creating an object. }

{ GM_DOMAIN }

    PgpDomain = ^tgpDomain;
     tgpDomain = record
          MethodID : ULONG;
          gpd_GInfo : PGadgetInfo;
          gpd_RPort : PRastPort;    { RastPort to layout for }
          gpd_Which : LONG;
          gpd_Domain : tIBox;       { Resulting domain }
          gpd_Attrs : PTagItem;     { Additional attributes }
       end;


  const
     GDOMAIN_MINIMUM = 0;
  { Minimum size  }

     GDOMAIN_NOMINAL = 1;
  { Nominal size  }

     GDOMAIN_MAXIMUM = 2;
  { Maximum size  }

{***************************************************************************}

{ The GM_KEYTEST method is used to determin if a key press matches an
 * object's activation key(s). }

{ GM_KEYTEST send this message.
 }

  type
     PgpKeyTest = ^tgpKeyTest;
     tgpKeyTest = record
          MethodID : ULONG;
          gpkt_GInfo : PGadgetInfo;
          gpkt_IMsg : PIntuiMessage;   { The IntuiMessage that triggered this }
          gpkt_VanillaKey : ULONG;
       end;

{***************************************************************************}

{ The GM_KEYGOACTIVE method is called to "simulate" a gadget going down.
 * A gadget should render itself in a selected state when receiving
 * this message. If the class supports this method, it must return
 * GMR_KEYACTIVE.
 *
 * If a gadget returns zero for this method, it will subsequently be
 * activated via ActivateGadget() with a NULL IEvent.
 }

     PgpKeyInput = ^tgpKeyInput;
     tgpKeyInput = record
          MethodID : ULONG;        { GM_KEYGOACTIVE }
          gpk_GInfo : PGadgetInfo;
          gpk_IEvent : PInputEvent;
          gpk_Termination : PLONG;
       end;


  const
     GMR_KEYACTIVE = 1 shl 4;

  { you MUST set gpk_Termination  }
     GMR_KEYVERIFY = 1 shl 5;

{ The GM_KEYGOINACTIVE method is called to simulate the gadget release.
 * Upon receiving this message, the gadget should do everything a
 * normal gadget release would do.
 }

  type
     PgpKeyGoInactive = ^tgpKeyGoInactive;
     tgpKeyGoInactive = record
          MethodID : ULONG;
          gpki_GInfo : PGadgetInfo;
          gpki_Abort : ULONG;
       end;

CONST
 ICM_Dummy      = ($0401);       { used for nothing             }
 ICM_SETLOOP    = ($0402);       { set/increment loop counter   }
 ICM_CLEARLOOP  = ($0403);       { clear/decrement loop counter }
 ICM_CHECKLOOP  = ($0404);       { set/increment loop           }

{ no parameters for ICM_SETLOOP, ICM_CLEARLOOP, ICM_CHECKLOOP  }

{ interconnection attributes used by icclass, modelclass, and gadgetclass }
 ICA_Dummy      = (TAG_USER+$40000);
 ICA_TARGET     = (ICA_Dummy + 1);
        { interconnection target               }
 ICA_MAP        = (ICA_Dummy + 2);
        { interconnection map tagitem list     }
 ICSPECIAL_CODE = (ICA_Dummy + 3);
        { a "pseudo-attribute", see below.     }

{ Normally, the value for ICA_TARGET is some object pointer,
 * but if you specify the special value ICTARGET_IDCMP, notification
 * will be send as an IDCMP_IDCMPUPDATE message to the appropriate window's
 * IDCMP port.  See the definition of IDCMP_IDCMPUPDATE.
 *
 * When you specify ICTARGET_IDCMP for ICA_TARGET, the map you
 * specify will be applied to derive the attribute list that is
 * sent with the IDCMP_IDCMPUPDATE message.  If you specify a map list
 * which results in the attribute tag id ICSPECIAL_CODE, the
 * lower sixteen bits of the corresponding ti_Data value will
 * be copied into the Code field of the IDCMP_IDCMPUPDATE IntuiMessage.
 }
 ICTARGET_IDCMP = (NOT 0);


CONST
 CUSTOMIMAGEDEPTH     =   (-1);
{ if image.Depth is this, it's a new Image class object }


{****************************************************}
CONST
 IA_Dummy             =   (TAG_USER + $20000);
 IA_Left              =   (IA_Dummy + $01);
 IA_Top               =   (IA_Dummy + $02);
 IA_Width             =   (IA_Dummy + $03);
 IA_Height            =   (IA_Dummy + $04);
 IA_FGPen             =   (IA_Dummy + $05);
                    { IA_FGPen also means "PlanePick"  }
 IA_BGPen             =   (IA_Dummy + $06);
                    { IA_BGPen also means "PlaneOnOff" }
 IA_Data              =   (IA_Dummy + $07);
                    { bitplanes, for classic image,
                     * other image classes may use it for other things
                     }
 IA_LineWidth         =   (IA_Dummy + $08);
 IA_Pens              =   (IA_Dummy + $0E);
                    { pointer to UWORD pens[],
                     * ala DrawInfo.Pens, MUST be
                     * terminated by ~0.  Some classes can
                     * choose to have this, or SYSIA_DrawInfo,
                     * or both.
                     }
 IA_Resolution        =   (IA_Dummy + $0F);
                    { packed uwords for x/y resolution into a longword
                     * ala DrawInfo.Resolution
                     }

{*** see class documentation to learn which    ****}
{*** classes recognize these                   ****}
 IA_APattern           =  (IA_Dummy + $10);
 IA_APatSize           =  (IA_Dummy + $11);
 IA_Mode               =  (IA_Dummy + $12);
 IA_Font               =  (IA_Dummy + $13);
 IA_Outline            =  (IA_Dummy + $14);
 IA_Recessed           =  (IA_Dummy + $15);
 IA_DoubleEmboss       =  (IA_Dummy + $16);
 IA_EdgesOnly          =  (IA_Dummy + $17);

{*** "sysiclass" attributes                    ****}
 SYSIA_Size            =  (IA_Dummy + $0B);
                    { 's below          }
 SYSIA_Depth           =  (IA_Dummy + $0C);
                    { this is unused by Intuition.  SYSIA_DrawInfo
                     * is used instead for V36
                     }
 SYSIA_Which           =  (IA_Dummy + $0D);
                    { see 's below      }
 SYSIA_DrawInfo        =  (IA_Dummy + $18);
                    { pass to sysiclass, please }

{****  obsolete: don't use these, use IA_Pens  ****}
 SYSIA_Pens            =  IA_Pens;
 IA_ShadowPen          =  (IA_Dummy + $09);
 IA_HighlightPen       =  (IA_Dummy + $0A);

{ New for V39: }
 SYSIA_ReferenceFont   =  (IA_Dummy + $19);
                    { Font to use as reference for scaling
                     * certain sysiclass images
                     }
 IA_SupportsDisable    =  (IA_Dummy + $1a);
                    { By default, Intuition ghosts gadgets itself,
                     * instead of relying on IDS_DISABLED or
                     * IDS_SELECTEDDISABLED.  An imageclass that
                     * supports these states should return this attribute
                     * as TRUE.  You cannot set or clear this attribute,
                     * however.
                     }

 IA_FrameType          =  (IA_Dummy + $1b);
                    { Starting with V39, FrameIClass recognizes
                     * several standard types of frame.  Use one
                     * of the FRAME_ specifiers below.  Defaults
                     * to FRAME_DEFAULT.
                     }

     IA_Underscore = IA_Dummy + $1c;
  { V44, Indicate underscore keyboard shortcut for image labels.
                       (UBYTE) Defaults to '_'
                      }

     IA_Scalable = IA_Dummy + $1d;
  { V44, Attribute indicates this image is allowed
                           to/can scale its rendering.
                       (BOOL) Defaults to FALSE.
                      }

     IA_ActivateKey = IA_Dummy + $1e;
  { V44, Used to get an underscored label shortcut.
                       Useful for labels attached to string gadgets.
                       (UBYTE) Defaults to NULL.
                      }

     IA_Screen = IA_Dummy + $1f;
  { V44 Screen pointer, may be useful/required by certain classes.
                       (struct Screen  )
                      }

     IA_Precision = IA_Dummy + $20;
  { V44 Precision value, typically pen precision but may be
                       used for similar custom purposes.
                       (ULONG)
                      }

{* next attribute: (IA_Dummy + $1c)   *}

{***********************************************}

{ data values for SYSIA_Size   }
 SYSISIZE_MEDRES = (0);
 SYSISIZE_LOWRES = (1);
 SYSISIZE_HIRES  = (2);

{
 * SYSIA_Which tag data values:
 * Specifies which system gadget you want an image for.
 * Some numbers correspond to internal Intuition s
 }
 DEPTHIMAGE     = ($00);
 ZOOMIMAGE      = ($01);
 SIZEIMAGE      = ($02);
 CLOSEIMAGE     = ($03);
 SDEPTHIMAGE    = ($05); { screen depth gadget }
 LEFTIMAGE      = ($0A);
 UPIMAGE        = ($0B);
 RIGHTIMAGE     = ($0C);
 DOWNIMAGE      = ($0D);
 CHECKIMAGE     = ($0E);
 MXIMAGE        = ($0F); { mutual exclude "button" }
{* New for V39: *}
 MENUCHECK      = ($10); { Menu checkmark image }
 AMIGAKEY       = ($11); { Menu Amiga-key image }

{ Data values for IA_FrameType (recognized by FrameIClass)
 *
 * FRAME_DEFAULT:  The standard V37-type frame, which has
 *      thin edges.
 * FRAME_BUTTON:  Standard button gadget frames, having thicker
 *      sides and nicely edged corners.
 * FRAME_RIDGE:  A ridge such as used by standard string gadgets.
 *      You can recess the ridge to get a groove image.
 * FRAME_ICONDROPBOX: A broad ridge which is the standard imagery
 *      for areas in AppWindows where icons may be dropped.
 }

     FRAME_DEFAULT = 0;
     FRAME_BUTTON = 1;
     FRAME_RIDGE = 2;
     FRAME_ICONDROPBOX = 3;

{ image message id's   }
    IM_DRAW     = $202;  { draw yourself, with "state"          }
    IM_HITTEST  = $203;  { return TRUE IF click hits image      }
    IM_ERASE    = $204;  { erase yourself                       }
    IM_MOVE     = $205;  { draw new AND erase old, smoothly     }

    IM_DRAWFRAME= $206;  { draw with specified dimensions       }
    IM_FRAMEBOX = $207;  { get recommended frame around some box}
    IM_HITFRAME = $208;  { hittest with dimensions              }
    IM_ERASEFRAME= $209; { hittest with dimensions              }
    IM_DOMAINFRAME = $20A;{ query image for its domain info (V44)  }

{ image draw states or styles, for IM_DRAW }
    IDS_NORMAL          = (0);
    IDS_SELECTED        = (1);    { for selected gadgets     }
    IDS_DISABLED        = (2);    { for disabled gadgets     }
    IDS_BUSY            = (3);    { for future functionality }
    IDS_INDETERMINATE   = (4);    { for future functionality }
    IDS_INACTIVENORMAL  = (5);    { normal, in inactive window border }
    IDS_INACTIVESELECTED= (6);    { selected, in inactive border }
    IDS_INACTIVEDISABLED= (7);    { disabled, in inactive border }
    IDS_SELECTEDDISABLED = 8;     { disabled and selected     }

{ oops, please forgive spelling error by jimm }
 IDS_INDETERMINANT = IDS_INDETERMINATE;

{ IM_FRAMEBOX  }
Type

  pimpFrameBox = ^timpFrameBox;
  timpFrameBox = record
    MethodID   : ULONG;
    imp_ContentsBox  : pIBox;       { input: relative box of contents }
    imp_FrameBox     : pIBox;          { output: rel. box of encl frame  }
    imp_DrInfo       : pDrawInfo;
    imp_FrameFlags   : ULONG;
  END;

CONST
 FRAMEF_SPECIFY = (1);  { Make do with the dimensions of FrameBox
                                 * provided.
                                 }

{ IM_DRAW, IM_DRAWFRAME        }
Type

   pimpDraw = ^timpDraw;
   timpDraw = record
    MethodID    : ULONG;
    imp_RPort   : pRastPort;
    imp_Offset  : record
              x : Word;
              y : Word;
                  end;
    imp_State   : ULONG;
    imp_DrInfo  : pDrawInfo;

    { these parameters only valid for IM_DRAWFRAME }
    imp_Dimensions : record
             Width : Word;
            Height : Word;
                     end;
   END;

{ IM_ERASE, IM_ERASEFRAME      }
{ NOTE: This is a subset of impDraw    }

   pimpErase = ^timpErase;
   timpErase = record
    MethodID       : ULONG;
    imp_RPort      : pRastPort;
    imp_Offset     : record
                 x : Word;
                 y : Word;
                     end;

    { these parameters only valid for IM_ERASEFRAME }
    imp_Dimensions : record
             Width : Word;
            Height : Word;
                     end;
   END;

{ IM_HITTEST, IM_HITFRAME      }

   pimpHitTest = ^timpHitTest;
   timpHitTest = record
    MethodID   : ULONG;
    imp_Point  : record
             x : Word;
             y : Word;
                 end;

    { these parameters only valid for IM_HITFRAME }
    imp_Dimensions : record
               Width : Word;
               Height : Word;
               end;
   END;


{ The IM_DOMAINFRAME method is used to obtain the sizing
 * requirements of an image object within a layout group.
 }

{ IM_DOMAINFRAME }
     PimpDomainFrame = ^timpDomainFrame;
     timpDomainFrame = record
          MethodID : ULONG;
          imp_DrInfo : PDrawInfo;
          imp_RPort : PRastPort;
          imp_Which : LONG;
          imp_Domain : tIBox;
          imp_Attrs : PTagItem;
       end;

  { Accepted vales for imp_Which.
    }

  const
     IDOMAIN_MINIMUM = 0;
     IDOMAIN_NOMINAL = 1;
     IDOMAIN_MAXIMUM = 2;

 { **  'boopsi' pointer class interface }

const
{ The following tags are recognized at NewObject() time by
 * pointerclass:
 *
 * POINTERA_BitMap (struct BitMap *) - Pointer to bitmap to
 *      get pointer imagery from.  Bitplane data need not be
 *      in chip RAM.
 * POINTERA_XOffset (LONG) - X-offset of the pointer hotspot.
 * POINTERA_YOffset (LONG) - Y-offset of the pointer hotspot.
 * POINTERA_WordWidth (ULONG) - designed width of the pointer in words
 * POINTERA_XResolution (ULONG) - one of the POINTERXRESN_ flags below
 * POINTERA_YResolution (ULONG) - one of the POINTERYRESN_ flags below
 *
 }

 POINTERA_Dummy = (TAG_USER + $39000);

 POINTERA_BitMap        = (POINTERA_Dummy + $01);
 POINTERA_XOffset       = (POINTERA_Dummy + $02);
 POINTERA_YOffset       = (POINTERA_Dummy + $03);
 POINTERA_WordWidth     = (POINTERA_Dummy + $04);
 POINTERA_XResolution   = (POINTERA_Dummy + $05);
 POINTERA_YResolution   = (POINTERA_Dummy + $06);

{ These are the choices for the POINTERA_XResolution attribute which
 * will determine what resolution pixels are used for this pointer.
 *
 * POINTERXRESN_DEFAULT (ECS-compatible pointer width)
 *      = 70 ns if SUPERHIRES-type mode, 140 ns if not
 *
 * POINTERXRESN_SCREENRES
 *      = Same as pixel speed of screen
 *
 * POINTERXRESN_LORES (pointer always in lores-like pixels)
 *      = 140 ns in 15kHz modes, 70 ns in 31kHz modes
 *
 * POINTERXRESN_HIRES (pointer always in hires-like pixels)
 *      = 70 ns in 15kHz modes, 35 ns in 31kHz modes
 *
 * POINTERXRESN_140NS (pointer always in 140 ns pixels)
 *      = 140 ns always
 *
 * POINTERXRESN_70NS (pointer always in 70 ns pixels)
 *      = 70 ns always
 *
 * POINTERXRESN_35NS (pointer always in 35 ns pixels)
 *      = 35 ns always
 }

 POINTERXRESN_DEFAULT   = 0;
 POINTERXRESN_140NS     = 1;
 POINTERXRESN_70NS      = 2;
 POINTERXRESN_35NS      = 3;

 POINTERXRESN_SCREENRES = 4;
 POINTERXRESN_LORES     = 5;
 POINTERXRESN_HIRES     = 6;

{ These are the choices for the POINTERA_YResolution attribute which
 * will determine what vertical resolution is used for this pointer.
 *
 * POINTERYRESN_DEFAULT
 *      = In 15 kHz modes, the pointer resolution will be the same
 *        as a non-interlaced screen.  In 31 kHz modes, the pointer
 *        will be doubled vertically.  This means there will be about
 *        200-256 pointer lines per screen.
 *
 * POINTERYRESN_HIGH
 * POINTERYRESN_HIGHASPECT
 *      = Where the hardware/software supports it, the pointer resolution
 *        will be high.  This means there will be about 400-480 pointer
 *        lines per screen.  POINTERYRESN_HIGHASPECT also means that
 *        when the pointer comes out double-height due to hardware/software
 *        restrictions, its width would be doubled as well, if possible
 *        (to preserve aspect).
 *
 * POINTERYRESN_SCREENRES
 * POINTERYRESN_SCREENRESASPECT
 *      = Will attempt to match the vertical resolution of the pointer
 *        to the screen's vertical resolution.  POINTERYRESN_SCREENASPECT also
 *        means that when the pointer comes out double-height due to
 *        hardware/software restrictions, its width would be doubled as well,
 *        if possible (to preserve aspect).
 *
 }

 POINTERYRESN_DEFAULT          =  0;
 POINTERYRESN_HIGH             =  2;
 POINTERYRESN_HIGHASPECT       =  3;
 POINTERYRESN_SCREENRES        =  4;
 POINTERYRESN_SCREENRESASPECT  =  5;

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


Type

   pStringExtend = ^tStringExtend;
   tStringExtend = record
    { display specifications   }
    Font        : pTextFont;      { must be an open Font (not TextAttr)  }
    Pens        : Array[0..1] of Byte;        { color of text/backgroun              }
    ActivePens  : Array[0..1] of Byte;  { colors when gadget is active         }

    { edit specifications      }
    InitialModes : ULONG;   { initial mode flags, below            }
    EditHook     : pHook;      { IF non-NULL, must supply WorkBuffer  }
    WorkBuffer   : STRPTR;    { must be as large as StringInfo.Buffer}

    Reserved     : Array[0..3] of ULONG;    { set to 0                             }
   END;

   pSGWork = ^tSGWork;
   tSGWork = record
    { set up when gadget is first activated    }
    Gad       : pGadget;         { the contestant itself        }   { Gadget in C-Includes }
    StrInfo   : pStringInfo;     { easy access to sinfo         }   { StrInfo in C-Includes }
    WorkBuffer : STRPTR;           { intuition's planned result   }
    PrevBuffer : STRPTR;           { what was there before        }
    Modes      : ULONG;          { current mode                 }

    { modified for each input event    }
    IEvent     : pInputEvent;    { actual event: do not change  }
    Code       : Word;            { character code, IF one byte  }
    BufferPos  : smallint;            { cursor position              }
    NumChars   : smallint;
    Actions    : ULONG;          { what Intuition will do       }
    LongInt_   : Longint;          { temp storage for longint     }

    GInfo      : pGadgetInfo;    { see cghooks.h                }   { GadgetInfo in C-Includes }
    EditOp     : Word;            { from constants below         }
   END;

{ SGWork.EditOp -
 * These values indicate what basic type of operation the global
 * editing hook has performed on the string before your gadget's custom
 * editing hook gets called.  You do not have to be concerned with the
 * value your custom hook leaves in the EditOp field, only if you
 * write a global editing hook.
 *
 * For most of these general edit operations, you'll want to compare
 * the BufferPos and NumChars of the StringInfo (before global editing)
 * and SGWork (after global editing).
 }

CONST
 EO_NOOP       =  ($0001);
        { did nothing                                                  }
 EO_DELBACKWARD=  ($0002);
        { deleted some chars (maybe 0).                                }
 EO_DELFORWARD =  ($0003);
        { deleted some characters under and in front of the cursor     }
 EO_MOVECURSOR =  ($0004);
        { moved the cursor                                             }
 EO_ENTER      =  ($0005);
        { "enter" or "return" key, terminate                           }
 EO_RESET      =  ($0006);
        { current Intuition-style undo                                 }
 EO_REPLACECHAR=  ($0007);
        { replaced one character and (maybe) advanced cursor           }
 EO_INSERTCHAR =  ($0008);
        { inserted one char into string or added one at end            }
 EO_BADFORMAT  =  ($0009);
        { didn't like the text data, e.g., Bad LONGINT                 }
 EO_BIGCHANGE  =  ($000A);        { unused by Intuition  }
        { complete or major change to the text, e.g. new string        }
 EO_UNDO       =  ($000B);        { unused by Intuition  }
        { some other style of undo                                     }
 EO_CLEAR      =  ($000C);
        { clear the string                                             }
 EO_SPECIAL    =  ($000D);        { unused by Intuition  }
        { some operation that doesn't fit into the categories here     }


{ Mode Flags definitions (ONLY first group allowed as InitialModes)    }
 SGM_REPLACE   =  (1);       { replace mode                 }
{ please initialize StringInfo with in-range value of BufferPos
 * if you are using SGM_REPLACE mode.
 }

 SGM_FIXEDFIELD = (2);       { fixed length buffer          }
                                        { always set SGM_REPLACE, too  }
 SGM_NOFILTER   = (4);       { don't filter control chars   }

{ SGM_EXITHELP is new for V37, and ignored by V36: }
 SGM_EXITHELP   = (128);       { exit with code = $5F IF HELP hit }


{ These Mode Flags are for internal use only                           }
 SGM_NOCHANGE   = (8);       { no edit changes yet          }
 SGM_NOWORKB    = (16);       { Buffer == PrevBuffer         }
 SGM_CONTROL    = (32);       { control char escape mode     }
 SGM_LONGINT    = (64);       { an intuition longint gadget  }

{ String Gadget Action Flags (put in SGWork.Actions by EditHook)       }
 SGA_USE        = ($1);  { use contents of SGWork               }
 SGA_END        = ($2);  { terminate gadget, code in Code field }
 SGA_BEEP       = ($4);  { flash the screen for the user        }
 SGA_REUSE      = ($8);  { reuse input event                    }
 SGA_REDISPLAY  = ($10); { gadget visuals changed               }

{ New for V37: }
 SGA_NEXTACTIVE = ($20); { Make next possible gadget active.    }
 SGA_PREVACTIVE = ($40); { Make previous possible gadget active.}

{ function id for only existing custom string gadget edit hook }

 SGH_KEY        = (1);    { process editing keystroke            }
 SGH_CLICK      = (2);    { process mouse click cursor position  }

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

FUNCTION ActivateGadget(gadgets : pGadget; window : pWindow; requester : pRequester) : BOOLEAN;
PROCEDURE ActivateWindow(window : pWindow);
PROCEDURE AddClass(classPtr : pIClass);
FUNCTION AddGadget(window : pWindow; gadget : pGadget; position : ULONG) : WORD;
FUNCTION AddGList(window : pWindow; gadget : pGadget; position : ULONG; numGad : LONGINT; requester : pRequester) : WORD;
FUNCTION AllocRemember(VAR rememberKey : pRemember; size : ULONG; flags : ULONG) : POINTER;
FUNCTION AllocScreenBuffer(sc : pScreen; bm : pBitMap; flags : ULONG) : pScreenBuffer;
FUNCTION AutoRequest(window : pWindow;const body : pIntuiText;const posText : pIntuiText;const negText : pIntuiText; pFlag : ULONG; nFlag : ULONG; width : ULONG; height : ULONG) : BOOLEAN;
PROCEDURE BeginRefresh(window : pWindow);
FUNCTION BuildEasyRequestArgs(window : pWindow;const easyStruct : pEasyStruct; idcmp : ULONG;const args : POINTER) : pWindow;
FUNCTION BuildSysRequest(window : pWindow;const body : pIntuiText;const posText : pIntuiText;const negText : pIntuiText; flags : ULONG; width : ULONG; height : ULONG) : pWindow;
FUNCTION ChangeScreenBuffer(sc : pScreen; sb : pScreenBuffer) : ULONG;
PROCEDURE ChangeWindowBox(window : pWindow; left : LONGINT; top : LONGINT; width : LONGINT; height : LONGINT);
FUNCTION ClearDMRequest(window : pWindow) : BOOLEAN;
PROCEDURE ClearMenuStrip(window : pWindow);
PROCEDURE ClearPointer(window : pWindow);
PROCEDURE CloseScreen(screen : pScreen);
PROCEDURE CloseWindow(window : pWindow);
FUNCTION CloseWorkBench : BOOLEAN;
PROCEDURE CurrentTime(VAR seconds : ULONG; VAR micros : ULONG);
FUNCTION DisplayAlert(alertNumber : ULONG;const string_ : pCHAR; height : ULONG) : BOOLEAN;
PROCEDURE DisplayBeep(screen : pScreen);
PROCEDURE DisposeObject(obj : POINTER);
FUNCTION DoGadgetMethodA(gad : pGadget; win : pWindow; req : pRequester; message : tMsg) : ULONG;
FUNCTION DoubleClick(sSeconds : ULONG; sMicros : ULONG; cSeconds : ULONG; cMicros : ULONG) : BOOLEAN;
PROCEDURE DrawBorder(rp : pRastPort;const border : pBorder; leftOffset : LONGINT; topOffset : LONGINT);
PROCEDURE DrawImage(rp : pRastPort; image : pImage; leftOffset : LONGINT; topOffset : LONGINT);
PROCEDURE DrawImageState(rp : pRastPort; image : pImage; leftOffset : LONGINT; topOffset : LONGINT; state : ULONG;const drawInfo : pDrawInfo);
FUNCTION EasyRequestArgs(window : pWindow;const easyStruct : pEasyStruct; idcmpPtr : pULONG;const args : POINTER) : LONGINT;
PROCEDURE EndRefresh(window : pWindow; complete : LONGINT);
PROCEDURE EndRequest(requester : pRequester; window : pWindow);
PROCEDURE EraseImage(rp : pRastPort; image : pImage; leftOffset : LONGINT; topOffset : LONGINT);
FUNCTION FreeClass(classPtr : pIClass) : BOOLEAN;
PROCEDURE FreeRemember(VAR rememberKey : pRemember; reallyForget : LONGINT);
PROCEDURE FreeScreenBuffer(sc : pScreen; sb : pScreenBuffer);
PROCEDURE FreeScreenDrawInfo(screen : pScreen; drawInfo : pDrawInfo);
PROCEDURE FreeSysRequest(window : pWindow);
PROCEDURE GadgetMouse(gadget : pGadget; gInfo : pGadgetInfo; mousePoint : psmallint);
FUNCTION GetAttr(attrID : ULONG; obj : POINTER; storagePtr : pULONG) : ULONG;
PROCEDURE GetDefaultPubScreen(nameBuffer : pCHAR);
FUNCTION GetDefPrefs(preferences : pPreferences; size : LONGINT) : pPreferences;
FUNCTION GetPrefs(preferences : pPreferences; size : LONGINT) : pPreferences;
FUNCTION GetScreenData(buffer : POINTER; size : ULONG; type_ : ULONG;const screen : pScreen) : BOOLEAN;
FUNCTION GetScreenDrawInfo(screen : pScreen) : pDrawInfo;
PROCEDURE HelpControl(win : pWindow; flags : ULONG);
PROCEDURE InitRequester(requester : pRequester);
FUNCTION IntuiTextLength(const iText : pIntuiText) : LONGINT;
FUNCTION ItemAddress(const menuStrip : pMenu; menuNumber : ULONG) : pMenuItem;
PROCEDURE LendMenus(fromwindow : pWindow; towindow : pWindow);
FUNCTION LockIBase(dontknow : ULONG) : ULONG;
FUNCTION LockPubScreen(const name : pCHAR) : pScreen;
FUNCTION LockPubScreenList : pList;
FUNCTION MakeClass(const classID : pCHAR;const superClassID : pCHAR;const superClassPtr : pIClass; instanceSize : ULONG; flags : ULONG) : pIClass;
FUNCTION MakeScreen(screen : pScreen) : LONGINT;
FUNCTION ModifyIDCMP(window : pWindow; flags : ULONG) : BOOLEAN;
PROCEDURE ModifyProp(gadget : pGadget; window : pWindow; requester : pRequester; flags : ULONG; horizPot : ULONG; vertPot : ULONG; horizBody : ULONG; vertBody : ULONG);
PROCEDURE MoveScreen(screen : pScreen; dx : LONGINT; dy : LONGINT);
PROCEDURE MoveWindow(window : pWindow; dx : LONGINT; dy : LONGINT);
PROCEDURE MoveWindowInFrontOf(window : pWindow; behindWindow : pWindow);
PROCEDURE NewModifyProp(gadget : pGadget; window : pWindow; requester : pRequester; flags : ULONG; horizPot : ULONG; vertPot : ULONG; horizBody : ULONG; vertBody : ULONG; numGad : LONGINT);
FUNCTION NewObjectA(classPtr : pIClass;const classID : pCHAR;const tagList : pTagItem) : POINTER;
FUNCTION NextObject(objectPtrPtr : POINTER) : POINTER;
FUNCTION NextPubScreen(const screen : pScreen; namebuf : pCHAR) : pCHAR;
FUNCTION ObtainGIRPort(gInfo : pGadgetInfo) : pRastPort;
PROCEDURE OffGadget(gadget : pGadget; window : pWindow; requester : pRequester);
PROCEDURE OffMenu(window : pWindow; menuNumber : ULONG);
PROCEDURE OnGadget(gadget : pGadget; window : pWindow; requester : pRequester);
PROCEDURE OnMenu(window : pWindow; menuNumber : ULONG);
FUNCTION OpenScreen(const newScreen : pNewScreen) : pScreen;
FUNCTION OpenScreenTagList(const newScreen : pNewScreen;const tagList : pTagItem) : pScreen;
FUNCTION OpenWindow(const newWindow : pNewWindow) : pWindow;
FUNCTION OpenWindowTagList(const newWindow : pNewWindow;const tagList : pTagItem) : pWindow;
FUNCTION OpenWorkBench : ULONG;
FUNCTION PointInImage(point : ULONG; image : pImage) : BOOLEAN;
PROCEDURE PrintIText(rp : pRastPort;const iText : pIntuiText; left : LONGINT; top : LONGINT);
FUNCTION PubScreenStatus(screen : pScreen; statusFlags : ULONG) : WORD;
FUNCTION QueryOverscan(displayID : ULONG; rect : pRectangle; oScanType : LONGINT) : LONGINT;
PROCEDURE RefreshGadgets(gadgets : pGadget; window : pWindow; requester : pRequester);
PROCEDURE RefreshGList(gadgets : pGadget; window : pWindow; requester : pRequester; numGad : LONGINT);
PROCEDURE RefreshWindowFrame(window : pWindow);
PROCEDURE ReleaseGIRPort(rp : pRastPort);
FUNCTION RemakeDisplay : LONGINT;
PROCEDURE RemoveClass(classPtr : pIClass);
FUNCTION RemoveGadget(window : pWindow; gadget : pGadget) : WORD;
FUNCTION RemoveGList(remPtr : pWindow; gadget : pGadget; numGad : LONGINT) : WORD;
PROCEDURE ReportMouse(flag : LONGINT; window : pWindow);
FUNCTION Request(requester : pRequester; window : pWindow) : BOOLEAN;
FUNCTION ResetMenuStrip(window : pWindow; menu : pMenu) : BOOLEAN;
FUNCTION RethinkDisplay : LONGINT;
PROCEDURE ScreenDepth(screen : pScreen; flags : ULONG; reserved : POINTER);
PROCEDURE ScreenPosition(screen : pScreen; flags : ULONG; x1 : LONGINT; y1 : LONGINT; x2 : LONGINT; y2 : LONGINT);
PROCEDURE ScreenToBack(screen : pScreen);
PROCEDURE ScreenToFront(screen : pScreen);
PROCEDURE ScrollWindowRaster(win : pWindow; dx : LONGINT; dy : LONGINT; xMin : LONGINT; yMin : LONGINT; xMax : LONGINT; yMax : LONGINT);
FUNCTION SetAttrsA(obj : POINTER;const tagList : pTagItem) : ULONG;
PROCEDURE SetDefaultPubScreen(const name : pCHAR);
FUNCTION SetDMRequest(window : pWindow; requester : pRequester) : BOOLEAN;
FUNCTION SetEditHook(hook : pHook) : pHook;
FUNCTION SetGadgetAttrsA(gadget : pGadget; window : pWindow; requester : pRequester;const tagList : pTagItem) : ULONG;
FUNCTION SetMenuStrip(window : pWindow; menu : pMenu) : BOOLEAN;
FUNCTION SetMouseQueue(window : pWindow; queueLength : ULONG) : LONGINT;
PROCEDURE SetPointer(window : pWindow; pointer_ : pword; height : LONGINT; width : LONGINT; xOffset : LONGINT; yOffset : LONGINT);
FUNCTION SetPrefs(const   preferences : pPreferences; size : LONGINT; inform : LONGINT) : pPreferences;
FUNCTION SetPubScreenModes(modes : ULONG) : WORD;
PROCEDURE SetWindowPointerA(win : pWindow;const taglist : pTagItem);
PROCEDURE SetWindowTitles(window : pWindow;const windowTitle : pCHAR;const screenTitle : pCHAR);
PROCEDURE ShowTitle(screen : pScreen; showIt : LONGINT);
PROCEDURE SizeWindow(window : pWindow; dx : LONGINT; dy : LONGINT);
FUNCTION SysReqHandler(window : pWindow; idcmpPtr : pULONG; waitInput : LONGINT) : LONGINT;
FUNCTION TimedDisplayAlert(alertNumber : ULONG;const string_ : pCHAR; height : ULONG; time : ULONG) : BOOLEAN;
PROCEDURE UnlockIBase(ibLock : ULONG);
PROCEDURE UnlockPubScreen(const name : pCHAR; screen : pScreen);
PROCEDURE UnlockPubScreenList;
FUNCTION ViewAddress : pView;
FUNCTION ViewPortAddress(const window : pWindow) : pViewPort;
FUNCTION WBenchToBack : BOOLEAN;
FUNCTION WBenchToFront : BOOLEAN;
FUNCTION WindowLimits(window : pWindow; widthMin : LONGINT; heightMin : LONGINT; widthMax : ULONG; heightMax : ULONG) : BOOLEAN;
PROCEDURE WindowToBack(window : pWindow);
PROCEDURE WindowToFront(window : pWindow);
PROCEDURE ZipWindow(window : pWindow);

{ Intuition macros }
function INST_DATA (cl: pIClass; o: p_Object): Pointer;
function SIZEOF_INSTANCE (cl: pIClass): Longint;
function BASEOBJECT (o: p_Object): Pointer;
function _OBJ(o: p_Object): p_Object;
function __OBJECT (o: Pointer): p_Object;
function OCLASS (o: Pointer): pIClass;
function SHIFTITEM (n: smallint): word;
function SHIFTMENU (n: smallint): word;
function SHIFTSUB (n: smallint): word;
function FULLMENUNUM (menu, item, sub: smallint): word;
function IM_BGPEN (im: pImage): byte;
function IM_BOX (im: pImage): pIBox;
function IM_FGPEN (im: pImage): byte;
function GADGET_BOX (g: pGadget): pIBox;
function CUSTOM_HOOK (gadget: pGadget): pHook;
function ITEMNUM( n : Word): Word;
function MENUNUM( n : Word): Word;
function SUBNUM( n : Word): Word;

FUNCTION DisplayAlert(alertNumber : ULONG;const string_ : string; height : ULONG) : BOOLEAN;
FUNCTION LockPubScreen(const name : string) : pScreen;
FUNCTION MakeClass(const classID : string;const superClassID : pCHAR;const superClassPtr : pIClass; instanceSize : ULONG; flags : ULONG) : pIClass;
FUNCTION MakeClass(const classID : pCHAR;const superClassID : string;const superClassPtr : pIClass; instanceSize : ULONG; flags : ULONG) : pIClass;
FUNCTION MakeClass(const classID : string;const superClassID : string;const superClassPtr : pIClass; instanceSize : ULONG; flags : ULONG) : pIClass;
FUNCTION NewObjectA(classPtr : pIClass;const classID : string;const tagList : pTagItem) : POINTER;
PROCEDURE SetDefaultPubScreen(const name : string);
PROCEDURE SetWindowTitles(window : pWindow;const windowTitle : string;const screenTitle : pCHAR);
PROCEDURE SetWindowTitles(window : pWindow;const windowTitle : pCHAR;const screenTitle : string);
PROCEDURE SetWindowTitles(window : pWindow;const windowTitle : string;const screenTitle : string);
FUNCTION TimedDisplayAlert(alertNumber : ULONG;const string_ : string; height : ULONG; time : ULONG) : BOOLEAN;
PROCEDURE UnlockPubScreen(const name : string; screen : pScreen);

IMPLEMENTATION

uses pastoc;

function INST_DATA (cl: pIClass; o: p_Object): Pointer;
begin
    INST_DATA := Pointer(Longint(o) + cl^.cl_InstOffset);
end;

function SIZEOF_INSTANCE (cl: pIClass): Longint;
begin
    SIZEOF_INSTANCE := cl^.cl_InstOffset + cl^.cl_InstSize + sizeof(t_Object);
end;

function BASEOBJECT (o: p_Object): Pointer;
begin
    BASEOBJECT := Pointer(Longint(o) + sizeof(t_Object));
end;

function _OBJ(o: p_Object): p_Object;
begin
     _OBJ := p_Object(o);
END;

function __OBJECT (o: Pointer): p_Object;
begin
    __OBJECT := p_Object(Longint(o) - sizeof(t_Object))
end;

function OCLASS (o: Pointer): pIClass;
var
    obj: p_Object;
begin
    obj := p_Object(Longint(o) - sizeof(t_Object));
    OCLASS := obj^.o_Class;
end;

function SHIFTITEM (n: smallint): word;
begin
    SHIFTITEM := (n and $3f) shl 5
end;

function SHIFTMENU (n: smallint): word;
begin
    SHIFTMENU := n and $1f
end;

function SHIFTSUB (n: smallint): word;
begin
    SHIFTSUB := (n and $1f) shl 11
end;

function FULLMENUNUM (menu, item, sub: smallint): word;
begin
    FULLMENUNUM := ((sub and $1f) shl 11) or
                    ((item and $3f) shl 5) or
                      (menu and $1f)
end;



{ The next functons _BGPEN AND _FGPEN aren't a full replacement of the
  C macros because the C preprocessor makes it possible to set the
  A/BPen values of the image class objects as well. This can't work
  in pascal, of course! }

function IM_BGPEN (im: pImage): byte;
begin
    IM_BGPEN := im^.PlaneOnOff;
end;

function IM_BOX (im: pImage): pIBox;
begin
    IM_BOX := pIBox(@im^.LeftEdge);
END;

function IM_FGPEN (im: pImage): byte;
begin
    IM_FGPEN := im^.PlanePick;
end;

function GADGET_BOX (g: pGadget): pIBox;
begin
    GADGET_BOX := pIBox(@g^.LeftEdge);
end;

function CUSTOM_HOOK (gadget: pGadget): pHook;
begin
    CUSTOM_HOOK := pHook(gadget^.MutualExclude);
end;

function ITEMNUM( n : Word): Word;
begin
    ITEMNUM := (n shr 5) and $3F
end;

function MENUNUM( n : Word): Word;
begin
    MENUNUM := n and $1f
end;

function SUBNUM( n : Word): Word;
begin
    SUBNUM := (n shr 11) and $1f
end;

FUNCTION ActivateGadget(gadgets : pGadget; window : pWindow; requester : pRequester) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gadgets,A0
    MOVEA.L window,A1
    MOVEA.L requester,A2
    MOVEA.L _IntuitionBase,A6
    JSR -462(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE ActivateWindow(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -450(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AddClass(classPtr : pIClass);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L classPtr,A0
    MOVEA.L _IntuitionBase,A6
    JSR -684(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AddGadget(window : pWindow; gadget : pGadget; position : ULONG) : WORD;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L gadget,A1
    MOVE.L  position,D0
    MOVEA.L _IntuitionBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AddGList(window : pWindow; gadget : pGadget; position : ULONG; numGad : LONGINT; requester : pRequester) : WORD;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L gadget,A1
    MOVE.L  position,D0
    MOVE.L  numGad,D1
    MOVEA.L requester,A2
    MOVEA.L _IntuitionBase,A6
    JSR -438(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocRemember(VAR rememberKey : pRemember; size : ULONG; flags : ULONG) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rememberKey,A0
    MOVE.L  size,D0
    MOVE.L  flags,D1
    MOVEA.L _IntuitionBase,A6
    JSR -396(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocScreenBuffer(sc : pScreen; bm : pBitMap; flags : ULONG) : pScreenBuffer;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sc,A0
    MOVEA.L bm,A1
    MOVE.L  flags,D0
    MOVEA.L _IntuitionBase,A6
    JSR -768(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AutoRequest(window : pWindow;const body : pIntuiText;const posText : pIntuiText;const negText : pIntuiText; pFlag : ULONG; nFlag : ULONG; width : ULONG; height : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L body,A1
    MOVEA.L posText,A2
    MOVEA.L negText,A3
    MOVE.L  pFlag,D0
    MOVE.L  nFlag,D1
    MOVE.L  width,D2
    MOVE.L  height,D3
    MOVEA.L _IntuitionBase,A6
    JSR -348(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE BeginRefresh(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -354(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION BuildEasyRequestArgs(window : pWindow;const easyStruct : pEasyStruct; idcmp : ULONG;const args : POINTER) : pWindow;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L easyStruct,A1
    MOVE.L  idcmp,D0
    MOVEA.L args,A3
    MOVEA.L _IntuitionBase,A6
    JSR -594(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION BuildSysRequest(window : pWindow;const body : pIntuiText;const posText : pIntuiText;const negText : pIntuiText; flags : ULONG; width : ULONG; height : ULONG) : pWindow;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L body,A1
    MOVEA.L posText,A2
    MOVEA.L negText,A3
    MOVE.L  flags,D0
    MOVE.L  width,D1
    MOVE.L  height,D2
    MOVEA.L _IntuitionBase,A6
    JSR -360(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ChangeScreenBuffer(sc : pScreen; sb : pScreenBuffer) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sc,A0
    MOVEA.L sb,A1
    MOVEA.L _IntuitionBase,A6
    JSR -780(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ChangeWindowBox(window : pWindow; left : LONGINT; top : LONGINT; width : LONGINT; height : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVE.L  left,D0
    MOVE.L  top,D1
    MOVE.L  width,D2
    MOVE.L  height,D3
    MOVEA.L _IntuitionBase,A6
    JSR -486(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION ClearDMRequest(window : pWindow) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE ClearMenuStrip(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ClearPointer(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE CloseScreen(screen : pScreen);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L _IntuitionBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE CloseWindow(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION CloseWorkBench : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _IntuitionBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE CurrentTime(VAR seconds : ULONG; VAR micros : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L seconds,A0
    MOVEA.L micros,A1
    MOVEA.L _IntuitionBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION DisplayAlert(alertNumber : ULONG;const string_ : pCHAR; height : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  alertNumber,D0
    MOVEA.L string_,A0
    MOVE.L  height,D1
    MOVEA.L _IntuitionBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE DisplayBeep(screen : pScreen);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L _IntuitionBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DisposeObject(obj : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L obj,A0
    MOVEA.L _IntuitionBase,A6
    JSR -642(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION DoGadgetMethodA(gad : pGadget; win : pWindow; req : pRequester; message : tMsg) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gad,A0
    MOVEA.L win,A1
    MOVEA.L req,A2
    MOVEA.L message,A3
    MOVEA.L _IntuitionBase,A6
    JSR -810(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DoubleClick(sSeconds : ULONG; sMicros : ULONG; cSeconds : ULONG; cMicros : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  sSeconds,D0
    MOVE.L  sMicros,D1
    MOVE.L  cSeconds,D2
    MOVE.L  cMicros,D3
    MOVEA.L _IntuitionBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE DrawBorder(rp : pRastPort;const border : pBorder; leftOffset : LONGINT; topOffset : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rp,A0
    MOVEA.L border,A1
    MOVE.L  leftOffset,D0
    MOVE.L  topOffset,D1
    MOVEA.L _IntuitionBase,A6
    JSR -108(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DrawImage(rp : pRastPort; image : pImage; leftOffset : LONGINT; topOffset : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rp,A0
    MOVEA.L image,A1
    MOVE.L  leftOffset,D0
    MOVE.L  topOffset,D1
    MOVEA.L _IntuitionBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DrawImageState(rp : pRastPort; image : pImage; leftOffset : LONGINT; topOffset : LONGINT; state : ULONG;const drawInfo : pDrawInfo);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rp,A0
    MOVEA.L image,A1
    MOVE.L  leftOffset,D0
    MOVE.L  topOffset,D1
    MOVE.L  state,D2
    MOVEA.L drawInfo,A2
    MOVEA.L _IntuitionBase,A6
    JSR -618(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION EasyRequestArgs(window : pWindow;const easyStruct : pEasyStruct; idcmpPtr : pULONG;const args : POINTER) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L easyStruct,A1
    MOVEA.L idcmpPtr,A2
    MOVEA.L args,A3
    MOVEA.L _IntuitionBase,A6
    JSR -588(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE EndRefresh(window : pWindow; complete : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVE.L  complete,D0
    MOVEA.L _IntuitionBase,A6
    JSR -366(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE EndRequest(requester : pRequester; window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L requester,A0
    MOVEA.L window,A1
    MOVEA.L _IntuitionBase,A6
    JSR -120(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE EraseImage(rp : pRastPort; image : pImage; leftOffset : LONGINT; topOffset : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rp,A0
    MOVEA.L image,A1
    MOVE.L  leftOffset,D0
    MOVE.L  topOffset,D1
    MOVEA.L _IntuitionBase,A6
    JSR -630(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION FreeClass(classPtr : pIClass) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L classPtr,A0
    MOVEA.L _IntuitionBase,A6
    JSR -714(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE FreeRemember(VAR rememberKey : pRemember; reallyForget : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rememberKey,A0
    MOVE.L  reallyForget,D0
    MOVEA.L _IntuitionBase,A6
    JSR -408(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeScreenBuffer(sc : pScreen; sb : pScreenBuffer);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sc,A0
    MOVEA.L sb,A1
    MOVEA.L _IntuitionBase,A6
    JSR -774(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeScreenDrawInfo(screen : pScreen; drawInfo : pDrawInfo);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L drawInfo,A1
    MOVEA.L _IntuitionBase,A6
    JSR -696(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeSysRequest(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -372(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE GadgetMouse(gadget : pGadget; gInfo : pGadgetInfo; mousePoint : psmallint);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gadget,A0
    MOVEA.L gInfo,A1
    MOVEA.L mousePoint,A2
    MOVEA.L _IntuitionBase,A6
    JSR -570(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION GetAttr(attrID : ULONG; obj : POINTER; storagePtr : pULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  attrID,D0
    MOVEA.L obj,A0
    MOVEA.L storagePtr,A1
    MOVEA.L _IntuitionBase,A6
    JSR -654(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE GetDefaultPubScreen(nameBuffer : pCHAR);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L nameBuffer,A0
    MOVEA.L _IntuitionBase,A6
    JSR -582(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION GetDefPrefs(preferences : pPreferences; size : LONGINT) : pPreferences;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L preferences,A0
    MOVE.L  size,D0
    MOVEA.L _IntuitionBase,A6
    JSR -126(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetPrefs(preferences : pPreferences; size : LONGINT) : pPreferences;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L preferences,A0
    MOVE.L  size,D0
    MOVEA.L _IntuitionBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetScreenData(buffer : POINTER; size : ULONG; type_ : ULONG;const screen : pScreen) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L buffer,A0
    MOVE.L  size,D0
    MOVE.L  type_,D1
    MOVEA.L screen,A1
    MOVEA.L _IntuitionBase,A6
    JSR -426(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION GetScreenDrawInfo(screen : pScreen) : pDrawInfo;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L _IntuitionBase,A6
    JSR -690(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE HelpControl(win : pWindow; flags : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L win,A0
    MOVE.L  flags,D0
    MOVEA.L _IntuitionBase,A6
    JSR -828(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE InitRequester(requester : pRequester);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L requester,A0
    MOVEA.L _IntuitionBase,A6
    JSR -138(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION IntuiTextLength(const iText : pIntuiText) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L iText,A0
    MOVEA.L _IntuitionBase,A6
    JSR -330(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ItemAddress(const menuStrip : pMenu; menuNumber : ULONG) : pMenuItem;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L menuStrip,A0
    MOVE.L  menuNumber,D0
    MOVEA.L _IntuitionBase,A6
    JSR -144(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE LendMenus(fromwindow : pWindow; towindow : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L fromwindow,A0
    MOVEA.L towindow,A1
    MOVEA.L _IntuitionBase,A6
    JSR -804(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION LockIBase(dontknow : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  dontknow,D0
    MOVEA.L _IntuitionBase,A6
    JSR -414(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LockPubScreen(const name : pCHAR) : pScreen;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L name,A0
    MOVEA.L _IntuitionBase,A6
    JSR -510(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LockPubScreenList : pList;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _IntuitionBase,A6
    JSR -522(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MakeClass(const  classID : pCHAR;const superClassID : pCHAR;const superClassPtr : pIClass; instanceSize : ULONG; flags : ULONG) : pIClass;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L classID,A0
    MOVEA.L superClassID,A1
    MOVEA.L superClassPtr,A2
    MOVE.L  instanceSize,D0
    MOVE.L  flags,D1
    MOVEA.L _IntuitionBase,A6
    JSR -678(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MakeScreen(screen : pScreen): LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L _IntuitionBase,A6
    JSR -378(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ModifyIDCMP(window : pWindow; flags : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVE.L  flags,D0
    MOVEA.L _IntuitionBase,A6
    JSR -150(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE ModifyProp(gadget : pGadget; window : pWindow; requester : pRequester; flags : ULONG; horizPot : ULONG; vertPot : ULONG; horizBody : ULONG; vertBody : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gadget,A0
    MOVEA.L window,A1
    MOVEA.L requester,A2
    MOVE.L  flags,D0
    MOVE.L  horizPot,D1
    MOVE.L  vertPot,D2
    MOVE.L  horizBody,D3
    MOVE.L  vertBody,D4
    MOVEA.L _IntuitionBase,A6
    JSR -156(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MoveScreen(screen : pScreen; dx : LONGINT; dy : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVE.L  dx,D0
    MOVE.L  dy,D1
    MOVEA.L _IntuitionBase,A6
    JSR -162(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MoveWindow(window : pWindow; dx : LONGINT; dy : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVE.L  dx,D0
    MOVE.L  dy,D1
    MOVEA.L _IntuitionBase,A6
    JSR -168(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MoveWindowInFrontOf(window : pWindow; behindWindow : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L behindWindow,A1
    MOVEA.L _IntuitionBase,A6
    JSR -480(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE NewModifyProp(gadget : pGadget; window : pWindow; requester : pRequester; flags : ULONG; horizPot : ULONG; vertPot : ULONG; horizBody : ULONG; vertBody : ULONG; numGad : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gadget,A0
    MOVEA.L window,A1
    MOVEA.L requester,A2
    MOVE.L  flags,D0
    MOVE.L  horizPot,D1
    MOVE.L  vertPot,D2
    MOVE.L  horizBody,D3
    MOVE.L  vertBody,D4
    MOVE.L  numGad,D5
    MOVEA.L _IntuitionBase,A6
    JSR -468(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION NewObjectA(classPtr : pIClass;const classID : pCHAR;const tagList : pTagItem) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L classPtr,A0
    MOVEA.L classID,A1
    MOVEA.L tagList,A2
    MOVEA.L _IntuitionBase,A6
    JSR -636(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION NextObject(objectPtrPtr : POINTER) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L objectPtrPtr,A0
    MOVEA.L _IntuitionBase,A6
    JSR -666(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION NextPubScreen(const screen : pScreen; namebuf : pCHAR) : pCHAR;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L namebuf,A1
    MOVEA.L _IntuitionBase,A6
    JSR -534(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ObtainGIRPort(gInfo : pGadgetInfo) : pRastPort;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gInfo,A0
    MOVEA.L _IntuitionBase,A6
    JSR -558(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE OffGadget(gadget : pGadget; window : pWindow; requester : pRequester);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gadget,A0
    MOVEA.L window,A1
    MOVEA.L requester,A2
    MOVEA.L _IntuitionBase,A6
    JSR -174(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE OffMenu(window : pWindow; menuNumber : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVE.L  menuNumber,D0
    MOVEA.L _IntuitionBase,A6
    JSR -180(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE OnGadget(gadget : pGadget; window : pWindow; requester : pRequester);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gadget,A0
    MOVEA.L window,A1
    MOVEA.L requester,A2
    MOVEA.L _IntuitionBase,A6
    JSR -186(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE OnMenu(window : pWindow; menuNumber : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVE.L  menuNumber,D0
    MOVEA.L _IntuitionBase,A6
    JSR -192(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION OpenScreen(const newScreen : pNewScreen) : pScreen;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L newScreen,A0
    MOVEA.L _IntuitionBase,A6
    JSR -198(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION OpenScreenTagList(const newScreen : pNewScreen;const tagList : pTagItem) : pScreen;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L newScreen,A0
    MOVEA.L tagList,A1
    MOVEA.L _IntuitionBase,A6
    JSR -612(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION OpenWindow(const newWindow : pNewWindow) : pWindow;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L newWindow,A0
    MOVEA.L _IntuitionBase,A6
    JSR -204(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION OpenWindowTagList(const newWindow : pNewWindow;const tagList : pTagItem) : pWindow;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L newWindow,A0
    MOVEA.L tagList,A1
    MOVEA.L _IntuitionBase,A6
    JSR -606(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION OpenWorkBench : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _IntuitionBase,A6
    JSR -210(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION PointInImage(point : ULONG; image : pImage) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  point,D0
    MOVEA.L image,A0
    MOVEA.L _IntuitionBase,A6
    JSR -624(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE PrintIText(rp : pRastPort;const iText : pIntuiText; left : LONGINT; top : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rp,A0
    MOVEA.L iText,A1
    MOVE.L  left,D0
    MOVE.L  top,D1
    MOVEA.L _IntuitionBase,A6
    JSR -216(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION PubScreenStatus(screen : pScreen; statusFlags : ULONG) : WORD;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVE.L  statusFlags,D0
    MOVEA.L _IntuitionBase,A6
    JSR -552(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QueryOverscan(displayID : ULONG; rect : pRectangle; oScanType : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L displayID,A0
    MOVEA.L rect,A1
    MOVE.L  oScanType,D0
    MOVEA.L _IntuitionBase,A6
    JSR -474(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE RefreshGadgets(gadgets : pGadget; window : pWindow; requester : pRequester);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gadgets,A0
    MOVEA.L window,A1
    MOVEA.L requester,A2
    MOVEA.L _IntuitionBase,A6
    JSR -222(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RefreshGList(gadgets : pGadget; window : pWindow; requester : pRequester; numGad : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gadgets,A0
    MOVEA.L window,A1
    MOVEA.L requester,A2
    MOVE.L  numGad,D0
    MOVEA.L _IntuitionBase,A6
    JSR -432(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RefreshWindowFrame(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -456(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ReleaseGIRPort(rp : pRastPort);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rp,A0
    MOVEA.L _IntuitionBase,A6
    JSR -564(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION RemakeDisplay : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _IntuitionBase,A6
    JSR -384(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE RemoveClass(classPtr : pIClass);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L classPtr,A0
    MOVEA.L _IntuitionBase,A6
    JSR -708(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION RemoveGadget(window : pWindow; gadget : pGadget) : WORD;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L gadget,A1
    MOVEA.L _IntuitionBase,A6
    JSR -228(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION RemoveGList(remPtr : pWindow; gadget : pGadget; numGad : LONGINT) : WORD;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L remPtr,A0
    MOVEA.L gadget,A1
    MOVE.L  numGad,D0
    MOVEA.L _IntuitionBase,A6
    JSR -444(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ReportMouse(flag : LONGINT; window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  flag,D0
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -234(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION Request(requester : pRequester; window : pWindow) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L requester,A0
    MOVEA.L window,A1
    MOVEA.L _IntuitionBase,A6
    JSR -240(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION ResetMenuStrip(window : pWindow; menu : pMenu) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L menu,A1
    MOVEA.L _IntuitionBase,A6
    JSR -702(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION RethinkDisplay : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _IntuitionBase,A6
    JSR -390(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ScreenDepth(screen : pScreen; flags : ULONG; reserved : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVE.L  flags,D0
    MOVEA.L reserved,A1
    MOVEA.L _IntuitionBase,A6
    JSR -786(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ScreenPosition(screen : pScreen; flags : ULONG; x1 : LONGINT; y1 : LONGINT; x2 : LONGINT; y2 : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVE.L  flags,D0
    MOVE.L  x1,D1
    MOVE.L  y1,D2
    MOVE.L  x2,D3
    MOVE.L  y2,D4
    MOVEA.L _IntuitionBase,A6
    JSR -792(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ScreenToBack(screen : pScreen);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L _IntuitionBase,A6
    JSR -246(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ScreenToFront(screen : pScreen);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L _IntuitionBase,A6
    JSR -252(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ScrollWindowRaster(win : pWindow; dx : LONGINT; dy : LONGINT; xMin : LONGINT; yMin : LONGINT; xMax : LONGINT; yMax : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L win,A1
    MOVE.L  dx,D0
    MOVE.L  dy,D1
    MOVE.L  xMin,D2
    MOVE.L  yMin,D3
    MOVE.L  xMax,D4
    MOVE.L  yMax,D5
    MOVEA.L _IntuitionBase,A6
    JSR -798(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION SetAttrsA(obj : POINTER;const tagList : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L obj,A0
    MOVEA.L tagList,A1
    MOVEA.L _IntuitionBase,A6
    JSR -648(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE SetDefaultPubScreen(const name : pCHAR);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L name,A0
    MOVEA.L _IntuitionBase,A6
    JSR -540(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION SetDMRequest(window : pWindow; requester : pRequester) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L requester,A1
    MOVEA.L _IntuitionBase,A6
    JSR -258(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetEditHook(hook : pHook) : pHook;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L hook,A0
    MOVEA.L _IntuitionBase,A6
    JSR -492(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetGadgetAttrsA(gadget : pGadget; window : pWindow; requester : pRequester;const tagList : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gadget,A0
    MOVEA.L window,A1
    MOVEA.L requester,A2
    MOVEA.L tagList,A3
    MOVEA.L _IntuitionBase,A6
    JSR -660(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetMenuStrip(window : pWindow; menu : pMenu) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L menu,A1
    MOVEA.L _IntuitionBase,A6
    JSR -264(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetMouseQueue(window : pWindow; queueLength : ULONG) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVE.L  queueLength,D0
    MOVEA.L _IntuitionBase,A6
    JSR -498(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE SetPointer(window : pWindow; pointer_ : pword; height : LONGINT; width : LONGINT; xOffset : LONGINT; yOffset : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L pointer_,A1
    MOVE.L  height,D0
    MOVE.L  width,D1
    MOVE.L  xOffset,D2
    MOVE.L  yOffset,D3
    MOVEA.L _IntuitionBase,A6
    JSR -270(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION SetPrefs(const preferences : pPreferences; size : LONGINT; inform : LONGINT) : pPreferences;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L preferences,A0
    MOVE.L  size,D0
    MOVE.L  inform,D1
    MOVEA.L _IntuitionBase,A6
    JSR -324(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetPubScreenModes(modes : ULONG) : WORD;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  modes,D0
    MOVEA.L _IntuitionBase,A6
    JSR -546(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE SetWindowPointerA(win : pWindow;const taglist : pTagItem);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L win,A0
    MOVEA.L taglist,A1
    MOVEA.L _IntuitionBase,A6
    JSR -816(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE SetWindowTitles(window : pWindow;const windowTitle : pCHAR;const screenTitle : pCHAR);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L windowTitle,A1
    MOVEA.L screenTitle,A2
    MOVEA.L _IntuitionBase,A6
    JSR -276(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ShowTitle(screen : pScreen; showIt : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVE.L  showIt,D0
    MOVEA.L _IntuitionBase,A6
    JSR -282(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE SizeWindow(window : pWindow; dx : LONGINT; dy : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVE.L  dx,D0
    MOVE.L  dy,D1
    MOVEA.L _IntuitionBase,A6
    JSR -288(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION SysReqHandler(window : pWindow; idcmpPtr : pULONG; waitInput : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L idcmpPtr,A1
    MOVE.L  waitInput,D0
    MOVEA.L _IntuitionBase,A6
    JSR -600(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TimedDisplayAlert(alertNumber : ULONG;const string_ : pCHAR; height : ULONG; time : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  alertNumber,D0
    MOVEA.L string_,A0
    MOVE.L  height,D1
    MOVEA.L time,A1
    MOVEA.L _IntuitionBase,A6
    JSR -822(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE UnlockIBase(ibLock : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L ibLock,A0
    MOVEA.L _IntuitionBase,A6
    JSR -420(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE UnlockPubScreen(const name : pCHAR; screen : pScreen);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L name,A0
    MOVEA.L screen,A1
    MOVEA.L _IntuitionBase,A6
    JSR -516(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE UnlockPubScreenList;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _IntuitionBase,A6
    JSR -528(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION ViewAddress : pView;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _IntuitionBase,A6
    JSR -294(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ViewPortAddress(const window : pWindow) : pViewPort;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -300(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION WBenchToBack : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _IntuitionBase,A6
    JSR -336(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION WBenchToFront : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _IntuitionBase,A6
    JSR -342(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION WindowLimits(window : pWindow; widthMin : LONGINT; heightMin : LONGINT; widthMax : ULONG; heightMax : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVE.L  widthMin,D0
    MOVE.L  heightMin,D1
    MOVE.L  widthMax,D2
    MOVE.L  heightMax,D3
    MOVEA.L _IntuitionBase,A6
    JSR -318(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE WindowToBack(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -306(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE WindowToFront(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -312(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ZipWindow(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L _IntuitionBase,A6
    JSR -504(A6)
    MOVEA.L (A7)+,A6
  END;
END;


FUNCTION DisplayAlert(alertNumber : ULONG;const string_ : string; height : ULONG) : BOOLEAN;
begin
      DisplayAlert := DisplayAlert(alertNumber,pas2c(string_),height);
end;

FUNCTION LockPubScreen(const name : string) : pScreen;
begin
      LockPubScreen := LockPubScreen(pas2c(name));
end;

FUNCTION MakeClass(const classID : string;const superClassID : pCHAR;const superClassPtr : pIClass; instanceSize : ULONG; flags : ULONG) : pIClass;
begin
      MakeClass := MakeClass(pas2c(classID),superClassID,superClassPtr,instanceSize,flags);
end;

FUNCTION MakeClass(const classID : pCHAR;const superClassID : string;const superClassPtr : pIClass; instanceSize : ULONG; flags : ULONG) : pIClass;
begin
      MakeClass := MakeClass(classID,pas2c(superClassID),superClassPtr,instanceSize,flags);
end;

FUNCTION MakeClass(const classID : string;const superClassID : string;const superClassPtr : pIClass; instanceSize : ULONG; flags : ULONG) : pIClass;
begin
      MakeClass := MakeClass(pas2c(classID),pas2c(superClassID),superClassPtr,instanceSize,flags);
end;

FUNCTION NewObjectA(classPtr : pIClass;const classID : string;const tagList : pTagItem) : POINTER;
begin
      NewObjectA := NewObjectA(classPtr,pas2c(classID),taglist);
end;

PROCEDURE SetDefaultPubScreen(const name : string);
begin
      SetDefaultPubScreen(pas2c(name));
end;

PROCEDURE SetWindowTitles(window : pWindow;const windowTitle : string;const screenTitle : pCHAR);
begin
      SetWindowTitles(window,pas2c(windowTitle),screenTitle);
end;

PROCEDURE SetWindowTitles(window : pWindow;const windowTitle : pCHAR;const screenTitle : string);
begin
      SetWindowTitles(window,windowTitle,pas2c(screenTitle));
end;

PROCEDURE SetWindowTitles(window : pWindow;const windowTitle : string;const screenTitle : string);
begin
      SetWindowTitles(window,pas2c(windowTitle),pas2c(screenTitle));
end;

FUNCTION TimedDisplayAlert(alertNumber : ULONG;const string_ : string; height : ULONG; time : ULONG) : BOOLEAN;
begin
      TimedDisplayAlert := TimedDisplayAlert(alertNumber,pas2c(string_),height,time);
end;

PROCEDURE UnlockPubScreen(const name : string; screen : pScreen);
begin
      UnlockPubScreen(pas2c(name),screen);
end;


END. (* UNIT INTUITION *)


















