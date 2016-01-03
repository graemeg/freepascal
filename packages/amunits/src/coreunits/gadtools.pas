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

    Added autoopening of gadtools.library.
    15 Jul 2000.

    Added MessageBox for error report.
    31 Jul 2000.

    Added the macros GTMENUITEM_USERDATA and GTMENU_USERDATA.
    19 Aug 2000.

    Added functions and procedures with array of const.
    For use with fpc 1.0. They are in systemvartags.
    11 Nov 2002.

    Added the defines use_amiga_smartlink and
    use_auto_openlib.
    13 Jan 2003.

    Update for AmigaOS 3.9.
    Changed startup code for the unit.
    01 Feb 2003.

    Changed integer > smallint,
            cardinal > longword.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se

}


{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit gadtools;

INTERFACE

uses exec, intuition, graphics, utility;


{------------------------------------------------------------------------}

{  The kinds (almost classes) of gadgets in the toolkit.  Use these
    identifiers when calling CreateGadgetA() }

CONST
 GENERIC_KIND  =  0;
 BUTTON_KIND   =  1;
 CHECKBOX_KIND =  2;
 INTEGER_KIND  =  3;
 LISTVIEW_KIND =  4;
 MX_KIND       =  5;
 NUMBER_KIND   =  6;
 CYCLE_KIND    =  7;
 PALETTE_KIND  =  8;
 SCROLLER_KIND =  9;
{ Kind number 10 is reserved }
 SLIDER_KIND   =  11;
 STRING_KIND   =  12;
 TEXT_KIND     =  13;

 NUM_KINDS     =  14;

 GADTOOLSNAME   : PChar = 'gadtools.library';


{------------------------------------------------------------------------}

{  'Or' the appropriate set together for your Window IDCMPFlags: }

 ARROWIDCMP    =  (IDCMP_GADGETUP + IDCMP_GADGETDOWN +
                   IDCMP_INTUITICKS + IDCMP_MOUSEBUTTONS);

 BUTTONIDCMP   =  (IDCMP_GADGETUP);
 CHECKBOXIDCMP =  (IDCMP_GADGETUP);
 INTEGERIDCMP  =  (IDCMP_GADGETUP);
 LISTVIEWIDCMP =  (IDCMP_GADGETUP + IDCMP_GADGETDOWN +
                   IDCMP_MOUSEMOVE + ARROWIDCMP);

 MXIDCMP       =  (IDCMP_GADGETDOWN);
 NUMBERIDCMP   =  0;
 CYCLEIDCMP    =  (IDCMP_GADGETUP);
 PALETTEIDCMP  =  (IDCMP_GADGETUP);

{  Use ARROWIDCMP+SCROLLERIDCMP if your scrollers have arrows: }
 SCROLLERIDCMP =  (IDCMP_GADGETUP + IDCMP_GADGETDOWN + IDCMP_MOUSEMOVE);
 SLIDERIDCMP   =  (IDCMP_GADGETUP + IDCMP_GADGETDOWN + IDCMP_MOUSEMOVE);
 STRINGIDCMP   =  (IDCMP_GADGETUP);

 TEXTIDCMP     =  0;


{------------------------------------------------------------------------}

{  Generic NewGadget used by several of the gadget classes: }

Type
   pNewGadget = ^tNewGadget;
   tNewGadget = record
    ng_LeftEdge, ng_TopEdge : smallint;     {  gadget position }
    ng_Width, ng_Height     : smallint;     {  gadget size }
    ng_GadgetText           : STRPTR;      {  gadget label }
    ng_TextAttr             : pTextAttr;   {  desired font for gadget label }
    ng_GadgetID             : Word;        {  gadget ID }
    ng_Flags                : ULONG;       {  see below }
    ng_VisualInfo           : Pointer;     {  Set to retval of GetVisualInfo() }
    ng_UserData             : Pointer;     {  gadget UserData }
   END;


{  ng_Flags control certain aspects of the gadget.  The first five control
    the placement of the descriptive text.  All larger groups supply a
    default: }

CONST
 PLACETEXT_LEFT  = $0001;  { Right-align text on left side }
 PLACETEXT_RIGHT = $0002;  { Left-align text on right side }
 PLACETEXT_ABOVE = $0004;  { Center text above }
 PLACETEXT_BELOW = $0008;  { Center text below }
 PLACETEXT_IN    = $0010;  { Center text on }

 NG_HIGHLABEL    = $0020;  { Highlight the label }

{------------------------------------------------------------------------}

{ Fill out an array of these and pass that to CreateMenus(): }

Type
   pNewMenu = ^tNewMenu;
   tNewMenu = record
    nm_Type           : Byte;              {  See below }
    nm_Label          : STRPTR;            {  Menu's label }
    nm_CommKey        : STRPTR;            {  MenuItem Command Key Equiv }
    nm_Flags          : Word;              {  Menu OR MenuItem flags (see note) }
    nm_MutualExclude  : Longint;           {  MenuItem MutualExclude word }
    nm_UserData       : Pointer;           {  For your own use, see note }
   END;

const
{ Needed only by inside IM_ definitions below }
 MENU_IMAGE     = 128;

{ nm_Type determines what each NewMenu structure corresponds to.
 * for the NM_TITLE, NM_ITEM, and NM_SUB values, nm_Label should
 * be a text string to use for that menu title, item, or sub-item.
 * For IM_ITEM or IM_SUB, set nm_Label to point at the Image structure
 * you wish to use for this item or sub-item.
 * NOTE: At present, you may only use conventional images.
 * Custom images created from Intuition image-classes do not work.
 }
 NM_TITLE      =  1;       { Menu header }
 NM_ITEM       =  2;       { Textual menu item }
 NM_SUB        =  3;       { Textual menu sub-item }

 IM_ITEM       =  (NM_ITEM OR MENU_IMAGE);    { Graphical menu item }
 IM_SUB        =  (NM_SUB OR MENU_IMAGE);     { Graphical menu sub-item }

{ The NewMenu array should be terminated with a NewMenu whose
 * nm_Type equals NM_END.
 }
 NM_END        =  0;       { End of NewMenu array }

{ Starting with V39, GadTools will skip any NewMenu entries whose
 * nm_Type field has the NM_IGNORE bit set.
 }
 NM_IGNORE     =  64;


{ nm_Label should be a text string for textual items, a pointer
 * to an Image structure for graphical menu items, or the special
 * constant NM_BARLABEL, to get a separator bar.
 }
 NM_BARLABEL   = -1;

{ The nm_Flags field is used to fill out either the Menu->Flags or
 * MenuItem->Flags field.  Note that the sense of the MENUENABLED or
 * ITEMENABLED bit is inverted between this use and Intuition's use,
 * in other words, NewMenus are enabled by default.  The following
 * labels are provided to disable them:
 }
 NM_MENUDISABLED = MENUENABLED;
 NM_ITEMDISABLED = ITEMENABLED;

{ New for V39:  NM_COMMANDSTRING.  For a textual MenuItem or SubItem,
 * point nm_CommKey at an arbitrary string, and set the NM_COMMANDSTRING
 * flag.
 }
 NM_COMMANDSTRING = COMMSEQ;

{ The following are pre-cleared (COMMSEQ, ITEMTEXT, and HIGHxxx are set
 * later as appropriate):
 * Under V39, the COMMSEQ flag bit is not cleared, since it now has
 * meaning.
 }
 NM_FLAGMASK    = NOT (COMMSEQ OR ITEMTEXT OR HIGHFLAGS);
 NM_FLAGMASK_V39 = NOT (ITEMTEXT OR HIGHFLAGS);

{ You may choose among CHECKIT, MENUTOGGLE, and CHECKED.
 * Toggle-select menuitems are of type CHECKIT|MENUTOGGLE, along
 * with CHECKED if currently selected.  Mutually exclusive ones
 * are of type CHECKIT, and possibly CHECKED too.  The nm_MutualExclude
 * is a bit-wise representation of the items excluded by this one,
 * so in the simplest case (choose 1 among n), these flags would be
 * ~1, ~2, ~4, ~8, ~16, etc.  See the Intuition Menus chapter.
 }

{ A UserData pointer can be associated with each Menu and MenuItem structure.
 * The CreateMenus() call allocates space for a UserData after each
 * Menu or MenuItem (header, item or sub-item).  You should use the
 * GTMENU_USERDATA() or GTMENUITEM_USERDATA() macro to extract it.
 }

const
{ These return codes can be obtained through the GTMN_SecondaryError tag }
 GTMENU_TRIMMED = $00000001;      { Too many menus, items, or subitems,
                                         * menu has been trimmed down
                                         }
 GTMENU_INVALID = $00000002;      { Invalid NewMenu array }
 GTMENU_NOMEM   = $00000003;      { Out of memory }

{------------------------------------------------------------------------}

{ Starting with V39, checkboxes and mx gadgets can be scaled to your
 * specified gadget width/height.  Use the new GTCB_Scaled or GTMX_Scaled
 * tags, respectively.  Under V37, and by default in V39, the imagery
 * is of the following fixed size:
 }

{ MX gadget default dimensions: }
 MX_WIDTH      =  17;
 MX_HEIGHT     =  9;

{ Checkbox default dimensions: }
 CHECKBOX_WIDTH  = 26;
 CHECKBOX_HEIGHT = 11;

{------------------------------------------------------------------------}


{------------------------------------------------------------------------}

{  Tags for GadTools functions: }
CONST
 GT_TagBase        =   TAG_USER + $80000;

 GTVI_NewWindow    =   GT_TagBase+1;  { Unused }
 GTVI_NWTags       =   GT_TagBase+2;  { Unused }

 GT_Private0       =   GT_TagBase+3;  { (private) }

 GTCB_Checked      =   GT_TagBase+4;  { State of checkbox }

 GTLV_Top          =   GT_TagBase+5;  { Top visible one in listview }
 GTLV_Labels       =   GT_TagBase+6;  { List to display in listview }
 GTLV_ReadOnly     =   GT_TagBase+7;  { TRUE IF listview is to be
                                              read-only }
 GTLV_ScrollWidth  =   GT_TagBase+8;  { Width of scrollbar }

 GTMX_Labels       =   GT_TagBase+9;  { NULL-terminated array of labels }
 GTMX_Active       =   GT_TagBase+10; { Active one in mx gadget }

 GTTX_Text         =   GT_TagBase+11; { Text to display }
 GTTX_CopyText     =   GT_TagBase+12; { Copy text label instead of
                                              referencing it }

 GTNM_Number       =   GT_TagBase+13; { Number to display }

 GTCY_Labels       =   GT_TagBase+14; { NULL-terminated array of labels }
 GTCY_Active       =   GT_TagBase+15; { The active one in the cycle gad }

 GTPA_Depth        =   GT_TagBase+16; { Number of bitplanes in palette }
 GTPA_Color        =   GT_TagBase+17; { Palette color }
 GTPA_ColorOffset  =   GT_TagBase+18; { First color to use in palette }
 GTPA_IndicatorWidth = GT_TagBase+19; { Width of current-color indicator }
 GTPA_IndicatorHeight = GT_TagBase+20; { Height of current-color indicator }

 GTSC_Top          =   GT_TagBase+21; { Top visible in scroller }
 GTSC_Total        =   GT_TagBase+22; { Total in scroller area }
 GTSC_Visible      =   GT_TagBase+23; { Number visible in scroller }
 GTSC_Overlap      =   GT_TagBase+24; { Unused }

{  GT_TagBase+25 through GT_TagBase+37 are reserved }

 GTSL_Min          =   GT_TagBase+38; { Slider min value }
 GTSL_Max          =   GT_TagBase+39; { Slider max value }
 GTSL_Level        =   GT_TagBase+40; { Slider level }
 GTSL_MaxLevelLen  =   GT_TagBase+41; { Max length of printed level }
 GTSL_LevelFormat  =   GT_TagBase+42; { Format string for level }
 GTSL_LevelPlace   =   GT_TagBase+43; { Where level should be placed }
 GTSL_DispFunc     =   GT_TagBase+44; { Callback for number calculation
                                              before display }

 GTST_String       =   GT_TagBase+45; { String gadget's displayed string }
 GTST_MaxChars     =   GT_TagBase+46; { Max length of string }

 GTIN_Number       =   GT_TagBase+47; { Number in integer gadget }
 GTIN_MaxChars     =   GT_TagBase+48; { Max number of digits }

 GTMN_TextAttr     =   GT_TagBase+49; { MenuItem font TextAttr }
 GTMN_FrontPen     =   GT_TagBase+50; { MenuItem text pen color }

 GTBB_Recessed     =   GT_TagBase+51; { Make BevelBox recessed }

 GT_VisualInfo     =   GT_TagBase+52; { result of VisualInfo call }

 GTLV_ShowSelected =   GT_TagBase+53; { show selected entry beneath
                listview, set tag data = NULL for display-only, or pointer
                to a string gadget you've created }
 GTLV_Selected     =   GT_TagBase+54; { Set ordinal number of selected
                                              entry in the list }
 GT_Reserved0      =   GT_TagBase+55; { Reserved }
 GT_Reserved1      =   GT_TagBase+56; { Reserved for future use }

 GTTX_Border       =   GT_TagBase+57; { Put a border around
                                              Text-display gadgets }
 GTNM_Border       =   GT_TagBase+58; { Put a border around
                                              Number-display gadgets }

 GTSC_Arrows       =   GT_TagBase+59; { Specify size of arrows for
                                              scroller }

 GTMN_Menu         =   GT_TagBase+60; { Pointer to Menu for use by
                                              LayoutMenuItems() }
 GTMX_Spacing      =   GT_TagBase+61; { Added to font height to
                figure spacing between mx choices.  Use this instead
                of LAYOUTA_SPACING for mx gadgets. }

{ New to V37 GadTools.  Ignored by GadTools V36 }
 GTMN_FullMenu     =   GT_TagBase+62; { Asks CreateMenus() to
                validate that this is a complete menu structure }
 GTMN_SecondaryError =  GT_TagBase+63; { ti_Data is a pointer
                to a ULONG to receive error reports from CreateMenus() }
 GT_Underscore     =   GT_TagBase+64; { ti_Data points to the symbol
                that preceeds the character you'd like to underline in a
                gadget label }

{ New to V39 GadTools.  Ignored by GadTools V36 and V37 }
 GTMN_Checkmark     =  GT_TagBase+65; { ti_Data is checkmark img to use }
 GTMN_AmigaKey      =  GT_TagBase+66; { ti_Data is Amiga-key img to use }
 GTMN_NewLookMenus  =  GT_TagBase+67; { ti_Data is boolean }

{ New to V39 GadTools.  Ignored by GadTools V36 and V37.
 * Set to TRUE if you want the checkbox or mx image scaled to
 * the gadget width/height you specify.  Defaults to FALSE,
 * for compatibility.
 }
 GTCB_Scaled         = GT_TagBase+68; { ti_Data is boolean }
 GTMX_Scaled         = GT_TagBase+69; { ti_Data is boolean }

 GTPA_NumColors      = GT_TagBase+70; { Number of colors in palette }

 GTMX_TitlePlace     = GT_TagBase+71; { Where to put the title }

 GTTX_FrontPen       = GT_TagBase+72; { Text color in TEXT_KIND gad }
 GTTX_BackPen        = GT_TagBase+73; { Bgrnd color in TEXT_KIND gad }
 GTTX_Justification  = GT_TagBase+74; { See GTJ_#? constants }

 GTNM_FrontPen       = GT_TagBase+72; { Text color in NUMBER_KIND gad }
 GTNM_BackPen        = GT_TagBase+73; { Bgrnd color in NUMBER_KIND gad }
 GTNM_Justification  = GT_TagBase+74; { See GTJ_#? constants }
 GTNM_Format         = GT_TagBase+75; { Formatting string for number }
 GTNM_MaxNumberLen   = GT_TagBase+76; { Maximum length of number }

 GTBB_FrameType      = GT_TagBase+77; { defines what kind of boxes
                                            * DrawBevelBox() renders. See
                                            * the BBFT_#? constants for
                                            * possible values
                                            }

 GTLV_MakeVisible    = GT_TagBase+78; { Make this item visible }
 GTLV_ItemHeight     = GT_TagBase+79; { Height of an individual item }

 GTSL_MaxPixelLen    = GT_TagBase+80; { Max pixel size of level display }
 GTSL_Justification  = GT_TagBase+81; { how should the level be displayed }

 GTPA_ColorTable     = GT_TagBase+82; { colors to use in palette }

 GTLV_CallBack       = GT_TagBase+83; { general-purpose listview call back }
 GTLV_MaxPen         = GT_TagBase+84; { maximum pen number used by call back }

 GTTX_Clipped        = GT_TagBase+85; { make a TEXT_KIND clip text }
 GTNM_Clipped        = GT_TagBase+85; { make a NUMBER_KIND clip text }


{------------------------------------------------------------------------}

{ Justification types for GTTX_Justification and GTNM_Justification tags }
 GTJ_LEFT   = 0;
 GTJ_RIGHT  = 1;
 GTJ_CENTER = 2;

{------------------------------------------------------------------------}

{ Bevel box frame types for GTBB_FrameType tag }
 BBFT_BUTTON      = 1;  { Standard button gadget box }
 BBFT_RIDGE       = 2;  { Standard string gadget box }
 BBFT_ICONDROPBOX = 3;  { Standard icon drop box     }

{------------------------------------------------------------------------}

{ Typical suggested spacing between "elements": }
 INTERWIDTH    =  8;
 INTERHEIGHT   =  4;

{------------------------------------------------------------------------}


{  "NWay" is an old synonym for cycle gadgets }
 NWAY_KIND    =   CYCLE_KIND;
 NWAYIDCMP    =   CYCLEIDCMP;
 GTNW_Labels  =   GTCY_Labels;
 GTNW_Active  =   GTCY_Active;

{------------------------------------------------------------------------}

{ These two definitions are obsolete, but are here for backwards
 * compatibility.  You never need to worry about these:
 }
 GADTOOLBIT    =  ($8000);
{ Use this mask to isolate the user part: }
 GADTOOLMASK   =  NOT (GADTOOLBIT);

{------------------------------------------------------------------------}

{ These definitions are for the GTLV_CallBack tag }

{ The different types of messages that a listview callback hook can see }
 LV_DRAW     =  $202;    { draw yourself, with state }

{ Possible return values from a callback hook }
 LVCB_OK      = 0;         { callback understands this message type    }
 LVCB_UNKNOWN = 1;         { callback does not understand this message }

{ states for LVDrawMsg.lvdm_State }
 LVR_NORMAL           = 0; { the usual                 }
 LVR_SELECTED         = 1; { for selected gadgets      }
 LVR_NORMALDISABLED   = 2;         { for disabled gadgets      }
 LVR_SELECTEDDISABLED = 8;         { disabled and selected     }

Type
{ structure of LV_DRAW messages, object is a (struct Node *) }
 pLVDrawMsg = ^tLVDrawMsg;
 tLVDrawMsg = record
    lvdm_MethodID       : ULONG;       { LV_DRAW                   }
    lvdm_RastPort       : pRastPort;   { where to render to        }
    lvdm_DrawInfo       : pDrawInfo;   { useful to have around     }
    lvdm_Bounds         : tRectangle;  { limits of where to render }
    lvdm_State          : ULONG;     { how to render     }
 end;


VAR
    GadToolsBase : pLibrary;

FUNCTION CreateContext(glistptr : pGadget): pGadget;
FUNCTION CreateGadgetA(kind : ULONG; gad : pGadget;const ng : pNewGadget;const taglist : pTagItem) : pGadget;
FUNCTION CreateMenusA(const newmenu : pNewMenu;const taglist : pTagItem) : pMenu;
PROCEDURE DrawBevelBoxA(rport : pRastPort; left : LONGINT; top : LONGINT; width : LONGINT; height : LONGINT;const taglist : pTagItem);
PROCEDURE FreeGadgets(gad : pGadget);
PROCEDURE FreeMenus(menu : pMenu);
PROCEDURE FreeVisualInfo(vi : POINTER);
FUNCTION GetVisualInfoA(screen : pScreen;const taglist : pTagItem) : POINTER;
PROCEDURE GT_BeginRefresh(win : pWindow);
PROCEDURE GT_EndRefresh(win : pWindow; complete : LONGINT);
FUNCTION GT_FilterIMsg(const imsg : pIntuiMessage) : pIntuiMessage;
FUNCTION GT_GetGadgetAttrsA(gad : pGadget; win : pWindow; req : pRequester;const taglist : pTagItem) : LONGINT;
FUNCTION GT_GetIMsg(iport : pMsgPort) : pIntuiMessage;
FUNCTION GT_PostFilterIMsg(imsg : pIntuiMessage) : pIntuiMessage;
PROCEDURE GT_RefreshWindow(win : pWindow; req : pRequester);
PROCEDURE GT_ReplyIMsg(imsg : pIntuiMessage);
PROCEDURE GT_SetGadgetAttrsA(gad : pGadget; win : pWindow; req : pRequester;const taglist : pTagItem);
FUNCTION LayoutMenuItemsA(firstitem : pMenuItem; vi : POINTER;const taglist : pTagItem) : BOOLEAN;
FUNCTION LayoutMenusA(firstmenu : pMenu; vi : POINTER;const taglist : pTagItem) : BOOLEAN;

function GTMENUITEM_USERDATA(menuitem : pMenuItem): pointer;
function GTMENU_USERDATA(menu : pMenu): pointer;

{Here we read how to compile this unit}
{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitGADTOOLSLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    GADTOOLSIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox;
{$endif dont_use_openlib}

function GTMENUITEM_USERDATA(menuitem : pMenuItem): pointer;
begin
    GTMENUITEM_USERDATA := pointer((pMenuItem(menuitem)+1));
end;

function GTMENU_USERDATA(menu : pMenu): pointer;
begin
    GTMENU_USERDATA := pointer((pMenu(menu)+1));
end;

FUNCTION CreateContext(glistptr : pGadget): pGadget;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L glistptr,A0
    MOVEA.L GadToolsBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateGadgetA(kind : ULONG; gad : pGadget;const ng : pNewGadget;const taglist : pTagItem) : pGadget;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  kind,D0
    MOVEA.L gad,A0
    MOVEA.L ng,A1
    MOVEA.L taglist,A2
    MOVEA.L GadToolsBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateMenusA(const newmenu : pNewMenu;const taglist : pTagItem) : pMenu;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L newmenu,A0
    MOVEA.L taglist,A1
    MOVEA.L GadToolsBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE DrawBevelBoxA(rport : pRastPort; left : LONGINT; top : LONGINT; width : LONGINT; height : LONGINT;const taglist : pTagItem);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rport,A0
    MOVE.L  left,D0
    MOVE.L  top,D1
    MOVE.L  width,D2
    MOVE.L  height,D3
    MOVEA.L taglist,A1
    MOVEA.L GadToolsBase,A6
    JSR -120(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeGadgets(gad : pGadget);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gad,A0
    MOVEA.L GadToolsBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeMenus(menu : pMenu);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L menu,A0
    MOVEA.L GadToolsBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeVisualInfo(vi : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L vi,A0
    MOVEA.L GadToolsBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION GetVisualInfoA(screen : pScreen;const taglist : pTagItem) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L taglist,A1
    MOVEA.L GadToolsBase,A6
    JSR -126(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE GT_BeginRefresh(win : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L win,A0
    MOVEA.L GadToolsBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE GT_EndRefresh(win : pWindow; complete : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L win,A0
    MOVE.L  complete,D0
    MOVEA.L GadToolsBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION GT_FilterIMsg(const imsg : pIntuiMessage) : pIntuiMessage;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L imsg,A1
    MOVEA.L GadToolsBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GT_GetGadgetAttrsA(gad : pGadget; win : pWindow; req : pRequester;const taglist : pTagItem) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gad,A0
    MOVEA.L win,A1
    MOVEA.L req,A2
    MOVEA.L taglist,A3
    MOVEA.L GadToolsBase,A6
    JSR -174(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GT_GetIMsg(iport : pMsgPort) : pIntuiMessage;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L iport,A0
    MOVEA.L GadToolsBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GT_PostFilterIMsg(imsg : pIntuiMessage) : pIntuiMessage;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L imsg,A1
    MOVEA.L GadToolsBase,A6
    JSR -108(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE GT_RefreshWindow(win : pWindow; req : pRequester);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L win,A0
    MOVEA.L req,A1
    MOVEA.L GadToolsBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE GT_ReplyIMsg(imsg : pIntuiMessage);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L imsg,A1
    MOVEA.L GadToolsBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE GT_SetGadgetAttrsA(gad : pGadget; win : pWindow; req : pRequester;const taglist : pTagItem);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L gad,A0
    MOVEA.L win,A1
    MOVEA.L req,A2
    MOVEA.L taglist,A3
    MOVEA.L GadToolsBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION LayoutMenuItemsA(firstitem : pMenuItem; vi : POINTER;const taglist : pTagItem) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L firstitem,A0
    MOVEA.L vi,A1
    MOVEA.L taglist,A2
    MOVEA.L GadToolsBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION LayoutMenusA(firstmenu : pMenu; vi : POINTER;const taglist : pTagItem) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L firstmenu,A0
    MOVEA.L vi,A1
    MOVEA.L taglist,A2
    MOVEA.L GadToolsBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of gadtools.library}
  {$Info don't forget to use InitGADTOOLSLibrary in the beginning of your program}

var
    gadtools_exit : Pointer;

procedure ClosegadtoolsLibrary;
begin
    ExitProc := gadtools_exit;
    if GadToolsBase <> nil then begin
        CloseLibrary(GadToolsBase);
        GadToolsBase := nil;
    end;
end;

procedure InitGADTOOLSLibrary;
begin
    GadToolsBase := nil;
    GadToolsBase := OpenLibrary(GADTOOLSNAME,LIBVERSION);
    if GadToolsBase <> nil then begin
        gadtools_exit := ExitProc;
        ExitProc := @ClosegadtoolsLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open gadtools.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    GADTOOLSIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of gadtools.library}

var
    gadtools_exit : Pointer;

procedure ClosegadtoolsLibrary;
begin
    ExitProc := gadtools_exit;
    if GadToolsBase <> nil then begin
        CloseLibrary(GadToolsBase);
        GadToolsBase := nil;
    end;
end;

begin
    GadToolsBase := nil;
    GadToolsBase := OpenLibrary(GADTOOLSNAME,LIBVERSION);
    if GadToolsBase <> nil then begin
        gadtools_exit := ExitProc;
        ExitProc := @ClosegadtoolsLibrary;
        GADTOOLSIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open gadtools.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    GADTOOLSIsCompiledHow := 3;
   {$Warning No autoopening of gadtools.library compiled}
   {$Warning Make sure you open gadtools.library yourself}
{$endif dont_use_openlib}


END. (* UNIT GADTOOLS *)




