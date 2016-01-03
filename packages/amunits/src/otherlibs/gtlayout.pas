{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 2000-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
    History:

    This is just a quick translation of gtlayout.h.
    I have made a small testprogram and all is well.
    16 Jul 2000.

    Added MessageBox for error report.
    31 Jul 2000.

    Added functions and procedures with array of const.
    For use with fpc 1.0.
    06 Jan 2003.

    Added the defines use_amiga_smartlink and
    use_auto_openlib.
    12 Jan 2003.

    Changed startcode for unit.
    10 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se

}
{$mode objfpc}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}


UNIT GTLAYOUT;


INTERFACE

USES Exec,intuition,utility, gadtools,graphics;



  { Kinds of objects supported in addition to the normal GadTools kinds  }

  const

     GTLAYOUTNAME : PCHar = 'gtlayout.library';

     HORIZONTAL_KIND = 45;
     VERTICAL_KIND = 46;
     END_KIND = 47;
     FRAME_KIND = 48;
     BOX_KIND = 49;
     FRACTION_KIND = 50;
     XBAR_KIND = 51;
     YBAR_KIND = 52;
     PASSWORD_KIND = 53;
     GAUGE_KIND = 54;
     TAPEDECK_KIND = 55;
     LEVEL_KIND = 56;
     BOOPSI_KIND = 57;
     POPUP_KIND = 58;
     TAB_KIND = 59;
     BLANK_KIND = 60;
     IMAGE_KIND = 61;

  { Where to place a gadget label  }

     PLACE_Left   = 0;
     PLACE_Right  = 1;
     PLACE_Above  = 2;
     PLACE_In     = 3;
     PLACE_Below  = 4;

    { How to align text lines in BOX_KIND gadgets  }

     ALIGNTEXT_Left      = 0;
     ALIGNTEXT_Centered  = 1;
     ALIGNTEXT_Right     = 2;
     ALIGNTEXT_Pad       = 3;

    { The button images available for TAPEDECK_KIND gadgets.  }

        TDBT_Backward   = 0;
        TDBT_Forward    = 1;
        TDBT_Previous   = 2;
        TDBT_Next       = 3;
        TDBT_Stop       = 4;
        TDBT_Pause      = 5;
        TDBT_Record     = 6;
        TDBT_Rewind     = 7;
        TDBT_Eject      = 8;
        TDBT_Play       = 9;

        TDBTLAST        = 10;

    { The frame types for groups.  }
        FRAMETYPE_None    = 0;
        FRAMETYPE_Label   = 1;
        FRAMETYPE_Tab     = 2;

    { How to align the window opened by LT_Build() on the screen.  }
       ALIGNF_Right = 1 shl 0;
       ALIGNF_Left = 1 shl 1;
       ALIGNF_Top = 1 shl 2;
       ALIGNF_Bottom = 1 shl 3;
       ALIGNF_ExtraRight = 1 shl 4;
       ALIGNF_ExtraLeft = 1 shl 5;
       ALIGNF_ExtraTop = 1 shl 6;
       ALIGNF_ExtraBottom = 1 shl 7;

{ Generic tags, applicable for several object types }
     LA_Chars           = TAG_USER+2;
     LA_LabelPlace              = TAG_USER+3;
     LA_ExtraSpace              = TAG_USER+4;
     LA_NoKey           = TAG_USER+30;
     LA_HighLabel               = TAG_USER+31;
     LA_LabelText               = TAG_USER+37;
     LA_LabelID         = TAG_USER+38;
     LA_ID                      = TAG_USER+39;
     LA_Type                    = TAG_USER+40;
     LA_PageSelector            = TAG_USER+79;
     LA_LabelChars              = TAG_USER+107;
     LA_DefaultSize             = TAG_USER+170;
     LA_LayoutSpace             = TAG_USER+189;

{ Storage type tags }
     LA_BYTE                    = TAG_USER+63;
     LA_UBYTE           = TAG_USER+64;
     LA_WORD                    = TAG_USER+65;
     LA_BOOL                    = TAG_USER+65;
     LA_UWORD           = TAG_USER+66;
     LA_LONG                    = TAG_USER+67;
     LA_ULONG           = TAG_USER+68;
     LA_STRPTR          = TAG_USER+69;

{ for use with LT_GetAttributes() only }
     LA_Left                    = TAG_USER+16;
     LA_Top                     = TAG_USER+17;
     LA_Width           = TAG_USER+18;
     LA_Height          = TAG_USER+19;
     LA_LabelLeft               = TAG_USER+114;
     LA_LabelTop                = TAG_USER+115;

{ BOOPSI_KIND }
     LABO_TagCurrent            = TAG_USER+119;
     LABO_TagTextAttr   = TAG_USER+120;
     LABO_TagDrawInfo   = TAG_USER+121;
     LABO_TagLink               = TAG_USER+129;
     LABO_TagScreen             = TAG_USER+132;
     LABO_Link          = TAG_USER+7;
     LABO_ClassInstance = TAG_USER+122;
     LABO_ClassName             = TAG_USER+123;
     LABO_ClassLibraryName      = TAG_USER+124;
     LABO_ExactWidth            = TAG_USER+127;
     LABO_ExactHeight   = TAG_USER+128;
     LABO_RelFontHeight = TAG_USER+131;
     LABO_Object                = TAG_USER+133;
     LABO_FullWidth             = TAG_USER+135;
     LABO_FullHeight            = TAG_USER+136;
     LABO_ActivateHook  = TAG_USER+141;

{ BOX_KIND }
     LABX_Labels                = TAG_USER+12;
     LABX_Lines         = TAG_USER+13;
     LABX_Chars         = TAG_USER+2;
     LABX_Rows          = TAG_USER+1;
     LABX_Index         = TAG_USER+14;
     LABX_Text          = TAG_USER+15;
     LABX_AlignText             = TAG_USER+27;
     LABX_DrawBox               = TAG_USER+11;
     LABX_FirstLabel            = TAG_USER+44;
     LABX_LastLabel             = TAG_USER+45;
     LABX_ReserveSpace  = TAG_USER+72;
     LABX_LabelTable            = TAG_USER+98;
     LABX_FirstLine             = TAG_USER+152;
     LABX_LastLine              = TAG_USER+153;
     LABX_LineTable             = TAG_USER+156;
     LABX_Line          = TAG_USER+161;
     LABX_LineID                = TAG_USER+162;
     LABX_TextPen               = TAG_USER+172;
     LABX_BackPen               = TAG_USER+173;
     LABX_Spacing               = TAG_USER+180;

{ BUTTON_KIND }
     LABT_ReturnKey             = TAG_USER+34;
     LABT_DefaultButton = TAG_USER+34;
     LABT_EscKey                = TAG_USER+56;
     LABT_ExtraFat              = TAG_USER+29;
     LABT_Lines         = TAG_USER+140;
     LABT_FirstLine             = TAG_USER+44;
     LABT_LastLine              = TAG_USER+45;
     LABT_DefaultCorrection     = TAG_USER+145;
     LABT_Smaller               = TAG_USER+147;

{ CYCLE_KIND }
     LACY_FirstLabel            = TAG_USER+44;
     LACY_LastLabel             = TAG_USER+45;
     LACY_LabelTable            = TAG_USER+98;
     LACY_AutoPageID            = TAG_USER+103;
     LACY_TabKey                = TAG_USER+118;

{ FRACTION_KIND }
     LAFR_IncrementerHook       = TAG_USER+85;

{ FRAME_KIND }
     LAFR_InnerWidth            = TAG_USER+9;
     LAFR_InnerHeight   = TAG_USER+10;
     LAFR_DrawBox               = TAG_USER+11;
     LAFR_RefreshHook   = TAG_USER+117;
     LAFR_GenerateEvents        = TAG_USER+155;
     LAFR_ResizeX               = TAG_USER+109;
     LAFR_ResizeY               = TAG_USER+110;

{ GAUGE_KIND }
     LAGA_Percent               = TAG_USER+36;
     LAGA_InfoLength            = TAG_USER+70;
     LAGA_InfoText              = TAG_USER+71;
     LAGA_NoTicks               = TAG_USER+143;
     LAGA_Discrete              = TAG_USER+144;
     LAGA_Tenth         = TAG_USER+144;

{ IMAGE_KIND }
     LAIM_Image         = TAG_USER+181;
     LAIM_BitMap                = TAG_USER+182;
     LAIM_BitMapLeft            = TAG_USER+183;
     LAIM_BitMapTop             = TAG_USER+184;
     LAIM_BitMapWidth   = TAG_USER+185;
     LAIM_BitMapHeight  = TAG_USER+186;
     LAIM_BitMapMask            = TAG_USER+187;

{ INTEGER_KIND }
     LAIN_LastGadget            = TAG_USER+28;
     LAIN_Min           = TAG_USER+23;
     LAIN_Max           = TAG_USER+24;
     LAIN_UseIncrementers       = TAG_USER+57;
     LAIN_Incrementers  = TAG_USER+57;
     LAIN_HistoryLines  = TAG_USER+59;
     LAIN_HistoryHook   = TAG_USER+80;
     LAIN_IncrementerHook       = TAG_USER+85;
     LAIN_Activate              = TAG_USER+148;

{ LISTVIEW_KIND }
     LALV_ExtraLabels   = TAG_USER+26;
     LALV_Labels                = TAG_USER+33;
     LALV_CursorKey             = TAG_USER+35;
     LALV_Columns               = TAG_USER+2;
     LALV_Lines         = TAG_USER+1;
     LALV_Link          = TAG_USER+7;
     LALV_FirstLabel            = TAG_USER+44;
     LALV_LastLabel             = TAG_USER+45;
     LALV_MaxGrowX              = TAG_USER+77;
     LALV_MaxGrowY              = TAG_USER+78;
     LALV_LabelTable            = TAG_USER+98;
     LALV_LockSize              = TAG_USER+106;
     LALV_ResizeX               = TAG_USER+109;
     LALV_ResizeY               = TAG_USER+110;
     LALV_MinChars              = TAG_USER+111;
     LALV_MinLines              = TAG_USER+112;
     LALV_FlushLabelLeft        = TAG_USER+113;
     LALV_TextAttr              = TAG_USER+138;
     LALV_AutoPageID            = TAG_USER+103;
     LALV_Selected              = TAG_USER+167;
     LALV_AdjustForString       = TAG_USER+174;

{ LEVEL_KIND }
     LAVL_Min           = GTSL_Min;
     LAVL_Max           = GTSL_Max;
     LAVL_Level         = GTSL_Level;
     LAVL_LevelFormat   = GTSL_LevelFormat;
     LAVL_LevelPlace            = GTSL_LevelPlace;
     LAVL_DispFunc              = GTSL_DispFunc;
     LAVL_FullCheck             = TAG_USER+22;
     LAVL_Freedom               = TAG_USER+177  { (I)  New in V41 };
     LAVL_Ticks         = TAG_USER+178  { (I)  New in V41 };
     LAVL_NumTicks              = TAG_USER+179  { (IS) New in V41 };
     LAVL_Lines         = TAG_USER+1;

{ MX_KIND }
     LAMX_FirstLabel            = TAG_USER+44;
     LAMX_LastLabel             = TAG_USER+45;
     LAMX_LabelTable            = TAG_USER+98;
     LAMX_TabKey                = TAG_USER+118;
     LAMX_AutoPageID            = TAG_USER+103;

{ PALETTE_KIND }
     LAPA_SmallPalette  = TAG_USER+32;
     LAPA_Lines         = TAG_USER+1;
     LAPA_UsePicker             = TAG_USER+137;
     LAPA_Picker                = TAG_USER+137;

{ PASSWORD_KIND }
     LAPW_String             = GTST_String;
     LAPW_LastGadget            = TAG_USER+28;
     LAPW_HistoryLines  = TAG_USER+59;
     LAPW_HistoryHook   = TAG_USER+80;
     LAPW_Activate              = TAG_USER+148;
     LAPW_MaxChars              = GTST_MaxChars;

{ POPUP_KIND }
     LAPU_FirstLabel            = TAG_USER+44;
     LAPU_LastLabel             = TAG_USER+45;
     LAPU_LabelTable            = TAG_USER+98;
     LAPU_AutoPageID            = TAG_USER+103;
     LAPU_TabKey                = TAG_USER+118;
     LAPU_Labels                = GTCY_Labels;
     LAPU_Active                = GTCY_Active;
     LAPU_CentreActive  = TAG_USER+163;

{ SLIDER_KIND }
     LASL_FullCheck             = TAG_USER+22;

{ SCROLLER_KIND }
     LASC_Thin          = TAG_USER+62;
     LASC_FullSize              = TAG_USER+188;

{ STRING_KIND }
     LAST_LastGadget            = TAG_USER+28;
     LAST_Link          = TAG_USER+7;
     LAST_Picker                = TAG_USER+5;
     LAST_UsePicker             = TAG_USER+5;
     LAST_HistoryLines  = TAG_USER+59;
     LAST_HistoryHook   = TAG_USER+80;
     LAST_CursorPosition        = TAG_USER+105;
     LAST_Activate              = TAG_USER+148;
     LAST_ValidateHook  = TAG_USER+165;

{ TAB_KIND }
     LATB_FirstLabel            = TAG_USER+44;
     LATB_LastLabel             = TAG_USER+45;
     LATB_LabelTable            = TAG_USER+98;
     LATB_AutoPageID            = TAG_USER+103;
     LATB_TabKey                = TAG_USER+118;
     LATB_Labels                = GTCY_Labels;
     LATB_Active                = GTCY_Active;
     LATB_FullWidth             = TAG_USER+149;
     LATB_FullSize              = TAG_USER+149;

{ TAPEDECK_KIND }
     LATD_ButtonType            = TAG_USER+86;
     LATD_Toggle                = TAG_USER+87;
     LATD_Pressed               = TAG_USER+88;
     LATD_Smaller               = TAG_USER+89;
     LATD_Tick          = TAG_USER+139;

{ TEXT_KIND }
     LATX_Picker                = TAG_USER+5;
     LATX_UsePicker             = TAG_USER+5;
     LATX_LockSize              = TAG_USER+106;

{ VERTICAL_KIND and HORIZONTAL_KIND }
     LAGR_Spread                = TAG_USER+6;
     LAGR_SameSize              = TAG_USER+8;
     LAGR_LastAttributes        = TAG_USER+46;
     LAGR_ActivePage            = TAG_USER+58;
     LAGR_Frame         = TAG_USER+104;
     LAGR_IndentX               = TAG_USER+130;
     LAGR_IndentY               = TAG_USER+134;
     LAGR_NoIndent              = TAG_USER+146;
     LAGR_SameWidth             = TAG_USER+150;
     LAGR_SameHeight            = TAG_USER+151;
     LAGR_FrameGroup            = TAG_USER+168;
     LAGR_AlignRight            = TAG_USER+171;

{ XBAR_KIND }
     LAXB_FullSize              = TAG_USER+50;
     LAXB_FullWidth             = TAG_USER+50;

{ Applicable for layout handle only }
     LAHN_TextAttr              = TAG_USER+41;
     LAHN_AutoActivate  = TAG_USER+42;
     LAHN_LocaleHook            = TAG_USER+4;
     LAHN_CloningPermitted      = TAG_USER+61;
     LAHN_EditHook              = TAG_USER+74;
     LAHN_ExactClone            = TAG_USER+75;
     LAHN_MenuGlyphs            = TAG_USER+76;
     LAHN_Parent                = TAG_USER+83;
     LAHN_BlockParent   = TAG_USER+84;
     LAHN_SimpleClone   = TAG_USER+90;
     LAHN_ExitFlush             = TAG_USER+108;
     LAHN_UserData              = TAG_USER+116;
     LAHN_RawKeyFilter  = TAG_USER+142;
     LAHN_DontPickShortcuts     = TAG_USER+154;
     LAHN_NoKeys                = TAG_USER+154;
     LAHN_PubScreen             = TAG_USER+157;
     LAHN_PubScreenName = TAG_USER+158;
     LAHN_PubScreenFallBack     = TAG_USER+159;
     LAHN_CloneScreenTitle      = TAG_USER+175;
     LAHN_CloneScreenTitleID    = TAG_USER+176;
     LAHN_TopGroupType  = TAG_USER+190;

{ Applicable for menus only. }
     LAMN_FirstLabel            = LABX_FirstLabel;
     LAMN_LastLabel             = LABX_LastLabel;
     LAMN_LabelTable            = TAG_USER+98;
     LAMN_TitleText             = TAG_USER+17000;
     LAMN_TitleID               = TAG_USER+17001;
     LAMN_ItemText              = TAG_USER+17002;
     LAMN_ItemID                = TAG_USER+17003;
     LAMN_SubText               = TAG_USER+17004;
     LAMN_SubID         = TAG_USER+17005;
     LAMN_KeyText               = TAG_USER+17006;
     LAMN_KeyID         = TAG_USER+17007;
     LAMN_CommandText   = TAG_USER+17008;
     LAMN_CommandID             = TAG_USER+17009;
     LAMN_MutualExclude = TAG_USER+17010;
     LAMN_UserData              = TAG_USER+17011;
     LAMN_Disabled              = TAG_USER+17012;
     LAMN_CheckIt               = TAG_USER+17013;
     LAMN_Checked               = TAG_USER+17014;
     LAMN_Toggle                = TAG_USER+17015;
     LAMN_Code          = TAG_USER+17016;
     LAMN_Qualifier             = TAG_USER+17017;
     LAMN_Char          = TAG_USER+17018;
     LAMN_ID                    = TAG_USER+17019;
     LAMN_AmigaGlyph            = TAG_USER+17020;
     LAMN_CheckmarkGlyph        = TAG_USER+17021;
     LAMN_Error         = TAG_USER+17022;
     LAMN_Screen                = TAG_USER+17023;
     LAMN_TextAttr              = TAG_USER+17024;
     LAMN_LayoutHandle  = TAG_USER+17025;
     LAMN_Handle                = TAG_USER+17025;
     LAMN_ExtraSpace            = TAG_USER+17026;
     LAMN_FullMenuNum   = TAG_USER+160;

{ Applicable for window only }
     LAWN_Menu          = TAG_USER+25;
     LAWN_UserPort              = TAG_USER+47;
     LAWN_Left          = TAG_USER+48;
     LAWN_Top           = TAG_USER+49;
     LAWN_Zoom          = TAG_USER+50;
     LAWN_MaxPen                = TAG_USER+52;
     LAWN_BelowMouse            = TAG_USER+53;
     LAWN_MoveToWindow  = TAG_USER+54;
     LAWN_AutoRefresh   = TAG_USER+55;
     LAWN_HelpHook              = TAG_USER+73;
     LAWN_Parent                = TAG_USER+81;
     LAWN_BlockParent   = TAG_USER+82;
     LAWN_SmartZoom             = TAG_USER+91;
     LAWN_Title         = TAG_USER+92;
     LAWN_TitleText             = TAG_USER+92;
     LAWN_Bounds                = TAG_USER+93;
     LAWN_ExtraWidth            = TAG_USER+94;
     LAWN_ExtraHeight   = TAG_USER+95;
     LAWN_IDCMP         = TAG_USER+96;
     LAWN_AlignWindow   = TAG_USER+97;
     LAWN_TitleID               = TAG_USER+99;
     LAWN_FlushLeft             = TAG_USER+14000        { NOTEZ-BIEN: = TAG_USER+99 = WA_Dummy and can clash };
     LAWN_FlushTop              = TAG_USER+14001        {             with Intuition!                      };
     LAWN_Show          = TAG_USER+14002;
     LAWN_MenuTemplate  = TAG_USER+14003;
     LAWN_MenuTags              = TAG_USER+14004;
     LAWN_NoInitialRefresh      = TAG_USER+164;
     LAWN_LimitWidth            = TAG_USER+165;
     LAWN_LimitHeight   = TAG_USER+166;
     LAWN_UserData              = TAG_USER+169;

{ Private tags; do not use, or you'll run into trouble! }
     LA_Private1                = TAG_USER+100;
     LA_Private2                = TAG_USER+101;

{ Last tag item value used }
     LAST_TAG           = TAG_USER+190;

    { Identifies the absence of a link for a listview or a string gadget  }
       NIL_LINK = -(2);
    {                                                                            }
    { String gadget type history hook support: you will either get
       the following value passed as the message parameter to your
       hook function, or a pointer to a null-terminated string you should
       copy and create a Node from, which you should then add to the tail
       of your history list. Place a pointer to your history list in the
       Hook.h_Data entry.
      }
       HISTORYHOOK_DiscardOldest = 0;
    {                                                                            }
    { Refresh hook support: you will get the following structure
       passed as the message and a pointer to the LayoutHandle as
       the object.
      }

    type

       tRefreshMsg = record
            ID : LONG;
            Left : WORD;
            Top : WORD;
            Width : WORD;
            Height : WORD;
         end;

       pRefreshMsg = ^tRefreshMsg;
    {                                                                            }
    { Incrementer hook support: you will get the current value
       passed as the object and one of the following values as
       the message. Return the number to be used.
      }
const
      INCREMENTERMSG_Decrement = -1;    { Decrement value }
      INCREMENTERMSG_Initial   =  0;    { Initial value passed upon gadget creation }
      INCREMENTERMSG_Increment =  1;    { Increment value }


    const
       TICKS_None  = 0; {/* No ticks please */}
       TICKS_Left  = 1; {/* Place ticks left of the slider (FREEVERT only) */}
       TICKS_Both  = 2;         {/* Place ticks on both sides of the slider */}
       TICKS_Above = TICKS_Left;
    {                                                                            }
    { The central data structure of the layout process.  }
    { Requires gtlayout.library V9  }
    { Requires gtlayout.library V13  }
    { Hands off, private fields follow....  }

    type

       tLayoutHandle = record
            Screen : PScreen;
            DrawInfo : PDrawInfo;
            Window : PWindow;
            VisualInfo : Pointer;
            AmigaGlyph : PImage;
            CheckGlyph : PImage;
            UserData : Pointer;
            Menu : PMenu;
         end;

       pLayoutHandle = ^tLayoutHandle;

    { Help key hook support: the hook will be called with a "struct IBox  "
       as the object and a "struct HelpMsg  ". The IBox describes the object
       the mouse was positioned over, such as a button, a listview, etc.
       The "ObjectID" will indicate the ID of the object the mouse was
       positioned over. The ID will be -1 if no object was to be found.
      }
    { Window layout handle  }
    { ID of the object, -1 for full window  }

       tHelpMsg = record
            Handle : PLayoutHandle;
            ObjectID : LONG;
         end;

       pHelpMsg = ^tHelpMsg;
    {   pUWord = ^UWord; }
       ppGadget = ^pGadget;
       pUWORD   = ^UWORD;

PROCEDURE LT_LevelWidth(par1 : pLayoutHandle; par2 : pCHAR; par3 : POINTER; par4 : LONGINT; par5 : LONGINT; par6 : pLONGINT; par7 : pLONGINT; last : LONGINT);
PROCEDURE LT_DeleteHandle(last : pLayoutHandle);
FUNCTION LT_CreateHandle(par1 : pScreen; last : pTextAttr) : pLayoutHandle;
FUNCTION LT_CreateHandleTagList(par1 : pScreen; tags : pTagItem) : pLayoutHandle;
FUNCTION LT_Rebuild(par1 : pLayoutHandle; par2 : pIBox; par3 : LONGINT; par4 : LONGINT; last : LONGINT) : BOOLEAN;
PROCEDURE LT_HandleInput(par1 : pLayoutHandle; par2 : ulong; par3 : pulong; par4 : pUWORD; last :ppGadget);
PROCEDURE LT_BeginRefresh(last : pLayoutHandle);
PROCEDURE LT_EndRefresh(par1 : pLayoutHandle; last : LONGINT);
FUNCTION LT_GetAttributesA(par1 : pLayoutHandle; par2 : LONGINT; tags : pTagItem) : LONGINT;
PROCEDURE LT_SetAttributesA(par1 : pLayoutHandle; par2 : LONGINT; tags : pTagItem);
PROCEDURE LT_AddA(par1 : pLayoutHandle; par2 : LONGINT; par3 : pCHAR; par4 : LONGINT; tags : pTagItem);
PROCEDURE LT_NewA(par1 : pLayoutHandle; tags : pTagItem);
PROCEDURE LT_EndGroup(last : pLayoutHandle);
FUNCTION LT_LayoutA(par1 : pLayoutHandle; par2 : pCHAR; par3 : pIBox; par4 : LONGINT; par5 : LONGINT; par6 : ulong; par7 : LONGINT; tags : pTagItem) : pWindow;
FUNCTION LT_LayoutMenusA(par1 : pLayoutHandle; par2 : pNewMenu; tags : pTagItem) : pMenu;
FUNCTION LT_LabelWidth(par1 : pLayoutHandle; last : pCHAR) : LONGINT;
FUNCTION LT_LabelChars(par1 : pLayoutHandle; last : pCHAR) : LONGINT;
PROCEDURE LT_LockWindow(last : pWindow);
PROCEDURE LT_UnlockWindow(last : pWindow);
PROCEDURE LT_DeleteWindowLock(last : pWindow);
PROCEDURE LT_ShowWindow(par1 : pLayoutHandle; last : LONGINT);
PROCEDURE LT_Activate(par1 : pLayoutHandle; last : LONGINT);
FUNCTION LT_PressButton(par1 : pLayoutHandle; last : LONGINT) : BOOLEAN;
FUNCTION LT_GetCode(par1 : ulong; par2 : ulong; par3 : ulong; last : pGadget) : LONGINT;
FUNCTION LT_GetIMsg(last : pLayoutHandle) : pIntuiMessage;
PROCEDURE LT_ReplyIMsg(last : pIntuiMessage);
FUNCTION LT_BuildA(par1 : pLayoutHandle; tags : pTagItem) : pWindow;
FUNCTION LT_RebuildTagList(par1 : pLayoutHandle; par2 : LONGINT; tags : pTagItem) : BOOLEAN;
PROCEDURE LT_UpdateStrings(last : pLayoutHandle);
PROCEDURE LT_DisposeMenu(last : pMenu);
FUNCTION LT_NewMenuTemplate(par1 : pScreen; par2 : pTextAttr; par3 : pImage; par4 : pImage; par5 : pLONGINT; last : pNewMenu) : pMenu;
FUNCTION LT_NewMenuTagList(tags : pTagItem) : pMenu;
PROCEDURE LT_MenuControlTagList(par1 : pWindow; par2 : pMenu; tags : pTagItem);
FUNCTION LT_GetMenuItem(par1 : pMenu; last : ulong) : pMenuItem;
FUNCTION LT_FindMenuCommand(par1 : pMenu; par2 : ulong; par3 : ulong; last : pGadget) : pMenuItem;
PROCEDURE LT_NewLevelWidth(par1 : pLayoutHandle; par2 : pCHAR; par3 : POINTER; par4 : LONGINT; par5 : LONGINT; par6 : pLONGINT; par7 : pLONGINT; last : LONGINT);
PROCEDURE LT_Refresh(last : pLayoutHandle);
PROCEDURE LT_CatchUpRefresh(last : pLayoutHandle);
FUNCTION LT_GetWindowUserData(par1 : pWindow; last : POINTER) : POINTER;

{
     This is functions and procedures with array of const.
     For use with fpc 1.0.7 and above.
}

FUNCTION LT_CreateHandleTags(screen : pScreen; const tagList : Array Of Const) : pLayoutHandle;
FUNCTION LT_GetAttributes(handle : pLayoutHandle; id : LONGINT; const tagList : Array Of Const) : LONGINT;
PROCEDURE LT_SetAttributes(handle : pLayoutHandle; id : LONGINT; const tagList : Array Of Const);
PROCEDURE LT_Add(handle : pLayoutHandle; _type : LONGINT; _label : pCHAR; id : LONGINT; const tagList : Array Of Const);
PROCEDURE LT_New(handle : pLayoutHandle; const tagList : Array Of Const);
FUNCTION LT_Layout(handle : pLayoutHandle; title : pCHAR; bounds : pIBox; extraWidth : LONGINT; extraHeight : LONGINT; idcmp : longword; align : LONGINT; const tagParams : Array Of Const) : pWindow;
FUNCTION LT_LayoutMenus(handle : pLayoutHandle; menuTemplate : pNewMenu; const tagParams : Array Of Const) : pMenu;
FUNCTION LT_Build(handle : pLayoutHandle; const tagParams : Array Of Const) : pWindow;
FUNCTION LT_RebuildTags(handle : pLayoutHandle; clear : LONGINT; const tags : Array Of Const) : BOOLEAN;
FUNCTION LT_NewMenuTags(const tagList : Array Of Const) : pMenu;
PROCEDURE LT_MenuControlTags(window : pWindow; intuitionMenu : pMenu; const tags : Array Of Const);


VAR GTLayoutBase : pLibrary;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitGTLAYOUTLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    GTLAYOUTIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray;

PROCEDURE LT_LevelWidth(par1 : pLayoutHandle; par2 : pCHAR; par3 : POINTER; par4 : LONGINT; par5 : LONGINT;  par6 : pLONGINT;  par7 : pLONGINT; last : LONGINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L par2,A1
        MOVEA.L par3,A2
        MOVE.L  par4,D0
        MOVE.L  par5,D1
        MOVEA.L par6,A3
        MOVEA.L par7,A5
        MOVE.L  last,D2
        MOVEA.L GTLayoutBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_DeleteHandle(last : pLayoutHandle);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION LT_CreateHandle(par1 : pScreen; last : pTextAttr) : pLayoutHandle;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L last,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LT_CreateHandleTagList(par1 : pScreen; tags : pTagItem) : pLayoutHandle;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L tags,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LT_Rebuild(par1 : pLayoutHandle; par2 : pIBox; par3 : LONGINT; par4 : LONGINT; last : LONGINT) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L par2,A1
        MOVEA.L par3,A2
        MOVE.L  par4,D0
        MOVE.L  last,D1
        MOVEA.L GTLayoutBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE LT_HandleInput(par1 : pLayoutHandle; par2 : ulong; par3 : pulong;par4 : pUWORD; last : ppGadget);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVE.L  par2,D0
        MOVEA.L par3,A1
        MOVEA.L par4,A2
        MOVEA.L last,A3
        MOVEA.L GTLayoutBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_BeginRefresh(last : pLayoutHandle);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_EndRefresh(par1 : pLayoutHandle; last : LONGINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVE.L  last,D0
        MOVEA.L GTLayoutBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION LT_GetAttributesA(par1 : pLayoutHandle; par2 : LONGINT; tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVE.L  par2,D0
        MOVEA.L tags,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE LT_SetAttributesA(par1 : pLayoutHandle; par2 : LONGINT; tags : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVE.L  par2,D0
        MOVEA.L tags,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_AddA(par1 : pLayoutHandle; par2 : LONGINT; par3 : pCHAR; par4 : LONGINT; tags : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVE.L  par2,D0
        MOVE.L  par3,D1
        MOVE.L  par4,D2
        MOVEA.L tags,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_NewA(par1 : pLayoutHandle; tags : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L tags,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_EndGroup(last : pLayoutHandle);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION LT_LayoutA(par1 : pLayoutHandle; par2 : pCHAR; par3 : pIBox; par4 : LONGINT; par5 : LONGINT; par6 : ulong; par7 : LONGINT; tags : pTagItem) : pWindow;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L par2,A1
        MOVEA.L par3,A2
        MOVE.L  par4,D0
        MOVE.L  par5,D1
        MOVE.L  par6,D2
        MOVE.L  par7,D3
        MOVEA.L tags,A3
        MOVEA.L GTLayoutBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LT_LayoutMenusA(par1 : pLayoutHandle; par2 : pNewMenu; tags : pTagItem) : pMenu;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L par2,A1
        MOVEA.L tags,A2
        MOVEA.L GTLayoutBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LT_LabelWidth(par1 : pLayoutHandle; last : pCHAR) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L last,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -138(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LT_LabelChars(par1 : pLayoutHandle; last : pCHAR) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L last,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -144(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE LT_LockWindow(last : pWindow);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -150(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_UnlockWindow(last : pWindow);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -156(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_DeleteWindowLock(last : pWindow);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -162(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_ShowWindow(par1 : pLayoutHandle; last : LONGINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L last,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -168(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_Activate(par1 : pLayoutHandle; last : LONGINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVE.L  last,D0
        MOVEA.L GTLayoutBase,A6
        JSR     -174(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION LT_PressButton(par1 : pLayoutHandle; last : LONGINT) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVE.L  last,D0
        MOVEA.L GTLayoutBase,A6
        JSR     -180(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION LT_GetCode(par1 : ulong; par2 : ulong; par3 : ulong; last : pGadget) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  par1,D0
        MOVE.L  par2,D1
        MOVE.L  par3,D2
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -186(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LT_GetIMsg(last : pLayoutHandle) : pIntuiMessage;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -192(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE LT_ReplyIMsg(last : pIntuiMessage);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -198(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION LT_BuildA(par1 : pLayoutHandle; tags : pTagItem) : pWindow;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L tags,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -204(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LT_RebuildTagList(par1 : pLayoutHandle; par2 : LONGINT; tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVE.L  par2,D0
        MOVEA.L tags,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -210(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE LT_UpdateStrings(last : pLayoutHandle);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -216(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_DisposeMenu(last : pMenu);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -222(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION LT_NewMenuTemplate(par1 : pScreen; par2 : pTextAttr; par3 : pImage; par4 : pImage; par5 : pLONGINT; last : pNewMenu) : pMenu;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L par2,A1
        MOVEA.L par3,A2
        MOVEA.L par4,A3
        MOVE.L  par5,D0
        MOVE.L  last,D1
        MOVEA.L GTLayoutBase,A6
        JSR     -228(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LT_NewMenuTagList(tags : pTagItem) : pMenu;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L tags,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -234(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE LT_MenuControlTagList(par1 : pWindow; par2 : pMenu; tags : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L par2,A1
        MOVEA.L tags,A2
        MOVEA.L GTLayoutBase,A6
        JSR     -240(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION LT_GetMenuItem(par1 : pMenu; last : ulong) : pMenuItem;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVE.L  last,D0
        MOVEA.L GTLayoutBase,A6
        JSR     -246(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LT_FindMenuCommand(par1 : pMenu; par2 : ulong; par3 : ulong; last : pGadget) : pMenuItem;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVE.L  par2,D0
        MOVE.L  par3,D1
        MOVEA.L last,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -252(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE LT_NewLevelWidth(par1 : pLayoutHandle; par2 : pCHAR; par3 : POINTER; par4 : LONGINT; par5 : LONGINT;  par6 : pLONGINT;  par7 : pLONGINT; last : LONGINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L par2,A1
        MOVEA.L par3,A2
        MOVE.L  par4,D0
        MOVE.L  par5,D1
        MOVEA.L par6,A3
        MOVE.L  par7,D3
        MOVE.L  last,D2
        MOVEA.L GTLayoutBase,A6
        JSR     -258(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_Refresh(last : pLayoutHandle);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -264(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LT_CatchUpRefresh(last : pLayoutHandle);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L last,A0
        MOVEA.L GTLayoutBase,A6
        JSR     -270(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION LT_GetWindowUserData(par1 : pWindow; last : POINTER) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L par1,A0
        MOVEA.L last,A1
        MOVEA.L GTLayoutBase,A6
        JSR     -276(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;


{
 Functions and procedures with array of const go here
}
FUNCTION LT_CreateHandleTags(screen : pScreen; const tagList : Array Of Const) : pLayoutHandle;
begin
    LT_CreateHandleTags := LT_CreateHandleTagList(screen , readintags(tagList));
end;

FUNCTION LT_GetAttributes(handle : pLayoutHandle; id : LONGINT; const tagList : Array Of Const) : LONGINT;
begin
    LT_GetAttributes := LT_GetAttributesA(handle , id , readintags(tagList));
end;

PROCEDURE LT_SetAttributes(handle : pLayoutHandle; id : LONGINT; const tagList : Array Of Const);
begin
    LT_SetAttributesA(handle , id , readintags(tagList));
end;

PROCEDURE LT_Add(handle : pLayoutHandle; _type : LONGINT; _label : pCHAR; id : LONGINT; const tagList : Array Of Const);
begin
    LT_AddA(handle , _type , _label , id , readintags(tagList));
end;

PROCEDURE LT_New(handle : pLayoutHandle; const tagList : Array Of Const);
begin
    LT_NewA(handle , readintags(tagList));
end;

FUNCTION LT_Layout(handle : pLayoutHandle; title : pCHAR; bounds : pIBox; extraWidth : LONGINT; extraHeight : LONGINT; idcmp : longword; align : LONGINT; const tagParams : Array Of Const) : pWindow;
begin
    LT_Layout := LT_LayoutA(handle , title , bounds , extraWidth , extraHeight , idcmp , align , readintags(tagParams));
end;

FUNCTION LT_LayoutMenus(handle : pLayoutHandle; menuTemplate : pNewMenu; const tagParams : Array Of Const) : pMenu;
begin
    LT_LayoutMenus := LT_LayoutMenusA(handle , menuTemplate , readintags(tagParams));
end;

FUNCTION LT_Build(handle : pLayoutHandle; const tagParams : Array Of Const) : pWindow;
begin
    LT_Build := LT_BuildA(handle , readintags(tagParams));
end;

FUNCTION LT_RebuildTags(handle : pLayoutHandle; clear : LONGINT; const tags : Array Of Const) : BOOLEAN;
begin
    LT_RebuildTags := LT_RebuildTagList(handle , clear , readintags(tags));
end;

FUNCTION LT_NewMenuTags(const tagList : Array Of Const) : pMenu;
begin
    LT_NewMenuTags := LT_NewMenuTagList(readintags(tagList));
end;

PROCEDURE LT_MenuControlTags(window : pWindow; intuitionMenu : pMenu; const tags : Array Of Const);
begin
    LT_MenuControlTagList(window , intuitionMenu , readintags(tags));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : Cardinal = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of gtlayout.library}
  {$Info don't forget to use InitGTLAYOUTLibrary in the beginning of your program}

var
    gtlayout_exit : Pointer;

procedure ClosegtlayoutLibrary;
begin
    ExitProc := gtlayout_exit;
    if GTLayoutBase <> nil then begin
        CloseLibrary(GTLayoutBase);
        GTLayoutBase := nil;
    end;
end;

procedure InitGTLAYOUTLibrary;
begin
    GTLayoutBase := nil;
    GTLayoutBase := OpenLibrary(GTLAYOUTNAME,LIBVERSION);
    if GTLayoutBase <> nil then begin
        gtlayout_exit := ExitProc;
        ExitProc := @ClosegtlayoutLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open gtlayout.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    GTLAYOUTIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of gtlayout.library}

var
    gtlayout_exit : Pointer;

procedure ClosegtlayoutLibrary;
begin
    ExitProc := gtlayout_exit;
    if GTLayoutBase <> nil then begin
        CloseLibrary(GTLayoutBase);
        GTLayoutBase := nil;
    end;
end;

begin
    GTLayoutBase := nil;
    GTLayoutBase := OpenLibrary(GTLAYOUTNAME,LIBVERSION);
    if GTLayoutBase <> nil then begin
        gtlayout_exit := ExitProc;
        ExitProc := @ClosegtlayoutLibrary;
        GTLAYOUTIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open gtlayout.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    GTLAYOUTIsCompiledHow := 3;
   {$Warning No autoopening of gtlayout.library compiled}
   {$Warning Make sure you open gtlayout.library yourself}
{$endif dont_use_openlib}


END. (* UNIT GTLAYOUT *)



