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
    History

    Updated to triton 2.0. Added function with array of const.
    09 Jan 2003.

    Added the defines use_amiga_smartlink and
    use_auto_openlib.
    12 Jan 2003.

    Changed integer > smallint.
    Changed cardinal > longword.
    Changed startcode for unit.
    11 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se Nils Sjoholm

}

{$mode objfpc}
{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

UNIT TRITON;

INTERFACE

uses exec, intuition, graphics, utility;


{* ------------------------------------------------------------------------------ *}
{* library name and version                                                       *}
{* ------------------------------------------------------------------------------ *}

CONST   TRITONNAME        : PChar = 'triton.library';
        TRITON10VERSION   = 1;
        TRITON11VERSION   = 2;
        TRITON12VERSION   = 3;
        TRITON13VERSION   = 4;
        TRITON14VERSION   = 5;
        TRITON20VERSION   = 6;
        TRITONVERSION     = TRITON20VERSION;

{* ------------------------------------------------------------------------------ *}
{* Triton Message                                                                 *}
{* ------------------------------------------------------------------------------ *}

TYPE
     pTR_Project = ^tTR_Project;
     pTR_App     = ^tTR_App;

     pTR_Message = ^tTR_Message;
     tTR_Message = RECORD
         trm_Project   : pTR_Project;   {* the message                  *}
         trm_Id        : ULONG;         {* The object's ID              *}
         trm_Class     : ULONG;         {* The Triton message class     *}
         trm_Data      : ULONG;         {* The class-specific data      *}
         trm_Code      : ULONG;         {* Currently only used BY       *}
                                        {* TRMS_KEYPRESSED              *}
         trm_Pad0      : ULONG;         {* qualifier is only 16 Bit     *}
         trm_Qualifier : ULONG;         {* Qualifiers                   *}
         trm_Seconds   : ULONG;         {* \ Copy of system clock time  *}
         trm_Micros    : ULONG;         {* / (Only where available! IF  *}
                                        {*    not set, seconds is NULL) *}
         trm_App       : pTR_App;
     END;                               {* End of TR_Message            *}

{* Data for TROM_NEW *}
     pTROM_NewDataPtr = ^tTROM_NewData;
     tTROM_NewData = record
         {* The following elements are set up for the object (read only) *}
         project    : pTR_Project;     {*  TR_ProjectPtr;  *}
         firstitem  : pTagItem;
         objecttype : ULONG;
         grouptype  : ULONG;
         itemdata   : ULONG;
         backfilltype : ULONG;
         {* The following elements have to be set by the object and read by
class_DisplayObject *}
         parseargs  : BOOL;
     end;


{* Data for TROM_INSTALL *}
     pTROM_InstallData = ^tTROM_InstallData;
     tTROM_InstallData = record
         left : ULONG;
         top  : ULONG;
         width : ULONG;
         height : ULONG;
     end;


{* Data for TROM_SETATTRIBUTE *}
     pTROM_SetAttributeData = ^tTROM_SetAttributeData;
     tTROM_SetAttributeData = record
         attribute : ULONG;
         value     : ULONG;
     end;


{* Data for TROM_EVENT *}
     pTROM_EventData = ^tTROM_EventData;
     tTROM_EventData = record
         imsg : pIntuiMessage;
     end;

{* ------------------------------------------------------------------------------ *}
{* The Application Structure                                                      *}
{* ------------------------------------------------------------------------------ *}

     tTR_App = RECORD {* This structure is PRIVATE! *}
         tra_MemPool    : Pointer;       {* The memory pool             *}
         tra_BitMask    : ULONG;         {* Bits to Wait() for          *}
         tra_Name       : STRPTR;        {* Unique name                 *}
     END; {* TR_App *}

     pTR_Class = ^tTR_Class;
     tTR_Class = RECORD
         trc_Node     : tMinNode;         {* PRIVATE! *}
         trc_SuperClass : pTR_Class;
     end;




{* ------------------------------------------------------------------------------ *}
{* The Dimension Structure                                                        *}
{* ------------------------------------------------------------------------------ *}

     pTR_Dimensions = ^tTR_Dimensions;
     tTR_Dimensions = RECORD
         trd_Left          : Word;
         trd_Top           : Word;
         trd_Width         : Word;
         trd_Height        : Word;
         trd_Left2         : Word;
         trd_Top2          : Word;
         trd_Width2        : Word;
         trd_Height2       : Word;
         trd_Zoomed        : BOOL;
         reserved          : ARRAY [0..2] OF Word;
     END; {* TR_Dimensions *}


{* ////////////////////////////////////////////////////////////////////// *}
{* ///////////////////////////// Default classes, attributes and flags // *}
{* ////////////////////////////////////////////////////////////////////// *}


     pTROD_Object = ^tTROD_Object;
     tTROD_Object = record
         Node : tMinNode;             {* The node for linking all objects *}
         tr_Class : pTR_Class;        {* The object's class *}
         Project : pTR_Project;    {* The object's project *}
         Reserved : array [0..5] of ULONG;
     end;



     pTROD_DisplayObject = ^tTROD_DisplayObject;
     tTROD_DisplayObject = record
         O           : tTROD_Object;       {* Superclass object data *}
         ID          : ULONG;             {* The object's ID *}
         MinWidth    : ULONG;             {* The precalculated minimum width *}
         MinHeight   : ULONG;             {* The precalculated minimum height *}
         Left        : ULONG;             {* The X coordinate of the object *}
         Top         : ULONG;             {* The Y coordinate of the object *}
         Width       : ULONG;             {* The width of the object *}
         Height      : ULONG;             {* The height of the object *}
         Flags       : ULONG;             {* See below for flags *}
         XResize     : BOOL;              {* Horizontally resizable? *}
         YResize     : BOOL;              {* Vertically resizable? *}
         QuickHelpString : STRPTR;        {* QuickHelp string *}
         Shortcut    : smallint;           {* The object's shortcut *}
         Backfilltype : ULONG;            {* The object's backfill type *}
         Installed   : BOOL;              {* Does the object have an on-screen
representation? *}
         Reserved    : array [0..3] of ULONG;   {* Private! *}
     end;


{* Data for TROM_HIT *}
     pTROM_HitData = ^tTROM_HitData;
     tTROM_HitData = record
         x : ULONG;
         y : ULONG;
         tr_object : pTROD_DisplayObject;
     end;


{* ------------------------------------------------------------------------------ *}
{* The Projects Structure                                                         *}
{* ------------------------------------------------------------------------------ *}

     tTR_Project = RECORD
         tro_SC_Object                : tTROD_Object;          {* PRIVATE *}
         trp_App                      : pTR_App;            {* Our application
*}
         trp_MemPool                  : Pointer;              {* The memory pool *}
         trp_ID                       : ULONG;                {* The project's ID *}
         trp_IDCMPFlags               : ULONG;                {* The IDCMP flags *}
         trp_Window                   : pWindow;            {* The default window *}
         trp_AspectFixing             : Word;                 {* Pixel aspect
correction factor *}
     END;                                               {* End of TR_Projects
*}

{* Message classes *}
CONST   TRMS_CLOSEWINDOW        = 1;  {* The window should be closed *}
        TRMS_ERROR              = 2;  {* An error occured. Error code in trm_Data *}
        TRMS_NEWVALUE           = 3;  {* Object's VALUE has changed. New VALUE in
trm_Data *}
        TRMS_ACTION             = 4;  {* Object has triggered an action *}
        TRMS_ICONDROPPED        = 5;  {* Icon dropped over window (ID=0) or DropBox.
AppMessage* in trm_Data *}
        TRMS_KEYPRESSED         = 6;  {* Key pressed. trm_Data contains ASCII code,
trm_Code raw code and *}
                                      {* trm_Qualifier contains qualifiers *}
        TRMS_HELP               = 7;  {* The user requested help for the specified ID
*}
        TRMS_DISKINSERTED       = 8;  {* A disk has been inserted into a drive *}
        TRMS_DISKREMOVED        = 9;  {* A disk has been removed from a drive *}


{* ////////////////////////////////////////////////////////////////////// *}
{* //////////////////////////////////////////////// Triton error codes // *}
{* ////////////////////////////////////////////////////////////////////// *}

        TRER_OK                 = 0;        {* No error *}

        TRER_ALLOCMEM           = 1;        {* Not enough memory *}
        TRER_OPENWINDOW         = 2;        {* Can't open window *}
        TRER_WINDOWTOOBIG       = 3;        {* Window would be too big for screen *}
        TRER_DRAWINFO           = 4;        {* Can't get screen's DrawInfo *}
        TRER_OPENFONT           = 5;        {* Can't open font *}
        TRER_CREATEMSGPORT      = 6;        {* Can't create message port *}
        TRER_INSTALLOBJECT      = 7;        {* Can't create an object *}
        TRER_CREATECLASS        = 8;        {* Can't create a class *}
        TRER_NOLOCKPUBSCREEN    = 9;        {* Can't lock public screen *}
        TRER_CREATEMENUS        = 12;       {* Error while creating the menus *}
        TRER_GT_CREATECONTEXT   = 14;       {* Can't create gadget context *}

        TRER_MAXERRORNUM        = 15;       {* PRIVATE! *}


{* ////////////////////////////////////////////////////////////////////// *}
{* /////////////////////////////////////////////////// Object messages // *}
{* ////////////////////////////////////////////////////////////////////// *}

        TROM_NEW                = 1;         {* Create object *}
        TROM_INSTALL            = 2;         {* Tell object to install itself in the
window *}
        TROM_REFRESH            = 4;         {* Refresh object *}
        TROM_REMOVE             = 6;         {* Remove object from window *}
        TROM_DISPOSE            = 7;         {* Dispose an object's private data *}
        TROM_GETATTRIBUTE       = 8;         {* Get an object's attribute *}
        TROM_SETATTRIBUTE       = 9;         {* Set an object's attribute *}
        TROM_EVENT              = 10;        {* IDCMP event *}
        TROM_DISABLED           = 11;        {* Disabled object *}
        TROM_ENABLED            = 12;        {* Enabled object *}
        TROM_KEYDOWN            = 13;        {* Key pressed *}
        TROM_REPEATEDKEYDOWN    = 14;        {* Key pressed repeatedly *}
        TROM_KEYUP              = 15;        {* Key released *}
        TROM_KEYCANCELLED       = 16;        {* Key cancelled *}
        TROM_CREATECLASS        = 17;        {* Create class-specific data *}
        TROM_DISPOSECLASS       = 18;        {* Dispose class-specific data *}
        TROM_HIT                = 22;        {* Find an object for a coordinate pair
*}
        TROM_ACTIVATE           = 23;        {* Activate an object *}


        TROM_EVENT_SWALLOWED    = 1;
        TROM_EVENT_CONTINUE     = 0;


{* ////////////////////////////////////////////////////////////////////// *}
{* ///////////////////////////////////////// Tags for TR_OpenProject() // *}
{* ////////////////////////////////////////////////////////////////////// *}

{* Tag bases *}
        TRTG_OAT              = (TAG_USER+$400);  {* Object attribute *}
        TRTG_CLS              = (TAG_USER+$100);  {* Class ID $1 to $2FF *}
        TRTG_OAT2             = (TAG_USER+$80);   {* PRIVATE! *}
        TRTG_PAT              = (TAG_USER);        {* Project attribute *}


{* Window/Project *}
        TRWI_Title              = (TRTG_PAT+$01); {* STRPTR: The window title *}
        TRWI_Flags              = (TRTG_PAT+$02); {* See below for window flags *}
        TRWI_Underscore         = (TRTG_PAT+$03); {* BYTE *: The underscore for menu
and gadget shortcuts *}
        TRWI_Position           = (TRTG_PAT+$04); {* Window position,  see below *}
        TRWI_CustomScreen       = (TRTG_PAT+$05); {* STRUCT Screen * *}
        TRWI_PubScreen          = (TRTG_PAT+$06); {* STRUCT Screen *,  must have been
locked! *}
        TRWI_PubScreenName      = (TRTG_PAT+$07); {* ADDRESS,  Triton is doing the
locking *}
        TRWI_PropFontAttr       = (TRTG_PAT+$08); {* STRUCT TextAttr *: The
proportional font *}
        TRWI_FixedWidthFontAttr = (TRTG_PAT+$09); {* STRUCT TextAttr *: The fixed-
width font *}
        TRWI_Backfill           = (TRTG_PAT+$0A); {* The backfill type,  see below *}
        TRWI_ID                 = (TRTG_PAT+$0B); {* ULONG: The window ID *}
        TRWI_Dimensions         = (TRTG_PAT+$0C); {* STRUCT TR_Dimensions * *}
        TRWI_ScreenTitle        = (TRTG_PAT+$0D); {* STRPTR: The screen title *}
        TRWI_QuickHelp          = (TRTG_PAT+$0E); {* BOOL: Quick help active? *}

{* Menus *}
        TRMN_Title              = (TRTG_PAT+$65); {* STRPTR: Menu *}
        TRMN_Item               = (TRTG_PAT+$66); {* STRPTR: Menu item *}
        TRMN_Sub                = (TRTG_PAT+$67); {* STRPTR: Menu subitem *}
        TRMN_Flags              = (TRTG_PAT+$68); {* See below for flags *}

{* General object attributes *}
        TRAT_ID               = (TRTG_OAT2+$16);  {* The object's/menu's ID *}
        TRAT_Flags            = (TRTG_OAT2+$17);  {* The object's flags *}
        TRAT_Value            = (TRTG_OAT2+$18);  {* The object's value *}
        TRAT_Text             = (TRTG_OAT2+$19);  {* The object's text *}
        TRAT_Disabled         = (TRTG_OAT2+$1A);  {* Disabled object? *}
        TRAT_Backfill         = (TRTG_OAT2+$1B);  {* Backfill pattern *}
        TRAT_MinWidth         = (TRTG_OAT2+$1C);  {* Minimum width *}
        TRAT_MinHeight        = (TRTG_OAT2+$1D);  {* Minimum height *}


{* ////////////////////////////////////////////////////////////////////// *}
{* ////////////////////////////////////////////////////// Window flags // *}
{* ////////////////////////////////////////////////////////////////////// *}

        TRWF_BACKDROP           = $00000001;     {* Create a backdrop borderless
window *}
        TRWF_NODRAGBAR          = $00000002;     {* Don't use a dragbar *}
        TRWF_NODEPTHGADGET      = $00000004;     {* Don't use a depth-gadget *}
        TRWF_NOCLOSEGADGET      = $00000008;     {* Don't use a close-gadget *}
        TRWF_NOACTIVATE         = $00000010;     {* Don't activate window *}
        TRWF_NOESCCLOSE         = $00000020;     {* Don't send TRMS_CLOSEWINDOW when
Esc is pressed *}
        TRWF_NOPSCRFALLBACK     = $00000040;     {* Don't fall back onto default
PubScreen *}
        TRWF_NOZIPGADGET        = $00000080;     {* Don't use a zip-gadget *}
        TRWF_ZIPCENTERTOP       = $00000100;     {* Center the zipped window on the
title bar *}
        TRWF_NOMINTEXTWIDTH     = $00000200;     {* Minimum window width not according
to title text *}
        TRWF_NOSIZEGADGET       = $00000400;     {* Don't use a sizing-gadget *}
        TRWF_NOFONTFALLBACK     = $00000800;     {* Don't fall back to topaz.8 *}
        TRWF_NODELZIP           = $00001000;     {* Don't zip the window when Del is
pressed *}
        TRWF_SIMPLEREFRESH      = $00002000;     {* *** OBSOLETE *** (V3+) *}
        TRWF_ZIPTOCURRENTPOS    = $00004000;     {* Will zip the window at the current
position (OS3.0+) *}
        TRWF_APPWINDOW          = $00008000;     {* Create an AppWindow without using
class_dropbox *}
        TRWF_ACTIVATESTRGAD     = $00010000;     {* Activate the first string gadget
after opening the window *}
        TRWF_HELP               = $00020000;     {* Pressing <Help> will create a
TRMS_HELP message (V4) *}
        TRWF_SYSTEMACTION       = $00040000;     {* System status messages will be
sent (V4) *}


{* ////////////////////////////////////////////////////////////////////// *}
{* //////////////////////////////////////////////////////// Menu flags // *}
{* ////////////////////////////////////////////////////////////////////// *}

        TRMF_CHECKIT            = $00000001;     {* Leave space for a checkmark *}
        TRMF_CHECKED            = $00000002;     {* Check the item (includes
TRMF_CHECKIT) *}
        TRMF_DISABLED           = $00000004;     {* Ghost the menu/item *}


{* ////////////////////////////////////////////////////////////////////// *}
{* ////////////////////////////////////////////////// Window positions // *}
{* ////////////////////////////////////////////////////////////////////// *}

        TRWP_DEFAULT            = 0;              {* Let Triton choose a good position
*}
        TRWP_BELOWTITLEBAR      = 1;              {* Left side of screen,  below title
bar *}
        TRWP_CENTERTOP          = 1025;           {* Top of screen,  centered on the
title bar *}
        TRWP_TOPLEFTSCREEN      = 1026;           {* Top left corner of screen *}
        TRWP_CENTERSCREEN       = 1027;           {* Centered on the screen *}
        TRWP_CENTERDISPLAY      = 1028;           {* Centered on the currently
displayed clip *}
        TRWP_MOUSEPOINTER       = 1029;           {* Under the mouse pointer *}
        TRWP_ABOVECOORDS        = 2049;           {* Above coordinates from the
dimensions STRUCT *}
        TRWP_BELOWCOORDS        = 2050;           {* Below coordinates from the
dimensions STRUCT *}


{* ////////////////////////////////////////////////////////////////////// *}
{* //////////////////////////////////// Backfill types / System images // *}
{* ////////////////////////////////////////////////////////////////////// *}

        TRBF_WINDOWBACK         = $00000000;     {* Window backfill *}
        TRBF_REQUESTERBACK      = $00000001;     {* Requester backfill *}

        TRBF_NONE               = $00000002;     {* No backfill (= Fill with
BACKGROUNDPEN) *}
        TRBF_SHINE              = $00000003;     {* Fill with SHINEPEN *}
        TRBF_SHINE_SHADOW       = $00000004;     {* Fill with SHINEPEN + SHADOWPEN *}
        TRBF_SHINE_FILL         = $00000005;     {* Fill with SHINEPEN + FILLPEN *}
        TRBF_SHINE_BACKGROUND   = $00000006;     {* Fill with SHINEPEN + BACKGROUNDPEN
*}
        TRBF_SHADOW             = $00000007;     {* Fill with SHADOWPEN *}
        TRBF_SHADOW_FILL        = $00000008;     {* Fill with SHADOWPEN + FILLPEN *}
        TRBF_SHADOW_BACKGROUND  = $00000009;     {* Fill with SHADOWPEN +
BACKGROUNDPEN *}
        TRBF_FILL               = $0000000A;     {* Fill with FILLPEN *}
        TRBF_FILL_BACKGROUND    = $0000000B;     {* Fill with FILLPEN + BACKGROUNDPEN
*}

        TRSI_USBUTTONBACK       = $00010002;     {* Unselected button backfill *}
        TRSI_SBUTTONBACK        = $00010003;     {* Selected button backfill *}



{* ////////////////////////////////////////////////////////////////////// *}
{* /////////////////////////////////////////////////////// Frame types // *}
{* ////////////////////////////////////////////////////////////////////// *}

        { * Copies of the GadTools BBFT_#? types *}
        TRFT_BUTTON       = 1;
        TRFT_RIDGE        = 2;
        TRFT_ICONDROPBOX  = 3;
        { * Triton's low-level frame types *}
        TRFT_XENBUTTON1   = 33;
        TRFT_XENBUTTON2   = 34;
        TRFT_NEXTBUTTON   = 35;
        { * Triton's abstract frame types *}
        TRFT_ABSTRACT_BUTTON      = 65;
        TRFT_ABSTRACT_ICONDROPBOX = 66;
        TRFT_ABSTRACT_FRAMEBOX    = 67;
        TRFT_ABSTRACT_PROGRESS    = 68;
        TRFT_ABSTRACT_GROUPBOX    = 69;


{* ////////////////////////////////////////////////////////////////////// *}
{* ///////////////////////////////////////////////////////// Pen types // *}
{* ////////////////////////////////////////////////////////////////////// *}

        TRPT_SYSTEMPEN   = 0;
        TRPT_GRAPHICSPEN = 1;
        TRPT_TRITONPEN   = 128;

        TRTP_NORMUSCORE         = 0;
        TRTP_HIGHUSCORE         = 1;
        TRTP_HALFSHINE          = 2;
        TRTP_HALFSHADOW         = 3;
        TRTP_USSTRINGGADBACK    = 4;
        TRTP_SSTRINGGADBACK     = 5;
        TRTP_USSTRINGGADFRONT   = 6;
        TRTP_SSTRINGGADFRONT    = 7;

{* ////////////////////////////////////////////////////////////////////// *}
{* ////////////////////////////////////////////// Display Object flags // *}
{* ////////////////////////////////////////////////////////////////////// *}

{* General flags *}
        TROF_RAISED             = $00000001;     {* Raised object *}
        TROF_HORIZ              = $00000002;     {* Horizontal object \ Works
automatically *}
        TROF_VERT               = $00000004;     {* Vertical object   / in groups *}
        TROF_RIGHTALIGN         = $00000008;     {* Align object to the right border
if available *}
        TROF_GENERAL_MASK       = $000000FF;     {* PRIVATE *}

{* Text flags for different kinds of text-related objects *}
        TRTX_NOUNDERSCORE       = $00000100;     {* Don't interpret underscores *}
        TRTX_HIGHLIGHT          = $00000200;     {* Highlight text *}
        TRTX_3D                 = $00000400;     {* 3D design *}
        TRTX_BOLD               = $00000800;     {* Softstyle 'bold' *}
        TRTX_TITLE              = $00001000;     {* A title (e.g. of a group) *}
        TRTX_MULTILINE          = $00002000;     {* A multi-line text. See
TR_PrintText() autodoc clip *}
        TRTX_RIGHTALIGN         = TROF_RIGHTALIGN;
        TRTX_CENTER             = $00004000;     {* Center text *}
        TRTX_SELECTED           = $00002000;     {* PRIVATE! *}

{* ////////////////////////////////////////////////////////////////////// *}
{* ////////////////////////////////////////////////////// Menu entries // *}
{* ////////////////////////////////////////////////////////////////////// *}

        TRMN_BARLABEL           = (-1);           {* A barlabel instead of text *}


{* ////////////////////////////////////////////////////////////////////// *}
{* /////////////////////////////////////////// Tags for TR_CreateApp() // *}
{* ////////////////////////////////////////////////////////////////////// *}

        TRCA_Name               = (TAG_USER+1);
        TRCA_LongName           = (TAG_USER+2);
        TRCA_Info               = (TAG_USER+3);
        TRCA_Version            = (TAG_USER+4);
        TRCA_Release            = (TAG_USER+5);
        TRCA_Date               = (TAG_USER+6);


{* ////////////////////////////////////////////////////////////////////// *}
{* ///////////////////////////////////////// Tags for TR_EasyRequest() // *}
{* ////////////////////////////////////////////////////////////////////// *}

        TREZ_ReqPos             = (TAG_USER+1);
        TREZ_LockProject        = (TAG_USER+2);
        TREZ_Return             = (TAG_USER+3);
        TREZ_Title              = (TAG_USER+4);
        TREZ_Activate           = (TAG_USER+5);

{* class_DisplayObject *}

        TROB_DisplayObject      = (TRTG_CLS+$3C); {* A basic display object *}

        TRDO_QuickHelpString    = (TRTG_OAT+$1E3);

{* Flags *}
        TROB_DISPLAYOBJECT_DISABLED    = $00100000; {* Disabled? *}
        TROB_DISPLAYOBJECT_RETURNOK    = $00200000; {* Activate with <Return> *}
        TROB_DISPLAYOBJECT_ESCOK       = $00400000; {* Activate with <Esc> *}
        TROB_DISPLAYOBJECT_TABOK       = $00800000; {* Activate with <Tab> *}
        TROB_DISPLAYOBJECT_SPACE       = $01000000; {* A spacing object? *}

{* class_DragItem *}

         TROB_DragItem          = (TRTG_CLS+$3E); {* A draggable item *}

{* class_Image *}

        TROB_Image              = (TRTG_CLS+$3B); {* An image *}

        TRIM_BOOPSI             = $00010000;     {* Use a BOOPSI IClass image *}

{* class_String *}

        TROB_String             = (TRTG_CLS+$37); {* A string gadget *}
        TRST_Filter             = (TRTG_OAT+$1E4);

        TRST_INVISIBLE          = $00010000;     {* A password gadget -> invisible
typing *}
        TRST_NORETURNBROADCAST  = $00020000;     {* <Return> keys will not be
broadcast to the window *}
        TRST_FLOAT              = $00040000;     {* Separators "." and "," will be
accepted only once *}

{* class_Cycle *}

        TROB_Cycle              = (TRTG_CLS+$36); {* A cycle gadget *}

        TRCY_MX                 = $00010000;     {* Unfold the cycle gadget to a MX
gadget *}
        TRCY_RIGHTLABELS        = $00020000;     {* Put the labels to the right of a
MX gadget *}

{* class_Palette *}

        TROB_Palette            = (TRTG_CLS+$33); {* A palette gadget *}

{* class_DropBox *}

        TROB_DropBox            = (TRTG_CLS+$38); {* An icon drop box *}

{* class_Group *}

        TRGR_Horiz              = (TAG_USER+201);  {* Horizontal group *}
        TRGR_Vert               = (TAG_USER+202);  {* Vertical group *}
        TRGR_End                = (TRTG_OAT2+$4B); {* End of a group *}

        TRGR_PROPSHARE          = $00000000;     {* Default: Divide objects
proportionally *}
        TRGR_EQUALSHARE         = $00000001;     {* Divide objects equally *}
        TRGR_PROPSPACES         = $00000002;     {* Divide spaces proportionally *}
        TRGR_ARRAY              = $00000004;     {* Top-level array group *}

        TRGR_ALIGN              = $00000008;     {* Align resizeable objects in
secondary dimension *}
        TRGR_CENTER             = $00000010;     {* Center unresizeable objects in
secondary dimension *}

        TRGR_FIXHORIZ           = $00000020;     {* Don't allow horizontal resizing *}
        TRGR_FIXVERT            = $00000040;     {* Don't allow vertical resizing *}
        TRGR_INDEP              = $00000080;     {* Group is independant of
surrounding array *}

{* class_Line *}

        TROB_Line               = (TRTG_CLS+$2D); {* A simple line *}

{* class_Slider *}

        TROB_Slider             = (TRTG_CLS+$34); {* A slider gadget *}

        TRSL_Min                = (TRTG_OAT+$1DE);
        TRSL_Max                = (TRTG_OAT+$1DF);

{* class_Listview *}

        TROB_Listview           = (TRTG_CLS+$39); {* A listview gadget *}

        TRLV_Top                = (TRTG_OAT+$1E2);
        TRLV_VisibleLines       = (TRTG_OAT+$1E4);

        TRLV_READONLY           = $00010000;     {* A read-only list *}
        TRLV_SELECT             = $00020000;     {* You may select an entry *}
        TRLV_SHOWSELECTED       = $00040000;     {* Selected entry will be shown *}
        TRLV_NOCURSORKEYS       = $00080000;     {* Don't use arrow keys *}
        TRLV_NONUMPADKEYS       = $00100000;     {* Don't use numeric keypad keys *}
        TRLV_FWFONT             = $00200000;     {* Use the fixed-width font *}
        TRLV_NOGAP              = $00400000;     {* Don't leave a gap below the list
*}

{* class_Progress *}

        TROB_Progress           = (TRTG_CLS+$3A); {* A progress indicator *}

{* class_Space *}

        TROB_Space              = (TRTG_CLS+$285); {* The spaces class *}

        TRST_NONE               = 1;              {* No space *}
        TRST_SMALL              = 2;              {* Small space *}
        TRST_NORMAL             = 3;              {* Normal space (default) *}
        TRST_BIG                = 4;              {* Big space *}

{* class_Text *}

        TROB_Text               = (TRTG_CLS+$30); {* A line of text *}

        TRTX_CLIPPED            = $00010000;     {* Text is clipped *}

{* class_Button *}

        TROB_Button             = (TRTG_CLS+$31); {* A BOOPSI button gadget *}

        TRBU_RETURNOK           = $00010000;     {* <Return> answers the button *}
        TRBU_ESCOK              = $00020000;     {* <Esc> answers the button *}
        TRBU_SHIFTED            = $00040000;     {* Shifted shortcut only *}
        TRBU_UNSHIFTED          = $00080000;     {* Unshifted shortcut only *}
        TRBU_YRESIZE            = $00100000;     {* Button resizeable in Y direction
*}
        TRBT_TEXT               = 0;              {* Text button *}
        TRBT_GETFILE            = 1;              {* GetFile button *}
        TRBT_GETDRAWER          = 2;              {* GetDrawer button *}
        TRBT_GETENTRY           = 3;              {* GetEntry button *}

{* class_CheckBox *}

        TROB_CheckBox           = (TRTG_CLS+$2F); {* A checkbox gadget *}

{* class_Object *}

        TROB_Object             = (TRTG_CLS+$3D); {* A rootclass object *}

{* class_Scroller *}

        TROB_Scroller           = (TRTG_CLS+$35); {* A scroller gadget *}

        TRSC_Total              = (TRTG_OAT+$1E0);
        TRSC_Visible            = (TRTG_OAT+$1E1);

{* class_FrameBox *}

        TROB_FrameBox           = (TRTG_CLS+$32); {* A framing box *}

        TRFB_GROUPING           = $00000001;     {* A grouping box *}
        TRFB_FRAMING            = $00000002;     {* A framing box *}
        TRFB_TEXT               = $00000004;     {* A text container *}


VAR TritonBase : pLibrary;

FUNCTION TR_AddClass(app : pTR_App; d0arg : longword; supertag : longword; defaultmethod : LONGINT;
datasize : longword; tags : pTagItem) : BOOLEAN;
PROCEDURE TR_AreaFill(project : pTR_Project; rp : pRastPort; left : ULONG; top :
ULONG; right : ULONG; bottom : ULONG; typ : ULONG; dummy : POINTER);
FUNCTION TR_AutoRequest(app : pTR_App; lockproject : pTR_Project; wintags : pTagItem)
: ULONG;
PROCEDURE TR_CloseProject(project : pTR_Project);
PROCEDURE TR_CloseWindowSafely(window : pWindow);
FUNCTION TR_CreateApp(apptags : pTagItem) : pTR_App;
FUNCTION TR_CreateMsg(app : pTR_App) : pTR_Message;
PROCEDURE TR_DeleteApp(app : pTR_App);
FUNCTION TR_DoMethod(obj : pTROD_Object; messageid : ULONG; data : POINTER) : ULONG;
FUNCTION TR_DoMethodClass(obj : pTROD_Object; messageid : ULONG; data : POINTER;
trclass : pTR_Class) : ULONG;
PROCEDURE TR_DrawFrame(project : pTR_Project; rp : pRastPort; left : WORD; top : WORD;
width : WORD; height : WORD; typ : WORD; inverted : BOOLEAN);
FUNCTION TR_EasyRequest(app : pTR_App; bodyfmt : pCHAR; gadfmt : pCHAR; taglist :
pTagItem) : ULONG;
FUNCTION TR_EasyRequest(app : pTR_App; bodyfmt : pCHAR; gadfmt : String; taglist :
pTagItem) : ULONG;
FUNCTION TR_EasyRequest(app : pTR_App; bodyfmt : String; gadfmt : pCHAR; taglist :
pTagItem) : ULONG;
FUNCTION TR_EasyRequest(app : pTR_App; bodyfmt : String; gadfmt : String; taglist :
pTagItem) : ULONG;
FUNCTION TR_FirstOccurance(ch : BYTE; str : pCHAR) : LONGINT;
FUNCTION TR_FirstOccurance(ch : BYTE; str : String) : LONGINT;
FUNCTION TR_FrameBorderHeight(project : pTR_Project; typ : WORD) : ULONG;
FUNCTION TR_FrameBorderWidth(project : pTR_Project; typ : WORD) : ULONG;
FUNCTION TR_GetAttribute(project : pTR_Project; ID : ULONG; attribute : ULONG) :
ULONG;
FUNCTION TR_GetErrorString(num : WORD) : pCHAR;
FUNCTION TR_GetLastError(app : pTR_App) : WORD;
FUNCTION TR_GetMsg(app : pTR_App) : pTR_Message;
FUNCTION TR_GetPen(project : pTR_Project; pentype : ULONG; pendata : ULONG) : ULONG;
PROCEDURE TR_LockProject(project : pTR_Project);
FUNCTION TR_LockScreen(project : pTR_Project) : pScreen;
FUNCTION TR_NumOccurances(ch : BYTE; str : pCHAR) : LONGINT;
FUNCTION TR_NumOccurances(ch : BYTE; str : String) : LONGINT;
FUNCTION TR_ObtainWindow(project : pTR_Project) : pWindow;
FUNCTION TR_OpenProject(app : pTR_App; taglist : pTagItem) : pTR_Project;
PROCEDURE TR_PrintText(project : pTR_Project; rp : pRastPort; txt : pCHAR; x : ULONG;
y : ULONG; width : ULONG; flags : ULONG);
PROCEDURE TR_PrintText(project : pTR_Project; rp : pRastPort; txt : String; x : ULONG;
y : ULONG; width : ULONG; flags : ULONG);
PROCEDURE TR_ReleaseWindow(window : pWindow);
PROCEDURE TR_ReplyMsg(message : pTR_Message);
FUNCTION TR_SendMessage(project : pTR_Project; objectid : ULONG; messageid : ULONG;
messagedata : POINTER) : ULONG;
PROCEDURE TR_SetAttribute(project : pTR_Project; ID : ULONG; attribute : ULONG; value
: ULONG);
FUNCTION TR_TextHeight(project : pTR_Project; txt : pCHAR; flags : ULONG) : ULONG;
FUNCTION TR_TextHeight(project : pTR_Project; txt : String; flags : ULONG) : ULONG;
FUNCTION TR_TextWidth(project : pTR_Project; txt : pCHAR; flags : ULONG) : ULONG;
FUNCTION TR_TextWidth(project : pTR_Project; txt : String; flags : ULONG) : ULONG;
PROCEDURE TR_UnlockProject(project : pTR_Project);
PROCEDURE TR_UnlockScreen(screen : pScreen);
FUNCTION TR_Wait(app : pTR_App; otherbits : ULONG) : ULONG;

{
   Functions with array of const
}
FUNCTION TR_AddClassTags(app : pTR_App; d0arg : longword; supertag : longword;
defaultmethod : LONGINT; datasize : longword; const tags : Array Of Const) : BOOLEAN;
FUNCTION TR_OpenProjectTags(app : pTR_App; const taglist : Array Of Const) : pTR_Project;
FUNCTION TR_AutoRequestTags(app : pTR_App; lockproject : pTR_Project; const wintags : Array Of Const): ULONG;
FUNCTION TR_CreateAppTags(const apptags : Array of Const) : pTR_App;
FUNCTION TR_EasyRequestTags(app : pTR_App; bodyfmt : pCHAR; gadfmt : pCHAR; const taglist : Array Of Const) : ULONG;
FUNCTION TR_EasyRequestTags(app : pTR_App; bodyfmt : pCHAR; gadfmt : String; Const taglist : Array Of Const) : ULONG;
FUNCTION TR_EasyRequestTags(app : pTR_App; bodyfmt : String; gadfmt : pCHAR; Const taglist : Array Of Const) : ULONG;
FUNCTION TR_EasyRequestTags(app : pTR_App; bodyfmt : String; gadfmt : String; Const taglist : Array Of Const) : ULONG;

{  This are a few support functions for triton.
   Could be handy.
}

procedure TR_Disable(p : pTR_Project; id : Longint);
procedure TR_Enable(p : pTR_Project; id : Longint);
function TR_GetCheckBox(p : pTR_Project; id : Longint): boolean;
function TR_GetSTRPTR(p : pTR_Project; id : Longint): PChar;
function TR_GetString(p : pTR_Project; id : Longint): String;
function TR_GetValue(p : pTR_Project; gadid : Longint): Longint;
procedure TR_SetCheckBox(p : pTR_Project; id : Longint; onoff : boolean);
procedure TR_SetString(p : pTR_Project; id : Longint; txt : String);
procedure TR_SetString(p : pTR_Project; id : Longint; txt : PChar);
procedure TR_SetText(p : pTR_Project; id : Longint; txt : string);
procedure TR_SetText(p : pTR_Project; id : Longint; txt : PChar);
procedure TR_SetValue(p : pTR_Project; id : Longint; value : Longint);
procedure TR_SetWindowTitle(p : pTR_Project; thetitle : string);
procedure TR_SetWindowTitle(p : pTR_Project; thetitle : PChar);
procedure TR_UpdateListView(p : pTR_Project; gadid : Longint; thelist: pList);

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitTRITONLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    TRITONIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray,pastoc;

procedure TR_Disable(p : pTR_Project; id : Longint);
begin
    TR_SetAttribute(p,id, TRAT_Disabled,0);
end;

procedure TR_Enable(p : pTR_Project; id : Longint);
begin
    TR_SetAttribute(p,id,TRAT_Disabled,1);
end;

function TR_GetCheckBox(p : pTR_Project; id : Longint): boolean;
var
    temp : Longint;
begin
    temp := TR_GetAttribute(p,id,TRAT_Value);
    if temp = 0 then TR_GetCheckBox := false
    else TR_GetCheckBox := true;
end;

function TR_GetSTRPTR(p : pTR_Project; id : Longint): PChar;
var
    temp : Longint;
begin
    temp := TR_GetAttribute(p,id,0);
    TR_GetSTRPTR := PChar(Pointer(temp));
end;

function TR_GetString(p : pTR_Project; id : Longint): String;
var
    temp : Longint;
begin
    temp := TR_GetAttribute(p,id,0);
    TR_GetString := strpas(PChar(Pointer(temp)));
end;

function TR_GetValue(p : pTR_Project; gadid : Longint): Longint;
begin
   TR_GetValue := TR_GetAttribute(p,gadid,TRAT_Value);
end;

procedure TR_SetCheckBox(p : pTR_Project; id : Longint; onoff : boolean);
begin
    TR_SetAttribute(p,id,TRAT_Value,Longint(byte(onoff)));
end;

procedure TR_SetString(p : pTR_Project; id : Longint; txt : String);
begin
    TR_SetAttribute(p,id,0,Longint(pas2c(txt)));
end;

procedure TR_SetString(p : pTR_Project; id : Longint; txt : PChar);
begin
    TR_SetAttribute(p,id,0,Longint(txt));
end;

procedure TR_SetText(p : pTR_Project; id : Longint; txt : string);
begin
    TR_SetAttribute(p,id,TRAT_Text,Longint(pas2c(txt)));
end;

procedure TR_SetText(p : pTR_Project; id : Longint; txt : PChar);
begin
    TR_SetAttribute(p,id,TRAT_Text,Longint(txt));
end;

procedure TR_SetValue(p : pTR_Project; id : Longint; value : Longint);
begin
    TR_SetAttribute(p,id,TRAT_Value,value);
end;

procedure TR_SetWindowTitle(p : pTR_Project; thetitle : string);
begin
    TR_SetAttribute(p,0,TRWI_Title,Longint(pas2c(thetitle)));
end;

procedure TR_SetWindowTitle(p : pTR_Project; thetitle : PChar);
begin
    TR_SetAttribute(p,0,TRWI_Title,Longint(thetitle));
end;

procedure TR_UpdateListView(p : pTR_Project; gadid : Longint; thelist: pList);
begin
    TR_SetAttribute(p,gadid,0,Longint(thelist));
end;

FUNCTION TR_AddClass(app : pTR_App; d0arg : longword; supertag : longword; defaultmethod : LONGINT; datasize : longword; tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L app,A1
        MOVE.L  d0arg,D0
        MOVE.L  supertag,D1
        MOVEA.L defaultmethod,A2
        MOVE.L  datasize,D2
        MOVEA.L tags,A0
        MOVEA.L TritonBase,A6
        JSR     -168(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;


PROCEDURE TR_AreaFill(project : pTR_Project; rp : pRastPort; left : ULONG; top :
ULONG; right : ULONG; bottom : ULONG; typ : ULONG; dummy : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVEA.L rp,A1
    MOVE.L  left,D0
    MOVE.L  top,D1
    MOVE.L  right,D2
    MOVE.L  bottom,D3
    MOVE.L  typ,D4
    MOVEA.L dummy,A2
    MOVEA.L TritonBase,A6
    JSR -228(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION TR_AutoRequest(app : pTR_App; lockproject : pTR_Project; wintags : pTagItem)
: ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L app,A1
    MOVEA.L lockproject,A0
    MOVEA.L wintags,A2
    MOVEA.L TritonBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE TR_CloseProject(project : pTR_Project);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVEA.L TritonBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE TR_CloseWindowSafely(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L TritonBase,A6
    JSR -126(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION TR_CreateApp(apptags : pTagItem) : pTR_App;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L apptags,A1
    MOVEA.L TritonBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_CreateMsg(app : pTR_App) : pTR_Message;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L app,A1
    MOVEA.L TritonBase,A6
    JSR -234(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE TR_DeleteApp(app : pTR_App);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L app,A1
    MOVEA.L TritonBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION TR_DoMethod(obj : pTROD_Object; messageid : ULONG; data : POINTER) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L obj,A0
    MOVE.L  messageid,D0
    MOVEA.L data,A1
    MOVEA.L TritonBase,A6
    JSR -216(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_DoMethodClass(obj : pTROD_Object; messageid : ULONG; data : POINTER;
trclass : pTR_Class) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L obj,A0
    MOVE.L  messageid,D0
    MOVEA.L data,A1
    MOVEA.L trclass,A2
    MOVEA.L TritonBase,A6
    JSR -222(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE TR_DrawFrame(project : pTR_Project; rp : pRastPort; left : WORD; top : WORD;
width : WORD; height : WORD; typ : WORD; inverted : BOOLEAN);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVEA.L rp,A1
    MOVE.L  left,D1
    MOVE.L  top,D2
    MOVE.L  width,D3
    MOVE.L  height,D4
    MOVE.L  typ,D0
    MOVE.L  inverted,D5
    MOVEA.L TritonBase,A6
    JSR -174(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION TR_EasyRequest(app : pTR_App; bodyfmt : pCHAR; gadfmt : pCHAR; taglist :
pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L app,A1
    MOVEA.L bodyfmt,A2
    MOVEA.L gadfmt,A3
    MOVEA.L taglist,A0
    MOVEA.L TritonBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_EasyRequest(app : pTR_App; bodyfmt : PChar; gadfmt : String; taglist :
pTagItem) : ULONG;
begin
    TR_EasyRequest := TR_EasyRequest(app,bodyfmt,pas2c(gadfmt),taglist);
end;

FUNCTION TR_EasyRequest(app : pTR_App; bodyfmt : String; gadfmt : PChar; taglist :
pTagItem) : ULONG;
begin
    TR_EasyRequest := TR_EasyRequest(app,pas2c(bodyfmt),gadfmt,taglist);
end;

FUNCTION TR_EasyRequest(app : pTR_App; bodyfmt : String; gadfmt : String; taglist :
pTagItem) : ULONG;
begin
    TR_EasyRequest := TR_EasyRequest(app,pas2c(bodyfmt),pas2c(gadfmt),taglist);
end;

FUNCTION TR_FirstOccurance(ch : BYTE; str : pCHAR) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  ch,D0
    MOVEA.L str,A0
    MOVEA.L TritonBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_FirstOccurance(ch : BYTE; str : String) : LONGINT;
BEGIN
    TR_FirstOccurance := TR_FirstOccurance(ch, pas2c(str));
END;

FUNCTION TR_FrameBorderHeight(project : pTR_Project; typ : WORD) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVE.L  typ,D0
    MOVEA.L TritonBase,A6
    JSR -186(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_FrameBorderWidth(project : pTR_Project; typ : WORD) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVE.L  typ,D0
    MOVEA.L TritonBase,A6
    JSR -180(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_GetAttribute(project : pTR_Project; ID : ULONG; attribute : ULONG) :
ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVE.L  ID,D0
    MOVE.L  attribute,D1
    MOVEA.L TritonBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_GetErrorString(num : WORD) : pCHAR;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  num,D0
    MOVEA.L TritonBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_GetLastError(app : pTR_App) : WORD;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L app,A1
    MOVEA.L TritonBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_GetMsg(app : pTR_App) : pTR_Message;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L app,A1
    MOVEA.L TritonBase,A6
    JSR -108(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_GetPen(project : pTR_Project; pentype : ULONG; pendata : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVE.L  pentype,D0
    MOVE.L  pendata,D1
    MOVEA.L TritonBase,A6
    JSR -210(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE TR_LockProject(project : pTR_Project);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVEA.L TritonBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION TR_LockScreen(project : pTR_Project) : pScreen;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVEA.L TritonBase,A6
    JSR -138(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_NumOccurances(ch : BYTE; str : pCHAR) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  ch,D0
    MOVEA.L str,A0
    MOVEA.L TritonBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_NumOccurances(ch : BYTE; str : String) : LONGINT;
BEGIN
    TR_NumOccurances := TR_NumOccurances(ch, pas2c(str));
END;

FUNCTION TR_ObtainWindow(project : pTR_Project) : pWindow;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVEA.L TritonBase,A6
    JSR -150(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_OpenProject(app : pTR_App; taglist : pTagItem) : pTR_Project;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L app,A1
    MOVEA.L taglist,A0
    MOVEA.L TritonBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE TR_PrintText(project : pTR_Project; rp : pRastPort; txt : pCHAR; x : ULONG;
y : ULONG; width : ULONG; flags : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVEA.L rp,A1
    MOVEA.L txt,A2
    MOVE.L  x,D1
    MOVE.L  y,D2
    MOVE.L  width,D3
    MOVE.L  flags,D0
    MOVEA.L TritonBase,A6
    JSR -204(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE TR_PrintText(project : pTR_Project; rp : pRastPort; txt : String; x : ULONG;
y : ULONG; width : ULONG; flags : ULONG);
BEGIN
    TR_PrintText(project,rp,pas2c(txt),x,y,width,flags);
END;

PROCEDURE TR_ReleaseWindow(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L TritonBase,A6
    JSR -156(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE TR_ReplyMsg(message : pTR_Message);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L message,A1
    MOVEA.L TritonBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION TR_SendMessage(project : pTR_Project; objectid : ULONG; messageid : ULONG;
messagedata : POINTER) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVE.L  objectid,D0
    MOVE.L  messageid,D1
    MOVEA.L messagedata,A1
    MOVEA.L TritonBase,A6
    JSR -162(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE TR_SetAttribute(project : pTR_Project; ID : ULONG; attribute : ULONG; value
: ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVE.L  ID,D0
    MOVE.L  attribute,D1
    MOVE.L  value,D2
    MOVEA.L TritonBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION TR_TextHeight(project : pTR_Project; txt : pCHAR; flags : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVEA.L txt,A2
    MOVE.L  flags,D0
    MOVEA.L TritonBase,A6
    JSR -198(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_TextHeight(project : pTR_Project; txt : String; flags : ULONG) : ULONG;
BEGIN
    TR_TextHeight :=  TR_TextHeight(project,pas2c(txt),flags);
END;

FUNCTION TR_TextWidth(project : pTR_Project; txt : pCHAR; flags : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVEA.L txt,A2
    MOVE.L  flags,D0
    MOVEA.L TritonBase,A6
    JSR -192(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TR_TextWidth(project : pTR_Project; txt : String; flags : ULONG) : ULONG;
BEGIN
    TR_TextWidth := TR_TextWidth(project,pas2c(txt),flags);
END;

PROCEDURE TR_UnlockProject(project : pTR_Project);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L project,A0
    MOVEA.L TritonBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE TR_UnlockScreen(screen : pScreen);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L TritonBase,A6
    JSR -144(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION TR_Wait(app : pTR_App; otherbits : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L app,A1
    MOVE.L  otherbits,D0
    MOVEA.L TritonBase,A6
    JSR -120(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

{
   Functions with array of const
}
{
 Functions and procedures with array of const go here
}
FUNCTION TR_AddClassTags(app : pTR_App; d0arg : longword; supertag : longword; defaultmethod : LONGINT; datasize : longword; const tags : Array Of Const) : BOOLEAN;
begin
    TR_AddClassTags := TR_AddClass(app , d0arg , supertag , defaultmethod , datasize , readintags(tags));
end;

FUNCTION TR_EasyRequestTags(app : pTR_App; bodyfmt : pCHAR; gadfmt : pCHAR; const taglist : Array Of Const) : Ulong;
begin
    TR_EasyRequestTags := TR_EasyRequest(app , bodyfmt , gadfmt , readintags(taglist));
end;

FUNCTION TR_OpenProjectTags(app : pTR_App; const taglist : Array Of Const) : pTR_Project;
begin
    TR_OpenProjectTags := TR_OpenProject(app , readintags(taglist));
end;

FUNCTION TR_AutoRequestTags(app : pTR_App; lockproject : pTR_Project; const wintags : Array Of Const): ULONG;
begin
    TR_AutoRequestTags := TR_AutoRequest(app,lockproject,readintags(wintags));
end;

FUNCTION TR_CreateAppTags(const apptags : Array of Const) : pTR_App;
begin
    TR_CreateAppTags := TR_CreateApp(readintags(apptags));
end;

FUNCTION TR_EasyRequestTags(app : pTR_App; bodyfmt : pCHAR; gadfmt : String; Const taglist : Array Of Const) : ULONG;
begin
    TR_EasyRequestTags := TR_EasyRequest(app,bodyfmt,pas2c(gadfmt),readintags(taglist));
end;

FUNCTION TR_EasyRequestTags(app : pTR_App; bodyfmt : String; gadfmt : pCHAR; Const taglist : Array Of Const) : ULONG;
begin
    TR_EasyRequestTags := TR_EasyRequest(app,pas2c(bodyfmt),gadfmt,readintags(taglist));
end;

FUNCTION TR_EasyRequestTags(app : pTR_App; bodyfmt : String; gadfmt : String; Const taglist : Array Of Const) : ULONG;
begin
    TR_EasyRequestTags := TR_EasyRequest(app,pas2c(bodyfmt),pas2c(gadfmt),readintags(taglist));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of triton.library}
  {$Info don't forget to use InitTRITONLibrary in the beginning of your program}

var
    triton_exit : Pointer;

procedure ClosetritonLibrary;
begin
    ExitProc := triton_exit;
    if TritonBase <> nil then begin
        CloseLibrary(TritonBase);
        TritonBase := nil;
    end;
end;

procedure InitTRITONLibrary;
begin
    TritonBase := nil;
    TritonBase := OpenLibrary(TRITONNAME,LIBVERSION);
    if TritonBase <> nil then begin
        triton_exit := ExitProc;
        ExitProc := @ClosetritonLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open triton.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    TRITONIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of triton.library}

var
    triton_exit : Pointer;

procedure ClosetritonLibrary;
begin
    ExitProc := triton_exit;
    if TritonBase <> nil then begin
        CloseLibrary(TritonBase);
        TritonBase := nil;
    end;
end;

begin
    TritonBase := nil;
    TritonBase := OpenLibrary(TRITONNAME,LIBVERSION);
    if TritonBase <> nil then begin
        triton_exit := ExitProc;
        ExitProc := @ClosetritonLibrary;
        TRITONIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open triton.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    TRITONIsCompiledHow := 3;
   {$Warning No autoopening of triton.library compiled}
   {$Warning Make sure you open triton.library yourself}
{$endif dont_use_openlib}


END. (* UNIT TRITON *)






