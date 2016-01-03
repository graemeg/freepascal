{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006-2008 Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{  Declarations for commctrl WinCE API

}

{exported functions list = to do,
 * please remove functions done *
 
     Exports

       ordinal    name

            73    AddMRUData
            66    AddMRUStringA
            67    AddMRUStringW
            35    CenterWindow
            11    CommandBar_GetItemWindow
            63    CreateMRUListA
            64    CreateMRUListW
            15    CreateToolbar
            50    DPA_Create
            61    DPA_DeleteAllPtrs
            60    DPA_DeletePtr
            31    DPA_Destroy
            56    DPA_DestroyCallback
            32    DPA_GetPtr
            62    DPA_GetPtrIndex
            54    DPA_Grow
            51    DPA_InsertPtr
            59    DPA_Search
            53    DPA_SetPtr
            52    DPA_Sort
            46    DSA_Clone
            23    DSA_Create
            30    DSA_DeleteAllItems
            29    DSA_DeleteItem
            24    DSA_Destroy
            47    DSA_DestroyCallback
            48    DSA_EnumCallback
            25    DSA_GetItem
            26    DSA_GetItemPtr
            27    DSA_InsertItem
            45    DSA_Search
            28    DSA_SetItem
            58    DSA_SetRange
            57    DSA_Sort
            68    DelMRUString
            49    DoReaderMode
            71    EnumMRUListA
            72    EnumMRUListW
            74    FindMRUData
            69    FindMRUStringA
            70    FindMRUStringW
            65    FreeMRUList
            22    InvertRect
            55    IsCapEditAvailable
                  ListView_SetItemSpacing
            75    PopulateComboWithMRU
            76    PopulateMenuWithMRU
            80    SHCreateDefaultGradient
            79    SHDrawGradient
            81    SHDrawText
            77    SHGetSysColor
            78    SHGetSystemMetrics
            82    SHSetSysColors
            83    SHSetSystemMetrics
            34    StrToIntW
            33    Str_SetPtrW
}

unit commctrl;

interface

uses windows;

{$calling cdecl}

//*****************************************************************************
// consts
//*****************************************************************************
const
  ComctlDLL     = 'commctrl';

  COMCTL32_VERSION=$020c;

  // Common control shared messages
  CCM_FIRST = $2000;
  CCM_LAST  = CCM_FIRST + $200;

  CCM_SETBKCOLOR = CCM_FIRST + $1; // lParam is bkColor
  CCM_SETVERSION = CCM_FIRST + $7;
  CCM_GETVERSION = CCM_FIRST + $8;

  //InitCommonControlEx
  //I_IMAGENONE           = -2; // Desktop listview uses this same value for I_IMAGENONE when (_WIN32_IE >= 0x0501)

  //ICC_LISTVIEW_CLASSES  = $00000001; // listview, header
  //ICC_TREEVIEW_CLASSES  = $00000002; // treeview, tooltips
  //ICC_BAR_CLASSES       = $00000004; // toolbar, statusbar, trackbar, tooltips
  //ICC_TAB_CLASSES       = $00000008; // tab, tooltips
  //ICC_UPDOWN_CLASS      = $00000010; // updown
  //ICC_PROGRESS_CLASS    = $00000020; // progress
  ICC_ANIMATE_CLASS     = $00000080; // animate
  //ICC_WIN95_CLASSES     = $0000007F;
  //ICC_DATE_CLASSES      = $00000100; // month picker, date picker, time picker, updown
  //ICC_COOL_CLASSES      = $00000400; // rebar (coolbar) control
  //ICC_INTERNET_CLASSES  = $00000800; // IP Address control
  //ICC_TOOLTIP_CLASSES   = $00001000; // Tooltip static & button
  //ICC_CAPEDIT_CLASS     = $00002000; // All-caps edit control
  //ICC_FE_CLASSES        = $40000000; // FE specific input subclasses

  LVM_FIRST             =  $1000;      // ListView messages
  TV_FIRST              =  $1100;      // TreeView messages
  HDM_FIRST             =  $1200;      // Header messages

  //Ranges for control message IDs
  LVN_FIRST             =  (-100);       // listview
  LVN_LAST              =  (-199);
  HDN_FIRST             =  (-300);       // header
  HDN_LAST              =  (-399);
  TVN_FIRST             =  (-400);       // treeview
  TVN_LAST              =  (-499);
  TTN_FIRST             =  (-520);       // tooltips
  TTN_LAST              =  (-549);
  TCN_FIRST             =  (-550);       // tab control
  TCN_LAST              =  (-580);
  // Shell reserved               (0U-580U) -  (0U-589U)
  CDN_FIRST             =  (-601);        // common dialog (new)
  CDN_LAST              =  (-699);
  TBN_FIRST             =  (-700);        // toolbar
  TBN_LAST              =  (-720);
  UDN_FIRST             =  (-721);        // updown
  UDN_LAST              =  (-740);
  MCN_FIRST             =  (-750);        // monthcal
  MCN_LAST              =  (-759);
  RBN_FIRST             =  (-831);        // rebar
  RBN_LAST              =  (-859);
  IPN_FIRST             =  (-860);        // internet address
  IPN_LAST              =  (-879);        // internet address
  SBN_FIRST             =  (-880);        // status bar
  SBN_LAST              =  (-900);
  SHN_FIRST             =  (-1400);       // Shell reserved
  SHN_LAST              =  (-1500);

  MSGF_COMMCTRL_BEGINDRAG     = $4200;
  MSGF_COMMCTRL_SIZEHEADER    = $4201;
  MSGF_COMMCTRL_DRAGSELECT    = $4202;

  //Generic WM_NOTIFY notification codes
  NM_FIRST              =  0;
  NM_LAST               =  -99;
  //NM_OUTOFMEMORY        =  (NM_FIRST-1);
  //NM_CLICK              =  (NM_FIRST-2);
  //NM_DBLCLK             =  (NM_FIRST-3);
  //NM_RETURN             =  (NM_FIRST-4);
  //NM_RCLICK             =  (NM_FIRST-5);
  //NM_RDBLCLK            =  (NM_FIRST-6);
  //NM_SETFOCUS           =  (NM_FIRST-7);
  //NM_KILLFOCUS          =  (NM_FIRST-8);
  NM_CUSTOMDRAW         =  (NM_FIRST-12);
  NM_HOVER              =  (NM_FIRST-13);
  NM_NCHITTEST          =  (NM_FIRST-14);
  NM_KEYDOWN            =  (NM_FIRST-15);
  NM_RECOGNIZEGESTURE   =  (NM_FIRST-50);

  { Header control notifications  }
  HDN_BEGINTRACKW = HDN_FIRST-26;
  HDN_DIVIDERDBLCLICKW = HDN_FIRST-25;
  HDN_ENDTRACKW = HDN_FIRST-27;
  HDN_ITEMCHANGEDW = HDN_FIRST-21;
  HDN_ITEMCHANGINGW = HDN_FIRST-20;
  HDN_ITEMCLICKW = HDN_FIRST-22;
  HDN_ITEMDBLCLICKW = HDN_FIRST-23;
  HDN_TRACKW = HDN_FIRST-28;
  HDN_GETDISPINFOW = HDN_FIRST-29;
  HDN_BEGINDRAG = HDN_FIRST-10;
  HDN_ENDDRAG = HDN_FIRST-11;

  HDN_BEGINTRACK = HDN_BEGINTRACKW;
  HDN_DIVIDERDBLCLICK = HDN_DIVIDERDBLCLICKW;
  HDN_ENDTRACK = HDN_ENDTRACKW;
  HDN_ITEMCHANGED = HDN_ITEMCHANGEDW;
  HDN_ITEMCHANGING = HDN_ITEMCHANGINGW;
  HDN_ITEMCLICK = HDN_ITEMCLICKW;
  HDN_ITEMDBLCLICK = HDN_ITEMDBLCLICKW;
  HDN_TRACK = HDN_TRACKW;
  HDN_GETDISPINFO = HDN_GETDISPINFOW;
     
<<<<<<< HEAD
<<<<<<< HEAD
  // MONTHCAL CONTROL
  MONTHCAL_CLASS        = 'SysMonthCal32';
  MCM_FIRST             = $1000;
  MCM_GETCURSEL         = MCM_FIRST + 1;
  MCM_SETCURSEL         = MCM_FIRST + 2;
  MCM_GETMAXSELCOUNT    = MCM_FIRST + 3;
  MCM_SETMAXSELCOUNT    = MCM_FIRST + 4;
  MCM_GETSELRANGE       = MCM_FIRST + 5;
  MCM_SETSELRANGE       = MCM_FIRST + 6;
  MCM_GETMONTHRANGE     = MCM_FIRST + 7;
  MCM_SETDAYSTATE       = MCM_FIRST + 8;
  MCM_GETMINREQRECT     = MCM_FIRST + 9;
  MCM_SETCOLOR          = MCM_FIRST + 10;
  MCM_GETCOLOR          = MCM_FIRST + 11;

  // color consts
  MCSC_BACKGROUND   = 0;   // the background color (between months)
  MCSC_TEXT         = 1;   // the dates
  MCSC_TITLEBK      = 2;   // background of the title
  MCSC_TITLETEXT    = 3;
  MCSC_MONTHBK      = 4;   // background within the month cal
  MCSC_TRAILINGTEXT = 5;   // the text color of header & trailing days

  MCM_SETTODAY          = MCM_FIRST + 12;
  MCM_GETTODAY          = MCM_FIRST + 13;
  MCM_HITTEST           = MCM_FIRST + 14;

  // hittest contst
  MCHT_TITLE     = $00010000;
  MCHT_CALENDAR  = $00020000;
  MCHT_TODAYLINK = $00030000;
  MCHT_NONELINK  = $00040000;
  MCHT_NEXT      = $01000000;   // these indicate that hitting
  MCHT_PREV      = $02000000;  // here will go to the next/prev month
  MCHT_NOWHERE   = $00000000;
  MCHT_TITLEBK          = MCHT_TITLE;
  MCHT_TITLEMONTH       = MCHT_TITLE or $0001;
  MCHT_TITLEYEAR        = MCHT_TITLE or $0002;
  MCHT_TITLEBTNNEXT     = MCHT_TITLE or MCHT_NEXT or $0003;
  MCHT_TITLEBTNPREV     = MCHT_TITLE or MCHT_PREV or $0003;
  MCHT_CALENDARBK       = MCHT_CALENDAR;
  MCHT_CALENDARDATE     = MCHT_CALENDAR or $0001;
  MCHT_CALENDARDATENEXT = MCHT_CALENDARDATE or MCHT_NEXT;
  MCHT_CALENDARDATEPREV = MCHT_CALENDARDATE or MCHT_PREV;
  MCHT_CALENDARDAY      = MCHT_CALENDAR or $0002;
  MCHT_CALENDARWEEKNUM  = MCHT_CALENDAR or $0003;

  MCM_SETFIRSTDAYOFWEEK = MCM_FIRST + 15;
  MCM_GETFIRSTDAYOFWEEK = MCM_FIRST + 16;
  MCM_GETRANGE          = MCM_FIRST + 17;
  MCM_SETRANGE          = MCM_FIRST + 18;
  MCM_GETMONTHDELTA     = MCM_FIRST + 19;
  MCM_SETMONTHDELTA     = MCM_FIRST + 20;
  MCM_GETMAXTODAYWIDTH  = MCM_FIRST + 21;
  MCM_GETMAXNONEWIDTH   = MCM_FIRST + 22;

  // notifications
  MCN_SELCHANGE   = MCN_FIRST + 1;
  MCN_GETDAYSTATE = MCN_FIRST + 3;
  MCN_SELECT      = MCN_FIRST + 4;
  MCN_SELECTNONE  = MCN_FIRST + 5;

  // control styles
  MCS_DAYSTATE      = $0001;
  MCS_MULTISELECT   = $0002;
  MCS_WEEKNUMBERS   = $0004;
  MCS_SHOWNONE      = $0080;
  MCS_NOTODAYCIRCLE = $0008;
  MCS_NOTODAY       = $0010;

  GMR_VISIBLE  = 0; // visible portion of display
  GMR_DAYSTATE = 1; // above plus the grayed out parts of
                    // partially displayed months

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  // DATETIMEPICK CONTROL
  DATETIMEPICK_CLASS  ='SysDateTimePick32';
  DTM_FIRST           = $1000;
  DTM_GETSYSTEMTIME   = DTM_FIRST + 1;
  DTM_SETSYSTEMTIME   = DTM_FIRST + 2;
  DTM_GETRANGE        = DTM_FIRST + 3;
  DTM_SETRANGE        = DTM_FIRST + 4;
  DTM_SETFORMATA      = DTM_FIRST + 5;
  DTM_SETMCCOLOR      = DTM_FIRST + 6;
  DTM_GETMCCOLOR      = DTM_FIRST + 7;
  DTM_GETMONTHCAL     = DTM_FIRST + 8;
  DTM_SETMCFONT       = DTM_FIRST + 9;
  DTM_GETMCFONT       = DTM_FIRST + 10;
  DTM_SETFORMATW      = DTM_FIRST + 50;
  DTM_SETFORMAT       = DTM_SETFORMATW;


  DTS_UPDOWN                 = $0001;
  DTS_SHOWNONE               = $0002;
  DTS_SHORTDATEFORMAT        = $0000;
  DTS_LONGDATEFORMAT         = $0004;
  DTS_SHORTDATECENTURYFORMAT = $000C;
  DTS_TIMEFORMAT             = $0009;
  DTS_APPCANPARSE            = $0010;
  DTS_RIGHTALIGN             = $0020;
  DTS_NONEBUTTON             = $0080;

  //datetimepick
  DTN_FIRST                  =-760;
  DTN_LAST                   =-799;
  DTN_DATETIMECHANGE  = DTN_FIRST + 1;
  DTN_USERSTRINGA     = DTN_FIRST + 2;
  DTN_WMKEYDOWNA      = DTN_FIRST + 3;
  DTN_FORMATA         = DTN_FIRST + 4;
  DTN_FORMATQUERYA    = DTN_FIRST + 5;
  DTN_DROPDOWN        = DTN_FIRST + 6;
  DTN_CLOSEUP         = DTN_FIRST + 7;
  DTN_USERSTRINGW     = DTN_FIRST + 15;
  DTN_WMKEYDOWNW      = DTN_FIRST + 16;
  DTN_FORMATW         = DTN_FIRST + 17;
  DTN_FORMATQUERYW    = DTN_FIRST + 18;
  
  DTN_USERSTRING     = DTN_USERSTRINGW;

  GDTR_MIN            = $0001;
  GDTR_MAX            = $0002;

  GDT_ERROR           = -1;
  GDT_VALID           = 0;
  GDT_NONE            = 1;

  //TOOLBAR CONTROL
  //TOOLBARCLASSNAMEW   = 'ToolbarWindow32';
  //TOOLBARCLASSNAME    = TOOLBARCLASSNAMEW;

  TBIF_IMAGE          =  $00000001;
  TBIF_TEXT           =  $00000002;
  TBIF_STATE          =  $00000004;
  TBIF_STYLE          =  $00000008;
  TBIF_LPARAM         =  $00000010;
  TBIF_COMMAND        =  $00000020;
  TBIF_SIZE           =  $00000040;

  // BUTTONINFO APIs
  TB_GETBUTTONINFOW   = (WM_USER + 63);
  TB_SETBUTTONINFOW   = (WM_USER + 64);
  TB_GETBUTTONINFO    = TB_GETBUTTONINFOW;
  TB_SETBUTTONINFO    = TB_SETBUTTONINFOW;

  TB_INSERTBUTTONW    = (WM_USER + 67);
  TB_ADDBUTTONSW      = (WM_USER + 68);
  TB_HITTEST          = (WM_USER + 69);

  //TB_INSERTBUTTON     = TB_INSERTBUTTONW;
  //TB_ADDBUTTONS       = TB_ADDBUTTONSW;

  TB_SETDRAWTEXTFLAGS = (WM_USER + 70);

  //TBN_GETBUTTONINFOW  = (TBN_FIRST-20);
  //TBN_BEGINDRAG       = (TBN_FIRST-1);
  //TBN_ENDDRAG         = (TBN_FIRST-2);
  TBN_DROPDOWN        = (TBN_FIRST - 10);
  TBN_DRAGOUT         = (TBN_FIRST - 14);
  //TBN_GETBUTTONINFO   = TBN_GETBUTTONINFOW;

  // custom draw return flags
  // values under 0x00010000 are reserved for global custom draw values.
  // above that are for specific controls
  CDRF_DODEFAULT          = $00000000;
  CDRF_NEWFONT            = $00000002;
  CDRF_SKIPDEFAULT        = $00000004;
  CDRF_NOTIFYPOSTPAINT    = $00000010;
  CDRF_NOTIFYITEMDRAW     = $00000020;
  CDRF_NOTIFYSUBITEMDRAW  = $00000020;  // flags are the same, we can distinguish by context
  CDRF_NOTIFYPOSTERASE    = $00000040;

  // drawstage flags
  // values under = $00010000 are reserved for global custom draw values.
  // above that are for specific controls
  CDDS_PREPAINT           = $00000001;
  CDDS_POSTPAINT          = $00000002;
  CDDS_PREERASE           = $00000003;
  CDDS_POSTERASE          = $00000004;
  // the = $000010000 bit means it's individual item specific
  CDDS_ITEM               = $00010000;
  CDDS_ITEMPREPAINT       = CDDS_ITEM or CDDS_PREPAINT;
  CDDS_ITEMPOSTPAINT      = CDDS_ITEM or CDDS_POSTPAINT;
  CDDS_ITEMPREERASE       = CDDS_ITEM or CDDS_PREERASE;
  CDDS_ITEMPOSTERASE      = CDDS_ITEM or CDDS_POSTERASE;
  CDDS_SUBITEM            = $00020000;

  // Pocket PC  special controls
  WC_CAPEDIT    = 'CAPEDIT';
  WC_TSTATIC    = 'TTSTATIC';
  WC_TBUTTON    = 'TTBUTTON';

  // for FE, single byte character edit
  WC_SBEDIT     = 'sbedit';
  
  // REBAR CONTROL
  REBARCLASSNAME = 'ReBarWindow';

  RBIM_IMAGELIST = $00000001;
  RBS_VARHEIGHT = $0200;
  RBS_BANDBORDERS = $0400;
  RBS_FIXEDORDER = $0800;
  RBS_SMARTLABELS = $1000;
  RBS_AUTOSIZE = $2000;
  RBS_VERTICALGRIPPER = $4000;       { this always has the vertical gripper (default for horizontal mode) }

  RBBS_BREAK = $00000001;       { break to new line }
  RBBS_FIXEDSIZE = $00000002;       { band can't be sized }
  RBBS_CHILDEDGE = $00000004;       { edge around top & bottom of child window }
  RBBS_HIDDEN = $00000008;       { don't show }
  RBBS_NOVERT = $00000010;       { don't show when vertical }
  RBBS_FIXEDBMP = $00000020;       { bitmap doesn't move during band resize }
  RBBS_VARIABLEHEIGHT = $00000040;       { allow autosizing of this child vertically }
  RBBS_GRIPPERALWAYS = $00000080;       { always show the gripper }
  RBBS_NOGRIPPER = $00000100;       { never show gripper for this band }

  RBBIM_STYLE = $00000001;
  RBBIM_COLORS = $00000002;
  RBBIM_TEXT = $00000004;
  RBBIM_IMAGE = $00000008;
  RBBIM_CHILD = $00000010;
  RBBIM_CHILDSIZE = $00000020;
  RBBIM_SIZE = $00000040;
  RBBIM_BACKGROUND = $00000080;
  RBBIM_ID = $00000100;
  RBBIM_IDEALSIZE = $00000200;
  RBBIM_LPARAM = $00000400;
  
  RB_DELETEBAND = WM_USER+2;
  RB_GETBARINFO = WM_USER+3;
  RB_SETBARINFO = WM_USER+4;
  RB_SETPARENT = WM_USER+7;
  RB_HITTEST = WM_USER+8;
  RB_GETRECT = WM_USER+9;
  RB_INSERTBANDW = WM_USER+10;
  RB_SETBANDINFOW = WM_USER+11;
  RB_GETBANDCOUNT = WM_USER+12;
  RB_GETROWCOUNT = WM_USER+13;
  RB_GETROWHEIGHT = WM_USER+14;
  RB_IDTOINDEX = WM_USER+16;       { wParam == id }
  RB_SETBKCOLOR = WM_USER+19;       { sets the default BK color }
  RB_GETBKCOLOR = WM_USER+20;       { defaults to CLR_NONE }
  RB_SETTEXTCOLOR = WM_USER+21;
  RB_GETTEXTCOLOR = WM_USER+22;       { defaults to 0x00000000 }
  RB_SIZETORECT = WM_USER+23;       { resize the rebar/break bands and such to this rect (lparam) }
  RB_INSERTBAND = RB_INSERTBANDW;
  RB_SETBANDINFO = RB_SETBANDINFOW;
  RB_BEGINDRAG = WM_USER+24;
  RB_ENDDRAG = WM_USER+25;
  RB_DRAGMOVE = WM_USER+26;
  RB_GETBARHEIGHT = WM_USER+27;
  RB_GETBANDINFOW = WM_USER+28;
  RB_GETBANDINFO = RB_GETBANDINFOW;
  RB_MINIMIZEBAND = WM_USER+30;
  RB_MAXIMIZEBAND = WM_USER+31;
  RB_GETBANDBORDERS = WM_USER+34;       { returns in lparam = lprc the amount of edges added to band wparam }
  RB_SHOWBAND = WM_USER+35;       { show/hide band }
  RB_MOVEBAND = WM_USER+36;

  RBN_HEIGHTCHANGE = RBN_FIRST-0;
  RBN_LAYOUTCHANGED = RBN_FIRST-2;
  RBN_AUTOSIZE = RBN_FIRST-3;
  RBN_BEGINDRAG = RBN_FIRST-4;
  RBN_ENDDRAG = RBN_FIRST-5;

  RBHT_NOWHERE = $0001;
  RBHT_CAPTION = $0002;
  RBHT_CLIENT = $0003;
  RBHT_GRABBER = $0004;

<<<<<<< HEAD
<<<<<<< HEAD
  CMDBAR_HELP  = $000B;
  CMDBAR_OK    = $F000;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//*****************************************************************************
// types
//*****************************************************************************

type
<<<<<<< HEAD
<<<<<<< HEAD
  // MONTHCAL CONTROL
  MONTHDAYSTATE   = DWORD;
  LPMONTHDAYSTATE = ^MONTHDAYSTATE;
  TMonthDayState  = MONTHDAYSTATE;
  PMonthDayState  = LPMONTHDAYSTATE;

  MCHITTESTINFO = record
    cbSize: UINT;
    pt    : POINT;
    uHit  : UINT;          // out param
    st    : SYSTEMTIME;
  end;
  TMCMHITTESTINFO = MCHITTESTINFO;
  PMCMHitTestInfo = ^MCHITTESTINFO;

  tagNMSELCHANGE = record
    nmhdr     : NMHDR;      // this must be first, so we don't break WM_NOTIFY
    stSelStart: SYSTEMTIME;
    stSelEnd  : SYSTEMTIME;
  end;
  NMSELCHANGE = tagNMSELCHANGE;
  TNMSELCHANGE = tagNMSELCHANGE;
  LPNMSELCHANGE = ^tagNMSELCHANGE;

  tagNMDAYSTATE = record
    nmhdr      : NMHDR;           // this must be first, so we don't break WM_NOTIFY
    stStart    : SYSTEMTIME;
    cDayState  : longint;
    prgDayState: LPMONTHDAYSTATE; // points to cDayState MONTHDAYSTATEs
  end;
  NMDAYSTATE = tagNMDAYSTATE;
  TNMDAYSTATE = tagNMDAYSTATE;
  LPNMDAYSTATE = ^tagNMDAYSTATE;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  // DATETIMEPICK CONTROL
  tagNMDATETIMECHANGE = record
    nmhdr   : NMHDR;
    dwFlags : DWORD;
    st      : SYSTEMTIME;
  end;
  NMDATETIMECHANGE=tagNMDATETIMECHANGE;
  TNMDATETIMECHANGE=tagNMDATETIMECHANGE;
  LPNMDATETIMECHANGE=^tagNMDATETIMECHANGE;

  tagNMDATETIMESTRINGW = record
    nmhdr         : NMHDR;
    pszUserString : LPCWSTR;
    st            : SYSTEMTIME;
    dwFlags       : DWORD;
  end;
  NMDATETIMESTRINGW=tagNMDATETIMESTRINGW;
  TNMDATETIMESTRINGW=tagNMDATETIMESTRINGW;
  LPNMDATETIMESTRINGW=^tagNMDATETIMESTRINGW;
  NMDATETIMESTRING=tagNMDATETIMESTRINGW;
  TNMDATETIMESTRING=tagNMDATETIMESTRINGW;
  PNMDATETIMESTRING=^tagNMDATETIMESTRINGW;

  tagNMDATETIMEWMKEYDOWNA = record
    nmhdr     : NMHDR;
    nVirtKey  : longint;
    pszFormat : LPCSTR;
    st        : SYSTEMTIME;
  end;
  NMDATETIMEWMKEYDOWNA=tagNMDATETIMEWMKEYDOWNA;
  TNMDATETIMEWMKEYDOWNA=tagNMDATETIMEWMKEYDOWNA;
  LPNMDATETIMEWMKEYDOWNA=^tagNMDATETIMEWMKEYDOWNA;

  tagNMDATETIMEWMKEYDOWNW = record
    nmhdr     : NMHDR;
    nVirtKey  : longint;
    pszFormat : LPCWSTR;
    st        : SYSTEMTIME;
  end;
  NMDATETIMEWMKEYDOWNW=tagNMDATETIMEWMKEYDOWNW;
  TNMDATETIMEWMKEYDOWNW=tagNMDATETIMEWMKEYDOWNW;
  LPNMDATETIMEWMKEYDOWNW=^tagNMDATETIMEWMKEYDOWNW;


  tagNMDATETIMEFORMATA = record
    nmhdr      : NMHDR;
    pszFormat  : LPCSTR;
    st         : SYSTEMTIME;
    pszDisplay : LPCSTR;
    szDisplay  : Array[0..63] of CHAR;
  end;
  NMDATETIMEFORMATA=tagNMDATETIMEFORMATA;
  TNMDATETIMEFORMATA=tagNMDATETIMEFORMATA;
  LPNMDATETIMEFORMATA=^NMDATETIMEFORMATA;

  tagNMDATETIMEFORMATW = record
    nmhdr      : NMHDR;
    pszFormat  : LPCWSTR;
    st         : SYSTEMTIME;
    pszDisplay : LPCWSTR;
    szDisplay  : Array[0..63] of WCHAR;
  end;
  NMDATETIMEFORMATW=tagNMDATETIMEFORMATW;
  TNMDATETIMEFORMATW=tagNMDATETIMEFORMATW;
  LPNMDATETIMEFORMATW=^NMDATETIMEFORMATW;


  tagNMDATETIMEFORMATQUERYA = record
    nmhdr      : NMHDR;
    pszFormat  : LPCSTR;
    szMax      : SIZE;
  end;
  NMDATETIMEFORMATQUERYA=tagNMDATETIMEFORMATQUERYA;
  TNMDATETIMEFORMATQUERYA=tagNMDATETIMEFORMATQUERYA;
  LPNMDATETIMEFORMATQUERYA=^tagNMDATETIMEFORMATQUERYA;

  tagNMDATETIMEFORMATQUERYW = record
    nmhdr      : NMHDR;
    pszFormat  : LPCWSTR;
    szMax      : SIZE;
  end;
  NMDATETIMEFORMATQUERYW=tagNMDATETIMEFORMATQUERYW;
  TNMDATETIMEFORMATQUERYW=tagNMDATETIMEFORMATQUERYW;
  LPNMDATETIMEFORMATQUERYW=^tagNMDATETIMEFORMATQUERYW;

  //Generic structure for a key
  tagNMKEY = Record
    hdr   : NMHDR;
    wVKey : WORD;
    flags : UINT;
  end;
  NMKEY=tagNMKEY;
  LPNMKEY=^NMKEY;

  //Generic WM_NOTIFY notification structures
  tagNMMOUSE = Record
    hdr        : NMHDR;
    dwItemSpec : DWORD;
    dwItemData : DWORD;
    pt         : POINT;
  end;
  NMMOUSE=tagNMMOUSE;
  LPNMMOUSE=^NMMOUSE;
  PNMMOUSE=^NMMOUSE;
  NMCLICK=NMMOUSE;
  LPNMCLICK=LPNMMOUSE;

  //TOOLBAR CONTROL
  TBBUTTONINFOW = record
    cbSize    : UINT;
    dwMask    : DWORD;
    idCommand : Longint;
    iImage    : Longint;
    fsState   : BYTE;
    fsStyle   : BYTE;
    cx        : WORD;
    lParam    : DWORD;
    pszText   : LPWSTR;
    cchText   : Longint;
  end;
  LPTBBUTTONINFOW=^TBBUTTONINFOW;
  TBBUTTONINFO=TBBUTTONINFOW;
  TTBButtonInfo=TBBUTTONINFO;

  tagNMCUSTOMDRAWINFO = record
    hdr: TNMHdr;
    dwDrawStage: DWORD;
    hdc: HDC;
    rc: TRect;
    dwItemSpec: DWORD;
    uItemState: UINT;
    lItemlParam: LPARAM;
  end;
  PNMCustomDraw = ^TNMCustomDraw;
  TNMCustomDraw = tagNMCUSTOMDRAWINFO;
  
  tagNMLVCUSTOMDRAW = record
    nmcd: TNMCustomDraw;
    clrText: COLORREF;
    clrTextBk: COLORREF;
    iSubItem: longint;
  end;
  PNMLVCustomDraw = ^TNMLVCustomDraw;
  TNMLVCustomDraw = tagNMLVCUSTOMDRAW;
  
  tagNMLVODSTATECHANGE = record
    hdr: TNMHdr;
    iFrom: longint;
    iTo: longint;
    uNewState: UINT;
    uOldState: UINT;
  end;
  PNMLVODStateChange = ^TNMLVODStateChange;
  TNMLVODStateChange = tagNMLVODSTATECHANGE;
  
  tagREBARINFO = record
    cbSize : UINT;
    fMask : UINT;
    himl : HIMAGELIST;
  end;
  REBARINFO = tagREBARINFO;
  LPREBARINFO = ^tagREBARINFO;
  
  tagREBARBANDINFOW = record
    cbSize : UINT;
    fMask : UINT;
    fStyle : UINT;
    clrFore : COLORREF;
    clrBack : COLORREF;
    lpText : LPWSTR;
    cch : UINT;
    iImage : longint;
    hwndChild : HWND;
    cxMinChild : UINT;
    cyMinChild : UINT;
    cx : UINT;
    hbmBack : HBITMAP;
    wID : UINT;
    cyChild : UINT;
    cyMaxChild : UINT;
    cyIntegral : UINT;
    cxIdeal : UINT;
    lParam : LPARAM;
  end;
  REBARBANDINFOW = tagREBARBANDINFOW;
  LPREBARBANDINFOW = ^tagREBARBANDINFOW;
  LPCREBARBANDINFOW = REBARBANDINFOW;
  REBARBANDINFO = REBARBANDINFOW;
  LPREBARBANDINFO = LPREBARBANDINFOW;
  LPCREBARBANDINFO = LPCREBARBANDINFOW;

  tagNMREBAR = record
    hdr : NMHDR;
    uBand : UINT;
    wID : UINT;
    cyChild : UINT;
    cyBand : UINT;
   end;
  NMREBAR = tagNMREBAR;
  LPNMREBAR = ^tagNMREBAR;

  tagNMRBAUTOSIZE = record
    hdr : NMHDR;
    fChanged : BOOL;
    rcTarget : RECT;
    rcActual : RECT;
  end;
  NMRBAUTOSIZE = tagNMRBAUTOSIZE;
  LPNMRBAUTOSIZE = ^tagNMRBAUTOSIZE;

  _RB_HITTESTINFO = record
    pt : POINT;
    flags : UINT;
    iBand : longint;
  end;
  RBHITTESTINFO = _RB_HITTESTINFO;
  LPRBHITTESTINFO = ^_RB_HITTESTINFO;
       
  tagCOMMANDBANDSRESTOREINFO = record
    cbSize : UINT;
    wID : UINT;
    fStyle : UINT;
    cxRestored : UINT;
    fMaximized : BOOL;
  end;
  COMMANDBANDSRESTOREINFO = tagCOMMANDBANDSRESTOREINFO;
  LPCOMMANDBANDSRESTOREINFO = ^tagCOMMANDBANDSRESTOREINFO;
  LPCCOMMANDBANDSRESTOREINFO = COMMANDBANDSRESTOREINFO;

//*****************************************************************************
// functions
//*****************************************************************************

function CreatePropertySheetPage(lppsp:LPCPROPSHEETPAGE):HPROPSHEETPAGE; external ComctlDLL name 'CreatePropertySheetPageW';
function CreatePropertySheetPageW(lppsp:LPCPROPSHEETPAGE):HPROPSHEETPAGE; external ComctlDLL name 'CreatePropertySheetPageW';
function CreateStatusWindow(style:LONG; lpszText:LPCWSTR; hwndParent:HWND; wID:UINT):HWND; external ComctlDll name 'CreateStatusWindowW';
function CreateStatusWindowW(style:LONG; lpszText:LPCWSTR; hwndParent:HWND; wID:UINT):HWND; external ComctlDll name 'CreateStatusWindowW';
function CreateToolbarEx(hwnd:HWND; ws:DWORD; wID:UINT; nBitmaps:longint; hBMInst:HINST;wBMID:UINT; lpButtons:LPCTBBUTTON; iNumButtons:longint; dxButton:longint; dyButton:longint;dxBitmap:longint;
  dyBitmap:longint; uStructSize:UINT):HWND; external ComctlDLL name 'CreateToolbarEx';
function CreateUpDownControl(dwStyle:DWORD; x:longint; y:longint; cx:longint; cy:longint;hParent:HWND; nID:longint; hInst:HINST; hBuddy:HWND; nUpper:longint;nLower:longint; nPos:longint):HWND; external ComctlDLL name 'CreateUpDownControl';
function DestroyPropertySheetPage(hPSPage:HPROPSHEETPAGE):WINBOOL; external ComctlDLL name 'DestroyPropertySheetPage';
procedure DrawStatusText(hDC:HDC; lprc:LPRECT; pszText:LPCWSTR; uFlags:UINT); external ComctlDLL name 'DrawStatusTextW';
procedure DrawStatusTextW(hDC:HDC; lprc:LPRECT; pszText:LPCWSTR; uFlags:UINT); external ComctlDLL name 'DrawStatusTextW';
procedure InitCommonControls; external ComctlDLL name 'InitCommonControls';
function InitCommonControlsEx(_para1:LPINITCOMMONCONTROLSEX):WINBOOL; external ComctlDLL name 'InitCommonControlsEx';
function PropertySheet(lppsph:LPCPROPSHEETHEADER):longint; external ComctlDll name 'PropertySheetW';
function PropertySheetW(lppsph:LPCPROPSHEETHEADER):longint; external ComctlDll name 'PropertySheetW';

function CommandBands_AddAdornments(hwndCmdBands:HWND; hinst:THandle; dwFlags:DWORD; prbbi:LPREBARBANDINFO):BOOL;external ComctlDll name 'CommandBands_AddAdornments';
function CommandBands_AddBands(hwndCmdBands:HWND; hinst:THandle; cBands:UINT; prbbi:LPREBARBANDINFO):BOOL;external ComctlDll name 'CommandBands_AddBands';
function CommandBands_Create(hinst:THandle; hwndParent:HWND; wID:UINT; dwStyles:DWORD; himl:HIMAGELIST):HWND;external ComctlDll name 'CommandBands_Create';
function CommandBands_GetCommandBar(hwndCmdBands:HWND; uBand:UINT):HWND;external ComctlDll name 'CommandBands_GetCommandBar';
function CommandBands_GetRestoreInformation(hwndCmdBands:HWND; uBand:UINT; pcbri:LPCOMMANDBANDSRESTOREINFO):BOOL;external ComctlDll name 'CommandBands_GetRestoreInformation';
function CommandBands_Show(hwndCmdBands:HWND; fShow:BOOL):BOOL;external ComctlDll name 'CommandBands_Show';
function CommandBar_AddAdornments(hwndCB:HWND; dwFlags:DWORD; dwReserved:DWORD):BOOL;external ComctlDll name 'CommandBar_AddAdornments';
function CommandBar_AddBitmap(hwndCB:HWND; hInst:THandle; idBitmap:longint; iNumImages:longint; iImageWidth:longint; iImageHeight:longint):longint;external ComctlDll name 'CommandBar_AddBitmap';
function CommandBar_Create(hInst:THandle; hwndParent:HWND; idCmdBar:longint):HWND;external ComctlDll name 'CommandBar_Create';
function CommandBar_DrawMenuBar(hwndCB:HWND; iButton:WORD):BOOL;external ComctlDll name 'CommandBar_DrawMenuBar';
function CommandBar_GetMenu(hwndCB:HWND; iButton:WORD):HMENU;external ComctlDll name 'CommandBar_GetMenu';
function CommandBar_Height(hwndCB:HWND):longint;external ComctlDll name 'CommandBar_Height';
function CommandBar_InsertComboBox(hwndCB:HWND; THandle:THandle; iWidth:longint; dwStyle:UINT; idComboBox:WORD; iButton:WORD):HWND;external ComctlDll name 'CommandBar_InsertComboBox';
function CommandBar_InsertMenubar(hwndCB:HWND; hInst:THandle; idMenu:WORD; iButton:WORD):BOOL;external ComctlDll name 'CommandBar_InsertMenubar';
function CommandBar_InsertMenubarEx(hwndCB:HWND; hinst:THandle; pszMenu:LPTSTR; iButton:WORD):BOOL;external ComctlDll name 'CommandBar_InsertMenubarEx';
function CommandBar_Show(hwndCB:HWND; fShow:BOOL):BOOL;external ComctlDll name 'CommandBar_Show';
function IsCommandBarMessage(hwndCB:HWND; lpMsg:LPMSG):BOOL;external ComctlDll name 'IsCommandBarMessage';
procedure CommandBar_Destroy(hwndCB : HWND);
function CommandBar_IsVisible(hwndCB : HWND) : BOOL;
function CommandBar_AddButtons(hwndCB : HWND; cbButtons : UINT; lpButtons : LPTBBUTTON) : BOOL;
function CommandBar_InsertButton(hwndCB : HWND; iButton : longint; lpButton : LPTBBUTTON) : BOOL;
function CommandBar_AddToolTips(hwndCB: HWND; cbToolTips : UINT; lpToolTipsStrings : LPTSTR) : BOOL;
function CommandBands_Height(hwndCmdBands : HWND) : UINT;
function CommandBands_IsVisible(hwndCmdBands : HWND) : BOOL;

implementation

procedure CommandBar_Destroy(hwndCB : HWND);
begin
  DestroyWindow(hwndCB);
end;

function CommandBar_IsVisible(hwndCB : HWND) : BOOL;
begin
  CommandBar_IsVisible:=IsWindowVisible(hwndCB);
end;

function CommandBar_AddButtons(hwndCB : HWND; cbButtons : UINT; lpButtons : LPTBBUTTON) : BOOL;
begin
   CommandBar_AddButtons:=BOOL(SendMessage(hwndCB,TB_ADDBUTTONS,WPARAM(cbButtons),LPARAM(lpButtons)));
end;

function CommandBar_InsertButton(hwndCB : HWND; iButton : longint; lpButton : LPTBBUTTON) : BOOL;
begin
   CommandBar_InsertButton:=BOOL(SendMessage(hwndCB,TB_INSERTBUTTON,WPARAM(iButton),LPARAM(lpButton)));
end;

function CommandBar_AddToolTips(hwndCB: HWND; cbToolTips : UINT; lpToolTipsStrings : LPTSTR) : BOOL;
begin
   CommandBar_AddToolTips:=BOOL(SendMessage(hwndCB,TB_SETTOOLTIPS,WPARAM(cbToolTips),LPARAM(lpToolTipsStrings)));
end;

function CommandBands_Height(hwndCmdBands : HWND) : UINT;
begin
   CommandBands_Height:=UINT(SendMessage(hwndCmdBands,RB_GETBARHEIGHT,0,0));
end;

function CommandBands_IsVisible(hwndCmdBands : HWND) : BOOL;
begin
   CommandBands_IsVisible:=IsWindowVisible(hwndCmdBands);
end;

end.
