{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl,
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  Messages.h    Windows32 API message definitions
  Copyright (C) 1996 Free Software Foundation, Inc.

  Author: Scott Christley <scottc@net-community.com>

  This file is part of the Windows32 API Library.


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Library General Public
  License as published by the Free Software Foundation; either
  version 2 of the License, or (at your option) any later version.


  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Library General Public License for more details.


  If you are interested in a warranty or support for this source code,
  contact Scott Christley <scottc@net-community.com> for more information.


  You should have received a copy of the GNU Library General Public
  License along with this library; see the file COPYING.LIB.
  If not, write to the Free Software Foundation,

  59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

  Changes :

  08-15-2005 : orinaudo@gmail.com,  WCE 4.21 SE, First release
  09-02-2006 : updated
}

{$ifdef read_interface}

    const
       ACM_OPENW = 1127;
       ACM_OPENA = 1124;
  {$ifdef UNICODE}
    const
       ACM_OPEN = ACM_OPENW;
  {$else}
    const
       ACM_OPEN = ACM_OPENA;
  {$endif}
    { UNICODE  }

    const
    
       WM_USER = 1024;  //keep it first
<<<<<<< HEAD
<<<<<<< HEAD
       WM_APP  = $8000;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

       ACM_PLAY = 1125;
       ACM_STOP = 1126;
       ACN_START = 1;
       ACN_STOP = 2;
    { Buttons  }
       BM_CLICK = 245;
       BM_GETCHECK = 240;
       BM_GETIMAGE = 246;
       BM_GETSTATE = 242;
       BM_SETCHECK = 241;
       BM_SETIMAGE = 247;
       BM_SETSTATE = 243;
       BM_SETSTYLE = 244;
       BN_CLICKED = 0;
     BN_DBLCLK = 5;
     BN_DISABLE = 4;
     BN_DOUBLECLICKED = 5;
     BN_HILITE = 2;
     BN_KILLFOCUS = 7;
     BN_PAINT = 1;
     BN_PUSHED = 2;
     BN_SETFOCUS = 6;
     BN_UNHILITE = 3;
     BN_UNPUSHED = 3;
  { Combo Box  }
     CB_ADDSTRING = 323;
     CB_DELETESTRING = 324;
     CB_DIR = 325;
     CB_FINDSTRING = 332;
     CB_FINDSTRINGEXACT = 344;
     CB_GETCOUNT = 326;
     CB_GETCURSEL = 327;
     CB_GETDROPPEDCONTROLRECT = 338;
     CB_GETDROPPEDSTATE = 343;
     CB_GETDROPPEDWIDTH = 351;
     CB_GETEDITSEL = 320;
     CB_GETEXTENDEDUI = 342;
     CB_GETHORIZONTALEXTENT = 349;
     CB_GETITEMDATA = 336;
     CB_GETITEMHEIGHT = 340;
     CB_GETLBTEXT = 328;
     CB_GETLBTEXTLEN = 329;
     CB_GETLOCALE = 346;
     CB_GETTOPINDEX = 347;
     CB_INITSTORAGE = 353;
     CB_INSERTSTRING = 330;
     CB_LIMITTEXT = 321;
     CB_RESETCONTENT = 331;
     CB_SELECTSTRING = 333;
     CB_SETCURSEL = 334;
     CB_SETDROPPEDWIDTH = 352;
     CB_SETEDITSEL = 322;
     CB_SETEXTENDEDUI = 341;
     CB_SETHORIZONTALEXTENT = 350;
     CB_SETITEMDATA = 337;
     CB_SETITEMHEIGHT = 339;
     CB_SETLOCALE = 345;
     CB_SETTOPINDEX = 348;
     CB_SHOWDROPDOWN = 335;
     CB_GETCOMBOBOXINFO = $0162;
  { Combo Box notifications  }
     CBN_CLOSEUP = 8;
     CBN_DBLCLK = 2;
     CBN_DROPDOWN = 7;
     CBN_EDITCHANGE = 5;
     CBN_EDITUPDATE = 6;
     CBN_ERRSPACE = -(1);
     CBN_KILLFOCUS = 4;
     CBN_SELCHANGE = 1;
     CBN_SELENDCANCEL = 10;
     CBN_SELENDOK = 9;
     CBN_SETFOCUS = 3;
  { Control Panel  }
  { Device messages  }
  { Drag list box  }
     DL_BEGINDRAG = 1157;
     DL_CANCELDRAG = 1160;
     DL_DRAGGING = 1158;
     DL_DROPPED = 1159;
  { Default push button  }
     DM_GETDEFID = 1024;
     DM_REPOSITION = 1026;
     DM_SETDEFID = 1025;
  { RTF control  }
     EM_CANPASTE = 1074;
     EM_CANUNDO = 198;
     EM_CHARFROMPOS = 215;
     EM_DISPLAYBAND = 1075;
     EM_EMPTYUNDOBUFFER = 205;
     EM_EXGETSEL = 1076;
     EM_EXLIMITTEXT = 1077;
     EM_EXLINEFROMCHAR = 1078;
     EM_EXSETSEL = 1079;
     EM_FINDTEXT = 1080;
     EM_FINDTEXTEX = 1103;
     EM_FINDWORDBREAK = 1100;
     EM_FMTLINES = 200;
     EM_FORMATRANGE = 1081;
     EM_GETCHARFORMAT = 1082;
     EM_GETEVENTMASK = 1083;
     EM_GETFIRSTVISIBLELINE = 206;
     EM_GETHANDLE = 189;
     EM_GETLIMITTEXT = 213;
     EM_GETLINE = 196;
     EM_GETLINECOUNT = 186;
     EM_GETMARGINS = 212;
     EM_GETMODIFY = 184;
     EM_GETIMECOLOR = 1129;
     EM_GETIMEOPTIONS = 1131;
     EM_GETOPTIONS = 1102;
     EM_GETOLEINTERFACE = 1084;
     EM_GETPARAFORMAT = 1085;
     EM_GETPASSWORDCHAR = 210;
     EM_GETPUNCTUATION = 1125;
     EM_GETRECT = 178;
     EM_GETSEL = 176;
     EM_GETSELTEXT = 1086;
     EM_GETTEXTRANGE = 1099;
     EM_GETTHUMB = 190;
     EM_GETWORDBREAKPROC = 209;
     EM_GETWORDBREAKPROCEX = 1104;
     EM_GETWORDWRAPMODE = 1127;
     EM_HIDESELECTION = 1087;
     EM_LIMITTEXT = 197;
     EM_LINEFROMCHAR = 201;
     EM_LINEINDEX = 187;
     EM_LINELENGTH = 193;
     EM_LINESCROLL = 182;
     EM_PASTESPECIAL = 1088;
     EM_POSFROMCHAR = 214;
     EM_REPLACESEL = 194;
     EM_REQUESTRESIZE = 1089;
     EM_SCROLL = 181;
     EM_SCROLLCARET = 183;
     EM_SELECTIONTYPE = 1090;
     EM_SETBKGNDCOLOR = 1091;
     EM_SETCHARFORMAT = 1092;
     EM_SETEVENTMASK = 1093;
     EM_SETHANDLE = 188;
     EM_SETIMECOLOR = 1128;
     EM_SETIMEOPTIONS = 1130;
     EM_SETLIMITTEXT = 197;
     EM_SETMARGINS = 211;
     EM_SETMODIFY = 185;
     EM_SETOLECALLBACK = 1094;
     EM_SETOPTIONS = 1101;
     EM_SETPARAFORMAT = 1095;
     EM_SETPASSWORDCHAR = 204;
     EM_SETPUNCTUATION = 1124;
     EM_SETREADONLY = 207;
     EM_SETRECT = 179;
     EM_SETRECTNP = 180;
     EM_SETSEL = 177;
     EM_SETTABSTOPS = 203;
     EM_SETTARGETDEVICE = 1096;
     EM_SETWORDBREAKPROC = 208;
     EM_SETWORDBREAKPROCEX = 1105;
     EM_SETWORDWRAPMODE = 1126;
     EM_STREAMIN = 1097;
     EM_STREAMOUT = 1098;
     EM_UNDO = 199;

  { Edit control notifications  }
     EN_SETFOCUS     =$0100;
     EN_KILLFOCUS    =$0200;
     EN_CHANGE       =$0300;
     EN_UPDATE       =$0400;
     EN_ERRSPACE     =$0500;
     EN_MAXTEXT      =$0501;
     EN_HSCROLL      =$0601;
     EN_VSCROLL      =$0602;
     EN_ALIGN_LTR_EC = $0700;
     EN_ALIGN_RTL_EC = $0701;


     EN_CORRECTTEXT = 1797;
     EN_DROPFILES = 1795;
     EN_IMECHANGE = 1799;
     EN_MSGFILTER = 1792;
     EN_OLEOPFAILED = 1801;
     EN_PROTECTED = 1796;
     EN_REQUESTRESIZE = 1793;
     EN_SAVECLIPBOARD = 1800;
     EN_SELCHANGE = 1794;
     EN_STOPNOUNDO = 1798;
     
     { Header control  }
     HDM_DELETEITEM = 4610;
     HDM_GETITEMW = 4619;
     HDM_INSERTITEMW = 4618;
     HDM_SETITEMW = 4620;

     HDM_GETITEM = HDM_GETITEMW;
     HDM_INSERTITEM = HDM_INSERTITEMW;
     HDM_SETITEM = HDM_SETITEMW;

     HDM_GETITEMCOUNT = 4608;
     HDM_HITTEST = 4614;
     HDM_LAYOUT = 4613;

  { Hot key control  }
  const
     HKM_GETHOTKEY = 1026;
     HKM_SETHOTKEY = 1025;
     HKM_SETRULES = 1027;
  { List box  }
     LB_ADDFILE = 406;
     LB_ADDSTRING = 384;
     LB_DELETESTRING = 386;
     LB_DIR = 397;
     LB_FINDSTRING = 399;
     LB_FINDSTRINGEXACT = 418;
     LB_GETANCHORINDEX = 413;
     LB_GETCARETINDEX = 415;
     LB_GETCOUNT = 395;
     LB_GETCURSEL = 392;
     LB_GETHORIZONTALEXTENT = 403;
     LB_GETITEMDATA = 409;
     LB_GETITEMHEIGHT = 417;
     LB_GETITEMRECT = 408;
     LB_GETLOCALE = 422;
     LB_GETSEL = 391;
     LB_GETSELCOUNT = 400;
     LB_GETSELITEMS = 401;
     LB_GETTEXT = 393;
     LB_GETTEXTLEN = 394;
     LB_GETTOPINDEX = 398;
     LB_INITSTORAGE = 424;
     LB_INSERTSTRING = 385;
     LB_ITEMFROMPOINT = 425;
     LB_RESETCONTENT = 388;
     LB_SELECTSTRING = 396;
     LB_SELITEMRANGE = 411;
     LB_SELITEMRANGEEX = 387;
     LB_SETANCHORINDEX = 412;
     LB_SETCARETINDEX = 414;
     LB_SETCOLUMNWIDTH = 405;
     LB_SETCOUNT = 423;
     LB_SETCURSEL = 390;
     LB_SETHORIZONTALEXTENT = 404;
     LB_SETITEMDATA = 410;
     LB_SETITEMHEIGHT = 416;
     LB_SETLOCALE = 421;
     LB_SETSEL = 389;
     LB_SETTABSTOPS = 402;
     LB_SETTOPINDEX = 407;
  { List box notifications  }
     LBN_DBLCLK = 2;
     LBN_ERRSPACE = -(2);
     LBN_KILLFOCUS = 5;
     LBN_SELCANCEL = 3;
     LBN_SELCHANGE = 1;
     LBN_SETFOCUS = 4;
  { List view control  }
     LVM_ARRANGE = 4118;
     LVM_CREATEDRAGIMAGE = 4129;
     LVM_DELETEALLITEMS = 4105;
     LVM_DELETECOLUMN = 4124;
     LVM_DELETEITEM = 4104;
     LVM_ENSUREVISIBLE = 4115;
     LVM_GETBKCOLOR = 4096;
     LVM_GETCALLBACKMASK = 4106;
     LVM_GETCOLUMNWIDTH = 4125;
     LVM_GETCOUNTPERPAGE = 4136;
     LVM_GETEDITCONTROL = 4120;
     LVM_GETIMAGELIST = 4098;
     LVM_EDITLABELW = 4214;
     LVM_FINDITEMW = 4179;
     LVM_GETCOLUMNW = 4191;
     LVM_GETISEARCHSTRINGW = 4213;
     LVM_GETITEMW = 4171;
     LVM_GETITEMTEXTW = 4211;
     LVM_GETSTRINGWIDTHW = 4183;
     LVM_INSERTCOLUMNW = 4193;
     LVM_INSERTITEMW = 4173;
     LVM_SETCOLUMNW = 4192;
     LVM_SETITEMW = 4172;
     LVM_SETITEMTEXTW = 4212;
     LVM_EDITLABELA = 4119;
     LVM_FINDITEMA = 4109;
     LVM_GETCOLUMNA = 4121;
     LVM_GETISEARCHSTRINGA = 4148;
     LVM_GETITEMA = 4101;
     LVM_GETITEMTEXTA = 4141;
     LVM_GETSTRINGWIDTHA = 4113;
     LVM_INSERTCOLUMNA = 4123;
     LVM_INSERTITEMA = 4103;
     LVM_SETCOLUMNA = 4122;
     LVM_SETITEMA = 4102;
     LVM_SETITEMTEXTA = 4142;
     LVM_GETITEMCOUNT = 4100;
     LVM_GETITEMPOSITION = 4112;
     LVM_GETITEMRECT = 4110;
     LVM_GETITEMSPACING = 4147;
     LVM_GETITEMSTATE = 4140;
     LVM_GETNEXTITEM = 4108;
     LVM_GETORIGIN = 4137;
     LVM_GETSELECTEDCOUNT = 4146;
     LVM_GETTEXTBKCOLOR = 4133;
     LVM_GETTEXTCOLOR = 4131;
     LVM_GETTOPINDEX = 4135;
     LVM_GETVIEWRECT = 4130;
     LVM_HITTEST = 4114;
     LVM_REDRAWITEMS = 4117;
     LVM_SCROLL = 4116;
     LVM_SETBKCOLOR = 4097;
     LVM_SETCALLBACKMASK = 4107;
     LVM_SETCOLUMNWIDTH = 4126;
     LVM_SETIMAGELIST = 4099;
     LVM_SETITEMCOUNT = 4143;
     LVM_SETITEMPOSITION = 4111;
     LVM_SETITEMPOSITION32 = 4145;
     LVM_SETITEMSTATE = 4139;
     LVM_SETTEXTBKCOLOR = 4134;
     LVM_SETTEXTCOLOR = 4132;
     LVM_SORTITEMS = 4144;
     LVM_UPDATE = 4138;
     LVM_SETICONSPACING           = 4149;   //LVM_FIRST + 53
     LVM_SETEXTENDEDLISTVIEWSTYLE = 4150;   //LVM_FIRST + 54
     LVM_GETSUBITEMRECT           = 4152;   //LVM_FIRST + 56
     LVM_SUBITEMHITTEST           = 4153;   //LVM_FIRST + 57
     LVM_SETCOLUMNORDERARRAY      = 4154;   //LVM_FIRST + 58
     LVM_GETCOLUMNORDERARRAY      = 4155;   //LVM_FIRST + 59
     LVM_APPROXIMATEVIEWRECT      = 4160;   //LVM_FIRST + 64
     LVM_GETSELECTIONMARK         = 4162;   //LVM_FIRST + 66
     LVM_SETSELECTIONMARK         = 4163;   //LVM_FIRST + 67
     LVM_GETHEADER                = 4127;   //LVM_FIRST + 31

{$ifdef UNICODE}

  const
     LVM_EDITLABEL = LVM_EDITLABELW;
     LVM_FINDITEM = LVM_FINDITEMW;
     LVM_GETCOLUMN = LVM_GETCOLUMNW;
     LVM_GETISEARCHSTRING = LVM_GETISEARCHSTRINGW;
     LVM_GETITEM = LVM_GETITEMW;
     LVM_GETITEMTEXT = LVM_GETITEMTEXTW;
     LVM_GETSTRINGWIDTH = LVM_GETSTRINGWIDTHW;
     LVM_INSERTCOLUMN = LVM_INSERTCOLUMNW;
     LVM_INSERTITEM = LVM_INSERTITEMW;
     LVM_SETCOLUMN = LVM_SETCOLUMNW;
     LVM_SETITEM = LVM_SETITEMW;
     LVM_SETITEMTEXT = LVM_SETITEMTEXTW;
{$else}

  const
     LVM_EDITLABEL = LVM_EDITLABELA;
     LVM_FINDITEM = LVM_FINDITEMA;
     LVM_GETCOLUMN = LVM_GETCOLUMNA;
     LVM_GETISEARCHSTRING = LVM_GETISEARCHSTRINGA;
     LVM_GETITEM = LVM_GETITEMA;
     LVM_GETITEMTEXT = LVM_GETITEMTEXTA;
     LVM_GETSTRINGWIDTH = LVM_GETSTRINGWIDTHA;
     LVM_INSERTCOLUMN = LVM_INSERTCOLUMNA;
     LVM_INSERTITEM = LVM_INSERTITEMA;
     LVM_SETCOLUMN = LVM_SETCOLUMNA;
     LVM_SETITEM = LVM_SETITEMA;
     LVM_SETITEMTEXT = LVM_SETITEMTEXTA;
{$endif}
  { UNICODE  }

  { List view control notifications  }
     LVN_ITEMCHANGING     = -(100);
     LVN_ITEMCHANGED      = -(101);
     LVN_INSERTITEM       = -(102);
     LVN_DELETEITEM       = -(103);
     LVN_DELETEALLITEMS   = -(104);
     LVN_BEGINLABELEDITA  = -(105);
     LVN_ENDLABELEDITA    = -(106);
     LVN_COLUMNCLICK      = -(108);
     LVN_BEGINDRAG        = -(109);
     LVN_BEGINRDRAG       = -(111);
     LVN_ODCACHEHINT      = -(113); //#if (_WIN32_IE >= 0x0300)
     LVN_ITEMACTIVATE     = -(114);
     LVN_ODSTATECHANGED   = -(115);
     LVN_GETDISPINFOA     = -(150);
     LVN_SETDISPINFOA     = -(151);
     LVN_ODFINDITEMA      = -(152); //#if (_WIN32_IE >= 0x0300)
     LVN_KEYDOWN          = -(155);
     LVN_MARQUEEBEGIN     = -(156); //#if (_WIN32_IE >= 0x0300)
     LVN_BEGINLABELEDITW  = -(175);
     LVN_ENDLABELEDITW    = -(176);
     LVN_GETDISPINFOW     = -(177);
     LVN_SETDISPINFOW     = -(178);
     LVN_ODFINDITEMW      = -(179); //#if (_WIN32_IE >= 0x0300)

{$ifdef UNICODE}
     LVN_BEGINLABELEDIT = LVN_BEGINLABELEDITW;
     LVN_ENDLABELEDIT = LVN_ENDLABELEDITW;
     LVN_GETDISPINFO = LVN_GETDISPINFOW;
     LVN_SETDISPINFO = LVN_SETDISPINFOW;
     LVN_ODFINDITEM  = LVN_ODFINDITEMW;
{$else}
     LVN_BEGINLABELEDIT = LVN_BEGINLABELEDITA;
     LVN_ENDLABELEDIT = LVN_ENDLABELEDITA;
     LVN_GETDISPINFO = LVN_GETDISPINFOA;
     LVN_SETDISPINFO = LVN_SETDISPINFOA;
     LVN_ODFINDITEM  = LVN_ODFINDITEMA;
{$endif}

  { UNICODE  }

  { Control notification  }
     NM_CLICK = -(2);
     NM_DBLCLK = -(3);
     NM_KILLFOCUS = -(8);
     NM_OUTOFMEMORY = -(1);
     NM_RCLICK = -(5);
     NM_RDBLCLK = -(6);
     NM_RETURN = -(4);
     NM_SETFOCUS = -(7);
  { Power status  }
  { Progress bar control  }
     PBM_DELTAPOS = 1027;
     PBM_SETPOS = 1026;
     PBM_GETPOS = WM_USER+8;
     PBM_SETRANGE = 1025;
     PBM_SETRANGE32 = WM_USER+6;
     PBM_GETRANGE = WM_USER+7;
     PBM_SETSTEP = 1028;
     PBM_STEPIT = 1029;
  { Property sheets  } //prsht
     PSM_SETCURSEL          = (WM_USER + 101);
     PSM_REMOVEPAGE         = (WM_USER + 102);
     PSM_ADDPAGE            = (WM_USER + 103);
     PSM_CHANGED            = (WM_USER + 104);
     PSM_RESTARTWINDOWS     = (WM_USER + 105);
     PSM_REBOOTSYSTEM       = (WM_USER + 106);
     PSM_CANCELTOCLOSE      = (WM_USER + 107);
     PSM_QUERYSIBLINGS      = (WM_USER + 108);
     PSM_UNCHANGED          = (WM_USER + 109);
     PSM_APPLY              = (WM_USER + 110);
     PSM_SETTITLEA          = (WM_USER + 111);
     PSM_SETWIZBUTTONS      = (WM_USER + 112);
     PSM_SETCURSELID        = (WM_USER + 114);
     PSM_SETFINISHTEXTA     = (WM_USER + 115);
     PSM_GETTABCONTROL      = (WM_USER + 116);
     PSM_ISDIALOGMESSAGE    = (WM_USER + 117);
     PSM_GETCURRENTPAGEHWND = (WM_USER + 118);
     PSM_SETTITLEW          = (WM_USER + 120);
     PSM_SETFINISHTEXTW     = (WM_USER + 121);
     PSM_PRESSBUTTON = 1137;
{$ifdef UNICODE}
     PSM_SETFINISHTEXT = PSM_SETFINISHTEXTW;
     PSM_SETTITLE = PSM_SETTITLEW;
{$else UNICODE}
     PSM_SETFINISHTEXT = PSM_SETFINISHTEXTA;
     PSM_SETTITLE = PSM_SETTITLEA;
{$endif UNICODE}

  { Property sheet notifications  } //prsht
     PSN_FIRST       = -(200);
     PSN_SETACTIVE   = (PSN_FIRST-0);
     PSN_KILLACTIVE  = (PSN_FIRST-1);
     PSN_APPLY       = (PSN_FIRST-2);
     PSN_RESET       = (PSN_FIRST-3);
     PSN_HELP        = (PSN_FIRST-5);
     PSN_WIZBACK     = (PSN_FIRST-6);
     PSN_WIZNEXT     = (PSN_FIRST-7);
     PSN_WIZFINISH   = (PSN_FIRST-8);
     PSN_QUERYCANCEL = (PSN_FIRST-9);

  { Status window  }
     SB_GETBORDERS = 1031;
     SB_GETPARTS = 1030;
     SB_GETRECT = 1034;
     SB_GETTEXTW = 1037;
     SB_GETTEXTLENGTHW = 1036;
     SB_SETTEXTW = 1035;
     SB_GETTEXTA = 1026;
     SB_GETTEXTLENGTHA = 1027;
     SB_SETTEXTA = 1025;
{$ifdef UNICODE}

  const
     SB_GETTEXT = SB_GETTEXTW;
     SB_GETTEXTLENGTH = SB_GETTEXTLENGTHW;
     SB_SETTEXT = SB_SETTEXTW;
{$else}

  const
     SB_GETTEXT = SB_GETTEXTA;
     SB_GETTEXTLENGTH = SB_GETTEXTLENGTHA;
     SB_SETTEXT = SB_SETTEXTA;
{$endif}
  { UNICODE  }

  const
     SB_SETMINHEIGHT = 1032;
     SB_SETPARTS = 1028;
     SB_SIMPLE = 1033;
  { Scroll bar control  }
     SBM_ENABLE_ARROWS = 228;
     SBM_GETPOS = 225;
     SBM_GETRANGE = 227;
     SBM_GETSCROLLINFO = 234;
     SBM_SETPOS = 224;
     SBM_SETRANGE = 226;
     SBM_SETRANGEREDRAW = 230;
     SBM_SETSCROLLINFO = 233;
  { Static control  }
     STM_GETICON = 369;
     STM_GETIMAGE = 371;
     STM_SETICON = 368;
     STM_SETIMAGE = 370;
  { Static control notifications  }
     STN_CLICKED = 0;
     STN_DBLCLK = 1;
     STN_DISABLE = 3;
     STN_ENABLE = 2;
  { Toolbar control  }
     TB_ADDBITMAP = 1043;
     TB_ADDBUTTONS = 1044;
     TB_AUTOSIZE = 1057;
     TB_BUTTONCOUNT = 1048;
     TB_BUTTONSTRUCTSIZE = 1054;
     TB_CHANGEBITMAP = 1067;
     TB_CHECKBUTTON = 1026;
     TB_COMMANDTOINDEX = 1049;
     TB_CUSTOMIZE = 1051;
     TB_DELETEBUTTON = 1046;
     TB_ENABLEBUTTON = 1025;
     TB_GETBITMAP = 1068;
     TB_GETBITMAPFLAGS = 1065;
     TB_GETBUTTON = 1047;
     TB_ADDSTRINGW = 1101;
     TB_GETBUTTONTEXTW = 1099;
     TB_SAVERESTOREW = 1100;
     TB_ADDSTRINGA = 1052;
     TB_GETBUTTONTEXTA = 1069;
     TB_SAVERESTOREA = 1050;
     TB_SETINDENT            = WM_USER + 47;
     TB_SETIMAGELIST         = WM_USER + 48;
     TB_GETIMAGELIST         = WM_USER + 49;
     TB_LOADIMAGES           = WM_USER + 50;
     TB_GETRECT              = WM_USER + 51;
     TB_SETDISABLEDIMAGELIST = WM_USER + 54;
     TB_GETDISABLEDIMAGELIST = WM_USER + 55;
     TB_SETSTYLE             = WM_USER + 56;
     TB_GETSTYLE             = WM_USER + 57;
     TB_GETBUTTONSIZE        = WM_USER + 58;
     TB_SETBUTTONWIDTH       = WM_USER + 59;
     TB_SETMAXTEXTROWS       = WM_USER + 60;
     TB_GETTEXTROWS          = WM_USER + 61;
{$ifdef UNICODE}

  const
     TB_ADDSTRING = TB_ADDSTRINGW;
     TB_GETBUTTONTEXT = TB_GETBUTTONTEXTW;
     TB_SAVERESTORE = TB_SAVERESTOREW;
{$else}

  const
     TB_ADDSTRING = TB_ADDSTRINGA;
     TB_GETBUTTONTEXT = TB_GETBUTTONTEXTA;
     TB_SAVERESTORE = TB_SAVERESTOREA;
{$endif}
  { UNICODE  }

  const
     TB_GETITEMRECT = 1053;
     TB_GETROWS = 1064;
     TB_GETSTATE = 1042;
     TB_GETTOOLTIPS = 1059;
     TB_HIDEBUTTON = 1028;
     TB_INDETERMINATE = 1029;
     TB_INSERTBUTTON = 1045;
     TB_ISBUTTONCHECKED = 1034;
     TB_ISBUTTONENABLED = 1033;
     TB_ISBUTTONHIDDEN = 1036;
     TB_ISBUTTONINDETERMINATE = 1037;
     TB_ISBUTTONPRESSED = 1035;
     TB_PRESSBUTTON = 1027;
     TB_SETBITMAPSIZE = 1056;
     TB_SETBUTTONSIZE = 1055;
     TB_SETCMDID = 1066;
     TB_SETPARENT = 1061;
     TB_SETROWS = 1063;
     TB_SETSTATE = 1041;
     TB_SETTOOLTIPS = WM_USER + 81;
  { Track bar control  }
     TBM_CLEARSEL = 1043;
     TBM_CLEARTICS = 1033;
     TBM_GETCHANNELRECT = 1050;
     TBM_GETLINESIZE = 1048;
     TBM_GETNUMTICS = 1040;
     TBM_GETPAGESIZE = 1046;
     TBM_GETPOS = 1024;
     TBM_GETPTICS = 1038;
     TBM_GETRANGEMAX = 1026;
     TBM_GETRANGEMIN = 1025;
     TBM_GETSELEND = 1042;
     TBM_GETSELSTART = 1041;
     TBM_GETTHUMBLENGTH = 1052;
     TBM_GETTHUMBRECT = 1049;
     TBM_GETTIC = 1027;
     TBM_GETTICPOS = 1039;
     TBM_SETLINESIZE = 1047;
     TBM_SETPAGESIZE = 1045;
     TBM_SETPOS = 1029;
     TBM_SETRANGE = 1030;
     TBM_SETRANGEMAX = 1032;
     TBM_SETRANGEMIN = 1031;
     TBM_SETSEL = 1034;
     TBM_SETSELEND = 1036;
     TBM_SETSELSTART = 1035;
     TBM_SETTHUMBLENGTH = 1051;
     TBM_SETTIC = 1028;
     TBM_SETTICFREQ = 1044;
  { Tool bar control notifications  }
     TBN_BEGINADJUST = -(703);
     TBN_BEGINDRAG = -(701);
     TBN_CUSTHELP = -(709);
     TBN_ENDADJUST = -(704);
     TBN_ENDDRAG = -(702);
     TBN_GETBUTTONINFOW = -(720);
     TBN_GETBUTTONINFOA = -(700);

{$ifdef UNICODE}

  const
     TBN_GETBUTTONINFO = TBN_GETBUTTONINFOW;
{$else}

  const
     TBN_GETBUTTONINFO = TBN_GETBUTTONINFOA;
{$endif}
  { UNICODE  }

  const
     TBN_QUERYDELETE = -(707);
     TBN_QUERYINSERT = -(706);
     TBN_RESET = -(705);
     TBN_TOOLBARCHANGE = -(708);
  { Tab control  }
     TCM_ADJUSTRECT = 4904;
     TCM_DELETEALLITEMS = 4873;
     TCM_DELETEITEM = 4872;
     TCM_GETCURFOCUS = 4911;
     TCM_GETCURSEL = 4875;
     TCM_GETIMAGELIST = 4866;
     TCM_GETITEMW = 4924;
     TCM_INSERTITEMW = 4926;
     TCM_SETITEMW = 4925;
     TCM_GETITEMA = 4869;
     TCM_INSERTITEMA = 4871;
     TCM_SETITEMA = 4870;

{$ifdef UNICODE}
  const
     TCM_GETITEM = TCM_GETITEMW;  //~wint, W was missing
     TCM_INSERTITEM = TCM_INSERTITEMW;
     TCM_SETITEM = TCM_SETITEMW;
{$else}

  const
     TCM_GETITEM = TCM_GETITEMA;
     TCM_INSERTITEM = TCM_INSERTITEMA;
     TCM_SETITEM = TCM_SETITEMA;
{$endif}
  { UNICODE  }

  const
     TCM_GETITEMCOUNT = 4868;
     TCM_GETITEMRECT = 4874;
     TCM_GETROWCOUNT = 4908;
     TCM_GETTOOLTIPS = 4909;
     TCM_HITTEST = 4877;
     TCM_REMOVEIMAGE = 4906;
     TCM_SETCURFOCUS = 4912;
     TCM_SETCURSEL = 4876;
     TCM_SETIMAGELIST = 4867;
     TCM_SETITEMEXTRA = 4878;
     TCM_SETITEMSIZE = 4905;
     TCM_SETPADDING = 4907;
     TCM_SETTOOLTIPS = 4910;
  { Tab control notifications  }
     TCN_KEYDOWN = -(550);
     TCN_SELCHANGE = -(551);
     TCN_SELCHANGING = -(552);
  { Tool tip control  }
     TTM_ACTIVATE = 1025;
     TTM_ADDTOOLW = 1074;
     TTM_DELTOOLW = 1075;
     TTM_ENUMTOOLSW = 1082;
     TTM_GETCURRENTTOOLW = 1083;
     TTM_GETTEXTW = 1080;
     TTM_GETTOOLINFOW = 1077;
     TTM_HITTESTW = 1079;
     TTM_NEWTOOLRECTW = 1076;
     TTM_SETTOOLINFOW = 1078;
     TTM_UPDATETIPTEXTW = 1081;
     TTM_ADDTOOLA = 1028;
     TTM_DELTOOLA = 1029;
     TTM_ENUMTOOLSA = 1038;
     TTM_GETCURRENTTOOLA = 1039;
     TTM_GETTEXTA = 1035;
     TTM_GETTOOLINFOA = 1032;
     TTM_HITTESTA = 1034;
     TTM_NEWTOOLRECTA = 1030;
     TTM_SETTOOLINFOA = 1033;
     TTM_UPDATETIPTEXTA = 1036;
{$ifdef UNICODE}

  const
     TTM_ADDTOOL = TTM_ADDTOOLW;
     TTM_DELTOOL = TTM_DELTOOLW;
     TTM_ENUMTOOLS = TTM_ENUMTOOLSW;
     TTM_GETCURRENTTOOL = TTM_GETCURRENTTOOLW;
     TTM_GETTEXT = TTM_GETTEXTW;
     TTM_GETTOOLINFO = TTM_GETTOOLINFOW;
     TTM_HITTEST = TTM_HITTESTW;
     TTM_NEWTOOLRECT = TTM_NEWTOOLRECTW;
     TTM_SETTOOLINFO = TTM_SETTOOLINFOW;
     TTM_UPDATETIPTEXT = TTM_UPDATETIPTEXTW;
{$else}

  const
     TTM_ADDTOOL = TTM_ADDTOOLA;
     TTM_DELTOOL = TTM_DELTOOLA;
     TTM_ENUMTOOLS = TTM_ENUMTOOLSA;
     TTM_GETCURRENTTOOL = TTM_GETCURRENTTOOLA;
     TTM_GETTEXT = TTM_GETTEXTA;
     TTM_GETTOOLINFO = TTM_GETTOOLINFOA;
     TTM_HITTEST = TTM_HITTESTA;
     TTM_NEWTOOLRECT = TTM_NEWTOOLRECTA;
     TTM_SETTOOLINFO = TTM_SETTOOLINFOA;
     TTM_UPDATETIPTEXT = TTM_UPDATETIPTEXTA;
{$endif}
  { UNICODE  }

  const
     TTM_GETTOOLCOUNT = 1037;
     TTM_RELAYEVENT = 1031;
     TTM_SETDELAYTIME = 1027;
     TTM_WINDOWFROMPOINT = 1040;
  { UNICODE  }

  const
     TTN_POP = -(522);
     TTN_SHOW = -(521);
  { Tree view control  }
     TVM_CREATEDRAGIMAGE = 4370;
     TVM_DELETEITEM = 4353;
     TVM_ENDEDITLABELNOW = 4374;
     TVM_ENSUREVISIBLE = 4372;
     TVM_EXPAND = 4354;
     TVM_GETCOUNT = 4357;
     TVM_GETEDITCONTROL = 4367;
     TVM_GETIMAGELIST = 4360;
     TVM_GETINDENT = 4358;
     TVM_GETITEMRECT = 4356;
     TVM_GETNEXTITEM = 4362;
     TVM_GETVISIBLECOUNT = 4368;
     TVM_HITTEST = 4369;
     TVM_EDITLABELW = 4417;
     TVM_GETISEARCHSTRINGW = 4416;
     TVM_GETITEMW = 4414;
     TVM_INSERTITEMW = 4402;
     TVM_SETITEMW = 4415;
     TVM_EDITLABELA = 4366;
     TVM_GETISEARCHSTRINGA = 4375;
     TVM_GETITEMA = 4364;
     TVM_INSERTITEMA = 4352;
     TVM_SETITEMA = 4365;
{$ifdef UNICODE}

  const
     TVM_EDITLABEL = TVM_EDITLABELW;
     TVM_GETISEARCHSTRING = TVM_GETISEARCHSTRINGW;
     TVM_GETITEM = TVM_GETITEMW;
     TVM_INSERTITEM = TVM_INSERTITEMW;
     TVM_SETITEM = TVM_SETITEMW;
{$else}

  const
     TVM_EDITLABEL = TVM_EDITLABELA;
     TVM_GETISEARCHSTRING = TVM_GETISEARCHSTRINGA;
     TVM_GETITEM = TVM_GETITEMA;
     TVM_INSERTITEM = TVM_INSERTITEMA;
     TVM_SETITEM = TVM_SETITEMA;
{$endif}
  { UNICODE  }

  const
     TVM_SELECTITEM = 4363;
     TVM_SETIMAGELIST = 4361;
     TVM_SETINDENT = 4359;
     TVM_SORTCHILDREN = 4371;
     TVM_SORTCHILDRENCB = 4373;
  { Tree view control notification  }
     TVN_KEYDOWN = -(412);
     TVN_BEGINDRAGW = -(456);
     TVN_BEGINLABELEDITW = -(459);
     TVN_BEGINRDRAGW = -(457);
     TVN_DELETEITEMW = -(458);
     TVN_ENDLABELEDITW = -(460);
     TVN_GETDISPINFOW = -(452);
     TVN_ITEMEXPANDEDW = -(455);
     TVN_ITEMEXPANDINGW = -(454);
     TVN_SELCHANGEDW = -(451);
     TVN_SELCHANGINGW = -(450);
     TVN_SETDISPINFOW = -(453);
     TVN_BEGINDRAGA = -(407);
     TVN_BEGINLABELEDITA = -(410);
     TVN_BEGINRDRAGA = -(408);
     TVN_DELETEITEMA = -(409);
     TVN_ENDLABELEDITA = -(411);
     TVN_GETDISPINFOA = -(403);
     TVN_ITEMEXPANDEDA = -(406);
     TVN_ITEMEXPANDINGA = -(405);
     TVN_SELCHANGEDA = -(402);
     TVN_SELCHANGINGA = -(401);
     TVN_SETDISPINFOA = -(404);
{$ifdef UNICODE}

  const
     TVN_BEGINDRAG = TVN_BEGINDRAGW;
     TVN_BEGINLABELEDIT = TVN_BEGINLABELEDITW;
     TVN_BEGINRDRAG = TVN_BEGINRDRAGW;
     TVN_DELETEITEM = TVN_DELETEITEMW;
     TVN_ENDLABELEDIT = TVN_ENDLABELEDITW;
     TVN_GETDISPINFO = TVN_GETDISPINFOW;
     TVN_ITEMEXPANDED = TVN_ITEMEXPANDEDW;
     TVN_ITEMEXPANDING = TVN_ITEMEXPANDINGW;
     TVN_SELCHANGED = TVN_SELCHANGEDW;
     TVN_SELCHANGING = TVN_SELCHANGINGW;
     TVN_SETDISPINFO = TVN_SETDISPINFOW;
{$else}

  const
     TVN_BEGINDRAG = TVN_BEGINDRAGA;
     TVN_BEGINLABELEDIT = TVN_BEGINLABELEDITA;
     TVN_BEGINRDRAG = TVN_BEGINRDRAGA;
     TVN_DELETEITEM = TVN_DELETEITEMA;
     TVN_ENDLABELEDIT = TVN_ENDLABELEDITA;
     TVN_GETDISPINFO = TVN_GETDISPINFOA;
     TVN_ITEMEXPANDED = TVN_ITEMEXPANDEDA;
     TVN_ITEMEXPANDING = TVN_ITEMEXPANDINGA;
     TVN_SELCHANGED = TVN_SELCHANGEDA;
     TVN_SELCHANGING = TVN_SELCHANGINGA;
     TVN_SETDISPINFO = TVN_SETDISPINFOA;
{$endif}
  { UNICODE  }
  { Up/down control  }

  const
     UDM_GETACCEL = 1132;
     UDM_GETBASE = 1134;
     UDM_GETBUDDY = 1130;
     UDM_GETPOS = 1128;
     UDM_GETPOS32 = 1138;
     UDM_GETRANGE = 1126;
     UDM_GETRANGE32 = 1136;
     UDM_SETACCEL = 1131;
     UDM_SETBASE = 1133;
     UDM_SETBUDDY = 1129;
     UDM_SETPOS = 1127;
     UDM_SETPOS32 = 1137;
     UDM_SETRANGE = 1125;
     UDM_SETRANGE32 = 1135;
  { Up/down control notification  }
     UDN_DELTAPOS = -(722);
  { Window messages  }
     WM_ACTIVATE = 6;
     WM_ACTIVATEAPP = 28;
     WM_ASKCBFORMATNAME = 780;
     WM_CANCELJOURNAL = 75;
     WM_CANCELMODE = 31;
     WM_CAPTURECHANGED = 533;
     WM_CHANGECBCHAIN = 781;
     WM_CHAR = 258;
     WM_CHARTOITEM = 47;
     WM_CHILDACTIVATE = 34;
     WM_CHOOSEFONT_GETLOGFONT = 1025;
     WM_CHOOSEFONT_SETLOGFONT = 1125;
     WM_CHOOSEFONT_SETFLAGS = 1126;
     WM_CLEAR = 771;
     WM_CLOSE = 16;
     WM_COMMAND = 273;
     WM_COMPACTING = 65;
     WM_COMPAREITEM = 57;
     WM_CONTEXTMENU = 123;
     WM_COPY = 769;
     WM_COPYDATA = 74;
     WM_CREATE = 1;
     WM_CTLCOLORBTN = 309;
     WM_CTLCOLORDLG = 310;
     WM_CTLCOLOREDIT = 307;
     WM_CTLCOLORLISTBOX = 308;
     WM_CTLCOLORMSGBOX = 306;
     WM_CTLCOLORSCROLLBAR = 311;
     WM_CTLCOLORSTATIC = 312;
     WM_CUT = 768;
     WM_DEADCHAR = 259;
     WM_DELETEITEM = 45;
     WM_DESTROY = 2;
     WM_DESTROYCLIPBOARD = 775;
     WM_DEVICECHANGE = 537;
     WM_DEVMODECHANGE = 27;
     WM_DISPLAYCHANGE = 126;
     WM_DRAWCLIPBOARD = 776;
     WM_DRAWITEM = 43;
     WM_DROPFILES = 563;
     WM_ENABLE = 10;
     WM_ENDSESSION = 22;
     WM_ENTERIDLE = 289;
     WM_ENTERMENULOOP = 529;
     WM_ENTERSIZEMOVE = 561;
     WM_ERASEBKGND = 20;
     WM_EXITMENULOOP = 530;
     WM_EXITSIZEMOVE = 562;
     WM_FONTCHANGE = 29;
     WM_GETDLGCODE = 135;
     WM_GETFONT = 49;
     WM_GETHOTKEY = 51;
     WM_GETICON = 127;
     WM_GETMINMAXINFO = 36;
     WM_GETTEXT = 13;
     WM_GETTEXTLENGTH = 14;
     WM_HELP = 83;
     WM_HOTKEY = 786;
     WM_HSCROLL = 276;
     WM_HSCROLLCLIPBOARD = 782;
     WM_ICONERASEBKGND = 39;
     WM_IME_CHAR = 646;
     WM_IME_COMPOSITION = 271;
<<<<<<< HEAD
<<<<<<< HEAD
     WM_IME_CONTROL = 643;
     WM_IME_COMPOSITIONFULL = 644;
     WM_IME_REQUEST = $0288;
=======
     WM_IME_COMPOSITIONFULL = 644;
     WM_IME_CONTROL = 643;
>>>>>>> graemeg/fixes_2_2
=======
     WM_IME_COMPOSITIONFULL = 644;
     WM_IME_CONTROL = 643;
>>>>>>> origin/fixes_2_2
     WM_IME_ENDCOMPOSITION = 270;
     WM_IME_KEYDOWN = 656;
     WM_IME_KEYUP = 657;
     WM_IME_NOTIFY = 642;
     WM_IME_SELECT = 645;
     WM_IME_SETCONTEXT = 641;
     WM_IME_STARTCOMPOSITION = 269;
     WM_INITDIALOG = 272;
     WM_INITMENU = 278;
     WM_INITMENUPOPUP = 279;
     WM_INPUTLANGCHANGE = 81;
     WM_INPUTLANGCHANGEREQUEST = 80;
     WM_KEYDOWN = 256;
     WM_KEYUP = 257;
     WM_KILLFOCUS = 8;
     WM_LBUTTONDBLCLK = 515;
     WM_LBUTTONDOWN = 513;
     WM_LBUTTONUP = 514;
     WM_MBUTTONDBLCLK = 521;
     WM_MBUTTONDOWN = 519;
     WM_MBUTTONUP = 520;
     WM_MDIACTIVATE = 546;
     WM_MDICASCADE = 551;
     WM_MDICREATE = 544;
     WM_MDIDESTROY = 545;
     WM_MDIGETACTIVE = 553;
     WM_MDIICONARRANGE = 552;
     WM_MDIMAXIMIZE = 549;
     WM_MDINEXT = 548;
     WM_MDIREFRESHMENU = 564;
     WM_MDIRESTORE = 547;
     WM_MDISETMENU = 560;
     WM_MDITILE = 550;
     WM_MEASUREITEM = 44;
     WM_MENUCHAR = 288;
     WM_MENUSELECT = 287;
     WM_MOUSEACTIVATE = 33;
     WM_MOUSEMOVE = 512;
     WM_MOUSEWHEEL = 522;
     WM_MOUSEHOVER = 673;
     WM_MOUSELEAVE = 675;
     WM_MOVE = 3;
     WM_MOVING = 534;
     WM_NCACTIVATE = 134;
     WM_NCCALCSIZE = 131;
     WM_NCCREATE = 129;
     WM_NCDESTROY = 130;
     WM_NCHITTEST = 132;
     WM_NCLBUTTONDBLCLK = 163;
     WM_NCLBUTTONDOWN = 161;
     WM_NCLBUTTONUP = 162;
     WM_NCMBUTTONDBLCLK = 169;
     WM_NCMBUTTONDOWN = 167;
     WM_NCMBUTTONUP = 168;
     WM_NCMOUSEMOVE = 160;
     WM_NCPAINT = 133;
     WM_NCRBUTTONDBLCLK = 166;
     WM_NCRBUTTONDOWN = 164;
     WM_NCRBUTTONUP = 165;
     WM_NEXTDLGCTL = 40;
     WM_NOTIFY = 78;
     WM_NOTIFYFORMAT = 85;
     WM_NULL = 0;
     WM_PAINT = 15;
     WM_PAINTCLIPBOARD = 777;
     WM_PAINTICON = 38;
     WM_PALETTECHANGED = 785;
     WM_PALETTEISCHANGING = 784;
     WM_PARENTNOTIFY = 528;
     WM_PASTE = 770;
     WM_PENWINFIRST = 896;
     WM_PENWINLAST = 911;
     WM_POWER = 72;
     WM_POWERBROADCAST = 536;
     WM_PRINT = 791;
     WM_PRINTCLIENT = 792;
     WM_PSD_ENVSTAMPRECT = 1029;
     WM_PSD_FULLPAGERECT = 1025;
     WM_PSD_GREEKTEXTRECT = 1028;
     WM_PSD_MARGINRECT = 1027;
     WM_PSD_MINMARGINRECT = 1026;
     WM_PSD_PAGESETUPDLG = 1024;
     WM_PSD_YAFULLPAGERECT = 1030;
     WM_QUERYDRAGICON = 55;
     WM_QUERYENDSESSION = 17;
     WM_QUERYNEWPALETTE = 783;
     WM_QUERYOPEN = 19;
     WM_QUEUESYNC = 35;
     WM_QUIT = 18;
     WM_RBUTTONDBLCLK = 518;
     WM_RBUTTONDOWN = 516;
     WM_RBUTTONUP = 517;
     WM_RENDERALLFORMATS = 774;
     WM_RENDERFORMAT = 773;
     WM_SETCURSOR = 32;
     WM_SETFOCUS = 7;
     WM_SETFONT = 48;
     WM_SETHOTKEY = 50;
     WM_SETICON = 128;
     WM_SETREDRAW = 11;
     WM_SETTEXT = 12;
     WM_SETTINGCHANGE = 26;
     WM_SHOWWINDOW = 24;
     WM_SIZE = 5;
     WM_SIZECLIPBOARD = 779;
     WM_SIZING = 532;
     WM_SPOOLERSTATUS = 42;
     WM_STYLECHANGED = 125;
     WM_STYLECHANGING = 124;
     WM_SYSCHAR = 262;
     WM_SYSCOLORCHANGE = 21;
     WM_SYSCOMMAND = 274;
     WM_SYSDEADCHAR = 263;
     WM_SYSKEYDOWN = 260;
     WM_SYSKEYUP = 261;
     WM_TCARD = 82;
     WM_TIMECHANGE = 30;
     WM_TIMER = 275;
     WM_UNDO = 772;
     //WM_USER = 1024; moved at start
     WM_USERCHANGED = 84;
     WM_VKEYTOITEM = 46;
     WM_VSCROLL = 277;
     WM_VSCROLLCLIPBOARD = 778;
     WM_WINDOWPOSCHANGED = 71;
     WM_WINDOWPOSCHANGING = 70;
     WM_WININICHANGE = 26;
  { Window message ranges  }
     WM_KEYFIRST = 256;
     WM_KEYLAST = 264;
     WM_MOUSEFIRST = 512;
     WM_MOUSELAST     = 525;
     WM_XBUTTONDOWN   = 523;
     WM_XBUTTONUP     = 524;
     WM_XBUTTONDBLCLK = 525;

<<<<<<< HEAD
<<<<<<< HEAD
// winuserm.h
// Mobile extensions to winuser.h

// Edit control extensions for Mobile
const
     // If lParam is FALSE, gets default input mode.
     // If lParam is TRUE, gets actual input mode if control has focus, else
     // gets default input mode. Returns EIM_*|EIMMF_*
      EM_GETINPUTMODE       = $00DD;

     // Sets default input mode when control gets focus. lParam should be be EIM_*|EIMMF_*.
      EM_SETINPUTMODE       = $00DE;

     // Sets symbols that are displayed when the user presses the 1 key in
     // Multipress mode. lParam is a null-teminated string containing the symbols
     // or NULL to revert to standard symbols.
      EM_SETSYMBOLS         = $00DF;

     // Sets extended styles. wParam = Mask. lParam = Extended styles. Returns old styles.
      EM_SETEXTENDEDSTYLE   = $00E0;

     // Gets extended styles.
      EM_GETEXTENDEDSTYLE   = $00E1;


// Listbox control extensions for Mobile. Supported only for single selection listboxes.
const
     // If lParam is FALSE, gets default input mode.
     // If lParam is TRUE, gets actual input mode if control has focus, else
     // gets default input mode. Returns LIM_*
      LB_GETINPUTMODE       = $01C0;

     // Sets default input mode when control gets focus. lParam should be be LIM_*.
      LB_SETINPUTMODE       = $01C1;

// Dialog message extensions
//
// Reset scroll info. To be sent by a dialog if it does a relayout of child
// controls after WM_INITDIALOG. Supported only if the dialog returns DLGC_SCROLL
// on WM_GETDLGCODE.
// wParam: TRUE to scroll back to top if user scrolled dialog. FALSE to leave
// dialog in position that user scrolled to.
// lParam: TRUE to recompute scroll information. FALSE to use previously computed
// scroll information.
const
      DM_RESETSCROLL          = WM_USER + 2;

// End-of-winuserm.h

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Type
{$ifdef MESSAGESUNIT}

    MSG                 = Windows.MSG;
    TMessage            = Windows.TMessage;
    TWMSize             = Windows.TWMSize;
    TWMNoParams         = Windows.TWMNoParams;
    TWMScroll           = Windows.TWMScroll;
    TWMGetText          = Windows.TWMGetText;
    TWMKillFocus        = Windows.TWMKillFocus;
    TWMSetCursor        = Windows.TWMSetCursor;
    TWMSetFocus         = Windows.TWMSetFocus;
    TWMSetFont          = Windows.TWMSetFont;
    TWMShowWindow       = Windows.TWMShowWindow;
    TWMEraseBkgnd       = Windows.TWMEraseBkgnd;
    LPMSG               = ^Windows.MSG;
    tagMSG              = Windows.tagMSG;
    TMSG                = Windows.TMSG;
    PMSG                = Windows.PMSG;
    PMessage            = Windows.PMessage;
    TWMGetDlgCode       = TWMNoParams;
    TWMFontChange       = TWMNoParams;
    TWMGetFont          = TWMNoParams;
    TWMHScroll          = TWMScroll;
    TWMVScroll          = TWMScroll;
    TWMGetTextLength    = TWMNoParams;

{$else}

     MSG = record
          hwnd : HWND;
          message : UINT;
          wParam : WPARAM;
          lParam : LPARAM;
          time : DWORD;
          pt : POINT;
       end;

     LPMSG = ^MSG;
     tagMSG = MSG;
     TMSG = MSG;
     PMSG = ^MSG;


     PMessage = ^TMessage;
     TMessage = record       {fields according to ICS}
        msg      : UINT;
        case longint of
          0: (
              wParam : WPARAM;
              lParam : LPARAM;
              Result : LRESULT;
             );
          1: (
              wParamlo,
              wParamhi : WORD;  // Is there Windows type for half an wparam?
              lParamlo,
              lParamhi : WORD;
              Resultlo,
              Resulthi : WORD;
             );
       end;

    TWMSize = record
      Msg: Cardinal;
      SizeType : LongInt;
      Width : Word;
      Height : Word;
      Result : LongInt;
    End;

    TWMNoParams = record
      Msg : Cardinal;
      Unused : array[0..3] of Word;
      Result : Longint;
    end;

    TWMGetDlgCode = TWMNoParams;
    TWMFontChange = TWMNoParams;
    TWMGetFont = TWMNoParams;

    TWMScroll = record
      Msg : Cardinal;
      ScrollCode : SmallInt;
      Pos : SmallInt;
      ScrollBar : HWND;
      Result : LongInt;
    end;

    TWMHScroll = TWMScroll;
    TWMVScroll = TWMScroll;

    TWMGetText = record
      Msg : Cardinal;
      TextMax : LongInt;
      Text : PChar;
      Result : LongInt;
    end;

    TWMGetTextLength = TWMNoParams;

    TWMKillFocus = record
       Msg : Cardinal;
       FocusedWnd : HWND;
       UnUsed : LongInt;
       Result : LongInt;
    End;

    TWMSetCursor = record
      Msg : Cardinal;
      CursorWnd : HWND;
      HitTest : Word;
      MouseMsg : Word;
      Result : LongInt;
    end;

    TWMSetFocus = record
      Msg : Cardinal;
      FocusedWnd : HWND;
      Unused : LongInt;
      Result : LongInt;
    end;

    TWMSetFont = record
      Msg : Cardinal;
      Font : HFONT;
      Redraw : WordBool;
      Unused : Word;
      Result : LongInt;
    end;

    TWMShowWindow = record
      Msg : Cardinal;
      Show : BOOL;
      Status : LongInt;
      Result : LongInt;
    end;

    TWMEraseBkgnd = record
      Msg: Cardinal;
      DC: HDC;
      Unused: Longint;
      Result: Longint;
    end;

{$endif messagesunit}

{$endif read_interface}

