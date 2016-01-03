{ Converted by H2Pas from richedit.h }
unit richedit;

{$mode objfpc}
{$calling stdcall}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_OS_UNICODE}
  {$define UNICODE}
{$endif}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
interface

uses Messages, Windows;

{$IFDEF FPC}
{$PACKRECORDS C}
{$ENDIF}

{$ifdef UNICODE }
  const
     RICHEDIT_CLASS = 'RichEdit20W';
{$else}
  const
     RICHEDIT_CLASS = 'RichEdit20A';
{$endif}

  const
     RICHEDIT_CLASS10A = 'RICHEDIT';
     CF_RTF = 'Rich Text Format';
     CF_RTFNOOBJS = 'Rich Text Format Without Objects';
     CF_RETEXTOBJ = 'RichEdit Text and Objects';
     CFM_BOLD = 1;
     CFM_ITALIC = 2;
     CFM_UNDERLINE = 4;
     CFM_STRIKEOUT = 8;
     CFM_PROTECTED = 16;
     CFM_LINK = 32;
     CFM_SIZE = $80000000;
     CFM_COLOR = $40000000;
     CFM_FACE = $20000000;
     CFM_OFFSET = $10000000;
     CFM_CHARSET = $08000000;
     CFM_SUBSCRIPT = $00030000;
     CFM_SUPERSCRIPT = $00030000;
     CFM_EFFECTS = (((((CFM_BOLD or CFM_ITALIC) or CFM_UNDERLINE) or CFM_COLOR) or CFM_STRIKEOUT) or CFE_PROTECTED) or CFM_LINK;
     CFE_BOLD = 1;
     CFE_ITALIC = 2;
     CFE_UNDERLINE = 4;
     CFE_STRIKEOUT = 8;
     CFE_PROTECTED = 16;
     CFE_AUTOCOLOR = $40000000;
     CFE_SUBSCRIPT = $00010000;
     CFE_SUPERSCRIPT = $00020000;
     IMF_FORCENONE = 1;
     IMF_FORCEENABLE = 2;
     IMF_FORCEDISABLE = 4;
     IMF_CLOSESTATUSWINDOW = 8;
     IMF_VERTICAL = 32;
     IMF_FORCEACTIVE = 64;
     IMF_FORCEINACTIVE = 128;
     IMF_FORCEREMEMBER = 256;
     SEL_EMPTY = 0;
     SEL_TEXT = 1;
     SEL_OBJECT = 2;
     SEL_MULTICHAR = 4;
     SEL_MULTIOBJECT = 8;
     MAX_TAB_STOPS = 32;
     PFM_ALIGNMENT = 8;
     PFM_NUMBERING = 32;
     PFM_OFFSET = 4;
     PFM_OFFSETINDENT = $80000000;
     PFM_RIGHTINDENT = 2;
     PFM_STARTINDENT = 1;
     PFM_TABSTOPS = 16;
     PFM_BORDER = 2048;
     PFM_LINESPACING = 256;
     PFM_NUMBERINGSTART = 32768;
     PFM_NUMBERINGSTYLE = 8192;
     PFM_NUMBERINGTAB = 16384;
     PFM_SHADING = 4096;
     PFM_SPACEAFTER = 128;
     PFM_SPACEBEFORE = 64;
     PFM_STYLE = 1024;
     PFM_DONOTHYPHEN = 4194304;
     PFM_KEEP = 131072;
     PFM_KEEPNEXT = 262144;
     PFM_NOLINENUMBER = 1048576;
     PFM_NOWIDOWCONTROL = 2097152;
     PFM_PAGEBREAKBEFORE = 524288;
     PFM_RTLPARA = 65536;
     PFM_SIDEBYSIDE = 8388608;
     PFM_TABLE = 1073741824;
     PFN_BULLET = 1;
     PFE_DONOTHYPHEN = 64;
     PFE_KEEP = 2;
     PFE_KEEPNEXT = 4;
     PFE_NOLINENUMBER = 16;
     PFE_NOWIDOWCONTROL = 32;
     PFE_PAGEBREAKBEFORE = 8;
     PFE_RTLPARA = 1;
     PFE_SIDEBYSIDE = 128;
     PFE_TABLE = 16384;
     PFA_LEFT = 1;
     PFA_RIGHT = 2;
     PFA_CENTER = 3;
     PFA_JUSTIFY = 4;
     PFA_FULL_INTERWORD = 4;
     SF_TEXT = 1;
     SF_RTF = 2;
     SF_RTFNOOBJS = 3;
     SF_TEXTIZED = 4;
     SF_UNICODE = 16;
     SF_USECODEPAGE = 32;
     SF_NCRFORNONASCII = 64;
     SF_RTFVAL = $0700;
     SFF_PWD = $0800;
     SFF_KEEPDOCINFO = $1000;
     SFF_PERSISTVIEWSCALE = $2000;
     SFF_PLAINRTF = $4000;
     SFF_SELECTION = $8000;
     WB_CLASSIFY = 3;
     WB_MOVEWORDLEFT = 4;
     WB_MOVEWORDRIGHT = 5;
     WB_LEFTBREAK = 6;
     WB_RIGHTBREAK = 7;
     WB_MOVEWORDPREV = 4;
     WB_MOVEWORDNEXT = 5;
     WB_PREVBREAK = 6;
     WB_NEXTBREAK = 7;
     WBF_WORDWRAP = 16;
     WBF_WORDBREAK = 32;
     WBF_OVERFLOW = 64;
     WBF_LEVEL1 = 128;
     WBF_LEVEL2 = 256;
     WBF_CUSTOM = 512;
     ES_DISABLENOSCROLL = 8192;
     ES_EX_NOCALLOLEINIT = 16777216;
     ES_NOIME = 524288;
     ES_NOOLEDRAGDROP = 8;
     ES_SAVESEL = 32768;
     ES_SELECTIONBAR = 16777216;
     ES_SELFIME = 262144;
     ES_SUNKEN = 16384;
     ES_VERTICAL = 4194304;
     EM_CANPASTE = WM_USER+50;
     EM_DISPLAYBAND = WM_USER+51;
     EM_EXGETSEL = WM_USER+52;
     EM_EXLIMITTEXT = WM_USER+53;
     EM_EXLINEFROMCHAR = WM_USER+54;
     EM_EXSETSEL = WM_USER+55;
     EM_FINDTEXT = WM_USER+56;
     EM_FORMATRANGE = WM_USER+57;
     EM_GETCHARFORMAT = WM_USER+58;
     EM_GETEVENTMASK = WM_USER+59;
     EM_GETOLEINTERFACE = WM_USER+60;
     EM_GETPARAFORMAT = WM_USER+61;
     EM_GETSELTEXT = WM_USER+62;
     EM_HIDESELECTION = WM_USER+63;
     EM_PASTESPECIAL = WM_USER+64;
     EM_REQUESTRESIZE = WM_USER+65;
     EM_SELECTIONTYPE = WM_USER+66;
     EM_SETBKGNDCOLOR = WM_USER+67;
     EM_SETCHARFORMAT = WM_USER+68;
     EM_SETEVENTMASK = WM_USER+69;
     EM_SETOLECALLBACK = WM_USER+70;
     EM_SETPARAFORMAT = WM_USER+71;
     EM_SETTARGETDEVICE = WM_USER+72;
     EM_STREAMIN = WM_USER+73;
     EM_STREAMOUT = WM_USER+74;
     EM_GETTEXTRANGE = WM_USER+75;
     EM_FINDWORDBREAK = WM_USER+76;
     EM_SETOPTIONS = WM_USER+77;
     EM_GETOPTIONS = WM_USER+78;
     EM_FINDTEXTEX = WM_USER+79;
     EM_GETWORDBREAKPROCEX = WM_USER+80;
     EM_SETWORDBREAKPROCEX = WM_USER+81;
  { RichEdit 2.0 messages  }
     EM_SETUNDOLIMIT = WM_USER+82;
     EM_REDO = WM_USER+84;
     EM_CANREDO = WM_USER+85;
     EM_GETUNDONAME = WM_USER+86;
     EM_GETREDONAME = WM_USER+87;
     EM_STOPGROUPTYPING = WM_USER+88;
     EM_SETTEXTMODE = WM_USER+89;
     EM_GETTEXTMODE = WM_USER+90;
     EM_AUTOURLDETECT = WM_USER+91;
     EM_GETAUTOURLDETECT = WM_USER+92;
     EM_SETPALETTE = WM_USER+93;
     EM_GETTEXTEX = WM_USER+94;
     EM_GETTEXTLENGTHEX = WM_USER+95;
     EM_SHOWSCROLLBAR = WM_USER+96;
     EM_SETTEXTEX = WM_USER+97;
     EM_SETPUNCTUATION = WM_USER+100;
     EM_GETPUNCTUATION = WM_USER+101;
     EM_SETWORDWRAPMODE = WM_USER+102;
     EM_GETWORDWRAPMODE = WM_USER+103;
     EM_SETIMECOLOR = WM_USER+104;
     EM_GETIMECOLOR = WM_USER+105;
     EM_SETIMEOPTIONS = WM_USER+106;
     EM_GETIMEOPTIONS = WM_USER+107;
     EM_SETLANGOPTIONS = WM_USER+120;
     EM_GETLANGOPTIONS = WM_USER+121;
     EM_GETIMECOMPMODE = WM_USER+122;
     EM_FINDTEXTW = WM_USER+123;
     EM_FINDTEXTEXW = WM_USER+124;
     EM_RECONVERSION = WM_USER+125;
     EM_SETBIDIOPTIONS = WM_USER+200;
     EM_GETBIDIOPTIONS = WM_USER+201;
     EM_SETTYPOGRAPHYOPTIONS = WM_USER+202;
     EM_GETTYPOGRAPHYOPTIONS = WM_USER+203;
     EM_SETEDITSTYLE = WM_USER+204;
     EM_GETEDITSTYLE = WM_USER+205;
     EM_GETSCROLLPOS = WM_USER+221;
     EM_SETSCROLLPOS = WM_USER+222;
     EM_SETFONTSIZE = WM_USER+223;
     EM_GETZOOM = WM_USER+224;
     EM_SETZOOM = WM_USER+225;
     EN_CORRECTTEXT = 1797;
     EN_DROPFILES = 1795;
     EN_IMECHANGE = 1799;
     EN_LINK = 1803;
     EN_MSGFILTER = 1792;
     EN_OLEOPFAILED = 1801;
     EN_PROTECTED = 1796;
     EN_REQUESTRESIZE = 1793;
     EN_SAVECLIPBOARD = 1800;
     EN_SELCHANGE = 1794;
     EN_STOPNOUNDO = 1798;
     ENM_NONE = 0;
     ENM_CHANGE = 1;
     ENM_CORRECTTEXT = 4194304;
     ENM_DRAGDROPDONE = 16;
     ENM_DROPFILES = 1048576;
     ENM_IMECHANGE = 8388608;
     ENM_KEYEVENTS = 65536;
     ENM_LANGCHANGE = 16777216;
     ENM_LINK = 67108864;
     ENM_MOUSEEVENTS = 131072;
     ENM_OBJECTPOSITIONS = 33554432;
     ENM_PROTECTED = 2097152;
     ENM_REQUESTRESIZE = 262144;
     ENM_SCROLL = 4;
     ENM_SCROLLEVENTS = 8;
     ENM_SELCHANGE = 524288;
     ENM_UPDATE = 2;
     ECO_AUTOWORDSELECTION = 1;
     ECO_AUTOVSCROLL = 64;
     ECO_AUTOHSCROLL = 128;
     ECO_NOHIDESEL = 256;
     ECO_READONLY = 2048;
     ECO_WANTRETURN = 4096;
     ECO_SAVESEL = $8000;
     ECO_SELECTIONBAR = $1000000;
     ECO_VERTICAL = $400000;
     ECOOP_SET = 1;
     ECOOP_OR = 2;
     ECOOP_AND = 3;
     ECOOP_XOR = 4;
     SCF_DEFAULT = 0;
     SCF_SELECTION = 1;
     SCF_WORD = 2;
     SCF_ALL = 4;
     SCF_USEUIRULES = 8;
     TM_PLAINTEXT = 1;
     TM_RICHTEXT = 2;
     TM_SINGLELEVELUNDO = 4;
     TM_MULTILEVELUNDO = 8;
     TM_SINGLECODEPAGE = 16;
     TM_MULTICODEPAGE = 32;
     GT_DEFAULT = 0;
     GT_USECRLF = 1;
     yHeightCharPtsMost = 1638;
     lDefaultTab = 720;
     FT_MATCHCASE = 4;
     FT_WHOLEWORD = 2;

  type

     _charformat = record
          cbSize : UINT;
          dwMask : DWORD;
          dwEffects : DWORD;
          yHeight : LONG;
          yOffset : LONG;
          crTextColor : COLORREF;
          bCharSet : BYTE;
          bPitchAndFamily : BYTE;
          szFaceName : array[0..(LF_FACESIZE)-1] of char;
       end;
     CHARFORMATA = _charformat;
     TCHARFORMATA = _charformat;

     _charformatw = record
          cbSize : UINT;
          dwMask : DWORD;
          dwEffects : DWORD;
          yHeight : LONG;
          yOffset : LONG;
          crTextColor : COLORREF;
          bCharSet : BYTE;
          bPitchAndFamily : BYTE;
          szFaceName : array[0..(LF_FACESIZE)-1] of WCHAR;
       end;
     CHARFORMATW = _charformatw;
     TCHARFORMATW = _charformatw;

     _charformat2a = record
          cbSize : UINT;
          dwMask : DWORD;
          dwEffects : DWORD;
          yHeight : LONG;
          yOffset : LONG;
          crTextColor : COLORREF;
          bCharSet : BYTE;
          bPitchAndFamily : BYTE;
          szFaceName : array[0..(LF_FACESIZE)-1] of char;
          wWeight : WORD;
          sSpacing : SHORT;
          crBackColor : COLORREF;
          lcid : LCID;
          dwReserved : DWORD;
          sStyle : SHORT;
          wKerning : WORD;
          bUnderlineType : BYTE;
          bAnimation : BYTE;
          bRevAuthor : BYTE;
       end;
     CHARFORMAT2A = _charformat2a;
     TCHARFORMAT2A = _charformat2a;

     _charformat2w = record
          cbSize : UINT;
          dwMask : DWORD;
          dwEffects : DWORD;
          yHeight : LONG;
          yOffset : LONG;
          crTextColor : COLORREF;
          bCharSet : BYTE;
          bPitchAndFamily : BYTE;
          szFaceName : array[0..(LF_FACESIZE)-1] of WCHAR;
          wWeight : WORD;
          sSpacing : SHORT;
          crBackColor : COLORREF;
          lcid : LCID;
          dwReserved : DWORD;
          sStyle : SHORT;
          wKerning : WORD;
          bUnderlineType : BYTE;
          bAnimation : BYTE;
          bRevAuthor : BYTE;
       end;
     CHARFORMAT2W = _charformat2w;
     TCHARFORMAT2W = _charformat2w;

     _charrange = record
          cpMin : LONG;
          cpMax : LONG;
       end;
     CHARRANGE = _charrange;
     TCHARRANGE = _charrange;

     _compcolor = record
          crText : COLORREF;
          crBackground : COLORREF;
          dwEffects : DWORD;
       end;
     COMPCOLOR = _compcolor;
     TCOMPCOLOR = _compcolor;

     EDITSTREAMCALLBACK = function (dwCookie:PDWORD; pbBuff:LPBYTE; cb:LONG; var pcb:LONG):DWORD;

     _editstream = record
<<<<<<< HEAD
<<<<<<< HEAD
          dwCookie : DWORD_PTR;
=======
          dwCookie : DWORD;
>>>>>>> graemeg/fixes_2_2
=======
          dwCookie : DWORD;
>>>>>>> origin/fixes_2_2
          dwError : DWORD;
          pfnCallback : EDITSTREAMCALLBACK;
       end;
     EDITSTREAM = _editstream;
     TEDITSTREAM = _editstream;

     _encorrecttext = record
          nmhdr : NMHDR;
          chrg : CHARRANGE;
          seltyp : WORD;
       end;
     ENCORRECTTEXT = _encorrecttext;
     TENCORRECTTEXT = _encorrecttext;

     _endropfiles = record
          nmhdr : NMHDR;
          hDrop : HANDLE;
          cp : LONG;
          fProtected : BOOL;
       end;
     ENDROPFILES = _endropfiles;
     TENDROPFILES = _endropfiles;

     _enlink = record
          nmhdr : NMHDR;
          msg : UINT;
          wParam : WPARAM;
          lParam : LPARAM;
          chrg : CHARRANGE;
       end;
     ENLINK = _enlink;
     TENLINK = _enlink;

     ENOLEOPFAILED = record
          nmhdr : NMHDR;
          iob : LONG;
          lOper : LONG;
          hr : HRESULT;
       end;

     _enprotected = record
          nmhdr : NMHDR;
          msg : UINT;
          wParam : WPARAM;
          lParam : LPARAM;
          chrg : CHARRANGE;
       end;
     ENPROTECTED = _enprotected;
     TENPROTECTED = _enprotected;
     LPENPROTECTED = ^_enprotected;

     _ensaveclipboard = record
          nmhdr : NMHDR;
          cObjectCount : LONG;
          cch : LONG;
       end;
     ENSAVECLIPBOARD = _ensaveclipboard;
     TENSAVECLIPBOARD = _ensaveclipboard;

     _findtextA = record
          chrg : CHARRANGE;
          lpstrText : LPSTR;
       end;
     FINDTEXTA = _findtextA;
     TFINDTEXTA = _findtextA;

     _findtextW = record
          chrg : CHARRANGE;
          lpstrText : LPWSTR;
       end;
     FINDTEXTW = _findtextW;
     TFINDTEXTW = _findtextW;

     _findtextexA = record
          chrg : CHARRANGE;
          lpstrText : LPSTR;
          chrgText : CHARRANGE;
       end;
     FINDTEXTEXA = _findtextexA;
     TFINDTEXTEXA = _findtextexA;

     _findtextexW = record
          chrg : CHARRANGE;
          lpstrText : LPWSTR;
          chrgText : CHARRANGE;
       end;
     FINDTEXTEXW = _findtextexW;
     TFINDTEXTEXW = _findtextexW;

     _formatrange = record
          hdc : HDC;
          hdcTarget : HDC;
          rc : RECT;
          rcPage : RECT;
          chrg : CHARRANGE;
       end;
     FORMATRANGE = _formatrange;
     TFORMATRANGE = _formatrange;

     _msgfilter = record
          nmhdr : NMHDR;
          msg : UINT;
          wParam : WPARAM;
          lParam : LPARAM;
       end;
     MSGFILTER = _msgfilter;
     TMSGFILTER = _msgfilter;

     TReqSize = record
       nmhdr: TNMHdr;
       rc: TRect;
     end;
     PReqSize = ^TReqSize;

     _paraformat = record
          cbSize : UINT;
          dwMask : DWORD;
          wNumbering : WORD;
          wReserved : WORD;
          dxStartIndent : LONG;
          dxRightIndent : LONG;
          dxOffset : LONG;
          wAlignment : WORD;
          cTabCount : SHORT;
          rgxTabs : array[0..(MAX_TAB_STOPS)-1] of LONG;
       end;
     PARAFORMAT = _paraformat;
     TParaFormat = _paraformat;

     _paraformat2 = record
          cbSize : UINT;
          dwMask : DWORD;
          wNumbering : WORD;
          wEffects : WORD;
          dxStartIndent : LONG;
          dxRightIndent : LONG;
          dxOffset : LONG;
          wAlignment : WORD;
          cTabCount : SHORT;
          rgxTabs : array[0..(MAX_TAB_STOPS)-1] of LONG;
          dySpaceBefore : LONG;
          dySpaceAfter : LONG;
          dyLineSpacing : LONG;
          sStype : SHORT;
          bLineSpacingRule : BYTE;
          bOutlineLevel : BYTE;
          wShadingWeight : WORD;
          wShadingStyle : WORD;
          wNumberingStart : WORD;
          wNumberingStyle : WORD;
          wNumberingTab : WORD;
          wBorderSpace : WORD;
          wBorderWidth : WORD;
          wBorders : WORD;
       end;
     PARAFORMAT2 = _paraformat2;
     TPARAFORMAT2 = _paraformat2;

     _selchange = record
          nmhdr : NMHDR;
          chrg : CHARRANGE;
          seltyp : WORD;
       end;
     SELCHANGE = _selchange;
     TSELCHANGE = _selchange;

     _textrange = record
          chrg : CHARRANGE;
          lpstrText : LPSTR;
       end;
     TEXTRANGEA = _textrange;
     TTEXTRANGEA = _textrange;

     _textrangew = record
          chrg : CHARRANGE;
          lpstrText : LPWSTR;
       end;
     TEXTRANGEW = _textrangew;
     TTEXTRANGEW = _textrangew;

     _reqresize = record
          nmhdr : NMHDR;
          rc : RECT;
       end;
     REQRESIZE = _reqresize;
     TREQRESIZE = _reqresize;

     _repastespecial = record
          dwAspect : DWORD;
          dwParam : DWORD;
       end;
     REPASTESPECIAL = _repastespecial;
     TREPASTESPECIAL = _repastespecial;

     _punctuation = record
          iSize : UINT;
          szPunctuation : LPSTR;
       end;
     PUNCTUATION = _punctuation;
     TPUNCTUATION = _punctuation;

     _gettextex = record
          cb : DWORD;
          flags : DWORD;
          codepage : UINT;
          lpDefaultChar : LPCSTR;
          lpUsedDefaultChar : LPBOOL;
       end;
     GETTEXTEX = _gettextex;
     TGETTEXTEX = _gettextex;

     EDITWORDBREAKPROCEX = function (pchText:pchar; cchText:LONG; bCharSet:BYTE; action:LONG):LONG;
  { Defines for EM_SETTYPOGRAPHYOPTIONS  }

  const
     TO_ADVANCEDTYPOGRAPHY = 1;
     TO_SIMPLELINEBREAK = 2;
  { Defines for GETTEXTLENGTHEX  }
     GTL_DEFAULT = 0;
     GTL_USECRLF = 1;
     GTL_PRECISE = 2;
     GTL_CLOSE = 4;
     GTL_NUMCHARS = 8;
     GTL_NUMBYTES = 16;

  type

     _gettextlengthex = record
          flags : DWORD;
          codepage : UINT;
       end;
     GETTEXTLENGTHEX = _gettextlengthex;
{$ifdef UNICODE}

     CHARFORMAT = CHARFORMATW;
     TCHARFORMAT = CHARFORMATW;
     CHARFORMAT2 = CHARFORMAT2W;
     TCHARFORMAT2 = CHARFORMAT2W;
     FINDTEXT = FINDTEXTW;
     TFINDTEXT = FINDTEXTW;
     FINDTEXTEX = FINDTEXTEXW;
     TFINDTEXTEX = FINDTEXTEXW;
     TEXTRANGE = TEXTRANGEW;
     TTEXTRANGE = TEXTRANGEW;
{$else}

  type

     CHARFORMAT = CHARFORMATA;
     TCHARFORMAT = CHARFORMATA;
     CHARFORMAT2 = CHARFORMAT2A;
     TCHARFORMAT2 = CHARFORMAT2A;
     FINDTEXT = FINDTEXTA;
     TFINDTEXT = FINDTEXTA;
     FINDTEXTEX = FINDTEXTEXA;
     TFINDTEXTEX = FINDTEXTEXA;
     TEXTRANGE = TEXTRANGEA;
     TTEXTRANGE = TEXTRANGEA;
{$endif}

implementation

end.
