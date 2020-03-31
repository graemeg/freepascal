{
    This file is part of the Free Pascal run time library.
    This unit contains the record definition for the Win32 API
    Copyright (c) 1999-2000 by Florian KLaempfl,
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  Structures.h

  Declarations for all the Windows32 API Structures

  Copyright (C) 1996 Free Software Foundation, Inc.

  Author:  Scott Christley <scottc@net-community.com>
  Date: 1996

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
  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
}

{$ifdef read_interface}

Const
  IMAGE_SIZEOF_SHORT_NAME = 8;

  type
  {$I typshrdh.inc}

    { WARNING
      the variable argument list
      is not implemented for FPC
      va_list is just a dummy record
      MvdV: Nevertheless it should be a pointer type, not a record}

     va_list = pchar;

     ABC = record
          abcA : longint;
          abcB : UINT;
          abcC : longint;
       end;
     LPABC = ^ABC;
     _ABC = ABC;
     TABC = ABC;
     PABC = ^ABC;

     ABCFLOAT = record
          abcfA : Single;
          abcfB : Single;
          abcfC : Single;
       end;
     LPABCFLOAT = ^ABCFLOAT;
     _ABCFLOAT = ABCFLOAT;
     TABCFLOAT = ABCFLOAT;
     PABCFLOAT = ^ABCFLOAT;

     ACCEL = record
          fVirt : BYTE;
          key : WORD;
          cmd : WORD;
       end;
     LPACCEL = ^ACCEL;
     _ACCEL = ACCEL;
     TACCEL = ACCEL;
     PACCEL = ^ACCEL;

     ACE_HEADER = record
          AceType : BYTE;
          AceFlags : BYTE;
          AceSize : WORD;
       end;
     _ACE_HEADER = ACE_HEADER;
     TACE_HEADER = ACE_HEADER;
     PACE_HEADER = ^ACE_HEADER;

     ACCESS_MASK = DWORD;

     REGSAM = ACCESS_MASK;

     ACCESS_ALLOWED_ACE = record
          Header : ACE_HEADER;
          Mask : ACCESS_MASK;
          SidStart : DWORD;
       end;
     _ACCESS_ALLOWED_ACE = ACCESS_ALLOWED_ACE;
     TACCESS_ALLOWED_ACE = ACCESS_ALLOWED_ACE;
     PACCESS_ALLOWED_ACE = ^ACCESS_ALLOWED_ACE;

     ACCESS_DENIED_ACE = record
          Header : ACE_HEADER;
          Mask : ACCESS_MASK;
          SidStart : DWORD;
       end;
     _ACCESS_DENIED_ACE = ACCESS_DENIED_ACE;
     TACCESS_DENIED_ACE = ACCESS_DENIED_ACE;

     ACCESSTIMEOUT = record
          cbSize : UINT;
          dwFlags : DWORD;
          iTimeOutMSec : DWORD;
       end;
     _ACCESSTIMEOUT = ACCESSTIMEOUT;
     TACCESSTIMEOUT = ACCESSTIMEOUT;
     PACCESSTIMEOUT = ^ACCESSTIMEOUT;

     ACL = record
          AclRevision : BYTE;
          Sbz1 : BYTE;
          AclSize : WORD;
          AceCount : WORD;
          Sbz2 : WORD;
       end;
     PACL = ^ACL;
     _ACL = ACL;
     TACL = ACL;

     ACL_REVISION_INFORMATION = record
          AclRevision : DWORD;
       end;
     _ACL_REVISION_INFORMATION = ACL_REVISION_INFORMATION;
     TACLREVISIONINFORMATION = ACL_REVISION_INFORMATION;
     PACLREVISIONINFORMATION = ^ACL_REVISION_INFORMATION;

     ACL_SIZE_INFORMATION = record
          AceCount : DWORD;
          AclBytesInUse : DWORD;
          AclBytesFree : DWORD;
       end;
     _ACL_SIZE_INFORMATION = ACL_SIZE_INFORMATION;
     TACLSIZEINFORMATION = ACL_SIZE_INFORMATION;
     PACLSIZEINFORMATION = ^ACL_SIZE_INFORMATION;

     ACTION_HEADER = record
          transport_id : ULONG;
          action_code : USHORT;
          reserved : USHORT;
       end;
     _ACTION_HEADER = ACTION_HEADER;
     TACTIONHEADER = ACTION_HEADER;
     PACTIONHEADER = ^ACTION_HEADER;

     ADAPTER_STATUS = record
          adapter_address : array[0..5] of UCHAR;
          rev_major : UCHAR;
          reserved0 : UCHAR;
          adapter_type : UCHAR;
          rev_minor : UCHAR;
          duration : WORD;
          frmr_recv : WORD;
          frmr_xmit : WORD;
          iframe_recv_err : WORD;
          xmit_aborts : WORD;
          xmit_success : DWORD;
          recv_success : DWORD;
          iframe_xmit_err : WORD;
          recv_buff_unavail : WORD;
          t1_timeouts : WORD;
          ti_timeouts : WORD;
          reserved1 : DWORD;
          free_ncbs : WORD;
          max_cfg_ncbs : WORD;
          max_ncbs : WORD;
          xmit_buf_unavail : WORD;
          max_dgram_size : WORD;
          pending_sess : WORD;
          max_cfg_sess : WORD;
          max_sess : WORD;
          max_sess_pkt_size : WORD;
          name_count : WORD;
       end;
     _ADAPTER_STATUS = ADAPTER_STATUS;
     TADAPTERSTATUS = ADAPTER_STATUS;
     PADAPTERSTATUS = ^ADAPTER_STATUS;

     ADDJOB_INFO_1 = record
          Path : LPTSTR;
          JobId : DWORD;
       end;
     _ADDJOB_INFO_1 = ADDJOB_INFO_1;
     TADDJOB_INFO_1 = ADDJOB_INFO_1;
     PADDJOB_INFO_1 = ^ADDJOB_INFO_1;

     ANIMATIONINFO = record
          cbSize : UINT;
          iMinAnimate : longint;
       end;
     LPANIMATIONINFO = ^ANIMATIONINFO;
     _ANIMATIONINFO = ANIMATIONINFO;
     TANIMATIONINFO = ANIMATIONINFO;
     PANIMATIONINFO = ^ANIMATIONINFO;

     POINT = TPOINT;
     LPPOINT = PPOINT;
     tagPOINT = TPOINT;


     RECT = TRect;
     LPRECT = PRECT;
     _RECT = RECT;

     RECTL = record
          left : LONG;
          top : LONG;
          right : LONG;
          bottom : LONG;
       end;
     _RECTL = RECTL;
     TRECTL = RECTL;
     PRECTL = ^RECTL;

     APPBARDATA = record
          cbSize : DWORD;
          hWnd : HWND;
          uCallbackMessage : UINT;
          uEdge : UINT;
          rc : RECT;
          lParam : LPARAM;
       end;
     _AppBarData = APPBARDATA;
     TAppBarData = APPBARDATA;
     PAppBarData = ^APPBARDATA;

     BITMAP = record
          bmType : LONG;
          bmWidth : LONG;
          bmHeight : LONG;
          bmWidthBytes : LONG;
          bmPlanes : WORD;
          bmBitsPixel : WORD;
          bmBits : LPVOID;
       end;
     PBITMAP = ^BITMAP;
     NPBITMAP = ^BITMAP;
     LPBITMAP = ^BITMAP;
     tagBITMAP = BITMAP;
     TBITMAP = BITMAP;

     BITMAPCOREHEADER = record
          bcSize : DWORD;
          bcWidth : WORD;
          bcHeight : WORD;
          bcPlanes : WORD;
          bcBitCount : WORD;
       end;
     tagBITMAPCOREHEADER = BITMAPCOREHEADER;
     TBITMAPCOREHEADER = BITMAPCOREHEADER;
     PBITMAPCOREHEADER = ^BITMAPCOREHEADER;

     RGBTRIPLE = record
          rgbtBlue : BYTE;
          rgbtGreen : BYTE;
          rgbtRed : BYTE;
       end;
     tagRGBTRIPLE = RGBTRIPLE;
     TRGBTRIPLE = RGBTRIPLE;
     PRGBTRIPLE = ^RGBTRIPLE;

     BITMAPCOREINFO = record
          bmciHeader : BITMAPCOREHEADER;
          bmciColors : array[0..0] of RGBTRIPLE;
       end;
     PBITMAPCOREINFO = ^BITMAPCOREINFO;
     LPBITMAPCOREINFO = ^BITMAPCOREINFO;
     _BITMAPCOREINFO = BITMAPCOREINFO;
     TBITMAPCOREINFO = BITMAPCOREINFO;

(* error
  WORD    bfReserved1;
  WORD    bfReserved2;
 in declarator_list *)

     BITMAPINFOHEADER = record
          biSize : DWORD;
          biWidth : LONG;
          biHeight : LONG;
          biPlanes : WORD;
          biBitCount : WORD;
          biCompression : DWORD;
          biSizeImage : DWORD;
          biXPelsPerMeter : LONG;
          biYPelsPerMeter : LONG;
          biClrUsed : DWORD;
          biClrImportant : DWORD;
       end;
     LPBITMAPINFOHEADER = ^BITMAPINFOHEADER;
     TBITMAPINFOHEADER = BITMAPINFOHEADER;
     PBITMAPINFOHEADER = ^BITMAPINFOHEADER;

     RGBQUAD = record
          rgbBlue : BYTE;
          rgbGreen : BYTE;
          rgbRed : BYTE;
          rgbReserved : BYTE;
       end;
     tagRGBQUAD = RGBQUAD;
     TRGBQUAD = RGBQUAD;
     PRGBQUAD = ^RGBQUAD;

     BITMAPINFO = record
          bmiHeader : BITMAPINFOHEADER;
          bmiColors : array[0..0] of RGBQUAD;
       end;
     LPBITMAPINFO = ^BITMAPINFO;
     PBITMAPINFO = ^BITMAPINFO;
     TBITMAPINFO = BITMAPINFO;

     FXPT2DOT30   = longint;
     LPFXPT2DOT30 = ^FXPT2DOT30;
     TFXPT2DOT30  = FXPT2DOT30;
     PFXPT2DOT30  = LPFXPT2DOT30;

     CIEXYZ = record
          ciexyzX : FXPT2DOT30;
          ciexyzY : FXPT2DOT30;
          ciexyzZ : FXPT2DOT30;
       end;
     tagCIEXYZ = CIEXYZ;
     LPCIEXYZ = ^CIEXYZ;
     TPCIEXYZ = CIEXYZ;
     PCIEXYZ = ^CIEXYZ;

     CIEXYZTRIPLE = record
          ciexyzRed : CIEXYZ;
          ciexyzGreen : CIEXYZ;
          ciexyzBlue : CIEXYZ;
       end;
     tagCIEXYZTRIPLE = CIEXYZTRIPLE;
     LPCIEXYZTRIPLE = ^CIEXYZTRIPLE;
     TCIEXYZTRIPLE = CIEXYZTRIPLE;
     PCIEXYZTRIPLE = ^CIEXYZTRIPLE;

     BITMAPV4HEADER = record
          bV4Size : DWORD;
          bV4Width : LONG;
          bV4Height : LONG;
          bV4Planes : WORD;
          bV4BitCount : WORD;
          bV4V4Compression : DWORD;
          bV4SizeImage : DWORD;
          bV4XPelsPerMeter : LONG;
          bV4YPelsPerMeter : LONG;
          bV4ClrUsed : DWORD;
          bV4ClrImportant : DWORD;
          bV4RedMask : DWORD;
          bV4GreenMask : DWORD;
          bV4BlueMask : DWORD;
          bV4AlphaMask : DWORD;
          bV4CSType : DWORD;
          bV4Endpoints : CIEXYZTRIPLE;
          bV4GammaRed : DWORD;
          bV4GammaGreen : DWORD;
          bV4GammaBlue : DWORD;
       end;
     LPBITMAPV4HEADER = ^BITMAPV4HEADER;
     TBITMAPV4HEADER = BITMAPV4HEADER;
     PBITMAPV4HEADER = ^BITMAPV4HEADER;

     BITMAPFILEHEADER = packed record
       bfType : Word;
       bfSize : DWord;
       bfReserved1 : Word;
       bfReserved2 : Word;
       bfOffBits : DWord;
     end;
     tagBITMAPFILEHEADER = BITMAPFILEHEADER;

     TBitmapFileHeader = BITMAPFILEHEADER;
     PBitmapFileHeader = ^TBitmapFileHeader;

     BLOB = record
          cbSize : ULONG;
          pBlobData : ^BYTE;
       end;
     _BLOB = BLOB;
     TBLOB = BLOB;
     PBLOB = ^BLOB;

     SHITEMID = record
          cb : USHORT;
          abID : array[0..0] of BYTE;
       end;
     LPSHITEMID = ^SHITEMID;
     LPCSHITEMID = ^SHITEMID;
     _SHITEMID = SHITEMID;
     TSHITEMID = SHITEMID;
     PSHITEMID = ^SHITEMID;

     ITEMIDLIST = record
          mkid : SHITEMID;
       end;
     LPITEMIDLIST = ^ITEMIDLIST;
     LPCITEMIDLIST = ^ITEMIDLIST;
     _ITEMIDLIST = ITEMIDLIST;
     TITEMIDLIST = ITEMIDLIST;
     PITEMIDLIST = ^ITEMIDLIST;

     BROWSEINFO = record
          hwndOwner : HWND;
          pidlRoot : LPCITEMIDLIST;
          pszDisplayName : LPTSTR;
          lpszTitle : LPCTSTR;
          ulFlags : UINT;
          lpfn : BFFCALLBACK;
          lParam : LPARAM;
          iImage : longint;
       end deprecated 'Use shlobj version. Will be removed in the future'; 
     LPBROWSEINFO = ^BROWSEINFO deprecated;
     _browseinfo = BROWSEINFO deprecated;
     Tbrowseinfo = BROWSEINFO deprecated;
     PBROWSEINFO = ^BROWSEINFO deprecated;

     FILETIME = record
          dwLowDateTime : DWORD;
          dwHighDateTime : DWORD;
       end;
     LPFILETIME = ^FILETIME;
     _FILETIME = FILETIME;
     TFILETIME = FILETIME;
     PFILETIME = ^FILETIME;

     BY_HANDLE_FILE_INFORMATION = record
          dwFileAttributes : DWORD;
          ftCreationTime : FILETIME;
          ftLastAccessTime : FILETIME;
          ftLastWriteTime : FILETIME;
          dwVolumeSerialNumber : DWORD;
          nFileSizeHigh : DWORD;
          nFileSizeLow : DWORD;
          nNumberOfLinks : DWORD;
          nFileIndexHigh : DWORD;
          nFileIndexLow : DWORD;
       end;
     LPBY_HANDLE_FILE_INFORMATION = ^BY_HANDLE_FILE_INFORMATION;
     _BY_HANDLE_FILE_INFORMATION = BY_HANDLE_FILE_INFORMATION;
     TBYHANDLEFILEINFORMATION = BY_HANDLE_FILE_INFORMATION;
     PBYHANDLEFILEINFORMATION = ^BY_HANDLE_FILE_INFORMATION;

     FIXED = packed record
          fract : WORD;
          value : SHORT;
       end;
     _FIXED = FIXED;
     TFIXED = FIXED;
     PFIXED = ^FIXED;

     POINTFX = record
          x : FIXED;
          y : FIXED;
       end;
     tagPOINTFX = POINTFX;
     TPOINTFX = POINTFX;
     PPOINTFX = ^POINTFX;

     POINTL = record
          x : LONG;
          y : LONG;
       end;
     _POINTL = POINTL;
     TPOINTL = POINTL;
     PPOINTL = ^POINTL;

     POINTS = record
          x : SHORT;
          y : SHORT;
       end;
     tagPOINTS = POINTS;
     TPOINTS = POINTS;
     PPOINTS = ^POINTS;

     CANDIDATEFORM = record
          dwIndex : DWORD;
          dwStyle : DWORD;
          ptCurrentPos : POINT;
          rcArea : RECT;
       end;
     LPCANDIDATEFORM = ^CANDIDATEFORM;
     _tagCANDIDATEFORM = CANDIDATEFORM;
     TCANDIDATEFORM = CANDIDATEFORM;
     PCANDIDATEFORM = ^CANDIDATEFORM;

     CANDIDATELIST = record
          dwSize : DWORD;
          dwStyle : DWORD;
          dwCount : DWORD;
          dwSelection : DWORD;
          dwPageStart : DWORD;
          dwPageSize : DWORD;
          dwOffset : array[0..0] of DWORD;
       end;
     LPCANDIDATELIST = ^CANDIDATELIST;
     _tagCANDIDATELIST = CANDIDATELIST;
     TCANDIDATELIST = CANDIDATELIST;
     PCANDIDATELIST = ^CANDIDATELIST;

     CREATESTRUCT = record
          lpCreateParams : LPVOID;
          hInstance : HINST;
          hMenu : HMENU;
          hwndParent : HWND;
          cy : longint;
          cx : longint;
          y : longint;
          x : longint;
          style : LONG;
          lpszName : LPCTSTR;
          lpszClass : LPCTSTR;
          dwExStyle : DWORD;
       end;
     LPCREATESTRUCT = ^CREATESTRUCT;
     tagCREATESTRUCT = CREATESTRUCT;
     TCREATESTRUCT = CREATESTRUCT;
     PCREATESTRUCT = ^CREATESTRUCT;

     CBT_CREATEWND = record
          lpcs : LPCREATESTRUCT;
          hwndInsertAfter : HWND;
       end;
     tagCBT_CREATEWND = CBT_CREATEWND;
     TCBT_CREATEWND = CBT_CREATEWND;
     PCBT_CREATEWND = ^CBT_CREATEWND;

     CBTACTIVATESTRUCT = record
          fMouse : WINBOOL;
          hWndActive : HWND;
       end;
     tagCBTACTIVATESTRUCT = CBTACTIVATESTRUCT;
     TCBTACTIVATESTRUCT = CBTACTIVATESTRUCT;
     PCBTACTIVATESTRUCT = ^CBTACTIVATESTRUCT;


     CHAR_INFO = record
              case longint of
                 0 : ( UnicodeChar : WCHAR;
                       Attributes  : Word);
                 1 : ( AsciiChar : CHAR );
              end;
     _CHAR_INFO = CHAR_INFO;
     TCHAR_INFO = CHAR_INFO;
     PCHAR_INFO = ^CHAR_INFO;

     CHARFORMATA = record
          cbSize : UINT;
          dwMask : DWORD;
          dwEffects : DWORD;
          yHeight : LONG;
          yOffset : LONG;
          crTextColor : COLORREF;
          bCharSet : BYTE;
          bPitchAndFamily : BYTE;
          szFaceName : array[0..(LF_FACESIZE)-1] of AnsiCHAR;
       end;
     _CHARFORMATA = CHARFORMATA;
     TCHARFORMATA = CHARFORMATA;
     PCHARFORMATA = ^CHARFORMATA;

     CHARFORMATW = record
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
     _CHARFORMATW = CHARFORMATW;
     TCHARFORMATW = CHARFORMATW;
     PCHARFORMATW = ^CHARFORMATW;
{$ifdef Unicode}
   CHARFORMAT  = CHARFORMATW;
   _CHARFORMAT = CHARFORMATW;
   TCHARFORMAT = CHARFORMATW;
   PCHARFORMAT = PCHARFORMATW;
{$else}
   CHARFORMAT  = CHARFORMATA;
   _CHARFORMAT = CHARFORMATA;
   TCHARFORMAT = CHARFORMATA;
   PCHARFORMAT = PCHARFORMATA;
{$endif}

     CHARRANGE = record
          cpMin : LONG;
          cpMax : LONG;
       end;
     _charrange = CHARRANGE;
     Tcharrange = CHARRANGE;
     Pcharrange = ^CHARRANGE;

     CHARSET = record
          aflBlock : array[0..2] of DWORD;
          flLang : DWORD;
       end;
     tagCHARSET = CHARSET;
     TCHARSET = CHARSET;
     PCHARSET = ^CHARSET;

     FONTSIGNATURE = record
          fsUsb : array[0..3] of DWORD;
          fsCsb : array[0..1] of DWORD;
       end;
     LPFONTSIGNATURE = ^FONTSIGNATURE;
     tagFONTSIGNATURE = FONTSIGNATURE;
     TFONTSIGNATURE = FONTSIGNATURE;
     PFONTSIGNATURE = ^FONTSIGNATURE;

     CHARSETINFO = record
          ciCharset : UINT;
          ciACP : UINT;
          fs : FONTSIGNATURE;
       end;
     LPCHARSETINFO = ^CHARSETINFO;
     TCHARSETINFO = CHARSETINFO;
     PCHARSETINFO = ^CHARSETINFO;

     {CHOOSECOLOR = record confilcts with function ChooseColor }
     TCHOOSECOLOR = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HWND;
          rgbResult : COLORREF;
          lpCustColors : ^COLORREF;
          Flags : DWORD;
          lCustData : LPARAM;
          lpfnHook : LPCCHOOKPROC;
          lpTemplateName : LPCTSTR;
       end;
     LPCHOOSECOLOR = ^TCHOOSECOLOR;
     PCHOOSECOLOR = ^TCHOOSECOLOR;

     LOGFONTA = record
          lfHeight : LONG;
          lfWidth : LONG;
          lfEscapement : LONG;
          lfOrientation : LONG;
          lfWeight : LONG;
          lfItalic : BYTE;
          lfUnderline : BYTE;
          lfStrikeOut : BYTE;
          lfCharSet : BYTE;
          lfOutPrecision : BYTE;
          lfClipPrecision : BYTE;
          lfQuality : BYTE;
          lfPitchAndFamily : BYTE;
          lfFaceName : array[0..(LF_FACESIZE)-1] of AnsiCHAR;
       end;
     LPLOGFONTA = ^LOGFONTA;
     _LOGFONTA  = LOGFONTA;
     TLOGFONTA  = LOGFONTA;
     PLOGFONTA  = LPLOGFONTA;
     tagLOGFONTA= LOGFONTA;

     LOGFONTW = record
       lfHeight: LONG;
       lfWidth: LONG;
       lfEscapement: LONG;
       lfOrientation: LONG;
       lfWeight: LONG;
       lfItalic: BYTE;
       lfUnderline: BYTE;
       lfStrikeOut: BYTE;
       lfCharSet: BYTE;
       lfOutPrecision: BYTE;
       lfClipPrecision: BYTE;
       lfQuality: BYTE;
       lfPitchAndFamily: BYTE;
       lfFaceName: array [0..LF_FACESIZE - 1] of WCHAR;
     end;
     LPLOGFONTW = ^LOGFONTW;
     NPLOGFONTW = ^LOGFONTW;
     _LOGFONTW  = LOGFONTW;
     TLogFontW  = LOGFONTW;
     PLogFontW  = ^TLOGFONTW;
     tagLOGFONTW= LOGFONTW;

   {$IFNDEF UNICODE}
     LOGFONT    = LOGFONTA;
     LPLOGFONT  = ^LOGFONTA;
     _LOGFONT   = LOGFONTA;
     TLOGFONT   = LOGFONTA;
     PLOGFONT   = ^LOGFONTA;
     tagLOGFONT = LOGFONTA;
   {$ELSE}
     LOGFONT    = LOGFONTW;
     LPLOGFONT  = ^LOGFONTW;
     _LOGFONT   = LOGFONTW;
     TLOGFONT   = LOGFONTW;
     PLOGFONT   = ^LOGFONTW;
     tagLOGFONT = LOGFONTW;
    {$ENDIF}

     {CHOOSEFONT = record conflicts with ChosseFont function }
     TCHOOSEFONT = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDC : HDC;
          lpLogFont : LPLOGFONT;
          iPointSize : WINT;
          Flags : DWORD;
          rgbColors : COLORREF;
          lCustData : LPARAM;
          lpfnHook : LPCFHOOKPROC;
          lpTemplateName : LPCTSTR;
          hInstance : HINST;
          lpszStyle : LPTSTR;
          nFontType : WORD;
          ___MISSING_ALIGNMENT__ : WORD;
          nSizeMin : WINT;
          nSizeMax : WINT;
       end;
     LPCHOOSEFONT = ^TCHOOSEFONT;
     PCHOOSEFONT = ^TCHOOSEFONT;

     CIDA = record
          cidl : UINT;
          aoffset : array[0..0] of UINT;
       end;
     LPIDA = ^CIDA;
     _IDA = CIDA;
     TIDA = CIDA;
     PIDA = ^CIDA;

     CLIENTCREATESTRUCT = record
          hWindowMenu : HANDLE;
          idFirstChild : UINT;
       end;
     LPCLIENTCREATESTRUCT = ^CLIENTCREATESTRUCT;
     tagCLIENTCREATESTRUCT = CLIENTCREATESTRUCT;
     TCLIENTCREATESTRUCT = CLIENTCREATESTRUCT;
     PCLIENTCREATESTRUCT = ^CLIENTCREATESTRUCT;

     CMINVOKECOMMANDINFO = record
          cbSize : DWORD;
          fMask : DWORD;
          hwnd : HWND;
          lpVerb : LPCSTR;
          lpParameters : LPCSTR;
          lpDirectory : LPCSTR;
          nShow : longint;
          dwHotKey : DWORD;
          hIcon : HANDLE;
       end deprecated 'Use shlobj version. Will be removed in the future';
     LPCMINVOKECOMMANDINFO = ^CMINVOKECOMMANDINFO deprecated;
     _CMInvokeCommandInfo = CMINVOKECOMMANDINFO deprecated;
     TCMInvokeCommandInfo = CMINVOKECOMMANDINFO deprecated;
     PCMInvokeCommandInfo = ^CMINVOKECOMMANDINFO deprecated;

     COLORADJUSTMENT = record
          caSize : WORD;
          caFlags : WORD;
          caIlluminantIndex : WORD;
          caRedGamma : WORD;
          caGreenGamma : WORD;
          caBlueGamma : WORD;
          caReferenceBlack : WORD;
          caReferenceWhite : WORD;
          caContrast : SHORT;
          caBrightness : SHORT;
          caColorfulness : SHORT;
          caRedGreenTint : SHORT;
       end;
     LPCOLORADJUSTMENT = ^COLORADJUSTMENT;
     tagCOLORADJUSTMENT = COLORADJUSTMENT;
     TCOLORADJUSTMENT = COLORADJUSTMENT;
     PCOLORADJUSTMENT = ^COLORADJUSTMENT;

     COLORMAP = record
          from : COLORREF;
          _to : COLORREF;
       end;
     LPCOLORMAP = ^COLORMAP;
     _COLORMAP = COLORMAP;
     TCOLORMAP = COLORMAP;
     PCOLORMAP = ^COLORMAP;

     DCB = record
          DCBlength : DWORD;
          BaudRate : DWORD;
          flags : DWORD;
          wReserved : WORD;
          XonLim : WORD;
          XoffLim : WORD;
          ByteSize : BYTE;
          Parity : BYTE;
          StopBits : BYTE;
          XonChar : char;
          XoffChar : char;
          ErrorChar : char;
          EofChar : char;
          EvtChar : char;
          wReserved1 : WORD;
       end;
     LPDCB = ^DCB;
     _DCB = DCB;
     TDCB = DCB;
     PDCB = ^DCB;

     FLASHWINFO = record
         cbSize    : UINT;
         hwnd      : HWND;
         dwFlags   : DWORD;
         uCount    : UINT;
         dwTimeOut : DWORD;
     end;
    TFLASHWINFO  = FLASHWINFO;
    PFLASHWINFO  = ^FLASHWINFO;

  const
     bm_DCB_fBinary = $1;
     bp_DCB_fBinary = 0;
     bm_DCB_fParity = $2;
     bp_DCB_fParity = 1;
     bm_DCB_fOutxCtsFlow = $4;
     bp_DCB_fOutxCtsFlow = 2;
     bm_DCB_fOutxDsrFlow = $8;
     bp_DCB_fOutxDsrFlow = 3;
     bm_DCB_fDtrControl = $30;
     bp_DCB_fDtrControl = 4;
     bm_DCB_fDsrSensitivity = $40;
     bp_DCB_fDsrSensitivity = 6;
     bm_DCB_fTXContinueOnXoff = $80;
     bp_DCB_fTXContinueOnXoff = 7;
     bm_DCB_fOutX = $100;
     bp_DCB_fOutX = 8;
     bm_DCB_fInX = $200;
     bp_DCB_fInX = 9;
     bm_DCB_fErrorChar = $400;
     bp_DCB_fErrorChar = 10;
     bm_DCB_fNull = $800;
     bp_DCB_fNull = 11;
     bm_DCB_fRtsControl = $3000;
     bp_DCB_fRtsControl = 12;
     bm_DCB_fAbortOnError = $4000;
     bp_DCB_fAbortOnError = 14;
     bm_DCB_fDummy2 = $FFFF8000;
     bp_DCB_fDummy2 = 15;
  function fBinary(var a : DCB) : DWORD;
  procedure set_fBinary(var a : DCB; __fBinary : DWORD);
  function fParity(var a : DCB) : DWORD;
  procedure set_fParity(var a : DCB; __fParity : DWORD);
  function fOutxCtsFlow(var a : DCB) : DWORD;
  procedure set_fOutxCtsFlow(var a : DCB; __fOutxCtsFlow : DWORD);
  function fOutxDsrFlow(var a : DCB) : DWORD;
  procedure set_fOutxDsrFlow(var a : DCB; __fOutxDsrFlow : DWORD);
  function fDtrControl(var a : DCB) : DWORD;
  procedure set_fDtrControl(var a : DCB; __fDtrControl : DWORD);
  function fDsrSensitivity(var a : DCB) : DWORD;
  procedure set_fDsrSensitivity(var a : DCB; __fDsrSensitivity : DWORD);
  function fTXContinueOnXoff(var a : DCB) : DWORD;
  procedure set_fTXContinueOnXoff(var a : DCB; __fTXContinueOnXoff : DWORD);
  function fOutX(var a : DCB) : DWORD;
  procedure set_fOutX(var a : DCB; __fOutX : DWORD);
  function fInX(var a : DCB) : DWORD;
  procedure set_fInX(var a : DCB; __fInX : DWORD);
  function fErrorChar(var a : DCB) : DWORD;
  procedure set_fErrorChar(var a : DCB; __fErrorChar : DWORD);
  function fNull(var a : DCB) : DWORD;
  procedure set_fNull(var a : DCB; __fNull : DWORD);
  function fRtsControl(var a : DCB) : DWORD;
  procedure set_fRtsControl(var a : DCB; __fRtsControl : DWORD);
  function fAbortOnError(var a : DCB) : DWORD;
  procedure set_fAbortOnError(var a : DCB; __fAbortOnError : DWORD);
  function fDummy2(var a : DCB) : DWORD;
  procedure set_fDummy2(var a : DCB; __fDummy2 : DWORD);

  type

     COMMCONFIG = record
          dwSize : DWORD;
          wVersion : WORD;
          wReserved : WORD;
          dcb : DCB;
          dwProviderSubType : DWORD;
          dwProviderOffset : DWORD;
          dwProviderSize : DWORD;
          wcProviderData : array[0..0] of WCHAR;
       end;
     LPCOMMCONFIG = ^COMMCONFIG;
     _COMM_CONFIG = COMMCONFIG;
     TCOMMCONFIG = COMMCONFIG;
     PCOMMCONFIG = ^COMMCONFIG;

     COMMPROP = record
          wPacketLength : WORD;
          wPacketVersion : WORD;
          dwServiceMask : DWORD;
          dwReserved1 : DWORD;
          dwMaxTxQueue : DWORD;
          dwMaxRxQueue : DWORD;
          dwMaxBaud : DWORD;
          dwProvSubType : DWORD;
          dwProvCapabilities : DWORD;
          dwSettableParams : DWORD;
          dwSettableBaud : DWORD;
          wSettableData : WORD;
          wSettableStopParity : WORD;
          dwCurrentTxQueue : DWORD;
          dwCurrentRxQueue : DWORD;
          dwProvSpec1 : DWORD;
          dwProvSpec2 : DWORD;
          wcProvChar : array[0..0] of WCHAR;
       end;
     LPCOMMPROP = ^COMMPROP;
     _COMMPROP = COMMPROP;
     TCOMMPROP = COMMPROP;
     PCOMMPROP = ^COMMPROP;

     COMMTIMEOUTS = record
          ReadIntervalTimeout : DWORD;
          ReadTotalTimeoutMultiplier : DWORD;
          ReadTotalTimeoutConstant : DWORD;
          WriteTotalTimeoutMultiplier : DWORD;
          WriteTotalTimeoutConstant : DWORD;
       end;
     LPCOMMTIMEOUTS = ^COMMTIMEOUTS;
     _COMMTIMEOUTS = COMMTIMEOUTS;
     TCOMMTIMEOUTS = COMMTIMEOUTS;
     PCOMMTIMEOUTS = ^COMMTIMEOUTS;

     COMPAREITEMSTRUCT = record
          CtlType : UINT;
          CtlID : UINT;
          hwndItem : HWND;
          itemID1 : UINT;
          itemData1 : ULONG_PTR;
          itemID2 : UINT;
          itemData2 : ULONG_PTR;
          dwLocaleID : DWORD;
       end;
     tagCOMPAREITEMSTRUCT = COMPAREITEMSTRUCT;
     TCOMPAREITEMSTRUCT = COMPAREITEMSTRUCT;
     PCOMPAREITEMSTRUCT = ^COMPAREITEMSTRUCT;

     COMPCOLOR = record
          crText : COLORREF;
          crBackground : COLORREF;
          dwEffects : DWORD;
       end;
     TCOMPCOLOR = COMPCOLOR;
     PCOMPCOLOR = ^COMPCOLOR;

     COMPOSITIONFORM = record
          dwStyle : DWORD;
          ptCurrentPos : POINT;
          rcArea : RECT;
       end;
     LPCOMPOSITIONFORM = ^COMPOSITIONFORM;
     _tagCOMPOSITIONFORM = COMPOSITIONFORM;
     TCOMPOSITIONFORM = COMPOSITIONFORM;
     PCOMPOSITIONFORM = ^COMPOSITIONFORM;

//     TComStatFlags = set of (fCtsHold, fDsrHold, fRlsdHold , fXoffHold ,
//                    fXoffSent , fEof ,  fTxim  , fReserved);

     COMSTAT = record
          flag0 : DWORD;   // can't use tcomstatflags, set packing issues
                           // and conflicts with macro's
          cbInQue : DWORD;
          cbOutQue : DWORD;
       end;
     LPCOMSTAT = ^COMSTAT;
     _COMSTAT = COMSTAT;
     TCOMSTAT = COMSTAT;
     PCOMSTAT = ^COMSTAT;
  const
     bm_COMSTAT_fCtsHold = $1;
     bp_COMSTAT_fCtsHold = 0;
     bm_COMSTAT_fDsrHold = $2;
     bp_COMSTAT_fDsrHold = 1;
     bm_COMSTAT_fRlsdHold = $4;
     bp_COMSTAT_fRlsdHold = 2;
     bm_COMSTAT_fXoffHold = $8;
     bp_COMSTAT_fXoffHold = 3;
     bm_COMSTAT_fXoffSent = $10;
     bp_COMSTAT_fXoffSent = 4;
     bm_COMSTAT_fEof = $20;
     bp_COMSTAT_fEof = 5;
     bm_COMSTAT_fTxim = $40;
     bp_COMSTAT_fTxim = 6;
     bm_COMSTAT_fReserved = $FFFFFF80;
     bp_COMSTAT_fReserved = 7;
  function fCtsHold(var a : COMSTAT) : DWORD;  // should be renamed to get_<x>?
  procedure set_fCtsHold(var a : COMSTAT; __fCtsHold : DWORD);
  function fDsrHold(var a : COMSTAT) : DWORD;
  procedure set_fDsrHold(var a : COMSTAT; __fDsrHold : DWORD);
  function fRlsdHold(var a : COMSTAT) : DWORD;
  procedure set_fRlsdHold(var a : COMSTAT; __fRlsdHold : DWORD);
  function fXoffHold(var a : COMSTAT) : DWORD;
  procedure set_fXoffHold(var a : COMSTAT; __fXoffHold : DWORD);
  function fXoffSent(var a : COMSTAT) : DWORD;
  procedure set_fXoffSent(var a : COMSTAT; __fXoffSent : DWORD);
  function fEof(var a : COMSTAT) : DWORD;
  procedure set_fEof(var a : COMSTAT; __fEof : DWORD);
  function fTxim(var a : COMSTAT) : DWORD;
  procedure set_fTxim(var a : COMSTAT; __fTxim : DWORD);
  function fReserved(var a : COMSTAT) : DWORD;
  procedure set_fReserved(var a : COMSTAT; __fReserved : DWORD);

  type

     CONSOLE_CURSOR_INFO = record
          dwSize : DWORD;
          bVisible : WINBOOL;
       end;
     PCONSOLE_CURSOR_INFO = ^CONSOLE_CURSOR_INFO;
     _CONSOLE_CURSOR_INFO = CONSOLE_CURSOR_INFO;
     TCONSOLECURSORINFO = CONSOLE_CURSOR_INFO;
     PCONSOLECURSORINFO = ^CONSOLE_CURSOR_INFO;

     COORD = record
          X : SHORT;
          Y : SHORT;
       end;
     _COORD = COORD;
     TCOORD = COORD;
     PCOORD = ^COORD;

     SMALL_RECT = record
          Left : SHORT;
          Top : SHORT;
          Right : SHORT;
          Bottom : SHORT;
       end;
     _SMALL_RECT = SMALL_RECT;
     TSMALL_RECT = SMALL_RECT;
     PSMALL_RECT = ^SMALL_RECT;

     CONSOLE_SCREEN_BUFFER_INFO = packed record
          dwSize : COORD;
          dwCursorPosition : COORD;
          wAttributes : WORD;
          srWindow : SMALL_RECT;
          dwMaximumWindowSize : COORD;
       end;
     PCONSOLE_SCREEN_BUFFER_INFO = ^CONSOLE_SCREEN_BUFFER_INFO;
     _CONSOLE_SCREEN_BUFFER_INFO = CONSOLE_SCREEN_BUFFER_INFO;
     TCONSOLESCREENBUFFERINFO = CONSOLE_SCREEN_BUFFER_INFO;
     PCONSOLESCREENBUFFERINFO = ^CONSOLE_SCREEN_BUFFER_INFO;

{$ifdef cpui386}
{$define __HASCONTEXT__}
  type

     FLOATING_SAVE_AREA = record
          ControlWord : DWORD;
          StatusWord : DWORD;
          TagWord : DWORD;
          ErrorOffset : DWORD;
          ErrorSelector : DWORD;
          DataOffset : DWORD;
          DataSelector : DWORD;
          RegisterArea : array[0..79] of BYTE;
          Cr0NpxState : DWORD;
       end;
     _FLOATING_SAVE_AREA = FLOATING_SAVE_AREA;
     TFLOATINGSAVEAREA = FLOATING_SAVE_AREA;
     PFLOATINGSAVEAREA = ^FLOATING_SAVE_AREA;

     CONTEXT = record
          ContextFlags : DWORD;
          Dr0 : DWORD;
          Dr1 : DWORD;
          Dr2 : DWORD;
          Dr3 : DWORD;
          Dr6 : DWORD;
          Dr7 : DWORD;
          FloatSave : FLOATING_SAVE_AREA;
          SegGs : DWORD;
          SegFs : DWORD;
          SegEs : DWORD;
          SegDs : DWORD;
          Edi : DWORD;
          Esi : DWORD;
          Ebx : DWORD;
          Edx : DWORD;
          Ecx : DWORD;
          Eax : DWORD;
          Ebp : DWORD;
          Eip : DWORD;
          SegCs : DWORD;
          EFlags : DWORD;
          Esp : DWORD;
          SegSs : DWORD;
       end;
{$endif}

{$ifdef cpux86_64}
{$define __HASCONTEXT__}
//
// Define 128-bit 16-byte aligned xmm register type.
//

//typedef struct DECLSPEC_ALIGN(16) _M128A {
{$note todo, fix alignment }
  type
     M128A = record
          Low: ULONGLONG;
          High: LONGLONG;
       end;
     _M128A = M128A;
     TM128A = M128A;
     PM128A = TM128A;

//
// Format of data for 32-bit fxsave/fxrstor instructions.
//

//typedef struct _XMM_SAVE_AREA32 {
  type
     XMM_SAVE_AREA32 = record
          ControlWord: WORD;
          StatusWord: WORD;
          TagWord: BYTE;
          Reserved1: BYTE;
          ErrorOpcode: WORD;
          ErrorOffset: DWORD;
          ErrorSelector: WORD;
          Reserved2: WORD;
          DataOffset: DWORD;
          DataSelector: WORD;
          Reserved3: WORD;
          MxCsr: DWORD;
          MxCsr_Mask: DWORD;
          FloatRegisters: array[0..7] of M128A;
          XmmRegisters: array[0..15] of M128A;
          Reserved4: array[0..95] of BYTE;
       end;
     _XMM_SAVE_AREA32 = XMM_SAVE_AREA32;
     TXmmSaveArea = XMM_SAVE_AREA32;
     PXmmSaveArea = ^TXmmSaveArea;

  const
     LEGACY_SAVE_AREA_LENGTH = sizeof(XMM_SAVE_AREA32);

//
// Context Frame
//
//  This frame has a several purposes: 1) it is used as an argument to
//  NtContinue, 2) is is used to constuct a call frame for APC delivery,
//  and 3) it is used in the user level thread creation routines.
//
//
// The flags field within this record controls the contents of a CONTEXT
// record.
//
// If the context record is used as an input parameter, then for each
// portion of the context record controlled by a flag whose value is
// set, it is assumed that that portion of the context record contains
// valid context. If the context record is being used to modify a threads
// context, then only that portion of the threads context is modified.
//
// If the context record is used as an output parameter to capture the
// context of a thread, then only those portions of the thread's context
// corresponding to set flags will be returned.
//
// CONTEXT_CONTROL specifies SegSs, Rsp, SegCs, Rip, and EFlags.
//
// CONTEXT_INTEGER specifies Rax, Rcx, Rdx, Rbx, Rbp, Rsi, Rdi, and R8-R15.
//
// CONTEXT_SEGMENTS specifies SegDs, SegEs, SegFs, and SegGs.
//
// CONTEXT_DEBUG_REGISTERS specifies Dr0-Dr3 and Dr6-Dr7.
//
// CONTEXT_MMX_REGISTERS specifies the floating point and extended registers
//     Mm0/St0-Mm7/St7 and Xmm0-Xmm15).
//

//typedef struct DECLSPEC_ALIGN(16) _CONTEXT {
{$note todo, fix alignment }
  type
     CONTEXT = record

          //
          // Register parameter home addresses.
          //
          // N.B. These fields are for convience - they could be used to extend the
          //      context record in the future.
          //

          P1Home: DWORD64;
          P2Home: DWORD64;
          P3Home: DWORD64;
          P4Home: DWORD64;
          P5Home: DWORD64;
          P6Home: DWORD64;

          //
          // Control flags.
          //

          ContextFlags: DWORD;
          MxCsr: DWORD;

          //
          // Segment Registers and processor flags.
          //

          SegCs: WORD;
          SegDs: WORD;
          SegEs: WORD;
          SegFs: WORD;
          SegGs: WORD;
          SegSs: WORD;
          EFlags: DWORD;

          //
          // Debug registers
          //

          Dr0: DWORD64;
          Dr1: DWORD64;
          Dr2: DWORD64;
          Dr3: DWORD64;
          Dr6: DWORD64;
          Dr7: DWORD64;

          //
          // Integer registers.
          //

          Rax: DWORD64;
          Rcx: DWORD64;
          Rdx: DWORD64;
          Rbx: DWORD64;
          Rsp: DWORD64;
          Rbp: DWORD64;
          Rsi: DWORD64;
          Rdi: DWORD64;
          R8: DWORD64;
          R9: DWORD64;
          R10: DWORD64;
          R11: DWORD64;
          R12: DWORD64;
          R13: DWORD64;
          R14: DWORD64;
          R15: DWORD64;

          //
          // Program counter.
          //

          Rip: DWORD64;

          //
          // Floating point state.
          //

          FltSave: XMM_SAVE_AREA32; // MWE: only translated the FltSave part of the union
(*
          union  {
              XMM_SAVE_AREA32 FltSave;
              struct {
                  M128A Header[2];
                  M128A Legacy[8];
                  M128A Xmm0;
                  M128A Xmm1;
                  M128A Xmm2;
                  M128A Xmm3;
                  M128A Xmm4;
                  M128A Xmm5;
                  M128A Xmm6;
                  M128A Xmm7;
                  M128A Xmm8;
                  M128A Xmm9;
                  M128A Xmm10;
                  M128A Xmm11;
                  M128A Xmm12;
                  M128A Xmm13;
                  M128A Xmm14;
                  M128A Xmm15;
              };
          };
*)

          //
          // Vector registers.
          //

          VectorRegister: array[0..25] of M128A;
          VectorControl: DWORD64;

          //
          // Special debug control registers.
          //

          DebugControl: DWORD64;
          LastBranchToRip: DWORD64;
          LastBranchFromRip: DWORD64;
          LastExceptionToRip: DWORD64;
          LastExceptionFromRip: DWORD64;
      end;
{$endif}

{$ifdef cpupowerpc32}
{$define __HASCONTEXT__}
  { __ppc__  }
  { Floating point registers returned when CONTEXT_FLOATING_POINT is set  }
  { Integer registers returned when CONTEXT_INTEGER is set.   }
  { Condition register  }
  { Fixed point exception register  }
  { The following are set when CONTEXT_CONTROL is set.   }
  { Machine status register  }
  { Instruction address register  }
  { Link register  }
  { Control register  }
  { Control which context values are returned  }
  { Registers returned if CONTEXT_DEBUG_REGISTERS is set.   }
  { Breakpoint Register 1  }
  { Breakpoint Register 2  }
  { Breakpoint Register 3  }
  { Breakpoint Register 4  }
  { Breakpoint Register 5  }
  { Breakpoint Register 6  }
  { Debug Status Register  }
  { Debug Control Register  }

  type

     CONTEXT = record
          Fpr0 : double;
          Fpr1 : double;
          Fpr2 : double;
          Fpr3 : double;
          Fpr4 : double;
          Fpr5 : double;
          Fpr6 : double;
          Fpr7 : double;
          Fpr8 : double;
          Fpr9 : double;
          Fpr10 : double;
          Fpr11 : double;
          Fpr12 : double;
          Fpr13 : double;
          Fpr14 : double;
          Fpr15 : double;
          Fpr16 : double;
          Fpr17 : double;
          Fpr18 : double;
          Fpr19 : double;
          Fpr20 : double;
          Fpr21 : double;
          Fpr22 : double;
          Fpr23 : double;
          Fpr24 : double;
          Fpr25 : double;
          Fpr26 : double;
          Fpr27 : double;
          Fpr28 : double;
          Fpr29 : double;
          Fpr30 : double;
          Fpr31 : double;
          Fpscr : double;
          Gpr0 : DWORD;
          Gpr1 : DWORD;
          Gpr2 : DWORD;
          Gpr3 : DWORD;
          Gpr4 : DWORD;
          Gpr5 : DWORD;
          Gpr6 : DWORD;
          Gpr7 : DWORD;
          Gpr8 : DWORD;
          Gpr9 : DWORD;
          Gpr10 : DWORD;
          Gpr11 : DWORD;
          Gpr12 : DWORD;
          Gpr13 : DWORD;
          Gpr14 : DWORD;
          Gpr15 : DWORD;
          Gpr16 : DWORD;
          Gpr17 : DWORD;
          Gpr18 : DWORD;
          Gpr19 : DWORD;
          Gpr20 : DWORD;
          Gpr21 : DWORD;
          Gpr22 : DWORD;
          Gpr23 : DWORD;
          Gpr24 : DWORD;
          Gpr25 : DWORD;
          Gpr26 : DWORD;
          Gpr27 : DWORD;
          Gpr28 : DWORD;
          Gpr29 : DWORD;
          Gpr30 : DWORD;
          Gpr31 : DWORD;
          Cr : DWORD;
          Xer : DWORD;
          Msr : DWORD;
          Iar : DWORD;
          Lr : DWORD;
          Ctr : DWORD;
          ContextFlags : DWORD;
          Fill : array[0..2] of DWORD;
          Dr0 : DWORD;
          Dr1 : DWORD;
          Dr2 : DWORD;
          Dr3 : DWORD;
          Dr4 : DWORD;
          Dr5 : DWORD;
          Dr6 : DWORD;
          Dr7 : DWORD;
       end;
{$endif}

{$ifndef __HASCONTEXT__}
  { MWE: placeholder so it won't break compilation on others (CPUARM ?) }
  type
     CONTEXT = record
       end;
{$endif}

{$undef __HASCONTEXT__}

     LPCONTEXT = ^CONTEXT;
     _CONTEXT = CONTEXT;
     TCONTEXT = CONTEXT;
     PCONTEXT = ^CONTEXT;

  type

     LIST_ENTRY = record
          Flink : ^_LIST_ENTRY;
          Blink : ^_LIST_ENTRY;
       end;
     _LIST_ENTRY = LIST_ENTRY;
     TLISTENTRY = LIST_ENTRY;
     PLISTENTRY = ^LIST_ENTRY;

     CRITICAL_SECTION_DEBUG = record
          _Type : WORD;
          CreatorBackTraceIndex : WORD;
          CriticalSection : ^_CRITICAL_SECTION;
          ProcessLocksList : LIST_ENTRY;
          EntryCount : DWORD;
          ContentionCount : DWORD;
          Depth : DWORD;
          OwnerBackTrace : array[0..4] of PVOID;
       end;
     LPCRITICAL_SECTION_DEBUG = ^CRITICAL_SECTION_DEBUG;
     PCRITICAL_SECTION_DEBUG = ^CRITICAL_SECTION_DEBUG;
     _CRITICAL_SECTION_DEBUG = CRITICAL_SECTION_DEBUG;
     TCRITICALSECTIONDEBUG = CRITICAL_SECTION_DEBUG;
     PCRITICALSECTIONDEBUG = ^CRITICAL_SECTION_DEBUG;

     CRITICAL_SECTION = TRTLCriticalSection;
     _CRITICAL_SECTION = TRTLCriticalSection;
     TCRITICAL_SECTION = TRTLCriticalSection;
     PCRITICAL_SECTION = PRTLCriticalSection;
     LPCRITICAL_SECTION = PRTLCriticalSection;
     TCRITICALSECTION = TRTLCriticalSection;
     PCRITICALSECTION = PRTLCriticalSection;

  { SECURITY_CONTEXT_TRACKING_MODE ContextTrackingMode;  }

     SECURITY_QUALITY_OF_SERVICE = record
          Length : DWORD;
          ImpersonationLevel : SECURITY_IMPERSONATION_LEVEL;
          ContextTrackingMode : BOOLEAN;    // SECURITY_CONTEXT_TRACKING_MODE
          EffectiveOnly : BOOLEAN;
       end;
     PSECURITY_QUALITY_OF_SERVICE = ^SECURITY_QUALITY_OF_SERVICE;
     _SECURITY_QUALITY_OF_SERVICE = SECURITY_QUALITY_OF_SERVICE;
     TSECURITYQUALITYOFSERVICE = SECURITY_QUALITY_OF_SERVICE;
     PSECURITYQUALITYOFSERVICE = ^SECURITY_QUALITY_OF_SERVICE;

     CONVCONTEXT = record
          cb : UINT;
          wFlags : UINT;
          wCountryID : UINT;
          iCodePage : longint;
          dwLangID : DWORD;
          dwSecurity : DWORD;
          qos : SECURITY_QUALITY_OF_SERVICE;
       end;
     tagCONVCONTEXT = CONVCONTEXT;
     TCONVCONTEXT = CONVCONTEXT;
     PCONVCONTEXT = ^CONVCONTEXT;

     CONVINFO = record
          cb : DWORD;
          hUser : DWORD;
          hConvPartner : HCONV;
          hszSvcPartner : HSZ;
          hszServiceReq : HSZ;
          hszTopic : HSZ;
          hszItem : HSZ;
          wFmt : UINT;
          wType : UINT;
          wStatus : UINT;
          wConvst : UINT;
          wLastError : UINT;
          hConvList : HCONVLIST;
          ConvCtxt : CONVCONTEXT;
          _hwnd : HWND;
          hwndPartner : HWND;
       end;
     tagCONVINFO = CONVINFO;
     TCONVINFO = CONVINFO;
     PCONVINFO = ^CONVINFO;

     COPYDATASTRUCT = record
          dwData : ULONG_PTR;
          cbData : DWORD;
          lpData : PVOID;
       end;
     tagCOPYDATASTRUCT = COPYDATASTRUCT;
     TCOPYDATASTRUCT = COPYDATASTRUCT;
     PCOPYDATASTRUCT = ^COPYDATASTRUCT;

     CPINFO = record
          MaxCharSize : UINT;
          DefaultChar : array[0..(MAX_DEFAULTCHAR)-1] of BYTE;
          LeadByte : array[0..(MAX_LEADBYTES)-1] of BYTE;
       end;
     LPCPINFO = ^CPINFO;
     _cpinfo = CPINFO;
     Tcpinfo = CPINFO;
     Pcpinfo = ^CPINFO;

     CPINFOEXA = record
        MaxCharSize    : UINT;
        DefaultChar    : array[0..(MAX_DEFAULTCHAR)-1] of BYTE;
        LeadByte       : array[0..(MAX_LEADBYTES)-1] of BYTE;
        DefaultUnicode : WCHAR;
        CodePage       : UINT;
        CodePageName   : array[0..(MAX_PATH)-1] of CHAR;
      end;
     LPCPINFOEXA = ^CPINFOEXA;
     _cpinfoexA  = CPINFOEXA;
     TcpinfoexA  = CPINFOEXA;
     PcpinfoexA  = ^CPINFOEXA;

     CPINFOEXW = record
        MaxCharSize    : UINT;
        DefaultChar    : array[0..(MAX_DEFAULTCHAR)-1] of BYTE;
        LeadByte       : array[0..(MAX_LEADBYTES)-1] of BYTE;
        DefaultUnicode : WCHAR;
        CodePage       : UINT;
        CodePageName   : array[0..(MAX_PATH)-1] of WCHAR;
     end;
     LPCPINFOEXW = ^CPINFOEXW;
     _cpinfoexW  = CPINFOEXW;
     TcpinfoexW  = CPINFOEXW;
     PcpinfoexW  = ^CPINFOEXW;

{$ifndef UNICODE}
     CPINFOEX   = CPINFOEXA;
     LPCPINFOEX = PCPINFOEXA;
     _cpinfoex  = CPINFOEXA;
     Tcpinfoex  = CPINFOEXA;
     Pcpinfoex  = PCPINFOEXA;
{$ELSE}
     CPINFOEX   = CPINFOEXW;
     LPCPINFOEX = PCPINFOEXW;
     _cpinfoex  = CPINFOEXW;
     Tcpinfoex  = CPINFOEXW;
     Pcpinfoex  = PCPINFOEXW;
{$ENDIF}

     CPLINFO = record
          idIcon : longint;
          idName : longint;
          idInfo : longint;
          lData : LONG_PTR;
       end;
     tagCPLINFO = CPLINFO;
     TCPLINFO = CPLINFO;
     PCPLINFO = ^CPLINFO;

     CREATE_PROCESS_DEBUG_INFO = record
          hFile : HANDLE;
          hProcess : HANDLE;
          hThread : HANDLE;
          lpBaseOfImage : LPVOID;
          dwDebugInfoFileOffset : DWORD;
          nDebugInfoSize : DWORD;
          lpThreadLocalBase : LPVOID;
          lpStartAddress : LPTHREAD_START_ROUTINE;
          lpImageName : LPVOID;
          fUnicode : WORD;
       end;
     _CREATE_PROCESS_DEBUG_INFO = CREATE_PROCESS_DEBUG_INFO;
     TCREATEPROCESSDEBUGINFO = CREATE_PROCESS_DEBUG_INFO;
     PCREATEPROCESSDEBUGINFO = ^CREATE_PROCESS_DEBUG_INFO;

     CREATE_THREAD_DEBUG_INFO = record
          hThread : HANDLE;
          lpThreadLocalBase : LPVOID;
          lpStartAddress : LPTHREAD_START_ROUTINE;
       end;
     _CREATE_THREAD_DEBUG_INFO = CREATE_THREAD_DEBUG_INFO;
     TCREATETHREADDEBUGINFO = CREATE_THREAD_DEBUG_INFO;
     PCREATETHREADDEBUGINFO = ^CREATE_THREAD_DEBUG_INFO;

     CURRENCYFMTA = record
          NumDigits     : UINT;
          LeadingZero   : UINT;
          Grouping      : UINT;
          lpDecimalSep  : LPSTR;
          lpThousandSep : LPSTR;
          NegativeOrder : UINT;
          PositiveOrder : UINT;
          lpCurrencySymbol : LPSTR;
       end;
     _currencyfmtA = CURRENCYFMTA;
     TcurrencyfmtA = CURRENCYFMTA;
     PcurrencyfmtA = ^CURRENCYFMTA;

     CURRENCYFMTW = record
          NumDigits     : UINT;
          LeadingZero   : UINT;
          Grouping      : UINT;
          lpDecimalSep  : LPWSTR;
          lpThousandSep : LPWSTR;
          NegativeOrder : UINT;
          PositiveOrder : UINT;
          lpCurrencySymbol : LPWSTR;
       end;

     _CURRENCYFMTW = CURRENCYFMTW;
     TCURRENCYFMTW = CURRENCYFMTW;
     PCURRENCYFMTW = ^CURRENCYFMTW;

     {$ifdef Unicode}
     _CURRENCYFMT  = CURRENCYFMTW;
     CURRENCYFMT   = CURRENCYFMTW;
     TCURRENCYFMT  = CURRENCYFMTW;
     PCURRENCYFMT  = ^CURRENCYFMTW;
     {$else}
     _CURRENCYFMT  = CURRENCYFMTA;
     CURRENCYFMT   = CURRENCYFMTA;
     TCURRENCYFMT  = CURRENCYFMTA;
     PCURRENCYFMT  = ^CURRENCYFMTA;
     {$endif}

     CURSORSHAPE = record
          xHotSpot : longint;
          yHotSpot : longint;
          cx : longint;
          cy : longint;
          cbWidth : longint;
          Planes : BYTE;
          BitsPixel : BYTE;
       end;
     LPCURSORSHAPE = ^CURSORSHAPE;
     tagCURSORSHAPE = CURSORSHAPE;
     TCURSORSHAPE = CURSORSHAPE;
     PCURSORSHAPE = ^CURSORSHAPE;

     CWPRETSTRUCT = record
          lResult : LRESULT;
          lParam : LPARAM;
          wParam : WPARAM;
          message : DWORD;
          hwnd : HWND;
       end;
     tagCWPRETSTRUCT = CWPRETSTRUCT;
     TCWPRETSTRUCT = CWPRETSTRUCT;
     PCWPRETSTRUCT = ^CWPRETSTRUCT;

     CWPSTRUCT = record
          lParam : LPARAM;
          wParam : WPARAM;
          message : UINT;
          hwnd : HWND;
       end;
     tagCWPSTRUCT = CWPSTRUCT;
     TCWPSTRUCT = CWPSTRUCT;
     PCWPSTRUCT = ^CWPSTRUCT;

     DATATYPES_INFO_1 = record
          pName : LPTSTR;
       end;
     _DATATYPES_INFO_1 = DATATYPES_INFO_1;
     TDATATYPESINFO1 = DATATYPES_INFO_1;
     PDATATYPESINFO1 = ^DATATYPES_INFO_1;

     DDEACK = record
          flag0 : word;
       end;
     TDDEACK = DDEACK;
     PDDEACK = ^DDEACK;
  const
     bm_DDEACK_bAppReturnCode = $FF;
     bp_DDEACK_bAppReturnCode = 0;
     bm_DDEACK_reserved = $3F00;
     bp_DDEACK_reserved = 8;
     bm_DDEACK_fBusy = $4000;
     bp_DDEACK_fBusy = 14;
     bm_DDEACK_fAck = $8000;
     bp_DDEACK_fAck = 15;
  function bAppReturnCode(var a : DDEACK) : word;
  procedure set_bAppReturnCode(var a : DDEACK; __bAppReturnCode : word);
  function reserved(var a : DDEACK) : word;
  procedure set_reserved(var a : DDEACK; __reserved : word);
  function fBusy(var a : DDEACK) : word;
  procedure set_fBusy(var a : DDEACK; __fBusy : word);
  function fAck(var a : DDEACK) : word;
  procedure set_fAck(var a : DDEACK; __fAck : word);

  type

     DDEADVISE = record
          flag0 : word;
          cfFormat : SHORT;
       end;
     TDDEADVISE = DDEADVISE;
     PDDEADVISE = ^DDEADVISE;
  const
     bm_DDEADVISE_reserved = $3FFF;
     bp_DDEADVISE_reserved = 0;
     bm_DDEADVISE_fDeferUpd = $4000;
     bp_DDEADVISE_fDeferUpd = 14;
     bm_DDEADVISE_fAckReq = $8000;
     bp_DDEADVISE_fAckReq = 15;
  function reserved(var a : DDEADVISE) : word;
  procedure set_reserved(var a : DDEADVISE; __reserved : word);
  function fDeferUpd(var a : DDEADVISE) : word;
  procedure set_fDeferUpd(var a : DDEADVISE; __fDeferUpd : word);
  function fAckReq(var a : DDEADVISE) : word;
  procedure set_fAckReq(var a : DDEADVISE; __fAckReq : word);

  type

     DDEDATA = record
          flag0 : word;
          cfFormat : SHORT;
          Value : array[0..0] of BYTE;
       end;
     PDDEDATA = ^DDEDATA;
  const
     bm_DDEDATA_unused = $FFF;
     bp_DDEDATA_unused = 0;
     bm_DDEDATA_fResponse = $1000;
     bp_DDEDATA_fResponse = 12;
     bm_DDEDATA_fRelease = $2000;
     bp_DDEDATA_fRelease = 13;
     bm_DDEDATA_reserved = $4000;
     bp_DDEDATA_reserved = 14;
     bm_DDEDATA_fAckReq = $8000;
     bp_DDEDATA_fAckReq = 15;
  function unused(var a : DDEDATA) : word;
  procedure set_unused(var a : DDEDATA; __unused : word);
  function fResponse(var a : DDEDATA) : word;
  procedure set_fResponse(var a : DDEDATA; __fResponse : word);
  function fRelease(var a : DDEDATA) : word;
  procedure set_fRelease(var a : DDEDATA; __fRelease : word);
  function reserved(var a : DDEDATA) : word;
  procedure set_reserved(var a : DDEDATA; __reserved : word);
  function fAckReq(var a : DDEDATA) : word;
  procedure set_fAckReq(var a : DDEDATA; __fAckReq : word);

  type

     DDELN = record
          flag0 : word;
          cfFormat : SHORT;
       end;
     TDDELN = DDELN;
     PDDELN = ^DDELN;
  const
     bm_DDELN_unused = $1FFF;
     bp_DDELN_unused = 0;
     bm_DDELN_fRelease = $2000;
     bp_DDELN_fRelease = 13;
     bm_DDELN_fDeferUpd = $4000;
     bp_DDELN_fDeferUpd = 14;
     bm_DDELN_fAckReq = $8000;
     bp_DDELN_fAckReq = 15;
  function unused(var a : DDELN) : word;
  procedure set_unused(var a : DDELN; __unused : word);
  function fRelease(var a : DDELN) : word;
  procedure set_fRelease(var a : DDELN; __fRelease : word);
  function fDeferUpd(var a : DDELN) : word;
  procedure set_fDeferUpd(var a : DDELN; __fDeferUpd : word);
  function fAckReq(var a : DDELN) : word;
  procedure set_fAckReq(var a : DDELN; __fAckReq : word);

  type

     DDEML_MSG_HOOK_DATA = record
          uiLo : UINT_PTR;
          uiHi : UINT_PTR;
          cbData : DWORD;
          Data : array[0..7] of DWORD;
       end;
     tagDDEML_MSG_HOOK_DATA = DDEML_MSG_HOOK_DATA;
     TDDEMLMSGHOOKDATA = DDEML_MSG_HOOK_DATA;
     PDDEMLMSGHOOKDATA = ^DDEML_MSG_HOOK_DATA;

     DDEPOKE = record
          flag0 : word;
          cfFormat : SHORT;
          Value : array[0..0] of BYTE;
       end;
     TDDEPOKE = DDEPOKE;
     PDDEPOKE = ^DDEPOKE;
  const
     bm_DDEPOKE_unused = $1FFF;
     bp_DDEPOKE_unused = 0;
     bm_DDEPOKE_fRelease = $2000;
     bp_DDEPOKE_fRelease = 13;
     bm_DDEPOKE_fReserved = $C000;
     bp_DDEPOKE_fReserved = 14;
  function unused(var a : DDEPOKE) : word;
  procedure set_unused(var a : DDEPOKE; __unused : word);
  function fRelease(var a : DDEPOKE) : word;
  procedure set_fRelease(var a : DDEPOKE; __fRelease : word);
  function fReserved(var a : DDEPOKE) : word;
  procedure set_fReserved(var a : DDEPOKE; __fReserved : word);

  type

     DDEUP = record
          flag0 : word;
          cfFormat : SHORT;
          rgb : array[0..0] of BYTE;
       end;
     TDDEUP = DDEUP;
     PDDEUP = ^DDEUP;
  const
     bm_DDEUP_unused = $FFF;
     bp_DDEUP_unused = 0;
     bm_DDEUP_fAck = $1000;
     bp_DDEUP_fAck = 12;
     bm_DDEUP_fRelease = $2000;
     bp_DDEUP_fRelease = 13;
     bm_DDEUP_fReserved = $4000;
     bp_DDEUP_fReserved = 14;
     bm_DDEUP_fAckReq = $8000;
     bp_DDEUP_fAckReq = 15;
  function unused(var a : DDEUP) : word;
  procedure set_unused(var a : DDEUP; __unused : word);
  function fAck(var a : DDEUP) : word;
  procedure set_fAck(var a : DDEUP; __fAck : word);
  function fRelease(var a : DDEUP) : word;
  procedure set_fRelease(var a : DDEUP; __fRelease : word);
  function fReserved(var a : DDEUP) : word;
  procedure set_fReserved(var a : DDEUP; __fReserved : word);
  function fAckReq(var a : DDEUP) : word;
  procedure set_fAckReq(var a : DDEUP; __fAckReq : word);

  type

     EXCEPTION_RECORD = record
          ExceptionCode : DWORD;
          ExceptionFlags : DWORD;
          ExceptionRecord : ^_EXCEPTION_RECORD;
          ExceptionAddress : PVOID;
          NumberParameters : DWORD;
          ExceptionInformation : array[0..(EXCEPTION_MAXIMUM_PARAMETERS)-1] of ULONG_PTR;
       end;
     PEXCEPTION_RECORD = ^EXCEPTION_RECORD;
     _EXCEPTION_RECORD = EXCEPTION_RECORD;
     TEXCEPTIONRECORD = EXCEPTION_RECORD;
     PEXCEPTIONRECORD = ^EXCEPTION_RECORD;

     EXCEPTION_DEBUG_INFO = record
          ExceptionRecord : EXCEPTION_RECORD;
          dwFirstChance : DWORD;
       end;
     PEXCEPTION_DEBUG_INFO = ^EXCEPTION_DEBUG_INFO;
     _EXCEPTION_DEBUG_INFO = EXCEPTION_DEBUG_INFO;
     TEXCEPTIONDEBUGINFO = EXCEPTION_DEBUG_INFO;
     PEXCEPTIONDEBUGINFO = ^EXCEPTION_DEBUG_INFO;

     { Explicit forms to debug targets running on a different architecture (32-bit versus 64-bit)}

     EXCEPTION_RECORD32 = record
          ExceptionCode : DWORD;
          ExceptionFlags : DWORD;
          ExceptionRecord : DWORD;
          ExceptionAddress : DWORD;
          NumberParameters : DWORD;
          ExceptionInformation : array[0..(EXCEPTION_MAXIMUM_PARAMETERS)-1] of DWORD;
       end;
     PEXCEPTION_RECORD32 = ^EXCEPTION_RECORD32;
     _EXCEPTION_RECORD32 = EXCEPTION_RECORD32;
     TExceptionRecord32 = EXCEPTION_RECORD32;
     PExceptionRecord32 = ^EXCEPTION_RECORD32;

     EXCEPTION_DEBUG_INFO32 = record
          ExceptionRecord : EXCEPTION_RECORD32;
          dwFirstChance : DWORD;
       end;
     PEXCEPTION_DEBUG_INFO32 = ^EXCEPTION_DEBUG_INFO32;
     _EXCEPTION_DEBUG_INFO32 = EXCEPTION_DEBUG_INFO32;
     TExceptionDebugInfo32 = EXCEPTION_DEBUG_INFO32;
     PExceptionDebugInfo32 = ^EXCEPTION_DEBUG_INFO32;

     EXCEPTION_RECORD64 = record
          ExceptionCode : DWORD;
          ExceptionFlags : DWORD;
          ExceptionRecord : DWORD64;
          ExceptionAddress : DWORD64;
          NumberParameters : DWORD;
          __unusedAlignment : DWORD;
          ExceptionInformation : array[0..(EXCEPTION_MAXIMUM_PARAMETERS)-1] of DWORD64;
       end;
     PEXCEPTION_RECORD64 = ^EXCEPTION_RECORD64;
     _EXCEPTION_RECORD64 = EXCEPTION_RECORD64;
     TExceptionRecord64 = EXCEPTION_RECORD64;
     PExceptionRecord64 = ^EXCEPTION_RECORD64;

     EXCEPTION_DEBUG_INFO64 = record
          ExceptionRecord : EXCEPTION_RECORD64;
          dwFirstChance : DWORD;
       end;
     PEXCEPTION_DEBUG_INFO64 = ^EXCEPTION_DEBUG_INFO64;
     _EXCEPTION_DEBUG_INFO64 = EXCEPTION_DEBUG_INFO64;
     TExceptionDebugInfo64 = EXCEPTION_DEBUG_INFO64;
     PExceptionDebugInfo64 = ^EXCEPTION_DEBUG_INFO64;

     EXIT_PROCESS_DEBUG_INFO = record
          dwExitCode : DWORD;
       end;
     _EXIT_PROCESS_DEBUG_INFO = EXIT_PROCESS_DEBUG_INFO;
     TEXITPROCESSDEBUGINFO = EXIT_PROCESS_DEBUG_INFO;
     PEXITPROCESSDEBUGINFO = ^EXIT_PROCESS_DEBUG_INFO;


     EXIT_THREAD_DEBUG_INFO = record
          dwExitCode : DWORD;
       end;
     _EXIT_THREAD_DEBUG_INFO = EXIT_THREAD_DEBUG_INFO;
     TEXITTHREADDEBUGINFO = EXIT_THREAD_DEBUG_INFO;
     PEXITTHREADDEBUGINFO = ^EXIT_THREAD_DEBUG_INFO;

     LOAD_DLL_DEBUG_INFO = record
          hFile : HANDLE;
          lpBaseOfDll : LPVOID;
          dwDebugInfoFileOffset : DWORD;
          nDebugInfoSize : DWORD;
          lpImageName : LPVOID;
          fUnicode : WORD;
       end;
     _LOAD_DLL_DEBUG_INFO = LOAD_DLL_DEBUG_INFO;
     TLOADDLLDEBUGINFO = LOAD_DLL_DEBUG_INFO;
     PLOADDLLDEBUGINFO = ^LOAD_DLL_DEBUG_INFO;

     UNLOAD_DLL_DEBUG_INFO = record
          lpBaseOfDll : LPVOID;
       end;
     _UNLOAD_DLL_DEBUG_INFO = UNLOAD_DLL_DEBUG_INFO;
     TUNLOADDLLDEBUGINFO = UNLOAD_DLL_DEBUG_INFO;
     PUNLOADDLLDEBUGINFO = ^UNLOAD_DLL_DEBUG_INFO;

     OUTPUT_DEBUG_STRING_INFO = record
          lpDebugStringData : LPSTR;
          fUnicode : WORD;
          nDebugStringLength : WORD;
       end;
     _OUTPUT_DEBUG_STRING_INFO = OUTPUT_DEBUG_STRING_INFO;
     TOUTPUTDEBUGSTRINGINFO = OUTPUT_DEBUG_STRING_INFO;
     POUTPUTDEBUGSTRINGINFO = ^OUTPUT_DEBUG_STRING_INFO;

     RIP_INFO = record
          dwError : DWORD;
          dwType : DWORD;
       end;
     _RIP_INFO = RIP_INFO;
     TRIPINFO = RIP_INFO;
     PRIPINFO = ^RIP_INFO;

     DEBUG_EVENT = record
          dwDebugEventCode : DWORD;
          dwProcessId : DWORD;
          dwThreadId : DWORD;
              case longint of
                 0 : ( Exception : EXCEPTION_DEBUG_INFO );
                 1 : ( CreateThread : CREATE_THREAD_DEBUG_INFO );
                 2 : ( CreateProcessInfo : CREATE_PROCESS_DEBUG_INFO );
                 3 : ( ExitThread : EXIT_THREAD_DEBUG_INFO );
                 4 : ( ExitProcess : EXIT_PROCESS_DEBUG_INFO );
                 5 : ( LoadDll : LOAD_DLL_DEBUG_INFO );
                 6 : ( UnloadDll : UNLOAD_DLL_DEBUG_INFO );
                 7 : ( DebugString : OUTPUT_DEBUG_STRING_INFO );
                 8 : ( RipInfo : RIP_INFO );
       end;
     LPDEBUG_EVENT = ^DEBUG_EVENT;
     _DEBUG_EVENT = DEBUG_EVENT;
     TDEBUGEVENT = DEBUG_EVENT;
     PDEBUGEVENT = ^DEBUG_EVENT;

     DEBUGHOOKINFO = record
          idThread : DWORD;
          idThreadInstaller : DWORD;
          lParam : LPARAM;
          wParam : WPARAM;
          code : longint;
       end;
     tagDEBUGHOOKINFO = DEBUGHOOKINFO;
     TDEBUGHOOKINFO = DEBUGHOOKINFO;
     PDEBUGHOOKINFO = ^DEBUGHOOKINFO;

     DELETEITEMSTRUCT = record
          CtlType : UINT;
          CtlID : UINT;
          itemID : UINT;
          hwndItem : HWND;
          itemData : ULONG_PTR;
       end;
     tagDELETEITEMSTRUCT = DELETEITEMSTRUCT;
     TDELETEITEMSTRUCT = DELETEITEMSTRUCT;
     PDELETEITEMSTRUCT = ^DELETEITEMSTRUCT;

     DEV_BROADCAST_HDR = record
          dbch_size : ULONG;
          dbch_devicetype : ULONG;
          dbch_reserved : ULONG;
       end;
     PDEV_BROADCAST_HDR = ^DEV_BROADCAST_HDR;
     _DEV_BROADCAST_HDR = DEV_BROADCAST_HDR;
     TDEVBROADCASTHDR = DEV_BROADCAST_HDR;
     PDEVBROADCASTHDR = ^DEV_BROADCAST_HDR;

     DEV_BROADCAST_OEM = record
          dbco_size : DWORD;
          dbco_devicetype : DWORD;
          dbco_reserved : DWORD;
          dbco_identifier : DWORD;
          dbco_suppfunc : DWORD;
       end;
     PDEV_BROADCAST_OEM = ^DEV_BROADCAST_OEM;
     _DEV_BROADCAST_OEM = DEV_BROADCAST_OEM;
     TDEVBROADCASTOEM = DEV_BROADCAST_OEM;
     PDEVBROADCASTOEM = ^DEV_BROADCAST_OEM;

     DEV_BROADCAST_PORT = record
          dbcp_size : DWORD;
          dbcp_devicetype : DWORD;
          dbcp_reserved : DWORD;
          dbcp_name : array[0..0] of TCHAR;
       end;
     PDEV_BROADCAST_PORT = ^DEV_BROADCAST_PORT;
     _DEV_BROADCAST_PORT = DEV_BROADCAST_PORT;
     TDEVBROADCASTPORT = DEV_BROADCAST_PORT;
     PDEVBROADCASTPORT = ^DEV_BROADCAST_PORT;

     _DEV_BROADCAST_USERDEFINED = record
          dbud_dbh : _DEV_BROADCAST_HDR;
          dbud_szName : array[0..0] of char;
          dbud_rgbUserDefined : array[0..0] of BYTE;
       end;
     TDEVBROADCASTUSERDEFINED = _DEV_BROADCAST_USERDEFINED;
     PDEVBROADCASTUSERDEFINED = ^_DEV_BROADCAST_USERDEFINED;

     DEV_BROADCAST_VOLUME = record
          dbcv_size : DWORD;
          dbcv_devicetype : DWORD;
          dbcv_reserved : DWORD;
          dbcv_unitmask : DWORD;
          dbcv_flags : WORD;
       end;
     PDEV_BROADCAST_VOLUME = ^DEV_BROADCAST_VOLUME;
     _DEV_BROADCAST_VOLUME = DEV_BROADCAST_VOLUME;
     TDEVBROADCASTVOLUME = DEV_BROADCAST_VOLUME;
     PDEVBROADCASTVOLUME = ^DEV_BROADCAST_VOLUME;

     DEVMODEA = record
          dmDeviceName : array[0..(CCHDEVICENAME)-1] of AnsiChar;
          dmSpecVersion : WORD;
          dmDriverVersion : WORD;
          dmSize : WORD;
          dmDriverExtra : WORD;
          dmFields : DWORD;
          case byte of
            1: (dmOrientation : SmallInt;
                dmPaperSize : SmallInt;
                dmPaperLength : SmallInt;
                dmPaperWidth : SmallInt;
                dmScale : SmallInt;
                dmCopies : SmallInt;
                dmDefaultSource : SmallInt;
                dmPrintQuality : SmallInt;
                dmColor : SmallInt;
                dmDuplex : SmallInt;
                dmYResolution : SmallInt;
                dmTTOption : SmallInt;
                dmCollate : SmallInt;
                dmFormName : array[0..(CCHFORMNAME)-1] of AnsiCHAR;
                dmLogPixels : WORD;
                dmBitsPerPel : DWORD;
                dmPelsWidth : DWORD;
                dmPelsHeight : DWORD;
                dmDisplayFlags : DWORD;
                dmDisplayFrequency : DWORD;
                dmICMMethod : DWORD;
                dmICMIntent : DWORD;
                dmMediaType : DWORD;
                dmDitherType : DWORD;
                dmICCManufacturer : DWORD;
                dmICCModel : DWORD
               );
            2: (dmPosition: POINTL;
                dmDisplayOrientation: DWORD;
                dmDisplayFixedOutput: DWORD;
               );
       end;

     LPDEVMODEA = ^DEVMODEA;
     _DEVMODEA  = DEVMODEA;
     TDEVMODEA  = DEVMODEA;
     PDEVMODEA  = LPDEVMODEA;

     _devicemodeA = DEVMODEA;
     devicemodeA  = DEVMODEA;
     tdevicemodeA = DEVMODEA;
     PDeviceModeA = LPDEVMODEA;

     devmodeW   = record
                     dmDeviceName   : array[0.. CCHDEVICENAME-1] of WCHAR;
                     dmSpecVersion  : WORD;
                     dmDriverVersion: WORD;
                     dmSize        : WORD;
                     dmDriverExtra  : WORD;
                     dmFields       : DWORD;
                     case byte of
                  1: (
                     dmOrientation  : short;
                     dmPaperSize    : short;
                     dmPaperLength  : short;
                     dmPaperWidth   : short;
                     dmScale       : short;
                     dmCopies      : short;
                     dmDefaultSource: short;
                     dmPrintQuality : short;
                     dmColor       : short;
                     dmDuplex      : short;
                     dmYResolution  : short;
                     dmTTOption    : short;
                     dmCollate     : short;
                     dmFormName     : array [0..CCHFORMNAME-1] of wchar;
                     dmLogPixels    : WORD;
                     dmBitsPerPel   : DWORD;
                     dmPelsWidth    : DWORD;
                     dmPelsHeight   : DWORD;
                     dmDisplayFlags : DWORD;
                     dmDisplayFrequency     : DWORD;
                     dmICMMethod    : DWORD;
                     dmICMIntent    : DWORD;
                     dmMediaType    : DWORD;
                     dmDitherType   : DWORD;
                     dmReserved1    : DWORD;
                     dmReserved2    : DWORD;
                     dmPanningWidth : DWORD;
                     dmPanningHeight: DWORD;
                     );
                  2: (dmPosition: POINTL;
                      dmDisplayOrientation: DWORD;
                      dmDisplayFixedOutput: DWORD;
                     );
                   end;

     LPDEVMODEW   = ^DEVMODEW;
     _DEVMODEW    = DEVMODEW;
     TDEVMODEW    = DEVMODEW;
     PDEVMODEW    = LPDEVMODEW;

     _devicemodeW = DEVMODEW;
     devicemodeW  = DEVMODEW;
     TDeviceModeW = DEVMODEW;
     PDeviceModeW = LPDEVMODEW;


    {$IFNDEF UNICODE}
      DEVMODE      = DEVMODEA;
      LPDEVMODE    = ^DEVMODEA;
      _DEVMODE     = DEVMODEA;
      TDEVMODE     = DEVMODEA;
      PDEVMODE     = ^DEVMODEA;
    {$ELSE}
      DEVMODE      = DEVMODEW;
      LPDEVMODE    = ^DEVMODEW;
      _DEVMODE     = DEVMODEW;
      TDEVMODE     = DEVMODEW;
      PDEVMODE     = ^DEVMODEW;
    {$ENDIF}
  
    _devicemode  = DEVMODE;
    DeviceMode   = DEVMODE;
    TDeviceMode  = DEVMODE;
    PDeviceMode  = ^DEVMODE;
  
     DEVNAMES = record
          wDriverOffset : WORD;
          wDeviceOffset : WORD;
          wOutputOffset : WORD;
          wDefault : WORD;
       end;
     LPDEVNAMES = ^DEVNAMES;
     tagDEVNAMES = DEVNAMES;
     TDEVNAMES = DEVNAMES;
     PDEVNAMES = ^DEVNAMES;

     DIBSECTION = record
          dsBm : BITMAP;
          dsBmih : BITMAPINFOHEADER;
          dsBitfields : array[0..2] of DWORD;
          dshSection : HANDLE;
          dsOffset : DWORD;
       end;
     tagDIBSECTION = DIBSECTION;
     TDIBSECTION = DIBSECTION;
     PDIBSECTION = ^DIBSECTION;

     LARGE_INTEGER = record
        case byte of
          0: (LowPart : DWORD;
              HighPart : LONG);
          1: (QuadPart : LONGLONG);
       end;
     PLARGE_INTEGER = ^LARGE_INTEGER;
     _LARGE_INTEGER = LARGE_INTEGER;

     TLargeInteger = Int64;
     PLargeInteger = ^TLargeInteger;

     ULARGE_INTEGER = record
        case byte of
          0: (LowPart : DWORD;
              HighPart : DWORD);
          1: (QuadPart : ULONGLONG);
       end;
     PULARGE_INTEGER = ^ULARGE_INTEGER;
     _ULARGE_INTEGER = ULARGE_INTEGER;

     TULargeInteger = QWord;
     PULargeInteger = ^TULargeInteger;

     DISK_GEOMETRY = record
          Cylinders : LARGE_INTEGER;
          MediaType : MEDIA_TYPE;
          TracksPerCylinder : DWORD;
          SectorsPerTrack : DWORD;
          BytesPerSector : DWORD;
       end;
     _DISK_GEOMETRY = DISK_GEOMETRY;
     TDISKGEOMETRY = DISK_GEOMETRY;
     PDISKGEOMETRY = ^DISK_GEOMETRY;

     DISK_PERFORMANCE = record
          BytesRead : LARGE_INTEGER;
          BytesWritten : LARGE_INTEGER;
          ReadTime : LARGE_INTEGER;
          WriteTime : LARGE_INTEGER;
          IdleTime : LARGE_INTEGER;
          ReadCount : DWORD;
          WriteCount : DWORD;
          QueueDepth : DWORD;
          SplitCount : DWORD;
          QueryTime : LARGE_INTEGER;
          StorageDeviceNumber : DWORD;
          StorageManagerName : array [0..7] of WCHAR;
       end;
     _DISK_PERFORMANCE = DISK_PERFORMANCE;
     TDISKPERFORMANCE = DISK_PERFORMANCE;
     PDISKPERFORMANCE = ^DISK_PERFORMANCE;

     DLGITEMTEMPLATE = packed record
          style : DWORD;
          dwExtendedStyle : DWORD;
          x : smallint;
          y : smallint;
          cx : smallint;
          cy : smallint;
          id : WORD;
       end;
     LPDLGITEMTEMPLATE = ^DLGITEMTEMPLATE;
     TDLGITEMTEMPLATE = DLGITEMTEMPLATE;
     PDLGITEMTEMPLATE = ^DLGITEMTEMPLATE;

     DLGTEMPLATE = packed record
          style : DWORD;
          dwExtendedStyle : DWORD;
          cdit : WORD;
          x : smallint;
          y : smallint;
          cx : smallint;
          cy : smallint;
       end;
     LPDLGTEMPLATE = ^DLGTEMPLATE;
     LPCDLGTEMPLATE = ^DLGTEMPLATE;
     TDLGTEMPLATE = DLGTEMPLATE;
     PDLGTEMPLATE = ^DLGTEMPLATE;

     DOC_INFO_1 = record
          pDocName : LPTSTR;
          pOutputFile : LPTSTR;
          pDatatype : LPTSTR;
       end;
     _DOC_INFO_1 = DOC_INFO_1;
     TDOCINFO1 = DOC_INFO_1;
     PDOCINFO1 = ^DOC_INFO_1;

     DOC_INFO_2 = record
          pDocName : LPTSTR;
          pOutputFile : LPTSTR;
          pDatatype : LPTSTR;
          dwMode : DWORD;
          JobId : DWORD;
       end;
     _DOC_INFO_2 = DOC_INFO_2;
     TDOCINFO2 = DOC_INFO_2;
     PDOCINFO2 = ^DOC_INFO_2;

     DOCINFOA = record
          cbSize : longint;
          lpszDocName  : LPCSTR;
          lpszOutput   : LPCSTR;
          lpszDatatype : LPCSTR;
          fwType : DWORD;
       end;
     TDOCINFOA = DOCINFOA;
     PDOCINFOa = ^DOCINFOA;

     DOCINFOW = record
          cbSize : longint;
          lpszDocName  : LPCWSTR;
          lpszOutput   : LPCWSTR;
          lpszDatatype : LPCWSTR;
          fwType : DWORD;
       end;
     TDOCINFOW = DOCINFOW;
     PDOCINFOW = ^DOCINFOW;

     {$ifdef Unicode}
     DOCINFO   = DOCINFOW;
     TDOCINFO  = DOCINFOW;
     PDOCINFO  = ^DOCINFOW;
     {$else}
     DOCINFO   = DOCINFOA;
     TDOCINFO  = DOCINFOA;
     PDOCINFO  = ^DOCINFOA;
     {$endif}

     DRAGLISTINFO = record
          uNotification : UINT;
          hWnd : HWND;
          ptCursor : POINT;
       end;
     LPDRAGLISTINFO = ^DRAGLISTINFO;
     TDRAGLISTINFO = DRAGLISTINFO;
     PDRAGLISTINFO = ^DRAGLISTINFO;

     DRAWITEMSTRUCT = record
          CtlType : UINT;
          CtlID : UINT;
          itemID : UINT;
          itemAction : UINT;
          itemState : UINT;
          hwndItem : HWND;
          hDC : HDC;
          rcItem : RECT;
          itemData : ULONG_PTR;
       end;
     LPDRAWITEMSTRUCT = ^DRAWITEMSTRUCT;
     tagDRAWITEMSTRUCT = DRAWITEMSTRUCT;
     TDRAWITEMSTRUCT = DRAWITEMSTRUCT;
     PDRAWITEMSTRUCT = ^DRAWITEMSTRUCT;

     DRAWTEXTPARAMS = record
          cbSize : UINT;
          iTabLength : longint;
          iLeftMargin : longint;
          iRightMargin : longint;
          uiLengthDrawn : UINT;
       end;
     LPDRAWTEXTPARAMS = ^DRAWTEXTPARAMS;
     TDRAWTEXTPARAMS = DRAWTEXTPARAMS;
     PDRAWTEXTPARAMS = ^DRAWTEXTPARAMS;

     PARTITION_INFORMATION = record
          StartingOffset : LARGE_INTEGER;
          PartitionLength: LARGE_INTEGER;
          HiddenSectors  : DWORD;
	  PartitionNumber: DWORD;
          PartitionType  : BYTE;
	  BootIndicator       : BYTEBOOL;
          RecognizedPartition : BYTEBOOL;
          RewritePartition    : BYTEBOOL;
       end;
     _PARTITION_INFORMATION = PARTITION_INFORMATION;
     TPARTITIONINFORMATION = PARTITION_INFORMATION;
     PPARTITIONINFORMATION = ^PARTITION_INFORMATION;

     PARTITION_INFORMATION_GPT = record
		                   PartitionType  : TGUID;
				   PartitionID    : TGUID;
				   Attributes     : DWORD64;
				   Name		  : array[0..35] OF WCHAR;
				  end; 	
     TPARTITION_INFORMATION_GPT= PARTITION_INFORMATION_GPT;
     PPARTITION_INFORMATION_GPT= ^PARTITION_INFORMATION_GPT;

     PARTITION_INFORMATION_MBR = record
		                   PartitionType  : Byte;
				   BootIndicator  : ByteBool;
				   RecognizedPartition : ByteBool;
				   HiddenSectors  : DWORD;
				  end; 	
     TPARTITION_INFORMATION_MBR=  PARTITION_INFORMATION_MBR;
     PPARTITION_INFORMATION_MBR= ^PARTITION_INFORMATION_MBR;

     SET_PARTITION_INFORMATION = record
          PartitionType : BYTE;
       end;
     _SET_PARTITION_INFORMATION = SET_PARTITION_INFORMATION;
     TSETPARTITIONINFORMATION = SET_PARTITION_INFORMATION;
     PSETPARTITIONINFORMATION = ^SET_PARTITION_INFORMATION;

     SET_PARTITION_INFORMATION_MBR  = PARTITION_INFORMATION;
     TSET_PARTITION_INFORMATION_MBR = PARTITION_INFORMATION;

     SET_PARTITION_INFORMATION_GPT  = PARTITION_INFORMATION_GPT;
     TSET_PARTITION_INFORMATION_GPT = PARTITION_INFORMATION_GPT;

     SET_PARTITION_INFORMATION_EX   = record
                                      PartitionStyle : TPartition_Style;
				      case integer of
					0 : (mbr : SET_PARTITION_INFORMATION_MBR);
					1 : (GPT : SET_PARTITION_INFORMATION_GPT);
				      end;
     TSET_PARTITION_INFORMATION_EX  = SET_PARTITION_INFORMATION_EX;
     PSET_PARTITION_INFORMATION_EX  = ^SET_PARTITION_INFORMATION_EX;

     CREATE_DISK_GPT = record
			 DiskID 	   : TGUID;
			 MaxPartitionCount : DWord;
		        end;
     TCREATE_DISK_GPT = CREATE_DISK_GPT;
     PCREATE_DISK_GPT = ^TCREATE_DISK_GPT;

     CREATE_DISK_MBR = record
			 Signature 	   : DWord;
		        end;
     TCREATE_DISK_MBR = CREATE_DISK_MBR;
     PCREATE_DISK_MBR = ^TCREATE_DISK_MBR;

     CREATE_DISK  = record
		       PartitionStyle : TPartition_Style;
		       case integer of
			 0: ( MBR : Create_Disk_MBR);
		         1: ( GPT : Create_Disk_GPT);	
		      end;
     TCREATE_DISK = CREATE_DISK;
     PCREATE_DISK = ^TCREATE_DISK;

     GET_LENGTH_INFORMATION = record
	                        Length : LARGE_INTEGER;
			      end;
     TGET_LENGTH_INFORMATION = GET_LENGTH_INFORMATION;
     PGET_LENGTH_INFORMATION = ^GET_LENGTH_INFORMATION;

     PARTITION_INFORMATION_EX = record
	  PartitionStyle  : TPartition_Style;
          StartingOffset  : LARGE_INTEGER;
          PartitionLength : LARGE_INTEGER;
	  PartitionNumber : DWORD;
          RewritePartition: BYTEBOOL;
	  case integer of
	    0: ( MBR : PARTITION_INFORMATION_MBR);
	    1: ( GPT : PARTITION_INFORMATION_GPT);	
       end;
     _PARTITION_INFORMATION_EX = PARTITION_INFORMATION_EX;
     TPARTITIONINFORMATION_EX  = PARTITION_INFORMATION_EX;
     PPARTITIONINFORMATION_EX  = ^PARTITION_INFORMATION_EX;

     DRIVE_LAYOUT_INFORMATION = record
          PartitionCount : DWORD;
          Signature : DWORD;
          PartitionEntry : array[0..0] of PARTITION_INFORMATION;
       end;
     _DRIVE_LAYOUT_INFORMATION = DRIVE_LAYOUT_INFORMATION;
     TDRIVELAYOUTINFORMATION = DRIVE_LAYOUT_INFORMATION;
     PDRIVELAYOUTINFORMATION = ^DRIVE_LAYOUT_INFORMATION;

     DRIVER_INFO_1 = record
          pName : LPTSTR;
       end;
     _DRIVER_INFO_1 = DRIVER_INFO_1;
     TDRIVERINFO1 = DRIVER_INFO_1;
     PDRIVERINFO1 = ^DRIVER_INFO_1;

     DRIVER_INFO_2 = record
          cVersion : DWORD;
          pName : LPTSTR;
          pEnvironment : LPTSTR;
          pDriverPath : LPTSTR;
          pDataFile : LPTSTR;
          pConfigFile : LPTSTR;
       end;
     _DRIVER_INFO_2 = DRIVER_INFO_2;
     TDRIVERINFO2 = DRIVER_INFO_2;
     PDRIVERINFO2 = ^DRIVER_INFO_2;

     DRIVER_INFO_3 = record
          cVersion : DWORD;
          pName : LPTSTR;
          pEnvironment : LPTSTR;
          pDriverPath : LPTSTR;
          pDataFile : LPTSTR;
          pConfigFile : LPTSTR;
          pHelpFile : LPTSTR;
          pDependentFiles : LPTSTR;
          pMonitorName : LPTSTR;
          pDefaultDataType : LPTSTR;
       end;
     _DRIVER_INFO_3 = DRIVER_INFO_3;
     TDRIVERINFO3 = DRIVER_INFO_3;
     PDRIVERINFO3 = ^DRIVER_INFO_3;

     EDITSTREAM = record
          dwCookie : DWORD_PTR;
          dwError : DWORD;
          pfnCallback : EDITSTREAMCALLBACK;
       end;
     _editstream = EDITSTREAM;
     Teditstream = EDITSTREAM;
     Peditstream = ^EDITSTREAM;

     EMR = record
          iType : DWORD;
          nSize : DWORD;
       end;
     tagEMR = EMR;
     TEMR = EMR;
     PEMR = ^EMR;

     EMRANGLEARC = record
          emr : EMR;
          ptlCenter : POINTL;
          nRadius : DWORD;
          eStartAngle : Single;
          eSweepAngle : Single;
       end;
     tagEMRANGLEARC = EMRANGLEARC;
     TEMRANGLEARC = EMRANGLEARC;
     PEMRANGLEARC = ^EMRANGLEARC;

     EMRARC = record
          emr : EMR;
          rclBox : RECTL;
          ptlStart : POINTL;
          ptlEnd : POINTL;
       end;
     tagEMRARC = EMRARC;
     TEMRARC = EMRARC;
     PEMRARC = ^EMRARC;

     EMRARCTO = EMRARC;
     TEMRARCTO = EMRARC;
     PEMRARCTO = ^EMRARC;

     EMRCHORD = EMRARC;
     TEMRCHORD = EMRARC;
     PEMRCHORD = ^EMRARC;

     EMRPIE = EMRARC;
     TEMRPIE = EMRARC;
     PEMRPIE = ^EMRARC;

     XFORM = record
          eM11 : Single;
          eM12 : Single;
          eM21 : Single;
          eM22 : Single;
          eDx : Single;
          eDy : Single;
       end;
     LPXFORM = ^XFORM;
     _XFORM = XFORM;
     TXFORM = XFORM;
     PXFORM = ^XFORM;

     EMRBITBLT = record
          emr : EMR;
          rclBounds : RECTL;
          xDest : LONG;
          yDest : LONG;
          cxDest : LONG;
          cyDest : LONG;
          dwRop : DWORD;
          xSrc : LONG;
          ySrc : LONG;
          xformSrc : XFORM;
          crBkColorSrc : COLORREF;
          iUsageSrc : DWORD;
          offBmiSrc : DWORD;
          cbBmiSrc : DWORD;
          offBitsSrc : DWORD;
          cbBitsSrc : DWORD;
       end;
     tagEMRBITBLT = EMRBITBLT;
     TEMRBITBLT = EMRBITBLT;
     PEMRBITBLT = ^EMRBITBLT;

     LOGBRUSH = record
          lbStyle : UINT;
          lbColor : COLORREF;
          lbHatch : ULONG_PTR;
       end;
     tagLOGBRUSH = LOGBRUSH;
     TLOGBRUSH = LOGBRUSH;
     PLOGBRUSH = ^LOGBRUSH;

     EMRCREATEBRUSHINDIRECT = record
          emr : EMR;
          ihBrush : DWORD;
          lb : LOGBRUSH;
       end;
     tagEMRCREATEBRUSHINDIRECT = EMRCREATEBRUSHINDIRECT;
     TEMRCREATEBRUSHINDIRECT = EMRCREATEBRUSHINDIRECT;
     PEMRCREATEBRUSHINDIRECT = ^EMRCREATEBRUSHINDIRECT;

     LCSCSTYPE = LONG;

     LCSGAMUTMATCH = LONG;

     LOGCOLORSPACEA = record
          lcsSignature : DWORD;
          lcsVersion : DWORD;
          lcsSize : DWORD;
          lcsCSType : LCSCSTYPE;
          lcsIntent : LCSGAMUTMATCH;
          lcsEndpoints : CIEXYZTRIPLE;
          lcsGammaRed : DWORD;
          lcsGammaGreen : DWORD;
          lcsGammaBlue : DWORD;
          lcsFilename : array[0..(MAX_PATH)-1] of ANSICHAR;
       end;
     LPLOGCOLORSPACEA  = ^LOGCOLORSPACEA;
     tagLOGCOLORSPACEA = LOGCOLORSPACEA;
     TLOGCOLORSPACEA   = LOGCOLORSPACEA;
     PLOGCOLORSPACEA   = ^LOGCOLORSPACEA;

     LOGCOLORSPACEW = record
          lcsSignature : DWORD;
          lcsVersion : DWORD;
          lcsSize : DWORD;
          lcsCSType : LCSCSTYPE;
          lcsIntent : LCSGAMUTMATCH;
          lcsEndpoints : CIEXYZTRIPLE;
          lcsGammaRed : DWORD;
          lcsGammaGreen : DWORD;
          lcsGammaBlue : DWORD;
          lcsFilename : array[0..(MAX_PATH)-1] of WIDECHAR;
       end;
     LPLOGCOLORSPACEW  = ^LOGCOLORSPACEW;
     tagLOGCOLORSPACEW = LOGCOLORSPACEW;
     TLOGCOLORSPACEW   = LOGCOLORSPACEW;
     PLOGCOLORSPACEW   = ^LOGCOLORSPACEW;
     {$ifdef UNICODE}
        LPLOGCOLORSPACE  = PLOGCOLORSPACEW;
        tagLOGCOLORSPACE = LOGCOLORSPACEW;
        TLOGCOLORSPACE   = LOGCOLORSPACEW;
        LOGCOLORSPACE    = LOGCOLORSPACEW;
        PLOGCOLORSPACE   = PLOGCOLORSPACEW;
     {$else}
        LPLOGCOLORSPACE  = PLOGCOLORSPACEA;
        tagLOGCOLORSPACE = LOGCOLORSPACEA;
        TLOGCOLORSPACE   = LOGCOLORSPACEA;
        LOGCOLORSPACE    = LOGCOLORSPACEA;
        PLOGCOLORSPACE   = PLOGCOLORSPACEA;
     {$endif}

     EMRCREATECOLORSPACE = record
          emr : EMR;
          ihCS : DWORD;
          lcs : LOGCOLORSPACE;
       end;
     tagEMRCREATECOLORSPACE = EMRCREATECOLORSPACE;
     TEMRCREATECOLORSPACE = EMRCREATECOLORSPACE;
     PEMRCREATECOLORSPACE = ^EMRCREATECOLORSPACE;

     EMRCREATEDIBPATTERNBRUSHPT = record
          emr : EMR;
          ihBrush : DWORD;
          iUsage : DWORD;
          offBmi : DWORD;
          cbBmi : DWORD;
          offBits : DWORD;
          cbBits : DWORD;
       end;
     tagEMRCREATEDIBPATTERNBRUSHPT = EMRCREATEDIBPATTERNBRUSHPT;
     TEMRCREATEDIBPATTERNBRUSHPT = EMRCREATEDIBPATTERNBRUSHPT;
     PEMRCREATEDIBPATTERNBRUSHPT = ^EMRCREATEDIBPATTERNBRUSHPT;

     EMRCREATEMONOBRUSH = record
          emr : EMR;
          ihBrush : DWORD;
          iUsage : DWORD;
          offBmi : DWORD;
          cbBmi : DWORD;
          offBits : DWORD;
          cbBits : DWORD;
       end;
     tagEMRCREATEMONOBRUSH = EMRCREATEMONOBRUSH;
     TEMRCREATEMONOBRUSH = EMRCREATEMONOBRUSH;
     PEMRCREATEMONOBRUSH = ^EMRCREATEMONOBRUSH;

     PALETTEENTRY = record
          peRed : BYTE;
          peGreen : BYTE;
          peBlue : BYTE;
          peFlags : BYTE;
       end;
     LPPALETTEENTRY = ^PALETTEENTRY;
     tagPALETTEENTRY = PALETTEENTRY;
     TPALETTEENTRY = PALETTEENTRY;
     PPALETTEENTRY = ^PALETTEENTRY;

     LOGPALETTE = record
          palVersion : WORD;
          palNumEntries : WORD;
          palPalEntry : array[0..0] of PALETTEENTRY;
       end;
     LPLOGPALETTE = ^LOGPALETTE;
     tagLOGPALETTE = LOGPALETTE;
     TLOGPALETTE = LOGPALETTE;
     PLOGPALETTE = ^LOGPALETTE;

     EMRCREATEPALETTE = record
          emr : EMR;
          ihPal : DWORD;
          lgpl : LOGPALETTE;
       end;
     tagEMRCREATEPALETTE = EMRCREATEPALETTE;
     TEMRCREATEPALETTE = EMRCREATEPALETTE;
     PEMRCREATEPALETTE = ^EMRCREATEPALETTE;

     LOGPEN = record
          lopnStyle : UINT;
          lopnWidth : POINT;
          lopnColor : COLORREF;
       end;
     tagLOGPEN = LOGPEN;
     TLOGPEN = LOGPEN;
     PLOGPEN = ^LOGPEN;

     EMRCREATEPEN = record
          emr : EMR;
          ihPen : DWORD;
          lopn : LOGPEN;
       end;
     tagEMRCREATEPEN = EMRCREATEPEN;
     TEMRCREATEPEN = EMRCREATEPEN;
     PEMRCREATEPEN = ^EMRCREATEPEN;

     EMRELLIPSE = record
          emr : EMR;
          rclBox : RECTL;
       end;
     tagEMRELLIPSE = EMRELLIPSE;
     TEMRELLIPSE = EMRELLIPSE;
     PEMRELLIPSE = ^EMRELLIPSE;

     EMRRECTANGLE = EMRELLIPSE;
     TEMRRECTANGLE = EMRELLIPSE;
     PEMRRECTANGLE = ^EMRELLIPSE;

     EMREOF = record
          emr : EMR;
          nPalEntries : DWORD;
          offPalEntries : DWORD;
          nSizeLast : DWORD;
       end;
     tagEMREOF = EMREOF;
     TEMREOF = EMREOF;
     PEMREOF = ^EMREOF;

     EMREXCLUDECLIPRECT = record
          emr : EMR;
          rclClip : RECTL;
       end;
     tagEMREXCLUDECLIPRECT = EMREXCLUDECLIPRECT;
     TEMREXCLUDECLIPRECT = EMREXCLUDECLIPRECT;
     PEMREXCLUDECLIPRECT = ^EMREXCLUDECLIPRECT;

     EMRINTERSECTCLIPRECT = EMREXCLUDECLIPRECT;
     TEMRINTERSECTCLIPRECT = EMREXCLUDECLIPRECT;
     PEMRINTERSECTCLIPRECT = ^EMREXCLUDECLIPRECT;

     PANOSE = record
          bFamilyType : BYTE;
          bSerifStyle : BYTE;
          bWeight : BYTE;
          bProportion : BYTE;
          bContrast : BYTE;
          bStrokeVariation : BYTE;
          bArmStyle : BYTE;
          bLetterform : BYTE;
          bMidline : BYTE;
          bXHeight : BYTE;
       end;
     tagPANOSE = PANOSE;
     TPANOSE = PANOSE;
     PPANOSE = ^PANOSE;

     EXTLOGFONTA = record
          elfLogFont : LOGFONT;
          elfFullName : array[0..(LF_FULLFACESIZE)-1] of AnsiCHAR;
          elfStyle : array[0..(LF_FACESIZE)-1] of AnsiCHAR;
          elfVersion : DWORD;
          elfStyleSize : DWORD;
          elfMatch : DWORD;
          elfReserved : DWORD;
          elfVendorId : array[0..(ELF_VENDOR_SIZE)-1] of BYTE;
          elfCulture : DWORD;
          elfPanose : PANOSE;
       end;
     LPEXTLOGFONTA  = ^EXTLOGFONTA;
     tagEXTLOGFONTA = EXTLOGFONTA;
     TEXTLOGFONTA   = EXTLOGFONTA;
     PEXTLOGFONTA   = LPEXTLOGFONTA;

 
     EXTLOGFONTW    = record
       elfLogFont   :LOGFONTW;
       elfFullName  :array[0..(LF_FULLFACESIZE) - 1] of WCHAR;
       elfStyle     :array[0..(LF_FACESIZE) - 1] of WCHAR;
       elfVersion   :DWORD;
       elfStyleSize :DWORD;
       elfMatch     :DWORD;
       elfReserved  :DWORD;
       elfVendorId  :array[0..(ELF_VENDOR_SIZE) - 1] of Byte;
       elfCulture   :DWORD;
       elfPanose    :PANOSE;
      end;
     LPEXTLOGFONTW  = ^EXTLOGFONTW;
     tagEXTLOGFONTW = EXTLOGFONTW;
     TEXTLOGFONTW   = EXTLOGFONTW;
     PEXTLOGFONTW   = ^EXTLOGFONTW;
  
    {$IFNDEF UNICODE}
      EXTLOGFONT     = EXTLOGFONTA;
      LPEXTLOGFONT   = ^EXTLOGFONTA;
      tagEXTLOGFONT  = EXTLOGFONTA;
      TEXTLOGFONT    = EXTLOGFONTA;
      PEXTLOGFONT    = ^EXTLOGFONTA;
    {$ELSE}
      EXTLOGFONT     = EXTLOGFONTW;
      LPEXTLOGFONT   = ^EXTLOGFONTW;
      tagEXTLOGFONT  = EXTLOGFONTW;
      TEXTLOGFONT    = EXTLOGFONTW;
      PEXTLOGFONT    = ^EXTLOGFONTW;
    {$ENDIF}

     EMREXTCREATEFONTINDIRECTW = record
          emr : EMR;
          ihFont : DWORD;
          elfw : EXTLOGFONTW;
       end;
     tagEMREXTCREATEFONTINDIRECTW = EMREXTCREATEFONTINDIRECTW;
     TEMREXTCREATEFONTINDIRECTW = EMREXTCREATEFONTINDIRECTW;
     PEMREXTCREATEFONTINDIRECTW = ^EMREXTCREATEFONTINDIRECTW;

     EXTLOGPEN = record
          elpPenStyle : DWORD;
          elpWidth : DWORD;
          elpBrushStyle : UINT;
          elpColor : COLORREF;
          elpHatch : ULONG_PTR;
          elpNumEntries : DWORD;
          elpStyleEntry : array[0..0] of DWORD;
       end;
     tagEXTLOGPEN = EXTLOGPEN;
     TEXTLOGPEN = EXTLOGPEN;
     PEXTLOGPEN = ^EXTLOGPEN;

     EMREXTCREATEPEN = record
          emr : EMR;
          ihPen : DWORD;
          offBmi : DWORD;
          cbBmi : DWORD;
          offBits : DWORD;
          cbBits : DWORD;
          elp : EXTLOGPEN;
       end;
     tagEMREXTCREATEPEN = EMREXTCREATEPEN;
     TEMREXTCREATEPEN = EMREXTCREATEPEN;
     PEMREXTCREATEPEN = ^EMREXTCREATEPEN;

     EMREXTFLOODFILL = record
          emr : EMR;
          ptlStart : POINTL;
          crColor : COLORREF;
          iMode : DWORD;
       end;
     tagEMREXTFLOODFILL = EMREXTFLOODFILL;
     TEMREXTFLOODFILL = EMREXTFLOODFILL;
     PEMREXTFLOODFILL = ^EMREXTFLOODFILL;

     EMREXTSELECTCLIPRGN = record
          emr : EMR;
          cbRgnData : DWORD;
          iMode : DWORD;
          RgnData : array[0..0] of BYTE;
       end;
     tagEMREXTSELECTCLIPRGN = EMREXTSELECTCLIPRGN;
     TEMREXTSELECTCLIPRGN = EMREXTSELECTCLIPRGN;
     PEMREXTSELECTCLIPRGN = ^EMREXTSELECTCLIPRGN;

     EMRTEXT = record
          ptlReference : POINTL;
          nChars : DWORD;
          offString : DWORD;
          fOptions : DWORD;
          rcl : RECTL;
          offDx : DWORD;
       end;
     tagEMRTEXT = EMRTEXT;
     TEMRTEXT = EMRTEXT;
     PEMRTEXT = ^EMRTEXT;

     EMREXTTEXTOUTA = record
          emr : EMR;
          rclBounds : RECTL;
          iGraphicsMode : DWORD;
          exScale : Single;
          eyScale : Single;
          emrtext : EMRTEXT;
       end;
     tagEMREXTTEXTOUTA = EMREXTTEXTOUTA;
     TEMREXTTEXTOUTA = EMREXTTEXTOUTA;
     PEMREXTTEXTOUTA = ^EMREXTTEXTOUTA;

     EMREXTTEXTOUTW = EMREXTTEXTOUTA;
     TEMREXTTEXTOUTW = EMREXTTEXTOUTA;
     PEMREXTTEXTOUTW = ^EMREXTTEXTOUTA;

     EMRFILLPATH = record
          emr : EMR;
          rclBounds : RECTL;
       end;
     tagEMRFILLPATH = EMRFILLPATH;
     TEMRFILLPATH = EMRFILLPATH;
     PEMRFILLPATH = ^EMRFILLPATH;

     EMRSTROKEANDFILLPATH = EMRFILLPATH;
     TEMRSTROKEANDFILLPATH = EMRFILLPATH;
     PEMRSTROKEANDFILLPATH = ^EMRFILLPATH;

     EMRSTROKEPATH = EMRFILLPATH;
     TEMRSTROKEPATH = EMRFILLPATH;
     PEMRSTROKEPATH = ^EMRFILLPATH;

     EMRFILLRGN = record
          emr : EMR;
          rclBounds : RECTL;
          cbRgnData : DWORD;
          ihBrush : DWORD;
          RgnData : array[0..0] of BYTE;
       end;
     tagEMRFILLRGN = EMRFILLRGN;
     TEMRFILLRGN = EMRFILLRGN;
     PEMRFILLRGN = ^EMRFILLRGN;

     EMRFORMAT = record
          dSignature : DWORD;
          nVersion : DWORD;
          cbData : DWORD;
          offData : DWORD;
       end;
     tagEMRFORMAT = EMRFORMAT;
     TEMRFORMAT = EMRFORMAT;
     PEMRFORMAT = ^EMRFORMAT;

     SIZE = TSize;
     LPSIZE = PSIZE;
     tagSIZE = SIZE;

     SIZEL = SIZE;
     TSIZEL = SIZE;
     PSIZEL = ^SIZE;
     LPSIZEL = ^SIZE;

     EMRFRAMERGN = record
          emr : EMR;
          rclBounds : RECTL;
          cbRgnData : DWORD;
          ihBrush : DWORD;
          szlStroke : SIZEL;
          RgnData : array[0..0] of BYTE;
       end;
     tagEMRFRAMERGN = EMRFRAMERGN;
     TEMRFRAMERGN = EMRFRAMERGN;
     PEMRFRAMERGN = ^EMRFRAMERGN;

     EMRGDICOMMENT = record
          emr : EMR;
          cbData : DWORD;
          Data : array[0..0] of BYTE;
       end;
     tagEMRGDICOMMENT = EMRGDICOMMENT;
     TEMRGDICOMMENT = EMRGDICOMMENT;
     PEMRGDICOMMENT = ^EMRGDICOMMENT;

     EMRINVERTRGN = record
          emr : EMR;
          rclBounds : RECTL;
          cbRgnData : DWORD;
          RgnData : array[0..0] of BYTE;
       end;
     tagEMRINVERTRGN = EMRINVERTRGN;
     TEMRINVERTRGN = EMRINVERTRGN;
     PEMRINVERTRGN = ^EMRINVERTRGN;

     EMRPAINTRGN = EMRINVERTRGN;
     TEMRPAINTRGN = EMRINVERTRGN;
     PEMRPAINTRGN = ^EMRINVERTRGN;

     EMRLINETO = record
          emr : EMR;
          ptl : POINTL;
       end;
     tagEMRLINETO = EMRLINETO;
     TEMRLINETO = EMRLINETO;
     PEMRLINETO = ^EMRLINETO;

     EMRMOVETOEX = EMRLINETO;
     TEMRMOVETOEX = EMRLINETO;
     PEMRMOVETOEX = ^EMRLINETO;

     EMRMASKBLT = record
          emr : EMR;
          rclBounds : RECTL;
          xDest : LONG;
          yDest : LONG;
          cxDest : LONG;
          cyDest : LONG;
          dwRop : DWORD;
          xSrc : LONG;
          ySrc : LONG;
          xformSrc : XFORM;
          crBkColorSrc : COLORREF;
          iUsageSrc : DWORD;
          offBmiSrc : DWORD;
          cbBmiSrc : DWORD;
          offBitsSrc : DWORD;
          cbBitsSrc : DWORD;
          xMask : LONG;
          yMask : LONG;
          iUsageMask : DWORD;
          offBmiMask : DWORD;
          cbBmiMask : DWORD;
          offBitsMask : DWORD;
          cbBitsMask : DWORD;
       end;
     tagEMRMASKBLT = EMRMASKBLT;
     TEMRMASKBLT = EMRMASKBLT;
     PEMRMASKBLT = ^EMRMASKBLT;

     EMRMODIFYWORLDTRANSFORM = record
          emr : EMR;
          xform : XFORM;
          iMode : DWORD;
       end;
     tagEMRMODIFYWORLDTRANSFORM = EMRMODIFYWORLDTRANSFORM;
     TEMRMODIFYWORLDTRANSFORM = EMRMODIFYWORLDTRANSFORM;
     PEMRMODIFYWORLDTRANSFORM = ^EMRMODIFYWORLDTRANSFORM;

     EMROFFSETCLIPRGN = record
          emr : EMR;
          ptlOffset : POINTL;
       end;
     tagEMROFFSETCLIPRGN = EMROFFSETCLIPRGN;
     TEMROFFSETCLIPRGN = EMROFFSETCLIPRGN;
     PEMROFFSETCLIPRGN = ^EMROFFSETCLIPRGN;

     EMRPLGBLT = record
          emr : EMR;
          rclBounds : RECTL;
          aptlDest : array[0..2] of POINTL;
          xSrc : LONG;
          ySrc : LONG;
          cxSrc : LONG;
          cySrc : LONG;
          xformSrc : XFORM;
          crBkColorSrc : COLORREF;
          iUsageSrc : DWORD;
          offBmiSrc : DWORD;
          cbBmiSrc : DWORD;
          offBitsSrc : DWORD;
          cbBitsSrc : DWORD;
          xMask : LONG;
          yMask : LONG;
          iUsageMask : DWORD;
          offBmiMask : DWORD;
          cbBmiMask : DWORD;
          offBitsMask : DWORD;
          cbBitsMask : DWORD;
       end;
     tagEMRPLGBLT = EMRPLGBLT;
     TEMRPLGBLT = EMRPLGBLT;
     PEMRPLGBLT = ^EMRPLGBLT;

     EMRPOLYDRAW = record
          emr : EMR;
          rclBounds : RECTL;
          cptl : DWORD;
          aptl : array[0..0] of POINTL;
          abTypes : array[0..0] of BYTE;
       end;
     tagEMRPOLYDRAW = EMRPOLYDRAW;
     TEMRPOLYDRAW = EMRPOLYDRAW;
     PEMRPOLYDRAW = ^EMRPOLYDRAW;

     EMRPOLYDRAW16 = record
          emr : EMR;
          rclBounds : RECTL;
          cpts : DWORD;
          apts : array[0..0] of POINTS;
          abTypes : array[0..0] of BYTE;
       end;
     tagEMRPOLYDRAW16 = EMRPOLYDRAW16;
     TEMRPOLYDRAW16 = EMRPOLYDRAW16;
     PEMRPOLYDRAW16 = ^EMRPOLYDRAW16;

     EMRPOLYLINE = record
          emr : EMR;
          rclBounds : RECTL;
          cptl : DWORD;
          aptl : array[0..0] of POINTL;
       end;
     tagEMRPOLYLINE = EMRPOLYLINE;
     TEMRPOLYLINE = EMRPOLYLINE;
     PEMRPOLYLINE = ^EMRPOLYLINE;

     EMRPOLYBEZIER = EMRPOLYLINE;
     TEMRPOLYBEZIER = EMRPOLYLINE;
     PEMRPOLYBEZIER = ^EMRPOLYLINE;

     EMRPOLYGON = EMRPOLYLINE;
     TEMRPOLYGON = EMRPOLYLINE;
     PEMRPOLYGON = ^EMRPOLYLINE;

     EMRPOLYBEZIERTO = EMRPOLYLINE;
     TEMRPOLYBEZIERTO = EMRPOLYLINE;
     PEMRPOLYBEZIERTO = ^EMRPOLYLINE;

     EMRPOLYLINETO = EMRPOLYLINE;
     TEMRPOLYLINETO = EMRPOLYLINE;
     PEMRPOLYLINETO = ^EMRPOLYLINE;

     EMRPOLYLINE16 = record
          emr : EMR;
          rclBounds : RECTL;
          cpts : DWORD;
          apts : array[0..0] of POINTS;
       end;
     tagEMRPOLYLINE16 = EMRPOLYLINE16;
     TEMRPOLYLINE16 = EMRPOLYLINE16;
     PEMRPOLYLINE16 = ^EMRPOLYLINE16;

     EMRPOLYBEZIER16 = EMRPOLYLINE16;
     TEMRPOLYBEZIER16 = EMRPOLYLINE16;
     PEMRPOLYBEZIER16 = ^EMRPOLYLINE16;

     EMRPOLYGON16 = EMRPOLYLINE16;
     TEMRPOLYGON16 = EMRPOLYLINE16;
     PEMRPOLYGON16 = ^EMRPOLYLINE16;

     EMRPOLYBEZIERTO16 = EMRPOLYLINE16;
     TEMRPOLYBEZIERTO16 = EMRPOLYLINE16;
     PEMRPOLYBEZIERTO16 = ^EMRPOLYLINE16;

     EMRPOLYLINETO16 = EMRPOLYLINE16;
     TEMRPOLYLINETO16 = EMRPOLYLINE16;
     PEMRPOLYLINETO16 = ^EMRPOLYLINE16;

     EMRPOLYPOLYLINE = record
          emr : EMR;
          rclBounds : RECTL;
          nPolys : DWORD;
          cptl : DWORD;
          aPolyCounts : array[0..0] of DWORD;
          aptl : array[0..0] of POINTL;
       end;
     tagEMRPOLYPOLYLINE = EMRPOLYPOLYLINE;
     TEMRPOLYPOLYLINE = EMRPOLYPOLYLINE;
     PEMRPOLYPOLYLINE = ^EMRPOLYPOLYLINE;

     EMRPOLYPOLYGON = EMRPOLYPOLYLINE;
     TEMRPOLYPOLYGON = EMRPOLYPOLYLINE;
     PEMRPOLYPOLYGON = ^EMRPOLYPOLYLINE;

     EMRPOLYPOLYLINE16 = record
          emr : EMR;
          rclBounds : RECTL;
          nPolys : DWORD;
          cpts : DWORD;
          aPolyCounts : array[0..0] of DWORD;
          apts : array[0..0] of POINTS;
       end;
     tagEMRPOLYPOLYLINE16 = EMRPOLYPOLYLINE16;
     TEMRPOLYPOLYLINE16 = EMRPOLYPOLYLINE16;
     PEMRPOLYPOLYLINE16 = ^EMRPOLYPOLYLINE16;

     EMRPOLYPOLYGON16 = EMRPOLYPOLYLINE16;
     TEMRPOLYPOLYGON16 = EMRPOLYPOLYLINE16;
     PEMRPOLYPOLYGON16 = ^EMRPOLYPOLYLINE16;

     EMRPOLYTEXTOUTA = record
          emr : EMR;
          rclBounds : RECTL;
          iGraphicsMode : DWORD;
          exScale : Single;
          eyScale : Single;
          cStrings : LONG;
          aemrtext : array[0..0] of EMRTEXT;
       end;
     tagEMRPOLYTEXTOUTA = EMRPOLYTEXTOUTA;
     TEMRPOLYTEXTOUTA = EMRPOLYTEXTOUTA;
     PEMRPOLYTEXTOUTA = ^EMRPOLYTEXTOUTA;

     EMRPOLYTEXTOUTW = EMRPOLYTEXTOUTA;
     TEMRPOLYTEXTOUTW = EMRPOLYTEXTOUTA;
     PEMRPOLYTEXTOUTW = ^EMRPOLYTEXTOUTA;

     EMRRESIZEPALETTE = record
          emr : EMR;
          ihPal : DWORD;
          cEntries : DWORD;
       end;
     tagEMRRESIZEPALETTE = EMRRESIZEPALETTE;
     TEMRRESIZEPALETTE = EMRRESIZEPALETTE;
     PEMRRESIZEPALETTE = ^EMRRESIZEPALETTE;

     EMRRESTOREDC = record
          emr : EMR;
          iRelative : LONG;
       end;
     tagEMRRESTOREDC = EMRRESTOREDC;
     TEMRRESTOREDC = EMRRESTOREDC;
     PEMRRESTOREDC = ^EMRRESTOREDC;

     EMRROUNDRECT = record
          emr : EMR;
          rclBox : RECTL;
          szlCorner : SIZEL;
       end;
     tagEMRROUNDRECT = EMRROUNDRECT;
     TEMRROUNDRECT = EMRROUNDRECT;
     PEMRROUNDRECT = ^EMRROUNDRECT;

     EMRSCALEVIEWPORTEXTEX = record
          emr : EMR;
          xNum : LONG;
          xDenom : LONG;
          yNum : LONG;
          yDenom : LONG;
       end;
     tagEMRSCALEVIEWPORTEXTEX = EMRSCALEVIEWPORTEXTEX;
     TEMRSCALEVIEWPORTEXTEX = EMRSCALEVIEWPORTEXTEX;
     PEMRSCALEVIEWPORTEXTEX = ^EMRSCALEVIEWPORTEXTEX;

     EMRSCALEWINDOWEXTEX = EMRSCALEVIEWPORTEXTEX;
     TEMRSCALEWINDOWEXTEX = EMRSCALEVIEWPORTEXTEX;
     PEMRSCALEWINDOWEXTEX = ^EMRSCALEVIEWPORTEXTEX;

     EMRSELECTCOLORSPACE = record
          emr : EMR;
          ihCS : DWORD;
       end;
     tagEMRSELECTCOLORSPACE = EMRSELECTCOLORSPACE;
     TEMRSELECTCOLORSPACE = EMRSELECTCOLORSPACE;
     PEMRSELECTCOLORSPACE = ^EMRSELECTCOLORSPACE;

     EMRDELETECOLORSPACE = EMRSELECTCOLORSPACE;
     TEMRDELETECOLORSPACE = EMRSELECTCOLORSPACE;
     PEMRDELETECOLORSPACE = ^EMRSELECTCOLORSPACE;

     EMRSELECTOBJECT = record
          emr : EMR;
          ihObject : DWORD;
       end;
     tagEMRSELECTOBJECT = EMRSELECTOBJECT;
     TEMRSELECTOBJECT = EMRSELECTOBJECT;
     PEMRSELECTOBJECT = ^EMRSELECTOBJECT;

     EMRDELETEOBJECT = EMRSELECTOBJECT;
     TEMRDELETEOBJECT = EMRSELECTOBJECT;
     PEMRDELETEOBJECT = ^EMRSELECTOBJECT;

     EMRSELECTPALETTE = record
          emr : EMR;
          ihPal : DWORD;
       end;
     tagEMRSELECTPALETTE = EMRSELECTPALETTE;
     TEMRSELECTPALETTE = EMRSELECTPALETTE;
     PEMRSELECTPALETTE = ^EMRSELECTPALETTE;

     EMRSETARCDIRECTION = record
          emr : EMR;
          iArcDirection : DWORD;
       end;
     tagEMRSETARCDIRECTION = EMRSETARCDIRECTION;
     TEMRSETARCDIRECTION = EMRSETARCDIRECTION;
     PEMRSETARCDIRECTION = ^EMRSETARCDIRECTION;

     EMRSETBKCOLOR = record
          emr : EMR;
          crColor : COLORREF;
       end;
     tagEMRSETTEXTCOLOR = EMRSETBKCOLOR;
     TEMRSETBKCOLOR = EMRSETBKCOLOR;
     PEMRSETBKCOLOR = ^EMRSETBKCOLOR;

     EMRSETTEXTCOLOR = EMRSETBKCOLOR;
     TEMRSETTEXTCOLOR = EMRSETBKCOLOR;
     PEMRSETTEXTCOLOR = ^EMRSETBKCOLOR;

     EMRSETCOLORADJUSTMENT = record
          emr : EMR;
          ColorAdjustment : COLORADJUSTMENT;
       end;
     tagEMRSETCOLORADJUSTMENT = EMRSETCOLORADJUSTMENT;
     TEMRSETCOLORADJUSTMENT = EMRSETCOLORADJUSTMENT;
     PEMRSETCOLORADJUSTMENT = ^EMRSETCOLORADJUSTMENT;

     EMRSETDIBITSTODEVICE = record
          emr : EMR;
          rclBounds : RECTL;
          xDest : LONG;
          yDest : LONG;
          xSrc : LONG;
          ySrc : LONG;
          cxSrc : LONG;
          cySrc : LONG;
          offBmiSrc : DWORD;
          cbBmiSrc : DWORD;
          offBitsSrc : DWORD;
          cbBitsSrc : DWORD;
          iUsageSrc : DWORD;
          iStartScan : DWORD;
          cScans : DWORD;
       end;
     tagEMRSETDIBITSTODEVICE = EMRSETDIBITSTODEVICE;
     TEMRSETDIBITSTODEVICE = EMRSETDIBITSTODEVICE;
     PEMRSETDIBITSTODEVICE = ^EMRSETDIBITSTODEVICE;

     EMRSETMAPPERFLAGS = record
          emr : EMR;
          dwFlags : DWORD;
       end;
     tagEMRSETMAPPERFLAGS = EMRSETMAPPERFLAGS;
     TEMRSETMAPPERFLAGS = EMRSETMAPPERFLAGS;
     PEMRSETMAPPERFLAGS = ^EMRSETMAPPERFLAGS;

     EMRSETMITERLIMIT = record
          emr : EMR;
          eMiterLimit : Single;
       end;
     tagEMRSETMITERLIMIT = EMRSETMITERLIMIT;
     TEMRSETMITERLIMIT = EMRSETMITERLIMIT;
     PEMRSETMITERLIMIT = ^EMRSETMITERLIMIT;

     EMRSETPALETTEENTRIES = record
          emr : EMR;
          ihPal : DWORD;
          iStart : DWORD;
          cEntries : DWORD;
          aPalEntries : array[0..0] of PALETTEENTRY;
       end;
     tagEMRSETPALETTEENTRIES = EMRSETPALETTEENTRIES;
     TEMRSETPALETTEENTRIES = EMRSETPALETTEENTRIES;
     PEMRSETPALETTEENTRIES = ^EMRSETPALETTEENTRIES;

     EMRSETPIXELV = record
          emr : EMR;
          ptlPixel : POINTL;
          crColor : COLORREF;
       end;
     tagEMRSETPIXELV = EMRSETPIXELV;
     TEMRSETPIXELV = EMRSETPIXELV;
     PEMRSETPIXELV = ^EMRSETPIXELV;

     EMRSETVIEWPORTEXTEX = record
          emr : EMR;
          szlExtent : SIZEL;
       end;
     tagEMRSETVIEWPORTEXTEX = EMRSETVIEWPORTEXTEX;
     TEMRSETVIEWPORTEXTEX = EMRSETVIEWPORTEXTEX;
     PEMRSETVIEWPORTEXTEX = ^EMRSETVIEWPORTEXTEX;

     EMRSETWINDOWEXTEX = EMRSETVIEWPORTEXTEX;
     TEMRSETWINDOWEXTEX = EMRSETVIEWPORTEXTEX;
     PEMRSETWINDOWEXTEX = ^EMRSETVIEWPORTEXTEX;

     EMRSETVIEWPORTORGEX = record
          emr : EMR;
          ptlOrigin : POINTL;
       end;
     tagEMRSETVIEWPORTORGEX = EMRSETVIEWPORTORGEX;
     TEMRSETVIEWPORTORGEX = EMRSETVIEWPORTORGEX;
     PEMRSETVIEWPORTORGEX = ^EMRSETVIEWPORTORGEX;

     EMRSETWINDOWORGEX = EMRSETVIEWPORTORGEX;
     TEMRSETWINDOWORGEX = EMRSETVIEWPORTORGEX;
     PEMRSETWINDOWORGEX = ^EMRSETVIEWPORTORGEX;

     EMRSETBRUSHORGEX = EMRSETVIEWPORTORGEX;
     TEMRSETBRUSHORGEX = EMRSETVIEWPORTORGEX;
     PEMRSETBRUSHORGEX = ^EMRSETVIEWPORTORGEX;

     EMRSETWORLDTRANSFORM = record
          emr : EMR;
          xform : XFORM;
       end;
     tagEMRSETWORLDTRANSFORM = EMRSETWORLDTRANSFORM;
     TEMRSETWORLDTRANSFORM = EMRSETWORLDTRANSFORM;
     PEMRSETWORLDTRANSFORM = ^EMRSETWORLDTRANSFORM;

     EMRSTRETCHBLT = record
          emr : EMR;
          rclBounds : RECTL;
          xDest : LONG;
          yDest : LONG;
          cxDest : LONG;
          cyDest : LONG;
          dwRop : DWORD;
          xSrc : LONG;
          ySrc : LONG;
          xformSrc : XFORM;
          crBkColorSrc : COLORREF;
          iUsageSrc : DWORD;
          offBmiSrc : DWORD;
          cbBmiSrc : DWORD;
          offBitsSrc : DWORD;
          cbBitsSrc : DWORD;
          cxSrc : LONG;
          cySrc : LONG;
       end;
     tagEMRSTRETCHBLT = EMRSTRETCHBLT;
     TEMRSTRETCHBLT = EMRSTRETCHBLT;
     PEMRSTRETCHBLT = ^EMRSTRETCHBLT;

     EMRSTRETCHDIBITS = record
          emr : EMR;
          rclBounds : RECTL;
          xDest : LONG;
          yDest : LONG;
          xSrc : LONG;
          ySrc : LONG;
          cxSrc : LONG;
          cySrc : LONG;
          offBmiSrc : DWORD;
          cbBmiSrc : DWORD;
          offBitsSrc : DWORD;
          cbBitsSrc : DWORD;
          iUsageSrc : DWORD;
          dwRop : DWORD;
          cxDest : LONG;
          cyDest : LONG;
       end;
     tagEMRSTRETCHDIBITS = EMRSTRETCHDIBITS;
     TEMRSTRETCHDIBITS = EMRSTRETCHDIBITS;
     PEMRSTRETCHDIBITS = ^EMRSTRETCHDIBITS;

     EMRABORTPATH = record
          emr : EMR;
       end;
     TEMRABORTPATH = EMRABORTPATH;
     PEMRABORTPATH = ^EMRABORTPATH;

     tagABORTPATH = EMRABORTPATH;
     TABORTPATH = EMRABORTPATH;

     EMRBEGINPATH = EMRABORTPATH;
     TEMRBEGINPATH = EMRABORTPATH;
     PEMRBEGINPATH = ^EMRABORTPATH;

     EMRENDPATH = EMRABORTPATH;
     TEMRENDPATH = EMRABORTPATH;
     PEMRENDPATH = ^EMRABORTPATH;

     EMRCLOSEFIGURE = EMRABORTPATH;
     TEMRCLOSEFIGURE = EMRABORTPATH;
     PEMRCLOSEFIGURE = ^EMRABORTPATH;

     EMRFLATTENPATH = EMRABORTPATH;
     TEMRFLATTENPATH = EMRABORTPATH;
     PEMRFLATTENPATH = ^EMRABORTPATH;

     EMRWIDENPATH = EMRABORTPATH;
     TEMRWIDENPATH = EMRABORTPATH;
     PEMRWIDENPATH = ^EMRABORTPATH;

     EMRSETMETARGN = EMRABORTPATH;
     TEMRSETMETARGN = EMRABORTPATH;
     PEMRSETMETARGN = ^EMRABORTPATH;

     EMRSAVEDC = EMRABORTPATH;
     TEMRSAVEDC = EMRABORTPATH;
     PEMRSAVEDC = ^EMRABORTPATH;

     EMRREALIZEPALETTE = EMRABORTPATH;
     TEMRREALIZEPALETTE = EMRABORTPATH;
     PEMRREALIZEPALETTE = ^EMRABORTPATH;

     EMRSELECTCLIPPATH = record
          emr : EMR;
          iMode : DWORD;
       end;
     tagEMRSELECTCLIPPATH = EMRSELECTCLIPPATH;
     TEMRSELECTCLIPPATH = EMRSELECTCLIPPATH;
     PEMRSELECTCLIPPATH = ^EMRSELECTCLIPPATH;

     EMRSETBKMODE = EMRSELECTCLIPPATH;
     TEMRSETBKMODE = EMRSELECTCLIPPATH;
     PEMRSETBKMODE = ^EMRSELECTCLIPPATH;

     EMRSETMAPMODE = EMRSELECTCLIPPATH;
     TEMRSETMAPMODE = EMRSELECTCLIPPATH;
     PEMRSETMAPMODE = ^EMRSELECTCLIPPATH;

     EMRSETPOLYFILLMODE = EMRSELECTCLIPPATH;
     TEMRSETPOLYFILLMODE = EMRSELECTCLIPPATH;
     PEMRSETPOLYFILLMODE = ^EMRSELECTCLIPPATH;

     EMRSETROP2 = EMRSELECTCLIPPATH;
     TEMRSETROP2 = EMRSELECTCLIPPATH;
     PEMRSETROP2 = ^EMRSELECTCLIPPATH;

     EMRSETSTRETCHBLTMODE = EMRSELECTCLIPPATH;
     TEMRSETSTRETCHBLTMODE = EMRSELECTCLIPPATH;
     PEMRSETSTRETCHBLTMODE = ^EMRSELECTCLIPPATH;

     EMRSETTEXTALIGN = EMRSELECTCLIPPATH;
     TEMRSETTEXTALIGN = EMRSELECTCLIPPATH;
     PEMRSETTEXTALIGN = ^EMRSELECTCLIPPATH;

     EMRENABLEICM = EMRSELECTCLIPPATH;
     TEMRENABLEICM = EMRSELECTCLIPPATH;
     PEMRENABLEICM = ^EMRSELECTCLIPPATH;

     NMHDR = record
          hwndFrom : HWND;
          idFrom : UINT_PTR;
          code : UINT;
       end;
     tagNMHDR = NMHDR;
     TNMHDR = NMHDR;
     PNMHDR = ^NMHDR;

     ENCORRECTTEXT = record
          nmhdr : NMHDR;
          chrg : CHARRANGE;
          seltyp : WORD;
       end;
     _encorrecttext = ENCORRECTTEXT;
     Tencorrecttext = ENCORRECTTEXT;
     Pencorrecttext = ^ENCORRECTTEXT;

     ENDROPFILES = record
          nmhdr : NMHDR;
          hDrop : HANDLE;
          cp : LONG;
          fProtected : WINBOOL;
       end;
     _endropfiles = ENDROPFILES;
     Tendropfiles = ENDROPFILES;
     Pendropfiles = ^ENDROPFILES;

     ENSAVECLIPBOARD = record
          nmhdr : NMHDR;
          cObjectCount : LONG;
          cch : LONG;
       end;
     TENSAVECLIPBOARD = ENSAVECLIPBOARD;
     PENSAVECLIPBOARD = ^ENSAVECLIPBOARD;

     ENOLEOPFAILED = record
          nmhdr : NMHDR;
          iob : LONG;
          lOper : LONG;
          hr : HRESULT;
       end;
     TENOLEOPFAILED = ENOLEOPFAILED;
     PENOLEOPFAILED = ^ENOLEOPFAILED;

     ENHMETAHEADER = record
          iType : DWORD;
          nSize : DWORD;
          rclBounds : RECTL;
          rclFrame : RECTL;
          dSignature : DWORD;
          nVersion : DWORD;
          nBytes : DWORD;
          nRecords : DWORD;
          nHandles : WORD;
          sReserved : WORD;
          nDescription : DWORD;
          offDescription : DWORD;
          nPalEntries : DWORD;
          szlDevice : SIZEL;
          szlMillimeters : SIZEL;
// #if (WINVER >= 0x0400)
          cbPixelFormat : DWORD;
          offPixelFormat : DWORD;
          bOpenGL : DWORD;
// #endif
// #if (WINVER >= 0x0500)
          szlMicrometers : SIZEL;
// #endif
       end;
     LPENHMETAHEADER = ^ENHMETAHEADER;
     tagENHMETAHEADER = ENHMETAHEADER;
     TENHMETAHEADER = ENHMETAHEADER;
     PENHMETAHEADER = ^ENHMETAHEADER;

     ENHMETARECORD = record
          iType : DWORD;
          nSize : DWORD;
          dParm : array[0..0] of DWORD;
       end;
     LPENHMETARECORD = ^ENHMETARECORD;
     tagENHMETARECORD = ENHMETARECORD;
     TENHMETARECORD = ENHMETARECORD;
     PENHMETARECORD = ^ENHMETARECORD;

     ENPROTECTED = record
          nmhdr : NMHDR;
          msg : UINT;
          wParam : WPARAM;
          lParam : LPARAM;
          chrg : CHARRANGE;
       end;
     _enprotected = ENPROTECTED;
     Tenprotected = ENPROTECTED;
     Penprotected = ^ENPROTECTED;

     SERVICE_STATUS = record
          dwServiceType : DWORD;
          dwCurrentState : DWORD;
          dwControlsAccepted : DWORD;
          dwWin32ExitCode : DWORD;
          dwServiceSpecificExitCode : DWORD;
          dwCheckPoint : DWORD;
          dwWaitHint : DWORD;
       end;
     LPSERVICE_STATUS = ^SERVICE_STATUS;
     _SERVICE_STATUS = SERVICE_STATUS;
     TSERVICESTATUS = SERVICE_STATUS;
     PSERVICESTATUS = ^SERVICE_STATUS;

     ENUM_SERVICE_STATUSA= record
          lpServiceName : LPSTR;
          lpDisplayName : LPSTR;
          ServiceStatus : SERVICE_STATUS;
       end;
     LPENUM_SERVICE_STATUSA = ^ENUM_SERVICE_STATUSA;
     _ENUM_SERVICE_STATUSA  = ENUM_SERVICE_STATUSA;
     TENUMSERVICESTATUSA    = ENUM_SERVICE_STATUSA;
     PENUMSERVICESTATUSA   = ^ENUM_SERVICE_STATUSA;

     ENUM_SERVICE_STATUSW = record
          lpServiceName : LPWSTR;
          lpDisplayName : LPWSTR;
          ServiceStatus : SERVICE_STATUS;
       end;
     LPENUM_SERVICE_STATUSW = ^ENUM_SERVICE_STATUSW;
     _ENUM_SERVICE_STATUSW  = ENUM_SERVICE_STATUSW;
     TENUMSERVICESTATUSW    = ENUM_SERVICE_STATUSW;
     PENUMSERVICESTATUSW    = ^ENUM_SERVICE_STATUSW;

     {$ifdef Unicode}
     LPENUM_SERVICE_STATUS = ^ENUM_SERVICE_STATUSW;
     _ENUM_SERVICE_STATUS  = ENUM_SERVICE_STATUSW;
     TENUMSERVICESTATUS    = ENUM_SERVICE_STATUSW;
     ENUMSERVICESTATUS     = ENUM_SERVICE_STATUSW;
     PENUMSERVICESTATUS    = ^ENUM_SERVICE_STATUSW;
     {$else}
     LPENUM_SERVICE_STATUS = ^ENUM_SERVICE_STATUSA;
     _ENUM_SERVICE_STATUS  = ENUM_SERVICE_STATUSA;
     TENUMSERVICESTATUS    = ENUM_SERVICE_STATUSA;
     ENUMSERVICESTATUS     = ENUM_SERVICE_STATUSA;
     PENUMSERVICESTATUS    = ^ENUM_SERVICE_STATUSA;
     {$endif}

     ENUMLOGFONTA = record
          elfLogFont : LOGFONTA;
          elfFullName : array[0..(LF_FULLFACESIZE)-1] of AnsiCHAR;
          elfStyle : array[0..(LF_FACESIZE)-1] of AnsiCHAR;
       end;
     tagENUMLOGFONTA =  ENUMLOGFONTA;
     TENUMLOGFONTA   =  ENUMLOGFONTA;
     PENUMLOGFONTA   = ^ENUMLOGFONTA;

     ENUMLOGFONTW = record
          elfLogFont : LOGFONTW;
          elfFullName : array[0..(LF_FULLFACESIZE)-1] of WCHAR;
          elfStyle : array[0..(LF_FACESIZE)-1] of WCHAR;
       end;
     tagENUMLOGFONTW = ENUMLOGFONTW;
     TENUMLOGFONTW   = ENUMLOGFONTW;
     PENUMLOGFONTW   = ^ENUMLOGFONTW;
     {$ifdef Unicode}
     tagENUMLOGFONT  = ENUMLOGFONTW;
     TENUMLOGFONT    = ENUMLOGFONTW;
     ENUMLOGFONT     = ENUMLOGFONTW;
     PENUMLOGFONT    = ^ENUMLOGFONTW;
     {$else}
     tagENUMLOGFONT  = ENUMLOGFONTA;
     TENUMLOGFONT    = ENUMLOGFONTA;
     ENUMLOGFONT     = ENUMLOGFONTA;
     PENUMLOGFONT    = ^ENUMLOGFONTA;
     {$endif}
     ENUMLOGFONTEXA = record
          elfLogFont : LOGFONTA;
          elfFullName : array[0..(LF_FULLFACESIZE)-1] of AnsiCHAR;
          elfStyle : array[0..(LF_FACESIZE)-1] of AnsiCHAR;
          elfScript : array[0..(LF_FACESIZE)-1] of AnsiCHAR;
       end;
     tagENUMLOGFONTEXA = ENUMLOGFONTEXA;
     TENUMLOGFONTEXA   = ENUMLOGFONTEXA;
     PENUMLOGFONTEXA   = ^ENUMLOGFONTEXA;

     ENUMLOGFONTEXW = record
          elfLogFont : LOGFONTW;
          elfFullName : array[0..(LF_FULLFACESIZE)-1] of WCHAR;
          elfStyle : array[0..(LF_FACESIZE)-1] of WCHAR;
          elfScript : array[0..(LF_FACESIZE)-1] of WCHAR;
       end;
          {
            Then follow:

            TCHAR SourceName[]
            TCHAR Computername[]
            SID   UserSid
            TCHAR Strings[]
            BYTE  Data[]
            CHAR  Pad[]
            DWORD Length;
           }
     tagENUMLOGFONTEXW = ENUMLOGFONTEXW;
     TENUMLOGFONTEXW   = ENUMLOGFONTEXW;
     PENUMLOGFONTEXW   = ^ENUMLOGFONTEXW;

     {$ifdef Unicode}
     tagENUMLOGFONTEX  = ENUMLOGFONTEXW;
     ENUMLOGFONTEX     = ENUMLOGFONTEXW;
     TENUMLOGFONTEX    = ENUMLOGFONTEXW;
     PENUMLOGFONTEX    = ^ENUMLOGFONTEXW;
     {$else}
     tagENUMLOGFONTEX  = ENUMLOGFONTEXA;
     ENUMLOGFONTEX     = ENUMLOGFONTEXA;
     TENUMLOGFONTEX    = ENUMLOGFONTEXA;
     PENUMLOGFONTEX    = ^ENUMLOGFONTEXA;
     {$endif}

     EVENTLOGRECORD = record
          Length : DWORD;
          Reserved : DWORD;
          RecordNumber : DWORD;
          TimeGenerated : DWORD;
          TimeWritten : DWORD;
          EventID : DWORD;
          EventType : WORD;
          NumStrings : WORD;
          EventCategory : WORD;
          ReservedFlags : WORD;
          ClosingRecordNumber : DWORD;
          StringOffset : DWORD;
          UserSidLength : DWORD;
          UserSidOffset : DWORD;
          DataLength : DWORD;
          DataOffset : DWORD;
       end;
     _EVENTLOGRECORD = EVENTLOGRECORD;
     TEVENTLOGRECORD = EVENTLOGRECORD;
     PEVENTLOGRECORD = ^EVENTLOGRECORD;

     EVENTMSG = record
          message : UINT;
          paramL : UINT;
          paramH : UINT;
          time : DWORD;
          hwnd : HWND;
       end;
     tagEVENTMSG = EVENTMSG;
     TEVENTMSG = EVENTMSG;
     PEVENTMSG = ^EVENTMSG;

     EXCEPTION_POINTERS = record
          ExceptionRecord : PEXCEPTION_RECORD;
          ContextRecord : PCONTEXT;
       end;
     LPEXCEPTION_POINTERS = ^EXCEPTION_POINTERS;
     PEXCEPTION_POINTERS = ^EXCEPTION_POINTERS;
     _EXCEPTION_POINTERS = EXCEPTION_POINTERS;
     TEXCEPTIONPOINTERS = EXCEPTION_POINTERS;
     PEXCEPTIONPOINTERS = ^EXCEPTION_POINTERS;

     EXT_BUTTON = record
          idCommand : WORD;
          idsHelp : WORD;
          fsStyle : WORD;
       end;
     LPEXT_BUTTON = ^EXT_BUTTON;
     _EXT_BUTTON = EXT_BUTTON;
     TEXTBUTTON = EXT_BUTTON;
     PEXTBUTTON = ^EXT_BUTTON;

     FILTERKEYS = record
          cbSize : UINT;
          dwFlags : DWORD;
          iWaitMSec : DWORD;
          iDelayMSec : DWORD;
          iRepeatMSec : DWORD;
          iBounceMSec : DWORD;
       end;
     tagFILTERKEYS = FILTERKEYS;
     TFILTERKEYS = FILTERKEYS;
     PFILTERKEYS = ^FILTERKEYS;

     FIND_NAME_BUFFER = record
          length : UCHAR;
          access_control : UCHAR;
          frame_control : UCHAR;
          destination_addr : array[0..5] of UCHAR;
          source_addr : array[0..5] of UCHAR;
          routing_info : array[0..17] of UCHAR;
       end;
     _FIND_NAME_BUFFER = FIND_NAME_BUFFER;
     TFINDNAMEBUFFER = FIND_NAME_BUFFER;
     PFINDNAMEBUFFER = ^FIND_NAME_BUFFER;

     FIND_NAME_HEADER = record
          node_count : WORD;
          reserved : UCHAR;
          unique_group : UCHAR;
       end;
     _FIND_NAME_HEADER = FIND_NAME_HEADER;
     TFINDNAMEHEADER = FIND_NAME_HEADER;
     PFINDNAMEHEADER = ^FIND_NAME_HEADER;

     FINDREPLACE = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HINST;
          Flags : DWORD;
          lpstrFindWhat : LPTSTR;
          lpstrReplaceWith : LPTSTR;
          wFindWhatLen : WORD;
          wReplaceWithLen : WORD;
          lCustData : LPARAM;
          lpfnHook : LPFRHOOKPROC;
          lpTemplateName : LPCTSTR;
       end;
     LPFINDREPLACE = ^FINDREPLACE;
     TFINDREPLACE = FINDREPLACE;
     PFINDREPLACE = ^FINDREPLACE;

     {FINDTEXT = record conflicts with FindText function }
     TFINDTEXT = record
          chrg : CHARRANGE;
          lpstrText : LPSTR;
       end;
     _findtext = TFINDTEXT;
     Pfindtext = ^TFINDTEXT;

     FINDTEXTEX = record
          chrg : CHARRANGE;
          lpstrText : LPSTR;
          chrgText : CHARRANGE;
       end;
     _findtextex = FINDTEXTEX;
     Tfindtextex = FINDTEXTEX;
     Pfindtextex = ^FINDTEXTEX;

     FMS_GETDRIVEINFOA = record
          dwTotalSpace : DWORD;
          dwFreeSpace : DWORD;
          szPath : array[0..259] of AnsiCHAR;
          szVolume : array[0..13] of AnsiCHAR;
          szShare : array[0..127] of AnsiCHAR;
       end;
     _FMS_GETDRIVEINFOA = FMS_GETDRIVEINFOA;
     TFMSGETDRIVEINFOA = FMS_GETDRIVEINFOA;
     PFMSGETDRIVEINFOA = ^FMS_GETDRIVEINFOA;

     FMS_GETDRIVEINFOW = record
          dwTotalSpace : DWORD;
          dwFreeSpace : DWORD;
          szPath : array[0..259] of WCHAR;
          szVolume : array[0..13] of WCHAR;
          szShare : array[0..127] of WCHAR;
       end;
     _FMS_GETDRIVEINFOW = FMS_GETDRIVEINFOW;
     TFMSGETDRIVEINFOW  = FMS_GETDRIVEINFOW;
     PFMSGETDRIVEINFOW  = ^FMS_GETDRIVEINFOW;
     {$ifdef Unicode}
       _FMS_GETDRIVEINFO = FMS_GETDRIVEINFOW;
       TFMSGETDRIVEINFO  = FMS_GETDRIVEINFOW;
       PFMSGETDRIVEINFO  = PFMSGETDRIVEINFOW;
       FMS_GETDRIVEINFO  = FMS_GETDRIVEINFOW;
     {$else}
       _FMS_GETDRIVEINFO = FMS_GETDRIVEINFOA;
       TFMSGETDRIVEINFO  = FMS_GETDRIVEINFOA;
       PFMSGETDRIVEINFO  = PFMSGETDRIVEINFOA;
       FMS_GETDRIVEINFO  = FMS_GETDRIVEINFOA;
     {$endif}

     FMS_GETFILESELA = record
          ftTime : FILETIME;
          dwSize : DWORD;
          bAttr : BYTE;
          szName : array[0..259] of AnsiCHAR;
       end;
     _FMS_GETFILESELA = FMS_GETFILESELA;
     TFMSGETFILESELA  = FMS_GETFILESELA;
     PFMSGETFILESELA  = ^FMS_GETFILESELA;

     FMS_GETFILESELW = record
          ftTime : FILETIME;
          dwSize : DWORD;
          bAttr : BYTE;
          szName : array[0..259] of WCHAR;
       end;
     _FMS_GETFILESELW = FMS_GETFILESELW;
     TFMSGETFILESELW  = FMS_GETFILESELW;
     PFMSGETFILESELW  = ^FMS_GETFILESELW;
     {$ifdef Unicode}
       _FMS_GETFILESEL  = FMS_GETFILESELW;
       TFMSGETFILESEL   = FMS_GETFILESELW;
       FMS_GETFILESEL   = FMS_GETFILESELW;
       PFMSGETFILESEL   = PFMSGETFILESELW;
     {$else}
       _FMS_GETFILESEL  = FMS_GETFILESELA;
       TFMSGETFILESEL   = FMS_GETFILESELA;
       FMS_GETFILESEL   = FMS_GETFILESELA;
       PFMSGETFILESEL   = PFMSGETFILESELA;
     {$endif}

     FMS_LOADA = record
          dwSize : DWORD;
          szMenuName : array[0..(MENU_TEXT_LEN)-1] of AnsiCHAR;
          hMenu : HMENU;
          wMenuDelta : UINT;
       end;
     _FMS_LOADA = FMS_LOADA;
     TFMSLOADA  = FMS_LOADA;
     PFMSLOADA  = ^FMS_LOADA;

     FMS_LOADW = record
          dwSize : DWORD;
          szMenuName : array[0..(MENU_TEXT_LEN)-1] of WCHAR;
          hMenu : HMENU;
          wMenuDelta : UINT;
       end;
     _FMS_LOADW = FMS_LOADW;
     TFMSLOADW = FMS_LOADW;
     PFMSLOADW = ^FMS_LOADW;
     {$ifdef unicode}
       _FMS_LOAD = FMS_LOADW;
       TFMSLOAD  = FMS_LOADW;
       PFMSLOAD  = PFMSLOADW;
       FMS_LOAD  = FMS_LOADW;
     {$else}
       _FMS_LOAD = FMS_LOADA;
       TFMSLOAD  = FMS_LOADA;
       PFMSLOAD  = PFMSLOADA;
       FMS_LOAD  = FMS_LOADA;
     {$endif}

     FMS_TOOLBARLOAD = record
          dwSize : DWORD;
          lpButtons : LPEXT_BUTTON;
          cButtons : WORD;
          cBitmaps : WORD;
          idBitmap : WORD;
          hBitmap : HBITMAP;
       end;
     _FMS_TOOLBARLOAD = FMS_TOOLBARLOAD;
     TFMSTOOLBARLOAD = FMS_TOOLBARLOAD;
     PFMSTOOLBARLOAD = ^FMS_TOOLBARLOAD;

     FOCUS_EVENT_RECORD = record
          bSetFocus : WINBOOL;
       end;
     _FOCUS_EVENT_RECORD = FOCUS_EVENT_RECORD;
     TFOCUSEVENTRECORD = FOCUS_EVENT_RECORD;
     PFOCUSEVENTRECORD = ^FOCUS_EVENT_RECORD;

     FORM_INFO_1 = record
          Flags : DWORD;
          pName : LPTSTR;
          Size : SIZEL;
          ImageableArea : RECTL;
       end;
     _FORM_INFO_1 = FORM_INFO_1;
     TFORMINFO1 = FORM_INFO_1;
     PFORMINFO1 = ^FORM_INFO_1;

     FORMAT_PARAMETERS = record
          MediaType : MEDIA_TYPE;
          StartCylinderNumber : DWORD;
          EndCylinderNumber : DWORD;
          StartHeadNumber : DWORD;
          EndHeadNumber : DWORD;
       end;
     _FORMAT_PARAMETERS = FORMAT_PARAMETERS;
     TFORMATPARAMETERS = FORMAT_PARAMETERS;
     PFORMATPARAMETERS = ^FORMAT_PARAMETERS;

     FORMATRANGE = record
          _hdc : HDC;
          hdcTarget : HDC;
          rc : RECT;
          rcPage : RECT;
          chrg : CHARRANGE;
       end;
     _formatrange = FORMATRANGE;
     Tformatrange = FORMATRANGE;
     Pformatrange = ^FORMATRANGE;

     GCP_RESULTSA = record
          lStructSize : DWORD;
          lpOutString : LPSTR;
          lpOrder : ^UINT;
          lpDx : ^WINT;
          lpCaretPos : ^WINT;
          lpClass : LPSTR;           { Unicode-independent, i.e. not LPTSTR }
          lpGlyphs : LPWSTR;         { Unicode-independent }
          nGlyphs : UINT;
          nMaxFit : WINT;
       end;
     LPGCP_RESULTSA = ^GCP_RESULTSA;
     tagGCP_RESULTSA= GCP_RESULTSA;
     TGCPRESULTSA   = GCP_RESULTSA;
     PGCPRESULTSA   = ^GCP_RESULTSA;

     GCP_RESULTSW = record
          lStructSize : DWORD;
          lpOutString : LPWSTR;
          lpOrder : ^UINT;
          lpDx : ^WINT;
          lpCaretPos : ^WINT;
          lpClass : LPSTR;           { Unicode-independent, i.e. not LPTSTR }
          lpGlyphs : LPWSTR;         { Unicode-independent }
          nGlyphs : UINT;
          nMaxFit : WINT;
       end;
     LPGCP_RESULTSW  = ^GCP_RESULTSW;
     tagGCP_RESULTSW = GCP_RESULTSW;
     TGCPRESULTSW    = GCP_RESULTSW;
     PGCPRESULTSW    = ^GCP_RESULTSW;

     {$ifdef Unicode}
     LPGCP_RESULTS   = ^GCP_RESULTSW;
     tagGCP_RESULTS  = GCP_RESULTSW;
     TGCPRESULTS     = GCP_RESULTSW;
     GCPRESULTS      = GCP_RESULTSW;
     PGCPRESULTS     = ^GCP_RESULTSW;
     {$else}
     LPGCP_RESULTS   = ^GCP_RESULTSA;
     tagGCP_RESULTS  = GCP_RESULTSA;
     TGCPRESULTS     = GCP_RESULTSA;
     GCPRESULTS      = GCP_RESULTSA;
     PGCPRESULTS     = ^GCP_RESULTSA;
     {$endif}

     GENERIC_MAPPING = record
          GenericRead : ACCESS_MASK;
          GenericWrite : ACCESS_MASK;
          GenericExecute : ACCESS_MASK;
          GenericAll : ACCESS_MASK;
       end;
     PGENERIC_MAPPING = ^GENERIC_MAPPING;
     _GENERIC_MAPPING = GENERIC_MAPPING;
     TGENERICMAPPING = GENERIC_MAPPING;
     PGENERICMAPPING = ^GENERIC_MAPPING;

     GLYPHMETRICS = record
          gmBlackBoxX : UINT;
          gmBlackBoxY : UINT;
          gmptGlyphOrigin : POINT;
          gmCellIncX : SHORT;
          gmCellIncY : SHORT;
       end;
     LPGLYPHMETRICS = ^GLYPHMETRICS;
     _GLYPHMETRICS = GLYPHMETRICS;
     TGLYPHMETRICS = GLYPHMETRICS;
     PGLYPHMETRICS = ^GLYPHMETRICS;

     HANDLETABLE = record
          objectHandle : array[0..0] of HGDIOBJ;
       end;
     tagHANDLETABLE = HANDLETABLE;
     THANDLETABLE = HANDLETABLE;
     LPHANDLETABLE = ^HANDLETABLE;

     HD_HITTESTINFO = record
          pt : POINT;
          flags : UINT;
          iItem : longint;
       end;
     _HD_HITTESTINFO = HD_HITTESTINFO;
     THDHITTESTINFO = HD_HITTESTINFO;
     PHDHITTESTINFO = ^HD_HITTESTINFO;

     HD_ITEM = record
          mask : UINT;
          cxy : longint;
          pszText : LPTSTR;
          hbm : HBITMAP;
          cchTextMax : longint;
          fmt : longint;
          lParam : LPARAM;
       end;
     _HD_ITEM = HD_ITEM;
     THDITEM = HD_ITEM;
     PHDITEM = ^HD_ITEM;

     WINDOWPOS = record
          _hwnd : HWND;
          hwndInsertAfter : HWND;
          x : longint;
          y : longint;
          cx : longint;
          cy : longint;
          flags : UINT;
       end;
     LPWINDOWPOS = ^WINDOWPOS;
     _WINDOWPOS = WINDOWPOS;
     TWINDOWPOS = WINDOWPOS;
     PWINDOWPOS = ^WINDOWPOS;

     PWindowInfo = ^TWindowInfo;
     LPWindowInfo = ^TWindowInfo;
     tagWINDOWINFO = record
          cbSize: DWORD;
          rcWindow: TRect;
          rcClient: TRect;
          dwStyle: DWORD;
          dwExStyle: DWORD;
          dwWindowStatus: DWORD;
          cxWindowBorders: UINT;
          cyWindowBorders: UINT;
          atomWindowType: TAtom;
          wCreatorVersion: WORD;
       end;
     TWindowInfo = tagWINDOWINFO;

     HD_LAYOUT = record
          prc : ^RECT;
          pwpos : ^WINDOWPOS;
       end;
     _HD_LAYOUT = HD_LAYOUT;
     THDLAYOUT = HD_LAYOUT;
     PHDLAYOUT = ^HD_LAYOUT;

     HD_NOTIFY = record
          hdr : NMHDR;
          iItem : longint;
          iButton : longint;
          pitem : ^HD_ITEM;
       end;
     _HD_NOTIFY = HD_NOTIFY;
     THDNOTIFY = HD_NOTIFY;
     PHDNOTIFY = ^HD_NOTIFY;

     HELPINFO = record
          cbSize : UINT;
          iContextType : longint;
          iCtrlId : longint;
          hItemHandle : HANDLE;
          dwContextId : DWORD_PTR;
 // NOTE:  this is defined as DWORD in http://msdn.microsoft.com/en-us/library/windows/desktop/bb773313%28v=vs.85%29.asp
// but mingw has it as DWORD_PTR, so we will use that
          MousePos : POINT;
       end;
     LPHELPINFO = ^HELPINFO;
     tagHELPINFO = HELPINFO;
     THELPINFO = HELPINFO;
     PHELPINFO = ^HELPINFO;

     HELPWININFOA = record
          wStructSize : longint;
          x : longint;
          y : longint;
          dx : longint;
          dy : longint;
          wMax : longint;
          rgchMember : array[0..1] of AnsiCHAR;
       end;
     THELPWININFOA = HELPWININFOA;
     PHELPWININFOA = ^HELPWININFOA;

     HELPWININFOW = record
          wStructSize : longint;
          x : longint;
          y : longint;
          dx : longint;
          dy : longint;
          wMax : longint;
          rgchMember : array[0..1] of WCHAR;
       end;
     THELPWININFOW = HELPWININFOW;
     PHELPWININFOW = ^HELPWININFOW;

     {$ifdef Unicode}
       HELPWININFO  = HELPWININFOW;
       THELPWININFO = HELPWININFOW;
       PHELPWININFO = PHELPWININFOW;
     {$else}
       HELPWININFO  = HELPWININFOA;
       THELPWININFO = HELPWININFOA;
       PHELPWININFO = PHELPWININFOA;
     {$endif}

     HIGHCONTRAST = record
          cbSize : UINT;
          dwFlags : DWORD;
          lpszDefaultScheme : LPTSTR;
       end;
     LPHIGHCONTRAST = ^HIGHCONTRAST;
     tagHIGHCONTRAST = HIGHCONTRAST;
     THIGHCONTRAST = HIGHCONTRAST;
     PHIGHCONTRAST = ^HIGHCONTRAST;

     HSZPAIR = record
          hszSvc : HSZ;
          hszTopic : HSZ;
       end;
     tagHSZPAIR = HSZPAIR;
     THSZPAIR = HSZPAIR;
     PHSZPAIR = ^HSZPAIR;

     ICONINFO = record
          fIcon : WINBOOL;
          xHotspot : DWORD;
          yHotspot : DWORD;
          hbmMask : HBITMAP;
          hbmColor : HBITMAP;
       end;
     _ICONINFO = ICONINFO;
     TICONINFO = ICONINFO;
     PICONINFO = ^ICONINFO;

     ICONMETRICS = record
          cbSize : UINT;
          iHorzSpacing : longint;
          iVertSpacing : longint;
          iTitleWrap : longint;
          lfFont : LOGFONT;
       end;
     LPICONMETRICS = ^ICONMETRICS;
     tagICONMETRICS = ICONMETRICS;
     TICONMETRICS = ICONMETRICS;
     PICONMETRICS = ^ICONMETRICS;

     IMAGEINFO = record
          hbmImage : HBITMAP;
          hbmMask : HBITMAP;
          Unused1 : longint;
          Unused2 : longint;
          rcImage : RECT;
       end;
     _IMAGEINFO = IMAGEINFO;
     TIMAGEINFO = IMAGEINFO;
     PIMAGEINFO = ^IMAGEINFO;

     KEY_EVENT_RECORD = packed record
          bKeyDown : WINBOOL;
          wRepeatCount : WORD;
          wVirtualKeyCode : WORD;
          wVirtualScanCode : WORD;
          case longint of
             0 : ( UnicodeChar : WCHAR;
                   dwControlKeyState : DWORD; );
             1 : ( AsciiChar : CHAR );
       end;
     _KEY_EVENT_RECORD = KEY_EVENT_RECORD;
     TKEYEVENTRECORD = KEY_EVENT_RECORD;
     PKEYEVENTRECORD = ^KEY_EVENT_RECORD;

     MOUSE_EVENT_RECORD = record
          dwMousePosition : COORD;
          dwButtonState : DWORD;
          dwControlKeyState : DWORD;
          dwEventFlags : DWORD;
       end;
     _MOUSE_EVENT_RECORD = MOUSE_EVENT_RECORD;
     TMOUSEEVENTRECORD = MOUSE_EVENT_RECORD;
     PMOUSEEVENTRECORD = ^MOUSE_EVENT_RECORD;

     WINDOW_BUFFER_SIZE_RECORD = record
          dwSize : COORD;
       end;
     _WINDOW_BUFFER_SIZE_RECORD = WINDOW_BUFFER_SIZE_RECORD;
     TWINDOWBUFFERSIZERECORD = WINDOW_BUFFER_SIZE_RECORD;
     PWINDOWBUFFERSIZERECORD = ^WINDOW_BUFFER_SIZE_RECORD;

     MENU_EVENT_RECORD = record
          dwCommandId : UINT;
       end;
     PMENU_EVENT_RECORD = ^MENU_EVENT_RECORD;
     _MENU_EVENT_RECORD = MENU_EVENT_RECORD;
     TMENUEVENTRECORD = MENU_EVENT_RECORD;
     PMENUEVENTRECORD = ^MENU_EVENT_RECORD;

     INPUT_RECORD = record
          EventType: Word;
          Reserved: Word;
          Event : record case longint of
                 0 : ( KeyEvent : KEY_EVENT_RECORD );
                 1 : ( MouseEvent : MOUSE_EVENT_RECORD );
                 2 : ( WindowBufferSizeEvent : WINDOW_BUFFER_SIZE_RECORD );
                 3 : ( MenuEvent : MENU_EVENT_RECORD );
                 4 : ( FocusEvent : FOCUS_EVENT_RECORD );
       end;
      end;
     PINPUT_RECORD = ^INPUT_RECORD;
     _INPUT_RECORD = INPUT_RECORD;
     TINPUTRECORD = INPUT_RECORD;
     PINPUTRECORD = ^INPUT_RECORD;

     SYSTEMTIME = record
          case integer of
           1 : (
             wYear : WORD;
             wMonth : WORD;
             wDayOfWeek : WORD;
             wDay : WORD;
             wHour : WORD;
             wMinute : WORD;
             wSecond : WORD;
             wMilliseconds : WORD;
               );
           { Compatibility for FPC }
           2 : (
             Year : WORD;
             Month : WORD;
             DayOfWeek : WORD;
             Day : WORD;
             Hour : WORD;
             Minute : WORD;
             Second : WORD;
             Millisecond : WORD;
               );
       end;
     LPSYSTEMTIME = ^SYSTEMTIME;
     _SYSTEMTIME = SYSTEMTIME;
     TSYSTEMTIME = SYSTEMTIME;
     PSYSTEMTIME = ^SYSTEMTIME;

     JOB_INFO_1 = record
          JobId : DWORD;
          pPrinterName : LPTSTR;
          pMachineName : LPTSTR;
          pUserName : LPTSTR;
          pDocument : LPTSTR;
          pDatatype : LPTSTR;
          pStatus : LPTSTR;
          Status : DWORD;
          Priority : DWORD;
          Position : DWORD;
          TotalPages : DWORD;
          PagesPrinted : DWORD;
          Submitted : SYSTEMTIME;
       end;
     _JOB_INFO_1 = JOB_INFO_1;
     TJOBINFO1 = JOB_INFO_1;
     PJOBINFO1 = ^JOB_INFO_1;

     SID_IDENTIFIER_AUTHORITY = record
          Value : array[0..5] of BYTE;
       end;
     LPSID_IDENTIFIER_AUTHORITY = ^SID_IDENTIFIER_AUTHORITY;
     PSID_IDENTIFIER_AUTHORITY = ^SID_IDENTIFIER_AUTHORITY;
     _SID_IDENTIFIER_AUTHORITY = SID_IDENTIFIER_AUTHORITY;
     TSIDIDENTIFIERAUTHORITY = SID_IDENTIFIER_AUTHORITY;
     PSIDIDENTIFIERAUTHORITY = ^SID_IDENTIFIER_AUTHORITY;

     SID = record
          Revision : BYTE;
          SubAuthorityCount : BYTE;
          IdentifierAuthority : SID_IDENTIFIER_AUTHORITY;
          SubAuthority : array[0..(ANYSIZE_ARRAY)-1] of DWORD;
       end;
     _SID = SID;
     TSID = SID;
     PSID = ^SID;

     SECURITY_DESCRIPTOR_CONTROL = WORD;
     PSECURITY_DESCRIPTOR_CONTROL = ^SECURITY_DESCRIPTOR_CONTROL;
     TSECURITYDESCRIPTORCONTROL = SECURITY_DESCRIPTOR_CONTROL;
     PSECURITYDESCRIPTORCONTROL = ^SECURITY_DESCRIPTOR_CONTROL;

     SECURITY_DESCRIPTOR = record
          Revision : BYTE;
          Sbz1 : BYTE;
          Control : SECURITY_DESCRIPTOR_CONTROL;
          Owner : PSID;
          Group : PSID;
          Sacl : PACL;
          Dacl : PACL;
       end;
     PSECURITY_DESCRIPTOR = ^SECURITY_DESCRIPTOR;
     _SECURITY_DESCRIPTOR = SECURITY_DESCRIPTOR;
     TSECURITYDESCRIPTOR = SECURITY_DESCRIPTOR;
     PSECURITYDESCRIPTOR = ^SECURITY_DESCRIPTOR;

     JOB_INFO_2 = record
          JobId : DWORD;
          pPrinterName : LPTSTR;
          pMachineName : LPTSTR;
          pUserName : LPTSTR;
          pDocument : LPTSTR;
          pNotifyName : LPTSTR;
          pDatatype : LPTSTR;
          pPrintProcessor : LPTSTR;
          pParameters : LPTSTR;
          pDriverName : LPTSTR;
          pDevMode : LPDEVMODE;
          pStatus : LPTSTR;
          pSecurityDescriptor : PSECURITY_DESCRIPTOR;
          Status : DWORD;
          Priority : DWORD;
          Position : DWORD;
          StartTime : DWORD;
          UntilTime : DWORD;
          TotalPages : DWORD;
          Size : DWORD;
          Submitted : SYSTEMTIME;
          Time : DWORD;
          PagesPrinted : DWORD;
       end;
     _JOB_INFO_2 = JOB_INFO_2;
     TJOBINFO2 = JOB_INFO_2;
     PJOBINFO2 = ^JOB_INFO_2;

     KERNINGPAIR = record
          wFirst : WORD;
          wSecond : WORD;
          iKernAmount : longint;
       end;
     LPKERNINGPAIR = ^KERNINGPAIR;
     tagKERNINGPAIR = KERNINGPAIR;
     TKERNINGPAIR = KERNINGPAIR;
     PKERNINGPAIR = ^KERNINGPAIR;

     LANA_ENUM = record
          length : UCHAR;
          lana : array[0..(MAX_LANA)-1] of UCHAR;
       end;
     _LANA_ENUM = LANA_ENUM;
     TLANAENUM = LANA_ENUM;
     PLANAENUM = ^LANA_ENUM;

     LDT_ENTRY = record
          LimitLow : WORD;
          BaseLow : WORD;
          HighWord : record
              case longint of
                 0 : ( Bytes : record
                      BaseMid : BYTE;
                      Flags1 : BYTE;
                      Flags2 : BYTE;
                      BaseHi : BYTE;
                   end );
                 1 : ( Bits : record
                      flag0 : longint;
                   end );
              end;
       end;
     LPLDT_ENTRY = ^LDT_ENTRY;
     PLDT_ENTRY = ^LDT_ENTRY;
     _LDT_ENTRY = LDT_ENTRY;
     TLDTENTRY = LDT_ENTRY;
     PLDTENTRY = ^LDT_ENTRY;

  const
     bm_LDT_ENTRY_BaseMid = $FF;
     bp_LDT_ENTRY_BaseMid = 0;
     bm_LDT_ENTRY_Type = $1F00;
     bp_LDT_ENTRY_Type = 8;
     bm_LDT_ENTRY_Dpl = $6000;
     bp_LDT_ENTRY_Dpl = 13;
     bm_LDT_ENTRY_Pres = $8000;
     bp_LDT_ENTRY_Pres = 15;
     bm_LDT_ENTRY_LimitHi = $F0000;
     bp_LDT_ENTRY_LimitHi = 16;
     bm_LDT_ENTRY_Sys = $100000;
     bp_LDT_ENTRY_Sys = 20;
     bm_LDT_ENTRY_Reserved_0 = $200000;
     bp_LDT_ENTRY_Reserved_0 = 21;
     bm_LDT_ENTRY_Default_Big = $400000;
     bp_LDT_ENTRY_Default_Big = 22;
     bm_LDT_ENTRY_Granularity = $800000;
     bp_LDT_ENTRY_Granularity = 23;
     bm_LDT_ENTRY_BaseHi = $FF000000;
     bp_LDT_ENTRY_BaseHi = 24;

  type

     LOCALESIGNATURE = record
          lsUsb : array[0..3] of DWORD;
          lsCsbDefault : array[0..1] of DWORD;
          lsCsbSupported : array[0..1] of DWORD;
       end;
     tagLOCALESIGNATURE = LOCALESIGNATURE;
     TLOCALESIGNATURE = LOCALESIGNATURE;
     PLOCALESIGNATURE = ^LOCALESIGNATURE;

     LOCALGROUP_MEMBERS_INFO_0 = record
          lgrmi0_sid : PSID;
       end;
     _LOCALGROUP_MEMBERS_INFO_0 = LOCALGROUP_MEMBERS_INFO_0;
     TLOCALGROUPMEMBERSINFO0 = LOCALGROUP_MEMBERS_INFO_0;
     PLOCALGROUPMEMBERSINFO0 = ^LOCALGROUP_MEMBERS_INFO_0;

     LOCALGROUP_MEMBERS_INFO_3 = record
          lgrmi3_domainandname : LPWSTR;
       end;
     _LOCALGROUP_MEMBERS_INFO_3 = LOCALGROUP_MEMBERS_INFO_3;
     TLOCALGROUPMEMBERSINFO3 = LOCALGROUP_MEMBERS_INFO_3;
     PLOCALGROUPMEMBERSINFO3 = ^LOCALGROUP_MEMBERS_INFO_3;

     FXPT16DOT16 = longint;
     LPFXPT16DOT16 = ^FXPT16DOT16;
     TFXPT16DOT16 = FXPT16DOT16;
     PFXPT16DOT16 = ^FXPT16DOT16;

     LUID = TlargeInteger;
     TLUID = LUID;
     PLUID = ^LUID;

     LUID_AND_ATTRIBUTES = packed record
          Luid : LUID;
          Attributes : DWORD;
       end;
     _LUID_AND_ATTRIBUTES = LUID_AND_ATTRIBUTES;
     TLUIDANDATTRIBUTES = LUID_AND_ATTRIBUTES;
     PLUIDANDATTRIBUTES = ^LUID_AND_ATTRIBUTES;

     LUID_AND_ATTRIBUTES_ARRAY = array[0..(ANYSIZE_ARRAY)-1] of LUID_AND_ATTRIBUTES;
     PLUID_AND_ATTRIBUTES_ARRAY = ^LUID_AND_ATTRIBUTES_ARRAY;
     TLUIDANDATTRIBUTESARRAY = LUID_AND_ATTRIBUTES_ARRAY;
     PLUIDANDATTRIBUTESARRAY = ^LUID_AND_ATTRIBUTES_ARRAY;

     LV_COLUMN = record
          mask : UINT;
          fmt : longint;
          cx : longint;
          pszText : LPTSTR;
          cchTextMax : longint;
          iSubItem : longint;
       end;
     _LV_COLUMN = LV_COLUMN;
     TLVCOLUMN = LV_COLUMN;
     PLVCOLUMN = ^LV_COLUMN;

     LV_ITEM = record
          mask : UINT;
          iItem : longint;
          iSubItem : longint;
          state : UINT;
          stateMask : UINT;
          pszText : LPTSTR;
          cchTextMax : longint;
          iImage : longint;
          lParam : LPARAM;
       end;
     _LV_ITEM = LV_ITEM;
     TLVITEM = LV_ITEM;
     PLVITEM = ^LV_ITEM;

     LV_DISPINFO = record
          hdr : NMHDR;
          item : LV_ITEM;
       end;
     tagLV_DISPINFO = LV_DISPINFO;
     TLVDISPINFO = LV_DISPINFO;
     PLVDISPINFO = ^LV_DISPINFO;

     LV_FINDINFO = record
          flags : UINT;
          psz : LPCTSTR;
          lParam : LPARAM;
          pt : POINT;
          vkDirection : UINT;
       end;
     _LV_FINDINFO = LV_FINDINFO;
     TLVFINDINFO = LV_FINDINFO;
     PLVFINDINFO = ^LV_FINDINFO;

     LV_HITTESTINFO = record
          pt : POINT;
          flags : UINT;
          iItem : longint;
          iSubItem : longint;
          iGroup : longint;
       end;
     _LV_HITTESTINFO = LV_HITTESTINFO;
     TLVHITTESTINFO = LV_HITTESTINFO;
     PLVHITTESTINFO = ^LV_HITTESTINFO;

     LV_KEYDOWN = record
          hdr : NMHDR;
          wVKey : WORD;
          flags : UINT;
       end;
     tagLV_KEYDOWN = LV_KEYDOWN;
     TLVKEYDOWN = LV_KEYDOWN;
     PLVKEYDOWN = ^LV_KEYDOWN;

     MAT2 = packed record
          eM11 : FIXED;
          eM12 : FIXED;
          eM21 : FIXED;
          eM22 : FIXED;
       end;
     _MAT2 = MAT2;
     TMAT2 = MAT2;
     PMAT2 = ^MAT2;

     MDICREATESTRUCT = record
          szClass : LPCTSTR;
          szTitle : LPCTSTR;
          hOwner : HANDLE;
          x : longint;
          y : longint;
          cx : longint;
          cy : longint;
          style : DWORD;
          lParam : LPARAM;
       end;
     LPMDICREATESTRUCT = ^MDICREATESTRUCT;
     tagMDICREATESTRUCT = MDICREATESTRUCT;
     TMDICREATESTRUCT = MDICREATESTRUCT;
     PMDICREATESTRUCT = ^MDICREATESTRUCT;

     MEASUREITEMSTRUCT = record
          CtlType : UINT;
          CtlID : UINT;
          itemID : UINT;
          itemWidth : UINT;
          itemHeight : UINT;
          itemData : ULONG_PTR;
       end;
     LPMEASUREITEMSTRUCT = ^MEASUREITEMSTRUCT;
     tagMEASUREITEMSTRUCT = MEASUREITEMSTRUCT;
     TMEASUREITEMSTRUCT = MEASUREITEMSTRUCT;
     PMEASUREITEMSTRUCT = ^MEASUREITEMSTRUCT;

     MEMORY_BASIC_INFORMATION = record
          BaseAddress : PVOID;
          AllocationBase : PVOID;
          AllocationProtect : DWORD;
          RegionSize : PTRUINT; // MvdV: size_t in win SDK 6.0.
          State : DWORD;
          Protect : DWORD;
          _Type : DWORD;
       end;
     PMEMORY_BASIC_INFORMATION = ^MEMORY_BASIC_INFORMATION;
     _MEMORY_BASIC_INFORMATION = MEMORY_BASIC_INFORMATION;
     TMEMORYBASICINFORMATION = MEMORY_BASIC_INFORMATION;
     PMEMORYBASICINFORMATION = ^MEMORY_BASIC_INFORMATION;

     MEMORYSTATUS = record
          dwLength : DWORD;
          dwMemoryLoad : DWORD;
          dwTotalPhys : SIZE_T;
          dwAvailPhys : SIZE_T;
          dwTotalPageFile : SIZE_T;
          dwAvailPageFile : SIZE_T;
          dwTotalVirtual : SIZE_T;
          dwAvailVirtual : SIZE_T;
       end;
     LPMEMORYSTATUS = ^MEMORYSTATUS;
     _MEMORYSTATUS = MEMORYSTATUS;
     TMEMORYSTATUS = MEMORYSTATUS;
     PMEMORYSTATUS = ^MEMORYSTATUS;

     MENUEX_TEMPLATE_HEADER = record
          wVersion : WORD;
          wOffset : WORD;
          dwHelpId : DWORD;
       end;
     TMENUXTEMPLATEHEADER = MENUEX_TEMPLATE_HEADER;
     PMENUXTEMPLATEHEADER = ^MENUEX_TEMPLATE_HEADER;

     MENUEX_TEMPLATE_ITEM = record
          dwType : DWORD;
          dwState : DWORD;
          uId : UINT;
          bResInfo : BYTE;
          szText : array[0..0] of WCHAR;
          dwHelpId : DWORD;
       end;
     TMENUEXTEMPLATEITEM = MENUEX_TEMPLATE_ITEM;
     PMENUEXTEMPLATEITEM = ^MENUEX_TEMPLATE_ITEM;

     MENUINFO = record
          cbSize : DWORD;
          fMask : DWORD;
          dwStyle : DWORD;
          cyMax : UINT;
          hbrBack : HBRUSH;
          dwContextHelpID : DWORD;
          dwMenuData : ULONG_PTR;
       end;
     LPMENUINFO = ^MENUINFO;
     LPCMENUINFO = ^MENUINFO;
     tagMENUINFO = MENUINFO;
     TMENUINFO = MENUINFO;
     PMENUINFO = ^MENUINFO;

     MENUITEMINFOA = record
          cbSize : UINT;
          fMask : UINT;
          fType : UINT;
          fState : UINT;
          wID : UINT;
          hSubMenu : HMENU;
          hbmpChecked : HBITMAP;
          hbmpUnchecked : HBITMAP;
          dwItemData : ULONG_PTR;
          dwTypeData : LPSTR;
          cch : UINT;
          hbmpItem : HBITMAP;
       end;
     LPMENUITEMINFOA = ^MENUITEMINFOA;
     LPCMENUITEMINFOA = ^MENUITEMINFOA;
     tagMENUITEMINFOA = MENUITEMINFOA;
     TMENUITEMINFOA = MENUITEMINFOA;
     PMENUITEMINFOA = ^MENUITEMINFOA;

     MENUITEMINFOW = record
          cbSize : UINT;
          fMask : UINT;
          fType : UINT;
          fState : UINT;
          wID : UINT;
          hSubMenu : HMENU;
          hbmpChecked : HBITMAP;
          hbmpUnchecked : HBITMAP;
          dwItemData : ULONG_PTR;
          dwTypeData : LPWSTR;
          cch : UINT;
          hbmpItem : HBITMAP;
       end;
     LPMENUITEMINFOW = ^MENUITEMINFOW;
     LPCMENUITEMINFOW = ^MENUITEMINFOW;
     tagMENUITEMINFOW = MENUITEMINFOW;
     TMENUITEMINFOW = MENUITEMINFOW;
     PMENUITEMINFOW = ^MENUITEMINFOW;
     {$ifdef unicode}
     LPMENUITEMINFO   = ^MENUITEMINFOW;
     LPCMENUITEMINFO  = ^MENUITEMINFOW;
     tagMENUITEMINFO  =  MENUITEMINFOW;
     TMENUITEMINFO    =  MENUITEMINFOW;
     PMENUITEMINFO    = ^MENUITEMINFOW;
     MENUITEMINFO     =  MENUITEMINFOW;
     {$ELSE}
     LPMENUITEMINFO   = ^MENUITEMINFOA;
     LPCMENUITEMINFO  = ^MENUITEMINFOA;
     tagMENUITEMINFO  = MENUITEMINFOA;
     TMENUITEMINFO    = MENUITEMINFOA;
     PMENUITEMINFO    = ^MENUITEMINFOA;
     MENUITEMINFO     =  MENUITEMINFOA;
     {$ENDIF}

     MENUITEMTEMPLATE = record
          mtOption : WORD;
          mtID : WORD;
          mtString : array[0..0] of WCHAR;
       end;
     TMENUITEMTEMPLATE = MENUITEMTEMPLATE;
     PMENUITEMTEMPLATE = ^MENUITEMTEMPLATE;

     MENUITEMTEMPLATEHEADER = record
          versionNumber : WORD;
          offset : WORD;
       end;
     TMENUITEMTEMPLATEHEADER = MENUITEMTEMPLATEHEADER;
     PMENUITEMTEMPLATEHEADER = ^MENUITEMTEMPLATEHEADER;

     MENUTEMPLATE = record
                    end;
     LPMENUTEMPLATE = ^MENUTEMPLATE;
     TMENUTEMPLATE = MENUTEMPLATE;
     PMENUTEMPLATE = ^MENUTEMPLATE;

     METAFILEPICT = record
          mm : LONG;
          xExt : LONG;
          yExt : LONG;
          hMF : HMETAFILE;
       end;
     LPMETAFILEPICT = ^METAFILEPICT;
     tagMETAFILEPICT = METAFILEPICT;
     TMETAFILEPICT = METAFILEPICT;
     PMETAFILEPICT = ^METAFILEPICT;

     METAHEADER = packed record
          mtType : WORD;
          mtHeaderSize : WORD;
          mtVersion : WORD;
          mtSize : DWORD;
          mtNoObjects : WORD;
          mtMaxRecord : DWORD;
          mtNoParameters : WORD;
       end;
     tagMETAHEADER = METAHEADER;
     TMETAHEADER = METAHEADER;
     PMETAHEADER = ^METAHEADER;

     METARECORD = record
          rdSize : DWORD;
          rdFunction : WORD;
          rdParm : array[0..0] of WORD;
       end;
     LPMETARECORD = ^METARECORD;
     tagMETARECORD = METARECORD;
     TMETARECORD = METARECORD;
     PMETARECORD = ^METARECORD;

     MINIMIZEDMETRICS = record
          cbSize : UINT;
          iWidth : longint;
          iHorzGap : longint;
          iVertGap : longint;
          iArrange : longint;
       end;
     LPMINIMIZEDMETRICS = ^MINIMIZEDMETRICS;
     tagMINIMIZEDMETRICS = MINIMIZEDMETRICS;
     TMINIMIZEDMETRICS = MINIMIZEDMETRICS;
     PMINIMIZEDMETRICS = ^MINIMIZEDMETRICS;

     MINMAXINFO = record
          ptReserved : POINT;
          ptMaxSize : POINT;
          ptMaxPosition : POINT;
          ptMinTrackSize : POINT;
          ptMaxTrackSize : POINT;
       end;
     tagMINMAXINFO = MINMAXINFO;
     TMINMAXINFO = MINMAXINFO;
     PMINMAXINFO = ^MINMAXINFO;

     MODEMDEVCAPS = record
          dwActualSize : DWORD;
          dwRequiredSize : DWORD;
          dwDevSpecificOffset : DWORD;
          dwDevSpecificSize : DWORD;
          dwModemProviderVersion : DWORD;
          dwModemManufacturerOffset : DWORD;
          dwModemManufacturerSize : DWORD;
          dwModemModelOffset : DWORD;
          dwModemModelSize : DWORD;
          dwModemVersionOffset : DWORD;
          dwModemVersionSize : DWORD;
          dwDialOptions : DWORD;
          dwCallSetupFailTimer : DWORD;
          dwInactivityTimeout : DWORD;
          dwSpeakerVolume : DWORD;
          dwSpeakerMode : DWORD;
          dwModemOptions : DWORD;
          dwMaxDTERate : DWORD;
          dwMaxDCERate : DWORD;
          abVariablePortion : array[0..0] of BYTE;
       end;
     LPMODEMDEVCAPS = ^MODEMDEVCAPS;
     TMODEMDEVCAPS = MODEMDEVCAPS;
     PMODEMDEVCAPS = ^MODEMDEVCAPS;

     modemdevcaps_tag = MODEMDEVCAPS;

     MODEMSETTINGS = record
          dwActualSize : DWORD;
          dwRequiredSize : DWORD;
          dwDevSpecificOffset : DWORD;
          dwDevSpecificSize : DWORD;
          dwCallSetupFailTimer : DWORD;
          dwInactivityTimeout : DWORD;
          dwSpeakerVolume : DWORD;
          dwSpeakerMode : DWORD;
          dwPreferredModemOptions : DWORD;
          dwNegotiatedModemOptions : DWORD;
          dwNegotiatedDCERate : DWORD;
          abVariablePortion : array[0..0] of BYTE;
       end;
     LPMODEMSETTINGS = ^MODEMSETTINGS;
     TMODEMSETTINGS = MODEMSETTINGS;
     PMODEMSETTINGS = ^MODEMSETTINGS;

     modemsettings_tag = MODEMSETTINGS;

     MONCBSTRUCT = record
          cb : UINT;
          dwTime : DWORD;
          hTask : HANDLE;
          dwRet : DWORD;
          wType : UINT;
          wFmt : UINT;
          hConv : HCONV;
          hsz1 : HSZ;
          hsz2 : HSZ;
          hData : HDDEDATA;
          dwData1 : ULONG_PTR;
          dwData2 : ULONG_PTR;
          cc : CONVCONTEXT;
          cbData : DWORD;
          Data : array[0..7] of DWORD;
       end;
     tagMONCBSTRUCT = MONCBSTRUCT;
     TMONCBSTRUCT = MONCBSTRUCT;
     PMONCBSTRUCT = ^MONCBSTRUCT;

     MONCONVSTRUCT = record
          cb : UINT;
          fConnect : WINBOOL;
          dwTime : DWORD;
          hTask : HANDLE;
          hszSvc : HSZ;
          hszTopic : HSZ;
          hConvClient : HCONV;
          hConvServer : HCONV;
       end;
     tagMONCONVSTRUCT = MONCONVSTRUCT;
     TMONCONVSTRUCT = MONCONVSTRUCT;
     PMONCONVSTRUCT = ^MONCONVSTRUCT;

     MONERRSTRUCT = record
          cb : UINT;
          wLastError : UINT;
          dwTime : DWORD;
          hTask : HANDLE;
       end;
     tagMONERRSTRUCT = MONERRSTRUCT;
     TMONERRSTRUCT = MONERRSTRUCT;
     PMONERRSTRUCT = ^MONERRSTRUCT;

     MONHSZSTRUCTA = record
          cb : UINT;
          fsAction : WINBOOL;
          dwTime : DWORD;
          hsz : HSZ;
          hTask : HANDLE;
          str : array[0..0] of AnsiCHAR;
       end;
     tagMONHSZSTRUCTA = MONHSZSTRUCTA;
     TMONHSZSTRUCTA   = MONHSZSTRUCTA;
     PMONHSZSTRUCTA   = ^MONHSZSTRUCTA;

     MONHSZSTRUCTW = record
          cb : UINT;
          fsAction : WINBOOL;
          dwTime : DWORD;
          hsz : HSZ;
          hTask : HANDLE;
          str : array[0..0] of WCHAR;
       end;
     tagMONHSZSTRUCTW = MONHSZSTRUCTW;
     TMONHSZSTRUCTW   = MONHSZSTRUCTW;
     PMONHSZSTRUCTW   = ^MONHSZSTRUCTW;
     {$ifdef unicode}
       tagMONHSZSTRUCT  = MONHSZSTRUCTW;
       TMONHSZSTRUCT    = MONHSZSTRUCTW;
       PMONHSZSTRUCT    = PMONHSZSTRUCTW;
       MONHSZSTRUCT     = MONHSZSTRUCTW;
     {$else}
       tagMONHSZSTRUCT  = MONHSZSTRUCTA;
       TMONHSZSTRUCT    = MONHSZSTRUCTA;
       PMONHSZSTRUCT    = PMONHSZSTRUCTA;
       MONHSZSTRUCT     = MONHSZSTRUCTA;
     {$endif}

     MONITOR_INFO_1 = record
          pName : LPTSTR;
       end;
     _MONITOR_INFO_1 = MONITOR_INFO_1;
     TMONITORINFO1 = MONITOR_INFO_1;
     PMONITORINFO1 = ^MONITOR_INFO_1;

     MONITOR_INFO_2 = record
          pName : LPTSTR;
          pEnvironment : LPTSTR;
          pDLLName : LPTSTR;
       end;
     _MONITOR_INFO_2 = MONITOR_INFO_2;
     TMONITORINFO2 = MONITOR_INFO_2;
     PMONITORINFO2 = ^MONITOR_INFO_2;

     MONLINKSTRUCT = record
          cb : UINT;
          dwTime : DWORD;
          hTask : HANDLE;
          fEstablished : WINBOOL;
          fNoData : WINBOOL;
          hszSvc : HSZ;
          hszTopic : HSZ;
          hszItem : HSZ;
          wFmt : UINT;
          fServer : WINBOOL;
          hConvServer : HCONV;
          hConvClient : HCONV;
       end;
     tagMONLINKSTRUCT = MONLINKSTRUCT;
     TMONLINKSTRUCT = MONLINKSTRUCT;
     PMONLINKSTRUCT = ^MONLINKSTRUCT;

     MONMSGSTRUCT = record
          cb : UINT;
          hwndTo : HWND;
          dwTime : DWORD;
          hTask : HANDLE;
          wMsg : UINT;
          wParam : WPARAM;
          lParam : LPARAM;
          dmhd : DDEML_MSG_HOOK_DATA;
       end;
     tagMONMSGSTRUCT = MONMSGSTRUCT;
     TMONMSGSTRUCT = MONMSGSTRUCT;
     PMONMSGSTRUCT = ^MONMSGSTRUCT;

     MOUSEHOOKSTRUCT = record
          pt : POINT;
          hwnd : HWND;
          wHitTestCode : UINT;
          dwExtraInfo : ULONG_PTR;
       end;
     LPMOUSEHOOKSTRUCT = ^MOUSEHOOKSTRUCT;
     tagMOUSEHOOKSTRUCT = MOUSEHOOKSTRUCT;
     TMOUSEHOOKSTRUCT = MOUSEHOOKSTRUCT;
     PMOUSEHOOKSTRUCT = ^MOUSEHOOKSTRUCT;

     MOUSEKEYS = record
          cbSize : DWORD;
          dwFlags : DWORD;
          iMaxSpeed : DWORD;
          iTimeToMaxSpeed : DWORD;
          iCtrlSpeed : DWORD;
          dwReserved1 : DWORD;
          dwReserved2 : DWORD;
       end;
     TMOUSEKEYS = MOUSEKEYS;
     PMOUSEKEYS = ^MOUSEKEYS;

     MSGBOXCALLBACK = procedure (lpHelpInfo:LPHELPINFO);stdcall;
     TMSGBOXCALLBACK = MSGBOXCALLBACK;

     MSGBOXPARAMSA = record
          cbSize : UINT;
          hwndOwner : HWND;
          hInstance : HINST;
          lpszText : LPCSTR;
          lpszCaption : LPCSTR;
          dwStyle : DWORD;
          lpszIcon : LPCSTR;
          dwContextHelpId : DWORD_PTR;
          lpfnMsgBoxCallback : MSGBOXCALLBACK;
          dwLanguageId : DWORD;
       end;
     LPMSGBOXPARAMSA = ^MSGBOXPARAMSA;
     TMSGBOXPARAMSA  =  MSGBOXPARAMSA;
     PMSGBOXPARAMSA  = ^MSGBOXPARAMSA;

     MSGBOXPARAMSW = record
          cbSize : UINT;
          hwndOwner : HWND;
          hInstance : HINST;
          lpszText : LPCWSTR;
          lpszCaption : LPCWSTR;
          dwStyle : DWORD;
          lpszIcon : LPCWSTR;
          dwContextHelpId : DWORD_PTR;
          lpfnMsgBoxCallback : MSGBOXCALLBACK;
          dwLanguageId : DWORD;
     end;
     LPMSGBOXPARAMSW = ^MSGBOXPARAMSW;
     TMSGBOXPARAMSW  =  MSGBOXPARAMSW;
     PMSGBOXPARAMSW  = ^MSGBOXPARAMSW;

     {$ifdef Unicode}
     LPMSGBOXPARAMS  = ^MSGBOXPARAMSW;
     TMSGBOXPARAMS   =  MSGBOXPARAMSW;
     PMSGBOXPARAMS   = ^MSGBOXPARAMSW;
     MSGBOXPARAMS    =  MSGBOXPARAMSW;
     {$else}
     LPMSGBOXPARAMS  = ^MSGBOXPARAMSA;
     TMSGBOXPARAMS   =  MSGBOXPARAMSA;
     PMSGBOXPARAMS   = ^MSGBOXPARAMSA;
     MSGBOXPARAMS    =  MSGBOXPARAMSA;
     {$endif}

     MSGFILTER = record
          nmhdr : NMHDR;
          msg : UINT;
          wParam : WPARAM;
          lParam : LPARAM;
       end;
     _msgfilter = MSGFILTER;
     Tmsgfilter = MSGFILTER;
     Pmsgfilter = ^MSGFILTER;

     MULTIKEYHELPA = record
          mkSize : DWORD;
          mkKeylist : AnsiCHAR;
          szKeyphrase : array[0..0] of AnsiCHAR;
       end;
     tagMULTIKEYHELPA = MULTIKEYHELPA;
     TMULTIKEYHELPA   = MULTIKEYHELPA;
     PMULTIKEYHELPA   = ^MULTIKEYHELPA;

     MULTIKEYHELPW = record
          mkSize : DWORD;
          mkKeylist : WideCHAR;
          szKeyphrase : array[0..0] of WideCHAR;
       end;
     tagMULTIKEYHELPW = MULTIKEYHELPW;
     TMULTIKEYHELPW   = MULTIKEYHELPW;
     PMULTIKEYHELPW   = ^MULTIKEYHELPW;
     {$ifdef Unicode}
        tagMULTIKEYHELP = MULTIKEYHELPW;
        TMULTIKEYHELP   = MULTIKEYHELPW;
        PMULTIKEYHELP   = PMULTIKEYHELPW;
        MULTIKEYHELP    = MULTIKEYHELPW;
     {$else}
        tagMULTIKEYHELP = MULTIKEYHELPA;
        TMULTIKEYHELP   = MULTIKEYHELPA;
        PMULTIKEYHELP   = PMULTIKEYHELPA;
        MULTIKEYHELP    = MULTIKEYHELPA;
     {$endif}

     NAME_BUFFER = record
          name : array[0..(NCBNAMSZ)-1] of UCHAR;
          name_num : UCHAR;
          name_flags : UCHAR;
       end;
     _NAME_BUFFER = NAME_BUFFER;
     TNAMEBUFFER = NAME_BUFFER;
     PNAMEBUFFER = ^NAME_BUFFER;

     p_NCB = ^_NCB;
     NCB = record
          ncb_command : UCHAR;
          ncb_retcode : UCHAR;
          ncb_lsn : UCHAR;
          ncb_num : UCHAR;
          ncb_buffer : PUCHAR;
          ncb_length : WORD;
          ncb_callname : array[0..(NCBNAMSZ)-1] of UCHAR;
          ncb_name : array[0..(NCBNAMSZ)-1] of UCHAR;
          ncb_rto : UCHAR;
          ncb_sto : UCHAR;
          ncb_post : procedure (_para1:p_NCB);CDECL;
          ncb_lana_num : UCHAR;
          ncb_cmd_cplt : UCHAR;
          ncb_reserve : array[0..9] of UCHAR;
          ncb_event : HANDLE;
       end;
     _NCB = NCB;
     TNCB = NCB;
     PNCB = ^NCB;

     NCCALCSIZE_PARAMS = record
          rgrc : array[0..2] of RECT;
          lppos : PWINDOWPOS;
       end;
     _NCCALCSIZE_PARAMS = NCCALCSIZE_PARAMS;
     TNCCALCSIZEPARAMS = NCCALCSIZE_PARAMS;
     PNCCALCSIZEPARAMS = ^NCCALCSIZE_PARAMS;

     NDDESHAREINFO = record
          lRevision : LONG;
          lpszShareName : LPTSTR;
          lShareType : LONG;
          lpszAppTopicList : LPTSTR;
          fSharedFlag : LONG;
          fService : LONG;
          fStartAppFlag : LONG;
          nCmdShow : LONG;
          qModifyId : array[0..1] of LONG;
          cNumItems : LONG;
          lpszItemList : LPTSTR;
       end;
     _NDDESHAREINFO = NDDESHAREINFO;
     TNDDESHAREINFO = NDDESHAREINFO;
     PNDDESHAREINFO = ^NDDESHAREINFO;

     NETRESOURCEA = record
          dwScope : DWORD;
          dwType : DWORD;
          dwDisplayType : DWORD;
          dwUsage : DWORD;
          lpLocalName : LPSTR;
          lpRemoteName : LPSTR;
          lpComment : LPSTR;
          lpProvider : LPSTR;
       end;
     LPNETRESOURCEA = ^NETRESOURCEA;
     _NETRESOURCEA  = NETRESOURCEA;
     TNETRESOURCEA  = NETRESOURCEA;
     PNETRESOURCEA  = ^NETRESOURCEA;

     NETRESOURCEW = record
          dwScope : DWORD;
          dwType : DWORD;
          dwDisplayType : DWORD;
          dwUsage : DWORD;
          lpLocalName : LPWSTR;
          lpRemoteName : LPWSTR;
          lpComment : LPWSTR;
          lpProvider : LPWSTR;
       end;
     LPNETRESOURCEW = ^NETRESOURCEW;
     _NETRESOURCEW = NETRESOURCEW;
     TNETRESOURCEW = NETRESOURCEW;
     PNETRESOURCEW = ^NETRESOURCEW;

     {$ifdef Unicode}
     LPNETRESOURCE = ^NETRESOURCEW;
     _NETRESOURCE  = NETRESOURCEW;
     TNETRESOURCE  = NETRESOURCEW;
     NETRESOURCE   = NETRESOURCEW;
     PNETRESOURCE  = ^NETRESOURCEW;
     {$else}
     LPNETRESOURCE = ^NETRESOURCEA;
     _NETRESOURCE  = NETRESOURCEA;
     TNETRESOURCE  = NETRESOURCEA;
     NETRESOURCE   = NETRESOURCEA;
     PNETRESOURCE  = ^NETRESOURCEA;
     {$endif}

     NEWCPLINFOA = record
          dwSize : DWORD;
          dwFlags : DWORD;
          dwHelpContext : DWORD;
          lData : LONG_PTR;
          hIcon : HICON;
          szName : array[0..31] of AnsiCHAR;
          szInfo : array[0..63] of AnsiCHAR;
          szHelpFile : array[0..127] of AnsiCHAR;
       end;
     tagNEWCPLINFOA = NEWCPLINFOA;
     TNEWCPLINFOA   = NEWCPLINFOA;
     PNEWCPLINFOA   = ^NEWCPLINFOA;

     NEWCPLINFOW = record
          dwSize : DWORD;
          dwFlags : DWORD;
          dwHelpContext : DWORD;
          lData : LONG;
          hIcon : HICON;
          szName : array[0..31] of WideCHAR;
          szInfo : array[0..63] of WideCHAR;
          szHelpFile : array[0..127] of WideCHAR;
       end;
     tagNEWCPLINFOW = NEWCPLINFOW;
     TNEWCPLINFOW   = NEWCPLINFOW;
     PNEWCPLINFOW   = ^NEWCPLINFOW;

     {$ifdef Unicode}
       tagNEWCPLINFO = NEWCPLINFOW;
       TNEWCPLINFO   = NEWCPLINFOW;
       PNEWCPLINFO   = PNEWCPLINFOW;
       NEWCPLINFO    = NEWCPLINFOW;
     {$else}
       tagNEWCPLINFO = NEWCPLINFOA;
       TNEWCPLINFO   = NEWCPLINFOA;
       PNEWCPLINFO   = PNEWCPLINFOA;
       NEWCPLINFO    = NEWCPLINFOA;
     {$endif}
     NEWTEXTMETRICA = record
          tmHeight : LONG;
          tmAscent : LONG;
          tmDescent : LONG;
          tmInternalLeading : LONG;
          tmExternalLeading : LONG;
          tmAveCharWidth : LONG;
          tmMaxCharWidth : LONG;
          tmWeight : LONG;
          tmOverhang : LONG;
          tmDigitizedAspectX : LONG;
          tmDigitizedAspectY : LONG;
          tmFirstChar : AnsiChar;
          tmLastChar : AnsiChar;
          tmDefaultChar : AnsiChar;
          tmBreakChar : AnsiChar;
          tmItalic : BYTE;
          tmUnderlined : BYTE;
          tmStruckOut : BYTE;
          tmPitchAndFamily : BYTE;
          tmCharSet : BYTE;
          ntmFlags : DWORD;
          ntmSizeEM : UINT;
          ntmCellHeight : UINT;
          ntmAvgWidth : UINT;
       end;
     LPNEWTEXTMETRICA  = ^NEWTEXTMETRICA;
     tagNEWTEXTMETRICA = NEWTEXTMETRICA;
     _NEWTEXTMETRICA   = NEWTEXTMETRICA;
     TNEWTEXTMETRICA   = NEWTEXTMETRICA;
     PNEWTEXTMETRICA   = ^NEWTEXTMETRICA;
   
     NEWTEXTMETRICW    = record
       tmHeight           :LONG;
       tmAscent           :LONG;
       tmDescent          :LONG;
       tmInternalLeading  :LONG;
       tmExternalLeading  :LONG;
       tmAveCharWidth     :LONG;
       tmMaxCharWidth     :LONG;
       tmWeight           :LONG;
       tmOverhang         :LONG;
       tmDigitizedAspectX :LONG;
       tmDigitizedAspectY :LONG;
       tmFirstChar        :WCHAR;
       tmLastChar         :WCHAR;
       tmDefaultChar      :WCHAR;
       tmBreakChar        :WCHAR;
       tmItalic           :Byte;
       tmUnderlined       :Byte;
       tmStruckOut        :Byte;
       tmPitchAndFamily   :Byte;
       tmCharSet          :Byte;
       ntmFlags           :DWORD;
       ntmSizeEM          :UINT;
       ntmCellHeight      :UINT;
       ntmAvgWidth        :UINT;
     end;
     LPNEWTEXTMETRICW  = ^NEWTEXTMETRICW;
     tagNEWTEXTMETRICW = NEWTEXTMETRICW;
     _NEWTEXTMETRICW   = NEWTEXTMETRICW;
     TNEWTEXTMETRICW   = NEWTEXTMETRICW;
     PNEWTEXTMETRICW   = ^NEWTEXTMETRICW;
   
    {$IFNDEF UNICODE}
     LPNEWTEXTMETRIC   = ^NEWTEXTMETRICA;
     _NEWTEXTMETRIC    = NEWTEXTMETRICA;
     tagNEWTEXTMETRIC  = NEWTEXTMETRICA;
     TNEWTEXTMETRIC    = NEWTEXTMETRICA;
     PNEWTEXTMETRIC    = ^NEWTEXTMETRICA;
     NEWTEXTMETRIC     = NEWTEXTMETRICA;
    {$ELSE}
     LPNEWTEXTMETRIC   = ^NEWTEXTMETRICW;
     _NEWTEXTMETRIC    = NEWTEXTMETRICW;
     tagNEWTEXTMETRIC  = NEWTEXTMETRICW;
     TNEWTEXTMETRIC    = NEWTEXTMETRICW;
     PNEWTEXTMETRIC    = ^NEWTEXTMETRICW;
     NEWTEXTMETRIC     = NEWTEXTMETRICW;
    {$ENDIF}

     NEWTEXTMETRICEXA = record
          ntmentm : NEWTEXTMETRICA;
          ntmeFontSignature : FONTSIGNATURE;
       end;
     LPNEWTEXTMETRICEXA  = ^NEWTEXTMETRICEXA;
     tagNEWTEXTMETRICEXA = NEWTEXTMETRICEXA;
     TNEWTEXTMETRICEXA   = NEWTEXTMETRICEXA;
     PNEWTEXTMETRICEXA   = ^NEWTEXTMETRICEXA;
   
     NEWTEXTMETRICEXW    = record
       ntmentm           :NEWTEXTMETRICW;
       ntmeFontSignature :FONTSIGNATURE;
    end;
     LPNEWTEXTMETRICEXW  = ^NEWTEXTMETRICEXW;
     tagNEWTEXTMETRICEXW = NEWTEXTMETRICEXW;
     TNEWTEXTMETRICEXW   = NEWTEXTMETRICEXW;
     PNEWTEXTMETRICEXW   = ^NEWTEXTMETRICEXW;
   
    {$IFNDEF UNICODE}
     LPNEWTEXTMETRICEX   = ^NEWTEXTMETRICEXA;
     _NEWTEXTMETRICEX    = NEWTEXTMETRICEXA;
     tagNEWTEXTMETRICEX  = NEWTEXTMETRICEXA;
     TNEWTEXTMETRICEX    = NEWTEXTMETRICEXA;
     PNEWTEXTMETRICEX    = ^NEWTEXTMETRICEXA;
     NEWTEXTMETRICEX     = NEWTEXTMETRICEXA;
    {$ELSE}
     LPNEWTEXTMETRICEX   = ^NEWTEXTMETRICEXW;
     _NEWTEXTMETRICEX    = NEWTEXTMETRICEXW;
     tagNEWTEXTMETRICEX  = NEWTEXTMETRICEXW;
     TNEWTEXTMETRICEX    = NEWTEXTMETRICEXW;
     PNEWTEXTMETRICEX    = ^NEWTEXTMETRICEXW;
     NEWTEXTMETRICEX     = NEWTEXTMETRICEXW;
    {$ENDIF}
   
     NM_LISTVIEW = record
          hdr : NMHDR;
          iItem : longint;
          iSubItem : longint;
          uNewState : UINT;
          uOldState : UINT;
          uChanged : UINT;
          ptAction : POINT;
          lParam : LPARAM;
       end;
     tagNM_LISTVIEW = NM_LISTVIEW;
     TNMLISTVIEW = NM_LISTVIEW;
     PNMLISTVIEW = ^NM_LISTVIEW;

     TV_ITEM = record
          mask : UINT;
          hItem : HTREEITEM;
          state : UINT;
          stateMask : UINT;
          pszText : LPTSTR;
          cchTextMax : longint;
          iImage : longint;
          iSelectedImage : longint;
          cChildren : longint;
          lParam : LPARAM;
       end;
     LPTV_ITEM = ^TV_ITEM;
     _TV_ITEM = TV_ITEM;
     TTVITEM = TV_ITEM;
     PTVITEM = ^TV_ITEM;

     NM_TREEVIEW = record
          hdr : NMHDR;
          action : UINT;
          itemOld : TV_ITEM;
          itemNew : TV_ITEM;
          ptDrag : POINT;
       end;
     LPNM_TREEVIEW = ^NM_TREEVIEW;
     _NM_TREEVIEW = NM_TREEVIEW;
     TNMTREEVIEW = NM_TREEVIEW;
     PNMTREEVIEW = ^NM_TREEVIEW;

     NM_UPDOWNW = record
          hdr : NMHDR;
          iPos : longint;
          iDelta : longint;
       end;
     _NM_UPDOWN = NM_UPDOWNW;
     TNMUPDOWN = NM_UPDOWNW;
     PNMUPDOWN = ^NM_UPDOWNW;

     NONCLIENTMETRICS = record
          cbSize : UINT;
          iBorderWidth : longint;
          iScrollWidth : longint;
          iScrollHeight : longint;
          iCaptionWidth : longint;
          iCaptionHeight : longint;
          lfCaptionFont : LOGFONT;
          iSmCaptionWidth : longint;
          iSmCaptionHeight : longint;
          lfSmCaptionFont : LOGFONT;
          iMenuWidth : longint;
          iMenuHeight : longint;
          lfMenuFont : LOGFONT;
          lfStatusFont : LOGFONT;
          lfMessageFont : LOGFONT;
          //iPaddedBorderWidth : longint;     // WINVER >= 0x0600
       end;
     LPNONCLIENTMETRICS = ^NONCLIENTMETRICS;
     tagNONCLIENTMETRICS = NONCLIENTMETRICS;
     TNONCLIENTMETRICS = NONCLIENTMETRICS;
     PNONCLIENTMETRICS = ^NONCLIENTMETRICS;

     SERVICE_ADDRESS = record
          dwAddressType : DWORD;
          dwAddressFlags : DWORD;
          dwAddressLength : DWORD;
          dwPrincipalLength : DWORD;
          lpAddress : ^BYTE;
          lpPrincipal : ^BYTE;
       end;
     _SERVICE_ADDRESS = SERVICE_ADDRESS;
     TSERVICEADDRESS = SERVICE_ADDRESS;
     PSERVICEADDRESS = ^SERVICE_ADDRESS;

     SERVICE_ADDRESSES = record
          dwAddressCount : DWORD;
          Addresses : array[0..0] of SERVICE_ADDRESS;
       end;
     LPSERVICE_ADDRESSES = ^SERVICE_ADDRESSES;
     _SERVICE_ADDRESSES = SERVICE_ADDRESSES;
     TSERVICEADDRESSES = SERVICE_ADDRESSES;
     PSERVICEADDRESSES = ^SERVICE_ADDRESSES;

     GUID = system.tguid;
     LPGUID = ^GUID;
     _GUID = GUID;
     TGUID = GUID;
     PGUID = ^GUID;

     CLSID = GUID;
     LPCLSID = ^CLSID;
     TCLSID = CLSID;
     PCLSID = ^CLSID;

     SERVICE_INFO = record
          lpServiceType : LPGUID;
          lpServiceName : LPTSTR;
          lpComment : LPTSTR;
          lpLocale : LPTSTR;
          dwDisplayHint : DWORD;
          dwVersion : DWORD;
          dwTime : DWORD;
          lpMachineName : LPTSTR;
          lpServiceAddress : LPSERVICE_ADDRESSES;
          ServiceSpecificInfo : BLOB;
       end;
     _SERVICE_INFO = SERVICE_INFO;
     TSERVICEINFO = SERVICE_INFO;
     PSERVICEINFO = ^SERVICE_INFO;

     NS_SERVICE_INFO = record
          dwNameSpace : DWORD;
          ServiceInfo : SERVICE_INFO;
       end;
     _NS_SERVICE_INFO = NS_SERVICE_INFO;
     TNSSERVICEINFO = NS_SERVICE_INFO;
     PNSSERVICEINFO = ^NS_SERVICE_INFO;

     NUMBERFMTA = record
          NumDigits : UINT;
          LeadingZero : UINT;
          Grouping : UINT;
          lpDecimalSep : LPSTR;
          lpThousandSep : LPSTR;
          NegativeOrder : UINT;
       end;
     _numberfmtA = NUMBERFMTA;
     TnumberfmtA = NUMBERFMTA;
     PnumberfmtA = ^NUMBERFMTA;

     NUMBERFMTW = record
          NumDigits : UINT;
          LeadingZero : UINT;
          Grouping : UINT;
          lpDecimalSep : LPWSTR;
          lpThousandSep : LPWSTR;
          NegativeOrder : UINT;
       end;
     _numberfmtW = NUMBERFMTW;
     TnumberfmtW = NUMBERFMTW;
     PnumberfmtW = ^NUMBERFMTW;

     {$ifdef Unicode}
     _NUMBERFMT = NUMBERFMTW;
     TNUMBERFMT = NUMBERFMTW;
     NUMBERFMT  = NUMBERFMTW;
     PNUMBERFMT = ^NUMBERFMTW;
     {$else}
     _NUMBERFMT = NUMBERFMTA;
     TNUMBERFMT = NUMBERFMTA;
     NUMBERFMT  = NUMBERFMTA;
     PNUMBERFMT = ^NUMBERFMTA;
     {$endif}

     OFSTRUCT = record
          cBytes : BYTE;
          fFixedDisk : BYTE;
          nErrCode : WORD;
          Reserved1 : WORD;
          Reserved2 : WORD;
          szPathName : array[0..(OFS_MAXPATHNAME)-1] of CHAR;
       end;
     LPOFSTRUCT = ^OFSTRUCT;
     _OFSTRUCT = OFSTRUCT;
     TOFSTRUCT = OFSTRUCT;
     POFSTRUCT = ^OFSTRUCT;

     OPENFILENAME_NT4 = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HINST;
          lpstrFilter : LPCTSTR;
          lpstrCustomFilter : LPTSTR;
          nMaxCustFilter : DWORD;
          nFilterIndex : DWORD;
          lpstrFile : LPTSTR;
          nMaxFile : DWORD;
          lpstrFileTitle : LPTSTR;
          nMaxFileTitle : DWORD;
          lpstrInitialDir : LPCTSTR;
          lpstrTitle : LPCTSTR;
          Flags : DWORD;
          nFileOffset : WORD;
          nFileExtension : WORD;
          lpstrDefExt : LPCTSTR;
          lCustData : LPARAM;
          lpfnHook : LPOFNHOOKPROC;
          lpTemplateName : LPCTSTR;
       end;
     LPOPENFILENAME_NT4 = ^OPENFILENAME_NT4;
     TOPENFILENAME_NT4 = OPENFILENAME_NT4;
     POPENFILENAME_NT4 = ^OPENFILENAME_NT4;

     OPENFILENAME = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HINST;
          lpstrFilter : LPCTSTR;
          lpstrCustomFilter : LPTSTR;
          nMaxCustFilter : DWORD;
          nFilterIndex : DWORD;
          lpstrFile : LPTSTR;
          nMaxFile : DWORD;
          lpstrFileTitle : LPTSTR;
          nMaxFileTitle : DWORD;
          lpstrInitialDir : LPCTSTR;
          lpstrTitle : LPCTSTR;
          Flags : DWORD;
          nFileOffset : WORD;
          nFileExtension : WORD;
          lpstrDefExt : LPCTSTR;
          lCustData : LPARAM;
          lpfnHook : LPOFNHOOKPROC;
          lpTemplateName : LPCTSTR;
          pvReserved : pointer;
          dwreserved : dword;
          FlagsEx    : dword;
       end;
     LPOPENFILENAME = ^OPENFILENAME;
     TOPENFILENAME = OPENFILENAME;
     POPENFILENAME = ^OPENFILENAME;

     tagOFN = OPENFILENAME;
     TOFN = OPENFILENAME;
     POFN = ^OPENFILENAME;

     OFNOTIFY = record
          hdr : NMHDR;
          lpOFN : LPOPENFILENAME;
          pszFile : LPTSTR;
       end;
     LPOFNOTIFY = ^OFNOTIFY;
     _OFNOTIFY = OFNOTIFY;
     TOFNOTIFY = OFNOTIFY;
     POFNOTIFY = ^OFNOTIFY;

     OSVERSIONINFOA = record
          dwOSVersionInfoSize : DWORD;
          dwMajorVersion : DWORD;
          dwMinorVersion : DWORD;
          dwBuildNumber : DWORD;
          dwPlatformId : DWORD;
          szCSDVersion : array[0..127] of AnsiCHAR;
       end;
     LPOSVERSIONINFOA= ^OSVERSIONINFOA;
     _OSVERSIONINFOA = OSVERSIONINFOA;
     TOSVERSIONINFOA = OSVERSIONINFOA;
     POSVERSIONINFOA = LPOSVERSIONINFOA;

     OSVERSIONINFOW = record
       dwOSVersionInfoSize : DWORD;
       dwMajorVersion : DWORD;
       dwMinorVersion : DWORD;
       dwBuildNumber : DWORD;
       dwPlatformId : DWORD;
       szCSDVersion : array[0..127] of WCHAR;
     end;
     LPOSVERSIONINFOW = ^OSVERSIONINFOW;
     _OSVERSIONINFOW = OSVERSIONINFOW;
     TOSVERSIONINFOW = OSVERSIONINFOW;
     POSVERSIONINFOW = LPOSVERSIONINFOW;

     {$IFNDEF UNICODE}
       LPOSVERSIONINFO  = ^OSVERSIONINFOA;
       _OSVERSIONINFO   = OSVERSIONINFOA;
       TOSVERSIONINFO   = OSVERSIONINFOA;
       POSVERSIONINFO   = ^OSVERSIONINFOA;
       OSVERSIONINFO    = OSVERSIONINFOA;
     {$ELSE}
       LPOSVERSIONINFO  = ^OSVERSIONINFOW;
       _OSVERSIONINFO   = OSVERSIONINFOW;
       TOSVERSIONINFO   = OSVERSIONINFOW;
       POSVERSIONINFO   = ^OSVERSIONINFOW;
       OSVERSIONINFO    = OSVERSIONINFOW;
     {$ENDIF}

     {$push}
     {$packrecords 4}

     TEXTMETRICA = record
          tmHeight : LONG;
          tmAscent : LONG;
          tmDescent : LONG;
          tmInternalLeading : LONG;
          tmExternalLeading : LONG;
          tmAveCharWidth : LONG;
          tmMaxCharWidth : LONG;
          tmWeight : LONG;
          tmOverhang : LONG;
          tmDigitizedAspectX : LONG;
          tmDigitizedAspectY : LONG;
          tmFirstChar : AnsiChar;
          tmLastChar : AnsiChar;
          tmDefaultChar : AnsiChar;
          tmBreakChar : AnsiChar;
          tmItalic : BYTE;
          tmUnderlined : BYTE;
          tmStruckOut : BYTE;
          tmPitchAndFamily : BYTE;
          tmCharSet : BYTE;
       end;
     LPTEXTMETRICA  = ^TEXTMETRICA;
     tagTEXTMETRICA = TEXTMETRICA;
     _TEXTMETRICA   = TEXTMETRICA;
     TTEXTMETRICA   = TEXTMETRICA;
     PTEXTMETRICA   = LPTEXTMETRICA;

     TEXTMETRICW = record
          tmHeight : LONG;
          tmAscent : LONG;
          tmDescent : LONG;
          tmInternalLeading : LONG;
          tmExternalLeading : LONG;
          tmAveCharWidth : LONG;
          tmMaxCharWidth : LONG;
          tmWeight : LONG;
          tmOverhang : LONG;
          tmDigitizedAspectX : LONG;
          tmDigitizedAspectY : LONG;
          tmFirstChar : WCHAR;
          tmLastChar : WCHAR;
          tmDefaultChar : WCHAR;
          tmBreakChar : WCHAR;
          tmItalic : BYTE;
          tmUnderlined : BYTE;
          tmStruckOut : BYTE;
          tmPitchAndFamily : BYTE;
          tmCharSet : BYTE;
       end;
     LPTEXTMETRICW = ^TEXTMETRICW;
     tagTEXTMETRICW = TEXTMETRICW;
     _TEXTMETRICW   = TEXTMETRICW;
     TTEXTMETRICW = TEXTMETRICW;
     PTEXTMETRICW = ^TEXTMETRICW;

     {$IFNDEF UNICODE}
       LPTEXTMETRIC       = ^TEXTMETRICA;
       _TEXTMETRIC        = TEXTMETRICA;
       tagTEXTMETRIC      = TEXTMETRICA;
       TTEXTMETRIC        = TEXTMETRICA;
       PTEXTMETRIC        = ^TEXTMETRICA;
       TEXTMETRIC         = TEXTMETRICA;
     {$ELSE}
       LPTEXTMETRIC       = ^TEXTMETRICW;
       _TEXTMETRIC        = TEXTMETRICW;
       tagTEXTMETRIC      = TEXTMETRICW;
       TTEXTMETRIC        = TEXTMETRICW;
       PTEXTMETRIC        = ^TEXTMETRICW;
       TEXTMETRIC         = TEXTMETRICW;
     {$ENDIF}
     {$PACKRECORDS C}

     OUTLINETEXTMETRICA = record
          otmSize : UINT;
          otmTextMetrics : TEXTMETRICA;
          otmFiller : BYTE;
          otmPanoseNumber : PANOSE;
          otmfsSelection : UINT;
          otmfsType : UINT;
          otmsCharSlopeRise : longint;
          otmsCharSlopeRun : longint;
          otmItalicAngle : longint;
          otmEMSquare : UINT;
          otmAscent : longint;
          otmDescent : longint;
          otmLineGap : UINT;
          otmsCapEmHeight : UINT;
          otmsXHeight : UINT;
          otmrcFontBox : RECT;
          otmMacAscent : longint;
          otmMacDescent : longint;
          otmMacLineGap : UINT;
          otmusMinimumPPEM : UINT;
          otmptSubscriptSize : POINT;
          otmptSubscriptOffset : POINT;
          otmptSuperscriptSize : POINT;
          otmptSuperscriptOffset : POINT;
          otmsStrikeoutSize : UINT;
          otmsStrikeoutPosition : longint;
          otmsUnderscoreSize : longint;
          otmsUnderscorePosition : longint;
          otmpFamilyName : PSTR;
          otmpFaceName : PSTR;
          otmpStyleName : PSTR;
          otmpFullName : PSTR;
       end;
     LPOUTLINETEXTMETRICA = ^OUTLINETEXTMETRICA;
     _OUTLINETEXTMETRICA = OUTLINETEXTMETRICA;
     TOUTLINETEXTMETRICA = OUTLINETEXTMETRICA;
     POUTLINETEXTMETRICA = ^OUTLINETEXTMETRICA;

     OUTLINETEXTMETRICW = record
          otmSize : UINT;
          otmTextMetrics : TEXTMETRICW;
          otmFiller : BYTE;
          otmPanoseNumber : PANOSE;
          otmfsSelection : UINT;
          otmfsType : UINT;
          otmsCharSlopeRise : longint;
          otmsCharSlopeRun : longint;
          otmItalicAngle : longint;
          otmEMSquare : UINT;
          otmAscent : longint;
          otmDescent : longint;
          otmLineGap : UINT;
          otmsCapEmHeight : UINT;
          otmsXHeight : UINT;
          otmrcFontBox : RECT;
          otmMacAscent : longint;
          otmMacDescent : longint;
          otmMacLineGap : UINT;
          otmusMinimumPPEM : UINT;
          otmptSubscriptSize : POINT;
          otmptSubscriptOffset : POINT;
          otmptSuperscriptSize : POINT;
          otmptSuperscriptOffset : POINT;
          otmsStrikeoutSize : UINT;
          otmsStrikeoutPosition : longint;
          otmsUnderscoreSize : longint;
          otmsUnderscorePosition : longint;
          otmpFamilyName : PSTR;
          otmpFaceName : PSTR;
          otmpStyleName : PSTR;
          otmpFullName : PSTR;
       end;
     LPOUTLINETEXTMETRICW = ^OUTLINETEXTMETRICW;
     _OUTLINETEXTMETRICW = OUTLINETEXTMETRICW;
     TOUTLINETEXTMETRICW = OUTLINETEXTMETRICW;
     POUTLINETEXTMETRICW = LPOUTLINETEXTMETRICW;

     {$ifdef Unicode}
     LPOUTLINETEXTMETRIC = LPOUTLINETEXTMETRICW;
     _OUTLINETEXTMETRIC  = OUTLINETEXTMETRICW;
     TOUTLINETEXTMETRIC  = OUTLINETEXTMETRICW;
     POUTLINETEXTMETRIC  = LPOUTLINETEXTMETRICW;
     OUTLINETEXTMETRIC   = OUTLINETEXTMETRICW;
     {$else}
     LPOUTLINETEXTMETRIC = LPOUTLINETEXTMETRICA;
     _OUTLINETEXTMETRIC  = OUTLINETEXTMETRICA;
     TOUTLINETEXTMETRIC  = OUTLINETEXTMETRICA;
     POUTLINETEXTMETRIC  = LPOUTLINETEXTMETRICA;
     OUTLINETEXTMETRIC   = OUTLINETEXTMETRICA;
     {$endif}

     {$POP}
     OVERLAPPED = record
          Internal : ULONG_PTR;
          InternalHigh : ULONG_PTR;
          Offset : DWORD;
          OffsetHigh : DWORD;
          hEvent : HANDLE;
       end;
     LPOVERLAPPED = ^OVERLAPPED;
     _OVERLAPPED = OVERLAPPED;
     TOVERLAPPED = OVERLAPPED;
     POVERLAPPED = ^OVERLAPPED;

     {PAGESETUPDLG = record conflicts with function PageSetupDlg }
     TPAGESETUPDLG = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDevMode : HGLOBAL;
          hDevNames : HGLOBAL;
          Flags : DWORD;
          ptPaperSize : POINT;
          rtMinMargin : RECT;
          rtMargin : RECT;
          hInstance : HINST;
          lCustData : LPARAM;
          lpfnPageSetupHook : LPPAGESETUPHOOK;
          lpfnPagePaintHook : LPPAGEPAINTHOOK;
          lpPageSetupTemplateName : LPCTSTR;
          hPageSetupTemplate : HGLOBAL;
       end;
     LPPAGESETUPDLG = ^TPAGESETUPDLG;
     PPAGESETUPDLG = ^TPAGESETUPDLG;

     tagPSD = TPAGESETUPDLG;
     TPSD = TPAGESETUPDLG;
     PPSD = ^TPAGESETUPDLG;

     PAINTSTRUCT = record
          hdc : HDC;
          fErase : WINBOOL;
          rcPaint : RECT;
          fRestore : WINBOOL;
          fIncUpdate : WINBOOL;
          rgbReserved : array[0..31] of BYTE;
       end;
     LPPAINTSTRUCT = ^PAINTSTRUCT;
     tagPAINTSTRUCT = PAINTSTRUCT;
     TPAINTSTRUCT = PAINTSTRUCT;
     PPAINTSTRUCT = ^PAINTSTRUCT;

     PARAFORMAT = record
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
     _paraformat = PARAFORMAT;
     Tparaformat = PARAFORMAT;
     Pparaformat = ^PARAFORMAT;

     PERF_COUNTER_BLOCK = record
          ByteLength : DWORD;
       end;
     _PERF_COUNTER_BLOCK = PERF_COUNTER_BLOCK;
     TPERFCOUNTERBLOCK = PERF_COUNTER_BLOCK;
     PPERFCOUNTERBLOCK = ^PERF_COUNTER_BLOCK;

     PERF_COUNTER_DEFINITION = record
          ByteLength : DWORD;
          CounterNameTitleIndex : DWORD;
          CounterNameTitle : LPWSTR;
          CounterHelpTitleIndex : DWORD;
          CounterHelpTitle : LPWSTR;
          DefaultScale : DWORD;
          DetailLevel : DWORD;
          CounterType : DWORD;
          CounterSize : DWORD;
          CounterOffset : DWORD;
       end;
     _PERF_COUNTER_DEFINITION = PERF_COUNTER_DEFINITION;
     TPERFCOUNTERDEFINITION = PERF_COUNTER_DEFINITION;
     PPERFCOUNTERDEFINITION = ^PERF_COUNTER_DEFINITION;

     PERF_DATA_BLOCK = record
          Signature : array[0..3] of WCHAR;
          LittleEndian : DWORD;
          Version : DWORD;
          Revision : DWORD;
          TotalByteLength : DWORD;
          HeaderLength : DWORD;
          NumObjectTypes : DWORD;
          DefaultObject : DWORD;
          SystemTime : SYSTEMTIME;
          PerfTime : LARGE_INTEGER;
          PerfFreq : LARGE_INTEGER;
          PerfTime100nSec : LARGE_INTEGER;
          SystemNameLength : DWORD;
          SystemNameOffset : DWORD;
       end;
     _PERF_DATA_BLOCK = PERF_DATA_BLOCK;
     TPERFDATABLOCK = PERF_DATA_BLOCK;
     PPERFDATABLOCK = ^PERF_DATA_BLOCK;

     PERF_INSTANCE_DEFINITION = record
          ByteLength : DWORD;
          ParentObjectTitleIndex : DWORD;
          ParentObjectInstance : DWORD;
          UniqueID : DWORD;
          NameOffset : DWORD;
          NameLength : DWORD;
       end;
     _PERF_INSTANCE_DEFINITION = PERF_INSTANCE_DEFINITION;
     TPERFINSTANCEDEFINITION = PERF_INSTANCE_DEFINITION;
     PPERFINSTANCEDEFINITION = PERF_INSTANCE_DEFINITION;

     PERF_OBJECT_TYPE = record
          TotalByteLength : DWORD;
          DefinitionLength : DWORD;
          HeaderLength : DWORD;
          ObjectNameTitleIndex : DWORD;
          ObjectNameTitle : LPWSTR;
          ObjectHelpTitleIndex : DWORD;
          ObjectHelpTitle : LPWSTR;
          DetailLevel : DWORD;
          NumCounters : DWORD;
          DefaultCounter : DWORD;
          NumInstances : DWORD;
          CodePage : DWORD;
          PerfTime : LARGE_INTEGER;
          PerfFreq : LARGE_INTEGER;
       end;
     _PERF_OBJECT_TYPE = PERF_OBJECT_TYPE;
     TPERFOBJECTTYPE = PERF_OBJECT_TYPE;
     PPERFOBJECTTYPE = ^PERF_OBJECT_TYPE;

     POLYTEXTA = record
          x : longint;
          y : longint;
          n : UINT;
          lpstr : LPCSTR;
          uiFlags : UINT;
          rcl : RECT;
          pdx : ^longint;
       end;
     _POLYTEXTA = POLYTEXTA;
     TPOLYTEXTA = POLYTEXTA;
     PPOLYTEXTA = ^POLYTEXTA;

     POLYTEXTW = record
          x : longint;
          y : longint;
          n : UINT;
          lpstr : LPCWSTR;
          uiFlags : UINT;
          rcl : RECT;
          pdx : ^longint;
       end;
     _POLYTEXTW = POLYTEXTW;
     TPOLYTEXTW = POLYTEXTW;
     PPOLYTEXTW = ^POLYTEXTW;

     {$ifdef unicode}
     _POLYTEXT = POLYTEXTW;
     TPOLYTEXT = POLYTEXTW;
     POLYTEXT  = POLYTEXTW;
     PPOLYTEXT = ^POLYTEXTW;
     {$else}
     _POLYTEXT = POLYTEXTA;
     TPOLYTEXT = POLYTEXTA;
     POLYTEXT  = POLYTEXTA;
     PPOLYTEXT = ^POLYTEXTA;
     {$endif}

     PORT_INFO_1 = record
          pName : LPTSTR;
       end;
     _PORT_INFO_1 = PORT_INFO_1;
     TPORTINFO1 = PORT_INFO_1;
     PPORTINFO1 = ^PORT_INFO_1;

     PORT_INFO_2 = record
          pPortName : LPSTR;
          pMonitorName : LPSTR;
          pDescription : LPSTR;
          fPortType : DWORD;
          Reserved : DWORD;
       end;
     _PORT_INFO_2 = PORT_INFO_2;
     TPORTINFO2 = PORT_INFO_2;
     PPORTINFO2 = ^PORT_INFO_2;

     PREVENT_MEDIA_REMOVAL = record
          PreventMediaRemoval : BOOLEAN;
       end;
     _PREVENT_MEDIA_REMOVAL = PREVENT_MEDIA_REMOVAL;
     TPREVENTMEDIAREMOVAL = PREVENT_MEDIA_REMOVAL;
     PPREVENTMEDIAREMOVAL = ^PREVENT_MEDIA_REMOVAL;

     {PRINTDLG = record conflicts with PrintDlg function }
     TPRINTDLG = {$ifndef win64}packed {$endif}record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDevMode : HGLOBAL;
          hDevNames : HGLOBAL;
          hDC : HDC;
          Flags : DWORD;
          nFromPage : WORD;
          nToPage : WORD;
          nMinPage : WORD;
          nMaxPage : WORD;
          nCopies : WORD;
          hInstance : HINST;
          lCustData : LPARAM;
          lpfnPrintHook : LPPRINTHOOKPROC;
          lpfnSetupHook : LPSETUPHOOKPROC;
          lpPrintTemplateName : LPCTSTR;
          lpSetupTemplateName : LPCTSTR;
          hPrintTemplate : HGLOBAL;
          hSetupTemplate : HGLOBAL;
       end;
     LPPRINTDLG = ^TPRINTDLG;
     PPRINTDLG = ^TPRINTDLG;

     tagPD = TPRINTDLG;
     TPD = TPRINTDLG;
     PPD = ^TPRINTDLG;

     PRINTER_DEFAULTS = record
          pDatatype : LPTSTR;
          pDevMode : LPDEVMODE;
          DesiredAccess : ACCESS_MASK;
       end;
     _PRINTER_DEFAULTS = PRINTER_DEFAULTS;
     TPRINTERDEFAULTS = PRINTER_DEFAULTS;
     PPRINTERDEFAULTS = ^PRINTER_DEFAULTS;

     PRINTER_INFO_1 = record
          Flags : DWORD;
          pDescription : LPTSTR;
          pName : LPTSTR;
          pComment : LPTSTR;
       end;
     LPPRINTER_INFO_1 = ^PRINTER_INFO_1;
     PPRINTER_INFO_1 = ^PRINTER_INFO_1;
     _PRINTER_INFO_1 = PRINTER_INFO_1;
     TPRINTERINFO1 = PRINTER_INFO_1;
     PPRINTERINFO1 = ^PRINTER_INFO_1;

     PRINTER_INFO_2 = record
          pServerName : LPTSTR;
          pPrinterName : LPTSTR;
          pShareName : LPTSTR;
          pPortName : LPTSTR;
          pDriverName : LPTSTR;
          pComment : LPTSTR;
          pLocation : LPTSTR;
          pDevMode : LPDEVMODE;
          pSepFile : LPTSTR;
          pPrintProcessor : LPTSTR;
          pDatatype : LPTSTR;
          pParameters : LPTSTR;
          pSecurityDescriptor : PSECURITY_DESCRIPTOR;
          Attributes : DWORD;
          Priority : DWORD;
          DefaultPriority : DWORD;
          StartTime : DWORD;
          UntilTime : DWORD;
          Status : DWORD;
          cJobs : DWORD;
          AveragePPM : DWORD;
       end;
     _PRINTER_INFO_2 = PRINTER_INFO_2;
     TPRINTERINFO2 = PRINTER_INFO_2;
     PPRINTERINFO2 = ^PRINTER_INFO_2;

     PRINTER_INFO_3 = record
          pSecurityDescriptor : PSECURITY_DESCRIPTOR;
       end;
     _PRINTER_INFO_3 = PRINTER_INFO_3;
     TPRINTERINFO3 = PRINTER_INFO_3;
     PPRINTERINFO3 = ^PRINTER_INFO_3;

     PRINTER_INFO_4 = record
          pPrinterName : LPTSTR;
          pServerName : LPTSTR;
          Attributes : DWORD;
       end;
     _PRINTER_INFO_4 = PRINTER_INFO_4;
     TPRINTERINFO4 = PRINTER_INFO_4;
     PPRINTERINFO4 = ^PRINTER_INFO_4;

     PRINTER_INFO_5 = record
          pPrinterName : LPTSTR;
          pPortName : LPTSTR;
          Attributes : DWORD;
          DeviceNotSelectedTimeout : DWORD;
          TransmissionRetryTimeout : DWORD;
       end;
     _PRINTER_INFO_5 = PRINTER_INFO_5;
     TPRINTERINFO5 = PRINTER_INFO_5;
     PPRINTERINFO5 = ^PRINTER_INFO_5;

     PRINTER_NOTIFY_INFO_DATA = record
          _Type : WORD;
          Field : WORD;
          Reserved : DWORD;
          Id : DWORD;
          NotifyData : record
              case longint of
                 0 : ( adwData : array[0..1] of DWORD );
                 1 : ( Data : record
                      cbBuf : DWORD;
                      pBuf : LPVOID;
                   end );
              end;
       end;
     _PRINTER_NOTIFY_INFO_DATA = PRINTER_NOTIFY_INFO_DATA;
     TPRINTERNOTIFYINFODATA = PRINTER_NOTIFY_INFO_DATA;
     PPRINTERNOTIFYINFODATA = ^PRINTER_NOTIFY_INFO_DATA;

     PRINTER_NOTIFY_INFO = record
          Version : DWORD;
          Flags : DWORD;
          Count : DWORD;
          aData : array[0..0] of PRINTER_NOTIFY_INFO_DATA;
       end;
     _PRINTER_NOTIFY_INFO = PRINTER_NOTIFY_INFO;
     TPRINTERNOTIFYINFO = PRINTER_NOTIFY_INFO;
     PPRINTERNOTIFYINFO = ^PRINTER_NOTIFY_INFO;

     PRINTER_NOTIFY_OPTIONS_TYPE = record
          _Type : WORD;
          Reserved0 : WORD;
          Reserved1 : DWORD;
          Reserved2 : DWORD;
          Count : DWORD;
          pFields : PWORD;
       end;
     PPRINTER_NOTIFY_OPTIONS_TYPE = ^PRINTER_NOTIFY_OPTIONS_TYPE;
     _PRINTER_NOTIFY_OPTIONS_TYPE = PRINTER_NOTIFY_OPTIONS_TYPE;
     TPRINTERNOTIFYOPTIONSTYPE = PRINTER_NOTIFY_OPTIONS_TYPE;
     PPRINTERNOTIFYOPTIONSTYPE = ^PRINTER_NOTIFY_OPTIONS_TYPE;

     PRINTER_NOTIFY_OPTIONS = record
          Version : DWORD;
          Flags : DWORD;
          Count : DWORD;
          pTypes : PPRINTER_NOTIFY_OPTIONS_TYPE;
       end;
     _PRINTER_NOTIFY_OPTIONS = PRINTER_NOTIFY_OPTIONS;
     TPRINTERNOTIFYOPTIONS = PRINTER_NOTIFY_OPTIONS;
     PPRINTERNOTIFYOPTIONS = ^PRINTER_NOTIFY_OPTIONS;

     PRINTPROCESSOR_INFO_1 = record
          pName : LPTSTR;
       end;
     _PRINTPROCESSOR_INFO_1 = PRINTPROCESSOR_INFO_1;
     TPRINTPROCESSORINFO1 = PRINTPROCESSOR_INFO_1;
     PPRINTPROCESSORINFO1 = ^PRINTPROCESSOR_INFO_1;

     PRIVILEGE_SET = record
          PrivilegeCount : DWORD;
          Control : DWORD;
          Privilege : array[0..(ANYSIZE_ARRAY)-1] of LUID_AND_ATTRIBUTES;
       end;
     LPPRIVILEGE_SET = ^PRIVILEGE_SET;
     PPRIVILEGE_SET = ^PRIVILEGE_SET;
     _PRIVILEGE_SET = PRIVILEGE_SET;
     TPRIVILEGESET = PRIVILEGE_SET;
     PPRIVILEGESET = ^PRIVILEGE_SET;

     PROCESS_HEAPENTRY = record
          lpData : PVOID;
          cbData : DWORD;
          cbOverhead : BYTE;
          iRegionIndex : BYTE;
          wFlags : WORD;
          case longint of
            0 : (hMem : HANDLE;
                 dwReserved : array [0..2] of DWORD;
                );
            1 : (dwCommittedSize : DWORD;
                 dwUnCommittedSize : DWORD;
                 lpFirstBlock : LPVOID;
                 lpLastBlock : LPVOID;
                );
       end;
     LPPROCESS_HEAP_ENTRY = ^PROCESS_HEAPENTRY;
     _PROCESS_HEAP_ENTRY = PROCESS_HEAPENTRY;
     TPROCESSHEAPENTRY = PROCESS_HEAPENTRY;
     PPROCESSHEAPENTRY = ^PROCESS_HEAPENTRY;

     PROCESS_INFORMATION = record
          hProcess : HANDLE;
          hThread : HANDLE;
          dwProcessId : DWORD;
          dwThreadId : DWORD;
       end;
     LPPROCESS_INFORMATION = ^PROCESS_INFORMATION;
     _PROCESS_INFORMATION = PROCESS_INFORMATION;
     TPROCESSINFORMATION = PROCESS_INFORMATION;
     PPROCESSINFORMATION = ^PROCESS_INFORMATION;

     LPFNPSPCALLBACK = function (_para1:HWND; _para2:UINT; _para3:LPVOID):UINT;stdcall;
     TFNPSPCALLBACK = LPFNPSPCALLBACK;

     PROPSHEETPAGE = record
          dwSize : DWORD;
          dwFlags : DWORD;
          hInstance : HINST;
          case longint of
            0 : (pszTemplate : LPCTSTR);
            1 : (pResource : LPCDLGTEMPLATE;
                 case longint of
                    0 : (hIcon : HICON);
                    1 : (pszIcon : LPCTSTR;
                         pszTitle : LPCTSTR;
                         pfnDlgProc : DLGPROC;
                         lParam : LPARAM;
                         pfnCallback : LPFNPSPCALLBACK;
                         pcRefParent : ^UINT;
                        );
                );
       end;
     LPPROPSHEETPAGE = ^PROPSHEETPAGE;
     LPCPROPSHEETPAGE = ^PROPSHEETPAGE;
     _PROPSHEETPAGE = PROPSHEETPAGE;
     TPROPSHEETPAGE = PROPSHEETPAGE;
     PPROPSHEETPAGE = ^PROPSHEETPAGE;

     emptyrecord = record
       end;
     lpemptyrecord = ^emptyrecord;
     HPROPSHEETPAGE = ^emptyrecord;

     PROPSHEETHEADER = record
          dwSize : DWORD;
          dwFlags : DWORD;
          hwndParent : HWND;
          hInstance : HINST;
          case longint of
              0 : (hIcon : HICON);
              1 : (pszIcon : LPCTSTR;
                   pszCaption : LPCTSTR;
                   nPages : UINT;
                   case longint of
                      0 : (nStartPage : UINT);
                      1 : (pStartPage : LPCTSTR;
                           case longint of
                              0 : (ppsp : LPCPROPSHEETPAGE);
                              1 : (phpage : ^HPROPSHEETPAGE;
                                   pfnCallback : PFNPROPSHEETCALLBACK;
                                   case longint of
                                     0 : (hbmWatermark : HBITMAP);
                                     1 : (pszbmWatermark : LPCTSTR;
                                          hplWatermark : HPALETTE;
                                          case longint of
                                             0 : (hbmHeader : HBITMAP);
                                             1 : (pszbmHeader: PAnsiChar);
                                         );
                                  );
                          );
                  );
       end;
     LPPROPSHEETHEADER = ^PROPSHEETHEADER;
     LPCPROPSHEETHEADER = ^PROPSHEETHEADER;
     _PROPSHEETHEADER = PROPSHEETHEADER;
     TPROPSHEETHEADER = PROPSHEETHEADER;
     PPROPSHEETHEADER = ^PROPSHEETHEADER;

     { PropertySheet callbacks  }
     LPFNADDPROPSHEETPAGE = function (_para1:HPROPSHEETPAGE; _para2:LPARAM):WINBOOL;stdcall;
     TFNADDPROPSHEETPAGE = LPFNADDPROPSHEETPAGE;

     LPFNADDPROPSHEETPAGES = function (_para1:LPVOID; _para2:LPFNADDPROPSHEETPAGE; _para3:LPARAM):WINBOOL;stdcall;
     TFNADDPROPSHEETPAGES = LPFNADDPROPSHEETPAGES;

     PROTOCOL_INFO = record
          dwServiceFlags : DWORD;
          iAddressFamily : WINT;
          iMaxSockAddr : WINT;
          iMinSockAddr : WINT;
          iSocketType : WINT;
          iProtocol : WINT;
          dwMessageSize : DWORD;
          lpProtocol : LPTSTR;
       end;
     _PROTOCOL_INFO = PROTOCOL_INFO;
     TPROTOCOLINFO = PROTOCOL_INFO;
     PPROTOCOLINFO = ^PROTOCOL_INFO;

     PROVIDOR_INFO_1 = record
          pName : LPTSTR;
          pEnvironment : LPTSTR;
          pDLLName : LPTSTR;
       end;
     _PROVIDOR_INFO_1 = PROVIDOR_INFO_1;
     TPROVIDORINFO1 = PROVIDOR_INFO_1;
     PPROVIDORINFO1 = ^PROVIDOR_INFO_1;

     PSHNOTIFY = record
          hdr : NMHDR;
          lParam : LPARAM;
       end;
     LPPSHNOTIFY = ^PSHNOTIFY;
     _PSHNOTIFY = PSHNOTIFY;
     TPSHNOTIFY = PSHNOTIFY;
     PPSHNOTIFY = ^PSHNOTIFY;

     PUNCTUATION = record
          iSize : UINT;
          szPunctuation : LPSTR;
       end;
     _punctuation = PUNCTUATION;
     Tpunctuation = PUNCTUATION;
     Ppunctuation = ^PUNCTUATION;

     QUERY_SERVICE_CONFIG = record
          dwServiceType : DWORD;
          dwStartType : DWORD;
          dwErrorControl : DWORD;
          lpBinaryPathName : LPTSTR;
          lpLoadOrderGroup : LPTSTR;
          dwTagId : DWORD;
          lpDependencies : LPTSTR;
          lpServiceStartName : LPTSTR;
          lpDisplayName : LPTSTR;
       end;
     LPQUERY_SERVICE_CONFIG = ^QUERY_SERVICE_CONFIG;
     _QUERY_SERVICE_CONFIG = QUERY_SERVICE_CONFIG;
     TQUERYSERVICECONFIG = QUERY_SERVICE_CONFIG;
     PQUERYSERVICECONFIG = ^QUERY_SERVICE_CONFIG;

     QUERY_SERVICE_CONFIGA = record
          dwServiceType : DWORD;
          dwStartType : DWORD;
          dwErrorControl : DWORD;
          lpBinaryPathName : LPSTR;
          lpLoadOrderGroup : LPSTR;
          dwTagId : DWORD;
          lpDependencies : LPSTR;
          lpServiceStartName : LPSTR;
          lpDisplayName : LPSTR;
       end;
     LPQUERY_SERVICE_CONFIGA = ^QUERY_SERVICE_CONFIGA;
     _QUERY_SERVICE_CONFIGA  = QUERY_SERVICE_CONFIGA;
     TQUERYSERVICECONFIGA    = QUERY_SERVICE_CONFIGA;
     PQUERYSERVICECONFIGA    = ^QUERY_SERVICE_CONFIGA;

     QUERY_SERVICE_CONFIGW = record
          dwServiceType : DWORD;
          dwStartType : DWORD;
          dwErrorControl : DWORD;
          lpBinaryPathName : LPWSTR;
          lpLoadOrderGroup : LPWSTR;
          dwTagId : DWORD;
          lpDependencies : LPWSTR;
          lpServiceStartName : LPWSTR;
          lpDisplayName : LPWSTR;
       end;
     LPQUERY_SERVICE_CONFIGW = ^QUERY_SERVICE_CONFIGW;
     _QUERY_SERVICE_CONFIGW  = QUERY_SERVICE_CONFIGW;
     TQUERYSERVICECONFIGW    = QUERY_SERVICE_CONFIGW;
     PQUERYSERVICECONFIGW    = ^QUERY_SERVICE_CONFIGW;

     QUERY_SERVICE_LOCK_STATUSA = record
          fIsLocked : DWORD;
          lpLockOwner : LPSTR;
          dwLockDuration : DWORD;
       end;
     LPQUERY_SERVICE_LOCK_STATUSA = ^QUERY_SERVICE_LOCK_STATUSA;
     _QUERY_SERVICE_LOCK_STATUSA  = QUERY_SERVICE_LOCK_STATUSA;
     TQUERYSERVICELOCKSTATUSA     = QUERY_SERVICE_LOCK_STATUSA;
     PQUERYSERVICELOCKSTATUSA     = ^QUERY_SERVICE_LOCK_STATUSA;

     QUERY_SERVICE_LOCK_STATUSW = record
          fIsLocked : DWORD;
          lpLockOwner : LPWSTR;
          dwLockDuration : DWORD;
       end;
     LPQUERY_SERVICE_LOCK_STATUSW = ^QUERY_SERVICE_LOCK_STATUSW;
     _QUERY_SERVICE_LOCK_STATUSW = QUERY_SERVICE_LOCK_STATUSW;
     TQUERYSERVICELOCKSTATUSW = QUERY_SERVICE_LOCK_STATUSW;
     PQUERYSERVICELOCKSTATUSW = ^QUERY_SERVICE_LOCK_STATUSW;
     {$ifdef Unicode}
     LPQUERY_SERVICE_LOCK_STATUS = ^QUERY_SERVICE_LOCK_STATUSW;
     _QUERY_SERVICE_LOCK_STATUS  = QUERY_SERVICE_LOCK_STATUSW;
     QUERY_SERVICE_LOCK_STATUS   = QUERY_SERVICE_LOCK_STATUSW;
     TQUERYSERVICELOCKSTATUS     = QUERY_SERVICE_LOCK_STATUSW;
     PQUERYSERVICELOCKSTATUS     = ^QUERY_SERVICE_LOCK_STATUSW;
     {$else}
     LPQUERY_SERVICE_LOCK_STATUS = ^QUERY_SERVICE_LOCK_STATUSA;
     _QUERY_SERVICE_LOCK_STATUS  = QUERY_SERVICE_LOCK_STATUSA;
     QUERY_SERVICE_LOCK_STATUS   = QUERY_SERVICE_LOCK_STATUSA;
     TQUERYSERVICELOCKSTATUS     = QUERY_SERVICE_LOCK_STATUSA;
     PQUERYSERVICELOCKSTATUS     = ^QUERY_SERVICE_LOCK_STATUSA;
     {$endif}

     RASAMBA = record
          dwSize : DWORD;
          dwError : DWORD;
          szNetBiosError : array[0..(NETBIOS_NAME_LEN + 1)-1] of AnsiCHAR;
          bLana : BYTE;
       end;
     _RASAMBA = RASAMBA;
     TRASAMBA = RASAMBA;
     PRASAMBA = ^RASAMBA;

     RASAMBW = record
          dwSize : DWORD;
          dwError : DWORD;
          szNetBiosError : array[0..(NETBIOS_NAME_LEN + 1)-1] of WideCHAR;
          bLana : BYTE;
       end;
     _RASAMBW = RASAMBW;
     TRASAMBW = RASAMBW;
     PRASAMBW = ^RASAMBW;
     {$ifdef Unicode}
       _RASAMB = RASAMBW;
       TRASAMB = RASAMBW;
       PRASAMB = PRASAMBW;
       RASAMB  = RASAMBW;
     {$else}
       _RASAMB = RASAMBA;
       TRASAMB = RASAMBA;
       RASAMB  = RASAMBA;
       PRASAMB = PRASAMBA;
     {$endif}

     RASCONNA = record
          dwSize : DWORD;
          hrasconn : HRASCONN;
          szEntryName : array[0..(RAS_MaxEntryName + 1)-1] of AnsiCHAR;
          szDeviceType : array[0..(RAS_MaxDeviceType + 1)-1] of AnsiCHAR;
          szDeviceName : array[0..(RAS_MaxDeviceName + 1)-1] of AnsiCHAR;
       end;
     _RASCONNA = RASCONNA;
     TRASCONNA = RASCONNA;
     PRASCONNA = ^RASCONNA;

     RASCONNW = record
          dwSize : DWORD;
          hrasconn : HRASCONN;
          szEntryName : array[0..(RAS_MaxEntryName + 1)-1] of WideCHAR;
          szDeviceType : array[0..(RAS_MaxDeviceType + 1)-1] of WideCHAR;
          szDeviceName : array[0..(RAS_MaxDeviceName + 1)-1] of WideCHAR;
       end;
     _RASCONNW = RASCONNW;
     TRASCONNW = RASCONNW;
     PRASCONNW = ^RASCONNW;
     {$ifdef Unicode}
        _RASCONN = RASCONNW;
        TRASCONN = RASCONNW;
        PRASCONN = PRASCONNW;
        RASCONN  = RASCONNW;
     {$else}
        _RASCONN = RASCONNA;
        TRASCONN = RASCONNA;
        PRASCONN = PRASCONNA;
        RASCONN  = RASCONNA;
     {$endif}

     { actually: in_addr }
     RASIPV4ADDR = record
       case longint of
         0 : (s_b1 : BYTE;
              s_b2 : BYTE;
              s_b3 : BYTE;
              s_b4 : BYTE;
             );
         1 : (s_w1 : USHORT;
              s_w2 : USHORT;
             );
         2 : (S_addr : ULONG;);
     end;

     { actually: in6_addr }
     RASIPV6ADDR = record
       case longint of
         0 : (rbyte : array[0..15] of BYTE);
         1 : (rword : array[0..7] of USHORT);
     end;

     RASTUNNELENDPOINT = record
       dwType : DWORD;
       case longint of
         0 : (ipv4 : RASIPV4ADDR);
         1 : (ipv6 : RASIPV6ADDR);
     end;

     RASCONNSUBSTATE = longint;   //enumerated type

     RASCONNSTATUSA = record
          dwSize : DWORD;
          rasconnstate : RASCONNSTATE;
          dwError : DWORD;
          szDeviceType : array[0..(RAS_MaxDeviceType + 1)-1] of AnsiCHAR;
          szDeviceName : array[0..(RAS_MaxDeviceName + 1)-1] of AnsiCHAR;
          szPhoneNumber : array[0..(RAS_MaxPhoneNumber + 1)-1] of AnsiCHAR; // http://msdn.microsoft.com/en-us/library/windows/desktop/aa376728%28v=vs.85%29.aspx
          localEndPoint : RASTUNNELENDPOINT;
          remoteEndPoint : RASTUNNELENDPOINT;
          rasconnsubstate : RASCONNSUBSTATE;
       end;
     _RASCONNSTATUSA = RASCONNSTATUSA;
     TRASCONNSTATUSA = RASCONNSTATUSA;
     PRASCONNSTATUSA = ^RASCONNSTATUSA;

     RASCONNSTATUSW = record
          dwSize : DWORD;
          rasconnstate : RASCONNSTATE;
          dwError : DWORD;
          szDeviceType : array[0..(RAS_MaxDeviceType + 1)-1] of WideCHAR;
          szDeviceName : array[0..(RAS_MaxDeviceName + 1)-1] of WideCHAR;
          szPhoneNumber : array[0..(RAS_MaxPhoneNumber + 1)-1] of WideCHAR; // http://msdn.microsoft.com/en-us/library/windows/desktop/aa376728%28v=vs.85%29.aspx
          localEndPoint : RASTUNNELENDPOINT;
          remoteEndPoint : RASTUNNELENDPOINT;
          rasconnsubstate : RASCONNSUBSTATE;
       end;
     _RASCONNSTATUSW = RASCONNSTATUSW;
     TRASCONNSTATUSW = RASCONNSTATUSW;
     PRASCONNSTATUSW = ^RASCONNSTATUSW;
     {$ifdef Unicode}
       _RASCONNSTATUS = RASCONNSTATUSW;
       TRASCONNSTATUS = RASCONNSTATUSW;
       PRASCONNSTATUS = PRASCONNSTATUSW;
       RASCONNSTATUS  = RASCONNSTATUSW;
     {$else}
       _RASCONNSTATUS = RASCONNSTATUSA;
       TRASCONNSTATUS = RASCONNSTATUSA;
       PRASCONNSTATUS = PRASCONNSTATUSA;
       RASCONNSTATUS  = RASCONNSTATUSA;
     {$endif}

     RASDIALEXTENSIONS = record
          dwSize : DWORD;
          dwfOptions : DWORD;
          hwndParent : HWND;
          reserved : DWORD;
       end;
     _RASDIALEXTENSIONS = RASDIALEXTENSIONS;
     TRASDIALEXTENSIONS = RASDIALEXTENSIONS;
     PRASDIALEXTENSIONS = ^RASDIALEXTENSIONS;

     RASDIALPARAMSA = record
          dwSize : DWORD;
          szEntryName : array[0..(RAS_MaxEntryName + 1)-1] of AnsiCHAR;
          szPhoneNumber : array[0..(RAS_MaxPhoneNumber + 1)-1] of AnsiCHAR;
          szCallbackNumber : array[0..(RAS_MaxCallbackNumber + 1)-1] of AnsiCHAR;
          szUserName : array[0..(UNLEN + 1)-1] of AnsiCHAR;
          szPassword : array[0..(PWLEN + 1)-1] of AnsiCHAR;
          szDomain : array[0..(DNLEN + 1)-1] of AnsiCHAR;
          dwSubEntry : DWORD; // http://msdn.microsoft.com/en-us/library/windows/desktop/aa377238%28v=vs.85%29.aspx
          dwCallbackId : ULONG_PTR;
          dwIfIndex : DWORD;
       end;
     _RASDIALPARAMSA = RASDIALPARAMSA;
     TRASDIALPARAMSA = RASDIALPARAMSA;
     PRASDIALPARAMSA = ^RASDIALPARAMSA;

     RASDIALPARAMSW = record
          dwSize : DWORD;
          szEntryName : array[0..(RAS_MaxEntryName + 1)-1] of WCHAR;
          szPhoneNumber : array[0..(RAS_MaxPhoneNumber + 1)-1] of WCHAR;
          szCallbackNumber : array[0..(RAS_MaxCallbackNumber + 1)-1] of WCHAR;
          szUserName : array[0..(UNLEN + 1)-1] of WCHAR;
          szPassword : array[0..(PWLEN + 1)-1] of WCHAR;
          szDomain : array[0..(DNLEN + 1)-1] of WCHAR;
          dwSubEntry : DWORD; // http://msdn.microsoft.com/en-us/library/windows/desktop/aa377238%28v=vs.85%29.aspx
          dwCallbackId : ULONG_PTR;
          dwIfIndex : DWORD;
       end;
     _RASDIALPARAMSW = RASDIALPARAMSW;
     TRASDIALPARAMSW = RASDIALPARAMSW;
     PRASDIALPARAMSW = ^RASDIALPARAMSW;
     {$ifdef Unicode}
        _RASDIALPARAMS = RASDIALPARAMSW;
        TRASDIALPARAMS = RASDIALPARAMSW;
        PRASDIALPARAMS = PRASDIALPARAMSW;
        RASDIALPARAMS  = RASDIALPARAMSW;
     {$else}
        _RASDIALPARAMS = RASDIALPARAMSA;
        TRASDIALPARAMS = RASDIALPARAMSA;
        PRASDIALPARAMS = PRASDIALPARAMSA;
        RASDIALPARAMS  = RASDIALPARAMSA;
     {$endif}

     RASENTRYNAMEA = record
          dwSize : DWORD;
          szEntryName : array[0..(RAS_MaxEntryName + 1)-1] of AnsiCHAR;
          { Winver>=0x500 members }
          dwFlags : DWORD;
          szPhonebookPath : array[0..(MAX_PATH + 1)-1] of AnsiCHAR;
       end;
     _RASENTRYNAMEA = RASENTRYNAMEA;
     TRASENTRYNAMEA = RASENTRYNAMEA;
     PRASENTRYNAMEA = ^RASENTRYNAMEA;

     RASENTRYNAMEW = record
          dwSize : DWORD;
          szEntryName : array[0..(RAS_MaxEntryName + 1)-1] of WCHAR;
          { Winver>=0x500 members }
          dwFlags : DWORD;
          szPhonebookPath : array[0..(MAX_PATH + 1)-1] of WCHAR;
       end;
     _RASENTRYNAMEW = RASENTRYNAMEW;
     TRASENTRYNAMEW = RASENTRYNAMEW;
     PRASENTRYNAMEW = ^RASENTRYNAMEW;
     {$ifdef Unicode}
         _RASENTRYNAME = RASENTRYNAMEW;
         TRASENTRYNAME = RASENTRYNAMEW;
         PRASENTRYNAME = ^RASENTRYNAMEW;
         RASENTRYNAME  = RASENTRYNAMEW;
     {$else}
         _RASENTRYNAME = RASENTRYNAMEA;
         TRASENTRYNAME = RASENTRYNAMEA;
         PRASENTRYNAME = ^RASENTRYNAMEA;
         RASENTRYNAME  = RASENTRYNAMEA;
     {$endif}

     RASPPPIPA = record
          dwSize : DWORD;
          dwError : DWORD;
          szIpAddress : array[0..(RAS_MaxIpAddress + 1)-1] of AnsiCHAR;
       end;
     _RASPPPIPA = RASPPPIPA;
     TRASPPPIPA = RASPPPIPA;
     PRASPPPIPA = ^RASPPPIPA;

     RASPPPIPW = record
          dwSize : DWORD;
          dwError : DWORD;
          szIpAddress : array[0..(RAS_MaxIpAddress + 1)-1] of WCHAR;
       end;
     _RASPPPIPW = RASPPPIPW;
     TRASPPPIPW = RASPPPIPW;
     PRASPPPIPW = ^RASPPPIPW;
     {$ifdef Unicode}
        _RASPPPIP  = RASPPPIPW;
        TRASPPPIP  = RASPPPIPW;
        PRASPPPIP  = PRASPPPIPW;
        RASPPPIP   = RASPPPIPW;
     {$else}
        _RASPPPIP  = RASPPPIPA;
        TRASPPPIP  = RASPPPIPA;
        PRASPPPIP  = PRASPPPIPA;
        RASPPPIP   = RASPPPIPA;
     {$endif}

     RASPPPIPXA = record
          dwSize : DWORD;
          dwError : DWORD;
          szIpxAddress : array[0..(RAS_MaxIpxAddress + 1)-1] of AnsiCHAR;
       end;
     _RASPPPIPXA = RASPPPIPXA;
     TRASPPPIPXA = RASPPPIPXA;
     PRASPPPIPXA = ^RASPPPIPXA;

     RASPPPIPXW = record
          dwSize : DWORD;
          dwError : DWORD;
          szIpxAddress : array[0..(RAS_MaxIpxAddress + 1)-1] of WCHAR;
       end;
     _RASPPPIPXW = RASPPPIPXW;
     TRASPPPIPXW = RASPPPIPXW;
     PRASPPPIPXW = ^RASPPPIPXW;

     {$ifdef Unicode}
        _RASPPPIPX  = RASPPPIPXW;
        TRASPPPIPX  = RASPPPIPXW;
        PRASPPPIPX  = PRASPPPIPXW;
        RASPPPIPX   = RASPPPIPXW;
     {$else}
        _RASPPPIPX  = RASPPPIPXA;
        TRASPPPIPX  = RASPPPIPXA;
        PRASPPPIPX  = PRASPPPIPXA;
        RASPPPIPX   = RASPPPIPXA;
      {$endif}

     RASPPPNBFA = record
          dwSize : DWORD;
          dwError : DWORD;
          dwNetBiosError : DWORD;
          szNetBiosError : array[0..(NETBIOS_NAME_LEN + 1)-1] of AnsiCHAR;
          szWorkstationName : array[0..(NETBIOS_NAME_LEN + 1)-1] of AnsiCHAR;
          bLana : BYTE;
       end;
     _RASPPPNBFA = RASPPPNBFA;
     TRASPPPNBFA = RASPPPNBFA;
     PRASPPPNBFA = ^RASPPPNBFA;

     RASPPPNBFW = record
          dwSize : DWORD;
          dwError : DWORD;
          dwNetBiosError : DWORD;
          szNetBiosError : array[0..(NETBIOS_NAME_LEN + 1)-1] of WCHAR;
          szWorkstationName : array[0..(NETBIOS_NAME_LEN + 1)-1] of WCHAR;
          bLana : BYTE;
       end;
     _RASPPPNBFW = RASPPPNBFW;
     TRASPPPNBFW = RASPPPNBFW;
     PRASPPPNBFW = ^RASPPPNBFW;
     {$ifdef Unicode}
        _RASPPPNBF  = RASPPPNBFW;
        TRASPPPNBF  = RASPPPNBFW;
        PRASPPPNBF  = PRASPPPNBFW;
        RASPPPNBF   = RASPPPNBFW;
     {$else}
        _RASPPPNBF  = RASPPPNBFA;
        TRASPPPNBF  = RASPPPNBFA;
        PRASPPPNBF  = PRASPPPNBFA;
        RASPPPNBF   = RASPPPNBFA;
     {$endif}
     RASTERIZER_STATUS = record
          nSize : short;
          wFlags : short;
          nLanguageID : short;
       end;
     LPRASTERIZER_STATUS = ^RASTERIZER_STATUS;
     _RASTERIZER_STATUS = RASTERIZER_STATUS;
     TRASTERIZERSTATUS = RASTERIZER_STATUS;
     PRASTERIZERSTATUS = ^RASTERIZER_STATUS;

     REASSIGN_BLOCKS = record
          Reserved : WORD;
          Count : WORD;
          BlockNumber : array[0..0] of DWORD;
       end;
     _REASSIGN_BLOCKS = REASSIGN_BLOCKS;
     TREASSIGNBLOCKS = REASSIGN_BLOCKS;
     PREASSIGNBLOCKS = ^REASSIGN_BLOCKS;

     REMOTE_NAME_INFOA = record
          lpUniversalName : LPTSTR;
          lpConnectionName : LPTSTR;
          lpRemainingPath : LPTSTR;
       end;
     _REMOTE_NAME_INFOA = REMOTE_NAME_INFOA;
     TREMOTENAMEINFOA = REMOTE_NAME_INFOA;
     PREMOTENAMEINFOA = ^REMOTE_NAME_INFOA;

     REMOTE_NAME_INFOW = record
          lpUniversalName : LPWSTR;
          lpConnectionName : LPWSTR;
          lpRemainingPath : LPWSTR;
       end;
     _REMOTE_NAME_INFOW = REMOTE_NAME_INFOW;
     TREMOTENAMEINFOW = REMOTE_NAME_INFOW;
     PREMOTENAMEINFOW = ^REMOTE_NAME_INFOW;

{$IFNDEF UNICODE}
     REMOTE_NAME_INFO = REMOTE_NAME_INFOA;
     _REMOTE_NAME_INFO = REMOTE_NAME_INFOA;
     TREMOTENAMEINFO = REMOTE_NAME_INFOA;
     PREMOTENAMEINFO = ^REMOTE_NAME_INFOA;
{$ELSE UNICODE}
     REMOTE_NAME_INFO = REMOTE_NAME_INFOW;
     _REMOTE_NAME_INFO = REMOTE_NAME_INFOW;
     TREMOTENAMEINFO = REMOTE_NAME_INFOW;
     PREMOTENAMEINFO = ^REMOTE_NAME_INFOW;
{$ENDIF UNICODE}

  (*
   TODO: OLE
  typedef struct _reobject {
    DWORD  cbStruct;
    LONG   cp;
    CLSID  clsid;
    LPOLEOBJECT      poleobj;
    LPSTORAGE        pstg;
    LPOLECLIENTSITE  polesite;
    SIZEL  sizel;
    DWORD  dvaspect;
    DWORD  dwFlags;
    DWORD  dwUser;
  } REOBJECT;
   *)

     REPASTESPECIAL = record
          dwAspect : DWORD;
          dwParam : DWORD;
       end;
     _repastespecial = REPASTESPECIAL;
     Trepastespecial = REPASTESPECIAL;
     Prepastespecial = ^REPASTESPECIAL;

     REQRESIZE = record
          nmhdr : NMHDR;
          rc : RECT;
       end;
     _reqresize = REQRESIZE;
     Treqresize = REQRESIZE;
     Preqresize = ^REQRESIZE;

     RGNDATAHEADER = record
          dwSize : DWORD;
          iType : DWORD;
          nCount : DWORD;
          nRgnSize : DWORD;
          rcBound : RECT;
       end;
     _RGNDATAHEADER = RGNDATAHEADER;
     TRGNDATAHEADER = RGNDATAHEADER;
     PRGNDATAHEADER = ^RGNDATAHEADER;

     RGNDATA = record
          rdh : RGNDATAHEADER;
          Buffer : array[0..0] of Ansichar;
       end;
     LPRGNDATA = ^RGNDATA;
     _RGNDATA = RGNDATA;
     TRGNDATA = RGNDATA;
     PRGNDATA = ^RGNDATA;

     SCROLLINFO = record
          cbSize : UINT;
          fMask : UINT;
          nMin : longint;
          nMax : longint;
          nPage : UINT;
          nPos : longint;
          nTrackPos : longint;
       end;
     LPSCROLLINFO = ^SCROLLINFO;
     LPCSCROLLINFO = ^SCROLLINFO;
     tagSCROLLINFO = SCROLLINFO;
     TSCROLLINFO = SCROLLINFO;
     PSCROLLINFO = ^SCROLLINFO;

     SECURITY_ATTRIBUTES = record
          nLength : DWORD;
          lpSecurityDescriptor : LPVOID;
          bInheritHandle : WINBOOL;
       end;
     LPSECURITY_ATTRIBUTES = ^SECURITY_ATTRIBUTES;
     _SECURITY_ATTRIBUTES = SECURITY_ATTRIBUTES;
     TSECURITYATTRIBUTES = SECURITY_ATTRIBUTES;
     PSECURITYATTRIBUTES = ^SECURITY_ATTRIBUTES;

     SECURITY_INFORMATION = DWORD;
     PSECURITY_INFORMATION = ^SECURITY_INFORMATION;
     TSECURITYINFORMATION = SECURITY_INFORMATION;
     PSECURITYINFORMATION = ^SECURITY_INFORMATION;

     SELCHANGE = record
          nmhdr : NMHDR;
          chrg : CHARRANGE;
          seltyp : WORD;
       end;
     _selchange = SELCHANGE;
     Tselchange = SELCHANGE;
     Pselchange = ^SELCHANGE;

     SERIALKEYS = record
          cbSize : UINT;
          dwFlags : DWORD;
          lpszActivePort : LPSTR;
          lpszPort : LPSTR;
          iBaudRate : DWORD;
          iPortState : DWORD;
       end;
     LPSERIALKEYS = ^SERIALKEYS;
     tagSERIALKEYS = SERIALKEYS;
     TSERIALKEYS = SERIALKEYS;
     PSERIALKEYS = ^SERIALKEYS;

     SERVICE_TABLE_ENTRY = record
          lpServiceName : LPTSTR;
          lpServiceProc : LPSERVICE_MAIN_FUNCTION;
       end;
     LPSERVICE_TABLE_ENTRY = ^SERVICE_TABLE_ENTRY;
     _SERVICE_TABLE_ENTRY = SERVICE_TABLE_ENTRY;
     TSERVICETABLEENTRY = SERVICE_TABLE_ENTRY;
     PSERVICETABLEENTRY = ^SERVICE_TABLE_ENTRY;

     SERVICE_TABLE_ENTRYA = record
          lpServiceName : LPSTR;
          lpServiceProc : LPSERVICE_MAIN_FUNCTIONA;
       end;
     LPSERVICE_TABLE_ENTRYA = ^SERVICE_TABLE_ENTRYA;
     _SERVICE_TABLE_ENTRYA = SERVICE_TABLE_ENTRYA;
     TSERVICETABLEENTRYA = SERVICE_TABLE_ENTRYA;
     PSERVICETABLEENTRYA = ^SERVICE_TABLE_ENTRYA;

     SERVICE_TABLE_ENTRYW = record
          lpServiceName : LPWSTR;
          lpServiceProc : LPSERVICE_MAIN_FUNCTIONW;
       end;
     LPSERVICE_TABLE_ENTRYW = ^SERVICE_TABLE_ENTRYW;
     _SERVICE_TABLE_ENTRYW = SERVICE_TABLE_ENTRYW;
     TSERVICETABLEENTRYW = SERVICE_TABLE_ENTRYW;
     PSERVICETABLEENTRYW = ^SERVICE_TABLE_ENTRYW;

     SERVICE_TYPE_VALUE_ABS = record
          dwNameSpace : DWORD;
          dwValueType : DWORD;
          dwValueSize : DWORD;
          lpValueName : LPTSTR;
          lpValue : PVOID;
       end;
     _SERVICE_TYPE_VALUE_ABS = SERVICE_TYPE_VALUE_ABS;
     TSERVICETYPEVALUEABS = SERVICE_TYPE_VALUE_ABS;
     PSERVICETYPEVALUEABS = ^SERVICE_TYPE_VALUE_ABS;

     SERVICE_TYPE_INFO_ABS = record
          lpTypeName : LPTSTR;
          dwValueCount : DWORD;
          Values : array[0..0] of SERVICE_TYPE_VALUE_ABS;
       end;
     _SERVICE_TYPE_INFO_ABS = SERVICE_TYPE_INFO_ABS;
     TSERVICETYPEINFOABS = SERVICE_TYPE_INFO_ABS;
     PSERVICETYPEINFOABS = ^SERVICE_TYPE_INFO_ABS;

     SESSION_BUFFER = record
          lsn : UCHAR;
          state : UCHAR;
          local_name : array[0..(NCBNAMSZ)-1] of UCHAR;
          remote_name : array[0..(NCBNAMSZ)-1] of UCHAR;
          rcvs_outstanding : UCHAR;
          sends_outstanding : UCHAR;
       end;
     _SESSION_BUFFER = SESSION_BUFFER;
     TSESSIONBUFFER = SESSION_BUFFER;
     PSESSIONBUFFER = ^SESSION_BUFFER;

     SESSION_HEADER = record
          sess_name : UCHAR;
          num_sess : UCHAR;
          rcv_dg_outstanding : UCHAR;
          rcv_any_outstanding : UCHAR;
       end;
     _SESSION_HEADER = SESSION_HEADER;
     TSESSIONHEADER = SESSION_HEADER;
     PSESSIONHEADER = ^SESSION_HEADER;


     SHCONTF = (SHCONTF_FOLDERS := 32,SHCONTF_NONFOLDERS := 64,
       SHCONTF_INCLUDEHIDDEN := 128);
     tagSHCONTF = SHCONTF;
     TSHCONTF = SHCONTF;

     SHFILEINFO = record
          hIcon : HICON;
          iIcon : longint;
          dwAttributes : DWORD;
          szDisplayName : array[0..(MAX_PATH)-1] of TCHAR;
          szTypeName : array[0..79] of TCHAR;
       end;
     _SHFILEINFO = SHFILEINFO;
     TSHFILEINFO = SHFILEINFO;
     PSHFILEINFO = ^SHFILEINFO;

     FILEOP_FLAGS = WORD;
     TFILEOPFLAGS = FILEOP_FLAGS;
     PFILEOPFLAGS = ^FILEOP_FLAGS;

     SHFILEOPSTRUCT = record
          hwnd : HWND;
          wFunc : UINT;
          pFrom : LPCSTR;   // PCZZTSTR
          pTo : LPCSTR;     // PCZZTSTR
          fFlags : FILEOP_FLAGS;
          fAnyOperationsAborted : WINBOOL;
          hNameMappings : LPVOID;
          lpszProgressTitle : LPCSTR;   // PCTSTR
       end;
     LPSHFILEOPSTRUCT = ^SHFILEOPSTRUCT;
     _SHFILEOPSTRUCT = SHFILEOPSTRUCT;
     TSHFILEOPSTRUCT = SHFILEOPSTRUCT;
     PSHFILEOPSTRUCT = ^SHFILEOPSTRUCT;

     SHGNO = (SHGDN_NORMAL := 0,SHGDN_INFOLDER := 1,
       SHGDN_FORPARSING := $8000);
     tagSHGDN = SHGNO;
     TSHGDN = SHGNO;

     SHNAMEMAPPING = record
          pszOldPath : LPSTR;
          pszNewPath : LPSTR;
          cchOldPath : longint;
          cchNewPath : longint;
       end;
     LPSHNAMEMAPPING = ^SHNAMEMAPPING;
     _SHNAMEMAPPING = SHNAMEMAPPING;
     TSHNAMEMAPPING = SHNAMEMAPPING;
     PSHNAMEMAPPING = ^SHNAMEMAPPING;

     SID_AND_ATTRIBUTES = record
          Sid : PSID;
          Attributes : DWORD;
       end;
     _SID_AND_ATTRIBUTES = SID_AND_ATTRIBUTES;
     TSIDANDATTRIBUTES = SID_AND_ATTRIBUTES;
     PSIDANDATTRIBUTES = ^SID_AND_ATTRIBUTES;

     SID_AND_ATTRIBUTES_ARRAY = array[0..(ANYSIZE_ARRAY)-1] of SID_AND_ATTRIBUTES;
     PSID_AND_ATTRIBUTES_ARRAY = ^SID_AND_ATTRIBUTES_ARRAY;
     TSIDANDATTRIBUTESARRAY = SID_AND_ATTRIBUTES_ARRAY;
     PSIDANDATTRIBUTESARRAY = ^SID_AND_ATTRIBUTES_ARRAY;

     SINGLE_LIST_ENTRY = record
          Next : ^_SINGLE_LIST_ENTRY;
       end;
     _SINGLE_LIST_ENTRY = SINGLE_LIST_ENTRY;
     TSINGLELISTENTRY = SINGLE_LIST_ENTRY;
     PSINGLELISTENTRY = ^SINGLE_LIST_ENTRY;

     SOUNDSENTRY = record
          cbSize : UINT;
          dwFlags : DWORD;
          iFSTextEffect : DWORD;
          iFSTextEffectMSec : DWORD;
          iFSTextEffectColorBits : DWORD;
          iFSGrafEffect : DWORD;
          iFSGrafEffectMSec : DWORD;
          iFSGrafEffectColor : DWORD;
          iWindowsEffect : DWORD;
          iWindowsEffectMSec : DWORD;
          lpszWindowsEffectDLL : LPTSTR;
          iWindowsEffectOrdinal : DWORD;
       end;
     LPSOUNDSENTRY = ^SOUNDSENTRY;
     tagSOUNDSENTRY = SOUNDSENTRY;
     TSOUNDSENTRY = SOUNDSENTRY;
     PSOUNDSENTRY = ^SOUNDSENTRY;
     STARTUPINFOA = record
          cb : DWORD;
          lpReserved : LPSTR;
          lpDesktop : LPSTR;
          lpTitle : LPSTR;
          dwX : DWORD;
          dwY : DWORD;
          dwXSize : DWORD;
          dwYSize : DWORD;
          dwXCountChars : DWORD;
          dwYCountChars : DWORD;
          dwFillAttribute : DWORD;
          dwFlags : DWORD;
          wShowWindow : WORD;
          cbReserved2 : WORD;
          lpReserved2 : LPBYTE;
          hStdInput : HANDLE;
          hStdOutput : HANDLE;
          hStdError : HANDLE;
       end;
     LPSTARTUPINFOA = ^STARTUPINFOA;
     _STARTUPINFOA = STARTUPINFOA;
     TSTARTUPINFOA = STARTUPINFOA;
     PSTARTUPINFOA = ^STARTUPINFOA;
     STARTUPINFOW = record
          cb : DWORD;
          lpReserved : LPWSTR;
          lpDesktop : LPWSTR;
          lpTitle : LPWSTR;
          dwX : DWORD;
          dwY : DWORD;
          dwXSize : DWORD;
          dwYSize : DWORD;
          dwXCountChars : DWORD;
          dwYCountChars : DWORD;
          dwFillAttribute : DWORD;
          dwFlags : DWORD;
          wShowWindow : WORD;
          cbReserved2 : WORD;
          lpReserved2 : LPBYTE;
          hStdInput : HANDLE;
          hStdOutput : HANDLE;
          hStdError : HANDLE;
       end;
     LPSTARTUPINFOW = ^STARTUPINFOW;
     _STARTUPINFOW = STARTUPINFOW;
     TSTARTUPINFOW = STARTUPINFOW;
     PSTARTUPINFOW = ^STARTUPINFOW;

     STARTUPINFO = record
          cb : DWORD;
          lpReserved : LPTSTR;
          lpDesktop : LPTSTR;
          lpTitle : LPTSTR;
          dwX : DWORD;
          dwY : DWORD;
          dwXSize : DWORD;
          dwYSize : DWORD;
          dwXCountChars : DWORD;
          dwYCountChars : DWORD;
          dwFillAttribute : DWORD;
          dwFlags : DWORD;
          wShowWindow : WORD;
          cbReserved2 : WORD;
          lpReserved2 : LPBYTE;
          hStdInput : HANDLE;
          hStdOutput : HANDLE;
          hStdError : HANDLE;
       end;
     {$ifdef Unicode}
     LPSTARTUPINFO = LPSTARTUPINFOW;
     _STARTUPINFO = _STARTUPINFOW;
     TSTARTUPINFO = TSTARTUPINFOW;
     PSTARTUPINFO = PSTARTUPINFOW;
     {$else}
     LPSTARTUPINFO = LPSTARTUPINFOA;
     _STARTUPINFO = _STARTUPINFOA;
     TSTARTUPINFO = TSTARTUPINFOA;
     PSTARTUPINFO = PSTARTUPINFOA;
     {$endif}
     STICKYKEYS = record
          cbSize : DWORD;
          dwFlags : DWORD;
       end;
     LPSTICKYKEYS = ^STICKYKEYS;
     tagSTICKYKEYS = STICKYKEYS;
     TSTICKYKEYS = STICKYKEYS;
     PSTICKYKEYS = ^STICKYKEYS;

     STRRET = record
          utype : UINT;
          case longint of
           0: ( polestr:LPWSTR);
           1: ( uOffset : UINT);
           2: ( cStr : array[0..(MAX_PATH)-1] of char );
       end;
     LPSTRRET = ^STRRET;
     _STRRET = STRRET;
     TSTRRET = STRRET;
     PSTRRET = ^STRRET;

     STYLEBUF = record
          dwStyle : DWORD;
          szDescription : array[0..31] of TCHAR;
       end;
     LPSTYLEBUF = ^STYLEBUF;
     _tagSTYLEBUF = STYLEBUF;
     TSTYLEBUF = STYLEBUF;
     PSTYLEBUF = ^STYLEBUF;

     STYLESTRUCT = record
          styleOld : DWORD;
          styleNew : DWORD;
       end;
     LPSTYLESTRUCT = ^STYLESTRUCT;
     tagSTYLESTRUCT = STYLESTRUCT;
     TSTYLESTRUCT = STYLESTRUCT;
     PSTYLESTRUCT = ^STYLESTRUCT;

     SYSTEM_AUDIT_ACE = record
          Header : ACE_HEADER;
          Mask : ACCESS_MASK;
          SidStart : DWORD;
       end;
     _SYSTEM_AUDIT_ACE = SYSTEM_AUDIT_ACE;
     TSYSTEMAUDITACE = SYSTEM_AUDIT_ACE;
     PSYSTEMAUDITACE = ^SYSTEM_AUDIT_ACE;

     SYSTEM_INFO = record
              case longint of
                 0 : ( dwOemId : DWORD;
		       dwPageSize : DWORD;
          	       lpMinimumApplicationAddress : LPVOID;
          	       lpMaximumApplicationAddress : LPVOID;
          	       dwActiveProcessorMask : DWORD_PTR;
          	       dwNumberOfProcessors : DWORD;
                       dwProcessorType : DWORD;
                       dwAllocationGranularity : DWORD;
                       wProcessorLevel : WORD;
                       wProcessorRevision : WORD;	
			 );
                 1 : (
                      wProcessorArchitecture : WORD;
                      wReserved : WORD;
                    );
       end;
     LPSYSTEM_INFO = ^SYSTEM_INFO;
     _SYSTEM_INFO = SYSTEM_INFO;
     TSYSTEMINFO = SYSTEM_INFO;
     PSYSTEMINFO = ^SYSTEM_INFO;

     SYSTEM_POWER_STATUS = record
          ACLineStatus : BYTE;
          BatteryFlag : BYTE;
          BatteryLifePercent : BYTE;
          Reserved1 : BYTE;
          BatteryLifeTime : DWORD;
          BatteryFullLifeTime : DWORD;
       end;
     _SYSTEM_POWER_STATUS = SYSTEM_POWER_STATUS;
     TSYSTEMPOWERSTATUS = SYSTEM_POWER_STATUS;
     PSYSTEMPOWERSTATUS = ^SYSTEM_POWER_STATUS;

     LPSYSTEM_POWER_STATUS = ^emptyrecord;

     TAPE_ERASE = record
          _Type : ULONG;
          Immediate : BOOLEAN;
       end;
     _TAPE_ERASE = TAPE_ERASE;
     TTAPEERASE = TAPE_ERASE;
     PTAPEERASE = ^TAPE_ERASE;

     TAPE_GET_DRIVE_PARAMETERS = record
          ECC : BOOLEAN;
          Compression : BOOLEAN;
          DataPadding : BOOLEAN;
          ReportSetmarks : BOOLEAN;
          DefaultBlockSize : ULONG;
          MaximumBlockSize : ULONG;
          MinimumBlockSize : ULONG;
          MaximumPartitionCount : ULONG;
          FeaturesLow : ULONG;
          FeaturesHigh : ULONG;
          EOTWarningZoneSize : ULONG;
       end;
     _TAPE_GET_DRIVE_PARAMETERS = TAPE_GET_DRIVE_PARAMETERS;
     TTAPEGETDRIVEPARAMETERS = TAPE_GET_DRIVE_PARAMETERS;
     PTAPEGETDRIVEPARAMETERS = ^TAPE_GET_DRIVE_PARAMETERS;

     TAPE_GET_MEDIA_PARAMETERS = record
          Capacity : LARGE_INTEGER;
          Remaining : LARGE_INTEGER;
          BlockSize : ULONG;
          PartitionCount : ULONG;
          WriteProtected : BOOLEAN;
       end;
     _TAPE_GET_MEDIA_PARAMETERS = TAPE_GET_MEDIA_PARAMETERS;
     TTAPEGETMEDIAPARAMETERS = TAPE_GET_MEDIA_PARAMETERS;
     PTAPEGETMEDIAPARAMETERS = ^TAPE_GET_MEDIA_PARAMETERS;

     TAPE_GET_POSITION = record
          _Type : ULONG;
          Partition : ULONG;
          OffsetLow : ULONG;
          OffsetHigh : ULONG;
       end;
     _TAPE_GET_POSITION = TAPE_GET_POSITION;
     TTAPEGETPOSITION = TAPE_GET_POSITION;
     PTAPEGETPOSITION = ^TAPE_GET_POSITION;

     TAPE_PREPARE = record
          Operation : ULONG;
          Immediate : BOOLEAN;
       end;
     _TAPE_PREPARE = TAPE_PREPARE;
     TTAPEPREPARE = TAPE_PREPARE;
     PTAPEPREPARE = ^TAPE_PREPARE;

     TAPE_SET_DRIVE_PARAMETERS = record
          ECC : BOOLEAN;
          Compression : BOOLEAN;
          DataPadding : BOOLEAN;
          ReportSetmarks : BOOLEAN;
          EOTWarningZoneSize : ULONG;
       end;
     _TAPE_SET_DRIVE_PARAMETERS = TAPE_SET_DRIVE_PARAMETERS;
     TTAPESETDRIVEPARAMETERS = TAPE_SET_DRIVE_PARAMETERS;
     PTAPESETDRIVEPARAMETERS = ^TAPE_SET_DRIVE_PARAMETERS;

     TAPE_SET_MEDIA_PARAMETERS = record
          BlockSize : ULONG;
       end;
     _TAPE_SET_MEDIA_PARAMETERS = TAPE_SET_MEDIA_PARAMETERS;
     TTAPESETMEDIAPARAMETERS = TAPE_SET_MEDIA_PARAMETERS;
     PTAPESETMEDIAPARAMETERS = ^TAPE_SET_MEDIA_PARAMETERS;

     TAPE_SET_POSITION = record
          Method : ULONG;
          Partition : ULONG;
          OffsetLow : ULONG;
          OffsetHigh : ULONG;
          Immediate : BOOLEAN;
       end;
     _TAPE_SET_POSITION = TAPE_SET_POSITION;
     TTAPESETPOSITION = TAPE_SET_POSITION;
     PTAPESETPOSITION = ^TAPE_SET_POSITION;

     TAPE_WRITE_MARKS = record
          _Type : ULONG;
          Count : ULONG;
          Immediate : BOOLEAN;
       end;
     _TAPE_WRITE_MARKS = TAPE_WRITE_MARKS;
     TTAPEWRITEMARKS = TAPE_WRITE_MARKS;
     PTAPEWRITEMARKS = ^TAPE_WRITE_MARKS;

     TBADDBITMAP = record
          hInst : HINST;
          nID : UINT_PTR;
       end;
     LPTBADDBITMAP = ^TBADDBITMAP;
     TTBADDBITMAP = TBADDBITMAP;
     PTBADDBITMAP = ^TBADDBITMAP;

     TBBUTTON = record
          iBitmap : longint;
          idCommand : longint;
          fsState : BYTE;
          fsStyle : BYTE;
{$ifdef WIN64}
          bReserved : array[0..5] of BYTE;
{$else}
          bReserved : array[0..1] of BYTE;
{$endif}
          dwData : DWORD_PTR;
          iString : INT_PTR;
       end;
     LPTBBUTTON = ^TBBUTTON;
     LPCTBBUTTON = ^TBBUTTON;
     _TBBUTTON = TBBUTTON;
     TTBBUTTON = TBBUTTON;
     PTBBUTTON = ^TBBUTTON;

     TBNOTIFY = record
          hdr : NMHDR;
          iItem : longint;
          tbButton : TBBUTTON;
          cchText : longint;
          pszText : LPTSTR;
       end;
     LPTBNOTIFY = ^TBNOTIFY;
     TTBNOTIFY = TBNOTIFY;
     PTBNOTIFY = ^TBNOTIFY;

     TBSAVEPARAMS = record
          hkr : HKEY;
          pszSubKey : LPCTSTR;
          pszValueName : LPCTSTR;
       end;
     TTBSAVEPARAMS = TBSAVEPARAMS;
     PTBSAVEPARAMS = ^TBSAVEPARAMS;

     TC_HITTESTINFO = record
          pt : POINT;
          flags : UINT;
       end;
     _TC_HITTESTINFO = TC_HITTESTINFO;
     TTCHITTESTINFO = TC_HITTESTINFO;
     PTCHITTESTINFO = ^TC_HITTESTINFO;

     TC_ITEM = record
          mask : UINT;
          lpReserved1 : UINT;
          lpReserved2 : UINT;
          pszText : LPTSTR;
          cchTextMax : longint;
          iImage : longint;
          lParam : LPARAM;
       end;
     _TC_ITEM = TC_ITEM;
     TTCITEM = TC_ITEM;
     PTCITEM = ^TC_ITEM;

     TC_ITEMHEADER = record
          mask : UINT;
          lpReserved1 : UINT;
          lpReserved2 : UINT;
          pszText : LPTSTR;
          cchTextMax : longint;
          iImage : longint;
       end;
     _TC_ITEMHEADER = TC_ITEMHEADER;
     TTCITEMHEADER = TC_ITEMHEADER;
     PTCITEMHEADER = ^TC_ITEMHEADER;

     TC_KEYDOWN = record
          hdr : NMHDR;
          wVKey : WORD;
          flags : UINT;
       end;
     _TC_KEYDOWN = TC_KEYDOWN;
     TTCKEYDOWN = TC_KEYDOWN;
     PTCKEYDOWN = ^TC_KEYDOWN;

     TEXTRANGE = record
          chrg : CHARRANGE;
          lpstrText : LPSTR;
       end;
     _textrange = TEXTRANGE;
     Ttextrange = TEXTRANGE;
     Ptextrange = ^TEXTRANGE;

     TIME_ZONE_INFORMATION = record
          Bias : LONG;
          StandardName : array[0..31] of WCHAR;
          StandardDate : SYSTEMTIME;
          StandardBias : LONG;
          DaylightName : array[0..31] of WCHAR;
          DaylightDate : SYSTEMTIME;
          DaylightBias : LONG;
       end;
     LPTIME_ZONE_INFORMATION = ^TIME_ZONE_INFORMATION;
     _TIME_ZONE_INFORMATION = TIME_ZONE_INFORMATION;
     TTIMEZONEINFORMATION = TIME_ZONE_INFORMATION;
     PTIMEZONEINFORMATION = ^TIME_ZONE_INFORMATION;

     TOGGLEKEYS = record
          cbSize : DWORD;
          dwFlags : DWORD;
       end;
     tagTOGGLEKEYS = TOGGLEKEYS;
     TTOGGLEKEYS = TOGGLEKEYS;
     PTOGGLEKEYS = ^TOGGLEKEYS;

     TOKEN_SOURCE = record
          SourceName : array[0..7] of CHAR;  // 0..(TOKEN_SOURCE_LENGTH-1)
          SourceIdentifier : LUID;
       end;
     _TOKEN_SOURCE = TOKEN_SOURCE;
     TTOKENSOURCE = TOKEN_SOURCE;
     PTOKENSOURCE = ^TOKEN_SOURCE;

     TOKEN_CONTROL = record
          TokenId : LUID;
          AuthenticationId : LUID;
          ModifiedId : LUID;
          TokenSource : TOKEN_SOURCE;
       end;
     _TOKEN_CONTROL = TOKEN_CONTROL;
     TTOKENCONTROL = TOKEN_CONTROL;
     PTOKENCONTROL = ^TOKEN_CONTROL;

     TOKEN_DEFAULT_DACL = record
          DefaultDacl : PACL;
       end;
     _TOKEN_DEFAULT_DACL = TOKEN_DEFAULT_DACL;
     TTOKENDEFAULTDACL = TOKEN_DEFAULT_DACL;
     PTOKENDEFAULTDACL = ^TOKEN_DEFAULT_DACL;

     TOKEN_GROUPS = record
          GroupCount : DWORD;
          Groups : array[0..(ANYSIZE_ARRAY)-1] of SID_AND_ATTRIBUTES;
       end;
     PTOKEN_GROUPS = ^TOKEN_GROUPS;
     LPTOKEN_GROUPS = ^TOKEN_GROUPS;
     _TOKEN_GROUPS = TOKEN_GROUPS;
     TTOKENGROUPS = TOKEN_GROUPS;
     PTOKENGROUPS = ^TOKEN_GROUPS;

     TOKEN_OWNER = record
          Owner : PSID;
       end;
     _TOKEN_OWNER = TOKEN_OWNER;
     TTOKENOWNER = TOKEN_OWNER;
     PTOKENOWNER = ^TOKEN_OWNER;

     TOKEN_PRIMARY_GROUP = record
          PrimaryGroup : PSID;
       end;
     _TOKEN_PRIMARY_GROUP = TOKEN_PRIMARY_GROUP;
     TTOKENPRIMARYGROUP = TOKEN_PRIMARY_GROUP;
     PTOKENPRIMARYGROUP = ^TOKEN_PRIMARY_GROUP;

     TOKEN_PRIVILEGES = packed record
          PrivilegeCount : DWORD;
          Privileges : array[0..(ANYSIZE_ARRAY)-1] of LUID_AND_ATTRIBUTES;
       end;

     PTOKEN_PRIVILEGES = ^TOKEN_PRIVILEGES;
     LPTOKEN_PRIVILEGES = ^TOKEN_PRIVILEGES;
     _TOKEN_PRIVILEGES = TOKEN_PRIVILEGES;
     TTOKENPRIVILEGES = TOKEN_PRIVILEGES;
     PTOKENPRIVILEGES = ^TOKEN_PRIVILEGES;

     TOKEN_STATISTICS = record
          TokenId : LUID;
          AuthenticationId : LUID;
          ExpirationTime : LARGE_INTEGER;
          TokenType : TOKEN_TYPE;
          ImpersonationLevel : SECURITY_IMPERSONATION_LEVEL;
          DynamicCharged : DWORD;
          DynamicAvailable : DWORD;
          GroupCount : DWORD;
          PrivilegeCount : DWORD;
          ModifiedId : LUID;
       end;
     _TOKEN_STATISTICS = TOKEN_STATISTICS;
     TTOKENSTATISTICS = TOKEN_STATISTICS;
     PTOKENSTATISTICS = ^TOKEN_STATISTICS;

     TOKEN_USER = record
          User : SID_AND_ATTRIBUTES;
       end;
     _TOKEN_USER = TOKEN_USER;
     TTOKENUSER = TOKEN_USER;
     PTOKENUSER = ^TOKEN_USER;

     TOOLINFO = record
          cbSize : UINT;
          uFlags : UINT;
          hwnd : HWND;
          uId : UINT;
          rect : RECT;
          hinst : HINST;
          lpszText : LPTSTR;
          lParam : LPARAM;
          lpReserved : LPVOID;
       end;
     LPTOOLINFO = ^TOOLINFO;
     TTOOLINFO = TOOLINFO;
     PTOOLINFO = ^TOOLINFO;

     TOOLTIPTEXTA = record
          hdr : NMHDR;
          lpszText : LPSTR;
          szText : array[0..79] of AnsiCHAR;
          hinst : HINST;
          uFlags : UINT;
          lParam : LPARAM;
       end;
     LPTOOLTIPTEXTA = ^TOOLTIPTEXTA;
     TTOOLTIPTEXTA  = TOOLTIPTEXTA;
     PTOOLTIPTEXTA  = ^TOOLTIPTEXTA;

     TOOLTIPTEXTW = record
          hdr : NMHDR;
          lpszText : LPWSTR;
          szText : array[0..79] of WideCHAR;
          hinst : HINST;
          uFlags : UINT;
          lParam : LPARAM;
       end;
     LPTOOLTIPTEXTW = ^TOOLTIPTEXTW;
     TTOOLTIPTEXTW  = TOOLTIPTEXTW;
     PTOOLTIPTEXTW  = ^TOOLTIPTEXTW;

     {$ifdef Unicode}
     LPTOOLTIPTEXT  = ^TOOLTIPTEXTW;
     TOOLTIPTEXT    = TOOLTIPTEXTW;
     TTOOLTIPTEXT   = TOOLTIPTEXTW;
     PTOOLTIPTEXT   = ^TOOLTIPTEXTW;
     {$else}
     LPTOOLTIPTEXT  = ^TOOLTIPTEXTA;
     TOOLTIPTEXT    = TOOLTIPTEXTA;
     TTOOLTIPTEXT   = TOOLTIPTEXTA;
     PTOOLTIPTEXT   = ^TOOLTIPTEXTA;
     {$endif}

     TPMPARAMS = record
          cbSize : UINT;
          rcExclude : RECT;
       end;
     LPTPMPARAMS = ^TPMPARAMS;
     tagTPMPARAMS = TPMPARAMS;
     TTPMPARAMS = TPMPARAMS;
     PTPMPARAMS = ^TPMPARAMS;

     TRANSMIT_FILE_BUFFERS = record
          Head : PVOID;
          HeadLength : DWORD;
          Tail : PVOID;
          TailLength : DWORD;
       end;
     _TRANSMIT_FILE_BUFFERS = TRANSMIT_FILE_BUFFERS;
     TTRANSMITFILEBUFFERS = TRANSMIT_FILE_BUFFERS;
     PTRANSMITFILEBUFFERS = ^TRANSMIT_FILE_BUFFERS;

     TTHITTESTINFO = record
          hwnd : HWND;
          pt : POINT;
          ti : TOOLINFO;
       end;
     LPHITTESTINFO = ^TTHITTESTINFO;
     _TT_HITTESTINFO = TTHITTESTINFO;
     TTTHITTESTINFO = TTHITTESTINFO;
     PTTHITTESTINFO = ^TTHITTESTINFO;

     TTPOLYCURVE = record
          wType : WORD;
          cpfx : WORD;
          apfx : array[0..0] of POINTFX;
       end;
     LPTTPOLYCURVE = ^TTPOLYCURVE;
     tagTTPOLYCURVE = TTPOLYCURVE;
     TTTPOLYCURVE = TTPOLYCURVE;
     PTTPOLYCURVE = ^TTPOLYCURVE;

     TTPOLYGONHEADER = record
          cb : DWORD;
          dwType : DWORD;
          pfxStart : POINTFX;
       end;
     LPTTPOLYGONHEADER = ^TTPOLYGONHEADER;
     _TTPOLYGONHEADER = TTPOLYGONHEADER;
     TTTPOLYGONHEADER = TTPOLYGONHEADER;
     PTTPOLYGONHEADER = ^TTPOLYGONHEADER;

     TV_DISPINFO = record
          hdr : NMHDR;
          item : TV_ITEM;
       end;
     _TV_DISPINFO = TV_DISPINFO;
     TTVDISPINFO = TV_DISPINFO;
     PTVDISPINFO = ^TV_DISPINFO;

     TV_HITTESTINFO = record
          pt : POINT;
          flags : UINT;
          hItem : HTREEITEM;
       end;
     LPTV_HITTESTINFO = ^TV_HITTESTINFO;
     _TVHITTESTINFO = TV_HITTESTINFO;
     TTVHITTESTINFO = TV_HITTESTINFO;
     PTVHITTESTINFO = ^TV_HITTESTINFO;

     TVITEMEX = record
          mask : UINT;
          hItem : HTREEITEM;
          state : UINT;
          stateMask : UINT;
          pszText : LPTSTR;
          cchTextMax : longint;
          iImage : longint;
          iSelectedImage : longint;
          cChildren : longint;
          lParam : LPARAM;
          iIntegral : longint;
       end;

     TV_INSERTSTRUCT = record
          hParent : HTREEITEM;
          hInsertAfter : HTREEITEM;
          case longint of
          0 : (item : TV_ITEM;);
          1 : (itemex : TVITEMEX;);
       end;
     LPTV_INSERTSTRUCT = ^TV_INSERTSTRUCT;
     _TV_INSERTSTRUCT = TV_INSERTSTRUCT;
     TTVINSERTSTRUCT = TV_INSERTSTRUCT;
     PTVINSERTSTRUCT = ^TV_INSERTSTRUCT;

     TV_KEYDOWN = record
          hdr : NMHDR;
          wVKey : WORD;
          flags : UINT;
       end;
     _TV_KEYDOWN = TV_KEYDOWN;
     TTVKEYDOWN = TV_KEYDOWN;
     PTVKEYDOWN = ^TV_KEYDOWN;

     TV_SORTCB = record
          hParent : HTREEITEM;
          lpfnCompare : PFNTVCOMPARE;
          lParam : LPARAM;
       end;
     LPTV_SORTCB = ^TV_SORTCB;
     _TV_SORTCB = TV_SORTCB;
     TTVSORTCB = TV_SORTCB;
     PTVSORTCB = ^TV_SORTCB;

     UDACCEL = record
          nSec : UINT;
          nInc : UINT;
       end;
     TUDACCEL = UDACCEL;
     PUDACCEL = ^UDACCEL;


     UNIVERSAL_NAME_INFO = record
          lpUniversalName : LPTSTR;
       end;
     _UNIVERSAL_NAME_INFO = UNIVERSAL_NAME_INFO;
     TUNIVERSALNAMEINFO = UNIVERSAL_NAME_INFO;
     PUNIVERSALNAMEINFO = ^UNIVERSAL_NAME_INFO;

     USEROBJECTFLAGS = record
          fInherit : WINBOOL;
          fReserved : WINBOOL;
          dwFlags : DWORD;
       end;
     tagUSEROBJECTFLAGS = USEROBJECTFLAGS;
     TUSEROBJECTFLAGS = USEROBJECTFLAGS;
     PUSEROBJECTFLAGS = ^USEROBJECTFLAGS;

     VALENTA = record
          ve_valuename : LPSTR;
          ve_valuelen : DWORD;
          ve_valueptr : DWORD_PTR;
          ve_type : DWORD;
       end;
     TVALENTA = VALENTA;
     PVALENTA = ^VALENTA;

     VALENTW = record
          ve_valuename : LPWSTR;
          ve_valuelen : DWORD;
          ve_valueptr : DWORD_PTR;
          ve_type : DWORD;
       end;
     TVALENTW = VALENTW;
     PVALENTW = ^VALENTW;

     {$ifdef Unicode}
     VALENT  = VALENTW;
     TVALENT = VALENTW;
     PVALENT = ^VALENTW;
     {$else}
     VALENT  = VALENTA;
     TVALENT = VALENTA;
     PVALENT = ^VALENTA;
     {$endif}

     value_ent = VALENT;
     Tvalue_ent = VALENT;
     Pvalue_ent = ^VALENT;

     VERIFY_INFORMATION = record
          StartingOffset : LARGE_INTEGER;
          Length : DWORD;
       end;
     _VERIFY_INFORMATION = VERIFY_INFORMATION;
     TVERIFYINFORMATION = VERIFY_INFORMATION;
     PVERIFYINFORMATION = ^VERIFY_INFORMATION;

     VS_FIXEDFILEINFO = record
          dwSignature : DWORD;
          dwStrucVersion : DWORD;
          dwFileVersionMS : DWORD;
          dwFileVersionLS : DWORD;
          dwProductVersionMS : DWORD;
          dwProductVersionLS : DWORD;
          dwFileFlagsMask : DWORD;
          dwFileFlags : DWORD;
          dwFileOS : DWORD;
          dwFileType : DWORD;
          dwFileSubtype : DWORD;
          dwFileDateMS : DWORD;
          dwFileDateLS : DWORD;
       end;
     _VS_FIXEDFILEINFO = VS_FIXEDFILEINFO;
     TVSFIXEDFILEINFO = VS_FIXEDFILEINFO;
     PVSFIXEDFILEINFO = ^VS_FIXEDFILEINFO;

     WIN32_FIND_DATAA = record
          dwFileAttributes : DWORD;
          ftCreationTime : FILETIME;
          ftLastAccessTime : FILETIME;
          ftLastWriteTime : FILETIME;
          nFileSizeHigh : DWORD;
          nFileSizeLow : DWORD;
          dwReserved0 : DWORD;
          dwReserved1 : DWORD;
          cFileName : array[0..(MAX_PATH)-1] of AnsiCHAR;
          cAlternateFileName : array[0..13] of AnsiCHAR;
       end;
     LPWIN32_FIND_DATAA = ^WIN32_FIND_DATAA;
     PWIN32_FIND_DATAA = ^WIN32_FIND_DATAA;
     _WIN32_FIND_DATAA = WIN32_FIND_DATAA;
     TWIN32FINDDATAA = WIN32_FIND_DATAA;
     WIN32FINDDATAA = WIN32_FIND_DATAA;
     PWIN32FINDDATAA = ^WIN32_FIND_DATAA;

     WIN32_FIND_DATAW = record
          dwFileAttributes : DWORD;
          ftCreationTime : FILETIME;
          ftLastAccessTime : FILETIME;
          ftLastWriteTime : FILETIME;
          nFileSizeHigh : DWORD;
          nFileSizeLow : DWORD;
          dwReserved0 : DWORD;
          dwReserved1 : DWORD;
          cFileName : array[0..(MAX_PATH)-1] of WCHAR;
          cAlternateFileName : array[0..13] of WCHAR;
       end;
     LPWIN32_FIND_DATAW = ^WIN32_FIND_DATAW;
     PWIN32_FIND_DATAW = ^WIN32_FIND_DATAW;
     _WIN32_FIND_DATAW = WIN32_FIND_DATAW;
     TWIN32FINDDATAW = WIN32_FIND_DATAW;
     PWIN32FINDDATAW = ^WIN32_FIND_DATAW;

     {$ifdef Unicode}
        LPWIN32_FIND_DATA = PWIN32_FIND_DATAW;
        PWIN32_FIND_DATA  = PWIN32_FIND_DATAW;
        _WIN32_FIND_DATA  = WIN32_FIND_DATAW;
        WIN32_FIND_DATA   = WIN32_FIND_DATAW;
        TWIN32FINDDATA    = WIN32_FIND_DATAW;
        WIN32FINDDATA     = WIN32_FIND_DATAW;
        PWIN32FINDDATA    = PWIN32_FIND_DATAW;
     {$else}
        LPWIN32_FIND_DATA = PWIN32_FIND_DATAA;
        PWIN32_FIND_DATA  = PWIN32_FIND_DATAA;
        _WIN32_FIND_DATA  = WIN32_FIND_DATAA;
        WIN32_FIND_DATA   = WIN32_FIND_DATAA;
        TWIN32FINDDATA    = WIN32_FIND_DATAA;
        WIN32FINDDATA     = WIN32_FIND_DATAA;
        PWIN32FINDDATA    = PWIN32_FIND_DATAA;
     {$endif}
     WIN32_STREAM_ID = record
          dwStreamId : DWORD;
          dwStreamAttributes : DWORD;
          Size : LARGE_INTEGER;
          dwStreamNameSize : DWORD;
          cStreamName : array[0..0] of WCHAR;  // really array[0..(dwStreamNameSize)] of WCHAR
       end;
     _WIN32_STREAM_ID = WIN32_STREAM_ID;
     TWIN32STREAMID = WIN32_STREAM_ID;
     PWIN32STREAMID = ^WIN32_STREAM_ID;

     WINDOWPLACEMENT = record
          length : UINT;
          flags : UINT;
          showCmd : UINT;
          ptMinPosition : POINT;
          ptMaxPosition : POINT;
          rcNormalPosition : RECT;
       end;
     _WINDOWPLACEMENT = WINDOWPLACEMENT;
     TWINDOWPLACEMENT = WINDOWPLACEMENT;
     PWINDOWPLACEMENT = ^WINDOWPLACEMENT;

     WNDCLASSA = record
          style : UINT;
          lpfnWndProc : WNDPROC;
          cbClsExtra : longint;
          cbWndExtra : longint;
          hInstance : HINST;
          hIcon : HICON;
          hCursor : HCURSOR;
          hbrBackground : HBRUSH;
          lpszMenuName : LPCSTR;
          lpszClassName : LPCSTR;
       end;
     LPWNDCLASSA = ^WNDCLASSA;
     _WNDCLASSA = WNDCLASSA;
     TWNDCLASSA = WNDCLASSA;
     PWNDCLASSA = ^WNDCLASSA;

     WNDCLASSW = record
          style : UINT;
          lpfnWndProc : WNDPROC;
          cbClsExtra : longint;
          cbWndExtra : longint;
          hInstance : HINST;
          hIcon : HICON;
          hCursor : HCURSOR;
          hbrBackground : HBRUSH;
          lpszMenuName : LPCWSTR;
          lpszClassName : LPCWSTR;
       end;
     LPWNDCLASSW = ^WNDCLASSW;
     _WNDCLASSW  = WNDCLASSW;
     TWNDCLASSW  = WNDCLASSW;
     PWNDCLASSW  = ^WNDCLASSW;
     {$ifdef UNICODE}
     LPWNDCLASS  = LPWNDCLASSW;
     _WNDCLASS   = _WNDCLASSW;
     TWNDCLASS   = TWNDCLASSW;
     PWNDCLASS   = PWNDCLASSW;
     WNDCLASS    = WNDCLASSW;
     {$else}
     LPWNDCLASS  = LPWNDCLASSA;
     _WNDCLASS   = _WNDCLASSA;
     TWNDCLASS   = TWNDCLASSA;
     PWNDCLASS   = PWNDCLASSA;
     WNDCLASS    = WNDCLASSA;
     {$endif}

     WNDCLASSEXA = record
          cbSize : UINT;
          style : UINT;
          lpfnWndProc : WNDPROC;
          cbClsExtra : longint;
          cbWndExtra : longint;
          hInstance : HINST;
          hIcon : HICON;
          hCursor : HCURSOR;
          hbrBackground : HBRUSH;
          lpszMenuName : LPCSTR;
          lpszClassName : LPCSTR;
          hIconSm : HANDLE;
       end;
     LPWNDCLASSEXA = ^WNDCLASSEXA;
     _WNDCLASSEXA = WNDCLASSEXA;
     TWNDCLASSEXA = WNDCLASSEXA;
     PWNDCLASSEXA = ^WNDCLASSEXA;

     WNDCLASSEXW = record
          cbSize : UINT;
          style : UINT;
          lpfnWndProc : WNDPROC;
          cbClsExtra : longint;
          cbWndExtra : longint;
          hInstance : HINST;
          hIcon : HICON;
          hCursor : HCURSOR;
          hbrBackground : HBRUSH;
          lpszMenuName : LPCWSTR;
          lpszClassName : LPCWSTR;
          hIconSm : HANDLE;
       end;
     LPWNDCLASSEXW = ^WNDCLASSEXW;
     _WNDCLASSEXW  = WNDCLASSEXW;
     TWNDCLASSEXW  = WNDCLASSEXW;
     PWNDCLASSEXW  = ^WNDCLASSEXW;

     {$ifdef UNICODE}
     LPWNDClassEx  = LPWNDClassExW;
     _WNDClassEx   = _WNDClassExW;
     TWNDClassEx   = TWNDClassExW;
     PWNDClassEx   = PWNDClassExW;
     WNDClassEx    = WNDClassExW;
     {$ELSE}
     LPWNDClassEx  = LPWNDClassExA;
     _WNDClassEx   = _WNDClassExA;
     TWNDClassEx   = TWNDClassExA;
     PWNDClassEx   = PWNDClassExA;
     WNDClassEx    = WNDClassExA;
     {$endif}

     CONNECTDLGSTRUCTA = record
          cbStructure : DWORD;
          hwndOwner   : HWND;
          lpConnRes   : LPNETRESOURCEA;
          dwFlags     : DWORD;
          dwDevNum    : DWORD;
       end;
     LPCONNECTDLGSTRUCTA = ^CONNECTDLGSTRUCTA;
     _CONNECTDLGSTRUCTA  = CONNECTDLGSTRUCTA;
     TCONNECTDLGSTRUCTA  = CONNECTDLGSTRUCTA;
     PCONNECTDLGSTRUCTA  = ^CONNECTDLGSTRUCTA;

     CONNECTDLGSTRUCTW = record
          cbStructure : DWORD;
          hwndOwner   : HWND;
          lpConnRes   : LPNETRESOURCEW;
          dwFlags     : DWORD;
          dwDevNum    : DWORD;
       end;
     LPCONNECTDLGSTRUCTW = ^CONNECTDLGSTRUCTW;
     _CONNECTDLGSTRUCTW = CONNECTDLGSTRUCTW;
     TCONNECTDLGSTRUCTW = CONNECTDLGSTRUCTW;
     PCONNECTDLGSTRUCTW = ^CONNECTDLGSTRUCTW;

     {$ifdef Unicode}
     LPCONNECTDLGSTRUCT = ^CONNECTDLGSTRUCTW;
     _CONNECTDLGSTRUCT  = CONNECTDLGSTRUCTW;
     CONNECTDLGSTRUCT   = CONNECTDLGSTRUCTW;
     TCONNECTDLGSTRUCT  = CONNECTDLGSTRUCTW;
     PCONNECTDLGSTRUCT  = ^CONNECTDLGSTRUCTW;
     {$else}
     LPCONNECTDLGSTRUCT = ^CONNECTDLGSTRUCTA;
     _CONNECTDLGSTRUCT  = CONNECTDLGSTRUCTA;
     CONNECTDLGSTRUCT   = CONNECTDLGSTRUCTA;
     TCONNECTDLGSTRUCT  = CONNECTDLGSTRUCTA;
     PCONNECTDLGSTRUCT  = ^CONNECTDLGSTRUCTA;
     {$endif}

     DISCDLGSTRUCTA = record
          cbStructure : DWORD;
          hwndOwner : HWND;
          lpLocalName : LPSTR;
          lpRemoteName : LPSTR;
          dwFlags : DWORD;
       end;
     LPDISCDLGSTRUCTA = ^DISCDLGSTRUCTA;
     _DISCDLGSTRUCTA  = DISCDLGSTRUCTA;
     TDISCDLGSTRUCTA  = DISCDLGSTRUCTA;
     PDISCDLGSTRUCTA  = ^DISCDLGSTRUCTA;

     DISCDLGSTRUCTW = record
          cbStructure : DWORD;
          hwndOwner : HWND;
          lpLocalName : LPWSTR;
          lpRemoteName : LPWSTR;
          dwFlags : DWORD;
       end;
     LPDISCDLGSTRUCTW = ^DISCDLGSTRUCTW;
     _DISCDLGSTRUCTW  = DISCDLGSTRUCTW;
     TDISCDLGSTRUCTW  = DISCDLGSTRUCTW;
     PDISCDLGSTRUCTW  = ^DISCDLGSTRUCTW;
     {$ifdef Unicode}
     LPDISCDLGSTRUCT = ^DISCDLGSTRUCTW;
     _DISCDLGSTRUCT  = DISCDLGSTRUCTW;
     TDISCDLGSTRUCT  = DISCDLGSTRUCTW;
     DISCDLGSTRUCT   = DISCDLGSTRUCTW;
     PDISCDLGSTRUCT  = ^DISCDLGSTRUCTW;
     {$else}
     LPDISCDLGSTRUCT = ^DISCDLGSTRUCTA;
     _DISCDLGSTRUCT  = DISCDLGSTRUCTA;
     TDISCDLGSTRUCT  = DISCDLGSTRUCTA;
     DISCDLGSTRUCT   = DISCDLGSTRUCTA;
     PDISCDLGSTRUCT  = ^DISCDLGSTRUCTA;
     {$endif}

     NETINFOSTRUCT = record
          cbStructure : DWORD;
          dwProviderVersion : DWORD;
          dwStatus : DWORD;
          dwCharacteristics : DWORD;
          dwHandle : ULONG_PTR;
          wNetType : WORD;
          dwPrinters : DWORD;
          dwDrives : DWORD;
       end;
     LPNETINFOSTRUCT = ^NETINFOSTRUCT;
     _NETINFOSTRUCT = NETINFOSTRUCT;
     TNETINFOSTRUCT = NETINFOSTRUCT;
     PNETINFOSTRUCT = ^NETINFOSTRUCT;

     NETCONNECTINFOSTRUCT = record
          cbStructure : DWORD;
          dwFlags : DWORD;
          dwSpeed : DWORD;
          dwDelay : DWORD;
          dwOptDataSize : DWORD;
       end;
     LPNETCONNECTINFOSTRUCT = ^NETCONNECTINFOSTRUCT;
     _NETCONNECTINFOSTRUCT = NETCONNECTINFOSTRUCT;
     TNETCONNECTINFOSTRUCT = NETCONNECTINFOSTRUCT;
     PNETCONNECTINFOSTRUCT = ^NETCONNECTINFOSTRUCT;

     ENUMMETAFILEPROC = function (_para1:HDC; _para2:LPHANDLETABLE; _para3:LPMETARECORD; _para4:longint; _para5:LPARAM):longint;stdcall;

     ENHMETAFILEPROC = function (_para1:HDC; _para2:LPHANDLETABLE; _para3:LPENHMETARECORD; _para4:longint; _para5:LPARAM):longint;stdcall;

     ENUMFONTSPROC = function (_para1:LPLOGFONT; _para2:LPTEXTMETRIC; _para3:DWORD; _para4:LPARAM):longint;stdcall;
     ENUMFONTSPROCA= function (_para1:LPLOGFONTA; _para2:LPTEXTMETRICA; _para3:DWORD; _para4:LPARAM):longint;stdcall;
     ENUMFONTSPROCW= function (_para1:LPLOGFONTW; _para2:LPTEXTMETRICW; _para3:DWORD; _para4:LPARAM):longint;stdcall;
     ENUMFONTSEXPROC = function (_para1:PENUMLOGFONTEX;  _para2:LPNEWTEXTMETRICEX; _para3:DWORD; _para4:LPARAM):longint;stdcall;
     ENUMFONTSEXPROCA= function (_para1:PENUMLOGFONTEXA; _para2:LPNEWTEXTMETRICEXA; _para3:DWORD; _para4:LPARAM):longint;stdcall;
     ENUMFONTSEXPROCW= function (_para1:PENUMLOGFONTEXW; _para2:LPNEWTEXTMETRICEXW; _para3:DWORD; _para4:LPARAM):longint;stdcall;

     FONTENUMPROC   = function (var _para1:ENUMLOGFONT; var _para2:NEWTEXTMETRIC; _para3:longint; _para4:LPARAM):longint;stdcall;
     FONTENUMPROCA  = function (var _para1:ENUMLOGFONTA; var _para2:NEWTEXTMETRICA; _para3:longint; _para4:LPARAM):longint;stdcall;
     FONTENUMPROCW  = function (var _para1:ENUMLOGFONTW; var _para2:NEWTEXTMETRICW; _para3:longint; _para4:LPARAM):longint;stdcall;
     FONTENUMEXPROC = function (var _para1:ENUMLOGFONTEX;var _para2:NEWTEXTMETRICEX; _para3:longint; _para4:LPARAM):longint;stdcall;
     FONTENUMEXPROCA= function (var _para1:ENUMLOGFONTEXA;var _para2:NEWTEXTMETRICEXA; _para3:longint; _para4:LPARAM):longint;stdcall;
     FONTENUMEXPROCW= function (var _para1:ENUMLOGFONTEXW;var _para2:NEWTEXTMETRICEXW; _para3:longint; _para4:LPARAM):longint;stdcall;

     LPOVERLAPPED_COMPLETION_ROUTINE = procedure (_para1:DWORD; _para2:DWORD; _para3:LPOVERLAPPED);stdcall;

     { Structures for the extensions to OpenGL }

     POINTFLOAT = record
          x : Single;
          y : Single;
       end;
     _POINTFLOAT = POINTFLOAT;
     TPOINTFLOAT = POINTFLOAT;
     PPOINTFLOAT = ^POINTFLOAT;

     GLYPHMETRICSFLOAT = record
          gmfBlackBoxX : Single;
          gmfBlackBoxY : Single;
          gmfptGlyphOrigin : POINTFLOAT;
          gmfCellIncX : Single;
          gmfCellIncY : Single;
       end;
     LPGLYPHMETRICSFLOAT = ^GLYPHMETRICSFLOAT;
     _GLYPHMETRICSFLOAT = GLYPHMETRICSFLOAT;
     TGLYPHMETRICSFLOAT = GLYPHMETRICSFLOAT;
     PGLYPHMETRICSFLOAT = ^GLYPHMETRICSFLOAT;

     LAYERPLANEDESCRIPTOR = record
          nSize : WORD;
          nVersion : WORD;
          dwFlags : DWORD;
          iPixelType : BYTE;
          cColorBits : BYTE;
          cRedBits : BYTE;
          cRedShift : BYTE;
          cGreenBits : BYTE;
          cGreenShift : BYTE;
          cBlueBits : BYTE;
          cBlueShift : BYTE;
          cAlphaBits : BYTE;
          cAlphaShift : BYTE;
          cAccumBits : BYTE;
          cAccumRedBits : BYTE;
          cAccumGreenBits : BYTE;
          cAccumBlueBits : BYTE;
          cAccumAlphaBits : BYTE;
          cDepthBits : BYTE;
          cStencilBits : BYTE;
          cAuxBuffers : BYTE;
          iLayerPlane : BYTE;
          bReserved : BYTE;
          crTransparent : COLORREF;
       end;
     LPLAYERPLANEDESCRIPTOR = ^LAYERPLANEDESCRIPTOR;
     tagLAYERPLANEDESCRIPTOR = LAYERPLANEDESCRIPTOR;
     TLAYERPLANEDESCRIPTOR = LAYERPLANEDESCRIPTOR;
     PLAYERPLANEDESCRIPTOR = ^LAYERPLANEDESCRIPTOR;

     PIXELFORMATDESCRIPTOR = record
          nSize : WORD;
          nVersion : WORD;
          dwFlags : DWORD;
          iPixelType : BYTE;
          cColorBits : BYTE;
          cRedBits : BYTE;
          cRedShift : BYTE;
          cGreenBits : BYTE;
          cGreenShift : BYTE;
          cBlueBits : BYTE;
          cBlueShift : BYTE;
          cAlphaBits : BYTE;
          cAlphaShift : BYTE;
          cAccumBits : BYTE;
          cAccumRedBits : BYTE;
          cAccumGreenBits : BYTE;
          cAccumBlueBits : BYTE;
          cAccumAlphaBits : BYTE;
          cDepthBits : BYTE;
          cStencilBits : BYTE;
          cAuxBuffers : BYTE;
          iLayerType : BYTE;
          bReserved : BYTE;
          dwLayerMask : DWORD;
          dwVisibleMask : DWORD;
          dwDamageMask : DWORD;
       end;
     LPPIXELFORMATDESCRIPTOR = ^PIXELFORMATDESCRIPTOR;
     tagPIXELFORMATDESCRIPTOR = PIXELFORMATDESCRIPTOR;
     TPIXELFORMATDESCRIPTOR = PIXELFORMATDESCRIPTOR;
     PPIXELFORMATDESCRIPTOR = ^PIXELFORMATDESCRIPTOR;

     USER_INFO_2 = record
          usri2_name : LPWSTR;
          usri2_password : LPWSTR;
          usri2_password_age : DWORD;
          usri2_priv : DWORD;
          usri2_home_dir : LPWSTR;
          usri2_comment : LPWSTR;
          usri2_flags : DWORD;
          usri2_script_path : LPWSTR;
          usri2_auth_flags : DWORD;
          usri2_full_name : LPWSTR;
          usri2_usr_comment : LPWSTR;
          usri2_parms : LPWSTR;
          usri2_workstations : LPWSTR;
          usri2_last_logon : DWORD;
          usri2_last_logoff : DWORD;
          usri2_acct_expires : DWORD;
          usri2_max_storage : DWORD;
          usri2_units_per_week : DWORD;
          usri2_logon_hours : PBYTE;
          usri2_bad_pw_count : DWORD;
          usri2_num_logons : DWORD;
          usri2_logon_server : LPWSTR;
          usri2_country_code : DWORD;
          usri2_code_page : DWORD;
       end;
     PUSER_INFO_2 = ^USER_INFO_2;
     LPUSER_INFO_2 = ^USER_INFO_2;
     TUSERINFO2 = USER_INFO_2;
     PUSERINFO2 = ^USER_INFO_2;

     USER_INFO_0 = record
          usri0_name : LPWSTR;
       end;
     PUSER_INFO_0 = ^USER_INFO_0;
     LPUSER_INFO_0 = ^USER_INFO_0;
     TUSERINFO0 = USER_INFO_0;
     PUSERINFO0 = ^USER_INFO_0;

     USER_INFO_3 = record
          usri3_name : LPWSTR;
          usri3_password : LPWSTR;
          usri3_password_age : DWORD;
          usri3_priv : DWORD;
          usri3_home_dir : LPWSTR;
          usri3_comment : LPWSTR;
          usri3_flags : DWORD;
          usri3_script_path : LPWSTR;
          usri3_auth_flags : DWORD;
          usri3_full_name : LPWSTR;
          usri3_usr_comment : LPWSTR;
          usri3_parms : LPWSTR;
          usri3_workstations : LPWSTR;
          usri3_last_logon : DWORD;
          usri3_last_logoff : DWORD;
          usri3_acct_expires : DWORD;
          usri3_max_storage : DWORD;
          usri3_units_per_week : DWORD;
          usri3_logon_hours : PBYTE;
          usri3_bad_pw_count : DWORD;
          usri3_num_logons : DWORD;
          usri3_logon_server : LPWSTR;
          usri3_country_code : DWORD;
          usri3_code_page : DWORD;
          usri3_user_id : DWORD;
          usri3_primary_group_id : DWORD;
          usri3_profile : LPWSTR;
          usri3_home_dir_drive : LPWSTR;
          usri3_password_expired : DWORD;
       end;
     PUSER_INFO_3 = ^USER_INFO_3;
     LPUSER_INFO_3 = ^USER_INFO_3;
     TUSERINFO3 = USER_INFO_3;
     PUSERINFO3 = ^USER_INFO_3;

     GROUP_INFO_2 = record
          grpi2_name : LPWSTR;
          grpi2_comment : LPWSTR;
          grpi2_group_id : DWORD;
          grpi2_attributes : DWORD;
       end;
     PGROUP_INFO_2 = ^GROUP_INFO_2;
     TGROUPINFO2 = GROUP_INFO_2;
     PGROUPINFO2 = ^GROUP_INFO_2;

     LOCALGROUP_INFO_0 = record
          lgrpi0_name : LPWSTR;
       end;
     PLOCALGROUP_INFO_0 = ^LOCALGROUP_INFO_0;
     LPLOCALGROUP_INFO_0 = ^LOCALGROUP_INFO_0;
     TLOCALGROUPINFO0 = LOCALGROUP_INFO_0;
     PLOCALGROUPINFO0 = ^LOCALGROUP_INFO_0;

  { PE executable header.   }
  { Magic number, 0x5a4d  }
  { Bytes on last page of file, 0x90  }
  { Pages in file, 0x3  }
  { Relocations, 0x0  }
  { Size of header in paragraphs, 0x4  }
  { Minimum extra paragraphs needed, 0x0  }
  { Maximum extra paragraphs needed, 0xFFFF  }
  { Initial (relative) SS value, 0x0  }
  { Initial SP value, 0xb8  }
  { Checksum, 0x0  }
  { Initial IP value, 0x0  }
  { Initial (relative) CS value, 0x0  }
  { File address of relocation table, 0x40  }
  { Overlay number, 0x0  }
  { Reserved words, all 0x0  }
  { OEM identifier (for e_oeminfo), 0x0  }
  { OEM information; e_oemid specific, 0x0  }
  { Reserved words, all 0x0  }
  { File address of new exe header, 0x80  }
  { We leave out the next two fields, since they aren't in the header file }
  { DWORD dos_message[16];   text which always follows dos header  }
  { DWORD nt_signature;      required NT signature, 0x4550  }

     IMAGE_DOS_HEADER = record
          e_magic : WORD;
          e_cblp : WORD;
          e_cp : WORD;
          e_crlc : WORD;
          e_cparhdr : WORD;
          e_minalloc : WORD;
          e_maxalloc : WORD;
          e_ss : WORD;
          e_sp : WORD;
          e_csum : WORD;
          e_ip : WORD;
          e_cs : WORD;
          e_lfarlc : WORD;
          e_ovno : WORD;
          e_res : array[0..3] of WORD;
          e_oemid : WORD;
          e_oeminfo : WORD;
          e_res2 : array[0..9] of WORD;
          case boolean of
             true : (e_lfanew : LONG);
             false: (_lfanew : LONG); // delphi naming
       end;
     PIMAGE_DOS_HEADER = ^IMAGE_DOS_HEADER;
     TIMAGE_DOS_HEADER = IMAGE_DOS_HEADER;
     TIMAGEDOSHEADER = IMAGE_DOS_HEADER;
     PIMAGEDOSHEADER = ^IMAGE_DOS_HEADER;

      _NOTIFYICONDATAA = record
          cbSize: DWORD;
          Wnd: HWND;
          uID: UINT;
          uFlags: UINT;
          uCallbackMessage: UINT;
          hIcon: HICON;
          szTip: array [0..127] of AnsiChar;
          dwState : DWORD;
          dwStateMask : DWORD;
          szInfo : array[0..255] of AnsiChar;
          case longint of
            0 : (uTimeout : UINT;);
            1 : (uVersion : UINT;
                 szInfoTitle : array[0..63] of AnsiChar;
                 dwInfoFlags : DWORD;
                 //guidItem : TGUID;     // #if (_WIN32_IE >= 0x600)}
                );
     end;
     TNotifyIconDataA = _NOTIFYICONDATAA;
     NOTIFYICONDATAA = _NOTIFYICONDATAA;
     PNotifyIconDataA = ^TNotifyIconDataA;

     _NOTIFYICONDATAW = record
         cbSize: DWORD;
         Wnd: HWND;
         uID: UINT;
         uFlags: UINT;
         uCallbackMessage: UINT;
         hIcon: HICON;
         szTip: array [0..127] of WCHAR;
         dwState : DWORD;
         dwStateMask : DWORD;
         szInfo : array[0..255] of WCHAR;
         case longint of
           0 : (uTimeout : UINT;);
           1 : (uVersion : UINT;
                szInfoTitle : array[0..63] of WCHAR;
                dwInfoFlags : DWORD;
                //guidItem : TGUID;     // #if (_WIN32_IE >= 0x600)}
               );
     end;
     TNotifyIconDataW = _NOTIFYICONDATAW;
     NOTIFYICONDATAW = _NOTIFYICONDATAW;
     PNotifyIconDataW = ^TNotifyIconDataW;

     {$ifdef Unicode}
     _NOTIFYICONDATA = _NOTIFYICONDATAW;
     TNotifyIconData = TNotifyIconDataW;
     NOTIFYICONDATA  = NOTIFYICONDATAW;
     PNotifyIconData = PNotifyIconDataW;
     {$else}
     _NOTIFYICONDATA = _NOTIFYICONDATAA;
     TNotifyIconData = TNotifyIconDataA;
     NOTIFYICONDATA  = NOTIFYICONDATAA;
     PNotifyIconData = PNotifyIconDataA;
     {$endif}

     TWOHandleArray = array[0..MAXIMUM_WAIT_OBJECTS-1] of HANDLE;
     PWOHandleArray = ^TWOHandleArray;
     MMRESULT = Longint;

type
  PWaveFormatEx = ^TWaveFormatEx;
  TWaveFormatEx = packed record
    wFormatTag: Word;       { format type }
    nChannels: Word;        { number of channels (i.e. mono, stereo, etc.) }
    nSamplesPerSec: DWORD;  { sample rate }
    nAvgBytesPerSec: DWORD; { for buffer estimation }
    nBlockAlign: Word;      { block size of data }
    wBitsPerSample: Word;   { number of bits per sample of mono data }
    cbSize: Word;           { the count in bytes of the size of }
  end;

  _WIN32_FILE_ATTRIBUTE_DATA = packed record
    dwFileAttributes:     DWORD;
    ftCreationTime:       FILETIME;
    ftLastAccessTime:     FILETIME;
    ftLastWriteTime:      FILETIME;
    nFileSizeHigh:        DWORD;
    nFileSizeLow:         DWORD;
    end;
  WIN32_FILE_ATTRIBUTE_DATA   = _WIN32_FILE_ATTRIBUTE_DATA ;
  LPWIN32_FILE_ATTRIBUTE_DATA = ^_WIN32_FILE_ATTRIBUTE_DATA;
  TWIN32FILEATTRIBUTEDATA     = _WIN32_FILE_ATTRIBUTE_DATA ;
  PWIN32FILEATTRIBUTEDATA     = ^_WIN32_FILE_ATTRIBUTE_DATA;

  // TrackMouseEvent. NT or higher only.
  TTrackMouseEvent = Record
    cbSize : DWORD;
    dwFlags : DWORD;
    hwndTrack : HWND;
    dwHoverTime : DWORD;
  end;
  PTrackMouseEvent = ^TTrackMouseEvent;


// File header format.
//

  PIMAGE_FILE_HEADER = ^IMAGE_FILE_HEADER;
  _IMAGE_FILE_HEADER = record
    Machine: WORD;
    NumberOfSections: WORD;
    TimeDateStamp: DWORD;
    PointerToSymbolTable: DWORD;
    NumberOfSymbols: DWORD;
    SizeOfOptionalHeader: WORD;
    Characteristics: WORD;
  end;
  IMAGE_FILE_HEADER = _IMAGE_FILE_HEADER;
  TImageFileHeader = IMAGE_FILE_HEADER;
  PImageFileHeader = PIMAGE_FILE_HEADER;


//
// Debug Format
//

  PIMAGE_DEBUG_DIRECTORY = ^IMAGE_DEBUG_DIRECTORY;
  _IMAGE_DEBUG_DIRECTORY = record
    Characteristics: DWORD;
    TimeDateStamp: DWORD;
    MajorVersion: Word;
    MinorVersion: Word;
    Type_: DWORD;
    SizeOfData: DWORD;
    AddressOfRawData: DWORD;
    PointerToRawData: DWORD;
  end;
  IMAGE_DEBUG_DIRECTORY = _IMAGE_DEBUG_DIRECTORY;
  TImageDebugDirectory = IMAGE_DEBUG_DIRECTORY;
  PImageDebugDirectory = PIMAGE_DEBUG_DIRECTORY;

//
// Optional header format.
//


  PIMAGE_DATA_DIRECTORY = ^IMAGE_DATA_DIRECTORY;
  _IMAGE_DATA_DIRECTORY = record
    VirtualAddress: DWORD;
    Size: DWORD;
  end;
  IMAGE_DATA_DIRECTORY = _IMAGE_DATA_DIRECTORY;
  TIMAGE_DATA_DIRECTORY = _IMAGE_DATA_DIRECTORY;
  TImageDataDirectory = IMAGE_DATA_DIRECTORY;
  PImageDataDirectory = PIMAGE_DATA_DIRECTORY;

const
  IMAGE_NUMBEROF_DIRECTORY_ENTRIES = 16;

type
  PIMAGE_OPTIONAL_HEADER32 = ^IMAGE_OPTIONAL_HEADER32;
  _IMAGE_OPTIONAL_HEADER = record
    //
    // Standard fields.
    //
    Magic: Word;
    MajorLinkerVersion: Byte;
    MinorLinkerVersion: Byte;
    SizeOfCode: DWORD;
    SizeOfInitializedData: DWORD;
    SizeOfUninitializedData: DWORD;
    AddressOfEntryPoint: DWORD;
    BaseOfCode: DWORD;
    BaseOfData: DWORD;
    //
    // NT additional fields.
    //
    ImageBase: DWORD;
    SectionAlignment: DWORD;
    FileAlignment: DWORD;
    MajorOperatingSystemVersion: Word;
    MinorOperatingSystemVersion: Word;
    MajorImageVersion: Word;
    MinorImageVersion: Word;
    MajorSubsystemVersion: Word;
    MinorSubsystemVersion: Word;
    Win32VersionValue: DWORD;
    SizeOfImage: DWORD;
    SizeOfHeaders: DWORD;
    CheckSum: DWORD;
    Subsystem: Word;
    DllCharacteristics: Word;
    SizeOfStackReserve: DWORD;
    SizeOfStackCommit: DWORD;
    SizeOfHeapReserve: DWORD;
    SizeOfHeapCommit: DWORD;
    LoaderFlags: DWORD;
    NumberOfRvaAndSizes: DWORD;
    DataDirectory: array [0..IMAGE_NUMBEROF_DIRECTORY_ENTRIES - 1] of IMAGE_DATA_DIRECTORY;
  end;
  IMAGE_OPTIONAL_HEADER32 = _IMAGE_OPTIONAL_HEADER;
  TImageOptionalHeader32 = IMAGE_OPTIONAL_HEADER32;
  PImageOptionalHeader32 = PIMAGE_OPTIONAL_HEADER32;

  PIMAGE_ROM_OPTIONAL_HEADER = ^IMAGE_ROM_OPTIONAL_HEADER;
  _IMAGE_ROM_OPTIONAL_HEADER = record
    Magic: Word;
    MajorLinkerVersion: Byte;
    MinorLinkerVersion: Byte;
    SizeOfCode: DWORD;
    SizeOfInitializedData: DWORD;
    SizeOfUninitializedData: DWORD;
    AddressOfEntryPoint: DWORD;
    BaseOfCode: DWORD;
    BaseOfData: DWORD;
    BaseOfBss: DWORD;
    GprMask: DWORD;
    CprMask: array [0..3] of DWORD;
    GpValue: DWORD;
  end;
  IMAGE_ROM_OPTIONAL_HEADER = _IMAGE_ROM_OPTIONAL_HEADER;
  TIMAGE_ROM_OPTIONAL_HEADER = _IMAGE_ROM_OPTIONAL_HEADER;
  TImageRomOptionalHeader = IMAGE_ROM_OPTIONAL_HEADER;
  PImageRomOptionalHeader = PIMAGE_ROM_OPTIONAL_HEADER;

  PIMAGE_OPTIONAL_HEADER64 = ^IMAGE_OPTIONAL_HEADER64;
  _IMAGE_OPTIONAL_HEADER64 = record
    Magic: Word;
    MajorLinkerVersion: Byte;
    MinorLinkerVersion: Byte;
    SizeOfCode: DWORD;
    SizeOfInitializedData: DWORD;
    SizeOfUninitializedData: DWORD;
    AddressOfEntryPoint: DWORD;
    BaseOfCode: DWORD;
    ImageBase: Int64;
    SectionAlignment: DWORD;
    FileAlignment: DWORD;
    MajorOperatingSystemVersion: Word;
    MinorOperatingSystemVersion: Word;
    MajorImageVersion: Word;
    MinorImageVersion: Word;
    MajorSubsystemVersion: Word;
    MinorSubsystemVersion: Word;
    Win32VersionValue: DWORD;
    SizeOfImage: DWORD;
    SizeOfHeaders: DWORD;
    CheckSum: DWORD;
    Subsystem: Word;
    DllCharacteristics: Word;
    SizeOfStackReserve: Int64;
    SizeOfStackCommit: Int64;
    SizeOfHeapReserve: Int64;
    SizeOfHeapCommit: Int64;
    LoaderFlags: DWORD;
    NumberOfRvaAndSizes: DWORD;
    DataDirectory: array [0..IMAGE_NUMBEROF_DIRECTORY_ENTRIES - 1] of IMAGE_DATA_DIRECTORY;
  end;
  IMAGE_OPTIONAL_HEADER64 = _IMAGE_OPTIONAL_HEADER64;
  TImageOptionalHeader64 = IMAGE_OPTIONAL_HEADER64;
  PImageOptionalHeader64 = PIMAGE_OPTIONAL_HEADER64;

const
  IMAGE_SIZEOF_ROM_OPTIONAL_HEADER  = 56;
  IMAGE_SIZEOF_STD_OPTIONAL_HEADER  = 28;
  IMAGE_SIZEOF_NT_OPTIONAL32_HEADER = 224;
  IMAGE_SIZEOF_NT_OPTIONAL64_HEADER = 240;

  IMAGE_NT_OPTIONAL_HDR32_MAGIC = $10b;
  IMAGE_NT_OPTIONAL_HDR64_MAGIC = $20b;
  IMAGE_ROM_OPTIONAL_HDR_MAGIC  = $107;

type
{$ifdef WIN64}
  IMAGE_OPTIONAL_HEADER = IMAGE_OPTIONAL_HEADER64;
  PIMAGE_OPTIONAL_HEADER = PIMAGE_OPTIONAL_HEADER64;
{$else}
  IMAGE_OPTIONAL_HEADER = IMAGE_OPTIONAL_HEADER32;
  PIMAGE_OPTIONAL_HEADER = PIMAGE_OPTIONAL_HEADER32;
{$endif}
  TImageOptionalHeader = IMAGE_OPTIONAL_HEADER;
  PImageOptionalHeader = PIMAGE_OPTIONAL_HEADER;

const
  IMAGE_SIZEOF_NT_OPTIONAL_HEADER = IMAGE_SIZEOF_NT_OPTIONAL32_HEADER;
  IMAGE_NT_OPTIONAL_HDR_MAGIC     = IMAGE_NT_OPTIONAL_HDR32_MAGIC;

type
  PIMAGE_NT_HEADERS64 = ^IMAGE_NT_HEADERS64;
  _IMAGE_NT_HEADERS64 = record
    Signature: DWORD;
    FileHeader: IMAGE_FILE_HEADER;
    OptionalHeader: IMAGE_OPTIONAL_HEADER64;
  end;
  IMAGE_NT_HEADERS64 = _IMAGE_NT_HEADERS64;
  TImageNtHeaders64 = IMAGE_NT_HEADERS64;
  PImageNtHeaders64 = PIMAGE_NT_HEADERS64;

  PIMAGE_NT_HEADERS32 = ^IMAGE_NT_HEADERS32;
  _IMAGE_NT_HEADERS = record
    Signature: DWORD;
    FileHeader: IMAGE_FILE_HEADER;
    OptionalHeader: IMAGE_OPTIONAL_HEADER32;
  end;
  IMAGE_NT_HEADERS32 = _IMAGE_NT_HEADERS;
  TImageNtHeaders32 = IMAGE_NT_HEADERS32;
  PImageNtHeaders32 = PIMAGE_NT_HEADERS32;

  PIMAGE_ROM_HEADERS = ^IMAGE_ROM_HEADERS;
  _IMAGE_ROM_HEADERS = record
    FileHeader: IMAGE_FILE_HEADER;
    OptionalHeader: IMAGE_ROM_OPTIONAL_HEADER;
  end;
  IMAGE_ROM_HEADERS = _IMAGE_ROM_HEADERS;
  TImageRomHeaders = IMAGE_ROM_HEADERS;
  PImageRomHeaders = PIMAGE_ROM_HEADERS;

{$ifdef WIN64}
  IMAGE_NT_HEADERS = IMAGE_NT_HEADERS64;
  PIMAGE_NT_HEADERS = PIMAGE_NT_HEADERS64;
{$else}
  IMAGE_NT_HEADERS = IMAGE_NT_HEADERS32;
  PIMAGE_NT_HEADERS = PIMAGE_NT_HEADERS32;
{$endif}

  TImageNtHeaders = IMAGE_NT_HEADERS;
  PImageNtHeaders = PIMAGE_NT_HEADERS;

  _GET_FILEEX_INFO_LEVELS = (GetFileExInfoStandard, GetFileExMaxInfoLevel);
  GET_FILEEX_INFO_LEVELS = _GET_FILEEX_INFO_LEVELS;
  TGetFileExInfoLevels = GET_FILEEX_INFO_LEVELS;
  TGet_FileEx_Info_Levels = GET_FILEEX_INFO_LEVELS;

  tagBSTRBLOB = record
                  cbsize : ULONG;
		  pdata  : pbyte;
                  end;
  BSTRBLOB=TagBSTRBlob;
  TBSTRBLOB=BSTRBLOB;
  PBSTRBLOB=^BSTRBLOB;

  tagCLIPDATA = record
                  cbsize : ULONG;
		  ulClipFmt : long;
		  pclipdata : pbyte;
                  end;
  CLIPDATA=TagCLIPDATA;
  TCLIPDATA=CLIPDATA;
  PCLIPDATA=^CLIPDATA;

   TImage_Section_SubHeader= record
            case longint of
               0 : ( PhysicalAddress : DWORD );
               1 : ( VirtualSize : DWORD );
            end;

   _IMAGE_SECTION_HEADER = record
        Name : array[0..(IMAGE_SIZEOF_SHORT_NAME)-1] of BYTE;
        Misc : TImage_Section_SubHeader;
        VirtualAddress : DWORD;
        SizeOfRawData : DWORD;
        PointerToRawData : DWORD;
        PointerToRelocations : DWORD;
        PointerToLinenumbers : DWORD;
        NumberOfRelocations : WORD;
        NumberOfLinenumbers : WORD;
        Characteristics : DWORD;
     end;
   IMAGE_SECTION_HEADER = _IMAGE_SECTION_HEADER;
   TIMAGE_SECTION_HEADER = _IMAGE_SECTION_HEADER;
   PIMAGE_SECTION_HEADER = ^_IMAGE_SECTION_HEADER;
   PPIMAGE_SECTION_HEADER = ^PIMAGE_SECTION_HEADER;
   IMAGESECTIONHEADER = _IMAGE_SECTION_HEADER;
   TIMAGESECTIONHEADER = _IMAGE_SECTION_HEADER;
   PIMAGESECTIONHEADER = ^_IMAGE_SECTION_HEADER;


   _IMAGE_FUNCTION_ENTRY = record
      StartingAddress,
      EndingAddress,
      EndOfPrologue     : DWord;
      end;
   IMAGE_FUNCTION_ENTRY = _IMAGE_FUNCTION_ENTRY;
   TIMAGE_FUNCTION_ENTRY= IMAGE_FUNCTION_ENTRY;
   PIMAGE_FUNCTION_ENTRY= ^IMAGE_FUNCTION_ENTRY;
   LPIMAGE_FUNCTION_ENTRY= PIMAGE_FUNCTION_ENTRY;


   _IMAGE_FUNCTION_ENTRY64 = record
       StartingAddress,
       EndingAddress :   ULONGLONG   ;
       case boolean of
         false : (EndOfPrologue : ULONGLONG);
         true  : (UnwindInfoAddress : ULONGLONG);
       end;
   IMAGE_FUNCTION_ENTRY64  =  _IMAGE_FUNCTION_ENTRY64;
   TIMAGE_FUNCTION_ENTRY64 =  _IMAGE_FUNCTION_ENTRY64;
   PIMAGE_FUNCTION_ENTRY64 =  ^_IMAGE_FUNCTION_ENTRY64;
   LPIMAGE_FUNCTION_ENTRY64=  ^_IMAGE_FUNCTION_ENTRY64;

   _IMAGE_COFF_SYMBOLS_HEADER  = record
      NumberOfSymbols,
      LvaToFirstSymbol,
      NumberOfLinenumbers,
      LvaToFirstLinenumber,
      RvaToFirstByteOfCode,
      RvaToLastByteOfCode,
      RvaToFirstByteOfData,
      RvaToLastByteOfData    : DWORD;
    end;
   TIMAGE_COFF_SYMBOLS_HEADER = _IMAGE_COFF_SYMBOLS_HEADER;
   IMAGE_COFF_SYMBOLS_HEADER  = _IMAGE_COFF_SYMBOLS_HEADER;
   PIMAGE_COFF_SYMBOLS_HEADER = ^IMAGE_COFF_SYMBOLS_HEADER;
   LPIMAGE_COFF_SYMBOLS_HEADER= PIMAGE_COFF_SYMBOLS_HEADER;


   _FPO_DATA = record
    ulOffStart: DWORD;             // offset 1st byte of function code
    cbProcSize: DWORD;             // # bytes in function
    cdwLocals : DWORD;             // # bytes in locals/4
    bitvalues : word;              //
{
    WORD        cdwParams;              // # bytes in params/4
    WORD        cbProlog : 8;           // # bytes in prolog
    WORD        cbRegs   : 3;           // # regs saved
    WORD        fHasSEH  : 1;           // TRUE if SEH in func
    WORD        fUseBP   : 1;           // TRUE if EBP has been allocated
    WORD        reserved : 1;           // reserved for future use
    WORD        cbFrame  : 2;           // frame type
}
    end;
   FPO_DATA   = _FPO_DATA;
   TFPO_DATA  = _FPO_DATA;
   PFPO_DATA  = ^_FPO_DATA;
   LPFPO_DATA = PFPO_DATA;


     IMAGE_LOAD_CONFIG_DIRECTORY32 = record
          Size : DWORD;
          TimeDateStamp : DWORD;
          MajorVersion : WORD;
          MinorVersion : WORD;
          GlobalFlagsClear : DWORD;
          GlobalFlagsSet : DWORD;
          CriticalSectionDefaultTimeout : DWORD;
          DeCommitFreeBlockThreshold : DWORD;
          DeCommitTotalFreeThreshold : DWORD;
          LockPrefixTable : DWORD;
          MaximumAllocationSize : DWORD;
          VirtualMemoryThreshold : DWORD;
          ProcessHeapFlags : DWORD;
          ProcessAffinityMask : DWORD;
          CSDVersion : WORD;
          Reserved1 : WORD;
          EditList : DWORD;
          SecurityCookie : DWORD;
          SEHandlerTable : DWORD;
          SEHandlerCount : DWORD;
       end;
     PIMAGE_LOAD_CONFIG_DIRECTORY32 = ^IMAGE_LOAD_CONFIG_DIRECTORY32;
     TIMAGE_LOAD_CONFIG_DIRECTORY32 = IMAGE_LOAD_CONFIG_DIRECTORY32;
     IMAGE_LOAD_CONFIG_DIRECTORY64 = record
          Size : DWORD;
          TimeDateStamp : DWORD;
          MajorVersion : WORD;
          MinorVersion : WORD;
          GlobalFlagsClear : DWORD;
          GlobalFlagsSet : DWORD;
          CriticalSectionDefaultTimeout : DWORD;
          DeCommitFreeBlockThreshold : ULONGLONG;
          DeCommitTotalFreeThreshold : ULONGLONG;
          LockPrefixTable : ULONGLONG;
          MaximumAllocationSize : ULONGLONG;
          VirtualMemoryThreshold : ULONGLONG;
          ProcessAffinityMask : ULONGLONG;
          ProcessHeapFlags : DWORD;
          CSDVersion : WORD;
          Reserved1 : WORD;
          EditList : ULONGLONG;
          SecurityCookie : ULONGLONG;
          SEHandlerTable : ULONGLONG;
          SEHandlerCount : ULONGLONG;
       end;
     PIMAGE_LOAD_CONFIG_DIRECTORY64 = ^IMAGE_LOAD_CONFIG_DIRECTORY64;
     TIMAGE_LOAD_CONFIG_DIRECTORY64 = IMAGE_LOAD_CONFIG_DIRECTORY64;
{$ifdef WIN64}
     IMAGE_LOAD_CONFIG_DIRECTORY = IMAGE_LOAD_CONFIG_DIRECTORY64;
     TIMAGE_LOAD_CONFIG_DIRECTORY = TIMAGE_LOAD_CONFIG_DIRECTORY64;
     PIMAGE_LOAD_CONFIG_DIRECTORY = PIMAGE_LOAD_CONFIG_DIRECTORY64;
{$else}
     IMAGE_LOAD_CONFIG_DIRECTORY = IMAGE_LOAD_CONFIG_DIRECTORY32;
     TIMAGE_LOAD_CONFIG_DIRECTORY = TIMAGE_LOAD_CONFIG_DIRECTORY32;
     PIMAGE_LOAD_CONFIG_DIRECTORY = PIMAGE_LOAD_CONFIG_DIRECTORY32;
{$endif}

     MOUSEINPUT = record
       dx        : LONG;
       dy        : LONG;
       MouseData : DWORD;
       dwFlags   : DWORD;
       Time      : DWORD;
       ExtraInfo : ULONG_PTR;
     end;
     TMOUSEINPUT  = MOUSEINPUT;
     PMOUSEINPUT  = ^MOUSEINPUT;
     LPMOUSEINPUT = PMOUSEINPUT;

     KEYBDINPUT = record
       wVk       : WORD;
       wScan     : WORD;
       dwFlags   : DWORD;
       Time      : DWORD;
       ExtraInfo : ULONG_PTR;
     end;
     TKEYBDINPUT  = KEYBDINPUT;
     PKEYBDINPUT  = ^KEYBDINPUT;
     LPKEYBDINPUT = PKEYBDINPUT;

     HARDWAREINPUT = record
       uMsg     : DWORD;
       wParamL  : WORD;
       wParamH  : WORD;
     end;
     THARDWAREINPUT  = HARDWAREINPUT;
     PHARDWAREINPUT  = ^HARDWAREINPUT;
     LPHARDWAREINPUT = PHARDWAREINPUT;

     TINPUT = record
       _Type : DWORD;
       case integer of
         1 : (mi : MOUSEINPUT);
         2 : (ki : KEYBDINPUT);
         3 : (hi : HARDWAREINPUT);
     end;
     PINPUT = ^TINPUT;
     LPINPUT = PINPUT;

{$push}
{$packrecords 4}

    PIMAGE_EXPORT_DIRECTORY = ^TIMAGE_EXPORT_DIRECTORY;
    IMAGE_EXPORT_DIRECTORY = record
        Characteristics : DWORD;
        TimeDateStamp   : DWORD;
        MajorVersion    : WORD;
        MinorVersion    : WORD;
        Name 	        : DWORD;
        Base 		    : DWORD;
        NumberOfFunctions : DWORD;
        NumberOfNames   : DWORD;
        AddressOfFunctions : DWORD;     { RVA from base of image }
        AddressOfNames  : DWORD;        { RVA from base of image }
        AddressOfNameOrdinals : DWORD;  { RVA from base of image }
      end;
    TIMAGE_EXPORT_DIRECTORY = IMAGE_EXPORT_DIRECTORY; 
    _IMAGE_EXPORT_DIRECTORY = IMAGE_EXPORT_DIRECTORY;
    LPIMAGE_EXPORT_DIRECTORY= PIMAGE_EXPORT_DIRECTORY;

  P_IMAGE_IMPORT_BY_NAME = ^_IMAGE_IMPORT_BY_NAME;
  _IMAGE_IMPORT_BY_NAME =  record
      Hint : WORD;
      Name : array[0..0] of AnsiCHAR;
    end;
  IMAGE_IMPORT_BY_NAME = _IMAGE_IMPORT_BY_NAME;
  PIMAGE_IMPORT_BY_NAME = ^IMAGE_IMPORT_BY_NAME;
  LPIMAGE_IMPORT_BY_NAME = P_IMAGE_IMPORT_BY_NAME;
  PPIMAGE_IMPORT_BY_NAME = ^PIMAGE_IMPORT_BY_NAME;

  {$push}{$packrecords 8}              // Use align 8 for the 64-bit IAT.}
  P_IMAGE_THUNK_DATA64 = ^_IMAGE_THUNK_DATA64;
  _IMAGE_THUNK_DATA64 =  record
      u1 :  record
          case longint of
            0 : ( ForwarderString : ULONGLONG );    { PBYTE  }
            1 : ( _Function : ULONGLONG );          { PDWORD }
            2 : ( Ordinal : ULONGLONG );
            3 : ( AddressOfData : ULONGLONG );      { PIMAGE_IMPORT_BY_NAME }
          end;
    end;
  IMAGE_THUNK_DATA64 = _IMAGE_THUNK_DATA64;
  PIMAGE_THUNK_DATA64 = ^IMAGE_THUNK_DATA64;

  PPIMAGE_THUNK_DATA64 = ^PIMAGE_THUNK_DATA64;
  LPIMAGE_THUNK_DATA64 = PIMAGE_THUNK_DATA64;
  {$pop}                        // Back to 4 byte packing}

  P_IMAGE_THUNK_DATA32 = ^_IMAGE_THUNK_DATA32;
  _IMAGE_THUNK_DATA32 =  record
      u1 :  record
          case longint of
            0 : ( ForwarderString : DWORD );          { PBYTE  }
            1 : ( _Function : DWORD );                { PDWORD }
            2 : ( Ordinal : DWORD );
            3 : ( AddressOfData : DWORD );            { PIMAGE_IMPORT_BY_NAME }
          end;
    end;
  IMAGE_THUNK_DATA32 = _IMAGE_THUNK_DATA32;
  PIMAGE_THUNK_DATA32 = ^IMAGE_THUNK_DATA32;

  PPIMAGE_THUNK_DATA32 = ^PIMAGE_THUNK_DATA32;
  LPIMAGE_THUNK_DATA32 = PIMAGE_THUNK_DATA32;

  { }
  { Thread Local Storage }
  { }

  PIMAGE_TLS_CALLBACK = procedure (DllHandle:PVOID; Reason:DWORD; Reserved:PVOID);stdcall; {NTAPI}

  P_IMAGE_TLS_DIRECTORY64 = ^_IMAGE_TLS_DIRECTORY64;
  _IMAGE_TLS_DIRECTORY64 =  record
      StartAddressOfRawData : ULONGLONG;
      EndAddressOfRawData : ULONGLONG;
      AddressOfIndex : ULONGLONG;               { PDWORD }
      AddressOfCallBacks : ULONGLONG;           { PIMAGE_TLS_CALLBACK *; }
      SizeOfZeroFill : DWORD;
          case longint of
            0 : ( Characteristics : DWORD );
            1 : ( CharacteristicsFields:  bitpacked record
                                  Reserved0 : 0..$FFFFF; // 5 nibbles=20 bits
                                  Alignment : 0..$F;      // 4 bits
                                  Reserved1 : 0..$FF;     // 8 bits
              end );
    end;
  IMAGE_TLS_DIRECTORY64 = _IMAGE_TLS_DIRECTORY64;
  PIMAGE_TLS_DIRECTORY64 = ^IMAGE_TLS_DIRECTORY64;

  PPIMAGE_TLS_DIRECTORY64 = ^PIMAGE_TLS_DIRECTORY64;
  LPIMAGE_TLS_DIRECTORY64 = PIMAGE_TLS_DIRECTORY64;
  P_IMAGE_TLS_DIRECTORY32 = ^_IMAGE_TLS_DIRECTORY32;
  _IMAGE_TLS_DIRECTORY32 =  record
      StartAddressOfRawData : DWORD;
      EndAddressOfRawData : DWORD;
      AddressOfIndex : DWORD;                      { PDWORD }
      AddressOfCallBacks : DWORD;                  { PIMAGE_TLS_CALLBACK * }
      SizeOfZeroFill : DWORD;
          case longint of
            0 : ( Characteristics : DWORD );
            1 : ( CharacteristicsFields : bitpacked  record
                                 Reserved0 : 0..$FFFFF; // 5 nibbles=20 bits
                                 Alignment : 0..$F;      // 4 bits
                                 Reserved1 : 0..$FF;     // 8 bits
              end );

    end;
  IMAGE_TLS_DIRECTORY32 = _IMAGE_TLS_DIRECTORY32;
  PIMAGE_TLS_DIRECTORY32 = ^IMAGE_TLS_DIRECTORY32;



  PPIMAGE_TLS_DIRECTORY32 = ^PIMAGE_TLS_DIRECTORY32;
  LPIMAGE_TLS_DIRECTORY32 = PIMAGE_TLS_DIRECTORY32;

  {$ifdef WIN64}

  PIMAGE_THUNK_DATA = PIMAGE_THUNK_DATA64;
  IMAGE_THUNK_DATA = IMAGE_THUNK_DATA64;

  PPIMAGE_THUNK_DATA = ^PIMAGE_THUNK_DATA64;
  LPIMAGE_THUNK_DATA = PIMAGE_THUNK_DATA64;

  PIMAGE_TLS_DIRECTORY = ^IMAGE_TLS_DIRECTORY;
  IMAGE_TLS_DIRECTORY = IMAGE_TLS_DIRECTORY64;

  PPIMAGE_TLS_DIRECTORY = ^PIMAGE_TLS_DIRECTORY;
  LPIMAGE_TLS_DIRECTORY = PIMAGE_TLS_DIRECTORY64;
  {$else}

  PIMAGE_THUNK_DATA = PIMAGE_THUNK_DATA32;
  IMAGE_THUNK_DATA = IMAGE_THUNK_DATA32;

  PPIMAGE_THUNK_DATA = ^PIMAGE_THUNK_DATA;
  LPIMAGE_THUNK_DATA = PIMAGE_THUNK_DATA32;
  PIMAGE_TLS_DIRECTORY = ^IMAGE_TLS_DIRECTORY;
  IMAGE_TLS_DIRECTORY = IMAGE_TLS_DIRECTORY32;

  PPIMAGE_TLS_DIRECTORY = ^PIMAGE_TLS_DIRECTORY;
  LPIMAGE_TLS_DIRECTORY = PIMAGE_TLS_DIRECTORY32;
  {$endif}

  P_IMAGE_IMPORT_DESCRIPTOR = ^_IMAGE_IMPORT_DESCRIPTOR;
  _IMAGE_IMPORT_DESCRIPTOR =  record
          case longint of
            0 : ( Characteristics : DWORD );     { 0 for terminating null import descriptor }
            1 : ( OriginalFirstThunk : DWORD;    { RVA to original unbound IAT (PIMAGE_THUNK_DATA) }
                  TimeDateStamp : DWORD;         { 0 if not bound, }
                                                 // -1 if bound, and real date\time stamp
                                                 //     in IMAGE_DIRECTORY_ENTRY_BOUND_IMPORT (new BIND)
                                                 // O.W. date/time stamp of DLL bound to (Old BIND)
                  ForwarderChain : DWORD;        // -1 if no forwarders
                  Name : DWORD;
                  FirstThunk : DWORD;            // RVA to IAT (if bound this IAT has actual addresses)
                );  
    end;
  IMAGE_IMPORT_DESCRIPTOR = _IMAGE_IMPORT_DESCRIPTOR;
  PIMAGE_IMPORT_DESCRIPTOR = ^IMAGE_IMPORT_DESCRIPTOR   {UNALIGNED  }     ;


  PPIMAGE_IMPORT_DESCRIPTOR = ^PIMAGE_IMPORT_DESCRIPTOR;
  LPIMAGE_IMPORT_DESCRIPTOR = PIMAGE_IMPORT_DESCRIPTOR;
  { }
  { New format import descriptors pointed to by DataDirectory[ IMAGE_DIRECTORY_ENTRY_BOUND_IMPORT ] }
  { }


  P_IMAGE_BOUND_IMPORT_DESCRIPTOR = ^_IMAGE_BOUND_IMPORT_DESCRIPTOR;
  _IMAGE_BOUND_IMPORT_DESCRIPTOR =  record
      TimeDateStamp : DWORD;
      OffsetModuleName : WORD;
      NumberOfModuleForwarderRefs : WORD;
      { Array of zero or more IMAGE_BOUND_FORWARDER_REF follows }
    end;

  IMAGE_BOUND_IMPORT_DESCRIPTOR = _IMAGE_BOUND_IMPORT_DESCRIPTOR;
  PIMAGE_BOUND_IMPORT_DESCRIPTOR = ^IMAGE_BOUND_IMPORT_DESCRIPTOR;
  LPIMAGE_BOUND_IMPORT_DESCRIPTOR = P_IMAGE_BOUND_IMPORT_DESCRIPTOR;
  PPIMAGE_BOUND_IMPORT_DESCRIPTOR = ^PIMAGE_BOUND_IMPORT_DESCRIPTOR;

  P_IMAGE_BOUND_FORWARDER_REF = ^_IMAGE_BOUND_FORWARDER_REF;
  _IMAGE_BOUND_FORWARDER_REF =  record
      TimeDateStamp : DWORD;
      OffsetModuleName : WORD;
      Reserved : WORD;
    end;
  IMAGE_BOUND_FORWARDER_REF = _IMAGE_BOUND_FORWARDER_REF;
  PIMAGE_BOUND_FORWARDER_REF = ^IMAGE_BOUND_FORWARDER_REF;
  LPIMAGE_BOUND_FORWARDER_REF = P_IMAGE_BOUND_FORWARDER_REF;
  PPIMAGE_BOUND_FORWARDER_REF = ^PIMAGE_BOUND_FORWARDER_REF;
  { Delay load version 2 }

  _IMAGE_DELAYLOAD_DESCRIPTOR = record
        case longint of
        0: (AllAttributes :Dword;
            DllNameRVA,                       // RVA to the name of the target library (NULL-terminate ASCII string)
            ModuleHandleRVA,                  // RVA to the HMODULE caching location (PHMODULE)
            ImportAddressTableRVA,            // RVA to the start of the IAT (PIMAGE_THUNK_DATA)
            ImportNameTableRVA,               // RVA to the start of the name table (PIMAGE_THUNK_DATA::AddressOfData)
            BoundImportAddressTableRVA,       // RVA to an optional bound IAT
            UnloadInformationTableRVA,        // RVA to an optional unload info table
            TimeDateStamp            : DWORD; // 0 if not bound,
                                            // Otherwise, date/time of the target DLL
         );
        1: (Attributes:bitpacked record
             rvabased:0..1;  {1 bits}                 // Delay load version 2
             ReservedAttributes: 0..$7FFFFFF; {31 bits}
             end;)
     end;

  IMAGE_DELAYLOAD_DESCRIPTOR= _IMAGE_DELAYLOAD_DESCRIPTOR;
  PIMAGE_DELAYLOAD_DESCRIPTOR= ^_IMAGE_DELAYLOAD_DESCRIPTOR;
  PCIMAGE_DELAYLOAD_DESCRIPTOR= PIMAGE_DELAYLOAD_DESCRIPTOR;
{$pop}

 tagCOMBOBOXINFO = record
    cbSize: DWORD;
    rcItem: TRect;
    rcButton: TRect;
    stateButton: DWORD;
    hwndCombo: HWND;
    hwndItem: HWND;
    hwndList: HWND;
  end;
  TComboboxInfo = tagCOMBOBOXINFO;
  PComboboxInfo = ^TComboboxInfo;
  LPComboboxInfo = PComboboxInfo;

  _TRIVERTEX = record
          x : LONG;
          y : LONG;
          Red : COLOR16;
          Green : COLOR16;
          Blue : COLOR16;
          Alpha : COLOR16;
       end;
  TRIVERTEX = _TRIVERTEX;
  PTRIVERTEX = ^_TRIVERTEX;
  LPTRIVERTEX = PTRIVERTEX;

  _GRADIENT_TRIANGLE = record
          Vertex1 : ULONG;
          Vertex2 : ULONG;
          Vertex3 : ULONG;
       end;
  GRADIENT_TRIANGLE = _GRADIENT_TRIANGLE;
  TGRADIENT_TRIANGLE = _GRADIENT_TRIANGLE;
  PGRADIENT_TRIANGLE = ^_GRADIENT_TRIANGLE;
  LPGRADIENT_TRIANGLE = PGRADIENT_TRIANGLE;

  _GRADIENT_RECT = record
          UpperLeft : ULONG;
          LowerRight : ULONG;
       end;
  GRADIENT_RECT = _GRADIENT_RECT;
  TGRADIENT_RECT = _GRADIENT_RECT;
  PGRADIENT_RECT = ^_GRADIENT_RECT;
  LPGRADIENT_RECT = PGRADIENT_RECT;

  CURSORINFO = record
    		 cbSize  : DWORD;
     		 flags   : DWORD;
    		 hCursor : HCURSOR;
    	         ptScreenPos : POINT;
                 end;
  PCURSORINFO = ^CURSORINFO;
  TCURSORINFO = CURSORINFO;

const
  SECURITY_DESCRIPTOR_MIN_LENGTH = SIZEOF(SECURITY_DESCRIPTOR);


// #if(WINVER >= 0x0500)
type
     tagMENUGETOBJECTINFO = packed record
       dwFlags:DWORD;
       uPos:UINT;
       _hmenu:HMENU;
       riid:PVOID;
       pvObj:PVOID;
     end;
     MENUGETOBJECTINFO = tagMENUGETOBJECTINFO;
     PMENUGETOBJECTINFO = ^tagMENUGETOBJECTINFO;
     LPMENUGETOBJECTINFO = ^tagMENUGETOBJECTINFO;
//#endif /* WINVER >= 0x0500 */


// Menubar information
type
     tagMENUBARINFO = record
       cbSize:DWORD;
       rcBar:Windows.RECT; // rect of bar, popup, item
       _hmenu:HMENU;       // real menu handle of bar, popup
       hwndMenu:HWND;      // hwnd of item submenu if one
//    fBarFocused:1:BOOL;  // bar, popup has the focus
//    fFocused:1:BOOL;     // item has the focus
      FocusedBits:DWORD;
    end;
    MENUBARINFO = tagMENUBARINFO;
    PMENUBARINFO = ^tagMENUBARINFO;
    LPMENUBARINFO = ^tagMENUBARINFO;


    // defined here because large_integer is defined in struct.inc
    LPPROGRESS_ROUTINE = function(TotalFileSize,TotalBytesTransferred,StreamSize,StreamBytesTransferred:LARGE_INTEGER;dwStreamNumber,dwCallbackReason:DWord;hSourceFile,hDestinationFile :THandle;lpdata:pointer):Dword; Stdcall;
    TPROGRESS_ROUTINE = LPPROGRESS_ROUTINE;

// Raw Input Messages
//#if(_WIN32_WINNT >= 0x0501)
{$push}{$packrecords 4}
    tagRAWINPUTHEADER = record
      dwType: DWORD;
      dwSize: DWORD;
      hDevice: HANDLE;
      wParam: WPARAM;
    end;
    RAWINPUTHEADER = tagRAWINPUTHEADER;
    TRAWINPUTHEADER = tagRAWINPUTHEADER;
    PRAWINPUTHEADER = ^tagRAWINPUTHEADER;
    LPRAWINPUTHEADER = ^tagRAWINPUTHEADER;

(*
 * Raw format of the mouse input
 *)
    tagRAWMOUSE = record
      (*
       * Indicator flags.
       *)
      usFlags: USHORT;

      case longint of
        0: (
          (*
           * The transition state of the mouse buttons.
           *)
          ulButtons: ULONG;

          (*
           * The raw state of the mouse buttons.
           *)
          ulRawButtons: ULONG;

          (*
           * The signed relative or absolute motion in the X direction.
           *)
          lLastX: LONG;

          (*
           * The signed relative or absolute motion in the Y direction.
           *)
          lLastY: LONG;

          (*
           * Device-specific additional information for the event.
           *)
          ulExtraInformation: ULONG
        );
        1: (
          (*
           * The transition state of the mouse buttons.
           *)
          usButtonFlags: USHORT;
          usButtonData: USHORT
        );
    end;
    RAWMOUSE = tagRAWMOUSE;
    TRAWMOUSE = tagRAWMOUSE;
    PRAWMOUSE = ^tagRAWMOUSE;
    LPRAWMOUSE = ^tagRAWMOUSE;

(*
 * Raw format of the keyboard input
 *)
    tagRAWKEYBOARD = record
      (*
       * The "make" scan code (key depression).
       *)
      MakeCode: USHORT;

      (*
       * The flags field indicates a "break" (key release) and other
       * miscellaneous scan code information defined in ntddkbd.h.
       *)
      Flags: USHORT;

      Reserved: USHORT;

      (*
       * Windows message compatible information
       *)
      VKey: USHORT;
      Message: UINT;

      (*
       * Device-specific additional information for the event.
       *)
      ExtraInformation: ULONG;
    end;
    RAWKEYBOARD = tagRAWKEYBOARD;
    TRAWKEYBOARD = tagRAWKEYBOARD;
    PRAWKEYBOARD = ^tagRAWKEYBOARD;
    LPRAWKEYBOARD = ^tagRAWKEYBOARD;

    (*
     * Raw format of the input from Human Input Devices
     *)
    tagRAWHID = record
      dwSizeHid: DWORD;    // byte size of each report
      dwCount: DWORD;      // number of input packed
      bRawData: array [0..0] of BYTE;
    end;
    RAWHID = tagRAWHID;
    TRAWHID = tagRAWHID;
    PRAWHID = ^tagRAWHID;
    LPRAWHID = ^tagRAWHID;

(*
 * RAWINPUT data structure.
 *)
    tagRAWINPUT = record
      header: RAWINPUTHEADER;
      data: record
        case longint of
          0: (mouse: RAWMOUSE);
          1: (keyboard: RAWKEYBOARD);
          2: (hid: RAWHID);
      end;
    end;
    RAWINPUT = tagRAWINPUT;
    TRAWINPUT = tagRAWINPUT;
    PRAWINPUT = ^tagRAWINPUT;
    LPRAWINPUT = ^tagRAWINPUT;
    PPRAWINPUT = ^PRAWINPUT;
    LPLPRAWINPUT = ^LPRAWINPUT;

    tagRID_DEVICE_INFO_MOUSE = record
      dwId: DWORD;
      dwNumberOfButtons: DWORD;
      dwSampleRate: DWORD;
      fHasHorizontalWheel: BOOL;
    end;
    RID_DEVICE_INFO_MOUSE = tagRID_DEVICE_INFO_MOUSE;
    TRID_DEVICE_INFO_MOUSE = tagRID_DEVICE_INFO_MOUSE;
    PRID_DEVICE_INFO_MOUSE = ^tagRID_DEVICE_INFO_MOUSE;

    tagRID_DEVICE_INFO_KEYBOARD = record
      dwType: DWORD;
      dwSubType: DWORD;
      dwKeyboardMode: DWORD;
      dwNumberOfFunctionKeys: DWORD;
      dwNumberOfIndicators: DWORD;
      dwNumberOfKeysTotal: DWORD;
    end;
    RID_DEVICE_INFO_KEYBOARD = tagRID_DEVICE_INFO_KEYBOARD;
    TRID_DEVICE_INFO_KEYBOARD = tagRID_DEVICE_INFO_KEYBOARD;
    PRID_DEVICE_INFO_KEYBOARD = ^tagRID_DEVICE_INFO_KEYBOARD;

    tagRID_DEVICE_INFO_HID = record
      dwVendorId: DWORD;
      dwProductId: DWORD;
      dwVersionNumber: DWORD;

      (*
       * Top level collection UsagePage and Usage
       *)
      usUsagePage: USHORT;
      usUsage: USHORT;
    end;
    RID_DEVICE_INFO_HID = tagRID_DEVICE_INFO_HID;
    TRID_DEVICE_INFO_HID = tagRID_DEVICE_INFO_HID;
    PRID_DEVICE_INFO_HID = ^tagRID_DEVICE_INFO_HID;

    tagRID_DEVICE_INFO = record
      cbSize: DWORD;
      dwType: DWORD;
      case longint of
        0: (mouse: RID_DEVICE_INFO_MOUSE);
        1: (keyboard: RID_DEVICE_INFO_KEYBOARD);
        2: (hid: RID_DEVICE_INFO_HID);
    end;
    RID_DEVICE_INFO = tagRID_DEVICE_INFO;
    PRID_DEVICE_INFO = ^tagRID_DEVICE_INFO;
    LPRID_DEVICE_INFO = ^tagRID_DEVICE_INFO;

(*
 * Raw Input request APIs
 *)
    tagRAWINPUTDEVICE = record
      usUsagePage: USHORT; // Toplevel collection UsagePage
      usUsage: USHORT;     // Toplevel collection Usage
      dwFlags: DWORD;
      hwndTarget: HWND;    // Target hwnd. NULL = follows keyboard focus
    end;
    RAWINPUTDEVICE = tagRAWINPUTDEVICE;
    TRAWINPUTDEVICE = tagRAWINPUTDEVICE;
    PRAWINPUTDEVICE = ^tagRAWINPUTDEVICE;
    LPRAWINPUTDEVICE = ^tagRAWINPUTDEVICE;
    PCRAWINPUTDEVICE = ^tagRAWINPUTDEVICE;

{$packrecords C}	
    tagRAWINPUTDEVICELIST = record
      hDevice: HANDLE;
      dwType: DWORD;
    end;
    RAWINPUTDEVICELIST = tagRAWINPUTDEVICELIST;
    PRAWINPUTDEVICELIST = ^tagRAWINPUTDEVICELIST;
//#endif /* _WIN32_WINNT >= 0x0501 */

// Wincon.h  

    _CONSOLE_SCREEN_BUFFER_INFOEX = record
        cbSize : ULONG;
        dwSize : COORD;
        dwCursorPosition : COORD;
        wAttributes : WORD;
        srWindow : SMALL_RECT;
        dwMaximumWindowSize : COORD;
        wPopupAttributes : WORD;
        bFullscreenSupported : BOOL;
        ColorTable : array[0..15] of COLORREF;
      end;
    CONSOLE_SCREEN_BUFFER_INFOEX = _CONSOLE_SCREEN_BUFFER_INFOEX;
    PCONSOLE_SCREEN_BUFFER_INFOEX = ^_CONSOLE_SCREEN_BUFFER_INFOEX;

    
    _CONSOLE_FONT_INFO = record
        nFont : DWORD;
        dwFontSize : COORD;
      end;
    CONSOLE_FONT_INFO = _CONSOLE_FONT_INFO;
    PCONSOLE_FONT_INFO = ^_CONSOLE_FONT_INFO;

    _CONSOLE_FONT_INFOEX = record
        cbSize : ULONG;
        nFont : DWORD;
        dwFontSize : COORD;
        FontFamily : UINT;
        FontWeight : UINT;
        FaceName : array[0..(LF_FACESIZE)-1] of WCHAR;
      end;
    CONSOLE_FONT_INFOEX = _CONSOLE_FONT_INFOEX;
    PCONSOLE_FONT_INFOEX = ^_CONSOLE_FONT_INFOEX;



    _CONSOLE_HISTORY_INFO = record
        cbSize : UINT;
        HistoryBufferSize : UINT;
        NumberOfHistoryBuffers : UINT;
        dwFlags : DWORD;
      end;
    CONSOLE_HISTORY_INFO = _CONSOLE_HISTORY_INFO;
    PCONSOLE_HISTORY_INFO = ^_CONSOLE_HISTORY_INFO;

    _CONSOLE_SELECTION_INFO = record
        dwFlags : DWORD;
        dwSelectionAnchor : COORD;
        srSelection : SMALL_RECT;
      end;
    CONSOLE_SELECTION_INFO = _CONSOLE_SELECTION_INFO;
    PCONSOLE_SELECTION_INFO = ^_CONSOLE_SELECTION_INFO;

    _CONSOLE_READCONSOLE_CONTROL = record
        nLength : ULONG;
        nInitialChars : ULONG;
        dwCtrlWakeupMask : ULONG;
        dwControlKeyState : ULONG;
      end;
    CONSOLE_READCONSOLE_CONTROL = _CONSOLE_READCONSOLE_CONTROL;
    PCONSOLE_READCONSOLE_CONTROL = ^_CONSOLE_READCONSOLE_CONTROL;


    COMPUTER_NAME_FORMAT =  (
          ComputerNameNetBIOS=0,
          ComputerNameDnsHostname,
          ComputerNameDnsDomain,
          ComputerNameDnsFullyQualified,
          ComputerNamePhysicalNetBIOS,
          ComputerNamePhysicalDnsHostname,
          ComputerNamePhysicalDnsDomain,
          ComputerNamePhysicalDnsFullyQualified,
          ComputerNameMax=31);
    TCOMPUTER_NAME_FORMAT = COMPUTER_NAME_FORMAT;
{$pop}
{$endif read_interface}


{$ifdef read_implementation}

  function fBinary(var a : DCB) : DWORD;
    begin
       fBinary:=(a.flags and bm_DCB_fBinary) shr bp_DCB_fBinary;
    end;

  procedure set_fBinary(var a : DCB; __fBinary : DWORD);
    begin
       a.flags:=a.flags or ((__fBinary shl bp_DCB_fBinary) and bm_DCB_fBinary);
    end;

  function fParity(var a : DCB) : DWORD;
    begin
       fParity:=(a.flags and bm_DCB_fParity) shr bp_DCB_fParity;
    end;

  procedure set_fParity(var a : DCB; __fParity : DWORD);
    begin
       a.flags:=a.flags or ((__fParity shl bp_DCB_fParity) and bm_DCB_fParity);
    end;

  function fOutxCtsFlow(var a : DCB) : DWORD;
    begin
       fOutxCtsFlow:=(a.flags and bm_DCB_fOutxCtsFlow) shr bp_DCB_fOutxCtsFlow;
    end;

  procedure set_fOutxCtsFlow(var a : DCB; __fOutxCtsFlow : DWORD);
    begin
       a.flags:=a.flags or ((__fOutxCtsFlow shl bp_DCB_fOutxCtsFlow) and bm_DCB_fOutxCtsFlow);
    end;

  function fOutxDsrFlow(var a : DCB) : DWORD;
    begin
       fOutxDsrFlow:=(a.flags and bm_DCB_fOutxDsrFlow) shr bp_DCB_fOutxDsrFlow;
    end;

  procedure set_fOutxDsrFlow(var a : DCB; __fOutxDsrFlow : DWORD);
    begin
       a.flags:=a.flags or ((__fOutxDsrFlow shl bp_DCB_fOutxDsrFlow) and bm_DCB_fOutxDsrFlow);
    end;

  function fDtrControl(var a : DCB) : DWORD;
    begin
       fDtrControl:=(a.flags and bm_DCB_fDtrControl) shr bp_DCB_fDtrControl;
    end;

  procedure set_fDtrControl(var a : DCB; __fDtrControl : DWORD);
    begin
       a.flags:=a.flags or ((__fDtrControl shl bp_DCB_fDtrControl) and bm_DCB_fDtrControl);
    end;

  function fDsrSensitivity(var a : DCB) : DWORD;
    begin
       fDsrSensitivity:=(a.flags and bm_DCB_fDsrSensitivity) shr bp_DCB_fDsrSensitivity;
    end;

  procedure set_fDsrSensitivity(var a : DCB; __fDsrSensitivity : DWORD);
    begin
       a.flags:=a.flags or ((__fDsrSensitivity shl bp_DCB_fDsrSensitivity) and bm_DCB_fDsrSensitivity);
    end;

  function fTXContinueOnXoff(var a : DCB) : DWORD;
    begin
       fTXContinueOnXoff:=(a.flags and bm_DCB_fTXContinueOnXoff) shr bp_DCB_fTXContinueOnXoff;
    end;

  procedure set_fTXContinueOnXoff(var a : DCB; __fTXContinueOnXoff : DWORD);
    begin
       a.flags:=a.flags or ((__fTXContinueOnXoff shl bp_DCB_fTXContinueOnXoff) and bm_DCB_fTXContinueOnXoff);
    end;

  function fOutX(var a : DCB) : DWORD;
    begin
       fOutX:=(a.flags and bm_DCB_fOutX) shr bp_DCB_fOutX;
    end;

  procedure set_fOutX(var a : DCB; __fOutX : DWORD);
    begin
       a.flags:=a.flags or ((__fOutX shl bp_DCB_fOutX) and bm_DCB_fOutX);
    end;

  function fInX(var a : DCB) : DWORD;
    begin
       fInX:=(a.flags and bm_DCB_fInX) shr bp_DCB_fInX;
    end;

  procedure set_fInX(var a : DCB; __fInX : DWORD);
    begin
       a.flags:=a.flags or ((__fInX shl bp_DCB_fInX) and bm_DCB_fInX);
    end;

  function fErrorChar(var a : DCB) : DWORD;
    begin
       fErrorChar:=(a.flags and bm_DCB_fErrorChar) shr bp_DCB_fErrorChar;
    end;

  procedure set_fErrorChar(var a : DCB; __fErrorChar : DWORD);
    begin
       a.flags:=a.flags or ((__fErrorChar shl bp_DCB_fErrorChar) and bm_DCB_fErrorChar);
    end;

  function fNull(var a : DCB) : DWORD;
    begin
       fNull:=(a.flags and bm_DCB_fNull) shr bp_DCB_fNull;
    end;

  procedure set_fNull(var a : DCB; __fNull : DWORD);
    begin
       a.flags:=a.flags or ((__fNull shl bp_DCB_fNull) and bm_DCB_fNull);
    end;

  function fRtsControl(var a : DCB) : DWORD;
    begin
       fRtsControl:=(a.flags and bm_DCB_fRtsControl) shr bp_DCB_fRtsControl;
    end;

  procedure set_fRtsControl(var a : DCB; __fRtsControl : DWORD);
    begin
       a.flags:=a.flags or ((__fRtsControl shl bp_DCB_fRtsControl) and bm_DCB_fRtsControl);
    end;

  function fAbortOnError(var a : DCB) : DWORD;
    begin
       fAbortOnError:=(a.flags and bm_DCB_fAbortOnError) shr bp_DCB_fAbortOnError;
    end;

  procedure set_fAbortOnError(var a : DCB; __fAbortOnError : DWORD);
    begin
       a.flags:=a.flags or ((__fAbortOnError shl bp_DCB_fAbortOnError) and bm_DCB_fAbortOnError);
    end;

  function fDummy2(var a : DCB) : DWORD;
    begin
       fDummy2:=(a.flags and bm_DCB_fDummy2) shr bp_DCB_fDummy2;
    end;

  procedure set_fDummy2(var a : DCB; __fDummy2 : DWORD);
    begin
       a.flags:=a.flags or ((__fDummy2 shl bp_DCB_fDummy2) and bm_DCB_fDummy2);
    end;

  function fCtsHold(var a : COMSTAT) : DWORD;
    begin
       fCtsHold:=(a.flag0 and bm_COMSTAT_fCtsHold) shr bp_COMSTAT_fCtsHold;
    end;

  procedure set_fCtsHold(var a : COMSTAT; __fCtsHold : DWORD);
    begin
       a.flag0:=a.flag0 or ((__fCtsHold shl bp_COMSTAT_fCtsHold) and bm_COMSTAT_fCtsHold);
    end;

  function fDsrHold(var a : COMSTAT) : DWORD;
    begin
       fDsrHold:=(a.flag0 and bm_COMSTAT_fDsrHold) shr bp_COMSTAT_fDsrHold;
    end;

  procedure set_fDsrHold(var a : COMSTAT; __fDsrHold : DWORD);
    begin
       a.flag0:=a.flag0 or ((__fDsrHold shl bp_COMSTAT_fDsrHold) and bm_COMSTAT_fDsrHold);
    end;

  function fRlsdHold(var a : COMSTAT) : DWORD;
    begin
       fRlsdHold:=(a.flag0 and bm_COMSTAT_fRlsdHold) shr bp_COMSTAT_fRlsdHold;
    end;

  procedure set_fRlsdHold(var a : COMSTAT; __fRlsdHold : DWORD);
    begin
       a.flag0:=a.flag0 or ((__fRlsdHold shl bp_COMSTAT_fRlsdHold) and bm_COMSTAT_fRlsdHold);
    end;

  function fXoffHold(var a : COMSTAT) : DWORD;
    begin
       fXoffHold:=(a.flag0 and bm_COMSTAT_fXoffHold) shr bp_COMSTAT_fXoffHold;
    end;

  procedure set_fXoffHold(var a : COMSTAT; __fXoffHold : DWORD);
    begin
       a.flag0:=a.flag0 or ((__fXoffHold shl bp_COMSTAT_fXoffHold) and bm_COMSTAT_fXoffHold);
    end;

  function fXoffSent(var a : COMSTAT) : DWORD;
    begin
       fXoffSent:=(a.flag0 and bm_COMSTAT_fXoffSent) shr bp_COMSTAT_fXoffSent;
    end;

  procedure set_fXoffSent(var a : COMSTAT; __fXoffSent : DWORD);
    begin
       a.flag0:=a.flag0 or ((__fXoffSent shl bp_COMSTAT_fXoffSent) and bm_COMSTAT_fXoffSent);
    end;

  function fEof(var a : COMSTAT) : DWORD;
    begin
       fEof:=(a.flag0 and bm_COMSTAT_fEof) shr bp_COMSTAT_fEof;
    end;

  procedure set_fEof(var a : COMSTAT; __fEof : DWORD);
    begin
       a.flag0:=a.flag0 or ((__fEof shl bp_COMSTAT_fEof) and bm_COMSTAT_fEof);
    end;

  function fTxim(var a : COMSTAT) : DWORD;
    begin
       fTxim:=(a.flag0 and bm_COMSTAT_fTxim) shr bp_COMSTAT_fTxim;
    end;

  procedure set_fTxim(var a : COMSTAT; __fTxim : DWORD);
    begin
       a.flag0:=a.flag0 or ((__fTxim shl bp_COMSTAT_fTxim) and bm_COMSTAT_fTxim);
    end;

  function fReserved(var a : COMSTAT) : DWORD;
    begin
       fReserved:=(a.flag0 and bm_COMSTAT_fReserved) shr bp_COMSTAT_fReserved;
    end;

  procedure set_fReserved(var a : COMSTAT; __fReserved : DWORD);
    begin
       a.flag0:=a.flag0 or ((__fReserved shl bp_COMSTAT_fReserved) and bm_COMSTAT_fReserved);
    end;

  function bAppReturnCode(var a : DDEACK) : word;
    begin
       bAppReturnCode:=(a.flag0 and bm_DDEACK_bAppReturnCode) shr bp_DDEACK_bAppReturnCode;
    end;

  procedure set_bAppReturnCode(var a : DDEACK; __bAppReturnCode : word);
    begin
       a.flag0:=a.flag0 or ((__bAppReturnCode shl bp_DDEACK_bAppReturnCode) and bm_DDEACK_bAppReturnCode);
    end;

  function reserved(var a : DDEACK) : word;
    begin
       reserved:=(a.flag0 and bm_DDEACK_reserved) shr bp_DDEACK_reserved;
    end;

  procedure set_reserved(var a : DDEACK; __reserved : word);
    begin
       a.flag0:=a.flag0 or ((__reserved shl bp_DDEACK_reserved) and bm_DDEACK_reserved);
    end;

  function fBusy(var a : DDEACK) : word;
    begin
       fBusy:=(a.flag0 and bm_DDEACK_fBusy) shr bp_DDEACK_fBusy;
    end;

  procedure set_fBusy(var a : DDEACK; __fBusy : word);
    begin
       a.flag0:=a.flag0 or ((__fBusy shl bp_DDEACK_fBusy) and bm_DDEACK_fBusy);
    end;

  function fAck(var a : DDEACK) : word;
    begin
       fAck:=(a.flag0 and bm_DDEACK_fAck) shr bp_DDEACK_fAck;
    end;

  procedure set_fAck(var a : DDEACK; __fAck : word);
    begin
       a.flag0:=a.flag0 or ((__fAck shl bp_DDEACK_fAck) and bm_DDEACK_fAck);
    end;

  function reserved(var a : DDEADVISE) : word;
    begin
       reserved:=(a.flag0 and bm_DDEADVISE_reserved) shr bp_DDEADVISE_reserved;
    end;

  procedure set_reserved(var a : DDEADVISE; __reserved : word);
    begin
       a.flag0:=a.flag0 or ((__reserved shl bp_DDEADVISE_reserved) and bm_DDEADVISE_reserved);
    end;

  function fDeferUpd(var a : DDEADVISE) : word;
    begin
       fDeferUpd:=(a.flag0 and bm_DDEADVISE_fDeferUpd) shr bp_DDEADVISE_fDeferUpd;
    end;

  procedure set_fDeferUpd(var a : DDEADVISE; __fDeferUpd : word);
    begin
       a.flag0:=a.flag0 or ((__fDeferUpd shl bp_DDEADVISE_fDeferUpd) and bm_DDEADVISE_fDeferUpd);
    end;

  function fAckReq(var a : DDEADVISE) : word;
    begin
       fAckReq:=(a.flag0 and bm_DDEADVISE_fAckReq) shr bp_DDEADVISE_fAckReq;
    end;

  procedure set_fAckReq(var a : DDEADVISE; __fAckReq : word);
    begin
       a.flag0:=a.flag0 or ((__fAckReq shl bp_DDEADVISE_fAckReq) and bm_DDEADVISE_fAckReq);
    end;

  function unused(var a : DDEDATA) : word;
    begin
       unused:=(a.flag0 and bm_DDEDATA_unused) shr bp_DDEDATA_unused;
    end;

  procedure set_unused(var a : DDEDATA; __unused : word);
    begin
       a.flag0:=a.flag0 or ((__unused shl bp_DDEDATA_unused) and bm_DDEDATA_unused);
    end;

  function fResponse(var a : DDEDATA) : word;
    begin
       fResponse:=(a.flag0 and bm_DDEDATA_fResponse) shr bp_DDEDATA_fResponse;
    end;

  procedure set_fResponse(var a : DDEDATA; __fResponse : word);
    begin
       a.flag0:=a.flag0 or ((__fResponse shl bp_DDEDATA_fResponse) and bm_DDEDATA_fResponse);
    end;

  function fRelease(var a : DDEDATA) : word;
    begin
       fRelease:=(a.flag0 and bm_DDEDATA_fRelease) shr bp_DDEDATA_fRelease;
    end;

  procedure set_fRelease(var a : DDEDATA; __fRelease : word);
    begin
       a.flag0:=a.flag0 or ((__fRelease shl bp_DDEDATA_fRelease) and bm_DDEDATA_fRelease);
    end;

  function reserved(var a : DDEDATA) : word;
    begin
       reserved:=(a.flag0 and bm_DDEDATA_reserved) shr bp_DDEDATA_reserved;
    end;

  procedure set_reserved(var a : DDEDATA; __reserved : word);
    begin
       a.flag0:=a.flag0 or ((__reserved shl bp_DDEDATA_reserved) and bm_DDEDATA_reserved);
    end;

  function fAckReq(var a : DDEDATA) : word;
    begin
       fAckReq:=(a.flag0 and bm_DDEDATA_fAckReq) shr bp_DDEDATA_fAckReq;
    end;

  procedure set_fAckReq(var a : DDEDATA; __fAckReq : word);
    begin
       a.flag0:=a.flag0 or ((__fAckReq shl bp_DDEDATA_fAckReq) and bm_DDEDATA_fAckReq);
    end;

  function unused(var a : DDELN) : word;
    begin
       unused:=(a.flag0 and bm_DDELN_unused) shr bp_DDELN_unused;
    end;

  procedure set_unused(var a : DDELN; __unused : word);
    begin
       a.flag0:=a.flag0 or ((__unused shl bp_DDELN_unused) and bm_DDELN_unused);
    end;

  function fRelease(var a : DDELN) : word;
    begin
       fRelease:=(a.flag0 and bm_DDELN_fRelease) shr bp_DDELN_fRelease;
    end;

  procedure set_fRelease(var a : DDELN; __fRelease : word);
    begin
       a.flag0:=a.flag0 or ((__fRelease shl bp_DDELN_fRelease) and bm_DDELN_fRelease);
    end;

  function fDeferUpd(var a : DDELN) : word;
    begin
       fDeferUpd:=(a.flag0 and bm_DDELN_fDeferUpd) shr bp_DDELN_fDeferUpd;
    end;

  procedure set_fDeferUpd(var a : DDELN; __fDeferUpd : word);
    begin
       a.flag0:=a.flag0 or ((__fDeferUpd shl bp_DDELN_fDeferUpd) and bm_DDELN_fDeferUpd);
    end;

  function fAckReq(var a : DDELN) : word;
    begin
       fAckReq:=(a.flag0 and bm_DDELN_fAckReq) shr bp_DDELN_fAckReq;
    end;

  procedure set_fAckReq(var a : DDELN; __fAckReq : word);
    begin
       a.flag0:=a.flag0 or ((__fAckReq shl bp_DDELN_fAckReq) and bm_DDELN_fAckReq);
    end;

  function unused(var a : DDEPOKE) : word;
    begin
       unused:=(a.flag0 and bm_DDEPOKE_unused) shr bp_DDEPOKE_unused;
    end;

  procedure set_unused(var a : DDEPOKE; __unused : word);
    begin
       a.flag0:=a.flag0 or ((__unused shl bp_DDEPOKE_unused) and bm_DDEPOKE_unused);
    end;

  function fRelease(var a : DDEPOKE) : word;
    begin
       fRelease:=(a.flag0 and bm_DDEPOKE_fRelease) shr bp_DDEPOKE_fRelease;
    end;

  procedure set_fRelease(var a : DDEPOKE; __fRelease : word);
    begin
       a.flag0:=a.flag0 or ((__fRelease shl bp_DDEPOKE_fRelease) and bm_DDEPOKE_fRelease);
    end;

  function fReserved(var a : DDEPOKE) : word;
    begin
       fReserved:=(a.flag0 and bm_DDEPOKE_fReserved) shr bp_DDEPOKE_fReserved;
    end;

  procedure set_fReserved(var a : DDEPOKE; __fReserved : word);
    begin
       a.flag0:=a.flag0 or ((__fReserved shl bp_DDEPOKE_fReserved) and bm_DDEPOKE_fReserved);
    end;

  function unused(var a : DDEUP) : word;
    begin
       unused:=(a.flag0 and bm_DDEUP_unused) shr bp_DDEUP_unused;
    end;

  procedure set_unused(var a : DDEUP; __unused : word);
    begin
       a.flag0:=a.flag0 or ((__unused shl bp_DDEUP_unused) and bm_DDEUP_unused);
    end;

  function fAck(var a : DDEUP) : word;
    begin
       fAck:=(a.flag0 and bm_DDEUP_fAck) shr bp_DDEUP_fAck;
    end;

  procedure set_fAck(var a : DDEUP; __fAck : word);
    begin
       a.flag0:=a.flag0 or ((__fAck shl bp_DDEUP_fAck) and bm_DDEUP_fAck);
    end;

  function fRelease(var a : DDEUP) : word;
    begin
       fRelease:=(a.flag0 and bm_DDEUP_fRelease) shr bp_DDEUP_fRelease;
    end;

  procedure set_fRelease(var a : DDEUP; __fRelease : word);
    begin
       a.flag0:=a.flag0 or ((__fRelease shl bp_DDEUP_fRelease) and bm_DDEUP_fRelease);
    end;

  function fReserved(var a : DDEUP) : word;
    begin
       fReserved:=(a.flag0 and bm_DDEUP_fReserved) shr bp_DDEUP_fReserved;
    end;

  procedure set_fReserved(var a : DDEUP; __fReserved : word);
    begin
       a.flag0:=a.flag0 or ((__fReserved shl bp_DDEUP_fReserved) and bm_DDEUP_fReserved);
    end;

  function fAckReq(var a : DDEUP) : word;
    begin
       fAckReq:=(a.flag0 and bm_DDEUP_fAckReq) shr bp_DDEUP_fAckReq;
    end;

  procedure set_fAckReq(var a : DDEUP; __fAckReq : word);
    begin
       a.flag0:=a.flag0 or ((__fAckReq shl bp_DDEUP_fAckReq) and bm_DDEUP_fAckReq);
    end;

  {$i typshrd.inc}
{$endif read_implementation}

