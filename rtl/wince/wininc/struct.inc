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


  Changes :

  22/15/2005 update for wince4.2 port, orinaudo@gmail.com

}

{$ifdef read_interface}

  type
    {$calling default}
    {$i typshrdh.inc}
    {$calling cdecl}
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

     //
     //  The structure of an ACE is a common ace header followed by ace type
     //  specific data.  Pictorally the structure of the common ace header is
     //  as follows:
     //
     //       3 3 2 2 2 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 1
     //       1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
     //      +---------------+-------+-------+---------------+---------------+
     //      |            AceSize            |    AceFlags   |     AceType   |
     //      +---------------+-------+-------+---------------+---------------+
     //
     //  AceType denotes the type of the ace, there are some predefined ace
     //  types
     //
     //  AceSize is the size, in bytes, of ace.
     //
     //  AceFlags are the Ace flags for audit and inheritance, defined shortly.

     ACE_HEADER = record           //winnt
          AceType : BYTE;
          AceFlags : BYTE;
          AceSize : WORD;
       end;
     _ACE_HEADER = ACE_HEADER;
     TACE_HEADER = ACE_HEADER;
     PACE_HEADER = ^ACE_HEADER;

     ACCESS_MASK = DWORD;          //winnt
     PACCESS_MASK = ^ACCESS_MASK;  //+winnt

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

     PACCESS_TOKEN  = PVOID; //+winnt

     ////////////////////////////////////////////////////////////////////////
     //                                                                    //
     //                         ACL  and  ACE                              //
     //                                                                    //
     ////////////////////////////////////////////////////////////////////////

     //
     //  Define an ACL and the ACE format.  The structure of an ACL header
     //  followed by one or more ACEs.  Pictorally the structure of an ACL header
     //  is as follows:
     //
     //       3 3 2 2 2 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 1
     //       1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
     //      +-------------------------------+---------------+---------------+
     //      |            AclSize            |      Sbz1     |  AclRevision  |
     //      +-------------------------------+---------------+---------------+
     //      |              Sbz2             |           AceCount            |
     //      +-------------------------------+-------------------------------+
     //
     //  The current AclRevision is defined to be ACL_REVISION.
     //
     //  AclSize is the size, in bytes, allocated for the ACL.  This includes
     //  the ACL header, ACES, and remaining free space in the buffer.
     //
     //  AceCount is the number of ACES in the ACL.
     //

     ACL = record           //winnt
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
     LPRCECT = PRECT;

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

     FXPT2DOT30 = longint;
     LPFXPT2DOT30 = ^FXPT2DOT30;
     TPFXPT2DOT30 = FXPT2DOT30;
     PPFXPT2DOT30 = ^FXPT2DOT30;

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
     TBitmapFileHeader = BITMAPFILEHEADER;

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

     FILETIME = record
          dwLowDateTime : DWORD;
          dwHighDateTime : DWORD;
       end;
     LPFILETIME = ^FILETIME;
     _FILETIME = FILETIME;
     TFILETIME = FILETIME;
     PFILETIME = ^FILETIME;

     _GET_FILEEX_INFO_LEVELS=(GetFileExInfoStandard,GetFileExMaxInfoLevel); //+winbase
     GET_FILEEX_INFO_LEVELS=_GET_FILEEX_INFO_LEVELS;                        //+winbase

     _FINDEX_INFO_LEVELS = (FindExInfoStandard,FindExInfoMaxInfoLevel); //+winbase
     FINDEX_INFO_LEVELS = _FINDEX_INFO_LEVELS;                          //+winbase

     _FINDEX_SEARCH_OPS = ( FindExSearchNameMatch, FindExSearchLimitToDirectories,
                            FindExSearchLimitToDevices, FindExSearchMaxSearchOp);
     FINDEX_SEARCH_OPS=_FINDEX_SEARCH_OPS;

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

     FIXED = record
          fract : WORD;
          value : integer;
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
     tagPOINTS = POINTS;  //windef
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

     CHARFORMAT = record
          cbSize : UINT;
          dwMask : DWORD;
          dwEffects : DWORD;
          yHeight : LONG;
          yOffset : LONG;
          crTextColor : COLORREF;
          bCharSet : BYTE;
          bPitchAndFamily : BYTE;
          szFaceName : array[0..(LF_FACESIZE)-1] of TCHAR;
       end;
     _charformat = CHARFORMAT;
     Tcharformat = CHARFORMAT;
     Pcharformat = ^CHARFORMAT;

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

     FLOAT128 = record        //+winnt
          LowPart  : int64;
          HighPart : int64;
       end;
     PFLOAT128 = ^FLOAT128;

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
          lfFaceName : array[0..(LF_FACESIZE)-1] of TCHAR;
       end;
     TLOGFONTA = LOGFONTA;
     tagLOGFONTA = LOGFONTA;
     NPLOGFONTA = ^LOGFONTA;
     PLOGFONTA = ^LOGFONTA;

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
     tagLOGFONTW = LOGFONTW;
     TLogFontW = LOGFONTW;
     PLogFontW = ^TLogFontW;

     TLOGFONT = LOGFONTW;
     LOGFONT = LOGFONTW;
     tagLOGFONT = LOGFONTW;
     LPLOGFONT = ^LOGFONTW;
     PLOGFONT = ^LOGFONTW;
     NPLOGFONT = ^LOGFONTW;

     TCHOOSEFONTW = record //+commdlg
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDC : HDC;
          lpLogFont : LPLOGFONTW;
          iPointSize : Integer;
          Flags : DWORD;
          rgbColors : DWORD;
          lCustData : LPARAM;
          lpfnHook : LPCFHOOKPROC;
          lpTemplateName : LPCWSTR;
          hInstance : HINST;
          lpszStyle : LPTSTR;
          nFontType : WORD;
          ___MISSING_ALIGNMENT__ : WORD;
          nSizeMin : Integer;
          nSizeMax : Integer;
       end;
     tagCHOOSEFONTW = TCHOOSEFONTW; //+commdlg
     LPCHOOSEFONTW = ^TCHOOSEFONTW; //+commdlg
     PCHOOSEFONTW = ^TCHOOSEFONTW; //+commdlg

     {CHOOSEFONT = record conflicts with ChooseFont function }
     TCHOOSEFONT = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDC : HDC;
          lpLogFont : LPLOGFONT;
          iPointSize : WINT;
          Flags : DWORD;
          rgbColors : DWORD;
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
       end;
     LPCMINVOKECOMMANDINFO = ^CMINVOKECOMMANDINFO;
     _CMInvokeCommandInfo = CMINVOKECOMMANDINFO;
     TCMInvokeCommandInfo = CMINVOKECOMMANDINFO;
     PCMInvokeCommandInfo = ^CMINVOKECOMMANDINFO;

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

{*
 * Combo Box Structures
 *}
     tagCOMBOBOXINFO = record
       cbSize:DWORD;
       rcItem:RECT;
       rcButton:RECT;
       stateButton:DWORD; // Currently unsupported on CE
       hwndCombo:HWND;
       hwndItem:HWND;
       hwndList:HWND;
     end;
     COMBOBOXINFO = tagCOMBOBOXINFO;
     PCOMBOBOXINFO = ^tagCOMBOBOXINFO;
     LPCOMBOBOXINFO = ^tagCOMBOBOXINFO;

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

     tagINITCOMMONCONTROLSEX = record
          dwSize:DWORD;             // size of this structure
          dwICC:DWORD;              // flags indicating which classes to be initialized
        end;  
     TINITCOMMONCONTROLSEX=tagINITCOMMONCONTROLSEX;
     LPINITCOMMONCONTROLSEX=^tagINITCOMMONCONTROLSEX;

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
          itemData1 : DWORD;
          itemID2 : UINT;
          itemData2 : DWORD;
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
     TCURSORINFO = CONSOLE_CURSOR_INFO;

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

{$ifdef i386}                              //+winnt
  type
     FLOATING_SAVE_AREA = record //~winnt
          ControlWord : DWORD;
          StatusWord : DWORD;
          TagWord : DWORD;
          ErrorOffset : DWORD;
          ErrorSelector : DWORD;
          DataOffset : DWORD;
          DataSelector : DWORD;
          RegisterArea : array[0..SIZE_OF_80387_REGISTERS-1] of BYTE; //~winnt
          Cr0NpxState : DWORD;
       end;
     _FLOATING_SAVE_AREA = FLOATING_SAVE_AREA; //winnt
     TFLOATINGSAVEAREA = FLOATING_SAVE_AREA;   //winnt
     PFLOATING_SAVE_AREA = ^FLOATING_SAVE_AREA;  //~winnt

//
// Context Frame
//
//  This frame has a several purposes: 1) it is used as an argument to
//  NtContinue, 2) is is used to constuct a call frame for APC delivery,
//  and 3) it is used in the user level thread creation routines.
//
//  The layout of the record conforms to a standard call frame.
//

     CONTEXT = record                       //winnt
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

     LDT_ENTRY = record  //~winnt, moved into _X86_
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

{$endif i386}                              //+winnt

{$ifdef _MIPS_}                             //+winnt all block added
//
// Context Frame
//
//  N.B. This frame must be exactly a multiple of 16 bytes in length.
//
//  This frame has a several purposes: 1) it is used as an argument to
//  NtContinue, 2) it is used to constuct a call frame for APC delivery,
//  3) it is used to construct a call frame for exception dispatching
//  in user mode, and 4) it is used in the user level thread creation
//  routines.
//
//  The layout of the record conforms to a standard call frame.
//
  type
    {$ifdef  _MIPS64} //+winnt
     FREG_TYPE =  ULONGLONG;
     REG_TYPE  =  ULONGLONG;
    {$else}
     FREG_TYPE =  DWORD;
     REG_TYPE  =  ULONG;
    {$endif _MIPS64}
     CONTEXT = record                       //+winnt
          Argument : Array[0..3] of REG_TYPE;
          FltF0    : FREG_TYPE;
          FltF1    : FREG_TYPE;
          FltF2    : FREG_TYPE;
          FltF3    : FREG_TYPE;
          FltF4    : FREG_TYPE;
          FltF5    : FREG_TYPE;
          FltF6    : FREG_TYPE;
          FltF7    : FREG_TYPE;
          FltF8    : FREG_TYPE;
          FltF9    : FREG_TYPE;
          FltF10   : FREG_TYPE;
          FltF11   : FREG_TYPE;
          FltF12   : FREG_TYPE;
          FltF13   : FREG_TYPE;
          FltF14   : FREG_TYPE;
          FltF15   : FREG_TYPE;
          FltF16   : FREG_TYPE;
          FltF17   : FREG_TYPE;
          FltF18   : FREG_TYPE;
          FltF19   : FREG_TYPE;
          FltF20   : FREG_TYPE;
          FltF21   : FREG_TYPE;
          FltF22   : FREG_TYPE;
          FltF23   : FREG_TYPE;
          FltF24   : FREG_TYPE;
          FltF25   : FREG_TYPE;
          FltF26   : FREG_TYPE;
          FltF27   : FREG_TYPE;
          FltF28   : FREG_TYPE;
          FltF29   : FREG_TYPE;
          FltF30   : FREG_TYPE;
          FltF31   : FREG_TYPE;

          IntZero : REG_TYPE;
          IntAt   : REG_TYPE;
          IntV0   : REG_TYPE;
          IntV1   : REG_TYPE;
          IntA0   : REG_TYPE;
          IntA1   : REG_TYPE;
          IntA2   : REG_TYPE;
          IntA3   : REG_TYPE;
          IntT0   : REG_TYPE;
          IntT1   : REG_TYPE;
          IntT2   : REG_TYPE;
          IntT3   : REG_TYPE;
          IntT4   : REG_TYPE;
          IntT5   : REG_TYPE;
          IntT6   : REG_TYPE;
          IntT7   : REG_TYPE;
          IntS0   : REG_TYPE;
          IntS1   : REG_TYPE;
          IntS2   : REG_TYPE;
          IntS3   : REG_TYPE;
          IntS4   : REG_TYPE;
          IntS5   : REG_TYPE;
          IntS6   : REG_TYPE;
          IntS7   : REG_TYPE;
          IntT8   : REG_TYPE;
          IntT9   : REG_TYPE;
          IntK0   : REG_TYPE;
          IntK1   : REG_TYPE;
          IntGp   : REG_TYPE;
          IntSp   : REG_TYPE;
          IntS8   : REG_TYPE;
          IntRa   : REG_TYPE;
          IntLo   : REG_TYPE;
          IntHi   : REG_TYPE;

          Fsr     : DWORD;
          Fir     : DWORD;
          Psr     : DWORD;
          ContextFlags : DWORD;
          Fill    : Array[0..1] of DWORD;
      end;

{$endif _MIPS_}                             //+winnt

{$ifdef _PPC_}                             //+winnt
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
     _STACK_FRAME_HEADER = record  // GPR 1 points here
          BackChain : DWORD;       // Addr of previous frame
          Reserved1 : DWORD;       // Reserved

          Parameter0 : DWORD;     // First 8 parameter words are
          Parameter1 : DWORD;     //   always present
          Parameter2 : DWORD;
          Parameter3 : DWORD;
          Parameter4 : DWORD;
          Parameter5 : DWORD;
          Parameter6 : DWORD;
          Parameter7 : DWORD;
       end;

     PSTACK_FRAME_HEADER=^STACK_FRAME_HEADER;
 {$endif _PPC_}                    //~winnt

 {$ifdef _MPPC_} //+winnt all block
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
     _STACK_FRAME_HEADER = record  // GPR 1 points here
          BackChain  : DWORD;      // Addr of previous frame
          GlueSaved1 : DWORD;      // Used by glue code
          GlueSaved2 : DWORD;
          Reserved1  : DWORD;
          Spare1     : DWORD;
          Spare2     : DWORD;

          Parameter0 : DWORD;     // First 8 parameter words are
          Parameter1 : DWORD;     //   always present
          Parameter2 : DWORD;
          Parameter3 : DWORD;
          Parameter4 : DWORD;
          Parameter5 : DWORD;
          Parameter6 : DWORD;
          Parameter7 : DWORD;
       end;

     PSTACK_FRAME_HEADER=^STACK_FRAME_HEADER;

 {$endif _MPPC_} //+winnt

 {$ifdef _IA64_} //+winnt all block
  type
     CONTEXT = record

          ContextFlags : DWORD;
          Fill1 : array[0..2] of DWORD;
          DbI0 : ULONGLONG;              // Instruction debug registers
          DbI1 : ULONGLONG;
          DbI2 : ULONGLONG;
          DbI3 : ULONGLONG;
          DbI4 : ULONGLONG;
          DbI5 : ULONGLONG;
          DbI6 : ULONGLONG;
          DbI7 : ULONGLONG;

          DbD0 : ULONGLONG;              // Data debug registers
          DbD1 : ULONGLONG;
          DbD2 : ULONGLONG;
          DbD3 : ULONGLONG;
          DbD4 : ULONGLONG;
          DbD5 : ULONGLONG;
          DbD6 : ULONGLONG;
          DbD7 : ULONGLONG;

          FltS0 : FLOAT128;             // Lower floating point (f2-f5) - saved (preserved)
          FltS1 : FLOAT128;
          FltS2 : FLOAT128;
          FltS3 : FLOAT128;
          FltT0 : FLOAT128;             // Lower floating point (f6-f15) - temporary (volatile)
          FltT1 : FLOAT128;
          FltT2 : FLOAT128;
          FltT3 : FLOAT128;
          FltT4 : FLOAT128;
          FltT5 : FLOAT128;
          FltT6 : FLOAT128;
          FltT7 : FLOAT128;
          FltT8 : FLOAT128;
          FltT9 : FLOAT128;
          FltS4 : FLOAT128;             // Higher floating point (f16-f31) - saved (preserved)
          FltS5 : FLOAT128;
          FltS6 : FLOAT128;
          FltS7 : FLOAT128;
          FltS8 : FLOAT128;
          FltS9 : FLOAT128;
          FltS10 : FLOAT128;
          FltS11 : FLOAT128;
          FltS12 : FLOAT128;
          FltS13 : FLOAT128;
          FltS14 : FLOAT128;
          FltS15 : FLOAT128;
          FltS16 : FLOAT128;
          FltS17 : FLOAT128;
          FltS18 : FLOAT128;
          FltS19 : FLOAT128;

          FltF32 : FLOAT128;           // Higher floating point (f32-f127) - temporary (volatile)
          FltF33 : FLOAT128;
          FltF34 : FLOAT128;
          FltF35 : FLOAT128;
          FltF36 : FLOAT128;
          FltF37 : FLOAT128;
          FltF38 : FLOAT128;
          FltF39 : FLOAT128;

          FltF40 : FLOAT128;
          FltF41 : FLOAT128;
          FltF42 : FLOAT128;
          FltF43 : FLOAT128;
          FltF44 : FLOAT128;
          FltF45 : FLOAT128;
          FltF46 : FLOAT128;
          FltF47 : FLOAT128;
          FltF48 : FLOAT128;
          FltF49 : FLOAT128;

          FltF50 : FLOAT128;
          FltF51 : FLOAT128;
          FltF52 : FLOAT128;
          FltF53 : FLOAT128;
          FltF54 : FLOAT128;
          FltF55 : FLOAT128;
          FltF56 : FLOAT128;
          FltF57 : FLOAT128;
          FltF58 : FLOAT128;
          FltF59 : FLOAT128;

          FltF60 : FLOAT128;
          FltF61 : FLOAT128;
          FltF62 : FLOAT128;
          FltF63 : FLOAT128;
          FltF64 : FLOAT128;
          FltF65 : FLOAT128;
          FltF66 : FLOAT128;
          FltF67 : FLOAT128;
          FltF68 : FLOAT128;
          FltF69 : FLOAT128;

          FltF70 : FLOAT128;
          FltF71 : FLOAT128;
          FltF72 : FLOAT128;
          FltF73 : FLOAT128;
          FltF74 : FLOAT128;
          FltF75 : FLOAT128;
          FltF76 : FLOAT128;
          FltF77 : FLOAT128;
          FltF78 : FLOAT128;
          FltF79 : FLOAT128;

          FltF80 : FLOAT128;
          FltF81 : FLOAT128;
          FltF82 : FLOAT128;
          FltF83 : FLOAT128;
          FltF84 : FLOAT128;
          FltF85 : FLOAT128;
          FltF86 : FLOAT128;
          FltF87 : FLOAT128;
          FltF88 : FLOAT128;
          FltF89 : FLOAT128;

          FltF90 : FLOAT128;
          FltF91 : FLOAT128;
          FltF92 : FLOAT128;
          FltF93 : FLOAT128;
          FltF94 : FLOAT128;
          FltF95 : FLOAT128;
          FltF96 : FLOAT128;
          FltF97 : FLOAT128;
          FltF98 : FLOAT128;
          FltF99 : FLOAT128;

          FltF100 : FLOAT128;
          FltF101 : FLOAT128;
          FltF102 : FLOAT128;
          FltF103 : FLOAT128;
          FltF104 : FLOAT128;
          FltF105 : FLOAT128;
          FltF106 : FLOAT128;
          FltF107 : FLOAT128;
          FltF108 : FLOAT128;
          FltF109 : FLOAT128;

          FltF110 : FLOAT128;
          FltF111 : FLOAT128;
          FltF112 : FLOAT128;
          FltF113 : FLOAT128;
          FltF114 : FLOAT128;
          FltF115 : FLOAT128;
          FltF116 : FLOAT128;
          FltF117 : FLOAT128;
          FltF118 : FLOAT128;
          FltF119 : FLOAT128;

          FltF120 : FLOAT128;
          FltF121 : FLOAT128;
          FltF122 : FLOAT128;
          FltF123 : FLOAT128;
          FltF124 : FLOAT128;
          FltF125 : FLOAT128;
          FltF126 : FLOAT128;
          FltF127 : FLOAT128;

          StFPSR : ULONGLONG;          // FP status
          StFSR  : ULONGLONG;          // x86 FP status (a copy of AR28)
          StFIR  : ULONGLONG;          // x86 FP status (a copy of AR29)
          StFDR  : ULONGLONG;          // x86 FP status (a copy of AR30)

          IntGp  : ULONGLONG;          // global pointer (r1) - temporary (volatile)
          IntT0  : ULONGLONG;          // integer registers (r2-r3) - temporary (volatile)
          IntT1  : ULONGLONG;
          IntS0  : ULONGLONG;          // integer registers (r4-r7) - saved (preserved)
          IntS1  : ULONGLONG;
          IntS2  : ULONGLONG;
          IntS3  : ULONGLONG;
          IntV0  : ULONGLONG;          // return value (r8) - temporary (volatile)
          IntT2  : ULONGLONG;          // integer registers (r9-r11) - temporary (volatile)
          IntT3  : ULONGLONG;
          IntT4  : ULONGLONG;
          IntSP  : ULONGLONG;          // stack pointer (r12) - special
          IntTeb : ULONGLONG;          // teb (r13) - special
          IntT5  : ULONGLONG;          // integer registers (r14-r31) - temporary (volatile)
          IntT6  : ULONGLONG;
          IntT7  : ULONGLONG;
          IntT8  : ULONGLONG;
          IntT9  : ULONGLONG;
          IntT10 : ULONGLONG;
          IntT11 : ULONGLONG;
          IntT12 : ULONGLONG;
          IntT13 : ULONGLONG;
          IntT14 : ULONGLONG;
          IntT15 : ULONGLONG;
          IntT16 : ULONGLONG;
          IntT17 : ULONGLONG;
          IntT18 : ULONGLONG;
          IntT19 : ULONGLONG;
          IntT20 : ULONGLONG;
          IntT21 : ULONGLONG;
          IntT22 : ULONGLONG;

          IntNats : ULONGLONG;         // Nat bits for general registers
                                       // r1-r31 in bit positions 1 to 31.
          Preds   : ULONGLONG;         // predicates - saved (preserved)
          BrRp    : ULONGLONG;         // return pointer (b0) - saved (preserved)
          BrS0    : ULONGLONG;         // branch registers (b1-b5) - saved (preserved)
          BrS1    : ULONGLONG;
          BrS2    : ULONGLONG;
          BrS3    : ULONGLONG;
          BrS4    : ULONGLONG;

          BrT0    : ULONGLONG;         // branch registers (b6-b7) - temporary (volatile)
          BrT1    : ULONGLONG;
          // iA32 related Interger registers
          SegCSD  : ULONGLONG;         // iA32 CSDescriptor (Ar25)
          SegSSD  : ULONGLONG;         // iA32 SSDescriptor (Ar26)

          // Other application registers
          ApUNAT  : ULONGLONG;         // User Nat collection register - saved (preserved)
          ApLC    : ULONGLONG;         // Loop counter register - saved (preserved)
          ApEC    : ULONGLONG;         // Epilog counter register - saved (preserved)
          ApCCV   : ULONGLONG;         // CMPXCHG value register - temporary (volatile)
          ApDCR   : ULONGLONG;         // Default control register (TBD)

          // Register stack info
          RsPFS   : ULONGLONG;         // Previous function state - saved (preserved)
          RsBSP   : ULONGLONG;         // Backing store pointer - saved (preserved)
          RsBSPSTORE : ULONGLONG;      // BSP Store - saved (preserved)
          RsRSC   : ULONGLONG;         // RSE configuration - temporary (volatile)
          RsRNAT  : ULONGLONG;         // RSE Nat collection register - saved (preserved)
          // iA32 related control registers
          Eflag   : ULONGLONG;         // Eflag  copy of Ar24
          Cflag   : ULONGLONG;         // Cr0+Cr4 copy of Ar27

          // Trap Status Information
          StIPSR  : ULONGLONG;         // Interruption Processor Status
          StIIP   : ULONGLONG;         // Interruption IP
          StIFS   : ULONGLONG;         // Interruption Function State

       end;

     FLOATING_SAVE_AREA = record
          ControlWord : DWORD;
          StatusWord : DWORD;
          TagWord : DWORD;
          ErrorOffset : DWORD;
          ErrorSelector : DWORD;
          DataOffset : DWORD;
          DataSelector : DWORD;
          RegisterArea : array[0..SIZE_OF_80387_REGISTERS-1] of BYTE;
          Cr0NpxState : DWORD;
       end;
     _FLOATING_SAVE_AREA = FLOATING_SAVE_AREA;
     TFLOATINGSAVEAREA = FLOATING_SAVE_AREA;
     PFLOATING_SAVE_AREA = ^FLOATING_SAVE_AREA;

     CONTEXT86 = record
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

     PLABEL_DESCRIPTOR = record
          EntryPoint : ULONGLONG;
          GlobalPointer : ULONGLONG;
       end;
     _PLABEL_DESCRIPTOR = PLABEL_DESCRIPTOR;
     PPLABEL_DESCRIPTOR = ^PLABEL_DESCRIPTOR;


 {$endif _IA64_} //+winnt

 {$ifdef SHx} //+winnt all block added
     //These are the debug or break registers on the SH3
     DEBUG_REGISTERS = RECORD
          BarA  : ULONG;
          BasrA : UCHAR;
          BamrA : UCHAR;
          BbrA  : USHORT;
          BarB  : ULONG;
          BasrB : UCHAR;
          BamrB : UCHAR;
          BbrB  : USHORT;
          BdrB  : ULONG;
          BdmrB : ULONG;
          Brcr  : USHORT;
          Align : USHORT;
       end;
     _DEBUG_REGISTERS=DEBUG_REZGISTERS;
     PDEBUG_REGISTERS=^DEBUG_REZGISTERS;

     CONTEXT = record
          ContextFlags : ULONG;
          PR   : ULONG;
          MACH : ULONG;
          MACL : ULONG;
          GBR  : ULONG;
          R0   : ULONG;
          R1   : ULONG;
          R2   : ULONG;
          R3   : ULONG;
          R4   : ULONG;
          R5   : ULONG;
          R6   : ULONG;
          R7   : ULONG;
          R8   : ULONG;
          R9   : ULONG;
          R10  : ULONG;
          R11  : ULONG;
          R12  : ULONG;
          R13  : ULONG;
          R14  : ULONG;
          R15  : ULONG;

    // N.B. The registers r15 and ra are defined in the integer section,
    //   but are considered part of the control context rather than part of
    //   the integer context.
    //

          Fir  : ULONG;
          Psr  : ULONG;

          {$ifndef SH3e}
          {$ifndef SH4}
          OldStuff : Array[0..1] of ULONG;
          DebugRegisters : DEBUG_REGISTERS;
          DSR  : ULONG;
          MOD_ : ULONG; // MOD replaced by MOD_
          RS   : ULONG;
          RE   : ULONG;
          A0   : ULONG;
          A1   : ULONG;
          M0   : ULONG;
          M1   : ULONG;
          X0   : ULONG;
          X1   : ULONG;
          Y0   : ULONG;
          Y1   : ULONG;
         //  The guard bit registers are 8 bits long, but can only be saved and
         //  restored using 16 or 32 bit moves.  Note, the contents of the guard
         //  bit register are in the upper 8 bits.
          A0G : USHORT;
          A1G : USHORT;
          {$else}
          Fpscr  : ULONG;
          Fpul   : ULONG;
          FRegs  : Array[0..15] of ULONG;
          xFRegs : Array[0..15] of ULONG;
          {$endif SH4}
          {$else}
          Fpscr  : ULONG;
          Fpul   : ULONG;
          FRegs  : Array[0..15] of ULONG;
          {$endif SH3e}
       end;

 {$endif SHx} //+winnt

 {$ifdef CPUARM} //+winnt all block added
     CONTEXT = record
          ContextFlags : ULONG;
          R0  : ULONG;
          R1  : ULONG;
          R2  : ULONG;
          R3  : ULONG;
          R4  : ULONG;
          R5  : ULONG;
          R6  : ULONG;
          R7  : ULONG;
          R8  : ULONG;
          R9  : ULONG;
          R10 : ULONG;
          R11 : ULONG;
          R12 : ULONG;

          SP  : ULONG;
          Lr  : ULONG;
          Pc  : ULONG;
          Psr : ULONG;
          Fpscr : ULONG;
          FpExc : ULONG;
          S       : Array [0..NUM_VFP_REGS] of ULONG;
          FpExtra : Array [0..NUM_EXTRA_CONTROL_REGS-1] of ULONG;
       end;
 {$endif CPUARM} //+winnt

     LPCONTEXT = ^CONTEXT;
     _CONTEXT = CONTEXT;
     TCONTEXT = CONTEXT;
     PCONTEXT = ^CONTEXT;         //winnt

  type

     LIST_ENTRY = record          //winnt
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
          ContextTrackingMode : WINBOOL;
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
          dwData : DWORD;
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

     CPLINFO = record
          idIcon : longint;
          idName : longint;
          idInfo : longint;
          lData : LONG;
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
  (*
   TODO: sockets
  typedef struct _SOCKET_ADDRESS {
    LPSOCKADDR lpSockaddr ;
    INT iSockaddrLength ;
  } SOCKET_ADDRESS,  PSOCKET_ADDRESS,  LPSOCKET_ADDRESS;
   }
  {
  typedef struct _CSADDR_INFO {
    SOCKET_ADDRESS  LocalAddr;
    SOCKET_ADDRESS  RemoteAddr;
    INT             iSocketType;
    INT             iProtocol;
  } CSADDR_INFO;
    *)

     CURRENCYFMT = record
          NumDigits : UINT;
          LeadingZero : UINT;
          Grouping : UINT;
          lpDecimalSep : LPTSTR;
          lpThousandSep : LPTSTR;
          NegativeOrder : UINT;
          PositiveOrder : UINT;
          lpCurrencySymbol : LPTSTR;
       end;
     _currencyfmt = CURRENCYFMT;
     Tcurrencyfmt = CURRENCYFMT;
     Pcurrencyfmt = ^CURRENCYFMT;

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
          cfFormat : integer;
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
          cfFormat : integer;
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
          cfFormat : integer;
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
          uiLo : UINT;
          uiHi : UINT;
          cbData : DWORD;
          Data : array[0..7] of DWORD;
       end;
     tagDDEML_MSG_HOOK_DATA = DDEML_MSG_HOOK_DATA;
     TDDEMLMSGHOOKDATA = DDEML_MSG_HOOK_DATA;
     PDDEMLMSGHOOKDATA = ^DDEML_MSG_HOOK_DATA;

     DDEPOKE = record
          flag0 : word;
          cfFormat : integer;
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
          cfFormat : integer;
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

     // definition of _exception struct - this struct is passed to the matherr
     // routine when a floating point exception is detected
     _EXCEPTION = record  //+stdlib
       type_  : Integer;  //identifier type replaced by Type_ for compilation
       Name   : PChar;    // name of function where error occurred
       arg1   : double;   // first argument to function
       arg2   : double;   // second argument (if any) to function
       retval : double;   // value to be returned by function
      end;

     EXCEPTION_RECORD = record        //winnt
          ExceptionCode : DWORD;
          ExceptionFlags : DWORD;
          ExceptionRecord : ^_EXCEPTION_RECORD;
          ExceptionAddress : PVOID;
          NumberParameters : DWORD;
          ExceptionInformation : array[0..(EXCEPTION_MAXIMUM_PARAMETERS)-1] of Pointer; //~winnt, was DWORD, SDK:ULONG_PTR
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
          u : record
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
          itemData : UINT;
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
          dbco_size : ULONG;
          dbco_devicetype : ULONG;
          dbco_reserved : ULONG;
          dbco_identifier : ULONG;
          dbco_suppfunc : ULONG;
       end;
     PDEV_BROADCAST_OEM = ^DEV_BROADCAST_OEM;
     _DEV_BROADCAST_OEM = DEV_BROADCAST_OEM;
     TDEVBROADCASTOEM = DEV_BROADCAST_OEM;
     PDEVBROADCASTOEM = ^DEV_BROADCAST_OEM;

     DEV_BROADCAST_PORT = record
          dbcp_size : ULONG;
          dbcp_devicetype : ULONG;
          dbcp_reserved : ULONG;
          dbcp_name : array[0..0] of char;
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
          dbcv_size : ULONG;
          dbcv_devicetype : ULONG;
          dbcv_reserved : ULONG;
          dbcv_unitmask : ULONG;
          dbcv_flags : USHORT;
       end;
     PDEV_BROADCAST_VOLUME = ^DEV_BROADCAST_VOLUME;
     _DEV_BROADCAST_VOLUME = DEV_BROADCAST_VOLUME;
     TDEVBROADCASTVOLUME = DEV_BROADCAST_VOLUME;
     PDEVBROADCASTVOLUME = ^DEV_BROADCAST_VOLUME;

     DEVMODEA   = record
                     dmDeviceName   : array[0.. CCHDEVICENAME-1] of BCHAR;
                     dmSpecVersion  : WORD;
                     dmDriverVersion: WORD;
                     dmSize        : WORD;
                     dmDriverExtra  : WORD;
                     dmFields       : DWORD;
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
                     dmFormName     : array [0..CCHFORMNAME-1] of BCHAR;
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
                     dmDisplayOrientation : DWORD;
                   end;
     _DEVICEMODE1=DEVMODEA;
     PDEVMODEA=^DEVMODEA;
     NPDEVMODEA=^DEVMODEA;
     LPDEVMODEA=^DEVMODEA;

     DEVMODEW   = record
                     dmDeviceName   : array[0.. CCHDEVICENAME-1] of WCHAR;
                     dmSpecVersion  : WORD;
                     dmDriverVersion: WORD;
                     dmSize        : WORD;
                     dmDriverExtra  : WORD;
                     dmFields       : DWORD;
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
                     dmDisplayFrequency   : DWORD;
                     dmDisplayOrientation : DWORD;
                   end;

     LPDEVMODEW   = ^DEVMODEW;
     _devicemodeW = DEVMODEW;
     devicemodeW  = DEVMODEW;
     TDeviceModeW = DEVMODEW;
     PDeviceModeW = LPDEVMODEW;
     TDEVMODEW    = DEVMODEW;
     PDEVMODEW    = LPDEVMODEW;

{$IFDEF UNICODE}
     DEVMODE=DEVMODEW;
     LPDEVMODE = ^DEVMODEW;
     _devicemode = DEVMODEW;
     devicemode = DEVMODEW;
     tdevicemode = DEVMODEW;
     tdevicemodeA = DEVMODEW;
     PDeviceModeA = LPDEVMODEW;
     PDeviceMode = LPDEVMODEW;
     TDEVMODE = DEVMODEW;
     PDEVMODE = LPDEVMODEW;
{$ELSE UNICODE}
     DEVMODE=DEVMODEA;
     LPDEVMODE = ^DEVMODEA;
     _devicemode = DEVMODEA;
     devicemode = DEVMODEA;
     tdevicemode = DEVMODEA;
     tdevicemodeA = DEVMODEA;
     PDeviceModeA = LPDEVMODEA;
     PDeviceMode = LPDEVMODEA;
     TDEVMODE = DEVMODEA;
     PDEVMODE = LPDEVMODEA;
{$ENDIF UNICODE}

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

     LARGE_INTEGER = record    //winnt
        case byte of
          0: (LowPart : DWORD;
              HighPart : LONG);
          1: (QuadPart : LONGLONG);
       end;
     PLARGE_INTEGER = ^LARGE_INTEGER; //winnt
     _LARGE_INTEGER = LARGE_INTEGER;  //winnt

     TLargeInteger = Int64;
     PLargeInteger = ^TLargeInteger;

     ULARGE_INTEGER = record
        case byte of
          0: (LowPart : DWORD;
              HighPart : DWORD);
          1: (QuadPart : LONGLONG);
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
          ReadCount : DWORD;
          WriteCount : DWORD;
          QueueDepth : DWORD;
       end;
     _DISK_PERFORMANCE = DISK_PERFORMANCE;
     TDISKPERFORMANCE = DISK_PERFORMANCE;
     PDISKPERFORMANCE = ^DISK_PERFORMANCE;

     DLGITEMTEMPLATE = packed record
          style : DWORD;
          dwExtendedStyle : DWORD;
          x : integer;
          y : integer;
          cx : integer;
          cy : integer;
          id : WORD;
       end;
     LPDLGITEMTEMPLATE = ^DLGITEMTEMPLATE;
     TDLGITEMTEMPLATE = DLGITEMTEMPLATE;
     PDLGITEMTEMPLATE = ^DLGITEMTEMPLATE;

     DLGTEMPLATE = packed record
          style : DWORD;
          dwExtendedStyle : DWORD;
          cdit : WORD;
          x : integer;
          y : integer;
          cx : integer;
          cy : integer;
       end;
     LPDLGTEMPLATE = ^DLGTEMPLATE;
     LPCDLGTEMPLATE = ^DLGTEMPLATE;
     LPCDLGTEMPLATEW = ^DLGTEMPLATE; //+winuser
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

     DOCINFO = record //+wingdi
          cbSize : Integer;
          lpszDocName : LPCWSTR;
          lpszOutput : LPCWSTR;
          lpszDatatype : LPCWSTR;
          fwType : DWORD;
       end;
     TDOCINFO = DOCINFO;
     PDOCINFO = ^DOCINFO;
     DOCINFOW = DOCINFO;
     TDOCINFOW = DOCINFOW;
     _DOCINFOW=DOCINFOW; //+wingdi
     LPDOCINFOW=^PDOCINFO; //+wingdi
     PDOCINFOW=^PDOCINFO; //+wingdi

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
          itemData : DWORD;
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
          PartitionType : BYTE;
          BootIndicator : BOOLEAN;
          RecognizedPartition : BOOLEAN;
          RewritePartition : BOOLEAN;
          StartingOffset : LARGE_INTEGER;
          PartitionLength : LARGE_INTEGER;
          HiddenSectors : LARGE_INTEGER;
       end;
     _PARTITION_INFORMATION = PARTITION_INFORMATION;
     TPARTITIONINFORMATION = PARTITION_INFORMATION;
     PPARTITIONINFORMATION = ^PARTITION_INFORMATION;

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
          dwCookie : DWORD;
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
          offBitsSrc : DWORD;
          cbBitsSrc : DWORD;
       end;
     tagEMRBITBLT = EMRBITBLT;
     TEMRBITBLT = EMRBITBLT;
     PEMRBITBLT = ^EMRBITBLT;

     LOGBRUSH = record
          lbStyle : UINT;
          lbColor : COLORREF;
          lbHatch : LONG;
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

     LOGCOLORSPACE = record
          lcsSignature : DWORD;
          lcsVersion : DWORD;
          lcsSize : DWORD;
          lcsCSType : LCSCSTYPE;
          lcsIntent : LCSGAMUTMATCH;
          lcsEndpoints : CIEXYZTRIPLE;
          lcsGammaRed : DWORD;
          lcsGammaGreen : DWORD;
          lcsGammaBlue : DWORD;
          lcsFilename : array[0..(MAX_PATH)-1] of TCHAR;
       end;
     LPLOGCOLORSPACE = ^LOGCOLORSPACE;
     tagLOGCOLORSPACE = LOGCOLORSPACE;
     TLOGCOLORSPACE = LOGCOLORSPACE;
     TLOGCOLORSPACEA = LOGCOLORSPACE;
     PLOGCOLORSPACE = ^LOGCOLORSPACE;

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
     PEMRCREATEDIBPATTERNBRUSHPT = EMRCREATEDIBPATTERNBRUSHPT;

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

     EXTLOGFONT = record
          elfLogFont : LOGFONT;
          elfFullName : array[0..(LF_FULLFACESIZE)-1] of BCHAR;
          elfStyle : array[0..(LF_FACESIZE)-1] of BCHAR;
          elfVersion : DWORD;
          elfStyleSize : DWORD;
          elfMatch : DWORD;
          elfReserved : DWORD;
          elfVendorId : array[0..(ELF_VENDOR_SIZE)-1] of BYTE;
          elfCulture : DWORD;
          elfPanose : PANOSE;
       end;
     tagEXTLOGFONT = EXTLOGFONT;
     TEXTLOGFONT = EXTLOGFONT;
     PEXTLOGFONT = ^EXTLOGFONT;

     EMREXTCREATEFONTINDIRECTW = record
          emr : EMR;
          ihFont : DWORD;
          elfw : EXTLOGFONT;
       end;
     tagEMREXTCREATEFONTINDIRECTW = EMREXTCREATEFONTINDIRECTW;
     TEMREXTCREATEFONTINDIRECTW = EMREXTCREATEFONTINDIRECTW;
     PEMREXTCREATEFONTINDIRECTW = ^EMREXTCREATEFONTINDIRECTW;


     EXTLOGPEN = record
          elpPenStyle : UINT;
          elpWidth : UINT;
          elpBrushStyle : UINT;
          elpColor : COLORREF;
          elpHatch : LONG;
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

     SIZEL = SIZE;        //windef
     TSIZEL = SIZE;       //windef
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
     PEMRMODIFYWORLDTRANSFORM = EMRMODIFYWORLDTRANSFORM;

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
          apts : array[0..0] of POINTL;
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
          idFrom : UINT;
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

     ENUM_SERVICE_STATUS = record
          lpServiceName : LPTSTR;
          lpDisplayName : LPTSTR;
          ServiceStatus : SERVICE_STATUS;
       end;
     LPENUM_SERVICE_STATUS = ^ENUM_SERVICE_STATUS;
     _ENUM_SERVICE_STATUS = ENUM_SERVICE_STATUS;
     TENUMSERVICESTATUS = ENUM_SERVICE_STATUS;
     PENUMSERVICESTATUS = ^ENUM_SERVICE_STATUS;

     ENUMLOGFONT = record
          elfLogFont : LOGFONT;
          elfFullName : array[0..(LF_FULLFACESIZE)-1] of BCHAR;
          elfStyle : array[0..(LF_FACESIZE)-1] of BCHAR;
       end;
     tagENUMLOGFONT = ENUMLOGFONT;
     TENUMLOGFONT = ENUMLOGFONT;
     PENUMLOGFONT = ^ENUMLOGFONT;

     ENUMLOGFONTEX = record
          elfLogFont : LOGFONT;
          elfFullName : array[0..(LF_FULLFACESIZE)-1] of BCHAR;
          elfStyle : array[0..(LF_FACESIZE)-1] of BCHAR;
          elfScript : array[0..(LF_FACESIZE)-1] of BCHAR;
       end;
     tagENUMLOGFONTEX = ENUMLOGFONTEX;
     TENUMLOGFONTEX = ENUMLOGFONTEX;
     PENUMLOGFONTEX = ^ENUMLOGFONTEX;
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

     EXCEPTION_POINTERS = record                //winnt
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

     FMS_GETDRIVEINFO = record
          dwTotalSpace : DWORD;
          dwFreeSpace : DWORD;
          szPath : array[0..259] of TCHAR;
          szVolume : array[0..13] of TCHAR;
          szShare : array[0..127] of TCHAR;
       end;
     _FMS_GETDRIVEINFO = FMS_GETDRIVEINFO;
     TFMSGETDRIVEINFO = FMS_GETDRIVEINFO;
     PFMSGETDRIVEINFO = ^FMS_GETDRIVEINFO;

     FMS_GETFILESEL = record
          ftTime : FILETIME;
          dwSize : DWORD;
          bAttr : BYTE;
          szName : array[0..259] of TCHAR;
       end;
     _FMS_GETFILESEL = FMS_GETFILESEL;
     TFMSGETFILESEL = FMS_GETFILESEL;
     PFMSGETFILESEL = ^FMS_GETFILESEL;

     FMS_LOAD = record
          dwSize : DWORD;
          szMenuName : array[0..(MENU_TEXT_LEN)-1] of TCHAR;
          hMenu : HMENU;
          wMenuDelta : UINT;
       end;
     _FMS_LOAD = FMS_LOAD;
     TFMSLOAD = FMS_LOAD;
     PFMSLOAD = ^FMS_LOAD;

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

     GCP_RESULTS = record
          lStructSize : DWORD;
          lpOutString : LPTSTR;
          lpOrder : ^UINT;
          lpDx : ^WINT;
          lpCaretPos : ^WINT;
          lpClass : LPTSTR;
          lpGlyphs : ^UINT;
          nGlyphs : UINT;
          nMaxFit : UINT;
       end;
     LPGCP_RESULTS = ^GCP_RESULTS;
     tagGCP_RESULTS = GCP_RESULTS;
     TGCPRESULTS = GCP_RESULTS;
     PGCPRESULTS = ^GCP_RESULTS;
//
//  Define the generic mapping array.  This is used to denote the
//  mapping of each generic access right to a specific access mask.
//
     GENERIC_MAPPING = record       //winnt
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
          gmCellIncX : integer;
          gmCellIncY : integer;
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
          dwContextId : DWORD;
          MousePos : POINT;
       end;
     LPHELPINFO = ^HELPINFO;
     tagHELPINFO = HELPINFO;
     THELPINFO = HELPINFO;
     PHELPINFO = ^HELPINFO;

     HELPWININFO = record
          wStructSize : longint;
          x : longint;
          y : longint;
          dx : longint;
          dy : longint;
          wMax : longint;
          rgchMember : array[0..1] of TCHAR;
       end;
     THELPWININFO = HELPWININFO;
     PHELPWININFO = ^HELPWININFO;

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

     IMAGELISTDRAWPARAMS = record  //+commctrl
          cbSize : DWORD;
          himl : HIMAGELIST;
          i : Integer;
          hdcDst : HDC;
          x : Integer;
          y : Integer;
          cx : Integer;
          cy : Integer;
          xBitmap : Integer;        // x offest from the upperleft of bitmap
          yBitmap : Integer;        // y offset from the upperleft of bitmap
          rgbBk  : COLORREF;
          rgbFg  : COLORREF;
          fStyle  : UINT;
          dwRop : DWORD;
       end;  
       
     _IMAGELISTDRAWPARAMS=IMAGELISTDRAWPARAMS;
     PIMAGELISTDRAWPARAMS=^IMAGELISTDRAWPARAMS;

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

     SID_AND_ATTRIBUTES = record  //~winnt, moved with SID declarations
          Sid : PSID;
          Attributes : DWORD;
       end;
     _SID_AND_ATTRIBUTES = SID_AND_ATTRIBUTES;
     TSIDANDATTRIBUTES = SID_AND_ATTRIBUTES;
     PSIDANDATTRIBUTES = ^SID_AND_ATTRIBUTES;

     SID_AND_ATTRIBUTES_ARRAY = array[0..(ANYSIZE_ARRAY)-1] of SID_AND_ATTRIBUTES; //winnt
     PSID_AND_ATTRIBUTES_ARRAY = ^SID_AND_ATTRIBUTES_ARRAY;
     TSIDANDATTRIBUTESARRAY = SID_AND_ATTRIBUTES_ARRAY;
     PSIDANDATTRIBUTESARRAY = ^SID_AND_ATTRIBUTES_ARRAY;

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

     LUID_AND_ATTRIBUTES = record  //winnt
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
{$IFDEF UNICODE}
          pszText : LPWSTR;
{$ELSE UNICODE}
          pszText : LPTSTR;
{$ENDIF UNICODE}
          cchTextMax : longint;
          iSubItem : longint;
          iImage : Longint;    //#if (_WIN32_IE >= 0x0300)
          iOrder : longint;    //#if (_WIN32_IE >= 0x0300)
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
{$IFDEF UNICODE}
          pszText : LPWSTR;
{$ELSE UNICODE}
          pszText : LPTSTR;
{$ENDIF UNICODE}
          cchTextMax : longint;
          iImage : longint;
          lParam : LPARAM;
          iIndent: Longint;    //#if (_WIN32_IE >= 0x0300)
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

     MAT2 = record
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
          itemData : DWORD;
       end;
     LPMEASUREITEMSTRUCT = ^MEASUREITEMSTRUCT;
     tagMEASUREITEMSTRUCT = MEASUREITEMSTRUCT;
     TMEASUREITEMSTRUCT = MEASUREITEMSTRUCT;
     PMEASUREITEMSTRUCT = ^MEASUREITEMSTRUCT;

     MEMORY_BASIC_INFORMATION = record
          BaseAddress : PVOID;
          AllocationBase : PVOID;
          AllocationProtect : DWORD;
          RegionSize : DWORD;
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
          dwTotalPhys : DWORD;
          dwAvailPhys : DWORD;
          dwTotalPageFile : DWORD;
          dwAvailPageFile : DWORD;
          dwTotalVirtual : DWORD;
          dwAvailVirtual : DWORD;
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

     MENUITEMINFO = record
          cbSize : UINT;
          fMask : UINT;
          fType : UINT;
          fState : UINT;
          wID : UINT;
          hSubMenu : HMENU;
          hbmpChecked : HBITMAP;
          hbmpUnchecked : HBITMAP;
          dwItemData : DWORD;
          dwTypeData : LPTSTR;
          cch : UINT;
       end;
     LPMENUITEMINFO = ^MENUITEMINFO;
     LPCMENUITEMINFO = ^MENUITEMINFO;
     tagMENUITEMINFO = MENUITEMINFO;
     TMENUITEMINFO = MENUITEMINFO;
     TMENUITEMINFOW = MENUITEMINFO;
     PMENUITEMINFO = ^MENUITEMINFO;
     PMENUITEMINFOW = ^MENUITEMINFO;

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
          dwData1 : DWORD;
          dwData2 : DWORD;
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

     MONHSZSTRUCT = record
          cb : UINT;
          fsAction : WINBOOL;
          dwTime : DWORD;
          hsz : HSZ;
          hTask : HANDLE;
          str : array[0..0] of TCHAR;
       end;
     tagMONHSZSTRUCT = MONHSZSTRUCT;
     TMONHSZSTRUCT = MONHSZSTRUCT;
     PMONHSZSTRUCT = ^MONHSZSTRUCT;

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
          dwExtraInfo : DWORD;
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

     MSGBOXCALLBACK = procedure (lpHelpInfo:LPHELPINFO);
     TMSGBOXCALLBACK = MSGBOXCALLBACK;

     MSGBOXPARAMS = record
          cbSize : UINT;
          hwndOwner : HWND;
          hInstance : HINST;
          lpszText : LPCSTR;
          lpszCaption : LPCSTR;
          dwStyle : DWORD;
          lpszIcon : LPCSTR;
          dwContextHelpId : DWORD;
          lpfnMsgBoxCallback : MSGBOXCALLBACK;
          dwLanguageId : DWORD;
       end;
     LPMSGBOXPARAMS = ^MSGBOXPARAMS;
     TMSGBOXPARAMS = MSGBOXPARAMS;
     TMSGBOXPARAMSA = MSGBOXPARAMS;
     PMSGBOXPARAMS = ^MSGBOXPARAMS;

     MSGFILTER = record
          nmhdr : NMHDR;
          msg : UINT;
          wParam : WPARAM;
          lParam : LPARAM;
       end;
     _msgfilter = MSGFILTER;
     Tmsgfilter = MSGFILTER;
     Pmsgfilter = ^MSGFILTER;

     MULTIKEYHELP = record
          mkSize : DWORD;
          mkKeylist : TCHAR;
          szKeyphrase : array[0..0] of TCHAR;
       end;
     tagMULTIKEYHELP = MULTIKEYHELP;
     TMULTIKEYHELP = MULTIKEYHELP;
     PMULTIKEYHELP = ^MULTIKEYHELP;

     EXTENDED_NAME_FORMAT=(
     // unknown name type
     NameUnknown = 0,
     // CN=Spencer Katt, OU=Software, OU=Engineering, O=Widget, C=US
     NameFullyQualifiedDN = 1,
     // Engineering\SpencerK
     NameSamCompatible = 2,
     // Probably "Spencer Katt" but could be something else.  I.e. The
     // display name is not necessarily the defining RDN.
     NameDisplay = 3,
     // xxx@engineering.widget.com where xxx could be "SpencerK" or
     // anything else.  Could be multi-valued to handle migration and aliasing.
     NameDomainSimple = 4,
     // xxx@widget.com where xxx could be "SpencerK" or anything else.
     // Could be multi-valued to handle migration and aliasing.
     NameEnterpriseSimple = 5,
     // String-ized GUID as returned by IIDFromString().
     // eg: {4fa050f0-f561-11cf-bdd9-00aa003a77b6}
     NameUniqueId = 6,
     // engineering.widget.com/software/spencer katt
     NameCanonical = 7,
     // local logon name
     NameWindowsCeLocal = longint($80000001));              //+winbase
     PEXTENDED_NAME_FORMAT=^EXTENDED_NAME_FORMAT;  //+winbase

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

     NETRESOURCE = record
          dwScope : DWORD;
          dwType : DWORD;
          dwDisplayType : DWORD;
          dwUsage : DWORD;
          lpLocalName : LPTSTR;
          lpRemoteName : LPTSTR;
          lpComment : LPTSTR;
          lpProvider : LPTSTR;
       end;
     LPNETRESOURCE = ^NETRESOURCE;
     _NETRESOURCE = NETRESOURCE;
     TNETRESOURCE = NETRESOURCE;
     TNETRESOURCEA = NETRESOURCE;
     PNETRESOURCE = ^NETRESOURCE;
     PNETRESOURCEA = ^NETRESOURCE;

     NETRESOURCEW = record  //+winnetwk
          dwScope : DWORD;
          dwType : DWORD;
          dwDisplayType : DWORD;
          dwUsage : DWORD;
          lpLocalName : LPWSTR;
          lpRemoteName : LPWSTR;
          lpComment : LPWSTR;
          lpProvider : LPWSTR;
       end;
     LPNETRESOURCEW = ^NETRESOURCEW; //+winnetwk
     _NETRESOURCEW = NETRESOURCEW;   //+winnetwk

     NEWCPLINFO = record
          dwSize : DWORD;
          dwFlags : DWORD;
          dwHelpContext : DWORD;
          lData : LONG;
          hIcon : HICON;
          szName : array[0..31] of TCHAR;
          szInfo : array[0..63] of TCHAR;
          szHelpFile : array[0..127] of TCHAR;
       end;
     tagNEWCPLINFO = NEWCPLINFO;
     TNEWCPLINFO = NEWCPLINFO;
     PNEWCPLINFO = ^NEWCPLINFO;

     NEWTEXTMETRIC = record
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
          tmFirstChar : BCHAR;
          tmLastChar : BCHAR;
          tmDefaultChar : BCHAR;
          tmBreakChar : BCHAR;
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
     tagNEWTEXTMETRIC = NEWTEXTMETRIC;
     TNEWTEXTMETRIC = NEWTEXTMETRIC;
     PNEWTEXTMETRIC = ^NEWTEXTMETRIC;

     NEWTEXTMETRICEX = record
          ntmentm : NEWTEXTMETRIC;
          ntmeFontSignature : FONTSIGNATURE;
       end;
     tagNEWTEXTMETRICEX = NEWTEXTMETRICEX;
     TNEWTEXTMETRICEX = NEWTEXTMETRICEX;
     PNEWTEXTMETRICEX = ^NEWTEXTMETRICEX;

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

     GUID = system.tguid;        //winnt
     LPGUID = ^GUID;
     _GUID = GUID;
     TGUID = GUID;
     PGUID = ^GUID;
     REFGUID = ^GUID;

     CLSID = GUID;
     LPCLSID = ^CLSID;
     TCLSID = CLSID;
     PCLSID = ^CLSID;
     REFCLSID = ^CLSID;

     FMTID = GUID;
     LPFMTID = ^FMTID;
     REFFMTID = ^FMTID;


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

     NUMBERFMT = record
          NumDigits : UINT;
          LeadingZero : UINT;
          Grouping : UINT;
          lpDecimalSep : LPTSTR;
          lpThousandSep : LPTSTR;
          NegativeOrder : UINT;
       end;
     _numberfmt = NUMBERFMT;
     Tnumberfmt = NUMBERFMT;
     Pnumberfmt = ^NUMBERFMT;

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

     OPENFILENAMEW = record     //+commdlg
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HINST;
          lpstrFilter : LPCWSTR;
          lpstrCustomFilter : LPWSTR;
          nMaxCustFilter : DWORD;
          nFilterIndex : DWORD;
          lpstrFile : LPWSTR;
          nMaxFile : DWORD;
          lpstrFileTitle : LPWSTR;
          nMaxFileTitle : DWORD;
          lpstrInitialDir : LPCWSTR;
          lpstrTitle : LPCWSTR;
          Flags : DWORD;
          nFileOffset : WORD;
          nFileExtension : WORD;
          lpstrDefExt : LPCWSTR;
          lCustData : LPARAM;
          lpfnHook : LPOFNHOOKPROC;
          lpTemplateName : LPCWSTR;
       end;
     LPOPENFILENAMEW = ^OPENFILENAMEW; //+commdlg
     tagOFNW = OPENFILENAMEW; //+commdlg
     LPOPENFILENAME = ^OPENFILENAMEW;
     TOPENFILENAME = OPENFILENAMEW;
     POPENFILENAME = ^OPENFILENAMEW;
     TOPENFILENAMEW = OPENFILENAMEW;
     POPENFILENAMEW = ^OPENFILENAMEW;

     tagOFN = OPENFILENAMEW;
     TOFN = OPENFILENAMEW;
     POFN = ^OPENFILENAMEW;

     OFNOTIFY = record
          hdr : NMHDR;
          lpOFN : LPOPENFILENAME;
          pszFile : LPTSTR;
       end;
     LPOFNOTIFY = ^OFNOTIFY;
     _OFNOTIFY = OFNOTIFY;
     TOFNOTIFY = OFNOTIFY;
     POFNOTIFY = ^OFNOTIFY;

     OSVERSIONINFO = record
          dwOSVersionInfoSize : DWORD;
          dwMajorVersion : DWORD;
          dwMinorVersion : DWORD;
          dwBuildNumber : DWORD;
          dwPlatformId : DWORD;
          szCSDVersion : array[0..127] of TCHAR;
       end;
     LPOSVERSIONINFO = ^OSVERSIONINFO;
     _OSVERSIONINFO = OSVERSIONINFO;
     TOSVERSIONINFO = OSVERSIONINFO;
     POSVERSIONINFO = ^OSVERSIONINFO;

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
     POSVERSIONINFOW = ^OSVERSIONINFOW;





     TEXTMETRIC = record
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
          tmFirstChar : BCHAR;
          tmLastChar : BCHAR;
          tmDefaultChar : BCHAR;
          tmBreakChar : BCHAR;
          tmItalic : BYTE;
          tmUnderlined : BYTE;
          tmStruckOut : BYTE;
          tmPitchAndFamily : BYTE;
          tmCharSet : BYTE;
       end;
     LPTEXTMETRIC = ^TEXTMETRIC;
     tagTEXTMETRIC = TEXTMETRIC;
     TTEXTMETRIC = TEXTMETRIC;
     PTEXTMETRIC = ^TEXTMETRIC;

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
     TTEXTMETRICW = TEXTMETRICW;
     PTEXTMETRICW = ^TEXTMETRICW;


     OUTLINETEXTMETRIC = record
          otmSize : UINT;
          otmTextMetrics : TEXTMETRIC;
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
     LPOUTLINETEXTMETRIC = ^OUTLINETEXTMETRIC;
     _OUTLINETEXTMETRIC = OUTLINETEXTMETRIC;
     TOUTLINETEXTMETRIC = OUTLINETEXTMETRIC;
     POUTLINETEXTMETRIC = ^OUTLINETEXTMETRIC;

     OVERLAPPED = record
          Internal : DWORD;
          InternalHigh : DWORD;
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

     TPAGESETUPDLGW = record    //+commdlg
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDevMode : HGLOBAL;
          hDevNames : HGLOBAL;
          Flags : DWORD;
          ptPaperSize : POINT;   // ignored in CE
          rtMinMargin : RECT;
          rtMargin : RECT;
          hInstance : HINST;
          lCustData : LPARAM;
          lpfnPageSetupHook : LPPAGESETUPHOOK;
          lpfnPagePaintHook : LPPAGEPAINTHOOK; // ignored in CE
          lpPageSetupTemplateName : LPCWSTR;
          hPageSetupTemplate : HGLOBAL;
       end;
     LPPAGESETUPDLGW = ^TPAGESETUPDLGW; //+commdlg
     tagPSDW = TPAGESETUPDLGW;          //+commdlg

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

     POLYTEXT = record
          x : longint;
          y : longint;
          n : UINT;
          lpstr : LPCTSTR;
          uiFlags : UINT;
          rcl : RECT;
          pdx : ^longint;
       end;
     _POLYTEXT = POLYTEXT;
     TPOLYTEXT = POLYTEXT;
     PPOLYTEXT = ^POLYTEXT;

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
     TPRINTDLG = packed record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDevMode : HANDLE;
          hDevNames : HANDLE;
          hDC : HDC;
          Flags : DWORD;
          nFromPage : WORD;
          nToPage : WORD;
          nMinPage : WORD;
          nMaxPage : WORD;
          nCopies : WORD;
          hInstance : HINST;
          lCustData : DWORD;
          lpfnPrintHook : LPPRINTHOOKPROC;
          lpfnSetupHook : LPSETUPHOOKPROC;
          lpPrintTemplateName : LPCTSTR;
          lpSetupTemplateName : LPCTSTR;
          hPrintTemplate : HANDLE;
          hSetupTemplate : HANDLE;
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
          dwCommittedSize : DWORD;
          dwUnCommittedSize : DWORD;
          lpFirstBlock : LPVOID;
          lpLastBlock : LPVOID;
          hMem : HANDLE;
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

     LPFNPSPCALLBACK = function (_para1:HWND; _para2:UINT; _para3:LPVOID):UINT;
     TFNPSPCALLBACK = LPFNPSPCALLBACK;

     PROPSHEETPAGEA = record
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
     LPPROPSHEETPAGEA = ^PROPSHEETPAGEA;
     LPCPROPSHEETPAGEA = ^PROPSHEETPAGEA;

     PROPSHEETPAGEW = record
          dwSize : DWORD;
          dwFlags : DWORD;
          hInstance : HINST;
          case longint of
            0 : (pszTemplate : LPCWSTR);
            1 : (pResource : LPCDLGTEMPLATE;
                 case longint of
                    0 : (hIcon : HICON);
                    1 : (pszIcon : LPCWSTR;
                         pszTitle : LPCWSTR;
                         pfnDlgProc : DLGPROC;
                         lParam : LPARAM;
                         pfnCallback : LPFNPSPCALLBACK;
                         pcRefParent : ^UINT;
                        );
                );
       end;
     LPPROPSHEETPAGEW = ^PROPSHEETPAGEW;
     LPCPROPSHEETPAGEW = ^PROPSHEETPAGEW;

{$IFDEF UNICODE}
     LPPROPSHEETPAGE = ^PROPSHEETPAGEW;
     LPCPROPSHEETPAGE = ^PROPSHEETPAGEW;
     _PROPSHEETPAGE = PROPSHEETPAGEW;
     PROPSHEETPAGE = PROPSHEETPAGEW;
     TPROPSHEETPAGE = PROPSHEETPAGEW;
     PPROPSHEETPAGE = ^PROPSHEETPAGEW;
{$ELSE UNICODE}
     LPPROPSHEETPAGE = ^PROPSHEETPAGEA;
     LPCPROPSHEETPAGE = ^PROPSHEETPAGEA;
     _PROPSHEETPAGE = PROPSHEETPAGEA;
     PROPSHEETPAGE = PROPSHEETPAGEA;
     TPROPSHEETPAGE = PROPSHEETPAGEA;
     PPROPSHEETPAGE = ^PROPSHEETPAGEA;
{$ENDIF UNICODE}

     emptyrecord = record
       end;
     lpemptyrecord = ^emptyrecord;
     HPROPSHEETPAGE = ^emptyrecord;

     PROPSHEETHEADERA = record //prsht
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
     LPPROPSHEETHEADERA = ^PROPSHEETHEADERA;
     LPCPROPSHEETHEADERA = ^PROPSHEETHEADERA;
     PPROPSHEETHEADERA = ^PROPSHEETHEADERA;

     PROPSHEETHEADERW = record //prsht
          dwSize : DWORD;
          dwFlags : DWORD;
          hwndParent : HWND;
          hInstance : HINST;
          case longint of
              0 : (hIcon : HICON);
              1 : (pszIcon : LPCWSTR;
                   pszCaption : LPCWSTR;
                   nPages : UINT;
                   case longint of
                      0 : (nStartPage : UINT);
                      1 : (pStartPage : LPCWSTR;
                           case longint of
                              0 : (ppsp : LPCPROPSHEETPAGE);
                              1 : (phpage : ^HPROPSHEETPAGE;
                                   pfnCallback : PFNPROPSHEETCALLBACK;
                                  );
                          );
                  );
       end;

     LPPROPSHEETHEADERW = ^PROPSHEETHEADERW;
     LPCPROPSHEETHEADERW = ^PROPSHEETHEADERW;
     PPROPSHEETHEADERW = ^PROPSHEETHEADERW;

{$IFDEF UNICODE}
     _PROPSHEETHEADER = PROPSHEETHEADERW;
     PROPSHEETHEADER = PROPSHEETHEADERW;
     TPROPSHEETHEADER = PROPSHEETHEADERW;
     PPROPSHEETHEADER = ^PROPSHEETHEADERW;
     LPPROPSHEETHEADER = ^PROPSHEETHEADERW;
     LPCPROPSHEETHEADER = ^PROPSHEETHEADERW;
{$ELSE UNICODE}
     _PROPSHEETHEADER = PROPSHEETHEADERA;
     PROPSHEETHEADER = PROPSHEETHEADERA;
     TPROPSHEETHEADER = PROPSHEETHEADERA;
     PPROPSHEETHEADER = ^PROPSHEETHEADERA;
     LPPROPSHEETHEADER = ^PROPSHEETHEADERA;
     LPCPROPSHEETHEADER = ^PROPSHEETHEADERA;
{$ENDIF UNICODE}


     { PropertySheet callbacks  }
     LPFNADDPROPSHEETPAGE = function (_para1:HPROPSHEETPAGE; _para2:LPARAM):WINBOOL;
     TFNADDPROPSHEETPAGE = LPFNADDPROPSHEETPAGE;

     LPFNADDPROPSHEETPAGES = function (_para1:LPVOID; _para2:LPFNADDPROPSHEETPAGE; _para3:LPARAM):WINBOOL;
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

     QUERY_SERVICE_LOCK_STATUS = record
          fIsLocked : DWORD;
          lpLockOwner : LPTSTR;
          dwLockDuration : DWORD;
       end;
     LPQUERY_SERVICE_LOCK_STATUS = ^QUERY_SERVICE_LOCK_STATUS;
     _QUERY_SERVICE_LOCK_STATUS = QUERY_SERVICE_LOCK_STATUS;
     TQUERYSERVICELOCKSTATUS = QUERY_SERVICE_LOCK_STATUS;
     PQUERYSERVICELOCKSTATUS = ^QUERY_SERVICE_LOCK_STATUS;

     RASAMB = record
          dwSize : DWORD;
          dwError : DWORD;
          szNetBiosError : array[0..(NETBIOS_NAME_LEN + 1)-1] of TCHAR;
          bLana : BYTE;
       end;
     _RASAMB = RASAMB;
     TRASAMB = RASAMB;
     PRASAMB = ^RASAMB;

     RASCONN = record
          dwSize : DWORD;
          hrasconn : HRASCONN;
          szEntryName : array[0..(RAS_MaxEntryName + 1)-1] of TCHAR;
          szDeviceType : array[0..(RAS_MaxDeviceType + 1)-1] of CHAR;
          szDeviceName : array[0..(RAS_MaxDeviceName + 1)-1] of CHAR;
       end;
     _RASCONN = RASCONN;
     TRASCONN = RASCONN;
     PRASCONN = ^RASCONN;

     RASCONNSTATUS = record
          dwSize : DWORD;
          rasconnstate : RASCONNSTATE;
          dwError : DWORD;
          szDeviceType : array[0..(RAS_MaxDeviceType + 1)-1] of TCHAR;
          szDeviceName : array[0..(RAS_MaxDeviceName + 1)-1] of TCHAR;
       end;
     _RASCONNSTATUS = RASCONNSTATUS;
     TRASCONNSTATUS = RASCONNSTATUS;
     PRASCONNSTATUS = ^RASCONNSTATUS;

     RASDIALEXTENSIONS = record
          dwSize : DWORD;
          dwfOptions : DWORD;
          hwndParent : HWND;
          reserved : DWORD;
       end;
     _RASDIALEXTENSIONS = RASDIALEXTENSIONS;
     TRASDIALEXTENSIONS = RASDIALEXTENSIONS;
     PRASDIALEXTENSIONS = ^RASDIALEXTENSIONS;

     RASDIALPARAMS = record
          dwSize : DWORD;
          szEntryName : array[0..(RAS_MaxEntryName + 1)-1] of TCHAR;
          szPhoneNumber : array[0..(RAS_MaxPhoneNumber + 1)-1] of TCHAR;
          szCallbackNumber : array[0..(RAS_MaxCallbackNumber + 1)-1] of TCHAR;
          szUserName : array[0..(UNLEN + 1)-1] of TCHAR;
          szPassword : array[0..(PWLEN + 1)-1] of TCHAR;
          szDomain : array[0..(DNLEN + 1)-1] of TCHAR;
       end;
     _RASDIALPARAMS = RASDIALPARAMS;
     TRASDIALPARAMS = RASDIALPARAMS;
     PRASDIALPARAMS = ^RASDIALPARAMS;

     RASENTRYNAME = record
          dwSize : DWORD;
          szEntryName : array[0..(RAS_MaxEntryName + 1)-1] of TCHAR;
       end;
     _RASENTRYNAME = RASENTRYNAME;
     TRASENTRYNAME = RASENTRYNAME;
     PRASENTRYNAME = ^RASENTRYNAME;

     RASPPPIP = record
          dwSize : DWORD;
          dwError : DWORD;
          szIpAddress : array[0..(RAS_MaxIpAddress + 1)-1] of TCHAR;
       end;
     _RASPPPIP = RASPPPIP;
     TRASPPPIP = RASPPPIP;
     PRASPPPIP = ^RASPPPIP;

     RASPPPIPX = record
          dwSize : DWORD;
          dwError : DWORD;
          szIpxAddress : array[0..(RAS_MaxIpxAddress + 1)-1] of TCHAR;
       end;
     _RASPPPIPX = RASPPPIPX;
     TRASPPPIPX = RASPPPIPX;
     PRASPPPIPX = ^RASPPPIPX;

     RASPPPNBF = record
          dwSize : DWORD;
          dwError : DWORD;
          dwNetBiosError : DWORD;
          szNetBiosError : array[0..(NETBIOS_NAME_LEN + 1)-1] of TCHAR;
          szWorkstationName : array[0..(NETBIOS_NAME_LEN + 1)-1] of TCHAR;
          bLana : BYTE;
       end;
     _RASPPPNBF = RASPPPNBF;
     TRASPPPNBF = RASPPPNBF;
     PRASPPPNBF = ^RASPPPNBF;

     RASTERIZER_STATUS = record
          nSize : integer;
          wFlags : integer;
          nLanguageID : integer;
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

     REMOTE_NAME_INFO = record
          lpUniversalName : LPTSTR;
          lpConnectionName : LPTSTR;
          lpRemainingPath : LPTSTR;
       end;
     _REMOTE_NAME_INFO = REMOTE_NAME_INFO;
     TREMOTENAMEINFO = REMOTE_NAME_INFO;
     PREMOTENAMEINFO = ^REMOTE_NAME_INFO;

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
          Buffer : array[0..0] of char;
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
          cbSize : DWORD;
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

     SET_PARTITION_INFORMATION = record
          PartitionType : BYTE;
       end;
     _SET_PARTITION_INFORMATION = SET_PARTITION_INFORMATION;
     TSETPARTITIONINFORMATION = SET_PARTITION_INFORMATION;
     PSETPARTITIONINFORMATION = ^SET_PARTITION_INFORMATION;

     SHELLEXECUTEINFO = record //+shellapi
          cbSize: DWORD;
          fMask: ULONG;
          hwnd: HWND;
          lpVerb: LPCTSTR;
          lpFile: LPCTSTR;
          lpParameters: LPCTSTR;
          lpDirectory: LPCTSTR;
          nShow: Integer ;
          hInstApp: HINST;
          // Optional fields
          lpIDList: LPVOID;
          lpClass: LPCTSTR;
          hkeyClass: HKEY;
          dwHotKey: DWORD;
          hIcon: HANDLE;
          hProcess :HANDLE;
        end;
     _SHELLEXECUTEINFO=SHELLEXECUTEINFO;   //+shellapi
     LPSHELLEXECUTEINFO=^SHELLEXECUTEINFO; //+shellapi

     SHCONTF = (SHCONTF_FOLDERS := 32,SHCONTF_NONFOLDERS := 64,
       SHCONTF_INCLUDEHIDDEN := 128);
     tagSHCONTF = SHCONTF;
     TSHCONTF = SHCONTF;

     SHFILEINFO = record
          hIcon : HICON;
          iIcon : longint;
          dwAttributes : DWORD;
          szDisplayName : array[0..MAX_PATH-1] of TCHAR;
          szTypeName : array[0..79] of TCHAR;
       end;
     _SHFILEINFO = SHFILEINFO;
     TSHFILEINFO = SHFILEINFO;
     PSHFILEINFO = ^SHFILEINFO;

     FILEOP_FLAGS = WORD;
     TFILEOPFLAGS = FILEOP_FLAGS;
     PFILEOPFLAGS = ^FILEOP_FLAGS;

     SHFILEOPSTRUCTW = record //+shellapi
          wnd : HWND;
          wFunc : UINT;
          pFrom : LPCWSTR;
          pTo : LPCWSTR;
          fFlags : FILEOP_FLAGS;
          fAnyOperationsAborted : WINBOOL;
          hNameMappings : LPVOID;
          lpszProgressTitle : LPCWSTR;
       end;
     LPSHFILEOPSTRUCTW = ^SHFILEOPSTRUCTW;
     _SHFILEOPSTRUCT = SHFILEOPSTRUCTW;
     TSHFILEOPSTRUCT = SHFILEOPSTRUCTW;
     PSHFILEOPSTRUCT = ^SHFILEOPSTRUCTW;
     TSHFILEOPSTRUCTW = SHFILEOPSTRUCTW;
     PSHFILEOPSTRUCTW = ^SHFILEOPSTRUCTW;

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
     LPSTARTUPINFO = ^STARTUPINFO;
     _STARTUPINFO = STARTUPINFO;
     TSTARTUPINFO = STARTUPINFO;
     PSTARTUPINFO = ^STARTUPINFO;

     STICKYKEYS = record
          cbSize : DWORD;
          dwFlags : DWORD;
       end;
     LPSTICKYKEYS = ^STICKYKEYS;
     tagSTICKYKEYS = STICKYKEYS;
     TSTICKYKEYS = STICKYKEYS;
     PSTICKYKEYS = ^STICKYKEYS;

     STRRET = record
          uType : UINT;
          DUMMYUNIONNAME : record
              case longint of
                 0 : ( pOleStr : LPWSTR );
                 1 : ( uOffset : UINT );
                 2 : ( cStr : array[0..(MAX_PATH)-1] of char );
              end;
       end;
     LPSTRRET = ^STRRET;
     _STRRET = STRRET;
     TSTRRET = STRRET;
     PSTRRET = ^STRRET;

     STYLEBUF = record
          dwStyle : DWORD;
          szDescription : array[0..31] of CHAR;
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
          u : record
              case longint of
                 0 : ( dwOemId : DWORD );
                 1 : ( s : record
                      wProcessorArchitecture : WORD;
                      wReserved : WORD;
                   end );
              end;
          dwPageSize : DWORD;
          lpMinimumApplicationAddress : LPVOID;
          lpMaximumApplicationAddress : LPVOID;
          dwActiveProcessorMask : DWORD;
          dwNumberOfProcessors : DWORD;
          dwProcessorType : DWORD;
          dwAllocationGranularity : DWORD;
          wProcessorLevel : WORD;
          wProcessorRevision : WORD;
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
          BlockSize : DWORD;
          PartitionCount : DWORD;
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
       end;
     _TAPE_SET_POSITION = TAPE_SET_POSITION;
     TTAPESETPOSITION = TAPE_SET_POSITION;
     PTAPESETPOSITION = ^TAPE_SET_POSITION;

     TAPE_WRITE_MARKS = record
          _Type : ULONG;
          Count : ULONG;
       end;
     _TAPE_WRITE_MARKS = TAPE_WRITE_MARKS;
     TTAPEWRITEMARKS = TAPE_WRITE_MARKS;
     PTAPEWRITEMARKS = ^TAPE_WRITE_MARKS;

     TBADDBITMAP = record
          hInst : HINST;
          nID : UINT;
       end;
     LPTBADDBITMAP = ^TBADDBITMAP;
     TTBADDBITMAP = TBADDBITMAP;
     PTBADDBITMAP = ^TBADDBITMAP;

     TBBUTTON = record
          iBitmap : longint;
          idCommand : longint;
          fsState : BYTE;
          fsStyle : BYTE;
          dwData : DWORD;
          iString : longint;
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
          SourceName : array[0..7] of CHAR;
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
       end;
     LPTOOLINFO = ^TOOLINFO;
     TTOOLINFO = TOOLINFO;
     PTOOLINFO = ^TOOLINFO;

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

     TV_INSERTSTRUCT = record
          hParent : HTREEITEM;
          hInsertAfter : HTREEITEM;
          item : TV_ITEM;
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

     VALENT = record
          ve_valuename : LPTSTR;
          ve_valuelen : DWORD;
          ve_valueptr : DWORD;
          ve_type : DWORD;
       end;
     TVALENT = VALENT;
     PVALENT = ^VALENT;

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


// Windows CE File API definitions copied from NT's winbase.h
     _WIN32_FIND_DATAA =record
       dwFileAttributes:DWORD;
       ftCreationTime:FILETIME;
       ftLastAccessTime:FILETIME;
       ftLastWriteTime:FILETIME;
       nFileSizeHigh:DWORD;
       nFileSizeLow:DWORD;
       dwReserved0:DWORD;
       dwReserved1:DWORD;
       cFileName:array[0..MAX_PATH-1] of char;
       cAlternateFileName:array[0..13] of char;
     end;
     WIN32_FIND_DATAA = _WIN32_FIND_DATAA;
     PWIN32_FIND_DATAA = ^_WIN32_FIND_DATAA;
     LPWIN32_FIND_DATAA = ^_WIN32_FIND_DATAA;


     WIN32_FIND_DATAW = record
          dwFileAttributes : DWORD;
          ftCreationTime : FILETIME;
          ftLastAccessTime : FILETIME;
          ftLastWriteTime : FILETIME;
          nFileSizeHigh : DWORD;
          nFileSizeLow : DWORD;
          dwOID: DWORD;
          cFileName : array[0..MAX_PATH-1] of WCHAR;
       end;

     LPWIN32_FIND_DATAW = ^WIN32_FIND_DATAW;
     PWIN32_FIND_DATAW = ^WIN32_FIND_DATAW;
     _WIN32_FIND_DATAW = WIN32_FIND_DATAW;
     TWIN32FINDDATAW = WIN32_FIND_DATAW;
     PWIN32FINDDATAW = ^WIN32_FIND_DATAW;

     WIN32_FIND_DATA = WIN32_FIND_DATAW;
     LPWIN32_FIND_DATA = ^WIN32_FIND_DATAW;
     PWIN32_FIND_DATA = ^WIN32_FIND_DATAW;
     _WIN32_FIND_DATA = WIN32_FIND_DATAW;
     TWIN32FINDDATA = WIN32_FIND_DATAW;
     PWIN32FINDDATA = ^WIN32_FIND_DATAW;

     WIN32_STREAM_ID = record
          dwStreamId : DWORD;
          dwStreamAttributes : DWORD;
          Size : LARGE_INTEGER;
          dwStreamNameSize : DWORD;
          cStreamName : ^WCHAR;
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

     WNDCLASS = record
          style : UINT;
          lpfnWndProc : WNDPROC;
          cbClsExtra : longint;
          cbWndExtra : longint;
          hInstance : HANDLE;
          hIcon : HICON;
          hCursor : HCURSOR;
          hbrBackground : HBRUSH;
          lpszMenuName : LPCTSTR;
          lpszClassName : LPCTSTR;
       end;
     LPWNDCLASS = ^WNDCLASS;
     _WNDCLASS = WNDCLASS;
     TWNDCLASSA = WNDCLASS;


     WNDCLASSW = record
          style : UINT;
          lpfnWndProc : WNDPROC;
          cbClsExtra : longint;
          cbWndExtra : longint;
          hInstance : HANDLE;
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
     TWNDCLASS = WNDCLASSW;
     PWNDCLASS = ^WNDCLASSW;

     WNDCLASSEX = record
          cbSize : UINT;
          style : UINT;
          lpfnWndProc : WNDPROC;
          cbClsExtra : longint;
          cbWndExtra : longint;
          hInstance : HANDLE;
          hIcon : HICON;
          hCursor : HCURSOR;
          hbrBackground : HBRUSH;
          lpszMenuName : LPCTSTR;
          lpszClassName : LPCTSTR;
          hIconSm : HANDLE;
       end;
     LPWNDCLASSEX = ^WNDCLASSEX;
     _WNDCLASSEX = WNDCLASSEX;
     TWNDCLASSEX = WNDCLASSEX;
     TWNDCLASSEXA = WNDCLASSEX;
     PWNDCLASSEX = ^WNDCLASSEX;

     WNDCLASSEXW = record
          cbSize : UINT;
          style : UINT;
          lpfnWndProc : WNDPROC;
          cbClsExtra : longint;
          cbWndExtra : longint;
          hInstance : HANDLE;
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

     CONNECTDLGSTRUCT = record
          cbStructure : DWORD;
          hwndOwner : HWND;
          lpConnRes : LPNETRESOURCE;
          dwFlags : DWORD;
          dwDevNum : DWORD;
       end;
     LPCONNECTDLGSTRUCT = ^CONNECTDLGSTRUCT;
     _CONNECTDLGSTRUCT = CONNECTDLGSTRUCT;
     TCONNECTDLGSTRUCT = CONNECTDLGSTRUCT;
     PCONNECTDLGSTRUCT = ^CONNECTDLGSTRUCT;

     CONNECTDLGSTRUCTW = record //+winnetwk
          cbStructure : DWORD;
          hwndOwner : HWND;
          lpConnRes : LPNETRESOURCEW;
          dwFlags : DWORD;
          dwDevNum : DWORD;
       end;
     LPCONNECTDLGSTRUCTW = ^CONNECTDLGSTRUCTW; //+winnetwk
     _CONNECTDLGSTRUCTW = CONNECTDLGSTRUCTW;  //+winnetwk

     DISCDLGSTRUCT = record
          cbStructure : DWORD;
          hwndOwner : HWND;
          lpLocalName : LPTSTR;
          lpRemoteName : LPTSTR;
          dwFlags : DWORD;
       end;
     LPDISCDLGSTRUCT = ^DISCDLGSTRUCT;
     _DISCDLGSTRUCT = DISCDLGSTRUCT;
     TDISCDLGSTRUCT = DISCDLGSTRUCT;
     TDISCDLGSTRUCTA = DISCDLGSTRUCT;
     PDISCDLGSTRUCT = ^DISCDLGSTRUCT;

     DISCDLGSTRUCTW = record  //+winnetwk
          cbStructure : DWORD;
          hwndOwner : HWND;
          lpLocalName : LPWSTR;
          lpRemoteName : LPWSTR;
          dwFlags : DWORD;
       end;
     LPDISCDLGSTRUCTW = ^DISCDLGSTRUCTW; //+winnetwk
     _DISCDLGSTRUCTW = DISCDLGSTRUCTW;   //+winnetwk

     NETINFOSTRUCT = record
          cbStructure : DWORD;
          dwProviderVersion : DWORD;
          dwStatus : DWORD;
          dwCharacteristics : DWORD;
          dwHandle : DWORD;
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

     ENUMMETAFILEPROC = function (_para1:HDC; _para2:HANDLETABLE; _para3:METARECORD; _para4:longint; _para5:LPARAM):longint;

     ENHMETAFILEPROC = function (_para1:HDC; _para2:HANDLETABLE; _para3:ENHMETARECORD; _para4:longint; _para5:LPARAM):longint;

     ENUMFONTSPROC = function (_para1:LPLOGFONT; _para2:LPTEXTMETRIC; _para3:DWORD; _para4:LPARAM):longint;

     FONTENUMPROC = function (var _para1:ENUMLOGFONT; var _para2:NEWTEXTMETRIC; _para3:longint; _para4:LPARAM):longint;

     FONTENUMEXPROC = function (var _para1:ENUMLOGFONTEX;var _para2:NEWTEXTMETRICEX; _para3:longint; _para4:LPARAM):longint;

     LPOVERLAPPED_COMPLETION_ROUTINE = procedure (_para1:DWORD; _para2:DWORD; _para3:LPOVERLAPPED);

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
          e_lfanew : LONG;
       end;
     PIMAGE_DOS_HEADER = ^IMAGE_DOS_HEADER;
     TIMAGEDOSHEADER = IMAGE_DOS_HEADER;
     PIMAGEDOSHEADER = ^IMAGE_DOS_HEADER;

      _NOTIFYICONDATAA = record
          cbSize: DWORD;
          Wnd: HWND;
          uID: UINT;
          uFlags: UINT;
          uCallbackMessage: UINT;
          hIcon: HICON;
          szTip: array [0..63] of Char;
     end;
     _NOTIFYICONDATA = _NOTIFYICONDATAA;

     _NOTIFYICONDATAW = record
         cbSize: DWORD;
         Wnd: HWND;
         uID: UINT;
         uFlags: UINT;
         uCallbackMessage: UINT;
         hIcon: HICON;
         szTip: array [0..63] of Word;
     end;
     TNotifyIconDataA = _NOTIFYICONDATAA;
     TNotifyIconDataW = _NOTIFYICONDATAW;
     TNotifyIconData = TNotifyIconDataA;
     NOTIFYICONDATAA = _NOTIFYICONDATAA;
     NOTIFYICONDATAW = _NOTIFYICONDATAW;
     NOTIFYICONDATA = NOTIFYICONDATAA;
     PNotifyIconDataA = ^TNotifyIconDataA;
     PNotifyIconDataW = ^TNotifyIconDataW;
     PNotifyIconData = PNotifyIconDataA;

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


type
     tagMOUSEINPUT = record
       dx:LONG;
       dy:LONG;
       mouseData:DWORD;
       dwFlags:DWORD;
       time:DWORD;
       dwExtraInfo:DWORD;
     end;
     MOUSEINPUT = tagMOUSEINPUT;
     PMOUSEINPUT = ^tagMOUSEINPUT;
     LPMOUSEINPUT = ^tagMOUSEINPUT;

     tagKEYBDINPUT = record
       wVk:word;
       wScan:word;
       dwFlags:DWORD;
       time:DWORD;
       dwExtraInfo:DWORD;
     end;
     KEYBDINPUT = tagKEYBDINPUT;
     PKEYBDINPUT = ^tagKEYBDINPUT;
     LPKEYBDINPUT = ^tagKEYBDINPUT;

     tagHARDWAREINPUT = record
       uMsg:DWORD;
       wParamL:word;
       wParamH:word;
       dwExtraInfo:DWORD;
     end;
     HARDWAREINPUT = tagHARDWAREINPUT;
     PHARDWAREINPUT = ^tagHARDWAREINPUT;
     LPHARDWAREINPUT = ^tagHARDWAREINPUT;

     tagINPUT = record
       _type:DWORD;
       case longint of
         0: (mi:MOUSEINPUT);
         1: (ki:KEYBDINPUT);
         2: (hi:HARDWAREINPUT);
     end;
    INPUT = tagINPUT;
    PINPUT = ^tagINPUT;
    LPINPUT = ^tagINPUT;


// For EnumDisplayMonitors
type
     MONITORENUMPROC = function(_hMonitor:HMONITOR; hdcMonitor:HDC; lprcMonitor:LPRECT; dwData:LPARAM):BOOL;

// For GetMonitorInfo
type
     tagMONITORINFO = record
       cbSize:DWORD;
       rcMonitor:RECT;
       rcWork:RECT;
       dwFlags:DWORD;
     end;
     MONITORINFO = tagMONITORINFO;
     LPMONITORINFO = ^tagMONITORINFO;

     tagMONITORINFOEX = record
       cbSize:DWORD;
       rcMonitor:RECT;
       rcWork:RECT;
       dwFlags:DWORD;
        szDevice:array[0..CCHDEVICENAME-1] of TCHAR;
     end;
     MONITORINFOEX = tagMONITORINFOEX;
     LPMONITORINFOEX = ^tagMONITORINFOEX;

// For EnumDisplayDevices
type
     _DISPLAY_DEVICE = record
       cb:DWORD;
       DeviceName:array[0..31] of TCHAR;
       DeviceString:array[0..127] of TCHAR;
       StateFlags:DWORD;
       DeviceID:array[0..127] of TCHAR;
       DeviceKey:array[0..127] of TCHAR;
     end;
     DISPLAY_DEVICE = _DISPLAY_DEVICE;
     PDISPLAY_DEVICE = ^_DISPLAY_DEVICE;
     LPDISPLAY_DEVICE = ^_DISPLAY_DEVICE;

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

 {$calling default}
 {$i typshrd.inc}
 {$calling cdecl}
{$endif read_implementation}

