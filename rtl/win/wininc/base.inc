{
    This file is part of the Free Pascal run time library.
    This unit contains base definition for the Win32 API
    Copyright (c) 1999-2000 by Florian Klaempfl,
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  Base.h

  Base definitions

  Copyright (C) 1996, 1997 Free Software Foundation, Inc.

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
}

{$ifdef read_interface}

{$PACKRECORDS C}

  type
     ATOM = word;
     TAtom = ATOM;

     WINBOOL = longbool;
     BOOL = WINBOOL;

     CALTYPE = cardinal;
     CALID = cardinal;

     CCHAR = char;

     COLORREF = cardinal;
     TCOLORREF = cardinal;

     SHORT = smallint;
     WINT  = longint;
     LONG  = longint;
     LONG64= int64;
     ULONG64 = qword;     // imagehlp header.
     ULONG32 = cardinal;
     DWORD = cardinal;

     PINTEGER = ^longint;
     PBOOL    = ^BOOL;

     LONGLONG  = int64;
     PLONGLONG = ^LONGLONG;
     LPLONGLONG = ^LONGLONG;
     ULONGLONG  = qword;      // used in AMD64 CONTEXT
     PULONGLONG = ^ULONGLONG; //
     DWORD64 = qword;         //
     PDWORD64 = ^DWORD64;     //


     INT_PTR = PtrInt;
     UINT_PTR = PtrUInt;
     LONG_PTR = PtrInt;
     ULONG_PTR = PtrUInt;
     DWORD_PTR   = ULONG_PTR;
     PDWORD_PTR  = ^DWORD_PTR;
     PULONG64    = ^ULONG64;
     PULONG32    = ^ULONG32;
     PULONG_PTR  = ^ULONG_PTR;
     PLONG64     = ^LONG64;

     DWORDLONG  = qword;  { was unsigned long  }
     PDWORDLONG = ^DWORDLONG;

     HANDLE = System.THandle;
     HRESULT = System.HResult;
     PHRESULT= ^HRESULT;

     HACCEL = HANDLE;
     HBITMAP = HANDLE;
     HBRUSH = HANDLE;
     HCOLORSPACE = HANDLE;
     HCONV = HANDLE;
     HCONVLIST = HANDLE;
     HCURSOR = HANDLE;
     HDBC = HANDLE;
     HDC = HANDLE;
     HDDEDATA = HANDLE;
     HDESK = HANDLE;
     HDEVNOTIFY = HANDLE;
     HDROP = HANDLE;
     HDWP = HANDLE;
     HENHMETAFILE = HANDLE;
     HENV = HANDLE;
     HFILE = HANDLE;
     HFONT = HANDLE;
     HGDIOBJ = HANDLE;
     HGLOBAL = HANDLE;
     HGLRC = HANDLE;
     HHOOK = HANDLE;
     HICON = HANDLE;
     HIMAGELIST = HANDLE;
     HINST = HANDLE;   { Not HINSTANCE, else it has problems with the var HInstance }
     HKEY = HANDLE;
     HKL = HANDLE;
     HLOCAL = HANDLE;
     HMENU = HANDLE;
     HMETAFILE = HANDLE;
     HMODULE = HANDLE;
     HMONITOR = HANDLE;
     HPALETTE = HANDLE;
     HPEN = HANDLE;
     HRASCONN = HANDLE;
     HRGN = HANDLE;
     HRSRC = HANDLE;
     HSTMT = HANDLE;
     HSTR  = HANDLE;
     HSZ = HANDLE;
     HWINSTA = HANDLE;
     HWND = HANDLE;
     HTASK = HANDLE;

     LANGID = word;
     LCID   = DWORD;
     LCTYPE = DWORD;
     LPARAM = LONG_PTR;

     LP     = ^word;
     LPBOOL = ^WINBOOL;
     LPBYTE = pbyte;
     LPCCH  = PCHAR;
     LPCH   = PCHAR;

     LPCOLORREF = ^COLORREF;

     LPCSTR  = Pchar;
{$ifdef UNICODE}
     LPCTSTR = Pwidechar;
{$else}
     LPCTSTR = Pchar;
{$endif}

     LPCWCH  = Pwidechar;
     LPCWSTR = Pwidechar;

     LPPCSTR = ^LPCSTR;
     LPPCTSTR = ^LPCTSTR;
     LPPCWSTR = ^LPCWSTR;

     LPDWORD = ^DWORD;

     LPHANDLE = ^HANDLE;

     LPINT  = ^longint;
     LPLONG = ^longint;

     LPSTR = Pchar;
{$ifdef UNICODE}
     LPTCH  = Pwidechar;
     LPTSTR = Pwidechar;
{$else}
     LPTCH  = Pchar;
     LPTSTR = Pchar;
{$endif}

     LRESULT = LONG_PTR;
     PLRESULT= ^LRESULT;

     LPVOID  = pointer;
     LPCVOID = pointer;

     LPWCH  = Pwidechar;
     LPWORD = ^word;
     LPWSTR = Pwidechar;
     NWPSTR = Pwidechar;

     LPPSTR = ^LPSTR;
     LPPWSTR = ^LPWSTR;
     LPPTSTR = ^LPTSTR;

     PWINBOOL = ^WINBOOL;
     PBOOLEAN = ^BYTE {$ifndef VER2_2} deprecated {$endif};

     PBYTE = System.PByte;

     PCCH = PCHAR;
     PCH  = PCHAR;

     PCSTR = Pchar;

     PCWCH  = Pwidechar;
     PCWSTR = Pwidechar;

     PDWORD = System.PDWORD;

     PHANDLE = ^HANDLE;
     PHKEY = ^HKEY;

     PINT = ^longint;
     PLONG = ^longint;
     PSHORT = ^SHORT;

     PSTR = Pchar;

     PSZ = Pchar;
{$ifdef UNICODE}
     PTBYTE = ^word;
     PTCH = Pwidechar;
     PTCHAR = Pwidechar;
     PTSTR = Pwidechar;
{$else}
     PTBYTE = ^byte;
     PTCH   = Pchar;
     PTCHAR = Pchar;
     PTSTR  = Pchar;
{$endif}

     PUCHAR = ^byte;
     PWCH   = Pwidechar;
     PWCHAR = Pwidechar;

     PWORD   = System.PWord;
     PUINT   = ^cardinal;
     PULONG  = ^cardinal;
     PUSHORT = ^word;

     PVOID = pointer;

     RETCODE = SHORT;

     SC_HANDLE = HANDLE;
     SC_LOCK = LPVOID;
     LPSC_HANDLE = ^SC_HANDLE;

     SERVICE_STATUS_HANDLE = DWORD;


{$ifdef UNICODE}
     TBYTE = word;
     TCHAR = widechar;
     BCHAR = word;
{$else}
     TBYTE = byte;
     TCHAR = ansichar;
     BCHAR = BYTE;
{$endif}

     UCHAR = byte;
     WCHAR = WideChar;

     UINT   = cardinal;
     ULONG  = cardinal;
     USHORT = word;

     WPARAM = LONG_PTR;
     PLPSTR = ^LPSTR;
     PLPWStr= ^LPWStr;
     COLOR16 = ushort;
{
  Enumerations
}

     ACL_INFORMATION_CLASS = (AclRevisionInformation = 1,AclSizeInformation
       );

     _ACL_INFORMATION_CLASS = ACL_INFORMATION_CLASS;

     MEDIA_TYPE = (Unknown,F5_1Pt2_512,F3_1Pt44_512,F3_2Pt88_512,
       F3_20Pt8_512,F3_720_512,F5_360_512,F5_320_512,
       F5_320_1024,F5_180_512,F5_160_512,RemovableMedia,
       FixedMedia);

     _MEDIA_TYPE = MEDIA_TYPE;

  const
     RASCS_DONE = $2000;
     RASCS_PAUSED = $1000;

  type

     RASCONNSTATE = (RASCS_OpenPort = 0,RASCS_PortOpened,
       RASCS_ConnectDevice,RASCS_DeviceConnected,
       RASCS_AllDevicesConnected,RASCS_Authenticate,
       RASCS_AuthNotify,RASCS_AuthRetry,RASCS_AuthCallback,
       RASCS_AuthChangePassword,RASCS_AuthProject,
       RASCS_AuthLinkSpeed,RASCS_AuthAck,RASCS_ReAuthenticate,
       RASCS_Authenticated,RASCS_PrepareForCallback,
       RASCS_WaitForModemReset,RASCS_WaitForCallback,
       RASCS_Projected,RASCS_StartAuthentication,
       RASCS_CallbackComplete,RASCS_LogonNetwork,
       RASCS_Interactive = RASCS_PAUSED,RASCS_RetryAuthentication,
       RASCS_CallbackSetByCaller,RASCS_PasswordExpired,
       RASCS_Connected = RASCS_DONE,RASCS_Disconnected
       );

     _RASCONNSTATE = RASCONNSTATE;

     RASPROJECTION = (RASP_PppIp = $8021, RASP_PppIpx = $802B, RASP_PppNbf = $803F,
       RASP_Amb = $10000);

     _RASPROJECTION = RASPROJECTION;

     SECURITY_IMPERSONATION_LEVEL = (SecurityAnonymous,SecurityIdentification,
       SecurityImpersonation,SecurityDelegation
       );

     _SECURITY_IMPERSONATION_LEVEL = SECURITY_IMPERSONATION_LEVEL;

     SID_NAME_USE = (SidTypeUser = 1,SidTypeGroup,SidTypeDomain,
       SidTypeAlias,SidTypeWellKnownGroup,SidTypeDeletedAccount,
       SidTypeInvalid,SidTypeUnknown);

     PSID_NAME_USE = ^SID_NAME_USE;

     _SID_NAME_USE = SID_NAME_USE;

     TOKEN_TYPE = (
       TokenPrimary = 1,
       TokenImpersonation
     );
     _TOKEN_TYPE = TOKEN_TYPE;
     TTokenType = TOKEN_TYPE;

     TOKEN_ELEVATION_TYPE = (
       TokenElevationTypeDefault = 1,
       TokenElevationTypeFull,
       TokenElevationTypeLimited
     );
     _TOKEN_ELEVATION_TYPE = TOKEN_ELEVATION_TYPE;
     TTokenElevationType = TOKEN_ELEVATION_TYPE;

     TOKEN_INFORMATION_CLASS = (
       TokenUser = 1,
       TokenGroups,
       TokenPrivileges,
       TokenOwner,
       TokenPrimaryGroup,
       TokenDefaultDacl,
       TokenSource,
       TokenType,
       TokenImpersonationLevel,
       TokenStatistics,
       TokenRestrictedSids,
       TokenSessionId,
       TokenGroupsAndPrivileges,
       TokenSessionReference,
       TokenSandBoxInert,
       TokenAuditPolicy,
       TokenOrigin,
       TokenElevationType,
       TokenLinkedToken,
       TokenElevation,
       TokenHasRestrictions,
       TokenAccessInformation,
       TokenVirtualizationAllowed,
       TokenVirtualizationEnabled,
       TokenIntegrityLevel,
       TokenUIAccess,
       TokenMandatoryPolicy,
       TokenLogonSid,
       MaxTokenInfoClass  // MaxTokenInfoClass should always be the last enum
     );
     _TOKEN_INFORMATION_CLASS = TOKEN_INFORMATION_CLASS;
     TTokenInformationClass   = TOKEN_INFORMATION_CLASS;

     _TOKEN_ELEVATION = record
       TokenIsElevated: DWORD;
     end;
     TOKEN_ELEVATION = _TOKEN_ELEVATION;
     TTokenElevation = _TOKEN_ELEVATION;
     PTokenElevation = ^TTokenElevation;

    _FINDEX_INFO_LEVELS = (FindExInfoStandard,FindExInfoBasic,FindExInfoMaxInfoLevel);
    _FINDEX_SEARCH_OPS  = (FindExSearchNameMatch, FindExSearchLimitToDirectories,
 			   FindExSearchLimitToDevices, FindExSearchMaxSearchOp);
    TFINDEX_INFO_LEVELS = _FINDEX_INFO_LEVELS;
    FINDEX_INFO_LEVELS  = _FINDEX_INFO_LEVELS;
    PFINDEX_INFO_LEVELS = ^TFINDEX_INFO_LEVELS;
    TFINDEX_SEARCH_OPS  = _FINDEX_SEARCH_OPS;
    FINDEX_SEARCH_OPS   = _FINDEX_SEARCH_OPS;
    PFINDEX_SEARCH_OPS  = ^TFINDEX_SEARCH_OPS;

    PARTITION_STYLE     = (PARTITION_STYLE_MBR=0,PARTITION_STYLE_GPT,PARTITION_STYLE_RAW);
    TPARTITION_STYLE    = PARTITION_STYLE;
    PPARTITION_STYLE    = ^TPARTITION_STYLE;

    _FINDEX_INFO_LEVELS = (FindExInfoStandard,FindExInfoBasic,FindExInfoMaxInfoLevel);
    _FINDEX_SEARCH_OPS  = (FindExSearchNameMatch, FindExSearchLimitToDirectories,
 			   FindExSearchLimitToDevices, FindExSearchMaxSearchOp);
    TFINDEX_INFO_LEVELS = _FINDEX_INFO_LEVELS;
    FINDEX_INFO_LEVELS  = _FINDEX_INFO_LEVELS;
    PFINDEX_INFO_LEVELS = ^TFINDEX_INFO_LEVELS;
    TFINDEX_SEARCH_OPS  = _FINDEX_SEARCH_OPS;
    FINDEX_SEARCH_OPS   = _FINDEX_SEARCH_OPS;
    PFINDEX_SEARCH_OPS  = ^TFINDEX_SEARCH_OPS;

    PARTITION_STYLE     = (PARTITION_STYLE_MBR:=0,PARTITION_STYLE_GPT,PARTITION_STYLE_RAW);
    TPARTITION_STYLE    = PARTITION_STYLE;
    PPARTITION_STYLE    = ^TPARTITION_STYLE;

    _FINDEX_INFO_LEVELS = (FindExInfoStandard,FindExInfoBasic,FindExInfoMaxInfoLevel);
    _FINDEX_SEARCH_OPS  = (FindExSearchNameMatch, FindExSearchLimitToDirectories,
 			   FindExSearchLimitToDevices, FindExSearchMaxSearchOp);
    TFINDEX_INFO_LEVELS = _FINDEX_INFO_LEVELS;
    FINDEX_INFO_LEVELS  = _FINDEX_INFO_LEVELS;
    PFINDEX_INFO_LEVELS = ^TFINDEX_INFO_LEVELS;
    TFINDEX_SEARCH_OPS  = _FINDEX_SEARCH_OPS;
    FINDEX_SEARCH_OPS   = _FINDEX_SEARCH_OPS;
    PFINDEX_SEARCH_OPS  = ^TFINDEX_SEARCH_OPS;

    PARTITION_STYLE     = (PARTITION_STYLE_MBR:=0,PARTITION_STYLE_GPT,PARTITION_STYLE_RAW);
    TPARTITION_STYLE    = PARTITION_STYLE;
    PPARTITION_STYLE    = ^TPARTITION_STYLE;

    _FINDEX_INFO_LEVELS = (FindExInfoStandard,FindExInfoBasic,FindExInfoMaxInfoLevel);
    _FINDEX_SEARCH_OPS  = (FindExSearchNameMatch, FindExSearchLimitToDirectories,
 			   FindExSearchLimitToDevices, FindExSearchMaxSearchOp);
    TFINDEX_INFO_LEVELS = _FINDEX_INFO_LEVELS;
    FINDEX_INFO_LEVELS  = _FINDEX_INFO_LEVELS;
    PFINDEX_INFO_LEVELS = ^TFINDEX_INFO_LEVELS;
    TFINDEX_SEARCH_OPS  = _FINDEX_SEARCH_OPS;
    FINDEX_SEARCH_OPS   = _FINDEX_SEARCH_OPS;
    PFINDEX_SEARCH_OPS  = ^TFINDEX_SEARCH_OPS;

    PARTITION_STYLE     = (PARTITION_STYLE_MBR:=0,PARTITION_STYLE_GPT,PARTITION_STYLE_RAW);
    TPARTITION_STYLE    = PARTITION_STYLE;
    PPARTITION_STYLE    = ^TPARTITION_STYLE;

    _FINDEX_INFO_LEVELS = (FindExInfoStandard,FindExInfoBasic,FindExInfoMaxInfoLevel);
    _FINDEX_SEARCH_OPS  = (FindExSearchNameMatch, FindExSearchLimitToDirectories,
 			   FindExSearchLimitToDevices, FindExSearchMaxSearchOp);
    TFINDEX_INFO_LEVELS = _FINDEX_INFO_LEVELS;
    FINDEX_INFO_LEVELS  = _FINDEX_INFO_LEVELS;
    PFINDEX_INFO_LEVELS = ^TFINDEX_INFO_LEVELS;
    TFINDEX_SEARCH_OPS  = _FINDEX_SEARCH_OPS;
    FINDEX_SEARCH_OPS   = _FINDEX_SEARCH_OPS;
    PFINDEX_SEARCH_OPS  = ^TFINDEX_SEARCH_OPS;

    PARTITION_STYLE     = (PARTITION_STYLE_MBR:=0,PARTITION_STYLE_GPT,PARTITION_STYLE_RAW);
    TPARTITION_STYLE    = PARTITION_STYLE;
    PPARTITION_STYLE    = ^TPARTITION_STYLE;

 {
   Macros
 }

  MakeIntResourceA = PAnsiChar;
  MakeIntResourceW = PWideChar;
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef UNICODE}
    MakeIntResource = MakeIntResourceW;
  {$else}
    MakeIntResource = MakeIntResourceA;
  {$endif}
=======
  MakeIntResource = MakeIntResourceA;
>>>>>>> graemeg/fixes_2_2
=======
  MakeIntResource = MakeIntResourceA;
>>>>>>> origin/fixes_2_2

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GetBValue(rgb : longint) : BYTE; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GetGValue(rgb : longint) : BYTE; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GetRValue(rgb : longint) : BYTE; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function RGB(r,g,b : longint) : DWORD; inline;

  {  Not convertable by H2PAS
  #define HANDLE_WM_NOTIFY(hwnd, wParam, lParam, fn) \
      (fn)((hwnd), (int)(wParam), (NMHDR FAR )(lParam))
   }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function HIBYTE(w : longint) : BYTE; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function HIWORD(l : longint) : WORD; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LOBYTE(w : longint) : BYTE; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LOWORD(l : longint) : WORD; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELONG(a,b : longint) : LONG; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKEWORD(a,b : longint) : WORD; inline;

  { original Cygnus headers also had the following defined:  }
  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function SEXT_HIWORD(l : longint) : longint; inline;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function ZEXT_HIWORD(l : longint) : longint; inline;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function SEXT_LOWORD(l : longint) : longint; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function INDEXTOOVERLAYMASK(i : longint) : longint; inline;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function INDEXTOSTATEIMAGEMASK(i : longint) : longint; inline;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
<<<<<<< HEAD
  function MAKEINTATOM(i : longint) : LPTSTR; inline;
=======
  function MAKEINTATOM(i : longint) : LPTSTR;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function MAKELANGID(p,s : longint) : longint; inline;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function PRIMARYLANGID(lgid : longint) : WORD; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function SUBLANGID(lgid : longint) : longint; inline;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LANGIDFROMLCID(lcid : longint) : WORD; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function SORTIDFROMLCID(lcid : longint) : WORD; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELCID(lgid,srtid : longint) : DWORD; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELPARAM(l,h : longint) : LPARAM; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELRESULT(l,h : longint) : LRESULT; inline;

  {  Not convertable by H2PAS
  #define MAKEPOINTS(l)   ( ((POINTS FAR  ) & (l)))
   }
  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKEROP4(fore,back : longint) : DWORD; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKEWPARAM(l,h : longint) : WPARAM; inline;

  function GET_X_LPARAM(lp : Windows.LParam) : longint; inline;

  function GET_Y_LPARAM(lp : Windows.LParam) : longint; inline;

  {$ifndef max}
  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function max(a,b : longint) : longint;  inline;
    { return type might be wrong }

{$endif}
{$ifndef min}
  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function min(a,b : longint) : longint; inline;
    { return type might be wrong }

{$endif}
  { was #define dname(params) def_expr }
  { argument types are unknown }
  function PALETTEINDEX(i : longint) : COLORREF; inline;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function PALETTERGB(r,g,b : longint) : longint; inline;
    { return type might be wrong }

  (*  Not convertable by H2PAS
  #define POINTSTOPOINT(pt, pts) {(pt).x = (SHORT) LOWORD(pts); \
        (pt).y = (SHORT) HIWORD(pts);}
  #define POINTTOPOINTS(pt) \
      (MAKELONG((short) ((pt).x), (short) ((pt).y)))
   *)
  { already declared before
  #define INDEXTOOVERLAYMASK(i) ((i) << 8)
  #define INDEXTOSTATEIMAGEMASK(i) ((i) << 12)
   }
  {  Not convertable by H2PAS
  #ifdef UNICODE
  #define TEXT(quote) L##quote
  #else
  #define TEXT(quote) quote
  #endif
   }


  function IMAGE_ORDINAL64(Ordinal : uint64) : uint64; inline;
  function IMAGE_ORDINAL32(Ordinal : cardinal) : cardinal;inline;
  function IMAGE_SNAP_BY_ORDINAL64(Ordinal : uint64) : boolean;  inline;
  function IMAGE_SNAP_BY_ORDINAL32(Ordinal : cardinal) : boolean; inline;
  {$ifdef WIN64}
    function IMAGE_ORDINAL(Ordinal : uint64) : uint64; inline;
    function IMAGE_SNAP_BY_ORDINAL(Ordinal : uint64) : boolean; inline;
  {$else}
    function IMAGE_ORDINAL(Ordinal : cardinal) : cardinal; inline;
    function IMAGE_SNAP_BY_ORDINAL(Ordinal : cardinal) : boolean; inline;
  {$endif}
 


 {
    Definitions for callback procedures
 }

 type

     BFFCALLBACK = function (_para1:HWND; _para2:UINT; _para3:LPARAM; _para4:LPARAM):longint;stdcall;

     LPCCHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT_PTR;stdcall;

     LPCFHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT_PTR;stdcall;

     PTHREAD_START_ROUTINE = Pointer;

     LPTHREAD_START_ROUTINE = PTHREAD_START_ROUTINE;

     EDITSTREAMCALLBACK = function (_para1:DWORD_PTR; _para2:LPBYTE; _para3:LONG; _para4:PLONG):DWORD;stdcall;

     LPFRHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT_PTR;stdcall;

     LPOFNHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT_PTR;stdcall;

     LPPRINTHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT_PTR;stdcall;

     LPSETUPHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT_PTR;stdcall;

     DLGPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):LRESULT;stdcall;

     PFNPROPSHEETCALLBACK = function (_para1:HWND; _para2:UINT; _para3:LPARAM):longint;stdcall;

     LPSERVICE_MAIN_FUNCTION = procedure (_para1:DWORD; _para2:LPPTSTR);stdcall;

     PFNTVCOMPARE = function (_para1:LPARAM; _para2:LPARAM; _para3:LPARAM):longint;stdcall;

     WNDPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):LRESULT;stdcall;

     FARPROC = pointer;

     TFarProc = FARPROC;

     TFNWndProc = TFarProc;

     PROC = FARPROC;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     ENUMRESTYPEPROC = function (hModule:HMODULE; lpszType:LPTSTR; lParam:LONG_PTR):WINBOOL;stdcall;

     ENUMRESNAMEPROC = function (hModule:HMODULE; lpszType:LPCTSTR; lpszName:LPTSTR; lParam:LONG_PTR):WINBOOL;stdcall;

     ENUMRESLANGPROC = function (hModule:HMODULE; lpszType:LPCTSTR; lpszName:LPCTSTR; wIDLanguage:WORD; lParam:LONG_PTR):WINBOOL;stdcall;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     ENUMRESTYPEPROC = function (hModule:HANDLE; lpszType:LPTSTR; lParam:LONG_PTR):WINBOOL;stdcall;

     ENUMRESNAMEPROC = function (hModule:HANDLE; lpszType:LPCTSTR; lpszName:LPTSTR; lParam:LONG_PTR):WINBOOL;stdcall;

     ENUMRESLANGPROC = function (hModule:HANDLE; lpszType:LPCTSTR; lpszName:LPCTSTR; wIDLanguage:WORD; lParam:LONG_PTR):WINBOOL;stdcall;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
     ENUMRESTYPEPROC = function (hModule:HANDLE; lpszType:LPTSTR; lParam:LONG_PTR):WINBOOL;stdcall;

     ENUMRESNAMEPROC = function (hModule:HANDLE; lpszType:LPCTSTR; lpszName:LPTSTR; lParam:LONG_PTR):WINBOOL;stdcall;

     ENUMRESLANGPROC = function (hModule:HANDLE; lpszType:LPCTSTR; lpszName:LPCTSTR; wIDLanguage:WORD; lParam:LONG_PTR):WINBOOL;stdcall;
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

     DESKTOPENUMPROC = FARPROC;

     ENUMWINDOWSPROC = function (_para1:HWND; _para2:LPARAM):WINBOOL;stdcall;

     ENUMWINDOWSTATIONPROC = function (_para1:LPTSTR; _para2:LPARAM):WINBOOL;stdcall;

     SENDASYNCPROC = procedure (_para1:HWND; _para2:UINT; _para3:ULONG_PTR; _para4:LRESULT);stdcall;

     TIMERPROC = procedure (hWnd: HWND; uMsg: UINT; idEvent: UINT_PTR; dwTime: DWORD); stdcall;

     GRAYSTRINGPROC = FARPROC;

     DRAWSTATEPROC = function (_para1:HDC; _para2:LPARAM; _para3:WPARAM; _para4:longint; _para5:longint):WINBOOL;stdcall;

     PROPENUMPROCEXA = function (_para1:HWND; _para2:LPCSTR;  _para3:HANDLE; _para4:ULONG_PTR):WINBOOL;stdcall;
     PROPENUMPROCEXW = function (_para1:HWND; _para2:LPCWSTR; _para3:HANDLE; _para4:ULONG_PTR):WINBOOL;stdcall;
     PROPENUMPROCEX  = function (_para1:HWND; _para2:LPCTSTR; _para3:HANDLE; _para4:ULONG_PTR):WINBOOL;stdcall;

     PROPENUMPROCA = function (_para1:HWND; _para2:LPCSTR;  _para3:HANDLE):WINBOOL;stdcall;
     PROPENUMPROCW = function (_para1:HWND; _para2:LPCWSTR; _para3:HANDLE):WINBOOL;stdcall;
     PROPENUMPROC  = function (_para1:HWND; _para2:LPCTSTR; _para3:HANDLE):WINBOOL;stdcall;

     HOOKPROC = function (_para1:longint; _para2:WPARAM; _para3:LPARAM):LRESULT;stdcall;

     ENUMOBJECTSPROC = procedure (_para1:LPVOID; _para2:LPARAM);stdcall;

     LINEDDAPROC = procedure (_para1:longint; _para2:longint; _para3:LPARAM);stdcall;

     TABORTPROC = function (_para1:HDC; _para2:longint):WINBOOL;stdcall;

     LPPAGEPAINTHOOK = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT_PTR;stdcall;

     LPPAGESETUPHOOK = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT_PTR;stdcall;

     ICMENUMPROCA = function (_para1:LPSTR;  _para2:LPARAM):longint;stdcall;
     ICMENUMPROCW = function (_para1:LPWSTR; _para2:LPARAM):longint;stdcall;
     ICMENUMPROC = function  (_para1:LPTSTR; _para2:LPARAM):longint;stdcall;

     EDITWORDBREAKPROCEX = function (_para1:pchar; _para2:LONG; _para3:BYTE; _para4:WINT):LONG;stdcall;

     PFNLVCOMPARE = function (_para1:LPARAM; _para2:LPARAM; _para3:LPARAM):longint;stdcall;

     LOCALE_ENUMPROCA = function (_para1:LPSTR):WINBOOL;stdcall;
     LOCALE_ENUMPROCW = function (_para1:LPWSTR):WINBOOL;stdcall;
     LOCALE_ENUMPROC  = function (_para1:LPTSTR):WINBOOL;stdcall;

     CODEPAGE_ENUMPROCA = function (_para1:LPSTR):WINBOOL;stdcall;
     CODEPAGE_ENUMPROCW = function (_para1:LPWSTR):WINBOOL;stdcall;
     CODEPAGE_ENUMPROC  = function (_para1:LPTSTR):WINBOOL;stdcall;

     DATEFMT_ENUMPROCA = function (_para1:LPSTR):WINBOOL;stdcall;
     DATEFMT_ENUMPROCW = function (_para1:LPWSTR):WINBOOL;stdcall;
     DATEFMT_ENUMPROC  = function (_para1:LPTSTR):WINBOOL;stdcall;

     TIMEFMT_ENUMPROCA = function (_para1:LPSTR):WINBOOL;stdcall;
     TIMEFMT_ENUMPROCW = function (_para1:LPWSTR):WINBOOL;stdcall;
     TIMEFMT_ENUMPROC  = function (_para1:LPTSTR):WINBOOL;stdcall;

     CALINFO_ENUMPROCA= function (_para1:LPSTR):WINBOOL;stdcall;
     CALINFO_ENUMPROCW= function (_para1:LPWSTR):WINBOOL;stdcall;
     CALINFO_ENUMPROC = function (_para1:LPTSTR):WINBOOL;stdcall;

     PHANDLER_ROUTINE = function (_para1:DWORD):WINBOOL;stdcall;

     LPHANDLER_FUNCTION = function (_para1:DWORD):WINBOOL;stdcall;

     PFNGETPROFILEPATHA = function (_para1:LPCSTR; _para2:LPSTR; _para3:UINT):UINT;stdcall;
     PFNGETPROFILEPATHW = function (_para1:LPCWSTR; _para2:LPSTR; _para3:UINT):UINT;stdcall;
     PFNGETPROFILEPATH  = function (_para1:LPCTSTR; _para2:LPSTR; _para3:UINT):UINT;stdcall;

     PFNRECONCILEPROFILEA = function (_para1:LPCSTR; _para2:LPCSTR; _para3:DWORD):UINT;stdcall;
     PFNRECONCILEPROFILEW = function (_para1:LPCWSTR; _para2:LPCWSTR; _para3:DWORD):UINT;stdcall;
     PFNRECONCILEPROFILE  = function (_para1:LPCTSTR; _para2:LPCTSTR; _para3:DWORD):UINT;stdcall;

     PFNPROCESSPOLICIESA = function (_para1:HWND; _para2:LPCSTR; _para3:LPCSTR; _para4:LPCSTR; _para5:DWORD):WINBOOL;stdcall;
     PFNPROCESSPOLICIESW = function (_para1:HWND; _para2:LPCWSTR; _para3:LPCWSTR; _para4:LPCWSTR; _para5:DWORD):WINBOOL;stdcall;
     PFNPROCESSPOLICIES  = function (_para1:HWND; _para2:LPCTSTR; _para3:LPCTSTR; _para4:LPCTSTR; _para5:DWORD):WINBOOL;stdcall;
  (*  Not convertable by H2PAS
  #define SECURITY_NULL_SID_AUTHORITY     {0,0,0,0,0,0}
  #define SECURITY_WORLD_SID_AUTHORITY    {0,0,0,0,0,1}
  #define SECURITY_LOCAL_SID_AUTHORITY    {0,0,0,0,0,2}
  #define SECURITY_CREATOR_SID_AUTHORITY  {0,0,0,0,0,3}
  #define SECURITY_NON_UNIQUE_AUTHORITY   {0,0,0,0,0,4}
  #define SECURITY_NT_AUTHORITY           {0,0,0,0,0,5}
   *)
  { TEXT("String") replaced by "String" below for H2PAS  }

  const
     SE_CREATE_TOKEN_NAME = 'SeCreateTokenPrivilege';
     SE_ASSIGNPRIMARYTOKEN_NAME = 'SeAssignPrimaryTokenPrivilege';
     SE_LOCK_MEMORY_NAME = 'SeLockMemoryPrivilege';
     SE_INCREASE_QUOTA_NAME = 'SeIncreaseQuotaPrivilege';
     SE_UNSOLICITED_INPUT_NAME = 'SeUnsolicitedInputPrivilege';
     SE_MACHINE_ACCOUNT_NAME = 'SeMachineAccountPrivilege';
     SE_TCB_NAME = 'SeTcbPrivilege';
     SE_SECURITY_NAME = 'SeSecurityPrivilege';
     SE_TAKE_OWNERSHIP_NAME = 'SeTakeOwnershipPrivilege';
     SE_LOAD_DRIVER_NAME = 'SeLoadDriverPrivilege';
     SE_SYSTEM_PROFILE_NAME = 'SeSystemProfilePrivilege';
     SE_SYSTEMTIME_NAME = 'SeSystemtimePrivilege';
     SE_PROF_SINGLE_PROCESS_NAME = 'SeProfileSingleProcessPrivilege';
     SE_INC_BASE_PRIORITY_NAME = 'SeIncreaseBasePriorityPrivilege';
     SE_CREATE_PAGEFILE_NAME = 'SeCreatePagefilePrivilege';
     SE_CREATE_PERMANENT_NAME = 'SeCreatePermanentPrivilege';
     SE_BACKUP_NAME = 'SeBackupPrivilege';
     SE_RESTORE_NAME = 'SeRestorePrivilege';
     SE_SHUTDOWN_NAME = 'SeShutdownPrivilege';
     SE_DEBUG_NAME = 'SeDebugPrivilege';
     SE_AUDIT_NAME = 'SeAuditPrivilege';
     SE_SYSTEM_ENVIRONMENT_NAME = 'SeSystemEnvironmentPrivilege';
     SE_CHANGE_NOTIFY_NAME = 'SeChangeNotifyPrivilege';
     SE_REMOTE_SHUTDOWN_NAME = 'SeRemoteShutdownPrivilege';
  {  Not convertable by H2PAS
  #define SERVICES_ACTIVE_DATABASEW      L"ServicesActive"
  #define SERVICES_FAILED_DATABASEW      L"ServicesFailed"
   }
     SERVICES_ACTIVE_DATABASEA = 'ServicesActive';
     SERVICES_FAILED_DATABASEA = 'ServicesFailed';
     SERVICES_ACTIVE_DATABASEW = 'ServicesActive'#0000;
     SERVICES_FAILED_DATABASEW = 'ServicesFailed'#0000;

  {  Not convertable by H2PAS
  #define SC_GROUP_IDENTIFIERW           L'+'
   }
     SC_GROUP_IDENTIFIERA = '+';
     SC_GROUP_IDENTIFIERW = '+'#0000;
{$ifdef UNICODE}
     SERVICES_ACTIVE_DATABASE = SERVICES_ACTIVE_DATABASEW;
     SERVICES_FAILED_DATABASE = SERVICES_FAILED_DATABASEW;
     SC_GROUP_IDENTIFIER = SC_GROUP_IDENTIFIERW;
{$else}
     SERVICES_ACTIVE_DATABASE = SERVICES_ACTIVE_DATABASEA;
     SERVICES_FAILED_DATABASE = SERVICES_FAILED_DATABASEA;
     SC_GROUP_IDENTIFIER = SC_GROUP_IDENTIFIERA;
{$endif}

type

     { PFNCALLBACK = CALLB; }
     PFNCALLBACK = function(_para1, _para2: UINT;_para3: HCONV;_para4, _para5: HSZ; _para6: HDDEDATA;_para7 ,_para8 :DWORD): HDDEData;stdcall;
     { CALLB = procedure ;CDECL; }
       CALLB = PFNCALLBACK;


     SECURITY_CONTEXT_TRACKING_MODE = WINBOOL;
  { End of stuff from ddeml.h in old Cygnus headers  }
  { -----------------------------------------------  }

     WNDENUMPROC = FARPROC;

     ENHMFENUMPROC = FARPROC;

     CCSTYLE = DWORD;

     PCCSTYLE = ^CCSTYLE;

     LPCCSTYLE = ^CCSTYLE;

     CCSTYLEFLAGA = DWORD;

     PCCSTYLEFLAGA = ^CCSTYLEFLAGA;

     LPCCSTYLEFLAGA = ^CCSTYLEFLAGA;

{$endif read_interface}


{$ifdef read_implementation}

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GetBValue(rgb : longint) : BYTE;
    begin
       GetBValue:=BYTE(rgb shr 16);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GetGValue(rgb : longint) : BYTE;
    begin
       GetGValue:=BYTE((WORD(rgb)) shr 8);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GetRValue(rgb : longint) : BYTE;
    begin
       GetRValue:=BYTE(rgb);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function RGB(r,g,b : longint) : DWORD;
    begin
       RGB:=DWORD(((DWORD(BYTE(r))) or ((DWORD(WORD(g))) shl 8)) or ((DWORD(BYTE(b))) shl 16));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function HIBYTE(w : longint) : BYTE;
    begin
       HIBYTE:=BYTE(((WORD(w)) shr 8) and $FF);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function HIWORD(l : longint) : WORD;
    begin
       HIWORD:=WORD(((DWORD(l)) shr 16) and $FFFF);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LOBYTE(w : longint) : BYTE;
    begin
       LOBYTE:=BYTE(w);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LOWORD(l : longint) : WORD;
    begin
       LOWORD:=WORD(l);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELONG(a,b : longint) : LONG;
    begin
       MAKELONG:=LONG((WORD(a)) or ((DWORD(WORD(b))) shl 16));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKEWORD(a,b : longint) : WORD;
    begin
       MAKEWORD:=WORD((BYTE(a)) or ((WORD(BYTE(b))) shl 8));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function SEXT_HIWORD(l : longint) : longint;
    { return type might be wrong }
    begin
       SEXT_HIWORD:=(longint(l)) shr 16;
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function ZEXT_HIWORD(l : longint) : longint;
    { return type might be wrong }
    begin
       ZEXT_HIWORD:=(cardinal(l)) shr 16;
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function SEXT_LOWORD(l : longint) : longint;
    begin
       SEXT_LOWORD:=longint(SHORT(l));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function INDEXTOOVERLAYMASK(i : longint) : longint;
    { return type might be wrong }
    begin
       INDEXTOOVERLAYMASK:=i shl 8;
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function INDEXTOSTATEIMAGEMASK(i : longint) : longint;
    { return type might be wrong }
    begin
       INDEXTOSTATEIMAGEMASK:=i shl 12;
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKEINTATOM(i : longint) : LPTSTR;
    begin
       MAKEINTATOM:=LPTSTR(ULONG_PTR(WORD(i)));
    end;


  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function MAKELANGID(p,s : longint) : longint;
    { return type might be wrong }
    begin
       MAKELANGID:=((WORD(s)) shl 10) or (WORD(p));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function PRIMARYLANGID(lgid : longint) : WORD;
    begin
       { PRIMARYLANGID:=WORD(lgid(@($3ff)));
         h2pas error here corrected by hand PM }
       PRIMARYLANGID:=WORD(lgid) and ($3ff);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function SUBLANGID(lgid : longint) : longint;
    { return type might be wrong }
    begin
       SUBLANGID:=(WORD(lgid)) shr 10;
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LANGIDFROMLCID(lcid : longint) : WORD;
    begin
       LANGIDFROMLCID:=WORD(lcid);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function SORTIDFROMLCID(lcid : longint) : WORD;
    begin
       SORTIDFROMLCID:=WORD(((DWORD(lcid)) and $000FFFFF) shr 16);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELCID(lgid,srtid : longint) : DWORD;
    begin
       MAKELCID:=DWORD(((DWORD(WORD(srtid))) shl 16) or (DWORD(WORD(lgid))));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELPARAM(l,h : longint) : LPARAM;
    begin
       MAKELPARAM:=LPARAM(MAKELONG(l,h));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELRESULT(l,h : longint) : LRESULT;
    begin
       MAKELRESULT:=LRESULT(MAKELONG(l,h));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKEROP4(fore,back : longint) : DWORD;
    begin
       MAKEROP4:=DWORD((DWORD(back shl 8) and $FF000000) or DWORD(fore));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKEWPARAM(l,h : longint) : WPARAM;
    begin
       MAKEWPARAM:=WPARAM(MAKELONG(l,h));
    end;

  function GET_X_LPARAM(lp : Windows.LParam) : longint;
    begin
      result:=smallint(LOWORD(lp));
    end;

  function GET_Y_LPARAM(lp : Windows.LParam) : longint;
    begin
      result:=smallint(HIWORD(lp));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function max(a,b : longint) : longint;
    begin
       if a > b then
         max:=a
       else
         max:=b;
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function min(a,b : longint) : longint;
    begin
       if a < b then
         min:=a
       else
         min:=b;
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function PALETTEINDEX(i : longint) : COLORREF;
    begin
       PALETTEINDEX:=COLORREF($01000000 or (DWORD(WORD(i))));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function PALETTERGB(r,g,b : longint) : longint;
    { return type might be wrong }
    begin
       PALETTERGB:=$02000000 or (RGB(r,g,b));
    end;

  function IMAGE_ORDINAL64(Ordinal : uint64) : uint64;
  begin
    IMAGE_ORDINAL64:=Ordinal and $ffff;
  end;

  function IMAGE_ORDINAL32(Ordinal : cardinal) : cardinal;
  begin
    IMAGE_ORDINAL32:=Ordinal and $ffff;
  end;

  function IMAGE_SNAP_BY_ORDINAL64(Ordinal : uint64) : boolean;
  begin
    IMAGE_SNAP_BY_ORDINAL64:=(Ordinal and IMAGE_ORDINAL_FLAG64)<>0;
  end;

  function IMAGE_SNAP_BY_ORDINAL32(Ordinal : cardinal) : boolean;
  begin
    IMAGE_SNAP_BY_ORDINAL32:=(Ordinal and IMAGE_ORDINAL_FLAG32)<>0;
  end;

  {$ifdef win64}
  function IMAGE_ORDINAL(Ordinal : uint64) : uint64;
  begin
    IMAGE_ORDINAL:=IMAGE_ORDINAL64(Ordinal);
  end;


  function IMAGE_SNAP_BY_ORDINAL(Ordinal : uint64) : boolean;
  begin
    IMAGE_SNAP_BY_ORDINAL:=IMAGE_SNAP_BY_ORDINAL64(Ordinal);
  end;

  {$else}

  function IMAGE_ORDINAL(Ordinal : cardinal) : cardinal;
  begin
    IMAGE_ORDINAL:=IMAGE_ORDINAL32(Ordinal);
  end;

  function IMAGE_SNAP_BY_ORDINAL(Ordinal : cardinal) : boolean;
  begin
    IMAGE_SNAP_BY_ORDINAL:=IMAGE_SNAP_BY_ORDINAL32(Ordinal);
  end;
 {$endif}
{$endif read_implementation}

