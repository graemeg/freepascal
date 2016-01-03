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

  Changes :

  08/15/2005 update for wince4.2 port,ORO06
}

{$ifdef read_interface}

{$PACKRECORDS C}
  const
     ANYSIZE_ARRAY = 1;               //~winnt, moved from define to compile SID
  type

     MAKEINTRESOURCE = pwidechar;
<<<<<<< HEAD
<<<<<<< HEAD
     
     ATOM = word;                     //windef
=======
	 ATOM = word;                     //windef
>>>>>>> graemeg/fixes_2_2
=======
	 ATOM = word;                     //windef
>>>>>>> origin/fixes_2_2
     TAtom = ATOM;

     WINBOOL = longbool;
     BOOL = WINBOOL;                  //windef

     CALTYPE = cardinal;
     CALID = cardinal;

     CCHAR = char;                    //winnt

     COLORREF = Cardinal;             //windef
     TCOLORREF = cardinal;

     SHORT = smallint;                //winnt
     WINT  = longint;
     LONG  = longint;                 //winnt
     LONG64= int64;
     ULONG64 = qword;                 // imagehlp header.
     ULONG32 = cardinal;
     DWORD = Cardinal;                //windef

     PINTEGER = ^longint;
     PBOOL    = ^BOOL;                //windef

     LONGLONG  = int64;               //winnt
     PLONGLONG = ^LONGLONG;           //winnt
     ULONGLONG  = qword;              //+winnt
     PULONGLONG = ^ULONGLONG;         //+winnt

     DWORDLONG  = qword;  { was unsigned long  }
     PDWORDLONG = ^DWORDLONG;
     
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

     HANDLE = System.THandle;
     HRESULT = System.HResult;  //winnt LONG
     PHRESULT= ^HRESULT;

     HACCEL = HANDLE;        //windef
     HBITMAP = HANDLE;       //windef
     HBRUSH = HANDLE;        //windef
     HCOLORSPACE = HANDLE;   //windef
     HCONV = HANDLE;
     HCONVLIST = HANDLE;
     HCURSOR = HANDLE;       //windef
     HDBC = HANDLE;
     HDC = HANDLE;           //windef
     HDDEDATA = HANDLE;
     HDESK = HANDLE;         //windef
     HDROP = HANDLE;
     HDWP = HANDLE;
     HENHMETAFILE = HANDLE;  //windef
     HENV = HANDLE;
     HEVENT = HANDLE;        //+windef
     HFILE = HANDLE;         //windef
     HFONT = HANDLE;         //windef
     HGDIOBJ = HANDLE;       //windef
     HGLOBAL = HANDLE;       //windef
     HGLRC = HANDLE;         //windef
     HHOOK = HANDLE;         //windef
     HICON = HCURSOR;        //~windef
     LPHICON = ^HICON;       //+tapi
     HIMAGELIST = HANDLE;
     HINST = HANDLE;   { Not HINSTANCE, else it has problems with the var HInstance }
     HKEY = HANDLE;          //windef
     HKL = HANDLE;           //windef
     HLOCAL = HANDLE;        //windef
     HMENU = HANDLE;         //windef
     HMETAFILE = HANDLE;     //windef
     HMODULE = HANDLE;       //windef
     HMONITOR = HANDLE;      //+windef
     HPALETTE = HANDLE;      //windef
     HPEN = HANDLE;          //windef
     HRASCONN = HANDLE;
     HRGN = HANDLE;          //windef
     HRSRC = HANDLE;         //windef
     HSTMT = HANDLE;
     HSTR  = HANDLE;         //windef
     HSZ = HANDLE;
     HTASK = HANDLE;         //windef
     HWINSTA = HANDLE;       //windef
     HWND = HANDLE;          //windef
     HWINEVENTHOOK = HANDLE; //+windef

     LANGID = word;          //winnt
     LCID   = DWORD;         //winnt
     PLCID  = ^LCID;         //+winnt
     LCTYPE = DWORD;
<<<<<<< HEAD
<<<<<<< HEAD
     LPARAM = ptrint;        //windef LONG_PTR

     LP     = ^word;
     LPBOOL = ^WINBOOL;      //windef
     LPBYTE = System.pbyte;         //windef
=======
=======
>>>>>>> origin/fixes_2_2
     LPARAM = longint;       //windef LONG_PTR

     LP     = ^word;
     LPBOOL = ^WINBOOL;      //windef
     LPBYTE = ^BYTE;         //windef
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     LPCCH  = PCHAR;         //winnt
     LPCH   = PCHAR;         //winnt

     LPCOLORREF = ^COLORREF; //windef

     LPCSTR  = Pchar;        //winnt
{$ifdef UNICODE}
     LPCTSTR = Pwidechar;    //winnt
{$else}
     LPCTSTR = Pchar;        //winnt
{$endif}

     LPCWCH  = Pwidechar;   //winnt
     LPCWSTR = Pwidechar;   //winnt

<<<<<<< HEAD
<<<<<<< HEAD
     LPDWORD = system.pdword;      //windef
=======
     LPDWORD = ^DWORD;      //windef
>>>>>>> graemeg/fixes_2_2
=======
     LPDWORD = ^DWORD;      //windef
>>>>>>> origin/fixes_2_2

     LPHANDLE = ^HANDLE;    //windef
     LPINT  = ^longint;     //windef
     LPLONG = ^longint;     //windef

     LPSTR = Pchar;         //winnt
{$ifdef UNICODE}
     LPTCH  = Pwidechar;
     LPTSTR = Pwidechar;
{$else}
     LPTCH  = Pchar;
     LPTSTR = Pchar;
{$endif}

     LRESULT = longint;     //windef LONG_PTR

     LPVOID  = pointer;     //windef

     LPCVOID = pointer;     //windef

     LPWCH  = Pwidechar;
<<<<<<< HEAD
<<<<<<< HEAD
     LPWORD = system.pword;        //windef
=======
     LPWORD = ^word;        //windef
>>>>>>> graemeg/fixes_2_2
=======
     LPWORD = ^word;        //windef
>>>>>>> origin/fixes_2_2
     LPWSTR = Pwidechar;    //winnt

     //
     // Locally Unique Identifier
     //
     //winnt : declaration moved and changed : was in struct as LUID = TlargeInteger
     LUID = record
       LowPart  : DWORD;
       HighPart : LONG ;
     end;
     TLUID = LUID;
     PLUID = ^LUID;

     NWPSTR = Pwidechar;    //winnt

     PWINBOOL = ^WINBOOL;
     PBOOLEAN = ^BYTE;

     PBYTE = ^BYTE;         //windef

     PCCH = PCHAR;          //winnt
     PCH  = PCHAR;          //winnt

     PCSTR = Pchar;

     PCWCH  = Pwidechar;    //winnt
     PCWSTR = Pwidechar;    //winnt

     PDWORD = ^DWORD;       //windef

     PHANDLE = ^HANDLE;     //windef
     PHKEY = ^HKEY;         //windef

     PINT = ^longint;
     PLONG = ^longint;      //windef
     PSHORT = ^SHORT;       //windef
     LPSHORT = ^SHORT;      //+windef

     PSTR = Pchar;

     PSZ = Pchar;         //winnt
{$ifdef UNICODE}
     PTBYTE = ^word;      //winnt
     PTCH = Pwidechar;
     PTCHAR = Pwidechar;  //winnt
     PTSTR = Pwidechar;
{$else}
     PTBYTE = ^byte;
     PTCH   = Pchar;
     PTCHAR = Pchar;
     PTSTR  = Pchar;
{$endif}

     PUCHAR = ^byte;        //windef
     PWCH   = Pwidechar;    //winnt
     PWCHAR = Pwidechar;    //winnt

     PWORD   = ^word;       //windef
     PUINT   = ^Cardinal;   //windef
     PULONG  = ^Cardinal;   //windef
     PUSHORT = ^word;       //windef

     PVOID = pointer;

     RETCODE = SHORT;

     SC_HANDLE = HANDLE;
     SC_LOCK = LPVOID;
     LPSC_HANDLE = ^SC_HANDLE;
     SPHANDLE = ^HANDLE;       //+windef

     SERVICE_STATUS_HANDLE = DWORD;
     SIZE_T = Cardinal;        //+winnt
{$ifdef UNICODE}
     TBYTE = word;
     TCHAR = widechar;
     BCHAR = word;
{$else}
     TBYTE = byte;
     TCHAR = char;
     BCHAR = BYTE;
{$endif}

     UCHAR = byte;          //windef
//     WCHAR = WideChar;      //winnt

     UINT   = Cardinal;     //windef
     ULONG  = Cardinal;     //windef
     USHORT = word;         //windef

     PLPSTR = ^LPSTR;
     PLPWStr= ^LPWStr;

     FLOAT  = single;       //+windef
     PFLOAT = ^FLOAT;       //+windef
     WCHAR_T = USHORT;      //+stdlib
     WINT_T = WCHAR_T;      //+stdlib
     WCTYPE_T = WCHAR_T;    //+stdlib
     TIME_T=ULONG;          //+stdlib
<<<<<<< HEAD
<<<<<<< HEAD
     WPARAM = ptruint;      //windef UINT_PTR
=======
     WPARAM = Longint;      //windef UINT_PTR
>>>>>>> graemeg/fixes_2_2
=======
     WPARAM = Longint;      //windef UINT_PTR
>>>>>>> origin/fixes_2_2

// The left side of these typedefs are machine and compiler dependent.
// From types.h
type
     INT8   = ShortInt;
     UINT8  = byte;
     INT16  = SmallInt;
     UINT16 = word;
     INT32  = longint;
     UINT32 = LongWord;
// INT64
     UINT64 = QWord;

type
     _INT128 = packed record
       u0,u1,u2:UINT32;
       u3:INT32;
     end;
     INT128 = _INT128;

     _UINT128 = packed record
       u0,u1,u2,u3:UINT32;
     end;
     UINT128 = _UINT128;

type
     PINT8 = ^INT8;
     PUINT8 = ^UINT8;
     PINT16 = ^INT16;
     PUINT16 = ^UINT16;
     PINT32 = ^INT32;
     PUINT32 = ^UINT32;
//     PINT64
     PUINT64 = ^UINT64;
     PINT128 = ^INT128;
     PUINT128 = ^UINT128;

type
     PPVOID = ^pointer;

type
     WIN32_ERROR_CODE = DWORD;


{
  Enumerations
}

     ACL_INFORMATION_CLASS = (AclRevisionInformation := 1,AclSizeInformation
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

     RASCONNSTATE = (RASCS_OpenPort := 0,RASCS_PortOpened,
       RASCS_ConnectDevice,RASCS_DeviceConnected,
       RASCS_AllDevicesConnected,RASCS_Authenticate,
       RASCS_AuthNotify,RASCS_AuthRetry,RASCS_AuthCallback,
       RASCS_AuthChangePassword,RASCS_AuthProject,
       RASCS_AuthLinkSpeed,RASCS_AuthAck,RASCS_ReAuthenticate,
       RASCS_Authenticated,RASCS_PrepareForCallback,
       RASCS_WaitForModemReset,RASCS_WaitForCallback,
       RASCS_Projected,RASCS_StartAuthentication,
       RASCS_CallbackComplete,RASCS_LogonNetwork,
       RASCS_Interactive := RASCS_PAUSED,RASCS_RetryAuthentication,
       RASCS_CallbackSetByCaller,RASCS_PasswordExpired,
       RASCS_Connected := RASCS_DONE,RASCS_Disconnected
       );

     _RASCONNSTATE = RASCONNSTATE;

     RASPROJECTION = (RASP_PppIp := $8021, RASP_PppIpx := $802B, RASP_PppNbf := $803F,
       RASP_Amb := $10000);

     _RASPROJECTION = RASPROJECTION;

     PSECURITY_DESCRIPTO = PVOID;        //+winnt
     SECURITY_IMPERSONATION_LEVEL = (SecurityAnonymous,SecurityIdentification,
       SecurityImpersonation,SecurityDelegation
       );

     _SECURITY_IMPERSONATION_LEVEL = SECURITY_IMPERSONATION_LEVEL;

     SID_IDENTIFIER_AUTHORITY_REC = array[0..5] of BYTE;  //~ added for pre-defined SID declaration
     SID_IDENTIFIER_AUTHORITY = record                    //~winnt, moved to declare pre-defined SID
          Value : SID_IDENTIFIER_AUTHORITY_REC;
       end;
     LPSID_IDENTIFIER_AUTHORITY = ^SID_IDENTIFIER_AUTHORITY;
     PSID_IDENTIFIER_AUTHORITY = ^SID_IDENTIFIER_AUTHORITY;
     _SID_IDENTIFIER_AUTHORITY = SID_IDENTIFIER_AUTHORITY;
     TSIDIDENTIFIERAUTHORITY = SID_IDENTIFIER_AUTHORITY;
     PSIDIDENTIFIERAUTHORITY = ^SID_IDENTIFIER_AUTHORITY;


     SID = record                 //~wint
       Revision : Byte;
       SubAuthorityCount : Byte;
       IdentifierAuthority : SID_IDENTIFIER_AUTHORITY;
       SubAuthority : Array [0..ANYSIZE_ARRAY-1] of DWORD;
     end;
     _SID = SID;
     PSID = ^SID;
     
     SID_NAME_USE = (             //~winnt, added SidTypeComputer
      SidTypeUser:= 1,
      SidTypeGroup,
      SidTypeDomain,
      SidTypeAlias,
      SidTypeWellKnownGroup,
      SidTypeDeletedAccount,
      SidTypeInvalid,
      SidTypeUnknown,
      SidTypeComputer);

     PSID_NAME_USE = ^SID_NAME_USE;      //winnt
     _SID_NAME_USE = SID_NAME_USE;

     TOKEN_INFORMATION_CLASS = (TokenUser := 1,TokenGroups,TokenPrivileges,
       TokenOwner,TokenPrimaryGroup,TokenDefaultDacl,
       TokenSource,TokenType,TokenImpersonationLevel,
       TokenStatistics);

     _TOKEN_INFORMATION_CLASS = TOKEN_INFORMATION_CLASS;
     TTokenInformationClass   = TOKEN_INFORMATION_CLASS;

     TOKEN_TYPE = (TokenPrimary := 1,TokenImpersonation
       );

     tagTOKEN_TYPE = TOKEN_TYPE;

 {
   Macros
 }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GetBValue(rgb : longint) : BYTE;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GetGValue(rgb : longint) : BYTE;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GetRValue(rgb : longint) : BYTE;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function RGB(r,g,b : longint) : DWORD;

  {  Not convertable by H2PAS
  #define HANDLE_WM_NOTIFY(hwnd, wParam, lParam, fn) \
      (fn)((hwnd), (int)(wParam), (NMHDR FAR )(lParam))
   }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function HIBYTE(w : longint) : BYTE;      //windef

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function HIWORD(l : longint) : WORD;      //windef

<<<<<<< HEAD
<<<<<<< HEAD
type
     LOBYTE = byte;
(*
  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LOBYTE(w : longint) : BYTE;      //windef
*)
=======
  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LOBYTE(w : longint) : BYTE;      //windef
>>>>>>> graemeg/fixes_2_2
=======
  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LOBYTE(w : longint) : BYTE;      //windef
>>>>>>> origin/fixes_2_2

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LOWORD(l : longint) : WORD;      //windef

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELONG(a,b : longint) : LONG;  //windef

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKEWORD(a,b : longint) : WORD;  //windef

  { original Cygnus headers also had the following defined:  }
  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function SEXT_HIWORD(l : longint) : longint;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function ZEXT_HIWORD(l : longint) : longint;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function SEXT_LOWORD(l : longint) : longint;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function INDEXTOOVERLAYMASK(i : longint) : longint;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function INDEXTOSTATEIMAGEMASK(i : longint) : longint;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
<<<<<<< HEAD
<<<<<<< HEAD
  function MAKEINTATOM(i : longint) : LPTSTR; 
=======
  function MAKEINTATOM(i : longint) : LPTSTR;

>>>>>>> graemeg/fixes_2_2
=======
  function MAKEINTATOM(i : longint) : LPTSTR;

>>>>>>> origin/fixes_2_2

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function MAKELANGID(p,s : longint) : longint;    //winnt
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function PRIMARYLANGID(lgid : longint) : WORD;   //winnt

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function SUBLANGID(lgid : longint) : longint;    //winnt
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LANGIDFROMLCID(lcid : longint) : WORD;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function SORTIDFROMLCID(lcid : longint) : WORD;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELCID(lgid,srtid : longint) : DWORD;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELPARAM(l,h : longint) : LPARAM;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKELRESULT(l,h : longint) : LRESULT;

  {  Not convertable by H2PAS
  #define MAKEPOINTS(l)   ( ((POINTS FAR  ) & (l)))
   }
  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKEROP4(fore,back : longint) : DWORD;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function MAKEWPARAM(l,h : longint) : WPARAM;

{$ifndef max}
  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function max(a,b : longint) : longint;  //windef
    { return type might be wrong }

{$endif}
{$ifndef min}
  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function min(a,b : longint) : longint;  //windef
    { return type might be wrong }

{$endif}
  { was #define dname(params) def_expr }
  { argument types are unknown }
  function PALETTEINDEX(i : longint) : COLORREF;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function PALETTERGB(r,g,b : longint) : longint;
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

 {
    Definitions for callback procedures
 }

 type

     BFFCALLBACK = function (_para1:HWND; _para2:UINT; _para3:LPARAM; _para4:LPARAM):longint;

     LPCCHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT;

     LPCFHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT;

     PTHREAD_START_ROUTINE = Pointer;

     LPTHREAD_START_ROUTINE = PTHREAD_START_ROUTINE;

     EDITSTREAMCALLBACK = function (_para1:DWORD; _para2:LPBYTE; _para3:LONG; _para4:LONG):DWORD;

     LPFRHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT;

     LPOFNHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT;

     LPPRINTHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT;

     LPSETUPHOOKPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT;

     DLGPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):LRESULT;

     PFNPROPSHEETCALLBACK = function (_para1:HWND; _para2:UINT; _para3:LPARAM):longint;

     LPSERVICE_MAIN_FUNCTION = procedure (_para1:DWORD; _para2:LPTSTR);

     PFNTVCOMPARE = function (_para1:LPARAM; _para2:LPARAM; _para3:LPARAM):longint;

     WNDPROC = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):LRESULT;


     FARPROC = pointer;    //windef
     NEARPROC= FARPROC;    //+windef
     PROC = FARPROC;       //~windef

     ENUMRESTYPEPROC = function (_para1:HANDLE; _para2:LPTSTR; _para3:LONG):WINBOOL;

     ENUMRESNAMEPROC = function (_para1:HANDLE; _para2:LPCTSTR; _para3:LPTSTR; _para4:LONG):WINBOOL;

     ENUMRESLANGPROC = function (_para1:HANDLE; _para2:LPCTSTR; _para3:LPCTSTR; _para4:WORD; _para5:LONG):WINBOOL;

     DESKTOPENUMPROC = FARPROC;

     ENUMWINDOWSPROC = function (_para1:HWND; _para2:LPARAM):WINBOOL;

     ENUMWINDOWSTATIONPROC = function (_para1:LPTSTR; _para2:LPARAM):WINBOOL;

     SENDASYNCPROC = procedure (_para1:HWND; _para2:UINT; _para3:DWORD; _para4:LRESULT);

     TIMERPROC = procedure (_para1:HWND; _para2:UINT; _para3:UINT; _para4:DWORD);

     GRAYSTRINGPROC = FARPROC;

     DRAWSTATEPROC = function (_para1:HDC; _para2:LPARAM; _para3:WPARAM; _para4:longint; _para5:longint):WINBOOL;

     PROPENUMPROCEX = function (_para1:HWND; _para2:LPCTSTR; _para3:HANDLE; _para4:DWORD):WINBOOL;

     PROPENUMPROC = function (_para1:HWND; _para2:LPCTSTR; _para3:HANDLE):WINBOOL;

     HOOKPROC = function (_para1:longint; _para2:WPARAM; _para3:LPARAM):LRESULT;

     ENUMOBJECTSPROC = procedure (_para1:LPVOID; _para2:LPARAM);

     LINEDDAPROC = procedure (_para1:longint; _para2:longint; _para3:LPARAM);

     TABORTPROC = function (_para1:HDC; _para2:longint):WINBOOL;

     LPPAGEPAINTHOOK = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT;

     LPPAGESETUPHOOK = function (_para1:HWND; _para2:UINT; _para3:WPARAM; _para4:LPARAM):UINT;

     ICMENUMPROC = function (_para1:LPTSTR; _para2:LPARAM):longint;

     EDITWORDBREAKPROCEX = function (_para1:pchar; _para2:LONG; _para3:BYTE; _para4:WINT):LONG;

     PFNLVCOMPARE = function (_para1:LPARAM; _para2:LPARAM; _para3:LPARAM):longint;

     LOCALE_ENUMPROC = function (_para1:LPTSTR):WINBOOL;

     CODEPAGE_ENUMPROC = function (_para1:LPTSTR):WINBOOL;
     CODEPAGE_ENUMPROCW = function (_para1:LPWSTR):WINBOOL;        //+winnls
     DATEFMT_ENUMPROC = function (_para1:LPTSTR):WINBOOL;

     TIMEFMT_ENUMPROC = function (_para1:LPTSTR):WINBOOL;

     CALINFO_ENUMPROC = function (_para1:LPTSTR):WINBOOL;

     PHANDLER_ROUTINE = function (_para1:DWORD):WINBOOL;

     LPHANDLER_FUNCTION = function (_para1:DWORD):WINBOOL;

     PFNGETPROFILEPATH = function (_para1:LPCTSTR; _para2:LPSTR; _para3:UINT):UINT;

     PFNRECONCILEPROFILE = function (_para1:LPCTSTR; _para2:LPCTSTR; _para3:DWORD):UINT;

     PFNPROCESSPOLICIES = function (_para1:HWND; _para2:LPCTSTR; _para3:LPCTSTR; _para4:LPCTSTR; _para5:DWORD):WINBOOL;
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

     //SERVICES_ACTIVE_DATABASEW      L"ServicesActive"
     //#define SERVICES_FAILED_DATABASEW      L"ServicesFailed"
     SERVICES_ACTIVE_DATABASEA = 'ServicesActive';
     SERVICES_FAILED_DATABASEA = 'ServicesFailed';
  {  Not convertable by H2PAS
  #define SC_GROUP_IDENTIFIERW           L'+'
   }
     SC_GROUP_IDENTIFIERA = '+';
{$ifdef UNICODE}
     //temporary removed to compile in unicode, ORO06 08/2005
     //SERVICES_ACTIVE_DATABASE = SERVICES_ACTIVE_DATABASEW;
     //SERVICES_FAILED_DATABASE = SERVICES_FAILED_DATABASEW;
     //SC_GROUP_IDENTIFIER = SC_GROUP_IDENTIFIERW;
     SERVICES_ACTIVE_DATABASE = SERVICES_ACTIVE_DATABASEA;
     SERVICES_FAILED_DATABASE = SERVICES_FAILED_DATABASEA;
     SC_GROUP_IDENTIFIER = SC_GROUP_IDENTIFIERA;
{$else}
     SERVICES_ACTIVE_DATABASE = SERVICES_ACTIVE_DATABASEA;
     SERVICES_FAILED_DATABASE = SERVICES_FAILED_DATABASEA;
     SC_GROUP_IDENTIFIER = SC_GROUP_IDENTIFIERA;
{$endif}

type

     { PFNCALLBACK = CALLB; }
     PFNCALLBACK = function(_para1, _para2: UINT;_para3: HCONV;_para4, _para5: HSZ; _para6: HDDEDATA;_para7 ,_para8 :DWORD): HDDEData;
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

<<<<<<< HEAD
<<<<<<< HEAD
(*
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LOBYTE(w : longint) : BYTE;
    begin
       LOBYTE:=BYTE(w);
    end;
<<<<<<< HEAD
<<<<<<< HEAD
*)

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LOWORD(l : longint) : WORD; inline;
=======
=======
>>>>>>> origin/fixes_2_2

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function LOWORD(l : longint) : WORD;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
       ZEXT_HIWORD:=(Cardinal(l)) shr 16;
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
<<<<<<< HEAD
<<<<<<< HEAD
  function MAKEINTATOM(i : longint) : LPTSTR; inline;
=======
  function MAKEINTATOM(i : longint) : LPTSTR;
>>>>>>> graemeg/fixes_2_2
=======
  function MAKEINTATOM(i : longint) : LPTSTR;
>>>>>>> origin/fixes_2_2
    begin
       MAKEINTATOM:=LPTSTR(DWORD(WORD(i)));
    end;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
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

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function max(a,b : longint) : longint;
    { return type might be wrong }
    var
       if_local1 : longint;
    (* result types are not known *)
    begin
       if a > b then
         if_local1:=a
       else
         if_local1:=b;
       max:=if_local1;
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function min(a,b : longint) : longint;
    { return type might be wrong }
    var
       if_local1 : longint;
    (* result types are not known *)
    begin
       if a < b then
         if_local1:=a
       else
         if_local1:=b;
       min:=if_local1;
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

{$endif read_implementation}



