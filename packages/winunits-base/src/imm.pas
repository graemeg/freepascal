(********************************************************************* *)
(*      imm.h - Input Method Manager definitions                       *)
(*                                                                     *)
(*      Copyright (c) Microsoft Corporation. All rights reserved.      *)
(*                                                                     *)
(* Converted to pascal by Dmitry Boyarintsev, using Chelper.           *)
(*                                                                     *)
(********************************************************************* *)

unit
  imm;

{$mode delphi}

interface

uses
  Windows;

{$PACKRECORDS C}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_OS_UNICODE}
  {$define UNICODE}
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
// IME_Codes
// bit field for conversion mode
const
  IME_CMODE_ALPHANUMERIC          = $0000;
  IME_CMODE_NATIVE                = $0001;
  IME_CMODE_CHINESE               = IME_CMODE_NATIVE;
  IME_CMODE_HANGUL                = IME_CMODE_NATIVE;
  IME_CMODE_JAPANESE              = IME_CMODE_NATIVE;
  IME_CMODE_KATAKANA              = $0002;  // only effect under IME_CMODE_NATIVE
  IME_CMODE_LANGUAGE              = $0003;
  IME_CMODE_FULLSHAPE             = $0008;
  IME_CMODE_ROMAN                 = $0010;
  IME_CMODE_CHARCODE              = $0020;
  IME_CMODE_HANJACONVERT          = $0040;

type
  HIMC = DWORD;
  HIMCC = DWORD;

type
  PHKL  = ^HKL;
  LPHKL = PHKL;

type
  LPUINT = PUINT;

// declared at rtl\win\wininc\struct.inc
{type
  PtagCOMPOSITIONFORM = ^tagCOMPOSITIONFORM;
  tagCOMPOSITIONFORM = record
    dwStyle       : DWORD;
    ptCurrentPos  : POINT;
    rcArea        : RECT;
  end;
  COMPOSITIONFORM = tagCOMPOSITIONFORM;
  PCOMPOSITIONFORM = PtagCOMPOSITIONFORM;
  NPCOMPOSITIONFORM = PtagCOMPOSITIONFORM;
  LPCOMPOSITIONFORM = PtagCOMPOSITIONFORM;}

// declared at rtl\win\wininc\struct.inc
{type
  PtagCANDIDATEFORM = ^tagCANDIDATEFORM;
  tagCANDIDATEFORM = packed record
    dwIndex      : DWORD;
    dwStyle      : DWORD;
    ptCurrentPos : POINT;
    rcArea       : RECT;
  end;
  CANDIDATEFORM = tagCANDIDATEFORM;
  PCANDIDATEFORM = PtagCANDIDATEFORM;
  NPCANDIDATEFORM = PtagCANDIDATEFORM;
  LPCANDIDATEFORM = PtagCANDIDATEFORM;}


// declared at rtl\win\wininc\struct.inc
{type
  PtagCANDIDATELIST = ^tagCANDIDATELIST;
  tagCANDIDATELIST = packed record
    dwSize : DWORD;
    dwStyle : DWORD;
    dwCount : DWORD;
    dwSelection : DWORD;
    dwPageStart : DWORD;
    dwPageSize : DWORD;
    dwOffset : array [0..0] of DWORD;
  end;
  CANDIDATELIST = tagCANDIDATELIST;
  PCANDIDATELIST = PtagCANDIDATELIST;
  NPCANDIDATELIST = PtagCANDIDATELIST;
  LPCANDIDATELIST = PtagCANDIDATELIST;}


type
  PtagREGISTERWORDA = ^tagREGISTERWORDA;
  tagREGISTERWORDA = packed record
    lpReading : LPSTR;
    lpWord : LPSTR;
  end;
  REGISTERWORDA = tagREGISTERWORDA;
  TREGISTERWORDA = REGISTERWORDA;
  PREGISTERWORDA = PtagREGISTERWORDA;
  NPREGISTERWORDA = PtagREGISTERWORDA;
  LPREGISTERWORDA = PtagREGISTERWORDA;

type
  PtagREGISTERWORDW = ^tagREGISTERWORDW;
  tagREGISTERWORDW = packed record
    lpReading : LPWSTR;
    lpWord : LPWSTR;
  end;
  REGISTERWORDW = tagREGISTERWORDW;
  TREGISTERWORDW = REGISTERWORDW;
  PREGISTERWORDW = PtagREGISTERWORDW;
  NPREGISTERWORDW = PtagREGISTERWORDW;
  LPREGISTERWORDW = PtagREGISTERWORDW;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef UNICODE}
  PtagREGISTERWORD = PtagREGISTERWORDW;
  tagREGISTERWORD  = tagREGISTERWORDW;
  REGISTERWORD     = REGISTERWORDW;     
  TREGISTERWORD    = TREGISTERWORDW;
  PREGISTERWORD    = PREGISTERWORDW;
  NPREGISTERWORD   = NPREGISTERWORDW;
  LPREGISTERWORD   = LPREGISTERWORDW;
{$else}
  PtagREGISTERWORD = PtagREGISTERWORDA;
  tagREGISTERWORD  = tagREGISTERWORDA;
  REGISTERWORD     = REGISTERWORDA;
  TREGISTERWORD    = TREGISTERWORDA;
  PREGISTERWORD    = PREGISTERWORDA;
  NPREGISTERWORD   = NPREGISTERWORDA;
  LPREGISTERWORD   = LPREGISTERWORDA;
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
type
  PtagRECONVERTSTRING = ^tagRECONVERTSTRING;
  tagRECONVERTSTRING = packed record
    dwSize : DWORD;
    dwVersion : DWORD;
    dwStrLen : DWORD;
    dwStrOffset : DWORD;
    dwCompStrLen : DWORD;
    dwCompStrOffset : DWORD;
    dwTargetStrLen : DWORD;
    dwTargetStrOffset : DWORD;
  end;
  RECONVERTSTRING = tagRECONVERTSTRING;
  TRECONVERTSTRING = RECONVERTSTRING;
  PRECONVERTSTRING = PtagRECONVERTSTRING;
  NPRECONVERTSTRING = PtagRECONVERTSTRING;
  LPRECONVERTSTRING = PtagRECONVERTSTRING;

const
  STYLE_DESCRIPTION_SIZE = 32;

type
  PtagSTYLEBUFA = ^tagSTYLEBUFA;
  tagSTYLEBUFA = packed record
    dwStyle : DWORD;
    szDescription : array [0..STYLE_DESCRIPTION_SIZE-1] of AnsiChar;
  end;
  STYLEBUFA = tagSTYLEBUFA;
  TSTYLEBUFA = STYLEBUFA;
  PSTYLEBUFA = PtagSTYLEBUFA;
  NPSTYLEBUFA = PtagSTYLEBUFA;
  LPSTYLEBUFA = PtagSTYLEBUFA;

type
  PtagSTYLEBUFW = ^tagSTYLEBUFW;
  tagSTYLEBUFW = packed record
    dwStyle : DWORD;
    szDescription : array [0..STYLE_DESCRIPTION_SIZE-1] of WCHAR;
  end;
  STYLEBUFW = tagSTYLEBUFW;
  TSTYLEBUFW = STYLEBUFW;
  PSTYLEBUFW = PtagSTYLEBUFW;
  NPSTYLEBUFW = PtagSTYLEBUFW;
  LPSTYLEBUFW = PtagSTYLEBUFW;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef UNICODE}
  STYLEBUF    = STYLEBUFW;
  TSTYLEBUF   = TSTYLEBUFW;
  PSTYLEBUF   = PSTYLEBUFW;
  NPSTYLEBUF  = NPSTYLEBUFW;
  LPSTYLEBUF  = LPSTYLEBUFW;
{$else}
  STYLEBUF    = STYLEBUFA;
  TSTYLEBUF   = TSTYLEBUFA;
  PSTYLEBUF   = PSTYLEBUFA;
  NPSTYLEBUF  = NPSTYLEBUFA;
  LPSTYLEBUF  = LPSTYLEBUFA;
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
const
  IMEMENUITEM_STRING_SIZE = 80;

type
  PtagIMEMENUITEMINFOA = ^tagIMEMENUITEMINFOA;
  tagIMEMENUITEMINFOA = packed record
    cbSize : UINT;
    fType : UINT;
    fState : UINT;
    wID : UINT;
    hbmpChecked : HBITMAP;
    hbmpUnchecked : HBITMAP;
    dwItemData : DWORD;
    szString : array [0..IMEMENUITEM_STRING_SIZE-1] of AnsiChar;
    hbmpItem : HBITMAP;
  end;
  IMEMENUITEMINFOA = tagIMEMENUITEMINFOA;
  TIMEMENUITEMINFOA = IMEMENUITEMINFOA;
  PIMEMENUITEMINFOA = PtagIMEMENUITEMINFOA;
  NPIMEMENUITEMINFOA = PtagIMEMENUITEMINFOA;
  LPIMEMENUITEMINFOA = PtagIMEMENUITEMINFOA;

type
  PtagIMEMENUITEMINFOW = ^tagIMEMENUITEMINFOW;
  tagIMEMENUITEMINFOW = packed record
    cbSize : UINT;
    fType : UINT;
    fState : UINT;
    wID : UINT;
    hbmpChecked : HBITMAP;
    hbmpUnchecked : HBITMAP;
    dwItemData : DWORD;
    szString : array [0..IMEMENUITEM_STRING_SIZE-1] of WCHAR;
    hbmpItem : HBITMAP;
  end;
  IMEMENUITEMINFOW = tagIMEMENUITEMINFOW;
  TIMEMENUITEMINFOW = IMEMENUITEMINFOW;
  PIMEMENUITEMINFOW = PtagIMEMENUITEMINFOW;
  NPIMEMENUITEMINFOW = PtagIMEMENUITEMINFOW;
  LPIMEMENUITEMINFOW = PtagIMEMENUITEMINFOW;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef UNICODE}
  IMEMENUITEMINFO    = IMEMENUITEMINFOW;
  TIMEMENUITEMINFO   = TIMEMENUITEMINFOW;
  PIMEMENUITEMINFO   = PIMEMENUITEMINFOW;
  NPIMEMENUITEMINFO  = NPIMEMENUITEMINFOW;
  LPIMEMENUITEMINFO  = LPIMEMENUITEMINFOW;
{$else}
  IMEMENUITEMINFO    = IMEMENUITEMINFOA;
  TIMEMENUITEMINFO   = TIMEMENUITEMINFOA;
  PIMEMENUITEMINFO   = PIMEMENUITEMINFOA;
  NPIMEMENUITEMINFO  = NPIMEMENUITEMINFOA;
  LPIMEMENUITEMINFO  = LPIMEMENUITEMINFOA;
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
type
  PtagIMECHARPOSITION = ^tagIMECHARPOSITION;
  tagIMECHARPOSITION = packed record
    dwSize : DWORD;
    dwCharPos : DWORD;
    pt : POINT;
    cLineHeight : UINT;
    rcDocument : RECT;
  end;
  IMECHARPOSITION = tagIMECHARPOSITION;
  TIMECHARPOSITION = IMECHARPOSITION;
  PIMECHARPOSITION = PtagIMECHARPOSITION;
  NPIMECHARPOSITION = PtagIMECHARPOSITION;
  LPIMECHARPOSITION = PtagIMECHARPOSITION;

type
  IMCENUMPROC = function(par0: HIMC; par1: LPARAM): LongBool; stdcall;
  TIMCENUMPROC = IMCENUMPROC;

// prototype of IMM API

const
  Imm = 'imm32';

function ImmInstallIMEA(lpszIMEFileName, lpszLayoutText: LPCSTR): HKL; stdcall; external Imm name 'ImmInstallIMEA';
function ImmInstallIMEW(lpszIMEFileName, lpszLayoutText: LPCWSTR): HKL; stdcall; external Imm name 'ImmInstallIMEW';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef UNICODE}
function ImmInstallIME(lpszIMEFileName, lpszLayoutText: LPCSTR): HKL; stdcall; external Imm name 'ImmInstallIMEA';
{$else}
function ImmInstallIME(lpszIMEFileName, lpszLayoutText: LPCWSTR): HKL; stdcall; external Imm name 'ImmInstallIMEW';
{$endif}

function ImmGetDefaultIMEWnd(wnd: HWND): HWND; stdcall; external Imm name 'ImmGetDefaultIMEWnd';
function NImmGetDescriptionA(kl: HKL; lpszDescription: LPSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetDescriptionA';
function ImmGetDescriptionW(kl: HKL; lpszDescription: LPWSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetDescriptionW';

{$ifndef UNICODE}
function ImmGetDescription(kl: HKL; lpszDescription: LPSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetDescriptionA';
{$else}
function ImmGetDescription(kl: HKL; lpszDescription: LPWSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetDescriptionW';
{$endif}

function ImmGetIMEFileNameA(kl: HKL; lpszFileName: LPSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetIMEFileNameA';
function ImmGetIMEFileNameW(kl: HKL; lpszFileName: LPWSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetIMEFileNameW';

{$ifndef UNICODE}
function ImmGetIMEFileName(kl: HKL; lpszFileName: LPSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetIMEFileNameA';
{$else}
function ImmGetIMEFileName(kl: HKL; lpszFileName: LPWSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetIMEFileNameW';
{$endif}

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmGetDefaultIMEWnd(wnd: HWND): HWND; stdcall; external Imm name 'ImmGetDefaultIMEWnd';
function ImmGetDescriptionA(kl: HKL; lpszDescription: LPSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetDescriptionA';
function ImmGetDescriptionW(kl: HKL; lpszDescription: LPWSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetDescriptionW';

function ImmGetIMEFileNameA(kl: HKL; lpszFileName: LPSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetIMEFileNameA';
function ImmGetIMEFileNameW(kl: HKL; lpszFileName: LPWSTR; uBufLen: UINT): UINT; stdcall; external Imm name 'ImmGetIMEFileNameW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmGetProperty (kl: HKL; fdwIndex: DWORD): DWORD; stdcall; external Imm name 'ImmGetProperty';
function ImmIsIME(kl: HKL): LongBool; stdcall; external Imm name 'ImmIsIME';
function ImmSimulateHotKey(wnd: HWND; dwHotKeyID: DWORD): LongBool; stdcall; external Imm name 'ImmSimulateHotKey';

function ImmCreateContext: HIMC; stdcall; external Imm name 'ImmCreateContext';
function ImmDestroyContext(imc: HIMC): LongBool; stdcall; external Imm name 'ImmDestroyContext';
function ImmGetContext(wnd: HWND): HIMC; stdcall; external Imm name 'ImmGetContext';

function ImmReleaseContext(wnd: HWND; imc: HIMC): LongBool; stdcall; external Imm name 'ImmReleaseContext';
function ImmAssociateContext(wnd: HWND; imc: HIMC): HIMC; stdcall; external Imm name 'ImmAssociateContext';

function ImmAssociateContextEx(wnd: HWND; imc: HIMC; dwFlags: DWORD): LongBool; stdcall; external Imm name 'ImmAssociateContextEx';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function ImmGetCompositionStringA(imc: HIMC; dwIndex: LONG;
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall; external Imm name 'ImmGetCompositionStringA';
function ImmGetCompositionStringW(imc: HIMC; dwIndex: LONG;
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall; external Imm name 'ImmGetCompositionStringW';

{$ifndef UNICODE}
function ImmGetCompositionString(imc: HIMC; dwIndex: DWORD;
    lpBuf: LPVOID; dwBufLen: DWORD): LONG; stdcall; external Imm name 'ImmGetCompositionStringA';
{$else}
function ImmGetCompositionString(imc: HIMC; dwIndex: DWORD;
    lpBuf: LPVOID; dwBufLen: DWORD): LONG; stdcall; external Imm name 'ImmGetCompositionStringW';
{$endif}

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmGetCompositionStringA(imc: HIMC; dwIndex: DWORD;
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall; external Imm name 'ImmGetCompositionStringA';
function ImmGetCompositionStringW(imc: HIMC; dwIndex: DWORD;
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall; external Imm name 'ImmGetCompositionStringW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmSetCompositionStringA(imc: HIMC; dwIndex: DWORD; lpComp: LPVOID;
    dwCompLen: DWORD; lpRead: LPVOID; dwReadLen: DWORD): LongBool; stdcall; external Imm name 'ImmSetCompositionStringA';
function ImmSetCompositionStringW(imc: HIMC; dwIndex: DWORD; lpComp: LPVOID;
    dwCompLen: DWORD; lpRead: LPVOID; dwReadLen: DWORD): LongBool; stdcall; external Imm name 'ImmSetCompositionStringW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef UNICODE}
function ImmSetCompositionString(imc: HIMC; dwIndex: DWORD; lpComp: LPVOID;
    dwCompLen: DWORD; lpRead: LPVOID; dwReadLen: DWORD): LongBool; stdcall; external Imm name 'ImmSetCompositionStringA';
{$else}
function ImmSetCompositionString(imc: HIMC; dwIndex: DWORD; lpComp: LPVOID;
    dwCompLen: DWORD; lpRead: LPVOID; dwReadLen: DWORD): LongBool; stdcall; external Imm name 'ImmSetCompositionStringW';
{$endif}

function ImmGetCandidateListCountA(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountA';
function ImmGetCandidateListCountW(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountW';

{$ifndef UNICODE}
function ImmGetCandidateListCount(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountA';
{$else}
function ImmGetCandidateListCount(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountW';
{$endif}

=======
function ImmGetCandidateListCountA(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountA';
function ImmGetCandidateListCountW(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountW';

>>>>>>> graemeg/cpstrnew
=======
function ImmGetCandidateListCountA(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountA';
function ImmGetCandidateListCountW(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountW';

>>>>>>> graemeg/cpstrnew
=======
function ImmGetCandidateListCountA(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountA';
function ImmGetCandidateListCountW(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountW';

>>>>>>> graemeg/cpstrnew
=======
function ImmGetCandidateListCountA(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountA';
function ImmGetCandidateListCountW(imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListCountW';

>>>>>>> origin/cpstrnew
function ImmGetCandidateListA(imc: HIMC; deIndex: DWORD;
    lpCandList: LPCANDIDATELIST; dwBufLen: DWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListA';
function ImmGetCandidateListW(imc: HIMC; deIndex: DWORD;
    lpCandList: LPCANDIDATELIST; dwBufLen: DWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef UNICODE}
function ImmGetCandidateList(imc: HIMC; deIndex: DWORD;
    lpCandList: LPCANDIDATELIST; dwBufLen: DWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListA';
{$else}
function ImmGetCandidateList(imc: HIMC; deIndex: DWORD;
    lpCandList: LPCANDIDATELIST; dwBufLen: DWORD): DWORD; stdcall; external Imm name 'ImmGetCandidateListW';
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmGetGuideLineA(imc: HIMC; dwIndex: DWORD; lpBuf: LPSTR;
    dwBufLen: DWORD): DWORD; stdcall ; external Imm name 'ImmGetGuideLineA';
function ImmGetGuideLineW(imc: HIMC; dwIndex: DWORD; lpBuf: LPWSTR;
    dwBufLen: DWORD): DWORD; stdcall ; external Imm name 'ImmGetGuideLineW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef UNICODE}
function ImmGetGuideLine(imc: HIMC; dwIndex: DWORD; lpBuf: LPSTR;
    dwBufLen: DWORD): DWORD; stdcall ; external Imm name 'ImmGetGuideLineA';
{$else}
function ImmGetGuideLine(imc: HIMC; dwIndex: DWORD; lpBuf: LPWSTR;
    dwBufLen: DWORD): DWORD; stdcall ; external Imm name 'ImmGetGuideLineW';
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmGetConversionStatus(imc: HIMC; lpfdwConversion, lpfdwSentence: LPDWORD): LongBool; stdcall ; external Imm name 'ImmGetConversionStatus';
function ImmSetConversionStatus(imc: HIMC; fdwConversion, fdwSentence: DWORD): LongBool; stdcall ; external Imm name 'ImmSetConversionStatus';
function ImmGetOpenStatus(imc: HIMC): LongBool; stdcall ; external Imm name 'ImmGetOpenStatus';
function ImmSetOpenStatus(imc: HIMC; par1: LongBool): LongBool; stdcall ; external Imm name 'ImmSetOpenStatus';

function ImmGetCompositionFontA(imc: HIMC; lplf: PLOGFONTA): LongBool; stdcall ; external Imm name 'ImmGetCompositionFontA';
function ImmGetCompositionFontW(imc: HIMC; lplf: PLOGFONTW): LongBool; stdcall ; external Imm name 'ImmGetCompositionFontW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef UNICODE}
function ImmGetCompositionFont(imc: HIMC; lplf: PLOGFONTA): LongBool; stdcall ; external Imm name 'ImmGetCompositionFontA';
{$else}
function ImmGetCompositionFont(imc: HIMC; lplf: PLOGFONTW): LongBool; stdcall ; external Imm name 'ImmGetCompositionFontW';
{$endif}

function ImmSetCompositionFontA(imc: HIMC; lplf: PLOGFONTA): LongBool; stdcall ; external Imm name 'ImmSetCompositionFontA';
function ImmSetCompositionFontW(imc: HIMC; lplf: PLOGFONTW): LongBool; stdcall ; external Imm name 'ImmSetCompositionFontW';

{$ifndef UNICODE}
function ImmSetCompositionFont(imc: HIMC; lplf: PLOGFONTA): LongBool; stdcall ; external Imm name 'ImmSetCompositionFontA';
{$else}
function ImmSetCompositionFont(imc: HIMC; lplf: PLOGFONTW): LongBool; stdcall ; external Imm name 'ImmSetCompositionFontW';
{$endif}

function ImmConfigureIMEA(kl: HKL; wnd: HWND; dwMode: DWORD; lpData: LPVOID): LongBool; stdcall ; external Imm name 'ImmConfigureIMEA';
function ImmConfigureIMEW(kl: HKL; wnd: HWND; dwMode: DWORD; lpData: LPVOID): LongBool; stdcall ; external Imm name 'ImmConfigureIMEW';

{$ifndef UNICODE}
function ImmConfigureIME(kl: HKL; wnd: HWND; dwMode: DWORD; lpData: LPVOID): LongBool; stdcall ; external Imm name 'ImmConfigureIMEA';
{$else}
function ImmConfigureIME(kl: HKL; wnd: HWND; dwMode: DWORD; lpData: LPVOID): LongBool; stdcall ; external Imm name 'ImmConfigureIMEW';
{$endif}

function ImmEscapeA(kl: HKL; imc: HIMC; uEscape: UINT; lpData: LPVOID): LRESULT; stdcall ; external Imm name 'ImmEscapeA';
function ImmEscapeW(kl: HKL; imc: HIMC; uEscape: UINT; lpData: LPVOID): LRESULT; stdcall ; external Imm name 'ImmEscapeW';

{$ifndef UNICODE}
function ImmEscape(kl: HKL; imc: HIMC; uEscape: UINT; lpData: LPVOID): LRESULT; stdcall ; external Imm name 'ImmEscapeA';
{$else}
function ImmEscape(kl: HKL; imc: HIMC; uEscape: UINT; lpData: LPVOID): LRESULT; stdcall ; external Imm name 'ImmEscapeW';
{$endif}

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmSetCompositionFontA(imc: HIMC; lplf: PLOGFONTA): LongBool; stdcall ; external Imm name 'ImmSetCompositionFontA';
function ImmSetCompositionFontW(imc: HIMC; lplf: PLOGFONTW): LongBool; stdcall ; external Imm name 'ImmSetCompositionFontW';

function ImmConfigureIMEA(kl: HKL; wnd: HWND; dwMode: DWORD; lpData: LPVOID): LongBool; stdcall ; external Imm name 'ImmConfigureIMEA';
function ImmConfigureIMEW(kl: HKL; wnd: HWND; dwMode: DWORD; lpData: LPVOID): LongBool; stdcall ; external Imm name 'ImmConfigureIMEW';

function ImmEscapeA(kl: HKL; imc: HIMC; uEscape: UINT; lpData: LPVOID): LRESULT; stdcall ; external Imm name 'ImmEscapeA';
function ImmEscapeW(kl: HKL; imc: HIMC; uEscape: UINT; lpData: LPVOID): LRESULT; stdcall ; external Imm name 'ImmEscapeW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmGetConversionListA(kl: HKL; imc: HIMC; lpSrc: LPCSTR;
    lpDst: LPCANDIDATELIST; dwBufLen: DWORD; uFlag: UINT): DWORD; stdcall ; external Imm name 'ImmGetConversionListA';
function ImmGetConversionListW(kl: HKL; imc: HIMC; lpSrc: LPCWSTR;
    lpDst: LPCANDIDATELIST; dwBufLen: DWORD; uFlag: UINT): DWORD; stdcall ; external Imm name 'ImmGetConversionListW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef UNICODE}
function ImmGetConversionList(kl: HKL; imc: HIMC; lpSrc: LPCSTR;
    lpDst: LPCANDIDATELIST; dwBufLen: DWORD; uFlag: UINT): DWORD; stdcall ; external Imm name 'ImmGetConversionListA';
{$else}
function ImmGetConversionList(kl: HKL; imc: HIMC; lpSrc: LPCWSTR;
    lpDst: LPCANDIDATELIST; dwBufLen: DWORD; uFlag: UINT): DWORD; stdcall ; external Imm name 'ImmGetConversionListW';
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmNotifyIME(imc: HIMC; dwAction, dwIndex, dwValue: DWORD): LongBool; stdcall ; external Imm name 'ImmNotifyIME';
function ImmGetStatusWindowPos(imc: HIMC; lpptPos: LPPOINT): LongBool; stdcall ; external Imm name 'ImmGetStatusWindowPos';
function ImmSetStatusWindowPos(imc: HIMC; lpptPos: LPPOINT): LongBool; stdcall ; external Imm name 'ImmSetStatusWindowPos';
function ImmGetCompositionWindow(imc: HIMC; lpCompForm: LPCOMPOSITIONFORM): LongBool; stdcall ; external Imm name 'ImmGetCompositionWindow';
function ImmSetCompositionWindow(imc: HIMC; lpCompForm: LPCOMPOSITIONFORM): LongBool; stdcall ; external Imm name 'ImmSetCompositionWindow';
function ImmGetCandidateWindow(imc: HIMC; par1: DWORD; lpCandidate: LPCANDIDATEFORM): LongBool; stdcall ; external Imm name 'ImmGetCandidateWindow(';
function ImmSetCandidateWindow(imc: HIMC; lpCandidate: LPCANDIDATEFORM): LongBool; stdcall ; external Imm name 'ImmSetCandidateWindow';

function ImmIsUIMessageA(wnd: HWND; msg: UINT; wPar: WPARAM; lPar: LPARAM): LongBool; stdcall ; external Imm name 'ImmIsUIMessageA';
function ImmIsUIMessageW(wnd: HWND; msg: UINT; wPar: WPARAM; lPar: LPARAM): LongBool; stdcall ; external Imm name 'ImmIsUIMessageW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef UNICODE}
function ImmIsUIMessage(wnd: HWND; msg: UINT; wPar: WPARAM; lPar: LPARAM): LongBool; stdcall ; external Imm name 'ImmIsUIMessageA';
{$else}
function ImmIsUIMessage(wnd: HWND; msg: UINT; wPar: WPARAM; lPar: LPARAM): LongBool; stdcall ; external Imm name 'ImmIsUIMessageW';
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmGetVirtualKey(wnd: HWND): UINT; stdcall ; external Imm name 'ImmGetVirtualKey';

type
  REGISTERWORDENUMPROCA = function(lpszReading: LPCSTR; par1: DWORD;
    lpszString: LPCSTR; par3: LPVOID): Integer; stdcall;
  TREGISTERWORDENUMPROCA = REGISTERWORDENUMPROCA;

  REGISTERWORDENUMPROCW = function(lpszReading: LPCWSTR; par1: DWORD;
    lpszString: LPCWSTR; par3: LPVOID): Integer; stdcall;
  TREGISTERWORDENUMPROCW = REGISTERWORDENUMPROCW;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef UNICODE}
    TREGISTERWORDENUMPROC = REGISTERWORDENUMPROCW;
    REGISTERWORDENUMPROC  = REGISTERWORDENUMPROCW;
  {$else}
    TREGISTERWORDENUMPROC = REGISTERWORDENUMPROCA;
    REGISTERWORDENUMPROC  = REGISTERWORDENUMPROCA;
  {$endif}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

function ImmRegisterWordA(kl: HKL; lpszReading: LPCSTR; dwStyle: DWORD; lpszRegister: LPCSTR): LongBool; stdcall ; external Imm name 'ImmRegisterWordA';
function ImmRegisterWordW(kl: HKL; lpszReading: LPCWSTR; dwStyle: DWORD; lpszRegister: LPCWSTR): LongBool; stdcall ; external Imm name 'ImmRegisterWordW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifndef UNICODE}
function ImmRegisterWord(kl: HKL; lpszReading: LPCSTR; dwStyle: DWORD; lpszRegister: LPCSTR): LongBool; stdcall ; external Imm name 'ImmRegisterWordA';
{$else}
function ImmRegisterWord(kl: HKL; lpszReading: LPCWSTR; dwStyle: DWORD; lpszRegister: LPCWSTR): LongBool; stdcall ; external Imm name 'ImmRegisterWordW';
{$endif}

function ImmUnregisterWordA(kl: HKL; lpszReading: LPCSTR; dwStyle: DWORD; lpszUnregister: LPCSTR): LongBool; stdcall ; external Imm name 'ImmUnregisterWordA';
function ImmUnregisterWordW(kl: HKL; lpszReading: LPCWSTR; dwStyle: DWORD; lpszUnregister: LPCWSTR): LongBool; stdcall ; external Imm name 'ImmUnregisterWordW';

{$ifndef UNICODE}
function ImmUnregisterWord(kl: HKL; lpszReading: LPCSTR; dwStyle: DWORD; lpszUnregister: LPCSTR): LongBool; stdcall ; external Imm name 'ImmUnregisterWordA';
{$else}
function ImmUnregisterWord(kl: HKL; lpszReading: LPCWSTR; dwStyle: DWORD; lpszUnregister: LPCWSTR): LongBool; stdcall ; external Imm name 'ImmUnregisterWordW';
{$endif}

function ImmGetRegisterWordStyleA(kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFA): UINT; stdcall ; external Imm name 'ImmGetRegisterWordStyleA';
function ImmGetRegisterWordStyleW(kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFW): UINT; stdcall ; external Imm name 'ImmGetRegisterWordStyleW';

{$ifndef UNICODE}
function ImmGetRegisterWordStyle(kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFA): UINT; stdcall ; external Imm name 'ImmGetRegisterWordStyleA';
{$else}
function ImmGetRegisterWordStyle(kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFW): UINT; stdcall ; external Imm name 'ImmGetRegisterWordStyleW';
{$endif}

=======
=======
>>>>>>> origin/cpstrnew
function ImmUnregisterWordA(kl: HKL; lpszReading: LPCSTR; dwStyle: DWORD; lpszUnregister: LPCSTR): LongBool; stdcall ; external Imm name 'ImmUnregisterWordA';
function ImmUnregisterWordW(kl: HKL; lpszReading: LPCWSTR; dwStyle: DWORD; lpszUnregister: LPCWSTR): LongBool; stdcall ; external Imm name 'ImmUnregisterWordW';

function ImmGetRegisterWordStyleA(kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFA): UINT; stdcall ; external Imm name 'ImmGetRegisterWordStyleA';
function ImmGetRegisterWordStyleW(kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFW): UINT; stdcall ; external Imm name 'ImmGetRegisterWordStyleW';

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
function ImmUnregisterWordA(kl: HKL; lpszReading: LPCSTR; dwStyle: DWORD; lpszUnregister: LPCSTR): LongBool; stdcall ; external Imm name 'ImmUnregisterWordA';
function ImmUnregisterWordW(kl: HKL; lpszReading: LPCWSTR; dwStyle: DWORD; lpszUnregister: LPCWSTR): LongBool; stdcall ; external Imm name 'ImmUnregisterWordW';

function ImmGetRegisterWordStyleA(kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFA): UINT; stdcall ; external Imm name 'ImmGetRegisterWordStyleA';
function ImmGetRegisterWordStyleW(kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFW): UINT; stdcall ; external Imm name 'ImmGetRegisterWordStyleW';

>>>>>>> graemeg/cpstrnew
=======
function ImmUnregisterWordA(kl: HKL; lpszReading: LPCSTR; dwStyle: DWORD; lpszUnregister: LPCSTR): LongBool; stdcall ; external Imm name 'ImmUnregisterWordA';
function ImmUnregisterWordW(kl: HKL; lpszReading: LPCWSTR; dwStyle: DWORD; lpszUnregister: LPCWSTR): LongBool; stdcall ; external Imm name 'ImmUnregisterWordW';

function ImmGetRegisterWordStyleA(kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFA): UINT; stdcall ; external Imm name 'ImmGetRegisterWordStyleA';
function ImmGetRegisterWordStyleW(kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFW): UINT; stdcall ; external Imm name 'ImmGetRegisterWordStyleW';

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmEnumRegisterWordA(kl: HKL; lpfnEnumProc: REGISTERWORDENUMPROCA;
    lpszReading: LPCSTR; dwStyle: DWORD; lpszRegister: LPCSTR; lpData: LPVOID): UINT; stdcall ; external Imm name 'ImmEnumRegisterWordA';
function ImmEnumRegisterWordW(kl: HKL; lpfnEnumProc: REGISTERWORDENUMPROCW;
    lpszReading: LPCWSTR; dwStyle: DWORD; lpszRegister: LPCWSTR; lpData: LPVOID): UINT; stdcall ; external Imm name 'ImmEnumRegisterWordW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef UNICODE}
function ImmEnumRegisterWord(kl: HKL; lpfnEnumProc: REGISTERWORDENUMPROCA;
    lpszReading: LPCSTR; dwStyle: DWORD; lpszRegister: LPCSTR; lpData: LPVOID): UINT; stdcall ; external Imm name 'ImmEnumRegisterWordA';
{$else}
function ImmEnumRegisterWord(kl: HKL; lpfnEnumProc: REGISTERWORDENUMPROCW;
    lpszReading: LPCWSTR; dwStyle: DWORD; lpszRegister: LPCWSTR; lpData: LPVOID): UINT; stdcall ; external Imm name 'ImmEnumRegisterWordW';
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmDisableIME(idThread: DWORD): LongBool; stdcall  ; external Imm name 'ImmDisableIME';
function ImmEnumInputContext(idThread: DWORD; lpfn: IMCENUMPROC; lParam: LPARAM): LongBool; stdcall ; external Imm name 'ImmEnumInputContext';
function ImmGetImeMenuItemsA(imc: HIMC; dwFlags, dwType: DWORD;
    lpImeParentMenu, lpImeMenu: LPIMEMENUITEMINFOA; dwSize: DWORD): DWORD; stdcall ; external Imm name 'ImmGetImeMenuItemsA';
function ImmGetImeMenuItemsW(par0: HIMC; dwFlags, dwType: DWORD;
    lpImeParentMenu, lpImeMenu: LPIMEMENUITEMINFOW; dwSize: DWORD): DWORD; stdcall ; external Imm name 'ImmGetImeMenuItemsW';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef UNICODE}
function ImmGetImeMenuItems(imc: HIMC; dwFlags, dwType: DWORD;
    lpImeParentMenu, lpImeMenu: LPIMEMENUITEMINFOA; dwSize: DWORD): DWORD; stdcall ; external Imm name 'ImmGetImeMenuItemsA';
{$else}
function ImmGetImeMenuItems(par0: HIMC; dwFlags, dwType: DWORD;
    lpImeParentMenu, lpImeMenu: LPIMEMENUITEMINFOW; dwSize: DWORD): DWORD; stdcall ; external Imm name 'ImmGetImeMenuItemsW';
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ImmDisableTextFrameService(idThread: DWORD): LongBool; stdcall ; external Imm name 'ImmDisableTextFrameService';

// wParam for WM_IME_CONTROL
const
  IMC_GETCANDIDATEPOS      = $0007;
  IMC_SETCANDIDATEPOS      = $0008;
  IMC_GETCOMPOSITIONFONT   = $0009;
  IMC_SETCOMPOSITIONFONT   = $000A;
  IMC_GETCOMPOSITIONWINDOW = $000B;
  IMC_SETCOMPOSITIONWINDOW = $000C;
  IMC_GETSTATUSWINDOWPOS   = $000F;
  IMC_SETSTATUSWINDOWPOS   = $0010;
  IMC_CLOSESTATUSWINDOW    = $0021;
  IMC_OPENSTATUSWINDOW     = $0022;

  // dwAction for ImmNotifyIME
  NI_OPENCANDIDATE            = $0010;
  NI_CLOSECANDIDATE           = $0011;
  NI_SELECTCANDIDATESTR       = $0012;
  NI_CHANGECANDIDATELIST      = $0013;
  NI_FINALIZECONVERSIONRESULT = $0014;
  NI_COMPOSITIONSTR           = $0015;
  NI_SETCANDIDATE_PAGESTART   = $0016;
  NI_SETCANDIDATE_PAGESIZE    = $0017;
  NI_IMEMENUSELECTED          = $0018;

  // lParam for WM_IME_SETCONTEXT
  ISC_SHOWUICANDIDATEWINDOW    = $00000001;
  ISC_SHOWUICOMPOSITIONWINDOW  = $80000000;
  ISC_SHOWUIGUIDELINE          = $40000000;
  ISC_SHOWUIALLCANDIDATEWINDOW = $0000000F;
  ISC_SHOWUIALL                = $C000000F;

  // dwIndex for ImmNotifyIME/NI_COMPOSITIONSTR
  CPS_COMPLETE  = $0001;
  CPS_CONVERT   = $0002;
  CPS_REVERT    = $0003;
  CPS_CANCEL    = $0004;

  // the modifiers of hot key
  MOD_ALT      = $0001;
  MOD_CONTROL  = $0002;
  MOD_SHIFT    = $0004;
  MOD_LEFT     = $8000;
  MOD_RIGHT    = $4000;
  MOD_ON_KEYUP = $0800;
  MOD_IGNORE_ALL_MODIFIER = $0400;

  // Windows for Simplified Chinese Edition hot key ID from 0x10 - 0x2F
  IME_CHOTKEY_IME_NONIME_TOGGLE = $10;
  IME_CHOTKEY_SHAPE_TOGGLE      = $11;
  IME_CHOTKEY_SYMBOL_TOGGLE     = $12;

  // Windows for Japanese Edition hot key ID from 0x30 - 0x4F
  IME_JHOTKEY_CLOSE_OPEN    = $30;

  // Windows for Korean Edition hot key ID from 0x50 - 0x6F
  IME_KHOTKEY_SHAPE_TOGGLE  = $50;
  IME_KHOTKEY_HANJACONVERT  = $51;
  IME_KHOTKEY_ENGLISH       = $52;

  // Windows for Traditional Chinese Edition hot key ID from 0x70 - 0x8F
  IME_THOTKEY_IME_NONIME_TOGGLE = $70;
  IME_THOTKEY_SHAPE_TOGGLE      = $71;
  IME_THOTKEY_SYMBOL_TOGGLE     = $72;

  // direct switch hot key ID from 0x100 - 0x11F
  IME_HOTKEY_DSWITCH_FIRST  = $100;
  IME_HOTKEY_DSWITCH_LAST   = $11F;

  // IME private hot key from 0x200 - 0x21F
  IME_HOTKEY_PRIVATE_FIRST          = $200;
  IME_ITHOTKEY_RESEND_RESULTSTR     = $200;
  IME_ITHOTKEY_PREVIOUS_COMPOSITION = $201;
  IME_ITHOTKEY_UISTYLE_TOGGLE       = $202;
  IME_ITHOTKEY_RECONVERTSTRING      = $203;
  IME_HOTKEY_PRIVATE_LAST           = $21F;

  // parameter of ImmGetCompositionString
  GCS_COMPREADSTR       = $0001;
  GCS_COMPREADATTR      = $0002;
  GCS_COMPREADCLAUSE    = $0004;
  GCS_COMPSTR           = $0008;
  GCS_COMPATTR          = $0010;
  GCS_COMPCLAUSE        = $0020;
  GCS_CURSORPOS         = $0080;
  GCS_DELTASTART        = $0100;
  GCS_RESULTREADSTR     = $0200;
  GCS_RESULTREADCLAUSE  = $0400;
  GCS_RESULTSTR         = $0800;
  GCS_RESULTCLAUSE      = $1000;

  // style bit flags for WM_IME_COMPOSITION
  CS_INSERTCHAR   = $2000;
  CS_NOMOVECARET  = $4000;

  // IME version constants
  IMEVER_0310 = $0003000A;
  IMEVER_0400 = $00040000;


  // IME property bits
  IME_PROP_AT_CARET              = $00010000;
  IME_PROP_SPECIAL_UI            = $00020000;
  IME_PROP_CANDLIST_START_FROM_1 = $00040000;
  IME_PROP_UNICODE               = $00080000;
  IME_PROP_COMPLETE_ON_UNSELECT  = $00100000;

  // IME UICapability bits
  UI_CAP_2700   = $00000001;
  UI_CAP_ROT90  = $00000002;
  UI_CAP_ROTANY = $00000004;

  // ImmSetCompositionString Capability bits
  SCS_CAP_COMPSTR            = $00000001;
  SCS_CAP_MAKEREAD           = $00000002;
  SCS_CAP_SETRECONVERTSTRING = $00000004;

  // IME WM_IME_SELECT inheritance Capability bits
  SELECT_CAP_CONVERSION = $00000001;
  SELECT_CAP_SENTENCE   = $00000002;

  // ID for deIndex of ImmGetGuideLine
  GGL_LEVEL   = $00000001;
  GGL_INDEX   = $00000002;
  GGL_STRING  = $00000003;
  GGL_PRIVATE = $00000004;

  // ID for dwLevel of GUIDELINE Structure
  GL_LEVEL_NOGUIDELINE = $00000000;
  GL_LEVEL_FATAL       = $00000001;
  GL_LEVEL_ERROR       = $00000002;
  GL_LEVEL_WARNING     = $00000003;
  GL_LEVEL_INFORMATION = $00000004;

  // ID for dwIndex of GUIDELINE Structure
  GL_ID_UNKNOWN           = $00000000;
  GL_ID_NOMODULE          = $00000001;
  GL_ID_NODICTIONARY      = $00000010;
  GL_ID_CANNOTSAVE        = $00000011;
  GL_ID_NOCONVERT         = $00000020;
  GL_ID_TYPINGERROR       = $00000021;
  GL_ID_TOOMANYSTROKE     = $00000022;
  GL_ID_READINGCONFLICT   = $00000023;
  GL_ID_INPUTREADING      = $00000024;
  GL_ID_INPUTRADICAL      = $00000025;
  GL_ID_INPUTCODE         = $00000026;
  GL_ID_INPUTSYMBOL       = $00000027;
  GL_ID_CHOOSECANDIDATE   = $00000028;
  GL_ID_REVERSECONVERSION = $00000029;
  GL_ID_PRIVATE_FIRST     = $00008000;
  GL_ID_PRIVATE_LAST      = $0000FFFF;

  // ID for dwIndex of ImmGetProperty
  IGP_GETIMEVERSION = -4;
  IGP_PROPERTY      = $00000004;
  IGP_CONVERSION    = $00000008;
  IGP_SENTENCE      = $0000000c;
  IGP_UI            = $00000010;
  IGP_SETCOMPSTR    = $00000014;
  IGP_SELECT        = $00000018;

  // dwIndex for ImmSetCompositionString API
  SCS_SETSTR               = (GCS_COMPREADSTR or GCS_COMPSTR);
  SCS_CHANGEATTR           = (GCS_COMPREADATTR or GCS_COMPATTR);
  SCS_CHANGECLAUSE         = (GCS_COMPREADCLAUSE or GCS_COMPCLAUSE);
  SCS_SETRECONVERTSTRING   = $00010000;
  SCS_QUERYRECONVERTSTRING = $00020000;

  // attribute for COMPOSITIONSTRING Structure
  ATTR_INPUT               = $00;
  ATTR_TARGET_CONVERTED    = $01;
  ATTR_CONVERTED           = $02;
  ATTR_TARGET_NOTCONVERTED = $03;
  ATTR_INPUT_ERROR         = $04;
  ATTR_FIXEDCONVERTED      = $05;

  // bit field for IMC_SETCOMPOSITIONWINDOW, IMC_SETCANDIDATEWINDOW
  CFS_DEFAULT        = $0000;
  CFS_RECT           = $0001;
  CFS_POINT          = $0002;
  CFS_FORCE_POSITION = $0020;
  CFS_CANDIDATEPOS   = $0040;
  CFS_EXCLUDE        = $0080;

  // conversion direction for ImmGetConversionList
  GCL_CONVERSION        = $0001;
  GCL_REVERSECONVERSION = $0002;
  GCL_REVERSE_LENGTH    = $0003;

  // bit field for conversion mode
  // IME_CMODE_HANGEUL is old name of IME_CMODE_HANGUL. It will be gone eventually.
  IME_CMODE_HANGEUL      = IME_CMODE_NATIVE;
  IME_CMODE_SOFTKBD      = $0080;
  IME_CMODE_NOCONVERSION = $0100;
  IME_CMODE_EUDC         = $0200;
  IME_CMODE_SYMBOL       = $0400;
  IME_CMODE_FIXED        = $0800;
  IME_CMODE_RESERVED     = $F0000000;

  // bit field for sentence mode
  IME_SMODE_NONE          = $0000;
  IME_SMODE_PLAURALCLAUSE = $0001;
  IME_SMODE_SINGLECONVERT = $0002;
  IME_SMODE_AUTOMATIC     = $0004;
  IME_SMODE_PHRASEPREDICT = $0008;
  IME_SMODE_CONVERSATION  = $0010;
  IME_SMODE_RESERVED      = $0000F000;

  // style of candidate
  IME_CAND_UNKNOWN = $0000;
  IME_CAND_READ    = $0001;
  IME_CAND_CODE    = $0002;
  IME_CAND_MEANING = $0003;
  IME_CAND_RADICAL = $0004;
  IME_CAND_STROKE  = $0005;

  // wParam of report message WM_IME_NOTIFY
  IMN_CLOSESTATUSWINDOW    = $0001;
  IMN_OPENSTATUSWINDOW     = $0002;
  IMN_CHANGECANDIDATE      = $0003;
  IMN_CLOSECANDIDATE       = $0004;
  IMN_OPENCANDIDATE        = $0005;
  IMN_SETCONVERSIONMODE    = $0006;
  IMN_SETSENTENCEMODE      = $0007;
  IMN_SETOPENSTATUS        = $0008;
  IMN_SETCANDIDATEPOS      = $0009;
  IMN_SETCOMPOSITIONFONT   = $000A;
  IMN_SETCOMPOSITIONWINDOW = $000B;
  IMN_SETSTATUSWINDOWPOS   = $000C;
  IMN_GUIDELINE            = $000D;
  IMN_PRIVATE              = $000E;

  // wParam of report message WM_IME_REQUEST
  IMR_COMPOSITIONWINDOW      = $0001;
  IMR_CANDIDATEWINDOW        = $0002;
  IMR_COMPOSITIONFONT        = $0003;
  IMR_RECONVERTSTRING        = $0004;
  IMR_CONFIRMRECONVERTSTRING = $0005;
  IMR_QUERYCHARPOSITION      = $0006;
  IMR_DOCUMENTFEED           = $0007;

  // error code of ImmGetCompositionString
  IMM_ERROR_NODATA  = -1;
  IMM_ERROR_GENERAL = -2;

  // dialog mode of ImmConfigureIME
  IME_CONFIG_GENERAL          = 1;
  IME_CONFIG_REGISTERWORD     = 2;
  IME_CONFIG_SELECTDICTIONARY = 3;

  // flags for ImmEscape
  IME_ESC_QUERY_SUPPORT        = $0003;
  IME_ESC_RESERVED_FIRST       = $0004;
  IME_ESC_RESERVED_LAST        = $07FF;
  IME_ESC_PRIVATE_FIRST        = $0800;
  IME_ESC_PRIVATE_LAST         = $0FFF;
  IME_ESC_SEQUENCE_TO_INTERNAL = $1001;
  IME_ESC_GET_EUDC_DICTIONARY  = $1003;
  IME_ESC_SET_EUDC_DICTIONARY  = $1004;
  IME_ESC_MAX_KEY              = $1005;
  IME_ESC_IME_NAME             = $1006;
  IME_ESC_SYNC_HOTKEY          = $1007;
  IME_ESC_HANJA_MODE           = $1008;
  IME_ESC_AUTOMATA             = $1009;
  IME_ESC_PRIVATE_HOTKEY       = $100a;
  IME_ESC_GETHELPFILENAME      = $100b;

  // style of word registration
  IME_REGWORD_STYLE_EUDC       = $00000001;
  IME_REGWORD_STYLE_USER_FIRST = $80000000;
  IME_REGWORD_STYLE_USER_LAST  = $FFFFFFFF;

  // dwFlags for ImmAssociateContextEx
  IACE_CHILDREN        = $0001;
  IACE_DEFAULT         = $0010;
  IACE_IGNORENOCONTEXT = $0020;

  // dwFlags for ImmGetImeMenuItems
  IGIMIF_RIGHTMENU = $0001;

  // dwType for ImmGetImeMenuItems
  IGIMII_CMODE      = $0001;
  IGIMII_SMODE      = $0002;
  IGIMII_CONFIGURE  = $0004;
  IGIMII_TOOLS      = $0008;
  IGIMII_HELP       = $0010;
  IGIMII_OTHER      = $0020;
  IGIMII_INPUTTOOLS = $0040;

  // fType of IMEMENUITEMINFO structure
  IMFT_RADIOCHECK = $00001;
  IMFT_SEPARATOR  = $00002;
  IMFT_SUBMENU    = $00004;

  // fState of IMEMENUITEMINFO structure
  IMFS_GRAYED    = MFS_GRAYED;
  IMFS_DISABLED  = MFS_DISABLED;
  IMFS_CHECKED   = MFS_CHECKED;
  IMFS_HILITE    = MFS_HILITE;
  IMFS_ENABLED   = MFS_ENABLED;
  IMFS_UNCHECKED = MFS_UNCHECKED;
  IMFS_UNHILITE  = MFS_UNHILITE;
  IMFS_DEFAULT   = MFS_DEFAULT;

  // type of soft keyboard
  // for Windows Tranditional Chinese Edition
  SOFTKEYBOARD_TYPE_T1 = $0001;

  // for Windows Simplified Chinese Edition
  SOFTKEYBOARD_TYPE_C1 = $0002;

implementation


end.
