(********************************************************************* *)
(*      imm.h - Input Method Manager definitions                       *)
(*                                                                     *)
(*      Copyright (c) Microsoft Corporation. All rights reserved.      *)
(*                                                                     *)
(* Converted to pascal by Dmitry Boyarintsev, using Chelper.           *)
(*                                                                     *)
(*   The dynamic interface is used to load function dynamicly.         *)
(* It's users responsibility to check functions have been loaded       *)
(* successfully.                                                       *)
(*   imm_dyn and imm function names match. In oreder to be sure that   *)
(* dynamicly loaded functions are used, always add imm_dyn to the uses *)
(* section after imm unit.                                             *)
(*                                                                     *)
(********************************************************************* *)
unit imm_dyn;

{$mode delphi}

interface

uses
  Windows, imm;

var
  ImmInstallIMEA : function (lpszIMEFileName, lpszLayoutText: LPCSTR): HKL; stdcall = nil;
  ImmInstallIMEW : function (lpszIMEFileName, lpszLayoutText: LPCWSTR): HKL; stdcall = nil;
  ImmGetDefaultIMEWnd : function (wnd: HWND): HWND; stdcall = nil;
  ImmGetDescriptionA : function (kl: HKL; lpszDescription: LPSTR; uBufLen: UINT): UINT; stdcall = nil;
  ImmGetDescriptionW : function (kl: HKL; lpszDescription: LPWSTR; uBufLen: UINT): UINT; stdcall = nil;

  ImmGetIMEFileNameA : function (kl: HKL; lpszFileName: LPSTR; uBufLen: UINT): UINT; stdcall = nil;
  ImmGetIMEFileNameW : function (kl: HKL; lpszFileName: LPWSTR; uBufLen: UINT): UINT; stdcall = nil;

  ImmGetProperty : function (kl: HKL; fdwIndex: DWORD): DWORD; stdcall = nil;
  ImmIsIME : function (kl: HKL): LongBool; stdcall = nil;
  ImmSimulateHotKey : function (wnd: HWND; dwHotKeyID: DWORD): LongBool; stdcall = nil;

  ImmCreateContext: function : HIMC; stdcall = nil;
  ImmDestroyContext: function (imc: HIMC): LongBool; stdcall = nil;
  ImmGetContext: function (wnd: HWND): HIMC; stdcall = nil;

  ImmReleaseContext: function (wnd: HWND; imc: HIMC): LongBool; stdcall = nil;
  ImmAssociateContext: function (wnd: HWND; imc: HIMC): HIMC; stdcall = nil;

  ImmAssociateContextEx: function (wnd: HWND; imc: HIMC; dwFlags: DWORD): LongBool; stdcall = nil;
  ImmGetCompositionStringA: function (imc: HIMC; dwIndex: DWORD;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    lpBuf: LPVOID; dwBufLen: DWORD): LONG; stdcall = nil;
  ImmGetCompositionStringW: function (imc: HIMC; dwIndex: DWORD;
    lpBuf: LPVOID; dwBufLen: DWORD): LONG; stdcall = nil;
=======
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall = nil;
  ImmGetCompositionStringW: function (imc: HIMC; dwIndex: DWORD;
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall = nil;
>>>>>>> graemeg/cpstrnew
=======
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall = nil;
  ImmGetCompositionStringW: function (imc: HIMC; dwIndex: DWORD;
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall = nil;
>>>>>>> graemeg/cpstrnew
=======
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall = nil;
  ImmGetCompositionStringW: function (imc: HIMC; dwIndex: DWORD;
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall = nil;
>>>>>>> graemeg/cpstrnew
=======
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall = nil;
  ImmGetCompositionStringW: function (imc: HIMC; dwIndex: DWORD;
    lpBuf: LPVOID; dwBufLen: DWORD): Longword; stdcall = nil;
>>>>>>> origin/cpstrnew

  ImmSetCompositionStringA: function (imc: HIMC; dwIndex: DWORD; lpComp: LPVOID;
    dwCompLen: DWORD; lpRead: LPVOID; dwReadLen: DWORD): LongBool; stdcall = nil;
  ImmSetCompositionStringW: function (imc: HIMC; dwIndex: DWORD; lpComp: LPVOID;
    dwCompLen: DWORD; lpRead: LPVOID; dwReadLen: DWORD): LongBool; stdcall = nil;

  ImmGetCandidateListCountA: function (imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall = nil;
  ImmGetCandidateListCountW: function (imc: HIMC; lpdwListCount: LPDWORD): DWORD; stdcall = nil;

  ImmGetCandidateListA: function (imc: HIMC; deIndex: DWORD;
    lpCandList: LPCANDIDATELIST; dwBufLen: DWORD): DWORD; stdcall = nil;
  ImmGetCandidateListW: function (imc: HIMC; deIndex: DWORD;
    lpCandList: LPCANDIDATELIST; dwBufLen: DWORD): DWORD; stdcall = nil;

  ImmGetGuideLineA: function (imc: HIMC; dwIndex: DWORD; lpBuf: LPSTR;
    dwBufLen: DWORD): DWORD; stdcall = nil;
  ImmGetGuideLineW: function (imc: HIMC; dwIndex: DWORD; lpBuf: LPWSTR;
    dwBufLen: DWORD): DWORD; stdcall = nil;

  ImmGetConversionStatus: function (imc: HIMC; lpfdwConversion, lpfdwSentence: LPDWORD): LongBool; stdcall = nil;
  ImmSetConversionStatus: function (imc: HIMC; fdwConversion, fdwSentence: DWORD): LongBool; stdcall = nil;
  ImmGetOpenStatus: function (imc: HIMC): LongBool; stdcall = nil;
  ImmSetOpenStatus: function (imc: HIMC; par1: LongBool): LongBool; stdcall = nil;

  ImmGetCompositionFontA: function (imc: HIMC; lplf: PLOGFONTA): LongBool; stdcall = nil;
  ImmGetCompositionFontW: function (imc: HIMC; lplf: PLOGFONTW): LongBool; stdcall = nil;

  ImmSetCompositionFontA: function (imc: HIMC; lplf: PLOGFONTA): LongBool; stdcall = nil;
  ImmSetCompositionFontW: function (imc: HIMC; lplf: PLOGFONTW): LongBool; stdcall = nil;

  ImmConfigureIMEA: function (kl: HKL; wnd: HWND; dwMode: DWORD; lpData: LPVOID): LongBool; stdcall = nil;
  ImmConfigureIMEW: function (kl: HKL; wnd: HWND; dwMode: DWORD; lpData: LPVOID): LongBool; stdcall = nil;

  ImmEscapeA: function (kl: HKL; imc: HIMC; uEscape: UINT; lpData: LPVOID): LRESULT; stdcall = nil;
  ImmEscapeW: function (kl: HKL; imc: HIMC; uEscape: UINT; lpData: LPVOID): LRESULT; stdcall = nil;

  ImmGetConversionListA: function (kl: HKL; imc: HIMC; lpSrc: LPCSTR;
    lpDst: LPCANDIDATELIST; dwBufLen: DWORD; uFlag: UINT): DWORD; stdcall = nil;
  ImmGetConversionListW: function (kl: HKL; imc: HIMC; lpSrc: LPCWSTR;
    lpDst: LPCANDIDATELIST; dwBufLen: DWORD; uFlag: UINT): DWORD; stdcall = nil;

  ImmNotifyIME: function (imc: HIMC; dwAction, dwIndex, dwValue: DWORD): LongBool; stdcall = nil;
  ImmGetStatusWindowPos: function (imc: HIMC; lpptPos: LPPOINT): LongBool; stdcall = nil;
  ImmSetStatusWindowPos: function (imc: HIMC; lpptPos: LPPOINT): LongBool; stdcall = nil;
  ImmGetCompositionWindow: function (imc: HIMC; lpCompForm: LPCOMPOSITIONFORM): LongBool; stdcall = nil;
  ImmSetCompositionWindow: function (imc: HIMC; lpCompForm: LPCOMPOSITIONFORM): LongBool; stdcall = nil;
  ImmGetCandidateWindow: function (imc: HIMC; par1: DWORD; lpCandidate: LPCANDIDATEFORM): LongBool; stdcall = nil;
  ImmSetCandidateWindow: function (imc: HIMC; lpCandidate: LPCANDIDATEFORM): LongBool; stdcall = nil;

  ImmIsUIMessageA: function (wnd: HWND; msg: UINT; wPar: WPARAM; lPar: LPARAM): LongBool; stdcall = nil;
  ImmIsUIMessageW: function (wnd: HWND; msg: UINT; wPar: WPARAM; lPar: LPARAM): LongBool; stdcall = nil;

  ImmGetVirtualKey: function (wnd: HWND): UINT; stdcall = nil;

  ImmRegisterWordA: function (kl: HKL; lpszReading: LPCSTR; dwStyle: DWORD; lpszRegister: LPCSTR): LongBool; stdcall = nil;
  ImmRegisterWordW: function (kl: HKL; lpszReading: LPCWSTR; dwStyle: DWORD; lpszRegister: LPCWSTR): LongBool; stdcall = nil;

  ImmUnregisterWordA: function (kl: HKL; lpszReading: LPCSTR; dwStyle: DWORD; lpszUnregister: LPCSTR): LongBool; stdcall = nil;

  ImmUnregisterWordW: function (kl: HKL; lpszReading: LPCWSTR; dwStyle: DWORD; lpszUnregister: LPCWSTR): LongBool; stdcall = nil;

  ImmGetRegisterWordStyleA: function (kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFA): UINT; stdcall = nil;
  ImmGetRegisterWordStyleW: function (kl: HKL; nItem: UINT; lpStyleBuf: LPSTYLEBUFW): UINT; stdcall = nil;

  ImmEnumRegisterWordA: function (kl: HKL; lpfnEnumProc: REGISTERWORDENUMPROCA;
    lpszReading: LPCSTR; dwStyle: DWORD; lpszRegister: LPCSTR; lpData: LPVOID): UINT; stdcall = nil;
  ImmEnumRegisterWordW: function (kl: HKL; lpfnEnumProc: REGISTERWORDENUMPROCW;
    lpszReading: LPCWSTR; dwStyle: DWORD; lpszRegister: LPCWSTR; lpData: LPVOID): UINT; stdcall = nil;

  ImmDisableIME: function (idThread: DWORD): LongBool; stdcall  = nil;
  ImmEnumInputContext: function (idThread: DWORD; lpfn: IMCENUMPROC; lParam: LPARAM)
    : LongBool; stdcall = nil;
  ImmGetImeMenuItemsA: function (imc: HIMC; dwFlags, dwType: DWORD;
    lpImeParentMenu, lpImeMenu: LPIMEMENUITEMINFOA; dwSize: DWORD): DWORD; stdcall = nil;
  ImmGetImeMenuItemsW: function (par0: HIMC; dwFlags, dwType: DWORD;
    lpImeParentMenu, lpImeMenu: LPIMEMENUITEMINFOW; dwSize: DWORD): DWORD; stdcall = nil;

  ImmDisableTextFrameService: function (idThread: DWORD): LongBool; stdcall = nil;

implementation

var
  lib : THandle=0;

procedure InitImm;
begin
  lib:=LoadLibraryA('Imm32.dll');
  if lib=0 then Exit;
  ImmInstallIMEA              := GetProcAddress(lib,'ImmInstallIMEA');
  ImmInstallIMEW              := GetProcAddress(lib,'ImmInstallIMEW');
  ImmGetDefaultIMEWnd         := GetProcAddress(lib,'ImmGetDefaultIMEWnd');
  ImmGetDescriptionA          := GetProcAddress(lib,'ImmGetDescriptionA');
  ImmGetDescriptionW          := GetProcAddress(lib,'ImmGetDescriptionW');

  ImmGetIMEFileNameA          := GetProcAddress(lib,'ImmGetIMEFileNameA');
  ImmGetIMEFileNameW          := GetProcAddress(lib,'ImmGetIMEFileNameW');

  ImmGetProperty              := GetProcAddress(lib,'ImmGetProperty');
  ImmIsIME                    := GetProcAddress(lib,'ImmIsIME');
  ImmSimulateHotKey           := GetProcAddress(lib,'ImmSimulateHotKey');

  ImmCreateContext            := GetProcAddress(lib,'ImmCreateContext');
  ImmDestroyContext           := GetProcAddress(lib,'ImmDestroyContext');
  ImmGetContext               := GetProcAddress(lib,'ImmGetContext');

  ImmReleaseContext           := GetProcAddress(lib,'ImmReleaseContext');
  ImmAssociateContext         := GetProcAddress(lib,'ImmAssociateContext');

  ImmAssociateContextEx       := GetProcAddress(lib,'ImmAssociateContextEx');
  ImmGetCompositionStringA    := GetProcAddress(lib,'ImmGetCompositionStringA');
  ImmGetCompositionStringW    := GetProcAddress(lib,'ImmGetCompositionStringW');

  ImmSetCompositionStringA    := GetProcAddress(lib,'ImmSetCompositionStringA');
  ImmSetCompositionStringW    := GetProcAddress(lib,'ImmSetCompositionStringW');
  ImmGetCandidateListCountA   := GetProcAddress(lib,'ImmGetCandidateListCountA');
  ImmGetCandidateListCountW   := GetProcAddress(lib,'ImmGetCandidateListCountW');
  ImmGetCandidateListA        := GetProcAddress(lib,'ImmGetCandidateListA');
  ImmGetCandidateListW        := GetProcAddress(lib,'ImmGetCandidateListW');
  ImmGetGuideLineA            := GetProcAddress(lib,'ImmGetGuideLineA');
  ImmGetGuideLineW            := GetProcAddress(lib,'ImmGetGuideLineW');
  ImmGetConversionStatus      := GetProcAddress(lib,'ImmGetConversionStatus');
  ImmSetConversionStatus      := GetProcAddress(lib,'ImmSetConversionStatus');
  ImmGetOpenStatus            := GetProcAddress(lib,'ImmGetOpenStatus');
  ImmSetOpenStatus            := GetProcAddress(lib,'ImmSetOpenStatus');
  ImmGetCompositionFontA      := GetProcAddress(lib,'ImmGetCompositionFontA');
  ImmGetCompositionFontW      := GetProcAddress(lib,'ImmGetCompositionFontW');
  ImmSetCompositionFontA      := GetProcAddress(lib,'ImmSetCompositionFontA');
  ImmSetCompositionFontW      := GetProcAddress(lib,'ImmSetCompositionFontW');
  ImmConfigureIMEA            := GetProcAddress(lib,'ImmConfigureIMEA');
  ImmConfigureIMEW            := GetProcAddress(lib,'ImmConfigureIMEW');
  ImmEscapeA                  := GetProcAddress(lib,'ImmEscapeA');
  ImmEscapeW                  := GetProcAddress(lib,'ImmEscapeW');
  ImmGetConversionListA       := GetProcAddress(lib,'ImmGetConversionListA');
  ImmGetConversionListW       := GetProcAddress(lib,'ImmGetConversionListW');

  ImmNotifyIME                := GetProcAddress(lib,'ImmNotifyIME');
  ImmGetStatusWindowPos       := GetProcAddress(lib,'ImmGetStatusWindowPos');
  ImmSetStatusWindowPos       := GetProcAddress(lib,'ImmSetStatusWindowPos');
  ImmGetCompositionWindow     := GetProcAddress(lib,'ImmGetCompositionWindow');
  ImmSetCompositionWindow     := GetProcAddress(lib,'ImmSetCompositionWindow');
  ImmGetCandidateWindow       := GetProcAddress(lib,'ImmGetCandidateWindow');
  ImmSetCandidateWindow       := GetProcAddress(lib,'ImmSetCandidateWindow');
  ImmIsUIMessageA             := GetProcAddress(lib,'ImmIsUIMessageA');
  ImmIsUIMessageW             := GetProcAddress(lib,'ImmIsUIMessageW');
  ImmGetVirtualKey            := GetProcAddress(lib,'ImmGetVirtualKey');
  ImmRegisterWordA            := GetProcAddress(lib,'ImmRegisterWordA');
  ImmRegisterWordW            := GetProcAddress(lib,'ImmRegisterWordW');

  ImmUnregisterWordA          := GetProcAddress(lib,'ImmUnregisterWordA');
  ImmUnregisterWordW          := GetProcAddress(lib,'ImmUnregisterWordW');

  ImmGetRegisterWordStyleA    := GetProcAddress(lib,'ImmGetRegisterWordStyleA');
  ImmGetRegisterWordStyleW    := GetProcAddress(lib,'ImmGetRegisterWordStyleW');

  ImmEnumRegisterWordA        := GetProcAddress(lib,'ImmEnumRegisterWordA');
  ImmEnumRegisterWordW        := GetProcAddress(lib,'ImmEnumRegisterWordW');

  ImmDisableIME               := GetProcAddress(lib,'ImmDisableIME');
  ImmEnumInputContext         := GetProcAddress(lib,'ImmEnumInputContext');
  ImmGetImeMenuItemsA         := GetProcAddress(lib,'ImmGetImeMenuItemsA');
  ImmGetImeMenuItemsW         := GetProcAddress(lib,'ImmGetImeMenuItemsW');

  ImmDisableTextFrameService  := GetProcAddress(lib,'ImmDisableTextFrameService');
end;

initialization
  InitImm;

finalization
  if lib<>0 then FreeLibrary(lib);


end.

