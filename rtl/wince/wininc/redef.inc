{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2001 by the Free Pascal development team

    This file defines type names as they are used by Delphi

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

}

{$ifdef read_interface}

type
  PIID = PGUID;
  TIID = TGUID;
<<<<<<< HEAD
<<<<<<< HEAD
  REFIID = ^TIID;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  THANDLE = HANDLE;

  PSmallRect = ^TSmallRect;
  TSmallRect = SMALL_RECT;

  PCharInfo = ^TCharInfo;
  TCharInfo = _CHAR_INFO;

  TFarProc = FARPROC;
  TFNDlgProc = FARPROC;
  TFNThreadStartRoutine = FARPROC;
  TFNTimerAPCRoutine    = FARPROC;
  TFNFiberStartRoutine  = FARPROC;

  PObjectTypeList = ^TObjectTypeList;
  _OBJECT_TYPE_LIST = record
     Level: WORD;
     Sbz: WORD;
     ObjectType: PGUID;
  end;
  TObjectTypeList = _OBJECT_TYPE_LIST;
  OBJECT_TYPE_LIST = _OBJECT_TYPE_LIST;

  AUDIT_EVENT_TYPE = DWORD;

  PBlendFunction = ^TBlendFunction;
  _BLENDFUNCTION = packed record
    BlendOp: BYTE;
    BlendFlags: BYTE;
    SourceConstantAlpha: BYTE;
    AlphaFormat: BYTE;
  end;
  TBlendFunction = _BLENDFUNCTION;
  BLENDFUNCTION = _BLENDFUNCTION;

  _WIN_CERTIFICATE =  Packed Record
                       dwLength         : DWord;
                       wRevision        : Word;
                       wCertificateType : Word;
                       bCertificate     : Array[0..0] of Byte;
                      End;
  TWinCertificate = _WIN_CERTIFICATE;
  PWinCertificate = ^TWinCertificate;

  TMaxLogPalette = Packed Record
     palVersion    : Word;
     palNumEntries : Word;
     palPalEntry   : array[Byte] of TPaletteEntry;
  end;
  PMaxLogPalette = ^TMaxLogPalette;

//
//  A language ID is a 16 bit value which is the combination of a
//  primary language ID and a secondary language ID.  The bits are
//  allocated as follows:
//
//       +-----------------------+-------------------------+
//       |     Sublanguage ID    |   Primary Language ID   |
//       +-----------------------+-------------------------+
//        15                   10 9                       0   bit
//
//
//  Language ID creation/extraction macros:
//
//    MAKELANGID    - construct language id from a primary language id and
//                    a sublanguage id.
//    PRIMARYLANGID - extract primary language id from a language id.
//    SUBLANGID     - extract sublanguage id from a language id.
//

function MAKELANGID(PrimaryLang, SubLang: USHORT): WORD;
function PRIMARYLANGID(LangId: WORD): WORD;
function SUBLANGID(LangId: WORD): WORD;

//
//  A locale ID is a 32 bit value which is the combination of a
//  language ID, a sort ID, and a reserved area.  The bits are
//  allocated as follows:
//
//       +-------------+---------+-------------------------+
//       |   Reserved  | Sort ID |      Language ID        |
//       +-------------+---------+-------------------------+
//        31         20 19     16 15                      0   bit
//
//
//  Locale ID creation/extraction macros:
//
//    MAKELCID            - construct the locale id from a language id and a sort id.
//    MAKESORTLCID        - construct the locale id from a language id, sort id, and sort version.
//    LANGIDFROMLCID      - extract the language id from a locale id.
//    SORTIDFROMLCID      - extract the sort id from a locale id.
//    SORTVERSIONFROMLCID - extract the sort version from a locale id.
//

const
  NLS_VALID_LOCALE_MASK = $000fffff;                              //winnt

function MAKELCID(LangId, SortId: WORD): DWORD;                   //winnt
function MAKESORTLCID(LangId, SortId, SortVersion: WORD): DWORD;  //winnt
function LANGIDFROMLCID(LocaleId: LCID): WORD;                    //winnt
function SORTIDFROMLCID(LocaleId: LCID): WORD;                    //winnt
function SORTVERSIONFROMLCID(LocaleId: LCID): WORD;               //winnt

//
//  Default System and User IDs for language and locale.
//

function LANG_SYSTEM_DEFAULT: WORD;                               //winnt
function LANG_USER_DEFAULT: WORD;                                 //winnt
function LOCALE_SYSTEM_DEFAULT: DWORD;                            //+winnt
function LOCALE_USER_DEFAULT: DWORD;                            //+winnt
function LOCALE_NEUTRAL: DWORD;                                   //winnt
function LOCALE_INVARIANT: DWORD;                                 //winnt

function BeginPaint(hWnd: HWND; var lpPaint: TPaintStruct): HDC; external KernelDLL name 'BeginPaint';
function ChangeDisplaySettingsEx(lpszDeviceName: PWideChar; var lpDevMode: TDeviceMode; wnd: HWND; dwFlags: DWORD; lParam: Pointer): Longint;external KernelDLL name 'ChangeDisplaySettingsEx';
function ClearCommError(hFile: THandle; var lpErrors: DWORD; lpStat: PComStat): BOOL; external KernelDLL name 'ClearCommError';
function ClientToScreen(hWnd: HWND; var lpPoint: TPoint): BOOL; external KernelDLL name 'ClientToScreen';
function ClipCursor(var lpRect:RECT):WINBOOL; external KernelDLL name 'ClipCursor';
function CopyRect(var lprcDst: TRect; const lprcSrc: TRect): BOOL; external KernelDLL name 'CopyRect';
function CreateAcceleratorTable(var Accel; Count: Integer): HACCEL; external KernelDLL name 'CreateAcceleratorTableW';
function CreateAcceleratorTableW(var Accel; Count: Integer): HACCEL; external KernelDLL name 'CreateAcceleratorTableW';
function CreateDialogIndirectParam(hInstance: HINST; const lpTemplate: TDlgTemplate; hWndParent: HWND; lpDialogFunc: TFNDlgProc; dwInitParam: LPARAM): HWND;external KernelDLL name 'CreateDialogIndirectParamW';
function CreateFontIndirect(const p1: TLogFont): HFONT; external KernelDLL name 'CreateFontIndirectW';
function CreateFontIndirectW(const p1: TLogFontW): HFONT; external KernelDLL name 'CreateFontIndirectW';
function CreateIconIndirect(var piconinfo: TIconInfo): HICON; external KernelDLL name 'CreateIconIndirect';
function CreateProcess(lpApplicationName: LPTSTR; lpCommandLine: LPTSTR; lpProcessAttributes, lpThreadAttributes: PSecurityAttributes; bInheritHandles: BOOL; dwCreationFlags: DWORD; lpEnvironment: Pointer; lpCurrentDirectory: LPTSTR;
  const lpStartupInfo: LPStartupInfo; var lpProcessInformation: TProcessInformation): BOOL;external KernelDLL name 'CreateProcessW';
function CreateProcess(lpApplicationName: LPTSTR; lpCommandLine: LPTSTR; lpProcessAttributes, lpThreadAttributes: PSecurityAttributes; bInheritHandles: BOOL; dwCreationFlags: DWORD; lpEnvironment: Pointer; lpCurrentDirectory: LPTSTR;
  const lpStartupInfo: TStartupInfo; var lpProcessInformation: TProcessInformation): BOOL;external KernelDLL name 'CreateProcessW';
function CreateProcessW(lpApplicationName: LPWSTR; lpCommandLine: LPWSTR; lpProcessAttributes, lpThreadAttributes: PSecurityAttributes; bInheritHandles: BOOL; dwCreationFlags: DWORD; lpEnvironment: Pointer; lpCurrentDirectory: LPWSTR;
  const lpStartupInfo: TStartupInfo; var lpProcessInformation: TProcessInformation): BOOL; external KernelDLL name 'CreateProcessW';
procedure DeleteCriticalSection(var CriticalSection : TRTLCriticalSection); external KernelDLL name 'DeleteCriticalSection';
function DeviceIoControl(hDevice: THandle; dwIoControlCode: DWORD; lpInBuffer: Pointer; nInBufferSize: DWORD; lpOutBuffer: Pointer; nOutBufferSize: DWORD; var lpBytesReturned: DWORD; lpOverlapped: POverlapped): BOOL;
  external KernelDLL name 'DeviceIoControl';
function DispatchMessage(const lpMsg: TMsg): Longint; external KernelDLL name 'DispatchMessageW';
function DispatchMessageW(const lpMsg: TMsg): Longint; external KernelDLL name 'DispatchMessageW';
function DrawEdge(hdc: HDC; var qrc: TRect; edge: UINT; grfFlags: UINT): BOOL; external KernelDLL name 'DrawEdge';
function DrawFrameControl(DC: HDC; const Rect: TRect; uType, uState: UINT): BOOL; external KernelDLL name 'DrawFrameControl';
function DrawText(hDC: HDC; lpString: LPWSTR; nCount: Integer; var lpRect: TRect; uFormat: UINT): Integer; external KernelDLL name 'DrawTextW';
function DrawTextW(hDC: HDC; lpString: LPWSTR; nCount: Integer; var lpRect: TRect; uFormat: UINT): Integer; external KernelDLL name 'DrawTextW';
function EndPaint(hWnd: HWND; const lpPaint: TPaintStruct): BOOL; external KernelDLL name 'EndPaint';
procedure EnterCriticalSection(var CriticalSection : TRTLCriticalSection); external KernelDLL name 'EnterCriticalSection';
function EnumDisplaySettings(lpszDeviceName: PWideChar; iModeNum: DWORD; var lpDevMode: TDeviceMode): BOOL;external KernelDLL name 'EnumDisplaySettings';
function EnumDisplaySettingsW(lpszDeviceName: PWideChar; iModeNum: DWORD; var lpDevMode: TDeviceModeW): BOOL;external KernelDLL name 'EnumDisplaySettings';
function ExtCreateRegion(p1: PXForm; p2: DWORD; const p3: TRgnData): HRGN; external KernelDLL name 'ExtCreateRegion';
function FileTimeToSystemTime(const lpFileTime: TFileTime; var lpSystemTime: TSystemTime): BOOL; external KernelDLL name 'FileTimeToSystemTime';
function FileTimeToLocalFileTime(const lpFileTime: TFileTime; var lpLocalFileTime: TFileTime): BOOL; external KernelDLL name 'FileTimeToLocalFileTime';
function FindFirstFile(lpFileName: LPTSTR; var lpFindFileData: TWIN32FindData): THandle; external KernelDLL name 'FindFirstFileW';
function FindFirstFileW(lpFileName: LPTSTR; var lpFindFileData: TWIN32FindDataW): THandle; external KernelDLL name 'FindFirstFileW';
function FindNextFile(hFindFile: THandle; var lpFindFileData: TWIN32FindData): BOOL; external KernelDLL name 'FindNextFileW';
function FindNextFileW(hFindFile: THandle; var lpFindFileData: TWIN32FindDataW): BOOL; external KernelDLL name 'FindNextFileW';
function GetCaretPos(var lpPoint: TPoint): BOOL; external KernelDLL name 'GetCaretPos';
function GetCharABCWidths(DC: HDC; p2, p3: UINT; const ABCStructs): BOOL;external KernelDLL name 'GetCharABCWidths';
function GetCharABCWidthsW(DC: HDC; p2, p3: UINT; const ABCStructs): BOOL;external KernelDLL name 'GetCharABCWidths';
function GetCharWidth32(DC: HDC; p2, p3: UINT; const Widths): BOOL;external KernelDLL name 'GetCharWidth32';
function GetCharWidth32W(DC: HDC; p2, p3: UINT; const Widths): BOOL;external KernelDLL name 'GetCharWidth32';
function GetClassInfo(hInstance: HINST; lpClassName: LPWSTR; var lpWndClass: TWndClassW): BOOL; external KernelDLL name 'GetClassInfoW';
function GetClassInfoW(hInstance: HINST; lpClassName: LPWSTR; var lpWndClass: TWndClassW): BOOL; external KernelDLL name 'GetClassInfoW';
function GetClientRect(hWnd: HWND; var lpRect: TRect): BOOL; external KernelDLL name 'GetClientRect';
function GetClipBox(DC: HDC; var Rect: TRect): Integer; external KernelDLL name 'GetClipBox';
function GetClipCursor(var lpRect: TRect): BOOL; external KernelDLL name 'GetClipCursor';
function GetCommModemStatus(hFile: THandle; var lpModemStat: DWORD): BOOL; external KernelDLL name 'GetCommModemStatus';
function GetCommProperties(hFile: THandle; var lpCommProp: TCommProp): BOOL; external KernelDLL name 'GetCommProperties';
function GetCommState(hFile: THandle; var lpDCB: TDCB): BOOL; external KernelDLL name 'GetCommState';
function GetCommTimeouts(hFile: THandle; var lpCommTimeouts: TCommTimeouts): BOOL; external KernelDLL name 'GetCommTimeouts';
function GetCursorPos(var lpPoint: TPoint): BOOL; external KernelDLL name 'GetCursorPos';
function GetCPInfo(CodePage: UINT; var lpCPInfo: TCPInfo): BOOL;external KernelDLL name 'GetCPInfo';
function GetDIBColorTable(DC: HDC; p2, p3: UINT; var RGBQuadStructs): UINT; external KernelDLL name 'GetDIBColorTable';
function GetDiskFreeSpaceEx(lpDirectoryName: PWideChar; var lpFreeBytesAvailableToCaller, lpTotalNumberOfBytes: TLargeInteger; lpTotalNumberOfFreeBytes: PLargeInteger): BOOL;external KernelDLL name 'GetDiskFreeSpaceExW';
function GetDiskFreeSpaceExW(lpDirectoryName: PWideChar; var lpFreeBytesAvailableToCaller, lpTotalNumberOfBytes: TLargeInteger; lpTotalNumberOfFreeBytes: PLargeInteger): BOOL;external KernelDLL name 'GetDiskFreeSpaceExW';
function GetExitCodeProcess(hProcess: THandle; var lpExitCode: DWORD): BOOL; external KernelDLL name 'GetExitCodeProcess';
function GetExitCodeThread(hThread: THandle; var lpExitCode: DWORD): BOOL; external KernelDLL name 'GetExitCodeThread';
function GetFileInformationByHandle(hFile: THandle; var lpFileInformation: TByHandleFileInformation): BOOL; external KernelDLL name 'GetFileInformationByHandle';
function GetKeyboardLayoutList(nBuff: Integer; var List): UINT; external KernelDLL name 'GetKeyboardLayoutList';
procedure GetLocalTime(var SystemTime: SYSTEMTIME); external KernelDLL name 'GetLocalTime';
function GetMenuItemInfo(p1: HMENU; p2: UINT; p3: BOOL; var p4: TMenuItemInfo): BOOL; external KernelDLL name 'GetMenuItemInfoW';
function GetMenuItemInfoW(p1: HMENU; p2: UINT; p3: BOOL; var p4: TMenuItemInfoW): BOOL; external KernelDLL name 'GetMenuItemInfoW';
function GetMessage(var lpMsg: TMsg; hWnd: HWND; wMsgFilterMin, wMsgFilterMax: UINT): BOOL;external KernelDLL name 'GetMessageW';
function GetOpenFileName(var OpenFile:TOPENFILENAMEW):WINBOOL; external KernelDLL name 'GetOpenFileNameW';
function GetOpenFileNameW(var OpenFile:TOPENFILENAMEW):WINBOOL; external KernelDLL name 'GetOpenFileNameW';
function GetPaletteEntries(Palette: HPALETTE; StartIndex, NumEntries: UINT; var PaletteEntries): UINT; external KernelDLL name 'GetPaletteEntries';
function GetRgnBox(RGN: HRGN; var p2: TRect): Integer; external KernelDLL name 'GetRgnBox';
function GetSaveFileName(var OpenFile:TOPENFILENAMEW):WINBOOL; external KernelDLL name 'GetSaveFileNameW';
function GetSaveFileNameW(var OpenFile:TOPENFILENAMEW):WINBOOL; external KernelDLL name 'GetSaveFileNameW';
function GetScrollInfo(hWnd: HWND; BarFlag: Integer; var ScrollInfo: TScrollInfo): BOOL; external KernelDLL name 'GetScrollInfo';
function GetStringType(dwInfoType: DWORD; const lpSrcStr: PWideChar; cchSrc: BOOL; var lpCharType: Word): BOOL;external KernelDLL name 'GetStringTypeW';
function GetStringTypeW(dwInfoType: DWORD; const lpSrcStr: PWideChar; cchSrc: BOOL; var lpCharType: Word): BOOL;external KernelDLL name 'GetStringTypeW';
function GetStringTypeEx(Locale: LCID; dwInfoType: DWORD; lpSrcStr: PWideChar; cchSrc: Integer; var lpCharType): BOOL; external KernelDLL name 'GetStringTypeExW';
function GetStringTypeExW(Locale: LCID; dwInfoType: DWORD; lpSrcStr: PWideChar; cchSrc: Integer; var lpCharType): BOOL; external KernelDLL name 'GetStringTypeExW';
procedure GetSystemInfo(var SystemInfo:SYSTEM_INFO); external KernelDLL name 'GetSystemInfo';
function GetSystemPaletteEntries(DC: HDC; StartIndex, NumEntries: UINT; var PaletteEntries): UINT; external KernelDLL name 'GetSystemPaletteEntries';
procedure GetSystemTime(var lpSystemTime:SYSTEMTIME); external KernelDLL name 'GetSystemTime';
function GetTextExtentExPoint(DC: HDC; p2: LPWSTR; p3, p4: Integer; p5, p6: PInteger; var p7: TSize): BOOL; external KernelDLL name 'GetTextExtentExPointW';
function GetTextExtentExPointW(DC: HDC; p2: LPWSTR; p3, p4: Integer; p5, p6: PInteger; var p7: TSize): BOOL; external KernelDLL name 'GetTextExtentExPointW';
function GetTextExtentPoint32(DC: HDC; Str: LPWSTR; Count: Integer; var Size: TSize): BOOL;
function GetTextExtentPoint32W(DC: HDC; Str: LPWSTR; Count: Integer; var Size: TSize): BOOL;
function GetTextMetrics(DC: HDC; var TM: TTextMetric): BOOL;external KernelDLL name 'GetTextMetricsW';
function GetThreadContext(hThread: THandle; var lpContext: TContext): BOOL; external KernelDLL name 'GetThreadContext';
function GetThreadTimes(hThread: THandle; var lpCreationTime, lpExitTime, lpKernelTime, lpUserTime: TFileTime): BOOL; external KernelDLL name 'GetThreadTimes';
function GetTimeZoneInformation(var lpTimeZoneInformation: TTimeZoneInformation): DWORD; external KernelDLL name 'GetTimeZoneInformation';
function GetUpdateRect(hWnd: HWND; var lpRect: TRect; bErase: BOOL): BOOL; external KernelDLL name 'GetUpdateRect';
function GetVersionEx(var lpVersionInformation: TOSVersionInfo): BOOL;external KernelDLL name 'GetVersionExW';
function GetVersionExW(var lpVersionInformation: TOSVersionInfoW): BOOL; external KernelDLL name 'GetVersionExW';
function GetWindowRect(hWnd: HWND; var lpRect: TRect): BOOL; external KernelDLL name 'GetWindowRect';
procedure GlobalMemoryStatus(var Buffer: MEMORYSTATUS); external KernelDLL name 'GlobalMemoryStatus';
function ImageList_GetDragImage(var ppt:POINT; var pptHotspot:POINT):HIMAGELIST; external KernelDLL name 'ImageList_GetDragImage';
function InflateRect(var lprc: TRect; dx, dy: Integer): BOOL; external KernelDLL name 'InflateRect';
procedure InitializeCriticalSection(var CriticalSection : TRTLCriticalSection); external KernelDLL name 'InitializeCriticalSection';
function IntersectRect(var lprcDst: TRect; const lprcSrc1, lprcSrc2: TRect): BOOL; external KernelDLL name 'IntersectRect';
function IsDialogMessage(hDlg: HWND; var lpMsg: TMsg): BOOL; external KernelDLL name 'IsDialogMessageW';
function IsDialogMessageW(hDlg: HWND; var lpMsg: TMsg): BOOL; external KernelDLL name 'IsDialogMessageW';
procedure LeaveCriticalSection(var CriticalSection : TRTLCriticalSection); external KernelDLL name 'LeaveCriticalSection';
function LocalFileTimeToFileTime(const lpLocalFileTime: TFileTime; var lpFileTime: TFileTime): BOOL; external KernelDLL name 'LocalFileTimeToFileTime';
function MapDialogRect(hDlg: HWND; var lpRect: TRect): BOOL; external KernelDLL name 'MapDialogRect';
function MapWindowPoints(hWndFrom, hWndTo: HWND; var lpPoints; cPoints: UINT): Integer; external KernelDLL name 'MapWindowPoints';
//redirected to MsgWaitForMultipleObjectsEx
function MsgWaitForMultipleObjects(nCount: DWORD; var pHandles; fWaitAll: BOOL; dwMilliseconds, dwWakeMask: DWORD): DWORD;
function OffsetRect(var lprc: TRect; dx, dy: Integer): BOOL; external KernelDLL name 'OffsetRect';
function PeekMessage(var lpMsg: TMsg; hWnd: HWND; wMsgFilterMin, wMsgFilterMax, wRemoveMsg: UINT): BOOL;external KernelDLL name 'PeekMessageW';
function PeekMessageW(var lpMsg: TMsg; hWnd: HWND; wMsgFilterMin, wMsgFilterMax, wRemoveMsg: UINT): BOOL; external KernelDLL name 'PeekMessageW';
function Polygon(DC: HDC; var Points; Count: Integer): BOOL; external KernelDLL name 'Polygon';
function Polyline(DC: HDC; var Points; Count: Integer): BOOL; external KernelDLL name 'Polyline';
function PtInRect(const lprc:TRect; pt:POINT):WINBOOL; external KernelDLL name 'PtInRect';
function QueryPerformanceCounter(var lpPerformanceCount: TLargeInteger): BOOL; external KernelDLL name 'QueryPerformanceCounter';
function QueryPerformanceFrequency(var lpFrequency: TLargeInteger): BOOL; external KernelDLL name 'QueryPerformanceFrequency';
procedure RaiseException(dwExceptionCode:DWORD; dwExceptionFlags:DWORD; nNumberOfArguments:DWORD; var lpArguments:DWORD); external KernelDLL name 'RaiseException';
function ReadFile(hFile: THandle; var Buffer; nNumberOfBytesToRead: DWORD; var lpNumberOfBytesRead: DWORD; lpOverlapped: POverlapped): BOOL; external KernelDLL name 'ReadFile';
function ReadProcessMemory(hProcess: THandle; const lpBaseAddress: Pointer; lpBuffer: Pointer; nSize: DWORD; var lpNumberOfBytesRead: DWORD): BOOL; external KernelDLL name 'ReadProcessMemory';
function RegCreateKeyEx(hKey: HKEY; lpSubKey: LPWSTR; Reserved: DWORD; lpClass: LPWSTR; dwOptions: DWORD; samDesired: REGSAM; lpSecurityAttributes: PSecurityAttributes; var phkResult: HKEY; lpdwDisposition: PDWORD): Longint; external KERNELDLL name 'RegCreateKeyExW';
function RegCreateKeyExW(hKey: HKEY; lpSubKey: LPWSTR; Reserved: DWORD; lpClass: LPWSTR; dwOptions: DWORD; samDesired: REGSAM; lpSecurityAttributes: PSecurityAttributes; var phkResult: HKEY; lpdwDisposition: PDWORD): Longint; external KERNELDLL name 'RegCreateKeyExW';
function RegEnumKeyEx(hKey: HKEY; dwIndex: DWORD; lpName: LPWSTR; var lpcbName: DWORD; lpReserved: Pointer; lpClass: LPWSTR; lpcbClass: PDWORD; lpftLastWriteTime: PFileTime): Longint; external KernelDLL name 'RegEnumKeyExW';
function RegEnumKeyExW(hKey: HKEY; dwIndex: DWORD; lpName: LPWSTR; var lpcbName: DWORD; lpReserved: Pointer; lpClass: LPWSTR; lpcbClass: PDWORD; lpftLastWriteTime: PFileTime): Longint; external KernelDLL name 'RegEnumKeyExW';
function RegEnumValue(hKey: HKEY; dwIndex: DWORD; lpValueName: LPWSTR; var lpcbValueName: DWORD; lpReserved: Pointer; lpType: PDWORD; lpData: PByte; lpcbData: PDWORD): Longint; external KernelDLL name 'RegEnumValueW';
function RegEnumValueW(hKey: HKEY; dwIndex: DWORD; lpValueName: LPWSTR; var lpcbValueName: DWORD; lpReserved: Pointer; lpType: PDWORD; lpData: PByte; lpcbData: PDWORD): Longint; external KernelDLL name 'RegEnumValueW';
function RegisterClass(const lpWndClass: TWndClass): ATOM;external KernelDLL name 'RegisterClassW';
function RegisterClassW(const lpWndClass: TWndClassW): ATOM; external KernelDLL name 'RegisterClassW';
function RegOpenKeyEx(hKey: HKEY; lpSubKey: LPWSTR; ulOptions: DWORD; samDesired: REGSAM; var phkResult: HKEY): Longint; external KernelDLL name 'RegOpenKeyExW';
function RegOpenKeyExW(hKey: HKEY; lpSubKey: LPWSTR; ulOptions: DWORD; samDesired: REGSAM; var phkResult: HKEY): Longint; external KernelDLL name 'RegOpenKeyExW';
function ScreenToClient(hWnd: HWND; var lpPoint: TPoint): BOOL; external KernelDLL name 'ScreenToClient';
function ScrollWindowEx(hWnd:HWND; dx:longint; dy:longint; prcScroll:lpRECT; prcClip:lpRECT;hrgnUpdate:HRGN; prcUpdate:LPRECT; flags:UINT):longint; external KernelDLL name 'ScrollWindowEx';
function SendMessageTimeout(hWnd: HWND; Msg: UINT; wParam: WPARAM; lParam: LPARAM; fuFlags, uTimeout: UINT; var lpdwResult: DWORD): LRESULT;external KernelDLL name 'SendMessageTimeout';
function SendMessageTimeoutW(hWnd: HWND; Msg: UINT; wParam: WPARAM; lParam: LPARAM; fuFlags, uTimeout: UINT; var lpdwResult: DWORD): LRESULT;external KernelDLL name 'SendMessageTimeout';
function SetCommState(hFile: THandle; const lpDCB: TDCB): BOOL; external KernelDLL name 'SetCommState';
function SetCommTimeouts(hFile: THandle; const lpCommTimeouts: TCommTimeouts): BOOL; external KernelDLL name 'SetCommTimeouts';
function SetDIBColorTable(DC: HDC; p2, p3: UINT; var RGBQuadSTructs): UINT; external KernelDLL name 'SetDIBColorTable';
function SetDIBits(DC: HDC; Bitmap: HBITMAP; StartScan, NumScans: UINT; Bits: Pointer; var BitsInfo: TBitmapInfo; Usage: UINT): Integer; external KernelDLL name 'SetDIBits';
function SetFileTime(hFile:HANDLE; var lpCreationTime:FILETIME; var lpLastAccessTime:FILETIME; var lpLastWriteTime:FILETIME):WINBOOL; external KernelDLL name 'SetFileTime';
function SetLocalTime(var lpSystemTime:SYSTEMTIME):WINBOOL; external KernelDLL name 'SetLocalTime';
function SetMenuItemInfo(p1: HMENU; p2: UINT; p3: BOOL; const p4: TMenuItemInfo): BOOL; external KernelDLL name 'SetMenuItemInfoW';
function SetMenuItemInfoW(p1: HMENU; p2: UINT; p3: BOOL; const p4: TMenuItemInfoW): BOOL; external KernelDLL name 'SetMenuItemInfoW';
function SetPaletteEntries(Palette: HPALETTE; StartIndex, NumEntries: UINT; var PaletteEntries): UINT; external KernelDLL name 'SetPaletteEntries';
function SetRect(var lprc:TRect; xLeft:longint; yTop:longint; xRight:longint; yBottom:longint):WINBOOL; external KernelDLL name 'SetRect';
function SetRectEmpty(var lprc: TRect): BOOL; external KernelDLL name 'SetRectEmpty';
function SetScrollInfo(hWnd: HWND; BarFlag: Integer; const ScrollInfo: TScrollInfo; Redraw: BOOL): Integer; external KernelDLL name 'SetScrollInfo';
function SetSysColors(cElements: Integer; const lpaElements; const lpaRgbValues): BOOL; external KernelDLL name 'SetSysColors';
function SetSystemTime(var lpSystemTime:SYSTEMTIME):WINBOOL; external KernelDLL name 'SetSystemTime';
function SetThreadContext(hThread: THandle; const lpContext: TContext): BOOL; external KernelDLL name 'SetThreadContext';
function StartDoc(DC: HDC; const p2: TDocInfo): Integer;external KernelDLL name 'StartDocW';
function StartDocW(DC: HDC; const p2: TDocInfoW): Integer;external KernelDLL name 'StartDocW';
function SubtractRect(var lprcDst: TRect; const lprcSrc1, lprcSrc2: TRect): BOOL; external KernelDLL name 'SubtractRect';
function SystemTimeToFileTime(const lpSystemTime: TSystemTime; var lpFileTime: TFileTime): BOOL; external KernelDLL name 'SystemTimeToFileTime';
function TranslateAccelerator(hWnd: HWND; hAccTable: HACCEL; var lpMsg: TMsg): Integer; external KernelDLL name 'TranslateAcceleratorW';
function TranslateAcceleratorW(hWnd: HWND; hAccTable: HACCEL; var lpMsg: TMsg): Integer; external KernelDLL name 'TranslateAcceleratorW';
function TranslateCharsetInfo(var lpSrc: DWORD; var lpCs: TCharsetInfo; dwFlags: DWORD): BOOL; external KernelDLL name 'TranslateCharsetInfo';
function TranslateMessage(const lpMsg: TMsg): BOOL; external KernelDLL name 'TranslateMessage';
function TryEnterCriticalSection(var CriticalSection : TRTLCriticalSection) : BOOL; external KernelDLL name 'TryEnterCriticalSection';
function UnionRect(var lprcDst: TRect; const lprcSrc1, lprcSrc2: TRect): BOOL; external KernelDLL name 'UnionRect';
function VerQueryValue(pBlock: Pointer; lpSubBlock: LPWSTR; var lplpBuffer: Pointer; var uLen: UINT): BOOL; external KernelDLL name 'VerQueryValueW';
function VerQueryValueW(pBlock: Pointer; lpSubBlock: LPWSTR; var lplpBuffer: Pointer; var uLen: UINT): BOOL; external KernelDLL name 'VerQueryValueW';
function VirtualProtect(lpAddress:LPVOID; dwSize:DWORD; flNewProtect:DWORD; var lpflOldProtect:DWORD):WINBOOL; external KernelDLL name 'VirtualProtect';
function VirtualQuery(lpAddress: Pointer; var lpBuffer: TMemoryBasicInformation; dwLength: DWORD): DWORD; external KernelDLL name 'VirtualQuery';
function WaitCommEvent(hFile: THandle; var lpEvtMask: DWORD; lpOverlapped: POverlapped): BOOL; external KernelDLL name 'WaitCommEvent';
function WaitForDebugEvent(var lpDebugEvent: TDebugEvent; dwMilliseconds: DWORD): BOOL; external KernelDLL name 'WaitForDebugEvent';
function WriteFile(hFile: THandle; const Buffer; nNumberOfBytesToWrite: DWORD; var lpNumberOfBytesWritten: DWORD; lpOverlapped: POverlapped): BOOL; external KernelDLL name 'WriteFile';
function WriteProcessMemory(hProcess: THandle; const lpBaseAddress: Pointer; lpBuffer: Pointer; nSize: DWORD; var lpNumberOfBytesWritten: DWORD): BOOL; external KernelDLL name 'WriteProcessMemory';

{$endif read_interface}


{$ifdef read_implementation}

function MAKELANGID(PrimaryLang, SubLang: USHORT): WORD;
begin
  MAKELANGID := (SubLang shl 10) or PrimaryLang;
end;

function PRIMARYLANGID(LangId: WORD): WORD;
begin
  PRIMARYLANGID := LangId and $3FF;
end;

function SUBLANGID(LangId: WORD): WORD;
begin
  SUBLANGID := LangId shr 10;
end;

function MAKELCID(LangId, SortId: WORD): DWORD;
begin
  MAKELCID := (DWORD(SortId) shl 16) or DWORD(LangId);
end;

function MAKESORTLCID(LangId, SortId, SortVersion: WORD): DWORD;
begin
  MAKESORTLCID := MAKELCID(LangId, SortId) or (SortVersion shl 20);
end;

function LANGIDFROMLCID(LocaleId: LCID): WORD;
begin
  LANGIDFROMLCID := WORD(LocaleId);
end;

function SORTIDFROMLCID(LocaleId: LCID): WORD;
begin
  SORTIDFROMLCID := WORD((DWORD(LocaleId) shr 16) and $F);
end;

function SORTVERSIONFROMLCID(LocaleId: LCID): WORD;
begin
  SORTVERSIONFROMLCID := WORD((DWORD(LocaleId) shr 20) and $F);
end;

function LANG_SYSTEM_DEFAULT: WORD;
begin
  LANG_SYSTEM_DEFAULT := MAKELANGID(LANG_NEUTRAL, SUBLANG_SYS_DEFAULT);
end;

function LANG_USER_DEFAULT: WORD;
begin
  LANG_USER_DEFAULT := MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT);
end;

function LOCALE_USER_DEFAULT: DWORD;
begin
  LOCALE_USER_DEFAULT:= MAKELCID(LANG_USER_DEFAULT, SORT_DEFAULT);
end;

function LOCALE_SYSTEM_DEFAULT: DWORD;
begin
  LOCALE_SYSTEM_DEFAULT:= MAKELCID(LANG_SYSTEM_DEFAULT, SORT_DEFAULT);
end;

function LOCALE_NEUTRAL: DWORD;
begin
  LOCALE_NEUTRAL := MAKELCID(MAKELANGID(LANG_NEUTRAL, SUBLANG_NEUTRAL), SORT_DEFAULT);
end;

function LOCALE_INVARIANT: DWORD;
begin
  LOCALE_INVARIANT := MAKELCID(MAKELANGID(LANG_INVARIANT, SUBLANG_NEUTRAL), SORT_DEFAULT);
end;

function MsgWaitForMultipleObjects(nCount: DWORD; var pHandles; fWaitAll: BOOL; dwMilliseconds, dwWakeMask: DWORD): DWORD;
begin
  MsgWaitForMultipleObjects:=MsgWaitForMultipleObjectsEx(nCount,@pHandles,dwMilliseconds,dwWakeMask,0);
end;

function GetTextExtentPoint32(DC: HDC; Str: LPWSTR; Count: Integer; var Size: TSize): BOOL;
begin
  Result:=GetTextExtentExPoint(DC, Str, Count, 0, nil, nil, @Size);
end;

function GetTextExtentPoint32W(DC: HDC; Str: LPWSTR; Count: Integer; var Size: TSize): BOOL;
begin
  Result:=GetTextExtentExPoint(DC, Str, Count, 0, nil, nil, @Size);
end;

{$endif read_implementation}

