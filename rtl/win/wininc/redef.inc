{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2001 by the Free Pascal development team

    This file defines type names as they are used by Delphi

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$ifdef read_interface}

type
  PIID = PGUID;
  TIID = TGUID;
  THANDLE = HANDLE;

  PSmallRect = ^TSmallRect;
  TSmallRect = SMALL_RECT;

  PCharInfo = ^TCharInfo;
  TCharInfo = _CHAR_INFO;

  TFNDlgProc = FARPROC;
  TFNThreadStartRoutine = FARPROC;
  TFNTimerAPCRoutine    = FARPROC;
  TFNFiberStartRoutine  = FARPROC;
  TFNHookProc =  HOOKPROC;

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

  TOwnerDrawState = set of (
    odSelected,odGrayed,odDisabled,odChecked,
    odFocused,odDefault,odHotLight,odInactive,odNoAccel,odNoFocusRect,
    odReserved1,odReserved2,odComboBoxEdit);

type
  PKeyboardState = ^TKeyboardState;
  TKeyboardState = array[0..255] of Byte;

const
  { dll names }
  advapi32 = 'advapi32.dll';
  kernel32 = 'kernel32.dll';
  mpr = 'mpr.dll';
  version = 'version.dll';
  comctl32 = 'comctl32.dll';
  gdi32 = 'gdi32.dll';
  opengl32 = 'opengl32.dll';
  user32 = 'user32.dll';
  wintrust = 'wintrust.dll';

  { Openfile Share modes normally declared in sysutils }
  fmShareCompat    = $00000000;
  fmShareExclusive = $10;
  fmShareDenyWrite = $20;
  fmShareDenyRead  = $30;
  fmShareDenyNone  = $40;

  { HRESULT codes, delphilike }
  NOERROR = 0;

  { Some missing consts }
  SIF_TRACKPOS = $10;
  CP_UTF7 = 65000;
  CP_UTF8 = 65001;

  WHEEL_DELTA = 120;
  WHEEL_PAGESCROLL = MAXDWORD;

  MSH_MOUSEWHEEL   = 'MSWHEEL_ROLLMSG';
  MOUSEZ_CLASSNAME = 'MouseZ';
  MOUSEZ_TITLE     = 'Magellan MSWHEEL';
  MSH_WHEELSUPPORT = 'MSH_WHEELSUPPORT_MSG';
  MSH_SCROLL_LINES = 'MSH_SCROLL_LINES_MSG';

  MSH_WHEELMODULE_CLASS = MOUSEZ_CLASSNAME;
  MSH_WHEELMODULE_TITLE = MOUSEZ_TITLE;

const
  { Severity values }
  FACILITY_NT_BIT = $10000000;
  HFILE_ERROR = HFILE(-1);

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
  NLS_VALID_LOCALE_MASK = $000fffff;

function MAKELCID(LangId, SortId: WORD): DWORD;
function MAKESORTLCID(LangId, SortId, SortVersion: WORD): DWORD;
function LANGIDFROMLCID(LocaleId: LCID): WORD;
function SORTIDFROMLCID(LocaleId: LCID): WORD;
function SORTVERSIONFROMLCID(LocaleId: LCID): WORD;

//
//  Default System and User IDs for language and locale.
//

function LANG_SYSTEM_DEFAULT: WORD;
function LANG_USER_DEFAULT: WORD;
function LOCALE_NEUTRAL: DWORD;
function LOCALE_INVARIANT: DWORD;


function Succeeded(Status : HRESULT) : BOOL;
function Failed(Status : HRESULT) : BOOL;
function IsError(Status : HRESULT) : BOOL;
function HResultCode(hr : HRESULT) : Longint;
function HResultFacility(hr : HRESULT) : Longint;
function HResultSeverity(hr : HRESULT) : Longint;
function MakeResult(p1,p2,mask : Longint) : HRESULT;
function HResultFromWin32(x : Longint) : HRESULT;
function HResultFromNT(x : Longint) : HRESULT;

function SmallPointToPoint(const P: TSmallPoint): TPoint;
function PointToSmallPoint(const P: TPoint): TSmallPoint;

function GetCurrentTime: Longint;inline;

procedure InitializeCriticalSection(var CriticalSection : TRTLCriticalSection); external 'kernel32' name 'InitializeCriticalSection';
procedure EnterCriticalSection(var CriticalSection : TRTLCriticalSection); external 'kernel32' name 'EnterCriticalSection';
procedure LeaveCriticalSection(var CriticalSection : TRTLCriticalSection); external 'kernel32' name 'LeaveCriticalSection';
procedure DeleteCriticalSection(var CriticalSection : TRTLCriticalSection); external 'kernel32' name 'DeleteCriticalSection';
function InitializeCriticalSectionAndSpinCount(var CriticalSection : TRTLCriticalSection;dwSpinCount : DWORD) : BOOL; external 'kernel32' name 'InitializeCriticalSectionAndSpinCount';
function SetCriticalSectionSpinCount(var CriticalSection : TRTLCriticalSection;dwSpinCount : DWORD ): DWORD; external 'kernel32' name 'SetCriticalSectionSpinCount';
function TryEnterCriticalSection(var CriticalSection : TRTLCriticalSection) : BOOL; external 'kernel32' name 'TryEnterCriticalSection';

function GetWindowInfo(hWnd:HWND; var pwi:TWindowInfo):WINBOOL; external 'user32' name 'GetWindowInfo';
function ControlService(hService:SC_HANDLE; dwControl:DWORD; var ServiceStatus:TSERVICESTATUS):WINBOOL; external 'advapi32' name 'ControlService';
function QueryServiceStatus(hService:SC_HANDLE; var lpServiceStatus:TSERVICESTATUS):WINBOOL; external 'advapi32' name 'QueryServiceStatus';
function SetServiceStatus(hServiceStatus:SERVICE_STATUS_HANDLE; const ServiceStatus:TSERVICESTATUS):WINBOOL; external 'advapi32' name 'SetServiceStatus';

//function _lwrite(hFile: HFILE; const lpBuffer: LPCSTR; uBytes: UINT): UINT; external 'kernel32' name '_lwrite';
//function AccessCheck(pSecurityDescriptor: PSecurityDescriptor; ClientToken: THandle; DesiredAccess: DWORD; const GenericMapping: TGenericMapping; var PrivilegeSet: TPrivilegeSet; var PrivilegeSetLength: DWORD; var GrantedAccess: DWORD;
//  var AccessStatus: BOOL): BOOL; external 'advapi32' name 'AccessCheck';
//function AccessCheckAndAuditAlarm(SubsystemName: PChar; HandleId: Pointer; ObjectTypeName, ObjectName: PChar; SecurityDescriptor: PSecurityDescriptor; DesiredAccess: DWORD; const GenericMapping: TGenericMapping;  ObjectCreation: BOOL;
//  var GrantedAccess: DWORD; var AccessStatus, pfGenerateOnClose: BOOL): BOOL; external 'advapi32' name 'AccessCheckAndAuditAlarmA';
//function AccessCheckAndAuditAlarmA(SubsystemName: LPCSTR; HandleId: Pointer; ObjectTypeName, ObjectName: LPCSTR; SecurityDescriptor: PSecurityDescriptor; DesiredAccess: DWORD; const GenericMapping: TGenericMapping;  ObjectCreation: BOOL;
//  var GrantedAccess: DWORD; var AccessStatus, pfGenerateOnClose: BOOL): BOOL; external 'advapi32' name 'AccessCheckAndAuditAlarmA';
//function AccessCheckAndAuditAlarmW(SubsystemName: LPWSTR; HandleId: Pointer; ObjectTypeName, ObjectName: LPWSTR; SecurityDescriptor: PSecurityDescriptor; DesiredAccess: DWORD; const GenericMapping: TGenericMapping;  ObjectCreation: BOOL;
//  var GrantedAccess: DWORD; var AccessStatus, pfGenerateOnClose: BOOL): BOOL; external 'advapi32' name 'AccessCheckAndAuditAlarmW';
//function AccessCheckByType(pSecurityDescriptor: PSecurityDescriptor; PrincipalSelfSid: PSID; ClientToken: THandle; DesiredAccess: DWORD; ObjectTypeList: PObjectTypeList; const GenericMapping: TGenericMapping; ObjectTypeListLength: DWORD;
//  var PrivilegeSet: TPrivilegeSet; var PrivilegeSetLength: DWORD; var GrantedAccess: DWORD; var AccessStatus: BOOL): BOOL;external 'advapi32' name 'AccessCheckByType';
//function AccessCheckByTypeAndAuditAlarm(SubsystemName: PChar; HandleId: Pointer; ObjectTypeName, ObjectName: PChar; SecurityDescriptor: PSecurityDescriptor; PrincipalSelfSid: PSID; DesiredAccess: DWORD; AuditType: AUDIT_EVENT_TYPE; Flags: DWORD;
//  ObjectTypeList: PObjectTypeList; ObjectTypeListLength: DWORD; const GenericMapping: TGenericMapping;  ObjectCreation: BOOL; var GrantedAccess: DWORD; var AccessStatus, pfGenerateOnClose: BOOL): BOOL;
//  external 'advapi32' name 'AccessCheckByTypeAndAuditAlarm';
//function AccessCheckByTypeAndAuditAlarmA(SubsystemName: LPCSTR; HandleId: Pointer; ObjectTypeName, ObjectName: LPCSTR; SecurityDescriptor: PSecurityDescriptor; PrincipalSelfSid: PSID; DesiredAccess: DWORD; AuditType: AUDIT_EVENT_TYPE; Flags: DWORD;
//  ObjectTypeList: PObjectTypeList; ObjectTypeListLength: DWORD; const GenericMapping: TGenericMapping;  ObjectCreation: BOOL; var GrantedAccess: DWORD; var AccessStatus, pfGenerateOnClose: BOOL): BOOL;
//  external 'advapi32' name 'AccessCheckByTypeAndAuditAlarmA';
//function AccessCheckByTypeAndAuditAlarmW(SubsystemName: LPWSTR; HandleId: Pointer; ObjectTypeName, ObjectName: LPWSTR; SecurityDescriptor: PSecurityDescriptor; PrincipalSelfSid: PSID; DesiredAccess: DWORD; AuditType: AUDIT_EVENT_TYPE; Flags: DWORD;
//  ObjectTypeList: PObjectTypeList; ObjectTypeListLength: DWORD; const GenericMapping: TGenericMapping;  ObjectCreation: BOOL; var GrantedAccess: DWORD; var AccessStatus, pfGenerateOnClose: BOOL): BOOL;
//  external 'advapi32' name 'AccessCheckByTypeAndAuditAlarmW';
//function AccessCheckByTypeResultList(pSecurityDescriptor: PSecurityDescriptor; PrincipalSelfSid: PSID; ClientToken: THandle; DesiredAccess: DWORD; ObjectTypeList: PObjectTypeList; const GenericMapping: TGenericMapping; ObjectTypeListLength: DWORD;
//  var PrivilegeSet: TPrivilegeSet; var PrivilegeSetLength: DWORD; var GrantedAccess: DWORD; var AccessStatusList: DWORD): BOOL;external 'advapi32' name 'AccessCheckByTypeResultList';
//function AccessCheckByTypeResultListAndAuditAlarm(SubsystemName: PChar; HandleId: Pointer; ObjectTypeName, ObjectName: PChar; SecurityDescriptor: PSecurityDescriptor; PrincipalSelfSid: PSID; DesiredAccess: DWORD; AuditType: AUDIT_EVENT_TYPE;
//  Flags: DWORD; ObjectTypeList: PObjectTypeList; ObjectTypeListLength: DWORD; const GenericMapping: TGenericMapping;  ObjectCreation: BOOL; var GrantedAccess: DWORD; var AccessStatusList: DWORD; var pfGenerateOnClose: BOOL): BOOL;
//  external 'advapi32' name 'AccessCheckByTypeResultListAndAuditAlarmA';
//function AccessCheckByTypeResultListAndAuditAlarmA(SubsystemName: LPCSTR; HandleId: Pointer; ObjectTypeName, ObjectName: LPCSTR; SecurityDescriptor: PSecurityDescriptor; PrincipalSelfSid: PSID; DesiredAccess: DWORD; AuditType: AUDIT_EVENT_TYPE;
//  Flags: DWORD; ObjectTypeList: PObjectTypeList; ObjectTypeListLength: DWORD; const GenericMapping: TGenericMapping;  ObjectCreation: BOOL; var GrantedAccess: DWORD; var AccessStatusList: DWORD; var pfGenerateOnClose: BOOL): BOOL;
//  external 'advapi32' name 'AccessCheckByTypeResultListAndAuditAlarmA';
//function AccessCheckByTypeResultListAndAuditAlarmW(SubsystemName: LPWSTR; HandleId: Pointer; ObjectTypeName, ObjectName: LPWSTR; SecurityDescriptor: PSecurityDescriptor; PrincipalSelfSid: PSID; DesiredAccess: DWORD; AuditType: AUDIT_EVENT_TYPE;
// Flags: DWORD; ObjectTypeList: PObjectTypeList; ObjectTypeListLength: DWORD; const GenericMapping: TGenericMapping;  ObjectCreation: BOOL; var GrantedAccess: DWORD; var AccessStatusList: DWORD; var pfGenerateOnClose: BOOL): BOOL;
// external 'advapi32' name 'AccessCheckByTypeResultListAndAuditAlarmW';
//function AddAccessAllowedAce(var pAcl: TACL; dwAceRevision: DWORD; AccessMask: DWORD; pSid: PSID): BOOL; external 'advapi32' name 'AddAccessAllowedAce';
//function AddAccessAllowedAceEx(var pAcl: TACL; dwAceRevision: DWORD; AceFlags: DWORD; AccessMask: DWORD; pSid: PSID): BOOL;external 'advapi32' name 'AddAccessAllowedAceEx';
//function AddAccessAllowedObjectAce(var pAcl: TACL; dwAceRevision: DWORD; AceFlags: DWORD; AccessMask: DWORD; ObjectTypeGuid, InheritedObjectTypeGuid: PGuid; pSid: Pointer): BOOL;external 'advapi32' name 'AddAccessAllowedObjectAce';
//function AddAccessDeniedAce(var pAcl: TACL; dwAceRevision: DWORD; AccessMask: DWORD; pSid: PSID): BOOL; external 'advapi32' name 'AddAccessDeniedAce';
//function AddAccessDeniedAceEx(var pAcl: TACL; dwAceRevision: DWORD; ACEFlags: DWORD; AccessMask: DWORD; pSid: PSID): BOOL;external 'advapi32' name 'AddAccessDeniedAceEx';
//function AddAccessDeniedObjectAce(var pAcl: TACL; dwAceRevision: DWORD; AceFlags: DWORD; AccessMask: DWORD; ObjectTypeGuid, InheritedObjectTypeGuid: PGuid; pSid: Pointer): BOOL;external 'advapi32' name 'AddAccessDeniedObjectAce';
//function AddAce(var pAcl: TACL; dwAceRevision, dwStartingAceIndex: DWORD; pAceList: Pointer; nAceListLength: DWORD): BOOL; external 'advapi32' name 'AddAce';
//function AddAuditAccessAce(var pAcl: TACL; dwAceRevision: DWORD; dwAccessMask: DWORD; pSid: Pointer; bAuditSuccess, bAuditFailure: BOOL): BOOL; external 'advapi32' name 'AddAuditAccessAce';
//function AddAuditAccessAceEx(var pAcl: TACL; dwAceRevision: DWORD; AceFlags: DWORD; dwAccessMask: DWORD; pSid: Pointer; bAuditSuccess, bAuditFailure: BOOL): BOOL;external 'advapi32' name 'AddAuditAccessAceEx';
//function AddAuditAccessObjectAce(var pAcl: TACL; dwAceRevision: DWORD; AceFlags: DWORD; AccessMask: DWORD; ObjectTypeGuid, InheritedObjectTypeGuid: PGuid; pSid: Pointer; bAuditSuccess, bAuditFailure: BOOL): BOOL;
//  external 'advapi32' name 'AddAuditAccessObjectAce';
//function AdjustTokenGroups(TokenHandle: THandle; ResetToDefault: BOOL; const NewState: TTokenGroups; BufferLength: DWORD; var PreviousState: TTokenGroups; var ReturnLength: DWORD): BOOL; external 'advapi32' name 'AdjustTokenGroups';
function AdjustTokenPrivileges(TokenHandle: THandle; DisableAllPrivileges: BOOL; const NewState: TTokenPrivileges; BufferLength: DWORD;
                               var PreviousState: TTokenPrivileges; var ReturnLength: DWORD): BOOL;   external 'advapi32' name 'AdjustTokenPrivileges';
function AdjustWindowRect(var lpRect: TRect; dwStyle: DWORD; bMenu: BOOL): BOOL; external 'user32' name 'AdjustWindowRect';
function AdjustWindowRectEx(var lpRect: TRect; dwStyle: DWORD; bMenu: BOOL; dwExStyle: DWORD): BOOL; external 'user32' name 'AdjustWindowRectEx';
function AllocateAndInitializeSid(const pIdentifierAuthority: TSIDIdentifierAuthority; nSubAuthorityCount: Byte; nSubAuthority0, nSubAuthority1: DWORD; nSubAuthority2, nSubAuthority3, nSubAuthority4: DWORD;
  nSubAuthority5, nSubAuthority6, nSubAuthority7:DWORD; var pSid: Pointer): BOOL; external 'advapi32' name 'AllocateAndInitializeSid';
function AllocateLocallyUniqueId(var Luid: TLargeInteger): BOOL; external 'advapi32' name 'AllocateLocallyUniqueId';
//function AlphaDIBBlend(DC: HDC; p2, p3, p4, p5: Integer; const p6: Pointer; const p7: PBitmapInfo; p8: UINT; p9, p10, p11, p12: Integer; p13: TBlendFunction): BOOL;external 'gdi32' name 'AlphaDIBBlend';
function BackupRead(hFile: THandle; lpBuffer: PByte; nNumberOfBytesToRead: DWORD; var lpNumberOfBytesRead: DWORD; bAbort: BOOL; bProcessSecurity: BOOL; var lpContext: Pointer): BOOL; external 'kernel32' name 'BackupRead';
function BackupSeek(hFile: THandle; dwLowBytesToSeek, dwHighBytesToSeek: DWORD; var lpdwLowByteSeeked, lpdwHighByteSeeked: DWORD; lpContext: Pointer): BOOL; external 'kernel32' name 'BackupSeek';
function BackupWrite(hFile: THandle; lpBuffer: PByte; nNumberOfBytesToWrite: DWORD; var lpNumberOfBytesWritten: DWORD; bAbort, bProcessSecurity: BOOL; var lpContext: Pointer): BOOL; external 'kernel32' name 'BackupWrite';
function BeginPaint(hWnd: HWND; var lpPaint: TPaintStruct): HDC; external 'user32' name 'BeginPaint';
function BuildCommDCB(lpDef: PChar; var lpDCB: TDCB): BOOL;external 'kernel32' name 'BuildCommDCBA';
function BuildCommDCBA(lpDef: LPCSTR; var lpDCB: TDCB): BOOL; external 'kernel32' name 'BuildCommDCBA';
function BuildCommDCBAndTimeouts(lpDef: PChar; var lpDCB: TDCB; var lpCommTimeouts: TCommTimeouts): BOOL;external 'kernel32' name 'BuildCommDCBAndTimeoutsA';
function BuildCommDCBAndTimeoutsA(lpDef: LPCSTR; var lpDCB: TDCB; var lpCommTimeouts: TCommTimeouts): BOOL; external 'kernel32' name 'BuildCommDCBAndTimeoutsA';
function BuildCommDCBAndTimeoutsW(lpDef: LPWSTR; var lpDCB: TDCB; var lpCommTimeouts: TCommTimeouts): BOOL; external 'kernel32' name 'BuildCommDCBAndTimeoutsW';
function BuildCommDCBW(lpDef: LPWSTR; var lpDCB: TDCB): BOOL; external 'kernel32' name 'BuildCommDCBW';
function CallMsgFilter(var lpMsg: TMsg; nCode: Integer): BOOL;external 'user32' name 'CallMsgFilterA';
function CallMsgFilterA(var lpMsg: TMsg; nCode: Integer): BOOL; external 'user32' name 'CallMsgFilterA';
function CallMsgFilterW(var lpMsg: TMsg; nCode: Integer): BOOL; external 'user32' name 'CallMsgFilterW';
function CallNamedPipe(lpNamedPipeName: PChar; lpInBuffer: Pointer; nInBufferSize: DWORD; lpOutBuffer: Pointer; nOutBufferSize: DWORD; var lpBytesRead: DWORD; nTimeOut: DWORD): BOOL;external 'kernel32' name 'CallNamedPipeA';
function CallNamedPipeA(lpNamedPipeName: LPCSTR; lpInBuffer: Pointer; nInBufferSize: DWORD; lpOutBuffer: Pointer; nOutBufferSize: DWORD; var lpBytesRead: DWORD; nTimeOut: DWORD): BOOL; external 'kernel32' name 'CallNamedPipeA';
function CallNamedPipeW(lpNamedPipeName: LPWSTR; lpInBuffer: Pointer; nInBufferSize: DWORD; lpOutBuffer: Pointer; nOutBufferSize: DWORD; var lpBytesRead: DWORD; nTimeOut: DWORD): BOOL; external 'kernel32' name 'CallNamedPipeW';
function CoRegisterClassObject(const _para1:TCLSID; _para2:IUnknown; _para3:DWORD; _para4:DWORD; out_para5:DWORD):HRESULT;external 'ole32.dll' name 'CoRegisterClassObject';
function ChangeDisplaySettings(var lpDevMode: TDeviceMode; dwFlags: DWORD): Longint;external 'user32' name 'ChangeDisplaySettingsA';
function ChangeDisplaySettingsA(var lpDevMode: TDeviceModeA; dwFlags: DWORD): Longint; external 'user32' name 'ChangeDisplaySettingsA';
{$ifndef NO_SMART_LINK}
function ChangeDisplaySettingsEx(lpszDeviceName: PChar; var lpDevMode: TDeviceMode; wnd: HWND; dwFlags: DWORD; lParam: Pointer): Longint;external 'user32' name 'ChangeDisplaySettingsExA';
function ChangeDisplaySettingsExA(lpszDeviceName: LPCSTR; var lpDevMode: TDeviceModeA; wnd: HWND; dwFlags: DWORD; lParam: Pointer): Longint;external 'user32' name 'ChangeDisplaySettingsExA';
function ChangeDisplaySettingsExW(lpszDeviceName: LPWSTR; var lpDevMode: TDeviceModeW; wnd: HWND; dwFlags: DWORD; lParam: Pointer): Longint;external 'user32' name 'ChangeDisplaySettingsExW';
{$endif NO_SMART_LINK}
function ChangeDisplaySettingsW(var lpDevMode: TDeviceModeW; dwFlags: DWORD): Longint; external 'user32' name 'ChangeDisplaySettingsW';
//function CheckColorsInGamut(DC: HDC; var RGBQuads, Results; Count: DWORD): BOOL; external 'gdi32' name 'CheckColorsInGamut';
function ChoosePixelFormat(DC:HDC; var pfd:PIXELFORMATDESCRIPTOR):longint; external 'gdi32' name 'ChoosePixelFormat';
function ClearCommError(hFile: THandle; var lpErrors: DWORD; lpStat: PComStat): BOOL; external 'kernel32' name 'ClearCommError';
function ClientToScreen(hWnd: HWND; var lpPoint: TPoint): BOOL; external 'user32' name 'ClientToScreen';
function ClipCursor(var lpRect:RECT):WINBOOL; external 'user32' name 'ClipCursor';
//function CombineTransform(var p1: TXForm; const p2, p3: TXForm): BOOL; external 'gdi32' name 'CombineTransform';
function CommConfigDialog(lpszName: PChar; hWnd: HWND; var lpCC: TCommConfig): BOOL;external 'kernel32' name 'CommConfigDialogA';
function CommConfigDialogA(lpszName: LPCSTR; hWnd: HWND; var lpCC: TCommConfig): BOOL; external 'kernel32' name 'CommConfigDialogA';
function CommConfigDialogW(lpszName: LPWSTR; hWnd: HWND; var lpCC: TCommConfig): BOOL; external 'kernel32' name 'CommConfigDialogW';
//function CompareFileTime(const lpFileTime1, lpFileTime2: TFileTime): Longint; external 'kernel32' name 'CompareFileTime';
//function ConvertToAutoInheritPrivateObjectSecurity(ParentDescriptor, CurrentSecurityDescriptor: PSecurityDescriptor; var NewDescriptor: PSecurityDescriptor; ObjectType: PGUID; IsDirectoryObject: BOOL; const GenericMapping: TGenericMapping): BOOL;
//  external 'advapi32' name 'ConvertToAutoInheritPrivateObjectSecurity';
function CopyAcceleratorTable(hAccelSrc: HACCEL; var lpAccelDst; cAccelEntries: Integer): Integer;external 'user32' name 'CopyAcceleratorTableA';
function CopyAcceleratorTableA(hAccelSrc: HACCEL; var lpAccelDst; cAccelEntries: Integer): Integer; external 'user32' name 'CopyAcceleratorTableA';
function CopyAcceleratorTableW(hAccelSrc: HACCEL; var lpAccelDst; cAccelEntries: Integer): Integer; external 'user32' name 'CopyAcceleratorTableW';
function CopyRect(var lprcDst: TRect; const lprcSrc: TRect): BOOL; external 'user32' name 'CopyRect';
function CreateAcceleratorTable(var Accel; Count: Integer): HACCEL;external 'user32' name 'CreateAcceleratorTableA';
function CreateAcceleratorTableA(var Accel; Count: Integer): HACCEL; external 'user32' name 'CreateAcceleratorTableA';
function CreateAcceleratorTableW(var Accel; Count: Integer): HACCEL; external 'user32' name 'CreateAcceleratorTableW';
//function CreateBitmapIndirect(const p1: TBitmap): HBITMAP; external 'gdi32' name 'CreateBitmapIndirect';
//function CreateBrushIndirect(const p1: TLogBrush): HBRUSH; external 'gdi32' name 'CreateBrushIndirect';
function CreateColorSpace(var ColorSpace: TLogColorSpace): HCOLORSPACE;external 'gdi32' name 'CreateColorSpaceA';
function CreateColorSpaceA(var ColorSpace: TLogColorSpaceA): HCOLORSPACE; external 'gdi32' name 'CreateColorSpaceA';
//function CreateColorSpaceW(var ColorSpace: TLogColorSpaceW): HCOLORSPACE; external 'gdi32' name 'CreateColorSpaceW';
function CreateDialogIndirectParam(hInstance: HINST; const lpTemplate: TDlgTemplate; hWndParent: HWND; lpDialogFunc: TFNDlgProc; dwInitParam: LPARAM): HWND;external 'user32' name 'CreateDialogIndirectParamA';
//function CreateDialogIndirectParamA(hInstance: HINST; const lpTemplate: TDlgTemplate; hWndParent: HWND; lpDialogFunc: TFNDlgProc; dwInitParam: LPARAM): HWND; external 'user32' name 'CreateDialogIndirectParamA';
//function CreateDialogIndirectParamW(hInstance: HINST; const lpTemplate: TDlgTemplate; hWndParent: HWND; lpDialogFunc: TFNDlgProc; dwInitParam: LPARAM): HWND; external 'user32' name 'CreateDialogIndirectParamW';
//function CreateDIBitmap(DC: HDC; var InfoHeader: TBitmapInfoHeader; dwUsage: DWORD; InitBits: PChar; var InitInfo: TBitmapInfo; wUsage: UINT): HBITMAP; external 'gdi32' name 'CreateDIBitmap';
//function CreateDIBPatternBrushPt(const p1: Pointer; p2: UINT): HBRUSH; external 'gdi32' name 'CreateDIBPatternBrushPt';
//function CreateDIBSection(DC: HDC; const p2: TBitmapInfo; p3: UINT; var p4: Pointer; p5: THandle; p6: DWORD): HBITMAP; external 'gdi32' name 'CreateDIBSection';
//function CreateEllipticRgnIndirect(const p1: TRect): HRGN; external 'gdi32' name 'CreateEllipticRgnIndirect';
//function CreateFontIndirect(const p1: TLogFont): HFONT;external 'gdi32' name 'CreateFontIndirectA';
//function CreateFontIndirectA(const p1: TLogFontA): HFONT; external 'gdi32' name 'CreateFontIndirectA';
//function CreateFontIndirectEx(const p1: PEnumLogFontExDV): HFONT;external 'gdi32' name 'CreateFontIndirectExA';
//function CreateFontIndirectExA(const p1: PEnumLogFontExDVA): HFONT;external 'gdi32' name 'CreateFontIndirectExA';
//function CreateFontIndirectExW(const p1: PEnumLogFontExDVW): HFONT;external 'gdi32' name 'CreateFontIndirectExW';
//function CreateFontIndirectW(const p1: TLogFontW): HFONT; external 'gdi32' name 'CreateFontIndirectW';
function CreateIconIndirect(var piconinfo: TIconInfo): HICON; external 'user32' name 'CreateIconIndirect';
//function CreatePalette(const LogPalette: TLogPalette): HPalette; external 'gdi32' name 'CreatePalette';
//function CreatePenIndirect(const LogPen: TLogPen): HPEN; external 'gdi32' name 'CreatePenIndirect';
function CreatePipe(var hReadPipe, hWritePipe: THandle; lpPipeAttributes: PSecurityAttributes; nSize: DWORD): BOOL; external 'kernel32' name 'CreatePipe';
function CreatePolygonRgn(const Points; Count, FillMode: Integer): HRGN; external 'gdi32' name 'CreatePolygonRgn';
function CreatePolyPolygonRgn(const pPtStructs; const pIntArray; p3, p4: Integer): HRGN; external 'gdi32' name 'CreatePolyPolygonRgn';
//function CreatePrivateObjectSecurity(ParentDescriptor, CreatorDescriptor: PSecurityDescriptor; var NewDescriptor: PSecurityDescriptor; IsDirectoryObject: BOOL; Token: THandle; const GenericMapping: TGenericMapping): BOOL;
//  external 'advapi32' name 'CreatePrivateObjectSecurity';
//function CreatePrivateObjectSecurityEx(ParentDescriptor, CreatorDescriptor: PSecurityDescriptor; var NewDescriptor: PSecurityDescriptor; ObjectType: PGUID; IsContainerObject: BOOL; AutoInheritFlags: ULONG; Token: THandle;
//  const GenericMapping: TGenericMapping): BOOL;external 'advapi32' name 'CreatePrivateObjectSecurityEx';
function CreateProcess(lpApplicationName: PChar; lpCommandLine: PChar; lpProcessAttributes, lpThreadAttributes: PSecurityAttributes; bInheritHandles: BOOL; dwCreationFlags: DWORD; lpEnvironment: Pointer; lpCurrentDirectory: PChar;
  const lpStartupInfo: TStartupInfo; var lpProcessInformation: TProcessInformation): BOOL;external 'kernel32' name 'CreateProcessA';
function CreateProcessA(lpApplicationName: LPCSTR; lpCommandLine: LPCSTR; lpProcessAttributes, lpThreadAttributes: PSecurityAttributes; bInheritHandles: BOOL; dwCreationFlags: DWORD; lpEnvironment: Pointer; lpCurrentDirectory: LPCSTR;
  const lpStartupInfo: TStartupInfoA; var lpProcessInformation: TProcessInformation): BOOL; external 'kernel32' name 'CreateProcessA';
//function CreateProcessAsUser(hToken: THandle; lpApplicationName: PChar; lpCommandLine: PChar; lpProcessAttributes: PSecurityAttributes; lpThreadAttributes: PSecurityAttributes; bInheritHandles: BOOL; dwCreationFlags: DWORD;
// lpEnvironment: Pointer; lpCurrentDirectory: PChar; const lpStartupInfo: TStartupInfo; var lpProcessInformation: TProcessInformation): BOOL;external 'advapi32' name 'CreateProcessAsUserA';
//function CreateProcessAsUserA(hToken: THandle; lpApplicationName: LPCSTR; lpCommandLine: LPCSTR; lpProcessAttributes: PSecurityAttributes; lpThreadAttributes: PSecurityAttributes; bInheritHandles: BOOL; dwCreationFlags: DWORD;
//  lpEnvironment: Pointer; lpCurrentDirectory: LPCSTR; const lpStartupInfo: TStartupInfo; var lpProcessInformation: TProcessInformation): BOOL; external 'advapi32' name 'CreateProcessAsUserA';
//function CreateProcessAsUserW(hToken: THandle; lpApplicationName: LPWSTR; lpCommandLine: LPWSTR; lpProcessAttributes: PSecurityAttributes; lpThreadAttributes: PSecurityAttributes; bInheritHandles: BOOL; dwCreationFlags: DWORD;
//  lpEnvironment: Pointer; lpCurrentDirectory: LPWSTR; const lpStartupInfo: TStartupInfo; var lpProcessInformation: TProcessInformation): BOOL; external 'advapi32' name 'CreateProcessAsUserW';
function CreateProcessW(lpApplicationName: LPWSTR; lpCommandLine: LPWSTR; lpProcessAttributes, lpThreadAttributes: PSecurityAttributes; bInheritHandles: BOOL; dwCreationFlags: DWORD; lpEnvironment: Pointer; lpCurrentDirectory: LPWSTR;
  const lpStartupInfo: TStartupInfoW; var lpProcessInformation: TProcessInformation): BOOL; external 'kernel32' name 'CreateProcessW';
//function CreateRectRgnIndirect(const p1: TRect): HRGN; external 'gdi32' name 'CreateRectRgnIndirect';
function CreateRemoteThread(hProcess: THandle; lpThreadAttributes: Pointer; dwStackSize: PTRUINT; lpStartAddress: TFNThreadStartRoutine; lpParameter: Pointer; dwCreationFlags: DWORD; var lpThreadId: DWORD): THandle;
  external 'kernel32' name 'CreateRemoteThread';
function CreateThread(lpThreadAttributes: Pointer; dwStackSize: PTRUINT; lpStartAddress: TFNThreadStartRoutine; lpParameter: Pointer; dwCreationFlags: DWORD; var lpThreadId: DWORD): THandle; external 'kernel32' name 'CreateThread';
function DdeSetQualityOfService(hWndClient: HWnd; const pqosNew: TSecurityQualityOfService; pqosPrev: PSecurityQualityOfService): BOOL;external 'user32' name 'DdeSetQualityOfService';
function DdeConnect(_para1:DWORD; _para2:HSZ; _para3:HSZ; var _para4:CONVCONTEXT):HCONV; external 'user32' name 'DdeConnect';
//function DeleteAce(var pAcl: TACL; dwAceIndex: DWORD): BOOL; external 'advapi32' name 'DeleteAce';
function DescribePixelFormat(DC: HDC; iPixelFormat: Integer; nBytes: UINT; var pfd: TPixelFormatDescriptor): BOOL; external 'gdi32' name 'DescribePixelFormat';
//function DestroyPrivateObjectSecurity(var ObjectDescriptor: PSecurityDescriptor): BOOL; external 'advapi32' name 'DestroyPrivateObjectSecurity';
function DeviceIoControl(hDevice: THandle; dwIoControlCode: DWORD; lpInBuffer: Pointer; nInBufferSize: DWORD; lpOutBuffer: Pointer; nOutBufferSize: DWORD; var lpBytesReturned: DWORD; lpOverlapped: POverlapped): BOOL;
  external 'kernel32' name 'DeviceIoControl';
function DialogBoxIndirectParam(hInstance: HINST; const lpDialogTemplate: TDlgTemplate; hWndParent: HWND; lpDialogFunc: TFNDlgProc; dwInitParam: LPARAM): Integer;external 'user32' name 'DialogBoxIndirectParamA';
function DialogBoxIndirectParamA(hInstance: HINST; const lpDialogTemplate: TDlgTemplate; hWndParent: HWND; lpDialogFunc: TFNDlgProc; dwInitParam: LPARAM): Integer; external 'user32' name 'DialogBoxIndirectParamA';
function DialogBoxIndirectParamW(hInstance: HINST; const lpDialogTemplate: TDlgTemplate; hWndParent: HWND; lpDialogFunc: TFNDlgProc; dwInitParam: LPARAM): Integer; external 'user32' name 'DialogBoxIndirectParamW';
function DispatchMessage(const lpMsg: TMsg): Longint;external 'user32' name 'DispatchMessageA';
function DispatchMessageA(const lpMsg: TMsg): Longint; external 'user32' name 'DispatchMessageA';
function DispatchMessageW(const lpMsg: TMsg): Longint; external 'user32' name 'DispatchMessageW';
function DosDateTimeToFileTime(wFatDate, wFatTime: Word; var lpFileTime: TFileTime): BOOL; external 'kernel32' name 'DosDateTimeToFileTime';
function DPtoLP(DC: HDC; var Points; Count: Integer): BOOL; external 'gdi32' name 'DPtoLP';
// function DrawAnimatedRects(hwnd: HWND; idAni: Integer; const lprcFrom, lprcTo: TRect): BOOL; external 'user32' name 'DrawAnimatedRects';
//function DrawCaption(p1: HWND; p2: HDC; const p3: TRect; p4: UINT): BOOL; external 'user32' name 'DrawCaption';
function DrawEdge(hdc: HDC; var qrc: TRect; edge: UINT; grfFlags: UINT): BOOL; external 'user32' name 'DrawEdge';
//function DrawFocusRect(hDC: HDC; const lprc: TRect): BOOL; external 'user32' name 'DrawFocusRect';
function DrawFrameControl(DC: HDC; const Rect: TRect; uType, uState: UINT): BOOL; external 'user32' name 'DrawFrameControl';
function DrawText(hDC: HDC; lpString: PChar; nCount: Integer; var lpRect: TRect; uFormat: UINT): Integer;external 'user32' name 'DrawTextA';
function DrawTextA(hDC: HDC; lpString: LPCSTR; nCount: Integer; var lpRect: TRect; uFormat: UINT): Integer; external 'user32' name 'DrawTextA';
function DrawTextEx(DC: HDC; lpchText: PChar; cchText: Integer; var p4: TRect; dwDTFormat: UINT; DTParams: PDrawTextParams): Integer;external 'user32' name 'DrawTextExA';
function DrawTextExA(DC: HDC; lpchText: LPCSTR; cchText: Integer; var p4: TRect; dwDTFormat: UINT; DTParams: PDrawTextParams): Integer; external 'user32' name 'DrawTextExA';
function DrawTextExW(DC: HDC; lpchText: LPWSTR; cchText: Integer; var p4: TRect; dwDTFormat: UINT; DTParams: PDrawTextParams): Integer; external 'user32' name 'DrawTextExW';
function DrawTextW(hDC: HDC; lpString: LPWSTR; nCount: Integer; var lpRect: TRect; uFormat: UINT): Integer; external 'user32' name 'DrawTextW';
//function DuplicateTokenEx(hExistingToken: THandle; dwDesiredAccess: DWORD; lpTokenAttributes: PSecurityAttributes; ImpersonationLevel: TSecurityImpersonationLevel; TokenType: TTokenType; var phNewToken: THandle): BOOL;
//  external 'advapi32' name 'DuplicateTokenEx';
function EndPaint(hWnd: HWND; const lpPaint: TPaintStruct): BOOL; external 'user32' name 'EndPaint';
//function EnumDisplayDevices(Unused: Pointer; iDevNum: DWORD; var lpDisplayDevice: TDisplayDevice; dwFlags: DWORD): BOOL;external 'user32' name 'EnumDisplayDevicesA';
//function EnumDisplayDevicesA(Unused: Pointer; iDevNum: DWORD; var lpDisplayDevice: TDisplayDeviceA; dwFlags: DWORD): BOOL;external 'user32' name 'EnumDisplayDevicesA';
//function EnumDisplayDevicesW(Unused: Pointer; iDevNum: DWORD; var lpDisplayDevice: TDisplayDeviceW; dwFlags: DWORD): BOOL;external 'user32' name 'EnumDisplayDevicesW';
function EnumDisplaySettings(lpszDeviceName: PChar; iModeNum: DWORD; var lpDevMode: TDeviceMode): BOOL;external 'user32' name 'EnumDisplaySettingsA';
function EnumDisplaySettingsA(lpszDeviceName: LPCSTR; iModeNum: DWORD; var lpDevMode: TDeviceModeA): BOOL; external 'user32' name 'EnumDisplaySettingsA';
function EnumDisplaySettingsW(lpszDeviceName: LPWSTR; iModeNum: DWORD; var lpDevMode: TDeviceModeW): BOOL; external 'user32' name 'EnumDisplaySettingsW';
//function EnumEnhMetaFile(DC: HDC; p2: HENHMETAFILE; p3: TFNEnhMFEnumProc; p4: Pointer; const p5: TRect): BOOL; external 'gdi32' name 'EnumEnhMetaFile';
function EnumFontFamiliesEx(DC: HDC; var p2: TLogFont; p3: FONTENUMEXPROC; p4: LPARAM; p5: DWORD): BOOL;external 'gdi32' name 'EnumFontFamiliesExA';
function EnumFontFamiliesExA(DC: HDC; var p2: TLogFontA; p3: FONTENUMEXPROCA; p4: LPARAM; p5: DWORD): BOOL; external 'gdi32' name 'EnumFontFamiliesExA';
function EnumFontFamiliesExW(DC: HDC; var p2: TLogFontW; p3: FONTENUMEXPROCW; p4: LPARAM; p5: DWORD): BOOL; external 'gdi32' name 'EnumFontFamiliesExW';
//function EqualRect(const lprc1, lprc2: TRect): BOOL; external 'user32' name 'EqualRect';
function ExtCreatePen(PenStyle, Width: DWORD; const Brush: TLogBrush; StyleCount: DWORD; Style: Pointer): HPEN; external 'gdi32' name 'ExtCreatePen';
function ExtCreateRegion(p1: PXForm; p2: DWORD; const p3: TRgnData): HRGN; external 'gdi32' name 'ExtCreateRegion';
// function ExtEscape(DC: HDC; p2, p3: Integer; const p4: LPCSTR; p5: Integer; p6: LPSTR): Integer; external 'gdi32' name 'ExtEscape';
function FileTimeToDosDateTime(const lpFileTime: TFileTime; var lpFatDate, lpFatTime: Word): BOOL; external 'kernel32' name 'FileTimeToDosDateTime';
function FileTimeToLocalFileTime(const lpFileTime: TFileTime; var lpLocalFileTime: TFileTime): BOOL; external 'kernel32' name 'FileTimeToLocalFileTime';
function FileTimeToSystemTime(const lpFileTime: TFileTime; var lpSystemTime: TSystemTime): BOOL; external 'kernel32' name 'FileTimeToSystemTime';
function FillConsoleOutputAttribute(hConsoleOutput: THandle; wAttribute: Word; nLength: DWORD; dwWriteCoord: TCoord; var lpNumberOfAttrsWritten: DWORD): BOOL; external 'kernel32' name 'FillConsoleOutputAttribute';
function FillConsoleOutputCharacter(hConsoleOutput: THandle; cCharacter: Char; nLength: DWORD; dwWriteCoord: TCoord; var lpNumberOfCharsWritten: DWORD): BOOL;external 'kernel32' name 'FillConsoleOutputCharacterA';
function FillConsoleOutputCharacterA(hConsoleOutput: THandle; cCharacter: AnsiChar; nLength: DWORD; dwWriteCoord: TCoord; var lpNumberOfCharsWritten: DWORD): BOOL; external 'kernel32' name 'FillConsoleOutputCharacterA';
function FillConsoleOutputCharacterW(hConsoleOutput: THandle; cCharacter: WideChar; nLength: DWORD; dwWriteCoord: TCoord; var lpNumberOfCharsWritten: DWORD): BOOL; external 'kernel32' name 'FillConsoleOutputCharacterW';
//function FillRect(hDC: HDC; const lprc: TRect; hbr: HBRUSH): Integer; external 'user32' name 'FillRect';
function FindFirstFile(lpFileName: PChar; var lpFindFileData: TWIN32FindData): THandle;external 'kernel32' name 'FindFirstFileA';
function FindFirstFileA(lpFileName: LPCSTR; var lpFindFileData: TWIN32FindDataA): THandle; external 'kernel32' name 'FindFirstFileA';
function FindFirstFileW(lpFileName: LPWSTR; var lpFindFileData: TWIN32FindDataW): THandle; external 'kernel32' name 'FindFirstFileW';
//function FindFirstFreeAce(var pAcl: TACL; var pAce: Pointer): BOOL; external 'advapi32' name 'FindFirstFreeAce';
function FindNextFile(hFindFile: THandle; var lpFindFileData: TWIN32FindData): BOOL;external 'kernel32' name 'FindNextFileA';
function FindNextFileA(hFindFile: THandle; var lpFindFileData: TWIN32FindDataA): BOOL; external 'kernel32' name 'FindNextFileA';
function FindNextFileW(hFindFile: THandle; var lpFindFileData: TWIN32FindDataW): BOOL; external 'kernel32' name 'FindNextFileW';
//function FlushInstructionCache(hProcess: THandle; const lpBaseAddress: Pointer; dwSize: PTRUINT): BOOL; external 'kernel32' name 'FlushInstructionCache';
//function FlushViewOfFile(const lpBaseAddress: Pointer; dwNumberOfBytesToFlush: DWORD): BOOL; external 'kernel32' name 'FlushViewOfFile';
//function FrameRect(hDC: HDC; const lprc: TRect; hbr: HBRUSH): Integer; external 'user32' name 'FrameRect';
//function GetAce(const pAcl: TACL; dwAceIndex: DWORD; var pAce: Pointer): BOOL; external 'advapi32' name 'GetAce';
//function GetAclInformation(const pAcl: TACL; pAclInformation: Pointer; nAclInformationLength: DWORD; dwAclInformationClass: TAclInformationClass): BOOL; external 'advapi32' name 'GetAclInformation';
//function GetAltTabInfo(hwnd: HWND; iItem: Integer; var pati: TAltTabInfo; pszItemText: PChar; cchItemText: UINT): BOOL;external 'user32' name 'GetAltTabInfoA';
//function GetAltTabInfoA(hwnd: HWND; iItem: Integer; var pati: TAltTabInfo; pszItemText: LPCSTR; cchItemText: UINT): BOOL;external 'user32' name 'GetAltTabInfoA';
//function GetAltTabInfoW(hwnd: HWND; iItem: Integer; var pati: TAltTabInfo; pszItemText: LPWSTR; cchItemText: UINT): BOOL;external 'user32' name 'GetAltTabInfoW';
function GetAspectRatioFilterEx(DC: HDC; var p2: TSize): BOOL; external 'gdi32' name 'GetAspectRatioFilterEx';
function GetBinaryType(lpApplicationName: PChar; var lpBinaryType: DWORD): BOOL;external 'kernel32' name 'GetBinaryTypeA';
function GetBinaryTypeA(lpApplicationName: LPCSTR; var lpBinaryType: DWORD): BOOL; external 'kernel32' name 'GetBinaryTypeA';
function GetBinaryTypeW(lpApplicationName: LPWSTR; var lpBinaryType: DWORD): BOOL; external 'kernel32' name 'GetBinaryTypeW';
function GetBitmapDimensionEx(p1: HBITMAP; var p2: TSize): BOOL; external 'gdi32' name 'GetBitmapDimensionEx';
function GetBoundsRect(DC: HDC; var p2: TRect; p3: UINT): UINT; external 'gdi32' name 'GetBoundsRect';
function GetBrushOrgEx(DC: HDC; var p2: TPoint): BOOL; external 'gdi32' name 'GetBrushOrgEx';
function GetCaretPos(var lpPoint: TPoint): BOOL; external 'user32' name 'GetCaretPos';
function GetCharABCWidths(DC: HDC; p2, p3: UINT; const ABCStructs): BOOL;external 'gdi32' name 'GetCharABCWidthsA';
function GetCharABCWidthsA(DC: HDC; p2, p3: UINT; const ABCStructs): BOOL; external 'gdi32' name 'GetCharABCWidthsA';
function GetCharABCWidthsFloat(DC: HDC; p2, p3: UINT; const ABCFloatSturcts): BOOL;external 'gdi32' name 'GetCharABCWidthsFloatA';
function GetCharABCWidthsFloatA(DC: HDC; p2, p3: UINT; const ABCFloatSturcts): BOOL; external 'gdi32' name 'GetCharABCWidthsFloatA';
function GetCharABCWidthsFloatW(DC: HDC; p2, p3: UINT; const ABCFloatSturcts): BOOL; external 'gdi32' name 'GetCharABCWidthsFloatW';
//function GetCharABCWidthsI(DC: HDC; p2, p3: UINT; p4: PWORD; const Widths): BOOL;external 'gdi32' name 'GetCharABCWidthsI';
function GetCharABCWidthsW(DC: HDC; p2, p3: UINT; const ABCStructs): BOOL; external 'gdi32' name 'GetCharABCWidthsW';
function GetCharacterPlacement(DC: HDC; p2: PChar; p3, p4: BOOL; var p5: TGCPResults; p6: DWORD): DWORD;external 'gdi32' name 'GetCharacterPlacementA';
function GetCharacterPlacementA(DC: HDC; p2: LPCSTR; p3, p4: BOOL; var p5: TGCPResults; p6: DWORD): DWORD; external 'gdi32' name 'GetCharacterPlacementA';
function GetCharacterPlacementW(DC: HDC; p2: LPWSTR; p3, p4: BOOL; var p5: TGCPResults; p6: DWORD): DWORD; external 'gdi32' name 'GetCharacterPlacementW';
function GetCharWidth(DC: HDC; p2, p3: UINT; const Widths): BOOL;external 'gdi32' name 'GetCharWidthA';
function GetCharWidth32(DC: HDC; p2, p3: UINT; const Widths): BOOL;external 'gdi32' name 'GetCharWidth32A';
function GetCharWidth32A(DC: HDC; p2, p3: UINT; const Widths): BOOL; external 'gdi32' name 'GetCharWidth32A';
function GetCharWidth32W(DC: HDC; p2, p3: UINT; const Widths): BOOL; external 'gdi32' name 'GetCharWidth32W';
function GetCharWidthA(DC: HDC; p2, p3: UINT; const Widths): BOOL; external 'gdi32' name 'GetCharWidthA';
function GetCharWidthFloat(DC: HDC; p2, p3: UINT; const Widths): BOOL;external 'gdi32' name 'GetCharWidthFloatA';
function GetCharWidthFloatA(DC: HDC; p2, p3: UINT; const Widths): BOOL; external 'gdi32' name 'GetCharWidthFloatA';
function GetCharWidthFloatW(DC: HDC; p2, p3: UINT; const Widths): BOOL; external 'gdi32' name 'GetCharWidthFloatW';
//function GetCharWidthI(DC: HDC; p2, p3: UINT; p4: PWORD; const Widths): BOOL;external 'gdi32' name 'GetCharWidthI';
function GetCharWidthW(DC: HDC; p2, p3: UINT; const Widths): BOOL; external 'gdi32' name 'GetCharWidthW';
function GetClassInfo(hInstance: HINST; lpClassName: PChar; var lpWndClass: TWndClass): BOOL;external 'user32' name 'GetClassInfoA';
function GetClassInfoA(hInstance: HINST; lpClassName: LPCSTR; var lpWndClass: TWndClassA): BOOL; external 'user32' name 'GetClassInfoA';
function GetClassInfoEx(Instance: HINST; Classname: PChar; var WndClass: TWndClassEx): BOOL;external 'user32' name 'GetClassInfoExA';
//function GetClassInfoExA(Instance: HINST; Classname: LPCSTR; var WndClass: TWndClassExA): BOOL; external 'user32' name 'GetClassInfoExA';
//function GetClassInfoExW(Instance: HINST; Classname: LPWSTR; var WndClass: TWndClassExW): BOOL; external 'user32' name 'GetClassInfoExW';
//function GetClassInfoW(hInstance: HINST; lpClassName: LPWSTR; var lpWndClass: TWndClassW): BOOL; external 'user32' name 'GetClassInfoW';
function GetClientRect(hWnd: HWND; var lpRect: TRect): BOOL; external 'user32' name 'GetClientRect';
function GetClipBox(DC: HDC; var Rect: TRect): Integer; external 'gdi32' name 'GetClipBox';
function GetClipCursor(var lpRect: TRect): BOOL; external 'user32' name 'GetClipCursor';
function GetColorAdjustment(DC: HDC; var p2: TColorAdjustment): BOOL; external 'gdi32' name 'GetColorAdjustment';
function GetCommConfig(hCommDev: THandle; var lpCC: TCommConfig; var lpdwSize: DWORD): BOOL; external 'kernel32' name 'GetCommConfig';
function GetCommMask(hFile: THandle; var lpEvtMask: DWORD): BOOL; external 'kernel32' name 'GetCommMask';
function GetCommModemStatus(hFile: THandle; var lpModemStat: DWORD): BOOL; external 'kernel32' name 'GetCommModemStatus';
function GetCommProperties(hFile: THandle; var lpCommProp: TCommProp): BOOL; external 'kernel32' name 'GetCommProperties';
function GetCommState(hFile: THandle; var lpDCB: TDCB): BOOL; external 'kernel32' name 'GetCommState';
function GetCommTimeouts(hFile: THandle; var lpCommTimeouts: TCommTimeouts): BOOL; external 'kernel32' name 'GetCommTimeouts';
function GetComputerName(lpBuffer: PChar; var nSize: DWORD): BOOL;external 'kernel32' name 'GetComputerNameA';
function GetComputerNameA(lpBuffer: LPCSTR; var nSize: DWORD): BOOL; external 'kernel32' name 'GetComputerNameA';
function GetComputerNameW(lpBuffer: LPWSTR; var nSize: DWORD): BOOL; external 'kernel32' name 'GetComputerNameW';
function GetConsoleCursorInfo(hConsoleOutput: THandle; var lpConsoleCursorInfo: TConsoleCursorInfo): BOOL; external 'kernel32' name 'GetConsoleCursorInfo';
function GetConsoleMode(hConsoleHandle: THandle; var lpMode: DWORD): BOOL; external 'kernel32' name 'GetConsoleMode';
function GetConsoleScreenBufferInfo(hConsoleOutput: THandle; var lpConsoleScreenBufferInfo: TConsoleScreenBufferInfo): BOOL; external 'kernel32' name 'GetConsoleScreenBufferInfo';
function GetCPInfo(CodePage: UINT; var lpCPInfo: TCPInfo): BOOL;external 'kernel32' name 'GetCPInfo';
function GetCPInfoExA(Codepage:UINT; dwFlags:DWORD; var CPInfoEx:TCPINFOEXA):WINBOOL; external 'kernel32' name 'GetCPInfoExA';
function GetCPInfoExW(Codepage:UINT; dwFlags:DWORD; var CPinfoEx:TCPINFOEXW):WINBOOL; external 'kernel32' name 'GetCPInfoExW';
{$ifndef UNICODE}
function GetCPInfoEx(Codepage:UINT; dwFlags:DWORD; var CPInfoEx:TCPINFOEX):WINBOOL; external 'kernel32' name 'GetCPInfoExA';
{$ELSE}
function GetCPInfoEx(Codepage:UINT; dwFlags:DWORD; var CPinfoEx:TCPINFOEX):WINBOOL; external 'kernel32' name 'GetCPInfoExW';
{$ENDIF}
//function GetCurrentHwProfile(var lpHwProfileInfo: THWProfileInfo): BOOL;external 'advapi32' name 'GetCurrentHwProfileA';
//function GetCurrentHwProfileA(var lpHwProfileInfo: THWProfileInfoA): BOOL;external 'advapi32' name 'GetCurrentHwProfileA';
//function GetCurrentHwProfileW(var lpHwProfileInfo: THWProfileInfoW): BOOL;external 'advapi32' name 'GetCurrentHwProfileW';
{$ifndef NO_SMART_LINK}
function GetCursorInfo(var pci: TCursorInfo): BOOL;external 'user32' name 'GetCursorInfo';
{$endif NO_SMART_LINK}
function GetCursorPos(out lpPoint: TPoint): BOOL; external 'user32' name 'GetCursorPos';
function GetDCOrgEx(DC: HDC; var Origin: TPoint): BOOL; external 'gdi32' name 'GetDCOrgEx';
function GetDefaultCommConfig(lpszName: PChar; var lpCC: TCommConfig; var lpdwSize: DWORD): BOOL;external 'kernel32' name 'GetDefaultCommConfigA';
function GetDefaultCommConfigA(lpszName: LPCSTR; var lpCC: TCommConfig; var lpdwSize: DWORD): BOOL; external 'kernel32' name 'GetDefaultCommConfigA';
function GetDefaultCommConfigW(lpszName: LPWSTR; var lpCC: TCommConfig; var lpdwSize: DWORD): BOOL; external 'kernel32' name 'GetDefaultCommConfigW';
function GetDeviceGammaRamp(DC: HDC; var Ramp): BOOL; external 'gdi32' name 'GetDeviceGammaRamp';
function GetDIBColorTable(DC: HDC; p2, p3: UINT; var RGBQuadStructs): UINT; external 'gdi32' name 'GetDIBColorTable';
function GetDIBits(DC: HDC; Bitmap: HBitmap; StartScan, NumScans: UINT; Bits: Pointer; var BitInfo: TBitmapInfo; Usage: UINT): Integer; external 'gdi32' name 'GetDIBits';
function GetDiskFreeSpace(lpRootPathName: PChar; var lpSectorsPerCluster, lpBytesPerSector, lpNumberOfFreeClusters, lpTotalNumberOfClusters: DWORD): BOOL;external 'kernel32' name 'GetDiskFreeSpaceA';
function GetDiskFreeSpaceA(lpRootPathName: LPCSTR; var lpSectorsPerCluster, lpBytesPerSector, lpNumberOfFreeClusters, lpTotalNumberOfClusters: DWORD): BOOL; external 'kernel32' name 'GetDiskFreeSpaceA';
function GetDiskFreeSpaceEx(lpDirectoryName: PChar; var lpFreeBytesAvailableToCaller, lpTotalNumberOfBytes: TLargeInteger; lpTotalNumberOfFreeBytes: PLargeInteger): BOOL;external 'kernel32' name 'GetDiskFreeSpaceExA';
function GetDiskFreeSpaceExA(lpDirectoryName: LPCSTR; var lpFreeBytesAvailableToCaller, lpTotalNumberOfBytes: TLargeInteger; lpTotalNumberOfFreeBytes: PLargeInteger): BOOL;external 'kernel32' name 'GetDiskFreeSpaceExA';
function GetDiskFreeSpaceExW(lpDirectoryName: LPWSTR; var lpFreeBytesAvailableToCaller, lpTotalNumberOfBytes: TLargeInteger; lpTotalNumberOfFreeBytes: PLargeInteger): BOOL;external 'kernel32' name 'GetDiskFreeSpaceExW';
function GetDiskFreeSpaceW(lpRootPathName: LPWSTR; var lpSectorsPerCluster, lpBytesPerSector, lpNumberOfFreeClusters, lpTotalNumberOfClusters: DWORD): BOOL; external 'kernel32' name 'GetDiskFreeSpaceW';
function GetDiskFreeSpaceEx(lpDirectoryName: PChar;  lpFreeBytesAvailableToCaller, lpTotalNumberOfBytes:pLargeInteger; lpTotalNumberOfFreeBytes: PLargeInteger): BOOL;external 'kernel32' name 'GetDiskFreeSpaceExA';
function GetDiskFreeSpaceExA(lpDirectoryName: LPCSTR;  lpFreeBytesAvailableToCaller, lpTotalNumberOfBytes: pLargeInteger; lpTotalNumberOfFreeBytes: PLargeInteger): BOOL;external 'kernel32' name 'GetDiskFreeSpaceExA';
function GetDiskFreeSpaceExW(lpDirectoryName: LPWSTR;  lpFreeBytesAvailableToCaller, lpTotalNumberOfBytes: pLargeInteger; lpTotalNumberOfFreeBytes: PLargeInteger): BOOL;external 'kernel32' name 'GetDiskFreeSpaceExW';
//function GetEnhMetaFilePixelFormat(p1: HENHMETAFILE; p2: Cardinal; var p3: TPixelFormatDescriptor): UINT;external 'gdi32' name 'GetEnhMetaFilePixelFormat';
function GetExitCodeProcess(hProcess: THandle; var lpExitCode: DWORD): BOOL; external 'kernel32' name 'GetExitCodeProcess';
function GetExitCodeThread(hThread: THandle; var lpExitCode: DWORD): BOOL; external 'kernel32' name 'GetExitCodeThread';
function GetFileInformationByHandle(hFile: THandle; var lpFileInformation: TByHandleFileInformation): BOOL; external 'kernel32' name 'GetFileInformationByHandle';
//function GetFileSecurity(lpFileName: PChar; RequestedInformation: SECURITY_INFORMATION; pSecurityDescriptor: PSecurityDescriptor; nLength: DWORD; var lpnLengthNeeded: DWORD): BOOL;external 'advapi32' name 'GetFileSecurityA';
//function GetFileSecurityA(lpFileName: LPCSTR; RequestedInformation: SECURITY_INFORMATION; pSecurityDescriptor: PSecurityDescriptor; nLength: DWORD; var lpnLengthNeeded: DWORD): BOOL; external 'advapi32' name 'GetFileSecurityA';
//function GetFileSecurityW(lpFileName: LPWSTR; RequestedInformation: SECURITY_INFORMATION; pSecurityDescriptor: PSecurityDescriptor; nLength: DWORD; var lpnLengthNeeded: DWORD): BOOL; external 'advapi32' name 'GetFileSecurityW';
function GetFileVersionInfoSize(lptstrFilename: PChar; var lpdwHandle: DWORD): DWORD;external 'version' name 'GetFileVersionInfoSizeA';
function GetFileVersionInfoSizeA(lptstrFilename: LPCSTR; var lpdwHandle: DWORD): DWORD; external 'version' name 'GetFileVersionInfoSizeA';
function GetFileVersionInfoSizeW(lptstrFilename: LPWSTR; var lpdwHandle: DWORD): DWORD; external 'version' name 'GetFileVersionInfoSizeW';

{$ifdef Unicode}
 function GetFullPathName (lpFileName: LPCWSTR; nBufferLength: DWORD; lpBuffer: LPWSTR; var lpFilePart: LPWSTR): DWORD; external 'kernel32' name 'GetFullPathNameW';
{$else}
 function GetFullPathName (lpFileName: LPCSTR;  nBufferLength: DWORD; lpBuffer: LPSTR;  var lpFilePart: LPSTR):  DWORD; external 'kernel32' name 'GetFullPathNameA';
{$endif}
 function GetFullPathNameW(lpFileName: LPCWSTR; nBufferLength: DWORD; lpBuffer: LPWSTR; var lpFilePart: LPWSTR): DWORD; external 'kernel32' name 'GetFullPathNameW';
 function GetFullPathNameA(lpFileName: LPCSTR;  nBufferLength: DWORD; lpBuffer: LPSTR;  var lpFilePart: LPSTR):  DWORD; external 'kernel32' name 'GetFullPathNameA';

function GetGlyphOutline(DC: HDC; p2, p3: UINT; const p4: TGlyphMetrics; p5: DWORD; p6: Pointer; const p7: TMat2): DWORD;external 'gdi32' name 'GetGlyphOutlineA';
function GetGlyphOutlineA(DC: HDC; p2, p3: UINT; const p4: TGlyphMetrics; p5: DWORD; p6: Pointer; const p7: TMat2): DWORD; external 'gdi32' name 'GetGlyphOutlineA';
function GetGlyphOutlineW(DC: HDC; p2, p3: UINT; const p4: TGlyphMetrics; p5: DWORD; p6: Pointer; const p7: TMat2): DWORD; external 'gdi32' name 'GetGlyphOutlineW';
//function GetGUIThreadInfo(idThread: DWORD; var pgui: TGUIThreadinfo): BOOL;external 'user32' name 'GetGUIThreadInfo';
function GetHandleInformation(hObject: THandle; var lpdwFlags: DWORD): BOOL; external 'kernel32' name 'GetHandleInformation';
//function GetICMProfile(DC: HDC; var Size: DWORD; Name: PChar): BOOL;external 'gdi32' name 'GetICMProfileA';
//function GetICMProfileA(DC: HDC; var Size: DWORD; Name: LPCSTR): BOOL; external 'gdi32' name 'GetICMProfileA';
//function GetICMProfileW(DC: HDC; var Size: DWORD; Name: LPWSTR): BOOL; external 'gdi32' name 'GetICMProfileW';
function GetIconInfo(hIcon: HICON; var piconinfo: TIconInfo): BOOL; external 'user32' name 'GetIconInfo';
//function GetKernelObjectSecurity(Handle: THandle; RequestedInformation: SECURITY_INFORMATION; pSecurityDescriptor: PSecurityDescriptor; nLength: DWORD; var lpnLengthNeeded: DWORD): BOOL; external 'advapi32' name 'GetKernelObjectSecurity';
function GetKerningPairs(DC: HDC; Count: DWORD; var KerningPairs): DWORD;external 'gdi32' name 'GetKerningPairs';
function GetKeyboardLayoutList(nBuff: Integer; var List): UINT; external 'user32' name 'GetKeyboardLayoutList';
//function GetKeyboardState(var KeyState: TKeyboardState): BOOL; external 'user32' name 'GetKeyboardState';
//function GetLastInputInfo(var plii: TLastInputInfo): BOOL;external 'user32' name 'GetLastInputInfo';
procedure GetSystemTime(var lpSystemTime:SYSTEMTIME); external 'kernel32' name 'GetSystemTime';
procedure GetLocalTime(var SystemTime: SYSTEMTIME); external 'kernel32' name 'GetLocalTime';
procedure GetSystemInfo(var SystemInfo:SYSTEM_INFO); external 'kernel32' name 'GetSystemInfo';
function SetSystemTime(var lpSystemTime:SYSTEMTIME):WINBOOL; external 'kernel32' name 'SetSystemTime';
function SetLocalTime(var lpSystemTime:SYSTEMTIME):WINBOOL; external 'kernel32' name 'SetLocalTime';
function GetLogColorSpace(p1: HCOLORSPACE; var ColorSpace: TLogColorSpace; Size: DWORD): BOOL;external 'gdi32' name 'GetLogColorSpaceA';
function GetLogColorSpaceA(p1: HCOLORSPACE; var ColorSpace: TLogColorSpaceA; Size: DWORD): BOOL; external 'gdi32' name 'GetLogColorSpaceA';
//function GetLogColorSpaceW(p1: HCOLORSPACE; var ColorSpace: TLogColorSpaceW; Size: DWORD): BOOL; external 'gdi32' name 'GetLogColorSpaceW';
function GetMailslotInfo(hMailslot: THandle; lpMaxMessageSize: Pointer; var lpNextSize: DWORD; lpMessageCount, lpReadTimeout: Pointer): BOOL; external 'kernel32' name 'GetMailslotInfo';
//function GetMenuBarInfo(hend: HWND; idObject, idItem: Longint; var pmbi: TMenuBarInfo): BOOL;external 'user32' name 'GetMenuBarInfo';
//function GetMenuInfo(hMenu: HMENU; var lpmi: TMenuInfo): BOOL;external 'user32' name 'GetMenuInfo';
function GetMenuItemInfo(p1: HMENU; p2: UINT; p3: BOOL; var p4: TMenuItemInfo): BOOL;external 'user32' name 'GetMenuItemInfoA';
function GetMenuItemInfoA(p1: HMENU; p2: UINT; p3: BOOL; var p4: TMenuItemInfoA): BOOL; external 'user32' name 'GetMenuItemInfoA';
//function GetMenuItemInfoW(p1: HMENU; p2: UINT; p3: BOOL; var p4: TMenuItemInfoW): BOOL; external 'user32' name 'GetMenuItemInfoW';
function GetMenuItemRect(hWnd: HWND; hMenu: HMENU; uItem: UINT; var lprcItem: TRect): BOOL; external 'user32' name 'GetMenuItemRect';
function GetMessage(var lpMsg: TMsg; hWnd: HWND; wMsgFilterMin, wMsgFilterMax: UINT): BOOL;external 'user32' name 'GetMessageA';
function GetMessageA(var lpMsg: TMsg; hWnd: HWND; wMsgFilterMin, wMsgFilterMax: UINT): BOOL; external 'user32' name 'GetMessageA';
function GetMessageW(var lpMsg: TMsg; hWnd: HWND; wMsgFilterMin, wMsgFilterMax: UINT): BOOL; external 'user32' name 'GetMessageW';
function GetMiterLimit(DC: HDC; var Limit: Single): BOOL; external 'gdi32' name 'GetMiterLimit';
//function GetMouseMovePoints(cbSize: UINT; var lppt, lpptBuf: TMouseMovePoint; nBufPoints: Integer; resolution: DWORD): Integer;external 'user32' name 'GetMouseMovePoints';
function GetNamedPipeInfo(hNamedPipe: THandle; var lpFlags: DWORD; lpOutBufferSize, lpInBufferSize, lpMaxInstances: Pointer): BOOL; external 'kernel32' name 'GetNamedPipeInfo';
function GetNumberOfConsoleInputEvents(hConsoleInput: THandle; var lpNumberOfEvents: DWORD): BOOL; external 'kernel32' name 'GetNumberOfConsoleInputEvents';
function GetNumberOfConsoleMouseButtons(var lpNumberOfMouseButtons: DWORD): BOOL; external 'kernel32' name 'GetNumberOfConsoleMouseButtons';
//function GetNumberOfEventLogRecords(hEventLog: THandle; var NumberOfRecords: DWORD): BOOL; external 'advapi32' name 'GetNumberOfEventLogRecords';
//function GetOldestEventLogRecord(hEventLog: THandle; var OldestRecord: DWORD): BOOL; external 'advapi32' name 'GetOldestEventLogRecord';
function GetOverlappedResult(hFile: THandle; const lpOverlapped: TOverlapped; var lpNumberOfBytesTransferred: DWORD; bWait: BOOL): BOOL; external 'kernel32' name 'GetOverlappedResult';
function GetPaletteEntries(Palette: HPALETTE; StartIndex, NumEntries: UINT; var PaletteEntries): UINT; external 'gdi32' name 'GetPaletteEntries';
function GetPath(DC: HDC; var Points, Types; nSize: Integer): Integer; external 'gdi32' name 'GetPath';
function GetPriorityClipboardFormat(var paFormatPriorityList; cFormats: Integer): Integer; external 'user32' name 'GetPriorityClipboardFormat';
//function GetPrivateObjectSecurity(ObjectDescriptor: PSecurityDescriptor; SecurityInformation: SECURITY_INFORMATION; ResultantDescriptor: PSecurityDescriptor; DescriptorLength: DWORD; var ReturnLength: DWORD): BOOL;
//  external 'advapi32' name 'GetPrivateObjectSecurity';
function GetPrivateProfileSectionNamesA(lpszReturnBuffer: LPSTR; nSize: DWORD; lpFileName: LPCSTR): DWORD; external 'kernel32' name 'GetPrivateProfileSectionNamesA';
function GetPrivateProfileSectionNamesW(lpszReturnBuffer: LPWSTR; nSize: DWORD; lpFileName: LPCWSTR): DWORD; external 'kernel32' name 'GetPrivateProfileSectionNamesW';
function GetPrivateProfileSectionNames(lpszReturnBuffer: LPTSTR; nSize: DWORD; lpFileName: LPCTSTR): DWORD; external 'kernel32' name 'GetPrivateProfileSectionNamesA';
function GetPrivateProfileStructA(lpszSection, lpszKey: LPCSTR; lpStruct: LPVOID; uSizeStruct: UINT; szFile: LPCSTR): BOOL; external 'kernel32' name 'GetPrivateProfileStructA';
function GetPrivateProfileStructW(lpszSection, lpszKey: LPCWSTR; lpStruct: LPVOID; uSizeStruct: UINT; szFile: LPCWSTR): BOOL; external 'kernel32' name 'GetPrivateProfileStructW';
function GetPrivateProfileStruct(lpszSection, lpszKey: LPCTSTR; lpStruct: LPVOID; uSizeStruct: UINT; szFile: LPCTSTR): BOOL; external 'kernel32' name 'GetPrivateProfileStructA';
function GetProcessAffinityMask(hProcess: THandle; var lpProcessAffinityMask, lpSystemAffinityMask: DWORD_PTR): BOOL; external 'kernel32' name 'GetProcessAffinityMask';
function GetProcessHeaps(NumberOfHeaps: DWORD; var ProcessHeaps: THandle): DWORD;external 'kernel32' name 'GetProcessHeaps';
{$ifndef NO_SMART_LINK}
function GetProcessPriorityBoost(hThread: THandle; var DisablePriorityBoost: Bool): BOOL;external 'kernel32' name 'GetProcessPriorityBoost';
{$endif NO_SMART_LINK}
function GetProcessShutdownParameters(var lpdwLevel, lpdwFlags: DWORD): BOOL; external 'kernel32' name 'GetProcessShutdownParameters';
function GetProcessTimes(hProcess: THandle; var lpCreationTime, lpExitTime, lpKernelTime, lpUserTime: TFileTime): BOOL; external 'kernel32' name 'GetProcessTimes';
function GetProcessWorkingSetSize(hProcess: THandle; var lpMinimumWorkingSetSize, lpMaximumWorkingSetSize: PTRUINT): BOOL; external 'kernel32' name 'GetProcessWorkingSetSize';
function GetQueuedCompletionStatus(CompletionPort: THandle; var lpNumberOfBytesTransferred: DWORD; var lpCompletionKey: ULONG_PTR; var lpOverlapped: POverlapped; dwMilliseconds: DWORD): BOOL; external 'kernel32' name 'GetQueuedCompletionStatus';
function PostQueuedCompletionStatus(CompletionPort: THandle; NumberOfBytesTransferred:dword; dwCompletionKey: ULONG_PTR; lpOverlapped: POverlapped): BOOL; external 'kernel32' name 'PostQueuedCompletionStatus';
function GetRasterizerCaps(var p1: TRasterizerStatus; p2: UINT): BOOL; external 'gdi32' name 'GetRasterizerCaps';
function GetRgnBox(RGN: HRGN; var p2: TRect): Integer; external 'gdi32' name 'GetRgnBox';
function GetScrollInfo(hWnd: HWND; BarFlag: Integer; var ScrollInfo: TScrollInfo): BOOL; external 'user32' name 'GetScrollInfo';
function GetScrollRange(hWnd: HWND; nBar: Integer; var lpMinPos, lpMaxPos: Integer): BOOL; external 'user32' name 'GetScrollRange';
//function GetSecurityDescriptorControl(pSecurityDescriptor: PSecurityDescriptor; var pControl: SECURITY_DESCRIPTOR_CONTROL; var lpdwRevision: DWORD): BOOL; external 'advapi32' name 'GetSecurityDescriptorControl';
//function GetSecurityDescriptorDacl(pSecurityDescriptor: PSecurityDescriptor; var lpbDaclPresent: BOOL; var pDacl: PACL; var lpbDaclDefaulted: BOOL): BOOL; external 'advapi32' name 'GetSecurityDescriptorDacl';
//function GetSecurityDescriptorGroup(pSecurityDescriptor: PSecurityDescriptor; var pGroup: PSID; var lpbGroupDefaulted: BOOL): BOOL; external 'advapi32' name 'GetSecurityDescriptorGroup';
//function GetSecurityDescriptorOwner(pSecurityDescriptor: PSecurityDescriptor; var pOwner: PSID; var lpbOwnerDefaulted: BOOL): BOOL; external 'advapi32' name 'GetSecurityDescriptorOwner';
//function GetSecurityDescriptorSacl(pSecurityDescriptor: PSecurityDescriptor; var lpbSaclPresent: BOOL; var pSacl: PACL; var lpbSaclDefaulted: BOOL): BOOL; external 'advapi32' name 'GetSecurityDescriptorSacl';
procedure GetStartupInfo(var lpStartupInfo:TSTARTUPINFO); external 'kernel32' name 'GetStartupInfoA';
function GetStringTypeA(Locale: LCID; dwInfoType: DWORD; const lpSrcStr: LPCSTR; cchSrc: BOOL; var lpCharType: Word): BOOL;external 'kernel32' name 'GetStringTypeA';
function GetStringTypeEx(Locale: LCID; dwInfoType: DWORD; lpSrcStr: PChar; cchSrc: Integer; var lpCharType): BOOL;external 'kernel32' name 'GetStringTypeExA';
function GetStringTypeExA(Locale: LCID; dwInfoType: DWORD; lpSrcStr: LPCSTR; cchSrc: Integer; var lpCharType): BOOL; external 'kernel32' name 'GetStringTypeExA';
function GetStringTypeExW(Locale: LCID; dwInfoType: DWORD; lpSrcStr: LPWSTR; cchSrc: Integer; var lpCharType): BOOL; external 'kernel32' name 'GetStringTypeExW';
function GetStringTypeW(dwInfoType: DWORD; const lpSrcStr: WCHAR; cchSrc: BOOL; var lpCharType: Word): BOOL;external 'kernel32' name 'GetStringTypeW';
function GetSystemPaletteEntries(DC: HDC; StartIndex, NumEntries: UINT; var PaletteEntries): UINT; external 'gdi32' name 'GetSystemPaletteEntries';
function GetSystemPowerStatus(var lpSystemPowerStatus: TSystemPowerStatus): BOOL;external 'kernel32' name 'GetSystemPowerStatus';
function GetSystemTimeAdjustment(var lpTimeAdjustment, lpTimeIncrement: DWORD; var lpTimeAdjustmentDisabled: BOOL): BOOL; external 'kernel32' name 'GetSystemTimeAdjustment';
procedure GetSystemTimeAsFileTime(var lpSystemTimeAsFileTime:TFILETIME); external 'kernel32' name 'GetSystemTimeAsFileTime';
function GetTabbedTextExtent(hDC: HDC; lpString: PChar; nCount, nTabPositions: Integer; var lpnTabStopPositions): DWORD;external 'user32' name 'GetTabbedTextExtentA';
function GetTabbedTextExtentA(hDC: HDC; lpString: LPCSTR; nCount, nTabPositions: Integer; var lpnTabStopPositions): DWORD; external 'user32' name 'GetTabbedTextExtentA';
function GetTabbedTextExtentW(hDC: HDC; lpString: LPWSTR; nCount, nTabPositions: Integer; var lpnTabStopPositions): DWORD; external 'user32' name 'GetTabbedTextExtentW';
function GetTapeParameters(hDevice: THandle; dwOperation: DWORD; var lpdwSize: DWORD; lpTapeInformation: Pointer): DWORD; external 'kernel32' name 'GetTapeParameters';
function GetTapePosition(hDevice: THandle; dwPositionType: DWORD; var lpdwPartition, lpdwOffsetLow: DWORD; lpdwOffsetHigh: Pointer): DWORD; external 'kernel32' name 'GetTapePosition';
function GetTextExtentExPoint(DC: HDC; p2: PChar; p3, p4: Integer; p5, p6: PInteger; var p7: TSize): BOOL;external 'gdi32' name 'GetTextExtentExPointA';
function GetTextExtentExPointA(DC: HDC; p2: LPCSTR; p3, p4: Integer; p5, p6: PInteger; var p7: TSize): BOOL; external 'gdi32' name 'GetTextExtentExPointA';
//function GetTextExtentExPointI(DC: HDC; p2: PWORD; p3, p4: Integer; p5, p6: PINT; var p7: TSize): BOOL;external 'gdi32' name 'GetTextExtentExPointI';
function GetTextExtentExPointW(DC: HDC; p2: LPWSTR; p3, p4: Integer; p5, p6: PInteger; var p7: TSize): BOOL; external 'gdi32' name 'GetTextExtentExPointW';
function GetTextExtentPoint(DC: HDC; Str: PChar; Count: Integer; var Size: TSize): BOOL;external 'gdi32' name 'GetTextExtentPointA';
function GetTextExtentPoint32(DC: HDC; Str: PChar; Count: Integer; var Size: TSize): BOOL;external 'gdi32' name 'GetTextExtentPoint32A';
function GetTextExtentPoint32A(DC: HDC; Str: LPCSTR; Count: Integer; var Size: TSize): BOOL; external 'gdi32' name 'GetTextExtentPoint32A';
function GetTextExtentPoint32W(DC: HDC; Str: LPWSTR; Count: Integer; var Size: TSize): BOOL; external 'gdi32' name 'GetTextExtentPoint32W';
function GetTextExtentPointA(DC: HDC; Str: LPCSTR; Count: Integer; var Size: TSize): BOOL; external 'gdi32' name 'GetTextExtentPointA';
//function GetTextExtentPointI(DC: HDC; p2: PWORD; p3: Integer; var p4: TSize): BOOL;external 'gdi32' name 'GetTextExtentPointI';
function GetTextExtentPointW(DC: HDC; Str: LPWSTR; Count: Integer; var Size: TSize): BOOL; external 'gdi32' name 'GetTextExtentPointW';
function GetTextMetrics(DC: HDC; var TM: TTextMetric): BOOL;external 'gdi32' name 'GetTextMetricsA';
//function GetTextMetricsA(DC: HDC; var TM: TTextMetricA): BOOL; external 'gdi32' name 'GetTextMetricsA';
//function GetTextMetricsW(DC: HDC; var TM: TTextMetricW): BOOL; external 'gdi32' name 'GetTextMetricsW';
function GetThreadContext(hThread: THandle; var lpContext: TContext): BOOL; external 'kernel32' name 'GetThreadContext';
{$ifndef NO_SMART_LINK}
function GetThreadPriorityBoost(hThread: THandle; var DisablePriorityBoost: Bool): BOOL;external 'kernel32' name 'GetThreadPriorityBoost';
{$endif NO_SMART_LINK}
function GetThreadSelectorEntry(hThread: THandle; dwSelector: DWORD; var lpSelectorEntry: TLDTEntry): BOOL; external 'kernel32' name 'GetThreadSelectorEntry';
function GetThreadTimes(hThread: THandle; var lpCreationTime, lpExitTime, lpKernelTime, lpUserTime: TFileTime): BOOL; external 'kernel32' name 'GetThreadTimes';
function GetTimeZoneInformation(var lpTimeZoneInformation: TTimeZoneInformation): DWORD; external 'kernel32' name 'GetTimeZoneInformation';
//function GetTimeZoneInformationForYear(wYear: USHORT; lpDynamicTimeZoneInformation: PDynamicTimeZoneInformation;
//  var lpTimeZoneInformation: TTimeZoneInformation): BOOL; external 'kernel32' name 'GetTimeZoneInformationForYear';
//function GetTitleBarInfo(hwnd: HWND; var pti: TTitleBarInfo): BOOL;external 'user32' name 'GetTitleBarInfo';
function GetTokenInformation(TokenHandle: THandle; TokenInformationClass: TTokenInformationClass; TokenInformation: Pointer; TokenInformationLength: DWORD; var ReturnLength: DWORD): BOOL; external 'advapi32' name 'GetTokenInformation';
function GetUpdateRect(hWnd: HWND; var lpRect: TRect; bErase: BOOL): BOOL; external 'user32' name 'GetUpdateRect';
function GetUserName(lpBuffer: PChar; var nSize: DWORD): BOOL;external 'advapi32' name 'GetUserNameA';
function GetUserNameA(lpBuffer: LPCSTR; var nSize: DWORD): BOOL; external 'advapi32' name 'GetUserNameA';
function GetUserNameW(lpBuffer: LPWSTR; var nSize: DWORD): BOOL; external 'advapi32' name 'GetUserNameW';
function GetUserObjectInformation(hObj: THandle; nIndex: Integer; pvInfo: Pointer; nLength: DWORD; var lpnLengthNeeded: DWORD): BOOL;external 'user32' name 'GetUserObjectInformationA';
function GetUserObjectInformationA(hObj: THandle; nIndex: Integer; pvInfo: Pointer; nLength: DWORD; var lpnLengthNeeded: DWORD): BOOL; external 'user32' name 'GetUserObjectInformationA';
function GetUserObjectInformationW(hObj: THandle; nIndex: Integer; pvInfo: Pointer; nLength: DWORD; var lpnLengthNeeded: DWORD): BOOL; external 'user32' name 'GetUserObjectInformationW';
function GetUserObjectSecurity(hObj: THandle; var pSIRequested: DWORD; pSID: PSecurityDescriptor; nLength: DWORD; var lpnLengthNeeded: DWORD): BOOL; external 'user32' name 'GetUserObjectSecurity';
function GetVersionEx(var lpVersionInformation: TOSVersionInfo): BOOL;external 'kernel32' name 'GetVersionExA';
function GetVersionExA(var lpVersionInformation: TOSVersionInfo): BOOL; external 'kernel32' name 'GetVersionExA';
function GetVersionExW(var lpVersionInformation: TOSVersionInfoW): BOOL; external 'kernel32' name 'GetVersionExW';
function GetViewportExtEx(DC: HDC; var Size: TSize): BOOL; external 'gdi32' name 'GetViewportExtEx';
function GetViewportOrgEx(DC: HDC; var Point: TPoint): BOOL; external 'gdi32' name 'GetViewportOrgEx';
function GetVolumeInformation(lpRootPathName: PChar; lpVolumeNameBuffer: PChar; nVolumeNameSize: DWORD; lpVolumeSerialNumber: PDWORD; var lpMaximumComponentLength, lpFileSystemFlags: DWORD;
  lpFileSystemNameBuffer: PChar; nFileSystemNameSize: DWORD): BOOL; external 'kernel32' name 'GetVolumeInformationA';
function GetVolumeInformationA(lpRootPathName: LPCSTR; lpVolumeNameBuffer: LPCSTR; nVolumeNameSize: DWORD; lpVolumeSerialNumber: PDWORD; var lpMaximumComponentLength, lpFileSystemFlags: DWORD;
  lpFileSystemNameBuffer: LPCSTR; nFileSystemNameSize: DWORD): BOOL; external 'kernel32' name 'GetVolumeInformationA';
function GetVolumeInformationW(lpRootPathName: LPWSTR; lpVolumeNameBuffer: LPWSTR; nVolumeNameSize: DWORD; lpVolumeSerialNumber: PDWORD; var lpMaximumComponentLength, lpFileSystemFlags: DWORD;
  lpFileSystemNameBuffer: LPWSTR; nFileSystemNameSize: DWORD): BOOL; external 'kernel32' name 'GetVolumeInformationW';
function GetWindowExtEx(DC: HDC; var Size: TSize): BOOL; external 'gdi32' name 'GetWindowExtEx';
//function GetWindowInfo(hwnd: HWND; var pwi: TWindowInfo): BOOL;external 'user32' name 'GetWindowInfo';
function GetWindowOrgEx(DC: HDC; var Point: TPoint): BOOL; external 'gdi32' name 'GetWindowOrgEx';
function GetWindowRect(hWnd: HWND; var lpRect: TRect): BOOL; external 'user32' name 'GetWindowRect';
function GetWorldTransform(DC: HDC; var p2: TXForm): BOOL; external 'gdi32' name 'GetWorldTransform';
//function GradientFill(DC: HDC; var p2: TTriVertex; p3: ULONG; p4: Pointer; p5, p6: ULONG): BOOL;external 'gdi32' name 'GradientFill';
procedure GlobalMemoryStatus(var Buffer: MEMORYSTATUS); external 'kernel32' name 'GlobalMemoryStatus';
function HeapWalk(hHeap: THandle; var lpEntry: TProcessHeapEntry): BOOL; external 'kernel32' name 'HeapWalk';
//function ImageList_GetDragImage(var ppt:POINT; var pptHotspot:POINT):HIMAGELIST; external 'comctl32' name 'ImageList_GetDragImage';
function InflateRect(var lprc: TRect; dx, dy: Integer): BOOL; external 'user32' name 'InflateRect';
function InitializeAcl(var pAcl: TACL; nAclLength, dwAclRevision: DWORD): BOOL; external 'advapi32' name 'InitializeAcl';
{$ifndef NO_SMART_LINK}
//function InitializeCriticalSectionAndSpinCount(var lpCriticalSection: TRTLCriticalSection; dwSpinCount: DWORD): BOOL;external 'kernel32' name 'InitializeCriticalSectionAndSpinCount';
{$endif NO_SMART_LINK}
function InitializeSid(Sid: Pointer; const pIdentifierAuthority: TSIDIdentifierAuthority; nSubAuthorityCount: Byte): BOOL; external 'advapi32' name 'InitializeSid';
function InsertMenuItem(p1: HMENU; p2: UINT; p3: BOOL; const p4: TMenuItemInfo): BOOL;external 'user32' name 'InsertMenuItemA';
function InsertMenuItemA(p1: HMENU; p2: UINT; p3: BOOL; const p4: TMenuItemInfoA): BOOL; external 'user32' name 'InsertMenuItemA';
//function InsertMenuItemW(p1: HMENU; p2: UINT; p3: BOOL; const p4: TMenuItemInfoW): BOOL; external 'user32' name 'InsertMenuItemW';
function IntersectRect(var lprcDst: TRect; const lprcSrc1, lprcSrc2: TRect): BOOL; external 'user32' name 'IntersectRect';
//function InvertRect(hDC: HDC; const lprc: TRect): BOOL; external 'user32' name 'InvertRect';
function IsDialogMessage(hDlg: HWND; var lpMsg: TMsg): BOOL;external 'user32' name 'IsDialogMessageA';
function IsDialogMessageA(hDlg: HWND; var lpMsg: TMsg): BOOL; external 'user32' name 'IsDialogMessageA';
function IsDialogMessageW(hDlg: HWND; var lpMsg: TMsg): BOOL; external 'user32' name 'IsDialogMessageW';
//function IsRectEmpty(const lprc: TRect): BOOL; external 'user32' name 'IsRectEmpty';
function IsValidAcl(const pAcl: TACL): BOOL; external 'advapi32' name 'IsValidAcl';
function LocalFileTimeToFileTime(const lpLocalFileTime: TFileTime; var lpFileTime: TFileTime): BOOL; external 'kernel32' name 'LocalFileTimeToFileTime';
function LockFileEx(hFile: THandle; dwFlags, dwReserved: DWORD; nNumberOfBytesToLockLow, nNumberOfBytesToLockHigh: DWORD; const lpOverlapped: TOverlapped): BOOL; external 'kernel32' name 'LockFileEx';
function LogonUser(lpszUsername, lpszDomain, lpszPassword: PChar; dwLogonType, dwLogonProvider: DWORD; var phToken: THandle): BOOL;external 'advapi32' name 'LogonUserA';
function LogonUserA(lpszUsername, lpszDomain, lpszPassword: LPCSTR; dwLogonType, dwLogonProvider: DWORD; var phToken: THandle): BOOL; external 'advapi32' name 'LogonUserA';
function LogonUserW(lpszUsername, lpszDomain, lpszPassword: LPWSTR; dwLogonType, dwLogonProvider: DWORD; var phToken: THandle): BOOL; external 'advapi32' name 'LogonUserW';
function LookupAccountName(lpSystemName, lpAccountName: PChar; Sid: PSID; var cbSid: DWORD; ReferencedDomainName: PChar; var cbReferencedDomainName: DWORD; var peUse: SID_NAME_USE): BOOL;external 'advapi32' name 'LookupAccountNameA';
function LookupAccountNameA(lpSystemName, lpAccountName: LPCSTR; Sid: PSID; var cbSid: DWORD; ReferencedDomainName: LPCSTR; var cbReferencedDomainName: DWORD; var peUse: SID_NAME_USE): BOOL; external 'advapi32' name 'LookupAccountNameA';
function LookupAccountNameW(lpSystemName, lpAccountName: LPWSTR; Sid: PSID; var cbSid: DWORD; ReferencedDomainName: LPWSTR; var cbReferencedDomainName: DWORD; var peUse: SID_NAME_USE): BOOL; external 'advapi32' name 'LookupAccountNameW';
function LookupAccountSid(lpSystemName: PChar; Sid: PSID; Name: PChar; var cbName: DWORD; ReferencedDomainName: PChar; var cbReferencedDomainName: DWORD; var peUse: SID_NAME_USE): BOOL;external 'advapi32' name 'LookupAccountSidA';
function LookupAccountSidA(lpSystemName: LPCSTR; Sid: PSID; Name: LPCSTR; var cbName: DWORD; ReferencedDomainName: LPCSTR; var cbReferencedDomainName: DWORD; var peUse: SID_NAME_USE): BOOL; external 'advapi32' name 'LookupAccountSidA';
function LookupAccountSidW(lpSystemName: LPWSTR; Sid: PSID; Name: LPWSTR; var cbName: DWORD; ReferencedDomainName: LPWSTR; var cbReferencedDomainName: DWORD; var peUse: SID_NAME_USE): BOOL; external 'advapi32' name 'LookupAccountSidW';
function LookupPrivilegeDisplayName(lpSystemName, lpName: LPCSTR; lpDisplayName: PChar; var cbDisplayName, lpLanguageId: DWORD): BOOL;external 'advapi32' name 'LookupPrivilegeDisplayNameA';
function LookupPrivilegeDisplayNameA(lpSystemName, lpName: LPCSTR; lpDisplayName: LPCSTR; var cbDisplayName, lpLanguageId: DWORD): BOOL; external 'advapi32' name 'LookupPrivilegeDisplayNameA';
function LookupPrivilegeDisplayNameW(lpSystemName, lpName: LPCSTR; lpDisplayName: LPWSTR; var cbDisplayName, lpLanguageId: DWORD): BOOL; external 'advapi32' name 'LookupPrivilegeDisplayNameW';
function LookupPrivilegeName(lpSystemName: PChar; var lpLuid: TLargeInteger; lpName: PChar; var cbName: DWORD): BOOL;external 'advapi32' name 'LookupPrivilegeNameA';
function LookupPrivilegeNameA(lpSystemName: LPCSTR; var lpLuid: TLargeInteger; lpName: LPCSTR; var cbName: DWORD): BOOL; external 'advapi32' name 'LookupPrivilegeNameA';
function LookupPrivilegeNameW(lpSystemName: LPWSTR; var lpLuid: TLargeInteger; lpName: LPWSTR; var cbName: DWORD): BOOL; external 'advapi32' name 'LookupPrivilegeNameW';
function LookupPrivilegeValue(lpSystemName, lpName: PChar; var lpLuid: TLargeInteger): BOOL;external 'advapi32' name 'LookupPrivilegeValueA';
function LookupPrivilegeValueA(lpSystemName, lpName: LPCSTR; var lpLuid: TLargeInteger): BOOL; external 'advapi32' name 'LookupPrivilegeValueA';
function LookupPrivilegeValueW(lpSystemName, lpName: LPWSTR; var lpLuid: TLargeInteger): BOOL; external 'advapi32' name 'LookupPrivilegeValueW';
function LPtoDP(DC: HDC; var Points; Count: Integer): BOOL; external 'gdi32' name 'LPtoDP';
function MakeAbsoluteSD(pSelfRelativeSecurityDescriptor: PSecurityDescriptor; pAbsoluteSecurityDescriptor: PSecurityDescriptor; var lpdwAbsoluteSecurityDescriptorSi: DWORD; var pDacl: TACL; var lpdwDaclSize: DWORD; var pSacl: TACL;
  var lpdwSaclSize: DWORD; pOwner: PSID; var lpdwOwnerSize: DWORD; pPrimaryGroup: Pointer; var lpdwPrimaryGroupSize: DWORD): BOOL; external 'advapi32' name 'MakeAbsoluteSD';
function MakeSelfRelativeSD(pAbsoluteSecurityDescriptor: PSecurityDescriptor; pSelfRelativeSecurityDescriptor: PSecurityDescriptor; var lpdwBufferLength: DWORD): BOOL; external 'advapi32' name 'MakeSelfRelativeSD';
function MapDialogRect(hDlg: HWND; var lpRect: TRect): BOOL; external 'user32' name 'MapDialogRect';
function MapWindowPoints(hWndFrom, hWndTo: HWND; var lpPoints; cPoints: UINT): Integer; external 'user32' name 'MapWindowPoints';
function MessageBoxIndirect(const MsgBoxParams: TMsgBoxParams): BOOL;external 'user32' name 'MessageBoxIndirectA';
function MessageBoxIndirectA(const MsgBoxParams: TMsgBoxParamsA): BOOL; external 'user32' name 'MessageBoxIndirectA';
//function MessageBoxIndirectW(const MsgBoxParams: TMsgBoxParamsW): BOOL; external 'user32' name 'MessageBoxIndirectW';
function ModifyWorldTransform(_para1:HDC; var _para2:XFORM; _para3:DWORD):WINBOOL; external 'gdi32' name 'ModifyWorldTransform';
function MsgWaitForMultipleObjects(nCount: DWORD; var pHandles; fWaitAll: BOOL; dwMilliseconds, dwWakeMask: DWORD): DWORD;external 'user32' name 'MsgWaitForMultipleObjects';
{$ifndef NO_SMART_LINK}
function MsgWaitForMultipleObjectsEx(nCount: DWORD; var pHandles; dwMilliseconds, dwWakeMask, dwFlags: DWORD): DWORD;external 'user32' name 'MsgWaitForMultipleObjectsEx';
{$endif NO_SMART_LINK}
// function MultiByteToWideChar(CodePage: UINT; dwFlags: DWORD; const lpMultiByteStr: LPCSTR; cchMultiByte: Integer; lLPWSTRStr: LPWSTR; cchWideChar: Integer): Integer; external 'kernel32' name 'MultiByteToWideChar';
function ObjectOpenAuditAlarm(SubsystemName: PChar; HandleId: Pointer; ObjectTypeName: PChar; ObjectName: PChar; pSecurityDescriptor: PSecurityDescriptor; ClientToken: THandle; DesiredAccess, GrantedAccess: DWORD; var Privileges: TPrivilegeSet;
  ObjectCreation, AccessGranted: BOOL; var GenerateOnClose: BOOL): BOOL;external 'advapi32' name 'ObjectOpenAuditAlarmA';
function ObjectOpenAuditAlarmA(SubsystemName: LPCSTR; HandleId: Pointer; ObjectTypeName: LPCSTR; ObjectName: LPCSTR; pSecurityDescriptor: PSecurityDescriptor; ClientToken: THandle; DesiredAccess, GrantedAccess: DWORD; var Privileges: TPrivilegeSet;
  ObjectCreation, AccessGranted: BOOL; var GenerateOnClose: BOOL): BOOL; external 'advapi32' name 'ObjectOpenAuditAlarmA';
function ObjectOpenAuditAlarmW(SubsystemName: LPWSTR; HandleId: Pointer; ObjectTypeName: LPWSTR; ObjectName: LPWSTR; pSecurityDescriptor: PSecurityDescriptor; ClientToken: THandle; DesiredAccess, GrantedAccess: DWORD; var Privileges: TPrivilegeSet;
  ObjectCreation, AccessGranted: BOOL; var GenerateOnClose: BOOL): BOOL; external 'advapi32' name 'ObjectOpenAuditAlarmW';
function ObjectPrivilegeAuditAlarm(SubsystemName: PChar; HandleId: Pointer; ClientToken: THandle; DesiredAccess: DWORD; var Privileges: TPrivilegeSet; AccessGranted: BOOL): BOOL;external 'advapi32' name 'ObjectPrivilegeAuditAlarmA';
function ObjectPrivilegeAuditAlarmA(SubsystemName: LPCSTR; HandleId: Pointer; ClientToken: THandle; DesiredAccess: DWORD; var Privileges: TPrivilegeSet; AccessGranted: BOOL): BOOL; external 'advapi32' name 'ObjectPrivilegeAuditAlarmA';
function ObjectPrivilegeAuditAlarmW(SubsystemName: LPWSTR; HandleId: Pointer; ClientToken: THandle; DesiredAccess: DWORD; var Privileges: TPrivilegeSet; AccessGranted: BOOL): BOOL; external 'advapi32' name 'ObjectPrivilegeAuditAlarmW';

function OffsetRect(var lprc: TRect; dx, dy: Integer): BOOL; external 'user32' name 'OffsetRect';
function OffsetViewportOrgEx(DC: HDC; X, Y: Integer; var Points): BOOL; external 'gdi32' name 'OffsetViewportOrgEx';
function OffsetWindowOrgEx(DC: HDC; X, Y: Integer; var Points): BOOL; external 'gdi32' name 'OffsetWindowOrgEx';
function OpenFile(const lpFileName: LPCSTR; var lpReOpenBuff: TOFStruct; uStyle: UINT): HFILE; external 'kernel32' name 'OpenFile';
function OpenProcessToken(ProcessHandle: THandle; DesiredAccess: DWORD; var TokenHandle: THandle): BOOL; external 'advapi32' name 'OpenProcessToken';
function OpenThreadToken(ThreadHandle: THandle; DesiredAccess: DWORD; OpenAsSelf: BOOL; var TokenHandle: THandle): BOOL; external 'advapi32' name 'OpenThreadToken';
function PeekConsoleInput(hConsoleInput: THandle; var lpBuffer: TInputRecord; nLength: DWORD; var lpNumberOfEventsRead: DWORD): BOOL;external 'kernel32' name 'PeekConsoleInputA';
function PeekConsoleInputA(hConsoleInput: THandle; var lpBuffer: TInputRecord; nLength: DWORD; var lpNumberOfEventsRead: DWORD): BOOL; external 'kernel32' name 'PeekConsoleInputA';
function PeekConsoleInputW(hConsoleInput: THandle; var lpBuffer: TInputRecord; nLength: DWORD; var lpNumberOfEventsRead: DWORD): BOOL; external 'kernel32' name 'PeekConsoleInputW';
function PeekMessage(var lpMsg: TMsg; hWnd: HWND; wMsgFilterMin, wMsgFilterMax, wRemoveMsg: UINT): BOOL;external 'user32' name 'PeekMessageA';
function PeekMessageA(var lpMsg: TMsg; hWnd: HWND; wMsgFilterMin, wMsgFilterMax, wRemoveMsg: UINT): BOOL; external 'user32' name 'PeekMessageA';
function PeekMessageW(var lpMsg: TMsg; hWnd: HWND; wMsgFilterMin, wMsgFilterMax, wRemoveMsg: UINT): BOOL; external 'user32' name 'PeekMessageW';
//function PlayEnhMetaFile(DC: HDC; p2: HENHMETAFILE; const p3: TRect): BOOL; external 'gdi32' name 'PlayEnhMetaFile';
function PlayEnhMetaFileRecord(DC: HDC; var p2: THandleTable; const p3: TEnhMetaRecord; p4: UINT): BOOL; external 'gdi32' name 'PlayEnhMetaFileRecord';
function PlayMetaFileRecord(DC: HDC; const p2: THandleTable; const p3: TMetaRecord; p4: UINT): BOOL; external 'gdi32' name 'PlayMetaFileRecord';
function PlgBlt(DC: HDC; const PointsArray; p3: HDC; p4, p5, p6, p7: Integer; p8: HBITMAP; p9, p10: Integer): BOOL; external 'gdi32' name 'PlgBlt';
function PolyBezier(DC: HDC; const Points; Count: DWORD): BOOL; external 'gdi32' name 'PolyBezier';
function PolyBezierTo(DC: HDC; const Points; Count: DWORD): BOOL; external 'gdi32' name 'PolyBezierTo';
function PolyDraw(DC: HDC; const Points, Types; cCount: Integer): BOOL; external 'gdi32' name 'PolyDraw';
function Polygon(DC: HDC; var Points; Count: Integer): BOOL; external 'gdi32' name 'Polygon';
function Polyline(DC: HDC; var Points; Count: Integer): BOOL; external 'gdi32' name 'Polyline';
function PolyLineTo(DC: HDC; const Points; Count: DWORD): BOOL; external 'gdi32' name 'PolylineTo';
function PolyPolygon(DC: HDC; var Points; var nPoints; p4: Integer): BOOL; external 'gdi32' name 'PolyPolygon';
function PolyPolyline(DC: HDC; const PointStructs; const Points; p4: DWORD): BOOL; external 'gdi32' name 'PolyPolyline';
function PolyTextOut(DC: HDC; const PolyTextArray; Strings: Integer): BOOL;external 'gdi32' name 'PolyTextOutA';
function PolyTextOutA(DC: HDC; const PolyTextArray; Strings: Integer): BOOL; external 'gdi32' name 'PolyTextOutA';
function PolyTextOutW(DC: HDC; const PolyTextArray; Strings: Integer): BOOL; external 'gdi32' name 'PolyTextOutW';
function PrivilegeCheck(ClientToken: THandle; const RequiredPrivileges: TPrivilegeSet; var pfResult: BOOL): BOOL; external 'advapi32' name 'PrivilegeCheck';
function PrivilegedServiceAuditAlarm(SubsystemName, ServiceName: PChar; ClientToken: THandle; var Privileges: TPrivilegeSet; AccessGranted: BOOL): BOOL;external 'advapi32' name 'PrivilegedServiceAuditAlarmA';
function PrivilegedServiceAuditAlarmA(SubsystemName, ServiceName: LPCSTR; ClientToken: THandle; var Privileges: TPrivilegeSet; AccessGranted: BOOL): BOOL; external 'advapi32' name 'PrivilegedServiceAuditAlarmA';
function PrivilegedServiceAuditAlarmW(SubsystemName, ServiceName: LPWSTR; ClientToken: THandle; var Privileges: TPrivilegeSet; AccessGranted: BOOL): BOOL; external 'advapi32' name 'PrivilegedServiceAuditAlarmW';
//function PtInRect(const lprc: TRect; pt: TPoint): BOOL; external 'user32' name 'PtInRect';
function QueryPerformanceCounter(var lpPerformanceCount: TLargeInteger): BOOL; external 'kernel32' name 'QueryPerformanceCounter';
function QueryPerformanceFrequency(var lpFrequency: TLargeInteger): BOOL; external 'kernel32' name 'QueryPerformanceFrequency';
//function QueryRecoveryAgents(p1: PChar; var p2: Pointer; var p3: TRecoveryAgentInformation): DWORD;external 'kernel32' name 'QueryRecoveryAgentsA';
//function QueryRecoveryAgentsA(p1: LPCSTR; var p2: Pointer; var p3: TRecoveryAgentInformationA): DWORD;external 'kernel32' name 'QueryRecoveryAgentsA';
//function QueryRecoveryAgentsW(p1: LPWSTR; var p2: Pointer; var p3: TRecoveryAgentInformationW): DWORD;external 'kernel32' name 'QueryRecoveryAgentsW';
procedure RaiseException(dwExceptionCode:DWORD; dwExceptionFlags:DWORD; nNumberOfArguments:DWORD; var lpArguments:DWORD); external 'kernel32' name 'RaiseException';
function UnhandledExceptionFilter(var ExceptionInfo:emptyrecord):LONG; external 'kernel32' name 'UnhandledExceptionFilter';
function ReadConsole(hConsoleInput: THandle; lpBuffer: Pointer; nNumberOfCharsToRead: DWORD; var lpNumberOfCharsRead: DWORD; lpReserved: Pointer): BOOL;external 'kernel32' name 'ReadConsoleA';
function ReadConsoleA(hConsoleInput: THandle; lpBuffer: Pointer; nNumberOfCharsToRead: DWORD; var lpNumberOfCharsRead: DWORD; lpReserved: Pointer): BOOL; external 'kernel32' name 'ReadConsoleA';
function ReadConsoleInput(hConsoleInput: THandle; var lpBuffer: TInputRecord; nLength: DWORD; var lpNumberOfEventsRead: DWORD): BOOL;external 'kernel32' name 'ReadConsoleInputA';
function ReadConsoleInputA(hConsoleInput: THandle; var lpBuffer: TInputRecord; nLength: DWORD; var lpNumberOfEventsRead: DWORD): BOOL; external 'kernel32' name 'ReadConsoleInputA';
function ReadConsoleInputW(hConsoleInput: THandle; var lpBuffer: TInputRecord; nLength: DWORD; var lpNumberOfEventsRead: DWORD): BOOL; external 'kernel32' name 'ReadConsoleInputW';
function ReadConsoleOutput(hConsoleOutput: THandle; lpBuffer: Pointer; dwBufferSize, dwBufferCoord: TCoord; var lpReadRegion: TSmallRect): BOOL;external 'kernel32' name 'ReadConsoleOutputA';
function ReadConsoleOutputA(hConsoleOutput: THandle; lpBuffer: Pointer; dwBufferSize, dwBufferCoord: TCoord; var lpReadRegion: TSmallRect): BOOL; external 'kernel32' name 'ReadConsoleOutputA';
function ReadConsoleOutputAttribute(hConsoleOutput: THandle; lpAttribute: Pointer; nLength: DWORD; dwReadCoord: TCoord; var lpNumberOfAttrsRead: DWORD): BOOL;external 'kernel32' name 'ReadConsoleOutputAttribute';
function ReadConsoleOutputCharacter(hConsoleOutput: THandle; lpCharacter: LPCSTR; nLength: DWORD; dwReadCoord: TCoord; var lpNumberOfCharsRead: DWORD): BOOL;external 'kernel32' name 'ReadConsoleOutputCharacterA';
function ReadConsoleOutputCharacterA(hConsoleOutput: THandle; lpCharacter: LPCSTR; nLength: DWORD; dwReadCoord: TCoord; var lpNumberOfCharsRead: DWORD): BOOL; external 'kernel32' name 'ReadConsoleOutputCharacterA';
function ReadConsoleOutputCharacterW(hConsoleOutput: THandle; lpCharacter: LPCSTR; nLength: DWORD; dwReadCoord: TCoord; var lpNumberOfCharsRead: DWORD): BOOL; external 'kernel32' name 'ReadConsoleOutputCharacterW';
function ReadConsoleOutputW(hConsoleOutput: THandle; lpBuffer: Pointer; dwBufferSize, dwBufferCoord: TCoord; var lpReadRegion: TSmallRect): BOOL; external 'kernel32' name 'ReadConsoleOutputW';
function ReadConsoleW(hConsoleInput: THandle; lpBuffer: Pointer; nNumberOfCharsToRead: DWORD; var lpNumberOfCharsRead: DWORD; lpReserved: Pointer): BOOL; external 'kernel32' name 'ReadConsoleW';
function ReadEventLog(hEventLog: THandle; dwReadFlags, dwRecordOffset: DWORD; lpBuffer: Pointer; nNumberOfBytesToRead: DWORD; var pnBytesRead, pnMinNumberOfBytesNeeded: DWORD): BOOL;external 'advapi32' name 'ReadEventLogA';
function ReadEventLogA(hEventLog: THandle; dwReadFlags, dwRecordOffset: DWORD; lpBuffer: Pointer; nNumberOfBytesToRead: DWORD; var pnBytesRead, pnMinNumberOfBytesNeeded: DWORD): BOOL; external 'advapi32' name 'ReadEventLogA';
function ReadEventLogW(hEventLog: THandle; dwReadFlags, dwRecordOffset: DWORD; lpBuffer: Pointer; nNumberOfBytesToRead: DWORD; var pnBytesRead, pnMinNumberOfBytesNeeded: DWORD): BOOL; external 'advapi32' name 'ReadEventLogW';
function ReadFile(hFile: THandle; var Buffer; nNumberOfBytesToRead: DWORD; var lpNumberOfBytesRead: DWORD; lpOverlapped: POverlapped): BOOL; external 'kernel32' name 'ReadFile';
function ReadProcessMemory(hProcess: THandle; const lpBaseAddress: Pointer; lpBuffer: Pointer; nSize: PTRUINT; var lpNumberOfBytesRead: PTRUINT): BOOL; external 'kernel32' name 'ReadProcessMemory';
//function RectInRegion(RGN: HRGN; const p2: TRect): BOOL; external 'gdi32' name 'RectInRegion';
//function RectVisible(DC: HDC; const Rect: TRect): BOOL; external 'gdi32' name 'RectVisible';
function RegConnectRegistry(lpMachineName: PChar; hKey: HKEY; var phkResult: HKEY): Longint;external 'advapi32' name 'RegConnectRegistryA';
function RegConnectRegistryA(lpMachineName: LPCSTR; hKey: HKEY; var phkResult: HKEY): Longint; external 'advapi32' name 'RegConnectRegistryA';
function RegConnectRegistryW(lpMachineName: LPWSTR; hKey: HKEY; var phkResult: HKEY): Longint; external 'advapi32' name 'RegConnectRegistryW';
function RegCreateKey(hKey: HKEY; lpSubKey: PChar; var phkResult: HKEY): Longint;external 'advapi32' name 'RegCreateKeyA';
function RegCreateKeyA(hKey: HKEY; lpSubKey: LPCSTR; var phkResult: HKEY): Longint; external 'advapi32' name 'RegCreateKeyA';
function RegCreateKeyEx(hKey: HKEY; lpSubKey: PChar; Reserved: DWORD; lpClass: PChar; dwOptions: DWORD; samDesired: REGSAM; lpSecurityAttributes: PSecurityAttributes; var phkResult: HKEY; lpdwDisposition: PDWORD): Longint;
  external 'advapi32' name 'RegCreateKeyExA';
function RegCreateKeyExA(hKey: HKEY; lpSubKey: LPCSTR; Reserved: DWORD; lpClass: LPCSTR; dwOptions: DWORD; samDesired: REGSAM; lpSecurityAttributes: PSecurityAttributes; var phkResult: HKEY; lpdwDisposition: PDWORD): Longint;
  external 'advapi32' name 'RegCreateKeyExA';
function RegCreateKeyExW(hKey: HKEY; lpSubKey: LPWSTR; Reserved: DWORD; lpClass: LPWSTR; dwOptions: DWORD; samDesired: REGSAM; lpSecurityAttributes: PSecurityAttributes; var phkResult: HKEY; lpdwDisposition: PDWORD): Longint;
  external 'advapi32' name 'RegCreateKeyExW';
function RegCreateKeyW(hKey: HKEY; lpSubKey: LPWSTR; var phkResult: HKEY): Longint; external 'advapi32' name 'RegCreateKeyW';
function RegEnumKeyEx(hKey: HKEY; dwIndex: DWORD; lpName: PChar; var lpcbName: DWORD; lpReserved: Pointer; lpClass: PChar; lpcbClass: PDWORD; lpftLastWriteTime: PFileTime): Longint;external 'advapi32' name 'RegEnumKeyExA';
function RegEnumKeyExA(hKey: HKEY; dwIndex: DWORD; lpName: LPCSTR; var lpcbName: DWORD; lpReserved: Pointer; lpClass: LPCSTR; lpcbClass: PDWORD; lpftLastWriteTime: PFileTime): Longint; external 'advapi32' name 'RegEnumKeyExA';
function RegEnumKeyExW(hKey: HKEY; dwIndex: DWORD; lpName: LPWSTR; var lpcbName: DWORD; lpReserved: Pointer; lpClass: LPWSTR; lpcbClass: PDWORD; lpftLastWriteTime: PFileTime): Longint; external 'advapi32' name 'RegEnumKeyExW';
function RegEnumValue(hKey: HKEY; dwIndex: DWORD; lpValueName: LPCSTR; var lpcbValueName: DWORD; lpReserved: Pointer; lpType: PDWORD; lpData: PByte; lpcbData: PDWORD): Longint;external 'advapi32' name 'RegEnumValueA';
function RegEnumValueA(hKey: HKEY; dwIndex: DWORD; lpValueName: LPCSTR; var lpcbValueName: DWORD; lpReserved: Pointer; lpType: PDWORD; lpData: PByte; lpcbData: PDWORD): Longint; external 'advapi32' name 'RegEnumValueA';
function RegEnumValueW(hKey: HKEY; dwIndex: DWORD; lpValueName: LPWSTR; var lpcbValueName: DWORD; lpReserved: Pointer; lpType: PDWORD; lpData: PByte; lpcbData: PDWORD): Longint; external 'advapi32' name 'RegEnumValueW';
function RegGetKeySecurity(hKey: HKEY; SecurityInformation: SECURITY_INFORMATION; pSecurityDescriptor: PSecurityDescriptor; var lpcbSecurityDescriptor: DWORD): Longint; external 'advapi32' name 'RegGetKeySecurity';
function RegSetValueEx(hKey:HKEY; lpValueName:LPCSTR; Reserved:DWORD; dwType:DWORD; lpData:pointer;cbData:DWORD):LONG; external 'advapi32' name 'RegSetValueExA';
function RegSetValueExA(hKey:HKEY; lpValueName:LPCSTR; Reserved:DWORD; dwType:DWORD; lpData:pointer;cbData:DWORD):LONG; external 'advapi32' name 'RegSetValueExA';
function RegSetValueExW(hKey:HKEY; lpValueName:LPCWSTR; Reserved:DWORD; dwType:DWORD; lpData:pointer;cbData:DWORD):LONG; external 'advapi32' name 'RegSetValueExW';
function RegisterClass(const lpWndClass: TWndClass): ATOM;external 'user32' name 'RegisterClassA';
function RegisterClassA(const lpWndClass: TWndClassA): ATOM; external 'user32' name 'RegisterClassA';
function RegisterClassEx(const WndClass: TWndClassEx): ATOM;external 'user32' name 'RegisterClassExA';
function RegisterClassExA(const WndClass: TWndClassExA): ATOM; external 'user32' name 'RegisterClassExA';
function RegisterClassExW(const WndClass: TWndClassExW): ATOM; external 'user32' name 'RegisterClassExW';
function RegisterClassW(const lpWndClass: TWndClassW): ATOM; external 'user32' name 'RegisterClassW';
function RegOpenKey(hKey: HKEY; lpSubKey: PChar; var phkResult: HKEY): Longint;external 'advapi32' name 'RegOpenKeyA';
function RegOpenKeyA(hKey: HKEY; lpSubKey: LPCSTR; var phkResult: HKEY): Longint; external 'advapi32' name 'RegOpenKeyA';
function RegOpenKeyEx(hKey: HKEY; lpSubKey: PChar; ulOptions: DWORD; samDesired: REGSAM; var phkResult: HKEY): Longint;external 'advapi32' name 'RegOpenKeyExA';
function RegOpenKeyExA(hKey: HKEY; lpSubKey: LPCSTR; ulOptions: DWORD; samDesired: REGSAM; var phkResult: HKEY): Longint; external 'advapi32' name 'RegOpenKeyExA';
function RegOpenKeyExW(hKey: HKEY; lpSubKey: LPWSTR; ulOptions: DWORD; samDesired: REGSAM; var phkResult: HKEY): Longint; external 'advapi32' name 'RegOpenKeyExW';
function RegOpenKeyW(hKey: HKEY; lpSubKey: LPWSTR; var phkResult: HKEY): Longint; external 'advapi32' name 'RegOpenKeyW';
function RegQueryMultipleValues(hKey: HKEY; var ValList; NumVals: DWORD; lpValueBuf: PChar; var ldwTotsize: DWORD): Longint;external 'advapi32' name 'RegQueryMultipleValuesA';
function RegQueryMultipleValuesA(hKey: HKEY; var ValList; NumVals: DWORD; lpValueBuf: LPCSTR; var ldwTotsize: DWORD): Longint; external 'advapi32' name 'RegQueryMultipleValuesA';
function RegQueryMultipleValuesW(hKey: HKEY; var ValList; NumVals: DWORD; lpValueBuf: LPWSTR; var ldwTotsize: DWORD): Longint; external 'advapi32' name 'RegQueryMultipleValuesW';
function RegQueryValue(hKey: HKEY; lpSubKey: PChar; lpValue: PChar; var lpcbValue: Longint): Longint;external 'advapi32' name 'RegQueryValueA';
function RegQueryValueA(hKey: HKEY; lpSubKey: LPCSTR; lpValue: LPCSTR; var lpcbValue: Longint): Longint; external 'advapi32' name 'RegQueryValueA';
function RegQueryValueW(hKey: HKEY; lpSubKey: LPWSTR; lpValue: LPWSTR; var lpcbValue: Longint): Longint; external 'advapi32' name 'RegQueryValueW';
function ResetDC(DC: HDC; const p2: TDeviceMode): HDC;external 'gdi32' name 'ResetDCA';
function ResetDCA(DC: HDC; const p2: TDeviceModeA): HDC; external 'gdi32' name 'ResetDCA';
//function ResetDCW(DC: HDC; const p2: TDeviceModeW): HDC; external 'gdi32' name 'ResetDCW';
function ScreenToClient(hWnd: HWND; var lpPoint: TPoint): BOOL; external 'user32' name 'ScreenToClient';
function ScrollConsoleScreenBuffer(hConsoleOutput: THandle; const lpScrollRectangle: TSmallRect; const lpClipRectangle: TSmallRect; dwDestinationOrigin: TCoord; var lpFill: TCharInfo): BOOL;external 'kernel32' name 'ScrollConsoleScreenBufferA';
function ScrollConsoleScreenBufferA(hConsoleOutput: THandle; const lpScrollRectangle: TSmallRect; const lpClipRectangle: TSmallRect; dwDestinationOrigin: TCoord; var lpFill: TCharInfo): BOOL; external 'kernel32' name 'ScrollConsoleScreenBufferA';
function ScrollConsoleScreenBufferW(hConsoleOutput: THandle; const lpScrollRectangle: TSmallRect; const lpClipRectangle: TSmallRect; dwDestinationOrigin: TCoord; var lpFill: TCharInfo): BOOL; external 'kernel32' name 'ScrollConsoleScreenBufferW';
function ScrollWindow(hWnd:HWND; XAmount:longint; YAmount:longint;lpRect:lpRECT; lpClipRect:lpRECT):WINBOOL; external 'user32' name 'ScrollWindow';
function ScrollWindowEx(hWnd:HWND; dx:longint; dy:longint; prcScroll:lpRECT; prcClip:lpRECT;hrgnUpdate:HRGN; prcUpdate:LPRECT; flags:UINT):longint; external 'user32' name 'ScrollWindowEx';
//function ScrollDC(DC: HDC; DX, DY: Integer; var Scroll, Clip: TRect; Rgn: HRGN; Update: PRect): BOOL; external 'user32' name 'ScrollDC';
//function SearchPath(lpPath, lpFileName, lpExtension: PChar; nBufferLength: DWORD; lpBuffer: PChar; var lpFilePart: PChar): DWORD;external 'kernel32' name 'SearchPathA';
//function SearchPathA(lpPath, lpFileName, lpExtension: LPCSTR; nBufferLength: DWORD; lpBuffer: LPCSTR; var lpFilePart: LPCSTR): DWORD; external 'kernel32' name 'SearchPathA';
//function SearchPathW(lpPath, lpFileName, lpExtension: LPWSTR; nBufferLength: DWORD; lpBuffer: LPWSTR; var lpFilePart: LPWSTR): DWORD; external 'kernel32' name 'SearchPathW';
//function SendInput(cInputs: UINT; var pInputs: TInput; cbSize: Integer): UINT;external 'user32' name 'SendInput';
function SendMessageTimeout(hWnd: HWND; Msg: UINT; wParam: WPARAM; lParam: LPARAM; fuFlags, uTimeout: UINT; var lpdwResult: DWORD_PTR): LRESULT;external 'user32' name 'SendMessageTimeoutA';
function SendMessageTimeoutA(hWnd: HWND; Msg: UINT; wParam: WPARAM; lParam: LPARAM; fuFlags, uTimeout: UINT; var lpdwResult: DWORD_PTR): LRESULT; external 'user32' name 'SendMessageTimeoutA';
function SendMessageTimeoutW(hWnd: HWND; Msg: UINT; wParam: WPARAM; lParam: LPARAM; fuFlags, uTimeout: UINT; var lpdwResult: DWORD_PTR): LRESULT; external 'user32' name 'SendMessageTimeoutW';
//function SetAclInformation(var pAcl: TACL; pAclInformation: Pointer; nAclInformationLength: DWORD; dwAclInformationClass: TAclInformationClass): BOOL; external 'advapi32' name 'SetAclInformation';
//function SetColorAdjustment(DC: HDC; const p2: TColorAdjustment): BOOL; external 'gdi32' name 'SetColorAdjustment';
function SetCommConfig(hCommDev: THandle; const lpCC: TCommConfig; dwSize: DWORD): BOOL; external 'kernel32' name 'SetCommConfig';
function SetCommState(hFile: THandle; const lpDCB: TDCB): BOOL; external 'kernel32' name 'SetCommState';
function SetCommTimeouts(hFile: THandle; const lpCommTimeouts: TCommTimeouts): BOOL; external 'kernel32' name 'SetCommTimeouts';
function SetConsoleCursorInfo(hConsoleOutput: THandle; const lpConsoleCursorInfo: TConsoleCursorInfo): BOOL; external 'kernel32' name 'SetConsoleCursorInfo';
//function SetConsoleWindowInfo(hConsoleOutput: THandle; bAbsolute: BOOL; const lpConsoleWindow: TSmallRect): BOOL; external 'kernel32' name 'SetConsoleWindowInfo';
{$ifndef NO_SMART_LINK}
//function SetCriticalSectionSpinCount(var lpCriticalSection: TRTLCriticalSection; dwSpinCount: DWORD): DWORD;external 'kernel32' name 'SetCriticalSectionSpinCount';
{$endif NO_SMART_LINK}
function SetDeviceGammaRamp(DC: HDC; var Ramp): BOOL; external 'gdi32' name 'SetDeviceGammaRamp';
function SetDIBColorTable(DC: HDC; p2, p3: UINT; var RGBQuadSTructs): UINT; external 'gdi32' name 'SetDIBColorTable';
function SetDIBits(DC: HDC; Bitmap: HBITMAP; StartScan, NumScans: UINT; Bits: Pointer; var BitsInfo: TBitmapInfo; Usage: UINT): Integer; external 'gdi32' name 'SetDIBits';
//function SetDIBitsToDevice(DC: HDC; DestX, DestY: Integer; Width, Height: DWORD; SrcX, SrcY: Integer; nStartScan, NumScans: UINT; Bits: Pointer; var BitsInfo: TBitmapInfo; Usage: UINT): Integer; external 'gdi32' name 'SetDIBitsToDevice';
function SetEnhMetaFileBits(_para1:UINT; _para2:pointer):HENHMETAFILE; external 'gdi32' name 'SetEnhMetaFileBits';
function SetFileTime(hFile:HANDLE; var lpCreationTime:FILETIME; var lpLastAccessTime:FILETIME; var lpLastWriteTime:FILETIME):WINBOOL; external 'kernel32' name 'SetFileTime';
//function SetKeyboardState(var KeyState: TKeyboardState): BOOL; external 'user32' name 'SetKeyboardState';
//function SetLocalTime(const lpSystemTime: TSystemTime): BOOL; external 'kernel32' name 'SetLocalTime';
//function SetMenuInfo(hMenu: HMENU; const lpcmi: TMenuInfo): BOOL;external 'user32' name 'SetMenuInfo';
function SetMenuItemInfo(p1: HMENU; p2: UINT; p3: BOOL; const p4: TMenuItemInfo): BOOL;external 'user32' name 'SetMenuItemInfoA';
function SetMenuItemInfoA(p1: HMENU; p2: UINT; p3: BOOL; const p4: TMenuItemInfoA): BOOL; external 'user32' name 'SetMenuItemInfoA';
//function SetMenuItemInfoW(p1: HMENU; p2: UINT; p3: BOOL; const p4: TMenuItemInfoW): BOOL; external 'user32' name 'SetMenuItemInfoW';
function SetMetaFileBitsEx(p1: UINT; const p2: PChar): HMETAFILE; external 'gdi32' name 'SetMetaFileBitsEx';
function SetNamedPipeHandleState(hNamedPipe: THandle; var lpMode: DWORD; lpMaxCollectionCount, lpCollectDataTimeout: Pointer): BOOL; external 'kernel32' name 'SetNamedPipeHandleState';
function SetPaletteEntries(Palette: HPALETTE; StartIndex, NumEntries: UINT; var PaletteEntries): UINT; external 'gdi32' name 'SetPaletteEntries';
//function SetPrivateObjectSecurity(SecurityInformation: SECURITY_INFORMATION; ModificationDescriptor: PSecurityDescriptor; var ObjectsSecurityDescriptor: PSecurityDescriptor; const GenericMapping: TGenericMapping; Token: THandle): BOOL;
//  external 'advapi32' name 'SetPrivateObjectSecurity';
//function SetPrivateObjectSecurityEx(SecurityInformation: SECURITY_INFORMATION; ModificationDescriptor: PSecurityDescriptor; var ObjectsSecurityDescriptor: PSecurityDescriptor; AutoInheritFlags: ULONG;
//  const GenericMapping: TGenericMapping; Token: THandle): BOOL;external 'advapi32' name 'SetPrivateObjectSecurityEx';
function SetRect(var lprc: TRect; xLeft, yTop, xRight, yBottom: Integer): BOOL; external 'user32' name 'SetRect';
function SetRectEmpty(var lprc: TRect): BOOL; external 'user32' name 'SetRectEmpty';
function SetScrollInfo(hWnd: HWND; BarFlag: Integer; const ScrollInfo: TScrollInfo; Redraw: BOOL): Integer; external 'user32' name 'SetScrollInfo';
function SetSysColors(cElements: Integer; const lpaElements; const lpaRgbValues): BOOL; external 'user32' name 'SetSysColors';
//function SetSystemTime(const lpSystemTime: TSystemTime): BOOL; external 'kernel32' name 'SetSystemTime';
function SetThreadContext(hThread: THandle; const lpContext: TContext): BOOL; external 'kernel32' name 'SetThreadContext';
//function SetTimeZoneInformation(const lpTimeZoneInformation: TTimeZoneInformation): BOOL; external 'kernel32' name 'SetTimeZoneInformation';
function SetUserObjectSecurity(hObj: THandle; var pSIRequested: DWORD; pSID: PSecurityDescriptor): BOOL;external 'user32' name 'SetUserObjectSecurity';
function SetWaitableTimer(hTimer: THandle; var lpDueTime: TLargeInteger; lPeriod: Longint; pfnCompletionRoutine: TFNTimerAPCRoutine; lpArgToCompletionRoutine: Pointer; fResume: BOOL): BOOL;external 'kernel32' name 'SetWaitableTimer';
function SetWinMetaFileBits(p1: UINT; p2: PChar; p3: HDC; const p4: TMetaFilePict): HENHMETAFILE; external 'gdi32' name 'SetWinMetaFileBits';
//function SetWorldTransform(DC: HDC; const p2: TXForm): BOOL; external 'gdi32' name 'SetWorldTransform';
function StartDoc(DC: HDC; const p2: TDocInfo): Integer;external 'gdi32' name 'StartDocA';
function StartDocA(DC: HDC; const p2: TDocInfoA): Integer; external 'gdi32' name 'StartDocA';
//function StartDocW(DC: HDC; const p2: TDocInfoW): Integer; external 'gdi32' name 'StartDocW';
//function StretchDIBits(DC: HDC; DestX, DestY, DestWidth, DestHegiht, SrcX, SrcY, SrcWidth, SrcHeight: Integer; Bits: Pointer; var BitsInfo: TBitmapInfo; Usage: UINT; Rop: DWORD): Integer; external 'gdi32' name 'StretchDIBits';
function SubtractRect(var lprcDst: TRect; const lprcSrc1, lprcSrc2: TRect): BOOL; external 'user32' name 'SubtractRect';
function SystemTimeToFileTime(const lpSystemTime: TSystemTime; var lpFileTime: TFileTime): BOOL; external 'kernel32' name 'SystemTimeToFileTime';
function SystemTimeToTzSpecificLocalTime(lpTimeZoneInformation: PTimeZoneInformation; var lpUniversalTime, lpLocalTime: TSystemTime): BOOL; external 'kernel32' name 'SystemTimeToTzSpecificLocalTime';
function TabbedTextOut(hDC: HDC; X, Y: Integer; lpString: PChar; nCount, nTabPositions: Integer; var lpnTabStopPositions; nTabOrigin: Integer): Longint;external 'user32' name 'TabbedTextOutA';
function TabbedTextOutA(hDC: HDC; X, Y: Integer; lpString: LPCSTR; nCount, nTabPositions: Integer; var lpnTabStopPositions; nTabOrigin: Integer): Longint; external 'user32' name 'TabbedTextOutA';
function TabbedTextOutW(hDC: HDC; X, Y: Integer; lpString: LPWSTR; nCount, nTabPositions: Integer; var lpnTabStopPositions; nTabOrigin: Integer): Longint; external 'user32' name 'TabbedTextOutW';
//function ToAscii(uVirtKey, uScanCode: UINT; const KeyState: TKeyboardState; lpChar: PChar; uFlags: UINT): Integer; external 'user32' name 'ToAscii';
//function ToAsciiEx(uVirtKey: UINT; uScanCode: UINT; const KeyState: TKeyboardState; lpChar: PChar; uFlags: UINT; dwhkl: HKL): Integer; external 'user32' name 'ToAsciiEx';
//function ToUnicode(wVirtKey, wScanCode: UINT; const KeyState: TKeyboardState; var pwszBuff; cchBuff: Integer; wFlags: UINT): Integer; external 'user32' name 'ToUnicode';
// Careful, NT and higher only.
function TrackMouseEvent(var EventTrack: TTrackMouseEvent): BOOL;external 'user32' name 'TrackMouseEvent';
function TrackMouseEvent(lpEventTrack: PTrackMouseEvent): BOOL;external 'user32' name 'TrackMouseEvent';
function TrackPopupMenu(hMenu:HMENU; uFlags:UINT; x:longint; y:longint; nReserved:longint;hWnd:HWND; prcRect: PRect):WINBOOL;  external 'user32' name 'TrackPopupMenu';
function TransactNamedPipe(hNamedPipe: THandle; lpInBuffer: Pointer; nInBufferSize: DWORD; lpOutBuffer: Pointer; nOutBufferSize: DWORD; var lpBytesRead: DWORD; lpOverlapped: POverlapped): BOOL; external 'kernel32' name 'TransactNamedPipe';
function TranslateAccelerator(hWnd: HWND; hAccTable: HACCEL; var lpMsg: TMsg): Integer;external 'user32' name 'TranslateAcceleratorA';
function TranslateAcceleratorA(hWnd: HWND; hAccTable: HACCEL; var lpMsg: TMsg): Integer; external 'user32' name 'TranslateAcceleratorA';
function TranslateAcceleratorW(hWnd: HWND; hAccTable: HACCEL; var lpMsg: TMsg): Integer; external 'user32' name 'TranslateAcceleratorW';
function TranslateCharsetInfo(var lpSrc: DWORD; var lpCs: TCharsetInfo; dwFlags: DWORD): BOOL; external 'gdi32' name 'TranslateCharsetInfo';
function TranslateMDISysAccel(hWndClient: HWND; const lpMsg: TMsg): BOOL; external 'user32' name 'TranslateMDISysAccel';
function TranslateMessage(const lpMsg: TMsg): BOOL; external 'user32' name 'TranslateMessage';
//function TransparentDIBits(DC: HDC; p2, p3, p4, p5: Integer; const p6: Pointer; const p7: PBitmapInfo; p8: UINT; p9, p10, p11, p12: Integer; p13: UINT): BOOL;external 'gdi32' name 'TransparentDIBits';
{$ifndef NO_SMART_LINK}
//function TryEnterCriticalSection(var lpCriticalSection: TRTLCriticalSection): BOOL;external 'kernel32' name 'TryEnterCriticalSection';
{$endif NO_SMART_LINK}
function UnhandledExceptionFilter(const ExceptionInfo: TExceptionPointers): Longint; external 'kernel32' name 'UnhandledExceptionFilter';
function UnionRect(var lprcDst: TRect; const lprcSrc1, lprcSrc2: TRect): BOOL; external 'user32' name 'UnionRect';
function UnlockFileEx(hFile: THandle; dwReserved, nNumberOfBytesToUnlockLow: DWORD; nNumberOfBytesToUnlockHigh: DWORD; const lpOverlapped: TOverlapped): BOOL; external 'kernel32' name 'UnlockFileEx';
function VerFindFile(uFlags: DWORD; szFileName, szWinDir, szAppDir, szCurDir: PChar; var lpuCurDirLen: UINT; szDestDir: PChar; var lpuDestDirLen: UINT): DWORD;external 'version' name 'VerFindFileA';
function VerFindFileA(uFlags: DWORD; szFileName, szWinDir, szAppDir, szCurDir: LPCSTR; var lpuCurDirLen: UINT; szDestDir: LPCSTR; var lpuDestDirLen: UINT): DWORD; external 'version' name 'VerFindFileA';
function VerFindFileW(uFlags: DWORD; szFileName, szWinDir, szAppDir, szCurDir: LPWSTR; var lpuCurDirLen: UINT; szDestDir: LPWSTR; var lpuDestDirLen: UINT): DWORD; external 'version' name 'VerFindFileW';
function VerInstallFile(uFlags: DWORD; szSrcFileName, szDestFileName, szSrcDir, szDestDir, szCurDir, szTmpFile: PChar; var lpuTmpFileLen: UINT): DWORD;external 'version' name 'VerInstallFileA';
function VerInstallFileA(uFlags: DWORD; szSrcFileName, szDestFileName, szSrcDir, szDestDir, szCurDir, szTmpFile: LPCSTR; var lpuTmpFileLen: UINT): DWORD; external 'version' name 'VerInstallFileA';
function VerInstallFileW(uFlags: DWORD; szSrcFileName, szDestFileName, szSrcDir, szDestDir, szCurDir, szTmpFile: LPWSTR; var lpuTmpFileLen: UINT): DWORD; external 'version' name 'VerInstallFileW';
function VerQueryValue(pBlock: Pointer; lpSubBlock: PChar; var lplpBuffer: Pointer; var puLen: UINT): BOOL;external 'version' name 'VerQueryValueA';
function VerQueryValueA(pBlock: Pointer; lpSubBlock: LPCSTR; var lplpBuffer: Pointer; var puLen: UINT): BOOL; external 'version' name 'VerQueryValueA';
function VerQueryValueW(pBlock: Pointer; lpSubBlock: LPWSTR; var lplpBuffer: Pointer; var puLen: UINT): BOOL; external 'version' name 'VerQueryValueW';
function VirtualProtect(lpAddress:LPVOID; dwSize:PTRUINT; flNewProtect:DWORD; var lpflOldProtect:DWORD):WINBOOL; external 'kernel32' name 'VirtualProtect';
function VirtualQuery(lpAddress: Pointer; var lpBuffer: TMemoryBasicInformation; dwLength: PTRUINT): PTRUINT; external 'kernel32' name 'VirtualQuery';
function VirtualQueryEx(hProcess: THandle; lpAddress: Pointer; var lpBuffer: TMemoryBasicInformation; dwLength: PTRUINT): PTRUINT; external 'kernel32' name 'VirtualQueryEx';
function WaitCommEvent(hFile: THandle; var lpEvtMask: DWORD; lpOverlapped: POverlapped): BOOL; external 'kernel32' name 'WaitCommEvent';
function WaitForDebugEvent(var lpDebugEvent: TDebugEvent; dwMilliseconds: DWORD): BOOL; external 'kernel32' name 'WaitForDebugEvent';
function wglDescribeLayerPlane(DC: HDC; iPixelFormat, iLayerPlane: Integer; nBytes: Cardinal; var pfd: TLayerPlaneDescriptor): BOOL;external 'opengl32' name 'wglDescribeLayerPlane';
function wglGetLayerPaletteEntries(DC: HDC; iLayerPlane, iStart, cEntries: Integer; var pcr): Integer;external 'opengl32' name 'wglGetLayerPaletteEntries';
function wglSetLayerPaletteEntries(DC: HDC; iLayerPlane, iStart, cEntries: Integer; var pcr): Integer;external 'opengl32' name 'wglSetLayerPaletteEntries';
//function wglSwapMultipleBuffers(p1: UINT; const p2: PWGLSwap): DWORD;external 'opengl32' name 'wglSwapMultipleBuffers';
//function WinSubmitCertificate(var lpCertificate: TWinCertificate): BOOL;external 'imaghlp' name 'WinSubmitCertificate';
//function WinVerifyTrust(hwnd: HWND; const ActionID: TGUID; ActionData: Pointer): Longint;external 'imaghlp' name 'WinVerifyTrust';
function WNetAddConnection2(var lpNetResource: TNetResource; lpPassword, lpUserName: PChar; dwFlags: DWORD): DWORD;external 'mpr' name 'WNetAddConnection2A';
function WNetAddConnection2A(var lpNetResource: TNetResourceA; lpPassword, lpUserName: LPCSTR; dwFlags: DWORD): DWORD; external 'mpr' name 'WNetAddConnection2A';
//function WNetAddConnection2W(var lpNetResource: TNetResourceW; lpPassword, lpUserName: LPWSTR; dwFlags: DWORD): DWORD; external 'mpr' name 'WNetAddConnection2W';
function WNetAddConnection3(hwndOwner: HWND; var lpNetResource: TNetResource; lpPassword, lpUserName: PChar; dwFlags: DWORD): DWORD;external 'mpr' name 'WNetAddConnection3A';
function WNetAddConnection3A(hwndOwner: HWND; var lpNetResource: TNetResourceA; lpPassword, lpUserName: LPCSTR; dwFlags: DWORD): DWORD; external 'mpr' name 'WNetAddConnection3A';
//function WNetAddConnection3W(hwndOwner: HWND; var lpNetResource: TNetResourceW; lpPassword, lpUserName: LPWSTR; dwFlags: DWORD): DWORD; external 'mpr' name 'WNetAddConnection3W';
function WNetConnectionDialog1(var lpConnDlgStruct: TConnectDlgStruct): DWORD;external 'mpr' name 'WNetConnectionDialog1A';
function WNetConnectionDialog1A(var lpConnDlgStruct: TConnectDlgStruct): DWORD; external 'mpr' name 'WNetConnectionDialog1A';
//function WNetConnectionDialog1W(var lpConnDlgStruct: TConnectDlgStruct): DWORD; external 'mpr' name 'WNetConnectionDialog1W';
function WNetDisconnectDialog1(var lpConnDlgStruct: TDiscDlgStruct): DWORD;external 'mpr' name 'WNetDisconnectDialog1A';
function WNetDisconnectDialog1A(var lpConnDlgStruct: TDiscDlgStructA): DWORD; external 'mpr' name 'WNetDisconnectDialog1A';
//function WNetDisconnectDialog1W(var lpConnDlgStruct: TDiscDlgStructW): DWORD; external 'mpr' name 'WNetDisconnectDialog1W';
function WNetEnumResource(hEnum: THandle; var lpcCount: DWORD; lpBuffer: Pointer; var lpBufferSize: DWORD): DWORD;external 'mpr' name 'WNetEnumResourceA';
function WNetEnumResourceA(hEnum: THandle; var lpcCount: DWORD; lpBuffer: Pointer; var lpBufferSize: DWORD): DWORD; external 'mpr' name 'WNetEnumResourceA';
function WNetEnumResourceW(hEnum: THandle; var lpcCount: DWORD; lpBuffer: Pointer; var lpBufferSize: DWORD): DWORD; external 'mpr' name 'WNetEnumResourceW';
function WNetGetConnection(lpLocalName: PChar; lpRemoteName: PChar; var lpnLength: DWORD): DWORD;external 'mpr' name 'WNetGetConnectionA';
function WNetGetConnectionA(lpLocalName: LPCSTR; lpRemoteName: LPCSTR; var lpnLength: DWORD): DWORD; external 'mpr' name 'WNetGetConnectionA';
function WNetGetConnectionW(lpLocalName: LPWSTR; lpRemoteName: LPWSTR; var lpnLength: DWORD): DWORD; external 'mpr' name 'WNetGetConnectionW';
function WNetGetLastError(var lpError: DWORD; lpErrorBuf: PChar; nErrorBufSize: DWORD; lpNameBuf: PChar; nNameBufSize: DWORD): DWORD;external 'mpr' name 'WNetGetLastErrorA';
function WNetGetLastErrorA(var lpError: DWORD; lpErrorBuf: LPCSTR; nErrorBufSize: DWORD; lpNameBuf: LPCSTR; nNameBufSize: DWORD): DWORD; external 'mpr' name 'WNetGetLastErrorA';
function WNetGetLastErrorW(var lpError: DWORD; lpErrorBuf: LPWSTR; nErrorBufSize: DWORD; lpNameBuf: LPWSTR; nNameBufSize: DWORD): DWORD; external 'mpr' name 'WNetGetLastErrorW';
function WNetGetNetworkInformation(lpProvider: PChar; var lpNetInfoStruct: TNetInfoStruct): DWORD;external 'mpr' name 'WNetGetNetworkInformationA';
function WNetGetNetworkInformationA(lpProvider: LPCSTR; var lpNetInfoStruct: TNetInfoStruct): DWORD; external 'mpr' name 'WNetGetNetworkInformationA';
function WNetGetNetworkInformationW(lpProvider: LPWSTR; var lpNetInfoStruct: TNetInfoStruct): DWORD; external 'mpr' name 'WNetGetNetworkInformationW';
function WNetGetProviderName(dwNetType: DWORD; lpProviderName: PChar; var lpBufferSize: DWORD): DWORD;external 'mpr' name 'WNetGetProviderNameA';
function WNetGetProviderNameA(dwNetType: DWORD; lpProviderName: LPCSTR; var lpBufferSize: DWORD): DWORD; external 'mpr' name 'WNetGetProviderNameA';
function WNetGetProviderNameW(dwNetType: DWORD; lpProviderName: LPWSTR; var lpBufferSize: DWORD): DWORD; external 'mpr' name 'WNetGetProviderNameW';
function WNetGetResourceParent(lpNetResource: PNetResource; lpBuffer: Pointer; var cbBuffer: DWORD): DWORD;external 'mpr' name 'WNetGetResourceParentA';
function WNetGetResourceParentA(lpNetResource: PNetResourceA; lpBuffer: Pointer; var cbBuffer: DWORD): DWORD;external 'mpr' name 'WNetGetResourceParentA';
//function WNetGetResourceParentW(lpNetResource: PNetResourceW; lpBuffer: Pointer; var cbBuffer: DWORD): DWORD;external 'mpr' name 'WNetGetResourceParentW';
function WNetGetUniversalName(lpLocalPath: PChar; dwInfoLevel: DWORD; lpBuffer: Pointer; var lpBufferSize: DWORD): DWORD;external 'mpr' name 'WNetGetUniversalNameA';
function WNetGetUniversalNameA(lpLocalPath: LPCSTR; dwInfoLevel: DWORD; lpBuffer: Pointer; var lpBufferSize: DWORD): DWORD; external 'mpr' name 'WNetGetUniversalNameA';
function WNetGetUniversalNameW(lpLocalPath: LPWSTR; dwInfoLevel: DWORD; lpBuffer: Pointer; var lpBufferSize: DWORD): DWORD; external 'mpr' name 'WNetGetUniversalNameW';
function WNetGetUser(lpName: PChar; lpUserName: PChar; var lpnLength: DWORD): DWORD;external 'mpr' name 'WNetGetUserA';
function WNetGetUserA(lpName: LPCSTR; lpUserName: LPCSTR; var lpnLength: DWORD): DWORD; external 'mpr' name 'WNetGetUserA';
function WNetGetUserW(lpName: LPWSTR; lpUserName: LPWSTR; var lpnLength: DWORD): DWORD; external 'mpr' name 'WNetGetUserW';
function WNetOpenEnum(dwScope, dwType, dwUsage: DWORD; lpNetResource: PNetResource; var lphEnum: THandle): DWORD;external 'mpr' name 'WNetOpenEnumA';
function WNetOpenEnumA(dwScope, dwType, dwUsage: DWORD; lpNetResource: PNetResourceA; var lphEnum: THandle): DWORD; external 'mpr' name 'WNetOpenEnumA';
//function WNetOpenEnumW(dwScope, dwType, dwUsage: DWORD; lpNetResource: PNetResourceW; var lphEnum: THandle): DWORD; external 'mpr' name 'WNetOpenEnumW';
function WNetUseConnection(hwndOwner: HWND; var lpNetResource: TNetResource; lpUserID: PChar; lpPassword: PChar; dwFlags: DWORD; lpAccessName: PChar; var lpBufferSize: DWORD; var lpResult: DWORD): DWORD;external 'mpr' name 'WNetUseConnectionA';
function WNetUseConnectionA(hwndOwner: HWND; var lpNetResource: TNetResourceA; lpUserID: LPCSTR; lpPassword: LPCSTR; dwFlags: DWORD; lpAccessName: LPCSTR; var lpBufferSize: DWORD; var lpResult: DWORD): DWORD; external 'mpr' name 'WNetUseConnectionA';
//function WNetUseConnectionW(hwndOwner: HWND; var lpNetResource: TNetResourceW; lpUserID: LPWSTR; lpPassword: LPWSTR; dwFlags: DWORD; lpAccessName: LPWSTR; var lpBufferSize: DWORD; var lpResult: DWORD): DWORD; external 'mpr' name 'WNetUseConnectionW';
function WriteConsole(hConsoleOutput: THandle; const lpBuffer: Pointer; nNumberOfCharsToWrite: DWORD; var lpNumberOfCharsWritten: DWORD; lpReserved: Pointer): BOOL;external 'kernel32' name 'WriteConsoleA';
function WriteConsoleA(hConsoleOutput: THandle; const lpBuffer: Pointer; nNumberOfCharsToWrite: DWORD; var lpNumberOfCharsWritten: DWORD; lpReserved: Pointer): BOOL; external 'kernel32' name 'WriteConsoleA';
function WriteConsoleInput(hConsoleInput: THandle; const lpBuffer: TInputRecord; nLength: DWORD; var lpNumberOfEventsWritten: DWORD): BOOL;external 'kernel32' name 'WriteConsoleInputA';
function WriteConsoleInputA(hConsoleInput: THandle; const lpBuffer: TInputRecord; nLength: DWORD; var lpNumberOfEventsWritten: DWORD): BOOL; external 'kernel32' name 'WriteConsoleInputA';
function WriteConsoleInputW(hConsoleInput: THandle; const lpBuffer: TInputRecord; nLength: DWORD; var lpNumberOfEventsWritten: DWORD): BOOL; external 'kernel32' name 'WriteConsoleInputW';
function WriteConsoleOutput(hConsoleOutput: THandle; lpBuffer: Pointer; dwBufferSize, dwBufferCoord: TCoord; var lpWriteRegion: TSmallRect): BOOL;external 'kernel32' name 'WriteConsoleOutputA';
function WriteConsoleOutputA(hConsoleOutput: THandle; lpBuffer: Pointer; dwBufferSize, dwBufferCoord: TCoord; var lpWriteRegion: TSmallRect): BOOL; external 'kernel32' name 'WriteConsoleOutputA';
function WriteConsoleOutputAttribute(hConsoleOutput: THandle; lpAttribute: Pointer; nLength: DWORD; dwWriteCoord: TCoord; var lpNumberOfAttrsWritten: DWORD): BOOL; external 'kernel32' name 'WriteConsoleOutputAttribute';
function WriteConsoleOutputCharacter(hConsoleOutput: THandle;lpCharacter: PChar; nLength: DWORD; dwWriteCoord: TCoord; var lpNumberOfCharsWritten: DWORD): BOOL;external 'kernel32' name 'WriteConsoleOutputCharacterA';
function WriteConsoleOutputCharacterA(hConsoleOutput: THandle;lpCharacter: LPCSTR; nLength: DWORD; dwWriteCoord: TCoord; var lpNumberOfCharsWritten: DWORD): BOOL; external 'kernel32' name 'WriteConsoleOutputCharacterA';
function WriteConsoleOutputCharacterW(hConsoleOutput: THandle;lpCharacter: LPWSTR; nLength: DWORD; dwWriteCoord: TCoord; var lpNumberOfCharsWritten: DWORD): BOOL; external 'kernel32' name 'WriteConsoleOutputCharacterW';
function WriteConsoleOutputW(hConsoleOutput: THandle; lpBuffer: Pointer; dwBufferSize, dwBufferCoord: TCoord; var lpWriteRegion: TSmallRect): BOOL; external 'kernel32' name 'WriteConsoleOutputW';
function WriteConsoleW(hConsoleOutput: THandle; const lpBuffer: Pointer; nNumberOfCharsToWrite: DWORD; var lpNumberOfCharsWritten: DWORD; lpReserved: Pointer): BOOL; external 'kernel32' name 'WriteConsoleW';
function WriteFile(hFile: THandle; const Buffer; nNumberOfBytesToWrite: DWORD; var lpNumberOfBytesWritten: DWORD; lpOverlapped: POverlapped): BOOL; external 'kernel32' name 'WriteFile';
function WriteFileEx(hFile: THandle; lpBuffer: Pointer; nNumberOfBytesToWrite: DWORD; const lpOverlapped: TOverlapped; lpCompletionRoutine: FARPROC): BOOL; external 'kernel32' name 'WriteFileEx';
function WritePrivateProfileStructA(lpszSection, lpszKey: LPCSTR; lpStruct: LPVOID; uSizeStruct: UINT; szFile: LPCSTR): BOOL; external 'kernel32' name 'WritePrivateProfileStructA';
function WritePrivateProfileStructW(lpszSection, lpszKey: LPCWSTR; lpStruct: LPVOID; uSizeStruct: UINT; szFile: LPCWSTR): BOOL; external 'kernel32' name 'WritePrivateProfileStructW';
function WritePrivateProfileStruct(lpszSection, lpszKey: LPCTSTR; lpStruct: LPVOID; uSizeStruct: UINT; szFile: LPCTSTR): BOOL; external 'kernel32' name 'WritePrivateProfileStructA';
function WriteProcessMemory(hProcess: THandle; const lpBaseAddress: Pointer; lpBuffer: Pointer; nSize: PTRUINT; var lpNumberOfBytesWritten: PTRUINT): BOOL; external 'kernel32' name 'WriteProcessMemory';
function SHFileOperation(Var para1: SHFILEOPSTRUCT):longint; external 'shell32' name 'SHFileOperation';
// these are old Win16 funcs that under win32 are aliases for several char* funcs.
// exist under Win32 (even in SDK's from 2002), but are officially "depreciated"
function AnsiNext(const lpsz: LPCSTR): LPSTR;external 'user32' name 'CharNextA';
function AnsiPrev(const lpszStart: LPCSTR; const lpszCurrent: LPCSTR): LPSTR;external 'user32' name 'CharPrevA';
function AnsiToOem(const lpszSrc: LPCSTR; lpszDst: LPSTR): BOOL;external 'user32' name 'CharToOemA';
function OemToAnsi(const lpszSrc: LPCSTR; lpszDst: LPSTR): BOOL;external 'user32' name 'OemToCharA';
function AnsiToOemBuff(lpszSrc:LPCSTR; lpszDst:LPSTR; cchDstLength:DWORD):WINBOOL; external 'user32' name 'CharToOemBuffA';
function OemToAnsiBuff(lpszSrc:LPCSTR; lpszDst:LPSTR; cchDstLength:DWORD):WINBOOL; external 'user32' name 'OemToCharBuffA';
function AnsiUpper(lpsz:LPSTR):LPSTR; external 'user32' name 'CharUpperA';
function AnsiUpperBuff(lpsz:LPSTR; cchLength:DWORD):DWORD; external 'user32' name 'CharUpperBuffA';
function AnsiLower(lpsz:LPSTR):LPSTR; external 'user32' name 'CharLowerA';
function AnsiLowerBuff(lpsz:LPSTR; cchLength:DWORD):DWORD; external 'user32' name 'CharLowerBuffA';
function CreateIcon(hInstance:HINST; nWidth:longint; nHeight:longint; cPlanes:BYTE; cBitsPixel:BYTE;lpbANDbits:pointer; lpbXORbits:pointer):HICON; external 'user32' name 'CreateIcon';
function GetKeyboardState(var KeyState:TKeyboardState):WINBOOL; external 'user32' name 'GetKeyboardState';
function SetKeyboardState(var KeyState:TKeyboardState):WINBOOL; external 'user32' name 'SetKeyboardState';
function GetWindowThreadProcessId(hWnd:HWND;var lpdwProcessId:DWORD):DWORD; external 'user32' name 'GetWindowThreadProcessId';
function HwndMSWheel(var puiMsh_MsgMouseWheel, puiMsh_Msg3DSupport,puiMsh_MsgScrollLines: UINT;
  var pf3DSupport: BOOL; var piScrollLines: Integer): HWND;
function CreateWaitableTimer(lpTimerAttributes :LPSECURITY_ATTRIBUTES; bManualReset:BOOl;lpTimerName:LPCTSTR):THandle;external 'kernel32' name 'CreateWaitableTimerA';
function OpenWaitableTimer(dwDesiredAccess:DWORD;bInheritHandle:BOOL;lpTimerName:LPCTSTR):THandle;external 'kernel32' name 'OpenWaitableTimerA';
//function PropertySheetA(p:TPROPSHEETHEADER):longint; external 'comctl32' name 'PropertySheetA';

// windows because of Delphi compat.

{$calling default}
function InterLockedIncrement (var Target: longint) : longint; register; external name 'FPC_INTERLOCKEDINCREMENT';
function InterLockedDecrement (var Target: longint) : longint; register; external name 'FPC_INTERLOCKEDDECREMENT';
function InterLockedExchange (var Target: longint;Source : longint) : longint; register; external name 'FPC_INTERLOCKEDEXCHANGE';
function InterLockedExchangeAdd (var Target: longint;Source : longint) : longint; register; external name 'FPC_INTERLOCKEDEXCHANGEADD';
function InterLockedExchangeAdd (Target: plongint;Source : longint) : longint; register; external name 'FPC_INTERLOCKEDEXCHANGEADD';
function InterlockedCompareExchange(var Target: longint; NewValue: longint; Comperand: longint): longint; register; external name 'FPC_INTERLOCKEDCOMPAREEXCHANGE';
{$calling stdcall}

{$endif read_interface}


{$ifdef read_implementation}

function Succeeded(Status : HRESULT) : BOOL;
  begin
     Succeeded:=Status and HRESULT($80000000)=0;
  end;

function Failed(Status : HRESULT) : BOOL;
  begin
     Failed:=Status and HRESULT($80000000)<>0;
  end;

function IsError(Status : HRESULT) : BOOL;
  begin
     IsError:=(Status shr 31)=SEVERITY_ERROR;
  end;

function HResultCode(hr : HRESULT) : Longint;
  begin
     HResultCode:=hr and $0000ffff;
  end;

function HResultFacility(hr : HRESULT) : Longint;
  begin
     HResultFacility:=(hr shr 16) and $00001fff;
  end;

function HResultSeverity(hr : HRESULT) : Longint;
  begin
     HResultSeverity:=(hr shr 31) and $00000001;
  end;

function MakeResult(p1,p2,mask : Longint): HRESULT;
  begin
     MakeResult:=(p1 shl 31) or (p2 shl 16) or mask;
  end;

function HResultFromWin32(x : Longint) : HRESULT;
  begin
     HResultFromWin32:=x;
     if HResultFromWin32<>0 then
       HResultFromWin32:=((HResultFromWin32 and $0000ffff) or
         (FACILITY_WIN32 shl 16) or HRESULT($80000000));
  end;

function HResultFromNT(x : Longint) : HRESULT;
  begin
     HResultFromNT:=x or FACILITY_NT_BIT;
  end;

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

function LOCALE_NEUTRAL: DWORD;
begin
  LOCALE_NEUTRAL := MAKELCID(MAKELANGID(LANG_NEUTRAL, SUBLANG_NEUTRAL), SORT_DEFAULT);
end;

function LOCALE_INVARIANT: DWORD;
begin
  LOCALE_INVARIANT := MAKELCID(MAKELANGID(LANG_INVARIANT, SUBLANG_NEUTRAL), SORT_DEFAULT);
end;

function SmallPointToPoint(const P: TSmallPoint): TPoint;
begin
  Result.X:=P.X;
  Result.Y:=P.Y;
end;

function PointToSmallPoint(const P: TPoint): TSmallPoint;
begin
  Result.X:=P.X;
  Result.Y:=P.Y;
end;

function GetCurrentTime: Longint;inline;
begin
  Result:=GetTickCount;
end;

function HwndMSWheel(var puiMsh_MsgMouseWheel, puiMsh_Msg3DSupport,puiMsh_MsgScrollLines: UINT;
  var pf3DSupport: BOOL; var piScrollLines: Integer): HWND;
begin
  Result:=FindWindow(MSH_WHEELMODULE_CLASS, MSH_WHEELMODULE_TITLE);
  puiMsh_MsgMouseWheel:=RegisterWindowMessage(MSH_MOUSEWHEEL);
  puiMsh_Msg3DSupport:=RegisterWindowMessage(MSH_WHEELSUPPORT);
  puiMsh_MsgScrollLines:=RegisterWindowMessage(MSH_SCROLL_LINES);
  if (puiMsh_Msg3DSupport<>0) and (Result<>0) then
    pf3DSupport:=BOOL(SendMessage(Result,puiMsh_Msg3DSupport,0,0))
  else
    pf3DSupport:=False;

  if (puiMsh_MsgScrollLines<>0)  and (Result<>0) then
    piScrollLines:=SendMessage(Result,puiMsh_MsgScrollLines,0,0)
  else
    piScrollLines:=3;
end;
{$endif read_implementation}

