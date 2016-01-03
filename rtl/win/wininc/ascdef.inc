{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    Contains the Ascii functions for windows unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  ASCIIFunctions.h

  Declarations for all the Win32 ASCII Functions

  Copyright (C) 1996 Free Software Foundation, Inc.

  Author:  Scott Christley <scottc@net-community.com>

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

function GetBinaryType(lpApplicationName:LPCSTR; lpBinaryType:LPDWORD):WINBOOL; external 'kernel32' name 'GetBinaryTypeA';
function GetShortPathName(lpszLongPath:LPCSTR; lpszShortPath:LPSTR; cchBuffer:DWORD):DWORD; external 'kernel32' name 'GetShortPathNameA';
function GetEnvironmentStrings : LPSTR; external 'kernel32' name 'GetEnvironmentStringsA';
function FreeEnvironmentStrings(_para1:LPSTR):WINBOOL; external 'kernel32' name 'FreeEnvironmentStringsA';
function FormatMessage(dwFlags:DWORD; lpSource:LPCVOID; dwMessageId:DWORD; dwLanguageId:DWORD; lpBuffer:LPSTR;nSize:DWORD; Arguments:va_list):DWORD; external 'kernel32' name 'FormatMessageA';
function CreateMailslot(lpName:LPCSTR; nMaxMessageSize:DWORD; lReadTimeout:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):HANDLE; external 'kernel32' name 'CreateMailslotA';
function lstrcmp(lpString1:LPCSTR; lpString2:LPCSTR):longint; external 'kernel32' name 'lstrcmpA';
function lstrcmpi(lpString1:LPCSTR; lpString2:LPCSTR):longint; external 'kernel32' name 'lstrcmpiA';
function lstrcpyn(lpString1:LPSTR; lpString2:LPCSTR; iMaxLength:longint):LPSTR; external 'kernel32' name 'lstrcpynA';
function lstrcpy(lpString1:LPSTR; lpString2:LPCSTR):LPSTR; external 'kernel32' name 'lstrcpyA';
function lstrcat(lpString1:LPSTR; lpString2:LPCSTR):LPSTR; external 'kernel32' name 'lstrcatA';
function lstrlen(lpString:LPCSTR):longint; external 'kernel32' name 'lstrlenA';
function CreateMutex(lpMutexAttributes:LPSECURITY_ATTRIBUTES; bInitialOwner:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'CreateMutexA';
function OpenMutex(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'OpenMutexA';
function CreateEvent(lpEventAttributes:LPSECURITY_ATTRIBUTES; bManualReset:WINBOOL; bInitialState:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'CreateEventA';
function OpenEvent(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'OpenEventA';
function CreateSemaphore(lpSemaphoreAttributes:LPSECURITY_ATTRIBUTES; lInitialCount:LONG; lMaximumCount:LONG; lpName:LPCSTR):HANDLE; external 'kernel32' name 'CreateSemaphoreA';
function OpenSemaphore(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'OpenSemaphoreA';
function CreateFileMapping(hFile:HANDLE; lpFileMappingAttributes:LPSECURITY_ATTRIBUTES; flProtect:DWORD; dwMaximumSizeHigh:DWORD; dwMaximumSizeLow:DWORD;lpName:LPCSTR):HANDLE; external 'kernel32' name 'CreateFileMappingA';
function OpenFileMapping(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'OpenFileMappingA';
function GetLogicalDriveStrings(nBufferLength:DWORD; lpBuffer:LPSTR):DWORD; external 'kernel32' name 'GetLogicalDriveStringsA';
function LoadLibrary(lpLibFileName:LPCSTR):HINST; external 'kernel32' name 'LoadLibraryA';
function LoadLibraryEx(lpLibFileName:LPCSTR; hFile:HANDLE; dwFlags:DWORD):HINST; external 'kernel32' name 'LoadLibraryExA';
function GetModuleFileName(hModule:HINST; lpFilename:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetModuleFileNameA';
function GetModuleHandle(lpModuleName:LPCSTR):HMODULE; external 'kernel32' name 'GetModuleHandleA';
procedure FatalAppExit(uAction:UINT; lpMessageText:LPCSTR); external 'kernel32' name 'FatalAppExitA';
function GetCommandLine : LPSTR; external 'kernel32' name 'GetCommandLineA';
function GetEnvironmentVariable(lpName:LPCSTR; lpBuffer:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetEnvironmentVariableA';
function SetEnvironmentVariable(lpName:LPCSTR; lpValue:LPCSTR):WINBOOL; external 'kernel32' name 'SetEnvironmentVariableA';
function ExpandEnvironmentStrings(lpSrc:LPCSTR; lpDst:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'ExpandEnvironmentStringsA';
procedure OutputDebugString(lpOutputString:LPCSTR); external 'kernel32' name 'OutputDebugStringA';
function FindResource(hModule:HINST; lpName:LPCSTR; lpType:LPCSTR):HRSRC; external 'kernel32' name 'FindResourceA';
function FindResourceEx(hModule:HINST; lpType:LPCSTR; lpName:LPCSTR; wLanguage:WORD):HRSRC; external 'kernel32' name 'FindResourceExA';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function EnumResourceTypes(hModule:HMODULE; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesA';
function EnumResourceNames(hModule:HMODULE; lpType:LPCSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesA';
function EnumResourceLanguages(hModule:HMODULE; lpType:LPCSTR; lpName:LPCSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesA';
=======
function EnumResourceTypes(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesA';
function EnumResourceNames(hModule:HINST; lpType:LPCSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesA';
function EnumResourceLanguages(hModule:HINST; lpType:LPCSTR; lpName:LPCSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesA';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypes(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesA';
function EnumResourceNames(hModule:HINST; lpType:LPCSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesA';
function EnumResourceLanguages(hModule:HINST; lpType:LPCSTR; lpName:LPCSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesA';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypes(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesA';
function EnumResourceNames(hModule:HINST; lpType:LPCSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesA';
function EnumResourceLanguages(hModule:HINST; lpType:LPCSTR; lpName:LPCSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesA';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypes(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesA';
function EnumResourceNames(hModule:HINST; lpType:LPCSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesA';
function EnumResourceLanguages(hModule:HINST; lpType:LPCSTR; lpName:LPCSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesA';
>>>>>>> origin/cpstrnew
function BeginUpdateResource(pFileName:LPCSTR; bDeleteExistingResources:WINBOOL):HANDLE; external 'kernel32' name 'BeginUpdateResourceA';
function UpdateResource(hUpdate:HANDLE; lpType:LPCSTR; lpName:LPCSTR; wLanguage:WORD; lpData:LPVOID;cbData:DWORD):WINBOOL; external 'kernel32' name 'UpdateResourceA';
function EndUpdateResource(hUpdate:HANDLE; fDiscard:WINBOOL):WINBOOL; external 'kernel32' name 'EndUpdateResourceA';
function GlobalAddAtom(lpString:LPCSTR):ATOM; external 'kernel32' name 'GlobalAddAtomA';
function GlobalFindAtom(lpString:LPCSTR):ATOM; external 'kernel32' name 'GlobalFindAtomA';
function GlobalGetAtomName(nAtom:ATOM; lpBuffer:LPSTR; nSize:longint):UINT; external 'kernel32' name 'GlobalGetAtomNameA';
function AddAtom(lpString:LPCSTR):ATOM; external 'kernel32' name 'AddAtomA';
function FindAtom(lpString:LPCSTR):ATOM; external 'kernel32' name 'FindAtomA';
function GetAtomName(nAtom:ATOM; lpBuffer:LPSTR; nSize:longint):UINT; external 'kernel32' name 'GetAtomNameA';
function GetProfileInt(lpAppName:LPCSTR; lpKeyName:LPCSTR; nDefault:WINT):UINT; external 'kernel32' name 'GetProfileIntA';
function GetProfileString(lpAppName:LPCSTR; lpKeyName:LPCSTR; lpDefault:LPCSTR; lpReturnedString:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetProfileStringA';
function WriteProfileString(lpAppName:LPCSTR; lpKeyName:LPCSTR; lpString:LPCSTR):WINBOOL; external 'kernel32' name 'WriteProfileStringA';
function GetProfileSection(lpAppName:LPCSTR; lpReturnedString:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetProfileSectionA';
function WriteProfileSection(lpAppName:LPCSTR; lpString:LPCSTR):WINBOOL; external 'kernel32' name 'WriteProfileSectionA';
function GetPrivateProfileInt(lpAppName:LPCSTR; lpKeyName:LPCSTR; nDefault:WINT; lpFileName:LPCSTR):UINT; external 'kernel32' name 'GetPrivateProfileIntA';
function GetPrivateProfileString(lpAppName:LPCSTR; lpKeyName:LPCSTR; lpDefault:LPCSTR; lpReturnedString:LPSTR; nSize:DWORD;lpFileName:LPCSTR):DWORD; external 'kernel32' name 'GetPrivateProfileStringA';
function WritePrivateProfileString(lpAppName:LPCSTR; lpKeyName:LPCSTR; lpString:LPCSTR; lpFileName:LPCSTR):WINBOOL; external 'kernel32' name 'WritePrivateProfileStringA';
function GetPrivateProfileSection(lpAppName:LPCSTR; lpReturnedString:LPSTR; nSize:DWORD; lpFileName:LPCSTR):DWORD; external 'kernel32' name 'GetPrivateProfileSectionA';
function WritePrivateProfileSection(lpAppName:LPCSTR; lpString:LPCSTR; lpFileName:LPCSTR):WINBOOL; external 'kernel32' name 'WritePrivateProfileSectionA';
function GetDriveType(lpRootPathName:LPCSTR):UINT; external 'kernel32' name 'GetDriveTypeA';
function GetSystemDirectory(lpBuffer:LPSTR; uSize:UINT):UINT; external 'kernel32' name 'GetSystemDirectoryA';
function GetTempPath(nBufferLength:DWORD; lpBuffer:LPSTR):DWORD; external 'kernel32' name 'GetTempPathA';
function GetTempFileName(lpPathName:LPCSTR; lpPrefixString:LPCSTR; uUnique:UINT; lpTempFileName:LPSTR):UINT; external 'kernel32' name 'GetTempFileNameA';
function GetWindowsDirectory(lpBuffer:LPSTR; uSize:UINT):UINT; external 'kernel32' name 'GetWindowsDirectoryA';
function SetCurrentDirectory(lpPathName:LPCSTR):WINBOOL; external 'kernel32' name 'SetCurrentDirectoryA';
function GetCurrentDirectory(nBufferLength:DWORD; lpBuffer:LPSTR):DWORD; external 'kernel32' name 'GetCurrentDirectoryA';
function GetDiskFreeSpace(lpRootPathName:LPCSTR; lpSectorsPerCluster:LPDWORD; lpBytesPerSector:LPDWORD; lpNumberOfFreeClusters:LPDWORD; lpTotalNumberOfClusters:LPDWORD):WINBOOL; external 'kernel32' name 'GetDiskFreeSpaceA';
function CreateDirectory(lpPathName:LPCSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):WINBOOL; external 'kernel32' name 'CreateDirectoryA';
function CreateDirectoryEx(lpTemplateDirectory:LPCSTR; lpNewDirectory:LPCSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):WINBOOL; external 'kernel32' name 'CreateDirectoryExA';
function RemoveDirectory(lpPathName:LPCSTR):WINBOOL; external 'kernel32' name 'RemoveDirectoryA';
function GetFullPathName(lpFileName:LPCSTR; nBufferLength:DWORD; lpBuffer:LPSTR; var lpFilePart:LPSTR):DWORD; external 'kernel32' name 'GetFullPathNameA';
function DefineDosDevice(dwFlags:DWORD; lpDeviceName:LPCSTR; lpTargetPath:LPCSTR):WINBOOL; external 'kernel32' name 'DefineDosDeviceA';
function QueryDosDevice(lpDeviceName:LPCSTR; lpTargetPath:LPSTR; ucchMax:DWORD):DWORD; external 'kernel32' name 'QueryDosDeviceA';
function CreateFile(lpFileName:LPCSTR; dwDesiredAccess:DWORD; dwShareMode:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES; dwCreationDisposition:DWORD;dwFlagsAndAttributes:DWORD; hTemplateFile:HANDLE):HANDLE; external 'kernel32' name 'CreateFileA';
function SetFileAttributes(lpFileName:LPCSTR; dwFileAttributes:DWORD):WINBOOL; external 'kernel32' name 'SetFileAttributesA';
function GetFileAttributes(lpFileName:LPCSTR):DWORD; external 'kernel32' name 'GetFileAttributesA';
function GetFileAttributesEx(lpFileName:LPCSTR; fInfoLevelId:TGET_FILEEX_INFO_LEVELS; lpFileInformation:pointer):BOOL;stdcall; external 'kernel32' name 'GetFileAttributesExA';
function GetCompressedFileSize(lpFileName:LPCSTR; lpFileSizeHigh:LPDWORD):DWORD; external 'kernel32' name 'GetCompressedFileSizeA';
function DeleteFile(lpFileName:LPCSTR):WINBOOL; external 'kernel32' name 'DeleteFileA';
function SearchPath(lpPath:LPCSTR; lpFileName:LPCSTR; lpExtension:LPCSTR; nBufferLength:DWORD; lpBuffer:LPSTR;lpFilePart:LPSTR):DWORD; external 'kernel32' name 'SearchPathA';
function CopyFile(lpExistingFileName:LPCSTR; lpNewFileName:LPCSTR; bFailIfExists:WINBOOL):WINBOOL; external 'kernel32' name 'CopyFileA';
function MoveFile(lpExistingFileName:LPCSTR; lpNewFileName:LPCSTR):WINBOOL; external 'kernel32' name 'MoveFileA';
function MoveFileEx(lpExistingFileName:LPCSTR; lpNewFileName:LPCSTR; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'MoveFileExA';
function CreateNamedPipe(lpName:LPCSTR; dwOpenMode:DWORD; dwPipeMode:DWORD; nMaxInstances:DWORD; nOutBufferSize:DWORD;nInBufferSize:DWORD; nDefaultTimeOut:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):HANDLE;
  external 'kernel32' name 'CreateNamedPipeA';
function GetNamedPipeHandleState(hNamedPipe:HANDLE; lpState:LPDWORD; lpCurInstances:LPDWORD; lpMaxCollectionCount:LPDWORD; lpCollectDataTimeout:LPDWORD;lpUserName:LPSTR; nMaxUserNameSize:DWORD):WINBOOL;
  external 'kernel32' name 'GetNamedPipeHandleStateA';
function CallNamedPipe(lpNamedPipeName:LPCSTR; lpInBuffer:LPVOID; nInBufferSize:DWORD; lpOutBuffer:LPVOID; nOutBufferSize:DWORD;lpBytesRead:LPDWORD; nTimeOut:DWORD):WINBOOL; external 'kernel32' name 'CallNamedPipeA';
function WaitNamedPipe(lpNamedPipeName:LPCSTR; nTimeOut:DWORD):WINBOOL; external 'kernel32' name 'WaitNamedPipeA';
function SetVolumeLabel(lpRootPathName:LPCSTR; lpVolumeName:LPCSTR):WINBOOL; external 'kernel32' name 'SetVolumeLabelA';
function GetVolumeInformation(lpRootPathName:LPCSTR; lpVolumeNameBuffer:LPSTR; nVolumeNameSize:DWORD; lpVolumeSerialNumber:LPDWORD; lpMaximumComponentLength:LPDWORD;lpFileSystemFlags:LPDWORD;
  lpFileSystemNameBuffer:LPSTR; nFileSystemNameSize:DWORD):WINBOOL; external 'kernel32' name 'GetVolumeInformationA';
function ClearEventLog(hEventLog:HANDLE; lpBackupFileName:LPCSTR):WINBOOL; external 'advapi32' name 'ClearEventLogA';
function BackupEventLog(hEventLog:HANDLE; lpBackupFileName:LPCSTR):WINBOOL; external 'advapi32' name 'BackupEventLogA';
function OpenEventLog(lpUNCServerName:LPCSTR; lpSourceName:LPCSTR):HANDLE; external 'advapi32' name 'OpenEventLogA';
function RegisterEventSource(lpUNCServerName:LPCSTR; lpSourceName:LPCSTR):HANDLE; external 'advapi32' name 'RegisterEventSourceA';
function OpenBackupEventLog(lpUNCServerName:LPCSTR; lpFileName:LPCSTR):HANDLE; external 'advapi32' name 'OpenBackupEventLogA';
function ReadEventLog(hEventLog:HANDLE; dwReadFlags:DWORD; dwRecordOffset:DWORD; lpBuffer:LPVOID; nNumberOfBytesToRead:DWORD;pnBytesRead:LPDWORD; pnMinNumberOfBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'ReadEventLogA';
function ReportEvent(hEventLog:HANDLE; wType:WORD; wCategory:WORD; dwEventID:DWORD; lpUserSid:PSID;wNumStrings:WORD; dwDataSize:DWORD; lpStrings:LPPCSTR; lpRawData:LPVOID):WINBOOL; external 'advapi32' name 'ReportEventA';
function AccessCheckAndAuditAlarm(SubsystemName:LPCSTR; HandleId:LPVOID; ObjectTypeName:LPSTR; ObjectName:LPSTR; SecurityDescriptor:PSECURITY_DESCRIPTOR;DesiredAccess:DWORD; GenericMapping:PGENERIC_MAPPING;
  ObjectCreation:WINBOOL; GrantedAccess:LPDWORD;AccessStatus:LPBOOL;pfGenerateOnClose:LPBOOL):WINBOOL; external 'advapi32' name 'AccessCheckAndAuditAlarmA';
function ObjectOpenAuditAlarm(SubsystemName:LPCSTR; HandleId:LPVOID; ObjectTypeName:LPSTR; ObjectName:LPSTR; pSecurityDescriptor:PSECURITY_DESCRIPTOR;ClientToken:HANDLE; DesiredAccess:DWORD; GrantedAccess:DWORD;
  Privileges:PPRIVILEGE_SET; ObjectCreation:WINBOOL;AccessGranted:WINBOOL; GenerateOnClose:LPBOOL):WINBOOL; external 'advapi32' name 'ObjectOpenAuditAlarmA';
function ObjectPrivilegeAuditAlarm(SubsystemName:LPCSTR; HandleId:LPVOID; ClientToken:HANDLE; DesiredAccess:DWORD; Privileges:PPRIVILEGE_SET;AccessGranted:WINBOOL):WINBOOL; external 'advapi32' name 'ObjectPrivilegeAuditAlarmA';
function ObjectCloseAuditAlarm(SubsystemName:LPCSTR; HandleId:LPVOID; GenerateOnClose:WINBOOL):WINBOOL; external 'advapi32' name 'ObjectCloseAuditAlarmA';
function PrivilegedServiceAuditAlarm(SubsystemName:LPCSTR; ServiceName:LPCSTR; ClientToken:HANDLE; Privileges:PPRIVILEGE_SET; AccessGranted:WINBOOL):WINBOOL; external 'advapi32' name 'PrivilegedServiceAuditAlarmA';
function SetFileSecurity(lpFileName:LPCSTR; SecurityInformation:SECURITY_INFORMATION; pSecurityDescriptor:PSECURITY_DESCRIPTOR):WINBOOL; external 'advapi32' name 'SetFileSecurityA';
function GetFileSecurity(lpFileName:LPCSTR; RequestedInformation:SECURITY_INFORMATION; pSecurityDescriptor:PSECURITY_DESCRIPTOR; nLength:DWORD; lpnLengthNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'GetFileSecurityA';
function FindFirstChangeNotification(lpPathName:LPCSTR; bWatchSubtree:WINBOOL; dwNotifyFilter:DWORD):HANDLE; external 'kernel32' name 'FindFirstChangeNotificationA';
function IsBadStringPtr(lpsz:LPCSTR; ucchMax:UINT):WINBOOL; external 'kernel32' name 'IsBadStringPtrA';
function LookupAccountSid(lpSystemName:LPCSTR; Sid:PSID; Name:LPSTR; cbName:LPDWORD; ReferencedDomainName:LPSTR;cbReferencedDomainName:LPDWORD; peUse:PSID_NAME_USE):WINBOOL; external 'advapi32' name 'LookupAccountSidA';
function LookupAccountName(lpSystemName:LPCSTR; lpAccountName:LPCSTR; Sid:PSID; cbSid:LPDWORD; ReferencedDomainName:LPSTR;cbReferencedDomainName:LPDWORD; peUse:PSID_NAME_USE):WINBOOL; external 'advapi32' name 'LookupAccountNameA';
function LookupPrivilegeValue(lpSystemName:LPCSTR; lpName:LPCSTR; lpLuid:PLUID):WINBOOL; external 'advapi32' name 'LookupPrivilegeValueA';
function LookupPrivilegeName(lpSystemName:LPCSTR; lpLuid:PLUID; lpName:LPSTR; cbName:LPDWORD):WINBOOL; external 'advapi32' name 'LookupPrivilegeNameA';
function LookupPrivilegeDisplayName(lpSystemName:LPCSTR; lpName:LPCSTR; lpDisplayName:LPSTR; cbDisplayName:LPDWORD; lpLanguageId:LPDWORD):WINBOOL; external 'advapi32' name 'LookupPrivilegeDisplayNameA';
function BuildCommDCB(lpDef:LPCSTR; lpDCB:LPDCB):WINBOOL; external 'kernel32' name 'BuildCommDCBA';
function BuildCommDCBAndTimeouts(lpDef:LPCSTR; lpDCB:LPDCB; lpCommTimeouts:LPCOMMTIMEOUTS):WINBOOL; external 'kernel32' name 'BuildCommDCBAndTimeoutsA';
function CommConfigDialog(lpszName:LPCSTR; hWnd:HWND; lpCC:LPCOMMCONFIG):WINBOOL; external 'kernel32' name 'CommConfigDialogA';
function GetDefaultCommConfig(lpszName:LPCSTR; lpCC:LPCOMMCONFIG; lpdwSize:LPDWORD):WINBOOL; external 'kernel32' name 'GetDefaultCommConfigA';
function SetDefaultCommConfig(lpszName:LPCSTR; lpCC:LPCOMMCONFIG; dwSize:DWORD):WINBOOL; external 'kernel32' name 'SetDefaultCommConfigA';
function GetComputerName(lpBuffer:LPSTR; nSize:LPDWORD):WINBOOL; external 'kernel32' name 'GetComputerNameA';
function SetComputerName(lpComputerName:LPCSTR):WINBOOL; external 'kernel32' name 'SetComputerNameA';
function GetCPInfoEx(Codepage:UINT; dwFlags:DWORD; CPInfoEx:LPCPINFOEX):WINBOOL; external 'kernel32' name 'GetCPInfoExA';
function GetUserName(lpBuffer:LPSTR; nSize:LPDWORD):WINBOOL; external 'advapi32' name 'GetUserNameA';
function wvsprintf(_para1:LPSTR; _para2:LPCSTR; arglist:va_list):longint; stdcall; external 'user32' name 'wvsprintfA';
function LoadKeyboardLayout(pwszKLID:LPCSTR; Flags:UINT):HKL; external 'user32' name 'LoadKeyboardLayoutA';
function GetKeyboardLayoutName(pwszKLID:LPSTR):WINBOOL; external 'user32' name 'GetKeyboardLayoutNameA';
function CreateDesktop(lpszDesktop:LPSTR; lpszDevice:LPSTR; pDevmode:LPDEVMODE; dwFlags:DWORD; dwDesiredAccess:DWORD;lpsa:LPSECURITY_ATTRIBUTES):HDESK; external 'user32' name 'CreateDesktopA';
function OpenDesktop(lpszDesktop:LPSTR; dwFlags:DWORD; fInherit:WINBOOL; dwDesiredAccess:DWORD):HDESK; external 'user32' name 'OpenDesktopA';
function EnumDesktops(hwinsta:HWINSTA; lpEnumFunc:DESKTOPENUMPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumDesktopsA';
function CreateWindowStation(lpwinsta:LPSTR; dwReserved:DWORD; dwDesiredAccess:DWORD; lpsa:LPSECURITY_ATTRIBUTES):HWINSTA; external 'user32' name 'CreateWindowStationA';
function OpenWindowStation(lpszWinSta:LPSTR; fInherit:WINBOOL; dwDesiredAccess:DWORD):HWINSTA; external 'user32' name 'OpenWindowStationA';
function EnumWindowStations(lpEnumFunc:ENUMWINDOWSTATIONPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumWindowStationsA';
function GetUserObjectInformation(hObj:HANDLE; nIndex:longint; pvInfo:PVOID; nLength:DWORD; lpnLengthNeeded:LPDWORD):WINBOOL; external 'user32' name 'GetUserObjectInformationA';
function SetUserObjectInformation(hObj:HANDLE; nIndex:longint; pvInfo:PVOID; nLength:DWORD):WINBOOL; external 'user32' name 'SetUserObjectInformationA';
function RegisterWindowMessage(lpString:LPCSTR):UINT; external 'user32' name 'RegisterWindowMessageA';
function GetMessage(lpMsg:LPMSG; hWnd:HWND; wMsgFilterMin:UINT; wMsgFilterMax:UINT):WINBOOL; external 'user32' name 'GetMessageA';
function DispatchMessage(lpMsg:LPMSG):LONG; external 'user32' name 'DispatchMessageA';
function PeekMessage(lpMsg:LPMSG; hWnd:HWND; wMsgFilterMin:UINT; wMsgFilterMax:UINT; wRemoveMsg:UINT):WINBOOL; external 'user32' name 'PeekMessageA';
function SendMessage(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'SendMessageA';
function SendMessageTimeout(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM; fuFlags:UINT;uTimeout:UINT; lpdwResult:PDWORD_PTR):LRESULT; external 'user32' name 'SendMessageTimeoutA';
function SendNotifyMessage(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'SendNotifyMessageA';
function SendMessageCallback(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM; lpResultCallBack:SENDASYNCPROC;dwData:ULONG_PTR):WINBOOL; external 'user32' name 'SendMessageCallbackA';
function PostMessage(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'PostMessageA';
function PostThreadMessage(idThread:DWORD; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'PostThreadMessageA';
function DefWindowProc(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefWindowProcA';
function CallWindowProc(lpPrevWndFunc:WNDPROC; hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'CallWindowProcA';
function RegisterClass(lpWndClass:LPWNDCLASS):ATOM; external 'user32' name 'RegisterClassA';
function UnregisterClass(lpClassName:LPCSTR; hInstance:HINST):WINBOOL; external 'user32' name 'UnregisterClassA';
function GetClassInfo(hInstance:HINST; lpClassName:LPCSTR; lpWndClass:LPWNDCLASS):WINBOOL; external 'user32' name 'GetClassInfoA';
function RegisterClassEx(_para1:LPWNDCLASSEX):ATOM; external 'user32' name 'RegisterClassExA';
function GetClassInfoEx(_para1:HINST; _para2:LPCSTR; _para3:LPWNDCLASSEX):WINBOOL; external 'user32' name 'GetClassInfoExA';
function CreateWindowEx(dwExStyle:DWORD; lpClassName:LPCSTR; lpWindowName:LPCSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
  external 'user32' name 'CreateWindowExA';
function CreateDialogParam(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):HWND; external 'user32' name 'CreateDialogParamA';
function CreateDialogIndirectParam(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):HWND; external 'user32' name 'CreateDialogIndirectParamA';
function DialogBoxParam(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):longint; external 'user32' name 'DialogBoxParamA';
function DialogBoxIndirectParam(hInstance:HINST; hDialogTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):longint; external 'user32' name 'DialogBoxIndirectParamA';
function SetDlgItemText(hDlg:HWND; nIDDlgItem:longint; lpString:LPCSTR):WINBOOL; external 'user32' name 'SetDlgItemTextA';
function GetDlgItemText(hDlg:HWND; nIDDlgItem:longint; lpString:LPSTR; nMaxCount:longint):UINT; external 'user32' name 'GetDlgItemTextA';
function SendDlgItemMessage(hDlg:HWND; nIDDlgItem:longint; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LONG; external 'user32' name 'SendDlgItemMessageA';
function DefDlgProc(hDlg:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefDlgProcA';
function CallMsgFilter(lpMsg:LPMSG; nCode:longint):WINBOOL; external 'user32' name 'CallMsgFilterA';
function RegisterClipboardFormat(lpszFormat:LPCSTR):UINT; external 'user32' name 'RegisterClipboardFormatA';
function GetClipboardFormatName(format:UINT; lpszFormatName:LPSTR; cchMaxCount:longint):longint; external 'user32' name 'GetClipboardFormatNameA';
function CharToOem(lpszSrc:LPCSTR; lpszDst:LPSTR):WINBOOL; external 'user32' name 'CharToOemA';
function OemToChar(lpszSrc:LPCSTR; lpszDst:LPSTR):WINBOOL; external 'user32' name 'OemToCharA';
function CharToOemBuff(lpszSrc:LPCSTR; lpszDst:LPSTR; cchDstLength:DWORD):WINBOOL; external 'user32' name 'CharToOemBuffA';
function OemToCharBuff(lpszSrc:LPCSTR; lpszDst:LPSTR; cchDstLength:DWORD):WINBOOL; external 'user32' name 'OemToCharBuffA';
function CharUpper(lpsz:LPSTR):LPSTR; external 'user32' name 'CharUpperA';
function CharUpperBuff(lpsz:LPSTR; cchLength:DWORD):DWORD; external 'user32' name 'CharUpperBuffA';
function CharLower(lpsz:LPSTR):LPSTR; external 'user32' name 'CharLowerA';
function CharLowerBuff(lpsz:LPSTR; cchLength:DWORD):DWORD; external 'user32' name 'CharLowerBuffA';
function CharNext(lpsz:LPCSTR):LPSTR; external 'user32' name 'CharNextA';
function CharPrev(lpszStart:LPCSTR; lpszCurrent:LPCSTR):LPSTR; external 'user32' name 'CharPrevA';
function IsCharAlpha(ch:CHAR):WINBOOL; external 'user32' name 'IsCharAlphaA';
function IsCharAlphaNumeric(ch:CHAR):WINBOOL; external 'user32' name 'IsCharAlphaNumericA';
function IsCharUpper(ch:CHAR):WINBOOL; external 'user32' name 'IsCharUpperA';
function IsCharLower(ch:CHAR):WINBOOL; external 'user32' name 'IsCharLowerA';
function GetKeyNameText(lParam:LONG; lpString:LPSTR; nSize:longint):longint; external 'user32' name 'GetKeyNameTextA';
function VkKeyScan(ch:CHAR):SHORT; external 'user32' name 'VkKeyScanA';
function VkKeyScanEx(ch:CHAR; dwhkl:HKL):SHORT; external 'user32' name 'VkKeyScanExA';
function MapVirtualKey(uCode:UINT; uMapType:UINT):UINT; external 'user32' name 'MapVirtualKeyA';
function MapVirtualKeyEx(uCode:UINT; uMapType:UINT; dwhkl:HKL):UINT; external 'user32' name 'MapVirtualKeyExA';
function LoadAccelerators(hInstance:HINST; lpTableName:LPCSTR):HACCEL; external 'user32' name 'LoadAcceleratorsA';
function CreateAcceleratorTable(_para1:LPACCEL; _para2:longint):HACCEL; external 'user32' name 'CreateAcceleratorTableA';
function CopyAcceleratorTable(hAccelSrc:HACCEL; lpAccelDst:LPACCEL; cAccelEntries:longint):longint; external 'user32' name 'CopyAcceleratorTableA';
function TranslateAccelerator(hWnd:HWND; hAccTable:HACCEL; lpMsg:LPMSG):longint; external 'user32' name 'TranslateAcceleratorA';
function LoadMenu(hInstance:HINST; lpMenuName:LPCSTR):HMENU; external 'user32' name 'LoadMenuA';
function LoadMenuIndirect(lpMenuTemplate:LPMENUTEMPLATE):HMENU; external 'user32' name 'LoadMenuIndirectA';
function ChangeMenu(hMenu:HMENU; cmd:UINT; lpszNewItem:LPCSTR; cmdInsert:UINT; flags:UINT):WINBOOL; external 'user32' name 'ChangeMenuA';
function GetMenuString(hMenu:HMENU; uIDItem:UINT; lpString:LPSTR; nMaxCount:longint; uFlag:UINT):longint; external 'user32' name 'GetMenuStringA';
function InsertMenu(hMenu:HMENU; uPosition:UINT; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCSTR):WINBOOL; external 'user32' name 'InsertMenuA';
function AppendMenu(hMenu:HMENU; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCSTR):WINBOOL; external 'user32' name 'AppendMenuA';
function ModifyMenu(hMnu:HMENU; uPosition:UINT; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCSTR):WINBOOL; external 'user32' name 'ModifyMenuA';
function InsertMenuItem(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPCMENUITEMINFO):WINBOOL; external 'user32' name 'InsertMenuItemA';
function GetMenuItemInfo(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPMENUITEMINFO):WINBOOL; external 'user32' name 'GetMenuItemInfoA';
function SetMenuItemInfo(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPCMENUITEMINFO):WINBOOL; external 'user32' name 'SetMenuItemInfoA';
function DrawText(hDC:HDC; lpString:LPCSTR; nCount:longint; lpRect:LPRECT; uFormat:UINT):longint; external 'user32' name 'DrawTextA';
function DrawTextEx(_para1:HDC; _para2:LPSTR; _para3:longint; _para4:LPRECT; _para5:UINT;_para6:LPDRAWTEXTPARAMS):longint; external 'user32' name 'DrawTextExA';
function GrayString(hDC:HDC; hBrush:HBRUSH; lpOutputFunc:GRAYSTRINGPROC; lpData:LPARAM; nCount:longint;X:longint; Y:longint; nWidth:longint; nHeight:longint):WINBOOL; external 'user32' name 'GrayStringA';
function DrawState(_para1:HDC; _para2:HBRUSH; _para3:DRAWSTATEPROC; _para4:LPARAM; _para5:WPARAM;_para6:longint; _para7:longint; _para8:longint; _para9:longint; _para10:UINT):WINBOOL; external 'user32' name 'DrawStateA';
function TabbedTextOut(hDC:HDC; X:longint; Y:longint; lpString:LPCSTR; nCount:longint;nTabPositions:longint; lpnTabStopPositions:LPINT; nTabOrigin:longint):LONG; external 'user32' name 'TabbedTextOutA';
function GetTabbedTextExtent(hDC:HDC; lpString:LPCSTR; nCount:longint; nTabPositions:longint; lpnTabStopPositions:LPINT):DWORD; external 'user32' name 'GetTabbedTextExtentA';
function SetProp(hWnd:HWND; lpString:LPCSTR; hData:HANDLE):WINBOOL; external 'user32' name 'SetPropA';
function GetProp(hWnd:HWND; lpString:LPCSTR):HANDLE; external 'user32' name 'GetPropA';
function RemoveProp(hWnd:HWND; lpString:LPCSTR):HANDLE; external 'user32' name 'RemovePropA';
function EnumPropsEx(hWnd:HWND; lpEnumFunc:PROPENUMPROCEX; lParam:LPARAM):longint; external 'user32' name 'EnumPropsExA';
function EnumProps(hWnd:HWND; lpEnumFunc:PROPENUMPROC):longint; external 'user32' name 'EnumPropsA';
function SetWindowText(hWnd:HWND; lpString:LPCSTR):WINBOOL; external 'user32' name 'SetWindowTextA';
function GetWindowText(hWnd:HWND; lpString:LPSTR; nMaxCount:longint):longint; external 'user32' name 'GetWindowTextA';
function GetWindowTextLength(hWnd:HWND):longint; external 'user32' name 'GetWindowTextLengthA';
function MessageBox(hWnd:HWND; lpText:LPCSTR; lpCaption:LPCSTR; uType:UINT):longint; external 'user32' name 'MessageBoxA';
function MessageBoxEx(hWnd:HWND; lpText:LPCSTR; lpCaption:LPCSTR; uType:UINT; wLanguageId:WORD):longint; external 'user32' name 'MessageBoxExA';
function MessageBoxIndirect(_para1:LPMSGBOXPARAMS):longint; external 'user32' name 'MessageBoxIndirectA';
function GetWindowLong(hWnd:HWND; nIndex:longint):LONG; external 'user32' name 'GetWindowLongA';
function SetWindowLong(hWnd:HWND; nIndex:longint; dwNewLong:LONG):LONG; external 'user32' name 'SetWindowLongA';
function GetClassLong(hWnd:HWND; nIndex:longint):DWORD; external 'user32' name 'GetClassLongA';
function SetClassLong(hWnd:HWND; nIndex:longint; dwNewLong:LONG):DWORD; external 'user32' name 'SetClassLongA';
{$ifdef cpu64}
function GetWindowLongPtr(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetWindowLongPtrA';
function SetWindowLongPtr(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetWindowLongPtrA';
function GetClassLongPtr(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetClassLongPtrA';
function SetClassLongPtr(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetClassLongPtrA';
{$else}
function GetWindowLongPtr(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetWindowLongA';
function SetWindowLongPtr(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetWindowLongA';
function GetClassLongPtr(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetClassLongA';
function SetClassLongPtr(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetClassLongA';
{$endif}
function FindWindow(lpClassName:LPCSTR; lpWindowName:LPCSTR):HWND; external 'user32' name 'FindWindowA';
function FindWindowEx(_para1:HWND; _para2:HWND; _para3:LPCSTR; _para4:LPCSTR):HWND; external 'user32' name 'FindWindowExA';
function GetClassName(hWnd:HWND; lpClassName:LPSTR; nMaxCount:longint):longint; external 'user32' name 'GetClassNameA';
function SetWindowsHookEx(idHook:longint; lpfn:HOOKPROC; hmod:HINST; dwThreadId:DWORD):HHOOK; external 'user32' name 'SetWindowsHookExA';
function LoadBitmap(hInstance:HINST; lpBitmapName:LPCSTR):HBITMAP; external 'user32' name 'LoadBitmapA';
function LoadCursor(hInstance:HINST; lpCursorName:LPCSTR):HCURSOR; external 'user32' name 'LoadCursorA';
function LoadCursorFromFile(lpFileName:LPCSTR):HCURSOR; external 'user32' name 'LoadCursorFromFileA';
function LoadIcon(hInstance:HINST; lpIconName:LPCSTR):HICON; external 'user32' name 'LoadIconA';
function LoadImage(_para1:HINST; _para2:LPCSTR; _para3:UINT; _para4:longint; _para5:longint;_para6:UINT):HANDLE; external 'user32' name 'LoadImageA';
function LoadString(hInstance:HINST; uID:UINT; lpBuffer:LPSTR; nBufferMax:longint):longint; external 'user32' name 'LoadStringA';
function IsDialogMessage(hDlg:HWND; lpMsg:LPMSG):WINBOOL; external 'user32' name 'IsDialogMessageA';
function DlgDirList(hDlg:HWND; lpPathSpec:LPSTR; nIDListBox:longint; nIDStaticPath:longint; uFileType:UINT):longint; external 'user32' name 'DlgDirListA';
function DlgDirSelectEx(hDlg:HWND; lpString:LPSTR; nCount:longint; nIDListBox:longint):WINBOOL; external 'user32' name 'DlgDirSelectExA';
function DlgDirListComboBox(hDlg:HWND; lpPathSpec:LPSTR; nIDComboBox:longint; nIDStaticPath:longint; uFiletype:UINT):longint; external 'user32' name 'DlgDirListComboBoxA';
function DlgDirSelectComboBoxEx(hDlg:HWND; lpString:LPSTR; nCount:longint; nIDComboBox:longint):WINBOOL; external 'user32' name 'DlgDirSelectComboBoxExA';
function DefFrameProc(hWnd:HWND; hWndMDIClient:HWND; uMsg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefFrameProcA';
function DefMDIChildProc(hWnd:HWND; uMsg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefMDIChildProcA';
function CreateMDIWindow(lpClassName:LPSTR; lpWindowName:LPSTR; dwStyle:DWORD; X:longint; Y:longint;nWidth:longint; nHeight:longint; hWndParent:HWND; hInstance:HINST; lParam:LPARAM):HWND; external 'user32' name 'CreateMDIWindowA';
function WinHelp(hWndMain:HWND; lpszHelp:LPCSTR; uCommand:UINT; dwData:DWORD):WINBOOL; external 'user32' name 'WinHelpA';
function ChangeDisplaySettings(lpDevMode:LPDEVMODE; dwFlags:DWORD):LONG; external 'user32' name 'ChangeDisplaySettingsA';
function EnumDisplaySettings(lpszDeviceName:LPCSTR; iModeNum:DWORD; lpDevMode:LPDEVMODE):WINBOOL; external 'user32' name 'EnumDisplaySettingsA';
function SystemParametersInfo(uiAction:UINT; uiParam:UINT; pvParam:PVOID; fWinIni:UINT):WINBOOL; external 'user32' name 'SystemParametersInfoA';
function AddFontResource(_para1:LPCSTR):longint; external 'gdi32' name 'AddFontResourceA';
function CopyMetaFile(_para1:HMETAFILE; _para2:LPCSTR):HMETAFILE; external 'gdi32' name 'CopyMetaFileA';
function CreateFont(_para1:longint; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:DWORD; _para7:DWORD; _para8:DWORD; _para9:DWORD; _para10:DWORD;_para11:DWORD; _para12:DWORD; _para13:DWORD; _para14:LPCSTR):HFONT;
  external 'gdi32' name 'CreateFontA';
function CreateFontIndirect(_para1:LPLOGFONT):HFONT; external 'gdi32' name 'CreateFontIndirectA';
function CreateFontIndirect(var _para1:LOGFONT):HFONT; external 'gdi32' name 'CreateFontIndirectA';
function CreateIC(_para1:LPCSTR; _para2:LPCSTR; _para3:LPCSTR; _para4:LPDEVMODE):HDC; external 'gdi32' name 'CreateICA';
function CreateMetaFile(_para1:LPCSTR):HDC; external 'gdi32' name 'CreateMetaFileA';
function CreateScalableFontResource(_para1:DWORD; _para2:LPCSTR; _para3:LPCSTR; _para4:LPCSTR):WINBOOL; external 'gdi32' name 'CreateScalableFontResourceA';
function EnumFontFamiliesEx(_para1:HDC; _para2:LPLOGFONT; _para3:FONTENUMEXPROC; _para4:LPARAM; _para5:DWORD):longint; external 'gdi32' name 'EnumFontFamiliesExA';
function EnumFontFamilies(_para1:HDC; _para2:LPCSTR; _para3:FONTENUMPROC; _para4:LPARAM):longint; external 'gdi32' name 'EnumFontFamiliesA';
function EnumFonts(_para1:HDC; _para2:LPCSTR; _para3:ENUMFONTSPROC; _para4:LPARAM):longint; external 'gdi32' name 'EnumFontsA';
function EnumFonts(_para1:HDC; _para2:LPCSTR; _para3:ENUMFONTSPROC; _para4:pointer):longint; external 'gdi32' name 'EnumFontsA';
function GetCharWidth(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPINT):WINBOOL; external 'gdi32' name 'GetCharWidthA';
function GetCharWidth32(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPINT):WINBOOL; external 'gdi32' name 'GetCharWidth32A';
function GetCharWidthFloat(_para1:HDC; _para2:UINT; _para3:UINT; _para4:PSingle):WINBOOL; external 'gdi32' name 'GetCharWidthFloatA';
function GetCharABCWidths(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPABC):WINBOOL; external 'gdi32' name 'GetCharABCWidthsA';
function GetCharABCWidthsFloat(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPABCFLOAT):WINBOOL; external 'gdi32' name 'GetCharABCWidthsFloatA';
function GetGlyphOutline(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPGLYPHMETRICS; _para5:DWORD;_para6:LPVOID; _para7:PMAT2):DWORD; external 'gdi32' name 'GetGlyphOutlineA';
function GetMetaFile(_para1:LPCSTR):HMETAFILE; external 'gdi32' name 'GetMetaFileA';
function GetOutlineTextMetrics(_para1:HDC; _para2:UINT; _para3:LPOUTLINETEXTMETRIC):UINT; external 'gdi32' name 'GetOutlineTextMetricsA';
function GetTextExtentPoint(_para1:HDC; _para2:LPCSTR; _para3:longint; _para4:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentPointA';
function GetTextExtentPoint32(_para1:HDC; _para2:LPCSTR; _para3:longint; _para4:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentPoint32A';
function GetTextExtentExPoint(_para1:HDC; _para2:LPCSTR; _para3:longint; _para4:longint; _para5:LPINT;_para6:LPINT; _para7:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentExPointA';
function GetCharacterPlacement(_para1:HDC; _para2:LPCSTR; _para3:longint; _para4:longint; _para5:LPGCP_RESULTS;_para6:DWORD):DWORD; external 'gdi32' name 'GetCharacterPlacementA';
function ResetDC(_para1:HDC; _para2:LPDEVMODE):HDC; external 'gdi32' name 'ResetDCA';
function RemoveFontResource(_para1:LPCSTR):WINBOOL; external 'gdi32' name 'RemoveFontResourceA';
function CopyEnhMetaFile(_para1:HENHMETAFILE; _para2:LPCSTR):HENHMETAFILE; external 'gdi32' name 'CopyEnhMetaFileA';
function CreateEnhMetaFile(_para1:HDC; _para2:LPCSTR; _para3:LPRECT; _para4:LPCSTR):HDC; external 'gdi32' name 'CreateEnhMetaFileA';
function GetEnhMetaFile(_para1:LPCSTR):HENHMETAFILE; external 'gdi32' name 'GetEnhMetaFileA';
function GetEnhMetaFileDescription(_para1:HENHMETAFILE; _para2:UINT; _para3:LPSTR):UINT; external 'gdi32' name 'GetEnhMetaFileDescriptionA';
function GetTextMetrics(_para1:HDC; _para2:LPTEXTMETRIC):WINBOOL; external 'gdi32' name 'GetTextMetricsA';
function StartDoc(_para1:HDC; _para2:PDOCINFO):longint; external 'gdi32' name 'StartDocA';
function GetObject(_para1:HGDIOBJ; _para2:longint; _para3:LPVOID):longint; external 'gdi32' name 'GetObjectA';
function TextOut(_para1:HDC; _para2:longint; _para3:longint; _para4:LPCSTR; _para5:longint):WINBOOL; external 'gdi32' name 'TextOutA';
function ExtTextOut(_para1:HDC; _para2:longint; _para3:longint; _para4:UINT; _para5:LPRECT;_para6:LPCSTR; _para7:UINT; _para8:LPINT):WINBOOL; external 'gdi32' name 'ExtTextOutA';
function PolyTextOut(_para1:HDC; _para2:PPOLYTEXT; _para3:longint):WINBOOL; external 'gdi32' name 'PolyTextOutA';
function GetTextFace(_para1:HDC; _para2:longint; _para3:LPSTR):longint; external 'gdi32' name 'GetTextFaceA';
function GetKerningPairs(_para1:HDC; _para2:DWORD; _para3:LPKERNINGPAIR):DWORD; external 'gdi32' name 'GetKerningPairsA';
function CreateColorSpace(_para1:LPLOGCOLORSPACE):HCOLORSPACE; external 'gdi32' name 'CreateColorSpaceA';
function GetLogColorSpace(_para1:HCOLORSPACE; _para2:LPLOGCOLORSPACE; _para3:DWORD):WINBOOL; external 'gdi32' name 'GetLogColorSpaceA';
function GetICMProfile(_para1:HDC; _para2:DWORD; _para3:LPSTR):WINBOOL; external 'gdi32' name 'GetICMProfileA';
function SetICMProfile(_para1:HDC; _para2:LPSTR):WINBOOL; external 'gdi32' name 'SetICMProfileA';
function UpdateICMRegKey(_para1:DWORD; _para2:DWORD; _para3:LPSTR; _para4:UINT):WINBOOL; external 'gdi32' name 'UpdateICMRegKeyA';
function EnumICMProfiles(_para1:HDC; _para2:ICMENUMPROC; _para3:LPARAM):longint; external 'gdi32' name 'EnumICMProfilesA';
{
function PropertySheet(lppsph:LPCPROPSHEETHEADER):longint; external 'comctl32' name 'PropertySheetA';
function ImageList_LoadImage(hi:HINST; lpbmp:LPCSTR; cx:longint; cGrow:longint; crMask:COLORREF;uType:UINT; uFlags:UINT):HIMAGELIST; external 'comctl32' name 'ImageList_LoadImageA';
function CreateStatusWindow(style:LONG; lpszText:LPCSTR; hwndParent:HWND; wID:UINT):HWND; external 'comctl32' name 'CreateStatusWindowA';
procedure DrawStatusText(hDC:HDC; lprc:LPRECT; pszText:LPCSTR; uFlags:UINT); external 'comctl32' name 'DrawStatusTextA';

function GetOpenFileName(_para1:LPOPENFILENAME):WINBOOL; external 'comdlg32' name 'GetOpenFileNameA';
function GetSaveFileName(_para1:LPOPENFILENAME):WINBOOL; external 'comdlg32' name 'GetSaveFileNameA';
function GetFileTitle(_para1:LPCSTR; _para2:LPSTR; _para3:WORD):integer; external 'comdlg32' name 'GetFileTitleA';
function ChooseColor(_para1:LPCHOOSECOLOR):WINBOOL; external 'comdlg32' name 'ChooseColorA';
function FindText(_para1:LPFINDREPLACE):HWND; external 'comdlg32' name 'FindTextA';
function ReplaceText(_para1:LPFINDREPLACE):HWND; external 'comdlg32' name 'ReplaceTextA';
function ChooseFont(_para1:LPCHOOSEFONT):WINBOOL; external 'comdlg32' name 'ChooseFontA';
function PrintDlg(_para1:LPPRINTDLG):WINBOOL; external 'comdlg32' name 'PrintDlgA';
function PageSetupDlg(_para1:LPPAGESETUPDLG):WINBOOL; external 'comdlg32' name 'PageSetupDlgA';
}

function CreateProcess(lpApplicationName:LPCSTR; lpCommandLine:LPSTR; lpProcessAttributes:LPSECURITY_ATTRIBUTES; lpThreadAttributes:LPSECURITY_ATTRIBUTES; bInheritHandles:WINBOOL;dwCreationFlags:DWORD; lpEnvironment:LPVOID;
  lpCurrentDirectory:LPCSTR; lpStartupInfo:LPSTARTUPINFO; lpProcessInformation:LPPROCESS_INFORMATION):WINBOOL; external 'kernel32' name 'CreateProcessA';
procedure GetStartupInfo(lpStartupInfo:LPSTARTUPINFO); external 'kernel32' name 'GetStartupInfoA';
function FindFirstFile(lpFileName:LPCSTR; lpFindFileData:LPWIN32_FIND_DATA):HANDLE; external 'kernel32' name 'FindFirstFileA';
function FindNextFile(hFindFile:HANDLE; lpFindFileData:LPWIN32_FIND_DATA):WINBOOL; external 'kernel32' name 'FindNextFileA';
function GetVersionEx(VersionInformation:LPOSVERSIONINFO):WINBOOL; external 'kernel32' name 'GetVersionExA';
function CreateWindow(lpClassName:LPCSTR; lpWindowName:LPCSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
function CreateDialog(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
function CreateDialogIndirect(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
function DialogBox(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
function DialogBoxIndirect(hInstance:HINST; hDialogTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
function CreateDC(_para1:LPCSTR; _para2:LPCSTR; _para3:LPCSTR; _para4:pDEVMODE):HDC; external 'gdi32' name 'CreateDCA';
function VerInstallFile(uFlags:DWORD; szSrcFileName:LPSTR; szDestFileName:LPSTR; szSrcDir:LPSTR; szDestDir:LPSTR;szCurDir:LPSTR; szTmpFile:LPSTR; lpuTmpFileLen:PUINT):DWORD; external 'version' name 'VerInstallFileA';
function GetFileVersionInfoSize(lptstrFilename:LPSTR; lpdwHandle:LPDWORD):DWORD; external 'version' name 'GetFileVersionInfoSizeA';
function GetFileVersionInfo(lptstrFilename:LPSTR; dwHandle:DWORD; dwLen:DWORD; lpData:LPVOID):WINBOOL; external 'version' name 'GetFileVersionInfoA';
function VerLanguageName(wLang:DWORD; szLang:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'VerLanguageNameA';
function VerQueryValue(pBlock:LPVOID; lpSubBlock:LPSTR; lplpBuffer:LPVOID; puLen:PUINT):WINBOOL; external 'version' name 'VerQueryValueA';
function VerFindFile(uFlags:DWORD; szFileName:LPSTR; szWinDir:LPSTR; szAppDir:LPSTR; szCurDir:LPSTR;lpuCurDirLen:PUINT; szDestDir:LPSTR; lpuDestDirLen:PUINT):DWORD; external 'version' name 'VerFindFileA';
function RegConnectRegistry(lpMachineName:LPSTR; hKey:HKEY; phkResult:PHKEY):LONG; external 'advapi32' name 'RegConnectRegistryA';
function RegCreateKey(hKey:HKEY; lpSubKey:LPCSTR; phkResult:PHKEY):LONG; external 'advapi32' name 'RegCreateKeyA';
function RegCreateKeyEx(hKey:HKEY; lpSubKey:LPCSTR; Reserved:DWORD; lpClass:LPSTR; dwOptions:DWORD;samDesired:REGSAM; lpSecurityAttributes:LPSECURITY_ATTRIBUTES; phkResult:PHKEY; lpdwDisposition:LPDWORD):LONG;
  external 'advapi32' name 'RegCreateKeyExA';
function RegDeleteKey(hKey:HKEY; lpSubKey:LPCSTR):LONG; external 'advapi32' name 'RegDeleteKeyA';
function RegDeleteValue(hKey:HKEY; lpValueName:LPCSTR):LONG; external 'advapi32' name 'RegDeleteValueA';
function RegEnumKey(hKey:HKEY; dwIndex:DWORD; lpName:LPSTR; cbName:DWORD):LONG; external 'advapi32' name 'RegEnumKeyA';
function RegEnumKeyEx(hKey:HKEY; dwIndex:DWORD; lpName:LPSTR; lpcbName:LPDWORD; lpReserved:LPDWORD;lpClass:LPSTR; lpcbClass:LPDWORD; lpftLastWriteTime:PFILETIME):LONG; external 'advapi32' name 'RegEnumKeyExA';
function RegEnumValue(hKey:HKEY; dwIndex:DWORD; lpValueName:LPSTR; lpcbValueName:LPDWORD; lpReserved:LPDWORD;lpType:LPDWORD; lpData:LPBYTE; lpcbData:LPDWORD):LONG; external 'advapi32' name 'RegEnumValueA';
function RegLoadKey(hKey:HKEY; lpSubKey:LPCSTR; lpFile:LPCSTR):LONG; external 'advapi32' name 'RegLoadKeyA';
function RegOpenKey(hKey:HKEY; lpSubKey:LPCSTR; phkResult:PHKEY):LONG; external 'advapi32' name 'RegOpenKeyA';
function RegOpenKeyEx(hKey:HKEY; lpSubKey:LPCSTR; ulOptions:DWORD; samDesired:REGSAM; phkResult:PHKEY):LONG; external 'advapi32' name 'RegOpenKeyExA';
function RegQueryInfoKey(hKey:HKEY; lpClass:LPSTR; lpcbClass:LPDWORD; lpReserved:LPDWORD; lpcSubKeys:LPDWORD;lpcbMaxSubKeyLen:LPDWORD; lpcbMaxClassLen:LPDWORD; lpcValues:LPDWORD; lpcbMaxValueNameLen:LPDWORD;
  lpcbMaxValueLen:LPDWORD;lpcbSecurityDescriptor:LPDWORD; lpftLastWriteTime:PFILETIME):LONG; external 'advapi32' name 'RegQueryInfoKeyA';
function RegQueryValue(hKey:HKEY; lpSubKey:LPCSTR; lpValue:LPSTR; lpcbValue:PLONG):LONG; external 'advapi32' name 'RegQueryValueA';
function RegQueryMultipleValues(hKey:HKEY; val_list:PVALENT; num_vals:DWORD; lpValueBuf:LPSTR; ldwTotsize:LPDWORD):LONG; external 'advapi32' name 'RegQueryMultipleValuesA';
function RegQueryValueEx(hKey:HKEY; lpValueName:LPCSTR; lpReserved:LPDWORD; lpType:LPDWORD; lpData:LPBYTE;lpcbData:LPDWORD):LONG; external 'advapi32' name 'RegQueryValueExA';
function RegReplaceKey(hKey:HKEY; lpSubKey:LPCSTR; lpNewFile:LPCSTR; lpOldFile:LPCSTR):LONG; external 'advapi32' name 'RegReplaceKeyA';
function RegRestoreKey(hKey:HKEY; lpFile:LPCSTR; dwFlags:DWORD):LONG; external 'advapi32' name 'RegRestoreKeyA';
function RegSaveKey(hKey:HKEY; lpFile:LPCSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):LONG; external 'advapi32' name 'RegSaveKeyA';
function RegSetValue(hKey:HKEY; lpSubKey:LPCSTR; dwType:DWORD; lpData:LPCSTR; cbData:DWORD):LONG; external 'advapi32' name 'RegSetValueA';
function RegSetValueEx(hKey:HKEY; lpValueName:LPCSTR; Reserved:DWORD; dwType:DWORD; lpData:LPBYTE;cbData:DWORD):LONG; external 'advapi32' name 'RegSetValueExA';
function RegUnLoadKey(hKey:HKEY; lpSubKey:LPCSTR):LONG; external 'advapi32' name 'RegUnLoadKeyA';
function InitiateSystemShutdown(lpMachineName:LPSTR; lpMessage:LPSTR; dwTimeout:DWORD; bForceAppsClosed:WINBOOL; bRebootAfterShutdown:WINBOOL):WINBOOL; external 'advapi32' name 'InitiateSystemShutdownA';
function AbortSystemShutdown(lpMachineName:LPSTR):WINBOOL; external 'advapi32' name 'AbortSystemShutdownA';
function CompareString(Locale:LCID; dwCmpFlags:DWORD; lpString1:LPCSTR; cchCount1:longint; lpString2:LPCSTR;cchCount2:longint):longint; external 'kernel32' name 'CompareStringA';
function LCMapString(Locale:LCID; dwMapFlags:DWORD; lpSrcStr:LPCSTR; cchSrc:longint; lpDestStr:LPSTR;cchDest:longint):longint; external 'kernel32' name 'LCMapStringA';
function GetLocaleInfo(Locale:LCID; LCType:LCTYPE; lpLCData:LPSTR; cchData:longint):longint; external 'kernel32' name 'GetLocaleInfoA';
function SetLocaleInfo(Locale:LCID; LCType:LCTYPE; lpLCData:LPCSTR):WINBOOL; external 'kernel32' name 'SetLocaleInfoA';
function GetTimeFormat(Locale:LCID; dwFlags:DWORD; lpTime:LPSYSTEMTIME; lpFormat:LPCSTR; lpTimeStr:LPSTR;cchTime:longint):longint; external 'kernel32' name 'GetTimeFormatA';
function GetDateFormat(Locale:LCID; dwFlags:DWORD; lpDate:LPSYSTEMTIME; lpFormat:LPCSTR; lpDateStr:LPSTR;cchDate:longint):longint; external 'kernel32' name 'GetDateFormatA';
function GetNumberFormat(Locale:LCID; dwFlags:DWORD; lpValue:LPCSTR; lpFormat:PNUMBERFMT; lpNumberStr:LPSTR;cchNumber:longint):longint; external 'kernel32' name 'GetNumberFormatA';
function GetCurrencyFormat(Locale:LCID; dwFlags:DWORD; lpValue:LPCSTR; lpFormat:PCURRENCYFMT; lpCurrencyStr:LPSTR;cchCurrency:longint):longint; external 'kernel32' name 'GetCurrencyFormatA';
function EnumCalendarInfo(lpCalInfoEnumProc:CALINFO_ENUMPROC; Locale:LCID; Calendar:CALID; CalType:CALTYPE):WINBOOL; external 'kernel32' name 'EnumCalendarInfoA';
function EnumTimeFormats(lpTimeFmtEnumProc:TIMEFMT_ENUMPROC; Locale:LCID; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumTimeFormatsA';
function EnumDateFormats(lpDateFmtEnumProc:DATEFMT_ENUMPROC; Locale:LCID; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumDateFormatsA';
function GetStringTypeEx(Locale:LCID; dwInfoType:DWORD; lpSrcStr:LPCSTR; cchSrc:longint; lpCharType:LPWORD):WINBOOL; external 'kernel32' name 'GetStringTypeExA';
function GetStringType(Locale:LCID; dwInfoType:DWORD; lpSrcStr:LPCSTR; cchSrc:longint; lpCharType:LPWORD):WINBOOL; external 'kernel32' name 'GetStringTypeA';
function FoldString(dwMapFlags:DWORD; lpSrcStr:LPCSTR; cchSrc:longint; lpDestStr:LPSTR; cchDest:longint):longint; external 'kernel32' name 'FoldStringA';
function EnumSystemLocales(lpLocaleEnumProc:LOCALE_ENUMPROC; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumSystemLocalesA';
function EnumSystemCodePages(lpCodePageEnumProc:CODEPAGE_ENUMPROC; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumSystemCodePagesA';
function PeekConsoleInput(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsRead:LPDWORD):WINBOOL; external 'kernel32' name 'PeekConsoleInputA';
function ReadConsoleInput(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsRead:LPDWORD):WINBOOL; external 'kernel32' name 'ReadConsoleInputA';
function WriteConsoleInput(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'WriteConsoleInputA';
function ReadConsoleOutput(hConsoleOutput:HANDLE; lpBuffer:PCHAR_INFO; dwBufferSize:COORD; dwBufferCoord:COORD; lpReadRegion:PSMALL_RECT):WINBOOL; external 'kernel32' name 'ReadConsoleOutputA';
function WriteConsoleOutput(hConsoleOutput:HANDLE; lpBuffer:PCHAR_INFO; dwBufferSize:COORD; dwBufferCoord:COORD; lpWriteRegion:PSMALL_RECT):WINBOOL; external 'kernel32' name 'WriteConsoleOutputA';
function ReadConsoleOutputCharacter(hConsoleOutput:HANDLE; lpCharacter:LPSTR; nLength:DWORD; dwReadCoord:COORD; lpNumberOfCharsRead:LPDWORD):WINBOOL; external 'kernel32' name 'ReadConsoleOutputCharacterA';
function WriteConsoleOutputCharacter(hConsoleOutput:HANDLE; lpCharacter:LPCSTR; nLength:DWORD; dwWriteCoord:COORD; lpNumberOfCharsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'WriteConsoleOutputCharacterA';
function FillConsoleOutputCharacter(hConsoleOutput:HANDLE; cCharacter:CHAR; nLength:DWORD; dwWriteCoord:COORD; lpNumberOfCharsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'FillConsoleOutputCharacterA';
function ScrollConsoleScreenBuffer(hConsoleOutput:HANDLE; lpScrollRectangle:PSMALL_RECT; lpClipRectangle:PSMALL_RECT; dwDestinationOrigin:COORD; lpFill:PCHAR_INFO):WINBOOL; external 'kernel32' name 'ScrollConsoleScreenBufferA';
function GetConsoleTitle(lpConsoleTitle:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetConsoleTitleA';
function SetConsoleTitle(lpConsoleTitle:LPCSTR):WINBOOL; external 'kernel32' name 'SetConsoleTitleA';
function ReadConsole(hConsoleInput:HANDLE; lpBuffer:LPVOID; nNumberOfCharsToRead:DWORD; lpNumberOfCharsRead:LPDWORD; lpReserved:LPVOID):WINBOOL; external 'kernel32' name 'ReadConsoleA';
function WriteConsole(hConsoleOutput:HANDLE;lpBuffer:pointer; nNumberOfCharsToWrite:DWORD; lpNumberOfCharsWritten:LPDWORD; lpReserved:LPVOID):WINBOOL; external 'kernel32' name 'WriteConsoleA';
function WNetAddConnection(lpRemoteName:LPCSTR; lpPassword:LPCSTR; lpLocalName:LPCSTR):DWORD; external 'mpr' name 'WNetAddConnectionA';
function WNetAddConnection2(lpNetResource:LPNETRESOURCE; lpPassword:LPCSTR; lpUserName:LPCSTR; dwFlags:DWORD):DWORD; external 'mpr' name 'WNetAddConnection2A';
function WNetAddConnection3(hwndOwner:HWND; lpNetResource:LPNETRESOURCE; lpPassword:LPCSTR; lpUserName:LPCSTR; dwFlags:DWORD):DWORD; external 'mpr' name 'WNetAddConnection3A';
function WNetCancelConnection(lpName:LPCSTR; fForce:WINBOOL):DWORD; external 'mpr' name 'WNetCancelConnectionA';
function WNetCancelConnection2(lpName:LPCSTR; dwFlags:DWORD; fForce:WINBOOL):DWORD; external 'mpr' name 'WNetCancelConnection2A';
function WNetGetConnection(lpLocalName:LPCSTR; lpRemoteName:LPSTR; lpnLength:LPDWORD):DWORD; external 'mpr' name 'WNetGetConnectionA';
function WNetUseConnection(hwndOwner:HWND; lpNetResource:LPNETRESOURCE; lpUserID:LPCSTR; lpPassword:LPCSTR; dwFlags:DWORD;lpAccessName:LPSTR; lpBufferSize:LPDWORD; lpResult:LPDWORD):DWORD; external 'mpr' name 'WNetUseConnectionA';
function WNetSetConnection(lpName:LPCSTR; dwProperties:DWORD; pvValues:LPVOID):DWORD; external 'mpr' name 'WNetSetConnectionA';
function WNetConnectionDialog1(lpConnDlgStruct:LPCONNECTDLGSTRUCT):DWORD; external 'mpr' name 'WNetConnectionDialog1A';
function WNetDisconnectDialog1(lpConnDlgStruct:LPDISCDLGSTRUCT):DWORD; external 'mpr' name 'WNetDisconnectDialog1A';
function WNetOpenEnum(dwScope:DWORD; dwType:DWORD; dwUsage:DWORD; lpNetResource:LPNETRESOURCE; lphEnum:LPHANDLE):DWORD; external 'mpr' name 'WNetOpenEnumA';
function WNetEnumResource(hEnum:HANDLE; lpcCount:LPDWORD; lpBuffer:LPVOID; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetEnumResourceA';
function WNetGetUniversalName(lpLocalPath:LPCSTR; dwInfoLevel:DWORD; lpBuffer:LPVOID; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetGetUniversalNameA';
function WNetGetUser(lpName:LPCSTR; lpUserName:LPSTR; lpnLength:LPDWORD):DWORD; external 'mpr' name 'WNetGetUserA';
function WNetGetProviderName(dwNetType:DWORD; lpProviderName:LPSTR; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetGetProviderNameA';
function WNetGetNetworkInformation(lpProvider:LPCSTR; lpNetInfoStruct:LPNETINFOSTRUCT):DWORD; external 'mpr' name 'WNetGetNetworkInformationA';
function WNetGetLastError(lpError:LPDWORD; lpErrorBuf:LPSTR; nErrorBufSize:DWORD; lpNameBuf:LPSTR; nNameBufSize:DWORD):DWORD; external 'mpr' name 'WNetGetLastErrorA';
function MultinetGetConnectionPerformance(lpNetResource:LPNETRESOURCE; lpNetConnectInfoStruct:LPNETCONNECTINFOSTRUCT):DWORD; external 'mpr' name 'MultinetGetConnectionPerformanceA';
function ChangeServiceConfig(hService:SC_HANDLE; dwServiceType:DWORD; dwStartType:DWORD; dwErrorControl:DWORD; lpBinaryPathName:LPCSTR;lpLoadOrderGroup:LPCSTR; lpdwTagId:LPDWORD; lpDependencies:LPCSTR; lpServiceStartName:LPCSTR;
  lpPassword:LPCSTR;lpDisplayName:LPCSTR):WINBOOL; external 'advapi32' name 'ChangeServiceConfigA';
function CreateService(hSCManager:SC_HANDLE; lpServiceName:LPCSTR; lpDisplayName:LPCSTR; dwDesiredAccess:DWORD; dwServiceType:DWORD;dwStartType:DWORD; dwErrorControl:DWORD; lpBinaryPathName:LPCSTR; lpLoadOrderGroup:LPCSTR;
  lpdwTagId:LPDWORD;lpDependencies:LPCSTR; lpServiceStartName:LPCSTR; lpPassword:LPCSTR):SC_HANDLE; external 'advapi32' name 'CreateServiceA';
function EnumDependentServices(hService:SC_HANDLE; dwServiceState:DWORD; lpServices:LPENUM_SERVICE_STATUS; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD;lpServicesReturned:LPDWORD):WINBOOL; external 'advapi32' name 'EnumDependentServicesA';
function EnumServicesStatus(hSCManager:SC_HANDLE; dwServiceType:DWORD; dwServiceState:DWORD; lpServices:LPENUM_SERVICE_STATUS; cbBufSize:DWORD;pcbBytesNeeded:LPDWORD; lpServicesReturned:LPDWORD; lpResumeHandle:LPDWORD):WINBOOL;
  external 'advapi32' name 'EnumServicesStatusA';
function GetServiceKeyName(hSCManager:SC_HANDLE; lpDisplayName:LPCSTR; lpServiceName:LPSTR; lpcchBuffer:LPDWORD):WINBOOL; external 'advapi32' name 'GetServiceKeyNameA';
function GetServiceDisplayName(hSCManager:SC_HANDLE; lpServiceName:LPCSTR; lpDisplayName:LPSTR; lpcchBuffer:LPDWORD):WINBOOL; external 'advapi32' name 'GetServiceDisplayNameA';
function OpenSCManager(lpMachineName:LPCSTR; lpDatabaseName:LPCSTR; dwDesiredAccess:DWORD):SC_HANDLE; external 'advapi32' name 'OpenSCManagerA';
function OpenService(hSCManager:SC_HANDLE; lpServiceName:LPCSTR; dwDesiredAccess:DWORD):SC_HANDLE; external 'advapi32' name 'OpenServiceA';
function QueryServiceConfig(hService:SC_HANDLE; lpServiceConfig:LPQUERY_SERVICE_CONFIG; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'QueryServiceConfigA';
function QueryServiceLockStatus(hSCManager:SC_HANDLE; lpLockStatus:LPQUERY_SERVICE_LOCK_STATUS; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'QueryServiceLockStatusA';
function RegisterServiceCtrlHandler(lpServiceName:LPCSTR; lpHandlerProc:LPHANDLER_FUNCTION):SERVICE_STATUS_HANDLE; external 'advapi32' name 'RegisterServiceCtrlHandlerA';
function StartServiceCtrlDispatcher(lpServiceStartTable:LPSERVICE_TABLE_ENTRY):WINBOOL; external 'advapi32' name 'StartServiceCtrlDispatcherA';
function StartService(hService:SC_HANDLE; dwNumServiceArgs:DWORD; lpServiceArgVectors:LPPCSTR):WINBOOL; external 'advapi32' name 'StartServiceA';
function DragQueryFile(_para1:HDROP; _para2:cardinal; _para3:Pchar;_para4:cardinal):cardinal; external 'shell32' name 'DragQueryFileA';
function ExtractAssociatedIcon(_para1:HINST; _para2:Pchar; _para3:LPWORD):HICON; external 'shell32' name 'ExtractAssociatedIconA';
function ExtractIcon(_para1:HINST; _para2:Pchar; _para3:cardinal):HICON;external 'shell32' name 'ExtractIconA';
function FindExecutable(_para1:pchar; _para2:pchar;_para3:pchar):HINST; external 'shell32' name 'FindExecutableA';
function ShellAbout(_para1:HWND; _para2:pchar; _para3:pchar;_para4:HICON):longint; external 'shell32' name 'ShellAboutA';
function ShellExecute(_para1:HWND; _para2:pchar; _para3:pchar;_para4:pchar; _para5:pchar;_para6:longint):HINST; external 'shell32' name 'ShellExecuteA';
function Shell_NotifyIcon(dwMessage: DWORD; lpData: PNotifyIconDataA): BOOL; external 'shell32' name 'Shell_NotifyIconA';
function DdeCreateStringHandle(_para1:DWORD; _para2:pchar;_para3:longint):HSZ; external 'user32' name 'DdeCreateStringHandleA';
function DdeInitialize(_para1:LPDWORD; _para2:PFNCALLBACK; _para3:DWORD; _para4:DWORD):UINT;external 'user32' name 'DdeInitializeA';
function DdeQueryString(_para1:DWORD; _para2:HSZ; _para3:pchar;_para4:DWORD; _para5:longint):DWORD; external 'user32' name 'DdeQueryStringA';
function LogonUser(_para1:LPSTR; _para2:LPSTR; _para3:LPSTR; _para4:DWORD; _para5:DWORD;_para6:PHANDLE):WINBOOL; external 'advapi32' name 'LogonUserA';
function CreateProcessAsUser(_para1:HANDLE; _para2:LPCTSTR; _para3:LPTSTR; _para4:LPSECURITY_ATTRIBUTES; _para5:LPSECURITY_ATTRIBUTES;_para6:WINBOOL; _para7:DWORD; _para8:LPVOID; _para9:LPCTSTR;
  _para10:LPSTARTUPINFO; _para11:LPPROCESS_INFORMATION):WINBOOL; external 'advapi32' name 'CreateProcessAsUserA';
function FindFirstFileEx(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external name 'FindFirstFileExA';
// winver>$0600
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function FindFirstFileTransacted(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall;
    external name 'FindFirstFileTransactedA';
=======
function FindFirstFileTransacted(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedA';
>>>>>>> graemeg/cpstrnew
=======
function FindFirstFileTransacted(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedA';
>>>>>>> graemeg/cpstrnew
=======
function FindFirstFileTransacted(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedA';
>>>>>>> graemeg/cpstrnew
=======
function FindFirstFileTransacted(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedA';
>>>>>>> origin/cpstrnew
{$endif read_interface}


{$ifdef read_implementation}

function CreateWindow(lpClassName:LPCSTR; lpWindowName:LPCSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
begin
  CreateWindow:=CreateWindowEx(0,lpClassName,lpWindowName,dwStyle,x,y,nWidth,nHeight,hWndParent,hMenu,hInstance,lpParam);
end;

function CreateDialog(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
begin
  CreateDialog:=CreateDialogParam(hInstance,lpTemplateName,hWndParent,lpDialogFunc,0);
end;

function CreateDialogIndirect(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
begin
  CreateDialogIndirect:=CreateDialogIndirectParam(hInstance,lpTemplate,hWndParent,lpDialogFunc,0);
end;

function DialogBox(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
begin
  DialogBox:=DialogBoxParam(hInstance,lpTemplateName,hWndParent,lpDialogFunc,0);
end;

function DialogBoxIndirect(hInstance:HINST; hDialogTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
begin
  DialogBoxIndirect:=DialogBoxIndirectParam(hInstance,hDialogTemplate,hWndParent,lpDialogFunc,0);
end;

{$endif read_implementation}

