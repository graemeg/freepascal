{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    Contains the Unicode functions for windows unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  UnicodeFunctions.h

  Declarations for all the Windows32 API Unicode Functions

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
  59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
}

{$ifdef read_interface}

function GetBinaryType(lpApplicationName:LPCWSTR; lpBinaryType:LPDWORD):WINBOOL; external 'kernel32' name 'GetBinaryTypeW';
function GetShortPathName(lpszLongPath:LPCWSTR; lpszShortPath:LPWSTR; cchBuffer:DWORD):DWORD; external 'kernel32' name 'GetShortPathNameW';
function GetEnvironmentStrings : LPWSTR; external 'kernel32' name 'GetEnvironmentStringsW';function FreeEnvironmentStrings(_para1:LPWSTR):WINBOOL; external 'kernel32' name 'FreeEnvironmentStringsW';
function FormatMessage(dwFlags:DWORD; lpSource:LPCVOID; dwMessageId:DWORD; dwLanguageId:DWORD; lpBuffer:LPWSTR;nSize:DWORD; Arguments:va_list):DWORD; external 'kernel32' name 'FormatMessageW';
function CreateMailslot(lpName:LPCWSTR; nMaxMessageSize:DWORD; lReadTimeout:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):HANDLE; external 'kernel32' name 'CreateMailslotW';
function lstrcmp(lpString1:LPCWSTR; lpString2:LPCWSTR):longint; external 'kernel32' name 'lstrcmpW';
function lstrcmpi(lpString1:LPCWSTR; lpString2:LPCWSTR):longint; external 'kernel32' name 'lstrcmpiW';
function lstrcpyn(lpString1:LPWSTR; lpString2:LPCWSTR; iMaxLength:longint):LPWSTR; external 'kernel32' name 'lstrcpynW';
function lstrcpy(lpString1:LPWSTR; lpString2:LPCWSTR):LPWSTR; external 'kernel32' name 'lstrcpyW';
function lstrcat(lpString1:LPWSTR; lpString2:LPCWSTR):LPWSTR; external 'kernel32' name 'lstrcatW';
function lstrlen(lpString:LPCWSTR):longint; external 'kernel32' name 'lstrlenW';
function CreateMutex(lpMutexAttributes:LPSECURITY_ATTRIBUTES; bInitialOwner:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'CreateMutexW';
function OpenMutex(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'OpenMutexW';
function CreateEvent(lpEventAttributes:LPSECURITY_ATTRIBUTES; bManualReset:WINBOOL; bInitialState:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'CreateEventW';
function OpenEvent(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'OpenEventW';
function CreateSemaphore(lpSemaphoreAttributes:LPSECURITY_ATTRIBUTES; lInitialCount:LONG; lMaximumCount:LONG; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'CreateSemaphoreW';
function OpenSemaphore(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'OpenSemaphoreW';
function CreateFileMapping(hFile:HANDLE; lpFileMappingAttributes:LPSECURITY_ATTRIBUTES; flProtect:DWORD; dwMaximumSizeHigh:DWORD; dwMaximumSizeLow:DWORD;lpName:LPCWSTR):HANDLE; external 'kernel32' name 'CreateFileMappingW';
function OpenFileMapping(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'OpenFileMappingW';
function GetLogicalDriveStrings(nBufferLength:DWORD; lpBuffer:LPWSTR):DWORD; external 'kernel32' name 'GetLogicalDriveStringsW';
function LoadLibrary(lpLibFileName:LPCWSTR):HINST; external 'kernel32' name 'LoadLibraryW';
function LoadLibraryEx(lpLibFileName:LPCWSTR; hFile:HANDLE; dwFlags:DWORD):HINST; external 'kernel32' name 'LoadLibraryExW';
function GetModuleFileName(hModule:HINST; lpFilename:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetModuleFileNameW';
function GetModuleHandle(lpModuleName:LPCWSTR):HMODULE; external 'kernel32' name 'GetModuleHandleW';
procedure FatalAppExit(uAction:UINT; lpMessageText:LPCWSTR); external 'kernel32' name 'FatalAppExitW';
function GetCommandLine : LPWSTR; external 'kernel32' name 'GetCommandLineW';function GetEnvironmentVariable(lpName:LPCWSTR; lpBuffer:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetEnvironmentVariableW';
function SetEnvironmentVariable(lpName:LPCWSTR; lpValue:LPCWSTR):WINBOOL; external 'kernel32' name 'SetEnvironmentVariableW';
function ExpandEnvironmentStrings(lpSrc:LPCWSTR; lpDst:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'ExpandEnvironmentStringsW';
procedure OutputDebugString(lpOutputString:LPCWSTR); external 'kernel32' name 'OutputDebugStringW';
function FindResource(hModule:HINST; lpName:LPCWSTR; lpType:LPCWSTR):HRSRC; external 'kernel32' name 'FindResourceW';
function FindResourceEx(hModule:HINST; lpType:LPCWSTR; lpName:LPCWSTR; wLanguage:WORD):HRSRC; external 'kernel32' name 'FindResourceExW';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function EnumResourceTypes(hModule:HMODULE; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesW';
function EnumResourceNames(hModule:HMODULE; lpType:LPCWSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesW';
function EnumResourceLanguages(hModule:HMODULE; lpType:LPCWSTR; lpName:LPCWSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesW';
=======
function EnumResourceTypes(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesW';
function EnumResourceNames(hModule:HINST; lpType:LPCWSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesW';
function EnumResourceLanguages(hModule:HINST; lpType:LPCWSTR; lpName:LPCWSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesW';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypes(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesW';
function EnumResourceNames(hModule:HINST; lpType:LPCWSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesW';
function EnumResourceLanguages(hModule:HINST; lpType:LPCWSTR; lpName:LPCWSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesW';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypes(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesW';
function EnumResourceNames(hModule:HINST; lpType:LPCWSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesW';
function EnumResourceLanguages(hModule:HINST; lpType:LPCWSTR; lpName:LPCWSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesW';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypes(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesW';
function EnumResourceNames(hModule:HINST; lpType:LPCWSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesW';
function EnumResourceLanguages(hModule:HINST; lpType:LPCWSTR; lpName:LPCWSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesW';
>>>>>>> origin/cpstrnew
function BeginUpdateResource(pFileName:LPCWSTR; bDeleteExistingResources:WINBOOL):HANDLE; external 'kernel32' name 'BeginUpdateResourceW';
function UpdateResource(hUpdate:HANDLE; lpType:LPCWSTR; lpName:LPCWSTR; wLanguage:WORD; lpData:LPVOID;cbData:DWORD):WINBOOL; external 'kernel32' name 'UpdateResourceW';
function EndUpdateResource(hUpdate:HANDLE; fDiscard:WINBOOL):WINBOOL; external 'kernel32' name 'EndUpdateResourceW';
function GlobalAddAtom(lpString:LPCWSTR):ATOM; external 'kernel32' name 'GlobalAddAtomW';
function GlobalFindAtom(lpString:LPCWSTR):ATOM; external 'kernel32' name 'GlobalFindAtomW';
function GlobalGetAtomName(nAtom:ATOM; lpBuffer:LPWSTR; nSize:longint):UINT; external 'kernel32' name 'GlobalGetAtomNameW';
function AddAtom(lpString:LPCWSTR):ATOM; external 'kernel32' name 'AddAtomW';
function FindAtom(lpString:LPCWSTR):ATOM; external 'kernel32' name 'FindAtomW';
function GetAtomName(nAtom:ATOM; lpBuffer:LPWSTR; nSize:longint):UINT; external 'kernel32' name 'GetAtomNameW';
function GetProfileInt(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; nDefault:WINT):UINT; external 'kernel32' name 'GetProfileIntW';
function GetProfileString(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; lpDefault:LPCWSTR; lpReturnedString:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetProfileStringW';
function WriteProfileString(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; lpString:LPCWSTR):WINBOOL; external 'kernel32' name 'WriteProfileStringW';
function GetProfileSection(lpAppName:LPCWSTR; lpReturnedString:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetProfileSectionW';
function WriteProfileSection(lpAppName:LPCWSTR; lpString:LPCWSTR):WINBOOL; external 'kernel32' name 'WriteProfileSectionW';
function GetPrivateProfileInt(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; nDefault:WINT; lpFileName:LPCWSTR):UINT; external 'kernel32' name 'GetPrivateProfileIntW';
function GetPrivateProfileString(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; lpDefault:LPCWSTR; lpReturnedString:LPWSTR; nSize:DWORD;lpFileName:LPCWSTR):DWORD; external 'kernel32' name 'GetPrivateProfileStringW';
function WritePrivateProfileString(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; lpString:LPCWSTR; lpFileName:LPCWSTR):WINBOOL; external 'kernel32' name 'WritePrivateProfileStringW';
function GetPrivateProfileSection(lpAppName:LPCWSTR; lpReturnedString:LPWSTR; nSize:DWORD; lpFileName:LPCWSTR):DWORD; external 'kernel32' name 'GetPrivateProfileSectionW';
function WritePrivateProfileSection(lpAppName:LPCWSTR; lpString:LPCWSTR; lpFileName:LPCWSTR):WINBOOL; external 'kernel32' name 'WritePrivateProfileSectionW';
function GetDriveType(lpRootPathName:LPCWSTR):UINT; external 'kernel32' name 'GetDriveTypeW';
function GetSystemDirectory(lpBuffer:LPWSTR; uSize:UINT):UINT; external 'kernel32' name 'GetSystemDirectoryW';
function GetTempPath(nBufferLength:DWORD; lpBuffer:LPWSTR):DWORD; external 'kernel32' name 'GetTempPathW';
function GetTempFileName(lpPathName:LPCWSTR; lpPrefixString:LPCWSTR; uUnique:UINT; lpTempFileName:LPWSTR):UINT; external 'kernel32' name 'GetTempFileNameW';
function GetWindowsDirectory(lpBuffer:LPWSTR; uSize:UINT):UINT; external 'kernel32' name 'GetWindowsDirectoryW';
function SetCurrentDirectory(lpPathName:LPCWSTR):WINBOOL; external 'kernel32' name 'SetCurrentDirectoryW';
function GetCurrentDirectory(nBufferLength:DWORD; lpBuffer:LPWSTR):DWORD; external 'kernel32' name 'GetCurrentDirectoryW';
function GetDiskFreeSpace(lpRootPathName:LPCWSTR; lpSectorsPerCluster:LPDWORD; lpBytesPerSector:LPDWORD; lpNumberOfFreeClusters:LPDWORD; lpTotalNumberOfClusters:LPDWORD):WINBOOL; external 'kernel32' name 'GetDiskFreeSpaceW';
function CreateDirectory(lpPathName:LPCWSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):WINBOOL; external 'kernel32' name 'CreateDirectoryW';
function CreateDirectoryEx(lpTemplateDirectory:LPCWSTR; lpNewDirectory:LPCWSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):WINBOOL; external 'kernel32' name 'CreateDirectoryExW';
function RemoveDirectory(lpPathName:LPCWSTR):WINBOOL; external 'kernel32' name 'RemoveDirectoryW';
function GetFullPathName(lpFileName:LPCWSTR; nBufferLength:DWORD; lpBuffer:LPWSTR; var lpFilePart:LPWSTR):DWORD; external 'kernel32' name 'GetFullPathNameW';
function DefineDosDevice(dwFlags:DWORD; lpDeviceName:LPCWSTR; lpTargetPath:LPCWSTR):WINBOOL; external 'kernel32' name 'DefineDosDeviceW';
function QueryDosDevice(lpDeviceName:LPCWSTR; lpTargetPath:LPWSTR; ucchMax:DWORD):DWORD; external 'kernel32' name 'QueryDosDeviceW';
function CreateFile(lpFileName:LPCWSTR; dwDesiredAccess:DWORD; dwShareMode:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES; dwCreationDisposition:DWORD;dwFlagsAndAttributes:DWORD; hTemplateFile:HANDLE):HANDLE; external 'kernel32' name 'CreateFileW';
function SetFileAttributes(lpFileName:LPCWSTR; dwFileAttributes:DWORD):WINBOOL; external 'kernel32' name 'SetFileAttributesW';
function GetFileAttributes(lpFileName:LPCWSTR):DWORD; external 'kernel32' name 'GetFileAttributesW';
<<<<<<< HEAD
<<<<<<< HEAD
function GetFileAttributesEx(lpFileName:LPCWSTR; fInfoLevelId:GET_FILEEX_INFO_LEVELS; lpFileInformation:LPVOID):BOOL; external 'kernel32' name 'GetFileAttributesExW';
=======
function GetFileAttributesExW(lpFileName:LPCWSTR; fInfoLevelId:GET_FILEEX_INFO_LEVELS; lpFileInformation:LPVOID):BOOL; external 'kernel32' name 'GetFileAttributesExW';
>>>>>>> graemeg/fixes_2_2
=======
function GetFileAttributesExW(lpFileName:LPCWSTR; fInfoLevelId:GET_FILEEX_INFO_LEVELS; lpFileInformation:LPVOID):BOOL; external 'kernel32' name 'GetFileAttributesExW';
>>>>>>> origin/fixes_2_2
function GetCompressedFileSize(lpFileName:LPCWSTR; lpFileSizeHigh:LPDWORD):DWORD; external 'kernel32' name 'GetCompressedFileSizeW';
function DeleteFile(lpFileName:LPCWSTR):WINBOOL; external 'kernel32' name 'DeleteFileW';
function SearchPath(lpPath:LPCWSTR; lpFileName:LPCWSTR; lpExtension:LPCWSTR; nBufferLength:DWORD; lpBuffer:LPWSTR;lpFilePart:LPWSTR):DWORD; external 'kernel32' name 'SearchPathW';
function CopyFile(lpExistingFileName:LPCWSTR; lpNewFileName:LPCWSTR; bFailIfExists:WINBOOL):WINBOOL; external 'kernel32' name 'CopyFileW';
function MoveFile(lpExistingFileName:LPCWSTR; lpNewFileName:LPCWSTR):WINBOOL; external 'kernel32' name 'MoveFileW';
function MoveFileEx(lpExistingFileName:LPCWSTR; lpNewFileName:LPCWSTR; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'MoveFileExW';
function CreateNamedPipe(lpName:LPCWSTR; dwOpenMode:DWORD; dwPipeMode:DWORD; nMaxInstances:DWORD; nOutBufferSize:DWORD;nInBufferSize:DWORD; nDefaultTimeOut:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):HANDLE;
  external 'kernel32' name 'CreateNamedPipeW';
function GetNamedPipeHandleState(hNamedPipe:HANDLE; lpState:LPDWORD; lpCurInstances:LPDWORD; lpMaxCollectionCount:LPDWORD; lpCollectDataTimeout:LPDWORD;lpUserName:LPWSTR; nMaxUserNameSize:DWORD):WINBOOL;
  external 'kernel32' name 'GetNamedPipeHandleStateW';
function CallNamedPipe(lpNamedPipeName:LPCWSTR; lpInBuffer:LPVOID; nInBufferSize:DWORD; lpOutBuffer:LPVOID; nOutBufferSize:DWORD;lpBytesRead:LPDWORD; nTimeOut:DWORD):WINBOOL; external 'kernel32' name 'CallNamedPipeW';
function WaitNamedPipe(lpNamedPipeName:LPCWSTR; nTimeOut:DWORD):WINBOOL; external 'kernel32' name 'WaitNamedPipeW';
function SetVolumeLabel(lpRootPathName:LPCWSTR; lpVolumeName:LPCWSTR):WINBOOL; external 'kernel32' name 'SetVolumeLabelW';
function GetVolumeInformation(lpRootPathName:LPCWSTR; lpVolumeNameBuffer:LPWSTR; nVolumeNameSize:DWORD; lpVolumeSerialNumber:LPDWORD; lpMaximumComponentLength:LPDWORD;lpFileSystemFlags:LPDWORD; lpFileSystemNameBuffer:LPWSTR;
  nFileSystemNameSize:DWORD):WINBOOL; external 'kernel32' name 'GetVolumeInformationW';
function ClearEventLog(hEventLog:HANDLE; lpBackupFileName:LPCWSTR):WINBOOL; external 'advapi32' name 'ClearEventLogW';
function BackupEventLog(hEventLog:HANDLE; lpBackupFileName:LPCWSTR):WINBOOL; external 'advapi32' name 'BackupEventLogW';
function OpenEventLog(lpUNCServerName:LPCWSTR; lpSourceName:LPCWSTR):HANDLE; external 'advapi32' name 'OpenEventLogW';
function RegisterEventSource(lpUNCServerName:LPCWSTR; lpSourceName:LPCWSTR):HANDLE; external 'advapi32' name 'RegisterEventSourceW';
function OpenBackupEventLog(lpUNCServerName:LPCWSTR; lpFileName:LPCWSTR):HANDLE; external 'advapi32' name 'OpenBackupEventLogW';
function ReadEventLog(hEventLog:HANDLE; dwReadFlags:DWORD; dwRecordOffset:DWORD; lpBuffer:LPVOID; nNumberOfBytesToRead:DWORD;pnBytesRead:LPDWORD; pnMinNumberOfBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'ReadEventLogW';
function ReportEvent(hEventLog:HANDLE; wType:WORD; wCategory:WORD; dwEventID:DWORD; lpUserSid:PSID;wNumStrings:WORD; dwDataSize:DWORD; lpStrings:LPPCWSTR; lpRawData:LPVOID):WINBOOL; external 'advapi32' name 'ReportEventW';
function AccessCheckAndAuditAlarm(SubsystemName:LPCWSTR; HandleId:LPVOID; ObjectTypeName:LPWSTR; ObjectName:LPWSTR; SecurityDescriptor:PSECURITY_DESCRIPTOR;DesiredAccess:DWORD; GenericMapping:PGENERIC_MAPPING; ObjectCreation:WINBOOL;
  GrantedAccess:LPDWORD; AccessStatus:LPBOOL;pfGenerateOnClose:LPBOOL):WINBOOL; external 'advapi32' name 'AccessCheckAndAuditAlarmW';
function ObjectOpenAuditAlarm(SubsystemName:LPCWSTR; HandleId:LPVOID; ObjectTypeName:LPWSTR; ObjectName:LPWSTR; pSecurityDescriptor:PSECURITY_DESCRIPTOR;ClientToken:HANDLE; DesiredAccess:DWORD; GrantedAccess:DWORD;
  Privileges:PPRIVILEGE_SET; ObjectCreation:WINBOOL;AccessGranted:WINBOOL; GenerateOnClose:LPBOOL):WINBOOL; external 'advapi32' name 'ObjectOpenAuditAlarmW';
function ObjectPrivilegeAuditAlarm(SubsystemName:LPCWSTR; HandleId:LPVOID; ClientToken:HANDLE; DesiredAccess:DWORD; Privileges:PPRIVILEGE_SET;AccessGranted:WINBOOL):WINBOOL; external 'advapi32' name 'ObjectPrivilegeAuditAlarmW';
function ObjectCloseAuditAlarm(SubsystemName:LPCWSTR; HandleId:LPVOID; GenerateOnClose:WINBOOL):WINBOOL; external 'advapi32' name 'ObjectCloseAuditAlarmW';
function PrivilegedServiceAuditAlarm(SubsystemName:LPCWSTR; ServiceName:LPCWSTR; ClientToken:HANDLE; Privileges:PPRIVILEGE_SET; AccessGranted:WINBOOL):WINBOOL; external 'advapi32' name 'PrivilegedServiceAuditAlarmW';
function SetFileSecurity(lpFileName:LPCWSTR; SecurityInformation:SECURITY_INFORMATION; pSecurityDescriptor:PSECURITY_DESCRIPTOR):WINBOOL; external 'advapi32' name 'SetFileSecurityW';
function GetFileSecurity(lpFileName:LPCWSTR; RequestedInformation:SECURITY_INFORMATION; pSecurityDescriptor:PSECURITY_DESCRIPTOR; nLength:DWORD; lpnLengthNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'GetFileSecurityW';
function FindFirstChangeNotification(lpPathName:LPCWSTR; bWatchSubtree:WINBOOL; dwNotifyFilter:DWORD):HANDLE; external 'kernel32' name 'FindFirstChangeNotificationW';
function IsBadStringPtr(lpsz:LPCWSTR; ucchMax:UINT):WINBOOL; external 'kernel32' name 'IsBadStringPtrW';
function LookupAccountSid(lpSystemName:LPCWSTR; Sid:PSID; Name:LPWSTR; cbName:LPDWORD; ReferencedDomainName:LPWSTR;cbReferencedDomainName:LPDWORD; peUse:PSID_NAME_USE):WINBOOL; external 'advapi32' name 'LookupAccountSidW';
function LookupAccountName(lpSystemName:LPCWSTR; lpAccountName:LPCWSTR; Sid:PSID; cbSid:LPDWORD; ReferencedDomainName:LPWSTR;cbReferencedDomainName:LPDWORD; peUse:PSID_NAME_USE):WINBOOL; external 'advapi32' name 'LookupAccountNameW';
function LookupPrivilegeValue(lpSystemName:LPCWSTR; lpName:LPCWSTR; lpLuid:PLUID):WINBOOL; external 'advapi32' name 'LookupPrivilegeValueW';
function LookupPrivilegeName(lpSystemName:LPCWSTR; lpLuid:PLUID; lpName:LPWSTR; cbName:LPDWORD):WINBOOL; external 'advapi32' name 'LookupPrivilegeNameW';
function LookupPrivilegeDisplayName(lpSystemName:LPCWSTR; lpName:LPCWSTR; lpDisplayName:LPWSTR; cbDisplayName:LPDWORD; lpLanguageId:LPDWORD):WINBOOL; external 'advapi32' name 'LookupPrivilegeDisplayNameW';
function BuildCommDCB(lpDef:LPCWSTR; lpDCB:LPDCB):WINBOOL; external 'kernel32' name 'BuildCommDCBW';
function BuildCommDCBAndTimeouts(lpDef:LPCWSTR; lpDCB:LPDCB; lpCommTimeouts:LPCOMMTIMEOUTS):WINBOOL; external 'kernel32' name 'BuildCommDCBAndTimeoutsW';
function CommConfigDialog(lpszName:LPCWSTR; hWnd:HWND; lpCC:LPCOMMCONFIG):WINBOOL; external 'kernel32' name 'CommConfigDialogW';
function GetDefaultCommConfig(lpszName:LPCWSTR; lpCC:LPCOMMCONFIG; lpdwSize:LPDWORD):WINBOOL; external 'kernel32' name 'GetDefaultCommConfigW';
function SetDefaultCommConfig(lpszName:LPCWSTR; lpCC:LPCOMMCONFIG; dwSize:DWORD):WINBOOL; external 'kernel32' name 'SetDefaultCommConfigW';
function GetComputerName(lpBuffer:LPWSTR; nSize:LPDWORD):WINBOOL; external 'kernel32' name 'GetComputerNameW';
function SetComputerName(lpComputerName:LPCWSTR):WINBOOL; external 'kernel32' name 'SetComputerNameW';
function GetCPInfoEx(Codepage:UINT; dwFlags:DWORD; CPinfoEx:LPCPINFOEX):BOOL; external 'kernel32' name 'GetCPInfoExW';
function GetUserName(lpBuffer:LPWSTR; nSize:LPDWORD):WINBOOL; external 'advapi32' name 'GetUserNameW';
function wvsprintf(_para1:LPWSTR; _para2:LPCWSTR; arglist:va_list):longint; stdcall; external 'user32' name 'wvsprintfW';
function wsprintf(_para1:LPWSTR; _para2:LPCWSTR; const args:array of const):longint;cdecl; external 'user32' name 'wsprintfW';
function wsprintf(_para1:LPWSTR; _para2:LPCWSTR):longint;CDECL; external 'user32' name 'wsprintfW';
function LoadKeyboardLayout(pwszKLID:LPCWSTR; Flags:UINT):HKL; external 'user32' name 'LoadKeyboardLayoutW';
function GetKeyboardLayoutName(pwszKLID:LPWSTR):WINBOOL; external 'user32' name 'GetKeyboardLayoutNameW';
function CreateDesktop(lpszDesktop:LPWSTR; lpszDevice:LPWSTR; pDevmode:LPDEVMODE; dwFlags:DWORD; dwDesiredAccess:DWORD;lpsa:LPSECURITY_ATTRIBUTES):HDESK; external 'user32' name 'CreateDesktopW';
function OpenDesktop(lpszDesktop:LPWSTR; dwFlags:DWORD; fInherit:WINBOOL; dwDesiredAccess:DWORD):HDESK; external 'user32' name 'OpenDesktopW';
function EnumDesktops(hwinsta:HWINSTA; lpEnumFunc:DESKTOPENUMPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumDesktopsW';
function CreateWindowStation(lpwinsta:LPWSTR; dwReserved:DWORD; dwDesiredAccess:DWORD; lpsa:LPSECURITY_ATTRIBUTES):HWINSTA; external 'user32' name 'CreateWindowStationW';
function OpenWindowStation(lpszWinSta:LPWSTR; fInherit:WINBOOL; dwDesiredAccess:DWORD):HWINSTA; external 'user32' name 'OpenWindowStationW';
function EnumWindowStations(lpEnumFunc:ENUMWINDOWSTATIONPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumWindowStationsW';
function GetUserObjectInformation(hObj:HANDLE; nIndex:longint; pvInfo:PVOID; nLength:DWORD; lpnLengthNeeded:LPDWORD):WINBOOL; external 'user32' name 'GetUserObjectInformationW';
function SetUserObjectInformation(hObj:HANDLE; nIndex:longint; pvInfo:PVOID; nLength:DWORD):WINBOOL; external 'user32' name 'SetUserObjectInformationW';
function RegisterWindowMessage(lpString:LPCWSTR):UINT; external 'user32' name 'RegisterWindowMessageW';
function GetMessage(lpMsg:LPMSG; hWnd:HWND; wMsgFilterMin:UINT; wMsgFilterMax:UINT):WINBOOL; external 'user32' name 'GetMessageW';
function DispatchMessage(lpMsg:LPMSG):LONG; external 'user32' name 'DispatchMessageW';
function PeekMessage(lpMsg:LPMSG; hWnd:HWND; wMsgFilterMin:UINT; wMsgFilterMax:UINT; wRemoveMsg:UINT):WINBOOL; external 'user32' name 'PeekMessageW';
function SendMessage(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'SendMessageW';
function SendMessageTimeout(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM; fuFlags:UINT;uTimeout:UINT; lpdwResult:PDWORD_PTR):LRESULT; external 'user32' name 'SendMessageTimeoutW';
function SendNotifyMessage(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'SendNotifyMessageW';
function SendMessageCallback(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM; lpResultCallBack:SENDASYNCPROC;dwData:ULONG_PTR):WINBOOL; external 'user32' name 'SendMessageCallbackW';
function PostMessage(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'PostMessageW';
function PostThreadMessage(idThread:DWORD; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'PostThreadMessageW';
function DefWindowProc(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefWindowProcW';
function CallWindowProc(lpPrevWndFunc:WNDPROC; hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'CallWindowProcW';
function RegisterClass(lpWndClass:LPWNDCLASS):ATOM; external 'user32' name 'RegisterClassW';
function UnregisterClass(lpClassName:LPCWSTR; hInstance:HINST):WINBOOL; external 'user32' name 'UnregisterClassW';
function GetClassInfo(hInstance:HINST; lpClassName:LPCWSTR; lpWndClass:LPWNDCLASS):WINBOOL; external 'user32' name 'GetClassInfoW';
function RegisterClassEx(_para1:LPWNDCLASSEXW):ATOM; external 'user32' name 'RegisterClassExW';
function GetClassInfoEx(_para1:HINST; _para2:LPCWSTR; _para3:LPWNDCLASSEX):WINBOOL; external 'user32' name 'GetClassInfoExW';
function CreateWindowEx(dwExStyle:DWORD; lpClassName:LPCWSTR; lpWindowName:LPCWSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
  external 'user32' name 'CreateWindowExW';
function CreateDialogParam(hInstance:HINST; lpTemplateName:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):HWND; external 'user32' name 'CreateDialogParamW';
function CreateDialogIndirectParam(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):HWND; external 'user32' name 'CreateDialogIndirectParamW';
function DialogBoxParam(hInstance:HINST; lpTemplateName:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):longint; external 'user32' name 'DialogBoxParamW';
function DialogBoxIndirectParam(hInstance:HINST; hDialogTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):longint; external 'user32' name 'DialogBoxIndirectParamW';
function SetDlgItemText(hDlg:HWND; nIDDlgItem:longint; lpString:LPCWSTR):WINBOOL; external 'user32' name 'SetDlgItemTextW';
function GetDlgItemText(hDlg:HWND; nIDDlgItem:longint; lpString:LPWSTR; nMaxCount:longint):UINT; external 'user32' name 'GetDlgItemTextW';
function SendDlgItemMessage(hDlg:HWND; nIDDlgItem:longint; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LONG; external 'user32' name 'SendDlgItemMessageW';
function DefDlgProc(hDlg:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefDlgProcW';
function CallMsgFilter(lpMsg:LPMSG; nCode:longint):WINBOOL; external 'user32' name 'CallMsgFilterW';
function RegisterClipboardFormat(lpszFormat:LPCWSTR):UINT; external 'user32' name 'RegisterClipboardFormatW';
function GetClipboardFormatName(format:UINT; lpszFormatName:LPWSTR; cchMaxCount:longint):longint; external 'user32' name 'GetClipboardFormatNameW';
function CharToOem(lpszSrc:LPCWSTR; lpszDst:LPSTR):WINBOOL; external 'user32' name 'CharToOemW';
function OemToChar(lpszSrc:LPCSTR; lpszDst:LPWSTR):WINBOOL; external 'user32' name 'OemToCharW';
function CharToOemBuff(lpszSrc:LPCWSTR; lpszDst:LPSTR; cchDstLength:DWORD):WINBOOL; external 'user32' name 'CharToOemBuffW';
function OemToCharBuff(lpszSrc:LPCSTR; lpszDst:LPWSTR; cchDstLength:DWORD):WINBOOL; external 'user32' name 'OemToCharBuffW';
function CharUpper(lpsz:LPWSTR):LPWSTR; external 'user32' name 'CharUpperW';
function CharUpperBuff(lpsz:LPWSTR; cchLength:DWORD):DWORD; external 'user32' name 'CharUpperBuffW';
function CharLower(lpsz:LPWSTR):LPWSTR; external 'user32' name 'CharLowerW';
function CharLowerBuff(lpsz:LPWSTR; cchLength:DWORD):DWORD; external 'user32' name 'CharLowerBuffW';
function CharNext(lpsz:LPCWSTR):LPWSTR; external 'user32' name 'CharNextW';
function CharPrev(lpszStart:LPCWSTR; lpszCurrent:LPCWSTR):LPWSTR; external 'user32' name 'CharPrevW';
function IsCharAlpha(ch:WCHAR):WINBOOL; external 'user32' name 'IsCharAlphaW';
function IsCharAlphaNumeric(ch:WCHAR):WINBOOL; external 'user32' name 'IsCharAlphaNumericW';
function IsCharUpper(ch:WCHAR):WINBOOL; external 'user32' name 'IsCharUpperW';
function IsCharLower(ch:WCHAR):WINBOOL; external 'user32' name 'IsCharLowerW';
function GetKeyNameText(lParam:LONG; lpString:LPWSTR; nSize:longint):longint; external 'user32' name 'GetKeyNameTextW';
function VkKeyScan(ch:WCHAR):SHORT; external 'user32' name 'VkKeyScanW';
function VkKeyScanEx(ch:WCHAR; dwhkl:HKL):SHORT; external 'user32' name 'VkKeyScanExW';
function MapVirtualKey(uCode:UINT; uMapType:UINT):UINT; external 'user32' name 'MapVirtualKeyW';
function MapVirtualKeyEx(uCode:UINT; uMapType:UINT; dwhkl:HKL):UINT; external 'user32' name 'MapVirtualKeyExW';
function LoadAccelerators(hInstance:HINST; lpTableName:LPCWSTR):HACCEL; external 'user32' name 'LoadAcceleratorsW';
function CreateAcceleratorTable(_para1:LPACCEL; _para2:longint):HACCEL; external 'user32' name 'CreateAcceleratorTableW';
function CopyAcceleratorTable(hAccelSrc:HACCEL; lpAccelDst:LPACCEL; cAccelEntries:longint):longint; external 'user32' name 'CopyAcceleratorTableW';
function TranslateAccelerator(hWnd:HWND; hAccTable:HACCEL; lpMsg:LPMSG):longint; external 'user32' name 'TranslateAcceleratorW';
function LoadMenu(hInstance:HINST; lpMenuName:LPCWSTR):HMENU; external 'user32' name 'LoadMenuW';
function LoadMenuIndirect(lpMenuTemplate:LPMENUTEMPLATE):HMENU; external 'user32' name 'LoadMenuIndirectW';
function ChangeMenu(hMenu:HMENU; cmd:UINT; lpszNewItem:LPCWSTR; cmdInsert:UINT; flags:UINT):WINBOOL; external 'user32' name 'ChangeMenuW';
function GetMenuString(hMenu:HMENU; uIDItem:UINT; lpString:LPWSTR; nMaxCount:longint; uFlag:UINT):longint; external 'user32' name 'GetMenuStringW';
function InsertMenu(hMenu:HMENU; uPosition:UINT; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCWSTR):WINBOOL; external 'user32' name 'InsertMenuW';
function AppendMenu(hMenu:HMENU; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCWSTR):WINBOOL; external 'user32' name 'AppendMenuW';
function ModifyMenu(hMnu:HMENU; uPosition:UINT; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCWSTR):WINBOOL; external 'user32' name 'ModifyMenuW';
function InsertMenuItem(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPCMENUITEMINFO):WINBOOL; external 'user32' name 'InsertMenuItemW';
function GetMenuItemInfo(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPMENUITEMINFO):WINBOOL; external 'user32' name 'GetMenuItemInfoW';
function SetMenuItemInfo(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPCMENUITEMINFO):WINBOOL; external 'user32' name 'SetMenuItemInfoW';
function DrawText(hDC:HDC; lpString:LPCWSTR; nCount:longint; lpRect:LPRECT; uFormat:UINT):longint; external 'user32' name 'DrawTextW';
function DrawTextEx(_para1:HDC; _para2:LPWSTR; _para3:longint; _para4:LPRECT; _para5:UINT;_para6:LPDRAWTEXTPARAMS):longint; external 'user32' name 'DrawTextExW';
function GrayString(hDC:HDC; hBrush:HBRUSH; lpOutputFunc:GRAYSTRINGPROC; lpData:LPARAM; nCount:longint;X:longint; Y:longint; nWidth:longint; nHeight:longint):WINBOOL; external 'user32' name 'GrayStringW';
function DrawState(_para1:HDC; _para2:HBRUSH; _para3:DRAWSTATEPROC; _para4:LPARAM; _para5:WPARAM;_para6:longint; _para7:longint; _para8:longint; _para9:longint; _para10:UINT):WINBOOL; external 'user32' name 'DrawStateW';
function TabbedTextOut(hDC:HDC; X:longint; Y:longint; lpString:LPCWSTR; nCount:longint;nTabPositions:longint; lpnTabStopPositions:LPINT; nTabOrigin:longint):LONG; external 'user32' name 'TabbedTextOutW';
function GetTabbedTextExtent(hDC:HDC; lpString:LPCWSTR; nCount:longint; nTabPositions:longint; lpnTabStopPositions:LPINT):DWORD; external 'user32' name 'GetTabbedTextExtentW';
function SetProp(hWnd:HWND; lpString:LPCWSTR; hData:HANDLE):WINBOOL; external 'user32' name 'SetPropW';
function GetProp(hWnd:HWND; lpString:LPCWSTR):HANDLE; external 'user32' name 'GetPropW';
function RemoveProp(hWnd:HWND; lpString:LPCWSTR):HANDLE; external 'user32' name 'RemovePropW';
function EnumPropsEx(hWnd:HWND; lpEnumFunc:PROPENUMPROCEX; lParam:LPARAM):longint; external 'user32' name 'EnumPropsExW';
function EnumProps(hWnd:HWND; lpEnumFunc:PROPENUMPROC):longint; external 'user32' name 'EnumPropsW';
function SetWindowText(hWnd:HWND; lpString:LPCWSTR):WINBOOL; external 'user32' name 'SetWindowTextW';
function GetWindowText(hWnd:HWND; lpString:LPWSTR; nMaxCount:longint):longint; external 'user32' name 'GetWindowTextW';
function GetWindowTextLength(hWnd:HWND):longint; external 'user32' name 'GetWindowTextLengthW';
function MessageBox(hWnd:HWND; lpText:LPCWSTR; lpCaption:LPCWSTR; uType:UINT):longint; external 'user32' name 'MessageBoxW';
function MessageBoxEx(hWnd:HWND; lpText:LPCWSTR; lpCaption:LPCWSTR; uType:UINT; wLanguageId:WORD):longint; external 'user32' name 'MessageBoxExW';
function MessageBoxIndirect(_para1:LPMSGBOXPARAMS):longint; external 'user32' name 'MessageBoxIndirectW';
function GetWindowLong(hWnd:HWND; nIndex:longint):LONG; external 'user32' name 'GetWindowLongW';
function SetWindowLong(hWnd:HWND; nIndex:longint; dwNewLong:LONG):LONG; external 'user32' name 'SetWindowLongW';
function GetClassLong(hWnd:HWND; nIndex:longint):DWORD; external 'user32' name 'GetClassLongW';
function SetClassLong(hWnd:HWND; nIndex:longint; dwNewLong:LONG):DWORD; external 'user32' name 'SetClassLongW';
{$ifdef cpu64}
function GetWindowLongPtr(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetWindowLongPtrW';
function SetWindowLongPtr(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetWindowLongPtrW';
function GetClassLongPtr(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetClassLongPtrW';
function SetClassLongPtr(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetClassLongPtrW';
{$else}
function GetWindowLongPtr(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetWindowLongW';
function SetWindowLongPtr(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetWindowLongW';
function GetClassLongPtr(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetClassLongW';
function SetClassLongPtr(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetClassLongW';
{$endif}
function FindWindow(lpClassName:LPCWSTR; lpWindowName:LPCWSTR):HWND; external 'user32' name 'FindWindowW';
function FindWindowEx(_para1:HWND; _para2:HWND; _para3:LPCWSTR; _para4:LPCWSTR):HWND; external 'user32' name 'FindWindowExW';
function GetClassName(hWnd:HWND; lpClassName:LPWSTR; nMaxCount:longint):longint; external 'user32' name 'GetClassNameW';
function SetWindowsHookEx(idHook:longint; lpfn:HOOKPROC; hmod:HINST; dwThreadId:DWORD):HHOOK; external 'user32' name 'SetWindowsHookExW';
function LoadBitmap(hInstance:HINST; lpBitmapName:LPCWSTR):HBITMAP; external 'user32' name 'LoadBitmapW';
function LoadCursor(hInstance:HINST; lpCursorName:LPCWSTR):HCURSOR; external 'user32' name 'LoadCursorW';
function LoadCursorFromFile(lpFileName:LPCWSTR):HCURSOR; external 'user32' name 'LoadCursorFromFileW';
function LoadIcon(hInstance:HINST; lpIconName:LPCWSTR):HICON; external 'user32' name 'LoadIconW';
function LoadImage(_para1:HINST; _para2:LPCWSTR; _para3:UINT; _para4:longint; _para5:longint;_para6:UINT):HANDLE; external 'user32' name 'LoadImageW';
function LoadString(hInstance:HINST; uID:UINT; lpBuffer:LPWSTR; nBufferMax:longint):longint; external 'user32' name 'LoadStringW';
function IsDialogMessage(hDlg:HWND; lpMsg:LPMSG):WINBOOL; external 'user32' name 'IsDialogMessageW';
function DlgDirList(hDlg:HWND; lpPathSpec:LPWSTR; nIDListBox:longint; nIDStaticPath:longint; uFileType:UINT):longint; external 'user32' name 'DlgDirListW';
function DlgDirSelectEx(hDlg:HWND; lpString:LPWSTR; nCount:longint; nIDListBox:longint):WINBOOL; external 'user32' name 'DlgDirSelectExW';
function DlgDirListComboBox(hDlg:HWND; lpPathSpec:LPWSTR; nIDComboBox:longint; nIDStaticPath:longint; uFiletype:UINT):longint; external 'user32' name 'DlgDirListComboBoxW';
function DlgDirSelectComboBoxEx(hDlg:HWND; lpString:LPWSTR; nCount:longint; nIDComboBox:longint):WINBOOL; external 'user32' name 'DlgDirSelectComboBoxExW';
function DefFrameProc(hWnd:HWND; hWndMDIClient:HWND; uMsg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefFrameProcW';
function DefMDIChildProc(hWnd:HWND; uMsg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefMDIChildProcW';
function CreateMDIWindow(lpClassName:LPWSTR; lpWindowName:LPWSTR; dwStyle:DWORD; X:longint; Y:longint;nWidth:longint; nHeight:longint; hWndParent:HWND; hInstance:HINST; lParam:LPARAM):HWND; external 'user32' name 'CreateMDIWindowW';
function WinHelp(hWndMain:HWND; lpszHelp:LPCWSTR; uCommand:UINT; dwData:DWORD):WINBOOL; external 'user32' name 'WinHelpW';
function ChangeDisplaySettings(lpDevMode:LPDEVMODE; dwFlags:DWORD):LONG; external 'user32' name 'ChangeDisplaySettingsW';
function EnumDisplaySettings(lpszDeviceName:LPCWSTR; iModeNum:DWORD; lpDevMode:LPDEVMODEW):WINBOOL; external 'user32' name 'EnumDisplaySettingsW';
function SystemParametersInfo(uiAction:UINT; uiParam:UINT; pvParam:PVOID; fWinIni:UINT):WINBOOL; external 'user32' name 'SystemParametersInfoW';
function AddFontResource(_para1:LPCWSTR):longint; external 'gdi32' name 'AddFontResourceW';
function CopyMetaFile(_para1:HMETAFILE; _para2:LPCWSTR):HMETAFILE; external 'gdi32' name 'CopyMetaFileW';
function CreateFontIndirect(_para1:PLOGFONT):HFONT; external 'gdi32' name 'CreateFontIndirectW';
function CreateFontIndirect(var _para1:LOGFONT):HFONT; external 'gdi32' name 'CreateFontIndirectW';
function CreateFont(_para1:longint; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:DWORD; _para7:DWORD; _para8:DWORD; _para9:DWORD; _para10:DWORD;_para11:DWORD; _para12:DWORD; _para13:DWORD; _para14:LPCWSTR):HFONT;
  external 'gdi32' name 'CreateFontW';
function CreateIC(_para1:LPCWSTR; _para2:LPCWSTR; _para3:LPCWSTR; _para4:LPDEVMODE):HDC; external 'gdi32' name 'CreateICW';
function CreateMetaFile(_para1:LPCWSTR):HDC; external 'gdi32' name 'CreateMetaFileW';
function CreateScalableFontResource(_para1:DWORD; _para2:LPCWSTR; _para3:LPCWSTR; _para4:LPCWSTR):WINBOOL; external 'gdi32' name 'CreateScalableFontResourceW';
function EnumFontFamiliesEx(_para1:HDC; _para2:LPLOGFONT; _para3:FONTENUMEXPROC; _para4:LPARAM; _para5:DWORD):longint; external 'gdi32' name 'EnumFontFamiliesExW';
function EnumFontFamilies(_para1:HDC; _para2:LPCWSTR; _para3:FONTENUMPROC; _para4:LPARAM):longint; external 'gdi32' name 'EnumFontFamiliesW';
function EnumFonts(_para1:HDC; _para2:LPCWSTR; _para3:ENUMFONTSPROC; _para4:LPARAM):longint; external 'gdi32' name 'EnumFontsW';
function EnumFonts(_para1:HDC; _para2:LPCWSTR; _para3:ENUMFONTSPROC; _para4:pointer):longint; external 'gdi32' name 'EnumFontsW';
function GetCharWidth(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPINT):WINBOOL; external 'gdi32' name 'GetCharWidthW';
function GetCharWidth32(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPINT):WINBOOL; external 'gdi32' name 'GetCharWidth32W';
function GetCharWidthFloat(_para1:HDC; _para2:UINT; _para3:UINT; _para4:PSingle):WINBOOL; external 'gdi32' name 'GetCharWidthFloatW';
function GetCharABCWidths(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPABC):WINBOOL; external 'gdi32' name 'GetCharABCWidthsW';
function GetCharABCWidthsFloat(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPABCFLOAT):WINBOOL; external 'gdi32' name 'GetCharABCWidthsFloatW';
function GetGlyphOutline(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPGLYPHMETRICS; _para5:DWORD;_para6:LPVOID; _para7:PMAT2):DWORD; external 'gdi32' name 'GetGlyphOutlineW';
function GetMetaFile(_para1:LPCWSTR):HMETAFILE; external 'gdi32' name 'GetMetaFileW';
function GetOutlineTextMetrics(_para1:HDC; _para2:UINT; _para3:LPOUTLINETEXTMETRIC):UINT; external 'gdi32' name 'GetOutlineTextMetricsW';
function GetTextExtentPoint(_para1:HDC; _para2:LPCWSTR; _para3:longint; _para4:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentPointW';
function GetTextExtentPoint32(_para1:HDC; _para2:LPCWSTR; _para3:longint; _para4:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentPoint32W';
function GetTextExtentExPoint(_para1:HDC; _para2:LPCWSTR; _para3:longint; _para4:longint; _para5:LPINT;_para6:LPINT; _para7:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentExPointW';
function GetCharacterPlacement(_para1:HDC; _para2:LPCWSTR; _para3:longint; _para4:longint; _para5:LPGCP_RESULTS;_para6:DWORD):DWORD; external 'gdi32' name 'GetCharacterPlacementW';
function ResetDC(_para1:HDC; _para2:LPDEVMODE):HDC; external 'gdi32' name 'ResetDCW';
function RemoveFontResource(_para1:LPCWSTR):WINBOOL; external 'gdi32' name 'RemoveFontResourceW';
function CopyEnhMetaFile(_para1:HENHMETAFILE; _para2:LPCWSTR):HENHMETAFILE; external 'gdi32' name 'CopyEnhMetaFileW';
function CreateEnhMetaFile(_para1:HDC; _para2:LPCWSTR; _para3:LPRECT; _para4:LPCWSTR):HDC; external 'gdi32' name 'CreateEnhMetaFileW';
function GetEnhMetaFile(_para1:LPCWSTR):HENHMETAFILE; external 'gdi32' name 'GetEnhMetaFileW';
function GetEnhMetaFileDescription(_para1:HENHMETAFILE; _para2:UINT; _para3:LPWSTR):UINT; external 'gdi32' name 'GetEnhMetaFileDescriptionW';
function GetTextMetrics(_para1:HDC; _para2:LPTEXTMETRIC):WINBOOL; external 'gdi32' name 'GetTextMetricsW';
function StartDoc(_para1:HDC; _para2:PDOCINFO):longint; external 'gdi32' name 'StartDocW';
function GetObject(_para1:HGDIOBJ; _para2:longint; _para3:LPVOID):longint; external 'gdi32' name 'GetObjectW';
function TextOut(_para1:HDC; _para2:longint; _para3:longint; _para4:LPCWSTR; _para5:longint):WINBOOL; external 'gdi32' name 'TextOutW';
function ExtTextOut(_para1:HDC; _para2:longint; _para3:longint; _para4:UINT; _para5:LPRECT;_para6:LPCWSTR; _para7:UINT; _para8:LPINT):WINBOOL; external 'gdi32' name 'ExtTextOutW';
function PolyTextOut(_para1:HDC; _para2:PPOLYTEXT; _para3:longint):WINBOOL; external 'gdi32' name 'PolyTextOutW';
function GetTextFace(_para1:HDC; _para2:longint; _para3:LPWSTR):longint; external 'gdi32' name 'GetTextFaceW';
function GetKerningPairs(_para1:HDC; _para2:DWORD; _para3:LPKERNINGPAIR):DWORD; external 'gdi32' name 'GetKerningPairsW';
function GetLogColorSpace(_para1:HCOLORSPACE; _para2:LPLOGCOLORSPACE; _para3:DWORD):WINBOOL; external 'gdi32' name 'GetLogColorSpaceW';
function CreateColorSpace(_para1:LPLOGCOLORSPACE):HCOLORSPACE; external 'gdi32' name 'CreateColorSpaceW';
function GetICMProfile(_para1:HDC; _para2:DWORD; _para3:LPWSTR):WINBOOL; external 'gdi32' name 'GetICMProfileW';
function SetICMProfile(_para1:HDC; _para2:LPWSTR):WINBOOL; external 'gdi32' name 'SetICMProfileW';
function UpdateICMRegKey(_para1:DWORD; _para2:DWORD; _para3:LPWSTR; _para4:UINT):WINBOOL; external 'gdi32' name 'UpdateICMRegKeyW';
function EnumICMProfiles(_para1:HDC; _para2:ICMENUMPROC; _para3:LPARAM):longint; external 'gdi32' name 'EnumICMProfilesW';
{
function CreatePropertySheetPage(lppsp:LPCPROPSHEETPAGE):HPROPSHEETPAGE; external 'comctl32' name 'CreatePropertySheetPageW';
function PropertySheet(lppsph:LPCPROPSHEETHEADER):longint; external 'comctl32' name 'PropertySheetW';
function ImageList_LoadImage(hi:HINST; lpbmp:LPCWSTR; cx:longint; cGrow:longint; crMask:COLORREF;uType:UINT; uFlags:UINT):HIMAGELIST; external 'comctl32' name 'ImageList_LoadImageW';
function CreateStatusWindow(style:LONG; lpszText:LPCWSTR; hwndParent:HWND; wID:UINT):HWND; external 'comctl32' name 'CreateStatusWindowW';
procedure DrawStatusText(hDC:HDC; lprc:LPRECT; pszText:LPCWSTR; uFlags:UINT); external 'comctl32' name 'DrawStatusTextW';


}
function CreateProcess(lpApplicationName:LPCWSTR; lpCommandLine:LPWSTR; lpProcessAttributes:LPSECURITY_ATTRIBUTES; lpThreadAttributes:LPSECURITY_ATTRIBUTES; bInheritHandles:WINBOOL;dwCreationFlags:DWORD; lpEnvironment:LPVOID;
  lpCurrentDirectory:LPCWSTR; lpStartupInfo:LPSTARTUPINFO; lpProcessInformation:LPPROCESS_INFORMATION):WINBOOL; external 'kernel32' name 'CreateProcessW';
procedure GetStartupInfo(lpStartupInfo:LPSTARTUPINFO); external 'kernel32' name 'GetStartupInfoW';
function FindFirstFile(lpFileName:LPCWSTR; lpFindFileData:LPWIN32_FIND_DATA):HANDLE; external 'kernel32' name 'FindFirstFileW';
function FindNextFile(hFindFile:HANDLE; lpFindFileData:LPWIN32_FIND_DATA):WINBOOL; external 'kernel32' name 'FindNextFileW';
function GetVersionEx(VersionInformation:LPOSVERSIONINFOW):WINBOOL; external 'kernel32' name 'GetVersionExW';
function CreateWindow(lpClassName:LPCWSTR; lpWindowName:LPCWSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
function CreateDialog(hInstance:HINST; lpName:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
function CreateDialogIndirect(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
function DialogBox(hInstance:HINST; lpTemplate:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
function DialogBoxIndirect(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
function CreateDC(_para1:LPCWSTR; _para2:LPCWSTR; _para3:LPCWSTR; _para4:pDEVMODE):HDC; external 'gdi32' name 'CreateDCW';
function VerInstallFile(uFlags:DWORD; szSrcFileName:LPWSTR; szDestFileName:LPWSTR; szSrcDir:LPWSTR; szDestDir:LPWSTR;szCurDir:LPWSTR; szTmpFile:LPWSTR; lpuTmpFileLen:PUINT):DWORD; external 'version' name 'VerInstallFileW';
function GetFileVersionInfoSize(lptstrFilename:LPWSTR; lpdwHandle:LPDWORD):DWORD; external 'version' name 'GetFileVersionInfoSizeW';
function GetFileVersionInfo(lptstrFilename:LPWSTR; dwHandle:DWORD; dwLen:DWORD; lpData:LPVOID):WINBOOL; external 'version' name 'GetFileVersionInfoW';
function VerLanguageName(wLang:DWORD; szLang:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'VerLanguageNameW';
function VerQueryValue(pBlock:LPVOID; lpSubBlock:LPWSTR; lplpBuffer:LPVOID; puLen:PUINT):WINBOOL; external 'version' name 'VerQueryValueW';
function VerFindFile(uFlags:DWORD; szFileName:LPWSTR; szWinDir:LPWSTR; szAppDir:LPWSTR; szCurDir:LPWSTR;lpuCurDirLen:PUINT; szDestDir:LPWSTR; lpuDestDirLen:PUINT):DWORD; external 'version' name 'VerFindFileW';
function RegSetValueEx(hKey:HKEY; lpValueName:LPCWSTR; Reserved:DWORD; dwType:DWORD; lpData:LPBYTE;cbData:DWORD):LONG; external 'advapi32' name 'RegSetValueExW';
function RegUnLoadKey(hKey:HKEY; lpSubKey:LPCWSTR):LONG; external 'advapi32' name 'RegUnLoadKeyW';
function InitiateSystemShutdown(lpMachineName:LPWSTR; lpMessage:LPWSTR; dwTimeout:DWORD; bForceAppsClosed:WINBOOL; bRebootAfterShutdown:WINBOOL):WINBOOL; external 'advapi32' name 'InitiateSystemShutdownW';
function AbortSystemShutdown(lpMachineName:LPWSTR):WINBOOL; external 'advapi32' name 'AbortSystemShutdownW';
function RegRestoreKey(hKey:HKEY; lpFile:LPCWSTR; dwFlags:DWORD):LONG; external 'advapi32' name 'RegRestoreKeyW';
function RegSaveKey(hKey:HKEY; lpFile:LPCWSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):LONG; external 'advapi32' name 'RegSaveKeyW';
function RegSetValue(hKey:HKEY; lpSubKey:LPCWSTR; dwType:DWORD; lpData:LPCWSTR; cbData:DWORD):LONG; external 'advapi32' name 'RegSetValueW';
function RegQueryValue(hKey:HKEY; lpSubKey:LPCWSTR; lpValue:LPWSTR; lpcbValue:PLONG):LONG; external 'advapi32' name 'RegQueryValueW';
function RegQueryMultipleValues(hKey:HKEY; val_list:PVALENT; num_vals:DWORD; lpValueBuf:LPWSTR; ldwTotsize:LPDWORD):LONG; external 'advapi32' name 'RegQueryMultipleValuesW';
function RegQueryValueEx(hKey:HKEY; lpValueName:LPCWSTR; lpReserved:LPDWORD; lpType:LPDWORD; lpData:LPBYTE;lpcbData:LPDWORD):LONG; external 'advapi32' name 'RegQueryValueExW';
function RegReplaceKey(hKey:HKEY; lpSubKey:LPCWSTR; lpNewFile:LPCWSTR; lpOldFile:LPCWSTR):LONG; external 'advapi32' name 'RegReplaceKeyW';
function RegConnectRegistry(lpMachineName:LPWSTR; hKey:HKEY; phkResult:PHKEY):LONG; external 'advapi32' name 'RegConnectRegistryW';
function RegCreateKey(hKey:HKEY; lpSubKey:LPCWSTR; phkResult:PHKEY):LONG; external 'advapi32' name 'RegCreateKeyW';
function RegCreateKeyEx(hKey:HKEY; lpSubKey:LPCWSTR; Reserved:DWORD; lpClass:LPWSTR; dwOptions:DWORD;samDesired:REGSAM; lpSecurityAttributes:LPSECURITY_ATTRIBUTES; phkResult:PHKEY; lpdwDisposition:LPDWORD):LONG;
  external 'advapi32' name 'RegCreateKeyExW';
function RegDeleteKey(hKey:HKEY; lpSubKey:LPCWSTR):LONG; external 'advapi32' name 'RegDeleteKeyW';
function RegDeleteValue(hKey:HKEY; lpValueName:LPCWSTR):LONG; external 'advapi32' name 'RegDeleteValueW';
function RegEnumKey(hKey:HKEY; dwIndex:DWORD; lpName:LPWSTR; cbName:DWORD):LONG; external 'advapi32' name 'RegEnumKeyW';
function RegEnumKeyEx(hKey:HKEY; dwIndex:DWORD; lpName:LPWSTR; lpcbName:LPDWORD; lpReserved:LPDWORD;lpClass:LPWSTR; lpcbClass:LPDWORD; lpftLastWriteTime:PFILETIME):LONG; external 'advapi32' name 'RegEnumKeyExW';
function RegEnumValue(hKey:HKEY; dwIndex:DWORD; lpValueName:LPWSTR; lpcbValueName:LPDWORD; lpReserved:LPDWORD;lpType:LPDWORD; lpData:LPBYTE; lpcbData:LPDWORD):LONG; external 'advapi32' name 'RegEnumValueW';
function RegLoadKey(hKey:HKEY; lpSubKey:LPCWSTR; lpFile:LPCWSTR):LONG; external 'advapi32' name 'RegLoadKeyW';
function RegOpenKey(hKey:HKEY; lpSubKey:LPCWSTR; phkResult:PHKEY):LONG; external 'advapi32' name 'RegOpenKeyW';
function RegOpenKeyEx(hKey:HKEY; lpSubKey:LPCWSTR; ulOptions:DWORD; samDesired:REGSAM; phkResult:PHKEY):LONG; external 'advapi32' name 'RegOpenKeyExW';
function RegQueryInfoKey(hKey:HKEY; lpClass:LPWSTR; lpcbClass:LPDWORD; lpReserved:LPDWORD; lpcSubKeys:LPDWORD;lpcbMaxSubKeyLen:LPDWORD; lpcbMaxClassLen:LPDWORD; lpcValues:LPDWORD; lpcbMaxValueNameLen:LPDWORD;
  lpcbMaxValueLen:LPDWORD;lpcbSecurityDescriptor:LPDWORD; lpftLastWriteTime:PFILETIME):LONG; external 'advapi32' name 'RegQueryInfoKeyW';
function CompareString(Locale:LCID; dwCmpFlags:DWORD; lpString1:LPCWSTR; cchCount1:longint; lpString2:LPCWSTR;cchCount2:longint):longint; external 'kernel32' name 'CompareStringW';
function LCMapString(Locale:LCID; dwMapFlags:DWORD; lpSrcStr:LPCWSTR; cchSrc:longint; lpDestStr:LPWSTR;cchDest:longint):longint; external 'kernel32' name 'LCMapStringW';
function GetLocaleInfo(Locale:LCID; LCType:LCTYPE; lpLCData:LPWSTR; cchData:longint):longint; external 'kernel32' name 'GetLocaleInfoW';
function SetLocaleInfo(Locale:LCID; LCType:LCTYPE; lpLCData:LPCWSTR):WINBOOL; external 'kernel32' name 'SetLocaleInfoW';
function GetTimeFormat(Locale:LCID; dwFlags:DWORD; lpTime:LPSYSTEMTIME; lpFormat:LPCWSTR; lpTimeStr:LPWSTR;cchTime:longint):longint; external 'kernel32' name 'GetTimeFormatW';
function GetDateFormat(Locale:LCID; dwFlags:DWORD; lpDate:LPSYSTEMTIME; lpFormat:LPCWSTR; lpDateStr:LPWSTR;cchDate:longint):longint; external 'kernel32' name 'GetDateFormatW';
function GetNumberFormat(Locale:LCID; dwFlags:DWORD; lpValue:LPCWSTR; lpFormat:PNUMBERFMT; lpNumberStr:LPWSTR;cchNumber:longint):longint; external 'kernel32' name 'GetNumberFormatW';
function GetCurrencyFormat(Locale:LCID; dwFlags:DWORD; lpValue:LPCWSTR; lpFormat:PCURRENCYFMT; lpCurrencyStr:LPWSTR;cchCurrency:longint):longint; external 'kernel32' name 'GetCurrencyFormatW';
function EnumCalendarInfo(lpCalInfoEnumProc:CALINFO_ENUMPROC; Locale:LCID; Calendar:CALID; CalType:CALTYPE):WINBOOL; external 'kernel32' name 'EnumCalendarInfoW';
function EnumTimeFormats(lpTimeFmtEnumProc:TIMEFMT_ENUMPROC; Locale:LCID; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumTimeFormatsW';
function EnumDateFormats(lpDateFmtEnumProc:DATEFMT_ENUMPROC; Locale:LCID; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumDateFormatsW';
function GetStringTypeEx(Locale:LCID; dwInfoType:DWORD; lpSrcStr:LPCWSTR; cchSrc:longint; lpCharType:LPWORD):WINBOOL; external 'kernel32' name 'GetStringTypeExW';
function GetStringType(dwInfoType:DWORD; lpSrcStr:LPCWSTR; cchSrc:longint; lpCharType:LPWORD):WINBOOL; external 'kernel32' name 'GetStringTypeW';
function FoldString(dwMapFlags:DWORD; lpSrcStr:LPCWSTR; cchSrc:longint; lpDestStr:LPWSTR; cchDest:longint):longint; external 'kernel32' name 'FoldStringW';
function EnumSystemLocales(lpLocaleEnumProc:LOCALE_ENUMPROC; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumSystemLocalesW';
function EnumSystemCodePages(lpCodePageEnumProc:CODEPAGE_ENUMPROC; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumSystemCodePagesW';
function PeekConsoleInput(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsRead:LPDWORD):WINBOOL; external 'kernel32' name 'PeekConsoleInputW';
function ReadConsoleInput(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsRead:LPDWORD):WINBOOL; external 'kernel32' name 'ReadConsoleInputW';
function WriteConsoleInput(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'WriteConsoleInputW';
function ReadConsoleOutput(hConsoleOutput:HANDLE; lpBuffer:PCHAR_INFO; dwBufferSize:COORD; dwBufferCoord:COORD; lpReadRegion:PSMALL_RECT):WINBOOL; external 'kernel32' name 'ReadConsoleOutputW';
function WriteConsoleOutput(hConsoleOutput:HANDLE; lpBuffer:PCHAR_INFO; dwBufferSize:COORD; dwBufferCoord:COORD; lpWriteRegion:PSMALL_RECT):WINBOOL; external 'kernel32' name 'WriteConsoleOutputW';
function ReadConsoleOutputCharacter(hConsoleOutput:HANDLE; lpCharacter:LPWSTR; nLength:DWORD; dwReadCoord:COORD; lpNumberOfCharsRead:LPDWORD):WINBOOL; external 'kernel32' name 'ReadConsoleOutputCharacterW';
function WriteConsoleOutputCharacter(hConsoleOutput:HANDLE; lpCharacter:LPCWSTR; nLength:DWORD; dwWriteCoord:COORD; lpNumberOfCharsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'WriteConsoleOutputCharacterW';
function FillConsoleOutputCharacter(hConsoleOutput:HANDLE; cCharacter:WCHAR; nLength:DWORD; dwWriteCoord:COORD; lpNumberOfCharsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'FillConsoleOutputCharacterW';
function ScrollConsoleScreenBuffer(hConsoleOutput:HANDLE; lpScrollRectangle:PSMALL_RECT; lpClipRectangle:PSMALL_RECT; dwDestinationOrigin:COORD; lpFill:PCHAR_INFO):WINBOOL; external 'kernel32' name 'ScrollConsoleScreenBufferW';
function GetConsoleTitle(lpConsoleTitle:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetConsoleTitleW';
function SetConsoleTitle(lpConsoleTitle:LPCWSTR):WINBOOL; external 'kernel32' name 'SetConsoleTitleW';
function ReadConsole(hConsoleInput:HANDLE; lpBuffer:LPVOID; nNumberOfCharsToRead:DWORD; lpNumberOfCharsRead:LPDWORD; lpReserved:LPVOID):WINBOOL; external 'kernel32' name 'ReadConsoleW';
function WriteConsole(hConsoleOutput:HANDLE;lpBuffer:pointer; nNumberOfCharsToWrite:DWORD; lpNumberOfCharsWritten:LPDWORD; lpReserved:LPVOID):WINBOOL; external 'kernel32' name 'WriteConsoleW';
function WNetAddConnection(lpRemoteName:LPCWSTR; lpPassword:LPCWSTR; lpLocalName:LPCWSTR):DWORD; external 'mpr' name 'WNetAddConnectionW';
function WNetAddConnection2(lpNetResource:LPNETRESOURCE; lpPassword:LPCWSTR; lpUserName:LPCWSTR; dwFlags:DWORD):DWORD; external 'mpr' name 'WNetAddConnection2W';
function WNetAddConnection3(hwndOwner:HWND; lpNetResource:LPNETRESOURCE; lpPassword:LPCWSTR; lpUserName:LPCWSTR; dwFlags:DWORD):DWORD; external 'mpr' name 'WNetAddConnection3W';
function WNetCancelConnection(lpName:LPCWSTR; fForce:WINBOOL):DWORD; external 'mpr' name 'WNetCancelConnectionW';
function WNetCancelConnection2(lpName:LPCWSTR; dwFlags:DWORD; fForce:WINBOOL):DWORD; external 'mpr' name 'WNetCancelConnection2W';
function WNetGetConnection(lpLocalName:LPCWSTR; lpRemoteName:LPWSTR; lpnLength:LPDWORD):DWORD; external 'mpr' name 'WNetGetConnectionW';
function WNetUseConnection(hwndOwner:HWND; lpNetResource:LPNETRESOURCE; lpUserID:LPCWSTR; lpPassword:LPCWSTR; dwFlags:DWORD;lpAccessName:LPWSTR; lpBufferSize:LPDWORD; lpResult:LPDWORD):DWORD; external 'mpr' name 'WNetUseConnectionW';
function WNetSetConnection(lpName:LPCWSTR; dwProperties:DWORD; pvValues:LPVOID):DWORD; external 'mpr' name 'WNetSetConnectionW';
function WNetConnectionDialog1(lpConnDlgStruct:LPCONNECTDLGSTRUCT):DWORD; external 'mpr' name 'WNetConnectionDialog1W';
function WNetDisconnectDialog1(lpConnDlgStruct:LPDISCDLGSTRUCT):DWORD; external 'mpr' name 'WNetDisconnectDialog1W';
function WNetOpenEnum(dwScope:DWORD; dwType:DWORD; dwUsage:DWORD; lpNetResource:LPNETRESOURCE; lphEnum:LPHANDLE):DWORD; external 'mpr' name 'WNetOpenEnumW';
function WNetEnumResource(hEnum:HANDLE; lpcCount:LPDWORD; lpBuffer:LPVOID; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetEnumResourceW';
function WNetGetUniversalName(lpLocalPath:LPCWSTR; dwInfoLevel:DWORD; lpBuffer:LPVOID; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetGetUniversalNameW';
function WNetGetUser(lpName:LPCWSTR; lpUserName:LPWSTR; lpnLength:LPDWORD):DWORD; external 'mpr' name 'WNetGetUserW';
function WNetGetProviderName(dwNetType:DWORD; lpProviderName:LPWSTR; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetGetProviderNameW';
function WNetGetNetworkInformation(lpProvider:LPCWSTR; lpNetInfoStruct:LPNETINFOSTRUCT):DWORD; external 'mpr' name 'WNetGetNetworkInformationW';
function WNetGetLastError(lpError:LPDWORD; lpErrorBuf:LPWSTR; nErrorBufSize:DWORD; lpNameBuf:LPWSTR; nNameBufSize:DWORD):DWORD; external 'mpr' name 'WNetGetLastErrorW';
function MultinetGetConnectionPerformance(lpNetResource:LPNETRESOURCE; lpNetConnectInfoStruct:LPNETCONNECTINFOSTRUCT):DWORD; external 'mpr' name 'MultinetGetConnectionPerformanceW';
function ChangeServiceConfig(hService:SC_HANDLE; dwServiceType:DWORD; dwStartType:DWORD; dwErrorControl:DWORD; lpBinaryPathName:LPCWSTR;lpLoadOrderGroup:LPCWSTR; lpdwTagId:LPDWORD; lpDependencies:LPCWSTR; lpServiceStartName:LPCWSTR;
  lpPassword:LPCWSTR;lpDisplayName:LPCWSTR):WINBOOL; external 'advapi32' name 'ChangeServiceConfigW';
function CreateService(hSCManager:SC_HANDLE; lpServiceName:LPCWSTR; lpDisplayName:LPCWSTR; dwDesiredAccess:DWORD; dwServiceType:DWORD;dwStartType:DWORD; dwErrorControl:DWORD; lpBinaryPathName:LPCWSTR; lpLoadOrderGroup:LPCWSTR;
  lpdwTagId:LPDWORD;lpDependencies:LPCWSTR; lpServiceStartName:LPCWSTR; lpPassword:LPCWSTR):SC_HANDLE; external 'advapi32' name 'CreateServiceW';
function EnumDependentServices(hService:SC_HANDLE; dwServiceState:DWORD; lpServices:LPENUM_SERVICE_STATUS; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD;lpServicesReturned:LPDWORD):WINBOOL; external 'advapi32' name 'EnumDependentServicesW';
function EnumServicesStatus(hSCManager:SC_HANDLE; dwServiceType:DWORD; dwServiceState:DWORD; lpServices:LPENUM_SERVICE_STATUS; cbBufSize:DWORD;pcbBytesNeeded:LPDWORD; lpServicesReturned:LPDWORD; lpResumeHandle:LPDWORD):WINBOOL;
  external 'advapi32' name 'EnumServicesStatusW';
function GetServiceKeyName(hSCManager:SC_HANDLE; lpDisplayName:LPCWSTR; lpServiceName:LPWSTR; lpcchBuffer:LPDWORD):WINBOOL; external 'advapi32' name 'GetServiceKeyNameW';
function GetServiceDisplayName(hSCManager:SC_HANDLE; lpServiceName:LPCWSTR; lpDisplayName:LPWSTR; lpcchBuffer:LPDWORD):WINBOOL; external 'advapi32' name 'GetServiceDisplayNameW';
function OpenSCManager(lpMachineName:LPCWSTR; lpDatabaseName:LPCWSTR; dwDesiredAccess:DWORD):SC_HANDLE; external 'advapi32' name 'OpenSCManagerW';
function OpenService(hSCManager:SC_HANDLE; lpServiceName:LPCWSTR; dwDesiredAccess:DWORD):SC_HANDLE; external 'advapi32' name 'OpenServiceW';
function QueryServiceConfig(hService:SC_HANDLE; lpServiceConfig:LPQUERY_SERVICE_CONFIG; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'QueryServiceConfigW';
function QueryServiceLockStatus(hSCManager:SC_HANDLE; lpLockStatus:LPQUERY_SERVICE_LOCK_STATUS; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'QueryServiceLockStatusW';
function RegisterServiceCtrlHandler(lpServiceName:LPCWSTR; lpHandlerProc:LPHANDLER_FUNCTION):SERVICE_STATUS_HANDLE; external 'advapi32' name 'RegisterServiceCtrlHandlerW';
function StartServiceCtrlDispatcher(lpServiceStartTable:LPSERVICE_TABLE_ENTRY):WINBOOL; external 'advapi32' name 'StartServiceCtrlDispatcherW';
function StartService(hService:SC_HANDLE; dwNumServiceArgs:DWORD; lpServiceArgVectors:LPPCWSTR):WINBOOL; external 'advapi32' name 'StartServiceW';
function DragQueryFile(_para1:HDROP; _para2:cardinal; _para3:LPCWSTR; _para4:cardinal):cardinal; external 'shell32' name 'DragQueryFileW';
function ExtractAssociatedIcon(_para1:HINST; _para2:LPCWSTR; _para3:LPWORD):HICON; external 'shell32' name 'ExtractAssociatedIconW';
function ExtractIcon(_para1:HINST; _para2:LPCWSTR; _para3:cardinal):HICON; external 'shell32' name 'ExtractIconW';
function FindExecutable(_para1:LPCWSTR; _para2:LPCWSTR; _para3:LPCWSTR):HINST; external 'shell32' name 'FindExecutableW';
function ShellAbout(_para1:HWND; _para2:LPCWSTR; _para3:LPCWSTR; _para4:HICON):longint; external 'shell32' name 'ShellAboutW';
function ShellExecute(_para1:HWND; _para2:LPCWSTR; _para3:LPCWSTR; _para4:LPCWSTR; _para5:LPCWSTR;_para6:longint):HINST; external 'shell32' name 'ShellExecuteW';
function Shell_NotifyIcon(dwMessage: DWORD; lpData: PNotifyIconDataA): BOOL; external 'shell32' name 'Shell_NotifyIconW';
function DdeCreateStringHandle(_para1:DWORD; _para2:LPCWSTR; _para3:longint):HSZ; external 'user32' name 'DdeCreateStringHandleW';
function DdeInitialize(_para1:LPDWORD; _para2:PFNCALLBACK; _para3:DWORD; _para4:DWORD):UINT; external 'user32' name 'DdeInitializeW';
function DdeQueryString(_para1:DWORD; _para2:HSZ; _para3:LPCWSTR; _para4:DWORD; _para5:longint):DWORD; external 'user32' name 'DdeQueryStringW';
function LogonUser(_para1:LPWSTR; _para2:LPWSTR; _para3:LPWSTR; _para4:DWORD; _para5:DWORD;_para6:PHANDLE):WINBOOL; external 'advapi32' name 'LogonUserW';
function CreateProcessAsUser(_para1:HANDLE; _para2:LPCWSTR; _para3:LPWSTR; _para4:LPSECURITY_ATTRIBUTES; _para5:LPSECURITY_ATTRIBUTES;_para6:WINBOOL; _para7:DWORD; _para8:LPVOID; _para9:LPCWSTR;
  _para10:LPSTARTUPINFO;_para11:LPPROCESS_INFORMATION):WINBOOL; external 'advapi32' name 'CreateProcessAsUserW';

function FindFirstFileEx(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external name 'FindFirstFileExW';
// winver>$0600 
function FindFirstFileTransacted(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedW';

{$endif read_interface}


{$ifdef read_implementation}

function CreateWindow(lpClassName:LPCWSTR; lpWindowName:LPCWSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
begin
  CreateWindow:=CreateWindowEx(0,lpClassName,lpWindowName,dwStyle,x,y,nWidth,nHeight,hWndParent,hMenu,hInstance,lpParam);
end;

function CreateDialog(hInstance:HINST; lpName:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
begin
  CreateDialog:=CreateDialogParam(hInstance,lpName,hWndParent,lpDialogFunc,0);
end;

function CreateDialogIndirect(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
begin
  CreateDialogIndirect:=CreateDialogIndirectParam(hInstance,lpTemplate,hWndParent,lpDialogFunc,0);
end;

function DialogBox(hInstance:HINST; lpTemplate:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
begin
  DialogBox:=DialogBoxParam(hInstance,lpTemplate,hWndParent,lpDialogFunc,0);
end;

function DialogBoxIndirect(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
begin
  DialogBoxIndirect:=DialogBoxIndirectParam(hInstance,lpTemplate,hWndParent,lpDialogFunc,0);
end;

{$endif read_implementation}

