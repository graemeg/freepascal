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

function GetBinaryTypeW(lpApplicationName:LPCWSTR; lpBinaryType:LPDWORD):WINBOOL; external 'kernel32' name 'GetBinaryTypeW';
function GetShortPathNameW(lpszLongPath:LPCWSTR; lpszShortPath:LPWSTR; cchBuffer:DWORD):DWORD; external 'kernel32' name 'GetShortPathNameW';
function GetEnvironmentStringsW:LPWSTR; external 'kernel32' name 'GetEnvironmentStringsW';function FreeEnvironmentStringsW(_para1:LPWSTR):WINBOOL; external 'kernel32' name 'FreeEnvironmentStringsW';
function FormatMessageW(dwFlags:DWORD; lpSource:LPCVOID; dwMessageId:DWORD; dwLanguageId:DWORD; lpBuffer:LPWSTR;nSize:DWORD; Arguments:va_list):DWORD; external 'kernel32' name 'FormatMessageW';
function CreateMailslotW(lpName:LPCWSTR; nMaxMessageSize:DWORD; lReadTimeout:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):HANDLE; external 'kernel32' name 'CreateMailslotW';
function lstrcmpW(lpString1:LPCWSTR; lpString2:LPCWSTR):longint; external 'kernel32' name 'lstrcmpW';
function lstrcmpiW(lpString1:LPCWSTR; lpString2:LPCWSTR):longint; external 'kernel32' name 'lstrcmpiW';
function lstrcpynW(lpString1:LPWSTR; lpString2:LPCWSTR; iMaxLength:longint):LPWSTR; external 'kernel32' name 'lstrcpynW';
function lstrcpyW(lpString1:LPWSTR; lpString2:LPCWSTR):LPWSTR; external 'kernel32' name 'lstrcpyW';
function lstrcatW(lpString1:LPWSTR; lpString2:LPCWSTR):LPWSTR; external 'kernel32' name 'lstrcatW';
function lstrlenW(lpString:LPCWSTR):longint; external 'kernel32' name 'lstrlenW';
function CreateMutexW(lpMutexAttributes:LPSECURITY_ATTRIBUTES; bInitialOwner:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'CreateMutexW';
function OpenMutexW(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'OpenMutexW';
function CreateEventW(lpEventAttributes:LPSECURITY_ATTRIBUTES; bManualReset:WINBOOL; bInitialState:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'CreateEventW';
function OpenEventW(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'OpenEventW';
function CreateSemaphoreW(lpSemaphoreAttributes:LPSECURITY_ATTRIBUTES; lInitialCount:LONG; lMaximumCount:LONG; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'CreateSemaphoreW';
function OpenSemaphoreW(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'OpenSemaphoreW';
function CreateFileMappingW(hFile:HANDLE; lpFileMappingAttributes:LPSECURITY_ATTRIBUTES; flProtect:DWORD; dwMaximumSizeHigh:DWORD; dwMaximumSizeLow:DWORD;lpName:LPCWSTR):HANDLE; external 'kernel32' name 'CreateFileMappingW';
function OpenFileMappingW(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCWSTR):HANDLE; external 'kernel32' name 'OpenFileMappingW';
function GetLogicalDriveStringsW(nBufferLength:DWORD; lpBuffer:LPWSTR):DWORD; external 'kernel32' name 'GetLogicalDriveStringsW';
function LoadLibraryW(lpLibFileName:LPCWSTR):HINST; external 'kernel32' name 'LoadLibraryW';
function LoadLibraryExW(lpLibFileName:LPCWSTR; hFile:HANDLE; dwFlags:DWORD):HINST; external 'kernel32' name 'LoadLibraryExW';
function GetModuleFileNameW(hModule:HINST; lpFilename:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetModuleFileNameW';
function GetModuleHandleW(lpModuleName:LPCWSTR):HMODULE; external 'kernel32' name 'GetModuleHandleW';
procedure FatalAppExitW(uAction:UINT; lpMessageText:LPCWSTR); external 'kernel32' name 'FatalAppExitW';
function GetCommandLineW:LPWSTR; external 'kernel32' name 'GetCommandLineW';function GetEnvironmentVariableW(lpName:LPCWSTR; lpBuffer:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetEnvironmentVariableW';
function SetEnvironmentVariableW(lpName:LPCWSTR; lpValue:LPCWSTR):WINBOOL; external 'kernel32' name 'SetEnvironmentVariableW';
function ExpandEnvironmentStringsW(lpSrc:LPCWSTR; lpDst:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'ExpandEnvironmentStringsW';
procedure OutputDebugStringW(lpOutputString:LPCWSTR); external 'kernel32' name 'OutputDebugStringW';
function FindResourceW(hModule:HINST; lpName:LPCWSTR; lpType:LPCWSTR):HRSRC; external 'kernel32' name 'FindResourceW';
function FindResourceExW(hModule:HINST; lpType:LPCWSTR; lpName:LPCWSTR; wLanguage:WORD):HRSRC; external 'kernel32' name 'FindResourceExW';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function EnumResourceTypesW(hModule:HMODULE; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesW';
function EnumResourceNamesW(hModule:HMODULE; lpType:LPCWSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesW';
function EnumResourceLanguagesW(hModule:HMODULE; lpType:LPCWSTR; lpName:LPCWSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesW';
=======
function EnumResourceTypesW(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesW';
function EnumResourceNamesW(hModule:HINST; lpType:LPCWSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesW';
function EnumResourceLanguagesW(hModule:HINST; lpType:LPCWSTR; lpName:LPCWSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesW';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypesW(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesW';
function EnumResourceNamesW(hModule:HINST; lpType:LPCWSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesW';
function EnumResourceLanguagesW(hModule:HINST; lpType:LPCWSTR; lpName:LPCWSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesW';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypesW(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesW';
function EnumResourceNamesW(hModule:HINST; lpType:LPCWSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesW';
function EnumResourceLanguagesW(hModule:HINST; lpType:LPCWSTR; lpName:LPCWSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesW';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypesW(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesW';
function EnumResourceNamesW(hModule:HINST; lpType:LPCWSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesW';
function EnumResourceLanguagesW(hModule:HINST; lpType:LPCWSTR; lpName:LPCWSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesW';
>>>>>>> origin/cpstrnew
function BeginUpdateResourceW(pFileName:LPCWSTR; bDeleteExistingResources:WINBOOL):HANDLE; external 'kernel32' name 'BeginUpdateResourceW';
function UpdateResourceW(hUpdate:HANDLE; lpType:LPCWSTR; lpName:LPCWSTR; wLanguage:WORD; lpData:LPVOID;cbData:DWORD):WINBOOL; external 'kernel32' name 'UpdateResourceW';
function EndUpdateResourceW(hUpdate:HANDLE; fDiscard:WINBOOL):WINBOOL; external 'kernel32' name 'EndUpdateResourceW';
function GlobalAddAtomW(lpString:LPCWSTR):ATOM; external 'kernel32' name 'GlobalAddAtomW';
function GlobalFindAtomW(lpString:LPCWSTR):ATOM; external 'kernel32' name 'GlobalFindAtomW';
function GlobalGetAtomNameW(nAtom:ATOM; lpBuffer:LPWSTR; nSize:longint):UINT; external 'kernel32' name 'GlobalGetAtomNameW';
function AddAtomW(lpString:LPCWSTR):ATOM; external 'kernel32' name 'AddAtomW';
function FindAtomW(lpString:LPCWSTR):ATOM; external 'kernel32' name 'FindAtomW';
function GetAtomNameW(nAtom:ATOM; lpBuffer:LPWSTR; nSize:longint):UINT; external 'kernel32' name 'GetAtomNameW';
function GetProfileIntW(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; nDefault:WINT):UINT; external 'kernel32' name 'GetProfileIntW';
function GetProfileStringW(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; lpDefault:LPCWSTR; lpReturnedString:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetProfileStringW';
function WriteProfileStringW(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; lpString:LPCWSTR):WINBOOL; external 'kernel32' name 'WriteProfileStringW';
function GetProfileSectionW(lpAppName:LPCWSTR; lpReturnedString:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetProfileSectionW';
function WriteProfileSectionW(lpAppName:LPCWSTR; lpString:LPCWSTR):WINBOOL; external 'kernel32' name 'WriteProfileSectionW';
function GetPrivateProfileIntW(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; nDefault:WINT; lpFileName:LPCWSTR):UINT; external 'kernel32' name 'GetPrivateProfileIntW';
function GetPrivateProfileStringW(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; lpDefault:LPCWSTR; lpReturnedString:LPWSTR; nSize:DWORD;lpFileName:LPCWSTR):DWORD; external 'kernel32' name 'GetPrivateProfileStringW';
function WritePrivateProfileStringW(lpAppName:LPCWSTR; lpKeyName:LPCWSTR; lpString:LPCWSTR; lpFileName:LPCWSTR):WINBOOL; external 'kernel32' name 'WritePrivateProfileStringW';
function GetPrivateProfileSectionW(lpAppName:LPCWSTR; lpReturnedString:LPWSTR; nSize:DWORD; lpFileName:LPCWSTR):DWORD; external 'kernel32' name 'GetPrivateProfileSectionW';
function WritePrivateProfileSectionW(lpAppName:LPCWSTR; lpString:LPCWSTR; lpFileName:LPCWSTR):WINBOOL; external 'kernel32' name 'WritePrivateProfileSectionW';
function GetDriveTypeW(lpRootPathName:LPCWSTR):UINT; external 'kernel32' name 'GetDriveTypeW';
function GetSystemDirectoryW(lpBuffer:LPWSTR; uSize:UINT):UINT; external 'kernel32' name 'GetSystemDirectoryW';
function GetTempPathW(nBufferLength:DWORD; lpBuffer:LPWSTR):DWORD; external 'kernel32' name 'GetTempPathW';
function GetTempFileNameW(lpPathName:LPCWSTR; lpPrefixString:LPCWSTR; uUnique:UINT; lpTempFileName:LPWSTR):UINT; external 'kernel32' name 'GetTempFileNameW';
function GetWindowsDirectoryW(lpBuffer:LPWSTR; uSize:UINT):UINT; external 'kernel32' name 'GetWindowsDirectoryW';
function SetCurrentDirectoryW(lpPathName:LPCWSTR):WINBOOL; external 'kernel32' name 'SetCurrentDirectoryW';
function GetCurrentDirectoryW(nBufferLength:DWORD; lpBuffer:LPWSTR):DWORD; external 'kernel32' name 'GetCurrentDirectoryW';
function GetDiskFreeSpaceW(lpRootPathName:LPCWSTR; lpSectorsPerCluster:LPDWORD; lpBytesPerSector:LPDWORD; lpNumberOfFreeClusters:LPDWORD; lpTotalNumberOfClusters:LPDWORD):WINBOOL; external 'kernel32' name 'GetDiskFreeSpaceW';
function CreateDirectoryW(lpPathName:LPCWSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):WINBOOL; external 'kernel32' name 'CreateDirectoryW';
function CreateDirectoryExW(lpTemplateDirectory:LPCWSTR; lpNewDirectory:LPCWSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):WINBOOL; external 'kernel32' name 'CreateDirectoryExW';
function RemoveDirectoryW(lpPathName:LPCWSTR):WINBOOL; external 'kernel32' name 'RemoveDirectoryW';
function GetFullPathNameW(lpFileName:LPCWSTR; nBufferLength:DWORD; lpBuffer:LPWSTR; var lpFilePart:LPWSTR):DWORD; external 'kernel32' name 'GetFullPathNameW';
function DefineDosDeviceW(dwFlags:DWORD; lpDeviceName:LPCWSTR; lpTargetPath:LPCWSTR):WINBOOL; external 'kernel32' name 'DefineDosDeviceW';
function QueryDosDeviceW(lpDeviceName:LPCWSTR; lpTargetPath:LPWSTR; ucchMax:DWORD):DWORD; external 'kernel32' name 'QueryDosDeviceW';
function CreateFileW(lpFileName:LPCWSTR; dwDesiredAccess:DWORD; dwShareMode:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES; dwCreationDisposition:DWORD;dwFlagsAndAttributes:DWORD; hTemplateFile:HANDLE):HANDLE; external 'kernel32' name 'CreateFileW';
function SetFileAttributesW(lpFileName:LPCWSTR; dwFileAttributes:DWORD):WINBOOL; external 'kernel32' name 'SetFileAttributesW';
function GetFileAttributesW(lpFileName:LPCWSTR):DWORD; external 'kernel32' name 'GetFileAttributesW';
function GetFileAttributesExW(lpFileName:LPCWSTR; fInfoLevelId:GET_FILEEX_INFO_LEVELS; lpFileInformation:LPVOID):BOOL; stdcall; external 'kernel32' name 'GetFileAttributesExW';
function GetCompressedFileSizeW(lpFileName:LPCWSTR; lpFileSizeHigh:LPDWORD):DWORD; external 'kernel32' name 'GetCompressedFileSizeW';
function DeleteFileW(lpFileName:LPCWSTR):WINBOOL; external 'kernel32' name 'DeleteFileW';
function SearchPathW(lpPath:LPCWSTR; lpFileName:LPCWSTR; lpExtension:LPCWSTR; nBufferLength:DWORD; lpBuffer:LPWSTR;lpFilePart:LPWSTR):DWORD; external 'kernel32' name 'SearchPathW';
function CopyFileW(lpExistingFileName:LPCWSTR; lpNewFileName:LPCWSTR; bFailIfExists:WINBOOL):WINBOOL; external 'kernel32' name 'CopyFileW';
function MoveFileW(lpExistingFileName:LPCWSTR; lpNewFileName:LPCWSTR):WINBOOL; external 'kernel32' name 'MoveFileW';
function MoveFileExW(lpExistingFileName:LPCWSTR; lpNewFileName:LPCWSTR; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'MoveFileExW';
function CreateNamedPipeW(lpName:LPCWSTR; dwOpenMode:DWORD; dwPipeMode:DWORD; nMaxInstances:DWORD; nOutBufferSize:DWORD;nInBufferSize:DWORD; nDefaultTimeOut:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):HANDLE;
  external 'kernel32' name 'CreateNamedPipeW';
function GetNamedPipeHandleStateW(hNamedPipe:HANDLE; lpState:LPDWORD; lpCurInstances:LPDWORD; lpMaxCollectionCount:LPDWORD; lpCollectDataTimeout:LPDWORD;lpUserName:LPWSTR; nMaxUserNameSize:DWORD):WINBOOL;
  external 'kernel32' name 'GetNamedPipeHandleStateW';
function CallNamedPipeW(lpNamedPipeName:LPCWSTR; lpInBuffer:LPVOID; nInBufferSize:DWORD; lpOutBuffer:LPVOID; nOutBufferSize:DWORD;lpBytesRead:LPDWORD; nTimeOut:DWORD):WINBOOL; external 'kernel32' name 'CallNamedPipeW';
function WaitNamedPipeW(lpNamedPipeName:LPCWSTR; nTimeOut:DWORD):WINBOOL; external 'kernel32' name 'WaitNamedPipeW';
function SetVolumeLabelW(lpRootPathName:LPCWSTR; lpVolumeName:LPCWSTR):WINBOOL; external 'kernel32' name 'SetVolumeLabelW';
function GetVolumeInformationW(lpRootPathName:LPCWSTR; lpVolumeNameBuffer:LPWSTR; nVolumeNameSize:DWORD; lpVolumeSerialNumber:LPDWORD; lpMaximumComponentLength:LPDWORD;lpFileSystemFlags:LPDWORD; lpFileSystemNameBuffer:LPWSTR;
  nFileSystemNameSize:DWORD):WINBOOL; external 'kernel32' name 'GetVolumeInformationW';
function ClearEventLogW(hEventLog:HANDLE; lpBackupFileName:LPCWSTR):WINBOOL; external 'advapi32' name 'ClearEventLogW';
function BackupEventLogW(hEventLog:HANDLE; lpBackupFileName:LPCWSTR):WINBOOL; external 'advapi32' name 'BackupEventLogW';
function OpenEventLogW(lpUNCServerName:LPCWSTR; lpSourceName:LPCWSTR):HANDLE; external 'advapi32' name 'OpenEventLogW';
function RegisterEventSourceW(lpUNCServerName:LPCWSTR; lpSourceName:LPCWSTR):HANDLE; external 'advapi32' name 'RegisterEventSourceW';
function OpenBackupEventLogW(lpUNCServerName:LPCWSTR; lpFileName:LPCWSTR):HANDLE; external 'advapi32' name 'OpenBackupEventLogW';
function ReadEventLogW(hEventLog:HANDLE; dwReadFlags:DWORD; dwRecordOffset:DWORD; lpBuffer:LPVOID; nNumberOfBytesToRead:DWORD;pnBytesRead:LPDWORD; pnMinNumberOfBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'ReadEventLogW';
function ReportEventW(hEventLog:HANDLE; wType:WORD; wCategory:WORD; dwEventID:DWORD; lpUserSid:PSID;wNumStrings:WORD; dwDataSize:DWORD; lpStrings:LPPCWSTR; lpRawData:LPVOID):WINBOOL; external 'advapi32' name 'ReportEventW';
function AccessCheckAndAuditAlarmW(SubsystemName:LPCWSTR; HandleId:LPVOID; ObjectTypeName:LPWSTR; ObjectName:LPWSTR; SecurityDescriptor:PSECURITY_DESCRIPTOR;DesiredAccess:DWORD; GenericMapping:PGENERIC_MAPPING; ObjectCreation:WINBOOL;
  GrantedAccess:LPDWORD; AccessStatus:LPBOOL;pfGenerateOnClose:LPBOOL):WINBOOL; external 'advapi32' name 'AccessCheckAndAuditAlarmW';
function ObjectOpenAuditAlarmW(SubsystemName:LPCWSTR; HandleId:LPVOID; ObjectTypeName:LPWSTR; ObjectName:LPWSTR; pSecurityDescriptor:PSECURITY_DESCRIPTOR;ClientToken:HANDLE; DesiredAccess:DWORD; GrantedAccess:DWORD;
  Privileges:PPRIVILEGE_SET; ObjectCreation:WINBOOL;AccessGranted:WINBOOL; GenerateOnClose:LPBOOL):WINBOOL; external 'advapi32' name 'ObjectOpenAuditAlarmW';
function ObjectPrivilegeAuditAlarmW(SubsystemName:LPCWSTR; HandleId:LPVOID; ClientToken:HANDLE; DesiredAccess:DWORD; Privileges:PPRIVILEGE_SET;AccessGranted:WINBOOL):WINBOOL; external 'advapi32' name 'ObjectPrivilegeAuditAlarmW';
function ObjectCloseAuditAlarmW(SubsystemName:LPCWSTR; HandleId:LPVOID; GenerateOnClose:WINBOOL):WINBOOL; external 'advapi32' name 'ObjectCloseAuditAlarmW';
function PrivilegedServiceAuditAlarmW(SubsystemName:LPCWSTR; ServiceName:LPCWSTR; ClientToken:HANDLE; Privileges:PPRIVILEGE_SET; AccessGranted:WINBOOL):WINBOOL; external 'advapi32' name 'PrivilegedServiceAuditAlarmW';
function SetFileSecurityW(lpFileName:LPCWSTR; SecurityInformation:SECURITY_INFORMATION; pSecurityDescriptor:PSECURITY_DESCRIPTOR):WINBOOL; external 'advapi32' name 'SetFileSecurityW';
function GetFileSecurityW(lpFileName:LPCWSTR; RequestedInformation:SECURITY_INFORMATION; pSecurityDescriptor:PSECURITY_DESCRIPTOR; nLength:DWORD; lpnLengthNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'GetFileSecurityW';
function FindFirstChangeNotificationW(lpPathName:LPCWSTR; bWatchSubtree:WINBOOL; dwNotifyFilter:DWORD):HANDLE; external 'kernel32' name 'FindFirstChangeNotificationW';
function IsBadStringPtrW(lpsz:LPCWSTR; ucchMax:UINT):WINBOOL; external 'kernel32' name 'IsBadStringPtrW';
function LookupAccountSidW(lpSystemName:LPCWSTR; Sid:PSID; Name:LPWSTR; cbName:LPDWORD; ReferencedDomainName:LPWSTR;cbReferencedDomainName:LPDWORD; peUse:PSID_NAME_USE):WINBOOL; external 'advapi32' name 'LookupAccountSidW';
function LookupAccountNameW(lpSystemName:LPCWSTR; lpAccountName:LPCWSTR; Sid:PSID; cbSid:LPDWORD; ReferencedDomainName:LPWSTR;cbReferencedDomainName:LPDWORD; peUse:PSID_NAME_USE):WINBOOL; external 'advapi32' name 'LookupAccountNameW';
function LookupPrivilegeValueW(lpSystemName:LPCWSTR; lpName:LPCWSTR; lpLuid:PLUID):WINBOOL; external 'advapi32' name 'LookupPrivilegeValueW';
function LookupPrivilegeNameW(lpSystemName:LPCWSTR; lpLuid:PLUID; lpName:LPWSTR; cbName:LPDWORD):WINBOOL; external 'advapi32' name 'LookupPrivilegeNameW';
function LookupPrivilegeDisplayNameW(lpSystemName:LPCWSTR; lpName:LPCWSTR; lpDisplayName:LPWSTR; cbDisplayName:LPDWORD; lpLanguageId:LPDWORD):WINBOOL; external 'advapi32' name 'LookupPrivilegeDisplayNameW';
function BuildCommDCBW(lpDef:LPCWSTR; lpDCB:LPDCB):WINBOOL; external 'kernel32' name 'BuildCommDCBW';
function BuildCommDCBAndTimeoutsW(lpDef:LPCWSTR; lpDCB:LPDCB; lpCommTimeouts:LPCOMMTIMEOUTS):WINBOOL; external 'kernel32' name 'BuildCommDCBAndTimeoutsW';
function CommConfigDialogW(lpszName:LPCWSTR; hWnd:HWND; lpCC:LPCOMMCONFIG):WINBOOL; external 'kernel32' name 'CommConfigDialogW';
function GetDefaultCommConfigW(lpszName:LPCWSTR; lpCC:LPCOMMCONFIG; lpdwSize:LPDWORD):WINBOOL; external 'kernel32' name 'GetDefaultCommConfigW';
function SetDefaultCommConfigW(lpszName:LPCWSTR; lpCC:LPCOMMCONFIG; dwSize:DWORD):WINBOOL; external 'kernel32' name 'SetDefaultCommConfigW';
function GetComputerNameW(lpBuffer:LPWSTR; nSize:LPDWORD):WINBOOL; external 'kernel32' name 'GetComputerNameW';
function SetComputerNameW(lpComputerName:LPCWSTR):WINBOOL; external 'kernel32' name 'SetComputerNameW';
function GetUserNameW(lpBuffer:LPWSTR; nSize:LPDWORD):WINBOOL; external 'advapi32' name 'GetUserNameW';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function wvsprintfW(_para1:LPWSTR; _para2:LPCWSTR; arglist:va_list):longint; stdcall; external 'user32' name 'wvsprintfW';
=======
function wvsprintfW(_para1:LPWSTR; _para2:LPCWSTR; arglist:va_list):longint; cdecl; external 'user32' name 'wvsprintfW';
>>>>>>> graemeg/cpstrnew
=======
function wvsprintfW(_para1:LPWSTR; _para2:LPCWSTR; arglist:va_list):longint; cdecl; external 'user32' name 'wvsprintfW';
>>>>>>> graemeg/cpstrnew
=======
function wvsprintfW(_para1:LPWSTR; _para2:LPCWSTR; arglist:va_list):longint; cdecl; external 'user32' name 'wvsprintfW';
>>>>>>> graemeg/cpstrnew
=======
function wvsprintfW(_para1:LPWSTR; _para2:LPCWSTR; arglist:va_list):longint; cdecl; external 'user32' name 'wvsprintfW';
>>>>>>> origin/cpstrnew
function wsprintfW(_para1:LPWSTR; _para2:LPCWSTR; const args:array of const):longint;cdecl; external 'user32' name 'wsprintfW';
function wsprintfW(_para1:LPWSTR; _para2:LPCWSTR):longint; cdecl; external 'user32' name 'wsprintfW';
function LoadKeyboardLayoutW(pwszKLID:LPCWSTR; Flags:UINT):HKL; external 'user32' name 'LoadKeyboardLayoutW';
function GetKeyboardLayoutNameW(pwszKLID:LPWSTR):WINBOOL; external 'user32' name 'GetKeyboardLayoutNameW';
function CreateDesktopW(lpszDesktop:LPWSTR; lpszDevice:LPWSTR;pDevmodew:LPDEVMODEw; dwFlags:DWORD; dwDesiredAccess:DWORD;lpsa:LPSECURITY_ATTRIBUTES):HDESK; external 'user32' name 'CreateDesktopW';
function OpenDesktopW(lpszDesktop:LPWSTR; dwFlags:DWORD; fInherit:WINBOOL; dwDesiredAccess:DWORD):HDESK; external 'user32' name 'OpenDesktopW';
function EnumDesktopsW(hwinsta:HWINSTA; lpEnumFunc:DESKTOPENUMPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumDesktopsW';
function CreateWindowStationW(lpwinsta:LPWSTR; dwReserved:DWORD; dwDesiredAccess:DWORD; lpsa:LPSECURITY_ATTRIBUTES):HWINSTA; external 'user32' name 'CreateWindowStationW';
function OpenWindowStationW(lpszWinSta:LPWSTR; fInherit:WINBOOL; dwDesiredAccess:DWORD):HWINSTA; external 'user32' name 'OpenWindowStationW';
function EnumWindowStationsW(lpEnumFunc:ENUMWINDOWSTATIONPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumWindowStationsW';
function GetUserObjectInformationW(hObj:HANDLE; nIndex:longint; pvInfo:PVOID; nLength:DWORD; lpnLengthNeeded:LPDWORD):WINBOOL; external 'user32' name 'GetUserObjectInformationW';
function SetUserObjectInformationW(hObj:HANDLE; nIndex:longint; pvInfo:PVOID; nLength:DWORD):WINBOOL; external 'user32' name 'SetUserObjectInformationW';
function RegisterWindowMessageW(lpString:LPCWSTR):UINT; external 'user32' name 'RegisterWindowMessageW';
function GetMessageW(lpMsg:LPMSG; hWnd:HWND; wMsgFilterMin:UINT; wMsgFilterMax:UINT):WINBOOL; external 'user32' name 'GetMessageW';
function DispatchMessageW(lpMsg:LPMSG):LONG; external 'user32' name 'DispatchMessageW';
function PeekMessageW(lpMsg:LPMSG; hWnd:HWND; wMsgFilterMin:UINT; wMsgFilterMax:UINT; wRemoveMsg:UINT):WINBOOL; external 'user32' name 'PeekMessageW';
function SendMessageW(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'SendMessageW';
function SendMessageTimeoutW(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM; fuFlags:UINT;uTimeout:UINT; lpdwResult:PDWORD_PTR):LRESULT; external 'user32' name 'SendMessageTimeoutW';
function SendNotifyMessageW(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'SendNotifyMessageW';
function SendMessageCallbackW(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM; lpResultCallBack:SENDASYNCPROC;dwData:ULONG_PTR):WINBOOL; external 'user32' name 'SendMessageCallbackW';
function PostMessageW(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'PostMessageW';
function PostThreadMessageW(idThread:DWORD; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'PostThreadMessageW';
function DefWindowProcW(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefWindowProcW';
function CallWindowProcW(lpPrevWndFunc:WNDPROC; hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'CallWindowProcW';
function RegisterClassW(lpWndClass:LPWNDCLASSW):ATOM; external 'user32' name 'RegisterClassW';
function UnregisterClassW(lpClassName:LPCWSTR; hInstance:HINST):WINBOOL; external 'user32' name 'UnregisterClassW';
function GetClassInfoW(hInstance:HINST; lpClassName:LPCWSTR; lpWndClass:LPWNDCLASS):WINBOOL; external 'user32' name 'GetClassInfoW';
function RegisterClassExW(_para1:LPWNDCLASSEXW):ATOM; external 'user32' name 'RegisterClassExW';
function GetClassInfoExW(_para1:HINST; _para2:LPCWSTR; _para3:LPWNDCLASSEX):WINBOOL; external 'user32' name 'GetClassInfoExW';
function CreateWindowExW(dwExStyle:DWORD; lpClassName:LPCWSTR; lpWindowName:LPCWSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
  external 'user32' name 'CreateWindowExW';
function CreateDialogParamW(hInstance:HINST; lpTemplateName:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):HWND; external 'user32' name 'CreateDialogParamW';
function CreateDialogIndirectParamW(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):HWND; external 'user32' name 'CreateDialogIndirectParamW';
function DialogBoxParamW(hInstance:HINST; lpTemplateName:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):longint; external 'user32' name 'DialogBoxParamW';
function DialogBoxIndirectParamW(hInstance:HINST; hDialogTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):longint; external 'user32' name 'DialogBoxIndirectParamW';
function SetDlgItemTextW(hDlg:HWND; nIDDlgItem:longint; lpString:LPCWSTR):WINBOOL; external 'user32' name 'SetDlgItemTextW';
function GetDlgItemTextW(hDlg:HWND; nIDDlgItem:longint; lpString:LPWSTR; nMaxCount:longint):UINT; external 'user32' name 'GetDlgItemTextW';
function SendDlgItemMessageW(hDlg:HWND; nIDDlgItem:longint; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LONG; external 'user32' name 'SendDlgItemMessageW';
function DefDlgProcW(hDlg:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefDlgProcW';
function CallMsgFilterW(lpMsg:LPMSG; nCode:longint):WINBOOL; external 'user32' name 'CallMsgFilterW';
function RegisterClipboardFormatW(lpszFormat:LPCWSTR):UINT; external 'user32' name 'RegisterClipboardFormatW';
function GetClipboardFormatNameW(format:UINT; lpszFormatName:LPWSTR; cchMaxCount:longint):longint; external 'user32' name 'GetClipboardFormatNameW';
function CharToOemW(lpszSrc:LPCWSTR; lpszDst:LPSTR):WINBOOL; external 'user32' name 'CharToOemW';
function OemToCharW(lpszSrc:LPCSTR; lpszDst:LPWSTR):WINBOOL; external 'user32' name 'OemToCharW';
function CharToOemBuffW(lpszSrc:LPCWSTR; lpszDst:LPSTR; cchDstLength:DWORD):WINBOOL; external 'user32' name 'CharToOemBuffW';
function OemToCharBuffW(lpszSrc:LPCSTR; lpszDst:LPWSTR; cchDstLength:DWORD):WINBOOL; external 'user32' name 'OemToCharBuffW';
function CharUpperW(lpsz:LPWSTR):LPWSTR; external 'user32' name 'CharUpperW';
function CharUpperBuffW(lpsz:LPWSTR; cchLength:DWORD):DWORD; external 'user32' name 'CharUpperBuffW';
function CharLowerW(lpsz:LPWSTR):LPWSTR; external 'user32' name 'CharLowerW';
function CharLowerBuffW(lpsz:LPWSTR; cchLength:DWORD):DWORD; external 'user32' name 'CharLowerBuffW';
function CharNextW(lpsz:LPCWSTR):LPWSTR; external 'user32' name 'CharNextW';
function CharPrevW(lpszStart:LPCWSTR; lpszCurrent:LPCWSTR):LPWSTR; external 'user32' name 'CharPrevW';
function IsCharAlphaW(ch:WCHAR):WINBOOL; external 'user32' name 'IsCharAlphaW';
function IsCharAlphaNumericW(ch:WCHAR):WINBOOL; external 'user32' name 'IsCharAlphaNumericW';
function IsCharUpperW(ch:WCHAR):WINBOOL; external 'user32' name 'IsCharUpperW';
function IsCharLowerW(ch:WCHAR):WINBOOL; external 'user32' name 'IsCharLowerW';
function GetKeyNameTextW(lParam:LONG; lpString:LPWSTR; nSize:longint):longint; external 'user32' name 'GetKeyNameTextW';
function VkKeyScanW(ch:WCHAR):SHORT; external 'user32' name 'VkKeyScanW';
function VkKeyScanExW(ch:WCHAR; dwhkl:HKL):SHORT; external 'user32' name 'VkKeyScanExW';
function MapVirtualKeyW(uCode:UINT; uMapType:UINT):UINT; external 'user32' name 'MapVirtualKeyW';
function MapVirtualKeyExW(uCode:UINT; uMapType:UINT; dwhkl:HKL):UINT; external 'user32' name 'MapVirtualKeyExW';
function LoadAcceleratorsW(hInstance:HINST; lpTableName:LPCWSTR):HACCEL; external 'user32' name 'LoadAcceleratorsW';
function CreateAcceleratorTableW(_para1:LPACCEL; _para2:longint):HACCEL; external 'user32' name 'CreateAcceleratorTableW';
function CopyAcceleratorTableW(hAccelSrc:HACCEL; lpAccelDst:LPACCEL; cAccelEntries:longint):longint; external 'user32' name 'CopyAcceleratorTableW';
function TranslateAcceleratorW(hWnd:HWND; hAccTable:HACCEL; lpMsg:LPMSG):longint; external 'user32' name 'TranslateAcceleratorW';
function LoadMenuW(hInstance:HINST; lpMenuName:LPCWSTR):HMENU; external 'user32' name 'LoadMenuW';
function LoadMenuIndirectW(lpMenuTemplate:LPMENUTEMPLATE):HMENU; external 'user32' name 'LoadMenuIndirectW';
function ChangeMenuW(hMenu:HMENU; cmd:UINT; lpszNewItem:LPCWSTR; cmdInsert:UINT; flags:UINT):WINBOOL; external 'user32' name 'ChangeMenuW';
function GetMenuStringW(hMenu:HMENU; uIDItem:UINT; lpString:LPWSTR; nMaxCount:longint; uFlag:UINT):longint; external 'user32' name 'GetMenuStringW';
function InsertMenuW(hMenu:HMENU; uPosition:UINT; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCWSTR):WINBOOL; external 'user32' name 'InsertMenuW';
function AppendMenuW(hMenu:HMENU; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCWSTR):WINBOOL; external 'user32' name 'AppendMenuW';
function ModifyMenuW(hMnu:HMENU; uPosition:UINT; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCWSTR):WINBOOL; external 'user32' name 'ModifyMenuW';
function InsertMenuItemW(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPCMENUITEMINFO):WINBOOL; external 'user32' name 'InsertMenuItemW';
function GetMenuItemInfoW(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPMENUITEMINFO):WINBOOL; external 'user32' name 'GetMenuItemInfoW';
function SetMenuItemInfoW(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPCMENUITEMINFO):WINBOOL; external 'user32' name 'SetMenuItemInfoW';
function DrawTextW(hDC:HDC; lpString:LPCWSTR; nCount:longint; lpRect:LPRECT; uFormat:UINT):longint; external 'user32' name 'DrawTextW';
function DrawTextExW(_para1:HDC; _para2:LPWSTR; _para3:longint; _para4:LPRECT; _para5:UINT;_para6:LPDRAWTEXTPARAMS):longint; external 'user32' name 'DrawTextExW';
function GrayStringW(hDC:HDC; hBrush:HBRUSH; lpOutputFunc:GRAYSTRINGPROC; lpData:LPARAM; nCount:longint;X:longint; Y:longint; nWidth:longint; nHeight:longint):WINBOOL; external 'user32' name 'GrayStringW';
function DrawStateW(_para1:HDC; _para2:HBRUSH; _para3:DRAWSTATEPROC; _para4:LPARAM; _para5:WPARAM;_para6:longint; _para7:longint; _para8:longint; _para9:longint; _para10:UINT):WINBOOL; external 'user32' name 'DrawStateW';
function TabbedTextOutW(hDC:HDC; X:longint; Y:longint; lpString:LPCWSTR; nCount:longint;nTabPositions:longint; lpnTabStopPositions:LPINT; nTabOrigin:longint):LONG; external 'user32' name 'TabbedTextOutW';
function GetTabbedTextExtentW(hDC:HDC; lpString:LPCWSTR; nCount:longint; nTabPositions:longint; lpnTabStopPositions:LPINT):DWORD; external 'user32' name 'GetTabbedTextExtentW';
function SetPropW(hWnd:HWND; lpString:LPCWSTR; hData:HANDLE):WINBOOL; external 'user32' name 'SetPropW';
function GetPropW(hWnd:HWND; lpString:LPCWSTR):HANDLE; external 'user32' name 'GetPropW';
function RemovePropW(hWnd:HWND; lpString:LPCWSTR):HANDLE; external 'user32' name 'RemovePropW';
function EnumPropsExW(hWnd:HWND; lpEnumFunc:PROPENUMPROCEX; lParam:LPARAM):longint; external 'user32' name 'EnumPropsExW';
function EnumPropsW(hWnd:HWND; lpEnumFunc:PROPENUMPROC):longint; external 'user32' name 'EnumPropsW';
function SetWindowTextW(hWnd:HWND; lpString:LPCWSTR):WINBOOL; external 'user32' name 'SetWindowTextW';
function GetWindowTextW(hWnd:HWND; lpString:LPWSTR; nMaxCount:longint):longint; external 'user32' name 'GetWindowTextW';
function GetWindowTextLengthW(hWnd:HWND):longint; external 'user32' name 'GetWindowTextLengthW';
function MessageBoxW(hWnd:HWND; lpText:LPCWSTR; lpCaption:LPCWSTR; uType:UINT):longint; external 'user32' name 'MessageBoxW';
function MessageBoxExW(hWnd:HWND; lpText:LPCWSTR; lpCaption:LPCWSTR; uType:UINT; wLanguageId:WORD):longint; external 'user32' name 'MessageBoxExW';
function MessageBoxIndirectW(_para1:LPMSGBOXPARAMS):longint; external 'user32' name 'MessageBoxIndirectW';
function GetWindowLongW(hWnd:HWND; nIndex:longint):LONG; external 'user32' name 'GetWindowLongW';
function SetWindowLongW(hWnd:HWND; nIndex:longint; dwNewLong:LONG):LONG; external 'user32' name 'SetWindowLongW';
function GetClassLongW(hWnd:HWND; nIndex:longint):DWORD; external 'user32' name 'GetClassLongW';
function SetClassLongW(hWnd:HWND; nIndex:longint; dwNewLong:LONG):DWORD; external 'user32' name 'SetClassLongW';
{$ifdef cpu64}
function GetWindowLongPtrW(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetWindowLongPtrW';
function SetWindowLongPtrW(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetWindowLongPtrW';
function GetClassLongPtrW(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetClassLongPtrW';
function SetClassLongPtrW(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetClassLongPtrW';
{$else}
function GetWindowLongPtrW(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetWindowLongW';
function SetWindowLongPtrW(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetWindowLongW';
function GetClassLongPtrW(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetClassLongW';
function SetClassLongPtrW(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetClassLongW';
{$endif}
function FindWindowW(lpClassName:LPCWSTR; lpWindowName:LPCWSTR):HWND; external 'user32' name 'FindWindowW';
function FindWindowExW(_para1:HWND; _para2:HWND; _para3:LPCWSTR; _para4:LPCWSTR):HWND; external 'user32' name 'FindWindowExW';
function GetClassNameW(hWnd:HWND; lpClassName:LPWSTR; nMaxCount:longint):longint; external 'user32' name 'GetClassNameW';
function SetWindowsHookExW(idHook:longint; lpfn:HOOKPROC; hmod:HINST; dwThreadId:DWORD):HHOOK; external 'user32' name 'SetWindowsHookExW';
function LoadBitmapW(hInstance:HINST; lpBitmapName:LPCWSTR):HBITMAP; external 'user32' name 'LoadBitmapW';
function LoadCursorW(hInstance:HINST; lpCursorName:LPCWSTR):HCURSOR; external 'user32' name 'LoadCursorW';
function LoadCursorFromFileW(lpFileName:LPCWSTR):HCURSOR; external 'user32' name 'LoadCursorFromFileW';
function LoadIconW(hInstance:HINST; lpIconName:LPCWSTR):HICON; external 'user32' name 'LoadIconW';
function LoadImageW(_para1:HINST; _para2:LPCWSTR; _para3:UINT; _para4:longint; _para5:longint;_para6:UINT):HANDLE; external 'user32' name 'LoadImageW';
function LoadStringW(hInstance:HINST; uID:UINT; lpBuffer:LPWSTR; nBufferMax:longint):longint; external 'user32' name 'LoadStringW';
function IsDialogMessageW(hDlg:HWND; lpMsg:LPMSG):WINBOOL; external 'user32' name 'IsDialogMessageW';
function DlgDirListW(hDlg:HWND; lpPathSpec:LPWSTR; nIDListBox:longint; nIDStaticPath:longint; uFileType:UINT):longint; external 'user32' name 'DlgDirListW';
function DlgDirSelectExW(hDlg:HWND; lpString:LPWSTR; nCount:longint; nIDListBox:longint):WINBOOL; external 'user32' name 'DlgDirSelectExW';
function DlgDirListComboBoxW(hDlg:HWND; lpPathSpec:LPWSTR; nIDComboBox:longint; nIDStaticPath:longint; uFiletype:UINT):longint; external 'user32' name 'DlgDirListComboBoxW';
function DlgDirSelectComboBoxExW(hDlg:HWND; lpString:LPWSTR; nCount:longint; nIDComboBox:longint):WINBOOL; external 'user32' name 'DlgDirSelectComboBoxExW';
function DefFrameProcW(hWnd:HWND; hWndMDIClient:HWND; uMsg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefFrameProcW';
function DefMDIChildProcW(hWnd:HWND; uMsg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefMDIChildProcW';
function CreateMDIWindowW(lpClassName:LPWSTR; lpWindowName:LPWSTR; dwStyle:DWORD; X:longint; Y:longint;nWidth:longint; nHeight:longint; hWndParent:HWND; hInstance:HINST; lParam:LPARAM):HWND; external 'user32' name 'CreateMDIWindowW';
function WinHelpW(hWndMain:HWND; lpszHelp:LPCWSTR; uCommand:UINT; dwData:DWORD):WINBOOL; external 'user32' name 'WinHelpW';
function ChangeDisplaySettingsW(lpDevMode:LPDEVMODEW; dwFlags:DWORD):LONG; external 'user32' name 'ChangeDisplaySettingsW';
function EnumDisplaySettingsW(lpszDeviceName:LPCWSTR; iModeNum:DWORD;lpDevMode:LPDEVMODEW):WINBOOL; external 'user32' name 'EnumDisplaySettingsW';
function SystemParametersInfoW(uiAction:UINT; uiParam:UINT; pvParam:PVOID; fWinIni:UINT):WINBOOL; external 'user32' name 'SystemParametersInfoW';
function AddFontResourceW(_para1:LPCWSTR):longint; external 'gdi32' name 'AddFontResourceW';
function CopyMetaFileW(_para1:HMETAFILE; _para2:LPCWSTR):HMETAFILE; external 'gdi32' name 'CopyMetaFileW';
function CreateFontIndirectW(_para1:PLOGFONTW):HFONT; external 'gdi32' name 'CreateFontIndirectW';
function CreateFontIndirectW(var _para1:LOGFONTW):HFONT; external 'gdi32' name 'CreateFontIndirectW';
function CreateFontW(_para1:longint; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:DWORD; _para7:DWORD; _para8:DWORD; _para9:DWORD; _para10:DWORD;_para11:DWORD; _para12:DWORD; _para13:DWORD; _para14:LPCWSTR):HFONT;
  external 'gdi32' name 'CreateFontW';
function CreateICW(_para1:LPCWSTR; _para2:LPCWSTR; _para3:LPCWSTR; _para4:LPDEVMODEw):HDC; external 'gdi32' name 'CreateICW';
function CreateMetaFileW(_para1:LPCWSTR):HDC; external 'gdi32' name 'CreateMetaFileW';
function CreateScalableFontResourceW(_para1:DWORD; _para2:LPCWSTR; _para3:LPCWSTR; _para4:LPCWSTR):WINBOOL; external 'gdi32' name 'CreateScalableFontResourceW';
function EnumFontFamiliesExW(_para1:HDC; _para2:LPLOGFONT; _para3:FONTENUMEXPROC; _para4:LPARAM; _para5:DWORD):longint; external 'gdi32' name 'EnumFontFamiliesExW';
function EnumFontFamiliesW(_para1:HDC; _para2:LPCWSTR; _para3:FONTENUMPROC; _para4:LPARAM):longint; external 'gdi32' name 'EnumFontFamiliesW';
function EnumFontsW(_para1:HDC; _para2:LPCWSTR; _para3:ENUMFONTSPROC; _para4:LPARAM):longint; external 'gdi32' name 'EnumFontsW';
function EnumFontsW(_para1:HDC; _para2:LPCWSTR; _para3:ENUMFONTSPROC; _para4:pointer):longint; external 'gdi32' name 'EnumFontsW';
function GetCharWidthW(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPINT):WINBOOL; external 'gdi32' name 'GetCharWidthW';
function GetCharWidth32W(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPINT):WINBOOL; external 'gdi32' name 'GetCharWidth32W';
function GetCharWidthFloatW(_para1:HDC; _para2:UINT; _para3:UINT; _para4:PSingle):WINBOOL; external 'gdi32' name 'GetCharWidthFloatW';
function GetCharABCWidthsW(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPABC):WINBOOL; external 'gdi32' name 'GetCharABCWidthsW';
function GetCharABCWidthsFloatW(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPABCFLOAT):WINBOOL; external 'gdi32' name 'GetCharABCWidthsFloatW';
function GetGlyphOutlineW(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPGLYPHMETRICS; _para5:DWORD;_para6:LPVOID; _para7:PMAT2):DWORD; external 'gdi32' name 'GetGlyphOutlineW';
function GetMetaFileW(_para1:LPCWSTR):HMETAFILE; external 'gdi32' name 'GetMetaFileW';
function GetOutlineTextMetricsW(_para1:HDC; _para2:UINT; _para3:LPOUTLINETEXTMETRIC):UINT; external 'gdi32' name 'GetOutlineTextMetricsW';
function GetTextExtentPointW(_para1:HDC; _para2:LPCWSTR; _para3:longint; _para4:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentPointW';
function GetTextExtentPoint32W(_para1:HDC; _para2:LPCWSTR; _para3:longint; _para4:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentPoint32W';
function GetTextExtentExPointW(_para1:HDC; _para2:LPCWSTR; _para3:longint; _para4:longint; _para5:LPINT;_para6:LPINT; _para7:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentExPointW';
function GetCharacterPlacementW(_para1:HDC; _para2:LPCWSTR; _para3:longint; _para4:longint; _para5:LPGCP_RESULTS;_para6:DWORD):DWORD; external 'gdi32' name 'GetCharacterPlacementW';
function ResetDCW(_para1:HDC; _para2:LPDEVMODEW):HDC; external 'gdi32' name 'ResetDCW';
function RemoveFontResourceW(_para1:LPCWSTR):WINBOOL; external 'gdi32' name 'RemoveFontResourceW';
function CopyEnhMetaFileW(_para1:HENHMETAFILE; _para2:LPCWSTR):HENHMETAFILE; external 'gdi32' name 'CopyEnhMetaFileW';
function CreateEnhMetaFileW(_para1:HDC; _para2:LPCWSTR; _para3:LPRECT; _para4:LPCWSTR):HDC; external 'gdi32' name 'CreateEnhMetaFileW';
function GetEnhMetaFileW(_para1:LPCWSTR):HENHMETAFILE; external 'gdi32' name 'GetEnhMetaFileW';
function GetEnhMetaFileDescriptionW(_para1:HENHMETAFILE; _para2:UINT; _para3:LPWSTR):UINT; external 'gdi32' name 'GetEnhMetaFileDescriptionW';
function GetTextMetricsW(_para1:HDC; _para2:LPTEXTMETRIC):WINBOOL; external 'gdi32' name 'GetTextMetricsW';
function StartDocW(_para1:HDC; _para2:PDOCINFO):longint; external 'gdi32' name 'StartDocW';
function GetObjectW(_para1:HGDIOBJ; _para2:longint; _para3:LPVOID):longint; external 'gdi32' name 'GetObjectW';
function TextOutW(_para1:HDC; _para2:longint; _para3:longint; _para4:LPCWSTR; _para5:longint):WINBOOL; external 'gdi32' name 'TextOutW';
function ExtTextOutW(_para1:HDC; _para2:longint; _para3:longint; _para4:UINT; _para5:LPRECT;_para6:LPCWSTR; _para7:UINT; _para8:LPINT):WINBOOL; external 'gdi32' name 'ExtTextOutW';
function PolyTextOutW(_para1:HDC; _para2:PPOLYTEXT; _para3:longint):WINBOOL; external 'gdi32' name 'PolyTextOutW';
function GetTextFaceW(_para1:HDC; _para2:longint; _para3:LPWSTR):longint; external 'gdi32' name 'GetTextFaceW';
function GetKerningPairsW(_para1:HDC; _para2:DWORD; _para3:LPKERNINGPAIR):DWORD; external 'gdi32' name 'GetKerningPairsW';
function GetLogColorSpaceW(_para1:HCOLORSPACE; _para2:LPLOGCOLORSPACE; _para3:DWORD):WINBOOL; external 'gdi32' name 'GetLogColorSpaceW';
function CreateColorSpaceW(_para1:LPLOGCOLORSPACE):HCOLORSPACE; external 'gdi32' name 'CreateColorSpaceW';
function GetICMProfileW(_para1:HDC; _para2:LPDWORD; _para3:LPWSTR):WINBOOL; external 'gdi32' name 'GetICMProfileW';
function SetICMProfileW(_para1:HDC; _para2:LPWSTR):WINBOOL; external 'gdi32' name 'SetICMProfileW';
function UpdateICMRegKeyW(_para1:DWORD; _para2:DWORD; _para3:LPWSTR; _para4:UINT):WINBOOL; external 'gdi32' name 'UpdateICMRegKeyW';
function EnumICMProfilesW(_para1:HDC; _para2:ICMENUMPROC; _para3:LPARAM):longint; external 'gdi32' name 'EnumICMProfilesW';
{
function CreatePropertySheetPageW(lppsp:LPCPROPSHEETPAGE):HPROPSHEETPAGE; external 'comctl32' name 'CreatePropertySheetPageW';
function PropertySheetW(lppsph:LPCPROPSHEETHEADER):longint; external 'comctl32' name 'PropertySheetW';
function ImageList_LoadImageW(hi:HINST; lpbmp:LPCWSTR; cx:longint; cGrow:longint; crMask:COLORREF;uType:UINT; uFlags:UINT):HIMAGELIST; external 'comctl32' name 'ImageList_LoadImageW';
function CreateStatusWindowW(style:LONG; lpszText:LPCWSTR; hwndParent:HWND; wID:UINT):HWND; external 'comctl32' name 'CreateStatusWindowW';
procedure DrawStatusTextW(hDC:HDC; lprc:LPRECT; pszText:LPCWSTR; uFlags:UINT); external 'comctl32' name 'DrawStatusTextW';
function GetOpenFileNameW(_para1:LPOPENFILENAME):WINBOOL; external 'comdlg32' name 'GetOpenFileNameW';
function GetSaveFileNameW(_para1:LPOPENFILENAME):WINBOOL; external 'comdlg32' name 'GetSaveFileNameW';
function GetFileTitleW(_para1:LPCWSTR; _para2:LPWSTR; _para3:WORD):integer; external 'comdlg32' name 'GetFileTitleW';
function ChooseColorW(_para1:LPCHOOSECOLOR):WINBOOL; external 'comdlg32' name 'ChooseColorW';
function ReplaceTextW(_para1:LPFINDREPLACE):HWND; external 'comdlg32' name 'ReplaceTextW';
function ChooseFontW(_para1:LPCHOOSEFONT):WINBOOL; external 'comdlg32' name 'ChooseFontW';
function FindTextW(_para1:LPFINDREPLACE):HWND; external 'comdlg32' name 'FindTextW';
function PrintDlgW(_para1:LPPRINTDLG):WINBOOL; external 'comdlg32' name 'PrintDlgW';
function PageSetupDlgW(_para1:LPPAGESETUPDLG):WINBOOL; external 'comdlg32' name 'PageSetupDlgW';
}

function CreateProcessW(lpApplicationName:LPCWSTR; lpCommandLine:LPWSTR; lpProcessAttributes:LPSECURITY_ATTRIBUTES; lpThreadAttributes:LPSECURITY_ATTRIBUTES; bInheritHandles:WINBOOL;dwCreationFlags:DWORD; lpEnvironment:LPVOID;
  lpCurrentDirectory:LPCWSTR; lpStartupInfo:LPSTARTUPINFOW; lpProcessInformation:LPPROCESS_INFORMATION):WINBOOL; external 'kernel32' name 'CreateProcessW';
procedure GetStartupInfoW(lpStartupInfo:LPSTARTUPINFOW); external 'kernel32' name 'GetStartupInfoW';
function FindFirstFileW(lpFileName:LPCWSTR; lpFindFileData:LPWIN32_FIND_DATAW):HANDLE; external 'kernel32' name 'FindFirstFileW';
function FindNextFileW(hFindFile:HANDLE; lpFindFileData:LPWIN32_FIND_DATAW):WINBOOL; external 'kernel32' name 'FindNextFileW';
function GetVersionExW(VersionInformation:LPOSVERSIONINFOW):WINBOOL; external 'kernel32' name 'GetVersionExW';
function CreateWindowW(lpClassName:LPCWSTR; lpWindowName:LPCWSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
function CreateDialogW(hInstance:HINST; lpName:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
function CreateDialogIndirectW(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
function DialogBoxW(hInstance:HINST; lpTemplate:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
function DialogBoxIndirectW(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
function CreateDCW(_para1:LPCWSTR; _para2:LPCWSTR; _para3:LPCWSTR; _para4:pDEVMODEW):HDC; external 'gdi32' name 'CreateDCW';
function VerInstallFileW(uFlags:DWORD; szSrcFileName:LPWSTR; szDestFileName:LPWSTR; szSrcDir:LPWSTR; szDestDir:LPWSTR;szCurDir:LPWSTR; szTmpFile:LPWSTR; lpuTmpFileLen:PUINT):DWORD; external 'version' name 'VerInstallFileW';
function GetFileVersionInfoSizeW(lptstrFilename:LPWSTR; lpdwHandle:LPDWORD):DWORD; external 'version' name 'GetFileVersionInfoSizeW';
function GetFileVersionInfoW(lptstrFilename:LPWSTR; dwHandle:DWORD; dwLen:DWORD; lpData:LPVOID):WINBOOL; external 'version' name 'GetFileVersionInfoW';
function VerLanguageNameW(wLang:DWORD; szLang:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'VerLanguageNameW';
function VerQueryValueW(pBlock:LPVOID; lpSubBlock:LPWSTR; lplpBuffer:LPVOID; puLen:PUINT):WINBOOL; external 'version' name 'VerQueryValueW';
function VerFindFileW(uFlags:DWORD; szFileName:LPWSTR; szWinDir:LPWSTR; szAppDir:LPWSTR; szCurDir:LPWSTR;lpuCurDirLen:PUINT; szDestDir:LPWSTR; lpuDestDirLen:PUINT):DWORD; external 'version' name 'VerFindFileW';
function RegSetValueExW(hKey:HKEY; lpValueName:LPCWSTR; Reserved:DWORD; dwType:DWORD; lpData:LPBYTE;cbData:DWORD):LONG; external 'advapi32' name 'RegSetValueExW';
function RegUnLoadKeyW(hKey:HKEY; lpSubKey:LPCWSTR):LONG; external 'advapi32' name 'RegUnLoadKeyW';
function InitiateSystemShutdownW(lpMachineName:LPWSTR; lpMessage:LPWSTR; dwTimeout:DWORD; bForceAppsClosed:WINBOOL; bRebootAfterShutdown:WINBOOL):WINBOOL; external 'advapi32' name 'InitiateSystemShutdownW';
function AbortSystemShutdownW(lpMachineName:LPWSTR):WINBOOL; external 'advapi32' name 'AbortSystemShutdownW';
function RegRestoreKeyW(hKey:HKEY; lpFile:LPCWSTR; dwFlags:DWORD):LONG; external 'advapi32' name 'RegRestoreKeyW';
function RegSaveKeyW(hKey:HKEY; lpFile:LPCWSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):LONG; external 'advapi32' name 'RegSaveKeyW';
function RegSetValueW(hKey:HKEY; lpSubKey:LPCWSTR; dwType:DWORD; lpData:LPCWSTR; cbData:DWORD):LONG; external 'advapi32' name 'RegSetValueW';
function RegQueryValueW(hKey:HKEY; lpSubKey:LPCWSTR; lpValue:LPWSTR; lpcbValue:PLONG):LONG; external 'advapi32' name 'RegQueryValueW';
function RegQueryMultipleValuesW(hKey:HKEY; val_list:PVALENT; num_vals:DWORD; lpValueBuf:LPWSTR; ldwTotsize:LPDWORD):LONG; external 'advapi32' name 'RegQueryMultipleValuesW';
function RegQueryValueExW(hKey:HKEY; lpValueName:LPCWSTR; lpReserved:LPDWORD; lpType:LPDWORD; lpData:LPBYTE;lpcbData:LPDWORD):LONG; external 'advapi32' name 'RegQueryValueExW';
function RegReplaceKeyW(hKey:HKEY; lpSubKey:LPCWSTR; lpNewFile:LPCWSTR; lpOldFile:LPCWSTR):LONG; external 'advapi32' name 'RegReplaceKeyW';
function RegConnectRegistryW(lpMachineName:LPWSTR; hKey:HKEY; phkResult:PHKEY):LONG; external 'advapi32' name 'RegConnectRegistryW';
function RegCreateKeyW(hKey:HKEY; lpSubKey:LPCWSTR; phkResult:PHKEY):LONG; external 'advapi32' name 'RegCreateKeyW';
function RegCreateKeyExW(hKey:HKEY; lpSubKey:LPCWSTR; Reserved:DWORD; lpClass:LPWSTR; dwOptions:DWORD;samDesired:REGSAM; lpSecurityAttributes:LPSECURITY_ATTRIBUTES; phkResult:PHKEY; lpdwDisposition:LPDWORD):LONG;
  external 'advapi32' name 'RegCreateKeyExW';
function RegDeleteKeyW(hKey:HKEY; lpSubKey:LPCWSTR):LONG; external 'advapi32' name 'RegDeleteKeyW';
function RegDeleteValueW(hKey:HKEY; lpValueName:LPCWSTR):LONG; external 'advapi32' name 'RegDeleteValueW';
function RegEnumKeyW(hKey:HKEY; dwIndex:DWORD; lpName:LPWSTR; cbName:DWORD):LONG; external 'advapi32' name 'RegEnumKeyW';
function RegEnumKeyExW(hKey:HKEY; dwIndex:DWORD; lpName:LPWSTR; lpcbName:LPDWORD; lpReserved:LPDWORD;lpClass:LPWSTR; lpcbClass:LPDWORD; lpftLastWriteTime:PFILETIME):LONG; external 'advapi32' name 'RegEnumKeyExW';
function RegEnumValueW(hKey:HKEY; dwIndex:DWORD; lpValueName:LPWSTR; lpcbValueName:LPDWORD; lpReserved:LPDWORD;lpType:LPDWORD; lpData:LPBYTE; lpcbData:LPDWORD):LONG; external 'advapi32' name 'RegEnumValueW';
function RegLoadKeyW(hKey:HKEY; lpSubKey:LPCWSTR; lpFile:LPCWSTR):LONG; external 'advapi32' name 'RegLoadKeyW';
function RegOpenKeyW(hKey:HKEY; lpSubKey:LPCWSTR; phkResult:PHKEY):LONG; external 'advapi32' name 'RegOpenKeyW';
function RegOpenKeyExW(hKey:HKEY; lpSubKey:LPCWSTR; ulOptions:DWORD; samDesired:REGSAM; phkResult:PHKEY):LONG; external 'advapi32' name 'RegOpenKeyExW';
function RegQueryInfoKeyW(hKey:HKEY; lpClass:LPWSTR; lpcbClass:LPDWORD; lpReserved:LPDWORD; lpcSubKeys:LPDWORD;lpcbMaxSubKeyLen:LPDWORD; lpcbMaxClassLen:LPDWORD; lpcValues:LPDWORD; lpcbMaxValueNameLen:LPDWORD;
  lpcbMaxValueLen:LPDWORD;lpcbSecurityDescriptor:LPDWORD; lpftLastWriteTime:PFILETIME):LONG; external 'advapi32' name 'RegQueryInfoKeyW';
function CompareStringW(Locale:LCID; dwCmpFlags:DWORD; lpString1:LPCWSTR; cchCount1:longint; lpString2:LPCWSTR;cchCount2:longint):longint; external 'kernel32' name 'CompareStringW';
function LCMapStringW(Locale:LCID; dwMapFlags:DWORD; lpSrcStr:LPCWSTR; cchSrc:longint; lpDestStr:LPWSTR;cchDest:longint):longint; external 'kernel32' name 'LCMapStringW';
function GetLocaleInfoW(Locale:LCID; LCType:LCTYPE; lpLCData:LPWSTR; cchData:longint):longint; external 'kernel32' name 'GetLocaleInfoW';
function SetLocaleInfoW(Locale:LCID; LCType:LCTYPE; lpLCData:LPCWSTR):WINBOOL; external 'kernel32' name 'SetLocaleInfoW';
function GetTimeFormatW(Locale:LCID; dwFlags:DWORD; lpTime:LPSYSTEMTIME; lpFormat:LPCWSTR; lpTimeStr:LPWSTR;cchTime:longint):longint; external 'kernel32' name 'GetTimeFormatW';
function GetDateFormatW(Locale:LCID; dwFlags:DWORD; lpDate:LPSYSTEMTIME; lpFormat:LPCWSTR; lpDateStr:LPWSTR;cchDate:longint):longint; external 'kernel32' name 'GetDateFormatW';
function GetNumberFormatW(Locale:LCID; dwFlags:DWORD; lpValue:LPCWSTR; lpFormat:PNUMBERFMT; lpNumberStr:LPWSTR;cchNumber:longint):longint; external 'kernel32' name 'GetNumberFormatW';
function GetCurrencyFormatW(Locale:LCID; dwFlags:DWORD; lpValue:LPCWSTR; lpFormat:PCURRENCYFMT; lpCurrencyStr:LPWSTR;cchCurrency:longint):longint; external 'kernel32' name 'GetCurrencyFormatW';
function EnumCalendarInfoW(lpCalInfoEnumProc:CALINFO_ENUMPROC; Locale:LCID; Calendar:CALID; CalType:CALTYPE):WINBOOL; external 'kernel32' name 'EnumCalendarInfoW';
function EnumTimeFormatsW(lpTimeFmtEnumProc:TIMEFMT_ENUMPROC; Locale:LCID; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumTimeFormatsW';
function EnumDateFormatsW(lpDateFmtEnumProc:DATEFMT_ENUMPROC; Locale:LCID; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumDateFormatsW';
function GetStringTypeExW(Locale:LCID; dwInfoType:DWORD; lpSrcStr:LPCWSTR; cchSrc:longint; lpCharType:LPWORD):WINBOOL; external 'kernel32' name 'GetStringTypeExW';
function GetStringTypeW(dwInfoType:DWORD; lpSrcStr:LPCWSTR; cchSrc:longint; lpCharType:LPWORD):WINBOOL; external 'kernel32' name 'GetStringTypeW';
function FoldStringW(dwMapFlags:DWORD; lpSrcStr:LPCWSTR; cchSrc:longint; lpDestStr:LPWSTR; cchDest:longint):longint; external 'kernel32' name 'FoldStringW';
function EnumSystemLocalesW(lpLocaleEnumProc:LOCALE_ENUMPROC; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumSystemLocalesW';
function EnumSystemCodePagesW(lpCodePageEnumProc:CODEPAGE_ENUMPROC; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumSystemCodePagesW';
function PeekConsoleInputW(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsRead:LPDWORD):WINBOOL; external 'kernel32' name 'PeekConsoleInputW';
function ReadConsoleInputW(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsRead:LPDWORD):WINBOOL; external 'kernel32' name 'ReadConsoleInputW';
function WriteConsoleInputW(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'WriteConsoleInputW';
function ReadConsoleOutputW(hConsoleOutput:HANDLE; lpBuffer:PCHAR_INFO; dwBufferSize:COORD; dwBufferCoord:COORD; lpReadRegion:PSMALL_RECT):WINBOOL; external 'kernel32' name 'ReadConsoleOutputW';
function WriteConsoleOutputW(hConsoleOutput:HANDLE; lpBuffer:PCHAR_INFO; dwBufferSize:COORD; dwBufferCoord:COORD; lpWriteRegion:PSMALL_RECT):WINBOOL; external 'kernel32' name 'WriteConsoleOutputW';
function ReadConsoleOutputCharacterW(hConsoleOutput:HANDLE; lpCharacter:LPWSTR; nLength:DWORD; dwReadCoord:COORD; lpNumberOfCharsRead:LPDWORD):WINBOOL; external 'kernel32' name 'ReadConsoleOutputCharacterW';
function WriteConsoleOutputCharacterW(hConsoleOutput:HANDLE; lpCharacter:LPCWSTR; nLength:DWORD; dwWriteCoord:COORD; lpNumberOfCharsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'WriteConsoleOutputCharacterW';
function FillConsoleOutputCharacterW(hConsoleOutput:HANDLE; cCharacter:WCHAR; nLength:DWORD; dwWriteCoord:COORD; lpNumberOfCharsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'FillConsoleOutputCharacterW';
function ScrollConsoleScreenBufferW(hConsoleOutput:HANDLE; lpScrollRectangle:PSMALL_RECT; lpClipRectangle:PSMALL_RECT; dwDestinationOrigin:COORD; lpFill:PCHAR_INFO):WINBOOL; external 'kernel32' name 'ScrollConsoleScreenBufferW';
function GetConsoleTitleW(lpConsoleTitle:LPWSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetConsoleTitleW';
function SetConsoleTitleW(lpConsoleTitle:LPCWSTR):WINBOOL; external 'kernel32' name 'SetConsoleTitleW';
function ReadConsoleW(hConsoleInput:HANDLE; lpBuffer:LPVOID; nNumberOfCharsToRead:DWORD; lpNumberOfCharsRead:LPDWORD; lpReserved:LPVOID):WINBOOL; external 'kernel32' name 'ReadConsoleW';
function WriteConsoleW(hConsoleOutput:HANDLE;lpBuffer:pointer; nNumberOfCharsToWrite:DWORD; lpNumberOfCharsWritten:LPDWORD; lpReserved:LPVOID):WINBOOL; external 'kernel32' name 'WriteConsoleW';
function GetCPInfoExW(Codepage:UINT; dwFlags:DWORD; CPinfoEx:LPCPINFOEXW):WINBOOL; external 'kernel32' name 'GetCPInfoExW';
function WNetAddConnectionW(lpRemoteName:LPCWSTR; lpPassword:LPCWSTR; lpLocalName:LPCWSTR):DWORD; external 'mpr' name 'WNetAddConnectionW';
function WNetAddConnection2W(lpNetResource:LPNETRESOURCE; lpPassword:LPCWSTR; lpUserName:LPCWSTR; dwFlags:DWORD):DWORD; external 'mpr' name 'WNetAddConnection2W';
function WNetAddConnection3W(hwndOwner:HWND; lpNetResource:LPNETRESOURCE; lpPassword:LPCWSTR; lpUserName:LPCWSTR; dwFlags:DWORD):DWORD; external 'mpr' name 'WNetAddConnection3W';
function WNetCancelConnectionW(lpName:LPCWSTR; fForce:WINBOOL):DWORD; external 'mpr' name 'WNetCancelConnectionW';
function WNetCancelConnection2W(lpName:LPCWSTR; dwFlags:DWORD; fForce:WINBOOL):DWORD; external 'mpr' name 'WNetCancelConnection2W';
function WNetGetConnectionW(lpLocalName:LPCWSTR; lpRemoteName:LPWSTR; lpnLength:LPDWORD):DWORD; external 'mpr' name 'WNetGetConnectionW';
function WNetUseConnectionW(hwndOwner:HWND; lpNetResource:LPNETRESOURCE; lpUserID:LPCWSTR; lpPassword:LPCWSTR; dwFlags:DWORD;lpAccessName:LPWSTR; lpBufferSize:LPDWORD; lpResult:LPDWORD):DWORD; external 'mpr' name 'WNetUseConnectionW';
function WNetSetConnectionW(lpName:LPCWSTR; dwProperties:DWORD; pvValues:LPVOID):DWORD; external 'mpr' name 'WNetSetConnectionW';
function WNetConnectionDialog1W(lpConnDlgStruct:LPCONNECTDLGSTRUCT):DWORD; external 'mpr' name 'WNetConnectionDialog1W';
function WNetDisconnectDialog1W(lpConnDlgStruct:LPDISCDLGSTRUCT):DWORD; external 'mpr' name 'WNetDisconnectDialog1W';
function WNetOpenEnumW(dwScope:DWORD; dwType:DWORD; dwUsage:DWORD; lpNetResource:LPNETRESOURCE; lphEnum:LPHANDLE):DWORD; external 'mpr' name 'WNetOpenEnumW';
function WNetEnumResourceW(hEnum:HANDLE; lpcCount:LPDWORD; lpBuffer:LPVOID; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetEnumResourceW';
function WNetGetUniversalNameW(lpLocalPath:LPCWSTR; dwInfoLevel:DWORD; lpBuffer:LPVOID; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetGetUniversalNameW';
function WNetGetUserW(lpName:LPCWSTR; lpUserName:LPWSTR; lpnLength:LPDWORD):DWORD; external 'mpr' name 'WNetGetUserW';
function WNetGetProviderNameW(dwNetType:DWORD; lpProviderName:LPWSTR; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetGetProviderNameW';
function WNetGetNetworkInformationW(lpProvider:LPCWSTR; lpNetInfoStruct:LPNETINFOSTRUCT):DWORD; external 'mpr' name 'WNetGetNetworkInformationW';
function WNetGetLastErrorW(lpError:LPDWORD; lpErrorBuf:LPWSTR; nErrorBufSize:DWORD; lpNameBuf:LPWSTR; nNameBufSize:DWORD):DWORD; external 'mpr' name 'WNetGetLastErrorW';
function MultinetGetConnectionPerformanceW(lpNetResource:LPNETRESOURCE; lpNetConnectInfoStruct:LPNETCONNECTINFOSTRUCT):DWORD; external 'mpr' name 'MultinetGetConnectionPerformanceW';
function ChangeServiceConfigW(hService:SC_HANDLE; dwServiceType:DWORD; dwStartType:DWORD; dwErrorControl:DWORD; lpBinaryPathName:LPCWSTR;lpLoadOrderGroup:LPCWSTR; lpdwTagId:LPDWORD; lpDependencies:LPCWSTR; lpServiceStartName:LPCWSTR;
  lpPassword:LPCWSTR;lpDisplayName:LPCWSTR):WINBOOL; external 'advapi32' name 'ChangeServiceConfigW';
function CreateServiceW(hSCManager:SC_HANDLE; lpServiceName:LPCWSTR; lpDisplayName:LPCWSTR; dwDesiredAccess:DWORD; dwServiceType:DWORD;dwStartType:DWORD; dwErrorControl:DWORD; lpBinaryPathName:LPCWSTR; lpLoadOrderGroup:LPCWSTR;
  lpdwTagId:LPDWORD;lpDependencies:LPCWSTR; lpServiceStartName:LPCWSTR; lpPassword:LPCWSTR):SC_HANDLE; external 'advapi32' name 'CreateServiceW';
function EnumDependentServicesW(hService:SC_HANDLE; dwServiceState:DWORD; lpServices:LPENUM_SERVICE_STATUS; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD;lpServicesReturned:LPDWORD):WINBOOL; external 'advapi32' name 'EnumDependentServicesW';
function EnumServicesStatusW(hSCManager:SC_HANDLE; dwServiceType:DWORD; dwServiceState:DWORD; lpServices:LPENUM_SERVICE_STATUS; cbBufSize:DWORD;pcbBytesNeeded:LPDWORD; lpServicesReturned:LPDWORD; lpResumeHandle:LPDWORD):WINBOOL;
  external 'advapi32' name 'EnumServicesStatusW';
function GetServiceKeyNameW(hSCManager:SC_HANDLE; lpDisplayName:LPCWSTR; lpServiceName:LPWSTR; lpcchBuffer:LPDWORD):WINBOOL; external 'advapi32' name 'GetServiceKeyNameW';
function GetServiceDisplayNameW(hSCManager:SC_HANDLE; lpServiceName:LPCWSTR; lpDisplayName:LPWSTR; lpcchBuffer:LPDWORD):WINBOOL; external 'advapi32' name 'GetServiceDisplayNameW';
function OpenSCManagerW(lpMachineName:LPCWSTR; lpDatabaseName:LPCWSTR; dwDesiredAccess:DWORD):SC_HANDLE; external 'advapi32' name 'OpenSCManagerW';
function OpenServiceW(hSCManager:SC_HANDLE; lpServiceName:LPCWSTR; dwDesiredAccess:DWORD):SC_HANDLE; external 'advapi32' name 'OpenServiceW';
function QueryServiceConfigW(hService:SC_HANDLE; lpServiceConfig:LPQUERY_SERVICE_CONFIG; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'QueryServiceConfigW';
function QueryServiceLockStatusW(hSCManager:SC_HANDLE; lpLockStatus:LPQUERY_SERVICE_LOCK_STATUS; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'QueryServiceLockStatusW';
function RegisterServiceCtrlHandlerW(lpServiceName:LPCWSTR; lpHandlerProc:LPHANDLER_FUNCTION):SERVICE_STATUS_HANDLE; external 'advapi32' name 'RegisterServiceCtrlHandlerW';
function StartServiceCtrlDispatcherW(lpServiceStartTable:LPSERVICE_TABLE_ENTRY):WINBOOL; external 'advapi32' name 'StartServiceCtrlDispatcherW';
function StartServiceW(hService:SC_HANDLE; dwNumServiceArgs:DWORD; lpServiceArgVectors:LPPCWSTR):WINBOOL; external 'advapi32' name 'StartServiceW';
function DragQueryFileW(_para1:HDROP; _para2:cardinal; _para3:LPCWSTR; _para4:cardinal):cardinal; external 'shell32' name 'DragQueryFileW';
function ExtractAssociatedIconW(_para1:HINST; _para2:LPCWSTR; _para3:LPWORD):HICON; external 'shell32' name 'ExtractAssociatedIconW';
function ExtractIconW(_para1:HINST; _para2:LPCWSTR; _para3:cardinal):HICON; external 'shell32' name 'ExtractIconW';
function FindExecutableW(_para1:LPCWSTR; _para2:LPCWSTR; _para3:LPCWSTR):HINST; external 'shell32' name 'FindExecutableW';
function ShellAboutW(_para1:HWND; _para2:LPCWSTR; _para3:LPCWSTR; _para4:HICON):longint; external 'shell32' name 'ShellAboutW';
function ShellExecuteW(_para1:HWND; _para2:LPCWSTR; _para3:LPCWSTR; _para4:LPCWSTR; _para5:LPCWSTR;_para6:longint):HINST; external 'shell32' name 'ShellExecuteW';
function Shell_NotifyIconW(dwMessage: DWORD; lpData: PNotifyIconDataA): BOOL; external 'shell32' name 'Shell_NotifyIconW';
function DdeCreateStringHandleW(_para1:DWORD; _para2:LPCWSTR; _para3:longint):HSZ; external 'user32' name 'DdeCreateStringHandleW';
function DdeInitializeW(_para1:LPDWORD; _para2:PFNCALLBACK; _para3:DWORD; _para4:DWORD):UINT; external 'user32' name 'DdeInitializeW';
function DdeQueryStringW(_para1:DWORD; _para2:HSZ; _para3:LPCWSTR; _para4:DWORD; _para5:longint):DWORD; external 'user32' name 'DdeQueryStringW';
function LogonUserW(_para1:LPWSTR; _para2:LPWSTR; _para3:LPWSTR; _para4:DWORD; _para5:DWORD;_para6:PHANDLE):WINBOOL; external 'advapi32' name 'LogonUserW';
function CreateProcessAsUserW(_para1:HANDLE; _para2:LPCWSTR; _para3:LPWSTR; _para4:LPSECURITY_ATTRIBUTES; _para5:LPSECURITY_ATTRIBUTES;_para6:WINBOOL; _para7:DWORD; _para8:LPVOID; _para9:LPCWSTR;
<<<<<<< HEAD
  _para10:LPSTARTUPINFOW;_para11:LPPROCESS_INFORMATION):WINBOOL; external 'advapi32' name 'CreateProcessAsUserW';
function CreateWaitableTimerW(lpTimerAttributes :LPSECURITY_ATTRIBUTES; bManualReset:BOOl;lpTimerName:LPWSTR):THandle;external 'kernel32' name 'CreateWaitableTimerW'; 
function OpenWaitableTimerW(dwDesiredAccess:DWORD;bInheritHandle:BOOL;lpTimerName:LPWSTR):THandle;external 'kernel32' name 'OpenWaitableTimerW'; 

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function FindFirstFileExW(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external 'kernel32' name 'FindFirstFileExW';
// winver>$0600 
function FindFirstFileTransactedW(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external 'kernel32' name 'FindFirstFileTransactedW';
=======
function FindFirstFileExW(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external name 'FindFirstFileExW';
// winver>$0600 
function FindFirstFileTransactedW(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedW';
>>>>>>> graemeg/cpstrnew
=======
function FindFirstFileExW(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external name 'FindFirstFileExW';
// winver>$0600 
function FindFirstFileTransactedW(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedW';
>>>>>>> graemeg/cpstrnew
=======
function FindFirstFileExW(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external name 'FindFirstFileExW';
// winver>$0600 
function FindFirstFileTransactedW(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedW';
>>>>>>> graemeg/cpstrnew
=======
function FindFirstFileExW(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external name 'FindFirstFileExW';
// winver>$0600 
function FindFirstFileTransactedW(lpfilename : LPCWStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedW';
>>>>>>> origin/cpstrnew

=======
  _para10:LPSTARTUPINFO;_para11:LPPROCESS_INFORMATION):WINBOOL; external 'advapi32' name 'CreateProcessAsUserW';
function CreateWaitableTimerW(lpTimerAttributes :LPSECURITY_ATTRIBUTES; bManualReset:BOOl;lpTimerName:LPWSTR):THandle;external 'kernel32' name 'CreateWaitableTimerW'; 
function OpenWaitableTimerW(dwDesiredAccess:DWORD;bInheritHandle:BOOL;lpTimerName:LPWSTR):THandle;external 'kernel32' name 'OpenWaitableTimerW'; 
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif read_interface}


{$ifdef read_implementation}

function CreateWindowW(lpClassName:LPCWSTR; lpWindowName:LPCWSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
begin
  CreateWindowW:=CreateWindowExW(0,lpClassName,lpWindowName,dwStyle,x,y,nWidth,nHeight,hWndParent,hMenu,hInstance,lpParam);
end;

function CreateDialogW(hInstance:HINST; lpName:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
begin
  CreateDialogW:=CreateDialogParamW(hInstance,lpName,hWndParent,lpDialogFunc,0);
end;

function CreateDialogIndirectW(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
begin
  CreateDialogIndirectW:=CreateDialogIndirectParamW(hInstance,lpTemplate,hWndParent,lpDialogFunc,0);
end;

function DialogBoxW(hInstance:HINST; lpTemplate:LPCWSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
begin
  DialogBoxW:=DialogBoxParamW(hInstance,lpTemplate,hWndParent,lpDialogFunc,0);
end;

function DialogBoxIndirectW(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
begin
  DialogBoxIndirectW:=DialogBoxIndirectParamW(hInstance,lpTemplate,hWndParent,lpDialogFunc,0);
end;

{$endif read_implementation}

