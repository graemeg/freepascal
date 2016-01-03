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

function GetBinaryTypeA(lpApplicationName:LPCSTR; lpBinaryType:LPDWORD):WINBOOL; external 'kernel32' name 'GetBinaryTypeA';
function GetShortPathNameA(lpszLongPath:LPCSTR; lpszShortPath:LPSTR; cchBuffer:DWORD):DWORD; external 'kernel32' name 'GetShortPathNameA';
function GetEnvironmentStringsA:LPSTR; external 'kernel32' name 'GetEnvironmentStringsA';
function FreeEnvironmentStringsA(_para1:LPSTR):WINBOOL; external 'kernel32' name 'FreeEnvironmentStringsA';
function FormatMessageA(dwFlags:DWORD; lpSource:LPCVOID; dwMessageId:DWORD; dwLanguageId:DWORD; lpBuffer:LPSTR;nSize:DWORD; Arguments:va_list):DWORD; external 'kernel32' name 'FormatMessageA';
function CreateMailslotA(lpName:LPCSTR; nMaxMessageSize:DWORD; lReadTimeout:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):HANDLE; external 'kernel32' name 'CreateMailslotA';
function lstrcmpA(lpString1:LPCSTR; lpString2:LPCSTR):longint; external 'kernel32' name 'lstrcmpA';
function lstrcmpiA(lpString1:LPCSTR; lpString2:LPCSTR):longint; external 'kernel32' name 'lstrcmpiA';
function lstrcpynA(lpString1:LPSTR; lpString2:LPCSTR; iMaxLength:longint):LPSTR; external 'kernel32' name 'lstrcpynA';
function lstrcpyA(lpString1:LPSTR; lpString2:LPCSTR):LPSTR; external 'kernel32' name 'lstrcpyA';
function lstrcatA(lpString1:LPSTR; lpString2:LPCSTR):LPSTR; external 'kernel32' name 'lstrcatA';
function lstrlenA(lpString:LPCSTR):longint; external 'kernel32' name 'lstrlenA';
function CreateMutexA(lpMutexAttributes:LPSECURITY_ATTRIBUTES; bInitialOwner:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'CreateMutexA';
function OpenMutexA(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'OpenMutexA';
function CreateEventA(lpEventAttributes:LPSECURITY_ATTRIBUTES; bManualReset:WINBOOL; bInitialState:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'CreateEventA';
function OpenEventA(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'OpenEventA';
function CreateSemaphoreA(lpSemaphoreAttributes:LPSECURITY_ATTRIBUTES; lInitialCount:LONG; lMaximumCount:LONG; lpName:LPCSTR):HANDLE; external 'kernel32' name 'CreateSemaphoreA';
function OpenSemaphoreA(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'OpenSemaphoreA';
function CreateFileMappingA(hFile:HANDLE; lpFileMappingAttributes:LPSECURITY_ATTRIBUTES; flProtect:DWORD; dwMaximumSizeHigh:DWORD; dwMaximumSizeLow:DWORD;lpName:LPCSTR):HANDLE; external 'kernel32' name 'CreateFileMappingA';
function OpenFileMappingA(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; lpName:LPCSTR):HANDLE; external 'kernel32' name 'OpenFileMappingA';
function GetLogicalDriveStringsA(nBufferLength:DWORD; lpBuffer:LPSTR):DWORD; external 'kernel32' name 'GetLogicalDriveStringsA';
function LoadLibraryA(lpLibFileName:LPCSTR):HINST; external 'kernel32' name 'LoadLibraryA';
function LoadLibraryExA(lpLibFileName:LPCSTR; hFile:HANDLE; dwFlags:DWORD):HINST; external 'kernel32' name 'LoadLibraryExA';
function GetModuleFileNameA(hModule:HINST; lpFilename:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetModuleFileNameA';
function GetModuleHandleA(lpModuleName:LPCSTR):HMODULE; external 'kernel32' name 'GetModuleHandleA';
procedure FatalAppExitA(uAction:UINT; lpMessageText:LPCSTR); external 'kernel32' name 'FatalAppExitA';
function GetCommandLineA:LPSTR; external 'kernel32' name 'GetCommandLineA';
function GetEnvironmentVariableA(lpName:LPCSTR; lpBuffer:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetEnvironmentVariableA';
function SetEnvironmentVariableA(lpName:LPCSTR; lpValue:LPCSTR):WINBOOL; external 'kernel32' name 'SetEnvironmentVariableA';
function ExpandEnvironmentStringsA(lpSrc:LPCSTR; lpDst:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'ExpandEnvironmentStringsA';
procedure OutputDebugStringA(lpOutputString:LPCSTR); external 'kernel32' name 'OutputDebugStringA';
function FindResourceA(hModule:HINST; lpName:LPCSTR; lpType:LPCSTR):HRSRC; external 'kernel32' name 'FindResourceA';
function FindResourceExA(hModule:HINST; lpType:LPCSTR; lpName:LPCSTR; wLanguage:WORD):HRSRC; external 'kernel32' name 'FindResourceExA';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function EnumResourceTypesA(hModule:HMODULE; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesA';
function EnumResourceNamesA(hModule:HMODULE; lpType:LPCSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesA';
function EnumResourceLanguagesA(hModule:HMODULE; lpType:LPCSTR; lpName:LPCSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesA';
=======
function EnumResourceTypesA(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesA';
function EnumResourceNamesA(hModule:HINST; lpType:LPCSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesA';
function EnumResourceLanguagesA(hModule:HINST; lpType:LPCSTR; lpName:LPCSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesA';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypesA(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesA';
function EnumResourceNamesA(hModule:HINST; lpType:LPCSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesA';
function EnumResourceLanguagesA(hModule:HINST; lpType:LPCSTR; lpName:LPCSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesA';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypesA(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesA';
function EnumResourceNamesA(hModule:HINST; lpType:LPCSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesA';
function EnumResourceLanguagesA(hModule:HINST; lpType:LPCSTR; lpName:LPCSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesA';
>>>>>>> graemeg/cpstrnew
=======
function EnumResourceTypesA(hModule:HINST; lpEnumFunc:ENUMRESTYPEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceTypesA';
function EnumResourceNamesA(hModule:HINST; lpType:LPCSTR; lpEnumFunc:ENUMRESNAMEPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceNamesA';
function EnumResourceLanguagesA(hModule:HINST; lpType:LPCSTR; lpName:LPCSTR; lpEnumFunc:ENUMRESLANGPROC; lParam:LONG_PTR):WINBOOL; external 'kernel32' name 'EnumResourceLanguagesA';
>>>>>>> origin/cpstrnew
function BeginUpdateResourceA(pFileName:LPCSTR; bDeleteExistingResources:WINBOOL):HANDLE; external 'kernel32' name 'BeginUpdateResourceA';
function UpdateResourceA(hUpdate:HANDLE; lpType:LPCSTR; lpName:LPCSTR; wLanguage:WORD; lpData:LPVOID;cbData:DWORD):WINBOOL; external 'kernel32' name 'UpdateResourceA';
function EndUpdateResourceA(hUpdate:HANDLE; fDiscard:WINBOOL):WINBOOL; external 'kernel32' name 'EndUpdateResourceA';
function GlobalAddAtomA(lpString:LPCSTR):ATOM; external 'kernel32' name 'GlobalAddAtomA';
function GlobalFindAtomA(lpString:LPCSTR):ATOM; external 'kernel32' name 'GlobalFindAtomA';
function GlobalGetAtomNameA(nAtom:ATOM; lpBuffer:LPSTR; nSize:longint):UINT; external 'kernel32' name 'GlobalGetAtomNameA';
function AddAtomA(lpString:LPCSTR):ATOM; external 'kernel32' name 'AddAtomA';
function FindAtomA(lpString:LPCSTR):ATOM; external 'kernel32' name 'FindAtomA';
function GetAtomNameA(nAtom:ATOM; lpBuffer:LPSTR; nSize:longint):UINT; external 'kernel32' name 'GetAtomNameA';
function GetProfileIntA(lpAppName:LPCSTR; lpKeyName:LPCSTR; nDefault:WINT):UINT; external 'kernel32' name 'GetProfileIntA';
function GetProfileStringA(lpAppName:LPCSTR; lpKeyName:LPCSTR; lpDefault:LPCSTR; lpReturnedString:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetProfileStringA';
function WriteProfileStringA(lpAppName:LPCSTR; lpKeyName:LPCSTR; lpString:LPCSTR):WINBOOL; external 'kernel32' name 'WriteProfileStringA';
function GetProfileSectionA(lpAppName:LPCSTR; lpReturnedString:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetProfileSectionA';
function WriteProfileSectionA(lpAppName:LPCSTR; lpString:LPCSTR):WINBOOL; external 'kernel32' name 'WriteProfileSectionA';
function GetPrivateProfileIntA(lpAppName:LPCSTR; lpKeyName:LPCSTR; nDefault:WINT; lpFileName:LPCSTR):UINT; external 'kernel32' name 'GetPrivateProfileIntA';
function GetPrivateProfileStringA(lpAppName:LPCSTR; lpKeyName:LPCSTR; lpDefault:LPCSTR; lpReturnedString:LPSTR; nSize:DWORD;lpFileName:LPCSTR):DWORD; external 'kernel32' name 'GetPrivateProfileStringA';
function WritePrivateProfileStringA(lpAppName:LPCSTR; lpKeyName:LPCSTR; lpString:LPCSTR; lpFileName:LPCSTR):WINBOOL; external 'kernel32' name 'WritePrivateProfileStringA';
function GetPrivateProfileSectionA(lpAppName:LPCSTR; lpReturnedString:LPSTR; nSize:DWORD; lpFileName:LPCSTR):DWORD; external 'kernel32' name 'GetPrivateProfileSectionA';
function WritePrivateProfileSectionA(lpAppName:LPCSTR; lpString:LPCSTR; lpFileName:LPCSTR):WINBOOL; external 'kernel32' name 'WritePrivateProfileSectionA';
function GetDriveTypeA(lpRootPathName:LPCSTR):UINT; external 'kernel32' name 'GetDriveTypeA';
function GetSystemDirectoryA(lpBuffer:LPSTR; uSize:UINT):UINT; external 'kernel32' name 'GetSystemDirectoryA';
function GetTempPathA(nBufferLength:DWORD; lpBuffer:LPSTR):DWORD; external 'kernel32' name 'GetTempPathA';
function GetTempFileNameA(lpPathName:LPCSTR; lpPrefixString:LPCSTR; uUnique:UINT; lpTempFileName:LPSTR):UINT; external 'kernel32' name 'GetTempFileNameA';
function GetWindowsDirectoryA(lpBuffer:LPSTR; uSize:UINT):UINT; external 'kernel32' name 'GetWindowsDirectoryA';
function SetCurrentDirectoryA(lpPathName:LPCSTR):WINBOOL; external 'kernel32' name 'SetCurrentDirectoryA';
function GetCurrentDirectoryA(nBufferLength:DWORD; lpBuffer:LPSTR):DWORD; external 'kernel32' name 'GetCurrentDirectoryA';
function GetDiskFreeSpaceA(lpRootPathName:LPCSTR; lpSectorsPerCluster:LPDWORD; lpBytesPerSector:LPDWORD; lpNumberOfFreeClusters:LPDWORD; lpTotalNumberOfClusters:LPDWORD):WINBOOL; external 'kernel32' name 'GetDiskFreeSpaceA';
function CreateDirectoryA(lpPathName:LPCSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):WINBOOL; external 'kernel32' name 'CreateDirectoryA';
function CreateDirectoryExA(lpTemplateDirectory:LPCSTR; lpNewDirectory:LPCSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):WINBOOL; external 'kernel32' name 'CreateDirectoryExA';
function RemoveDirectoryA(lpPathName:LPCSTR):WINBOOL; external 'kernel32' name 'RemoveDirectoryA';
function GetFullPathNameA(lpFileName:LPCSTR; nBufferLength:DWORD; lpBuffer:LPSTR; var lpFilePart:LPSTR):DWORD; external 'kernel32' name 'GetFullPathNameA';
function DefineDosDeviceA(dwFlags:DWORD; lpDeviceName:LPCSTR; lpTargetPath:LPCSTR):WINBOOL; external 'kernel32' name 'DefineDosDeviceA';
function QueryDosDeviceA(lpDeviceName:LPCSTR; lpTargetPath:LPSTR; ucchMax:DWORD):DWORD; external 'kernel32' name 'QueryDosDeviceA';
function CreateFileA(lpFileName:LPCSTR; dwDesiredAccess:DWORD; dwShareMode:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES; dwCreationDisposition:DWORD;dwFlagsAndAttributes:DWORD; hTemplateFile:HANDLE):HANDLE; external 'kernel32' name 'CreateFileA';
function SetFileAttributesA(lpFileName:LPCSTR; dwFileAttributes:DWORD):WINBOOL; external 'kernel32' name 'SetFileAttributesA';
function GetFileAttributesA(lpFileName:LPCSTR):DWORD; external 'kernel32' name 'GetFileAttributesA';
function GetFileAttributesExA(lpFileName:LPCSTR; fInfoLevelId:GET_FILEEX_INFO_LEVELS; lpFileInformation:pointer):BOOL;stdcall; external 'kernel32' name 'GetFileAttributesExA';
function GetCompressedFileSizeA(lpFileName:LPCSTR; lpFileSizeHigh:LPDWORD):DWORD; external 'kernel32' name 'GetCompressedFileSizeA';
function DeleteFileA(lpFileName:LPCSTR):WINBOOL; external 'kernel32' name 'DeleteFileA';
function SearchPathA(lpPath:LPCSTR; lpFileName:LPCSTR; lpExtension:LPCSTR; nBufferLength:DWORD; lpBuffer:LPSTR;lpFilePart:LPSTR):DWORD; external 'kernel32' name 'SearchPathA';
function CopyFileA(lpExistingFileName:LPCSTR; lpNewFileName:LPCSTR; bFailIfExists:WINBOOL):WINBOOL; external 'kernel32' name 'CopyFileA';
function MoveFileA(lpExistingFileName:LPCSTR; lpNewFileName:LPCSTR):WINBOOL; external 'kernel32' name 'MoveFileA';
function MoveFileExA(lpExistingFileName:LPCSTR; lpNewFileName:LPCSTR; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'MoveFileExA';
function CreateNamedPipeA(lpName:LPCSTR; dwOpenMode:DWORD; dwPipeMode:DWORD; nMaxInstances:DWORD; nOutBufferSize:DWORD;nInBufferSize:DWORD; nDefaultTimeOut:DWORD; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):HANDLE;
  external 'kernel32' name 'CreateNamedPipeA';
function GetNamedPipeHandleStateA(hNamedPipe:HANDLE; lpState:LPDWORD; lpCurInstances:LPDWORD; lpMaxCollectionCount:LPDWORD; lpCollectDataTimeout:LPDWORD;lpUserName:LPSTR; nMaxUserNameSize:DWORD):WINBOOL;
  external 'kernel32' name 'GetNamedPipeHandleStateA';
function CallNamedPipeA(lpNamedPipeName:LPCSTR; lpInBuffer:LPVOID; nInBufferSize:DWORD; lpOutBuffer:LPVOID; nOutBufferSize:DWORD;lpBytesRead:LPDWORD; nTimeOut:DWORD):WINBOOL; external 'kernel32' name 'CallNamedPipeA';
function WaitNamedPipeA(lpNamedPipeName:LPCSTR; nTimeOut:DWORD):WINBOOL; external 'kernel32' name 'WaitNamedPipeA';
function SetVolumeLabelA(lpRootPathName:LPCSTR; lpVolumeName:LPCSTR):WINBOOL; external 'kernel32' name 'SetVolumeLabelA';
function GetVolumeInformationA(lpRootPathName:LPCSTR; lpVolumeNameBuffer:LPSTR; nVolumeNameSize:DWORD; lpVolumeSerialNumber:LPDWORD; lpMaximumComponentLength:LPDWORD;lpFileSystemFlags:LPDWORD;
  lpFileSystemNameBuffer:LPSTR; nFileSystemNameSize:DWORD):WINBOOL; external 'kernel32' name 'GetVolumeInformationA';
function GetCPInfoExA(Codepage:UINT; dwFlags:DWORD; CPInfoEx:LPCPINFOEXA):WINBOOL; external 'kernel32' name 'GetCPInfoExA';
function ClearEventLogA(hEventLog:HANDLE; lpBackupFileName:LPCSTR):WINBOOL; external 'advapi32' name 'ClearEventLogA';
function BackupEventLogA(hEventLog:HANDLE; lpBackupFileName:LPCSTR):WINBOOL; external 'advapi32' name 'BackupEventLogA';
function OpenEventLogA(lpUNCServerName:LPCSTR; lpSourceName:LPCSTR):HANDLE; external 'advapi32' name 'OpenEventLogA';
function RegisterEventSourceA(lpUNCServerName:LPCSTR; lpSourceName:LPCSTR):HANDLE; external 'advapi32' name 'RegisterEventSourceA';
function OpenBackupEventLogA(lpUNCServerName:LPCSTR; lpFileName:LPCSTR):HANDLE; external 'advapi32' name 'OpenBackupEventLogA';
function ReadEventLogA(hEventLog:HANDLE; dwReadFlags:DWORD; dwRecordOffset:DWORD; lpBuffer:LPVOID; nNumberOfBytesToRead:DWORD;pnBytesRead:LPDWORD; pnMinNumberOfBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'ReadEventLogA';
function ReportEventA(hEventLog:HANDLE; wType:WORD; wCategory:WORD; dwEventID:DWORD; lpUserSid:PSID;wNumStrings:WORD; dwDataSize:DWORD; lpStrings:LPPCSTR; lpRawData:LPVOID):WINBOOL; external 'advapi32' name 'ReportEventA';
function AccessCheckAndAuditAlarmA(SubsystemName:LPCSTR; HandleId:LPVOID; ObjectTypeName:LPSTR; ObjectName:LPSTR; SecurityDescriptor:PSECURITY_DESCRIPTOR;DesiredAccess:DWORD; GenericMapping:PGENERIC_MAPPING;
  ObjectCreation:WINBOOL; GrantedAccess:LPDWORD;AccessStatus:LPBOOL;pfGenerateOnClose:LPBOOL):WINBOOL; external 'advapi32' name 'AccessCheckAndAuditAlarmA';
function ObjectOpenAuditAlarmA(SubsystemName:LPCSTR; HandleId:LPVOID; ObjectTypeName:LPSTR; ObjectName:LPSTR; pSecurityDescriptor:PSECURITY_DESCRIPTOR;ClientToken:HANDLE; DesiredAccess:DWORD; GrantedAccess:DWORD;
  Privileges:PPRIVILEGE_SET; ObjectCreation:WINBOOL;AccessGranted:WINBOOL; GenerateOnClose:LPBOOL):WINBOOL; external 'advapi32' name 'ObjectOpenAuditAlarmA';
function ObjectPrivilegeAuditAlarmA(SubsystemName:LPCSTR; HandleId:LPVOID; ClientToken:HANDLE; DesiredAccess:DWORD; Privileges:PPRIVILEGE_SET;AccessGranted:WINBOOL):WINBOOL; external 'advapi32' name 'ObjectPrivilegeAuditAlarmA';
function ObjectCloseAuditAlarmA(SubsystemName:LPCSTR; HandleId:LPVOID; GenerateOnClose:WINBOOL):WINBOOL; external 'advapi32' name 'ObjectCloseAuditAlarmA';
function PrivilegedServiceAuditAlarmA(SubsystemName:LPCSTR; ServiceName:LPCSTR; ClientToken:HANDLE; Privileges:PPRIVILEGE_SET; AccessGranted:WINBOOL):WINBOOL; external 'advapi32' name 'PrivilegedServiceAuditAlarmA';
function SetFileSecurityA(lpFileName:LPCSTR; SecurityInformation:SECURITY_INFORMATION; pSecurityDescriptor:PSECURITY_DESCRIPTOR):WINBOOL; external 'advapi32' name 'SetFileSecurityA';
function GetFileSecurityA(lpFileName:LPCSTR; RequestedInformation:SECURITY_INFORMATION; pSecurityDescriptor:PSECURITY_DESCRIPTOR; nLength:DWORD; lpnLengthNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'GetFileSecurityA';
function FindFirstChangeNotificationA(lpPathName:LPCSTR; bWatchSubtree:WINBOOL; dwNotifyFilter:DWORD):HANDLE; external 'kernel32' name 'FindFirstChangeNotificationA';
function IsBadStringPtrA(lpsz:LPCSTR; ucchMax:UINT):WINBOOL; external 'kernel32' name 'IsBadStringPtrA';
function LookupAccountSidA(lpSystemName:LPCSTR; Sid:PSID; Name:LPSTR; cbName:LPDWORD; ReferencedDomainName:LPSTR;cbReferencedDomainName:LPDWORD; peUse:PSID_NAME_USE):WINBOOL; external 'advapi32' name 'LookupAccountSidA';
function LookupAccountNameA(lpSystemName:LPCSTR; lpAccountName:LPCSTR; Sid:PSID; cbSid:LPDWORD; ReferencedDomainName:LPSTR;cbReferencedDomainName:LPDWORD; peUse:PSID_NAME_USE):WINBOOL; external 'advapi32' name 'LookupAccountNameA';
function LookupPrivilegeValueA(lpSystemName:LPCSTR; lpName:LPCSTR; lpLuid:PLUID):WINBOOL; external 'advapi32' name 'LookupPrivilegeValueA';
function LookupPrivilegeNameA(lpSystemName:LPCSTR; lpLuid:PLUID; lpName:LPSTR; cbName:LPDWORD):WINBOOL; external 'advapi32' name 'LookupPrivilegeNameA';
function LookupPrivilegeDisplayNameA(lpSystemName:LPCSTR; lpName:LPCSTR; lpDisplayName:LPSTR; cbDisplayName:LPDWORD; lpLanguageId:LPDWORD):WINBOOL; external 'advapi32' name 'LookupPrivilegeDisplayNameA';
function BuildCommDCBA(lpDef:LPCSTR; lpDCB:LPDCB):WINBOOL; external 'kernel32' name 'BuildCommDCBA';
function BuildCommDCBAndTimeoutsA(lpDef:LPCSTR; lpDCB:LPDCB; lpCommTimeouts:LPCOMMTIMEOUTS):WINBOOL; external 'kernel32' name 'BuildCommDCBAndTimeoutsA';
function CommConfigDialogA(lpszName:LPCSTR; hWnd:HWND; lpCC:LPCOMMCONFIG):WINBOOL; external 'kernel32' name 'CommConfigDialogA';
function GetDefaultCommConfigA(lpszName:LPCSTR; lpCC:LPCOMMCONFIG; lpdwSize:LPDWORD):WINBOOL; external 'kernel32' name 'GetDefaultCommConfigA';
function SetDefaultCommConfigA(lpszName:LPCSTR; lpCC:LPCOMMCONFIG; dwSize:DWORD):WINBOOL; external 'kernel32' name 'SetDefaultCommConfigA';
function GetComputerNameA(lpBuffer:LPSTR; nSize:LPDWORD):WINBOOL; external 'kernel32' name 'GetComputerNameA';
function SetComputerNameA(lpComputerName:LPCSTR):WINBOOL; external 'kernel32' name 'SetComputerNameA';
function GetUserNameA(lpBuffer:LPSTR; nSize:LPDWORD):WINBOOL; external 'advapi32' name 'GetUserNameA';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function wvsprintfA(_para1:LPSTR; _para2:LPCSTR; arglist:va_list):longint; stdcall; external 'user32' name 'wvsprintfA';
=======
function wvsprintfA(_para1:LPSTR; _para2:LPCSTR; arglist:va_list):longint; cdecl; external 'user32' name 'wvsprintfA';
>>>>>>> graemeg/cpstrnew
=======
function wvsprintfA(_para1:LPSTR; _para2:LPCSTR; arglist:va_list):longint; cdecl; external 'user32' name 'wvsprintfA';
>>>>>>> graemeg/cpstrnew
=======
function wvsprintfA(_para1:LPSTR; _para2:LPCSTR; arglist:va_list):longint; cdecl; external 'user32' name 'wvsprintfA';
>>>>>>> graemeg/cpstrnew
=======
function wvsprintfA(_para1:LPSTR; _para2:LPCSTR; arglist:va_list):longint; cdecl; external 'user32' name 'wvsprintfA';
>>>>>>> origin/cpstrnew
function wsprintfA(_para1:LPSTR; _para2:LPCSTR; const args:array of const):longint; cdecl; external 'user32' name 'wsprintfA';
function wsprintfA(_para1:LPSTR; _para2:LPCSTR):longint; cdecl; external 'user32' name 'wsprintfA';
function wsprintf(_para1:LPSTR; _para2:LPCSTR; const args:array of const):longint; cdecl; external 'user32' name 'wsprintfA';
function wsprintf(_para1:LPSTR; _para2:LPCSTR):longint; cdecl; external 'user32' name 'wsprintfA';
function LoadKeyboardLayoutA(pwszKLID:LPCSTR; Flags:UINT):HKL; external 'user32' name 'LoadKeyboardLayoutA';
function GetKeyboardLayoutNameA(pwszKLID:LPSTR):WINBOOL; external 'user32' name 'GetKeyboardLayoutNameA';
function CreateDesktopA(lpszDesktop:LPSTR; lpszDevice:LPSTR; pDevmode:LPDEVMODE; dwFlags:DWORD; dwDesiredAccess:DWORD;lpsa:LPSECURITY_ATTRIBUTES):HDESK; external 'user32' name 'CreateDesktopA';
function OpenDesktopA(lpszDesktop:LPSTR; dwFlags:DWORD; fInherit:WINBOOL; dwDesiredAccess:DWORD):HDESK; external 'user32' name 'OpenDesktopA';
function EnumDesktopsA(hwinsta:HWINSTA; lpEnumFunc:DESKTOPENUMPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumDesktopsA';
function CreateWindowStationA(lpwinsta:LPSTR; dwReserved:DWORD; dwDesiredAccess:DWORD; lpsa:LPSECURITY_ATTRIBUTES):HWINSTA; external 'user32' name 'CreateWindowStationA';
function OpenWindowStationA(lpszWinSta:LPSTR; fInherit:WINBOOL; dwDesiredAccess:DWORD):HWINSTA; external 'user32' name 'OpenWindowStationA';
function EnumWindowStationsA(lpEnumFunc:ENUMWINDOWSTATIONPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumWindowStationsA';
function GetUserObjectInformationA(hObj:HANDLE; nIndex:longint; pvInfo:PVOID; nLength:DWORD; lpnLengthNeeded:LPDWORD):WINBOOL; external 'user32' name 'GetUserObjectInformationA';
function SetUserObjectInformationA(hObj:HANDLE; nIndex:longint; pvInfo:PVOID; nLength:DWORD):WINBOOL; external 'user32' name 'SetUserObjectInformationA';
function RegisterWindowMessageA(lpString:LPCSTR):UINT; external 'user32' name 'RegisterWindowMessageA';
function GetMessageA(lpMsg:LPMSG; hWnd:HWND; wMsgFilterMin:UINT; wMsgFilterMax:UINT):WINBOOL; external 'user32' name 'GetMessageA';
function DispatchMessageA(lpMsg:LPMSG):LONG; external 'user32' name 'DispatchMessageA';
function PeekMessageA(lpMsg:LPMSG; hWnd:HWND; wMsgFilterMin:UINT; wMsgFilterMax:UINT; wRemoveMsg:UINT):WINBOOL; external 'user32' name 'PeekMessageA';
function SendMessageA(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'SendMessageA';
function SendMessageTimeoutA(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM; fuFlags:UINT;uTimeout:UINT; lpdwResult:PDWORD_PTR):LRESULT; external 'user32' name 'SendMessageTimeoutA';
function SendNotifyMessageA(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'SendNotifyMessageA';
function SendMessageCallbackA(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM; lpResultCallBack:SENDASYNCPROC;dwData:ULONG_PTR):WINBOOL; external 'user32' name 'SendMessageCallbackA';
function PostMessageA(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'PostMessageA';
function PostThreadMessageA(idThread:DWORD; Msg:UINT; wParam:WPARAM; lParam:LPARAM):WINBOOL; external 'user32' name 'PostThreadMessageA';
function DefWindowProcA(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefWindowProcA';
function CallWindowProcA(lpPrevWndFunc:WNDPROC; hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'CallWindowProcA';
function RegisterClassA(lpWndClass:LPWNDCLASS):ATOM; external 'user32' name 'RegisterClassA';
function UnregisterClassA(lpClassName:LPCSTR; hInstance:HINST):WINBOOL; external 'user32' name 'UnregisterClassA';
function GetClassInfoA(hInstance:HINST; lpClassName:LPCSTR; lpWndClass:LPWNDCLASS):WINBOOL; external 'user32' name 'GetClassInfoA';
function RegisterClassExA(_para1:LPWNDCLASSEX):ATOM; external 'user32' name 'RegisterClassExA';
function GetClassInfoExA(_para1:HINST; _para2:LPCSTR; _para3:LPWNDCLASSEX):WINBOOL; external 'user32' name 'GetClassInfoExA';
function CreateWindowExA(dwExStyle:DWORD; lpClassName:LPCSTR; lpWindowName:LPCSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
  external 'user32' name 'CreateWindowExA';
function CreateDialogParamA(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):HWND; external 'user32' name 'CreateDialogParamA';
function CreateDialogIndirectParamA(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):HWND; external 'user32' name 'CreateDialogIndirectParamA';
function DialogBoxParamA(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):longint; external 'user32' name 'DialogBoxParamA';
function DialogBoxIndirectParamA(hInstance:HINST; hDialogTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC; dwInitParam:LPARAM):longint; external 'user32' name 'DialogBoxIndirectParamA';
function SetDlgItemTextA(hDlg:HWND; nIDDlgItem:longint; lpString:LPCSTR):WINBOOL; external 'user32' name 'SetDlgItemTextA';
function GetDlgItemTextA(hDlg:HWND; nIDDlgItem:longint; lpString:LPSTR; nMaxCount:longint):UINT; external 'user32' name 'GetDlgItemTextA';
function SendDlgItemMessageA(hDlg:HWND; nIDDlgItem:longint; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LONG; external 'user32' name 'SendDlgItemMessageA';
function DefDlgProcA(hDlg:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefDlgProcA';
function CallMsgFilterA(lpMsg:LPMSG; nCode:longint):WINBOOL; external 'user32' name 'CallMsgFilterA';
function RegisterClipboardFormatA(lpszFormat:LPCSTR):UINT; external 'user32' name 'RegisterClipboardFormatA';
function GetClipboardFormatNameA(format:UINT; lpszFormatName:LPSTR; cchMaxCount:longint):longint; external 'user32' name 'GetClipboardFormatNameA';
function CharToOemA(lpszSrc:LPCSTR; lpszDst:LPSTR):WINBOOL; external 'user32' name 'CharToOemA';
function OemToCharA(lpszSrc:LPCSTR; lpszDst:LPSTR):WINBOOL; external 'user32' name 'OemToCharA';
function CharToOemBuffA(lpszSrc:LPCSTR; lpszDst:LPSTR; cchDstLength:DWORD):WINBOOL; external 'user32' name 'CharToOemBuffA';
function OemToCharBuffA(lpszSrc:LPCSTR; lpszDst:LPSTR; cchDstLength:DWORD):WINBOOL; external 'user32' name 'OemToCharBuffA';
function CharUpperA(lpsz:LPSTR):LPSTR; external 'user32' name 'CharUpperA';
function CharUpperBuffA(lpsz:LPSTR; cchLength:DWORD):DWORD; external 'user32' name 'CharUpperBuffA';
function CharLowerA(lpsz:LPSTR):LPSTR; external 'user32' name 'CharLowerA';
function CharLowerBuffA(lpsz:LPSTR; cchLength:DWORD):DWORD; external 'user32' name 'CharLowerBuffA';
function CharNextA(lpsz:LPCSTR):LPSTR; external 'user32' name 'CharNextA';
function CharPrevA(lpszStart:LPCSTR; lpszCurrent:LPCSTR):LPSTR; external 'user32' name 'CharPrevA';
function IsCharAlphaA(ch:CHAR):WINBOOL; external 'user32' name 'IsCharAlphaA';
function IsCharAlphaNumericA(ch:CHAR):WINBOOL; external 'user32' name 'IsCharAlphaNumericA';
function IsCharUpperA(ch:CHAR):WINBOOL; external 'user32' name 'IsCharUpperA';
function IsCharLowerA(ch:CHAR):WINBOOL; external 'user32' name 'IsCharLowerA';
function GetKeyNameTextA(lParam:LONG; lpString:LPSTR; nSize:longint):longint; external 'user32' name 'GetKeyNameTextA';
function VkKeyScanA(ch:CHAR):SHORT; external 'user32' name 'VkKeyScanA';
function VkKeyScanExA(ch:CHAR; dwhkl:HKL):SHORT; external 'user32' name 'VkKeyScanExA';
function MapVirtualKeyA(uCode:UINT; uMapType:UINT):UINT; external 'user32' name 'MapVirtualKeyA';
function MapVirtualKeyExA(uCode:UINT; uMapType:UINT; dwhkl:HKL):UINT; external 'user32' name 'MapVirtualKeyExA';
function LoadAcceleratorsA(hInstance:HINST; lpTableName:LPCSTR):HACCEL; external 'user32' name 'LoadAcceleratorsA';
function CreateAcceleratorTableA(_para1:LPACCEL; _para2:longint):HACCEL; external 'user32' name 'CreateAcceleratorTableA';
function CopyAcceleratorTableA(hAccelSrc:HACCEL; lpAccelDst:LPACCEL; cAccelEntries:longint):longint; external 'user32' name 'CopyAcceleratorTableA';
function TranslateAcceleratorA(hWnd:HWND; hAccTable:HACCEL; lpMsg:LPMSG):longint; external 'user32' name 'TranslateAcceleratorA';
function LoadMenuA(hInstance:HINST; lpMenuName:LPCSTR):HMENU; external 'user32' name 'LoadMenuA';
function LoadMenuIndirectA(lpMenuTemplate:LPMENUTEMPLATE):HMENU; external 'user32' name 'LoadMenuIndirectA';
function ChangeMenuA(hMenu:HMENU; cmd:UINT; lpszNewItem:LPCSTR; cmdInsert:UINT; flags:UINT):WINBOOL; external 'user32' name 'ChangeMenuA';
function GetMenuStringA(hMenu:HMENU; uIDItem:UINT; lpString:LPSTR; nMaxCount:longint; uFlag:UINT):longint; external 'user32' name 'GetMenuStringA';
function InsertMenuA(hMenu:HMENU; uPosition:UINT; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCSTR):WINBOOL; external 'user32' name 'InsertMenuA';
function AppendMenuA(hMenu:HMENU; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCSTR):WINBOOL; external 'user32' name 'AppendMenuA';
function ModifyMenuA(hMnu:HMENU; uPosition:UINT; uFlags:UINT; uIDNewItem:UINT; lpNewItem:LPCSTR):WINBOOL; external 'user32' name 'ModifyMenuA';
function InsertMenuItemA(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPCMENUITEMINFO):WINBOOL; external 'user32' name 'InsertMenuItemA';
function GetMenuItemInfoA(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPMENUITEMINFO):WINBOOL; external 'user32' name 'GetMenuItemInfoA';
function SetMenuItemInfoA(_para1:HMENU; _para2:UINT; _para3:WINBOOL; _para4:LPCMENUITEMINFO):WINBOOL; external 'user32' name 'SetMenuItemInfoA';
function DrawTextA(hDC:HDC; lpString:LPCSTR; nCount:longint; lpRect:LPRECT; uFormat:UINT):longint; external 'user32' name 'DrawTextA';
function DrawTextExA(_para1:HDC; _para2:LPSTR; _para3:longint; _para4:LPRECT; _para5:UINT;_para6:LPDRAWTEXTPARAMS):longint; external 'user32' name 'DrawTextExA';
function GrayStringA(hDC:HDC; hBrush:HBRUSH; lpOutputFunc:GRAYSTRINGPROC; lpData:LPARAM; nCount:longint;X:longint; Y:longint; nWidth:longint; nHeight:longint):WINBOOL; external 'user32' name 'GrayStringA';
function DrawStateA(_para1:HDC; _para2:HBRUSH; _para3:DRAWSTATEPROC; _para4:LPARAM; _para5:WPARAM;_para6:longint; _para7:longint; _para8:longint; _para9:longint; _para10:UINT):WINBOOL; external 'user32' name 'DrawStateA';
function TabbedTextOutA(hDC:HDC; X:longint; Y:longint; lpString:LPCSTR; nCount:longint;nTabPositions:longint; lpnTabStopPositions:LPINT; nTabOrigin:longint):LONG; external 'user32' name 'TabbedTextOutA';
function GetTabbedTextExtentA(hDC:HDC; lpString:LPCSTR; nCount:longint; nTabPositions:longint; lpnTabStopPositions:LPINT):DWORD; external 'user32' name 'GetTabbedTextExtentA';
function SetPropA(hWnd:HWND; lpString:LPCSTR; hData:HANDLE):WINBOOL; external 'user32' name 'SetPropA';
function GetPropA(hWnd:HWND; lpString:LPCSTR):HANDLE; external 'user32' name 'GetPropA';
function RemovePropA(hWnd:HWND; lpString:LPCSTR):HANDLE; external 'user32' name 'RemovePropA';
function EnumPropsExA(hWnd:HWND; lpEnumFunc:PROPENUMPROCEXA; lParam:LPARAM):longint; external 'user32' name 'EnumPropsExA';
function EnumPropsA(hWnd:HWND; lpEnumFunc:PROPENUMPROCA):longint; external 'user32' name 'EnumPropsA';
function SetWindowTextA(hWnd:HWND; lpString:LPCSTR):WINBOOL; external 'user32' name 'SetWindowTextA';
function GetWindowTextA(hWnd:HWND; lpString:LPSTR; nMaxCount:longint):longint; external 'user32' name 'GetWindowTextA';
function GetWindowTextLengthA(hWnd:HWND):longint; external 'user32' name 'GetWindowTextLengthA';
function MessageBoxA(hWnd:HWND; lpText:LPCSTR; lpCaption:LPCSTR; uType:UINT):longint; external 'user32' name 'MessageBoxA';
function MessageBoxExA(hWnd:HWND; lpText:LPCSTR; lpCaption:LPCSTR; uType:UINT; wLanguageId:WORD):longint; external 'user32' name 'MessageBoxExA';
function MessageBoxIndirectA(_para1:LPMSGBOXPARAMS):longint; external 'user32' name 'MessageBoxIndirectA';
function GetWindowLongA(hWnd:HWND; nIndex:longint):LONG; external 'user32' name 'GetWindowLongA';
function SetWindowLongA(hWnd:HWND; nIndex:longint; dwNewLong:LONG):LONG; external 'user32' name 'SetWindowLongA';
function GetClassLongA(hWnd:HWND; nIndex:longint):DWORD; external 'user32' name 'GetClassLongA';
function SetClassLongA(hWnd:HWND; nIndex:longint; dwNewLong:LONG):DWORD; external 'user32' name 'SetClassLongA';
{$ifdef cpu64}
function GetWindowLongPtrA(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetWindowLongPtrA';
function SetWindowLongPtrA(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetWindowLongPtrA';
function GetClassLongPtrA(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetClassLongPtrA';
function SetClassLongPtrA(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetClassLongPtrA';
{$else}
function GetWindowLongPtrA(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetWindowLongA';
function SetWindowLongPtrA(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetWindowLongA';
function GetClassLongPtrA(hWnd:HWND; nIndex:longint):LONG_PTR; external 'user32' name 'GetClassLongA';
function SetClassLongPtrA(hWnd:HWND; nIndex:longint; dwNewLong:LONG_PTR):LONG_PTR; external 'user32' name 'SetClassLongA';
{$endif}
function FindWindowA(lpClassName:LPCSTR; lpWindowName:LPCSTR):HWND; external 'user32' name 'FindWindowA';
function FindWindowExA(_para1:HWND; _para2:HWND; _para3:LPCSTR; _para4:LPCSTR):HWND; external 'user32' name 'FindWindowExA';
function GetClassNameA(hWnd:HWND; lpClassName:LPSTR; nMaxCount:longint):longint; external 'user32' name 'GetClassNameA';
function SetWindowsHookExA(idHook:longint; lpfn:HOOKPROC; hmod:HINST; dwThreadId:DWORD):HHOOK; external 'user32' name 'SetWindowsHookExA';
function LoadBitmapA(hInstance:HINST; lpBitmapName:LPCSTR):HBITMAP; external 'user32' name 'LoadBitmapA';
function LoadCursorA(hInstance:HINST; lpCursorName:LPCSTR):HCURSOR; external 'user32' name 'LoadCursorA';
function LoadCursorFromFileA(lpFileName:LPCSTR):HCURSOR; external 'user32' name 'LoadCursorFromFileA';
function LoadIconA(hInstance:HINST; lpIconName:LPCSTR):HICON; external 'user32' name 'LoadIconA';
function LoadImageA(_para1:HINST; _para2:LPCSTR; _para3:UINT; _para4:longint; _para5:longint;_para6:UINT):HANDLE; external 'user32' name 'LoadImageA';
function LoadStringA(hInstance:HINST; uID:UINT; lpBuffer:LPSTR; nBufferMax:longint):longint; external 'user32' name 'LoadStringA';
function IsDialogMessageA(hDlg:HWND; lpMsg:LPMSG):WINBOOL; external 'user32' name 'IsDialogMessageA';
function DlgDirListA(hDlg:HWND; lpPathSpec:LPSTR; nIDListBox:longint; nIDStaticPath:longint; uFileType:UINT):longint; external 'user32' name 'DlgDirListA';
function DlgDirSelectExA(hDlg:HWND; lpString:LPSTR; nCount:longint; nIDListBox:longint):WINBOOL; external 'user32' name 'DlgDirSelectExA';
function DlgDirListComboBoxA(hDlg:HWND; lpPathSpec:LPSTR; nIDComboBox:longint; nIDStaticPath:longint; uFiletype:UINT):longint; external 'user32' name 'DlgDirListComboBoxA';
function DlgDirSelectComboBoxExA(hDlg:HWND; lpString:LPSTR; nCount:longint; nIDComboBox:longint):WINBOOL; external 'user32' name 'DlgDirSelectComboBoxExA';
function DefFrameProcA(hWnd:HWND; hWndMDIClient:HWND; uMsg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefFrameProcA';
function DefMDIChildProcA(hWnd:HWND; uMsg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'DefMDIChildProcA';
function CreateMDIWindowA(lpClassName:LPSTR; lpWindowName:LPSTR; dwStyle:DWORD; X:longint; Y:longint;nWidth:longint; nHeight:longint; hWndParent:HWND; hInstance:HINST; lParam:LPARAM):HWND; external 'user32' name 'CreateMDIWindowA';
function WinHelpA(hWndMain:HWND; lpszHelp:LPCSTR; uCommand:UINT; dwData:DWORD):WINBOOL; external 'user32' name 'WinHelpA';
function ChangeDisplaySettingsA(lpDevMode:LPDEVMODE; dwFlags:DWORD):LONG; external 'user32' name 'ChangeDisplaySettingsA';
function EnumDisplaySettingsA(lpszDeviceName:LPCSTR; iModeNum:DWORD; lpDevMode:LPDEVMODE):WINBOOL; external 'user32' name 'EnumDisplaySettingsA';
function SystemParametersInfoA(uiAction:UINT; uiParam:UINT; pvParam:PVOID; fWinIni:UINT):WINBOOL; external 'user32' name 'SystemParametersInfoA';
function AddFontResourceA(_para1:LPCSTR):longint; external 'gdi32' name 'AddFontResourceA';
function CopyMetaFileA(_para1:HMETAFILE; _para2:LPCSTR):HMETAFILE; external 'gdi32' name 'CopyMetaFileA';
function CreateFontA(_para1:longint; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:DWORD; _para7:DWORD; _para8:DWORD; _para9:DWORD; _para10:DWORD;_para11:DWORD; _para12:DWORD; _para13:DWORD; _para14:LPCSTR):HFONT;
external 'gdi32' name 'CreateFontA';
function CreateFontIndirectA(_para1:LPLOGFONT):HFONT; external 'gdi32' name 'CreateFontIndirectA';
function CreateFontIndirectA(var _para1:LOGFONT):HFONT; external 'gdi32' name 'CreateFontIndirectA';
function CreateICA(_para1:LPCSTR; _para2:LPCSTR; _para3:LPCSTR; _para4:LPDEVMODE):HDC; external 'gdi32' name 'CreateICA';
function CreateMetaFileA(_para1:LPCSTR):HDC; external 'gdi32' name 'CreateMetaFileA';
function CreateScalableFontResourceA(_para1:DWORD; _para2:LPCSTR; _para3:LPCSTR; _para4:LPCSTR):WINBOOL; external 'gdi32' name 'CreateScalableFontResourceA';
function EnumFontFamiliesExA(_para1:HDC; _para2:LPLOGFONT; _para3:FONTENUMEXPROC; _para4:LPARAM; _para5:DWORD):longint; external 'gdi32' name 'EnumFontFamiliesExA';
function EnumFontFamiliesA(_para1:HDC; _para2:LPCSTR; _para3:FONTENUMPROC; _para4:LPARAM):longint; external 'gdi32' name 'EnumFontFamiliesA';
function EnumFontsA(_para1:HDC; _para2:LPCSTR; _para3:ENUMFONTSPROC; _para4:LPARAM):longint; external 'gdi32' name 'EnumFontsA';
function EnumFontsA(_para1:HDC; _para2:LPCSTR; _para3:ENUMFONTSPROC; _para4:pointer):longint; external 'gdi32' name 'EnumFontsA';
function GetCharWidthA(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPINT):WINBOOL; external 'gdi32' name 'GetCharWidthA';
function GetCharWidth32A(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPINT):WINBOOL; external 'gdi32' name 'GetCharWidth32A';
function GetCharWidthFloatA(_para1:HDC; _para2:UINT; _para3:UINT; _para4:PSingle):WINBOOL; external 'gdi32' name 'GetCharWidthFloatA';
function GetCharABCWidthsA(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPABC):WINBOOL; external 'gdi32' name 'GetCharABCWidthsA';
function GetCharABCWidthsFloatA(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPABCFLOAT):WINBOOL; external 'gdi32' name 'GetCharABCWidthsFloatA';
function GetGlyphOutlineA(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPGLYPHMETRICS; _para5:DWORD;_para6:LPVOID; _para7:PMAT2):DWORD; external 'gdi32' name 'GetGlyphOutlineA';
function GetMetaFileA(_para1:LPCSTR):HMETAFILE; external 'gdi32' name 'GetMetaFileA';
function GetOutlineTextMetricsA(_para1:HDC; _para2:UINT; _para3:LPOUTLINETEXTMETRIC):UINT; external 'gdi32' name 'GetOutlineTextMetricsA';
function GetTextExtentPointA(_para1:HDC; _para2:LPCSTR; _para3:longint; _para4:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentPointA';
function GetTextExtentPoint32A(_para1:HDC; _para2:LPCSTR; _para3:longint; _para4:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentPoint32A';
function GetTextExtentExPointA(_para1:HDC; _para2:LPCSTR; _para3:longint; _para4:longint; _para5:LPINT;_para6:LPINT; _para7:LPSIZE):WINBOOL; external 'gdi32' name 'GetTextExtentExPointA';
function GetCharacterPlacementA(_para1:HDC; _para2:LPCSTR; _para3:longint; _para4:longint; _para5:LPGCP_RESULTS;_para6:DWORD):DWORD; external 'gdi32' name 'GetCharacterPlacementA';
function ResetDCA(_para1:HDC; _para2:LPDEVMODE):HDC; external 'gdi32' name 'ResetDCA';
function RemoveFontResourceA(_para1:LPCSTR):WINBOOL; external 'gdi32' name 'RemoveFontResourceA';
function CopyEnhMetaFileA(_para1:HENHMETAFILE; _para2:LPCSTR):HENHMETAFILE; external 'gdi32' name 'CopyEnhMetaFileA';
function CreateEnhMetaFileA(_para1:HDC; _para2:LPCSTR; _para3:LPRECT; _para4:LPCSTR):HDC; external 'gdi32' name 'CreateEnhMetaFileA';
function GetEnhMetaFileA(_para1:LPCSTR):HENHMETAFILE; external 'gdi32' name 'GetEnhMetaFileA';
function GetEnhMetaFileDescriptionA(_para1:HENHMETAFILE; _para2:UINT; _para3:LPSTR):UINT; external 'gdi32' name 'GetEnhMetaFileDescriptionA';
function GetTextMetricsA(_para1:HDC; _para2:LPTEXTMETRIC):WINBOOL; external 'gdi32' name 'GetTextMetricsA';
function StartDocA(_para1:HDC; _para2:PDOCINFO):longint; external 'gdi32' name 'StartDocA';
function GetObjectA(_para1:HGDIOBJ; _para2:longint; _para3:LPVOID):longint; external 'gdi32' name 'GetObjectA';
function TextOutA(_para1:HDC; _para2:longint; _para3:longint; _para4:LPCSTR; _para5:longint):WINBOOL; external 'gdi32' name 'TextOutA';
function ExtTextOutA(_para1:HDC; _para2:longint; _para3:longint; _para4:UINT; _para5:LPRECT;_para6:LPCSTR; _para7:UINT; _para8:LPINT):WINBOOL; external 'gdi32' name 'ExtTextOutA';
function PolyTextOutA(_para1:HDC; _para2:PPOLYTEXT; _para3:longint):WINBOOL; external 'gdi32' name 'PolyTextOutA';
function GetTextFaceA(_para1:HDC; _para2:longint; _para3:LPSTR):longint; external 'gdi32' name 'GetTextFaceA';
function GetKerningPairsA(_para1:HDC; _para2:DWORD; _para3:LPKERNINGPAIR):DWORD; external 'gdi32' name 'GetKerningPairsA';
function CreateColorSpaceA(_para1:LPLOGCOLORSPACE):HCOLORSPACE; external 'gdi32' name 'CreateColorSpaceA';
function GetLogColorSpaceA(_para1:HCOLORSPACE; _para2:LPLOGCOLORSPACE; _para3:DWORD):WINBOOL; external 'gdi32' name 'GetLogColorSpaceA';
function GetICMProfileA(_para1:HDC; _para2:DWORD; _para3:LPSTR):WINBOOL; external 'gdi32' name 'GetICMProfileA';
function SetICMProfileA(_para1:HDC; _para2:LPSTR):WINBOOL; external 'gdi32' name 'SetICMProfileA';
function UpdateICMRegKeyA(_para1:DWORD; _para2:DWORD; _para3:LPSTR; _para4:UINT):WINBOOL; external 'gdi32' name 'UpdateICMRegKeyA';
<<<<<<< HEAD
function EnumICMProfilesA(_para1:HDC; _para2:ICMENUMPROCA; _para3:LPARAM):longint; external 'gdi32' name 'EnumICMProfilesA';
=======
function EnumICMProfilesA(_para1:HDC; _para2:ICMENUMPROC; _para3:LPARAM):longint; external 'gdi32' name 'EnumICMProfilesA';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{
function PropertySheetA(lppsph:LPCPROPSHEETHEADER):longint; external 'comctl32' name 'PropertySheetA';
function ImageList_LoadImageA(hi:HINST; lpbmp:LPCSTR; cx:longint; cGrow:longint; crMask:COLORREF;uType:UINT; uFlags:UINT):HIMAGELIST; external 'comctl32' name 'ImageList_LoadImageA';

function CreateStatusWindowA(style:LONG; lpszText:LPCSTR; hwndParent:HWND; wID:UINT):HWND; external 'comctl32' name 'CreateStatusWindowA';
procedure DrawStatusTextA(hDC:HDC; lprc:LPRECT; pszText:LPCSTR; uFlags:UINT); external 'comctl32' name 'DrawStatusTextA';
function GetOpenFileNameA(_para1:LPOPENFILENAME):WINBOOL; external 'comdlg32' name 'GetOpenFileNameA';
function GetSaveFileNameA(_para1:LPOPENFILENAME):WINBOOL; external 'comdlg32' name 'GetSaveFileNameA';
function GetFileTitleA(_para1:LPCSTR; _para2:LPSTR; _para3:WORD):integer; external 'comdlg32' name 'GetFileTitleA';
function ChooseColorA(_para1:LPCHOOSECOLOR):WINBOOL; external 'comdlg32' name 'ChooseColorA';
function FindTextA(_para1:LPFINDREPLACE):HWND; external 'comdlg32' name 'FindTextA';
function ReplaceTextA(_para1:LPFINDREPLACE):HWND; external 'comdlg32' name 'ReplaceTextA';
function ChooseFontA(_para1:LPCHOOSEFONT):WINBOOL; external 'comdlg32' name 'ChooseFontA';
function PrintDlgA(_para1:LPPRINTDLG):WINBOOL; external 'comdlg32' name 'PrintDlgA';
function PageSetupDlgA(_para1:LPPAGESETUPDLG):WINBOOL; external 'comdlg32' name 'PageSetupDlgA';
}
function CreateProcessA(lpApplicationName:LPCSTR; lpCommandLine:LPSTR; lpProcessAttributes:LPSECURITY_ATTRIBUTES; lpThreadAttributes:LPSECURITY_ATTRIBUTES; bInheritHandles:WINBOOL;dwCreationFlags:DWORD; lpEnvironment:LPVOID;
  lpCurrentDirectory:LPCSTR; lpStartupInfo:LPSTARTUPINFOA; lpProcessInformation:LPPROCESS_INFORMATION):WINBOOL; external 'kernel32' name 'CreateProcessA';
procedure GetStartupInfoA(lpStartupInfo:LPSTARTUPINFOA); external 'kernel32' name 'GetStartupInfoA';
function FindFirstFileA(lpFileName:LPCSTR; lpFindFileData:LPWIN32_FIND_DATA):HANDLE; external 'kernel32' name 'FindFirstFileA';
function FindNextFileA(hFindFile:HANDLE; lpFindFileData:LPWIN32_FIND_DATA):WINBOOL; external 'kernel32' name 'FindNextFileA';
function GetVersionExA(VersionInformation:LPOSVERSIONINFO):WINBOOL; external 'kernel32' name 'GetVersionExA';
function CreateWindowA(lpClassName:LPCSTR; lpWindowName:LPCSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
function CreateDialogA(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
function CreateDialogIndirectA(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
function DialogBoxA(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
function DialogBoxIndirectA(hInstance:HINST; hDialogTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
function CreateDCA(_para1:LPCSTR; _para2:LPCSTR; _para3:LPCSTR; _para4:pDEVMODE):HDC; external 'gdi32' name 'CreateDCA';
function VerInstallFileA(uFlags:DWORD; szSrcFileName:LPSTR; szDestFileName:LPSTR; szSrcDir:LPSTR; szDestDir:LPSTR;szCurDir:LPSTR; szTmpFile:LPSTR; lpuTmpFileLen:PUINT):DWORD; external 'version' name 'VerInstallFileA';
function GetFileVersionInfoSizeA(lptstrFilename:LPSTR; lpdwHandle:LPDWORD):DWORD; external 'version' name 'GetFileVersionInfoSizeA';
function GetFileVersionInfoA(lptstrFilename:LPSTR; dwHandle:DWORD; dwLen:DWORD; lpData:LPVOID):WINBOOL; external 'version' name 'GetFileVersionInfoA';
function VerLanguageNameA(wLang:DWORD; szLang:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'VerLanguageNameA';
function VerQueryValueA(pBlock:LPVOID; lpSubBlock:LPSTR; lplpBuffer:LPVOID; puLen:PUINT):WINBOOL; external 'version' name 'VerQueryValueA';
function VerFindFileA(uFlags:DWORD; szFileName:LPSTR; szWinDir:LPSTR; szAppDir:LPSTR; szCurDir:LPSTR;lpuCurDirLen:PUINT; szDestDir:LPSTR; lpuDestDirLen:PUINT):DWORD; external 'version' name 'VerFindFileA';
function RegConnectRegistryA(lpMachineName:LPSTR; hKey:HKEY; phkResult:PHKEY):LONG; external 'advapi32' name 'RegConnectRegistryA';
function RegCreateKeyA(hKey:HKEY; lpSubKey:LPCSTR; phkResult:PHKEY):LONG; external 'advapi32' name 'RegCreateKeyA';
function RegCreateKeyExA(hKey:HKEY; lpSubKey:LPCSTR; Reserved:DWORD; lpClass:LPSTR; dwOptions:DWORD;samDesired:REGSAM; lpSecurityAttributes:LPSECURITY_ATTRIBUTES; phkResult:PHKEY; lpdwDisposition:LPDWORD):LONG;
  external 'advapi32' name 'RegCreateKeyExA';
function RegDeleteKeyA(hKey:HKEY; lpSubKey:LPCSTR):LONG; external 'advapi32' name 'RegDeleteKeyA';
function RegDeleteValueA(hKey:HKEY; lpValueName:LPCSTR):LONG; external 'advapi32' name 'RegDeleteValueA';
function RegEnumKeyA(hKey:HKEY; dwIndex:DWORD; lpName:LPSTR; cbName:DWORD):LONG; external 'advapi32' name 'RegEnumKeyA';
function RegEnumKeyExA(hKey:HKEY; dwIndex:DWORD; lpName:LPSTR; lpcbName:LPDWORD; lpReserved:LPDWORD;lpClass:LPSTR; lpcbClass:LPDWORD; lpftLastWriteTime:PFILETIME):LONG; external 'advapi32' name 'RegEnumKeyExA';
function RegEnumValueA(hKey:HKEY; dwIndex:DWORD; lpValueName:LPSTR; lpcbValueName:LPDWORD; lpReserved:LPDWORD;lpType:LPDWORD; lpData:LPBYTE; lpcbData:LPDWORD):LONG; external 'advapi32' name 'RegEnumValueA';
function RegLoadKeyA(hKey:HKEY; lpSubKey:LPCSTR; lpFile:LPCSTR):LONG; external 'advapi32' name 'RegLoadKeyA';
function RegOpenKeyA(hKey:HKEY; lpSubKey:LPCSTR; phkResult:PHKEY):LONG; external 'advapi32' name 'RegOpenKeyA';
function RegOpenKeyExA(hKey:HKEY; lpSubKey:LPCSTR; ulOptions:DWORD; samDesired:REGSAM; phkResult:PHKEY):LONG; external 'advapi32' name 'RegOpenKeyExA';
function RegQueryInfoKeyA(hKey:HKEY; lpClass:LPSTR; lpcbClass:LPDWORD; lpReserved:LPDWORD; lpcSubKeys:LPDWORD;lpcbMaxSubKeyLen:LPDWORD; lpcbMaxClassLen:LPDWORD; lpcValues:LPDWORD; lpcbMaxValueNameLen:LPDWORD;
  lpcbMaxValueLen:LPDWORD;lpcbSecurityDescriptor:LPDWORD; lpftLastWriteTime:PFILETIME):LONG; external 'advapi32' name 'RegQueryInfoKeyA';
function RegQueryValueA(hKey:HKEY; lpSubKey:LPCSTR; lpValue:LPSTR; lpcbValue:PLONG):LONG; external 'advapi32' name 'RegQueryValueA';
function RegQueryMultipleValuesA(hKey:HKEY; val_list:PVALENT; num_vals:DWORD; lpValueBuf:LPSTR; ldwTotsize:LPDWORD):LONG; external 'advapi32' name 'RegQueryMultipleValuesA';
function RegQueryValueExA(hKey:HKEY; lpValueName:LPCSTR; lpReserved:LPDWORD; lpType:LPDWORD; lpData:LPBYTE;lpcbData:LPDWORD):LONG; external 'advapi32' name 'RegQueryValueExA';
function RegReplaceKeyA(hKey:HKEY; lpSubKey:LPCSTR; lpNewFile:LPCSTR; lpOldFile:LPCSTR):LONG; external 'advapi32' name 'RegReplaceKeyA';
function RegRestoreKeyA(hKey:HKEY; lpFile:LPCSTR; dwFlags:DWORD):LONG; external 'advapi32' name 'RegRestoreKeyA';
function RegSaveKeyA(hKey:HKEY; lpFile:LPCSTR; lpSecurityAttributes:LPSECURITY_ATTRIBUTES):LONG; external 'advapi32' name 'RegSaveKeyA';
function RegSetValueA(hKey:HKEY; lpSubKey:LPCSTR; dwType:DWORD; lpData:LPCSTR; cbData:DWORD):LONG; external 'advapi32' name 'RegSetValueA';
function RegSetValueExA(hKey:HKEY; lpValueName:LPCSTR; Reserved:DWORD; dwType:DWORD; lpData:LPBYTE;cbData:DWORD):LONG; external 'advapi32' name 'RegSetValueExA';
function RegUnLoadKeyA(hKey:HKEY; lpSubKey:LPCSTR):LONG; external 'advapi32' name 'RegUnLoadKeyA';
function InitiateSystemShutdownA(lpMachineName:LPSTR; lpMessage:LPSTR; dwTimeout:DWORD; bForceAppsClosed:WINBOOL; bRebootAfterShutdown:WINBOOL):WINBOOL; external 'advapi32' name 'InitiateSystemShutdownA';
function AbortSystemShutdownA(lpMachineName:LPSTR):WINBOOL; external 'advapi32' name 'AbortSystemShutdownA';
function CompareStringA(Locale:LCID; dwCmpFlags:DWORD; lpString1:LPCSTR; cchCount1:longint; lpString2:LPCSTR;cchCount2:longint):longint; external 'kernel32' name 'CompareStringA';
function LCMapStringA(Locale:LCID; dwMapFlags:DWORD; lpSrcStr:LPCSTR; cchSrc:longint; lpDestStr:LPSTR;cchDest:longint):longint; external 'kernel32' name 'LCMapStringA';
function GetLocaleInfoA(Locale:LCID; LCType:LCTYPE; lpLCData:LPSTR; cchData:longint):longint; external 'kernel32' name 'GetLocaleInfoA';
function SetLocaleInfoA(Locale:LCID; LCType:LCTYPE; lpLCData:LPCSTR):WINBOOL; external 'kernel32' name 'SetLocaleInfoA';
function GetTimeFormatA(Locale:LCID; dwFlags:DWORD; lpTime:LPSYSTEMTIME; lpFormat:LPCSTR; lpTimeStr:LPSTR;cchTime:longint):longint; external 'kernel32' name 'GetTimeFormatA';
function GetDateFormatA(Locale:LCID; dwFlags:DWORD; lpDate:LPSYSTEMTIME; lpFormat:LPCSTR; lpDateStr:LPSTR;cchDate:longint):longint; external 'kernel32' name 'GetDateFormatA';
function GetNumberFormatA(Locale:LCID; dwFlags:DWORD; lpValue:LPCSTR; lpFormat:PNUMBERFMT; lpNumberStr:LPSTR;cchNumber:longint):longint; external 'kernel32' name 'GetNumberFormatA';
function GetCurrencyFormatA(Locale:LCID; dwFlags:DWORD; lpValue:LPCSTR; lpFormat:PCURRENCYFMT; lpCurrencyStr:LPSTR;cchCurrency:longint):longint; external 'kernel32' name 'GetCurrencyFormatA';
function EnumCalendarInfoA(lpCalInfoEnumProc:CALINFO_ENUMPROCA; Locale:LCID; Calendar:CALID; CalType:CALTYPE):WINBOOL; external 'kernel32' name 'EnumCalendarInfoA';
function EnumTimeFormatsA(lpTimeFmtEnumProc:TIMEFMT_ENUMPROCA; Locale:LCID; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumTimeFormatsA';
function EnumDateFormatsA(lpDateFmtEnumProc:DATEFMT_ENUMPROCA; Locale:LCID; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumDateFormatsA';
function GetStringTypeExA(Locale:LCID; dwInfoType:DWORD; lpSrcStr:LPCSTR; cchSrc:longint; lpCharType:LPWORD):WINBOOL; external 'kernel32' name 'GetStringTypeExA';
function GetStringTypeA(Locale:LCID; dwInfoType:DWORD; lpSrcStr:LPCSTR; cchSrc:longint; lpCharType:LPWORD):WINBOOL; external 'kernel32' name 'GetStringTypeA';
function FoldStringA(dwMapFlags:DWORD; lpSrcStr:LPCSTR; cchSrc:longint; lpDestStr:LPSTR; cchDest:longint):longint; external 'kernel32' name 'FoldStringA';
function EnumSystemLocalesA(lpLocaleEnumProc:LOCALE_ENUMPROCA; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumSystemLocalesA';
function EnumSystemCodePagesA(lpCodePageEnumProc:CODEPAGE_ENUMPROCA; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'EnumSystemCodePagesA';
function PeekConsoleInputA(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsRead:LPDWORD):WINBOOL; external 'kernel32' name 'PeekConsoleInputA';
function ReadConsoleInputA(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsRead:LPDWORD):WINBOOL; external 'kernel32' name 'ReadConsoleInputA';
function WriteConsoleInputA(hConsoleInput:HANDLE; lpBuffer:PINPUTRECORD; nLength:DWORD; lpNumberOfEventsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'WriteConsoleInputA';
function ReadConsoleOutputA(hConsoleOutput:HANDLE; lpBuffer:PCHAR_INFO; dwBufferSize:COORD; dwBufferCoord:COORD; lpReadRegion:PSMALL_RECT):WINBOOL; external 'kernel32' name 'ReadConsoleOutputA';
function WriteConsoleOutputA(hConsoleOutput:HANDLE; lpBuffer:PCHAR_INFO; dwBufferSize:COORD; dwBufferCoord:COORD; lpWriteRegion:PSMALL_RECT):WINBOOL; external 'kernel32' name 'WriteConsoleOutputA';
function ReadConsoleOutputCharacterA(hConsoleOutput:HANDLE; lpCharacter:LPSTR; nLength:DWORD; dwReadCoord:COORD; lpNumberOfCharsRead:LPDWORD):WINBOOL; external 'kernel32' name 'ReadConsoleOutputCharacterA';
function WriteConsoleOutputCharacterA(hConsoleOutput:HANDLE; lpCharacter:LPCSTR; nLength:DWORD; dwWriteCoord:COORD; lpNumberOfCharsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'WriteConsoleOutputCharacterA';
function FillConsoleOutputCharacterA(hConsoleOutput:HANDLE; cCharacter:CHAR; nLength:DWORD; dwWriteCoord:COORD; lpNumberOfCharsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'FillConsoleOutputCharacterA';
function ScrollConsoleScreenBufferA(hConsoleOutput:HANDLE; lpScrollRectangle:PSMALL_RECT; lpClipRectangle:PSMALL_RECT; dwDestinationOrigin:COORD; lpFill:PCHAR_INFO):WINBOOL; external 'kernel32' name 'ScrollConsoleScreenBufferA';
function GetConsoleTitleA(lpConsoleTitle:LPSTR; nSize:DWORD):DWORD; external 'kernel32' name 'GetConsoleTitleA';
function SetConsoleTitleA(lpConsoleTitle:LPCSTR):WINBOOL; external 'kernel32' name 'SetConsoleTitleA';
function ReadConsoleA(hConsoleInput:HANDLE; lpBuffer:LPVOID; nNumberOfCharsToRead:DWORD; lpNumberOfCharsRead:LPDWORD; lpReserved:LPVOID):WINBOOL; external 'kernel32' name 'ReadConsoleA';
function WriteConsoleA(hConsoleOutput:HANDLE;lpBuffer:pointer; nNumberOfCharsToWrite:DWORD; lpNumberOfCharsWritten:LPDWORD; lpReserved:LPVOID):WINBOOL; external 'kernel32' name 'WriteConsoleA';
function WNetAddConnectionA(lpRemoteName:LPCSTR; lpPassword:LPCSTR; lpLocalName:LPCSTR):DWORD; external 'mpr' name 'WNetAddConnectionA';
function WNetAddConnection2A(lpNetResource:LPNETRESOURCE; lpPassword:LPCSTR; lpUserName:LPCSTR; dwFlags:DWORD):DWORD; external 'mpr' name 'WNetAddConnection2A';
function WNetAddConnection3A(hwndOwner:HWND; lpNetResource:LPNETRESOURCE; lpPassword:LPCSTR; lpUserName:LPCSTR; dwFlags:DWORD):DWORD; external 'mpr' name 'WNetAddConnection3A';
function WNetCancelConnectionA(lpName:LPCSTR; fForce:WINBOOL):DWORD; external 'mpr' name 'WNetCancelConnectionA';
function WNetCancelConnection2A(lpName:LPCSTR; dwFlags:DWORD; fForce:WINBOOL):DWORD; external 'mpr' name 'WNetCancelConnection2A';
function WNetGetConnectionA(lpLocalName:LPCSTR; lpRemoteName:LPSTR; lpnLength:LPDWORD):DWORD; external 'mpr' name 'WNetGetConnectionA';
function WNetUseConnectionA(hwndOwner:HWND; lpNetResource:LPNETRESOURCE; lpUserID:LPCSTR; lpPassword:LPCSTR; dwFlags:DWORD;lpAccessName:LPSTR; lpBufferSize:LPDWORD; lpResult:LPDWORD):DWORD; external 'mpr' name 'WNetUseConnectionA';
function WNetSetConnectionA(lpName:LPCSTR; dwProperties:DWORD; pvValues:LPVOID):DWORD; external 'mpr' name 'WNetSetConnectionA';
function WNetConnectionDialog1A(lpConnDlgStruct:LPCONNECTDLGSTRUCT):DWORD; external 'mpr' name 'WNetConnectionDialog1A';
function WNetDisconnectDialog1A(lpConnDlgStruct:LPDISCDLGSTRUCT):DWORD; external 'mpr' name 'WNetDisconnectDialog1A';
function WNetOpenEnumA(dwScope:DWORD; dwType:DWORD; dwUsage:DWORD; lpNetResource:LPNETRESOURCE; lphEnum:LPHANDLE):DWORD; external 'mpr' name 'WNetOpenEnumA';
function WNetEnumResourceA(hEnum:HANDLE; lpcCount:LPDWORD; lpBuffer:LPVOID; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetEnumResourceA';
function WNetGetUniversalNameA(lpLocalPath:LPCSTR; dwInfoLevel:DWORD; lpBuffer:LPVOID; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetGetUniversalNameA';
function WNetGetUserA(lpName:LPCSTR; lpUserName:LPSTR; lpnLength:LPDWORD):DWORD; external 'mpr' name 'WNetGetUserA';
function WNetGetProviderNameA(dwNetType:DWORD; lpProviderName:LPSTR; lpBufferSize:LPDWORD):DWORD; external 'mpr' name 'WNetGetProviderNameA';
function WNetGetNetworkInformationA(lpProvider:LPCSTR; lpNetInfoStruct:LPNETINFOSTRUCT):DWORD; external 'mpr' name 'WNetGetNetworkInformationA';
function WNetGetLastErrorA(lpError:LPDWORD; lpErrorBuf:LPSTR; nErrorBufSize:DWORD; lpNameBuf:LPSTR; nNameBufSize:DWORD):DWORD; external 'mpr' name 'WNetGetLastErrorA';
function MultinetGetConnectionPerformanceA(lpNetResource:LPNETRESOURCE; lpNetConnectInfoStruct:LPNETCONNECTINFOSTRUCT):DWORD; external 'mpr' name 'MultinetGetConnectionPerformanceA';
function ChangeServiceConfigA(hService:SC_HANDLE; dwServiceType:DWORD; dwStartType:DWORD; dwErrorControl:DWORD; lpBinaryPathName:LPCSTR;lpLoadOrderGroup:LPCSTR; lpdwTagId:LPDWORD; lpDependencies:LPCSTR; lpServiceStartName:LPCSTR;
  lpPassword:LPCSTR;lpDisplayName:LPCSTR):WINBOOL; external 'advapi32' name 'ChangeServiceConfigA';
function CreateServiceA(hSCManager:SC_HANDLE; lpServiceName:LPCSTR; lpDisplayName:LPCSTR; dwDesiredAccess:DWORD; dwServiceType:DWORD;dwStartType:DWORD; dwErrorControl:DWORD; lpBinaryPathName:LPCSTR; lpLoadOrderGroup:LPCSTR;
  lpdwTagId:LPDWORD;lpDependencies:LPCSTR; lpServiceStartName:LPCSTR; lpPassword:LPCSTR):SC_HANDLE; external 'advapi32' name 'CreateServiceA';
function EnumDependentServicesA(hService:SC_HANDLE; dwServiceState:DWORD; lpServices:LPENUM_SERVICE_STATUS; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD;lpServicesReturned:LPDWORD):WINBOOL; external 'advapi32' name 'EnumDependentServicesA';
function EnumServicesStatusA(hSCManager:SC_HANDLE; dwServiceType:DWORD; dwServiceState:DWORD; lpServices:LPENUM_SERVICE_STATUS; cbBufSize:DWORD;pcbBytesNeeded:LPDWORD; lpServicesReturned:LPDWORD; lpResumeHandle:LPDWORD):WINBOOL;
  external 'advapi32' name 'EnumServicesStatusA';
function GetServiceKeyNameA(hSCManager:SC_HANDLE; lpDisplayName:LPCSTR; lpServiceName:LPSTR; lpcchBuffer:LPDWORD):WINBOOL; external 'advapi32' name 'GetServiceKeyNameA';
function GetServiceDisplayNameA(hSCManager:SC_HANDLE; lpServiceName:LPCSTR; lpDisplayName:LPSTR; lpcchBuffer:LPDWORD):WINBOOL; external 'advapi32' name 'GetServiceDisplayNameA';
function OpenSCManagerA(lpMachineName:LPCSTR; lpDatabaseName:LPCSTR; dwDesiredAccess:DWORD):SC_HANDLE; external 'advapi32' name 'OpenSCManagerA';
function OpenServiceA(hSCManager:SC_HANDLE; lpServiceName:LPCSTR; dwDesiredAccess:DWORD):SC_HANDLE; external 'advapi32' name 'OpenServiceA';
function QueryServiceConfigA(hService:SC_HANDLE; lpServiceConfig:LPQUERY_SERVICE_CONFIG; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'QueryServiceConfigA';
function QueryServiceLockStatusA(hSCManager:SC_HANDLE; lpLockStatus:LPQUERY_SERVICE_LOCK_STATUS; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'QueryServiceLockStatusA';
function RegisterServiceCtrlHandlerA(lpServiceName:LPCSTR; lpHandlerProc:LPHANDLER_FUNCTION):SERVICE_STATUS_HANDLE; external 'advapi32' name 'RegisterServiceCtrlHandlerA';
function StartServiceCtrlDispatcherA(lpServiceStartTable:LPSERVICE_TABLE_ENTRY):WINBOOL; external 'advapi32' name 'StartServiceCtrlDispatcherA';
function StartServiceA(hService:SC_HANDLE; dwNumServiceArgs:DWORD; lpServiceArgVectors:LPPCSTR):WINBOOL; external 'advapi32' name 'StartServiceA';
function DragQueryFileA(_para1:HDROP; _para2:cardinal; _para3:Pchar;_para4:cardinal):cardinal; external 'shell32' name 'DragQueryFileA';
function ExtractAssociatedIconA(_para1:HINST; _para2:Pchar; _para3:LPWORD):HICON; external 'shell32' name 'ExtractAssociatedIconA';
function ExtractIconA(_para1:HINST; _para2:Pchar; _para3:cardinal):HICON;external 'shell32' name 'ExtractIconA';
function FindExecutableA(_para1:pchar; _para2:pchar;_para3:pchar):HINST; external 'shell32' name 'FindExecutableA';
function ShellAboutA(_para1:HWND; _para2:pchar; _para3:pchar;_para4:HICON):longint; external 'shell32' name 'ShellAboutA';
function ShellExecuteA(_para1:HWND; _para2:pchar; _para3:pchar;_para4:pchar; _para5:pchar;_para6:longint):HINST; external 'shell32' name 'ShellExecuteA';
function Shell_NotifyIconA(dwMessage: DWORD; lpData: PNotifyIconDataA): BOOL; external 'shell32' name 'Shell_NotifyIconA';
function DdeCreateStringHandleA(_para1:DWORD; _para2:pchar;_para3:longint):HSZ; external 'user32' name 'DdeCreateStringHandleA';
function DdeInitializeA(_para1:LPDWORD; _para2:PFNCALLBACK; _para3:DWORD; _para4:DWORD):UINT;external 'user32' name 'DdeInitializeA';
function DdeQueryStringA(_para1:DWORD; _para2:HSZ; _para3:pchar;_para4:DWORD; _para5:longint):DWORD; external 'user32' name 'DdeQueryStringA';
function LogonUserA(_para1:LPSTR; _para2:LPSTR; _para3:LPSTR; _para4:DWORD; _para5:DWORD;_para6:PHANDLE):WINBOOL; external 'advapi32' name 'LogonUserA';
function CreateProcessAsUserA(_para1:HANDLE; _para2:LPCTSTR; _para3:LPTSTR; _para4:LPSECURITY_ATTRIBUTES; _para5:LPSECURITY_ATTRIBUTES;_para6:WINBOOL; _para7:DWORD; _para8:LPVOID; _para9:LPCTSTR;
<<<<<<< HEAD
  _para10:LPSTARTUPINFOA; _para11:LPPROCESS_INFORMATION):WINBOOL; external 'advapi32' name 'CreateProcessAsUserA';
function CreateWaitableTimerA(lpTimerAttributes :LPSECURITY_ATTRIBUTES; bManualReset:BOOl;lpTimerName:LPCTSTR):THandle;external 'kernel32' name 'CreateWaitableTimerA'; 
function OpenWaitableTimerA(dwDesiredAccess:DWORD;bInheritHandle:BOOL;lpTimerName:LPCTSTR):THandle;external 'kernel32' name 'OpenWaitableTimerA'; 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function FindFirstFileExA(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external 'kernel32' name 'FindFirstFileExA';
// winver>$0600
function FindFirstFileTransactedA(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external 'kernel32' name 'FindFirstFileTransactedA';
=======
function FindFirstFileExA(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external name 'FindFirstFileExA';
// winver>$0600
function FindFirstFileTransactedA(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedA';
>>>>>>> graemeg/cpstrnew
=======
function FindFirstFileExA(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external name 'FindFirstFileExA';
// winver>$0600
function FindFirstFileTransactedA(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedA';
>>>>>>> graemeg/cpstrnew
=======
function FindFirstFileExA(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external name 'FindFirstFileExA';
// winver>$0600
function FindFirstFileTransactedA(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedA';
>>>>>>> graemeg/cpstrnew
=======
function FindFirstFileExA(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword):Handle; stdcall; external name 'FindFirstFileExA';
// winver>$0600
function FindFirstFileTransactedA(lpfilename : LPCStr;fInfoLevelId:FINDEX_INFO_LEVELS ;lpFindFileData:pointer;fSearchOp : FINDEX_SEARCH_OPS;lpSearchFilter:pointer;dwAdditionalFlags:dword;htransaction : HANDLE):Handle; stdcall; external name 'FindFirstFileTransactedA';
>>>>>>> origin/cpstrnew
=======
  _para10:LPSTARTUPINFO; _para11:LPPROCESS_INFORMATION):WINBOOL; external 'advapi32' name 'CreateProcessAsUserA';
function CreateWaitableTimerA(lpTimerAttributes :LPSECURITY_ATTRIBUTES; bManualReset:BOOl;lpTimerName:LPCTSTR):THandle;external 'kernel32' name 'CreateWaitableTimerA'; 
function OpenWaitableTimerA(dwDesiredAccess:DWORD;bInheritHandle:BOOL;lpTimerName:LPCTSTR):THandle;external 'kernel32' name 'OpenWaitableTimerA'; 
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$endif read_interface}


{$ifdef read_implementation}

function CreateWindowA(lpClassName:LPCSTR; lpWindowName:LPCSTR; dwStyle:DWORD; X:longint;Y:longint; nWidth:longint; nHeight:longint; hWndParent:HWND; hMenu:HMENU;hInstance:HINST; lpParam:LPVOID):HWND;
begin
  CreateWindowA:=CreateWindowExA(0,lpClassName,lpWindowName,dwStyle,x,y,nWidth,nHeight,hWndParent,hMenu,hInstance,lpParam);
end;

function CreateDialogA(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
begin
  CreateDialogA:=CreateDialogParamA(hInstance,lpTemplateName,hWndParent,lpDialogFunc,0);
end;

function CreateDialogIndirectA(hInstance:HINST; lpTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):HWND;
begin
  CreateDialogIndirectA:=CreateDialogIndirectParamA(hInstance,lpTemplate,hWndParent,lpDialogFunc,0);
end;

function DialogBoxA(hInstance:HINST; lpTemplateName:LPCSTR; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
begin
  DialogBoxA:=DialogBoxParamA(hInstance,lpTemplateName,hWndParent,lpDialogFunc,0);
end;

function DialogBoxIndirectA(hInstance:HINST; hDialogTemplate:LPCDLGTEMPLATE; hWndParent:HWND; lpDialogFunc:DLGPROC):longint;
begin
  DialogBoxIndirectA:=DialogBoxIndirectParamA(hInstance,hDialogTemplate,hWndParent,lpDialogFunc,0);
end;


{$endif read_implementation}

