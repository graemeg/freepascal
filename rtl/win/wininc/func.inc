{
    This file is part of the Free Pascal run time library.
    This unit contains the record definition for the Win32 API
    Copyright (c) 1999-2014 by Florian Klaempfl,
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{
  Functions.h

  Declarations for all the Windows32 API Functions

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

function GetRandomRgn(aHDC: HDC; aHRGN: HRGN; iNum: WINT): WINT; stdcall; external 'gdi32';
function AccessCheck(pSecurityDescriptor:PSECURITY_DESCRIPTOR; ClientToken:HANDLE; DesiredAccess:DWORD; GenericMapping:PGENERIC_MAPPING; PrivilegeSet:PPRIVILEGE_SET;PrivilegeSetLength:LPDWORD;
  GrantedAccess:LPDWORD; AccessStatus:LPBOOL):WINBOOL; external 'advapi32' name 'AccessCheck';
function FreeResource(hResData:HGLOBAL):WINBOOL; external 'kernel32' name 'FreeResource';
function LockResource(hResData:HGLOBAL):LPVOID; external 'kernel32' name 'LockResource';
{$ifdef Unknown_functions}
{ WARNING: function not found !!}
function WinMain(hInstance:HINST; hPrevInstance:HINST; lpCmdLine:LPSTR; nShowCmd:longint):longint; external External_library name 'WinMain';
{$endif Unknown_functions}
function FreeLibrary(hLibModule:HINST):WINBOOL; external 'kernel32' name 'FreeLibrary';
procedure FreeLibraryAndExitThread(hLibModule:HMODULE; dwExitCode:DWORD); external 'kernel32' name 'FreeLibraryAndExitThread';
function DisableThreadLibraryCalls(hLibModule:HMODULE):WINBOOL; external 'kernel32' name 'DisableThreadLibraryCalls';
function GetProcAddress(hModule:HINST; lpProcName:LPCSTR):FARPROC; external 'kernel32' name 'GetProcAddress';
function GetVersion:DWORD; external 'kernel32' name 'GetVersion';
function GlobalAlloc(uFlags:UINT; dwBytes:PTRUINT):HGLOBAL; external 'kernel32' name 'GlobalAlloc';
function GlobalDiscard(hglbMem:HGLOBAL):HGLOBAL;
function GlobalReAlloc(hMem:HGLOBAL; dwBytes:PTRUINT; uFlags:UINT):HGLOBAL; external 'kernel32' name 'GlobalReAlloc';
function GlobalSize(hMem:HGLOBAL):PTRUINT; external 'kernel32' name 'GlobalSize';
function GlobalFlags(hMem:HGLOBAL):UINT; external 'kernel32' name 'GlobalFlags';
function GlobalLock(hMem:HGLOBAL):LPVOID; external 'kernel32' name 'GlobalLock';
function GlobalHandle(pMem:LPCVOID):HGLOBAL; external 'kernel32' name 'GlobalHandle';
function GlobalUnlock(hMem:HGLOBAL):WINBOOL; external 'kernel32' name 'GlobalUnlock';
function GlobalFree(hMem:HGLOBAL):HGLOBAL; external 'kernel32' name 'GlobalFree';
function GlobalCompact(dwMinFree:DWORD):UINT; external 'kernel32' name 'GlobalCompact';
procedure GlobalFix(hMem:HGLOBAL); external 'kernel32' name 'GlobalFix';
procedure GlobalUnfix(hMem:HGLOBAL); external 'kernel32' name 'GlobalUnfix';
function GlobalWire(hMem:HGLOBAL):LPVOID; external 'kernel32' name 'GlobalWire';
function GlobalUnWire(hMem:HGLOBAL):WINBOOL; external 'kernel32' name 'GlobalUnWire';
procedure GlobalMemoryStatus(lpBuffer:LPMEMORYSTATUS); external 'kernel32' name 'GlobalMemoryStatus';
function LocalAlloc(uFlags:UINT; uBytes:PTRUINT):HLOCAL; external 'kernel32' name 'LocalAlloc';
function LocalDiscard(hlocMem:HLOCAL):HLOCAL;
function LocalReAlloc(hMem:HLOCAL; uBytes:PTRUINT; uFlags:UINT):HLOCAL; external 'kernel32' name 'LocalReAlloc';
function LocalLock(hMem:HLOCAL):LPVOID; external 'kernel32' name 'LocalLock';
function LocalHandle(pMem:LPCVOID):HLOCAL; external 'kernel32' name 'LocalHandle';
function LocalUnlock(hMem:HLOCAL):WINBOOL; external 'kernel32' name 'LocalUnlock';
function LocalSize(hMem:HLOCAL):UINT; external 'kernel32' name 'LocalSize';
function LocalFlags(hMem:HLOCAL):UINT; external 'kernel32' name 'LocalFlags';
function LocalFree(hMem:HLOCAL):HLOCAL; external 'kernel32' name 'LocalFree';
function LocalShrink(hMem:HLOCAL; cbNewSize:UINT):UINT; external 'kernel32' name 'LocalShrink';
function LocalCompact(uMinFree:UINT):UINT; external 'kernel32' name 'LocalCompact';
function FlushInstructionCache(hProcess:HANDLE; lpBaseAddress:LPCVOID; dwSize:PTRUINT):WINBOOL; external 'kernel32' name 'FlushInstructionCache';
function VirtualAlloc(lpAddress:LPVOID; dwSize:PTRUINT; flAllocationType:DWORD; flProtect:DWORD):LPVOID; external 'kernel32' name 'VirtualAlloc';
function VirtualAllocEx(hProcess:THandle;lpAddress:LPVOID;dwsize:PTRUINT;flAllocationType :DWORD;flProtect:DWORD):LPVOID; external 'kernel32' name 'VirtualAllocEx';
function VirtualAllocExNuma(hProcess:THandle;lpAddress:LPVOID;dwsize:PTRUINT;flAllocationType :DWORD;flProtect:DWORD;nndPreferred:DWORD):LPVOID; external 'kernel32' name 'VirtualAllocExNuma';
function VirtualFree(lpAddress:LPVOID; dwSize:PTRUINT; dwFreeType:DWORD):WINBOOL; external 'kernel32' name 'VirtualFree';
function VirtualFreeEx(hProcess:THandle;lpAddress:LPVOID; dwSize:PTRUINT; dwFreeType:DWORD):WINBOOL; external 'kernel32' name 'VirtualFreeEx';
function VirtualProtect(lpAddress:LPVOID; dwSize:PTRUINT; flNewProtect:DWORD; lpflOldProtect:PDWORD):WINBOOL; external 'kernel32' name 'VirtualProtect';
function VirtualQuery(lpAddress:LPCVOID; lpBuffer:PMEMORY_BASIC_INFORMATION; dwLength:PTRUINT):PTRUINT; external 'kernel32' name 'VirtualQuery';
function VirtualProtectEx(hProcess:HANDLE; lpAddress:LPVOID; dwSize:PTRUINT; flNewProtect:DWORD; lpflOldProtect:PDWORD):WINBOOL; external 'kernel32' name 'VirtualProtectEx';
function VirtualQueryEx(hProcess:HANDLE; lpAddress:LPCVOID; lpBuffer:PMEMORY_BASIC_INFORMATION; dwLength:PTRUINT):PTRUINT; external 'kernel32' name 'VirtualQueryEx';
function HeapCreate(flOptions:DWORD; dwInitialSize:PTRUINT; dwMaximumSize:PTRUINT):HANDLE; external 'kernel32' name 'HeapCreate';
function HeapDestroy(hHeap:HANDLE):WINBOOL; external 'kernel32' name 'HeapDestroy';
function HeapAlloc(hHeap:HANDLE; dwFlags:DWORD; dwBytes:PTRUINT):LPVOID; external 'kernel32' name 'HeapAlloc';
function HeapReAlloc(hHeap:HANDLE; dwFlags:DWORD; lpMem:LPVOID; dwBytes:PTRUINT):LPVOID; external 'kernel32' name 'HeapReAlloc';
function HeapFree(hHeap:HANDLE; dwFlags:DWORD; lpMem:LPVOID):WINBOOL; external 'kernel32' name 'HeapFree';
function HeapSize(hHeap:HANDLE; dwFlags:DWORD; lpMem:LPCVOID):PTRUINT; external 'kernel32' name 'HeapSize';
function HeapValidate(hHeap:HANDLE; dwFlags:DWORD; lpMem:LPCVOID):WINBOOL; external 'kernel32' name 'HeapValidate';
function HeapCompact(hHeap:HANDLE; dwFlags:DWORD):PTRUINT; external 'kernel32' name 'HeapCompact';
function GetProcessHeap:HANDLE; external 'kernel32' name 'GetProcessHeap';
function GetProcessHeaps(NumberOfHeaps:DWORD; ProcessHeaps:PHANDLE):DWORD; external 'kernel32' name 'GetProcessHeaps';
function HeapLock(hHeap:HANDLE):WINBOOL; external 'kernel32' name 'HeapLock';
function HeapUnlock(hHeap:HANDLE):WINBOOL; external 'kernel32' name 'HeapUnlock';
function HeapWalk(hHeap:HANDLE; lpEntry:LPPROCESS_HEAP_ENTRY):WINBOOL; external 'kernel32' name 'HeapWalk';
function GetProcessAffinityMask(hProcess:HANDLE; lpProcessAffinityMask:PDWORD_PTR; lpSystemAffinityMask:PDWORD_PTR):WINBOOL; external 'kernel32' name 'GetProcessAffinityMask';
function GetProcessTimes(hProcess:HANDLE; lpCreationTime:LPFILETIME; lpExitTime:LPFILETIME; lpKernelTime:LPFILETIME; lpUserTime:LPFILETIME):WINBOOL; external 'kernel32' name 'GetProcessTimes';
function GetProcessWorkingSetSize(hProcess:HANDLE; lpMinimumWorkingSetSize:PPTRUINT; lpMaximumWorkingSetSize:PPTRUINT):WINBOOL; external 'kernel32' name 'GetProcessWorkingSetSize';
function SetProcessWorkingSetSize(hProcess:HANDLE; dwMinimumWorkingSetSize:PTRUINT; dwMaximumWorkingSetSize:PTRUINT):WINBOOL; external 'kernel32' name 'SetProcessWorkingSetSize';
function OpenProcess(dwDesiredAccess:DWORD; bInheritHandle:WINBOOL; dwProcessId:DWORD):HANDLE; external 'kernel32' name 'OpenProcess';
function GetCurrentProcess:HANDLE; external 'kernel32' name 'GetCurrentProcess';
function GetCurrentProcessId:DWORD; external 'kernel32' name 'GetCurrentProcessId';
procedure ExitProcess(uExitCode:UINT);external 'kernel32' name 'ExitProcess';
function TerminateProcess(hProcess:HANDLE; uExitCode:UINT):WINBOOL; external 'kernel32' name 'TerminateProcess';
function SetProcessAffinityMask(hProcess: THandle; dwProcessAffinityMask: DWORD_PTR): BOOL; external 'kernel32' name 'SetProcessAffinityMask';
function GetExitCodeProcess(hProcess:HANDLE; lpExitCode:LPDWORD):WINBOOL; external 'kernel32' name 'GetExitCodeProcess';
procedure FatalExit(ExitCode:longint); external 'kernel32' name 'FatalExit';
procedure RaiseException(dwExceptionCode:DWORD; dwExceptionFlags:DWORD; nNumberOfArguments:DWORD; lpArguments:LPDWORD); external 'kernel32' name 'RaiseException';
function UnhandledExceptionFilter(ExceptionInfo:lpemptyrecord):LONG; external 'kernel32' name 'UnhandledExceptionFilter';
function CreateRemoteThread(hProcess:HANDLE; lpThreadAttributes:LPSECURITY_ATTRIBUTES; dwStackSize:PTRUINT; lpStartAddress:LPTHREAD_START_ROUTINE; lpParameter:LPVOID;
  dwCreationFlags:DWORD; lpThreadId:LPDWORD):HANDLE; external 'kernel32' name 'CreateRemoteThread';
function GetCurrentThread:HANDLE; external 'kernel32' name 'GetCurrentThread';
function GetCurrentThreadId:DWORD; external 'kernel32' name 'GetCurrentThreadId';
function SetThreadAffinityMask(hThread:HANDLE; dwThreadAffinityMask:DWORD_PTR):DWORD_PTR; external 'kernel32' name 'SetThreadAffinityMask';
function SetThreadPriority(hThread:HANDLE; nPriority:longint):WINBOOL; external 'kernel32' name 'SetThreadPriority';
function GetThreadPriority(hThread:HANDLE):longint; external 'kernel32' name 'GetThreadPriority';
function GetThreadTimes(hThread:HANDLE; lpCreationTime:LPFILETIME; lpExitTime:LPFILETIME; lpKernelTime:LPFILETIME; lpUserTime:LPFILETIME):WINBOOL; external 'kernel32' name 'GetThreadTimes';
procedure ExitThread(dwExitCode:DWORD); external 'kernel32' name 'ExitThread';
function TerminateThread(hThread:HANDLE; dwExitCode:DWORD):WINBOOL; external 'kernel32' name 'TerminateThread';
function GetExitCodeThread(hThread:HANDLE; lpExitCode:LPDWORD):WINBOOL; external 'kernel32' name 'GetExitCodeThread';
function GetThreadSelectorEntry(hThread:HANDLE; dwSelector:DWORD; lpSelectorEntry:LPLDT_ENTRY):WINBOOL; external 'kernel32' name 'GetThreadSelectorEntry';
function GetLastError:DWORD; external 'kernel32' name 'GetLastError';
procedure SetLastError(dwErrCode:DWORD); external 'kernel32' name 'SetLastError';
function CreateIoCompletionPort(FileHandle:HANDLE; ExistingCompletionPort:HANDLE; CompletionKey:ULONG_PTR; NumberOfConcurrentThreads:DWORD):HANDLE; external 'kernel32' name 'CreateIoCompletionPort';
function SetErrorMode(uMode:UINT):UINT; external 'kernel32' name 'SetErrorMode';
function ReadProcessMemory(hProcess:HANDLE; lpBaseAddress:LPCVOID; lpBuffer:LPVOID; nSize:PTRUINT; lpNumberOfBytesRead:PPTRUINT):WINBOOL; external 'kernel32' name 'ReadProcessMemory';
function WriteProcessMemory(hProcess:HANDLE; lpBaseAddress:LPVOID; lpBuffer:LPVOID; nSize:PTRUINT; lpNumberOfBytesWritten:PPTRUINT):WINBOOL; external 'kernel32' name 'WriteProcessMemory';
function GetThreadContext(hThread:HANDLE; lpContext:LPCONTEXT):WINBOOL; external 'kernel32' name 'GetThreadContext';
function SuspendThread(hThread:HANDLE):DWORD; external 'kernel32' name 'SuspendThread';
function ResumeThread(hThread:HANDLE):DWORD; external 'kernel32' name 'ResumeThread';
procedure DebugBreak; external 'kernel32' name 'DebugBreak';
function WaitForDebugEvent(lpDebugEvent:LPDEBUG_EVENT; dwMilliseconds:DWORD):WINBOOL; external 'kernel32' name 'WaitForDebugEvent';
function ContinueDebugEvent(dwProcessId:DWORD; dwThreadId:DWORD; dwContinueStatus:DWORD):WINBOOL; external 'kernel32' name 'ContinueDebugEvent';
function DebugActiveProcess(dwProcessId:DWORD):WINBOOL; external 'kernel32' name 'DebugActiveProcess';
procedure InitializeCriticalSection(lpCriticalSection:LPCRITICAL_SECTION); external 'kernel32' name 'InitializeCriticalSection';
procedure EnterCriticalSection(lpCriticalSection:LPCRITICAL_SECTION); external 'kernel32' name 'EnterCriticalSection';
procedure LeaveCriticalSection(lpCriticalSection:LPCRITICAL_SECTION); external 'kernel32' name 'LeaveCriticalSection';
procedure DeleteCriticalSection(lpCriticalSection:LPCRITICAL_SECTION); external 'kernel32' name 'DeleteCriticalSection';
function TryEnterCriticalSection(lpCriticalSection:LPCRITICAL_SECTION):WINBOOL; external 'kernel32' name 'TryEnterCriticalSection';
function SetEvent(hEvent:HANDLE):WINBOOL; external 'kernel32' name 'SetEvent';
function ResetEvent(hEvent:HANDLE):WINBOOL; external 'kernel32' name 'ResetEvent';
function PulseEvent(hEvent:HANDLE):WINBOOL; external 'kernel32' name 'PulseEvent';
function ReleaseSemaphore(hSemaphore:HANDLE; lReleaseCount:LONG; lpPreviousCount:LPLONG):WINBOOL; external 'kernel32' name 'ReleaseSemaphore';
function ReleaseMutex(hMutex:HANDLE):WINBOOL; external 'kernel32' name 'ReleaseMutex';
function WaitForSingleObject(hHandle:HANDLE; dwMilliseconds:DWORD):DWORD; external 'kernel32' name 'WaitForSingleObject';
function WaitForMultipleObjects(nCount:DWORD; lpHandles : PWOHandleArray; bWaitAll:WINBOOL; dwMilliseconds:DWORD):DWORD; external 'kernel32' name 'WaitForMultipleObjects';
procedure Sleep(dwMilliseconds:DWORD); external 'kernel32' name 'Sleep';
function LoadResource(hModule:HINST; hResInfo:HRSRC):HGLOBAL; external 'kernel32' name 'LoadResource';
function SizeofResource(hModule:HINST; hResInfo:HRSRC):DWORD; external 'kernel32' name 'SizeofResource';
function GlobalDeleteAtom(nAtom:ATOM):ATOM; external 'kernel32' name 'GlobalDeleteAtom';
function InitAtomTable(nSize:DWORD):WINBOOL; external 'kernel32' name 'InitAtomTable';
function DeleteAtom(nAtom:ATOM):ATOM; external 'kernel32' name 'DeleteAtom';
function SetHandleCount(uNumber:UINT):UINT; external 'kernel32' name 'SetHandleCount';
function GetLogicalDrives:DWORD; external 'kernel32' name 'GetLogicalDrives';
function LockFile(hFile:HANDLE; dwFileOffsetLow:DWORD; dwFileOffsetHigh:DWORD; nNumberOfBytesToLockLow:DWORD; nNumberOfBytesToLockHigh:DWORD):WINBOOL; external 'kernel32' name 'LockFile';
function UnlockFile(hFile:HANDLE; dwFileOffsetLow:DWORD; dwFileOffsetHigh:DWORD; nNumberOfBytesToUnlockLow:DWORD; nNumberOfBytesToUnlockHigh:DWORD):WINBOOL; external 'kernel32' name 'UnlockFile';
function LockFileEx(hFile:HANDLE; dwFlags:DWORD; dwReserved:DWORD; nNumberOfBytesToLockLow:DWORD; nNumberOfBytesToLockHigh:DWORD;lpOverlapped:LPOVERLAPPED):WINBOOL; external 'kernel32' name 'LockFileEx';
function UnlockFileEx(hFile:HANDLE; dwReserved:DWORD; nNumberOfBytesToUnlockLow:DWORD; nNumberOfBytesToUnlockHigh:DWORD; lpOverlapped:LPOVERLAPPED):WINBOOL; external 'kernel32' name 'UnlockFileEx';
function GetFileInformationByHandle(hFile:HANDLE; lpFileInformation:LPBY_HANDLE_FILE_INFORMATION):WINBOOL; external 'kernel32' name 'GetFileInformationByHandle';
function GetFileType(hFile:HANDLE):DWORD; external 'kernel32' name 'GetFileType';
function GetFileSize(hFile:HANDLE; lpFileSizeHigh:LPDWORD):DWORD; external 'kernel32' name 'GetFileSize';
function GetStdHandle(nStdHandle:DWORD):HANDLE; external 'kernel32' name 'GetStdHandle';
function SetStdHandle(nStdHandle:DWORD; hHandle:HANDLE):WINBOOL; external 'kernel32' name 'SetStdHandle';
function FlushFileBuffers(hFile:HANDLE):WINBOOL; external 'kernel32' name 'FlushFileBuffers';
function DeviceIoControl(hDevice:HANDLE; dwIoControlCode:DWORD; lpInBuffer:LPVOID; nInBufferSize:DWORD; lpOutBuffer:LPVOID;nOutBufferSize:DWORD; lpBytesReturned:LPDWORD; lpOverlapped:LPOVERLAPPED):WINBOOL; external 'kernel32' name 'DeviceIoControl';
function SetEndOfFile(hFile:HANDLE):WINBOOL; external 'kernel32' name 'SetEndOfFile';
function SetFilePointer(hFile:HANDLE; lDistanceToMove:LONG; lpDistanceToMoveHigh:PLONG; dwMoveMethod:DWORD):DWORD; external 'kernel32' name 'SetFilePointer';
function FindClose(hFindFile:HANDLE):WINBOOL; external 'kernel32' name 'FindClose';
function GetFileTime(hFile:HANDLE; lpCreationTime:LPFILETIME; lpLastAccessTime:LPFILETIME; lpLastWriteTime:LPFILETIME):WINBOOL; external 'kernel32' name 'GetFileTime';
function SetFileTime(hFile:HANDLE; lpCreationTime:LPFILETIME; lpLastAccessTime:LPFILETIME; lpLastWriteTime:LPFILETIME):WINBOOL; external 'kernel32' name 'SetFileTime';
function CloseHandle(hObject:HANDLE):WINBOOL; external 'kernel32' name 'CloseHandle';
function DuplicateHandle(hSourceProcessHandle:HANDLE; hSourceHandle:HANDLE; hTargetProcessHandle:HANDLE; lpTargetHandle:LPHANDLE; dwDesiredAccess:DWORD;bInheritHandle:WINBOOL; dwOptions:DWORD):WINBOOL; external 'kernel32' name 'DuplicateHandle';
function GetHandleInformation(hObject:HANDLE; lpdwFlags:LPDWORD):WINBOOL; external 'kernel32' name 'GetHandleInformation';
function SetHandleInformation(hObject:HANDLE; dwMask:DWORD; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'SetHandleInformation';
function LoadModule(lpModuleName:LPCSTR; lpParameterBlock:LPVOID):DWORD; external 'kernel32' name 'LoadModule';
// Winexec is a win16 legacy function, deprecated for 17 years (deprecated since Windows 95).
// Annotated the MS deprecated status with a FPC deprecated directive so it won't be used for new development.
function WinExec(lpCmdLine:LPCSTR; uCmdShow:UINT):UINT; external 'kernel32' name 'WinExec'; deprecated;
function ClearCommBreak(hFile:HANDLE):WINBOOL; external 'kernel32' name 'ClearCommBreak';
function ClearCommError(hFile:HANDLE; lpErrors:LPDWORD; lpStat:LPCOMSTAT):WINBOOL; external 'kernel32' name 'ClearCommError';
function SetupComm(hFile:HANDLE; dwInQueue:DWORD; dwOutQueue:DWORD):WINBOOL; external 'kernel32' name 'SetupComm';
function EscapeCommFunction(hFile:HANDLE; dwFunc:DWORD):WINBOOL; external 'kernel32' name 'EscapeCommFunction';
function GetCommConfig(hCommDev:HANDLE; lpCC:LPCOMMCONFIG; lpdwSize:LPDWORD):WINBOOL; external 'kernel32' name 'GetCommConfig';
function GetCommProperties(hFile:HANDLE; lpCommProp:LPCOMMPROP):WINBOOL; external 'kernel32' name 'GetCommProperties';
function GetCommModemStatus(hFile:HANDLE; lpModemStat:PDWORD):WINBOOL; external 'kernel32' name 'GetCommModemStatus';
function GetCommState(hFile:HANDLE; lpDCB:PDCB):WINBOOL; external 'kernel32' name 'GetCommState';
function GetCommTimeouts(hFile:HANDLE; lpCommTimeouts:PCOMMTIMEOUTS):WINBOOL; external 'kernel32' name 'GetCommTimeouts';
function PurgeComm(hFile:HANDLE; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'PurgeComm';
function SetCommBreak(hFile:HANDLE):WINBOOL; external 'kernel32' name 'SetCommBreak';
function SetCommConfig(hCommDev:HANDLE; lpCC:LPCOMMCONFIG; dwSize:DWORD):WINBOOL; external 'kernel32' name 'SetCommConfig';
function SetCommMask(hFile:HANDLE; dwEvtMask:DWORD):WINBOOL; external 'kernel32' name 'SetCommMask';
function SetCommState(hFile:HANDLE; lpDCB:LPDCB):WINBOOL; external 'kernel32' name 'SetCommState';
function SetCommTimeouts(hFile:HANDLE; lpCommTimeouts:LPCOMMTIMEOUTS):WINBOOL; external 'kernel32' name 'SetCommTimeouts';
function TransmitCommChar(hFile:HANDLE; cChar:char):WINBOOL; external 'kernel32' name 'TransmitCommChar';
function WaitCommEvent(hFile:HANDLE; lpEvtMask:LPDWORD; lpOverlapped:LPOVERLAPPED):WINBOOL; external 'kernel32' name 'WaitCommEvent';
function SetTapePosition(hDevice:HANDLE; dwPositionMethod:DWORD; dwPartition:DWORD; dwOffsetLow:DWORD; dwOffsetHigh:DWORD;bImmediate:WINBOOL):DWORD; external 'kernel32' name 'SetTapePosition';
function GetTapePosition(hDevice:HANDLE; dwPositionType:DWORD; lpdwPartition:LPDWORD; lpdwOffsetLow:LPDWORD; lpdwOffsetHigh:LPDWORD):DWORD; external 'kernel32' name 'GetTapePosition';
function PrepareTape(hDevice:HANDLE; dwOperation:DWORD; bImmediate:WINBOOL):DWORD; external 'kernel32' name 'PrepareTape';
function EraseTape(hDevice:HANDLE; dwEraseType:DWORD; bImmediate:WINBOOL):DWORD; external 'kernel32' name 'EraseTape';
function CreateTapePartition(hDevice:HANDLE; dwPartitionMethod:DWORD; dwCount:DWORD; dwSize:DWORD):DWORD; external 'kernel32' name 'CreateTapePartition';
function WriteTapemark(hDevice:HANDLE; dwTapemarkType:DWORD; dwTapemarkCount:DWORD; bImmediate:WINBOOL):DWORD; external 'kernel32' name 'WriteTapemark';
function GetTapeStatus(hDevice:HANDLE):DWORD; external 'kernel32' name 'GetTapeStatus';
function GetTapeParameters(hDevice:HANDLE; dwOperation:DWORD; lpdwSize:LPDWORD; lpTapeInformation:LPVOID):DWORD; external 'kernel32' name 'GetTapeParameters';
function SetTapeParameters(hDevice:HANDLE; dwOperation:DWORD; lpTapeInformation:LPVOID):DWORD; external 'kernel32' name 'SetTapeParameters';
function Beep(dwFreq:DWORD; dwDuration:DWORD):WINBOOL; external 'kernel32' name 'Beep';
{$ifdef Unknown_functions}
{ WARNING: functions not found !!}
procedure OpenSound; external External_library name 'OpenSound';
procedure CloseSound; external External_library name 'CloseSound';
procedure StartSound; external External_library name 'StartSound';
procedure StopSound; external External_library name 'StopSound';
function WaitSoundState(nState:DWORD):DWORD; external External_library name 'WaitSoundState';
function SyncAllVoices:DWORD; external External_library name 'SyncAllVoices';
function CountVoiceNotes(nVoice:DWORD):DWORD; external External_library name 'CountVoiceNotes';
function GetThresholdEvent:LPDWORD; external External_library name 'GetThresholdEvent';
function GetThresholdStatus:DWORD; external External_library name 'GetThresholdStatus';
function SetSoundNoise(nSource:DWORD; nDuration:DWORD):DWORD; external External_library name 'SetSoundNoise';
function SetVoiceAccent(nVoice:DWORD; nTempo:DWORD; nVolume:DWORD; nMode:DWORD; nPitch:DWORD):DWORD; external External_library name 'SetVoiceAccent';
function SetVoiceEnvelope(nVoice:DWORD; nShape:DWORD; nRepeat:DWORD):DWORD; external External_library name 'SetVoiceEnvelope';
function SetVoiceNote(nVoice:DWORD; nValue:DWORD; nLength:DWORD; nCdots:DWORD):DWORD; external External_library name 'SetVoiceNote';
function SetVoiceQueueSize(nVoice:DWORD; nBytes:DWORD):DWORD; external External_library name 'SetVoiceQueueSize';
function SetVoiceSound(nVoice:DWORD; Frequency:DWORD; nDuration:DWORD):DWORD; external External_library name 'SetVoiceSound';
function SetVoiceThreshold(nVoice:DWORD; nNotes:DWORD):DWORD; external External_library name 'SetVoiceThreshold';
{$endif Unknown_functions}
function MulDiv(nNumber:longint; nNumerator:longint; nDenominator:longint):longint; external 'kernel32' name 'MulDiv';
procedure GetSystemTime(lpSystemTime:LPSYSTEMTIME); external 'kernel32' name 'GetSystemTime';
procedure GetSystemTimeAsFileTime(lpSystemTimeAsFileTime:LPFILETIME); external 'kernel32' name 'GetSystemTimeAsFileTime';
function SetSystemTime(lpSystemTime:LPSYSTEMTIME):WINBOOL; external 'kernel32' name 'SetSystemTime';
procedure GetLocalTime(lpSystemTime:LPSYSTEMTIME); external 'kernel32' name 'GetLocalTime';
function SetLocalTime(lpSystemTime:LPSYSTEMTIME):WINBOOL; external 'kernel32' name 'SetLocalTime';
procedure GetSystemInfo(lpSystemInfo:LPSYSTEM_INFO); external 'kernel32' name 'GetSystemInfo';
procedure GetNativeSystemInfo(lpSystemInfo:LPSYSTEM_INFO); external 'kernel32' name 'GetNativeSystemInfo';
function SystemTimeToTzSpecificLocalTime(lpTimeZoneInformation:LPTIME_ZONE_INFORMATION; lpUniversalTime:LPSYSTEMTIME; lpLocalTime:LPSYSTEMTIME):WINBOOL; external 'kernel32' name 'SystemTimeToTzSpecificLocalTime';
function GetTimeZoneInformation(lpTimeZoneInformation:LPTIME_ZONE_INFORMATION):DWORD; external 'kernel32' name 'GetTimeZoneInformation';
function SetTimeZoneInformation(lpTimeZoneInformation:LPTIME_ZONE_INFORMATION):WINBOOL; external 'kernel32' name 'SetTimeZoneInformation';
function SystemTimeToFileTime(lpSystemTime:LPSYSTEMTIME; lpFileTime:LPFILETIME):WINBOOL; external 'kernel32' name 'SystemTimeToFileTime';
function FileTimeToLocalFileTime(lpFileTime:LPFILETIME; lpLocalFileTime:LPFILETIME):WINBOOL; external 'kernel32' name 'FileTimeToLocalFileTime';
function LocalFileTimeToFileTime(lpLocalFileTime:LPFILETIME; lpFileTime:LPFILETIME):WINBOOL; external 'kernel32' name 'LocalFileTimeToFileTime';
function FileTimeToSystemTime(lpFileTime:LPFILETIME; lpSystemTime:LPSYSTEMTIME):WINBOOL; external 'kernel32' name 'FileTimeToSystemTime';
function CompareFileTime(lpFileTime1:LPFILETIME; lpFileTime2:LPFILETIME):LONG; external 'kernel32' name 'CompareFileTime';
function FileTimeToDosDateTime(lpFileTime:LPFILETIME; lpFatDate:LPWORD; lpFatTime:LPWORD):WINBOOL; external 'kernel32' name 'FileTimeToDosDateTime';
function DosDateTimeToFileTime(wFatDate:WORD; wFatTime:WORD; lpFileTime:LPFILETIME):WINBOOL; external 'kernel32' name 'DosDateTimeToFileTime';
function GetTickCount:DWORD; external 'kernel32' name 'GetTickCount';
// GetTickCount64 requires Windows Vista/Windows Server 2008 or higher
function GetTickCount64:ULONGLONG; external 'kernel32' name 'GetTickCount64';
function SetSystemTimeAdjustment(dwTimeAdjustment:DWORD; bTimeAdjustmentDisabled:WINBOOL):WINBOOL; external 'kernel32' name 'SetSystemTimeAdjustment';
function GetSystemTimeAdjustment(lpTimeAdjustment:PDWORD; lpTimeIncrement:PDWORD; lpTimeAdjustmentDisabled:PWINBOOL):WINBOOL; external 'kernel32' name 'GetSystemTimeAdjustment';
function CreatePipe(hReadPipe:PHANDLE; hWritePipe:PHANDLE; lpPipeAttributes:LPSECURITY_ATTRIBUTES; nSize:DWORD):WINBOOL; external 'kernel32' name 'CreatePipe';
function ConnectNamedPipe(hNamedPipe:HANDLE; lpOverlapped:LPOVERLAPPED):WINBOOL; external 'kernel32' name 'ConnectNamedPipe';
function DisconnectNamedPipe(hNamedPipe:HANDLE):WINBOOL; external 'kernel32' name 'DisconnectNamedPipe';
function SetNamedPipeHandleState(hNamedPipe:HANDLE; lpMode:LPDWORD; lpMaxCollectionCount:LPDWORD; lpCollectDataTimeout:LPDWORD):WINBOOL; external 'kernel32' name 'SetNamedPipeHandleState';
function GetNamedPipeInfo(hNamedPipe:HANDLE; lpFlags:LPDWORD; lpOutBufferSize:LPDWORD; lpInBufferSize:LPDWORD; lpMaxInstances:LPDWORD):WINBOOL; external 'kernel32' name 'GetNamedPipeInfo';
function PeekNamedPipe(hNamedPipe:HANDLE; lpBuffer:LPVOID; nBufferSize:DWORD; lpBytesRead:LPDWORD; lpTotalBytesAvail:LPDWORD;lpBytesLeftThisMessage:LPDWORD):WINBOOL; external 'kernel32' name 'PeekNamedPipe';
function TransactNamedPipe(hNamedPipe:HANDLE; lpInBuffer:LPVOID; nInBufferSize:DWORD; lpOutBuffer:LPVOID; nOutBufferSize:DWORD;lpBytesRead:LPDWORD; lpOverlapped:LPOVERLAPPED):WINBOOL; external 'kernel32' name 'TransactNamedPipe';
function GetMailslotInfo(hMailslot:HANDLE; lpMaxMessageSize:LPDWORD; lpNextSize:LPDWORD; lpMessageCount:LPDWORD; lpReadTimeout:LPDWORD):WINBOOL; external 'kernel32' name 'GetMailslotInfo';
function SetMailslotInfo(hMailslot:HANDLE; lReadTimeout:DWORD):WINBOOL; external 'kernel32' name 'SetMailslotInfo';
function MapViewOfFile(hFileMappingObject:HANDLE; dwDesiredAccess:DWORD; dwFileOffsetHigh:DWORD; dwFileOffsetLow:DWORD; dwNumberOfBytesToMap:SIZE_T):LPVOID; external 'kernel32' name 'MapViewOfFile';
function FlushViewOfFile(lpBaseAddress:LPCVOID; dwNumberOfBytesToFlush:DWORD):WINBOOL; external 'kernel32' name 'FlushViewOfFile';
function UnmapViewOfFile(lpBaseAddress:LPVOID):WINBOOL; external 'kernel32' name 'UnmapViewOfFile';
function OpenFile(lpFileName:LPCSTR; lpReOpenBuff:LPOFSTRUCT; uStyle:UINT):HFILE; external 'kernel32' name 'OpenFile';
function _lopen(lpPathName:LPCSTR; iReadWrite:longint):HFILE; external 'kernel32' name '_lopen';
function _lcreat(lpPathName:LPCSTR; iAttribute:longint):HFILE; external 'kernel32' name '_lcreat';
function _lread(hFile:HFILE; lpBuffer:LPVOID; uBytes:UINT):UINT; external 'kernel32' name '_lread';
function _lwrite(hFile:HFILE; lpBuffer:LPCSTR; uBytes:UINT):UINT; external 'kernel32' name '_lwrite';
function _hread(hFile:HFILE; lpBuffer:LPVOID; lBytes:longint):longint; external 'kernel32' name '_hread';
function _hwrite(hFile:HFILE; lpBuffer:LPCSTR; lBytes:longint):longint; external 'kernel32' name '_hwrite';
function _lclose(hFile:HFILE):HFILE; external 'kernel32' name '_lclose';
function _llseek(hFile:HFILE; lOffset:LONG; iOrigin:longint):LONG; external 'kernel32' name '_llseek';
function IsTextUnicode(lpBuffer:LPVOID; cb:longint; lpi:LPINT):WINBOOL; external 'advapi32' name 'IsTextUnicode';
function TlsAlloc:DWORD; external 'kernel32' name 'TlsAlloc';
function TlsGetValue(dwTlsIndex:DWORD):LPVOID; external 'kernel32' name 'TlsGetValue';
function TlsSetValue(dwTlsIndex:DWORD; lpTlsValue:LPVOID):WINBOOL; external 'kernel32' name 'TlsSetValue';
function TlsFree(dwTlsIndex:DWORD):WINBOOL; external 'kernel32' name 'TlsFree';
function SleepEx(dwMilliseconds:DWORD; bAlertable:WINBOOL):DWORD; external 'kernel32' name 'SleepEx';
function WaitForSingleObjectEx(hHandle:HANDLE; dwMilliseconds:DWORD; bAlertable:WINBOOL):DWORD; external 'kernel32' name 'WaitForSingleObjectEx';
function WaitForMultipleObjectsEx(nCount:DWORD; lpHandles:LPHANDLE; bWaitAll:WINBOOL; dwMilliseconds:DWORD; bAlertable:WINBOOL):DWORD; external 'kernel32' name 'WaitForMultipleObjectsEx';
function ReadFileEx(hFile:HANDLE; lpBuffer:LPVOID; nNumberOfBytesToRead:DWORD; lpOverlapped:LPOVERLAPPED; lpCompletionRoutine:LPOVERLAPPED_COMPLETION_ROUTINE):WINBOOL; external 'kernel32' name 'ReadFileEx';
function WriteFileEx(hFile:HANDLE; lpBuffer:LPCVOID; nNumberOfBytesToWrite:DWORD; lpOverlapped:LPOVERLAPPED; lpCompletionRoutine:LPOVERLAPPED_COMPLETION_ROUTINE):WINBOOL; external 'kernel32' name 'WriteFileEx';
function BackupRead(hFile:HANDLE; lpBuffer:LPBYTE; nNumberOfBytesToRead:DWORD; lpNumberOfBytesRead:LPDWORD; bAbort:WINBOOL;bProcessSecurity:WINBOOL; var lpContext:LPVOID):WINBOOL; external 'kernel32' name 'BackupRead';
function BackupSeek(hFile:HANDLE; dwLowBytesToSeek:DWORD; dwHighBytesToSeek:DWORD; lpdwLowByteSeeked:LPDWORD; lpdwHighByteSeeked:LPDWORD;var lpContext:LPVOID):WINBOOL; external 'kernel32' name 'BackupSeek';
function BackupWrite(hFile:HANDLE; lpBuffer:LPBYTE; nNumberOfBytesToWrite:DWORD; lpNumberOfBytesWritten:LPDWORD; bAbort:WINBOOL;bProcessSecurity:WINBOOL; var lpContext:LPVOID):WINBOOL; external 'kernel32' name 'BackupWrite';
function SetProcessShutdownParameters(dwLevel:DWORD; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'SetProcessShutdownParameters';
function GetProcessShutdownParameters(lpdwLevel:LPDWORD; lpdwFlags:LPDWORD):WINBOOL; external 'kernel32' name 'GetProcessShutdownParameters';
procedure SetFileApisToOEM; external 'kernel32' name 'SetFileApisToOEM';
procedure SetFileApisToANSI; external 'kernel32' name 'SetFileApisToANSI';
function AreFileApisANSI:WINBOOL; external 'kernel32' name 'AreFileApisANSI';
function CloseEventLog(hEventLog:HANDLE):WINBOOL; external 'advapi32' name 'CloseEventLog';
function DeregisterEventSource(hEventLog:HANDLE):WINBOOL; external 'advapi32' name 'DeregisterEventSource';
function NotifyChangeEventLog(hEventLog:HANDLE; hEvent:HANDLE):WINBOOL; external 'advapi32' name 'NotifyChangeEventLog';
function GetNumberOfEventLogRecords(hEventLog:HANDLE; NumberOfRecords:PDWORD):WINBOOL; external 'advapi32' name 'GetNumberOfEventLogRecords';
function GetOldestEventLogRecord(hEventLog:HANDLE; OldestRecord:PDWORD):WINBOOL; external 'advapi32' name 'GetOldestEventLogRecord';
function DuplicateToken(ExistingTokenHandle:HANDLE; ImpersonationLevel:SECURITY_IMPERSONATION_LEVEL; DuplicateTokenHandle:PHANDLE):WINBOOL; external 'advapi32' name 'DuplicateToken';
function GetKernelObjectSecurity(Handle:HANDLE; RequestedInformation:SECURITY_INFORMATION; pSecurityDescriptor:PSECURITY_DESCRIPTOR; nLength:DWORD; lpnLengthNeeded:LPDWORD):WINBOOL; external 'advapi32' name 'GetKernelObjectSecurity';
function ImpersonateNamedPipeClient(hNamedPipe:HANDLE):WINBOOL; external 'advapi32' name 'ImpersonateNamedPipeClient';
function ImpersonateLoggedOnUser(hToken:HANDLE):WINBOOL; external 'advapi32' name 'ImpersonateLoggedOnUser';
function ImpersonateSelf(ImpersonationLevel:SECURITY_IMPERSONATION_LEVEL):WINBOOL; external 'advapi32' name 'ImpersonateSelf';
function RevertToSelf:WINBOOL; external 'advapi32' name 'RevertToSelf';
function SetThreadToken(Thread:PHANDLE; Token:HANDLE):WINBOOL; external 'advapi32' name 'SetThreadToken';
{  function AccessCheck(pSecurityDescriptor:PSECURITY_DESCRIPTOR; ClientToken:HANDLE; DesiredAccess:DWORD; GenericMapping:PGENERIC_MAPPING; PrivilegeSet:PPRIVILEGE_SET;PrivilegeSetLength:LPDWORD;
  GrantedAccess:LPDWORD; AccessStatus:LPBOOL):WINBOOL; external 'advapi32' name 'AccessCheck';
}
function OpenProcessToken(ProcessHandle:HANDLE; DesiredAccess:DWORD; TokenHandle:PHANDLE):WINBOOL; external 'advapi32' name 'OpenProcessToken';
function OpenThreadToken(ThreadHandle:HANDLE; DesiredAccess:DWORD; OpenAsSelf:WINBOOL; TokenHandle:PHANDLE):WINBOOL; external 'advapi32' name 'OpenThreadToken';
function GetTokenInformation(TokenHandle:HANDLE; TokenInformationClass:TOKEN_INFORMATION_CLASS; TokenInformation:LPVOID; TokenInformationLength:DWORD; ReturnLength:PDWORD):WINBOOL; external 'advapi32' name 'GetTokenInformation';
function SetTokenInformation(TokenHandle:HANDLE; TokenInformationClass:TOKEN_INFORMATION_CLASS; TokenInformation:LPVOID; TokenInformationLength:DWORD):WINBOOL; external 'advapi32' name 'SetTokenInformation';
function AdjustTokenPrivileges(TokenHandle:HANDLE; DisableAllPrivileges:WINBOOL; NewState:PTOKEN_PRIVILEGES; BufferLength:DWORD; PreviousState:PTOKEN_PRIVILEGES;ReturnLength:PDWORD):WINBOOL; external 'advapi32' name 'AdjustTokenPrivileges';
function AdjustTokenGroups(TokenHandle:HANDLE; ResetToDefault:WINBOOL; NewState:PTOKEN_GROUPS; BufferLength:DWORD; PreviousState:PTOKEN_GROUPS;ReturnLength:PDWORD):WINBOOL; external 'advapi32' name 'AdjustTokenGroups';
function PrivilegeCheck(ClientToken:HANDLE; RequiredPrivileges:PPRIVILEGE_SET; pfResult:LPBOOL):WINBOOL; external 'advapi32' name 'PrivilegeCheck';
function IsValidSid(pSid:PSID):WINBOOL; external 'advapi32' name 'IsValidSid';
function EqualSid(pSid1:PSID; pSid2:PSID):WINBOOL; external 'advapi32' name 'EqualSid';
function EqualPrefixSid(pSid1:PSID; pSid2:PSID):WINBOOL; external 'advapi32' name 'EqualPrefixSid';
function GetSidLengthRequired(nSubAuthorityCount:UCHAR):DWORD; external 'advapi32' name 'GetSidLengthRequired';
function AllocateAndInitializeSid(pIdentifierAuthority:PSID_IDENTIFIER_AUTHORITY; nSubAuthorityCount:BYTE; nSubAuthority0:DWORD; nSubAuthority1:DWORD; nSubAuthority2:DWORD;nSubAuthority3:DWORD; nSubAuthority4:DWORD;
  nSubAuthority5:DWORD; nSubAuthority6:DWORD; nSubAuthority7:DWORD;var pSid:PSID):WINBOOL; external 'advapi32' name 'AllocateAndInitializeSid';
function FreeSid(pSid:PSID):PVOID; external 'advapi32' name 'FreeSid';
function InitializeSid(Sid:PSID; pIdentifierAuthority:PSID_IDENTIFIER_AUTHORITY; nSubAuthorityCount:BYTE):WINBOOL; external 'advapi32' name 'InitializeSid';
function GetSidIdentifierAuthority(pSid:PSID):PSID_IDENTIFIER_AUTHORITY; external 'advapi32' name 'GetSidIdentifierAuthority';
function GetSidSubAuthority(pSid:PSID; nSubAuthority:DWORD):PDWORD; external 'advapi32' name 'GetSidSubAuthority';
function GetSidSubAuthorityCount(pSid:PSID):PUCHAR; external 'advapi32' name 'GetSidSubAuthorityCount';
function GetLengthSid(pSid:PSID):DWORD; external 'advapi32' name 'GetLengthSid';
function CopySid(nDestinationSidLength:DWORD; pDestinationSid:PSID; pSourceSid:PSID):WINBOOL; external 'advapi32' name 'CopySid';
function AreAllAccessesGranted(GrantedAccess:DWORD; DesiredAccess:DWORD):WINBOOL; external 'advapi32' name 'AreAllAccessesGranted';
function AreAnyAccessesGranted(GrantedAccess:DWORD; DesiredAccess:DWORD):WINBOOL; external 'advapi32' name 'AreAnyAccessesGranted';
procedure MapGenericMask(AccessMask:PDWORD; GenericMapping:PGENERIC_MAPPING); external 'advapi32' name 'MapGenericMask';
function IsValidAcl(pAcl:PACL):WINBOOL; external 'advapi32' name 'IsValidAcl';
function InitializeAcl(pAcl:PACL; nAclLength:DWORD; dwAclRevision:DWORD):WINBOOL; external 'advapi32' name 'InitializeAcl';
function GetAclInformation(pAcl:PACL; pAclInformation:LPVOID; nAclInformationLength:DWORD; dwAclInformationClass:ACL_INFORMATION_CLASS):WINBOOL; external 'advapi32' name 'GetAclInformation';
function SetAclInformation(pAcl:PACL; pAclInformation:LPVOID; nAclInformationLength:DWORD; dwAclInformationClass:ACL_INFORMATION_CLASS):WINBOOL; external 'advapi32' name 'SetAclInformation';
function AddAce(pAcl:PACL; dwAceRevision:DWORD; dwStartingAceIndex:DWORD; pAceList:LPVOID; nAceListLength:DWORD):WINBOOL; external 'advapi32' name 'AddAce';
function DeleteAce(pAcl:PACL; dwAceIndex:DWORD):WINBOOL; external 'advapi32' name 'DeleteAce';
function GetAce(pAcl:PACL; dwAceIndex:DWORD; var pAce:LPVOID):WINBOOL; external 'advapi32' name 'GetAce';
function AddAccessAllowedAce(pAcl:PACL; dwAceRevision:DWORD; AccessMask:DWORD; pSid:PSID):WINBOOL; external 'advapi32' name 'AddAccessAllowedAce';
function AddAccessDeniedAce(pAcl:PACL; dwAceRevision:DWORD; AccessMask:DWORD; pSid:PSID):WINBOOL; external 'advapi32' name 'AddAccessDeniedAce';
function AddAuditAccessAce(pAcl:PACL; dwAceRevision:DWORD; dwAccessMask:DWORD; pSid:PSID; bAuditSuccess:WINBOOL;bAuditFailure:WINBOOL):WINBOOL; external 'advapi32' name 'AddAuditAccessAce';
function FindFirstFreeAce(pAcl:PACL; var pAce:LPVOID):WINBOOL; external 'advapi32' name 'FindFirstFreeAce';
function InitializeSecurityDescriptor(pSecurityDescriptor:PSECURITY_DESCRIPTOR; dwRevision:DWORD):WINBOOL; external 'advapi32' name 'InitializeSecurityDescriptor';
function IsValidSecurityDescriptor(pSecurityDescriptor:PSECURITY_DESCRIPTOR):WINBOOL; external 'advapi32' name 'IsValidSecurityDescriptor';
function GetSecurityDescriptorLength(pSecurityDescriptor:PSECURITY_DESCRIPTOR):DWORD; external 'advapi32' name 'GetSecurityDescriptorLength';
function GetSecurityDescriptorControl(pSecurityDescriptor:PSECURITY_DESCRIPTOR; pControl:PSECURITY_DESCRIPTOR_CONTROL; lpdwRevision:LPDWORD):WINBOOL; external 'advapi32' name 'GetSecurityDescriptorControl';
function SetSecurityDescriptorDacl(pSecurityDescriptor:PSECURITY_DESCRIPTOR; bDaclPresent:WINBOOL; pDacl:PACL; bDaclDefaulted:WINBOOL):WINBOOL; external 'advapi32' name 'SetSecurityDescriptorDacl';
function GetSecurityDescriptorDacl(pSecurityDescriptor:PSECURITY_DESCRIPTOR; lpbDaclPresent:LPBOOL; var pDacl:PACL; lpbDaclDefaulted:LPBOOL):WINBOOL; external 'advapi32' name 'GetSecurityDescriptorDacl';
function SetSecurityDescriptorSacl(pSecurityDescriptor:PSECURITY_DESCRIPTOR; bSaclPresent:WINBOOL; pSacl:PACL; bSaclDefaulted:WINBOOL):WINBOOL; external 'advapi32' name 'SetSecurityDescriptorSacl';
function GetSecurityDescriptorSacl(pSecurityDescriptor:PSECURITY_DESCRIPTOR; lpbSaclPresent:LPBOOL; var pSacl:PACL; lpbSaclDefaulted:LPBOOL):WINBOOL; external 'advapi32' name 'GetSecurityDescriptorSacl';
function SetSecurityDescriptorOwner(pSecurityDescriptor:PSECURITY_DESCRIPTOR; pOwner:PSID; bOwnerDefaulted:WINBOOL):WINBOOL; external 'advapi32' name 'SetSecurityDescriptorOwner';
function GetSecurityDescriptorOwner(pSecurityDescriptor:PSECURITY_DESCRIPTOR; var pOwner:PSID; lpbOwnerDefaulted:LPBOOL):WINBOOL; external 'advapi32' name 'GetSecurityDescriptorOwner';
function SetSecurityDescriptorGroup(pSecurityDescriptor:PSECURITY_DESCRIPTOR; pGroup:PSID; bGroupDefaulted:WINBOOL):WINBOOL; external 'advapi32' name 'SetSecurityDescriptorGroup';
function GetSecurityDescriptorGroup(pSecurityDescriptor:PSECURITY_DESCRIPTOR; var pGroup:PSID; lpbGroupDefaulted:LPBOOL):WINBOOL; external 'advapi32' name 'GetSecurityDescriptorGroup';
function CreatePrivateObjectSecurity(ParentDescriptor:PSECURITY_DESCRIPTOR; CreatorDescriptor:PSECURITY_DESCRIPTOR; var NewDescriptor:PSECURITY_DESCRIPTOR;
  IsDirectoryObject:WINBOOL; Token:HANDLE;GenericMapping:PGENERIC_MAPPING):WINBOOL; external 'advapi32' name 'CreatePrivateObjectSecurity';
function SetPrivateObjectSecurity(SecurityInformation:SECURITY_INFORMATION; ModificationDescriptor:PSECURITY_DESCRIPTOR; var ObjectsSecurityDescriptor:PSECURITY_DESCRIPTOR; GenericMapping:PGENERIC_MAPPING; Token:HANDLE):WINBOOL;
  external 'advapi32' name 'SetPrivateObjectSecurity';
function GetPrivateObjectSecurity(ObjectDescriptor:PSECURITY_DESCRIPTOR; SecurityInformation:SECURITY_INFORMATION; ResultantDescriptor:PSECURITY_DESCRIPTOR;
  DescriptorLength:DWORD; ReturnLength:PDWORD):WINBOOL;external 'advapi32' name 'GetPrivateObjectSecurity';
function DestroyPrivateObjectSecurity(ObjectDescriptor:PSECURITY_DESCRIPTOR):WINBOOL; external 'advapi32' name 'DestroyPrivateObjectSecurity';
function MakeSelfRelativeSD(pAbsoluteSecurityDescriptor:PSECURITY_DESCRIPTOR; pSelfRelativeSecurityDescriptor:PSECURITY_DESCRIPTOR; lpdwBufferLength:LPDWORD):WINBOOL; external 'advapi32' name 'MakeSelfRelativeSD';
function MakeAbsoluteSD(pSelfRelativeSecurityDescriptor:PSECURITY_DESCRIPTOR; pAbsoluteSecurityDescriptor:PSECURITY_DESCRIPTOR; lpdwAbsoluteSecurityDescriptorSize:LPDWORD; pDacl:PACL; lpdwDaclSize:LPDWORD;pSacl:PACL;
  lpdwSaclSize:LPDWORD; pOwner:PSID; lpdwOwnerSize:LPDWORD; pPrimaryGroup:PSID;lpdwPrimaryGroupSize:LPDWORD):WINBOOL; external 'advapi32' name 'MakeAbsoluteSD';
function SetKernelObjectSecurity(Handle:HANDLE; SecurityInformation:SECURITY_INFORMATION; SecurityDescriptor:PSECURITY_DESCRIPTOR):WINBOOL; external 'advapi32' name 'SetKernelObjectSecurity';
function FindNextChangeNotification(hChangeHandle:HANDLE):WINBOOL; external 'kernel32' name 'FindNextChangeNotification';
function FindCloseChangeNotification(hChangeHandle:HANDLE):WINBOOL; external 'kernel32' name 'FindCloseChangeNotification';
<<<<<<< HEAD
function VirtualLock(lpAddress:LPVOID; dwSize:PTRUINT):WINBOOL; external 'kernel32' name 'VirtualLock';
function VirtualUnlock(lpAddress:LPVOID; dwSize:PTRUINT):WINBOOL; external 'kernel32' name 'VirtualUnlock';
=======
function VirtualLock(lpAddress:LPVOID; dwSize:DWORD):WINBOOL; external 'kernel32' name 'VirtualLock';
function VirtualUnlock(lpAddress:LPVOID; dwSize:DWORD):WINBOOL; external 'kernel32' name 'VirtualUnlock';
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
function MapViewOfFileEx(hFileMappingObject:HANDLE; dwDesiredAccess:DWORD; dwFileOffsetHigh:DWORD; dwFileOffsetLow:DWORD; dwNumberOfBytesToMap:SIZE_T;lpBaseAddress:LPVOID):LPVOID; external 'kernel32' name 'MapViewOfFileEx';
function SetPriorityClass(hProcess:HANDLE; dwPriorityClass:DWORD):WINBOOL; external 'kernel32' name 'SetPriorityClass';
function GetPriorityClass(hProcess:HANDLE):DWORD; external 'kernel32' name 'GetPriorityClass';
function IsBadReadPtr(lp:pointer; ucb:UINT):WINBOOL; external 'kernel32' name 'IsBadReadPtr';
function IsBadWritePtr(lp:LPVOID; ucb:UINT):WINBOOL; external 'kernel32' name 'IsBadWritePtr';
function IsBadHugeReadPtr(lp:pointer; ucb:UINT):WINBOOL; external 'kernel32' name 'IsBadHugeReadPtr';
function IsBadHugeWritePtr(lp:LPVOID; ucb:UINT):WINBOOL; external 'kernel32' name 'IsBadHugeWritePtr';
function IsBadCodePtr(lpfn:FARPROC):WINBOOL; external 'kernel32' name 'IsBadCodePtr';
function AllocateLocallyUniqueId(Luid:PLUID):WINBOOL; external 'advapi32' name 'AllocateLocallyUniqueId';
function QueryPerformanceCounter(lpPerformanceCount:PLARGE_INTEGER):WINBOOL; external 'kernel32' name 'QueryPerformanceCounter';
function QueryPerformanceFrequency(lpFrequency:PLARGE_INTEGER):WINBOOL; external 'kernel32' name 'QueryPerformanceFrequency';
procedure MoveMemory(Destination:PVOID; Source:pointer; Length:DWORD);
procedure CopyMemory(Destination:PVOID; Source:pointer; Length:DWORD);
procedure FillMemory(Destination:PVOID; Length:DWORD; Fill:BYTE);
procedure ZeroMemory(Destination:PVOID; Length:DWORD);
{$ifdef WIN95}
function ActivateKeyboardLayout(hkl:HKL; Flags:UINT):HKL; external 'user32' name 'ActivateKeyboardLayout';
{$else}
function ActivateKeyboardLayout(hkl:HKL; Flags:UINT):WINBOOL; external 'user32' name 'ActivateKeyboardLayout';
{$endif}
function UnloadKeyboardLayout(hkl:HKL):WINBOOL; external 'user32' name 'UnloadKeyboardLayout';
function GetKeyboardLayoutList(nBuff:longint; var lpList:HKL):longint; external 'user32' name 'GetKeyboardLayoutList';
function GetKeyboardLayout(dwLayout:DWORD):HKL; external 'user32' name 'GetKeyboardLayout';
function OpenInputDesktop(dwFlags:DWORD; fInherit:WINBOOL; dwDesiredAccess:DWORD):HDESK; external 'user32' name 'OpenInputDesktop';
function EnumDesktopWindows(hDesktop:HDESK; lpfn:ENUMWINDOWSPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumDesktopWindows';
function SwitchDesktop(hDesktop:HDESK):WINBOOL; external 'user32' name 'SwitchDesktop';
function SetThreadDesktop(hDesktop:HDESK):WINBOOL; external 'user32' name 'SetThreadDesktop';
function CloseDesktop(hDesktop:HDESK):WINBOOL; external 'user32' name 'CloseDesktop';
function GetThreadDesktop(dwThreadId:DWORD):HDESK; external 'user32' name 'GetThreadDesktop';
function CloseWindowStation(hWinSta:HWINSTA):WINBOOL; external 'user32' name 'CloseWindowStation';
function SetProcessWindowStation(hWinSta:HWINSTA):WINBOOL; external 'user32' name 'SetProcessWindowStation';
function GetProcessWindowStation:HWINSTA; external 'user32' name 'GetProcessWindowStation';
function SetUserObjectSecurity(hObj:HANDLE; pSIRequested:PSECURITY_INFORMATION; pSID:PSECURITY_DESCRIPTOR):WINBOOL; external 'user32' name 'SetUserObjectSecurity';
function CancelIoEx( hfile: THandle; overlapped:LPOverlapped):WINBOOL; external 'kernel32.dll' name 'CancelIoEx';
function CancelIo  ( hfile: THandle):WINBOOL; external 'kernel32.dll' name 'CancelIo';
function GetUserObjectSecurity(hObj:HANDLE; pSIRequested:PSECURITY_INFORMATION; pSID:PSECURITY_DESCRIPTOR; nLength:DWORD; lpnLengthNeeded:LPDWORD):WINBOOL; external 'user32' name 'GetUserObjectSecurity';
function TranslateMessage(lpMsg:LPMSG):WINBOOL; external 'user32' name 'TranslateMessage';
function SetMessageQueue(cMessagesMax:longint):WINBOOL; external 'user32' name 'SetMessageQueue';
function RegisterHotKey(hWnd:HWND; anID:longint; fsModifiers:UINT; vk:UINT):WINBOOL; external 'user32' name 'RegisterHotKey';
function UnregisterHotKey(hWnd:HWND; anID:longint):WINBOOL; external 'user32' name 'UnregisterHotKey';
function ExitWindowsEx(uFlags:UINT; dwReserved:DWORD):WINBOOL; external 'user32' name 'ExitWindowsEx';
function SwapMouseButton(fSwap:WINBOOL):WINBOOL; external 'user32' name 'SwapMouseButton';
function GetMessagePos:DWORD; external 'user32' name 'GetMessagePos';
function GetMessageTime:LONG; external 'user32' name 'GetMessageTime';
function GetMessageExtraInfo:LONG; external 'user32' name 'GetMessageExtraInfo';
function SetMessageExtraInfo(lParam:LPARAM):LPARAM; external 'user32' name 'SetMessageExtraInfo';
function BroadcastSystemMessage(_para1:DWORD; _para2:LPDWORD; _para3:UINT; _para4:WPARAM; _para5:LPARAM):longint; external 'user32' name 'BroadcastSystemMessage';
function AttachThreadInput(idAttach:DWORD; idAttachTo:DWORD; fAttach:WINBOOL):WINBOOL; external 'user32' name 'AttachThreadInput';
function ReplyMessage(lResult:LRESULT):WINBOOL; external 'user32' name 'ReplyMessage';
function WaitMessage:WINBOOL; external 'user32' name 'WaitMessage';
function WaitForInputIdle(hProcess:HANDLE; dwMilliseconds:DWORD):DWORD; external 'user32' name 'WaitForInputIdle';
procedure PostQuitMessage(nExitCode:longint); external 'user32' name 'PostQuitMessage';
function InSendMessage:WINBOOL; external 'user32' name 'InSendMessage';
function GetDoubleClickTime:UINT; external 'user32' name 'GetDoubleClickTime';
function SetDoubleClickTime(_para1:UINT):WINBOOL; external 'user32' name 'SetDoubleClickTime';
function IsWindow(hWnd:HWND):WINBOOL; external 'user32' name 'IsWindow';
function IsMenu(hMenu:HMENU):WINBOOL; external 'user32' name 'IsMenu';
function IsChild(hWndParent:HWND; hWnd:HWND):WINBOOL; external 'user32' name 'IsChild';
function DestroyWindow(hWnd:HWND):WINBOOL; external 'user32' name 'DestroyWindow';
function ShowWindow(hWnd:HWND; nCmdShow:longint):WINBOOL; external 'user32' name 'ShowWindow';
function ShowWindowAsync(hWnd:HWND; nCmdShow:longint):WINBOOL; external 'user32' name 'ShowWindowAsync';
function FlashWindow(hWnd:HWND; bInvert:WINBOOL):WINBOOL; external 'user32' name 'FlashWindow';
function ShowOwnedPopups(hWnd:HWND; fShow:WINBOOL):WINBOOL; external 'user32' name 'ShowOwnedPopups';
function OpenIcon(hWnd:HWND):WINBOOL; external 'user32' name 'OpenIcon';
function CloseWindow(hWnd:HWND):WINBOOL; external 'user32' name 'CloseWindow';
function MoveWindow(hWnd:HWND; X:longint; Y:longint; nWidth:longint; nHeight:longint;bRepaint:WINBOOL):WINBOOL; external 'user32' name 'MoveWindow';
function SetWindowPos(hWnd:HWND; hWndInsertAfter:HWND; X:longint; Y:longint; cx:longint;cy:longint; uFlags:UINT):WINBOOL; external 'user32' name 'SetWindowPos';
function GetWindowPlacement(hWnd:HWND; var lpwndpl:WINDOWPLACEMENT):WINBOOL; external 'user32' name 'GetWindowPlacement';
function SetWindowPlacement(hWnd:HWND; var lpwndpl:WINDOWPLACEMENT):WINBOOL; external 'user32' name 'SetWindowPlacement';
function GetWindowPlacement(hWnd:HWND; lpwndpl:PWINDOWPLACEMENT):WINBOOL; external 'user32' name 'GetWindowPlacement';
function SetWindowPlacement(hWnd:HWND; lpwndpl:PWINDOWPLACEMENT):WINBOOL; external 'user32' name 'SetWindowPlacement';
function BeginDeferWindowPos(nNumWindows:longint):HDWP; external 'user32' name 'BeginDeferWindowPos';
function DeferWindowPos(hWinPosInfo:HDWP; hWnd:HWND; hWndInsertAfter:HWND; x:longint; y:longint;cx:longint; cy:longint; uFlags:UINT):HDWP; external 'user32' name 'DeferWindowPos';
function EndDeferWindowPos(hWinPosInfo:HDWP):WINBOOL; external 'user32' name 'EndDeferWindowPos';
function IsWindowVisible(hWnd:HWND):WINBOOL; external 'user32' name 'IsWindowVisible';
function IsIconic(hWnd:HWND):WINBOOL; external 'user32' name 'IsIconic';
function AnyPopup:WINBOOL; external 'user32' name 'AnyPopup';
function BringWindowToTop(hWnd:HWND):WINBOOL; external 'user32' name 'BringWindowToTop';
function IsZoomed(hWnd:HWND):WINBOOL; external 'user32' name 'IsZoomed';
function EndDialog(hDlg:HWND; nResult:longint):WINBOOL; external 'user32' name 'EndDialog';
function GetDlgItem(hDlg:HWND; nIDDlgItem:longint):HWND; external 'user32' name 'GetDlgItem';
function SetDlgItemInt(hDlg:HWND; nIDDlgItem:longint; uValue:UINT; bSigned:WINBOOL):WINBOOL; external 'user32' name 'SetDlgItemInt';
function GetDlgItemInt(hDlg:HWND; nIDDlgItem:longint; var lpTranslated:WINBOOL; bSigned:WINBOOL):UINT; external 'user32' name 'GetDlgItemInt';
function CheckDlgButton(hDlg:HWND; nIDButton:longint; uCheck:UINT):WINBOOL; external 'user32' name 'CheckDlgButton';
function CheckRadioButton(hDlg:HWND; nIDFirstButton:longint; nIDLastButton:longint; nIDCheckButton:longint):WINBOOL; external 'user32' name 'CheckRadioButton';
function IsDlgButtonChecked(hDlg:HWND; nIDButton:longint):UINT; external 'user32' name 'IsDlgButtonChecked';
function GetNextDlgGroupItem(hDlg:HWND; hCtl:HWND; bPrevious:WINBOOL):HWND; external 'user32' name 'GetNextDlgGroupItem';
function GetNextDlgTabItem(hDlg:HWND; hCtl:HWND; bPrevious:WINBOOL):HWND; external 'user32' name 'GetNextDlgTabItem';
function GetDlgCtrlID(hWnd:HWND):longint; external 'user32' name 'GetDlgCtrlID';
function GetDialogBaseUnits:longint; external 'user32' name 'GetDialogBaseUnits';
function OpenClipboard(hWndNewOwner:HWND):WINBOOL; external 'user32' name 'OpenClipboard';
function CloseClipboard:WINBOOL; external 'user32' name 'CloseClipboard';
function GetClipboardOwner:HWND; external 'user32' name 'GetClipboardOwner';
function SetClipboardViewer(hWndNewViewer:HWND):HWND; external 'user32' name 'SetClipboardViewer';
function GetClipboardViewer:HWND; external 'user32' name 'GetClipboardViewer';
function ChangeClipboardChain(hWndRemove:HWND; hWndNewNext:HWND):WINBOOL; external 'user32' name 'ChangeClipboardChain';
function SetClipboardData(uFormat:UINT; hMem:HANDLE):HANDLE; external 'user32' name 'SetClipboardData';
function GetClipboardData(uFormat:UINT):HANDLE; external 'user32' name 'GetClipboardData';
function CountClipboardFormats:longint; external 'user32' name 'CountClipboardFormats';
function EnumClipboardFormats(format:UINT):UINT; external 'user32' name 'EnumClipboardFormats';
function EmptyClipboard:WINBOOL; external 'user32' name 'EmptyClipboard';
function IsClipboardFormatAvailable(format:UINT):WINBOOL; external 'user32' name 'IsClipboardFormatAvailable';
function GetPriorityClipboardFormat(var paFormatPriorityList:UINT; cFormats:longint):longint; external 'user32' name 'GetPriorityClipboardFormat';
function GetOpenClipboardWindow:HWND; external 'user32' name 'GetOpenClipboardWindow';
function CharNextExA(CodePage:WORD; lpCurrentChar:LPCSTR; dwFlags:DWORD):LPSTR; external 'user32' name 'CharNextExA';
function CharPrevExA(CodePage:WORD; lpStart:LPCSTR; lpCurrentChar:LPCSTR; dwFlags:DWORD):LPSTR; external 'user32' name 'CharPrevExA';
function SetFocus(hWnd:HWND):HWND; external 'user32' name 'SetFocus';
function GetActiveWindow:HWND; external 'user32' name 'GetActiveWindow';
function GetFocus:HWND; external 'user32' name 'GetFocus';
function GetKBCodePage:UINT; external 'user32' name 'GetKBCodePage';
function GetKeyState(nVirtKey:longint):SHORT; external 'user32' name 'GetKeyState';
function GetAsyncKeyState(vKey:longint):SHORT; external 'user32' name 'GetAsyncKeyState';
function GetKeyboardState(lpKeyState:PBYTE):WINBOOL; external 'user32' name 'GetKeyboardState';
function SetKeyboardState(lpKeyState:LPBYTE):WINBOOL; external 'user32' name 'SetKeyboardState';
function GetKeyboardType(nTypeFlag:longint):longint; external 'user32' name 'GetKeyboardType';
function ToAscii(uVirtKey:UINT; uScanCode:UINT; lpKeyState:PBYTE; lpChar:LPWORD; uFlags:UINT):longint; external 'user32' name 'ToAscii';
function ToAsciiEx(uVirtKey:UINT; uScanCode:UINT; lpKeyState:PBYTE; lpChar:LPWORD; uFlags:UINT;dwhkl:HKL):longint; external 'user32' name 'ToAsciiEx';
function ToUnicode(wVirtKey:UINT; wScanCode:UINT; lpKeyState:PBYTE; pwszBuff:LPWSTR; cchBuff:longint;wFlags:UINT):longint; external 'user32' name 'ToUnicode';
function OemKeyScan(wOemChar:WORD):DWORD; external 'user32' name 'OemKeyScan';
procedure keybd_event(bVk:BYTE; bScan:BYTE; dwFlags:DWORD; dwExtraInfo:DWORD); external 'user32' name 'keybd_event';
procedure mouse_event(dwFlags:DWORD; dx:DWORD; dy:DWORD; cButtons:DWORD; dwExtraInfo:DWORD); external 'user32' name 'mouse_event';
function GetInputState:WINBOOL; external 'user32' name 'GetInputState';
function GetQueueStatus(flags:UINT):DWORD; external 'user32' name 'GetQueueStatus';
function GetCapture:HWND; external 'user32' name 'GetCapture';
function SetCapture(hWnd:HWND):HWND; external 'user32' name 'SetCapture';
function ReleaseCapture:WINBOOL; external 'user32' name 'ReleaseCapture';
function MsgWaitForMultipleObjects(nCount:DWORD; pHandles:LPHANDLE; fWaitAll:WINBOOL; dwMilliseconds:DWORD; dwWakeMask:DWORD):DWORD; external 'user32' name 'MsgWaitForMultipleObjects';
function SetTimer(hWnd:HWND; nIDEvent:UINT_PTR; uElapse:UINT; lpTimerFunc:TIMERPROC):UINT_PTR; external 'user32' name 'SetTimer';
function KillTimer(hWnd:HWND; uIDEvent:UINT_PTR):WINBOOL; external 'user32' name 'KillTimer';
function IsWindowUnicode(hWnd:HWND):WINBOOL; external 'user32' name 'IsWindowUnicode';
function EnableWindow(hWnd:HWND; bEnable:WINBOOL):WINBOOL; external 'user32' name 'EnableWindow';
function IsWindowEnabled(hWnd:HWND):WINBOOL; external 'user32' name 'IsWindowEnabled';
function DestroyAcceleratorTable(hAccel:HACCEL):WINBOOL; external 'user32' name 'DestroyAcceleratorTable';
function GetSystemMetrics(nIndex:longint):longint; external 'user32' name 'GetSystemMetrics';
function GetMenu(hWnd:HWND):HMENU; external 'user32' name 'GetMenu';
function SetMenu(hWnd:HWND; hMenu:HMENU):WINBOOL; external 'user32' name 'SetMenu';
function HiliteMenuItem(hWnd:HWND; hMenu:HMENU; uIDHiliteItem:UINT; uHilite:UINT):WINBOOL; external 'user32' name 'HiliteMenuItem';
function GetMenuState(hMenu:HMENU; uId:UINT; uFlags:UINT):UINT; external 'user32' name 'GetMenuState';
function DrawMenuBar(hWnd:HWND):WINBOOL; external 'user32' name 'DrawMenuBar';
function GetSystemMenu(hWnd:HWND; bRevert:WINBOOL):HMENU; external 'user32' name 'GetSystemMenu';
function CreateMenu:HMENU; external 'user32' name 'CreateMenu';
function CreatePopupMenu:HMENU; external 'user32' name 'CreatePopupMenu';
function DestroyMenu(hMenu:HMENU):WINBOOL; external 'user32' name 'DestroyMenu';
function CheckMenuItem(hMenu:HMENU; uIDCheckItem:UINT; uCheck:UINT):DWORD; external 'user32' name 'CheckMenuItem';
function EnableMenuItem(hMenu:HMENU; uIDEnableItem:UINT; uEnable:UINT):WINBOOL; external 'user32' name 'EnableMenuItem';
function GetSubMenu(hMenu:HMENU; nPos:longint):HMENU; external 'user32' name 'GetSubMenu';
function GetMenuItemID(hMenu:HMENU; nPos:longint):UINT; external 'user32' name 'GetMenuItemID';
function GetMenuItemCount(hMenu:HMENU):longint; external 'user32' name 'GetMenuItemCount';
function RemoveMenu(hMenu:HMENU; uPosition:UINT; uFlags:UINT):WINBOOL; external 'user32' name 'RemoveMenu';
function DeleteMenu(hMenu:HMENU; uPosition:UINT; uFlags:UINT):WINBOOL; external 'user32' name 'DeleteMenu';
function SetMenuItemBitmaps(hMenu:HMENU; uPosition:UINT; uFlags:UINT; hBitmapUnchecked:HBITMAP; hBitmapChecked:HBITMAP):WINBOOL; external 'user32' name 'SetMenuItemBitmaps';
function GetMenuCheckMarkDimensions:LONG; external 'user32' name 'GetMenuCheckMarkDimensions';
function TrackPopupMenu(hMenu:HMENU; uFlags:UINT; x:longint; y:longint; nReserved:longint;hWnd:HWND; const prcRect:RECT):WINBOOL;  external 'user32' name 'TrackPopupMenu';
function GetMenuDefaultItem(hMenu:HMENU; fByPos:UINT; gmdiFlags:UINT):UINT; external 'user32' name 'GetMenuDefaultItem';
function SetMenuDefaultItem(hMenu:HMENU; uItem:UINT; fByPos:UINT):WINBOOL; external 'user32' name 'SetMenuDefaultItem';
function GetMenuItemRect(hWnd:HWND; hMenu:HMENU; uItem:UINT; lprcItem:LPRECT):WINBOOL; external 'user32' name 'GetMenuItemRect';
function MenuItemFromPoint(hWnd:HWND; hMenu:HMENU; ptScreen:POINT):longint; external 'user32' name 'MenuItemFromPoint';
function DragObject(_para1:HWND; _para2:HWND; _para3:UINT; _para4:DWORD; _para5:HCURSOR):DWORD; external 'user32' name 'DragObject';
function DragDetect(hwnd:HWND; pt:POINT):WINBOOL; external 'user32' name 'DragDetect';
function DrawIcon(hDC:HDC; X:longint; Y:longint; hIcon:HICON):WINBOOL; external 'user32' name 'DrawIcon';
function UpdateWindow(hWnd:HWND):WINBOOL; external 'user32' name 'UpdateWindow';
function SetActiveWindow(hWnd:HWND):HWND; external 'user32' name 'SetActiveWindow';
function GetForegroundWindow:HWND; external 'user32' name 'GetForegroundWindow';
function PaintDesktop(hdc:HDC):WINBOOL; external 'user32' name 'PaintDesktop';
function SetForegroundWindow(hWnd:HWND):WINBOOL; external 'user32' name 'SetForegroundWindow';
function WindowFromDC(hDC:HDC):HWND; external 'user32' name 'WindowFromDC';
function GetDC(hWnd:HWND):HDC; external 'user32' name 'GetDC';
function GetDCEx(hWnd:HWND; hrgnClip:HRGN; flags:DWORD):HDC; external 'user32' name 'GetDCEx';
function GetWindowDC(hWnd:HWND):HDC; external 'user32' name 'GetWindowDC';
function ReleaseDC(hWnd:HWND; hDC:HDC):longint; external 'user32' name 'ReleaseDC';
function BeginPaint(hWnd:HWND; lpPaint:LPPAINTSTRUCT):HDC; external 'user32' name 'BeginPaint';
function EndPaint(hWnd:HWND; lpPaint:LPPAINTSTRUCT):WINBOOL; external 'user32' name 'EndPaint';
function GetUpdateRect(hWnd:HWND; lpRect:LPRECT; bErase:WINBOOL):WINBOOL; external 'user32' name 'GetUpdateRect';
function GetUpdateRgn(hWnd:HWND; hRgn:HRGN; bErase:WINBOOL):longint; external 'user32' name 'GetUpdateRgn';
function SetWindowRgn(hWnd:HWND; hRgn:HRGN; bRedraw:WINBOOL):longint; external 'user32' name 'SetWindowRgn';
function GetWindowRgn(hWnd:HWND; hRgn:HRGN):longint; external 'user32' name 'GetWindowRgn';
function ExcludeUpdateRgn(hDC:HDC; hWnd:HWND):longint; external 'user32' name 'ExcludeUpdateRgn';
function InvalidateRect(hWnd:HWND; const lpRect:RECT; bErase:WINBOOL):WINBOOL; external 'user32' name 'InvalidateRect';
function InvalidateRect(hWnd:HWND;lpRect:LPRECT; bErase:WINBOOL):WINBOOL; external 'user32' name 'InvalidateRect';
function ValidateRect(hWnd:HWND; const lpRect:RECT):WINBOOL; external 'user32' name 'ValidateRect';
function ValidateRect(hWnd:HWND;lpRect:LPRECT):WINBOOL; external 'user32' name 'ValidateRect';
function InvalidateRgn(hWnd:HWND; hRgn:HRGN; bErase:WINBOOL):WINBOOL; external 'user32' name 'InvalidateRgn';
function ValidateRgn(hWnd:HWND; hRgn:HRGN):WINBOOL; external 'user32' name 'ValidateRgn';
function RedrawWindow(hWnd:HWND; const lprcUpdate:RECT; hrgnUpdate:HRGN; flags:UINT):WINBOOL; external 'user32' name 'RedrawWindow';
function RedrawWindow(hWnd:HWND; lprcUpdate:LPRECT; hrgnUpdate:HRGN; flags:UINT):WINBOOL; external 'user32' name 'RedrawWindow';
function LockWindowUpdate(hWndLock:HWND):WINBOOL; external 'user32' name 'LockWindowUpdate';
function ScrollWindow(hWnd:HWND; XAmount:longint; YAmount:longint; const lpRect:RECT; const lpClipRect:RECT):WINBOOL; external 'user32' name 'ScrollWindow';
function ScrollDC(hDC:HDC; dx:longint; dy:longint; const lprcScroll:RECT; const lprcClip:RECT;hrgnUpdate:HRGN; lprcUpdate:LPRECT):WINBOOL; external 'user32' name 'ScrollDC';
function ScrollWindowEx(hWnd:HWND; dx:longint; dy:longint; const prcScroll:RECT; const prcClip:RECT;hrgnUpdate:HRGN; prcUpdate:LPRECT; flags:UINT):longint; external 'user32' name 'ScrollWindowEx';
function SetScrollPos(hWnd:HWND; nBar:longint; nPos:longint; bRedraw:WINBOOL):longint; external 'user32' name 'SetScrollPos';
function GetScrollPos(hWnd:HWND; nBar:longint):longint; external 'user32' name 'GetScrollPos';
function SetScrollRange(hWnd:HWND; nBar:longint; nMinPos:longint; nMaxPos:longint; bRedraw:WINBOOL):WINBOOL; external 'user32' name 'SetScrollRange';
function GetScrollRange(hWnd:HWND; nBar:longint; lpMinPos:LPINT; lpMaxPos:LPINT):WINBOOL; external 'user32' name 'GetScrollRange';
function ShowScrollBar(hWnd:HWND; wBar:longint; bShow:WINBOOL):WINBOOL; external 'user32' name 'ShowScrollBar';
function EnableScrollBar(hWnd:HWND; wSBflags:UINT; wArrows:UINT):WINBOOL; external 'user32' name 'EnableScrollBar';
function GetClientRect(hWnd:HWND; lpRect:LPRECT):WINBOOL; external 'user32' name 'GetClientRect';
function GetWindowRect(hWnd:HWND; lpRect:LPRECT):WINBOOL; external 'user32' name 'GetWindowRect';
function AdjustWindowRect(lpRect:LPRECT; dwStyle:DWORD; bMenu:WINBOOL):WINBOOL; external 'user32' name 'AdjustWindowRect';
function AdjustWindowRectEx(lpRect:LPRECT; dwStyle:DWORD; bMenu:WINBOOL; dwExStyle:DWORD):WINBOOL; external 'user32' name 'AdjustWindowRectEx';
function SetWindowContextHelpId(_para1:HWND; _para2:DWORD):WINBOOL; external 'user32' name 'SetWindowContextHelpId';
function GetWindowContextHelpId(_para1:HWND):DWORD; external 'user32' name 'GetWindowContextHelpId';
function SetMenuContextHelpId(_para1:HMENU; _para2:DWORD):WINBOOL; external 'user32' name 'SetMenuContextHelpId';
function GetMenuContextHelpId(_para1:HMENU):DWORD; external 'user32' name 'GetMenuContextHelpId';
function MessageBeep(uType:UINT):WINBOOL; external 'user32' name 'MessageBeep';
function ShowCursor(bShow:WINBOOL):longint; external 'user32' name 'ShowCursor';
function SetCursorPos(X:longint; Y:longint):WINBOOL; external 'user32' name 'SetCursorPos';
function SetCursor(hCursor:HCURSOR):HCURSOR; external 'user32' name 'SetCursor';
function GetCursorPos(lpPoint:LPPOINT):WINBOOL; external 'user32' name 'GetCursorPos';
function ClipCursor(lpRect:LPRECT):WINBOOL; external 'user32' name 'ClipCursor';
function GetClipCursor(lpRect:LPRECT):WINBOOL; external 'user32' name 'GetClipCursor';
function GetCursor:HCURSOR; external 'user32' name 'GetCursor';
function CreateCaret(hWnd:HWND; hBitmap:HBITMAP; nWidth:longint; nHeight:longint):WINBOOL; external 'user32' name 'CreateCaret';
function GetCaretBlinkTime:UINT; external 'user32' name 'GetCaretBlinkTime';
function SetCaretBlinkTime(uMSeconds:UINT):WINBOOL; external 'user32' name 'SetCaretBlinkTime';
function DestroyCaret:WINBOOL; external 'user32' name 'DestroyCaret';
function HideCaret(hWnd:HWND):WINBOOL; external 'user32' name 'HideCaret';
function ShowCaret(hWnd:HWND):WINBOOL; external 'user32' name 'ShowCaret';
function SetCaretPos(X:longint; Y:longint):WINBOOL; external 'user32' name 'SetCaretPos';
function GetCaretPos(lpPoint:LPPOINT):WINBOOL; external 'user32' name 'GetCaretPos';
function ClientToScreen(hWnd:HWND; lpPoint:LPPOINT):WINBOOL; external 'user32' name 'ClientToScreen';
function ScreenToClient(hWnd:HWND; lpPoint:LPPOINT):WINBOOL; external 'user32' name 'ScreenToClient';
function MapWindowPoints(hWndFrom:HWND; hWndTo:HWND; lpPoints:LPPOINT; cPoints:UINT):longint; external 'user32' name 'MapWindowPoints';
function WindowFromPoint(Point:POINT):HWND; external 'user32' name 'WindowFromPoint';
function ChildWindowFromPoint(hWndParent:HWND; Point:POINT):HWND; external 'user32' name 'ChildWindowFromPoint';
function GetSysColor(nIndex:longint):DWORD; external 'user32' name 'GetSysColor';
function GetSysColorBrush(nIndex:longint):HBRUSH; external 'user32' name 'GetSysColorBrush';
function SetSysColors(cElements:longint; var lpaElements:wINT; var lpaRgbValues:COLORREF):WINBOOL; external 'user32' name 'SetSysColors';
function DrawFocusRect(hDC:HDC; const lprc:RECT):WINBOOL; external 'user32' name 'DrawFocusRect';
function FillRect(hDC:HDC; const lprc:RECT; hbr:HBRUSH):longint; external 'user32' name 'FillRect';
function FrameRect(hDC:HDC; const lprc:RECT; hbr:HBRUSH):longint; external 'user32' name 'FrameRect';
function InvertRect(hDC:HDC; const lprc:RECT):WINBOOL; external 'user32' name 'InvertRect';
function SetRect(lprc:LPRECT; xLeft:longint; yTop:longint; xRight:longint; yBottom:longint):WINBOOL; external 'user32' name 'SetRect';
function SetRectEmpty(lprc:LPRECT):WINBOOL; external 'user32' name 'SetRectEmpty';
function CopyRect(lprcDst:LPRECT; const lprcSrc:RECT):WINBOOL; external 'user32' name 'CopyRect';
function InflateRect(lprc:LPRECT; dx:longint; dy:longint):WINBOOL; external 'user32' name 'InflateRect';
function IntersectRect(lprcDst:LPRECT; const lprcSrc1:RECT; const lprcSrc2:RECT):WINBOOL; external 'user32' name 'IntersectRect';
function UnionRect(lprcDst:LPRECT; const lprcSrc1:RECT; const lprcSrc2:RECT):WINBOOL; external 'user32' name 'UnionRect';
function SubtractRect(lprcDst:LPRECT; const lprcSrc1:RECT; const lprcSrc2:RECT):WINBOOL; external 'user32' name 'SubtractRect';
function OffsetRect(lprc:LPRECT; dx:longint; dy:longint):WINBOOL; external 'user32' name 'OffsetRect';
function IsRectEmpty(const lprc:RECT):WINBOOL; external 'user32' name 'IsRectEmpty';
function EqualRect(const lprc1:RECT; const lprc2:RECT):WINBOOL; external 'user32' name 'EqualRect';
function PtInRect(const lprc:RECT; pt:POINT):WINBOOL; external 'user32' name 'PtInRect';
function PtInRect(lprc:LPRECT; pt:POINT):WINBOOL; external 'user32' name 'PtInRect';
function GetWindowWord(hWnd:HWND; nIndex:longint):WORD; external 'user32' name 'GetWindowWord';
function SetWindowWord(hWnd:HWND; nIndex:longint; wNewWord:WORD):WORD; external 'user32' name 'SetWindowWord';
function GetClassWord(hWnd:HWND; nIndex:longint):WORD; external 'user32' name 'GetClassWord';
function SetClassWord(hWnd:HWND; nIndex:longint; wNewWord:WORD):WORD; external 'user32' name 'SetClassWord';
function GetDesktopWindow:HWND; external 'user32' name 'GetDesktopWindow';
function GetParent(hWnd:HWND):HWND; external 'user32' name 'GetParent';
function SetParent(hWndChild:HWND; hWndNewParent:HWND):HWND; external 'user32' name 'SetParent';
function EnumChildWindows(hWndParent:HWND; lpEnumFunc:ENUMWINDOWSPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumChildWindows';
function EnumWindows(lpEnumFunc:ENUMWINDOWSPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumWindows';
function EnumThreadWindows(dwThreadId:DWORD; lpfn:ENUMWINDOWSPROC; lParam:LPARAM):WINBOOL; external 'user32' name 'EnumThreadWindows';
function EnumTaskWindows(hTask:HWND; lpfn:FARPROC; lParam: LPARAM): BOOL;external 'user32' name 'EnumThreadWindows';
function GetTopWindow(hWnd:HWND):HWND; external 'user32' name 'GetTopWindow';
function GetWindowThreadProcessId(hWnd:HWND; lpdwProcessId:LPDWORD):DWORD; external 'user32' name 'GetWindowThreadProcessId';
function GetLastActivePopup(hWnd:HWND):HWND; external 'user32' name 'GetLastActivePopup';
function GetWindow(hWnd:HWND; uCmd:UINT):HWND; external 'user32' name 'GetWindow';
function UnhookWindowsHook(nCode:longint; pfnFilterProc:HOOKPROC):WINBOOL; external 'user32' name 'UnhookWindowsHook';
function UnhookWindowsHookEx(hhk:HHOOK):WINBOOL; external 'user32' name 'UnhookWindowsHookEx';
function CallNextHookEx(hhk:HHOOK; nCode:longint; wParam:WPARAM; lParam:LPARAM):LRESULT; external 'user32' name 'CallNextHookEx';
function CheckMenuRadioItem(_para1:HMENU; _para2:UINT; _para3:UINT; _para4:UINT; _para5:UINT):WINBOOL; external 'user32' name 'CheckMenuRadioItem';
function CreateCursor(hInst:HINST; xHotSpot:longint; yHotSpot:longint; nWidth:longint; nHeight:longint;pvANDPlane:pointer; pvXORPlane:pointer):HCURSOR; external 'user32' name 'CreateCursor';
function DestroyCursor(hCursor:HCURSOR):WINBOOL; external 'user32' name 'DestroyCursor';
function SetSystemCursor(hcur:HCURSOR; anID:DWORD):WINBOOL; external 'user32' name 'SetSystemCursor';
function CreateIcon(hInstance:HINST; nWidth:longint; nHeight:longint; cPlanes:BYTE; cBitsPixel:BYTE;var lpbANDbits:BYTE; var lpbXORbits:BYTE):HICON; external 'user32' name 'CreateIcon';
function DestroyIcon(hIcon:HICON):WINBOOL; external 'user32' name 'DestroyIcon';
function LookupIconIdFromDirectory(presbits:PBYTE; fIcon:WINBOOL):longint; external 'user32' name 'LookupIconIdFromDirectory';
function LookupIconIdFromDirectoryEx(presbits:PBYTE; fIcon:WINBOOL; cxDesired:longint; cyDesired:longint; Flags:UINT):longint; external 'user32' name 'LookupIconIdFromDirectoryEx';
function CreateIconFromResource(presbits:PBYTE; dwResSize:DWORD; fIcon:WINBOOL; dwVer:DWORD):HICON; external 'user32' name 'CreateIconFromResource';
function CreateIconFromResourceEx(presbits:PBYTE; dwResSize:DWORD; fIcon:WINBOOL; dwVer:DWORD; cxDesired:longint;cyDesired:longint; Flags:UINT):HICON; external 'user32' name 'CreateIconFromResourceEx';
function CopyImage(_para1:HANDLE; _para2:UINT; _para3:longint; _para4:longint; _para5:UINT):HICON; external 'user32' name 'CopyImage';
function CreateIconIndirect(piconinfo:PICONINFO):HICON; external 'user32' name 'CreateIconIndirect';
function CopyIcon(hIcon:HICON):HICON; external 'user32' name 'CopyIcon';
function GetIconInfo(hIcon:HICON; piconinfo:PICONINFO):WINBOOL; external 'user32' name 'GetIconInfo';
function MapDialogRect(hDlg:HWND; lpRect:LPRECT):WINBOOL; external 'user32' name 'MapDialogRect';
function SetScrollInfo(_para1:HWND; _para2:longint; _para3:LPCSCROLLINFO; _para4:WINBOOL):longint; external 'user32' name 'SetScrollInfo';
function GetScrollInfo(_para1:HWND; _para2:longint; _para3:LPSCROLLINFO):WINBOOL; external 'user32' name 'GetScrollInfo';
function TranslateMDISysAccel(hWndClient:HWND; lpMsg:LPMSG):WINBOOL; external 'user32' name 'TranslateMDISysAccel';
function ArrangeIconicWindows(hWnd:HWND):UINT; external 'user32' name 'ArrangeIconicWindows';
function TileWindows(hwndParent:HWND; wHow:UINT; const lpRect:RECT; cKids:UINT; var lpKids:HWND):WORD; external 'user32' name 'TileWindows';
function CascadeWindows(hwndParent:HWND; wHow:UINT; const lpRect:RECT; cKids:UINT; var lpKids:HWND):WORD; external 'user32' name 'CascadeWindows';
procedure SetLastErrorEx(dwErrCode:DWORD; dwType:DWORD); external 'user32' name 'SetLastErrorEx';
procedure SetDebugErrorLevel(dwLevel:DWORD); external 'user32' name 'SetDebugErrorLevel';
function DrawEdge(hdc:HDC; qrc:LPRECT; edge:UINT; grfFlags:UINT):WINBOOL; external 'user32' name 'DrawEdge';
function DrawFrameControl(_para1:HDC; _para2:LPRECT; _para3:UINT; _para4:UINT):WINBOOL; external 'user32' name 'DrawFrameControl';
function DrawCaption(_para1:HWND; _para2:HDC; var _para3:RECT; _para4:UINT):WINBOOL; external 'user32' name 'DrawCaption';
function DrawAnimatedRects(hwnd:HWND; idAni:longint; const lprcFrom:RECT; const lprcTo:RECT):WINBOOL; external 'user32' name 'DrawAnimatedRects';
function TrackPopupMenuEx(_para1:HMENU; _para2:UINT; _para3:longint; _para4:longint; _para5:HWND;_para6:LPTPMPARAMS):WINBOOL; external 'user32' name 'TrackPopupMenuEx';
function ChildWindowFromPointEx(_para1:HWND; _para2:POINT; _para3:UINT):HWND; external 'user32' name 'ChildWindowFromPointEx';
function DrawIconEx(hdc:HDC; xLeft:longint; yTop:longint; hIcon:HICON; cxWidth:longint;cyWidth:longint; istepIfAniCur:UINT; hbrFlickerFreeDraw:HBRUSH; diFlags:UINT):WINBOOL; external 'user32' name 'DrawIconEx';
function GetWindowInfo(hWnd:HWND; pwi:PWindowInfo):WINBOOL; external 'user32' name 'GetWindowInfo';

function AnimatePalette(_para1:HPALETTE; _para2:UINT; _para3:UINT; var _para4:PALETTEENTRY):WINBOOL; external 'gdi32' name 'AnimatePalette';
function Arc(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:longint; _para7:longint; _para8:longint; _para9:longint):WINBOOL; external 'gdi32' name 'Arc';
function BitBlt(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:HDC; _para7:longint; _para8:longint; _para9:DWORD):WINBOOL; external 'gdi32' name 'BitBlt';
function CancelDC(_para1:HDC):WINBOOL; external 'gdi32' name 'CancelDC';
function Chord(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:longint; _para7:longint; _para8:longint; _para9:longint):WINBOOL; external 'gdi32' name 'Chord';
function CloseMetaFile(_para1:HDC):HMETAFILE; external 'gdi32' name 'CloseMetaFile';
function CombineRgn(_para1:HRGN; _para2:HRGN; _para3:HRGN; _para4:longint):longint; external 'gdi32' name 'CombineRgn';
function CreateBitmap(_para1:longint; _para2:longint; _para3:UINT; _para4:UINT; _para5:pointer):HBITMAP; external 'gdi32' name 'CreateBitmap';
function CreateBitmapIndirect(var _para1:BITMAP):HBITMAP; external 'gdi32' name 'CreateBitmapIndirect';
function CreateBrushIndirect(var _para1:LOGBRUSH):HBRUSH; external 'gdi32' name 'CreateBrushIndirect';
function CreateCompatibleBitmap(_para1:HDC; _para2:longint; _para3:longint):HBITMAP; external 'gdi32' name 'CreateCompatibleBitmap';
function CreateDiscardableBitmap(_para1:HDC; _para2:longint; _para3:longint):HBITMAP; external 'gdi32' name 'CreateDiscardableBitmap';
function CreateCompatibleDC(_para1:HDC):HDC; external 'gdi32' name 'CreateCompatibleDC';
function CreateDIBitmap(_para1:HDC; var _para2:BITMAPINFOHEADER; _para3:DWORD; _para4:pointer; var _para5:BITMAPINFO;_para6:UINT):HBITMAP; external 'gdi32' name 'CreateDIBitmap';
function CreateDIBPatternBrush(_para1:HGLOBAL; _para2:UINT):HBRUSH; external 'gdi32' name 'CreateDIBPatternBrush';
function CreateDIBPatternBrushPt(_para1:pointer; _para2:UINT):HBRUSH; external 'gdi32' name 'CreateDIBPatternBrushPt';
function CreateEllipticRgn(_para1:longint; _para2:longint; _para3:longint; _para4:longint):HRGN; external 'gdi32' name 'CreateEllipticRgn';
function CreateEllipticRgnIndirect(const _para1:RECT):HRGN; external 'gdi32' name 'CreateEllipticRgnIndirect';
function CreateHatchBrush(_para1:longint; _para2:COLORREF):HBRUSH; external 'gdi32' name 'CreateHatchBrush';
function CreatePalette(var _para1:LOGPALETTE):HPALETTE; external 'gdi32' name 'CreatePalette';
function CreatePen(_para1:longint; _para2:longint; _para3:COLORREF):HPEN; external 'gdi32' name 'CreatePen';
function CreatePenIndirect(var _para1:LOGPEN):HPEN; external 'gdi32' name 'CreatePenIndirect';
function CreatePolyPolygonRgn(var _para1:POINT; var _para2:wINT; _para3:longint; _para4:longint):HRGN; external 'gdi32' name 'CreatePolyPolygonRgn';
function CreatePatternBrush(_para1:HBITMAP):HBRUSH; external 'gdi32' name 'CreatePatternBrush';
function CreateRectRgn(_para1:longint; _para2:longint; _para3:longint; _para4:longint):HRGN; external 'gdi32' name 'CreateRectRgn';
function CreateRectRgnIndirect(const _para1:RECT):HRGN; external 'gdi32' name 'CreateRectRgnIndirect';
function CreateRoundRectRgn(_para1:longint; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:longint):HRGN; external 'gdi32' name 'CreateRoundRectRgn';
function CreateSolidBrush(_para1:COLORREF):HBRUSH; external 'gdi32' name 'CreateSolidBrush';
function DeleteDC(_para1:HDC):WINBOOL; external 'gdi32' name 'DeleteDC';
function DeleteMetaFile(_para1:HMETAFILE):WINBOOL; external 'gdi32' name 'DeleteMetaFile';
function DeleteObject(_para1:HGDIOBJ):WINBOOL; external 'gdi32' name 'DeleteObject';
function DrawEscape(_para1:HDC; _para2:longint; _para3:longint; _para4:LPCSTR):longint; external 'gdi32' name 'DrawEscape';
function Ellipse(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint):WINBOOL; external 'gdi32' name 'Ellipse';
function EnumObjects(_para1:HDC; _para2:longint; _para3:ENUMOBJECTSPROC; _para4:LPARAM):longint; external 'gdi32' name 'EnumObjects';
function EqualRgn(_para1:HRGN; _para2:HRGN):WINBOOL; external 'gdi32' name 'EqualRgn';
function Escape(_para1:HDC; _para2:longint; _para3:longint; _para4:LPCSTR; _para5:LPVOID):longint; external 'gdi32' name 'Escape';
function ExtEscape(_para1:HDC; _para2:longint; _para3:longint; _para4:LPCSTR; _para5:longint;_para6:LPSTR):longint; external 'gdi32' name 'ExtEscape';
function ExcludeClipRect(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint):longint; external 'gdi32' name 'ExcludeClipRect';
function ExtCreateRegion(var _para1:XFORM; _para2:DWORD; var _para3:RGNDATA):HRGN; external 'gdi32' name 'ExtCreateRegion';
function ExtFloodFill(_para1:HDC; _para2:longint; _para3:longint; _para4:COLORREF; _para5:UINT):WINBOOL; external 'gdi32' name 'ExtFloodFill';
function FillRgn(_para1:HDC; _para2:HRGN; _para3:HBRUSH):WINBOOL; external 'gdi32' name 'FillRgn';
function FloodFill(_para1:HDC; _para2:longint; _para3:longint; _para4:COLORREF):WINBOOL; external 'gdi32' name 'FloodFill';
function FrameRgn(_para1:HDC; _para2:HRGN; _para3:HBRUSH; _para4:longint; _para5:longint):WINBOOL; external 'gdi32' name 'FrameRgn';
function GetROP2(_para1:HDC):longint; external 'gdi32' name 'GetROP2';
function GetAspectRatioFilterEx(_para1:HDC; _para2:LPSIZE):WINBOOL; external 'gdi32' name 'GetAspectRatioFilterEx';
function GetBkColor(_para1:HDC):COLORREF; external 'gdi32' name 'GetBkColor';
function GetBkMode(_para1:HDC):longint; external 'gdi32' name 'GetBkMode';
function GetBitmapBits(_para1:HBITMAP; _para2:LONG; _para3:LPVOID):LONG; external 'gdi32' name 'GetBitmapBits';
function GetBitmapDimensionEx(_para1:HBITMAP; _para2:LPSIZE):WINBOOL; external 'gdi32' name 'GetBitmapDimensionEx';
function GetBoundsRect(_para1:HDC; _para2:LPRECT; _para3:UINT):UINT; external 'gdi32' name 'GetBoundsRect';
function GetBrushOrgEx(_para1:HDC; _para2:LPPOINT):WINBOOL; external 'gdi32' name 'GetBrushOrgEx';
function GetClipBox(_para1:HDC; _para2:LPRECT):longint; external 'gdi32' name 'GetClipBox';
function GetClipRgn(_para1:HDC; _para2:HRGN):longint; external 'gdi32' name 'GetClipRgn';
function GetMetaRgn(_para1:HDC; _para2:HRGN):longint; external 'gdi32' name 'GetMetaRgn';
function GetCurrentObject(_para1:HDC; _para2:UINT):HGDIOBJ; external 'gdi32' name 'GetCurrentObject';
function GetCurrentPositionEx(_para1:HDC; _para2:LPPOINT):WINBOOL; external 'gdi32' name 'GetCurrentPositionEx';
function GetDeviceCaps(_para1:HDC; _para2:longint):longint; external 'gdi32' name 'GetDeviceCaps';
function GetDIBits(_para1:HDC; _para2:HBITMAP; _para3:UINT; _para4:UINT; _para5:LPVOID;_para6:LPBITMAPINFO; _para7:UINT):longint; external 'gdi32' name 'GetDIBits';
function GetFontData(_para1:HDC; _para2:DWORD; _para3:DWORD; _para4:LPVOID; _para5:DWORD):DWORD; external 'gdi32' name 'GetFontData';
function GetGraphicsMode(_para1:HDC):longint; external 'gdi32' name 'GetGraphicsMode';
function GetMapMode(_para1:HDC):longint; external 'gdi32' name 'GetMapMode';
function GetMetaFileBitsEx(_para1:HMETAFILE; _para2:UINT; _para3:LPVOID):UINT; external 'gdi32' name 'GetMetaFileBitsEx';
function GetNearestColor(_para1:HDC; _para2:COLORREF):COLORREF; external 'gdi32' name 'GetNearestColor';
function GetNearestPaletteIndex(_para1:HPALETTE; _para2:COLORREF):UINT; external 'gdi32' name 'GetNearestPaletteIndex';
function GetObjectType(h:HGDIOBJ):DWORD; external 'gdi32' name 'GetObjectType';
function GetPaletteEntries(_para1:HPALETTE; _para2:UINT; _para3:UINT; _para4:LPPALETTEENTRY):UINT; external 'gdi32' name 'GetPaletteEntries';
function GetPixel(_para1:HDC; _para2:longint; _para3:longint):COLORREF; external 'gdi32' name 'GetPixel';
function GetPixelFormat(DC:HDC):longint; external 'gdi32' name 'GetPixelFormat';
function GetPolyFillMode(_para1:HDC):longint; external 'gdi32' name 'GetPolyFillMode';
function GetRasterizerCaps(_para1:LPRASTERIZER_STATUS; _para2:UINT):WINBOOL; external 'gdi32' name 'GetRasterizerCaps';
function GetRegionData(_para1:HRGN; _para2:DWORD; _para3:LPRGNDATA):DWORD; external 'gdi32' name 'GetRegionData';
function GetRgnBox(_para1:HRGN; _para2:LPRECT):longint; external 'gdi32' name 'GetRgnBox';
function GetStockObject(_para1:longint):HGDIOBJ; external 'gdi32' name 'GetStockObject';
function GetStretchBltMode(_para1:HDC):longint; external 'gdi32' name 'GetStretchBltMode';
function GetSystemPaletteEntries(_para1:HDC; _para2:UINT; _para3:UINT; _para4:LPPALETTEENTRY):UINT; external 'gdi32' name 'GetSystemPaletteEntries';
function GetSystemPaletteUse(_para1:HDC):UINT; external 'gdi32' name 'GetSystemPaletteUse';
function GetTextCharacterExtra(_para1:HDC):longint; external 'gdi32' name 'GetTextCharacterExtra';
function GetTextAlign(_para1:HDC):UINT; external 'gdi32' name 'GetTextAlign';
function GetTextColor(_para1:HDC):COLORREF; external 'gdi32' name 'GetTextColor';
function GetTextCharset(hdc:HDC):longint; external 'gdi32' name 'GetTextCharset';
function GetTextCharsetInfo(hdc:HDC; lpSig:LPFONTSIGNATURE; dwFlags:DWORD):longint; external 'gdi32' name 'GetTextCharsetInfo';
function TranslateCharsetInfo(var lpSrc:DWORD; lpCs:LPCHARSETINFO; dwFlags:DWORD):WINBOOL; external 'gdi32' name 'TranslateCharsetInfo';
function GetFontLanguageInfo(_para1:HDC):DWORD; external 'gdi32' name 'GetFontLanguageInfo';
function GetViewportExtEx(_para1:HDC; _para2:LPSIZE):WINBOOL; external 'gdi32' name 'GetViewportExtEx';
function GetViewportOrgEx(_para1:HDC; _para2:LPPOINT):WINBOOL; external 'gdi32' name 'GetViewportOrgEx';
function GetWindowExtEx(_para1:HDC; _para2:LPSIZE):WINBOOL; external 'gdi32' name 'GetWindowExtEx';
function GetWindowOrgEx(_para1:HDC; _para2:LPPOINT):WINBOOL; external 'gdi32' name 'GetWindowOrgEx';
function IntersectClipRect(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint):longint; external 'gdi32' name 'IntersectClipRect';
function InvertRgn(_para1:HDC; _para2:HRGN):WINBOOL; external 'gdi32' name 'InvertRgn';
function LineDDA(_para1:longint; _para2:longint; _para3:longint; _para4:longint; _para5:LINEDDAPROC;_para6:LPARAM):WINBOOL; external 'gdi32' name 'LineDDA';
function LineTo(_para1:HDC; _para2:longint; _para3:longint):WINBOOL; external 'gdi32' name 'LineTo';
function MaskBlt(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:HDC; _para7:longint; _para8:longint; _para9:HBITMAP; _para10:longint;_para11:longint; _para12:DWORD):WINBOOL; external 'gdi32' name 'MaskBlt';
function PlgBlt(_para1:HDC; var _para2:POINT; _para3:HDC; _para4:longint; _para5:longint;_para6:longint; _para7:longint; _para8:HBITMAP; _para9:longint; _para10:longint):WINBOOL; external 'gdi32' name 'PlgBlt';
function OffsetClipRgn(_para1:HDC; _para2:longint; _para3:longint):longint; external 'gdi32' name 'OffsetClipRgn';
function OffsetRgn(_para1:HRGN; _para2:longint; _para3:longint):longint; external 'gdi32' name 'OffsetRgn';
function PatBlt(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:DWORD):WINBOOL; external 'gdi32' name 'PatBlt';
function Pie(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:longint; _para7:longint; _para8:longint; _para9:longint):WINBOOL; external 'gdi32' name 'Pie';
function PlayMetaFile(_para1:HDC; _para2:HMETAFILE):WINBOOL; external 'gdi32' name 'PlayMetaFile';
function PaintRgn(_para1:HDC; _para2:HRGN):WINBOOL; external 'gdi32' name 'PaintRgn';
function PolyPolygon(_para1:HDC; var _para2:POINT; var _para3:wINT; _para4:longint):WINBOOL; external 'gdi32' name 'PolyPolygon';
function PtInRegion(_para1:HRGN; _para2:longint; _para3:longint):WINBOOL; external 'gdi32' name 'PtInRegion';
function PtVisible(_para1:HDC; _para2:longint; _para3:longint):WINBOOL; external 'gdi32' name 'PtVisible';
function RectInRegion(_para1:HRGN; const _para2:RECT):WINBOOL; external 'gdi32' name 'RectInRegion';
function RectVisible(_para1:HDC; const _para2:RECT):WINBOOL; external 'gdi32' name 'RectVisible';
function Rectangle(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint):WINBOOL; external 'gdi32' name 'Rectangle';
function RestoreDC(_para1:HDC; _para2:longint):WINBOOL; external 'gdi32' name 'RestoreDC';
function RealizePalette(_para1:HDC):UINT; external 'gdi32' name 'RealizePalette';
function RoundRect(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:longint; _para7:longint):WINBOOL; external 'gdi32' name 'RoundRect';
function ResizePalette(_para1:HPALETTE; _para2:UINT):WINBOOL; external 'gdi32' name 'ResizePalette';
function SaveDC(_para1:HDC):longint; external 'gdi32' name 'SaveDC';
function SelectClipRgn(_para1:HDC; _para2:HRGN):longint; external 'gdi32' name 'SelectClipRgn';
function ExtSelectClipRgn(_para1:HDC; _para2:HRGN; _para3:longint):longint; external 'gdi32' name 'ExtSelectClipRgn';
function SetMetaRgn(_para1:HDC):longint; external 'gdi32' name 'SetMetaRgn';
function SelectObject(_para1:HDC; _para2:HGDIOBJ):HGDIOBJ; external 'gdi32' name 'SelectObject';
function SelectPalette(_para1:HDC; _para2:HPALETTE; _para3:WINBOOL):HPALETTE; external 'gdi32' name 'SelectPalette';
function SetBkColor(_para1:HDC; _para2:COLORREF):COLORREF; external 'gdi32' name 'SetBkColor';
function SetBkMode(_para1:HDC; _para2:longint):longint; external 'gdi32' name 'SetBkMode';
function SetBitmapBits(_para1:HBITMAP; _para2:DWORD; _para3:pointer):LONG; external 'gdi32' name 'SetBitmapBits';
function SetBoundsRect(_para1:HDC; const _para2:RECT; _para3:UINT):UINT; external 'gdi32' name 'SetBoundsRect';
function SetDIBits(_para1:HDC; _para2:HBITMAP; _para3:UINT; _para4:UINT; _para5:pointer;_para6:PBITMAPINFO; _para7:UINT):longint; external 'gdi32' name 'SetDIBits';
function SetDIBitsToDevice(_para1:HDC; _para2:longint; _para3:longint; _para4:DWORD; _para5:DWORD;_para6:longint; _para7:longint; _para8:UINT; _para9:UINT; _para10:pointer;var _para11:BITMAPINFO; _para12:UINT):longint;
  external 'gdi32' name 'SetDIBitsToDevice';
function SetMapperFlags(_para1:HDC; _para2:DWORD):DWORD; external 'gdi32' name 'SetMapperFlags';
function SetGraphicsMode(hdc:HDC; iMode:longint):longint; external 'gdi32' name 'SetGraphicsMode';
function SetMapMode(_para1:HDC; _para2:longint):longint; external 'gdi32' name 'SetMapMode';
function SetMetaFileBitsEx(_para1:UINT; var _para2:BYTE):HMETAFILE; external 'gdi32' name 'SetMetaFileBitsEx';
function SetPaletteEntries(_para1:HPALETTE; _para2:UINT; _para3:UINT; var _para4:PALETTEENTRY):UINT; external 'gdi32' name 'SetPaletteEntries';
function SetPixel(_para1:HDC; _para2:longint; _para3:longint; _para4:COLORREF):COLORREF; external 'gdi32' name 'SetPixel';
function SetPixelV(_para1:HDC; _para2:longint; _para3:longint; _para4:COLORREF):WINBOOL; external 'gdi32' name 'SetPixelV';
function SetPolyFillMode(_para1:HDC; _para2:longint):longint; external 'gdi32' name 'SetPolyFillMode';
function StretchBlt(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:HDC; _para7:longint; _para8:longint; _para9:longint; _para10:longint;_para11:DWORD):WINBOOL; external 'gdi32' name 'StretchBlt';
function SetRectRgn(_para1:HRGN; _para2:longint; _para3:longint; _para4:longint; _para5:longint):WINBOOL; external 'gdi32' name 'SetRectRgn';
function StretchDIBits(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:longint; _para7:longint; _para8:longint; _para9:longint; _para10:pointer;var _para11:BITMAPINFO; _para12:UINT; _para13:DWORD):longint;
  external 'gdi32' name 'StretchDIBits';
function SetROP2(_para1:HDC; _para2:longint):longint; external 'gdi32' name 'SetROP2';
function SetStretchBltMode(_para1:HDC; _para2:longint):longint; external 'gdi32' name 'SetStretchBltMode';
function SetSystemPaletteUse(_para1:HDC; _para2:UINT):UINT; external 'gdi32' name 'SetSystemPaletteUse';
function SetTextCharacterExtra(_para1:HDC; _para2:longint):longint; external 'gdi32' name 'SetTextCharacterExtra';
function SetTextColor(_para1:HDC; _para2:COLORREF):COLORREF; external 'gdi32' name 'SetTextColor';
function SetTextAlign(_para1:HDC; _para2:UINT):UINT; external 'gdi32' name 'SetTextAlign';
function SetTextJustification(_para1:HDC; _para2:longint; _para3:longint):WINBOOL; external 'gdi32' name 'SetTextJustification';
function UpdateColors(_para1:HDC):WINBOOL; external 'gdi32' name 'UpdateColors';
function PlayMetaFileRecord(_para1:HDC; _para2:LPHANDLETABLE; _para3:LPMETARECORD; _para4:UINT):WINBOOL; external 'gdi32' name 'PlayMetaFileRecord';
function EnumMetaFile(_para1:HDC; _para2:HMETAFILE; _para3:ENUMMETAFILEPROC; _para4:LPARAM):WINBOOL; external 'gdi32' name 'EnumMetaFile';
function CloseEnhMetaFile(_para1:HDC):HENHMETAFILE; external 'gdi32' name 'CloseEnhMetaFile';
function DeleteEnhMetaFile(_para1:HENHMETAFILE):WINBOOL; external 'gdi32' name 'DeleteEnhMetaFile';
function EnumEnhMetaFile(_para1:HDC; _para2:HENHMETAFILE; _para3:ENHMETAFILEPROC; _para4:LPVOID; const _para5:RECT):WINBOOL; external 'gdi32' name 'EnumEnhMetaFile';
function GetEnhMetaFileHeader(_para1:HENHMETAFILE; _para2:UINT; _para3:LPENHMETAHEADER):UINT; external 'gdi32' name 'GetEnhMetaFileHeader';
function GetEnhMetaFilePaletteEntries(_para1:HENHMETAFILE; _para2:UINT; _para3:LPPALETTEENTRY):UINT; external 'gdi32' name 'GetEnhMetaFilePaletteEntries';
function GetEnhMetaFileBits(_para1:HENHMETAFILE; _para2:UINT; _para3:LPBYTE): UINT; external 'gdi32' name 'GetEnhMetaFileBits';
function GetWinMetaFileBits(_para1:HENHMETAFILE; _para2:UINT; _para3:LPBYTE; _para4:wINT; _para5:HDC):UINT; external 'gdi32' name 'GetWinMetaFileBits';
function PlayEnhMetaFile(_para1:HDC; _para2:HENHMETAFILE; const _para3:RECT):WINBOOL; external 'gdi32' name 'PlayEnhMetaFile';
function PlayEnhMetaFileRecord(_para1:HDC; _para2:LPHANDLETABLE; var _para3:ENHMETARECORD; _para4:UINT):WINBOOL; external 'gdi32' name 'PlayEnhMetaFileRecord';
function SetEnhMetaFileBits(_para1:UINT; var _para2:BYTE):HENHMETAFILE; external 'gdi32' name 'SetEnhMetaFileBits';
function SetWinMetaFileBits(_para1:UINT; var _para2:BYTE; _para3:HDC; var _para4:METAFILEPICT):HENHMETAFILE; external 'gdi32' name 'SetWinMetaFileBits';
function GdiComment(_para1:HDC; _para2:UINT; var _para3:BYTE):WINBOOL; external 'gdi32' name 'GdiComment';
function AngleArc(_para1:HDC; _para2:longint; _para3:longint; _para4:DWORD; _para5:Single;_para6:Single):WINBOOL; external 'gdi32' name 'AngleArc';
function PolyPolyline(_para1:HDC; var _para2:POINT; var _para3:DWORD; _para4:DWORD):WINBOOL; external 'gdi32' name 'PolyPolyline';
function GetWorldTransform(_para1:HDC; _para2:LPXFORM):WINBOOL; external 'gdi32' name 'GetWorldTransform';
function SetWorldTransform(_para1:HDC; var _para2:XFORM):WINBOOL; external 'gdi32' name 'SetWorldTransform';
function ModifyWorldTransform(_para1:HDC; var _para2:XFORM; _para3:DWORD):WINBOOL; external 'gdi32' name 'ModifyWorldTransform';
function CombineTransform(_para1:LPXFORM; var _para2:XFORM; var _para3:XFORM):WINBOOL; external 'gdi32' name 'CombineTransform';
function CreateDIBSection(_para1:HDC; var _para2:BITMAPINFO; _para3:UINT; var _para4:pointer; _para5:HANDLE;_para6:DWORD):HBITMAP; external 'gdi32' name 'CreateDIBSection';
function GetDIBColorTable(_para1:HDC; _para2:UINT; _para3:UINT; var _para4:RGBQUAD):UINT; external 'gdi32' name 'GetDIBColorTable';
function SetDIBColorTable(_para1:HDC; _para2:UINT; _para3:UINT; var _para4:RGBQUAD):UINT; external 'gdi32' name 'SetDIBColorTable';
function SetColorAdjustment(_para1:HDC; var _para2:COLORADJUSTMENT):WINBOOL; external 'gdi32' name 'SetColorAdjustment';
function GetColorAdjustment(_para1:HDC; _para2:LPCOLORADJUSTMENT):WINBOOL; external 'gdi32' name 'GetColorAdjustment';
function CreateHalftonePalette(_para1:HDC):HPALETTE; external 'gdi32' name 'CreateHalftonePalette';
function EndDoc(_para1:HDC):longint; external 'gdi32' name 'EndDoc';
function StartPage(_para1:HDC):longint; external 'gdi32' name 'StartPage';
function EndPage(_para1:HDC):longint; external 'gdi32' name 'EndPage';
function AbortDoc(_para1:HDC):longint; external 'gdi32' name 'AbortDoc';
function SetAbortProc(_para1:HDC; _para2:TABORTPROC):longint; external 'gdi32' name 'SetAbortProc';
{function AbortPath(_para1:HDC):WINBOOL; external 'gdi32' name 'AbortPath';}
function ArcTo(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:longint; _para7:longint; _para8:longint; _para9:longint):WINBOOL; external 'gdi32' name 'ArcTo';
function BeginPath(_para1:HDC):WINBOOL; external 'gdi32' name 'BeginPath';
function CloseFigure(_para1:HDC):WINBOOL; external 'gdi32' name 'CloseFigure';
function EndPath(_para1:HDC):WINBOOL; external 'gdi32' name 'EndPath';
function FillPath(_para1:HDC):WINBOOL; external 'gdi32' name 'FillPath';
function FlattenPath(_para1:HDC):WINBOOL; external 'gdi32' name 'FlattenPath';
function GetPath(_para1:HDC; _para2:LPPOINT; _para3:LPBYTE; _para4:longint):longint; external 'gdi32' name 'GetPath';
function PathToRegion(_para1:HDC):HRGN; external 'gdi32' name 'PathToRegion';
function PolyDraw(_para1:HDC;  _para2:LPPOINT;_para3:LPBYTE; _para4:longint):WINBOOL; external 'gdi32' name 'PolyDraw';
function SelectClipPath(_para1:HDC; _para2:longint):WINBOOL; external 'gdi32' name 'SelectClipPath';
function SetArcDirection(_para1:HDC; _para2:longint):longint; external 'gdi32' name 'SetArcDirection';
function SetMiterLimit(_para1:HDC; _para2:Single; _para3:PSingle):WINBOOL; external 'gdi32' name 'SetMiterLimit';
function StrokeAndFillPath(_para1:HDC):WINBOOL; external 'gdi32' name 'StrokeAndFillPath';
function StrokePath(_para1:HDC):WINBOOL; external 'gdi32' name 'StrokePath';
function WidenPath(_para1:HDC):WINBOOL; external 'gdi32' name 'WidenPath';
function ExtCreatePen(_para1:DWORD; _para2:DWORD; var _para3:LOGBRUSH; _para4:DWORD; var _para5:DWORD):HPEN; external 'gdi32' name 'ExtCreatePen';
function GetMiterLimit(_para1:HDC; _para2:PSingle):WINBOOL; external 'gdi32' name 'GetMiterLimit';
function GetArcDirection(_para1:HDC):longint; external 'gdi32' name 'GetArcDirection';
function MoveToEx(_para1:HDC; _para2:longint; _para3:longint; _para4:LPPOINT):WINBOOL; external 'gdi32' name 'MoveToEx';
function CreatePolygonRgn(var _para1:POINT; _para2:longint; _para3:longint):HRGN; external 'gdi32' name 'CreatePolygonRgn';
function DPtoLP(_para1:HDC; _para2:LPPOINT; _para3:longint):WINBOOL; external 'gdi32' name 'DPtoLP';
function LPtoDP(_para1:HDC; _para2:LPPOINT; _para3:longint):WINBOOL; external 'gdi32' name 'LPtoDP';
function Polygon(_para1:HDC; _para2:LPPOINT; _para3:longint):WINBOOL; external 'gdi32' name 'Polygon';
function Polyline(_para1:HDC; _para2:LPPOINT; _para3:longint):WINBOOL; external 'gdi32' name 'Polyline';
function PolyBezier(_para1:HDC; _para2:LPPOINT; _para3:DWORD):WINBOOL; external 'gdi32' name 'PolyBezier';
function PolyBezierTo(_para1:HDC; _para2:LPPOINT; _para3:DWORD):WINBOOL; external 'gdi32' name 'PolyBezierTo';
function PolylineTo(_para1:HDC; _para2:LPPOINT; _para3:DWORD):WINBOOL; external 'gdi32' name 'PolylineTo';
function SetViewportExtEx(_para1:HDC; _para2:longint; _para3:longint; _para4:LPSIZE):WINBOOL; external 'gdi32' name 'SetViewportExtEx';
function SetViewportOrgEx(_para1:HDC; _para2:longint; _para3:longint; _para4:LPPOINT):WINBOOL; external 'gdi32' name 'SetViewportOrgEx';
function SetWindowExtEx(_para1:HDC; _para2:longint; _para3:longint; _para4:LPSIZE):WINBOOL; external 'gdi32' name 'SetWindowExtEx';
function SetWindowOrgEx(_para1:HDC; _para2:longint; _para3:longint; _para4:LPPOINT):WINBOOL; external 'gdi32' name 'SetWindowOrgEx';
function OffsetViewportOrgEx(_para1:HDC; _para2:longint; _para3:longint; _para4:LPPOINT):WINBOOL; external 'gdi32' name 'OffsetViewportOrgEx';
function OffsetWindowOrgEx(_para1:HDC; _para2:longint; _para3:longint; _para4:LPPOINT):WINBOOL; external 'gdi32' name 'OffsetWindowOrgEx';
function ScaleViewportExtEx(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:LPSIZE):WINBOOL; external 'gdi32' name 'ScaleViewportExtEx';
function ScaleWindowExtEx(_para1:HDC; _para2:longint; _para3:longint; _para4:longint; _para5:longint;_para6:LPSIZE):WINBOOL; external 'gdi32' name 'ScaleWindowExtEx';
function SetBitmapDimensionEx(_para1:HBITMAP; _para2:longint; _para3:longint; _para4:LPSIZE):WINBOOL; external 'gdi32' name 'SetBitmapDimensionEx';
function SetBrushOrgEx(_para1:HDC; _para2:longint; _para3:longint; _para4:LPPOINT):WINBOOL; external 'gdi32' name 'SetBrushOrgEx';
function GetDCOrgEx(_para1:HDC; _para2:LPPOINT):WINBOOL; external 'gdi32' name 'GetDCOrgEx';
function FixBrushOrgEx(_para1:HDC; _para2:longint; _para3:longint; _para4:LPPOINT):WINBOOL; external 'gdi32' name 'FixBrushOrgEx';
function UnrealizeObject(_para1:HGDIOBJ):WINBOOL; external 'gdi32' name 'UnrealizeObject';
function GdiFlush:WINBOOL; external 'gdi32' name 'GdiFlush';
function GdiSetBatchLimit(_para1:DWORD):DWORD; external 'gdi32' name 'GdiSetBatchLimit';
function GdiGetBatchLimit:DWORD; external 'gdi32' name 'GdiGetBatchLimit';
function SetICMMode(_para1:HDC; _para2:longint):longint; external 'gdi32' name 'SetICMMode';
function CheckColorsInGamut(_para1:HDC; _para2:LPVOID; _para3:LPVOID; _para4:DWORD):WINBOOL; external 'gdi32' name 'CheckColorsInGamut';
function GetColorSpace(_para1:HDC):HANDLE; external 'gdi32' name 'GetColorSpace';
function SetColorSpace(_para1:HDC; _para2:HCOLORSPACE):WINBOOL; external 'gdi32' name 'SetColorSpace';
function DeleteColorSpace(_para1:HCOLORSPACE):WINBOOL; external 'gdi32' name 'DeleteColorSpace';
function GetDeviceGammaRamp(_para1:HDC; _para2:LPVOID):WINBOOL; external 'gdi32' name 'GetDeviceGammaRamp';
function SetDeviceGammaRamp(_para1:HDC; _para2:LPVOID):WINBOOL; external 'gdi32' name 'SetDeviceGammaRamp';
function ColorMatchToTarget(_para1:HDC; _para2:HDC; _para3:DWORD):WINBOOL; external 'gdi32' name 'ColorMatchToTarget';
{
function CreatePropertySheetPageA(lppsp:LPCPROPSHEETPAGE):HPROPSHEETPAGE; external 'comctl32' name 'CreatePropertySheetPageA';
function DestroyPropertySheetPage(hPSPage:HPROPSHEETPAGE):WINBOOL; external 'comctl32' name 'DestroyPropertySheetPage';
procedure InitCommonControls; external 'comctl32' name 'InitCommonControls';

function ImageList_AddIcon(himl:HIMAGELIST; hicon:HICON):longint;
function ImageList_Create(cx:longint; cy:longint; flags:UINT; cInitial:longint; cGrow:longint):HIMAGELIST; external 'comctl32' name 'ImageList_Create';
function ImageList_Destroy(himl:HIMAGELIST):WINBOOL; external 'comctl32' name 'ImageList_Destroy';
function ImageList_GetImageCount(himl:HIMAGELIST):longint; external 'comctl32' name 'ImageList_GetImageCount';
function ImageList_Add(himl:HIMAGELIST; hbmImage:HBITMAP; hbmMask:HBITMAP):longint; external 'comctl32' name 'ImageList_Add';
function ImageList_ReplaceIcon(himl:HIMAGELIST; i:longint; hicon:HICON):longint; external 'comctl32' name 'ImageList_ReplaceIcon';
function ImageList_SetBkColor(himl:HIMAGELIST; clrBk:COLORREF):COLORREF; external 'comctl32' name 'ImageList_SetBkColor';
function ImageList_GetBkColor(himl:HIMAGELIST):COLORREF; external 'comctl32' name 'ImageList_GetBkColor';
function ImageList_SetOverlayImage(himl:HIMAGELIST; iImage:longint; iOverlay:longint):WINBOOL; external 'comctl32' name 'ImageList_SetOverlayImage';
function ImageList_Draw(himl:HIMAGELIST; i:longint; hdcDst:HDC; x:longint; y:longint;fStyle:UINT):WINBOOL; external 'comctl32' name 'ImageList_Draw';
function ImageList_Replace(himl:HIMAGELIST; i:longint; hbmImage:HBITMAP; hbmMask:HBITMAP):WINBOOL; external 'comctl32' name 'ImageList_Replace';
function ImageList_AddMasked(himl:HIMAGELIST; hbmImage:HBITMAP; crMask:COLORREF):longint; external 'comctl32' name 'ImageList_AddMasked';
function ImageList_DrawEx(himl:HIMAGELIST; i:longint; hdcDst:HDC; x:longint; y:longint;dx:longint; dy:longint; rgbBk:COLORREF; rgbFg:COLORREF; fStyle:UINT):WINBOOL; external 'comctl32' name 'ImageList_DrawEx';
function ImageList_Remove(himl:HIMAGELIST; i:longint):WINBOOL; external 'comctl32' name 'ImageList_Remove';
function ImageList_GetIcon(himl:HIMAGELIST; i:longint; flags:UINT):HICON; external 'comctl32' name 'ImageList_GetIcon';
function ImageList_BeginDrag(himlTrack:HIMAGELIST; iTrack:longint; dxHotspot:longint; dyHotspot:longint):WINBOOL; external 'comctl32' name 'ImageList_BeginDrag';
procedure ImageList_EndDrag; external 'comctl32' name 'ImageList_EndDrag';
function ImageList_DragEnter(hwndLock:HWND; x:longint; y:longint):WINBOOL; external 'comctl32' name 'ImageList_DragEnter';
function ImageList_DragLeave(hwndLock:HWND):WINBOOL; external 'comctl32' name 'ImageList_DragLeave';
function ImageList_DragMove(x:longint; y:longint):WINBOOL; external 'comctl32' name 'ImageList_DragMove';
function ImageList_SetDragCursorImage(himlDrag:HIMAGELIST; iDrag:longint; dxHotspot:longint; dyHotspot:longint):WINBOOL; external 'comctl32' name 'ImageList_SetDragCursorImage';
function ImageList_DragShowNolock(fShow:WINBOOL):WINBOOL; external 'comctl32' name 'ImageList_DragShowNolock';
function ImageList_GetDragImage(ppt:LPPOINT; pptHotspot:LPPOINT):HIMAGELIST; external 'comctl32' name 'ImageList_GetDragImage';
function ImageList_GetIconSize(himl:HIMAGELIST; var cx:longint; var cy:longint):WINBOOL; external 'comctl32' name 'ImageList_GetIconSize';
function ImageList_SetIconSize(himl:HIMAGELIST; cx:longint; cy:longint):WINBOOL; external 'comctl32' name 'ImageList_SetIconSize';
function ImageList_GetImageInfo(himl:HIMAGELIST; i:longint; var pImageInfo:IMAGEINFO):WINBOOL; external 'comctl32' name 'ImageList_GetImageInfo';
function ImageList_Merge(himl1:HIMAGELIST; i1:longint; himl2:HIMAGELIST; i2:longint; dx:longint;dy:longint):HIMAGELIST; external 'comctl32' name 'ImageList_Merge';
function ImageList_SetImageCount(himl: HIMAGELIST; uNewCount: UINT): Integer; external 'comctl32.dll' name 'ImageList_SetImageCount';

function CreateToolbarEx(hwnd:HWND; ws:DWORD; wID:UINT; nBitmaps:longint; hBMInst:HINST;wBMID:UINT_PTR; lpButtons:LPCTBBUTTON; iNumButtons:longint; dxButton:longint; dyButton:longint;dxBitmap:longint;
  dyBitmap:longint; uStructSize:UINT):HWND; external 'comctl32' name 'CreateToolbarEx';
function CreateMappedBitmap(hInstance:HINST; idBitmap:longint; wFlags:UINT; lpColorMap:LPCOLORMAP; iNumMaps:longint):HBITMAP; external 'comctl32' name 'CreateMappedBitmap';
procedure MenuHelp(uMsg:UINT; wParam:WPARAM; lParam:LPARAM; hMainMenu:HMENU; hInst:HINST;hwndStatus:HWND; var lpwIDs:UINT); external 'comctl32' name 'MenuHelp';
function ShowHideMenuCtl(hWnd:HWND; uFlags:UINT; lpInfo:LPINT):WINBOOL; external 'comctl32' name 'ShowHideMenuCtl';
procedure GetEffectiveClientRect(hWnd:HWND; lprc:LPRECT; lpInfo:LPINT); external 'comctl32' name 'GetEffectiveClientRect';
function MakeDragList(hLB:HWND):WINBOOL; external 'comctl32' name 'MakeDragList';
procedure DrawInsert(handParent:HWND; hLB:HWND; nItem:longint); external 'comctl32' name 'DrawInsert';
function LBItemFromPt(hLB:HWND; pt:POINT; bAutoScroll:WINBOOL):longint; external 'comctl32' name 'LBItemFromPt';
function CreateUpDownControl(dwStyle:DWORD; x:longint; y:longint; cx:longint; cy:longint;hParent:HWND; nID:longint; hInst:HINST; hBuddy:HWND; nUpper:longint;nLower:longint; nPos:longint):HWND; external 'comctl32' name 'CreateUpDownControl';
}

function RegCloseKey(hKey:HKEY):LONG; external 'advapi32' name 'RegCloseKey';
function RegSetKeySecurity(hKey:HKEY; SecurityInformation:SECURITY_INFORMATION; pSecurityDescriptor:PSECURITY_DESCRIPTOR):LONG; external 'advapi32' name 'RegSetKeySecurity';
function RegFlushKey(hKey:HKEY):LONG; external 'advapi32' name 'RegFlushKey';
function RegGetKeySecurity(hKey:HKEY; SecurityInformation:SECURITY_INFORMATION; pSecurityDescriptor:PSECURITY_DESCRIPTOR; lpcbSecurityDescriptor:LPDWORD):LONG; external 'advapi32' name 'RegGetKeySecurity';
function RegNotifyChangeKeyValue(hKey:HKEY; bWatchSubtree:WINBOOL; dwNotifyFilter:DWORD; hEvent:HANDLE; fAsynchronus:WINBOOL):LONG; external 'advapi32' name 'RegNotifyChangeKeyValue';
function IsValidCodePage(CodePage:UINT):WINBOOL; external 'kernel32' name 'IsValidCodePage';
function GetACP:UINT; external 'kernel32' name 'GetACP';
function GetOEMCP:UINT; external 'kernel32' name 'GetOEMCP';
function GetCPInfo(_para1:UINT; _para2:LPCPINFO):WINBOOL; external 'kernel32' name 'GetCPInfo';
function IsDBCSLeadByte(TestChar:BYTE):WINBOOL; external 'kernel32' name 'IsDBCSLeadByte';
function IsDBCSLeadByteEx(CodePage:UINT; TestChar:BYTE):WINBOOL; external 'kernel32' name 'IsDBCSLeadByteEx';
function MultiByteToWideChar(CodePage:UINT; dwFlags:DWORD; lpMultiByteStr:LPCSTR; cchMultiByte:longint; lpWideCharStr:LPWSTR;cchWideChar:longint):longint; external 'kernel32' name 'MultiByteToWideChar';
function WideCharToMultiByte(CodePage:UINT; dwFlags:DWORD; lpWideCharStr:LPCWSTR; cchWideChar:longint; lpMultiByteStr:LPSTR;cchMultiByte:longint; lpDefaultChar:LPCSTR; lpUsedDefaultChar:LPBOOL):longint; external 'kernel32' name 'WideCharToMultiByte';
function IsValidLocale(Locale:LCID; dwFlags:DWORD):WINBOOL; external 'kernel32' name 'IsValidLocale';
function ConvertDefaultLocale(Locale:LCID):LCID; external 'kernel32' name 'ConvertDefaultLocale';
function GetThreadLocale:LCID; external 'kernel32' name 'GetThreadLocale';
function SetThreadLocale(Locale:LCID):WINBOOL; external 'kernel32' name 'SetThreadLocale';
function GetSystemDefaultLangID:LANGID; external 'kernel32' name 'GetSystemDefaultLangID';
function GetUserDefaultLangID:LANGID; external 'kernel32' name 'GetUserDefaultLangID';
function GetSystemDefaultLCID:LCID; external 'kernel32' name 'GetSystemDefaultLCID';
function GetUserDefaultLCID:LCID; external 'kernel32' name 'GetUserDefaultLCID';
function ReadConsoleOutputAttribute(hConsoleOutput:HANDLE; lpAttribute:LPWORD; nLength:DWORD; dwReadCoord:COORD; lpNumberOfAttrsRead:LPDWORD):WINBOOL; external 'kernel32' name 'ReadConsoleOutputAttribute';
function WriteConsoleOutputAttribute(hConsoleOutput:HANDLE; var lpAttribute:WORD; nLength:DWORD; dwWriteCoord:COORD; lpNumberOfAttrsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'WriteConsoleOutputAttribute';
function FillConsoleOutputAttribute(hConsoleOutput:HANDLE; wAttribute:WORD; nLength:DWORD; dwWriteCoord:COORD; lpNumberOfAttrsWritten:LPDWORD):WINBOOL; external 'kernel32' name 'FillConsoleOutputAttribute';
function GetConsoleMode(hConsoleHandle:HANDLE; lpMode:LPDWORD):WINBOOL; external 'kernel32' name 'GetConsoleMode';
function GetNumberOfConsoleInputEvents(hConsoleInput:HANDLE; lpNumberOfEvents:PDWORD):WINBOOL; external 'kernel32' name 'GetNumberOfConsoleInputEvents';
function GetConsoleScreenBufferInfo(hConsoleOutput:HANDLE; lpConsoleScreenBufferInfo:PCONSOLE_SCREEN_BUFFER_INFO):WINBOOL; external 'kernel32' name 'GetConsoleScreenBufferInfo';
//function GetLargestConsoleWindowSize(hConsoleOutput:HANDLE):COORD; external 'kernel32' name 'GetLargestConsoleWindowSize';
function GetLargestConsoleWindowSize(hConsoleOutput:HANDLE):COORD;
function GetConsoleCursorInfo(hConsoleOutput:HANDLE; lpConsoleCursorInfo:PCONSOLE_CURSOR_INFO):WINBOOL; external 'kernel32' name 'GetConsoleCursorInfo';
function GetNumberOfConsoleMouseButtons(lpNumberOfMouseButtons:LPDWORD):WINBOOL; external 'kernel32' name 'GetNumberOfConsoleMouseButtons';
function SetConsoleMode(hConsoleHandle:HANDLE; dwMode:DWORD):WINBOOL; external 'kernel32' name 'SetConsoleMode';
function SetConsoleActiveScreenBuffer(hConsoleOutput:HANDLE):WINBOOL; external 'kernel32' name 'SetConsoleActiveScreenBuffer';
function FlushConsoleInputBuffer(hConsoleInput:HANDLE):WINBOOL; external 'kernel32' name 'FlushConsoleInputBuffer';
function SetConsoleScreenBufferSize(hConsoleOutput:HANDLE; dwSize:COORD):WINBOOL; external 'kernel32' name 'SetConsoleScreenBufferSize';
function SetConsoleCursorPosition(hConsoleOutput:HANDLE; dwCursorPosition:COORD):WINBOOL; external 'kernel32' name 'SetConsoleCursorPosition';
function SetConsoleCursorInfo(hConsoleOutput:HANDLE; lpConsoleCursorInfo:PCONSOLE_CURSOR_INFO):WINBOOL; external 'kernel32' name 'SetConsoleCursorInfo';
function SetConsoleWindowInfo(hConsoleOutput:HANDLE; bAbsolute:WINBOOL; var lpConsoleWindow:SMALL_RECT):WINBOOL; external 'kernel32' name 'SetConsoleWindowInfo';
function SetConsoleTextAttribute(hConsoleOutput:HANDLE; wAttributes:WORD):WINBOOL; external 'kernel32' name 'SetConsoleTextAttribute';
function SetConsoleCtrlHandler(HandlerRoutine:PHANDLER_ROUTINE; Add:WINBOOL):WINBOOL; external 'kernel32' name 'SetConsoleCtrlHandler';
function GenerateConsoleCtrlEvent(dwCtrlEvent:DWORD; dwProcessGroupId:DWORD):WINBOOL; external 'kernel32' name 'GenerateConsoleCtrlEvent';
function AllocConsole:WINBOOL; external 'kernel32' name 'AllocConsole';
function FreeConsole:WINBOOL; external 'kernel32' name 'FreeConsole';
function CreateConsoleScreenBuffer(dwDesiredAccess:DWORD; dwShareMode:DWORD; var lpSecurityAttributes:SECURITY_ATTRIBUTES; dwFlags:DWORD; lpScreenBufferData:LPVOID):HANDLE; external 'kernel32' name 'CreateConsoleScreenBuffer';
function GetConsoleCP:UINT; external 'kernel32' name 'GetConsoleCP';
function SetConsoleCP(wCodePageID:UINT):WINBOOL; external 'kernel32' name 'SetConsoleCP';
function GetConsoleOutputCP:UINT; external 'kernel32' name 'GetConsoleOutputCP';
function SetConsoleOutputCP(wCodePageID:UINT):WINBOOL; external 'kernel32' name 'SetConsoleOutputCP';
function WNetConnectionDialog(hwnd:HWND; dwType:DWORD):DWORD; external 'mpr' name 'WNetConnectionDialog';
function WNetDisconnectDialog(hwnd:HWND; dwType:DWORD):DWORD; external 'mpr' name 'WNetDisconnectDialog';
function WNetCloseEnum(hEnum:HANDLE):DWORD; external 'mpr' name 'WNetCloseEnum';
function CloseServiceHandle(hSCObject:SC_HANDLE):WINBOOL; external 'advapi32' name 'CloseServiceHandle';
function ControlService(hService:SC_HANDLE; dwControl:DWORD; lpServiceStatus:LPSERVICE_STATUS):WINBOOL; external 'advapi32' name 'ControlService';
function DeleteService(hService:SC_HANDLE):WINBOOL; external 'advapi32' name 'DeleteService';
function LockServiceDatabase(hSCManager:SC_HANDLE):SC_LOCK; external 'advapi32' name 'LockServiceDatabase';
function NotifyBootConfigStatus(BootAcceptable:WINBOOL):WINBOOL; external 'advapi32' name 'NotifyBootConfigStatus';
function QueryServiceObjectSecurity(hService:SC_HANDLE; dwSecurityInformation:SECURITY_INFORMATION; lpSecurityDescriptor:PSECURITY_DESCRIPTOR; cbBufSize:DWORD; pcbBytesNeeded:LPDWORD):WINBOOL;external 'advapi32' name 'QueryServiceObjectSecurity';
function QueryServiceStatus(hService:SC_HANDLE; lpServiceStatus:LPSERVICE_STATUS):WINBOOL; external 'advapi32' name 'QueryServiceStatus';
function SetServiceObjectSecurity(hService:SC_HANDLE; dwSecurityInformation:SECURITY_INFORMATION; lpSecurityDescriptor:PSECURITY_DESCRIPTOR):WINBOOL;external 'advapi32' name 'SetServiceObjectSecurity';
function SetServiceStatus(hServiceStatus:SERVICE_STATUS_HANDLE; lpServiceStatus:LPSERVICE_STATUS):WINBOOL; external 'advapi32' name 'SetServiceStatus';
function UnlockServiceDatabase(ScLock:SC_LOCK):WINBOOL; external 'advapi32' name 'UnlockServiceDatabase';
function ChoosePixelFormat(DC:HDC; ppfd:PPIXELFORMATDESCRIPTOR):longint; external 'gdi32' name 'ChoosePixelFormat';
function DescribePixelFormat(DC:HDC; iPixelFormat:longint; nBytes:UINT; ppfd:LPPIXELFORMATDESCRIPTOR):longint; external 'gdi32' name 'DescribePixelFormat';
{$ifdef Unknown_functions}{ WARNING: function is not in my gdi32.dll !! PM}function GetEnhMetaFilePixelFormat(_para1:HENHMETAFILE; _para2:DWORD; var _para3:PIXELFORMATDESCRIPTOR):UINT; external 'gdi32' name 'GetEnhMetaFilePixelFormat';
{$endif Unknown_functions}{    function GetPixelFormat(_para1:HDC):longint; external 'gdi32' name 'GetPixelFormat'; }
function SetPixelFormat(DC:HDC; iPixelFormat:longint; ppfd:PPIXELFORMATDESCRIPTOR):WINBOOL; external 'gdi32' name 'SetPixelFormat';
function SwapBuffers(DC:HDC):WINBOOL; external 'gdi32' name 'SwapBuffers';
function DragQueryPoint(_para1:HDROP; _para2:LPPOINT):WINBOOL; external 'shell32' name 'DragQueryPoint';
procedure DragFinish(_para1:HDROP); external 'shell32' name 'DragFinish';
procedure DragAcceptFiles(_para1:HWND; _para2:WINBOOL); external 'shell32' name 'DragAcceptFiles';
function DuplicateIcon(_para1:HINST; _para2:HICON):HICON; external 'shell32' name 'DuplicateIcon';
function DdeAbandonTransaction(_para1:DWORD; _para2:HCONV; _para3:DWORD):BOOL;external 'user32' name 'DdeAbandonTransaction';
function DdeAccessData(_para1:HDDEDATA; _para2:PDWORD):PBYTE;external 'user32' name 'DdeAccessData';
function DdeAddData(_para1:HDDEDATA; _para2:PBYTE; _para3:DWORD; _para4:DWORD):HDDEDATA;external 'user32' name 'DdeAddData';
{ This is only a prototype PM
function DdeCallback(_para1, _para2:UINT; _para3:HCONV; _para4, _para5:HSZ;_para6: HDDEDATA; _para7, _para8:PDWORD):HDDEDATA;external 'user32' name 'DdeCallback';}
function DdeClientTransaction(_para1:PBYTE; _para2:DWORD; _para3:HCONV; _para4:HSZ; _para5:UINT;
           _para6:UINT; _para7:DWORD; _para8:PDWORD):HDDEDATA;external 'user32' name 'DdeClientTransaction';
function DdeCmpStringHandles(_para1:HSZ; _para2:HSZ):longint; external 'user32' name 'DdeCmpStringHandles';
function DdeConnect(_para1:DWORD; _para2:HSZ; _para3:HSZ; _para4:PCONVCONTEXT):HCONV; external 'user32' name 'DdeConnect';
function DdeConnectList(_para1:DWORD; _para2:HSZ; _para3:HSZ; _para4:HCONVLIST; _para5:PCONVCONTEXT):HCONVLIST;external 'user32' name 'DdeConnectList';
function DdeCreateDataHandle(_para1:DWORD; _para2:LPBYTE; _para3:DWORD; _para4:DWORD; _para5:HSZ;_para6:UINT; _para7:UINT):HDDEDATA; external 'user32' name 'DdeCreateDataHandle';
function DdeDisconnect(_para1:HCONV):WINBOOL; external 'user32' name 'DdeDisconnect';
function DdeDisconnectList(_para1:HCONVLIST):BOOL;external 'user32' name 'DdeDisconnectList';
function DdeEnableCallback(_para1:DWORD; _para2:HCONV; _para3:UINT):BOOL;external 'user32' name 'DdeEnableCallback';
function DdeFreeDataHandle(_para1:HDDEDATA):WINBOOL; external 'user32' name 'DdeFreeDataHandle';
function DdeFreeStringHandle(_para1:DWORD;_para2:HSZ):WINBOOL; external 'user32' name 'DdeFreeStringHandle';
function DdeGetData(_para1:HDDEDATA;_para2:LPBYTE; _para3:DWORD; _para4:DWORD):DWORD; external 'user32' name 'DdeGetData';
function DdeGetLastError(_para1:DWORD):UINT; external 'user32' name 'DdeGetLastError';
function DdeImpersonateClient(_para1:HCONV):BOOL;external 'user32' name 'DdeImpersonateClient';
function DdeKeepStringHandle(_para1:DWORD; _para2:HSZ):BOOL;external 'user32' name 'DdeKeepStringHandle';
function DdeNameService(_para1:DWORD; _para2:HSZ; _para3:HSZ; _para4:UINT):HDDEDATA; external 'user32' name 'DdeNameService';
function DdePostAdvise(_para1:DWORD; _para2:HSZ; _para3:HSZ):WINBOOL; external 'user32' name 'DdePostAdvise';
function DdeQueryConvInfo(_para1:HCONV; _para2:DWORD; _para3:PCONVINFO):UINT;external 'user32' name 'DdeQueryConvInfo';
function DdeQueryNextServer(_para1:HCONVLIST; _para2:HCONV):HCONV;external 'user32' name 'DdeQueryNextServer';
function DdeReconnect(_para1:HCONV):HCONV; external 'user32' name 'DdeReconnect';
function DdeSetUserHandle(_para1:HCONV; _para2:DWORD; _para3:DWORD):BOOL;external 'user32' name 'DdeSetUserHandle';
function DdeUnaccessData(_para1:HDDEDATA):BOOL;external 'user32' name 'DdeUnaccessData';
function DdeUninitialize(_para1:DWORD):WINBOOL; external 'user32' name 'DdeUninitialize';
{$ifdef Unknown_functions}
function NetUserEnum(_para1:LPWSTR; _para2:DWORD; _para3:DWORD; var _para4:LPBYTE; _para5:DWORD;_para6:LPDWORD; _para7:LPDWORD; _para8:LPDWORD):DWORD; external 'netapi32' name 'NetUserEnum';
function NetApiBufferFree(_para1:LPVOID):DWORD; external 'netapi32' name 'NetApiBufferFree';
function NetUserGetInfo(_para1:LPWSTR; _para2:LPWSTR; _para3:DWORD; _para4:LPBYTE):DWORD; external 'netapi32' name 'NetUserGetInfo';
function NetGetDCName(_para1:LPWSTR; _para2:LPWSTR; var _para3:LPBYTE):DWORD; external 'netapi32' name 'NetGetDCName';
function NetGroupEnum(_para1:LPWSTR; _para2:DWORD; var _para3:LPBYTE; _para4:DWORD; _para5:LPDWORD;_para6:LPDWORD; _para7:LPDWORD):DWORD; external 'netapi32' name 'NetGroupEnum';
function NetLocalGroupEnum(_para1:LPWSTR; _para2:DWORD; var _para3:LPBYTE; _para4:DWORD; _para5:LPDWORD;_para6:LPDWORD; _para7:LPDWORD):DWORD; external 'netapi32' name 'NetLocalGroupEnum';
{$endif Unknown_functions}
procedure SHAddToRecentDocs(_para1:UINT; _para2:LPCVOID); external 'shell32' name 'SHAddToRecentDocs';
function SHBrowseForFolder(_para1:LPBROWSEINFO):LPITEMIDLIST; external 'shell32' name 'SHBrowseForFolder';
procedure SHChangeNotify(_para1:LONG; _para2:UINT; _para3:LPCVOID; _para4:LPCVOID); external 'shell32' name 'SHChangeNotify';
function SHFileOperation(_para1:LPSHFILEOPSTRUCT):longint; external 'shell32' name 'SHFileOperation';
procedure SHFreeNameMappings(_para1:HANDLE); external 'shell32' name 'SHFreeNameMappings';
function SHGetFileInfo(_para1:LPCTSTR; _para2:DWORD; var _para3:SHFILEINFO; _para4:UINT; _para5:UINT):DWORD_PTR; external 'shell32' name 'SHGetFileInfo';
function SHGetPathFromIDList(_para1:LPCITEMIDLIST; _para2:LPTSTR):WINBOOL; external 'shell32' name 'SHGetPathFromIDList';
function SHGetSpecialFolderLocation(_para1:HWND; _para2:longint; var _para3:LPITEMIDLIST):HRESULT; external 'shell32' name 'SHGetSpecialFolderLocation';

{ was missing, bug report 1808 PM }
{
function CommDlgExtendedError : DWORD; external 'comdlg32' name 'CommDlgExtendedError';
}
{ wgl Windows OpenGL helper functions }
function wglUseFontBitmaps(DC:HDC; first:DWORD; count:DWORD; listbase:DWORD):WINBOOL; external 'opengl32' name 'wglUseFontBitmapsA';
function wglCreateContext(hdc:HDC):HGLRC; external 'opengl32' name 'wglCreateContext';
function wglCreateLayerContext(hdc:HDC; iLayerPlane:longint):HGLRC; external 'opengl32' name 'wglCreateLayerContext';
function wglCopyContext(glrcSrc:HGLRC; glrcDst:HGLRC; mask:UINT):WINBOOL; external 'opengl32' name 'wglCopyContext';
function wglDeleteContext(glrc:HGLRC):WINBOOL; external 'opengl32' name 'wglDeleteContext';
function wglGetCurrentContext:HGLRC; external 'opengl32' name 'wglGetCurrentContext';
function wglGetCurrentDC:HDC; external 'opengl32' name 'wglGetCurrentDC';
function wglMakeCurrent(DC:HDC; glrc:HGLRC):WINBOOL; external 'opengl32' name 'wglMakeCurrent';
function wglShareLists(glrc1:HGLRC; glrc2:HGLRC):WINBOOL; external 'opengl32' name 'wglShareLists';
function wglUseFontBitmapsW(DC:HDC; first:DWORD; count:DWORD; listbase:DWORD):WINBOOL; external 'opengl32' name 'wglUseFontBitmapsW';
{ Delphi doesn't declare these, but we do: }
function wglUseFontOutlines(DC:HDC; first:DWORD; count:DWORD; listbase:DWORD; deviation:Single;
  extrusion:Single; format:longint; lpgmf:LPGLYPHMETRICSFLOAT):WINBOOL; external 'opengl32' name 'wglUseFontOutlinesA';
function wglUseFontBitmapsA(DC:HDC; first:DWORD; count:DWORD; listbase:DWORD):WINBOOL; external 'opengl32' name 'wglUseFontBitmapsA';
function wglUseFontOutlinesA(DC:HDC; first:DWORD; count:DWORD; listbase:DWORD; deviation:Single;
  extrusion:Single; format:longint; lpgmf:LPGLYPHMETRICSFLOAT):WINBOOL; external 'opengl32' name 'wglUseFontOutlinesA';
function wglDescribeLayerPlane(DC:HDC; iPixelFormat:longint; iLayerPane:longint; nBytes:UINT; plpd:LPLAYERPLANEDESCRIPTOR):WINBOOL; external 'opengl32' name 'wglDescribeLayerPlane';
function wglGetLayerPaletteEntries(DC:HDC; iLayerPlane:longint; iStart:longint; cEntries:longint; var cr:COLORREF):longint; external 'opengl32' name 'wglGetLayerPaletteEntries';
function wglGetProcAddress(ProcName:LPCSTR):PROC; external 'opengl32' name 'wglGetProcAddress';
function wglRealizeLayerPalette(DC:HDC; iLayerPlane: longint; bRealize:WINBOOL):WINBOOL; external 'opengl32' name 'wglRealizeLayerPalette';
function wglSetLayerPaletteEntries(DC:HDC; iLayerPlane:longint; iStart:longint; cEntries:longint; var cr:COLORREF):longint; external 'opengl32' name 'wglSetLayerPaletteEntries';
function wglSwapLayerBuffers(DC:HDC; fuPlanes:UINT):WINBOOL; external 'opengl32' name 'wglSwapLayerBuffers';
function wglUseFontOutlinesW(DC:HDC; first:DWORD; count:DWORD;  listbase:DWORD; deviation:Single;
  extrusion:Single; format:longint; lpgmf:LPGLYPHMETRICSFLOAT):WINBOOL; external 'opengl32' name 'wglUseFontOutlinesW';


{ translated macros }
function Animate_Create(hWndP:HWND; id:HMENU;dwStyle:DWORD;hInstance:HINST):HWND;
function Animate_Open(hwnd : HWND;szName : LPTSTR) : LRESULT;
function Animate_Play(hwnd : HWND;from,_to : longint;rep : UINT) : LRESULT;
function Animate_Stop(hwnd : HWND) : LRESULT;
function Animate_Close(hwnd : HWND) : LRESULT;
function Animate_Seek(hwnd : HWND;frame : longint) : LRESULT;
function PropSheet_AddPage(hPropSheetDlg : HWND;hpage : HPROPSHEETPAGE) : LRESULT;
function PropSheet_Apply(hPropSheetDlg : HWND) : LRESULT;
function PropSheet_CancelToClose(hPropSheetDlg : HWND) : LRESULT;
function PropSheet_Changed(hPropSheetDlg,hwndPage : HWND) : LRESULT;
function PropSheet_GetCurrentPageHwnd(hDlg : HWND) : LRESULT;
function PropSheet_GetTabControl(hPropSheetDlg : HWND) : LRESULT;
function PropSheet_IsDialogMessage(hDlg : HWND;pMsg : longint) : LRESULT;
function PropSheet_PressButton(hPropSheetDlg : HWND;iButton : longint) : LRESULT;
function PropSheet_QuerySiblings(hPropSheetDlg : HWND;param1,param2 : longint) : LRESULT;
function PropSheet_RebootSystem(hPropSheetDlg : HWND) : LRESULT;
function PropSheet_RemovePage(hPropSheetDlg : HWND;hpage : HPROPSHEETPAGE; index : longint) : LRESULT;
function PropSheet_RestartWindows(hPropSheetDlg : HWND) : LRESULT;
function PropSheet_SetCurSel(hPropSheetDlg : HWND;hpage : HPROPSHEETPAGE; index : longint) : LRESULT;
function PropSheet_SetCurSelByID(hPropSheetDlg : HWND; id : longint) : LRESULT;
function PropSheet_SetFinishText(hPropSheetDlg:HWND;lpszText : LPTSTR) : LRESULT;
function PropSheet_SetTitle(hPropSheetDlg:HWND;dwStyle:DWORD;lpszText : LPCTSTR) : LRESULT;
function PropSheet_SetWizButtons(hPropSheetDlg:HWND;dwFlags : DWORD) : LRESULT;
function PropSheet_UnChanged(hPropSheetDlg:HWND;hwndPage : HWND) : LRESULT;
function Header_DeleteItem(hwndHD:HWND;index : longint) : WINBOOL;
function Header_GetItem(hwndHD:HWND;index:longint;var hdi : HD_ITEM) : WINBOOL;
function Header_GetItemCount(hwndHD : HWND) : longint;
function Header_InsertItem(hwndHD:HWND;index : longint;var hdi : HD_ITEM) : longint;
function Header_Layout(hwndHD:HWND;var layout : HD_LAYOUT) : WINBOOL;
function Header_SetItem(hwndHD:HWND;index : longint;var hdi : HD_ITEM) : WINBOOL;
function ListView_Arrange(hwndLV:HWND;code : UINT) : LRESULT;
function ListView_CreateDragImage(hwnd:HWND;i : longint;lpptUpLeft : LPPOINT) : LRESULT;
function ListView_DeleteAllItems(hwnd : HWND) : LRESULT;
function ListView_DeleteColumn(hwnd:HWND;iCol : longint) : LRESULT;
function ListView_DeleteItem(hwnd:HWND;iItem : longint) : LRESULT;
function ListView_EditLabel(hwndLV:HWND;i : longint) : LRESULT;
function ListView_EnsureVisible(hwndLV:HWND;i,fPartialOK : longint) : LRESULT;
function ListView_FindItem(hwnd:HWND;iStart : longint;var lvfi : LV_FINDINFO) : longint;
function ListView_GetBkColor(hwnd : HWND) : LRESULT;
function ListView_GetCallbackMask(hwnd : HWND) : LRESULT;
function ListView_GetColumn(hwnd:HWND;iCol : longint;var col : LV_COLUMN) : LRESULT;
function ListView_GetColumnWidth(hwnd:HWND;iCol : longint) : LRESULT;
function ListView_GetCountPerPage(hwndLV : HWND) : LRESULT;
function ListView_GetEditControl(hwndLV : HWND) : LRESULT;
function ListView_GetImageList(hwnd:HWND;iImageList : wINT) : LRESULT;
function ListView_GetISearchString(hwndLV:HWND;lpsz : LPTSTR) : LRESULT;
function ListView_GetItem(hwnd:HWND;var item : LV_ITEM) : LRESULT;
function ListView_GetItemCount(hwnd : HWND) : LRESULT;
function ListView_GetItemPosition(hwndLV:HWND;i : longint;var pt : POINT) : longint;
function ListView_GetItemSpacing(hwndLV:HWND;fSmall : longint) : LRESULT;
function ListView_GetItemState(hwndLV:HWND;i,mask : longint) : LRESULT;
function ListView_GetNextItem(hwnd:HWND; iStart, flags : longint) : LRESULT;
function ListView_GetOrigin(hwndLV:HWND;var pt : POINT) : LRESULT;
function ListView_GetSelectedCount(hwndLV : HWND) : LRESULT;
function ListView_GetStringWidth(hwndLV:HWND;psz : LPCTSTR) : LRESULT;
function ListView_GetTextBkColor(hwnd : HWND) : LRESULT;
function ListView_GetTextColor(hwnd : HWND) : LRESULT;
function ListView_GetTopIndex(hwndLV : HWND) : LRESULT;
function ListView_GetViewRect(hwnd:HWND;var rc : RECT) : LRESULT;
function ListView_HitTest(hwndLV:HWND;var info : LV_HITTESTINFO) : LRESULT;
function ListView_InsertColumn(hwnd:HWND;iCol : longint;var col : LV_COLUMN) : LRESULT;
function ListView_InsertItem(hwnd:HWND;var item : LV_ITEM) : LRESULT;
function ListView_RedrawItems(hwndLV:HWND;iFirst,iLast : longint) : LRESULT;
function ListView_Scroll(hwndLV:HWND;dx,dy : longint) : LRESULT;
function ListView_SetBkColor(hwnd:HWND;clrBk : COLORREF) : LRESULT;
function ListView_SetCallbackMask(hwnd:HWND;mask : UINT) : LRESULT;
function ListView_SetColumn(hwnd:HWND;iCol : longint; var col : LV_COLUMN) : LRESULT;
function ListView_SetColumnWidth(hwnd:HWND;iCol,cx : longint) : LRESULT;
function ListView_SetImageList(hwnd:HWND;himl : longint;iImageList : HIMAGELIST) : LRESULT;
function ListView_SetItem(hwnd:HWND;var item : LV_ITEM) : LRESULT;
function ListView_SetItemCount(hwndLV:HWND;cItems : longint) : LRESULT;
function ListView_SetItemPosition(hwndLV:HWND;i,x,y : longint) : LRESULT;
function ListView_SetItemPosition32(hwndLV:HWND;i,x,y : longint) : LRESULT;
function ListView_SetItemState(hwndLV:HWND; i, data, mask:longint) : LRESULT;
function ListView_SetItemText(hwndLV:HWND; i, iSubItem_:longint;pszText_ : LPTSTR) : LRESULT;
function ListView_SetTextBkColor(hwnd:HWND;clrTextBk : COLORREF) : LRESULT;
function ListView_SetTextColor(hwnd:HWND;clrText : COLORREF) : LRESULT;
function ListView_SortItems(hwndLV:HWND;_pfnCompare:PFNLVCOMPARE;_lPrm : LPARAM) : LRESULT;
function ListView_Update(hwndLV:HWND;i : longint) : LRESULT;
function TreeView_InsertItem(hwnd:HWND;lpis : LPTV_INSERTSTRUCT) : LRESULT;
function TreeView_DeleteItem(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_DeleteAllItems(hwnd : HWND) : LRESULT;
function TreeView_Expand(hwnd:HWND;hitem:HTREEITEM;code : longint) : LRESULT;
function TreeView_GetCount(hwnd : HWND) : LRESULT;
function TreeView_GetIndent(hwnd : HWND) : LRESULT;
function TreeView_SetIndent(hwnd:HWND;indent : longint) : LRESULT;
function TreeView_GetImageList(hwnd:HWND;iImage : WPARAM) : LRESULT;
function TreeView_SetImageList(hwnd:HWND;himl:HIMAGELIST;iImage : WPARAM) : LRESULT;
function TreeView_GetNextItem(hwnd:HWND;hitem:HTREEITEM;code : longint) : LRESULT;
function TreeView_GetChild(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_GetNextSibling(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_GetPrevSibling(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_GetParent(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_GetFirstVisible(hwnd : HWND) : LRESULT;
function TreeView_GetNextVisible(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_GetPrevVisible(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_GetSelection(hwnd : HWND) : LRESULT;
function TreeView_GetDropHilight(hwnd : HWND) : LRESULT;
function TreeView_GetRoot(hwnd : HWND) : LRESULT;
function TreeView_Select(hwnd:HWND;hitem:HTREEITEM;code : longint) : LRESULT;
function TreeView_SelectItem(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_SelectDropTarget(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_SelectSetFirstVisible(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_GetItem(hwnd:HWND;var item : TV_ITEM) : LRESULT;
function TreeView_SetItem(hwnd:HWND;var item : TV_ITEM) : LRESULT;
function TreeView_EditLabel(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_GetEditControl(hwnd : HWND) : LRESULT;
function TreeView_GetVisibleCount(hwnd : HWND) : LRESULT;
function TreeView_HitTest(hwnd:HWND;lpht : LPTV_HITTESTINFO) : LRESULT;
function TreeView_CreateDragImage(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_SortChildren(hwnd:HWND;hitem:HTREEITEM;recurse : longint) : LRESULT;
function TreeView_EnsureVisible(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
function TreeView_SortChildrenCB(hwnd:HWND;psort:LPTV_SORTCB;recurse : longint) : LRESULT;
function TreeView_EndEditLabelNow(hwnd:HWND;fCancel : longint) : LRESULT;
function TreeView_GetISearchString(hwndTV:HWND;lpsz : LPTSTR) : LRESULT;
function TabCtrl_GetImageList(hwnd : HWND) : LRESULT;
function TabCtrl_SetImageList(hwnd:HWND;himl : HIMAGELIST) : LRESULT;
function TabCtrl_GetItemCount(hwnd : HWND) : LRESULT;
function TabCtrl_GetItem(hwnd:HWND;iItem : longint;var item : TC_ITEM) : LRESULT;
function TabCtrl_SetItem(hwnd:HWND;iItem : longint;var item : TC_ITEM) : LRESULT;
function TabCtrl_InsertItem(hwnd:HWND;iItem : longint;var item : TC_ITEM) : LRESULT;
function TabCtrl_DeleteItem(hwnd:HWND;i : longint) : LRESULT;
function TabCtrl_DeleteAllItems(hwnd : HWND) : LRESULT;
function TabCtrl_GetItemRect(hwnd:HWND;i : longint;var rc : RECT) : LRESULT;
function TabCtrl_GetCurSel(hwnd : HWND) : LRESULT;
function TabCtrl_SetCurSel(hwnd:HWND;i : longint) : LRESULT;
function TabCtrl_HitTest(hwndTC:HWND;var info : TC_HITTESTINFO) : LRESULT;
function TabCtrl_SetItemExtra(hwndTC:HWND;cb : longint) : LRESULT;
function TabCtrl_AdjustRect(hwnd:HWND;bLarger:WINBOOL;var rc : RECT) : LRESULT;
function TabCtrl_SetItemSize(hwnd:HWND;x,y : longint) : LRESULT;
function TabCtrl_RemoveImage(hwnd:HWND;i : WPARAM) : LRESULT;
function TabCtrl_SetPadding(hwnd:HWND;cx,cy : longint) : LRESULT;
function TabCtrl_GetRowCount(hwnd : HWND) : LRESULT;
function TabCtrl_GetToolTips(hwnd : HWND) : LRESULT;
function TabCtrl_SetToolTips(hwnd:HWND;hwndTT : longint) : LRESULT;
function TabCtrl_GetCurFocus(hwnd : HWND) : LRESULT;
function TabCtrl_SetCurFocus(hwnd:HWND;i : longint) : LRESULT;
function SNDMSG(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT;
function CommDlg_OpenSave_GetSpecA(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
function CommDlg_OpenSave_GetSpecW(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
{$ifndef Unicode}
function CommDlg_OpenSave_GetSpec(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
{$else Unicode}
function CommDlg_OpenSave_GetSpec(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
{$endif Unicode}
function CommDlg_OpenSave_GetFilePathA(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
function CommDlg_OpenSave_GetFilePathW(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
{$ifndef Unicode}
function CommDlg_OpenSave_GetFilePath(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
{$else Unicode}
function CommDlg_OpenSave_GetFilePath(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
{$endif Unicode}
function CommDlg_OpenSave_GetFolderPathA(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
function CommDlg_OpenSave_GetFolderPathW(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
{$ifndef Unicode}
function CommDlg_OpenSave_GetFolderPath(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
{$else Unicode}
function CommDlg_OpenSave_GetFolderPath(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
{$endif Unicode}
function CommDlg_OpenSave_GetFolderIDList(_hdlg:HWND;_pidl:LPVOID;_cbmax : longint) : LRESULT;
function CommDlg_OpenSave_SetControlText(_hdlg:HWND;_id : longint;_text : LPSTR) : LRESULT;
function CommDlg_OpenSave_HideControl(_hdlg:HWND;_id : longint) : LRESULT;
function CommDlg_OpenSave_SetDefExt(_hdlg:HWND;_pszext : LPSTR) : LRESULT;

function GetNextWindow(hWnd:HWND; uCmd:UINT):HWND; external 'user32' name 'GetWindow';

function GlobalAllocPtr(flags,cb:DWord):Pointer;
function GlobalFreePtr(lp:Pointer):Pointer;
function GlobalUnlockPtr(lp:pointer):Pointer;
function GlobalLockPtr(lp:pointer):Pointer;
function GlobalReAllocPtr(lp:Pointer;cbNew,flags:DWord):Pointer;
function GlobalPtrHandle(lp:pointer):Pointer;
function SetLayeredWindowAttributes(HWND:hwnd;crKey :COLORREF;bAlpha : byte;dwFlags : DWORD):WINBOOL; external 'user32' name 'SetLayeredWindowAttributes';
function SignalObjectAndWait( hObjectToSignal:THANDLE;  hObjectToWaitOn:THANDLE ;dwMilliseconds:  DWORD ;bAlertable:BOOL):DWORD; external 'kernel32' name 'SignalObjectAndWait';
function CancelWaitableTimer(hTimer:THandle):BOOL; external 'kernel32' name 'CancelWaitableTimer';
function GetComboBoxInfo(hwndCombo:HWND;pcbi:PCOMBOBOXINFO):BOOL; external 'user32' name 'GetComboBoxInfo';
function GetAncestor(HWnd: HWND; Flag: UINT): HWND; StdCall; External 'user32';
<<<<<<< HEAD
<<<<<<< HEAD

// #if(WINVER >= 0x0500)
function GetMenuInfo(_hmenu:HMENU; lpcmi:LPMENUINFO):BOOL; external 'user32' name 'GetMenuInfo';
function SetMenuInfo(_hmenu:HMENU; lpcmi:LPCMENUINFO):BOOL; external 'user32' name 'SetMenuInfo';
function EndMenu:BOOL; external 'user32' name 'EndMenu';
//#endif /* WINVER >= 0x0500 */


function GetMenuBarInfo(_hwnd:HWND; idObject:longint; idItem:longint; pmbi:LPMENUBARINFO):BOOL; external 'user32' name 'GetMenuBarInfo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function IMAGE_FIRST_SECTION(ntheader : PIMAGE_NT_HEADERS):PIMAGE_SECTION_HEADER;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif read_interface}


{$ifdef read_implementation}

{ Win32 API calling convention
  pushes POINT struct passed by value directly
  on stack instead of just pushing an address
  to overcome this we use a internal function
  that just pushes the two arguments.
  Bug report 1807. PM

  not necessary anymore FK

function Internal_MenuItemFromPoint(hWnd:HWND; hMenu:HMENU; ptScreenX, ptScreenY : LONG):longint; external 'user32' name 'MenuItemFromPoint';

function MenuItemFromPoint(hWnd:HWND; hMenu:HMENU; ptScreen:POINT):longint;
begin
  MenuItemFromPoint:=Internal_MenuItemFromPoint(hWnd, hMenu, ptScreen.X, ptScreen.Y);
end;

function Internal_DragDetect(hwnd:HWND; ptX, ptY : LONG):WINBOOL; external 'user32' name 'DragDetect';

function DragDetect(hwnd:HWND; pt:POINT):WINBOOL;
begin
  DragDetect:=Internal_DragDetect(hWnd, pt.X, pt.Y);
end;

function Internal_WindowFromPoint(PointX,PointY : LONG):HWND; external 'user32' name 'WindowFromPoint';

function WindowFromPoint(Point:POINT):HWND;
begin
  WindowFromPoint:=Internal_WindowFromPoint(Point.X, Point.Y);
end;

function Internal_ChildWindowFromPoint(hWndParent:HWND; PointX,PointY : LONG):HWND; external 'user32' name 'ChildWindowFromPoint';

function ChildWindowFromPoint(hWndParent:HWND; Point:POINT):HWND;
begin
  ChildWindowFromPoint:=Internal_ChildWindowFromPoint(hWndParent, Point.X, Point.Y);
end;

function Internal_PtInRect(var lprc:RECT; ptX,ptY : LONG):WINBOOL; external 'user32' name 'PtInRect';

function PtInRect(var lprc:RECT; pt:POINT):WINBOOL;
begin
  PtInRect:=Internal_PtInRect(lprc,pt.X,pt.Y);
end;

function PtInRect(lprc:LPRECT; pt:POINT):WINBOOL;
begin
  PtInRect:=Internal_PtInRect(lprc^,pt.X,pt.Y);
end;

function Internal_ChildWindowFromPointEx(_para1:HWND; _para2X,_Para2Y : LONG; _para3:UINT):HWND; external 'user32' name 'ChildWindowFromPointEx';

function ChildWindowFromPointEx(_para1:HWND; _para2:POINT; _para3:UINT):HWND;
begin
  ChildWindowFromPointEx:=Internal_ChildWindowFromPointEx(_para1,_para2.X,_para2.Y,_para3);
end;

function Internal_LBItemFromPt(hLB:HWND; ptX, ptY : LONG; bAutoScroll:WINBOOL):longint; external 'comctl32' name 'LBItemFromPt';

function LBItemFromPt(hLB:HWND; pt:POINT; bAutoScroll:WINBOOL):longint;
begin
  LBItemFromPt:=Internal_LBItemFromPt(hLB, pt.X, pt.Y, bAutoScroll);
end;

 End of bug fixes for bug report 1807. PM }

function GlobalDiscard(hglbMem:HGLOBAL):HGLOBAL;
begin
  GlobalDiscard:=GlobalReAlloc(hglbMem,0,GMEM_MOVEABLE);
end;


function LocalDiscard(hlocMem:HLOCAL):HLOCAL;
begin
  LocalDiscard := LocalReAlloc(hlocMem,0,LMEM_MOVEABLE);
end;


procedure MoveMemory(Destination:PVOID; Source:pointer; Length:DWORD);
begin
  Move(Source^,Destination^,Length);
end;


procedure CopyMemory(Destination:PVOID; Source:pointer; Length:DWORD);
begin
  Move(Source^, Destination^, Length);
end;


procedure FillMemory(Destination:PVOID; Length:DWORD; Fill:BYTE);
begin
  FillChar(Destination^,Length,Char(Fill));
end;


procedure ZeroMemory(Destination:PVOID; Length:DWORD);
begin
  FillChar(Destination^,Length,#0);
end;


function GlobalAllocPtr(flags,cb:DWord):Pointer;
begin
  GlobalAllocPtr:=GlobalLock(GlobalAlloc(flags,cb));
end;


function GlobalFreePtr(lp:Pointer):Pointer;
begin
  GlobalFreePtr:=Pointer(GlobalFree(HWND(GlobalUnlockPtr(lp))));
end;


function GlobalUnlockPtr(lp:pointer):Pointer;
begin
  GlobalUnlock(GlobalHandle(lp));
  GlobalUnlockPtr:=lp;
end;


function GlobalLockPtr(lp:pointer):Pointer;
begin
  GlobalLockPtr:=GlobalLock(GlobalHandle(lp));
end;


function GlobalReAllocPtr(lp:Pointer;cbNew,flags:DWord):Pointer;
begin
  GlobalReAllocPtr:=GlobalLock(GlobalReAlloc(HWND(GlobalUnlockPtr(lp)),cbNew,flags));
end;


function GlobalPtrHandle(lp:pointer):Pointer;
begin
  GlobalPtrHandle:=Pointer(GlobalHandle(lp));
end;

{
function ImageList_AddIcon(himl:HIMAGELIST; hicon:HICON):longint;
begin
  ImageList_AddIcon:=ImageList_ReplaceIcon(himl,-(1),hicon);
end;
}

function Animate_Create(hWndP:HWND; id:HMENU;dwStyle:DWORD;hInstance:HINST):HWND;
begin
  Animate_Create:=CreateWindow(LPCTSTR(ANIMATE_CLASS),nil,dwStyle,0,0,0,0,hwndP,id,hInstance,nil);
end;


function Animate_Open(hwnd : HWND;szName : LPTSTR) : LRESULT;
begin
  Animate_Open:=SendMessage(hwnd,ACM_OPEN,0,LPARAM(szName));
end;


function Animate_Play(hwnd : HWND;from,_to : longint;rep : UINT) : LRESULT;
begin
  Animate_Play:=SendMessage(hwnd,ACM_PLAY,WPARAM(rep),LPARAM(MAKELONG(from,_to)));
end;


function Animate_Stop(hwnd : HWND) : LRESULT;
begin
  Animate_Stop:=SendMessage(hwnd,ACM_STOP,0,0);
end;


function Animate_Close(hwnd : HWND) : LRESULT;
begin
  Animate_Close:=Animate_Open(hwnd,nil);
end;


function Animate_Seek(hwnd : HWND;frame : longint) : LRESULT;
begin
  Animate_Seek:=Animate_Play(hwnd,frame,frame,1);
end;


function PropSheet_AddPage(hPropSheetDlg : HWND;hpage : HPROPSHEETPAGE) : LRESULT;
begin
  PropSheet_AddPage:=SendMessage(hPropSheetDlg,PSM_ADDPAGE,0,LPARAM(hpage));
end;


function PropSheet_Apply(hPropSheetDlg : HWND) : LRESULT;
begin
  PropSheet_Apply:=SendMessage(hPropSheetDlg,PSM_APPLY,0,0);
end;


function PropSheet_CancelToClose(hPropSheetDlg : HWND) : LRESULT;
begin
  PropSheet_CancelToClose:=SendMessage(hPropSheetDlg,PSM_CANCELTOCLOSE,0,0);
end;


function PropSheet_Changed(hPropSheetDlg,hwndPage : HWND) : LRESULT;
begin
  PropSheet_Changed:=SendMessage(hPropSheetDlg,PSM_CHANGED,WPARAM(hwndPage),0);
end;


function PropSheet_GetCurrentPageHwnd(hDlg : HWND) : LRESULT;
begin
  PropSheet_GetCurrentPageHwnd:=SendMessage(hDlg,PSM_GETCURRENTPAGEHWND,0,0);
end;


function PropSheet_GetTabControl(hPropSheetDlg : HWND) : LRESULT;
begin
  PropSheet_GetTabControl:=SendMessage(hPropSheetDlg,PSM_GETTABCONTROL,0,0);
end;


function PropSheet_IsDialogMessage(hDlg : HWND;pMsg : longint) : LRESULT;
begin
  PropSheet_IsDialogMessage:=SendMessage(hDlg,PSM_ISDIALOGMESSAGE,0,LPARAM(pMsg));
end;


function PropSheet_PressButton(hPropSheetDlg : HWND;iButton : longint) : LRESULT;
begin
  PropSheet_PressButton:=SendMessage(hPropSheetDlg,PSM_PRESSBUTTON,WPARAM(longint(iButton)),0);
end;


function PropSheet_QuerySiblings(hPropSheetDlg : HWND;param1,param2 : longint) : LRESULT;
begin
  PropSheet_QuerySiblings:=SendMessage(hPropSheetDlg,PSM_QUERYSIBLINGS,WPARAM(param1),LPARAM(param2));
end;


function PropSheet_RebootSystem(hPropSheetDlg : HWND) : LRESULT;
begin
  PropSheet_RebootSystem:=SendMessage(hPropSheetDlg,PSM_REBOOTSYSTEM,0,0);
end;


function PropSheet_RemovePage(hPropSheetDlg : HWND;hpage : HPROPSHEETPAGE; index : longint) : LRESULT;
begin
  PropSheet_RemovePage:=SendMessage(hPropSheetDlg,PSM_REMOVEPAGE,WPARAM(index),LPARAM(hpage));
end;


function PropSheet_RestartWindows(hPropSheetDlg : HWND) : LRESULT;
begin
  PropSheet_RestartWindows:=SendMessage(hPropSheetDlg,PSM_RESTARTWINDOWS,0,0);
end;


function PropSheet_SetCurSel(hPropSheetDlg : HWND;hpage : HPROPSHEETPAGE; index : longint) : LRESULT;
begin
  PropSheet_SetCurSel:=SendMessage(hPropSheetDlg,PSM_SETCURSEL,WPARAM(index),LPARAM(hpage));
end;


function PropSheet_SetCurSelByID(hPropSheetDlg : HWND; id : longint) : LRESULT;
begin
  PropSheet_SetCurSelByID:=SendMessage(hPropSheetDlg,PSM_SETCURSELID,0,LPARAM(id));
end;


function PropSheet_SetFinishText(hPropSheetDlg:HWND;lpszText : LPTSTR) : LRESULT;
begin
  PropSheet_SetFinishText:=SendMessage(hPropSheetDlg,PSM_SETFINISHTEXT,0,LPARAM(lpszText));
end;


function PropSheet_SetTitle(hPropSheetDlg:HWND;dwStyle:DWORD;lpszText : LPCTSTR) : LRESULT;
begin
  PropSheet_SetTitle:=SendMessage(hPropSheetDlg,PSM_SETTITLE,WPARAM(dwStyle),LPARAM(lpszText));
end;


function PropSheet_SetWizButtons(hPropSheetDlg:HWND;dwFlags : DWORD) : LRESULT;
begin
  PropSheet_SetWizButtons:=SendMessage(hPropSheetDlg,PSM_SETWIZBUTTONS,0,LPARAM(dwFlags));
end;


function PropSheet_UnChanged(hPropSheetDlg:HWND;hwndPage : HWND) : LRESULT;
begin
  PropSheet_UnChanged:=SendMessage(hPropSheetDlg,PSM_UNCHANGED,WPARAM(hwndPage),0);
end;


function Header_DeleteItem(hwndHD:HWND;index : longint) : WINBOOL;
begin
  Header_DeleteItem:=WINBOOL(SendMessage(hwndHD,HDM_DELETEITEM,WPARAM(index),0));
end;


function Header_GetItem(hwndHD:HWND;index:longint;var hdi : HD_ITEM) : WINBOOL;
begin
  Header_GetItem:=WINBOOL(SendMessage(hwndHD,HDM_GETITEM,WPARAM(index),LPARAM(@hdi)));
end;


function Header_GetItemCount(hwndHD : HWND) : longint;
begin
  Header_GetItemCount:=longint(SendMessage(hwndHD,HDM_GETITEMCOUNT,0,0));
end;


function Header_InsertItem(hwndHD:HWND;index : longint;var hdi : HD_ITEM) : longint;
begin
  Header_InsertItem:=longint(SendMessage(hwndHD,HDM_INSERTITEM,WPARAM(index),LPARAM(@hdi)));
end;


function Header_Layout(hwndHD:HWND;var layout : HD_LAYOUT) : WINBOOL;
begin
  Header_Layout:=WINBOOL(SendMessage(hwndHD,HDM_LAYOUT,0,LPARAM(@layout)));
end;


function Header_SetItem(hwndHD:HWND;index : longint;var hdi : HD_ITEM) : WINBOOL;
begin
  Header_SetItem:=WINBOOL(SendMessage(hwndHD,HDM_SETITEM,WPARAM(index),LPARAM(@hdi)));
end;


function ListView_Arrange(hwndLV:HWND;code : UINT) : LRESULT;
begin
  ListView_Arrange:=SendMessage(hwndLV,LVM_ARRANGE,WPARAM(UINT(code)),0);
end;


function ListView_CreateDragImage(hwnd:HWND;i : longint;lpptUpLeft : LPPOINT) : LRESULT;
begin
  ListView_CreateDragImage:=SendMessage(hwnd,LVM_CREATEDRAGIMAGE,WPARAM(i),LPARAM(lpptUpLeft));
end;


function ListView_DeleteAllItems(hwnd : HWND) : LRESULT;
begin
  ListView_DeleteAllItems:=SendMessage(hwnd,LVM_DELETEALLITEMS,0,0);
end;


function ListView_DeleteColumn(hwnd:HWND;iCol : longint) : LRESULT;
begin
  ListView_DeleteColumn:=SendMessage(hwnd,LVM_DELETECOLUMN,WPARAM(iCol),0);
end;


function ListView_DeleteItem(hwnd:HWND;iItem : longint) : LRESULT;
begin
  ListView_DeleteItem:=SendMessage(hwnd,LVM_DELETEITEM,WPARAM(iItem),0);
end;


function ListView_EditLabel(hwndLV:HWND;i : longint) : LRESULT;
begin
  ListView_EditLabel:=SendMessage(hwndLV,LVM_EDITLABEL,WPARAM(longint(i)),0);
end;


function ListView_EnsureVisible(hwndLV:HWND;i,fPartialOK : longint) : LRESULT;
begin
  ListView_EnsureVisible:=SendMessage(hwndLV,LVM_ENSUREVISIBLE,WPARAM(i),MAKELPARAM(fPartialOK,0));
end;


function ListView_FindItem(hwnd:HWND;iStart : longint;var lvfi : LV_FINDINFO) : longint;
begin
  ListView_FindItem:=SendMessage(hwnd,LVM_FINDITEM,WPARAM(iStart),LPARAM(@lvfi));
end;


function ListView_GetBkColor(hwnd : HWND) : LRESULT;
begin
  ListView_GetBkColor:=SendMessage(hwnd,LVM_GETBKCOLOR,0,0);
end;


function ListView_GetCallbackMask(hwnd : HWND) : LRESULT;
begin
  ListView_GetCallbackMask:=SendMessage(hwnd,LVM_GETCALLBACKMASK,0,0);
end;


function ListView_GetColumn(hwnd:HWND;iCol : longint;var col : LV_COLUMN) : LRESULT;
begin
  ListView_GetColumn:=SendMessage(hwnd,LVM_GETCOLUMN,WPARAM(iCol),LPARAM(@col));
end;


function ListView_GetColumnWidth(hwnd:HWND;iCol : longint) : LRESULT;
begin
  ListView_GetColumnWidth:=SendMessage(hwnd,LVM_GETCOLUMNWIDTH,WPARAM(iCol),0);
end;


function ListView_GetCountPerPage(hwndLV : HWND) : LRESULT;
begin
  ListView_GetCountPerPage:=SendMessage(hwndLV,LVM_GETCOUNTPERPAGE,0,0);
end;


function ListView_GetEditControl(hwndLV : HWND) : LRESULT;
begin
  ListView_GetEditControl:=SendMessage(hwndLV,LVM_GETEDITCONTROL,0,0);
end;


function ListView_GetImageList(hwnd:HWND;iImageList : wINT) : LRESULT;
begin
  ListView_GetImageList:=SendMessage(hwnd,LVM_GETIMAGELIST,WPARAM(iImageList),0);
end;


function ListView_GetISearchString(hwndLV:HWND;lpsz : LPTSTR) : LRESULT;
begin
  ListView_GetISearchString:=SendMessage(hwndLV,LVM_GETISEARCHSTRING,0,LPARAM(lpsz));
end;


function ListView_GetItem(hwnd:HWND;var item : LV_ITEM) : LRESULT;
begin
  ListView_GetItem:=SendMessage(hwnd,LVM_GETITEM,0,LPARAM(@item));
end;


function ListView_GetItemCount(hwnd : HWND) : LRESULT;
begin
  ListView_GetItemCount:=SendMessage(hwnd,LVM_GETITEMCOUNT,0,0);
end;


function ListView_GetItemPosition(hwndLV:HWND;i : longint;var pt : POINT) : longint;
begin
  ListView_GetItemPosition:=SendMessage(hwndLV,LVM_GETITEMPOSITION,WPARAM(longint(i)),LPARAM(@pt));
end;


function ListView_GetItemSpacing(hwndLV:HWND;fSmall : longint) : LRESULT;
begin
  ListView_GetItemSpacing:=SendMessage(hwndLV,LVM_GETITEMSPACING,fSmall,0);
end;


function ListView_GetItemState(hwndLV:HWND;i,mask : longint) : LRESULT;
begin
  ListView_GetItemState:=SendMessage(hwndLV,LVM_GETITEMSTATE,WPARAM(i),LPARAM(mask));
end;


function ListView_GetNextItem(hwnd:HWND; iStart, flags : longint) : LRESULT;
begin
  ListView_GetNextItem:=SendMessage(hwnd, LVM_GETNEXTITEM, WPARAM(iStart), LPARAM(flags));
end;


function ListView_GetOrigin(hwndLV:HWND;var pt : POINT) : LRESULT;
begin
  ListView_GetOrigin:=SendMessage(hwndLV,LVM_GETORIGIN,WPARAM(0),LPARAM(@pt));
end;


function ListView_GetSelectedCount(hwndLV : HWND) : LRESULT;
begin
  ListView_GetSelectedCount:=SendMessage(hwndLV,LVM_GETSELECTEDCOUNT,0,0);
end;


function ListView_GetStringWidth(hwndLV:HWND;psz : LPCTSTR) : LRESULT;
begin
  ListView_GetStringWidth:=SendMessage(hwndLV,LVM_GETSTRINGWIDTH,0,LPARAM(psz));
end;


function ListView_GetTextBkColor(hwnd : HWND) : LRESULT;
begin
  ListView_GetTextBkColor:=SendMessage(hwnd,LVM_GETTEXTBKCOLOR,0,0);
end;


function ListView_GetTextColor(hwnd : HWND) : LRESULT;
begin
  ListView_GetTextColor:=SendMessage(hwnd,LVM_GETTEXTCOLOR,0,0);
end;


function ListView_GetTopIndex(hwndLV : HWND) : LRESULT;
begin
  ListView_GetTopIndex:=SendMessage(hwndLV,LVM_GETTOPINDEX,0,0);
end;


function ListView_GetViewRect(hwnd:HWND;var rc : RECT) : LRESULT;
begin
  ListView_GetViewRect:=SendMessage(hwnd,LVM_GETVIEWRECT,0,LPARAM(@rc));
end;


function ListView_HitTest(hwndLV:HWND;var info : LV_HITTESTINFO) : LRESULT;
begin
  ListView_HitTest:=SendMessage(hwndLV,LVM_HITTEST,0,LPARAM(@info));
end;


function ListView_InsertColumn(hwnd:HWND;iCol : longint;var col : LV_COLUMN) : LRESULT;
begin
  ListView_InsertColumn:=SendMessage(hwnd,LVM_INSERTCOLUMN,WPARAM(iCol),LPARAM(@col));
end;


function ListView_InsertItem(hwnd:HWND;var item : LV_ITEM) : LRESULT;
begin
  ListView_InsertItem:=SendMessage(hwnd,LVM_INSERTITEM,0,LPARAM(@item));
end;


function ListView_RedrawItems(hwndLV:HWND;iFirst,iLast : longint) : LRESULT;
begin
  ListView_RedrawItems:=SendMessage(hwndLV,LVM_REDRAWITEMS,WPARAM(iFirst),LPARAM(iLast));
end;


function ListView_Scroll(hwndLV:HWND;dx,dy : longint) : LRESULT;
begin
  ListView_Scroll:=SendMessage(hwndLV,LVM_SCROLL,WPARAM(dx),LPARAM(dy));
end;


function ListView_SetBkColor(hwnd:HWND;clrBk : COLORREF) : LRESULT;
begin
  ListView_SetBkColor:=SendMessage(hwnd,LVM_SETBKCOLOR,0,LPARAM(clrBk));
end;


function ListView_SetCallbackMask(hwnd:HWND;mask : UINT) : LRESULT;
begin
  ListView_SetCallbackMask:=SendMessage(hwnd,LVM_SETCALLBACKMASK,WPARAM(mask),0);
end;


function ListView_SetColumn(hwnd:HWND;iCol : longint; var col : LV_COLUMN) : LRESULT;
begin
  ListView_SetColumn:=SendMessage(hwnd,LVM_SETCOLUMN,WPARAM(iCol),LPARAM(@col));
end;


function ListView_SetColumnWidth(hwnd:HWND;iCol,cx : longint) : LRESULT;
begin
  ListView_SetColumnWidth:=SendMessage(hwnd,LVM_SETCOLUMNWIDTH,WPARAM(iCol),MAKELPARAM(cx,0));
end;


function ListView_SetImageList(hwnd:HWND;himl : longint;iImageList : HIMAGELIST) : LRESULT;
begin
  ListView_SetImageList:=SendMessage(hwnd,LVM_SETIMAGELIST,WPARAM(iImageList),LPARAM(UINT(himl)));
end;


function ListView_SetItem(hwnd:HWND;var item : LV_ITEM) : LRESULT;
begin
  ListView_SetItem:=SendMessage(hwnd,LVM_SETITEM,0,LPARAM(@item));
end;


function ListView_SetItemCount(hwndLV:HWND;cItems : longint) : LRESULT;
begin
  ListView_SetItemCount:=SendMessage(hwndLV,LVM_SETITEMCOUNT,WPARAM(cItems),0);
end;


function ListView_SetItemPosition(hwndLV:HWND;i,x,y : longint) : LRESULT;
begin
  ListView_SetItemPosition:=SendMessage(hwndLV,LVM_SETITEMPOSITION,WPARAM(i),MAKELPARAM(x,y));
end;


function ListView_SetItemPosition32(hwndLV:HWND;i,x,y : longint) : LRESULT;
var
  ptNewPos : POINT;
begin
  ptNewPos.x:=x;
  ptNewPos.y:=y;
  ListView_SetItemPosition32:=SendMessage(hwndLV, LVM_SETITEMPOSITION32, WPARAM(i),LPARAM(@ptNewPos));
end;


function ListView_SetItemState(hwndLV:HWND; i, data, mask:longint) : LRESULT;
var
  _gnu_lvi : LV_ITEM;
begin
  _gnu_lvi.stateMask:=mask;
  _gnu_lvi.state:=data;
  ListView_SetItemState:=SendMessage(hwndLV, LVM_SETITEMSTATE, WPARAM(i),LPARAM(@_gnu_lvi));
end;


function ListView_SetItemText(hwndLV:HWND; i, iSubItem_:longint;pszText_ : LPTSTR) : LRESULT;
var
  _gnu_lvi : LV_ITEM;
begin
  _gnu_lvi.iSubItem:=iSubItem_;
  _gnu_lvi.pszText:=pszText_;
  ListView_SetItemText:=SendMessage(hwndLV, LVM_SETITEMTEXT, WPARAM(i),LPARAM(@_gnu_lvi));
end;


function ListView_SetTextBkColor(hwnd:HWND;clrTextBk : COLORREF) : LRESULT;
begin
  ListView_SetTextBkColor:=SendMessage(hwnd,LVM_SETTEXTBKCOLOR,0,LPARAM(clrTextBk));
end;


function ListView_SetTextColor(hwnd:HWND;clrText : COLORREF) : LRESULT;
begin
  ListView_SetTextColor:=SendMessage(hwnd,LVM_SETTEXTCOLOR,0,LPARAM(clrText));
end;


function ListView_SortItems(hwndLV:HWND;_pfnCompare:PFNLVCOMPARE;_lPrm : LPARAM) : LRESULT;
begin
  ListView_SortItems:=SendMessage(hwndLV,LVM_SORTITEMS,WPARAM(_lPrm),LPARAM(_pfnCompare));
end;


function ListView_Update(hwndLV:HWND;i : longint) : LRESULT;
begin
  ListView_Update:=SendMessage(hwndLV,LVM_UPDATE,WPARAM(i),0);
end;


function TreeView_InsertItem(hwnd:HWND;lpis : LPTV_INSERTSTRUCT) : LRESULT;
begin
  TreeView_InsertItem:=SendMessage(hwnd,TVM_INSERTITEM,0,LPARAM(lpis));
end;


function TreeView_DeleteItem(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_DeleteItem:=SendMessage(hwnd,TVM_DELETEITEM,0,LPARAM(hitem));
end;


function TreeView_DeleteAllItems(hwnd : HWND) : LRESULT;
begin
  TreeView_DeleteAllItems:=SendMessage(hwnd,TVM_DELETEITEM,0,LPARAM(TVI_ROOT));
end;


function TreeView_Expand(hwnd:HWND;hitem:HTREEITEM;code : longint) : LRESULT;
begin
  TreeView_Expand:=SendMessage(hwnd,TVM_EXPAND,WPARAM(code),LPARAM(hitem));
end;


function TreeView_GetCount(hwnd : HWND) : LRESULT;
begin
  TreeView_GetCount:=SendMessage(hwnd,TVM_GETCOUNT,0,0);
end;


function TreeView_GetIndent(hwnd : HWND) : LRESULT;
begin
  TreeView_GetIndent:=SendMessage(hwnd,TVM_GETINDENT,0,0);
end;


function TreeView_SetIndent(hwnd:HWND;indent : longint) : LRESULT;
begin
  TreeView_SetIndent:=SendMessage(hwnd,TVM_SETINDENT,WPARAM(indent),0);
end;


function TreeView_GetImageList(hwnd:HWND;iImage : WPARAM) : LRESULT;
begin
  TreeView_GetImageList:=SendMessage(hwnd,TVM_GETIMAGELIST,iImage,0);
end;


function TreeView_SetImageList(hwnd:HWND;himl:HIMAGELIST;iImage : WPARAM) : LRESULT;
begin
  TreeView_SetImageList:=SendMessage(hwnd,TVM_SETIMAGELIST,iImage,LPARAM(UINT(himl)));
end;


function TreeView_GetNextItem(hwnd:HWND;hitem:HTREEITEM;code : longint) : LRESULT;
begin
  TreeView_GetNextItem:=SendMessage(hwnd,TVM_GETNEXTITEM,WPARAM(code),LPARAM(hitem));
end;


function TreeView_GetChild(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_GetChild:=TreeView_GetNextItem(hwnd,hitem,TVGN_CHILD);
end;


function TreeView_GetNextSibling(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_GetNextSibling:=TreeView_GetNextItem(hwnd,hitem,TVGN_NEXT);
end;


function TreeView_GetPrevSibling(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_GetPrevSibling:=TreeView_GetNextItem(hwnd,hitem,TVGN_PREVIOUS);
end;


function TreeView_GetParent(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_GetParent:=TreeView_GetNextItem(hwnd,hitem,TVGN_PARENT);
end;


function TreeView_GetFirstVisible(hwnd : HWND) : LRESULT;
begin
  TreeView_GetFirstVisible:=TreeView_GetNextItem(hwnd,HTREEITEM(nil),TVGN_FIRSTVISIBLE);
end;


function TreeView_GetNextVisible(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_GetNextVisible:=TreeView_GetNextItem(hwnd,hitem,TVGN_NEXTVISIBLE);
end;


function TreeView_GetPrevVisible(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_GetPrevVisible:=TreeView_GetNextItem(hwnd,hitem,TVGN_PREVIOUSVISIBLE);
end;


function TreeView_GetSelection(hwnd : HWND) : LRESULT;
begin
  TreeView_GetSelection:=TreeView_GetNextItem(hwnd,HTREEITEM(nil),TVGN_CARET);
end;


function TreeView_GetDropHilight(hwnd : HWND) : LRESULT;
begin
  TreeView_GetDropHilight:=TreeView_GetNextItem(hwnd,HTREEITEM(nil),TVGN_DROPHILITE);
end;


function TreeView_GetRoot(hwnd : HWND) : LRESULT;
begin
  TreeView_GetRoot:=TreeView_GetNextItem(hwnd,HTREEITEM(nil),TVGN_ROOT);
end;


function TreeView_Select(hwnd:HWND;hitem:HTREEITEM;code : longint) : LRESULT;
begin
  TreeView_Select:=SendMessage(hwnd,TVM_SELECTITEM,WPARAM(code),LPARAM(hitem));
end;


function TreeView_SelectItem(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_SelectItem:=TreeView_Select(hwnd,hitem,TVGN_CARET);
end;


function TreeView_SelectDropTarget(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_SelectDropTarget:=TreeView_Select(hwnd,hitem,TVGN_DROPHILITE);
end;


function TreeView_SelectSetFirstVisible(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_SelectSetFirstVisible:=TreeView_Select(hwnd,hitem,TVGN_FIRSTVISIBLE);
end;


function TreeView_GetItem(hwnd:HWND;var item : TV_ITEM) : LRESULT;
begin
  TreeView_GetItem:=SendMessage(hwnd,TVM_GETITEM,0,LPARAM(@item));
end;


function TreeView_SetItem(hwnd:HWND;var item : TV_ITEM) : LRESULT;
begin
  TreeView_SetItem:=SendMessage(hwnd,TVM_SETITEM,0,LPARAM(@item));
end;


function TreeView_EditLabel(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_EditLabel:=SendMessage(hwnd,TVM_EDITLABEL,0,LPARAM(hitem));
end;


function TreeView_GetEditControl(hwnd : HWND) : LRESULT;
begin
  TreeView_GetEditControl:=SendMessage(hwnd,TVM_GETEDITCONTROL,0,0);
end;


function TreeView_GetVisibleCount(hwnd : HWND) : LRESULT;
begin
  TreeView_GetVisibleCount:=SendMessage(hwnd,TVM_GETVISIBLECOUNT,0,0);
end;


function TreeView_HitTest(hwnd:HWND;lpht : LPTV_HITTESTINFO) : LRESULT;
begin
  TreeView_HitTest:=SendMessage(hwnd,TVM_HITTEST,0,LPARAM(lpht));
end;


function TreeView_CreateDragImage(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_CreateDragImage:=SendMessage(hwnd,TVM_CREATEDRAGIMAGE,0,LPARAM(hitem));
end;


function TreeView_SortChildren(hwnd:HWND;hitem:HTREEITEM;recurse : longint) : LRESULT;
begin
  TreeView_SortChildren:=SendMessage(hwnd,TVM_SORTCHILDREN,WPARAM(recurse),LPARAM(hitem));
end;


function TreeView_EnsureVisible(hwnd:HWND;hitem : HTREEITEM) : LRESULT;
begin
  TreeView_EnsureVisible:=SendMessage(hwnd,TVM_ENSUREVISIBLE,0,LPARAM(hitem));
end;


function TreeView_SortChildrenCB(hwnd:HWND;psort:LPTV_SORTCB;recurse : longint) : LRESULT;
begin
  TreeView_SortChildrenCB:=SendMessage(hwnd,TVM_SORTCHILDRENCB,WPARAM(recurse),LPARAM(psort));
end;


function TreeView_EndEditLabelNow(hwnd:HWND;fCancel : longint) : LRESULT;
begin
  TreeView_EndEditLabelNow:=SendMessage(hwnd,TVM_ENDEDITLABELNOW,WPARAM(fCancel),0);
end;


function TreeView_GetISearchString(hwndTV:HWND;lpsz : LPTSTR) : LRESULT;
begin
  TreeView_GetISearchString:=SendMessage(hwndTV,TVM_GETISEARCHSTRING,0,LPARAM(lpsz));
end;


function TabCtrl_GetImageList(hwnd : HWND) : LRESULT;
begin
  TabCtrl_GetImageList:=SendMessage(hwnd,TCM_GETIMAGELIST,0,0);
end;


function TabCtrl_SetImageList(hwnd:HWND;himl : HIMAGELIST) : LRESULT;
begin
  TabCtrl_SetImageList:=SendMessage(hwnd,TCM_SETIMAGELIST,0,LPARAM(UINT(himl)));
end;


function TabCtrl_GetItemCount(hwnd : HWND) : LRESULT;
begin
  TabCtrl_GetItemCount:=SendMessage(hwnd,TCM_GETITEMCOUNT,0,0);
end;


function TabCtrl_GetItem(hwnd:HWND;iItem : longint;var item : TC_ITEM) : LRESULT;
begin
  TabCtrl_GetItem:=SendMessage(hwnd,TCM_GETITEM,WPARAM(iItem),LPARAM(@item));
end;


function TabCtrl_SetItem(hwnd:HWND;iItem : longint;var item : TC_ITEM) : LRESULT;
begin
  TabCtrl_SetItem:=SendMessage(hwnd,TCM_SETITEM,WPARAM(iItem),LPARAM(@item));
end;


function TabCtrl_InsertItem(hwnd:HWND;iItem : longint;var item : TC_ITEM) : LRESULT;
begin
  TabCtrl_InsertItem:=SendMessage(hwnd,TCM_INSERTITEM,WPARAM(iItem),LPARAM(@item));
end;


function TabCtrl_DeleteItem(hwnd:HWND;i : longint) : LRESULT;
begin
  TabCtrl_DeleteItem:=SendMessage(hwnd,TCM_DELETEITEM,WPARAM(i),0);
end;


function TabCtrl_DeleteAllItems(hwnd : HWND) : LRESULT;
begin
  TabCtrl_DeleteAllItems:=SendMessage(hwnd,TCM_DELETEALLITEMS,0,0);
end;


function TabCtrl_GetItemRect(hwnd:HWND;i : longint;var rc : RECT) : LRESULT;
begin
  TabCtrl_GetItemRect:=SendMessage(hwnd,TCM_GETITEMRECT,WPARAM(longint(i)),LPARAM(@rc));
end;


function TabCtrl_GetCurSel(hwnd : HWND) : LRESULT;
begin
  TabCtrl_GetCurSel:=SendMessage(hwnd,TCM_GETCURSEL,0,0);
end;


function TabCtrl_SetCurSel(hwnd:HWND;i : longint) : LRESULT;
begin
  TabCtrl_SetCurSel:=SendMessage(hwnd,TCM_SETCURSEL,WPARAM(i),0);
end;


function TabCtrl_HitTest(hwndTC:HWND;var info : TC_HITTESTINFO) : LRESULT;
begin
  TabCtrl_HitTest:=SendMessage(hwndTC,TCM_HITTEST,0,LPARAM(@info));
end;


function TabCtrl_SetItemExtra(hwndTC:HWND;cb : longint) : LRESULT;
begin
  TabCtrl_SetItemExtra:=SendMessage(hwndTC,TCM_SETITEMEXTRA,WPARAM(cb),0);
end;


function TabCtrl_AdjustRect(hwnd:HWND;bLarger:WINBOOL;var rc : RECT) : LRESULT;
begin
  TabCtrl_AdjustRect:=SendMessage(hwnd,TCM_ADJUSTRECT,WPARAM(bLarger),LPARAM(@rc));
end;


function TabCtrl_SetItemSize(hwnd:HWND;x,y : longint) : LRESULT;
begin
  TabCtrl_SetItemSize:=SendMessage(hwnd,TCM_SETITEMSIZE,0,MAKELPARAM(x,y));
end;


function TabCtrl_RemoveImage(hwnd:HWND;i : WPARAM) : LRESULT;
begin
  TabCtrl_RemoveImage:=SendMessage(hwnd,TCM_REMOVEIMAGE,i,0);
end;


function TabCtrl_SetPadding(hwnd:HWND;cx,cy : longint) : LRESULT;
begin
  TabCtrl_SetPadding:=SendMessage(hwnd,TCM_SETPADDING,0,MAKELPARAM(cx,cy));
end;


function TabCtrl_GetRowCount(hwnd : HWND) : LRESULT;
begin
  TabCtrl_GetRowCount:=SendMessage(hwnd,TCM_GETROWCOUNT,0,0);
end;


function TabCtrl_GetToolTips(hwnd : HWND) : LRESULT;
begin
  TabCtrl_GetToolTips:=SendMessage(hwnd,TCM_GETTOOLTIPS,0,0);
end;


function TabCtrl_SetToolTips(hwnd:HWND;hwndTT : longint) : LRESULT;
begin
  TabCtrl_SetToolTips:=SendMessage(hwnd,TCM_SETTOOLTIPS,WPARAM(hwndTT),0);
end;


function TabCtrl_GetCurFocus(hwnd : HWND) : LRESULT;
begin
  TabCtrl_GetCurFocus:=SendMessage(hwnd,TCM_GETCURFOCUS,0,0);
end;


function TabCtrl_SetCurFocus(hwnd:HWND;i : longint) : LRESULT;
begin
  TabCtrl_SetCurFocus:=SendMessage(hwnd,TCM_SETCURFOCUS,i,0);
end;


function SNDMSG(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT;
begin
  SNDMSG:=SendMessage(hWnd,Msg,wParam,lParam);
end;

function SNDMSGW(hWnd:HWND; Msg:UINT; wParam:WPARAM; lParam:LPARAM):LRESULT;
begin
  SNDMSGW:=SendMessageW(hWnd,Msg,wParam,lParam);
end;


function CommDlg_OpenSave_GetSpecA(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetSpecA:=SNDMSG(_hdlg,CDM_GETSPEC,WPARAM(_cbmax),LPARAM(_psz));
end;


function CommDlg_OpenSave_GetSpecW(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetSpecW:=SNDMSGW(_hdlg,CDM_GETSPEC,WPARAM(_cbmax),LPARAM(_psz));
end;


{$ifndef Unicode}
function CommDlg_OpenSave_GetSpec(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetSpec:=SNDMSG(_hdlg,CDM_GETSPEC,WPARAM(_cbmax),LPARAM(_psz));
end;
{$else Unicode}
function CommDlg_OpenSave_GetSpec(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetSpec:=SNDMSGW(_hdlg,CDM_GETSPEC,WPARAM(_cbmax),LPARAM(_psz));
end;
{$endif Unicode}


function CommDlg_OpenSave_GetFilePathA(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetFilePathA:=SNDMSG(_hdlg,CDM_GETFILEPATH,WPARAM(_cbmax),LPARAM(_psz));
end;


function CommDlg_OpenSave_GetFilePathW(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetFilePathW:=SNDMSGW(_hdlg,CDM_GETFILEPATH,WPARAM(_cbmax),LPARAM(LPWSTR(_psz)));
end;

{$ifndef Unicode}
function CommDlg_OpenSave_GetFilePath(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetFilePath:=SNDMSG(_hdlg,CDM_GETFILEPATH,WPARAM(_cbmax),LPARAM(_psz));
end;
{$else Unicode}
function CommDlg_OpenSave_GetFilePath(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetFilePath:=SNDMSGW(_hdlg,CDM_GETFILEPATH,WPARAM(_cbmax),LPARAM(_psz));
end;
{$endif Unicode}

function CommDlg_OpenSave_GetFolderPathA(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetFolderPathA:=SNDMSG(_hdlg,CDM_GETFOLDERPATH,WPARAM(_cbmax),LPARAM(LPSTR(_psz)));
end;


function CommDlg_OpenSave_GetFolderPathW(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetFolderPathW:=SNDMSGW(_hdlg,CDM_GETFOLDERPATH,WPARAM(_cbmax),LPARAM(LPWSTR(_psz)));
end;

{$ifndef Unicode}
function CommDlg_OpenSave_GetFolderPath(_hdlg:HWND;_psz:LPSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetFolderPath:=SNDMSG(_hdlg,CDM_GETFOLDERPATH,WPARAM(_cbmax),LPARAM(LPSTR(_psz)));
end;
{$else Unicode}
function CommDlg_OpenSave_GetFolderPath(_hdlg:HWND;_psz:LPWSTR;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetFolderPath:=SNDMSGW(_hdlg,CDM_GETFOLDERPATH,WPARAM(_cbmax),LPARAM(LPWSTR(_psz)));
end;
{$endif Unicode}

function CommDlg_OpenSave_GetFolderIDList(_hdlg:HWND;_pidl:LPVOID;_cbmax : longint) : LRESULT;
begin
  CommDlg_OpenSave_GetFolderIDList:=SNDMSG(_hdlg,CDM_GETFOLDERIDLIST,WPARAM(_cbmax),LPARAM(_pidl));
end;


function CommDlg_OpenSave_SetControlText(_hdlg:HWND;_id : longint;_text : LPSTR) : LRESULT;
begin
  CommDlg_OpenSave_SetControlText:=SNDMSG(_hdlg,CDM_SETCONTROLTEXT,WPARAM(_id),LPARAM(_text));
end;


function CommDlg_OpenSave_HideControl(_hdlg:HWND;_id : longint) : LRESULT;
begin
  CommDlg_OpenSave_HideControl:=SNDMSG(_hdlg,CDM_HIDECONTROL,WPARAM(_id),0);
end;


function CommDlg_OpenSave_SetDefExt(_hdlg:HWND;_pszext : LPSTR) : LRESULT;
begin
  CommDlg_OpenSave_SetDefExt:=SNDMSG(_hdlg,CDM_SETDEFEXT,0,LPARAM(_pszext));
end;

function InternalGetLargestConsoleWindowSize(hConsoleOutput:HANDLE):DWord; external 'kernel32' name 'GetLargestConsoleWindowSize';

function GetLargestConsoleWindowSize(hConsoleOutput:HANDLE):COORD;
var
  res : dword;
begin
  res:=InternalGetLargestConsoleWindowSize(hConsoleOutput);
  GetLargestConsoleWindowSize:=COORD(res);
end;

{
#define IMAGE_FIRST_SECTION( ntheader ) ((PIMAGE_SECTION_HEADER) \
    ((ULONG_PTR)(ntheader) + \
     FIELD_OFFSET( IMAGE_NT_HEADERS, OptionalHeader ) + \
     ((ntheader))->FileHeader.SizeOfOptionalHeader \
    )) 

}

function IMAGE_FIRST_SECTION(ntheader : PIMAGE_NT_HEADERS):PIMAGE_SECTION_HEADER;
// (PTRUINT(ntheader)+ OFS(ntheader^.OptionalHeader) - OFS(NTHEADER);  -> address of ntheader cancels.
begin
  IMAGE_FIRST_SECTION:=PIMAGE_SECTION_HEADER(OFS(ntheader^.OptionalHeader) + ntheader^.FileHeader.SizeOfOptionalHeader);
end;

{$endif read_implementation}

