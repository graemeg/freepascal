{
    Basic stuff for windows rtls

    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
   { constants for GetStdHandle }
   STD_INPUT_HANDLE = dword(-10);
   STD_OUTPUT_HANDLE = dword(-11);
   STD_ERROR_HANDLE = dword(-12);
   INVALID_HANDLE_VALUE = THandle(-1);

   IGNORE = 0;               { Ignore signal }
   INFINITE = longint($FFFFFFFF);     { Infinite timeout }

   { flags for CreateFile }
   GENERIC_READ=$80000000;
   GENERIC_WRITE=$40000000;
   CREATE_NEW = 1;
   CREATE_ALWAYS = 2;
   OPEN_EXISTING = 3;
   OPEN_ALWAYS = 4;
   TRUNCATE_EXISTING = 5;

   FILE_ATTRIBUTE_ARCHIVE = 32;
   FILE_ATTRIBUTE_COMPRESSED = 2048;
   FILE_ATTRIBUTE_NORMAL = 128;
   FILE_ATTRIBUTE_DIRECTORY = 16;
   FILE_ATTRIBUTE_HIDDEN = 2;
   FILE_ATTRIBUTE_READONLY = 1;
   FILE_ATTRIBUTE_SYSTEM = 4;
   FILE_ATTRIBUTE_TEMPORARY = 256;

   { Share mode open }
   fmShareCompat    = $00000000;
   fmShareExclusive = $10;
   fmShareDenyWrite = $20;
   fmShareDenyRead  = $30;
   fmShareDenyNone  = $40;

   { flags for SetFilePos }
   FILE_BEGIN = 0;
   FILE_CURRENT = 1;
   FILE_END = 2;

   { GetFileType  }
   FILE_TYPE_UNKNOWN = 0;
   FILE_TYPE_DISK = 1;
   FILE_TYPE_CHAR = 2;
   FILE_TYPE_PIPE = 3;

   VER_PLATFORM_WIN32s = 0;
   VER_PLATFORM_WIN32_WINDOWS = 1;
   VER_PLATFORM_WIN32_NT = 2;

   { DllEntryPoint }
   DLL_PROCESS_ATTACH = 1;
   DLL_THREAD_ATTACH = 2;
   DLL_PROCESS_DETACH = 0;
   DLL_THREAD_DETACH = 3;

  { These constants are used for conversion of error codes }
  { from win32 i/o errors to tp i/o errors                 }
  { errors 1 to 18 are the same as in Turbo Pascal         }
  { DO NOT MODIFY UNLESS YOU KNOW EXACTLY WHAT YOU ARE DOING! }

{  The media is write protected.                   }
    ERROR_WRITE_PROTECT       =      19;
{  The system cannot find the device specified.    }
    ERROR_BAD_UNIT            =      20;
{  The device is not ready.                        }
    ERROR_NOT_READY           =      21;
{  The device does not recognize the command.      }
    ERROR_BAD_COMMAND         =      22;
{  Data error (cyclic redundancy check)            }
    ERROR_CRC                 =      23;
{  The program issued a command but the            }
{  command length is incorrect.                    }
    ERROR_BAD_LENGTH           =     24;
{  The drive cannot locate a specific              }
{  area or track on the disk.                      }
    ERROR_SEEK                 =     25;
{  The specified disk or diskette cannot be accessed. }
    ERROR_NOT_DOS_DISK         =     26;
{  The drive cannot find the sector requested.     }
    ERROR_SECTOR_NOT_FOUND      =    27;
{  The printer is out of paper.                    }
    ERROR_OUT_OF_PAPER          =    28;
{  The system cannot write to the specified device. }
    ERROR_WRITE_FAULT           =    29;
{  The system cannot read from the specified device. }
    ERROR_READ_FAULT            =    30;
{  A device attached to the system is not functioning.}
    ERROR_GEN_FAILURE           =    31;
{  The process cannot access the file because         }
{  it is being used by another process.               }
    ERROR_SHARING_VIOLATION      =   32;
{   A pipe has been closed on the other end }
{   Removing that error allows eof to works as on other OSes }
    ERROR_BROKEN_PIPE = 109;
    ERROR_DISK_FULL = 112;
    ERROR_DIR_NOT_EMPTY = 145;
    ERROR_ALREADY_EXISTS = 183;
    ERROR_FILENAME_EXCED_RANGE = 206;

    EXCEPTION_MAXIMUM_PARAMETERS = 15;

    FORMAT_MESSAGE_FROM_SYSTEM = 4096;
    LANG_NEUTRAL = $00;
    SUBLANG_DEFAULT = $00;

type
   {UINT  = longint;
   BOOL  = longint; obsolete }
   UINT  = cardinal;
   BOOL  = longbool;
//   WCHAR = word;
{$ifdef FPC_OS_UNICODE}
   LPTCH   = ^word;
   LPTSTR  = ^word;
   LPCTSTR = ^word;
{$else FPC_OS_UNICODE}
   LPTCH   = ^char;
   LPTSTR  = ^char;
   LPCTSTR = ^char;
{$endif FPC_OS_UNICODE}
   LPWSTR  = ^wchar;
   PVOID   = pointer;
   LPVOID  = pointer;
   LPCVOID = pointer;
   LPDWORD = ^DWORD;
   HLocal  = THandle;
   PStr    = pchar;
   LPStr   = pchar;
   PLPSTR  = ^LPSTR;
   PLPWSTR = ^LPWSTR;

   { WARNING
     the variable argument list
     is not implemented for FPC
     va_list is just a dummy record
     MvdV: Nevertheless it should be a pointer type, not a record}
   va_list = pchar;

  PSecurityAttributes = ^TSecurityAttributes;
  TSecurityAttributes = record
    nLength : DWORD;
    lpSecurityDescriptor : Pointer;
    bInheritHandle : BOOL;
  end;

  PProcessInformation = ^TProcessInformation;
  TProcessInformation = record
    hProcess: THandle;
    hThread: THandle;
    dwProcessId: DWORD;
    dwThreadId: DWORD;
  end;

  PFileTime = ^TFileTime;
  TFileTime = record
    dwLowDateTime,
    dwHighDateTime : DWORD;
  end;

  LPSystemTime= ^PSystemTime;
  PSystemTime = ^TSystemTime;
  TSystemTime = record
    wYear,
    wMonth,
    wDayOfWeek,
    wDay,
    wHour,
    wMinute,
    wSecond,
    wMilliseconds: Word;
  end;

  TTlsDirectory=packed record
    data_start, data_end : pointer;
    index_pointer, callbacks_pointer : pointer;
    zero_fill_size : dword;
    flags : dword;
  end;

  PExceptionRecord = ^TExceptionRecord;
  TExceptionRecord = record
    ExceptionCode   : cardinal;
    ExceptionFlags  : cardinal;
    ExceptionRecord : PExceptionRecord;
    ExceptionAddress : Pointer;
    NumberParameters : Longint;
    ExceptionInformation : array[0..EXCEPTION_MAXIMUM_PARAMETERS-1] of Pointer;
  end;

  TSystemInfo = record
           case LongInt of
              0 : ( dwOemId : DWord;
               dwPageSize : DWord;
               lpMinimumApplicationAddress : Pointer;
               lpMaximumApplicationAddress : Pointer;
               dwActiveProcessorMask : PDWord;
               dwNumberOfProcessors : DWord;
                    dwProcessorType : DWord;
                    dwAllocationGranularity : DWord;
                    wProcessorLevel : Word;
                    wProcessorRevision : Word;
                 );
              1 : (
                   wProcessorArchitecture : Word;
                   wReserved : Word;
                 );
    end;
  PSystemInfo = ^TSystemInfo;


   { misc. functions }
   function GetLastError : DWORD;
     {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'GetLastError';

   procedure SetLastError(dwErrCode : DWORD);
     {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'SetLastError';

   { time and date functions }
   function GetTickCount : DWORD;
     {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'GetTickCount';

{$ifndef WINCE}
   { process functions }
   procedure ExitProcess(uExitCode : UINT);
     stdcall;external KernelDLL name 'ExitProcess';

   { Startup }
   procedure GetStartupInfo(p : pointer);
     stdcall;external KernelDLL name 'GetStartupInfoA';
   function GetStdHandle(nStdHandle:DWORD):THANDLE;
     stdcall;external KernelDLL name 'GetStdHandle';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   {$ifdef FPC_UNICODE_RTLx}
   { command line/environment functions }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   { command line/environment functions }
   function GetCommandLine : pchar;
     stdcall;external KernelDLL name 'GetCommandLineA';
>>>>>>> graemeg/cpstrnew

   function GetCommandLine : pwidechar;
     stdcall;external KernelDLL name 'GetCommandLineW';
   {$else}
   function GetCommandLine : pchar;
     stdcall;external KernelDLL name 'GetCommandLineA';
   {$endif}

   function GetCommandLineW : pwidechar;
     stdcall;external KernelDLL name 'GetCommandLineW';

   function GetCommandLineW : pwidechar;
     stdcall;external KernelDLL name 'GetCommandLineW';

  function GetCurrentProcessId:DWORD;
    stdcall; external KernelDLL name 'GetCurrentProcessId';

  function Win32GetCurrentThreadId:DWORD;
    stdcall; external KernelDLL name 'GetCurrentThreadId';

  function GetCurrentProcess : THandle;
    stdcall;external 'kernel32' name 'GetCurrentProcess';

  function ReadProcessMemory(process : THandle;address : pointer;dest : pointer;size : ptruint;bytesread : pptruint) :  longbool;
    stdcall;external 'kernel32' name 'ReadProcessMemory';

  { module functions }
  function GetModuleFileName(l1:THandle;p:PChar;l2:longint):longint;
    stdcall;external KernelDLL name 'GetModuleFileNameA';

  function GetModuleHandle(p : PChar) : THandle;
    stdcall;external KernelDLL name 'GetModuleHandleA';

{$else WINCE}

   { module functions }
   function GetModuleFileName(l1:THandle;p:PWideChar;l2:longint):longint;
     cdecl;external KernelDLL name 'GetModuleFileNameW';

   function GetModuleHandle(p : PWideChar) : THandle;
     cdecl;external KernelDLL name 'GetModuleHandleW';

{$endif WINCE}
   { file functions }
   function WriteFile(fh:thandle;buf:pointer;len:longint;var loaded:longint;
     overlap:pointer):longint;
     {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'WriteFile';
   function ReadFile(fh:thandle;buf:pointer;len:longint;var loaded:longint;
     overlap:pointer):longint;
     {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'ReadFile';
   function CloseHandle(h : thandle) : longint;
     {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'CloseHandle';
   function SetFilePointer(l1,l2 : thandle;l3 : pointer;l4 : longint) : longint;
     {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'SetFilePointer';
   function GetFileSize(h:thandle;p:pointer) : longint;
     {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'GetFileSize';
   function SetEndOfFile(h : thandle) : longbool;
     {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'SetEndOfFile';

   procedure GetSystemInfo(lpSystemInfo: PSystemInfo); {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'GetSystemInfo';

   function WinLoadLibraryW(lpLibFileName:pwidechar):THandle; {$ifdef wince}cdecl{$else}stdcall{$endif}; external KernelDLL name 'LoadLibraryW';
   function WinLoadLibraryA(lpLibFileName:pwidechar):THandle; {$ifdef wince}cdecl{$else}stdcall{$endif}; external KernelDLL name 'LoadLibraryA';
   {$ifdef wince}
   function WinGetProcAddress(hModule:THandle; lpProcName:pchar):pointer; {$ifdef wince}cdecl{$else}stdcall{$endif}; external KernelDLL name 'GetProcAddressA';
   {$else}
   function WinGetProcAddress(hModule:THandle; lpProcName:pchar):pointer; {$ifdef wince}cdecl{$else}stdcall{$endif}; external KernelDLL name 'GetProcAddress';
   {$endif}
   function WinFreeLibrary(hLibModule:THandle):ByteBool; {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'FreeLibrary';

   function FormatMessageW(dwFlags:DWORD; lpSource:LPCVOID; dwMessageId:DWORD; dwLanguageId:DWORD; lpBuffer:LPWSTR;nSize:DWORD; Arguments:va_list):DWORD;{$ifdef wince}cdecl{$else}stdcall{$endif}; external KernelDLL name 'FormatMessageW';

{$ifndef WINCE}
   function GetFileType(Handle:thandle):DWord;
     stdcall;external KernelDLL name 'GetFileType';

   { File }
   function DeleteFileW(p : punicodechar) : longint;
     stdcall;external KernelDLL name 'DeleteFileW';
   function MoveFileW(old,_new : punicodechar) : longint;
     stdcall;external KernelDLL name 'MoveFileW';
   function CreateFileW(lpFileName:punicodechar; dwDesiredAccess:DWORD; dwShareMode:DWORD;
                       lpSecurityAttributes:PSECURITYATTRIBUTES; dwCreationDisposition:DWORD;
                       dwFlagsAndAttributes:DWORD; hTemplateFile:DWORD):THandle;
     stdcall;external KernelDLL name 'CreateFileW';
   function GetFileAttributesW(p : punicodechar) : dword;
     stdcall;external KernelDLL name 'GetFileAttributesW';

   { Directory }
<<<<<<< HEAD
   function CreateDirectoryW(name : pointer;sec : pointer) : longbool;
     stdcall;external KernelDLL name 'CreateDirectoryW';
   function RemoveDirectoryW(name:pointer):longbool;
     stdcall;external KernelDLL name 'RemoveDirectoryW';
   function SetCurrentDirectoryW(name : pointer) : longbool;
     stdcall;external KernelDLL name 'SetCurrentDirectoryW';
   function GetCurrentDirectoryW(bufsize : longint;name : punicodechar) : Dword;
     stdcall;external KernelDLL name 'GetCurrentDirectoryW';
   function GetFullPathNameW (lpFileName: PUnicodeChar; nBufferLength: DWord;
     lpBuffer: PUnicodeChar; var lpFilePart: PUnicodeChar): DWord;
     stdcall; external KernelDLL name 'GetFullPathNameW';
   function SetEnvironmentVariableW (lpName: PUnicodeChar; lpValue: PUnicodeChar): BOOL;
     stdcall; external KernelDLL name 'SetEnvironmentVariableW';

=======
   function CreateDirectory(name : pointer;sec : pointer) : longbool;
     stdcall;external KernelDLL name 'CreateDirectoryA';
   function RemoveDirectory(name:pointer):longbool;
     stdcall;external KernelDLL name 'RemoveDirectoryA';
   function SetCurrentDirectory(name : pointer) : longbool;
     stdcall;external KernelDLL name 'SetCurrentDirectoryA';
   function GetCurrentDirectory(bufsize : longint;name : pchar) : longbool;
     stdcall;external KernelDLL name 'GetCurrentDirectoryA';
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
   { Console functions needed for WriteFile fix for bug 17550 }
   function GetConsoleMode(hConsoleHandle:thandle; lpMode:LPDWORD):BOOL;
     stdcall;external KernelDLL name 'GetConsoleMode';
   function GetConsoleOutputCP : UINT;
     stdcall; external KernelDLL name 'GetConsoleOutputCP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   function GetOEMCP: UINT;
     stdcall; external KernelDLL name 'GetOEMCP';

   function SysAllocStringLen(psz:pointer;len:dword):pointer;
     stdcall; external 'oleaut32.dll' name 'SysAllocStringLen';
   procedure SysFreeString(bstr:pointer);
     stdcall; external 'oleaut32.dll' name 'SysFreeString';
   function SysReAllocStringLen(var bstr:pointer;psz: pointer;
     len:dword): Integer; stdcall;external 'oleaut32.dll' name 'SysReAllocStringLen';
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
{$endif WINCE}

   Procedure Errno2InOutRes(oserror: longword);
   var
     res : Word;
   Begin
     { DO NOT MODIFY UNLESS YOU KNOW EXACTLY WHAT YOU ARE DOING }
     case oserror of
       ERROR_WRITE_PROTECT..ERROR_GEN_FAILURE :
         begin
           { This is the offset to the Win32 to add to directly map  }
           { to the DOS/TP compatible error codes when in this range }
           res := word(oserror)+131;
         end;
       ERROR_DISK_FULL :
         res := 101;
       ERROR_DIR_NOT_EMPTY,
       ERROR_ALREADY_EXISTS,
       ERROR_SHARING_VIOLATION :
         begin
           res :=5;
         end;
       ERROR_FILENAME_EXCED_RANGE : Res := 3;
       else
         begin
           { other error codes can directly be mapped }
           res := Word(oserror);
         end;
     end;
     InOutRes:=res;
   end;


  function OleStrToString(source: PWideChar) : UnicodeString;inline;
    begin
      OleStrToStrVar(source,result);
    end;


  procedure OleStrToStrVar(source : PWideChar;var dest : UnicodeString);inline;
    begin
      WideCharLenToStrVar(source,length(WideString(pointer(source))),dest);
    end;


  procedure OleStrToStrVar(source : PWideChar;var dest : AnsiString);inline;
    begin
      WideCharLenToStrVar(source,length(WideString(pointer(source))),dest);
    end;


  function StringToOleStr(const source : ansistring) : PWideChar;inline;
    begin
      result:=nil;
      widestringmanager.Ansi2WideMoveProc(pchar(pointer(source)),StringCodePage(source),widestring(pointer(result)),length(source));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end;


  Function NewWideString(Len : SizeInt) : Pointer;forward;


  function StringToOleStr(const source : UnicodeString) : PWideChar;inline;
    begin
      if source<>'' then
        begin
          result:=NewWideString(Length(source));
          move(source[1],result^,Length(source));
        end
      else
        result:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
    end;


  Function NewWideString(Len : SizeInt) : Pointer;forward;


  function StringToOleStr(const source : UnicodeString) : PWideChar;inline;
    begin
      if source<>'' then
        begin
          result:=NewWideString(Length(source));
          move(source[1],result^,Length(source));
        end
      else
        result:=nil;
>>>>>>> graemeg/cpstrnew
    end;


  function StringToOleStr(const source : UnicodeString) : PWideChar;inline;
    begin
      result:=nil;
      { will call UnicodeString overload if FPC_WIDESTRING_EQUAL_UNICODESTRING is set }
      SetString(widestring(pointer(result)),PWideChar(source),length(source));
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end;


{$ifndef WINCE}
{$define FPC_HAS_TRANSLATEPLACEHOLDERCP}
function TranslatePlaceholderCP(cp: TSystemCodePage): TSystemCodePage; {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  TranslatePlaceholderCP:=cp;
  case cp of
    CP_OEMCP:
      TranslatePlaceholderCP:=GetOEMCP;
    CP_ACP:
      TranslatePlaceholderCP:=DefaultSystemCodePage;
  end;
end;
{$endif not WINCE}

{$define HAS_GETCPUCOUNT}
function GetCPUCount: LongWord;
var
  info: TSystemInfo;
begin
  FillChar(info, SizeOf(info), 0);
  GetSystemInfo(@info);
  Result := info.dwNumberOfProcessors;
end;
