{
    Basic stuff for NativeNT RTLs

    This file is part of the Free Pascal run time library.
    Copyright (c) 2009-2010 by Sven Barth

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
{$ifdef kmode}
  ntdll = 'ntoskrnl.exe';
{$else}
  ntdll = 'ntdll.dll';
{$endif}

type
  PHandle = ^THandle;

  TNtUnicodeString = packed record
    Length: Word;        // used characters in buffer
    MaximumLength: Word; // maximum characters in buffer
    Buffer: PWideChar;
  end;
  PNtUnicodeString = ^TNtUnicodeString;

  // using Int64 is an alternative (QWord might have unintended side effects)
  TLargeInteger = packed record
    case Boolean of
      True:(LowPart: LongWord;
            HighPart: LongInt);
      False:(QuadPart: Int64);
  end;
  PLargeInteger = ^TLargeInteger;


  TObjectAttributes = record
    Length: LongWord;
    RootDirectory: THandle;
    ObjectName: PNtUnicodeString;
    Attributes: LongWord;
    SecurityDescriptor: Pointer;       // Points to type SECURITY_DESCRIPTOR
    SecurityQualityOfService: Pointer; // Points to type SECURITY_QUALITY_OF_SERVICE
  end;
  PObjectAttributes = ^TObjectAttributes;

  TRtlDriveLetterCurDir = packed record
    Flags: Word;
    Length: Word;
    TimeStamp: LongWord;
    DosPath: TNtUnicodeString;
  end;

  TCurDir = packed record
    DosPath: TNtUnicodeString;
    Handle: THandle;
  end;

  TRtlUserProcessParameters = packed record
    MaximumLength: LongWord;
    Length: LongWord;
    Flags: LongWord;
    DebugFlags: LongWord;
    ConsoleHandle: THandle;
    ConsoleFlags: LongWord;
    StandardInput: THandle;
    StandardOutput: THandle;
    StandardError: THandle;
    CurrentDirectory: TCurDir;
    DllPath: TNtUnicodeString;
    ImagePathName: TNtUnicodeString;
    CommandLine: TNtUnicodeString;
    Environment: ^Word; // PWSTR
    StartingX: LongWord;
    StartingY: LongWord;
    CountX: LongWord;
    CountY: LongWord;
    CountCharsX: LongWord;
    CountCharsY: LongWord;
    FillAttribute: LongWord;
    WindowFlags: LongWord;
    ShowWindowFlags: LongWord;
    WindowTitle: TNtUnicodeString;
    DesktopInfo: TNtUnicodeString;
    ShellInfo: TNtUnicodeString;
    RuntimeData: TNtUnicodeString;
    CurrentDirectories: array[0..31] of TRtlDriveLetterCurDir;
  end;
  PRtlUserProcessParameters = ^TRtlUserProcessParameters;

  // a simple version of the PEB that contains the common stuff
  TSimplePEB = packed record
    InheritedAddressSpace: Byte;
    ReadImageFileExecOptions: Byte;
    BeingDebugged: Byte;
    SpareBool: Byte;
    Mutant: THandle;
    ImageBaseAddress: Pointer;
    Ldr: Pointer;
    ProcessParameters: PRtlUserProcessParameters;
    SubSystemData: Pointer;
    ProcessHeap: Pointer;
    FastPebLock: Pointer;
    FastPebLockRoutine: Pointer;
    FastPebUnlockRoutine: Pointer;
    EnvironmentUpdateCount: LongWord;
    KernelCallbackTable: Pointer;
    EventLogSection: Pointer;
    EventLog: Pointer;
    FreeList: Pointer;
    TlsExpansionCounter: LongWord;
    TlsBitmap: Pointer;
    TlsBitmapBits: array[0..1] of LongWord;
    ReadOnlySharedMemoryBase: Pointer;
    ReadOnlySharedMemoryHeap: Pointer;
    ReadOnlyStaticServerData: Pointer;
    AnsiCodePageData: Pointer;
    OemCodePageData: Pointer;
    UnicodeCaseTableData: Pointer;
    NumberOfProcessors: LongWord;
    NtGlobalFlag: LongWord;
    CriticalSectionTimeout: TLargeInteger;
    HeapSegmentReserve: LongWord;
    HeapSegmentCommit: LongWord;
    HeapDeCommitTotalFreeThreshold: LongWord;
    HeapDeCommitFreeBlockThreshold: LongWord;
    NumberOfHeaps: LongWord;
    MaximumNumberOfHeaps: LongWord;
    ProcessHeaps: Pointer;
    GdiSharedHandleTable: Pointer;
    ProcessStarterHelper: Pointer;
    GdiDCAttributeList: LongWord;
    LoaderLock: Pointer;
    OSMajorVersion: LongWord;
    OSMinorVersion: LongWord;
    OSBuildNumber: Word;
    OSCSDVersion: Word;
    OSPlatformId: LongWord;
    ImageSubSystem: LongWord;
    ImageSubSystemMajorVersion: LongWord;
    ImageSubSystemMinorVersion: LongWord;
    ImageProcessAffinityMask: LongWord;
    GdiHandleBuffer: array[0..$21] of LongWord;
    PostProcessInitRoutine: Pointer;
    TlsExpansionBitmap: Pointer;
    TlsExpansionBitmapBits: array[0..$19] of Word;
    SessionId: LongWord;
  end;
  PSimplePEB = ^TSimplePEB;

  PExceptionRegistrationRecord = ^TExceptionRegistrationRecord;
  TExceptionRegistrationRecord = packed record
    Next: PExceptionRegistrationRecord;
    Handler: Pointer; //PExceptionRoutine;
  end;

  PNTTIB = ^TNTTIB;
  TNTTIB = packed record
    ExceptionList: PExceptionRegistrationRecord;
    StackBase: Pointer;
    StackLimit: Pointer;
    SubSystemTib: Pointer;
    union1: record
              case Boolean of
                True: (FiberData: Pointer);
                False: (Version: DWord);
            end;
    ArbitraryUserPointer: Pointer;
    Self: PNTTIB;
  end;

  TClientID = packed record
    UniqueProcess: LongWord;
    UniqueThread: LongWord;
  end;
  PClientID = ^TClientID;

  TSimpleTEB = packed record
    NtTib: TNTTIB;
    EnvironmentPointer: Pointer;
    ClientId: TClientID;
  end;
  PSimpleTEB = ^TSimpleTEB;

const
  STATUS_SUCCESS = LongInt($00000000);
  STATUS_PENDING = LongInt($00000103);
  STATUS_END_OF_FILE = LongInt($C0000011);
  STATUS_ACCESS_DENIED = LongInt($C0000022);
  STATUS_OBJECT_TYPE_MISMATCH = LongInt($C0000024);
  STATUS_PIPE_BROKEN = LongInt($C000014B);
  STATUS_OBJECT_NAME_NOT_FOUND = LongInt($C0000034);
  STATUS_FILE_IS_A_DIRECTORY = LongInt($C00000BA);

  OBJ_INHERIT = $00000002;
  OBJ_PERMANENT = $00000010;

  FILE_DIRECTORY_FILE = $00000001;
  FILE_NON_DIRECTORY_FILE = $00000040;
  FILE_SYNCHRONOUS_IO_NONALERT = $00000020;
  FILE_OPEN_FOR_BACKUP_INTENT = $00004000;
  FILE_OPEN_REMOTE_INSTANCE = $00000400;

  STANDARD_RIGHTS_REQUIRED = $000F0000;

  FILE_SHARE_READ = $00000001;
  FILE_SHARE_WRITE = $00000002;
  FILE_SHARE_DELETE = $00000004;

  FILE_OPEN = $00000001;
  FILE_CREATE = $00000002;
  FILE_OVERWRITE_IF = $00000005;

  FILE_ATTRIBUTE_NORMAL = $00000080;

  NT_SYNCHRONIZE = $00100000; // normally called SYNCHRONIZE
  NT_DELETE = $00010000; // normally called DELETE
  GENERIC_READ = LongWord($80000000);
  GENERIC_WRITE = $40000000;
  GENERIC_ALL = $10000000;
  FILE_READ_ATTRIBUTES = $00000080;

  FileStandardInformation = 5;
  FileRenameInformation = 10;
  FileDispositionInformation = 13;
  FilePositionInformation = 14;
  FileAllocationInformation = 19;
  FileEndOfFileInformation = 20;

  { Share mode open }
  fmShareCompat    = $00000000;
  fmShareExclusive = $10;
  fmShareDenyWrite = $20;
  fmShareDenyRead  = $30;
  fmShareDenyNone  = $40;

type
  TIoStatusBlock = record
    Status: LongInt;
    Information: PLongWord;
  end;
  PIoStatusBlock = ^TIoStatusBlock;

  TFileDispositionInformation = record
    DeleteFile: LongBool;
  end;

  TFileStandardInformation = record
    AllocationSize: TLargeInteger;
    EndOfFile: TLargeInteger;
    NumberOfLinks: LongWord;
    DeletePending: ByteBool;
    Directory: ByteBool;
  end;

  TFilePositionInformation = record
    CurrentByteOffset: TLargeInteger;
  end;

  TFileEndOfFileInformation = record
    EndOfFile: TLargeInteger;
  end;

  TFileAllocationInformation = record
    AllocationSize: TLargeInteger;
  end;

  TFileRenameInformation = record
    ReplaceIfExists: ByteBool;
    RootDirectory: THandle;
    FileNameLength: LongWord;
    FileName: array[0..0] of WideChar;
  end;
  PFileRenameInformation = ^TFileRenameInformation;

threadvar
  errno: LongInt;

procedure Errno2InoutRes;
var
  r: Word;
begin
{$message warning 'Correctly implement Errno2InoutRes'}
  case errno of
    STATUS_OBJECT_NAME_NOT_FOUND:
      r := 2;
    STATUS_OBJECT_TYPE_MISMATCH:
      r := 3;
    STATUS_ACCESS_DENIED:
      r := 5;
    STATUS_END_OF_FILE:
      r := 100;
    else
      r := errno;
  end;
  errno := 0;
  InOutRes := r;
end;

function NtCreateFile(FileHandle: PHandle; DesiredAccess: LongWord;
  ObjectAttributes: PObjectAttributes; IoStatusBlock: PIOStatusBlock;
  AllocationSize: PLargeInteger; FileAttributes: LongWord;
  ShareAccess: LongWord; CreateDisposition: LongWord; CreateOptions: LongWord;
  EaBuffer: Pointer; EaLength: LongWord): LongInt; stdcall; external ntdll;
function  NtCreateDirectoryObject(DirectoryHandle: PHandle;
  DesiredAccess: LongWord; ObjectAttributes: PObjectAttributes): LongInt;
  stdcall; external ntdll;
function  NtOpenDirectoryObject(DirectoryHandle: PHandle;
  DesiredAccess: LongWord; ObjectAttributes: PObjectAttributes): LongInt;
  stdcall; external ntdll;
function NtClose(Handle: THandle): LongInt; stdcall; external ntdll;
function NtMakeTemporaryObject(Handle: THandle): LongInt; stdcall;
  external ntdll;
function  NtSetInformationFile(FileHandle: THandle;
  IoStatusBlock: PIoStatusBlock; FileInformation: Pointer;
  FileInformationLength: LongWord; FileInformationClass: LongWord):
  LongInt; stdcall; external ntdll;
function  NtQueryInformationFile(FileHandle: THandle;
  IoStatusBlock: PIoStatusBlock; FileInformation: Pointer;
  FileInformationLength: LongWord; FileInformationClass: LongWord):
  LongInt; stdcall; external ntdll;
function NtReadFile(FileHandle: THandle; Event: THandle; ApcRoutine: Pointer;
  ApcContext: Pointer; IoStatusBlock: PIOStatusBlock; Buffer: Pointer;
  Length: LongWord; ByteOffset: PLargeInteger; Key: PLongWord): LongInt;
  stdcall; external ntdll;
function NtWriteFile(FileHandle: THandle; Event: THandle;
  ApcRoutine: Pointer; ApcContext: Pointer; IoStatusBlock: PIOStatusBlock;
  Buffer: Pointer; Length: LongWord; ByteOffset: PLargeInteger; Key: PLongWord):
  LongInt; stdcall; external ntdll;
function NtWaitForSingleObject(Handle: THandle; Alertable: ByteBool;
  Timeout: PLargeInteger): LongInt; stdcall; external ntdll;
function NtDisplayString(aString: PNtUnicodeString): LongInt; stdcall; external ntdll;

{ TODO : move to platform specific file }
function NtCurrentTEB: PSimpleTEB; assembler;
asm
  movl %fs:(0x18),%eax
end;

(* from NDKUtils *)
procedure SysInitializeObjectAttributes(var aObjectAttr: TObjectAttributes;
  aName: PNtUnicodeString; aAttributes: LongWord; aRootDir: THandle;
  aSecurity: Pointer);
begin
  with aObjectAttr do begin
    Length := SizeOf(TObjectAttributes);
    RootDirectory := aRootDir;
    Attributes := aAttributes;
    ObjectName := aName;
    SecurityDescriptor := aSecurity;
    SecurityQualityOfService := Nil;
  end;
end;

procedure SysPCharToNtStr(var aNtStr: TNtUnicodeString; aText: PChar;
  aLen: LongWord);
var
  i: Integer;
begin
  if (aLen = 0) and (aText <> Nil) and (aText^ <> #0) then
    aLen := StrLen(aText);
  aNtStr.Length := aLen * SizeOf(WideChar);
  aNtStr.MaximumLength := aNtStr.Length;
  aNtStr.Buffer := GetMem(aNtStr.Length);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  for i := 0 to aLen-1 do
    aNtStr.Buffer[i] := aText[i];
end;

procedure SysPWideCharToNtStr(var aNtStr: TNtUnicodeString; aText: PWideChar;
  aLen: LongWord);
var
  i: Integer;
begin
  if (aLen = 0) and (aText <> Nil) and (aText^ <> #0) then
    aLen := Length(aText);
  aNtStr.Length := aLen * SizeOf(WideChar);
  aNtStr.MaximumLength := aNtStr.Length;
  aNtStr.Buffer := GetMem(aNtStr.Length);
  Move(aText[0],aNtStr.Buffer[0],aLen);
end;

procedure SysUnicodeStringToNtStr(var aNtStr: TNtUnicodeString; const s: UnicodeString);
var
  i: Integer;
begin
  aNtStr.Length := Length(s) * SizeOf(WideChar);
  aNtStr.MaximumLength := aNtStr.Length;
  aNtStr.Buffer := GetMem(aNtStr.Length);
  if aNtStr.Length<>0 then
    Move(s[1],aNtStr.Buffer[0],aNtStr.Length);
end;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  for i := 0 to aLen do
    aNtStr.Buffer[i] := aText[i];
end;

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
procedure SysFreeNtStr(var aNtStr: TNtUnicodeString);
begin
  if aNtStr.Buffer <> Nil then begin
    FreeMem(aNtStr.Buffer);
    aNtStr.Buffer := Nil;
  end;
  aNtStr.Length := 0;
  aNtStr.MaximumLength := 0;
end;
