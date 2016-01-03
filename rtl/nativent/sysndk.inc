// These datatypes are used in system.pas and ndk.pas

const
{$ifdef kmode}
  ntdll = 'ntoskrnl.exe';
{$else}
  ntdll = 'ntdll.dll';
{$endif}

type
  //
  // some basic types
  //
  HANDLE = THandle;
  PVOID = Pointer;
  LONG = LongInt;
  ULONG = LongWord;


  NTSTATUS = LongInt;

  UNICODE_STRING = packed record
    Length: Word;        // used characters in buffer
    MaximumLength: Word; // maximum characters in buffer
    Buffer: PWideChar;
  end;
  PUNICODE_STRING = ^UNICODE_STRING;
  // alias to differ from TUnicodeString
  TNtUnicodeString = UNICODE_STRING;
  PNtUnicodeString = ^TNtUnicodeString;

  // using Int64 is an alternative (QWord might have unintended side effects)
  LARGE_INTEGER = packed record
    case Boolean of
      True:(LowPart: LongWord;
            HighPart: LongInt);
      False:(QuadPart: Int64);
  end;
  PLARGE_INTEGER = ^LARGE_INTEGER;
  TLargeInteger = LARGE_INTEGER;
  PLargeInteger = ^TLargeInteger;


//
// Object Attributes structure
//
  POBJECT_ATTRIBUTES = ^OBJECT_ATTRIBUTES;
  _OBJECT_ATTRIBUTES = record
    Length: ULONG;
    RootDirectory: HANDLE;
    ObjectName: PUNICODE_STRING;
    Attributes: ULONG;
    SecurityDescriptor: PVOID;       // Points to type SECURITY_DESCRIPTOR
    SecurityQualityOfService: PVOID; // Points to type SECURITY_QUALITY_OF_SERVICE
  end;
  OBJECT_ATTRIBUTES = _OBJECT_ATTRIBUTES;
  TObjectAttributes = OBJECT_ATTRIBUTES;
  PObjectAttributes = POBJECT_ATTRIBUTES;

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

  TSimplePEB = packed record
    InheritedAddressSpace: Byte;
    ReadImageFileExecOptions: Byte;
    BeingDebugged: Byte;
//#if (NTDDI_VERSION >= NTDDI_WS03)
//    struct
    {
        UCHAR ImageUsesLargePages:1;
    #if (NTDDI_VERSION >= NTDDI_LONGHORN)
        UCHAR IsProtectedProcess:1;
        UCHAR IsLegacyProcess:1;
        UCHAR SpareBits:5;
    #else
        UCHAR SpareBits:7;
    #endif
    }//;
//#else
    SpareBool: Byte;
//#endif
    Mutant: THandle;
    ImageBaseAddress: Pointer;
    Ldr: Pointer; // PPEB_LDR_DATA
    ProcessParameters: PRtlUserProcessParameters;
    SubSystemData: Pointer;
    ProcessHeap: Pointer;
//#if (NTDDI_VERSION >= NTDDI_LONGHORN)
(*    struct _RTL_CRITICAL_SECTION *FastPebLock;
    PVOID AltThunkSListPtr;
    PVOID IFEOKey;
    ULONG Spare;
    union
    {
        PVOID* KernelCallbackTable;
        PVOID UserSharedInfoPtr;
    };
    ULONG SystemReserved[1];
    ULONG SpareUlong;*)
//#else
    FastPebLock: Pointer;
    FastPebLockRoutine: Pointer; // PPEBLOCKROUTINE
    FastPebUnlockRoutine: Pointer; // PPEBLOCKROUTINE
    EnvironmentUpdateCount: LongWord;
    KernelCallbackTable: Pointer; // PVOID*
    EventLogSection: Pointer;
    EventLog: Pointer;
//#endif
    FreeList: Pointer; // PPEB_FREE_BLOCK
    TlsExpansionCounter: LongWord;
    TlsBitmap: Pointer;
    TlsBitmapBits: array[0..1] of LongWord; //TlsBitmapBits[0x2]
    ReadOnlySharedMemoryBase: Pointer;
    ReadOnlySharedMemoryHeap: Pointer;
    ReadOnlyStaticServerData: Pointer; //PVOID*
    AnsiCodePageData: Pointer;
    OemCodePageData: Pointer;
    UnicodeCaseTableData: Pointer;
    NumberOfProcessors: LongWord;
    NtGlobalFlag: LongWord;
    CriticalSectionTimeout: Int64; // LARGE_INTEGER
    HeapSegmentReserve: LongWord;
    HeapSegmentCommit: LongWord;
    HeapDeCommitTotalFreeThreshold: LongWord;
    HeapDeCommitFreeBlockThreshold: LongWord;
    NumberOfHeaps: LongWord;
    MaximumNumberOfHeaps: LongWord;
    ProcessHeaps: Pointer; // PVOID*
    GdiSharedHandleTable: Pointer;
    ProcessStarterHelper: Pointer;
    GdiDCAttributeList: LongWord;
//#if (NTDDI_VERSION >= NTDDI_LONGHORN)
//    struct _RTL_CRITICAL_SECTION *LoaderLock;
//#else
    LoaderLock: Pointer;
//#endif
    OSMajorVersion: LongWord;
    OSMinorVersion: LongWord;
    OSBuildNumber: Word; // USHORT
    OSCSDVersion: Word; // USHORT
    OSPlatformId: LongWord;
    ImageSubSystem: LongWord;
    ImageSubSystemMajorVersion: LongWord;
    ImageSubSystemMinorVersion: LongWord;
    ImageProcessAffinityMask: LongWord;
    GdiHandleBuffer: array[0..$21] of LongWord; // GdiHandleBuffer[0x22]
    PostProcessInitRoutine: Pointer; //PPOST_PROCESS_INIT_ROUTINE
    TlsExpansionBitmap: Pointer; //struct _RTL_BITMAP *TlsExpansionBitmap
    TlsExpansionBitmapBits: array[0..$19] of Word; //TlsExpansionBitmapBits[0x20]
    SessionId: LongWord;
{#if (NTDDI_VERSION >= NTDDI_WINXP)
    ULARGE_INTEGER AppCompatFlags;
    ULARGE_INTEGER AppCompatFlagsUser;
    PVOID pShimData;
    PVOID AppCompatInfo;
    UNICODE_STRING CSDVersion;
    struct _ACTIVATION_CONTEXT_DATA *ActivationContextData;
    struct _ASSEMBLY_STORAGE_MAP *ProcessAssemblyStorageMap;
    struct _ACTIVATION_CONTEXT_DATA *SystemDefaultActivationContextData;
    struct _ASSEMBLY_STORAGE_MAP *SystemAssemblyStorageMap;
    ULONG MinimumStackCommit;
#endif
#if (NTDDI_VERSION >= NTDDI_WS03)
    PVOID *FlsCallback;
    LIST_ENTRY FlsListHead;
    struct _RTL_BITMAP *FlsBitmap;
    ULONG FlsBitmapBits[4];
    ULONG FlsHighIndex;
#endif
#if (NTDDI_VERSION >= NTDDI_LONGHORN)
    PVOID WerRegistrationData;
    PVOID WerShipAssertPtr;
#endif}
  end;
  PPEB = ^TSimplePEB;

function NtDisplayString(aString: PNtUnicodeString): NTSTATUS; stdcall; external ntdll;

