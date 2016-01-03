{$mode delphi}
UNIT RAPI;

// Created and donated to the public domain 2001
//   by Scott Crossen, scottc@hotmail.com
// Modified 2003 by Terence Goguin, terencegoggin@hotmail.com
// Modified 2005 by Octavio Hernandez, dotnet@danysoft.com

INTERFACE

<<<<<<< HEAD
<<<<<<< HEAD
uses Windows, RAPITypes;
=======
uses Windows;
>>>>>>> graemeg/fixes_2_2
=======
uses Windows;
>>>>>>> origin/fixes_2_2

const
  FAF_ATTRIBUTES = $00000001;
  FAF_CREATION_TIME = $00000002;
  FAF_LASTACCESS_TIME = $00000004;
  FAF_LASTWRITE_TIME = $00000008;
  FAF_SIZE_HIGH = $00000010;
  FAF_SIZE_LOW = $00000020;
  FAF_OID = $00000040;
  FAF_NAME = $00000080;
  FAF_FLAG_COUNT = 8;
  FAF_ATTRIB_CHILDREN = $00001000;
  FAF_ATTRIB_NO_HIDDEN = $00002000;
  FAF_FOLDERS_ONLY = $00004000;
  FAF_NO_HIDDEN_SYS_ROMMODULES = $00008000;

  FAD_OID = $1;
  FAD_FLAGS = $2;
  FAD_NAME = $4;
  FAD_TYPE = $8;
  FAD_NUM_RECORDS = $10;
  FAD_NUM_SORT_ORDER = $20;
  FAD_SIZE = $40;
  FAD_LAST_MODIFIED = $80;
  FAD_SORT_SPECS = $100;
  FAD_FLAG_COUNT = $9;

  CeDB_SORT_DESCENDING = $00000001;
  CeDB_SORT_CASEINSENSITIVE = $00000002;
  CeDB_SORT_UNKNOWNFIRST = $00000004;
  CeDB_SORT_GENERICORDER = $00000008;

  CeDB_MAXDBASENAMELEN = 32;
  CeDB_MAXSORTORDER = 4;

  CeDB_VALIDNAME = $0001;
  CeDB_VALIDTYPE = $0002;
  CeDB_VALIDSORTSPEC = $0004;
  CeDB_VALIDMODTIME = $0008;
  OBJTYPE_INVALID = 0;
  OBJTYPE_FILE = 1;
  OBJTYPE_DIRECTORY = 2;
  OBJTYPE_DATABASE = 3;
  OBJTYPE_RECORD = 4;

  CeDB_AUTOINCREMENT = $00000001;

  CeDB_SEEK_CeOID = $00000001;
  CeDB_SEEK_BEGINNING = $00000002;
  CeDB_SEEK_END = $00000004;
  CeDB_SEEK_CURRENT = $00000008;
  CeDB_SEEK_VALUESMALLER = $00000010;
  CeDB_SEEK_VALUEFIRSTEQUAL = $00000020;
  CeDB_SEEK_VALUEGREATER = $00000040;
  CeDB_SEEK_VALUENEXTEQUAL = $00000080;
  CeVT_I2 = 2;
  CeVT_UI2 = 18;
  CeVT_I4 = 3;
  CeVT_UI4 = 19;
  CeVT_FILETIME = 64;
  CeVT_LPWSTR = 31;
  CeVT_BLOB = 65;
  CeDB_PROPNOTFOUND = $0100;
  CeDB_PROPDELETE = $0200;
  CeDB_MAXDATABLOCKSIZE = 4092;
  CeDB_MAXPROPDATASIZE =(CeDB_MAXDATABLOCKSIZE*16);
  CeDB_MAXRECORDSIZE =(128*1024);

  CeDB_ALLOWREALLOC = $00000001;

  SYSMEM_CHANGED = 0;
  SYSMEM_MUSTREBOOT = 1;
  SYSMEM_REBOOTPENDING = 2;
  SYSMEM_FAILED = 3;
  AC_LINE_OFFLINE = $00;
  AC_LINE_ONLINE = $01;
  AC_LINE_BACKUP_POWER = $02;
  AC_LINE_UNKNOWN = $FF;

  BATTERY_FLAG_HIGH = $01;
  BATTERY_FLAG_LOW = $02;
  BATTERY_FLAG_CRITICAL = $04;
  BATTERY_FLAG_CHARGING = $08;
  BATTERY_FLAG_NO_BATTERY = $80;
  BATTERY_FLAG_UNKNOWN = $FF;

  BATTERY_PERCENTAGE_UNKNOWN = $FF;

  BATTERY_LIFE_UNKNOWN = $FFFFFFFF;

type
  //
  // The Pegasus WIN32_FIND_DATA structure differs from the
  // Windows WIN32_FIND_DATA stucture so we copy the Pegasus
  // definition to here so that both sides match.
  //
  TCe_Find_Data = record
    dwFileAttributes: DWORD;
    ftCreationTime: TFileTime;
    ftLastAccessTime: TFileTime;
    ftLastWriteTime: TFileTime;
    nFileSizeHigh: DWORD;
    nFileSizeLow: DWORD;
    dwOID: DWord;
    cFileName: array[0..MAX_PATH - 1] of WideChar;
  end;
  PCe_Find_Data = ^TCe_Find_Data;
  TCe_Find_Data_array = array[0..MaxInt div sizeof(TCe_Find_Data)-1] of TCe_Find_Data;
  PCe_Find_Data_array = ^TCe_Find_Data_array;

  TStore_Information = record
    dwStoreSize: DWORD;
    dwFreeSize: DWORD;
  end;
  PStore_Information = ^TStore_Information;

  CePROPID = DWORD;
  PCePROPID = ^CePROPID;
  TCe_PropID_array = array[0..MaxInt div sizeof(CePROPID)-1] of CePROPID;
  PCe_PropID_array = ^TCe_PropID_array;

  CeOID = DWORD;
  PCeOID = ^CeOID;

  TCeFileInfo = record
    dwAttributes: DWORD;
    oidParent: CeOID;
    szFileName: array [0..MAX_PATH-1] of WCHAR;
    ftLastChanged: TFileTime;
    dwLength: DWORD;
  end;

  TCeDirInfo = record
    dwAttributes: DWORD;
    oidParent: CeOID;
    szDirName: array [0..MAX_PATH-1] of WCHAR;
  end;

  TCeRecordInfo = record
    oidParent: CeOID;
  end;

  TSortOrderSpec= record
    propid: CePROPID;
    dwFlags: DWORD;
  end;

  TCeDBaseInfo = record
    dwFlags: DWORD;
    szDbaseName: array [0..CeDB_MAXDBASENAMELEN-1] of WCHAR;
    dwDbaseType: DWORD;
    wNumRecords: WORD;
    wNumSortOrder: WORD;
    dwSize: DWORD;
    ftLastModified: TFileTime;
    rgSortSpecs: array [0..CeDB_MAXSORTORDER-1] of TSortOrderSpec;
  end;

  TCeDB_File_Data = record
    OidDb: CeOID;
    DbInfo: TCeDBaseInfo;
  end;
  PCeDB_File_Data= ^TCeDB_File_Data;

  TCeDB_File_Data_Array = array [0..MaxInt div sizeof(TCeDB_File_Data)-1] of TCeDB_File_Data;
  PCeDB_File_Data_Array = ^TCeDB_File_Data_Array;

  TCeOIdInfo = record
    wObjType: WORD;
    wPad: WORD;
    case Integer of
    0: (infFile: TCeFileInfo);
    1: (infDirectory: TCeDIRINFO);
    2: (infDatabase: TCeDBASEINFO);
    3: (infRecord: TCeRECORDINFO);
  end;
  PCeOIDInfo = ^TCeOIDInfo;

  TCeOIContainerStruct = record
    OID: CeOID;
    OIDInfo: TCeOIDInfo;
  end;
  PCeOIContainerStruct = ^TCeOIContainerStruct;

  TCeBlob = record
    dwCount: DWORD;
    lpb: DWORD;
  end;

  TCeValUnion = record
    iVal: SHORT;
    uiVal: WORD;
    lVal: LongInt;
    ulVal: ULONG;
    fletime: TFileTime;
    lpwstr: LPWSTR;
    blob: TCeBlob;
  end;

  TCePROPVAL = record
    propid: CePROPID;
    wLenData: Word;
    wFlags: WORD;
    val: TCeVALUNION;
  end;

  TCeOSVersionInfo= record
    wOSVersionInfoSize: DWORD;
    dwMajorVersion: DWORD;
    dwMinorVersion: DWORD;
    dwBuildNumber: DWORD;
    dwPlatformId: DWORD;
    szCSDVersion: array[0..128-1] of WCHAR;
  end;
  PCeOSVersionInfo = ^TCeOSVersionInfo;

  TSystem_Power_Status_Ex = record
    ACLineStatus: BYTE;
    BatteryFlag: BYTE;
    BatteryLifePercent: BYTE;
    Reserved1: BYTE;
    BatteryLifeTime: BYTE;
    BatteryFullLifeTime: BYTE;
    Reserved2: BYTE;
    BackupBatteryFlag: BYTE;
    BackupBatteryLifePercent: BYTE;
    Reserved3: BYTE;
    BackupBatteryLifeTime: DWORD;
    BackupBatteryFullLifeTime: DWORD;
  end;
  PSystem_Power_Status_Ex = ^TSystem_Power_Status_Ex;
  TSystem_Power_Status_ExArray = array [0..MaxInt div Sizeof(TSystem_Power_Status_Ex) -1] of TSystem_Power_Status_Ex;
  PSystem_Power_Status_ExArray = ^TSystem_Power_Status_ExArray;

  TRapiInit = record
    cbSize: DWORD;
    heRapiInit: THandle;
    hrRapiInit: HResult;
  end;

  TCeRapiInit = function : LongInt stdcall;
  TCeRapiInitEx = function(var RInit: TRapiInit) : LongInt stdcall;
  TCeCreateDatabase = function(lpszName: LPWSTR; dwDbaseType: DWORD; wNumSortOrder: WORD;
    var rgSortSpecs: TSortOrderSpec): CeOID stdcall;
  TCeDeleteDatabase = function(oidDBase: CeOId): BOOL stdcall;
  TCeDeleteRecord = function(hDatabase: THandle; oidRecord: CeOID): BOOL stdcall;
  TCeFindFirstDatabase = function(dwDbaseType: DWORD): THandle stdcall;
  TCeFindNextDatabase = function(hEnum: THandle): CeOID stdcall;
  TCeOidGetInfo = function(oid: CeOID; var poidInfo: TCeOIDINFO): BOOL stdcall;
  TCeOpenDatabase = function(var poid: CeOID; lpszName: LPWSTR; propid: CePROPID;
    dwFlags: DWORD; hwndNotify: HWND): THandle stdcall;
  TCeReadRecordProps = function(hDbase: THandle; dwFlags: DWORD;var cPropID : WORD;
    var rgPropID : PCe_PropID_array;Buffer: Pointer; var cbBuffer: DWORD): CeOID stdcall;
  TCeSeekDatabase = function(hDatabase: THandle; dwSeekType: DWORD; dwValue: LongInt;
    dwIndex: PDWORD): CeOID stdcall;
  TCeSetDatabaseInfo = function(oidDbase: CeOID; var NewInfo: TCeDBaseInfo): BOOL stdcall;
  TCeWriteRecordProps = function(hDbase: THandle; oidRecord: CeOID; cPropID: WORD; PropVal: TCePROPVAL): CeOID stdcall;
  TCeFindFirstFile = function(lpFileName: LPCWSTR; lpFindFileData: PCe_FIND_DATA): THandle stdcall;
  TCeFindNextFile = function(hFindFile: THandle; lpFindFileData: PCe_FIND_DATA): BOOL stdcall;
  TCeFindClose = function(hFindFile: THandle): BOOL stdcall;
  TCeGetFileAttributes = function(lpFileName: LPCWSTR): DWORD stdcall;
  TCeSetFileAttributes = function(FileName: LPCWSTR; dwFileAttributes: DWORD): BOOL stdcall;
  TCeCreateFile = function(lpFileName: LPCWSTR; dwDesiredAccess: DWORD; dwShareMode: DWORD;
    lpSecurityAttributes: PSecurityAttributes; dwCreationDistribution: DWORD;
    dwFlagsAndAttributes: DWORD;hTemplateFile: THandle): THandle stdcall;
  TCeReadFile = function(hFile: THandle; lpBuffer: Pointer; nNumberOfBytesToRead: DWORD;
    var NumberOfBytesRead : DWORD; Overlapped: POVERLAPPED): BOOL stdcall;
  TCeWriteFile = function(hFile: THandle; Buffer: Pointer; NumberOfBytesToWrite: DWORD;
    var NumberOfBytesWritten: DWORD; Overlapped: POVERLAPPED): BOOL stdcall;
  TCeCloseHandle = function(hObject: THandle): BOOL stdcall;
  TCeFindAllDatabases = function(dwDbaseType: DWORD; wFlags: WORD; var cFindData: DWORD;
    var ppFindData: PCeDB_File_Data_Array): BOOL stdcall;
  TCeGetLastError = function : DWORD stdcall;
  TGetRapiError = function : LongInt stdcall;
  TCeSetFilePointer = function(hFile: THandle; DistanceToMove: LongInt; DistanceToMoveHigh: PULONG;
    dwMoveMethod: DWORD): DWORD stdcall;
  TCeSetEndOfFile = function(hFile: THandle): BOOL stdcall;
  TCeCreateDirectory = function(lpPathName: LPCWSTR; lpSecurityAttributes: PSecurityAttributes): BOOL stdcall;
  TCeRemoveDirectory = function(PathName: LPCWSTR): BOOL stdcall;
  TCeCreateProcess = function(lpApplicationName: LPCWSTR;lpCommandLine: LPCWSTR; lpProcessAttributes: PSecurityAttributes;
    lpThreadAttributes: PSecurityAttributes;bInheritHandles : BOOL;dwCreateFlags : DWORD; lpEnvironment: Pointer; lpCurrentDirectory : LPWSTR;
    lpStartupInfo: PSTARTUPINFO; lpProcessInformation: PProcessInformation): BOOL stdcall;
  TCeMoveFile = function(lpExistingFileName: LPCWSTR; lpNewFileName: LPCWSTR): BOOL stdcall;
  TCeCopyFile = function(lpExistingFileName: LPCWSTR; lpNewFileName: LPCWSTR; bFailIfExists: BOOL): BOOL stdcall;
  TCeDeleteFile = function(lpFileName: LPCWSTR): BOOL stdcall;
  TCeGetFileSize = function(hFile: THandle; lpFileSizeHigh: PDWORD): DWORD stdcall;
  TCeRegOpenKeyEx = function(hKey: HKEY; SubKey: LPCWSTR; Reserved: DWORD; samDesired: REGSAM;
    Result: PHKEY): LongInt stdcall;
  TCeRegEnumKeyEx = function(hKey: HKEY; dwIndex: DWORD; KeyName: LPWSTR; chName: PDWORD;
    reserved: PDWORD; szClass: LPWSTR; cchClass: PDWORD; ftLastWrite: PFILETIME): LongInt stdcall;
  TCeRegCreateKeyEx = function(hKey: HKEY; lpSzSubKey: LPCWSTR; dwReserved: DWORD;
    lpszClass: LPWSTR; dwOption: DWORD; samDesired: REGSAM; lpSecurityAttributes: PSecurityAttributes;
    phkResult: PHKEY; lpdwDisposition: PDWORD): LongInt stdcall;
  TCeRegCloseKey = function(hKey: HKEY): LongInt stdcall;
  TCeRegDeleteKey = function(hKey: HKEY; lpszSubKey: LPCWSTR): LongInt stdcall;
  TCeRegEnumValue = function(hKey: HKEY; dwIndex: DWORD; lpszName: LPWSTR; lpcchName: PDWORD;
    lpReserved: PDWORD; lpszClass: PDWORD; lpcchClass: PBYTE; lpftLastWrite: PDWORD): LongInt stdcall;
  TCeRegDeleteValue = function(hKey: HKEY; lpszValueName: LPCWSTR): LongInt stdcall;
  TCeRegQueryInfoKey = function(hKey: HKEY; ClassName: LPWSTR; cchClass: PDWORD; Reserved: PDWORD;
    cSubKeys: PDWORD; cchMaxSubKeyLen: PDWORD; cchMaxClassLen: PDWORD; cValues: PDWORD;
    cchMaxValueNameLen: PDWORD; cbMaxValueData: PDWORD; cbSecurityDescriptor: PDWORD;
    LastWriteTime: PFILETIME): LongInt stdcall;
  TCeRegQueryValueEx = function(hKey: HKEY; ValueName: LPCWSTR; Reserved: PDWORD; pType: PDWORD;
  pData: PBYTE; cbData: PDWORD): LongInt stdcall;
  TCeRegSetValueEx = function(hKey: HKEY; ValueName: LPCWSTR; reserved: DWORD;
    dwType: DWORD; pData: PBYTE; cbData: DWORD): LongInt stdcall;
  TCeGetStoreInformation= function(lpsi: PSTORE_INFORMATION): BOOL stdcall;
  TCeGetSystemMetrics = function(nIndex: Integer): Integer stdcall;
  TCeGetDesktopDeviceCaps= function(nIndedx: Integer): LongInt stdcall;
  TCeGetSystemInfo = procedure(lpSystemInfo: PSystemInfo)stdcall;
  TCeSHCreateShortcut = function(ShortCut: LPWSTR; Target: LPWSTR): DWORD stdcall;
  TCeSHGetShortcutTarget= function(ShortCut: LPWSTR; Target: LPWSTR; cbMax: integer): BOOL stdcall;
  TCeCheckPassword = function(lpszPassword: LPWSTR): BOOL stdcall;
  TCeGetFileTime = function(hFile: THandle; lpCreationTime: PFILETIME;
    lpLastAccessTime: PFILETIME; lpLastWriteTime: PFILETIME): BOOL stdcall;
  TCeSetFileTime = function(hFile: THandle; CreationTime: PFILETIME;
    LastAccessTime: PFILETIME; lastWriteTime: PFILETIME): BOOL stdcall;
  TCeGetVersionEx = function(lpVersionInfo: PCeOSVERSIONINFO): BOOL stdcall;
  TCeGetWindow = function(hWnd: HWND; uCmd: UINT): HWND stdcall;
  TCeGetWindowLong = function(hWnd: HWND; nIndex: integer): LongInt stdcall;
  TCeGetWindowText = function(hWnd: HWND; lpString: LPWSTR; nMaxCount: integer): Integer stdcall;
  TCeGetClassName = function(hWnd: HWND; lpClassName: LPWSTR; nMaxCount: integer): Integer stdcall;
  TCeGlobalMemoryStatus = procedure(lpmst: PMemoryStatus)stdcall;
  TCeGetSystemPowerStatusEx= function(pStatus: PSYSTEM_POWER_STATUS_EX; fUpdate: BOOL): BOOL stdcall;

  //added 10/16/2000 - Terence Goggin; terencegoggin@hotmail.com
  TDesktopToDevice = function(DesktopLocation, TableList: String; Sync: BOOL; Overwrite: Integer; DeviceLocation: String): Longint stdcall;
  //added 01/19/2003 - Octavio Hernandez; dotnet@danysoft.com
  TDeviceToDesktop = function(DesktopLocation, TableList: String; Sync: BOOL; Overwrite: Integer; DeviceLocation: String): Longint stdcall;

<<<<<<< HEAD
<<<<<<< HEAD
  TCeRapiUninit = function : LongInt stdcall;
  TCeFindAllFiles = function(Path: PWideChar; Attr: DWORD; var Count: DWord;
    var FindData: PCe_Find_Data_array): BOOL stdcall;
  TRapiFreeBuffer = procedure(p: Pointer) stdcall;
  TCeRapiInvoke = function (pDllPath: LPCWSTR; pFunctionName: LPCWSTR; cbInput: DWord; pInput: PByte;
    pcbOutput: PDWord; ppOutput: PPByte; ppIRAPIStream: PIRAPIStream; dwReserved: DWord): HResult; stdcall;

function CeRapiInit: LongInt;
function CeRapiInitEx(var RInit: TRapiInit) : LongInt;
function CeRapiUninit: LongInt;
=======
=======
>>>>>>> origin/fixes_2_2
  TCeRapiUnInit = function : LongInt stdcall;
  TCeFindAllFiles = function(Path: PWideChar; Attr: DWORD; var Count: DWord;
    var FindData: PCe_Find_Data_array): BOOL stdcall;
  TRapiFreeBuffer = procedure(p: Pointer) stdcall;

function CeRapiInit: LongInt;
function CeRapiInitEx(var RInit: TRapiInit) : LongInt;
function CeRapiUnInit: LongInt;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function CeFindAllFiles(Path: PWideChar; Attr: DWORD;
  var Count: DWord; var FindData: PCe_Find_Data_array): BOOL;
procedure RapiFreeBuffer(p: Pointer);
function CeCreateDatabase(lpszName: LPWSTR; dwDbaseType: DWORD; wNumSortOrder: WORD;
  var rgSortSpecs: TSortOrderSpec): CeOID;
function CeDeleteDatabase(oidDBase: CeOId): BOOL;
function CeDeleteRecord(hDatabase: THandle; oidRecord: CeOID): BOOL;
function CeFindFirstDatabase(dwDbaseType: DWORD): THandle;
function CeFindNextDatabase(hEnum: THandle): CeOID;
function CeOidGetInfo(oid: CeOID; var poidInfo: TCeOIDINFO): BOOL;
function CeOpenDatabase(var poid: CeOID; lpszName: LPWSTR; propid: CePROPID;
  dwFlags: DWORD; hwndNotify: HWND): THandle;
function CeReadRecordProps(hDbase: THandle; dwFlags: DWORD;var cPropID : WORD;
  var rgPropID : PCe_PropID_array; Buffer: Pointer; var cbBuffer: DWORD): CeOID;
function CeSeekDatabase(hDatabase: THandle; dwSeekType: DWORD; dwValue: LongInt;
  dwIndex: PDWORD): CeOID;
function CeSetDatabaseInfo(oidDbase: CeOID; var NewInfo: TCeDBaseInfo): BOOL;
function CeWriteRecordProps(hDbase: THandle; oidRecord: CeOID; cPropID: WORD; PropVal: TCePROPVAL): CeOID;
function CeFindFirstFile(lpFileName: LPCWSTR; lpFindFileData: PCe_FIND_DATA): THandle;
function CeFindNextFile(hFindFile: THandle; lpFindFileData: PCe_FIND_DATA): BOOL;
function CeFindClose(hFindFile: THandle): BOOL;
function CeGetFileAttributes(lpFileName: LPCWSTR): DWORD;
function CeSetFileAttributes(FileName: LPCWSTR; dwFileAttributes: DWORD): BOOL;
function CeCreateFile(lpFileName: LPCWSTR; dwDesiredAccess: DWORD; dwShareMode: DWORD;
  lpSecurityAttributes: PSecurityAttributes; dwCreationDistribution: DWORD;
  dwFlagsAndAttributes: DWORD;hTemplateFile: THandle): THandle;
function CeReadFile(hFile: THandle; lpBuffer: Pointer; nNumberOfBytesToRead: DWORD;
  var NumberOfBytesRead : DWORD; Overlapped: POVERLAPPED): BOOL;
function CeWriteFile(hFile: THandle; Buffer: Pointer; NumberOfBytesToWrite: DWORD;
  var NumberOfBytesWritten: DWORD; Overlapped: POVERLAPPED): BOOL;
function CeCloseHandle(hObject: THandle): BOOL;
function CeFindAllDatabases(dwDbaseType: DWORD; wFlags: WORD; var cFindData: DWORD;var ppFindData: PCeDB_File_Data_Array): BOOL;
function CeGetLastError : DWORD;
function GetRapiError : LongInt;
function CeSetFilePointer(hFile: THandle; DistanceToMove: LongInt; DistanceToMoveHigh: PULONG;
  dwMoveMethod: DWORD): DWORD;
function CeSetEndOfFile(hFile: THandle): BOOL;
function CeCreateDirectory(lpPathName: LPCWSTR; lpSecurityAttributes: PSecurityAttributes): BOOL;
function CeRemoveDirectory(PathName: LPCWSTR): BOOL;
function CeCreateProcess(lpApplicationName: LPCWSTR;lpCommandLine: LPCWSTR; lpProcessAttributes: PSecurityAttributes;
  lpThreadAttributes: PSecurityAttributes;bInheritHandles : BOOL;dwCreateFlags : DWORD; lpEnvironment: Pointer; lpCurrentDirectory : LPWSTR;
  lpStartupInfo: PSTARTUPINFO; lpProcessInformation: PProcessInformation): BOOL;
function CeMoveFile(lpExistingFileName: LPCWSTR; lpNewFileName: LPCWSTR): BOOL;
function CeCopyFile(lpExistingFileName: LPCWSTR; lpNewFileName: LPCWSTR; bFailIfExists: BOOL): BOOL;
function CeDeleteFile(lpFileName: LPCWSTR): BOOL;
function CeGetFileSize(hFile: THandle; lpFileSizeHigh: PDWORD): DWORD;
function CeRegOpenKeyEx(hKey: HKEY; SubKey: LPCWSTR; Reserved: DWORD; samDesired: REGSAM;
  pResult: PHKEY): LongInt;
function CeRegEnumKeyEx(hKey: HKEY; dwIndex: DWORD; KeyName: LPWSTR; chName: PDWORD;
  reserved: PDWORD; szClass: LPWSTR; cchClass: PDWORD; ftLastWrite: PFILETIME): LongInt;
function CeRegCreateKeyEx(hKey: HKEY; lpSzSubKey: LPCWSTR; dwReserved: DWORD;
  lpszClass: LPWSTR; dwOption: DWORD; samDesired: REGSAM; lpSecurityAttributes: PSecurityAttributes;
  phkResult: PHKEY; lpdwDisposition: PDWORD): LongInt;
function CeRegCloseKey(hKey: HKEY): LongInt;
function CeRegDeleteKey(hKey: HKEY; lpszSubKey: LPCWSTR): LongInt;
function CeRegEnumValue(hKey: HKEY; dwIndex: DWORD; lpszName: LPWSTR; lpcchName: PDWORD;
  lpReserved: PDWORD; lpszClass: PDWORD; lpcchClass: PBYTE; lpftLastWrite: PDWORD): LongInt;
function CeRegDeleteValue(hKey: HKEY; lpszValueName: LPCWSTR): LongInt;
function CeRegQueryInfoKey(hKey: HKEY; ClassName: LPWSTR; cchClass: PDWORD; Reserved: PDWORD;
  cSubKeys: PDWORD; cchMaxSubKeyLen: PDWORD; cchMaxClassLen: PDWORD; cValues: PDWORD;
  cchMaxValueNameLen: PDWORD; cbMaxValueData: PDWORD; cbSecurityDescriptor: PDWORD;
  LastWriteTime: PFILETIME): LongInt;
function CeRegQueryValueEx(hKey: HKEY; ValueName: LPCWSTR; Reserved: PDWORD; pType: PDWORD;
  pData: PBYTE; cbData: PDWORD): LongInt;
function CeRegSetValueEx(hKey: HKEY; ValueName: LPCWSTR; reserved: DWORD;
  dwType: DWORD; pData: PBYTE; cbData: DWORD): LongInt;
function CeGetStoreInformation(lpsi: PSTORE_INFORMATION): BOOL;
function CeGetSystemMetrics(nIndex: Integer): Integer;
function CeGetDesktopDeviceCaps(nIndedx: Integer): LongInt;
procedure CeGetSystemInfo(lpSystemInfo: PSystemInfo);
function CeSHCreateShortcut(ShortCut: LPWSTR; Target: LPWSTR): DWORD;
function CeSHGetShortcutTarget(ShortCut: LPWSTR; Target: LPWSTR; cbMax: integer): BOOL;
function CeCheckPassword(lpszPassword: LPWSTR): BOOL;
function CeGetFileTime(hFile: THandle; lpCreationTime: PFILETIME;
  lpLastAccessTime: PFILETIME; lpLastWriteTime: PFILETIME): BOOL;
function CeSetFileTime(hFile: THandle; CreationTime: PFILETIME;
  LastAccessTime: PFILETIME; lastWriteTime: PFILETIME): BOOL;
function CeGetVersionEx(lpVersionInfo: PCeOSVERSIONINFO): BOOL;
function CeGetWindow(hWnd: HWND; uCmd: UINT): HWND;
function CeGetWindowLong(hWnd: HWND; nIndex: integer): LongInt;
function CeGetWindowText(hWnd: HWND; lpString: LPWSTR; nMaxCount: integer): Integer;
function CeGetClassName(hWnd: HWND; lpClassName: LPWSTR; nMaxCount: integer): Integer;
procedure CeGlobalMemoryStatus(lpmst: PMemoryStatus);
function CeGetSystemPowerStatusEx(pStatus: PSYSTEM_POWER_STATUS_EX; fUpdate: BOOL): BOOL;
//added 10/16/2000 - Terence Goggin; terencegoggin@hotmail.com
function DesktopToDevice(DesktopLocation, TableList: String; Sync: BOOL; Overwrite: Integer; DeviceLocation: String): Longint;
//added 01/19/2003 - Octavio Hernandez
function DeviceToDesktop(DesktopLocation, TableList: String; Sync: BOOL; Overwrite: Integer; DeviceLocation: String): Longint;
<<<<<<< HEAD
<<<<<<< HEAD
function CeRapiInvoke(pDllPath: LPCWSTR; pFunctionName: LPCWSTR; cbInput: DWord; pInput: PByte;
  pcbOutput: PDWord; ppOutput: PPByte; ppIRAPIStream: PIRAPIStream; dwReserved: DWord): HResult;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

IMPLEMENTATION

var
  mCeRapiInit : TCeRapiInit;
<<<<<<< HEAD
<<<<<<< HEAD
  mCeRapiUninit : TCeRapiUninit;
=======
  mCeRapiUnInit : TCeRapiUnInit;
>>>>>>> graemeg/fixes_2_2
=======
  mCeRapiUnInit : TCeRapiUnInit;
>>>>>>> origin/fixes_2_2
  mCeFindAllFiles: TCeFindAllFiles;
  mRapiFreeBuffer : TRapiFreeBuffer;
  mCeRapiInitEx: TCeRapiInitEx;
  mCeCreateDatabase: TCeCreateDatabase;
  mCeDeleteDatabase: TCeDeleteDatabase;
  mCeDeleteRecord: TCeDeleteRecord;
  mCeFindFirstDatabase: TCeFindFirstDatabase;
  mCeFindNextDatabase: TCeFindNextDatabase;
  mCeOidGetInfo: TCeOidGetInfo;
  mCeOpenDatabase: TCeOpenDatabase;
  mCeReadRecordProps: TCeReadRecordProps;
  mCeSeekDatabase: TCeSeekDatabase;
  mCeSetDatabaseInfo: TCeSetDatabaseInfo;
  mCeWriteRecordProps: TCeWriteRecordProps;
  mCeFindFirstFile: TCeFindFirstFile;
  mCeFindNextFile: TCeFindNextFile;
  mCeFindClose: TCeFindClose;
  mCeGetFileAttributes: TCeGetFileAttributes;
  mCeSetFileAttributes: TCeSetFileAttributes;
  mCeCreateFile: TCeCreateFile;
  mCeReadFile: TCeReadFile;
  mCeWriteFile: TCeWriteFile;
  mCeCloseHandle: TCeCloseHandle;
  mCeFindAllDatabases: TCeFindAllDatabases;
  mCeGetLastError: TCeGetLastError;
  mGetRapiError: TGetRapiError;
  mCeSetFilePointer: TCeSetFilePointer;
  mCeSetEndOfFile: TCeSetEndOfFile;
  mCeCreateDirectory: TCeCreateDirectory;
  mCeRemoveDirectory: TCeRemoveDirectory;
  mCeCreateProcess: TCeCreateProcess;
  mCeMoveFile: TCeMoveFile;
  mCeCopyFile: TCeCopyFile;
  mCeDeleteFile: TCeDeleteFile;
  mCeGetFileSize: TCeGetFileSize;
  mCeRegOpenKeyEx: TCeRegOpenKeyEx;
  mCeRegEnumKeyEx: TCeRegEnumKeyEx;
  mCeRegCreateKeyEx: TCeRegCreateKeyEx;
  mCeRegCloseKey: TCeRegCloseKey;
  mCeRegDeleteKey: TCeRegDeleteKey;
  mCeRegEnumValue: TCeRegEnumValue;
  mCeRegDeleteValue: TCeRegDeleteValue;
  mCeRegQueryInfoKey: TCeRegQueryInfoKey;
  mCeRegQueryValueEx: TCeRegQueryValueEx;
  mCeRegSetValueEx: TCeRegSetValueEx;
  mCeGetStoreInformation: TCeGetStoreInformation;
  mCeGetSystemMetrics: TCeGetSystemMetrics;
  mCeGetDesktopDeviceCaps: TCeGetDesktopDeviceCaps;
  mCeGetSystemInfo: TCeGetSystemInfo;
  mCeSHCreateShortcut: TCeSHCreateShortcut;
  mCeSHGetShortcutTarget: TCeSHGetShortcutTarget;
  mCeCheckPassword: TCeCheckPassword;
  mCeGetFileTime: TCeGetFileTime;
  mCeSetFileTime: TCeSetFileTime;
  mCeGetVersionEx: TCeGetVersionEx;
  mCeGetWindow: TCeGetWindow;
  mCeGetWindowLong: TCeGetWindowLong;
  mCeGetWindowText: TCeGetWindowText;
  mCeGetClassName: TCeGetClassName;
  mCeGlobalMemoryStatus: TCeGlobalMemoryStatus;
  mCeGetSystemPowerStatusEx: TCeGetSystemPowerStatusEx;
  //added 10/16/2000 - Terence Goggin; terencegoggin@hotmail.com
  mDesktopToDevice: TDesktopToDevice;
  //added 01/19/2003 - Octavio Hernandez
  mDeviceToDesktop: TDeviceToDesktop;
<<<<<<< HEAD
<<<<<<< HEAD
  mCeRapiInvoke: TCeRapiInvoke;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  RapiModule, AdoCEModule: THandle;

function RapiLoaded : BOOL;
{-Assure that RAPI is loaded and globals are set}
begin
  if RapiModule <> 0 then begin
    Result := True;
    Exit;
  end;

  {Load RAPI}
  RapiModule := LoadLibrary('RAPI.DLL');
  if RapiModule <> 0 then begin
    {Say it's loaded...}
    Result := True;
    {...and load all globals}
    @mCeRapiInit := GetProcAddress(RapiModule, 'CeRapiInit');
    @mCeRapiInitEx := GetProcAddress(RapiModule, 'CeRapiInitEx');
<<<<<<< HEAD
<<<<<<< HEAD
    @mCeRapiUninit := GetProcAddress(RapiModule, 'CeRapiUninit');
=======
    @mCeRapiUnInit := GetProcAddress(RapiModule, 'CeRapiUnInit');
>>>>>>> graemeg/fixes_2_2
=======
    @mCeRapiUnInit := GetProcAddress(RapiModule, 'CeRapiUnInit');
>>>>>>> origin/fixes_2_2
    @mCeFindAllFiles := GetProcAddress(RapiModule, 'CeFindAllFiles');
    @mRapiFreeBuffer := GetProcAddress(RapiModule, 'RapiFreeBuffer');


    @mCeCreateDatabase:= GetProcAddress(RapiModule, 'CeCreateDatabase');
    @mCeDeleteDatabase:= GetProcAddress(RapiModule, 'CeDeleteDatabase');
    @mCeDeleteRecord:= GetProcAddress(RapiModule, 'CeDeleteRecord');
    @mCeFindFirstDatabase:= GetProcAddress(RapiModule, 'CeFindFirstDatabase');
    @mCeFindNextDatabase:= GetProcAddress(RapiModule, 'CeFindNextDatabase');
    @mCeOidGetInfo:= GetProcAddress(RapiModule, 'CeOidGetInfo');
    @mCeOpenDatabase:= GetProcAddress(RapiModule, 'CeOpenDatabase');
    @mCeReadRecordProps:= GetProcAddress(RapiModule, 'CeReadRecordProps');
    @mCeSeekDatabase:= GetProcAddress(RapiModule, 'CeSeekDatabase');
    @mCeSetDatabaseInfo:= GetProcAddress(RapiModule, 'CeSetDatabaseInfo');
    @mCeWriteRecordProps:= GetProcAddress(RapiModule, 'CeWriteRecordProps');
    @mCeFindFirstFile:= GetProcAddress(RapiModule, 'CeFindFirstFile');
    @mCeFindNextFile:= GetProcAddress(RapiModule, 'CeFindNextFile');
    @mCeFindClose:= GetProcAddress(RapiModule, 'CeFindClose');
    @mCeGetFileAttributes:= GetProcAddress(RapiModule, 'CeGetFileAttributes');
    @mCeSetFileAttributes:= GetProcAddress(RapiModule, 'CeSetFileAttributes');
    @mCeCreateFile:= GetProcAddress(RapiModule, 'CeCreateFile');
    @mCeReadFile:= GetProcAddress(RapiModule, 'CeReadFile');
    @mCeWriteFile:= GetProcAddress(RapiModule, 'CeWriteFile');
    @mCeCloseHandle:= GetProcAddress(RapiModule, 'CeCloseHandle');
    @mCeFindAllDatabases:= GetProcAddress(RapiModule, 'CeFindAllDatabases');
    @mCeGetLastError:= GetProcAddress(RapiModule, 'CeGetLastError');
    @mGetRapiError:= GetProcAddress(RapiModule, 'GetRapiError');
    @mCeSetFilePointer:= GetProcAddress(RapiModule, 'CeSetFilePointer');
    @mCeSetEndOfFile:= GetProcAddress(RapiModule, 'CeSetEndOfFile');
    @mCeCreateDirectory:= GetProcAddress(RapiModule, 'CeCreateDirectory');
    @mCeRemoveDirectory:= GetProcAddress(RapiModule, 'CeRemoveDirectory');
    @mCeCreateProcess:= GetProcAddress(RapiModule, 'CeCreateProcess');
    @mCeMoveFile:= GetProcAddress(RapiModule, 'CeMoveFile');
    @mCeCopyFile:= GetProcAddress(RapiModule, 'CeCopyFile');
    @mCeDeleteFile:= GetProcAddress(RapiModule, 'CeDeleteFile');
    @mCeGetFileSize:= GetProcAddress(RapiModule, 'CeGetFileSize');
    @mCeRegOpenKeyEx:= GetProcAddress(RapiModule, 'CeRegOpenKeyEx');
    @mCeRegEnumKeyEx:= GetProcAddress(RapiModule, 'CeRegEnumKeyEx');
    @mCeRegCreateKeyEx:= GetProcAddress(RapiModule, 'CeRegCreateKeyEx');
    @mCeRegCloseKey:= GetProcAddress(RapiModule, 'CeRegCloseKey');
    @mCeRegDeleteKey:= GetProcAddress(RapiModule, 'CeRegDeleteKey');
    @mCeRegEnumValue:= GetProcAddress(RapiModule, 'CeRegEnumValue');
    @mCeRegDeleteValue:= GetProcAddress(RapiModule, 'CeRegDeleteValue');
    @mCeRegQueryInfoKey:= GetProcAddress(RapiModule, 'CeRegQueryInfoKey');
    @mCeRegQueryValueEx:= GetProcAddress(RapiModule, 'CeRegQueryValueEx');
    @mCeRegSetValueEx:= GetProcAddress(RapiModule, 'CeRegSetValueEx');
    @mCeGetStoreInformation:= GetProcAddress(RapiModule, 'CeGetStoreInformation');
    @mCeGetSystemMetrics:= GetProcAddress(RapiModule, 'CeGetSystemMetrics');
    @mCeGetDesktopDeviceCaps:= GetProcAddress(RapiModule, 'CeGetDesktopDeviceCaps');
    @mCeGetSystemInfo:= GetProcAddress(RapiModule, 'CeGetSystemInfo');
    @mCeSHCreateShortcut:= GetProcAddress(RapiModule, 'CeSHCreateShortcut');
    @mCeSHGetShortcutTarget:= GetProcAddress(RapiModule, 'CeSHGetShortcutTarget');
    @mCeCheckPassword:= GetProcAddress(RapiModule, 'CeCheckPassword');
    @mCeGetFileTime:= GetProcAddress(RapiModule, 'CeGetFileTime');
    @mCeSetFileTime:= GetProcAddress(RapiModule, 'CeSetFileTime');
    @mCeGetVersionEx:= GetProcAddress(RapiModule, 'CeGetVersionEx');
    @mCeGetWindow:= GetProcAddress(RapiModule, 'CeGetWindow');
    @mCeGetWindowLong:= GetProcAddress(RapiModule, 'CeGetWindowLong');
    @mCeGetWindowText:= GetProcAddress(RapiModule, 'CeGetWindowText');
    @mCeGetClassName:= GetProcAddress(RapiModule, 'CeGetClassName');
    @mCeGlobalMemoryStatus:= GetProcAddress(RapiModule, 'CeGlobalMemoryStatus');
    @mCeGetSystemPowerStatusEx:= GetProcAddress(RapiModule, 'CeGetSystemPowerStatusEx');
<<<<<<< HEAD
<<<<<<< HEAD
    @mCeRapiInvoke:= GetProcAddress(RapiModule, 'CeRapiInvoke');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end
  else
    Result := False;
end;

function AdoCELoaded : BOOL;
begin
  if AdoCEModule <> 0 then begin
    Result := True;
    Exit;
  end;

  {Load ADOCE}
  AdoCEModule := LoadLibrary('C: \Archivos de Programa\Microsoft ActiveSync\adofiltr.dll');

  if AdoCEModule > HINSTANCE_ERROR then
  begin
    {Say it's loaded...}
    Result := True;

    {...and load all globals}
    @mDesktopToDevice := GetProcAddress(AdoCEModule, 'DESKTOPTODEVICE');
    @mDeviceToDesktop := GetProcAddress(AdoCEModule, 'DEVICETODESKTOP');
  end
  else
    Result := False;
end;

function CeFindAllFiles(Path: PWideChar; Attr: DWORD; var Count: DWord;
  var FindData: PCe_Find_Data_array): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeFindAllFiles <> nil then
    Result := mCeFindAllFiles(Path, Attr, Count, FindData)
  else
    Result := False;
end;

procedure RapiFreeBuffer(p: Pointer);
begin
  if not RapiLoaded then begin
    Exit;
  end;

  if @mRapiFreeBuffer <> nil then
    mRapiFreeBuffer(p);
end;

function CeRapiInit : LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRapiInit <> nil then
    Result := mCeRapiInit
  else
    Result := $FFFF;
end;

function CeRapiUnInit : LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRapiUnInit <> nil then
    Result := mCeRapiUnInit
  else
    Result := $FFFF;
end;

function CeRapiInitEx(var RInit: TRapiInit) : LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRapiInitEx <> nil then
    Result := mCeRapiInitEx(RInit)
  else
    Result := $FFFF;
end;

function CeCreateDatabase(lpszName: LPWSTR; dwDbaseType: DWORD; wNumSortOrder: WORD;
  var rgSortSpecs: TSortOrderSpec): CeOID;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeCreateDatabase <> nil then
    Result := mCeCreateDatabase(lpszName, dwDbaseType, wNumSortOrder, rgSortSpecs)
  else
    Result := $FFFF;
end;

function CeDeleteDatabase(oidDBase: CeOId): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeDeleteDatabase <> nil then
    Result := mCeDeleteDatabase(oidDBase)
  else
    Result := False;
end;

function CeDeleteRecord(hDatabase: THandle; oidRecord: CeOID): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeDeleteRecord <> nil then
    Result := mCeDeleteRecord(hDatabase, oidRecord)
  else
    Result := False;
end;

function CeFindFirstDatabase(dwDbaseType: DWORD): THandle;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeFindFirstDatabase <> nil then
    Result := mCeFindFirstDatabase(dwDbaseType)
  else
    Result := $FFFF;
end;

function CeFindNextDatabase(hEnum: THandle): CeOID;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeFindNextDatabase <> nil then
    Result := mCeFindNextDatabase(hEnum)
  else
    Result := $FFFF;
end;

function CeOidGetInfo(oid: CeOID; var poidInfo: TCeOIDINFO): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeOidGetInfo <> nil then
    Result := mCeOidGetInfo(oid, poidInfo)
  else
    Result := False;
end;

function CeOpenDatabase(var poid: CeOID; lpszName: LPWSTR; propid: CePROPID;
  dwFlags: DWORD; hwndNotify: HWND): THandle;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeOpenDatabase <> nil then
    Result := mCeOpenDatabase(poid, lpszName, propid, dwFlags, hwndNotify)
  else
    Result := $FFFF;
end;

function CeReadRecordProps(hDbase: THandle; dwFlags: DWORD; var cPropID : WORD;
  var rgPropID : PCe_PropID_array; Buffer: Pointer; var cbBuffer: DWORD): CeOID;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeReadRecordProps <> nil then
    Result := mCeReadRecordProps(hDbase, dwFlags, cPropID, rgPropID, Buffer, cbBuffer)
  else
    Result := $FFFF;
end;

function CeSeekDatabase(hDatabase: THandle; dwSeekType: DWORD; dwValue: LongInt;
  dwIndex: PDWORD): CeOID;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeSeekDatabase <> nil then
    Result := mCeSeekDatabase(hDatabase, dwSeekType, dwValue, dwIndex)
  else
    Result := $FFFF;
end;

function CeSetDatabaseInfo(oidDbase: CeOID; var NewInfo: TCeDBaseInfo): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeSetDatabaseInfo <> nil then
    Result := mCeSetDatabaseInfo(oidDbase, NewInfo)
  else
    Result := False;
end;

function CeWriteRecordProps(hDbase: THandle; oidRecord: CeOID; cPropID: WORD; PropVal: TCePROPVAL): CeOID;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeWriteRecordProps <> nil then
    Result := mCeWriteRecordProps(hDbase, oidRecord, cPropID, PropVal)
  else
    Result := $FFFF;
end;

function CeFindFirstFile(lpFileName: LPCWSTR; lpFindFileData: PCe_FIND_DATA): THandle;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeFindFirstFile <> nil then
    Result := mCeFindFirstFile(lpFileName, lpFindFileData)
  else
    Result := $FFFF;
end;

function CeFindNextFile(hFindFile: THandle; lpFindFileData: PCe_FIND_DATA): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeFindNextFile <> nil then
    Result := mCeFindNextFile(hFindFile, lpFindFileData)
  else
    Result := False;
end;

function CeFindClose(hFindFile: THandle): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeFindClose <> nil then
    Result := mCeFindClose(hFindFile)
  else
    Result := False;
end;

function CeGetFileAttributes(lpFileName: LPCWSTR): DWORD;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeGetFileAttributes <> nil then
    Result := mCeGetFileAttributes(lpFileName)
  else
    Result := $FFFF;
end;

function CeSetFileAttributes(FileName: LPCWSTR; dwFileAttributes: DWORD): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeSetFileAttributes <> nil then
    Result := mCeSetFileAttributes(FileName, dwFileAttributes)
  else
    Result := False;
end;

function CeCreateFile(lpFileName: LPCWSTR; dwDesiredAccess: DWORD; dwShareMode: DWORD;
  lpSecurityAttributes: PSecurityAttributes; dwCreationDistribution: DWORD;
  dwFlagsAndAttributes: DWORD; hTemplateFile: THandle): THandle;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeCreateFile <> nil then
    Result := mCeCreateFile(lpFileName, dwDesiredAccess, dwShareMode,
      lpSecurityAttributes, dwCreationDistribution, dwFlagsAndAttributes, hTemplateFile)
  else
    Result := $FFFF;
end;

function CeReadFile(hFile: THandle; lpBuffer: Pointer; nNumberOfBytesToRead: DWORD;
  var NumberOfBytesRead : DWORD; Overlapped: POVERLAPPED): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeReadFile <> nil then
    Result := mCeReadFile(hFile, lpBuffer, nNumberOfBytesToRead,
      NumberOfBytesRead, Overlapped)
  else
    Result := False;
end;

function CeWriteFile(hFile: THandle; Buffer: Pointer; NumberOfBytesToWrite: DWORD;
  var NumberOfBytesWritten: DWORD; Overlapped: POVERLAPPED): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeWriteFile <> nil then
    Result := mCeWriteFile(hFile, Buffer, NumberOfBytesToWrite,
      NumberOfBytesWritten, Overlapped)
  else
    Result := False;
end;

function CeCloseHandle(hObject: THandle): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeCloseHandle <> nil then
    Result := mCeCloseHandle(hObject)
  else
    Result := False;
end;

function CeFindAllDatabases(dwDbaseType: DWORD; wFlags: WORD; var cFindData: DWORD; var ppFindData: PCeDB_File_Data_Array): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeFindAllDatabases <> nil then
    Result := mCeFindAllDatabases(dwDbaseType, wFlags, cFindData, ppFindData)
  else
    Result := False;
end;

function CeGetLastError : DWORD;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeGetLastError <> nil then
    Result := mCeGetLastError
  else
    Result := $FFFF;
end;

function GetRapiError : LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mGetRapiError <> nil then
    Result := mGetRapiError
  else
    Result := $FFFF;
end;

function CeSetFilePointer(hFile: THandle; DistanceToMove: LongInt; DistanceToMoveHigh: PULONG;
dwMoveMethod: DWORD): DWORD;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeSetFilePointer <> nil then
    Result := mCeSetFilePointer(hFile, DistanceToMove, DistanceToMoveHigh, dwMoveMethod)
  else
    Result := $FFFF;
end;

function CeSetEndOfFile(hFile: THandle): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeSetEndOfFile <> nil then
    Result := mCeSetEndOfFile(hFile)
  else
    Result := False;
end;

function CeCreateDirectory(lpPathName: LPCWSTR;
  lpSecurityAttributes: PSecurityAttributes): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeCreateDirectory <> nil then
    Result := mCeCreateDirectory(lpPathName, lpSecurityAttributes)
  else
    Result := False;
end;

function CeRemoveDirectory(PathName: LPCWSTR): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeRemoveDirectory <> nil then
    Result := mCeRemoveDirectory(PathName)
  else
    Result := False;
end;

function CeCreateProcess(lpApplicationName: LPCWSTR;lpCommandLine: LPCWSTR;
  lpProcessAttributes: PSecurityAttributes; lpThreadAttributes: PSecurityAttributes;
  bInheritHandles : BOOL;dwCreateFlags : DWORD; lpEnvironment: Pointer;
  lpCurrentDirectory : LPWSTR; lpStartupInfo: PSTARTUPINFO;
  lpProcessInformation: PProcessInformation): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeCreateProcess <> nil then
    Result := mCeCreateProcess(lpApplicationName, lpCommandLine, lpProcessAttributes,
      lpThreadAttributes, bInheritHandles, dwCreateFlags, lpEnvironment, lpCurrentDirectory,
      lpStartupInfo, lpProcessInformation)
  else
    Result := False;
end;

function CeMoveFile(lpExistingFileName: LPCWSTR; lpNewFileName: LPCWSTR): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeMoveFile <> nil then
    Result := mCeMoveFile(lpExistingFileName, lpNewFileName)
  else
    Result := False;
end;

function CeCopyFile(lpExistingFileName: LPCWSTR; lpNewFileName: LPCWSTR; bFailIfExists: BOOL): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeCopyFile <> nil then
    Result := mCeCopyFile(lpExistingFileName, lpNewFileName, bFailIfExists)
  else
    Result := False;
end;

function CeDeleteFile(lpFileName: LPCWSTR): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeDeleteFile <> nil then
    Result := mCeDeleteFile(lpFileName)
  else
    Result := False;
end;

function CeGetFileSize(hFile: THandle; lpFileSizeHigh: PDWORD): DWORD;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeGetFileSize <> nil then
    Result := mCeGetFileSize(hFile, lpFileSizeHigh)
  else
    Result := $FFFF;
end;

function CeRegOpenKeyEx(hKey: HKEY; SubKey: LPCWSTR; Reserved: DWORD;
  samDesired: REGSAM; pResult: PHKEY): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRegOpenKeyEx <> nil then
    Result := mCeRegOpenKeyEx(hKey, SubKey, Reserved, samDesired, pResult)
  else
    Result := $FFFF;
end;

function CeRegEnumKeyEx(hKey: HKEY; dwIndex: DWORD; KeyName: LPWSTR; chName: PDWORD;
  reserved: PDWORD; szClass: LPWSTR; cchClass: PDWORD; ftLastWrite: PFILETIME): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRegEnumKeyEx <> nil then
    Result := mCeRegEnumKeyEx(hKey, dwIndex, KeyName, chName,
      reserved, szClass, cchClass, ftLastWrite)
  else
    Result := $FFFF;
end;

function CeRegCreateKeyEx(hKey: HKEY; lpSzSubKey: LPCWSTR; dwReserved: DWORD;
  lpszClass: LPWSTR; dwOption: DWORD; samDesired: REGSAM;
  lpSecurityAttributes: PSecurityAttributes; phkResult: PHKEY; lpdwDisposition: PDWORD): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRegCreateKeyEx <> nil then
    Result := mCeRegCreateKeyEx(hKey, lpSzSubKey, dwReserved,
      lpszClass, dwOption, samDesired, lpSecurityAttributes, phkResult, lpdwDisposition)
  else
    Result := $FFFF;
end;

function CeRegCloseKey(hKey: HKEY): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRegCloseKey <> nil then
    Result := mCeRegCloseKey(hKey)
  else
    Result := $FFFF;
end;

function CeRegDeleteKey(hKey: HKEY; lpszSubKey: LPCWSTR): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRegDeleteKey <> nil then
    Result := mCeRegDeleteKey(hKey, lpszSubKey)
  else
    Result := $FFFF;
end;

function CeRegEnumValue(hKey: HKEY; dwIndex: DWORD; lpszName: LPWSTR; lpcchName: PDWORD;
lpReserved: PDWORD; lpszClass: PDWORD; lpcchClass: PBYTE; lpftLastWrite: PDWORD): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRegEnumValue <> nil then
    Result := mCeRegEnumValue(hKey, dwIndex, lpszName, lpcchName,
      lpReserved, lpszClass, lpcchClass, lpftLastWrite)
  else
    Result := $FFFF;
end;

function CeRegDeleteValue(hKey: HKEY; lpszValueName: LPCWSTR): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRegDeleteValue <> nil then
    Result := mCeRegDeleteValue(hKey, lpszValueName)
  else
    Result := $FFFF;
end;

function CeRegQueryInfoKey(hKey: HKEY; ClassName: LPWSTR; cchClass: PDWORD; Reserved: PDWORD;
  cSubKeys: PDWORD; cchMaxSubKeyLen: PDWORD; cchMaxClassLen: PDWORD; cValues: PDWORD;
  cchMaxValueNameLen: PDWORD; cbMaxValueData: PDWORD; cbSecurityDescriptor: PDWORD;
  LastWriteTime: PFILETIME): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRegQueryInfoKey <> nil then
    Result := mCeRegQueryInfoKey(hKey, ClassName, cchClass, Reserved,
      cSubKeys, cchMaxSubKeyLen, cchMaxClassLen, cValues,
      cchMaxValueNameLen, cbMaxValueData, cbSecurityDescriptor, LastWriteTime)
  else
    Result := $FFFF;
end;

function CeRegQueryValueEx(hKey: HKEY; ValueName: LPCWSTR; Reserved: PDWORD; pType: PDWORD;
  pData: PBYTE; cbData: PDWORD): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRegQueryValueEx <> nil then
    Result := mCeRegQueryValueEx(hKey, ValueName, Reserved, pType, pData, cbData)
  else
    Result := $FFFF;
end;

function CeRegSetValueEx(hKey: HKEY; ValueName: LPCWSTR; reserved: DWORD;
  dwType: DWORD; pData: PBYTE; cbData: DWORD): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRegSetValueEx <> nil then
    Result := mCeRegSetValueEx(hKey, ValueName, reserved, dwType, pData, cbData)
  else
    Result := $FFFF;
end;

function CeGetStoreInformation(lpsi: PSTORE_INFORMATION): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeGetStoreInformation <> nil then
    Result := mCeGetStoreInformation(lpsi)
  else
    Result := False;
end;

function CeGetSystemMetrics(nIndex: Integer): Integer;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeGetSystemMetrics <> nil then
    Result := mCeGetSystemMetrics(nIndex)
  else
    Result := $FFFF;
end;

function CeGetDesktopDeviceCaps(nIndedx: Integer): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeGetDesktopDeviceCaps <> nil then
    Result := mCeGetDesktopDeviceCaps(nIndedx)
  else
    Result := $FFFF;
end;

procedure CeGetSystemInfo(lpSystemInfo: PSystemInfo);
begin
  if not RapiLoaded then begin
    Exit;
  end;

  if @mCeGetSystemInfo <> nil then
    mCeGetSystemInfo(lpSystemInfo);
end;

function CeSHCreateShortcut(ShortCut: LPWSTR; Target: LPWSTR): DWORD;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeSHCreateShortcut <> nil then
    Result := mCeSHCreateShortcut(ShortCut, Target)
  else
    Result := $FFFF;
end;

function CeSHGetShortcutTarget(ShortCut: LPWSTR; Target: LPWSTR; cbMax: integer): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeSHGetShortcutTarget <> nil then
    Result := mCeSHGetShortcutTarget(ShortCut, Target, cbMax)
  else
    Result := False;
end;

function CeCheckPassword(lpszPassword: LPWSTR): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeCheckPassword <> nil then
    Result := mCeCheckPassword(lpszPassword)
  else
    Result := False;
end;

function CeGetFileTime(hFile: THandle; lpCreationTime: PFILETIME;
  lpLastAccessTime: PFILETIME; lpLastWriteTime: PFILETIME): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeGetFileTime <> nil then
    Result := mCeGetFileTime( hFile, lpCreationTime, lpLastAccessTime, lpLastWriteTime)
  else
    Result := False;
end;

function CeSetFileTime(hFile: THandle; CreationTime: PFILETIME;
  lastAccessTime: PFILETIME; lastWriteTime: PFILETIME): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeSetFileTime <> nil then
    Result := mCeSetFileTime(hFile, CreationTime, lastAccessTime, lastWriteTime)
else
  Result := False;
end;

function CeGetVersionEx(lpVersionInfo: PCeOSVERSIONINFO): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeGetVersionEx <> nil then
    Result := mCeGetVersionEx(lpVersionInfo)
  else
    Result := False;
end;

function CeGetWindow(hWnd: HWND; uCmd: UINT): HWND;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeGetWindow <> nil then
    Result := mCeGetWindow(hWnd, uCmd)
  else
    Result := $FFFF;
end;

function CeGetWindowLong(hWnd: HWND; nIndex: integer): LongInt;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeGetWindowLong <> nil then
    Result := mCeGetWindowLong(hWnd , nIndex)
  else
    Result := $FFFF;
end;

function CeGetWindowText(hWnd: HWND; lpString: LPWSTR; nMaxCount: integer): Integer;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeGetWindowText <> nil then
    Result := mCeGetWindowText(hWnd, lpString, nMaxCount)
  else
    Result := $FFFF;
end;

function CeGetClassName(hWnd: HWND; lpClassName: LPWSTR; nMaxCount: integer): Integer;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeGetClassName <> nil then
    Result := mCeGetClassName(hWnd, lpClassName, nMaxCount)
  else
    Result := $FFFF;
  end;

procedure CeGlobalMemoryStatus(lpmst: PMemoryStatus);
begin
  if not RapiLoaded then begin
    Exit;
  end;

  if @mCeGlobalMemoryStatus <> nil then
    mCeGlobalMemoryStatus(lpmst);
end;

function CeGetSystemPowerStatusEx(pStatus: PSYSTEM_POWER_STATUS_EX; fUpdate: BOOL): BOOL;
begin
  if not RapiLoaded then begin
    Result := False;
    Exit;
  end;

  if @mCeGetSystemPowerStatusEx <> nil then
    Result := mCeGetSystemPowerStatusEx(pStatus, fUpdate)
  else
    Result := False;
end;

//added 10/16/00 - Terence Goggin; terencegoggin@hotmail.com
function DesktopToDevice(DesktopLocation, TableList: String; Sync: BOOL; Overwrite: Integer; DeviceLocation: String): Longint;
begin
  if not AdoCELoaded then
  begin
    Result := $FFFF;
    Exit;
  end;

  if @mDesktopToDevice <> nil then
    Result := mDesktopToDevice(DesktopLocation, TableList, Sync, Overwrite, DeviceLocation)
  else
    Result := $FFFF;
end;

//added 01/19/2003 - Octavio Hernandez
function DeviceToDesktop(DesktopLocation, TableList: String; Sync: BOOL; Overwrite: Integer; DeviceLocation: String): Longint;
begin
  if not AdoCELoaded then
  begin
    Result := $FFFF;
    Exit;
  end;

  if @mDeviceToDesktop <> nil then
    Result := mDeviceToDesktop(DesktopLocation, TableList, Sync, Overwrite, DeviceLocation)
  else
    Result := $FFFF;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function CeRapiInvoke(pDllPath: LPCWSTR; pFunctionName: LPCWSTR;
  cbInput: DWord; pInput: PByte; pcbOutput: PDWord; ppOutput: PPByte;
  ppIRAPIStream: PIRAPIStream; dwReserved: DWord): HResult;
begin
  if not RapiLoaded then begin
    Result := $FFFF;
    Exit;
  end;

  if @mCeRapiInvoke <> nil then
    Result := mCeRapiInvoke(pDllPath, pFunctionName, cbInput, pInput, pcbOutput, ppOutput, ppIRAPIStream, dwReserved)
  else
    Result := $FFFF;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
INITIALIZATION
  RapiModule := 0;
  AdoCEModule := 0;
END.

