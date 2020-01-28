{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2014 by Free Pascal development team

    Sysutils unit for AmigaOS & clones

    Based on Amiga 1.x version by Carl Eric Codere, and other
    parts of the RTL

    AmigaOS and MorphOS support by Karoly Balogh
    AROS support by Marcus Sackrow

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit sysutils;

interface

{$MODE objfpc}
{$MODESWITCH OUT}
{ force ansistrings }
{$H+}
{$modeswitch typehelpers}
{$modeswitch advancedrecords}

{$DEFINE OS_FILESETDATEBYNAME}
{$DEFINE HAS_SLEEP}
{$DEFINE HAS_OSERROR}
{$DEFINE HAS_TEMPDIR}

{OS has only 1 byte version for ExecuteProcess}
{$define executeprocuni}

{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_ANSISTR_FILEUTIL_IMPL}
{ OS has an ansistring/single byte environment variable API }
{$define SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}

{ Include platform independent interface part }
{$i sysutilh.inc}

{ Platform dependent calls }

function DeviceByIdx(Idx: Integer): string;
function AddDisk(Const Path: string): Integer;
function RefreshDeviceList: Integer;
function DiskSize(Drive: AnsiString): Int64;
function DiskFree(Drive: AnsiString): Int64;


implementation

uses
  dos, sysconst;

{$DEFINE FPC_FEXPAND_VOLUMES} (* Full paths begin with drive specification *)
{$DEFINE FPC_FEXPAND_DRIVESEP_IS_ROOT}
{$DEFINE FPC_FEXPAND_NO_DEFAULT_PATHS}
{$DEFINE FPC_FEXPAND_DIRSEP_IS_UPDIR}

{ Include platform independent implementation part }
{$i sysutils.inc}


{ * Include system specific includes * }
{$include execd.inc}
{$include execf.inc}
{$include timerd.inc}
{$include doslibd.inc}
{$include doslibf.inc}
{$include utilf.inc}

{ * Followings are implemented in the system unit! * }
function PathConv(path: shortstring): shortstring; external name 'PATHCONV';
function PathConv(path: RawByteString): RawByteString; external name 'PATHCONVRBS';
procedure AddToList(var l: Pointer; h: LongInt); external name 'ADDTOLIST';
function RemoveFromList(var l: Pointer; h: LongInt): boolean; external name 'REMOVEFROMLIST';
function CheckInList(var l: Pointer; h: LongInt): pointer; external name 'CHECKINLIST';

var
  ASYS_FileList: Pointer; external name 'ASYS_FILELIST';


function BADDR(bval: BPTR): Pointer; Inline;
begin
  {$if defined(AROS)}  // deactivated for now //and (not defined(AROS_BINCOMPAT))}
  BADDR := Pointer(bval);
  {$else}
  BADDR:=Pointer(bval Shl 2);
  {$endif}
end;

function BSTR2STRING(s : Pointer): PChar; Inline;
begin
  {$if defined(AROS)}  // deactivated for now //and (not defined(AROS_BINCOMPAT))}
  BSTR2STRING:=PChar(s);
  {$else}
  BSTR2STRING:=PChar(BADDR(PtrInt(s)))+1;
  {$endif}
end;

function BSTR2STRING(s : BPTR): PChar; Inline;
begin
  {$if defined(AROS)}  // deactivated for now //and (not defined(AROS_BINCOMPAT))}
  BSTR2STRING:=PChar(s);
  {$else}
  BSTR2STRING:=PChar(BADDR(s))+1;
  {$endif}
end;

function AmigaFileDateToDateTime(aDate: TDateStamp; out success: boolean): TDateTime;
var
  tmpSecs: DWord;
  tmpDate: TDateTime;
  tmpTime: TDateTime;
  clockData: TClockData;
begin
  with aDate do
    tmpSecs:=(ds_Days * (24 * 60 * 60)) + (ds_Minute * 60) + (ds_Tick div TICKS_PER_SECOND);

  Amiga2Date(tmpSecs,@clockData);
{$HINT TODO: implement msec values, if possible}
  with clockData do begin
     success:=TryEncodeDate(year,month,mday,tmpDate) and
              TryEncodeTime(hour,min,sec,0,tmpTime);
  end;

  result:=ComposeDateTime(tmpDate,tmpTime);
end;

function DateTimeToAmigaDateStamp(dateTime: TDateTime): TDateStamp;
var
  tmpSecs: DWord;
  clockData: TClockData;
  tmpMSec: Word;
begin
{$HINT TODO: implement msec values, if possible}
  with clockData do begin
     DecodeDate(dateTime,year,month,mday);
     DecodeTime(dateTime,hour,min,sec,tmpMSec);
  end;

  tmpSecs:=Date2Amiga(@clockData);

  with result do begin
     ds_Days:= tmpSecs div (24 * 60 * 60);
     ds_Minute:= (tmpSecs div 60) mod ds_Days;
     ds_Tick:= (((tmpSecs mod 60) mod ds_Minute) mod ds_Days) * TICKS_PER_SECOND;
  end;
end;


{****************************************************************************
                              File Functions
****************************************************************************}
{$I-}{ Required for correct usage of these routines }


(****** non portable routines ******)

function FileOpen(const FileName: rawbytestring; Mode: Integer): THandle;
var
  SystemFileName: RawByteString;
  dosResult: LongInt;
begin
  SystemFileName:=PathConv(ToSingleByteFileSystemEncodedFileName(FileName));
  {$WARNING FIX ME! To do: FileOpen Access Modes}
  dosResult:=Open(PChar(SystemFileName),MODE_OLDFILE);
  if dosResult=0 then
    dosResult:=-1
  else
    AddToList(ASYS_fileList,dosResult);

  FileOpen:=dosResult;
end;


function FileGetDate(Handle: THandle) : Int64;
var
  tmpFIB : PFileInfoBlock;
  tmpDateTime: TDateTime;
  validFile: boolean;
begin
  validFile:=false;

  if (Handle <> 0) then begin
    new(tmpFIB);
    if ExamineFH(BPTR(Handle),tmpFIB) then begin
      tmpDateTime:=AmigaFileDateToDateTime(tmpFIB^.fib_Date,validFile);
    end;
    dispose(tmpFIB);
  end;

  if validFile then
    result:=DateTimeToFileDate(tmpDateTime)
  else
    result:=-1;
end;


function FileSetDate(Handle: THandle; Age: Int64) : LongInt;
var
  tmpDateStamp: TDateStamp;
  tmpName: array[0..255] of char;
begin
  result:=0;
  if (Handle <> 0) then begin
    if NameFromFH(BPTR(Handle), @tmpName, 256) then begin
      tmpDateStamp:=DateTimeToAmigaDateStamp(FileDateToDateTime(Age));
      if not SetFileDate(@tmpName,@tmpDateStamp) then begin
        IoErr(); // dump the error code for now (TODO)
        result:=-1;
      end;
    end;
  end;
end;


function FileSetDate(const FileName: RawByteString; Age: Int64) : LongInt;
var
  tmpDateStamp: TDateStamp;
  SystemFileName: RawByteString;
begin
  result:=0;
  SystemFileName:=PathConv(ToSingleByteFileSystemEncodedFileName(FileName));
  tmpDateStamp:=DateTimeToAmigaDateStamp(FileDateToDateTime(Age));
  if not SetFileDate(PChar(SystemFileName),@tmpDateStamp) then begin
    IoErr(); // dump the error code for now (TODO)
    result:=-1;
  end;
end;


function FileCreate(const FileName: RawByteString) : THandle;
var
  SystemFileName: RawByteString;
  dosResult: LongInt;
begin
  dosResult:=-1;

  { Open file in MODDE_READWRITE, then truncate it by hand rather than
    opening it in MODE_NEWFILE, because that returns an exclusive lock
    so some operations might fail with it (KB) }
  SystemFileName:=PathConv(ToSingleByteFileSystemEncodedFileName(FileName));
  dosResult:=Open(PChar(SystemFileName),MODE_READWRITE);
  if dosResult = 0 then exit;

  if SetFileSize(dosResult, 0, OFFSET_BEGINNING) = 0 then
    AddToList(ASYS_fileList,dosResult)
  else begin
    dosClose(dosResult);
    dosResult:=-1;
  end;

  FileCreate:=dosResult;
end;

function FileCreate(const FileName: RawByteString; Rights: integer): THandle;
begin
  {$WARNING FIX ME! To do: FileCreate Access Modes}
  FileCreate:=FileCreate(FileName);
end;

function FileCreate(const FileName: RawByteString; ShareMode: integer; Rights : integer): THandle;
begin
  {$WARNING FIX ME! To do: FileCreate Access Modes}
  FileCreate:=FileCreate(FileName);
end;


function FileRead(Handle: THandle; out Buffer; Count: LongInt): LongInt;
begin
  FileRead:=-1;
  if (Count<=0) or (Handle=0) or (Handle=-1) then exit;

  FileRead:=dosRead(Handle,@Buffer,Count);
end;


function FileWrite(Handle: THandle; const Buffer; Count: LongInt): LongInt;
begin
  FileWrite:=-1;
  if (Count<=0) or (Handle=0) or (Handle=-1) then exit;

  FileWrite:=dosWrite(Handle,@Buffer,Count);
end;


function FileSeek(Handle: THandle; FOffset, Origin: LongInt) : LongInt;
var
  seekMode: LongInt;
begin
  FileSeek:=-1;
  if (Handle=0) or (Handle=-1) then exit;

  case Origin of
    fsFromBeginning: seekMode:=OFFSET_BEGINNING;
    fsFromCurrent  : seekMode:=OFFSET_CURRENT;
    fsFromEnd      : seekMode:=OFFSET_END;
  end;

  dosSeek(Handle, FOffset, seekMode);
  { get the current position when FileSeek ends, which should return
    the *NEW* position, while Amiga Seek() returns the old one }
  FileSeek:=dosSeek(Handle, 0, OFFSET_CURRENT);
end;

function FileSeek(Handle: THandle; FOffset: Int64; Origin: Longint): Int64;
begin
  {$WARNING Need to add 64bit call }
  FileSeek:=FileSeek(Handle,LongInt(FOffset),LongInt(Origin));
end;


procedure FileClose(Handle: THandle);
begin
  if (Handle=0) or (Handle=-1) then exit;

  dosClose(Handle);
  RemoveFromList(ASYS_fileList,Handle);
end;


function FileTruncate(Handle: THandle; Size: Int64): Boolean;
var
  dosResult: LongInt;
begin
  FileTruncate:=False;

  if Size > high (longint) then exit;
{$WARNING Possible support for 64-bit FS to be checked!}

  if (Handle=0) or (Handle=-1) then exit;

  dosResult:=SetFileSize(Handle, Size, OFFSET_BEGINNING);
  if (dosResult<0) then exit;

  FileTruncate:=True;
end;


function DeleteFile(const FileName: RawByteString) : Boolean;
var
  SystemFileName: RawByteString;
begin
  SystemFileName:=PathConv(ToSingleByteFileSystemEncodedFileName(FileName));
  DeleteFile:=dosDeleteFile(PChar(SystemFileName));
end;


function RenameFile(const OldName, NewName: RawByteString): Boolean;
var
  OldSystemFileName, NewSystemFileName: RawByteString;
begin
  OldSystemFileName:=PathConv(ToSingleByteFileSystemEncodedFileName(OldName));
  NewSystemFileName:=PathConv(ToSingleByteFileSystemEncodedFileName(NewName));
  RenameFile:=dosRename(PChar(OldSystemFileName), PChar(NewSystemFileName)) <> 0;
end;


(****** end of non portable routines ******)


function FileAge (const FileName : RawByteString): Int64;
var
  tmpLock: BPTR;
  tmpFIB : PFileInfoBlock;
  tmpDateTime: TDateTime;
  validFile: boolean;
  SystemFileName: RawByteString;
begin
  validFile:=false;
  SystemFileName := PathConv(ToSingleByteFileSystemEncodedFileName(FileName));
  tmpLock := Lock(PChar(SystemFileName), SHARED_LOCK);

  if (tmpLock <> 0) then begin
    new(tmpFIB);
    if Examine(tmpLock,tmpFIB) <> 0 then begin
      tmpDateTime:=AmigaFileDateToDateTime(tmpFIB^.fib_Date,validFile);
    end;
    Unlock(tmpLock);
    dispose(tmpFIB);
  end;

  if validFile then
    result:=DateTimeToFileDate(tmpDateTime)
  else
    result:=-1;
end;


function FileGetSymLinkTarget(const FileName: RawByteString; out SymLinkRec: TRawbyteSymLinkRec): Boolean;
begin
  Result := False;
end;


function FileExists (const FileName : RawByteString; FollowLink : Boolean) : Boolean;
var
  tmpLock: BPTR;
  tmpFIB : PFileInfoBlock;
  SystemFileName: RawByteString;
begin
  result:=false;
  SystemFileName := PathConv(ToSingleByteFileSystemEncodedFileName(FileName));
  tmpLock := Lock(PChar(SystemFileName), SHARED_LOCK);

  if (tmpLock <> 0) then begin
    new(tmpFIB);
    if (Examine(tmpLock,tmpFIB) <> 0) and (tmpFIB^.fib_DirEntryType <= 0) then
      result:=true;
    Unlock(tmpLock);
    dispose(tmpFIB);
  end;
end;


Function InternalFindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name: RawByteString) : Longint;
var
  tmpStr: RawByteString;
  Anchor: PAnchorPath;
  tmpDateTime: TDateTime;
  validDate: boolean;
begin
  result:=-1; { We emulate Linux/Unix behaviour, and return -1 on errors. }

  tmpStr:=PathConv(ToSingleByteFileSystemEncodedFileName(Path));

  { $1e = faHidden or faSysFile or faVolumeID or faDirectory }
  Rslt.ExcludeAttr := (not Attr) and ($1e);
  Rslt.FindHandle  := nil;

  new(Anchor);
  FillChar(Anchor^,sizeof(TAnchorPath),#0);
  Rslt.FindHandle := Anchor;

  if MatchFirst(pchar(tmpStr),Anchor)<>0 then
    begin
      InternalFindClose(Rslt.FindHandle);
      exit;
    end;

  with Anchor^.ap_Info do begin
    Name := fib_FileName;
    SetCodePage(Name,DefaultFileSystemCodePage,false);

    Rslt.Size := fib_Size;
    Rslt.Time := DateTimeToFileDate(AmigaFileDateToDateTime(fib_Date,validDate));
    if not validDate then 
      begin
        InternalFindClose(Rslt.FindHandle);
        exit;
      end;

    { "128" is Windows "NORMALFILE" attribute. Some buggy code depend on this... :( (KB) }
    Rslt.Attr := 128;

    if fib_DirEntryType > 0 then Rslt.Attr:=Rslt.Attr or faDirectory;
    if ((fib_Protection and FIBF_READ) <> 0) and
       ((fib_Protection and FIBF_WRITE) = 0) then Rslt.Attr:=Rslt.Attr or faReadOnly;

    result:=0; { Return zero if everything went OK }
  end;
end;


Function InternalFindNext (var Rslt : TAbstractSearchRec; var Name : RawByteString) : Longint;
var
  Anchor: PAnchorPath;
  validDate: boolean;
begin
  result:=-1;

  Anchor:=PAnchorPath(Rslt.FindHandle);
  if not assigned(Anchor) then exit;
  if MatchNext(Anchor) <> 0 then exit;

  with Anchor^.ap_Info do begin
    Name := fib_FileName;
    SetCodePage(Name,DefaultFileSystemCodePage,false);
    Rslt.Size := fib_Size;
    Rslt.Time := DateTimeToFileDate(AmigaFileDateToDateTime(fib_Date,validDate));
    if not validDate then exit;

    { "128" is Windows "NORMALFILE" attribute. Some buggy code depend on this... :( (KB) }
    Rslt.Attr := 128;
    if fib_DirEntryType > 0 then Rslt.Attr:=Rslt.Attr or faDirectory;
    if ((fib_Protection and FIBF_READ) <> 0) and
       ((fib_Protection and FIBF_WRITE) = 0) then Rslt.Attr:=Rslt.Attr or faReadOnly;

    result:=0; { Return zero if everything went OK }
  end;
end;


Procedure InternalFindClose(var Handle: Pointer);
var
  Anchor: PAnchorPath absolute Handle;
begin
  if not assigned(Anchor) then
    exit;
  MatchEnd(Anchor);
  Dispose(Anchor);
  Handle:=nil;
end;


(****** end of non portable routines ******)

Function FileGetAttr (Const FileName : RawByteString) : Longint;
var
 F: file;
 attr: word;
begin
 Assign(F,FileName);
 dos.GetFAttr(F,attr);
 if DosError <> 0 then
    FileGetAttr := -1
 else
    FileGetAttr := Attr;
end;


Function FileSetAttr (Const Filename : RawByteString; Attr: longint) : Longint;
var
 F: file;
begin
 Assign(F, FileName);
 Dos.SetFAttr(F, Attr and $ffff);
 FileSetAttr := DosError;
end;



{****************************************************************************
                              Disk Functions
****************************************************************************}

{
  The Diskfree and Disksize functions need a file on the specified drive, since this
  is required for the statfs system call.
  These filenames are set in drivestr[0..26], and have been preset to :
   0 - ':'      (default drive - hence current dir is ok.)
   1 - 'DF0:'   (floppy drive 1 - should be adapted to local system )
   2 - 'DF1:'   (floppy drive 2 - should be adapted to local system )
   3 - 'SYS:'   (C: equivalent of dos is the SYS: partition)
   4..26          (can be set by you're own applications)
  ! Use AddDisk() to Add new drives !
  They both return -1 when a failure occurs.
}
var
  DeviceList: array[0..26] of string[20];
  NumDevices: Integer = 0;

const
  IllegalDevices: array[0..12] of string =(
                   'PED:',
                   'PRJ:',
                   'PIPE:',   // Pipes
                   'XPIPE:',  // Extented Pipe
                   'CON:',    // Console
                   'RAW:',    // RAW: Console
                   'KCON:',   // KingCON Console
                   'KRAW:',   // KingCON RAW
                   'SER:',    // serial Ports
                   'SER0:',
                   'SER1:',
                   'PAR:',    // Parallel Porty
                   'PRT:');   // Printer

function IsIllegalDevice(DeviceName: string): Boolean;
var
  i: Integer;
  Str: AnsiString;
begin
  IsIllegalDevice := False;
  Str := UpperCase(DeviceName);
  for i := Low(IllegalDevices) to High(IllegalDevices) do
  begin
    if Str = IllegalDevices[i] then
    begin
      IsIllegalDevice := True;
      Exit;
    end;
  end;
end;

function DeviceByIdx(Idx: Integer): string;
begin
  DeviceByIdx := '';
  if (Idx < 0) or (Idx >= NumDevices) then
    Exit;
  DeviceByIdx := DeviceList[Idx];
end;

function AddDisk(const Path: string): Integer;
begin
  // if hit border, restart at 4
  if NumDevices > 26 then
    NumDevices := 4;
  // set the device
  DeviceList[NumDevices] := Copy(Path, 1, 20);
  // return the Index increment for next run
  AddDisk := NumDevices;
  Inc(NumDevices);
end;

function RefreshDeviceList: Integer;
var
  List: PDosList;
  Temp: PChar;
  Str: string;
begin
  NumDevices := 0;
  AddDisk(':');          // Index 0
  AddDisk('DF0:');       // Index 1
  AddDisk('DF1:');       // Index 2
  AddDisk('SYS:');       // Index 3
  // Lock the List
  List := LockDosList(LDF_DEVICES or LDF_READ);
  // Inspect the List
  repeat
    List := NextDosEntry(List, LDF_DEVICES);
    if List <> nil then
    begin
      Temp := BSTR2STRING(List^.dol_Name);
      Str := strpas(Temp) + ':';
      if not IsIllegalDevice(str) then
        AddDisk(Str);
    end;
  until List = nil;
  UnLockDosList(LDF_DEVICES or LDF_READ);
  RefreshDeviceList := NumDevices;
end;

// New easier DiskSize()
//
function DiskSize(Drive: AnsiString): Int64;
var
  DirLock: BPTR;
  Inf: TInfoData;
  MyProc: PProcess;
  OldWinPtr: Pointer;
begin
  DiskSize := -1;
  //
  MyProc := PProcess(FindTask(Nil));
  OldWinPtr := MyProc^.pr_WindowPtr;
  MyProc^.pr_WindowPtr := Pointer(-1);
  //
  DirLock := Lock(PChar(Drive), SHARED_LOCK);
  if DirLock <> 0 then
  begin
    if Info(DirLock, @Inf) <> 0 then
      DiskSize := Int64(Inf.id_NumBlocks) * Inf.id_BytesPerBlock;
    UnLock(DirLock);
  end;
  if OldWinPtr <> Pointer(-1) then
    MyProc^.pr_WindowPtr := OldWinPtr;
end;

function DiskSize(Drive: Byte): Int64;
begin
  DiskSize := -1;
  if (Drive < 0) or (Drive >= NumDevices) then
    Exit;
  DiskSize := DiskSize(DeviceList[Drive]);
end;

// New easier DiskFree()
//
function DiskFree(Drive: AnsiString): Int64;
var
  DirLock: BPTR;
  Inf: TInfoData;
  MyProc: PProcess;
  OldWinPtr: Pointer;
begin
  DiskFree := -1;
  //
  MyProc := PProcess(FindTask(Nil));
  OldWinPtr := MyProc^.pr_WindowPtr;
  MyProc^.pr_WindowPtr := Pointer(-1);
  //
  DirLock := Lock(PChar(Drive), SHARED_LOCK);
  if DirLock <> 0 then
  begin
    if Info(DirLock, @Inf) <> 0 then
      DiskFree := Int64(Inf.id_NumBlocks - Inf.id_NumBlocksUsed) * Inf.id_BytesPerBlock;
    UnLock(DirLock);
  end;
  if OldWinPtr <> Pointer(-1) then
    MyProc^.pr_WindowPtr := OldWinPtr;
end;

function DiskFree(Drive: Byte): Int64;
begin
  DiskFree := -1;
  if (Drive < 0) or (Drive >= NumDevices) then
    Exit;
  DiskFree := DiskFree(DeviceList[Drive]);
end;

function DirectoryExists(const Directory: RawByteString; FollowLink : Boolean): Boolean;
var
  tmpLock: BPTR;
  FIB    : PFileInfoBlock;
  SystemDirName: RawByteString;
begin
  result:=false;
  if (Directory='') or (InOutRes<>0) then exit;

  SystemDirName:=PathConv(ToSingleByteFileSystemEncodedFileName(Directory));
  tmpLock:=Lock(PChar(SystemDirName),SHARED_LOCK);
  if tmpLock=0 then exit;

  FIB:=nil; new(FIB);

  if (Examine(tmpLock,FIB) <> 0) and (FIB^.fib_DirEntryType>0) then
    result:=True;

  if tmpLock<>0 then Unlock(tmpLock);
  if assigned(FIB) then dispose(FIB);
end;



{****************************************************************************
                              Locale Functions
****************************************************************************}

Procedure GetLocalTime(var SystemTime: TSystemTime);
var
 dayOfWeek: word;
 Sec100: Word;
begin
  dos.GetTime(SystemTime.Hour, SystemTime.Minute, SystemTime.Second, Sec100);
  SystemTime.Millisecond := Sec100 * 10;
  dos.GetDate(SystemTime.Year, SystemTime.Month, SystemTime.Day, DayOfWeek);
end;


Procedure InitAnsi;
Var
  i : longint;
begin
  {  Fill table entries 0 to 127  }
  for i := 0 to 96 do
    UpperCaseTable[i] := chr(i);
  for i := 97 to 122 do
    UpperCaseTable[i] := chr(i - 32);
  for i := 123 to 191 do
    UpperCaseTable[i] := chr(i);
  Move (CPISO88591UCT,UpperCaseTable[192],SizeOf(CPISO88591UCT));

  for i := 0 to 64 do
    LowerCaseTable[i] := chr(i);
  for i := 65 to 90 do
    LowerCaseTable[i] := chr(i + 32);
  for i := 91 to 191 do
    LowerCaseTable[i] := chr(i);
  Move (CPISO88591LCT,UpperCaseTable[192],SizeOf(CPISO88591UCT));
end;


Procedure InitInternational;
begin
  InitInternationalGeneric;
  InitAnsi;
end;

function SysErrorMessage(ErrorCode: Integer): String;

begin
  Result:=Format(SUnknownErrorCode,[ErrorCode]);
end;

function GetLastOSError: Integer;
begin
    result:=-1;
end;

{****************************************************************************
                              OS utility functions
****************************************************************************}

var
  StrOfPaths: String;

function SystemTags(const command: PChar; const tags: array of PtrUInt): LongInt;
begin
  SystemTags:=SystemTagList(command,@tags);
end;

function GetPathString: String;
var
   f : text;
   s : string;
begin
   s := '';
   result := '';

   { Alternatively, this could use PIPE: handler on systems which
     have this by default (not the case on classic Amiga), but then
     the child process should be started async, which for a simple
     Path command probably isn't worth the trouble. (KB) }
   assign(f,'T:'+HexStr(FindTask(nil))+'_path.tmp');
   rewrite(f);
   { This is a pretty ugly stunt, combining Pascal and Amiga system
     functions, but works... }
   SystemTags('C:Path',[SYS_Input, 0, SYS_Output, TextRec(f).Handle, TAG_END]);
   close(f);

   reset(f);
   { skip the first line, garbage }
   if not eof(f) then readln(f,s);
   while not eof(f) do begin
      readln(f,s);
      if result = '' then
        result := s
      else
        result := result + ';' + s;
   end;
   close(f);
   erase(f);
end;

Function GetEnvironmentVariable(Const EnvVar : String) : String;
begin
  if UpCase(envvar) = 'PATH' then begin
    if StrOfpaths = '' then StrOfPaths := GetPathString;
    Result:=StrOfPaths;
  end else
    Result:=Dos.Getenv(shortstring(EnvVar));
end;

Function GetEnvironmentVariableCount : Integer;

begin
  // Result:=FPCCountEnvVar(EnvP);
  Result:=Dos.envCount;
end;

Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};

begin
  // Result:=FPCGetEnvStrFromP(Envp,Index);
  Result:=Dos.EnvStr(Index);
end;

function ExecuteProcess (const Path: RawByteString; const ComLine: RawByteString;Flags:TExecuteFlags=[]):
                                                                       integer;
var
  tmpPath,
  convPath: RawByteString;
  CommandLine: AnsiString;
  tmpLock: BPTR;

  E: EOSError;
begin
  DosError:= 0;

  convPath:=PathConv(ToSingleByteFileSystemEncodedFileName(Path));
  tmpPath:=convPath+' '+ToSingleByteFileSystemEncodedFileName(ComLine);

  { Here we must first check if the command we wish to execute }
  { actually exists, because this is NOT handled by the        }
  { _SystemTagList call (program will abort!!)                 }

  { Try to open with shared lock }
  tmpLock:=Lock(PChar(convPath),SHARED_LOCK);
  if tmpLock<>0 then
    begin
      { File exists - therefore unlock it }
      Unlock(tmpLock);
      result:=SystemTagList(PChar(tmpPath),nil);
      { on return of -1 the shell could not be executed }
      { probably because there was not enough memory    }
      if result = -1 then
        DosError:=8;
    end
  else
    DosError:=3;

  if DosError <> 0 then begin
    if ComLine = '' then
      CommandLine := Path
    else
      CommandLine := Path + ' ' + ComLine;

    E := EOSError.CreateFmt (SExecuteProcessFailed, [CommandLine, DosError]);
    E.ErrorCode := DosError;
    raise E;
  end;
end;

function ExecuteProcess (const Path: RawByteString;
                                  const ComLine: array of RawByteString;Flags:TExecuteFlags=[]): integer;
var
  CommandLine: RawByteString;
  I: integer;

begin
  Commandline := '';
  for I := 0 to High (ComLine) do
   if Pos (' ', ComLine [I]) <> 0 then
    CommandLine := CommandLine + ' ' + '"' + ToSingleByteFileSystemEncodedFileName(ComLine [I]) + '"'
   else
    CommandLine := CommandLine + ' ' + ToSingleByteFileSystemEncodedFileName(Comline [I]);
  ExecuteProcess := ExecuteProcess (Path, CommandLine);
end;

procedure Sleep(Milliseconds: cardinal);
begin
  // Amiga dos.library Delay() has precision of 1/50 seconds
  DOSDelay(Milliseconds div 20);
end;


function GetTempDir(Global: Boolean): string;
begin
  if Assigned(OnGetTempDir) then
    Result := OnGetTempDir(Global)
  else
  begin
    Result := GetEnvironmentVariable('TEMP');
    if Result = '' Then
      Result:=GetEnvironmentVariable('TMP');
    if Result = '' then
      Result := 'T:'; // fallback.
  end;
  if Result <> '' then
    Result := IncludeTrailingPathDelimiter(Result);
end;


{****************************************************************************
                              Initialization code
****************************************************************************}

Initialization
  InitExceptions;
  InitInternational;    { Initialize internationalization settings }
  OnBeep:=Nil;          { No SysBeep() on Amiga, for now. Figure out if we want
                          to use intuition.library/DisplayBeep() for this (KB) }
  StrOfPaths:='';

  RefreshDeviceList;
Finalization
  FreeTerminateProcs;
  DoneExceptions;
end.
