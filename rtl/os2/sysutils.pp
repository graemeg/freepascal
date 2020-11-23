{

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Sysutils unit for OS/2

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

{$DEFINE HAS_SLEEP}
{$DEFINE HAS_OSERROR}

{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_ANSISTR_FILEUTIL_IMPL}
{ OS has an ansistring/single byte environment variable API }
{$define SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}
{ OS has an ansistring/single byte API for executing other processes }
{$DEFINE EXECUTEPROCUNI}

{ Include platform independent interface part }
{$i sysutilh.inc}


implementation

  uses
    sysconst, DosCalls;


type
(* Necessary here due to a different definition of TDateTime in DosCalls. *)
  TDateTime = System.TDateTime;

threadvar
  LastOSError: cardinal;

{$DEFINE FPC_FEXPAND_UNC} (* UNC paths are supported *)
{$DEFINE FPC_FEXPAND_DRIVES} (* Full paths begin with drive specification *)
{$DEFINE FPC_FEXPAND_GETENV_PCHAR}
{$DEFINE HAS_GETTICKCOUNT}
{$DEFINE HAS_GETTICKCOUNT64}
{$DEFINE HAS_LOCALTIMEZONEOFFSET}

{ Include platform independent implementation part }
{$i sysutils.inc}


{****************************************************************************
                              File Functions
****************************************************************************}

const
 ofRead        = $0000;     {Open for reading}
 ofWrite       = $0001;     {Open for writing}
 ofReadWrite   = $0002;     {Open for reading/writing}
 doDenyRW      = $0010;     {DenyAll (no sharing)}
 faCreateNew   = $00010000; {Create if file does not exist}
 faOpenReplace = $00040000; {Truncate if file exists}
 faCreate      = $00050000; {Create if file does not exist, truncate otherwise}

 FindResvdMask = $00003737  {Allowed bits for DosFindFirst parameter Attribute}
             and $000000FF; {combined with a mask for allowed attributes only}

function FileOpen (const FileName: rawbytestring; Mode: integer): THandle;
Var
  SystemFileName: RawByteString;
  Handle: THandle;
  Rc, Action: cardinal;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
(* DenyReadWrite if sharing not specified. *)
  if (Mode and 112 = 0) or (Mode and 112 > 64) then
   Mode := Mode or doDenyRW;
  Rc:=Sys_DosOpenL(PChar (SystemFileName), Handle, Action, 0, 0, 1, Mode, nil);
  If Rc=0 then
    FileOpen:=Handle
  else
   begin
    FileOpen:=feInvalidHandle; //FileOpen:=-RC;
    //should return feInvalidHandle(=-1) if fail, other negative returned value are no more errors
    OSErrorWatch (RC);
   end;
end;

function FileCreate (const FileName: RawByteString): THandle;
begin
  FileCreate := FileCreate (FileName, doDenyRW, 777); (* Sharing to DenyAll *)
end;

function FileCreate (const FileName: RawByteString; Rights: integer): THandle;
begin
  FileCreate := FileCreate (FileName, doDenyRW, Rights);
                                      (* Sharing to DenyAll *)
end;

function FileCreate (const FileName: RawByteString; ShareMode: integer;
                                                     Rights: integer): THandle;
var
  SystemFileName: RawByteString;
  Handle: THandle;
  RC, Action: cardinal;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  ShareMode := ShareMode and 112;
  (* Sharing to DenyAll as default in case of values not allowed by OS/2. *)
  if (ShareMode = 0) or (ShareMode > 64) then
   ShareMode := doDenyRW;
  RC := Sys_DosOpenL (PChar (SystemFileName), Handle, Action, 0, 0, $12,
                                    faCreate or ofReadWrite or ShareMode, nil);
  if RC = 0 then
   FileCreate := Handle
  else
   begin
    FileCreate := feInvalidHandle;
    OSErrorWatch (RC);
   end;
End;


function FileRead (Handle: THandle; Out Buffer; Count: longint): longint;
Var
  T: cardinal;
  RC: cardinal;
begin
  RC := DosRead (Handle, Buffer, Count, T);
  if RC = 0 then
   FileRead := longint (T)
  else
   begin
    FileRead := -1;
    OSErrorWatch (RC);
   end;
end;

function FileWrite (Handle: THandle; const Buffer; Count: longint): longint;
Var
  T: cardinal;
  RC: cardinal;
begin
  RC := DosWrite (Handle, Buffer, Count, T);
  if RC = 0 then
   FileWrite := longint (T)
  else
   begin
    FileWrite := -1;
    OSErrorWatch (RC);
   end;
end;

function FileSeek (Handle: THandle; FOffset, Origin: longint): longint;
var
  NPos: int64;
  RC: cardinal;
begin
  RC := Sys_DosSetFilePtrL (Handle, FOffset, Origin, NPos);
  if (RC = 0) and (NPos < high (longint)) then
    FileSeek:= longint (NPos)
  else
   begin
    FileSeek:=-1;
    OSErrorWatch (RC);
   end;
end;

function FileSeek (Handle: THandle; FOffset: Int64; Origin: Longint): Int64;
var
  NPos: int64;
  RC: cardinal;
begin
  RC := Sys_DosSetFilePtrL (Handle, FOffset, Origin, NPos);
  if RC = 0 then
    FileSeek:= NPos
  else
   begin
    FileSeek:=-1;
    OSErrorWatch (RC);
   end;
end;

procedure FileClose (Handle: THandle);
var
  RC: cardinal;
begin
  RC := DosClose (Handle);
  if RC <> 0 then
   OSErrorWatch (RC);
end;

function FileTruncate (Handle: THandle; Size: Int64): boolean;
var
  RC: cardinal;
begin
  RC := Sys_DosSetFileSizeL(Handle, Size);
  FileTruncate := RC = 0;
  if RC = 0 then
   FileSeek(Handle, 0, 2)
  else
   OSErrorWatch (RC);
end;

function FileAge (const FileName: RawByteString): Int64;
var Handle: longint;
begin
    Handle := FileOpen (FileName, 0);
    if Handle <> -1 then
        begin
            Result := FileGetDate (Handle);
            FileClose (Handle);
        end
    else
        Result := -1;
end;


function FileGetSymLinkTarget(const FileName: RawByteString; out SymLinkRec: TRawbyteSymLinkRec): Boolean;
begin
  Result := False;
end;


function FileExists (const FileName: RawByteString; FollowLink : Boolean): boolean;
var
  L: longint;
begin
  { no need to convert to DefaultFileSystemEncoding, FileGetAttr will do that }
  if FileName = '' then
    Result := false
  else
   begin
     L := FileGetAttr (FileName);
     Result := (L >= 0) and (L and (faDirectory or faVolumeID) = 0);
(* Neither VolumeIDs nor directories are files. *)
   end;
end;


type    PSearchRec = ^TSearchRec;

Function InternalFindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name: RawByteString) : Longint;

var SR: PSearchRec;
    FStat: PFileFindBuf3L;
    Count: cardinal;
    Err: cardinal;
    I: cardinal;
    SystemEncodedPath: RawByteString;

begin
  SystemEncodedPath := ToSingleByteFileSystemEncodedFileName(Path);
  New (FStat);
  Rslt.FindHandle := THandle ($FFFFFFFF);
  Count := 1;
  if FSApi64 then
   Err := DosFindFirst (PChar (SystemEncodedPath), Rslt.FindHandle,
            Attr and FindResvdMask, FStat, SizeOf (FStat^), Count, ilStandardL)
  else
   Err := DosFindFirst (PChar (SystemEncodedPath), Rslt.FindHandle,
            Attr and FindResvdMask, FStat, SizeOf (FStat^), Count, ilStandard);
  if Err <> 0 then
   OSErrorWatch (Err)
  else if Count = 0 then
   Err := 18;
  InternalFindFirst := -Err;
  if Err = 0 then
   begin
    Rslt.ExcludeAttr := 0;
    Rslt.Time := cardinal (FStat^.DateLastWrite) shl 16 + FStat^.TimeLastWrite;
    if FSApi64 then
     begin
      Rslt.Size := FStat^.FileSize;
      Name := FStat^.Name;
      Rslt.Attr := FStat^.AttrFile;
     end
    else
     begin
      Rslt.Size := PFileFindBuf3 (FStat)^.FileSize;
      Name := PFileFindBuf3 (FStat)^.Name;
      Rslt.Attr := PFileFindBuf3 (FStat)^.AttrFile;
     end;
    SetCodePage (Name, DefaultFileSystemCodePage, false);
   end
  else
   InternalFindClose(Rslt.FindHandle);

  Dispose (FStat);
end;


Function InternalFindNext (var Rslt : TAbstractSearchRec; var Name : RawByteString) : Longint;
var
  SR: PSearchRec;
  FStat: PFileFindBuf3L;
  Count: cardinal;
  Err: cardinal;
begin
  New (FStat);
  Count := 1;
  Err := DosFindNext (Rslt.FindHandle, FStat, SizeOf (FStat^), Count);
  if Err <> 0 then
   OSErrorWatch (Err)
  else if Count = 0 then
   Err := 18;
  InternalFindNext := -Err;
  if Err = 0 then
  begin
    Rslt.ExcludeAttr := 0;
    Rslt.Time := cardinal (FStat^.DateLastWrite) shl 16 + FStat^.TimeLastWrite;
    if FSApi64 then
     begin
      Rslt.Size := FStat^.FileSize;
      Name := FStat^.Name;
      Rslt.Attr := FStat^.AttrFile;
     end
    else
     begin
      Rslt.Size := PFileFindBuf3 (FStat)^.FileSize;
      Name := PFileFindBuf3 (FStat)^.Name;
      Rslt.Attr := PFileFindBuf3 (FStat)^.AttrFile;
     end;
    SetCodePage (Name, DefaultFileSystemCodePage, false);
  end;
  Dispose (FStat);
end;


Procedure InternalFindClose(var Handle: THandle);
var
  SR: PSearchRec;
  RC: cardinal;
begin
  RC := DosFindClose (Handle);
  Handle := 0;
  if RC <> 0 then
   OSErrorWatch (RC);
end;


function FileGetDate (Handle: THandle): Int64;
var
  FStat: TFileStatus3;
  Time: Longint;
  RC: cardinal;
begin
  RC := DosQueryFileInfo(Handle, ilStandard, @FStat, SizeOf(FStat));
  if RC = 0 then
  begin
    Time := FStat.TimeLastWrite + dword (FStat.DateLastWrite) shl 16;
    if Time = 0 then
      Time := FStat.TimeCreation + dword (FStat.DateCreation) shl 16;
  end else
   begin
    Time:=0;
    OSErrorWatch (RC);
   end;
  FileGetDate:=Time;
end;

function FileSetDate (Handle: THandle; Age: Int64): longint;
var
  FStat: PFileStatus3;
  RC: cardinal;
begin
  New (FStat);
  RC := DosQueryFileInfo (Handle, ilStandard, FStat, SizeOf (FStat^));
  if RC <> 0 then
   begin
    FileSetDate := -1;
    OSErrorWatch (RC);
   end
  else
   begin
    FStat^.DateLastAccess := Hi (dword (Age));
    FStat^.DateLastWrite := Hi (dword (Age));
    FStat^.TimeLastAccess := Lo (dword (Age));
    FStat^.TimeLastWrite := Lo (dword (Age));
    RC := DosSetFileInfo (Handle, ilStandard, FStat, SizeOf (FStat^));
    if RC <> 0 then
     begin
      FileSetDate := -1;
      OSErrorWatch (RC);
     end
    else
     FileSetDate := 0;
   end;
  Dispose (FStat);
end;

function FileGetAttr (const FileName: RawByteString): longint;
var
  FS: PFileStatus3;
  SystemFileName: RawByteString;
  RC: cardinal;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Filename);
  New(FS);
  RC := DosQueryPathInfo(PChar (SystemFileName), ilStandard, FS, SizeOf(FS^));
  if RC = 0 then
   Result := FS^.AttrFile
  else
   begin
    Result := - longint (RC);
    OSErrorWatch (RC);
   end;
  Dispose(FS);
end;

function FileSetAttr (const Filename: RawByteString; Attr: longint): longint;
Var
  FS: PFileStatus3;
  SystemFileName: RawByteString;
  RC: cardinal;
Begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Filename);
  New(FS);
  RC := DosQueryPathInfo (PChar (SystemFileName), ilStandard, FS, SizeOf (FS^));
  if RC = 0 then
   begin
    FS^.AttrFile:=Attr;
    RC := DosSetPathInfo(PChar (SystemFileName), ilStandard, FS, SizeOf(FS^), 0);
    if RC <> 0 then
     OSErrorWatch (RC);
   end
  else
   OSErrorWatch (RC);
  Result := - longint (RC);
  Dispose(FS);
end;


function DeleteFile (const FileName: RawByteString): boolean;
var
  SystemFileName: RawByteString;
  RC: cardinal;
Begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Filename);
  RC := DosDelete (PChar (SystemFileName));
  if RC <> 0 then
   begin
    Result := false;
    OSErrorWatch (RC);
   end
  else
   Result := true;
End;

function RenameFile (const OldName, NewName: RawByteString): boolean;
var
  OldSystemFileName, NewSystemFileName: RawByteString;
  RC: cardinal;
Begin
  OldSystemFileName:=ToSingleByteFileSystemEncodedFileName(OldName);
  NewSystemFileName:=ToSingleByteFileSystemEncodedFileName(NewName);
  RC := DosMove (PChar (OldSystemFileName), PChar (NewSystemFileName));
  if RC <> 0 then
   begin
    Result := false;
    OSErrorWatch (RC);
   end
  else
   Result := true;
End;

{****************************************************************************
                              Disk Functions
****************************************************************************}

function DiskFree (Drive: byte): int64;

var FI: TFSinfo;
    RC: cardinal;

begin
  {In OS/2, we use the filesystem information.}
  RC := DosQueryFSInfo (Drive, 1, FI, SizeOf (FI));
  if RC = 0 then
   DiskFree := int64 (FI.Free_Clusters) *
                   int64 (FI.Sectors_Per_Cluster) * int64 (FI.Bytes_Per_Sector)
  else
   begin
    DiskFree := -1;
    OSErrorWatch (RC);
   end;
end;

function DiskSize (Drive: byte): int64;

var FI: TFSinfo;
    RC: cardinal;

begin
  {In OS/2, we use the filesystem information.}
  RC := DosQueryFSinfo (Drive, 1, FI, SizeOf (FI));
  if RC = 0 then
   DiskSize := int64 (FI.Total_Clusters) *
                   int64 (FI.Sectors_Per_Cluster) * int64 (FI.Bytes_Per_Sector)
  else
   begin
    DiskSize := -1;
    OSErrorWatch (RC);
   end;
end;


function DirectoryExists (const Directory: RawByteString; FollowLink : Boolean): boolean;
var
  L: longint;
begin
  { no need to convert to DefaultFileSystemEncoding, FileGetAttr will do that }
  if Directory = '' then
   Result := false
  else
   begin
    if ((Length (Directory) = 2) or
        (Length (Directory) = 3) and
        (Directory [3] in AllowDirectorySeparators)) and
       (Directory [2] in AllowDriveSeparators) and
       (UpCase (Directory [1]) in ['A'..'Z']) then
(* Checking attributes for 'x:' is not possible but for 'x:.' it is. *)
     L := FileGetAttr (Directory + '.')
    else if (Directory [Length (Directory)] in AllowDirectorySeparators) and
                                              (Length (Directory) > 1) and
(* Do not remove '\' in '\\' (invalid path, possibly broken UNC path). *)
      not (Directory [Length (Directory) - 1] in AllowDirectorySeparators) then
     L := FileGetAttr (Copy (Directory, 1, Length (Directory) - 1))
    else
     L := FileGetAttr (Directory);
    Result := (L > 0) and (L and faDirectory = faDirectory);
   end;
end;


{****************************************************************************
                              Time Functions
****************************************************************************}

{$DEFINE HAS_DUAL_TZHANDLING}
{$I tzenv.inc}

var
  TZAlwaysFromEnv: boolean;

procedure InitTZ2; inline;
var
  DT: DosCalls.TDateTime;
begin
  DosGetDateTime (DT);
  TZAlwaysFromEnv := DT.TimeZone = -1;
end;


procedure GetLocalTime (var SystemTime: TSystemTime);
var
  DT: DosCalls.TDateTime;
begin
  DosGetDateTime(DT);
  with SystemTime do
  begin
    Year:=DT.Year;
    Month:=DT.Month;
    Day:=DT.Day;
    DayOfWeek:=DT.WeekDay;
    Hour:=DT.Hour;
    Minute:=DT.Minute;
    Second:=DT.Second;
    MilliSecond:=DT.Sec100 * 10;
  end;
end;


function GetUniversalTime (var SystemTime: TSystemTime): boolean;
var
  DT: DosCalls.TDateTime;
  Offset: longint;
begin
  if TZAlwaysFromEnv then
   begin
    GetLocalTime (SystemTime);
    Offset := GetLocalTimeOffset;
   end
  else
   begin
    DosGetDateTime (DT);
    with SystemTime do
     begin
      Year := DT.Year;
      Month := DT.Month;
      Day := DT.Day;
      DayOfWeek := DT.WeekDay;
      Hour := DT.Hour;
      Minute := DT.Minute;
      Second := DT.Second;
      MilliSecond := DT.Sec100 * 10;
     end;
    if DT.TimeZone = -1 then
     Offset := GetLocalTimeOffset
    else
     Offset := DT.TimeZone;
   end;
  UpdateTimeWithOffset (SystemTime, Offset);
  GetUniversalTime := true;
end;


function GetLocalTimeOffset: integer;
var
  DT: DosCalls.TDateTime;
begin
  if TZAlwaysFromEnv then
   begin
    if InDST then
     GetLocalTimeOffset := DSTOffsetMin
    else
     GetLocalTimeOffset := TZOffsetMin;
   end
  else
   begin
    DosGetDateTime (DT);
    if DT.TimeZone <> -1 then
     GetLocalTimeOffset := DT.TimeZone
    else
     begin
      if InDST then
       GetLocalTimeOffset := DSTOffsetMin
      else
       GetLocalTimeOffset := TZOffsetMin;
     end;
   end;
end;


{****************************************************************************
                              Misc Functions
****************************************************************************}
procedure sysbeep;

begin
  DosBeep (800, 250);
end;

{****************************************************************************
                              Locale Functions
****************************************************************************}

var
  Country: TCountryCode;
  CtryInfo: TCountryInfo;

procedure InitAnsi;
var
  I: byte;
  RC: cardinal;
begin
    for I := 0 to 255 do
        UpperCaseTable [I] := Chr (I);
    Move (UpperCaseTable, LowerCaseTable, SizeOf (UpperCaseTable));
            FillChar (Country, SizeOf (Country), 0);
            DosMapCase (SizeOf (UpperCaseTable), Country, @UpperCaseTable);
    for I := 0 to 255 do
        if UpperCaseTable [I] <> Chr (I) then
            LowerCaseTable [Ord (UpperCaseTable [I])] := Chr (I);
end;


procedure InitInternational;
var
  Size: cardinal;
  RC: cardinal;
begin
  Size := 0;
  FillChar (Country, SizeOf (Country), 0);
  FillChar (CtryInfo, SizeOf (CtryInfo), 0);
  RC := DosQueryCtryInfo (SizeOf (CtryInfo), Country, CtryInfo, Size);
  if RC = 0 then
   begin
    DateSeparator := CtryInfo.DateSeparator;
    case CtryInfo.DateFormat of
     1: begin
         ShortDateFormat := 'd/m/y';
         LongDateFormat := 'dd" "mmmm" "yyyy';
        end;
     2: begin
         ShortDateFormat := 'y/m/d';
         LongDateFormat := 'yyyy" "mmmm" "dd';
        end;
     3: begin
         ShortDateFormat := 'm/d/y';
         LongDateFormat := 'mmmm" "dd" "yyyy';
        end;
    end;
    TimeSeparator := CtryInfo.TimeSeparator;
    DecimalSeparator := CtryInfo.DecimalSeparator;
    ThousandSeparator := CtryInfo.ThousandSeparator;
    CurrencyFormat := CtryInfo.CurrencyFormat;
    CurrencyString := PChar (CtryInfo.CurrencyUnit);
   end
  else
   OSErrorWatch (RC);
  InitAnsi;
  InitInternationalGeneric;
end;

function SysErrorMessage(ErrorCode: Integer): String;
const
  SysMsgFile: array [0..10] of char = 'OSO001.MSG'#0;
var
  OutBuf: array [0..999] of char;
  RetMsgSize: cardinal;
  RC: cardinal;
begin
  RC := DosGetMessage (nil, 0, @OutBuf [0], SizeOf (OutBuf),
                                       ErrorCode, @SysMsgFile [0], RetMsgSize);
  if RC = 0 then
   begin
    SetLength (Result, RetMsgSize);
    Move (OutBuf [0], Result [1], RetMsgSize);
   end
  else
   begin
    Result:=Format(SUnknownErrorCode,[ErrorCode]);
    OSErrorWatch (RC);
   end;
end;


{****************************************************************************
                              OS Utils
****************************************************************************}

function GetEnvPChar (EnvVar: shortstring): PChar;
(* The assembler version is more than three times as fast as Pascal. *)
var
 P: PChar;
begin
 EnvVar := UpCase (EnvVar);
{$ASMMODE INTEL}
 asm
  cld
  mov edi, Environment
  lea esi, EnvVar
  xor eax, eax
  lodsb
@NewVar:
  cmp byte ptr [edi], 0
  jz @Stop
  push eax        { eax contains length of searched variable name }
  push esi        { esi points to the beginning of the variable name }
  mov ecx, -1     { our character ('=' - see below) _must_ be found }
  mov edx, edi    { pointer to beginning of variable name saved in edx }
  mov al, '='     { searching until '=' (end of variable name) }
  repne
  scasb           { scan until '=' not found }
  neg ecx         { what was the name length? }
  dec ecx         { corrected }
  dec ecx         { exclude the '=' character }
  pop esi         { restore pointer to beginning of variable name }
  pop eax         { restore length of searched variable name }
  push eax        { and save both of them again for later use }
  push esi
  cmp ecx, eax    { compare length of searched variable name with name }
  jnz @NotEqual   { ... of currently found variable, jump if different }
  xchg edx, edi   { pointer to current variable name restored in edi }
  repe
  cmpsb           { compare till the end of variable name }
  xchg edx, edi   { pointer to beginning of variable contents in edi }
  jz @Equal       { finish if they're equal }
@NotEqual:
  xor eax, eax    { look for 00h }
  mov ecx, -1     { it _must_ be found }
  repne
  scasb           { scan until found }
  pop esi         { restore pointer to beginning of variable name }
  pop eax         { restore length of searched variable name }
  jmp @NewVar     { ... or continue with new variable otherwise }
@Stop:
  xor eax, eax
  mov P, eax      { Not found - return nil }
  jmp @End
@Equal:
  pop esi         { restore the stack position }
  pop eax
  mov P, edi      { place pointer to variable contents in P }
@End:
 end ['eax','ecx','edx','esi','edi'];
 GetEnvPChar := P;
end;
{$ASMMODE ATT}


Function GetEnvironmentVariable(Const EnvVar : String) : String;

begin
    GetEnvironmentVariable := GetEnvPChar (EnvVar);
end;


Function GetEnvironmentVariableCount : Integer;

begin
(*  Result:=FPCCountEnvVar(EnvP); - the amount is already known... *)
  GetEnvironmentVariableCount := EnvC;
end;


Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};

begin
  Result:=FPCGetEnvStrFromP (EnvP, Index);
end;


procedure Sleep (Milliseconds: cardinal);

begin
 DosSleep (Milliseconds);
end;

function SysTimerTick: QWord;
var
  L: cardinal;
begin
  DosQuerySysInfo (svMsCount, svMsCount, L, 4);
  SysTimerTick := L;
end;

function ExecuteProcess (const Path: RawByteString;
                 const ComLine: RawByteString;Flags:TExecuteFlags=[]): integer;
var
 E: EOSError;
 CommandLine: RawByteString;
 Args0, Args: DosCalls.PByteArray;
 ObjNameBuf: PChar;
 ArgSize: word;
 Res: TResultCodes;
 ObjName: shortstring;
 RC: cardinal;
 ExecAppType: cardinal;
 MaxArgsSize: PtrUInt; (* Amount of memory reserved for arguments in bytes. *)
 MaxArgsSizeInc: word;

const
 ObjBufSize = 512;

function StartSession: cardinal;
var
 HQ: THandle;
 SPID, STID, QName: shortstring;
 SID, PID: cardinal;
 SD: TStartData;
 RD: TRequestData;
 PCI: PChildInfo;
 CISize: cardinal;
 Prio: byte;
begin
 Result := $FFFFFFFF;
 FillChar (SD, SizeOf (SD), 0);
 SD.Length := SizeOf (SD);
 SD.Related := ssf_Related_Child;
 if FileExists (Path) then
(* Full path necessary for starting different executable files from current *)
(* directory. *)
  CommandLine := ExpandFileName (Path)
 else
  CommandLine := Path;
 SD.PgmName := PChar (CommandLine);
 if ComLine <> '' then
  SD.PgmInputs := PChar (ComLine);
 if ExecInheritsHandles in Flags then
   SD.InheritOpt := ssf_InhertOpt_Parent;
 Str (GetProcessID, SPID);
 Str (ThreadID, STID);
 QName := '\QUEUES\FPC_ExecuteProcess_p' + SPID + 't' + STID + '.QUE'#0;
 SD.TermQ := @QName [1];
 SD.ObjectBuffer := ObjNameBuf;
 SD.ObjectBuffLen := ObjBufSize;
 RC := DosCreateQueue (HQ, quFIFO or quConvert_Address, @QName [1]);
 if RC <> 0 then
  begin
   Move (QName [1], ObjNameBuf^, Length (QName));
   OSErrorWatch (RC);
  end
 else
  begin
   RC := DosStartSession (SD, SID, PID);
   if (RC = 0) or (RC = 457) then
    begin
     RC := DosReadQueue (HQ, RD, CISize, PCI, 0, 0, Prio, 0);
     if RC = 0 then
      begin
       Result := PCI^.Return;
       RC := DosCloseQueue (HQ);
       if RC <> 0 then
        OSErrorWatch (RC);
       RC := DosFreeMem (PCI);
       if RC <> 0 then
        OSErrorWatch (RC);
       FreeMem (ObjNameBuf, ObjBufSize);
      end
     else
      begin
       OSErrorWatch (RC);
       RC := DosCloseQueue (HQ);
       OSErrorWatch (RC);
      end;
    end
   else
    begin
     OSErrorWatch (RC);
     RC := DosCloseQueue (HQ);
     if RC <> 0 then
      OSErrorWatch (RC);
    end;
  end;
end;

begin
 Result := integer ($FFFFFFFF);
 ObjName := '';
 GetMem (ObjNameBuf, ObjBufSize);
 FillChar (ObjNameBuf^, ObjBufSize, 0);

 RC := DosQueryAppType (PChar (Path), ExecAppType);
 if RC <> 0 then
  begin
   OSErrorWatch (RC);
   if (RC = 190) or (RC = 191) then
    Result := StartSession;
  end
 else
  begin
   if (ApplicationType and 3 = ExecAppType and 3) then
(* DosExecPgm should work... *)
    begin
     MaxArgsSize := Length (ComLine) + Length (Path) + 256; (* More than enough *)
     if MaxArgsSize > high (word) then
      Exit;
     if ComLine = '' then
      begin
       Args0 := nil;
       Args := nil;
      end
     else
      begin
       GetMem (Args0, MaxArgsSize);
       Args := Args0;
(* Work around a bug in OS/2 - argument to DosExecPgm *)
(* should not cross 64K boundary. *)
       while ((PtrUInt (Args) + MaxArgsSize) and $FFFF) < MaxArgsSize do
        begin
         MaxArgsSizeInc := MaxArgsSize -
                                    ((PtrUInt (Args) + MaxArgsSize) and $FFFF);
         Inc (MaxArgsSize, MaxArgsSizeInc);
         if MaxArgsSize > high (word) then
          Exit;
         ReallocMem (Args0, MaxArgsSize);
         Inc (pointer (Args), MaxArgsSizeInc);
        end;
       ArgSize := 0;
       Move (Path [1], Args^ [ArgSize], Length (Path));
       Inc (ArgSize, Length (Path));
       Args^ [ArgSize] := 0;
       Inc (ArgSize);
       {Now do the real arguments.}
       Move (ComLine [1], Args^ [ArgSize], Length (ComLine));
       Inc (ArgSize, Length (ComLine));
       Args^ [ArgSize] := 0;
       Inc (ArgSize);
       Args^ [ArgSize] := 0;
      end;
     Res.ExitCode := $FFFFFFFF;
     RC := DosExecPgm (ObjNameBuf, ObjBufSize, 0, Args, nil, Res,
                                                                 PChar (Path));
     if RC <> 0 then
      OSErrorWatch (RC);
     if Args0 <> nil then
      FreeMem (Args0, MaxArgsSize);
     if RC = 0 then
      begin
       Result := Res.ExitCode;
       FreeMem (ObjNameBuf, ObjBufSize);
      end
    end
  end;
 if RC <> 0 then
  begin
   ObjName := StrPas (ObjNameBuf);
   FreeMem (ObjNameBuf, ObjBufSize);
   if ComLine = '' then
    CommandLine := Path
   else
    CommandLine := Path + ' ' + ComLine;
   if ObjName = '' then
    E := EOSError.CreateFmt (SExecuteProcessFailed, [CommandLine, RC])
   else
    E := EOSError.CreateFmt (SExecuteProcessFailed + ' (' + ObjName + ')', [CommandLine, RC]);
   E.ErrorCode := Result;
   raise E;
  end;
end;


function ExecuteProcess (const Path: RawByteString;
        const ComLine: array of RawByteString;Flags:TExecuteFlags=[]): integer;

var
  CommandLine: AnsiString;
  I: integer;

begin
  Commandline := '';
  for I := 0 to High (ComLine) do
   if Pos (' ', ComLine [I]) <> 0 then
    CommandLine := CommandLine + ' ' + '"' + ComLine [I] + '"'
   else
    CommandLine := CommandLine + ' ' + Comline [I];
  ExecuteProcess := ExecuteProcess (Path, CommandLine);
end;


function GetTickCount: LongWord;
var
  L: cardinal;
begin
  DosQuerySysInfo (svMsCount, svMsCount, L, 4);
  GetTickCount := L;
end;

function GetTickCount64: QWord;
var
  Freq2: cardinal;
  T: QWord;
begin
  DosTmrQueryFreq (Freq2);
  DosTmrQueryTime (T);
  GetTickCount64 := T div (QWord (Freq2) div 1000);
{$NOTE GetTickCount64 takes 20 microseconds on 1GHz CPU, GetTickCount not measurable}
end;

const
  OrigOSErrorWatch: TOSErrorWatch = nil;

procedure TrackLastOSError (Error: cardinal);
begin
  LastOSError := Error;
  OrigOSErrorWatch (Error);
end;

function GetLastOSError: Integer;
begin
  GetLastOSError := Integer (LastOSError);
end;

{****************************************************************************
                              Initialization code
****************************************************************************}

Initialization
  InitExceptions;       { Initialize exceptions. OS independent }
  InitInternational;    { Initialize internationalization settings }
  OnBeep:=@SysBeep;
  LastOSError := 0;
  OrigOSErrorWatch := TOSErrorWatch (SetOSErrorTracking (@TrackLastOSError));
  InitTZ;
  InitTZ2;
Finalization
  FreeTerminateProcs;
  DoneExceptions;
end.
