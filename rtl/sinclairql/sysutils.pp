{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2021 by Free Pascal development team

    Sysutils unit for Sinclair QL

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

{OS has only 1 byte version for ExecuteProcess}
{$define executeprocuni}

{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_ANSISTR_FILEUTIL_IMPL}
{ OS has an ansistring/single byte environment variable API }
{$define SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}

{ Include platform independent interface part }
{$i sysutilh.inc}

{ Platform dependent calls }


implementation

uses
  sysconst;

{$DEFINE FPC_FEXPAND_UNC} (* UNC paths are supported *)
{$DEFINE FPC_FEXPAND_DRIVES} (* Full paths begin with drive specification *)

{ Include platform independent implementation part }
{$i sysutils.inc}

{$i qdosh.inc}
{$i qdosfuncs.inc}
{$i smsfuncs.inc}

{****************************************************************************
                              File Functions
****************************************************************************}
{$I-}{ Required for correct usage of these routines }


(****** non portable routines ******)

function FileOpen(const FileName: rawbytestring; Mode: Integer): THandle;
var
  QLMode: Integer;
begin
  FileOpen:=-1;
  case Mode of
    fmOpenRead: QLMode := Q_OPEN_IN;
    fmOpenWrite: QLMode :=  Q_OPEN_OVER;
    fmOpenReadWrite: QLMode := Q_OPEN;
  end;
  FileOpen := io_open(pchar(Filename), QLMode);
  if FileOpen < 0 then
    FileOpen:=-1;
end;


function FileGetDate(Handle: THandle) : Int64;
begin
  result:=-1;
end;


function FileSetDate(Handle: THandle; Age: Int64) : LongInt;
begin
  result:=0;
end;


function FileSetDate(const FileName: RawByteString; Age: Int64) : LongInt;
var
  f: THandle;
begin
  result:=-1;
  f:=FileOpen(FileName,fmOpenReadWrite);
  if f < 0 then
    exit;
  result:=FileSetDate(f,Age);
  FileClose(f);
end;


function FileCreate(const FileName: RawByteString) : THandle;
begin
  DeleteFile(FileName);
  FileCreate := io_open(pchar(FileName), Q_OPEN_NEW);
  if FileCreate < 0 then
    FileCreate:=-1;
end;

function FileCreate(const FileName: RawByteString; Rights: integer): THandle;
begin
  { Rights don't exist on the QL, so we simply map this to FileCreate() }
  FileCreate:=FileCreate(FileName);
end;

function FileCreate(const FileName: RawByteString; ShareMode: integer; Rights : integer): THandle;
begin
  { Rights and ShareMode don't exist on the QL so we simply map this to FileCreate() }
  FileCreate:=FileCreate(FileName);
end;


function FileRead(Handle: THandle; out Buffer; Count: LongInt): LongInt;
begin
  if (Count<=0) then
    exit;

  { io_fstrg handles EOF }
  FileRead := io_fstrg(Handle, -1, @Buffer, Count);
  if FileRead < 0 then
    FileRead:=-1;
end;


function FileWrite(Handle: THandle; const Buffer; Count: LongInt): LongInt;
begin
  FileWrite:=-1;
  if (Count<=0) then 
    exit;
  FileWrite:= io_sstrg(Handle, -1, @Buffer, Count);
  if FileWrite < 0 then
    FileWrite:=-1;
end;


function FileSeek(Handle: THandle; FOffset, Origin: LongInt) : LongInt;
var
  dosResult: longint;
  seekEOF: longint;
begin
  FileSeek := -1;

  case Origin of
    fsFromBeginning: dosResult := fs_posab(Handle, FOffset);
    fsFromCurrent: dosResult := fs_posre(Handle, FOffset);
    fsFromEnd: 
      begin
        seekEOF := $7FFFFFBF;
        dosResult := fs_posab(Handle, seekEOF);
        fOffset := -FOffset;
        dosResult := fs_posre(Handle, FOffset);
      end;  
  end;

  { We might need to handle Errors in dosResult, but
    EOF is permitted as a non-error in QDOS/SMSQ. }
  if dosResult = ERR_EF then
    dosResult := 0;

  if dosResult <> 0 then
    begin
      FileSeek := -1;
      exit;
    end;

  { However, BEWARE! FS_POSAB/FS_POSRE use FOFFSET as a VAR parameter.
    the new file position is returned in FOFFSET. }

  { Did we change FOffset? }
  FileSeek := FOffset;
end;

function FileSeek(Handle: THandle; FOffset: Int64; Origin: Longint): Int64;
var
  longOffset: longint;
begin
  longOffset := longint(FOffset);
  FileSeek:=FileSeek(Handle, longOffset, Origin);
  flush(output);
end;


procedure FileClose(Handle: THandle);
begin
  io_close(Handle);
end;


function FileTruncate(Handle: THandle; Size: Int64): Boolean;
begin
  FileTruncate := False;
  if FileSeek(Handle, LongInt(Size), fsFromBeginning) = -1 then
    exit;
  if fs_truncate(Handle) = 0 then
    FileTruncate := True;
end;

function DeleteFile(const FileName: RawByteString) : Boolean;
begin
  DeleteFile:=false;
  if io_delet(pchar(Filename)) < 0 then
    exit;
  DeleteFile := True;
end;


function RenameFile(const OldName, NewName: RawByteString): Boolean;
var
  Handle: THandle;
  QLerr: longint;
begin
  RenameFile:=false;
  Handle := FileOpen(OldName, fmOpenReadWrite);
  if Handle = -1 then
    exit;

  QLerr := fs_rename(Handle, pchar(NewName));
  FileClose(Handle);
  if QLerr >= 0 then
    RenameFile := true; 
end;



(****** end of non portable routines ******)


function FileAge (const FileName : RawByteString): Int64;
var
  f: THandle;
begin
  FileAge:=-1;
  f:=FileOpen(FileName,fmOpenRead);
  if f < 0 then
    exit;
  FileAge:=FileGetDate(f);
  FileClose(f);
end;


function FileGetSymLinkTarget(const FileName: RawByteString; out SymLinkRec: TRawbyteSymLinkRec): Boolean;
begin
  Result := False;
end;


function FileExists (const FileName : RawByteString; FollowLink : Boolean) : Boolean;
var
  Attr: longint;
begin
  FileExists:=false;
  Attr:=FileGetAttr(FileName);
  if Attr < 0 then
    exit;

  result:=(Attr and (faVolumeID or faDirectory)) = 0;
end;


type
  PInternalFindData = ^TInternalFindData;
  TInternalFindData = record
    dummy: pointer;
  end;


Function InternalFindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name: RawByteString) : Longint;
var
  dosResult: longint;
  IFD: PInternalFindData;
begin
  result:=-1; { We emulate Linux/Unix behaviour, and return -1 on errors. }

  new(IFD);
  IFD^.dummy:=nil;

  Rslt.FindHandle:=nil;
  dosResult:=-1; { add findfirst here }
  if dosResult < 0 then
    begin
      InternalFindClose(IFD);
      exit;
    end;

  Rslt.FindHandle:=IFD;

  Name:='';
  SetCodePage(Name,DefaultFileSystemCodePage,false);

  Rslt.Time:=0;
  Rslt.Size:=0;

  { "128" is Windows "NORMALFILE" attribute. Some buggy code depend on this... :( (KB) }
  Rslt.Attr := 128 or 0;

  result:=0;
end;


Function InternalFindNext (var Rslt : TAbstractSearchRec; var Name : RawByteString) : Longint;
var
  dosResult: longint;
  IFD: PInternalFindData;
begin
  result:=-1;
  IFD:=PInternalFindData(Rslt.FindHandle);
  if not assigned(IFD) then
    exit;

  dosResult:=-1;
  if dosResult < 0 then
    exit;

  Name:='';
  SetCodePage(Name,DefaultFileSystemCodePage,false);

  Rslt.Time:=0;
  Rslt.Size:=0;

  { "128" is Windows "NORMALFILE" attribute. Some buggy code depend on this... :( (KB) }
  Rslt.Attr := 128 or 0;

  result:=0;
end;


Procedure InternalFindClose(var Handle: Pointer);
var
  IFD: PInternalFindData;
begin
  IFD:=PInternalFindData(Handle);
  if not assigned(IFD) then
    exit;

  dispose(IFD);
end;


(****** end of non portable routines ******)

Function FileGetAttr (Const FileName : RawByteString) : Longint;
begin
  FileGetAttr:=0;
end;


Function FileSetAttr (Const Filename : RawByteString; Attr: longint) : Longint;
begin
  FileSetAttr:=-1;

  if FileSetAttr < -1 then
    FileSetAttr:=-1
  else
    FileSetAttr:=0;
end;



{****************************************************************************
                              Disk Functions
****************************************************************************}

function DiskSize(Drive: Byte): Int64;
var
  dosResult: longint;
begin
  DiskSize := -1;

  dosResult:=-1;
  if dosResult < 0 then
    exit;

  DiskSize:=0;
end;

function DiskFree(Drive: Byte): Int64;
var
  dosResult: longint;
begin
  DiskFree := -1;

  dosResult:=-1;
  if dosResult < 0 then
    exit;

  DiskFree:=0;
end;

function DirectoryExists(const Directory: RawByteString; FollowLink : Boolean): Boolean;
var
  Attr: longint;
begin
  DirectoryExists:=false;
  Attr:=FileGetAttr(Directory);
  if Attr < 0 then
    exit;

  result:=(Attr and faDirectory) <> 0;
end;



{****************************************************************************
                              Locale Functions
****************************************************************************}

Procedure GetLocalTime(var SystemTime: TSystemTime);
begin
   DateTimeToSystemTime(FileDateToDateTime(0),SystemTime);
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

function GetPathString: String;
begin
  {writeln('Unimplemented GetPathString');}
  result := '';
end;

Function GetEnvironmentVariable(Const EnvVar : String) : String;
begin
  {writeln('Unimplemented GetEnvironmentVariable');}
  result:='';
end;

Function GetEnvironmentVariableCount : Integer;
begin
  {writeln('Unimplemented GetEnvironmentVariableCount');}
  result:=0;
end;

Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};
begin
  {writeln('Unimplemented GetEnvironmentString');}
  result:='';
end;

function ExecuteProcess (const Path: RawByteString; const ComLine: RawByteString;Flags:TExecuteFlags=[]):
                                                                       integer;
var
  tmpPath: RawByteString;
  pcmdline: ShortString;
  CommandLine: RawByteString;
  E: EOSError;
begin
  tmpPath:=ToSingleByteFileSystemEncodedFileName(Path);
  pcmdline:=ToSingleByteFileSystemEncodedFileName(ComLine);

  result:=-1; { execute here }

  if result < 0 then begin
    if ComLine = '' then
      CommandLine := Path
    else
      CommandLine := Path + ' ' + ComLine;

    E := EOSError.CreateFmt (SExecuteProcessFailed, [CommandLine, result]);
    E.ErrorCode := result;
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
  {writeln('Unimplemented sleep');}
end;


{****************************************************************************
                              Initialization code
****************************************************************************}

Initialization
  InitExceptions;
  InitInternational;    { Initialize internationalization settings }
  OnBeep:=Nil;          { No SysBeep() on the QL for now. }

Finalization
  FreeTerminateProcs;
  DoneExceptions;
end.
