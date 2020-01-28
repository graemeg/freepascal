{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2016 by Free Pascal development team

    Sysutils unit for Atari

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
{  dos,} sysconst;

{$DEFINE FPC_FEXPAND_UNC} (* UNC paths are supported *)
{$DEFINE FPC_FEXPAND_DRIVES} (* Full paths begin with drive specification *)

{ Include platform independent implementation part }
{$i sysutils.inc}


{$i gemdos.inc}



{****************************************************************************
                              File Functions
****************************************************************************}
{$I-}{ Required for correct usage of these routines }


(****** non portable routines ******)

function FileOpen(const FileName: rawbytestring; Mode: Integer): THandle;
begin
  { Mode has some Share modes. Maybe something for MiNT? }
  { Lower three bits of Mode are actually TOS compatible }
  FileOpen:=gemdos_fopen(pchar(FileName), Mode and 3);
  if FileOpen < -1 then
    FileOpen:=-1;
end;


function FileGetDate(Handle: THandle) : Int64;
var
  td: TDOSTIME;
begin
  { Fdatime doesn't report errors... }
  gemdos_fdatime(@td,handle,0);
  result:=(td.date shl 16) or td.time;
end;


function FileSetDate(Handle: THandle; Age: Int64) : LongInt;
var
  td: TDOSTIME;
begin
  td.date:=(Age shr 16) and $ffff;
  td.time:=Age and $ffff;
  gemdos_fdatime(@td,handle,1);
  { Fdatime doesn't report errors... }
  result:=0;
end;


function FileSetDate(const FileName: RawByteString; Age: Int64) : LongInt;
var
  f: THandle;
begin
  FileSetDate:=-1;
  f:=FileOpen(FileName,fmOpenReadWrite);
  if f < 0 then
    exit;
  FileSetDate(f,Age);
  FileClose(f);
end;


function FileCreate(const FileName: RawByteString) : THandle;
begin
  FileCreate:=gemdos_fcreate(pchar(FileName),0);
  if FileCreate < -1 then
    FileCreate:=-1;
end;

function FileCreate(const FileName: RawByteString; Rights: integer): THandle;
begin
  { Rights are Un*x extension. Maybe something for MiNT? }
  FileCreate:=FileCreate(FileName);
end;

function FileCreate(const FileName: RawByteString; ShareMode: integer; Rights : integer): THandle;
begin
  { Rights and ShareMode are Un*x extension. Maybe something for MiNT? }
  FileCreate:=FileCreate(FileName);
end;


function FileRead(Handle: THandle; out Buffer; Count: LongInt): LongInt;
begin
  FileRead:=-1;
  if (Count<=0) then
    exit;

  FileRead:=gemdos_fread(handle, count, @buffer);
  if FileRead < -1 then
    FileRead:=-1;
end;


function FileWrite(Handle: THandle; const Buffer; Count: LongInt): LongInt;
begin
  FileWrite:=-1;
  if (Count<=0) then 
    exit;

  FileWrite:=gemdos_fwrite(handle, count, @buffer);
  if FileWrite < -1 then
    FileWrite:=-1;
end;


function FileSeek(Handle: THandle; FOffset, Origin: LongInt) : LongInt;
var
  dosResult: longint;
begin
  FileSeek:=-1;

  { TOS seek mode flags are actually compatible to DOS/TP }
  dosResult:=gemdos_fseek(FOffset, Handle, Origin);
  if dosResult < 0 then
    exit;

  FileSeek:=dosResult;
end;

function FileSeek(Handle: THandle; FOffset: Int64; Origin: Longint): Int64;
begin
  FileSeek:=FileSeek(Handle,LongInt(FOffset),Origin);
end;


procedure FileClose(Handle: THandle);
begin
  gemdos_fclose(handle);
end;


function FileTruncate(Handle: THandle; Size: Int64): Boolean;
begin
  FileTruncate:=False;
end;


function DeleteFile(const FileName: RawByteString) : Boolean;
begin
  DeleteFile:=gemdos_fdelete(pchar(FileName)) >= 0;
end;


function RenameFile(const OldName, NewName: RawByteString): Boolean;
begin
  RenameFile:=gemdos_frename(0,pchar(oldname),pchar(newname)) >= 0;
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
    dta_original: pointer;
    dta_search: TDTA;
  end;


Function InternalFindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name: RawByteString) : Longint;
var
  dosResult: longint;
  IFD: PInternalFindData;
begin
  result:=-1; { We emulate Linux/Unix behaviour, and return -1 on errors. }

  new(IFD);
  IFD^.dta_original:=gemdos_getdta;
  gemdos_setdta(@IFD^.dta_search);

  Rslt.FindHandle:=nil;
  dosResult:=gemdos_fsfirst(pchar(path), Attr and faAnyFile);
  if dosResult < 0 then
    begin
      InternalFindClose(IFD);
      exit;
    end;

  Rslt.FindHandle:=IFD;
  with IFD^.dta_search do
    begin
      Name:=d_fname;
      SetCodePage(Name,DefaultFileSystemCodePage,false);

      Rslt.Time:=(d_date shl 16) or d_time;
      Rslt.Size:=d_length;

      { "128" is Windows "NORMALFILE" attribute. Some buggy code depend on this... :( (KB) }
      Rslt.Attr := 128 or d_attrib;
    end;

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

  dosResult:=gemdos_fsnext;
  if dosResult < 0 then
    exit;

  with IFD^.dta_search do
    begin
      Name:=d_fname;
      SetCodePage(Name,DefaultFileSystemCodePage,false);

      Rslt.Time:=(d_date shl 16) or d_time;
      Rslt.Size:=d_length;

      { "128" is Windows "NORMALFILE" attribute. Some buggy code depend on this... :( (KB) }
      Rslt.Attr := 128 or d_attrib;
    end;

  result:=0;
end;


Procedure InternalFindClose(var Handle: Pointer);
var
  IFD: PInternalFindData;
begin
  IFD:=PInternalFindData(Handle);
  if not assigned(IFD) then
    exit;

  gemdos_setdta(IFD^.dta_original);

  dispose(IFD);
  IFD:=nil;
end;


(****** end of non portable routines ******)

Function FileGetAttr (Const FileName : RawByteString) : Longint;
begin
  FileGetAttr:=gemdos_fattrib(pchar(FileName),0,0);
end;


Function FileSetAttr (Const Filename : RawByteString; Attr: longint) : Longint;
begin
  FileSetAttr:=gemdos_fattrib(pchar(FileName),1,Attr and faAnyFile);

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
  di: TDISKINFO;
begin
  DiskSize := -1;

  dosResult:=gemdos_dfree(@di,drive);
  if dosResult < 0 then
    exit;

  DiskSize:=di.b_total * di.b_secsiz * di.b_clsiz;
end;

function DiskFree(Drive: Byte): Int64;
var
  dosResult: longint;
  di: TDISKINFO;
begin
  DiskFree := -1;

  dosResult:=gemdos_dfree(@di,drive);
  if dosResult < 0 then
    exit;

  DiskFree:=di.b_free * di.b_secsiz * di.b_clsiz;
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
var
  TOSTime: Longint;
begin
   LongRec(TOSTime).hi:=gemdos_tgetdate;
   LongRec(TOSTime).lo:=gemdos_tgettime;
   DateTimeToSystemTime(FileDateToDateTime(TOSTime),SystemTime);
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

  { the zero offset for cmdline is actually correct here. pexec() expects
    pascal formatted string for cmdline, so length in first byte }
  result:=gemdos_pexec(0,PChar(tmpPath),@pcmdline[0],nil);

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
  {writeln('Unimplemented Sleep');}
end;


{****************************************************************************
                              Initialization code
****************************************************************************}

Initialization
  InitExceptions;
  InitInternational;    { Initialize internationalization settings }
  OnBeep:=Nil;          { No SysBeep() on Atari for now. }

Finalization
  FreeTerminateProcs;
  DoneExceptions;
end.
