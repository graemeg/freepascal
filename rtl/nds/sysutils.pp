{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2004 by Karoly Balogh

    Sysutils unit for Nintendo DS.
    This unit is based on the MorphOS one and is adapted for Nintendo DS
    simply by stripping out all stuff inside funcs and procs. 
    Copyright (c) 2006 by Francesco Lombardi

    Based on Amiga version by Carl Eric Codere, and other
    parts of the RTL

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

{$DEFINE HAS_OSERROR}
{$DEFINE HAS_SLEEP}
{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_ANSISTR_FILEUTIL_IMPL}
{ OS has an ansistring/single byte environment variable API }
{$define SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}

{ Include platform independent interface part }
{$i sysutilh.inc}

implementation

uses
  sysconst;

{ Include platform independent implementation part }
{$i sysutils.inc}


{****************************************************************************
                              File Functions
****************************************************************************}
function FileOpen(const FileName: rawbytestring; Mode: Integer): LongInt;
var
  NDSFlags: longint;
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  NDSFlags := 0;

  case (Mode and (fmOpenRead or fmOpenWrite or fmOpenReadWrite)) of
    fmOpenRead : NDSFlags := NDSFlags or O_RdOnly;
    fmOpenWrite : NDSFlags := NDSFlags or O_WrOnly;
    fmOpenReadWrite : NDSFlags := NDSFlags or O_RdWr;
  end;
  FileOpen := _open(pchar(SystemFileName), NDSFlags);
end;


function FileGetDate(Handle: LongInt) : Int64;
begin
  result := -1;
end;


function FileSetDate(Handle: LongInt; Age: Int64) : LongInt;
begin
  result := -1;
end;


function FileCreate(const FileName: RawByteString) : LongInt;
var
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  FileCreate:=_open(pointer(SystemFileName), O_RdWr or O_Creat or O_Trunc);
end;


function FileCreate(const FileName: RawByteString; Rights: integer): LongInt;
var
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  FileCreate:=_Open(pointer(SystemFileName),O_RdWr or O_Creat or O_Trunc,Rights);
end;


function FileCreate(const FileName: RawByteString; ShareMode : Integer; Rights: integer): LongInt;
begin
  result := FileCreate(FileName, Rights);
end;


function FileRead(Handle: LongInt; Out Buffer; Count: LongInt): LongInt;
begin
  FileRead := _Read(Handle, Buffer, Count);
end;


function FileWrite(Handle: LongInt; const Buffer; Count: LongInt): LongInt;
begin
  FileWrite := _Write(Handle, @Buffer, Count);
end;


function FileSeek(Handle, FOffset, Origin: LongInt) : LongInt;
begin
  result := longint(FileSeek(Handle, int64(FOffset), Origin));
end;

function FileSeek(Handle: LongInt; FOffset: Int64; Origin: Longint): Int64;
begin
  FileSeek := _lSeek(Handle, FOffset, Origin);
end;


procedure FileClose(Handle: LongInt);
begin
  _close(Handle);
end;


function FileTruncate(Handle: THandle; Size: Int64): Boolean;
begin
  if Size > high (longint) then
    FileTruncate := false
  else
    FileTruncate:=(_truncate(Handle,Size) = 0);
end;


function DeleteFile(const FileName: RawByteString) : Boolean;
var
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  Result := _UnLink(pointer(SystemFileName))>= 0;
end;


function RenameFile(const OldName, NewName: RawByteString): Boolean;
var
  OldSystemFileName, NewSystemFileName: RawByteString;
begin
  OldSystemFileName:=ToSingleByteFileSystemEncodedFileName(OldName);
  NewSystemFileName:=ToSingleByteFileSystemEncodedFileName(NewName);
  RenameFile := _Rename(pointer(OldSystemFileName), pointer(NewSystemFileName)) >= 0;
end;


(****** end of non portable routines ******)


Function FileAge (Const FileName : RawByteString): Int64;
var 
  info: Stat;
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  if (_stat(pchar(SystemFileName), Info) < 0) or S_ISDIR(info.st_mode) then
    exit(-1)
  else 
    Result := (info.st_mtime);
end;


function FileGetSymLinkTarget(const FileName: RawByteString; out SymLinkRec: TRawbyteSymLinkRec): Boolean;
begin
  Result := False;
end;


Function FileExists (Const FileName : RawByteString; FollowLink : Boolean) : Boolean;
var
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  FileExists := _Access(pointer(SystemFileName), F_OK) = 0;
end;



Function InternalFindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name: RawByteString) : Longint;
begin
  result := -1;
end;


Function InternalFindNext (var Rslt : TAbstractSearchRec; var Name : RawByteString) : Longint;
begin
  result := -1;
end;

Procedure InternalFindClose(var Handle: THandle);
begin

end;

Function FileGetAttr (Const FileName : RawByteString) : Longint;
var
  Info : TStat;
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  If _stat(pchar(SystemFileName), Info) <> 0 then
    Result := -1
  Else
    Result := (Info.st_mode shr 16) and $ffff;
end;


Function FileSetAttr (Const Filename : RawByteString; Attr: longint) : Longint;
begin
  result := -1;
end;



{****************************************************************************
                              Disk Functions
****************************************************************************}

Procedure AddDisk(const path:string);
begin

end;



Function DiskFree(Drive: Byte): int64;
Begin
  result := -1;
End;


Function DiskSize(Drive: Byte): int64;
Begin
  result := -1;
End;


function DirectoryExists(const Directory: RawByteString; FollowLink : Boolean): Boolean;
begin
  result := false;
end;



{****************************************************************************
                              Misc Functions
****************************************************************************}

Procedure SysBeep;
begin
end;

Procedure Sleep(Milliseconds : Cardinal);
var
  i,j : Cardinal;
  calib : Cardinal;
begin
{ $warning no idea if this calibration value is correct (FK) }
{ I estimated it roughly on the CPU clock of 16 MHz and 1+3 clock cycles for the loop }
{ 
  calib:=4000000;
  for i:=1 to Milliseconds do
    asm
      ldr r0,calib
    .L1:
      sub r0,r0,#1
      bne .L1
    end;
    }
end;
{****************************************************************************
                              Locale Functions
****************************************************************************}

Procedure GetLocalTime(var SystemTime: TSystemTime);
begin
end ;


function SysErrorMessage(ErrorCode: Integer): String;
begin
{  Result:=StrError(ErrorCode);}
  result := '';
end;

{****************************************************************************
                              OS utility functions
****************************************************************************}

Function GetEnvironmentVariable(Const EnvVar : String) : String;
begin
  result := '';
end;

Function GetEnvironmentVariableCount : Integer;
begin
  result := -1;
end;

Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};
begin
  result := '';
end;

function ExecuteProcess (const Path: RawByteString; const ComLine: RawByteString;Flags:TExecuteFlags=[]): integer;
begin
  result := -1;
end;

function ExecuteProcess (const Path: RawByteString; const ComLine: array of RawByteString;Flags:TExecuteFlags=[]): integer;
begin
  result := -1;
end;

function ExecuteProcess(const Path: UnicodeString; const ComLine: UnicodeString;
  Flags: TExecuteFlags = []): Integer;
begin
  { TODO : implement }
  result := -1;
end;

function ExecuteProcess(const Path: UnicodeString;
  const ComLine: Array of UnicodeString; Flags:TExecuteFlags = []): Integer;
var
  CommandLine: UnicodeString;
  I: integer;
begin
  Commandline := '';
  for I := 0 to High (ComLine) do
   if Pos (' ', ComLine [I]) <> 0 then
    CommandLine := CommandLine + ' ' + '"' + ComLine [I] + '"'
   else
    CommandLine := CommandLine + ' ' + Comline [I];
  ExecuteProcess := ExecuteProcess (Path, CommandLine,Flags);
end;

function GetLastOSError: Integer;
begin
  Result := -1;
end;
{****************************************************************************
                              Initialization code
****************************************************************************}

Initialization
  InitExceptions;
Finalization
  FreeTerminateProcs;
  DoneExceptions;
end.
