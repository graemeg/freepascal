{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2004 by the Free Pascal development team

    Sysutils unit for the embedded target.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$MODE objfpc}
{$MODESWITCH OUT}
{$H+}
unit sysutils;

interface

{$DEFINE HAS_SLEEP}
{$DEFINE HAS_OSERROR}
{$modeswitch typehelpers}
{$modeswitch advancedrecords}

{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_ANSISTR_FILEUTIL_IMPL}
{ OS has an ansistring/single byte environment variable API }
{$define SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}

  { Include platform independent interface part }
  {$i sysutilh.inc}

  var
    SleepHandler: procedure(ms: cardinal) = nil;

implementation

uses
  sysconst;

  { Include platform independent implementation part }
  {$i sysutils.inc}

{****************************************************************************
                              File Functions
****************************************************************************}
function FileOpen(const FileName: RawByteString; Mode: Integer): LongInt;
begin
  result := -1;
end;


function FileGetDate(Handle: LongInt) : Int64;
begin
  result := -1;
end;


function FileSetDate(Handle, Age: Int64) : LongInt;
begin
  result := -1;
end;


function FileCreate(const FileName: RawByteString) : LongInt;
begin
  result := -1;
end;


function FileCreate(const FileName: RawByteString; Rights: integer): LongInt;
begin
  result := -1;
end;


function FileCreate(const FileName: RawByteString; ShareMode: integer; rights : integer): LongInt;
begin
  result := -1;
end;


function FileRead(Handle: LongInt; Out Buffer; Count: LongInt): LongInt;
begin
  result := -1;
end;


function FileWrite(Handle: LongInt; const Buffer; Count: LongInt): LongInt;
begin
  result := -1;
end;


function FileSeek(Handle, FOffset, Origin: LongInt) : LongInt;
begin
  result := -1;
end;

function FileSeek(Handle: LongInt; FOffset: Int64; Origin: Longint): Int64;
begin
  result := -1;
end;


procedure FileClose(Handle: LongInt);
begin
end;


function FileTruncate(Handle: THandle; Size: Int64): Boolean;
begin
  result := false;
end;


function DeleteFile(const FileName: RawByteString) : Boolean;
begin
  result := false;
end;


function RenameFile(const OldName, NewName: RawByteString): Boolean;
begin
  result := false;
end;


Function FileAge (Const FileName : RawByteString): Int64;
begin
  result := -1;
end;


function FileGetSymLinkTarget(const FileName: RawByteString; out SymLinkRec: TRawbyteSymLinkRec): Boolean;
begin
  Result := False;
end;


Function FileExists (Const FileName : RawByteString; FollowLink : Boolean) : Boolean;
Begin
  result := false;
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
begin
  result := -1;
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

procedure sysBeep;
begin
end;


Procedure Sleep(Milliseconds : Cardinal);
begin
  if assigned(SleepHandler) then
    SleepHandler(Milliseconds);
end;

Function GetLastOSError : Integer;
begin
  Result:=-1;
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

function ExecuteProcess (const Path: RawByteString;
                               const ComLine: array of RawByteString;Flags:TExecuteFlags=[]): integer;
begin
  result := -1;
end;

function ExecuteProcess (const Path: UnicodeString; const ComLine: UnicodeString;Flags:TExecuteFlags=[]): integer;
begin
  result := -1;
end;

function ExecuteProcess (const Path: UnicodeString;
                               const ComLine: array of UnicodeString;Flags:TExecuteFlags=[]): integer;
begin
  result := -1;
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
