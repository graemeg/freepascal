{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Sysutils unit for netware

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

uses DOS;

{$I nwsys.inc}
{$I errno.inc}
{$DEFINE HAS_SLEEP}
{$DEFINE HAS_OSERROR}

{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_ANSISTR_FILEUTIL_IMPL}
{ OS has an ansistring/single byte environment variable API }
{$define SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}

{$DEFINE executeprocuni} (* Only 1 byte version of ExecuteProcess is provided by the OS *)

TYPE
  TNetwareFindData =
  RECORD
    DirP  : PNWDirEnt;               { used for opendir }
    EntryP: PNWDirEnt;               { and readdir }
    Magic : WORD;                    { to avoid abends with uninitialized TSearchRec }
  END;

{ Include platform independent interface part }
{$i sysutilh.inc}



{ additional NetWare file flags}
CONST
  faSHARE              = $00000080;  { Sharable file                   }

  faNO_SUBALLOC        = $00000800;  { Don't sub alloc. this file      }
  faTRANS              = $00001000;  { Transactional file (TTS usable) }
  faREADAUD            = $00004000;  { Read audit                      }
  faWRITAUD            = $00008000;  { Write audit                     }

  faIMMPURG            = $00010000;  { Immediate purge                 }
  faNORENAM            = $00020000;  { Rename inhibit                  }
  faNODELET            = $00040000;  { Delete inhibit                  }
  faNOCOPY             = $00080000;  { Copy inhibit                    }

  faFILE_MIGRATED      = $00400000;  { File has been migrated          }
  faDONT_MIGRATE       = $00800000;  { Don't migrate this file         }
  faIMMEDIATE_COMPRESS = $02000000;  { Compress this file immediately  }
  faFILE_COMPRESSED    = $04000000;  { File is compressed              }
  faDONT_COMPRESS      = $08000000;  { Don't compress this file        }
  faCANT_COMPRESS      = $20000000;  { Can't compress this file        }
  faATTR_ARCHIVE       = $40000000;  { Entry has had an EA modified,   }
                                     { an ownerID changed, or trustee  }
                                     { info changed, etc.              }



implementation

  uses
    sysconst;

{$define FPC_FEXPAND_DRIVES}
{$define FPC_FEXPAND_VOLUMES}
{$define FPC_FEXPAND_NO_DEFAULT_PATHS}

{ Include platform independent implementation part }
{$i sysutils.inc}


{****************************************************************************
                              File Functions
****************************************************************************}

Function FileOpen (Const FileName : rawbytestring; Mode : Integer) : THandle;
VAR NWOpenFlags : longint;
    SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  NWOpenFlags:=0;
  Case (Mode and 3) of
    0 : NWOpenFlags:=NWOpenFlags or O_RDONLY;
    1 : NWOpenFlags:=NWOpenFlags or O_WRONLY;
    2 : NWOpenFlags:=NWOpenFlags or O_RDWR;
  end;
  FileOpen := _open (pchar(SystemFileName),NWOpenFlags,0);

  //!! We need to set locking based on Mode !!
end;


Function FileCreate (Const FileName : RawByteString) : THandle;
VAR SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  FileCreate:=_open(Pchar(SystemFileName),O_RdWr or O_Creat or O_Trunc,0);
end;

Function FileCreate (Const FileName : RawByteString; Rights:longint) : THandle;

begin
  FileCreate:=FileCreate (FileName);
end;


Function FileCreate (Const FileName : RawByteString; ShareMode: Longint; Rights:longint) : THandle;

begin
  FileCreate:=FileCreate (FileName);
end;


Function FileRead (Handle : THandle; Out Buffer; Count : longint) : longint;

begin
  FileRead:=_read (Handle,@Buffer,Count);
end;


Function FileWrite (Handle : THandle; const Buffer; Count : Longint) : longint;

begin
  FileWrite:=_write (Handle,@Buffer,Count);
end;


Function FileSeek (Handle : THandle; FOffset,Origin : Longint) : Longint;

begin
  FileSeek:=_lseek (Handle,FOffset,Origin);
end;


Function FileSeek (Handle : THandle; FOffset: Int64; Origin: Longint) : Int64;
begin
  {$warning need to add 64bit FileSeek }
  FileSeek:=FileSeek(Handle,Longint(FOffset),Longint(Origin));
end;


Procedure FileClose (Handle : THandle);

begin
  _close(Handle);
end;

Function FileTruncate (Handle : THandle; Size: Int64) : boolean;

begin
  if Size > high (longint) then
   FileTruncate := false
{$WARNING Possible support for 64-bit FS to be checked!}
  else
   FileTruncate:=(_chsize(Handle,Size) = 0);
end;

Function FileAge (Const FileName : RawByteString): Int64;
var Handle: longint;
begin
  Handle := FileOpen(FileName, 0);
  if Handle <> -1 then
   begin
     result := FileGetDate(Handle);
     FileClose(Handle);
   end
  else
   result := -1;
end;

Function FileLock (Handle,FOffset,FLen : Longint) : Longint;
begin
  FileLock := _lock (Handle,FOffset,FLen);
end;

Function FileLock (Handle : Longint; FOffset,FLen : Int64) : Longint;
begin
  {$warning need to add 64bit FileLock call }
  FileLock := FileLock (Handle, longint(FOffset),longint(FLen));
end;

Function FileUnlock (Handle,FOffset,FLen : Longint) : Longint;
begin
  FileUnlock := _unlock (Handle,FOffset,FLen);
end;

Function FileUnlock (Handle : Longint; FOffset,FLen : Int64) : Longint;
begin
  {$warning need to add 64bit FileUnlock call }
  FileUnlock := FileUnlock (Handle, longint(FOffset),longint(FLen));
end;

Function FileAge (Const FileName : String): Int64;

VAR Info : NWStatBufT;
    PTM  : PNWTM;
begin
  If _stat (pchar(FileName),Info) <> 0 then
    exit(-1)
  else
    begin
      PTM := _localtime (Info.st_mtime);
      IF PTM = NIL THEN
        exit(-1)
      else
        WITH PTM^ DO
          Result:=DateTimeToFileDate(EncodeDate(tm_year+1900,tm_mon+1,tm_mday)+EncodeTime(tm_hour,tm_min,tm_sec,0));
    end;
end;


function FileGetSymLinkTarget(const FileName: RawByteString; out SymLinkRec: TRawbyteSymLinkRec): Boolean;
begin
  Result := False;
end;


Function FileExists (Const FileName : RawByteString; FollowLink : Boolean) : Boolean;
VAR Info : NWStatBufT;
    SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  FileExists:=(_stat(pchar(SystemFileName),Info) = 0);
end;

Function DirectoryExists (Const Directory : RawByteString; FollowLink : Boolean) : Boolean;
Var
  Dir : RawByteString;
  drive : byte;
  FADir, StoredIORes : longint;
begin
  Dir:=Directory;
  if (length(dir)=2) and (dir[2]=':') and
     ((dir[1] in ['A'..'Z']) or (dir[1] in ['a'..'z'])) then
    begin
      { We want to test GetCurDir }
      if dir[1] in ['A'..'Z'] then
        drive:=ord(dir[1])-ord('A')+1
      else
        drive:=ord(dir[1])-ord('a')+1;
{$push}
{$I-}
      StoredIORes:=InOutRes;
      InOutRes:=0;
      GetDir(drive,dir);
      if InOutRes <> 0 then
        begin
          InOutRes:=StoredIORes;
          result:=false;
          exit;
        end;
    end;
{$pop}
  if (Length (Dir) > 1) and
    (Dir [Length (Dir)] in AllowDirectorySeparators) and
(* Do not remove '\' after ':' (root directory of a drive)
   or in '\\' (invalid path, possibly broken UNC path). *)
     not (Dir [Length (Dir) - 1] in (AllowDriveSeparators + AllowDirectorySeparators)) then
    dir:=copy(dir,1,length(dir)-1);
(* FileGetAttr returns -1 on error *)
  FADir := FileGetAttr (Dir);
  Result := (FADir <> -1) and
            ((FADir and faDirectory) = faDirectory);
end;



PROCEDURE find_setfields (VAR f : TAbstractSearchRec; VAR Name : RawByteString);
VAR T : Dos.DateTime;
BEGIN
  WITH F DO
  BEGIN
    IF FindData.Magic = $AD01 THEN
    BEGIN
      {attr := FindData.EntryP^.d_attr AND $FF;}  // lowest 8 bit -> same as dos
      attr := FindData.EntryP^.d_attr;   { return complete netware attributes }
      UnpackTime(FindData.EntryP^.d_time + (LONGINT (FindData.EntryP^.d_date) SHL 16), T);
      time := DateTimeToFileDate(EncodeDate(T.Year,T.Month,T.day)+EncodeTime(T.Hour,T.Min,T.Sec,0));
      size := FindData.EntryP^.d_size;
      name := FindData.EntryP^.d_nameDOS;
      SetCodePage(name, DefaultFileSystemCodePage, false);
    END ELSE
    BEGIN
      name := '';
    END;
  END;
END;



Function InternalFindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name: RawByteString) : Longint;
var
  SystemEncodedPath: RawByteString;
begin
  IF path = '' then
    exit (18);
  SystemEncodedPath := ToSingleByteFileSystemEncodedFileName(Path);
  Rslt.FindData.DirP := _opendir (pchar(SystemEncodedPath));
  IF Rslt.FindData.DirP = NIL THEN
    exit (18);
  IF attr <> faAnyFile THEN
    _SetReaddirAttribute (Rslt.FindData.DirP, attr);
  Rslt.FindData.Magic := $AD01;
  Rslt.FindData.EntryP := _readdir (Rslt.FindData.DirP);
  if Rslt.FindData.EntryP = nil then
  begin
    _closedir (Rslt.FindData.DirP);
    Rslt.FindData.DirP := NIL;
    result := 18;
  end else
  begin
    find_setfields (Rslt,Name);
    result := 0;
  end;
end;


Function InternalFindNext (var Rslt : TAbstractSearchRec; var Name : RawByteString) : Longint;

begin
  IF Rslt.FindData.Magic <> $AD01 THEN
    exit (18);
  Rslt.FindData.EntryP := _readdir (Rslt.FindData.DirP);
  IF Rslt.FindData.EntryP = NIL THEN
    exit (18);
  find_setfields (Rslt,Name);
  result := 0;
end;


Procedure InternalFindClose (var Handle: THandle; var FindData: TFindData);
begin
  IF FindData.Magic = $AD01 THEN
  BEGIN
    IF FindData.DirP <> NIL THEN
      _closedir (FindData.DirP);
    FindData.Magic := 0;
    FindData.DirP := NIL;
    FindData.EntryP := NIL;
  END;
end;


Function FileGetDate (Handle : THandle) : Int64;
Var Info : NWStatBufT;
    PTM  : PNWTM;
begin
  If _fstat(Handle,Info) <> 0 then
    Result:=-1
  else
    begin
      PTM := _localtime (Info.st_mtime);
      IF PTM = NIL THEN
        exit(-1)
      else
        WITH PTM^ DO
          Result:=DateTimeToFileDate(EncodeDate(tm_year+1900,tm_mon+1,tm_mday)+EncodeTime(tm_hour,tm_min,tm_sec,0));
    end;
end;


Function FileSetDate (Handle : THandle; Age : Int64) : Longint;
begin
  { i think its impossible under netware from FileHandle. I dident found a way to get the
    complete pathname of a filehandle, that would be needed for ChangeDirectoryEntry }
  FileSetDate:=-1;
  ConsolePrintf ('warning: fpc sysutils.FileSetDate not implemented'#13#10,0);
  {$warning FileSetDate not implemented (i think is impossible) }
end;


Function FileGetAttr (Const FileName : RawByteString) : Longint;
Var Info : NWStatBufT;
    SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  If _stat (pchar(SystemFileName),Info) <> 0 then
    Result:=-1
  Else
    Result := Info.st_attr AND $FFFF;
end;


Function FileSetAttr (Const Filename : RawByteString; Attr: longint) : Longint;
VAR MS : NWModifyStructure;
    SystemFileName: RawByteString;
begin
  { The Attr parameter is not used! }
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  FillChar (MS, SIZEOF (MS), 0);
  if _ChangeDirectoryEntry (PChar (SystemFilename), MS, MFileAtrributesBit, 0) <> 0 then
    result := -1
  else
    result := 0;
end;


Function DeleteFile (Const FileName : RawByteString) : Boolean;
var
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  Result:= (_UnLink (pchar(SystemFileName)) = 0);
end;


Function RenameFile (Const OldName, NewName : RawByteString) : Boolean;
var
  OldSystemFileName, NewSystemFileName: RawByteString;
begin
  OldSystemFileName:=ToSingleByteFileSystemEncodedFileName(OldName);
  NewSystemFileName:=ToSingleByteFileSystemEncodedFileName(NewName);
  RenameFile:=(_rename(pchar(OldSystemFileName),pchar(NewSystemFileName)) = 0);
end;


{****************************************************************************
                              Disk Functions
****************************************************************************}

{
  The Diskfree and Disksize functions need a file on the specified drive, since this
  is required for the statfs system call.
  These filenames are set in drivestr[0..26], and have been preset to :
   0 - '.'      (default drive - hence current dir is ok.)
   1 - '/fd0/.'  (floppy drive 1 - should be adapted to local system )
   2 - '/fd1/.'  (floppy drive 2 - should be adapted to local system )
   3 - '/'       (C: equivalent of dos is the root partition)
   4..26          (can be set by you're own applications)
  ! Use AddDisk() to Add new drives !
  They both return -1 when a failure occurs.
}
Const
  FixDriveStr : array[0..3] of pchar=(
    '.',
    'a:.',
    'b:.',
    'sys:/'
    );
var
  Drives   : byte;
  DriveStr : array[4..26] of pchar;

Procedure AddDisk(const path:string);
begin
  if not (DriveStr[Drives]=nil) then
   FreeMem(DriveStr[Drives],StrLen(DriveStr[Drives])+1);
  GetMem(DriveStr[Drives],length(Path)+1);
  StrPCopy(DriveStr[Drives],path);
  inc(Drives);
  if Drives>26 then
   Drives:=4;
end;


Function DiskFree(Drive: Byte): int64;
//var fs : statfs;
Begin
{  if ((Drive<4) and (not (fixdrivestr[Drive]=nil)) and fsstat(StrPas(fixdrivestr[drive]),fs)) or
     ((not (drivestr[Drive]=nil)) and fsstat(StrPas(drivestr[drive]),fs)) then
   Diskfree:=int64(fs.bavail)*int64(fs.bsize)
  else
   Diskfree:=-1;}
  DiskFree := -1;
  ConsolePrintf ('warning: fpc sysutils.diskfree not implemented'#13#10,0);
  {$warning DiskFree not implemented (does it make sense ?) }
End;



Function DiskSize(Drive: Byte): int64;
//var fs : statfs;
Begin
{  if ((Drive<4) and (not (fixdrivestr[Drive]=nil)) and fsstat(StrPas(fixdrivestr[drive]),fs)) or
     ((not (drivestr[Drive]=nil)) and fsstat(StrPas(drivestr[drive]),fs)) then
   DiskSize:=int64(fs.blocks)*int64(fs.bsize)
  else
   DiskSize:=-1;}
  DiskSize := -1;
  ConsolePrintf ('warning: fpc sysutils.disksize not implemented'#13#10,0);
  {$warning DiskSize not implemented (does it make sense ?) }
End;


function DirectoryExists (const Directory: string; FollowLink : Boolean): boolean;
var
  Info : NWStatBufT;
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Directory);
  If _stat (pchar(SystemFileName),Info) <> 0 then
    exit(false)
  else
    Exit ((Info.st_attr and faDirectory) <> 0);
end;


{****************************************************************************
                              Misc Functions
****************************************************************************}

procedure SysBeep;
begin
  _RingTheBell;
end;


{****************************************************************************
                              Locale Functions
****************************************************************************}

Procedure GetLocalTime(var SystemTime: TSystemTime);
var xx : word;
begin
  Dos.GetTime(SystemTime.Hour, SystemTime.Minute, SystemTime.Second, xx);
  Dos.GetDate(SystemTime.Year, SystemTime.Month, SystemTime.Day, xx);
  SystemTime.MilliSecond := 0;
end;


Procedure InitAnsi;
Var i : longint;
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
  Result:='';  // StrError(ErrorCode);
end;

{****************************************************************************
                              OS utility functions
****************************************************************************}

Function GetEnvironmentVariable(Const EnvVar : String) : String;

begin
  Result:=_getenv(PChar(EnvVar));
end;

Function GetEnvironmentVariableCount : Integer;

begin
  // Result:=FPCCountEnvVar(EnvP);
  Result:=0;
end;

Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};

begin
  // Result:=FPCGetEnvStrFromP(Envp,Index);
  Result:='';
end;




function ExecuteProcess(Const Path: RawByteString; Const ComLine: RawByteString;Flags:TExecuteFlags=[]):integer;
var
  e : EOSError;
  CommandLine: RawByteString;

begin
  dos.exec(path,comline);

  if (Dos.DosError <> 0) then
    begin
      if ComLine <> '' then
       CommandLine := Path + ' ' + ComLine
      else
       CommandLine := Path;
      e:=EOSError.CreateFmt(SExecuteProcessFailed,[CommandLine,Dos.DosError]);
      e.ErrorCode:=Dos.DosError;
      raise e;
    end;
  Result := DosExitCode;
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
    CommandLine := CommandLine + ' ' + '"' + ComLine [I] + '"'
   else
    CommandLine := CommandLine + ' ' + Comline [I];
  ExecuteProcess := ExecuteProcess (Path, CommandLine);
end;

procedure Sleep(milliseconds: Cardinal);
begin
  _delay (milliseconds);
end;

Function GetLastOSError : Integer;

begin
  Result:=Integer(__get_errno_ptr^);
end;



{****************************************************************************
                              Initialization code
****************************************************************************}

Initialization
  InitExceptions;       { Initialize exceptions. OS independent }
  InitInternational;    { Initialize internationalization settings }
  OnBeep:=@SysBeep;
Finalization
  FreeTerminateProcs;
  DoneExceptions;
end.
