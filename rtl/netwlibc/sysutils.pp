{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2004 by the Free Pascal development team.

    Sysutils unit for netware (libc)

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

uses Libc,DOS;


TYPE
  TNetwareLibcFindData =
  RECORD
    DirP  : Pdirent;          { used for opendir }
    EntryP: Pdirent;          { and readdir }
    Magic : longint;          { to avoid abends with uninitialized TSearchRec }
    _mask : RawByteString;    { search mask i.e. *.* }
    _dir  : RawByteString;    { directory where to search }
    _attr : longint;          { specified attribute }
    fname : string;           { full pathname of found file }
  END;

{$DEFINE HAS_SLEEP}
{$DEFINE HAS_OSERROR}
{ Include platform independent interface part }
{$i sysutilh.inc}



{ additional NetWare file flags}
CONST
  faSHARE              = M_A_SHARE shr 16;              // Sharable file

  //faNO_SUBALLOC        = $00000800;                   // Don't sub alloc. this file
  faTRANS              = M_A_TRANS shr 16;              // Transactional file (TTS usable)
  //faREADAUD            = $00004000;                   // clib only: Read audit
  //faWRITAUD            = $00008000;                   // clib only: Write audit

  faIMMPURG            = M_A_IMMPURG shr 16;            // Immediate purge
  faNORENAM            = M_A_NORENAM shr 16;            // Rename inhibit
  faNODELET            = M_A_NODELET shr 16;            // Delete inhibit
  faNOCOPY             = M_A_NOCOPY shr 16;             // Copy inhibit

  //faFILE_MIGRATED      = $00400000;                   // clib only: File has been migrated
  //faDONT_MIGRATE       = $00800000;                   // clib only: Don't migrate this file
  faIMMEDIATE_COMPRESS = M_A_IMMCOMPRESS shr 16;        // Compress this file immediately
  faFILE_COMPRESSED    = M_A_FILE_COMPRESSED shr 16;    // File is compressed
  faDONT_COMPRESS      = M_A_DONT_COMPRESS shr 16;      // Don't compress this file
  faCANT_COMPRESS      = M_A_CANT_COMPRESS shr 16;      // Can't compress this file
  //faATTR_ARCHIVE       = $40000000;                   // clib only: Entry has had an EA modified,
                                                        // an ownerID changed, or trustee
                                                        // info changed, etc.
  faSetNetwareAttrs    = M_A_BITS_SIGNIFICANT;          // if this is set, netware flags are changed also



implementation

  uses
    sysconst;

{$DEFINE FPC_FEXPAND_DRIVES}
{$DEFINE FPC_FEXPAND_VOLUMES}
{$DEFINE FPC_FEXPAND_NO_DEFAULT_PATHS}

{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_ANSISTR_FILEUTIL_IMPL}
{ OS has an ansistring/single byte environment variable API }
{$define SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}

{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_ANSISTR_FILEUTIL_IMPL}

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
  FileOpen := Fpopen (pchar(SystemFileName),NWOpenFlags);

  //!! We need to set locking based on Mode !!
end;


Function FileCreate (Const FileName : RawByteString) : THandle;
var SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  FileCreate:=Fpopen(Pchar(SystemFileName),O_RdWr or O_Creat or O_Trunc or O_Binary);
  if FileCreate >= 0 then
    FileSetAttr (Filename, 0);  // dont know why but open always sets ReadOnly flag
end;

Function FileCreate (Const FileName : RawByteString; rights:longint) : THandle;
begin
  FileCreate:=FileCreate (FileName);
end;


Function FileCreate (Const FileName : RawByteString; ShareMode:longint; rights : longint) : THandle;
begin
  FileCreate:=FileCreate (FileName);
end;


Function FileRead (Handle : THandle; Out Buffer; Count : longint) : Longint;
begin
  FileRead:=libc.fpread (Handle,@Buffer,Count);
end;


Function FileWrite (Handle : THandle; const Buffer; Count : Longint) : Longint;
begin
  FileWrite:=libc.fpwrite (Handle,@Buffer,Count);
end;


Function FileSeek (Handle : THandle; FOffset,Origin : Longint) : Longint;
begin
  FileSeek:=libc.fplseek (Handle,FOffset,Origin);
end;


Function FileSeek (Handle : THandle; FOffset: Int64; Origin: Longint) : Int64;
begin
  FileSeek:=libc.fplseek64 (Handle,FOffset,Origin);
end;


Procedure FileClose (Handle : THandle);
begin
  libc.fpclose(Handle);
end;

Function FileTruncate (Handle : THandle; Size: Int64) : boolean;
begin
  if Size > high (longint) then
   FileTruncate := false
{$WARNING Possible support for 64-bit FS to be checked!}
  else
   FileTruncate:=(libc.fpchsize(Handle,Size) = 0);
end;

Function FileLock (Handle : THandle; FOffset,FLen : Longint) : Longint;
begin
  {$warning FileLock not implemented}
  //FileLock := _lock (Handle,FOffset,FLen);
  FileLock := -1;
end;

Function FileLock (Handle : THandle; FOffset,FLen : Int64) : Longint;
begin
  {$warning need to add 64bit FileLock call }
  //FileLock := FileLock (Handle, longint(FOffset),longint(FLen));
  FileLock := -1;
end;

Function FileUnlock (Handle : THandle; FOffset,FLen : Longint) : Longint;
begin
  //FileUnlock := _unlock (Handle,FOffset,FLen);
  {$warning FileUnLock not implemented}
  FileUnlock := -1;
end;

Function FileUnlock (Handle : THandle; FOffset,FLen : Int64) : Longint;
begin
  {$warning need to add 64bit FileUnlock call }
  //FileUnlock := FileUnlock (Handle, longint(FOffset),longint(FLen));
  FileUnlock := -1;
end;

Function FileAge (Const FileName : RawByteString): Int64;
var Info : TStat;
    TM  : TTM;
    SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  If Fpstat (pchar(SystemFileName),Info) <> 0 then
    exit(-1)
  else
    begin
      localtime_r (Info.st_mtim.tv_sec,tm);
      with TM do
        result:=DateTimeToFileDate(EncodeDate(tm_year+1900,tm_mon+1,tm_mday)+EncodeTime(tm_hour,tm_min,tm_sec,0));
    end;
end;


function FileGetSymLinkTarget(const FileName: RawByteString; out SymLinkRec: TRawbyteSymLinkRec): Boolean;
begin
  Result := False;
end;


Function FileExists (Const FileName : RawByteString; FollowLink : Boolean) : Boolean;
VAR Info : TStat;
    SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  FileExists:=(Fpstat(pchar(SystemFileName),Info) = 0);
end;


Function UnixToWinAge(UnixAge : time_t): Longint;
Var tm : TTm;
begin
  libc.localtime_r (UnixAge, tm);
  with tm do
    Result:=DateTimeToFileDate(EncodeDate(tm_year+1900,tm_mon+1,tm_mday)+EncodeTime(tm_hour,tm_min,tm_sec,0));
end;


{returns true if attributes match}
function find_setfields (var f : TsearchRec; var AttrsOk : boolean; var Name : RawByteString) : longint;
var
  StatBuf : TStat;
  fname   : RawByteString;
begin
  result := 0;
  with F do
  begin
    if FindData.Magic = $AD02 then
    begin
      attr := (Pdirent(FindData.EntryP)^.d_mode shr 16) and $ffff;
      size := Pdirent(FindData.EntryP)^.d_size;
      name := Pdirent(FindData.EntryP)^.d_name;
      SetCodePage(name, DefaultFileSystemCodePage, False);
      fname := FindData._dir + name;
      if Fpstat (pchar(fname),StatBuf) = 0 then
        time := UnixToWinAge (StatBuf.st_mtim.tv_sec)
      else
        time := 0;
      AttrsOk := false;
      if (f.FindData._attr and faHidden) = 0 then
        if attr and faHidden > 0 then exit;
      if (f.FindData._attr and faDirectory) = 0 then
        if attr and faDirectory > 0 then exit;
      if (f.FindData._attr and faSysFile) = 0 then
        if attr and faSysFile > 0 then exit;
      AttrsOk := true;
    end else
    begin
      name :='';
      result := 18;
    end;
  end;
end;

Function InternalFindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name: RawByteString) : Longint;
var
  SystemEncodedPath: RawByteString;
  path0 : string;
  p     : longint;
begin
  IF path = '' then
  begin
    result := 18;
    exit;
  end;
  SystemEncodedPath := ToSingleByteEncodedFileName(Path);
  Rslt.FindData._attr := attr;
  p := length (SystemEncodedPath);
  while (p > 0) and (not (SystemEncodedPath[p] in AllowDirectorySeparators)) do
    dec (p);
  if p > 0 then
  begin
    Rslt.FindData._mask := copy (SystemEncodedPath,p+1,high (longint));
    Rslt.FindData._dir := copy (SystemEncodedPath,1,p);
  end else
  begin
    Rslt.FindData._mask := SystemEncodedPath;
    Rslt.FindData._dir := GetCurrentDir;
    if (Rslt.FindData._dir[length(Rslt.FindData._dir)] <> '/') and
       (Rslt.FindData._dir[length(Rslt.FindData._dir)] <> '\') then
      Rslt.FindData._dir := Rslt.FindData._dir + '/';
  end;
  if Rslt.FindData._mask = '*' then Rslt.FindData._mask := '';
  if Rslt.FindData._mask = '*.*' then Rslt.FindData._mask := '';
  //writeln (stderr,'mask: "',Rslt._mask,'" dir:"',path0,'"');
  Pdirent(Rslt.FindData.DirP) := opendir (pchar(Rslt.FindData._dir));
  if Rslt.FindData.DirP = nil then
    result := 18
  else begin
    Rslt.FindData.Magic := $AD02;
    result := findnext (Rslt);
  end;
end;


Function InternalFindNext (var Rslt : TAbstractSearchRec; var Name : RawByteString) : Longint;
var attrsOk : boolean;
begin
  if Rslt.FindData.Magic <> $AD02 then
  begin
    result := 18;
    exit;
  end;
  result:=0;
  repeat
    Pdirent(Rslt.FindData.EntryP) := readdir (Pdirent(Rslt.FindData.DirP));
    if Rslt.FindData.EntryP = nil then
      result := 18
    else
    result := find_setfields (Rslt,attrsOk,Name);
    if (result = 0) and (attrsOk) then
    begin
      if Rslt.FindData._mask = #0 then exit;
      if fnmatch(@Rslt.FindData._mask[1],Pdirent(Rslt.FindData.EntryP)^.d_name,FNM_CASEFOLD) = 0 then
        exit;
    end;
  until result <> 0;
end;


Procedure InternalFindClose (var Handle: THandle; var FindData: TFindData);
begin
  if FindData.Magic <> $AD02 then exit;
  doserror:=0;
  closedir (Pdirent(FindData.DirP));
  FillChar (FindData,sizeof(FindData),0);
end;



Function FileGetDate (Handle : THandle) : Int64;
Var Info : TStat;
    _PTM : PTM;
begin
  If Fpfstat(Handle,Info) <> 0 then
    Result:=-1
  else
    begin
      _PTM := localtime (Info.st_mtim.tv_sec);
      IF _PTM = NIL THEN
        exit(-1)
      else
        with _PTM^ do
          Result:=DateTimeToFileDate(EncodeDate(tm_year+1900,tm_mon+1,tm_mday)+EncodeTime(tm_hour,tm_min,tm_sec,0));
    end;
end;


Function FileSetDate (Handle : THandle; Age : Longint) : Longint;
Begin
  {dont know how to do that, utime needs filename}
  result := -1;
end;


Function FileGetAttr (Const FileName : RawByteString) : Longint;
Var Info : TStat;
    SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  If Fpstat (pchar(SystemFileName),Info) <> 0 then
    Result:=-1
  Else
    Result := (Info.st_mode shr 16) and $ffff;
end;


Function FileSetAttr (Const Filename : RawByteString; Attr: longint) : Longint;
var
  StatBuf : TStat;
  newMode : longint;
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  if Fpstat (pchar(SystemFilename),StatBuf) = 0 then
  begin
    {what should i do here ?
     only support sysutils-standard attributes or also support the extensions defined
     only for netware libc ?
     For now i allow the complete attributes if the bit faSetNetwareAttrs is set. Otherwise
     only the standard attributes can be modified}
    if attr and faSetNetwareAttrs > 0 then
    begin
      newmode := ((attr shl 16) and $ffff0000) or M_A_BITS_SIGNIFICANT;
    end else
    begin
      attr := (attr and $2f) shl 16;
      newmode := StatBuf.st_mode and ($ffff0000-M_A_RDONLY-M_A_HIDDEN- M_A_SYSTEM-M_A_SUBDIR-M_A_ARCH);
      newmode := newmode or (attr shl 16) or M_A_BITS_SIGNIFICANT;
    end;
    if Fpchmod (pchar(SystemFilename),newMode) < 0 then
      result := ___errno^ else
      result := 0;
  end else
    result := ___errno^;
end;


Function DeleteFile (Const FileName : RawByteString) : Boolean;
var
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  Result:= (libc.UnLink (pchar(SystemFileName)) = 0);
end;


Function RenameFile (Const OldName, NewName : String) : Boolean;
var
  OldSystemFileName, NewSystemFileName: RawByteString;
begin
  OldSystemFileName:=ToSingleByteFileSystemEncodedFileName(OldName);
  NewSystemFileName:=ToSingleByteFileSystemEncodedFileName(NewName);
  RenameFile:=(libc.rename(pchar(OldSystemFileName),pchar(NewSystemFileName)) = 0);
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
//var fs : Tstatfs;
Begin
{  if ((Drive<4) and (not (fixdrivestr[Drive]=nil)) and fsstat(StrPas(fixdrivestr[drive]),fs)) or
     ((not (drivestr[Drive]=nil)) and fsstat(StrPas(drivestr[drive]),fs)) then
   Diskfree:=int64(fs.bavail)*int64(fs.bsize)
  else
   Diskfree:=-1;}
  DiskFree := -1;
  ConsolePrintf ('warning: fpc sysutils.diskfree not implemented'#13#10);
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
  ConsolePrintf ('warning: fpc sysutils.disksize not implemented'#13#10);
  {$warning DiskSize not implemented (does it make sense ?) }
End;


function DirectoryExists (const Directory: RawByteString; FollowLink : Boolean): boolean;
var
  Info : TStat;
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Directory);
  If Fpstat (pchar(SystemFileName),Info) <> 0 then
    exit(false)
  else
    Exit ((Info.st_mode and M_A_SUBDIR) <> 0);
end;


{****************************************************************************
                              Misc Functions
****************************************************************************}

procedure SysBeep;
begin
  RingBell;
end;


{****************************************************************************
                              Locale Functions
****************************************************************************}

Procedure GetLocalTime(var SystemTime: TSystemTime);
var t : TTime_t;
    tm: Ttm;
begin
  libc.time(t);
  libc.localtime_r(t,tm);
  with SystemTime do
  begin
    Hour := tm.tm_hour;
    Minute := tm.tm_min;
    Second := tm.tm_sec;
    MilliSecond := 0;
    Day := tm.tm_mday;
    Month := tm.tm_mon+1;
    Year := tm.tm_year+1900;
  end;
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
  Result:='';  // only found perror that prints the message
end;

{****************************************************************************
                              OS utility functions
****************************************************************************}

Function GetEnvironmentVariable(Const EnvVar : String) : String;

begin
  Result:=libc.getenv(PChar(EnvVar));
end;

Function GetEnvironmentVariableCount : Integer;

begin
  Result:=FPCCountEnvVar(EnvP);
end;

Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};

begin
  Result:=FPCGetEnvStrFromP(Envp,Index);
end;


function ExecuteProcess(Const Path: AnsiString; Const ComLine: AnsiString;Flags:TExecuteFlags=[]):integer;
var
  params:array of AnsiString;
  count,i: longint;
  Buf  : pchar;
  p    : pchar;
  CLine: AnsiString;
begin
  cLine := ComLine;
  buf:=pchar(CLine);
  count:=0;
  while(buf^<>#0) do
  begin
    while (buf^ in [' ',#9,#10]) do
      inc(buf);
    inc(count);
    while not (buf^ in [' ',#0,#9,#10]) do
      inc(buf);
  end;
  i := 0;
  setlength(params,count);
  buf:=pchar(CLine);
  while(buf^<>#0) do
  begin
    while (buf^ in [' ',#9,#10]) do
      inc(buf);
    p := buf;
    while not (buf^ in [' ',#0,#9,#10]) do
      inc(buf);
    if buf^ <> #0 then
    begin
      buf^ := #0;
      inc(buf);
    end;
    params[i]:=p;
    inc(i);
  end;
  result := ExecuteProcess (Path, params);
end;

Function GetLastOSError : Integer;

begin
  Result:=Integer(GetLastError);
end;


{******************************************************************************
                               --- Exec ---
******************************************************************************}

const maxargs=256;
function ExecuteProcess (const Path: AnsiString;
                                  const ComLine: array of AnsiString;Flags:TExecuteFlags=[]): integer;
var c : comstr;
    i : integer;
    args : array[0..maxargs+1] of pchar;
    arg0 : string;
    numargs,wstat : integer;
    Wiring : TWiring;
    newPath : string;
    e : EOSError;
begin
  if pos ('.',path) = 0 then
    arg0 := fexpand(path+'.nlm')
  else
    arg0 := fexpand (path);
  args[0] := pchar(arg0);
  numargs := 0;
  for I := 0 to High (ComLine) do
    if numargs < maxargs then
    begin
      inc(numargs);
      args[numargs] := pchar(ComLine[i]);
    end;
  args[numargs+1] := nil;
  Wiring.infd := StdInputHandle;  //textrec(Stdin).Handle;
  Wiring.outfd:= textrec(stdout).Handle;
  Wiring.errfd:= textrec(stderr).Handle;
  i := procve(args[0],
              PROC_CURRENT_SPACE+PROC_INHERIT_CWD,
              envP,         // const char * env[] If passed as NULL, the child process inherits the parent.s environment at the time of the call.
              @Wiring,      // wiring_t *wiring, Pass NULL to inherit system defaults for wiring.
              nil,          // struct fd_set *fds, Not currently implemented. Pass in NULL.
              nil,          // void *appdata, Not currently implemented. Pass in NULL.
              0,            // size_t appdata_size, Not currently implemented. Pass in 0
              nil,          // void *reserved, Reserved. Pass NULL.
              @args);       // const char *argv[]
  if i <> -1 then
  begin
    Fpwaitpid(i,@wstat,0);
    result := wstat;
  end else
  begin
    e:=EOSError.CreateFmt(SExecuteProcessFailed,[arg0,___errno^]);
    e.ErrorCode:=___errno^;
    raise e;
  end;
end;


procedure Sleep(milliseconds: Cardinal);
begin
  libc._delay (milliseconds);
end;



{****************************************************************************
                              Initialization code
****************************************************************************}

Initialization
  InitExceptions;       { Initialize exceptions. OS independent }
  InitInternational;    { Initialize internationalization settings }
Finalization
  FreeTerminateProcs;
  DoneExceptions;
end.
