{

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by Florian Klaempfl and Yury Sidorov
    members of the Free Pascal development team

    Sysutils unit for wince

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

uses
  dos,
  windows;

{$DEFINE HAS_SLEEP}
{$DEFINE HAS_OSERROR}
{$DEFINE HAS_OSCONFIG}
{$DEFINE HAS_TEMPDIR}
{$DEFINE HAS_LOCALTIMEZONEOFFSET}

{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_UNICODESTR_FILEUTIL_IMPL}
{ OS has an ansistring/single byte environment variable API (it has a dummy
  one currently, but that one uses ansistring) }
{$define SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}

{ Include platform independent interface part }
{$i sysutilh.inc}

type
  TSystemTime = Windows.TSystemTime;

  EWinCEError = class(Exception)
  public
    ErrorCode : DWORD;
  end;


Var
  WinCEPlatform : Longint;
  WinCEMajorVersion,
  WinCEMinorVersion,
  WinCEBuildNumber   : dword;
  WinCECSDVersion    : ShortString;   // CSD record is 128 bytes only?


implementation

  uses
    sysconst;

{$DEFINE FPC_NOGENERICANSIROUTINES}
{$define HASEXPANDUNCFILENAME}

{$DEFINE FPC_FEXPAND_NO_DEFAULT_PATHS}
{$DEFINE FPC_FEXPAND_UNC} (* UNC paths are supported *)

{ Include platform independent implementation part }
{$i sysutils.inc}

procedure PWideCharToString(const str: PWideChar; out Result: string; strlen: longint = -1);
var
  len: longint;
begin
  if (strlen < 1) and (str^ = #0) then
    Result:=''
  else
  begin
    while True do begin
      if strlen <> -1 then
        len:=strlen + 1
      else
        len:=WideToAnsiBuf(str, -1, nil, 0);
      if len > 0 then
      begin
        SetLength(Result, len - 1);
        if (WideToAnsiBuf(str, strlen, @Result[1], len) = 0) and (strlen <> -1) then
        begin
          strlen:=-1;
          continue;
        end;
      end
      else
        Result:='';
      break;
    end;
  end;
end;


function ExpandUNCFileName (const filename:rawbytestring) : rawbytestring;
var
  u: unicodestring;
begin
  u:=ExpandUNCFileName(unicodestring(filename));
  widestringmanager.Unicode2AnsiMoveProc(punicodechar(u),result,DefaultRTLFileSystemCodePage,length(u));
end;

function ExpandUNCFileName (const filename:unicodestring) : unicodestring;
{ returns empty string on errors }
var
  s    : unicodestring;
  size : dword;
  rc   : dword;
  buf  : pwidechar;
begin
  s := ExpandFileName (filename);

  size := max_path*SizeOf(WideChar);
  getmem(buf,size);

  try
    rc := WNetGetUniversalName (pwidechar(s), UNIVERSAL_NAME_INFO_LEVEL, buf, @size);

    if rc=ERROR_MORE_DATA then
      begin
        buf:=reallocmem(buf,size);
        rc := WNetGetUniversalName (pwidechar(s), UNIVERSAL_NAME_INFO_LEVEL, buf, @size);
      end;
    if rc = NO_ERROR then
      Result := PRemoteNameInfo(buf)^.lpUniversalName
    else if rc = ERROR_NOT_CONNECTED then
      Result := filename
    else
      Result := '';
  finally
    freemem(buf);
  end;
end;

{****************************************************************************
                              File Functions
****************************************************************************}

Function FileOpen (Const FileName : unicodestring; Mode : Integer) : THandle;
const
  AccessMode: array[0..2] of Cardinal  = (
    GENERIC_READ,
    GENERIC_WRITE,
    GENERIC_READ or GENERIC_WRITE);
  ShareMode: array[0..4] of Integer = (
               0,
               0,
               FILE_SHARE_READ,
               FILE_SHARE_WRITE,
               FILE_SHARE_READ or FILE_SHARE_WRITE);
begin
  result := CreateFile(PWideChar(FileName), dword(AccessMode[Mode and 3]),
                       dword(ShareMode[(Mode and $F0) shr 4]), nil, OPEN_EXISTING,
                       FILE_ATTRIBUTE_NORMAL, 0);
  //if fail api return feInvalidHandle (INVALIDE_HANDLE=feInvalidHandle=-1)
end;


Function FileCreate (Const FileName : UnicodeString) : THandle;
begin
  Result := CreateFile(PWideChar(FileName), GENERIC_READ or GENERIC_WRITE,
                       0, nil, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, 0);
end;


Function FileCreate (Const FileName : UnicodeString; Rights:longint) : THandle;
begin
  FileCreate:=FileCreate(FileName);
end;


Function FileCreate (Const FileName : UnicodeString; ShareMode:longint; Rights:longint) : THandle;
begin
  FileCreate:=FileCreate(FileName);
end;


Function FileRead (Handle : THandle; Out Buffer; Count : longint) : Longint;
Var
  res : dword;
begin
  if ReadFile(Handle, Buffer, Count, res, nil) then
   FileRead:=Res
  else
   FileRead:=-1;
end;


Function FileWrite (Handle : THandle; const Buffer; Count : Longint) : Longint;
Var
  Res : dword;
begin
  if WriteFile(Handle, Buffer, Count, Res, nil) then
   FileWrite:=Res
  else
   FileWrite:=-1;
end;


Function FileSeek (Handle : THandle;FOffset,Origin : Longint) : Longint;
begin
  Result := longint(SetFilePointer(Handle, FOffset, nil, Origin));
end;


Function FileSeek (Handle : THandle; FOffset: Int64; Origin: Longint) : Int64;
begin
  Result := SetFilePointer(Handle, longint(FOffset), nil, longint(Origin));
end;


Procedure FileClose (Handle : THandle);
begin
  if Handle<=4 then
   exit;
  CloseHandle(Handle);
end;


Function FileTruncate (Handle : THandle;Size: Int64) : boolean;
begin
  if FileSeek (Handle, Size, FILE_BEGIN) = Size then
   Result:=SetEndOfFile(handle)
  else
   Result := false;
end;


Function DosToWinTime (DTime:longint; out Wtime : TFileTime):longbool;
begin
  DosToWinTime:=dos.DosToWinTime(DTime, Wtime);
end;


Function WinToDosTime (Const Wtime : TFileTime; out DTime:longint):longbool;
begin
  WinToDosTime:=dos.WinToDosTime(Wtime, DTime);
end;


Function FileAge (Const FileName : UnicodeString): Int64;
var
  Handle: THandle;
  FindData: TWin32FindData;
  tmpdtime    : longint;
begin
  Handle := FindFirstFile(PWideChar(FileName), FindData);
  if Handle <> INVALID_HANDLE_VALUE then
    begin
      Windows.FindClose(Handle);
      if (FindData.dwFileAttributes and FILE_ATTRIBUTE_DIRECTORY) = 0 then
        If WinToDosTime(FindData.ftLastWriteTime,tmpdtime) then
          begin
            Result:=tmpdtime;
            exit;
          end;
    end;
  Result := -1;
end;


function FileGetSymLinkTarget(const FileName: UnicodeString; out SymLinkRec: TUnicodeSymLinkRec): Boolean;
begin
  Result := False;
end;


Function FileExists (Const FileName : UnicodeString; FollowLink : Boolean) : Boolean;
var
  Attr:Dword;
begin
  Attr:=FileGetAttr(FileName);
  if Attr <> $ffffffff then
    Result:= (Attr and FILE_ATTRIBUTE_DIRECTORY) = 0
  else
    Result:=False;
end;


Function DirectoryExists (Const Directory : UnicodeString; FollowLink : Boolean) : Boolean;
var
  Attr:Dword;
begin
  Attr:=FileGetAttr(Directory);
  if Attr <> $ffffffff then
    Result:= (Attr and FILE_ATTRIBUTE_DIRECTORY) <> 0
  else
    Result:=False;
end;


Function FindMatch(var f: TAbstractSearchRec; var Name: UnicodeString) : Longint;
var
  tmpdtime    : longint;
begin
  { Find file with correct attribute }
  While (F.FindData.dwFileAttributes and cardinal(F.ExcludeAttr))<>0 do
   begin
     if not FindNextFile (F.FindHandle,F.FindData) then
      begin
        Result:=GetLastError;
        exit;
      end;
   end;
  { Convert some attributes back }
  WinToDosTime(F.FindData.ftLastWriteTime,tmpdtime);
  F.Time:=tmpdtime;
  f.size:=F.FindData.NFileSizeLow;
  f.attr:=F.FindData.dwFileAttributes;
  Name:=F.FindData.cFileName;
  Result:=0;
end;


Function InternalFindFirst (Const Path : UnicodeString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name : UnicodeString) : Longint;
var
  fn: PWideChar;
begin
  fn:=PWideChar(Path);
  Name:=Path;
  Rslt.Attr:=attr;
  Rslt.ExcludeAttr:=(not Attr) and ($1e);
                 { $1e = faHidden or faSysFile or faVolumeID or faDirectory }
  { FindFirstFile is a WinCE Call }
  Rslt.FindHandle:=FindFirstFile (fn, Rslt.FindData);
  If Rslt.FindHandle=Invalid_Handle_value then
   begin
     Result:=GetLastError;
     exit;
   end;
  { Find file with correct attribute }
  Result:=FindMatch(Rslt, Name);
end;


Function InternalFindNext (Var Rslt : TAbstractSearchRec; var Name: UnicodeString) : Longint;
begin
  if FindNextFile(Rslt.FindHandle, Rslt.FindData) then
    Result := FindMatch(Rslt, Name)
  else
    Result := GetLastError;
end;


Procedure InternalFindClose (var Handle: THandle; var FindData: TFindData);
begin
   if Handle <> INVALID_HANDLE_VALUE then
     Windows.FindClose(Handle);
end;


Function FileGetDate (Handle : THandle) : Int64;
Var
  FT : TFileTime;
  tmpdtime : longint;
begin
  If GetFileTime(Handle,nil,nil,@ft) and
     WinToDosTime(FT, tmpdtime) then
     begin
       Result:=tmpdtime;       
       exit;
     end;
  Result:=-1;
end;


Function FileSetDate (Handle : THandle;Age : Int64) : Longint;
Var
  FT: TFileTime;
begin
  Result := 0;
  if DosToWinTime(Age, FT) and SetFileTime(Handle, FT, FT, FT) then
    Exit;
  Result := GetLastError;
end;


Function FileGetAttr (Const FileName : UnicodeString) : Longint;
var
  fn: PWideChar;
begin
  fn:=StringToPWideChar(FileName);
  Result:=GetFileAttributes(fn);
  FreeMem(fn);
end;


Function FileSetAttr (Const Filename : UnicodeString; Attr: longint) : Longint;
begin
  if not SetFileAttributes(PWideChar(FileName), Attr) then
    Result := GetLastError
  else
    Result:=0;
end;


Function DeleteFile (Const FileName : UnicodeString) : Boolean;
begin
  DeleteFile:=Windows.DeleteFile(PWideChar(FileName));
end;


Function RenameFile (Const OldName, NewName : UnicodeString) : Boolean;
begin
  Result := MoveFile(PWideChar(OldName), PWideChar(NewName));
end;


{****************************************************************************
                              Disk Functions
****************************************************************************}

function diskfree(drive : byte) : int64;
begin
  Result := Dos.diskfree(drive);
end;


function disksize(drive : byte) : int64;
begin
  Result := Dos.disksize(drive);
end;


{****************************************************************************
                              Time Functions
****************************************************************************}


Procedure GetLocalTime(var SystemTime: TSystemTime);
begin
  windows.Getlocaltime(SystemTime);
end;

function GetLocalTimeOffset: Integer;
var
  TZInfo: TTimeZoneInformation;
begin
   case GetTimeZoneInformation(TZInfo) of
     TIME_ZONE_ID_UNKNOWN:
       Result := TZInfo.Bias;
     TIME_ZONE_ID_STANDARD:
       Result := TZInfo.Bias + TZInfo.StandardBias;
     TIME_ZONE_ID_DAYLIGHT:
       Result := TZInfo.Bias + TZInfo.DaylightBias;
     else
       Result := 0;
   end;
end;

{****************************************************************************
                              Misc Functions
****************************************************************************}

procedure SysBeep;
begin
  MessageBeep(0);
end;


{****************************************************************************
                              Locale Functions
****************************************************************************}

function GetLocaleStr(LID, LT: Longint; const Def: string): ShortString;
var
  L: Integer;
  Buf: array[0..255] of WideChar;
  s: widestring;
begin
  L := GetLocaleInfo(LID, LT, Buf, SizeOf(Buf) div SizeOf(WideChar));
  if L > 0 then
  begin
    SetString(s, Buf, L - 1);
    Result:=s;
  end
  else
    Result := Def;
end;


function GetLocaleChar(LID, LT: Longint; Def: Char): Char;
var
  Buf: array[0..1] of WideChar;
  Buf2: array[0..1] of Char;
begin
  if GetLocaleInfo(LID, LT, Buf, 2) > 0 then
  begin
    WideToAnsiBuf(Buf, 1, Buf2, SizeOf(Buf2));
    Result := Buf2[0];
  end
  else
    Result := Def;
end;


Function GetLocaleInt(LID,TP,Def: LongInt): LongInt;
Var
  S: String;
  C: Integer;
Begin
  S:=GetLocaleStr(LID,TP,'0');
  Val(S,Result,C);
  If C<>0 Then
    Result:=Def;
End;


procedure GetFormatSettings;
var
  HF  : Shortstring;
  LID : LCID;
  I,Day,DateOrder : longint;
begin
  LID := GetUserDefaultLCID;
  { Date stuff }
  for I := 1 to 12 do
    begin
    ShortMonthNames[I]:=GetLocaleStr(LID,LOCALE_SABBREVMONTHNAME1+I-1,ShortMonthNames[i]);
    LongMonthNames[I]:=GetLocaleStr(LID,LOCALE_SMONTHNAME1+I-1,LongMonthNames[i]);
    end;
  for I := 1 to 7 do
    begin
    Day := (I + 5) mod 7;
    ShortDayNames[I]:=GetLocaleStr(LID,LOCALE_SABBREVDAYNAME1+Day,ShortDayNames[i]);
    LongDayNames[I]:=GetLocaleStr(LID,LOCALE_SDAYNAME1+Day,LongDayNames[i]);
    end;
  DateSeparator := GetLocaleChar(LID, LOCALE_SDATE, '/');
  DateOrder := GetLocaleInt(LID, LOCALE_IDate, 0);
  Case DateOrder Of
     1: Begin
        ShortDateFormat := 'dd/mm/yyyy';
        LongDateFormat := 'dddd, d. mmmm yyyy';
        End;
     2: Begin
        ShortDateFormat := 'yyyy/mm/dd';
        LongDateFormat := 'dddd, yyyy mmmm d.';
        End;
  else
    // Default american settings...
    ShortDateFormat := 'mm/dd/yyyy';
    LongDateFormat := 'dddd, mmmm d. yyyy';
  End;
  { Time stuff }
  TimeSeparator := GetLocaleChar(LID, LOCALE_STIME, ':');
  TimeAMString := GetLocaleStr(LID, LOCALE_S1159, 'AM');
  TimePMString := GetLocaleStr(LID, LOCALE_S2359, 'PM');
  if StrToIntDef(GetLocaleStr(LID, LOCALE_ITLZERO, '0'), 0) = 0 then
    HF:='h'
  else
    HF:='hh';
  // No support for 12 hour stuff at the moment...
  ShortTimeFormat := HF+':nn';
  LongTimeFormat := HF + ':nn:ss';
  { Currency stuff }
  CurrencyString:=GetLocaleStr(LID, LOCALE_SCURRENCY, '');
  CurrencyFormat:=StrToIntDef(GetLocaleStr(LID, LOCALE_ICURRENCY, '0'), 0);
  NegCurrFormat:=StrToIntDef(GetLocaleStr(LID, LOCALE_INEGCURR, '0'), 0);
  { Number stuff }
  ThousandSeparator:=GetLocaleChar(LID, LOCALE_STHOUSAND, ',');
  DecimalSeparator:=GetLocaleChar(LID, LOCALE_SDECIMAL, '.');
  CurrencyDecimals:=StrToIntDef(GetLocaleStr(LID, LOCALE_ICURRDIGITS, '0'), 0);
end;


Procedure InitInternational;
begin
  InitInternationalGeneric;
  SysLocale.MBCS:=GetSystemMetrics(SM_DBCSENABLED)<>0;
  SysLocale.RightToLeft:=GetSystemMetrics(SM_MIDEASTENABLED)<>0;
  GetFormatSettings;
end;


{****************************************************************************
                           Target Dependent
****************************************************************************}

function SysErrorMessage(ErrorCode: Integer): String;
var
  MsgBuffer: PWideChar;
  len: longint;
begin
  MsgBuffer:=nil;
  len:=FormatMessage(
         FORMAT_MESSAGE_ALLOCATE_BUFFER or FORMAT_MESSAGE_FROM_SYSTEM or FORMAT_MESSAGE_IGNORE_INSERTS,
         nil,
         ErrorCode,
         0,
         @MsgBuffer,    { This function allocs the memory (in this case you pass a PPwidechar)}
         0,
         nil);
         
  if MsgBuffer <> nil then begin
    while (len > 0) and (MsgBuffer[len - 1] <= #32) do
      Dec(len);
    MsgBuffer[len]:=#0;
    PWideCharToString(MsgBuffer, Result);
    LocalFree(HLOCAL(MsgBuffer));
  end
  else
    Result:='';
end;

{****************************************************************************
                              Initialization code
****************************************************************************}

// WinCE does not have environment. It can be emulated via registry or file. (YS)

Function GetEnvironmentVariable(Const EnvVar : String) : String;
begin
  Result := '';
end;

Function GetEnvironmentVariableCount : Integer;
begin
  Result := 0;
end;

Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};
begin
  Result := '';
end;


function ExecuteProcess(Const Path: RawByteString; Const ComLine: RawByteString;Flags:TExecuteFlags=[]):integer;
begin
  result:=ExecuteProcess(UnicodeString(Path),UnicodeString(ComLine),Flags);
end;

function ExecuteProcess(Const Path: UnicodeString; Const ComLine: UnicodeString;Flags:TExecuteFlags=[]):integer;
var
  PI: TProcessInformation;
  Proc : THandle;
  l    : DWord;
  e : EOSError;

begin
  DosError := 0;
  if not CreateProcess(PWideChar(Path), PWideChar(ComLine),
                       nil, nil, FALSE, 0, nil, nil, nil, PI) then
    begin
      e:=EOSError.CreateFmt(SExecuteProcessFailed,[Path,GetLastError]);
      e.ErrorCode:=GetLastError;
      raise e;
    end;
  Proc:=PI.hProcess;
  CloseHandle(PI.hThread);
  if WaitForSingleObject(Proc, dword($ffffffff)) <> $ffffffff then
    begin
      GetExitCodeProcess(Proc,l);
      CloseHandle(Proc);
      result:=l;
    end
  else
    begin
      e:=EOSError.CreateFmt(SExecuteProcessFailed,[Path,GetLastError]);
      e.ErrorCode:=GetLastError;
      CloseHandle(Proc);
      raise e;
    end;
end;

function ExecuteProcess(Const Path: UnicodeString; Const ComLine: Array of UnicodeString;Flags:TExecuteFlags=[]):integer;
 
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
  ExecuteProcess := ExecuteProcess (Path, CommandLine);
end;

function ExecuteProcess(Const Path: RawByteString; Const ComLine: Array of RawByteString;Flags:TExecuteFlags=[]):integer;

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
  ExecuteProcess := ExecuteProcess (UnicodeString(Path), CommandLine,Flags);
end;

Procedure Sleep(Milliseconds : Cardinal);

begin
  Windows.Sleep(MilliSeconds)
end;

Function GetLastOSError : Integer;

begin
  Result:=GetLastError;
end;

{****************************************************************************
                              Initialization code
****************************************************************************}

Procedure LoadVersionInfo;
Var
   versioninfo : TOSVERSIONINFO;
   i          : Integer;

begin
  versioninfo.dwOSVersionInfoSize:=sizeof(versioninfo);
  GetVersionEx(versioninfo);
  WinCEPlatform:=versionInfo.dwPlatformId;
  WinCEMajorVersion:=versionInfo.dwMajorVersion;
  WinCEMinorVersion:=versionInfo.dwMinorVersion;
  WinCEBuildNumber:=versionInfo.dwBuildNumber;
  i:=WideToAnsiBuf(@versioninfo.szCSDVersion[0], -1, @WinCECSDVersion[1], SizeOf(WinCECSDVersion) - 1);
  if i <> 0 then
    WinCECSDVersion[0]:=chr(i - 1);
end;

Function GetSpecialDir(ID: Integer) : String;

Var
  APath : array[0..MAX_PATH] of WideChar;
begin
  if SHGetSpecialFolderPath(0, APath, ID, True) then
  begin
    PWideCharToString(APath, Result);
    Result:=IncludeTrailingPathDelimiter(Result);
  end
  else
    Result:='';
end;

Function GetAppConfigDir(Global : Boolean) : String;

begin
  If Global then
    Result:=GetSpecialDir(CSIDL_WINDOWS)
  else
    Result:=GetSpecialDir(CSIDL_APPDATA);
  If (Result<>'') then
    begin
      if VendorName<>'' then
        Result:=IncludeTrailingPathDelimiter(Result+VendorName);
      Result:=IncludeTrailingPathDelimiter(Result+ApplicationName);
    end
  else
    Result:=IncludeTrailingPathDelimiter(DGetAppConfigDir(Global));
end;

Function GetAppConfigFile(Global : Boolean; SubDir : Boolean) : String;

begin
  result:=DGetAppConfigFile(Global,SubDir);
end;

Function GetTempDir(Global : Boolean) : String;
var
  buf: widestring;
begin
  SetLength(buf, MAX_PATH);
  SetLength(buf, GetTempPath(Length(buf) + 1, PWideChar(buf)));
  Result:=buf;
  Result := IncludeTrailingPathDelimiter(Result);
end;

{****************************************************************************
                    Target Dependent WideString stuff
****************************************************************************}


function DoCompareString(P1, P2: PWideChar; L1, L2: PtrUInt; Flags: DWORD): PtrInt;
begin
  SetLastError(0);
  Result:=CompareString(LOCALE_USER_DEFAULT,Flags,P1,L1,P2,L2)-2;
  if GetLastError<>0 then
    RaiseLastOSError;
end;


function WinCECompareWideString(const s1, s2 : WideString; Options : TCompareOptions) : PtrInt;
begin
  if coIgnoreCase in Options then
    Result:=DoCompareString(PWideChar(s1), PWideChar(s2), Length(s1), Length(s2), NORM_IGNORECASE)
  else  
    Result:=DoCompareString(PWideChar(s1), PWideChar(s2), Length(s1), Length(s2), 0);
end;


function WinCECompareTextWideString(const s1, s2 : WideString) : PtrInt;
begin
  Result:=DoCompareString(PWideChar(s1), PWideChar(s2), Length(s1), Length(s2), NORM_IGNORECASE);
end;


function WinCECompareUnicodeString(const s1, s2 : UnicodeString; Options : TCompareOptions) : PtrInt;
begin
   if coIgnoreCase in Options then
     Result:=DoCompareString(PWideChar(s1), PWideChar(s2), Length(s1), Length(s2), NORM_IGNORECASE)
   else  
     Result:=DoCompareString(PWideChar(s1), PWideChar(s2), Length(s1), Length(s2), 0);
end;


function WinCECompareTextUnicodeString(const s1, s2 : UnicodeString) : PtrInt;
begin
  Result:=DoCompareString(PWideChar(s1), PWideChar(s2), Length(s1), Length(s2), NORM_IGNORECASE);
end;


function WinCEAnsiUpperCase(const s: string): string;
var
  buf: PWideChar;
  len: longint;
begin
  if s <> '' then
  begin
    buf:=StringToPWideChar(s, @len);
    CharUpperBuff(buf, len-1);
    PWideCharToString(buf, Result, len-1);
    FreeMem(buf);
  end
  else
    Result:='';
end;


function WinCEAnsiLowerCase(const s: string): string;
var
  buf: PWideChar;
  len: longint;
begin
  if s <> '' then
  begin
    buf:=StringToPWideChar(s, @len);
    CharLowerBuff(buf, len-1);
    PWideCharToString(buf, Result, len-1);
    FreeMem(buf);
  end
  else
    Result:='';
end;


function WinCEAnsiCompareStr(const S1, S2: string): PtrInt;
var
  ws1, ws2: PWideChar;
begin
  ws1:=StringToPWideChar(S1);
  ws2:=StringToPWideChar(S2);
  Result:=CompareString(LOCALE_USER_DEFAULT, 0, ws1, Length(S1), ws2, Length(S2)) - 2;
  FreeMem(ws2);
  FreeMem(ws1);
end;


function WinCEAnsiCompareText(const S1, S2: string): PtrInt;
var
  ws1, ws2: PWideChar;
begin
  ws1:=StringToPWideChar(S1);
  ws2:=StringToPWideChar(S2);
  Result:=CompareString(LOCALE_USER_DEFAULT, NORM_IGNORECASE, ws1, Length(S1), ws2, Length(S2)) - 2;
  FreeMem(ws2);
  FreeMem(ws1);
end;

function WinCEAnsiStrComp(S1, S2: PChar): PtrInt;
var
  ws1, ws2: PWideChar;
begin
  ws1:=PCharToPWideChar(S1);
  ws2:=PCharToPWideChar(S2);
  Result:=CompareString(LOCALE_USER_DEFAULT, 0, ws1, -1, ws2, -1) - 2;
  FreeMem(ws2);
  FreeMem(ws1);
end;


function WinCEAnsiStrIComp(S1, S2: PChar): PtrInt;
var
  ws1, ws2: PWideChar;
begin
  ws1:=PCharToPWideChar(S1);
  ws2:=PCharToPWideChar(S2);
  Result:=CompareString(LOCALE_USER_DEFAULT, NORM_IGNORECASE, ws1, -1, ws2, -1) - 2;
  FreeMem(ws2);
  FreeMem(ws1);
end;


function WinCEAnsiStrLComp(S1, S2: PChar; MaxLen: PtrUInt): PtrInt;
var
  ws1, ws2: PWideChar;
  len1, len2: longint;
begin
  ws1:=PCharToPWideChar(S1, MaxLen, @len1);
  ws2:=PCharToPWideChar(S2, MaxLen, @len2);
  Result:=CompareString(LOCALE_USER_DEFAULT, 0, ws1, len1, ws2, len2) - 2;
  FreeMem(ws2);
  FreeMem(ws1);
end;


function WinCEAnsiStrLIComp(S1, S2: PChar; MaxLen: PtrUInt): PtrInt;
var
  ws1, ws2: PWideChar;
  len1, len2: longint;
begin
  ws1:=PCharToPWideChar(S1, MaxLen, @len1);
  ws2:=PCharToPWideChar(S2, MaxLen, @len2);
  Result:=CompareString(LOCALE_USER_DEFAULT, NORM_IGNORECASE, ws1, len1, ws2, len2) - 2;
  FreeMem(ws2);
  FreeMem(ws1);
end;


function WinCEAnsiStrLower(Str: PChar): PChar;
var
  buf: PWideChar;
  len: longint;
begin
  buf:=PCharToPWideChar(Str, -1, @len);
  CharLowerBuff(buf, len - 1);
  Result:=Str;
  WideToAnsiBuf(buf, -1, Result, StrLen(Str));
  FreeMem(buf);
end;


function WinCEAnsiStrUpper(Str: PChar): PChar;
var
  buf: PWideChar;
  len: longint;
begin
  buf:=PCharToPWideChar(Str, -1, @len);
  CharUpperBuff(buf, len - 1);
  Result:=Str;
  WideToAnsiBuf(buf, -1, Result, StrLen(Str));
  FreeMem(buf);
end;


{ there is a similiar procedure in the system unit which inits the fields which
  are relevant already for the system unit }
procedure InitWinCEWidestrings;
  begin
    widestringmanager.CompareWideStringProc:=@WinCECompareWideString;
    widestringmanager.CompareUnicodeStringProc:=@WinCECompareUnicodeString;

    widestringmanager.UpperAnsiStringProc:=@WinCEAnsiUpperCase;
    widestringmanager.LowerAnsiStringProc:=@WinCEAnsiLowerCase;
    widestringmanager.CompareStrAnsiStringProc:=@WinCEAnsiCompareStr;
    widestringmanager.CompareTextAnsiStringProc:=@WinCEAnsiCompareText;
    widestringmanager.StrCompAnsiStringProc:=@WinCEAnsiStrComp;
    widestringmanager.StrICompAnsiStringProc:=@WinCEAnsiStrIComp;
    widestringmanager.StrLCompAnsiStringProc:=@WinCEAnsiStrLComp;
    widestringmanager.StrLICompAnsiStringProc:=@WinCEAnsiStrLIComp;
    widestringmanager.StrLowerAnsiStringProc:=@WinCEAnsiStrLower;
    widestringmanager.StrUpperAnsiStringProc:=@WinCEAnsiStrUpper;
  end;



Initialization
  InitWinCEWidestrings;
  InitExceptions;       { Initialize exceptions. OS independent }
  InitInternational;    { Initialize internationalization settings }
  LoadVersionInfo;
  OnBeep:=@SysBeep;
  SysConfigDir:='\Windows';

Finalization
  FreeTerminateProcs;
  DoneExceptions;

end.
