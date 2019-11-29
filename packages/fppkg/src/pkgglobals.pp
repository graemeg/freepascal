{$mode objfpc}
{$h+}
unit pkgglobals;

interface

uses
{$ifdef unix}
  baseunix,
{$endif}
  SysUtils,
  Classes,
  fpmkunit;

Const
{$ifdef unix}
  ExeExt = '';
  AllFiles='*';
{$else unix}
  ExeExt = '.exe';
  AllFiles='*.*';
{$endif unix}

Type
  TFPMKUnitDep=record
    package    : string[64];
    reqver     : string[8];
    undef      : string[32];
    def        : string[32];
    PluginUnit : string[64];
    available  : boolean;
  end;

Const
  CmdLinePackageName='<cmdline>';
  CurrentDirPackageName='<currentdir>';
  URLPackageName='<url>';

  // Dependencies for compiling the fpmkunit unit
  FPMKUnitDepDefaultCount=5;
  FPMKUnitDepsDefaults : array[0..FPMKUnitDepDefaultCount-1] of TFPMKUnitDep = (
    (package: 'hash';
     reqver : '2.2.2';
     undef  : 'NO_UNIT_ZIPPER'),
    (package: 'paszlib';
     reqver : '2.2.2';
     undef  : 'NO_UNIT_ZIPPER'),
    (package: 'fcl-process';
     reqver : '2.2.2';
     undef  : 'NO_UNIT_PROCESS'),
   (package: 'libtar';
    reqver : '2.7.1';
    undef  : 'NO_TAR_SUPPORT'),
    (package: 'fpmkunit';
     reqver : '2.2.2-1';
     undef  : '')
  );

Type
  TLogLevel = (llError,llWarning,llInfo,llCommands,llDebug,llProgress);
  TLogLevels = Set of TLogLevel;
  TLogProc = procedure(Level:TLogLevel;Const Msg: String);

const
  DefaultLogLevels = [llError,llWarning, llProgress];
  AllLogLevels = [llError,llWarning,llCommands,llInfo,llProgress];

type
  EPackagerError = class(Exception);
  TPkgErrorProc = Procedure(Const Msg : String);

// Logging
Function StringToLogLevels (S : String) : TLogLevels;
Function LogLevelsToString (V : TLogLevels): String;
Procedure log(Level:TLogLevel; Const Fmt:String; const Args:array of const);
Procedure log(Level:TLogLevel; Const Msg:String);
Procedure Error(Const Fmt : String; const Args : array of const);
Procedure Error(Const Msg : String);

// Utils
function maybequoted(const s:string):string;
Function FixPath(const S : String) : string; inline; deprecated 'Use fpmkunit.FixPath instead';
Function DirectoryExistsLog(const ADir:string):Boolean;
Function FileExistsLog(const AFileName:string):Boolean;
procedure BackupFile(const AFileName: String);
Procedure DeleteDir(const ADir:string);
Procedure SearchFiles(SL:TStringList;const APattern:string);
Function GetCompilerInfo(const ACompiler,AOptions:string):string; overload;
Procedure GetCompilerInfo(const ACompiler, AOptions: string; out AVersion: string; out ACPU: TCpu; out aOS:TOS); overload;
function IsSuperUser:boolean;
procedure ReadIniFile(Const AFileName: String;L:TStrings);

var
  LogLevels : TLogLevels;
  FPMKUnitDeps : array of TFPMKUnitDep;
  LogHandler: TLogProc;
  ErrorHandler: TPkgErrorProc;

function GetFppkgConfigFile(Global : Boolean; SubDir : Boolean): string;
function GetFppkgConfigDir(Global : Boolean): string;


Implementation

// define use_shell to use sysutils.executeprocess
//  as alternate to using 'process' in getcompilerinfo
{$IF defined(GO32v2) or defined(WATCOM) or defined(OS2)}
 {$DEFINE USE_SHELL}
{$ENDIF GO32v2 or WATCOM or OS2}


uses
  typinfo,
{$IFNDEF USE_SHELL}
  process,
{$ENDIF USE_SHELL}
  contnrs,
  uriparser,
  pkgmessages;


function FPPkgGetVendorName:string;
begin
{$ifdef unix}
  result:='fpc';
{$else}
  result:='FreePascal'
{$endif}
end;


function FPPkgGetApplicationName:string;
begin
  result:='fppkg';
end;



function StringToLogLevels(S: String): TLogLevels;
Var
  I : integer;
begin
  I:=GetEnumValue(TypeInfo(TLogLevels),'v'+S);
  If (I<>-1) then
    Result:=TLogLevels(I)
  else
    Raise EPackagerError.CreateFmt(SErrInvalidLogLevels,[S]);
end;


Function LogLevelsToString (V : TLogLevels): String;
begin
  Result:=GetEnumName(TypeInfo(TLogLevels),Integer(V));
  Delete(Result,1,1);// Delete 'v'
end;


procedure LogCmd(Level:TLogLevel; Const Msg: String);
var
  Prefix : string;
begin
  if not(Level in LogLevels) then
    exit;
  Prefix:='';
  case Level of
    llWarning :
      Prefix:=SWarning;
    llError :
      Prefix:=SError;
{    llInfo :
      Prefix:='I: ';
    llCommands :
      Prefix:='C: ';
    llDebug :
      Prefix:='D: '; }
  end;
  Writeln(stdOut,Prefix,Msg);
end;


procedure ErrorCmd(Const Msg: String);
begin
  Raise EPackagerError.Create(Msg);
end;


Procedure log(Level:TLogLevel; Const Msg : String);
begin
  Loghandler(level,Msg)
end;

Procedure log(Level:TLogLevel; Const Fmt:String; const Args:array of const);
begin
  LogHandler(Level,Format(Fmt,Args));
end;

Procedure Error(Const Msg : String);
begin
  ErrorHandler(Msg)
end;

procedure Error(Const Fmt: String; const Args: array of const);
begin
  ErrorHandler(Format(Fmt, Args));
end;


function maybequoted(const s:string):string;
const
  {$IFDEF MSWINDOWS}
    FORBIDDEN_CHARS = ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
                       '{', '}', '''', '`', '~'];
  {$ELSE}
    FORBIDDEN_CHARS = ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
                       '{', '}', '''', ':', '\', '`', '~'];
  {$ENDIF}
var
  s1 : string;
  i  : integer;
  quoted : boolean;
begin
  quoted:=false;
  s1:='"';
  for i:=1 to length(s) do
   begin
     case s[i] of
       '"' :
         begin
           quoted:=true;
           s1:=s1+'\"';
         end;
       ' ',
       #128..#255 :
         begin
           quoted:=true;
           s1:=s1+s[i];
         end;
       else begin
         if s[i] in FORBIDDEN_CHARS then
           quoted:=True;
         s1:=s1+s[i];
       end;
     end;
   end;
  if quoted then
    maybequoted:=s1+'"'
  else
    maybequoted:=s;
end;


Function FixPath(const S : String) : string;
begin
  Result:=fpmkunit.FixPath(S, True);
end;


Function DirectoryExistsLog(const ADir:string):Boolean;
begin
  result:=SysUtils.DirectoryExists(ADir);
  if result then
    log(llDebug,SDbgDirectoryExists,[ADir,SDbgFound])
  else
    log(llDebug,SDbgDirectoryExists,[ADir,SDbgNotFound]);
end;


Function FileExistsLog(const AFileName:string):Boolean;
begin
  result:=SysUtils.FileExists(AFileName);
  if result then
    log(llDebug,SDbgFileExists,[AFileName,SDbgFound])
  else
    log(llDebug,SDbgFileExists,[AFileName,SDbgNotFound]);
end;


procedure BackupFile(const AFileName: String);
Var
  BFN : String;
begin
  BFN:=AFileName+'.bak';
  log(llDebug,SDbgBackupFile,[BFN]);
  If not RenameFile(AFileName,BFN) then
    Error(SErrBackupFailed,[AFileName,BFN]);
end;


Procedure DeleteDir(const ADir:string);
var
  Info : TSearchRec;
begin
  // Prevent accidently deleting all files in current or root dir
  if (ADir='') or (ADir=PathDelim) then
    exit;
  if FindFirst(ADir+PathDelim+AllFiles,faAnyFile, Info)=0 then
    try
      repeat
        if (Info.Attr and faDirectory)=faDirectory then
          begin
            if (Info.Name<>'.') and (Info.Name<>'..') then
              DeleteDir(ADir+PathDelim+Info.Name)
          end
        else
          DeleteFile(ADir+PathDelim+Info.Name);
      until FindNext(Info)<>0;
    finally
      FindClose(Info);
    end;
  RemoveDir(Adir);
end;


Procedure SearchFiles(SL:TStringList;const APattern:string);
var
  Info : TSearchRec;
  ADir : string;
begin
  ADir:=ExtractFilePath(APattern);
  if FindFirst(APattern,faAnyFile, Info)=0 then
    try
      repeat
        if (Info.Attr and faDirectory)=faDirectory then
          begin
            if (Info.Name<>'.') and (Info.Name<>'..') then
              SearchFiles(SL,ADir+Info.Name+PathDelim+ExtractFileName(APattern))
          end;
        SL.Add(ADir+Info.Name);
      until FindNext(Info)<>0;
    finally
      FindClose(Info);
    end;
end;


//
// if use_shell defined uses sysutils.executeprocess else uses 'process'
//
function GetCompilerInfo(const ACompiler,AOptions:string):string;
const
  BufSize = 1024;
var
{$IFDEF USE_SHELL}
  TmpFileName, ProcIDStr: shortstring;
  TmpFile: file;
  CmdLine2: string;
{$ELSE USE_SHELL}
  S: TProcess;
{$ENDIF USE_SHELL}
  Buf: array [0..BufSize - 1] of char;
  Count: longint;
begin
{$IFDEF USE_SHELL}
  Str (GetProcessID, ProcIDStr);
  TmpFileName := GetEnvironmentVariable ('TEMP');
  if TmpFileName <> '' then
   TmpFileName := TmpFileName + DirectorySeparator + 'fppkgout.' + ProcIDStr
  else
   TmpfileName := 'fppkgout.' + ProcIDStr;
  CmdLine2 := '/C ' + ACompiler + ' ' + AOptions + ' > ' + TmpFileName;
  SysUtils.ExecuteProcess (GetEnvironmentVariable ('COMSPEC'), CmdLine2);
  Assign (TmpFile, TmpFileName);
  Reset (TmpFile, 1);
  BlockRead (TmpFile, Buf, BufSize, Count);
  Close (TmpFile);
{$ELSE USE_SHELL}
  S:=TProcess.Create(Nil);
  S.Commandline:=ACompiler+' '+AOptions;
  S.ShowWindow:=swoHIDE;
  S.Options:=[poUsePipes];
  S.execute;
  Count:=s.output.read(buf,BufSize);
  S.Free;
{$ENDIF USE_SHELL}
  SetLength(Result,Count);
  Move(Buf,Result[1],Count);
end;


Procedure GetCompilerInfo(const ACompiler, AOptions: string; out AVersion: string; out ACPU: TCpu; out aOS:TOS); overload;
var
  infosl: TStringList;
begin
  infosl:=TStringList.Create;
  try
    infosl.Delimiter:=' ';
    infosl.DelimitedText:=GetCompilerInfo(ACompiler,AOptions);
    if infosl.Count<>3 then
      Raise EPackagerError.Create(SErrInvalidFPCInfo);
    AVersion:=infosl[0];
    ACPU:=StringToCPU(infosl[1]);
    AOS:=StringToOS(infosl[2]);
  finally
    infosl.Free;
  end;
end;

function IsSuperUser:boolean;
begin
{$ifdef unix}
  result:=(fpGetUID=0);
{$else unix}
  result:=false;
{$endif unix}
end;

procedure ReadIniFile(Const AFileName: String;L:TStrings);
Var
  F : TFileStream;
  Line : String;
  I,P,PC : Integer;
begin
  F:=TFileStream.Create(AFileName,fmOpenRead);
  Try
    L.LoadFromStream(F);
    // Fix lines.
    For I:=L.Count-1 downto 0 do
      begin
        Line:=L[I];
        P:=Pos('=',Line);
        PC:=Pos(';',Line);  // Comment line.
        If (P=0) or ((PC<>0) and (PC<P)) then
          L.Delete(I)
        else
          L[i]:=Trim(System.Copy(Line,1,P-1)+'='+Trim(System.Copy(Line,P+1,Length(Line)-P)));
      end;
  Finally
    F.Free;
  end;
end;

function GetFppkgConfigFile(Global : Boolean; SubDir : Boolean): string;
var
  StoredOnGetApplicationName: TGetAppNameEvent;
  StoredOnGetVendorName: TGetVendorNameEvent;
begin
  StoredOnGetApplicationName := OnGetApplicationName;
  StoredOnGetVendorName := OnGetVendorName;
  try
    OnGetApplicationName := @FPPkgGetApplicationName;
    OnGetVendorName := @FPPkgGetVendorName;
    result := GetAppConfigFile(Global, SubDir);
  finally
    OnGetApplicationName := StoredOnGetApplicationName;
    OnGetVendorName := StoredOnGetVendorName;
  end;
end;

function GetFppkgConfigDir(Global : Boolean): string;
var
  StoredOnGetApplicationName: TGetAppNameEvent;
  StoredOnGetVendorName: TGetVendorNameEvent;
begin
  StoredOnGetApplicationName := OnGetApplicationName;
  StoredOnGetVendorName := OnGetVendorName;
  try
    OnGetApplicationName := @FPPkgGetApplicationName;
    OnGetVendorName := @FPPkgGetVendorName;
    result := GetAppConfigDir(Global);
  finally
    OnGetApplicationName := StoredOnGetApplicationName;
    OnGetVendorName := StoredOnGetVendorName;
  end;
end;


initialization
  LogHandler := @LogCmd;
  ErrorHandler := @ErrorCmd;
end.
