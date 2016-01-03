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
  fprepos;

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
<<<<<<< HEAD
<<<<<<< HEAD
    package  : string[12];
    reqver   : string[8];
    undef    : string[32];
    def      : string[32];
    available: boolean;
=======
    package : string[12];
    reqver  : string[8];
    undef   : string[16];
>>>>>>> graemeg/fixes_2_2
=======
    package : string[12];
    reqver  : string[8];
    undef   : string[16];
>>>>>>> origin/fixes_2_2
  end;

Const
  CmdLinePackageName='<cmdline>';
  CurrentDirPackageName='<currentdir>';
<<<<<<< HEAD
<<<<<<< HEAD

  // Dependencies for compiling the fpmkunit unit
  FPMKUnitDepDefaultCount=4;
  FPMKUnitDepsDefaults : array[0..FPMKUnitDepDefaultCount-1] of TFPMKUnitDep = (
=======

  // Dependencies for compiling the fpmkunit unit
  FPMKUnitDepCount=4;
  FPMKUnitDeps : array[1..4] of TFPMKUnitDep = (
>>>>>>> origin/fixes_2_2
    (package: 'hash';
     reqver : '2.2.2';
     undef  : 'NO_UNIT_ZIPPER'),
    (package: 'paszlib';
     reqver : '2.2.2';
     undef  : 'NO_UNIT_ZIPPER'),
    (package: 'fcl-process';
     reqver : '2.2.2';
     undef  : 'NO_UNIT_PROCESS'),
    (package: 'fpmkunit';
     reqver : '2.2.2-1';
     undef  : '')
  );

Type
<<<<<<< HEAD
  TLogLevel = (vlError,vlWarning,vlInfo,vlCommands,vlDebug,vlProgres);
  TLogLevels = Set of TLogLevel;

const
  DefaultLogLevels = [vlError,vlWarning, vlProgres];
  AllLogLevels = [vlError,vlWarning,vlCommands,vlInfo];

=======

  // Dependencies for compiling the fpmkunit unit
  FPMKUnitDepCount=4;
  FPMKUnitDeps : array[1..4] of TFPMKUnitDep = (
    (package: 'hash';
     reqver : '2.2.2';
     undef  : 'NO_UNIT_ZIPPER'),
    (package: 'paszlib';
     reqver : '2.2.2';
     undef  : 'NO_UNIT_ZIPPER'),
    (package: 'fcl-process';
     reqver : '2.2.2';
     undef  : 'NO_UNIT_PROCESS'),
    (package: 'fpmkunit';
     reqver : '2.2.2-1';
     undef  : '')
  );

Type
  TLogLevel = (vlError,vlWarning,vlInfo,vlCommands,vlDebug);
  TLogLevels = Set of TLogLevel;

const
  DefaultLogLevels = [vlError,vlWarning];
  AllLogLevels = [vlError,vlWarning,vlCommands,vlInfo];

>>>>>>> graemeg/fixes_2_2
=======
  TLogLevel = (vlError,vlWarning,vlInfo,vlCommands,vlDebug);
  TLogLevels = Set of TLogLevel;

const
  DefaultLogLevels = [vlError,vlWarning];
  AllLogLevels = [vlError,vlWarning,vlCommands,vlInfo];

>>>>>>> origin/fixes_2_2
type
  EPackagerError = class(Exception);

// Logging
Function StringToLogLevels (S : String) : TLogLevels;
Function LogLevelsToString (V : TLogLevels): String;
Procedure Log(Level: TLogLevel;Msg : String);
Procedure Log(Level: TLogLevel;Fmt : String; const Args : array of const);
Procedure Error(Msg : String);
Procedure Error(Fmt : String; const Args : array of const);

// Utils
function maybequoted(const s:string):string;
Function FixPath(const S : String) : string;
Function DirectoryExistsLog(const ADir:string):Boolean;
Function FileExistsLog(const AFileName:string):Boolean;
procedure BackupFile(const AFileName: String);
Procedure DeleteDir(const ADir:string);
Procedure SearchFiles(SL:TStringList;const APattern:string);
<<<<<<< HEAD
<<<<<<< HEAD
Function GetCompilerInfo(const ACompiler,AOptions:string):string; overload;
Procedure GetCompilerInfo(const ACompiler, AOptions: string; out AVersion: string; out ACPU: TCpu; out aOS:TOS); overload;
=======
Function GetCompilerInfo(const ACompiler,AOptions:string):string;
>>>>>>> graemeg/fixes_2_2
=======
Function GetCompilerInfo(const ACompiler,AOptions:string):string;
>>>>>>> origin/fixes_2_2
function IsSuperUser:boolean;

var
  LogLevels : TLogLevels;
<<<<<<< HEAD
<<<<<<< HEAD
  FPMKUnitDeps : array of TFPMKUnitDep;
=======
  FPMKUnitDepAvailable : array[1..FPMKUnitDepCount] of boolean;
>>>>>>> graemeg/fixes_2_2
=======
  FPMKUnitDepAvailable : array[1..FPMKUnitDepCount] of boolean;
>>>>>>> origin/fixes_2_2


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
<<<<<<< HEAD
<<<<<<< HEAD
  result:='fppkg';
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef unix}
  result:='fppkg';
{$else}
  result:='Packages'
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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


procedure Log(Level:TLogLevel;Msg: String);
var
  Prefix : string;
begin
  if not(Level in LogLevels) then
    exit;
  Prefix:='';
  case Level of
    vlWarning :
      Prefix:=SWarning;
    vlError :
      Prefix:=SError;
{    vlInfo :
      Prefix:='I: ';
    vlCommands :
      Prefix:='C: ';
    vlDebug :
      Prefix:='D: '; }
  end;
  Writeln(stdOut,Prefix,Msg);
end;


Procedure Log(Level:TLogLevel; Fmt:String; const Args:array of const);
begin
  Log(Level,Format(Fmt,Args));
end;


procedure Error(Msg: String);
begin
  Raise EPackagerError.Create(Msg);
end;


procedure Error(Fmt: String; const Args: array of const);
begin
  Raise EPackagerError.CreateFmt(Fmt,Args);
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
  If (S<>'') then
    Result:=IncludeTrailingPathDelimiter(S)
  else
    Result:='';
end;


Function DirectoryExistsLog(const ADir:string):Boolean;
begin
  result:=SysUtils.DirectoryExists(ADir);
  if result then
    Log(vlDebug,SDbgDirectoryExists,[ADir,SDbgFound])
  else
    Log(vlDebug,SDbgDirectoryExists,[ADir,SDbgNotFound]);
end;


Function FileExistsLog(const AFileName:string):Boolean;
begin
  result:=SysUtils.FileExists(AFileName);
  if result then
    Log(vlDebug,SDbgFileExists,[AFileName,SDbgFound])
  else
    Log(vlDebug,SDbgFileExists,[AFileName,SDbgNotFound]);
end;


procedure BackupFile(const AFileName: String);
Var
  BFN : String;
begin
  BFN:=AFileName+'.bak';
  Log(vlDebug,SDbgBackupFile,[BFN]);
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
  S.Options:=[poUsePipes];
  S.execute;
  Count:=s.output.read(buf,BufSize);
  S.Free;
{$ENDIF USE_SHELL}
  SetLength(Result,Count);
  Move(Buf,Result[1],Count);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure GetCompilerInfo(const ACompiler, AOptions: string; out AVersion: string; out ACPU: TCpu; out aOS:TOS); overload;
var
  infosl: TStringList;
begin
  infosl:=TStringList.Create;
  infosl.Delimiter:=' ';
  infosl.DelimitedText:=GetCompilerInfo(ACompiler,AOptions);
  if infosl.Count<>3 then
    Raise EPackagerError.Create(SErrInvalidFPCInfo);
  AVersion:=infosl[0];
  ACPU:=StringToCPU(infosl[1]);
  AOS:=StringToOS(infosl[2]);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function IsSuperUser:boolean;
begin
{$ifdef unix}
  result:=(fpGetUID=0);
{$else unix}
  result:=false;
{$endif unix}
end;


initialization
  OnGetVendorName:=@FPPkgGetVendorName;
  OnGetApplicationName:=@FPPkgGetApplicationName;

end.
