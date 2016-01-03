{$mode objfpc}
{$h+}
unit pkghandler;

interface

uses
  Classes,SysUtils,
  pkgglobals,
  pkgoptions,
  fprepos;

type
  { TPackageHandler }

  TPackageHandler = Class(TComponent)
  private
    FPackageName : string;
  Protected
    Procedure Log(Level: TLogLevel;Msg : String);
    Procedure Log(Level: TLogLevel;Fmt : String; const Args : array of const);
    Procedure Error(Msg : String);
    Procedure Error(Fmt : String; const Args : array of const);
    Function ExecuteProcess(Const Prog,Args:String):Integer;
    Procedure SetCurrentDir(Const ADir:String);
  Public
    Constructor Create(AOwner:TComponent;const APackageName:string); virtual;
    function PackageLogPrefix:String;
    procedure ExecuteAction(const APackageName,AAction:string);
    procedure Execute; virtual; abstract;
    Property PackageName:string Read FPackageName;
  end;
  TPackageHandlerClass = class of TPackageHandler;

  EPackageHandler = Class(Exception);

// Actions/PkgHandler
procedure RegisterPkgHandler(const AAction:string;pkghandlerclass:TPackageHandlerClass);
function GetPkgHandler(const AAction:string):TPackageHandlerClass;
procedure ExecuteAction(const APackageName,AAction:string);

function PackageBuildPath(APackage:TFPPackage):String;
function PackageRemoteArchive(APackage:TFPPackage): String;
function PackageLocalArchive(APackage:TFPPackage): String;
function PackageManifestFile(APackage:TFPPackage): String;


Implementation

uses
  typinfo,
  contnrs,
  uriparser,
  pkgrepos,
  pkgmessages;

var
  PkgHandlerList  : TFPHashList;
  ExecutedActions : TFPHashList;
<<<<<<< HEAD
<<<<<<< HEAD
  CurrentDir      : string;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

procedure RegisterPkgHandler(const AAction:string;pkghandlerclass:TPackageHandlerClass);
begin
  if PkgHandlerList.Find(AAction)<>nil then
    begin
      Raise EPackageHandler.CreateFmt(SErrActionAlreadyRegistered,[AAction]);
      exit;
    end;
  PkgHandlerList.Add(AAction,pkghandlerclass);
end;


function GetPkgHandler(const AAction:string):TPackageHandlerClass;
begin
  result:=TPackageHandlerClass(PkgHandlerList.Find(AAction));
  if result=nil then
    Raise EPackageHandler.CreateFmt(SErrActionNotFound,[AAction]);
end;


procedure ExecuteAction(const APackageName,AAction:string);
var
  pkghandlerclass : TPackageHandlerClass;
  FullActionName : string;
begin
  // Check if we have already executed or are executing the action
  FullActionName:=APackageName+AAction;
  if ExecutedActions.Find(FullActionName)<>nil then
    begin
      Log(vlDebug,'Already executed or executing action '+FullActionName);
      exit;
    end;
  ExecutedActions.Add(FullActionName,Pointer(PtrUInt(1)));
  // Create action handler class
  pkghandlerclass:=GetPkgHandler(AAction);
  With pkghandlerclass.Create(nil,APackageName) do
    try
      Log(vlDebug,SLogRunAction+' start',[AAction]);
      Execute;
      Log(vlDebug,SLogRunAction+' end',[AAction]);
    finally
      Free;
    end;
end;


function PackageBuildPath(APackage:TFPPackage):String;
begin
  if APackage.Name=CurrentDirPackageName then
<<<<<<< HEAD
<<<<<<< HEAD
    begin
      // It could be that to resolve some dependencies, the current directory changes. The first time
      // PackageBuildPath is called the dependencies are not resolved yet, so store the current directory
      // for later calls.
      if CurrentDir='' then
        begin
          Result:='.';
          CurrentDir := SysUtils.GetCurrentDir;
        end
      else
        Result:=CurrentDir;
    end
  else if APackage.Name=CmdLinePackageName then
    Result:=GlobalOptions.BuildDir+ChangeFileExt(ExtractFileName(APackage.LocalFileName),'')
  else if (APackage.RecompileBroken) and (APackage.SourcePath<>'') then
    Result:=APackage.SourcePath
=======
    Result:='.'
  else if APackage.Name=CmdLinePackageName then
    Result:=GlobalOptions.BuildDir+ChangeFileExt(ExtractFileName(APackage.LocalFileName),'')
>>>>>>> graemeg/fixes_2_2
=======
    Result:='.'
  else if APackage.Name=CmdLinePackageName then
    Result:=GlobalOptions.BuildDir+ChangeFileExt(ExtractFileName(APackage.LocalFileName),'')
>>>>>>> origin/fixes_2_2
  else
    Result:=GlobalOptions.BuildDir+APackage.Name;
end;


function PackageRemoteArchive(APackage:TFPPackage): String;
begin
  if APackage.Name=CurrentDirPackageName then
    Error(SErrNoPackageSpecified)
  else if APackage.Name=CmdLinePackageName then
    Error(SErrPackageIsLocal);
  if APackage.DownloadURL<>'' then
    Result:=APackage.DownloadURL
  else
    Result:=GetRemoteRepositoryURL(APackage.FileName);
end;


function PackageLocalArchive(APackage:TFPPackage): String;
begin
  if APackage.Name=CurrentDirPackageName then
    Error(SErrNoPackageSpecified)
  else if APackage.Name=CmdLinePackageName then
    Result:=APackage.LocalFileName
  else
    Result:=GlobalOptions.ArchivesDir+APackage.FileName;
end;


function PackageManifestFile(APackage:TFPPackage): String;
begin
  Result:=ManifestFileName;
end;



{ TPackageHandler }

constructor TPackageHandler.Create(AOwner:TComponent;const APackageName:string);
begin
  inherited Create(AOwner);
  FPackageName:=APackageName;
end;

Function TPackageHandler.ExecuteProcess(Const Prog,Args:String):Integer;
begin
  Log(vlCommands,SLogExecute,[Prog,Args]);
<<<<<<< HEAD
<<<<<<< HEAD
  Flush(StdOut);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result:=SysUtils.ExecuteProcess(Prog,Args);
end;


Procedure TPackageHandler.SetCurrentDir(Const ADir:String);
begin
  Log(vlCommands,SLogChangeDir,[ADir]);
  if not SysUtils.SetCurrentDir(ADir) then
    Error(SErrChangeDirFailed,[ADir]);
end;


function TPackageHandler.PackageLogPrefix:String;
begin
  if PackageName<>'' then
    Result:='['+PackageName+'] '
  else
    Result:='';
end;


procedure TPackageHandler.ExecuteAction(const APackageName,AAction:string);
begin
  // Needed to override TComponent.ExecuteAction method
  pkghandler.ExecuteAction(APackageName,AAction);
end;


Procedure TPackageHandler.Log(Level:TLogLevel; Msg:String);
begin
  pkgglobals.Log(Level,PackageLogPrefix+Msg);
end;


Procedure TPackageHandler.Log(Level:TLogLevel; Fmt:String; const Args:array of const);
begin
  pkgglobals.Log(Level,PackageLogPrefix+Fmt,Args);
end;


Procedure TPackageHandler.Error(Msg:String);
begin
  pkgglobals.Error(PackageLogPrefix+Msg);
end;


Procedure TPackageHandler.Error(Fmt:String; const Args:array of const);
begin
  pkgglobals.Error(PackageLogPrefix+Fmt,Args);
end;


initialization
  PkgHandlerList:=TFPHashList.Create;
  ExecutedActions:=TFPHashList.Create;
finalization
  FreeAndNil(PkgHandlerList);
  FreeAndNil(ExecutedActions);
end.
