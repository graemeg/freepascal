{
    $Id: header,v 1.1 2000/07/13 06:33:45 michael Exp $
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit daemonapp;

{$mode objfpc}{$H+}

interface

uses
  Custapp, Classes, SysUtils, eventlog, rtlconsts;
  
Type
  TCustomDaemon = Class;
  TDaemonController = Class;

  TDaemonEvent = procedure(Sender: TCustomDaemon) of object;
  TDaemonOKEvent = procedure(Sender: TCustomDaemon; var OK: Boolean) of object;
  
  TDaemonOption = (doAllowStop,doAllowPause,doInteractive);
  TDaemonOptions = Set of TDaemonOption;

  TDaemonRunMode = (drmUnknown,drmInstall,drmUninstall,drmRun);
  
  { TCustomDaemonDescription }
  TDaemonDef = Class;
  TCurrentStatus =
    (csStopped, csStartPending, csStopPending, csRunning,
     csContinuePending, csPausePending, csPaused);

  TCustomDaemon = Class(TDataModule)
  private
    FController: TDaemonController;
    FDaemonDef: TDaemonDef;
    FThread : TThread;
    FStatus: TCurrentStatus;
    function GetLogger: TEventLog;
    procedure SetStatus(const AValue: TCurrentStatus);
  Protected
    Function Start : Boolean; virtual;
    Function Stop : Boolean; virtual;
    Function Pause : Boolean; virtual;
    Function Continue : Boolean; virtual;
    Function Execute : Boolean; virtual;
    Function ShutDown : Boolean; virtual;
    Function Install : Boolean; virtual;
    Function UnInstall: boolean; virtual;
    Function HandleCustomCode(ACode : DWord) : Boolean; Virtual;
  Public
    Procedure LogMessage(Msg : String);
    Procedure ReportStatus;
    
    // Filled in at runtime by controller
    Property Definition : TDaemonDef Read FDaemonDef;
    Property DaemonThread : TThread Read FThread;
    Property Controller : TDaemonController Read FController;
    Property Status : TCurrentStatus Read FStatus Write SetStatus;
    Property Logger : TEventLog Read GetLogger;
  end;

  TCustomDaemonClass = Class of TCustomDaemon;

  { TDaemon }
  TCustomControlCodeEvent = Procedure(Sender : TCustomDaemon; ACode : DWord; Var Handled : Boolean) of object;

  TDaemon = Class(TCustomDaemon)
  private
    FAfterInstall: TDaemonEvent;
    FAfterUnInstall: TDaemonEvent;
    FBeforeInstall: TDaemonEvent;
    FBeforeUnInstall: TDaemonEvent;
    FOnContinue: TDaemonOKEvent;
    FOnCustomControl: TCustomControlCodeEvent;
    FOnExecute: TDaemonEvent;
    FOnPause: TDaemonOKEvent;
    FOnShutDown: TDaemonEvent;
    FOnStart: TDaemonOKEvent;
    FOnStop: TDaemonOKEvent;
  Protected
    Function Start : Boolean; override;
    Function Stop : Boolean; override;
    Function Pause : Boolean; override;
    Function Continue : Boolean; override;
    Function Execute : Boolean; override;
    Function ShutDown : Boolean; override;
    Function Install : Boolean; override;
    Function UnInstall: boolean; override;
    Function HandleCustomCode(ACode : DWord) : Boolean; Override;
  Public
    Property Definition;
    Property Status;
  Published
    Property OnStart : TDaemonOKEvent Read FOnStart Write FOnStart;
    Property OnStop : TDaemonOKEvent Read FOnStop Write FOnStop;
    Property OnPause : TDaemonOKEvent Read FOnPause Write FOnPause;
    Property OnContinue : TDaemonOKEvent Read FOnContinue Write FOnContinue;
    Property OnShutDown : TDaemonEvent Read FOnShutDown Write FOnShutDown;
    Property OnExecute : TDaemonEvent Read FOnExecute Write FOnExecute;
    Property BeforeInstall : TDaemonEvent Read FBeforeInstall Write FBeforeInstall;
    Property AfterInstall : TDaemonEvent Read FAfterInstall Write FAfterInstall;
    Property BeforeUnInstall : TDaemonEvent Read FBeforeUnInstall Write FBeforeUnInstall;
    Property AfterUnInstall : TDaemonEvent Read FAfterUnInstall Write FAfterUnInstall;
    Property OnControlCode : TCustomControlCodeEvent Read FOnCustomControl Write FOnCustomControl;
  end;

  { TDaemonController }

  TDaemonController = Class(TComponent)
  Private
    FDaemon : TCustomDaemon;
    FLastStatus: TCurrentStatus;
    FSysData : TObject;
    FParams : TStrings;
    FCheckPoint : DWord;
    procedure ThreadTerminated(Sender: TObject);
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    Procedure StartService; virtual;
    Procedure Main(Argc : DWord; Args : PPChar); Virtual;
    Procedure Controller(ControlCode,EventType : DWord; EventData : Pointer); Virtual;
    Function ReportStatus : Boolean; virtual;
    Property Daemon : TCustomDaemon Read FDaemon;
    Property Params : TStrings Read FParams;
    Property LastStatus : TCurrentStatus Read FLastStatus;
    Property CheckPoint : DWord;
  end;
  
  TDaemonClass = Class of TDaemon;
  
  { Windows specific service registration types }
  
  TServiceType   = (stWin32, stDevice, stFileSystem);
  TErrorSeverity = (esIgnore, esNormal, esSevere, esCritical);
  TStartType     = (stBoot, stSystem, stAuto, stManual, stDisabled);
  
  { TDependency }

  TDependency = class(TCollectionItem)
  private
    FName: String;
    FIsGroup: Boolean;
  protected
    function GetDisplayName: string; override;
  Public
    Procedure Assign(Source : TPersistent); override;
  published
    property Name: String read FName write FName;
    property IsGroup: Boolean read FIsGroup write FIsGroup;
  end;
  
  { TDependencies }

  TDependencies = class(TCollection)
  private
    FOwner: TPersistent;
    function GetItem(Index: Integer): TDependency;
    procedure SetItem(Index: Integer; Value: TDependency);
  protected
    function GetOwner: TPersistent; override;
  public
    constructor Create(AOwner: TPersistent);
    property Items[Index: Integer]: TDependency read GetItem write SetItem; default;
  end;


  { TWinBindings }

  TWinBindings = class(TPersistent)
  private
    FDependencies: TDependencies;
    FErrCode: DWord;
    FErrorSeverity: TErrorSeverity;
    FLoadGroup: String;
    FPassWord: String;
    FServiceType: TServiceType;
    FStartType: TStartType;
    FTagID: DWord;
    FUserName: String;
    FWaitHint: Integer;
    FWin32ErrorCode: DWord;
    procedure SetDependencies(const AValue: TDependencies);
  Public
    Constructor Create;
    Destructor Destroy; override;
    Procedure Assign(Source : TPersistent); override;
    property ErrCode: DWord read FErrCode write FErrCode;
    property Win32ErrCode: DWord read FWin32ErrorCode write FWin32ErrorCode;
  Published
    Property Dependencies : TDependencies Read FDependencies Write SetDependencies;
    Property GroupName : String Read FLoadGroup Write FLoadGroup;
    Property Password : String Read FPassWord Write FPassword;
    Property UserName : String Read FUserName Write FUserName;
    Property StartType : TStartType Read FStartType Write FStartType;
    Property WaitHint : Integer Read FWaitHint Write FWaitHint;
    Property IDTag : DWord Read FTagID Write FTagID;
    Property ServiceType : TServiceType Read FServiceType Write FServiceType;
    Property ErrorSeverity : TErrorSeverity Read FErrorSeverity Write FErrorSeverity;
  end;

  { TDaemonDef }
  
  TDaemonDef = Class(TCollectionItem)
  private
    FDaemonClass: TCustomDaemonClass;
    FDaemonClassName: String;
    FDescription: String;
    FDisplayName: String;
    FEnabled: Boolean;
    FInstance: TCustomDaemon;
    FLogStatusReport: Boolean;
    FName: String;
    FOnCreateInstance: TNotifyEvent;
    FOptions: TDaemonOptions;
    FServiceName: String;
    FWinBindings: TWinBindings;
    FRunArgs : String;
    procedure SetName(const AValue: String);
    procedure SetWinBindings(const AValue: TWinBindings);
  Protected
    function GetDisplayName: string; override;
  Public
    Constructor Create(ACollection : TCollection); override;
    Destructor Destroy; override;
    Property DaemonClass : TCustomDaemonClass read FDaemonClass;
    Property Instance : TCustomDaemon Read FInstance Write FInstance;
  Published
    Property DaemonClassName : String Read FDaemonClassName Write FDaemonClassName;
    Property Name : String Read FName Write SetName;
    Property DisplayName : String Read FDisplayName Write FDisplayName;
    Property RunArguments : String Read FRunArgs Write FRunArgs;
    Property Options : TDaemonOptions Read FOptions Write FOptions;
    Property Enabled : Boolean Read FEnabled Write FEnabled default true;
    Property WinBindings : TWinBindings Read FWinBindings Write SetWinBindings;
    Property OnCreateInstance : TNotifyEvent Read FOnCreateInstance Write FOnCreateInstance;
    Property LogStatusReport : Boolean Read FLogStatusReport Write FLogStatusReport;
  end;

  { TDaemonDefs }

  TDaemonDefs = Class(TCollection)
    FOwner : TPersistent;
  private
    function GetDaemonDef(Index : Integer): TDaemonDef;
    procedure SetDaemonDef(Index : Integer; const AValue: TDaemonDef);
  Protected
    Procedure BindClasses;
    Function GetOwner : TPersistent; override;
  Public
    Constructor Create(AOwner : TPersistent; AClass : TCollectionItemClass);
    Function IndexOfDaemonDef(Const DaemonName : String) : Integer;
    Function FindDaemonDef(Const DaemonName : String) : TDaemonDef;
    Function DaemonDefByName(Const DaemonName : String) : TDaemonDef;
    Property Daemons[Index : Integer] : TDaemonDef Read GetDaemonDef Write SetDaemonDef; default;
  end;
  
  { TCustomDaemonMapper }
  TCustomDaemonMapper = Class(TComponent)
  private
    FDaemonDefs: TDaemonDefs;
    FOnCreate: TNotifyEvent;
    FOnDestroy: TNotifyEvent;
    FOnInstall: TNotifyEvent;
    FOnRun: TNotifyEvent;
    FOnUnInStall: TNotifyEvent;
    procedure SetDaemonDefs(const AValue: TDaemonDefs);
  Protected
    Procedure CreateDefs; virtual;
    Procedure DoOnCreate; virtual;
    Procedure DoOnDestroy; virtual;
    Procedure DoOnInstall; virtual;
    Procedure DoOnUnInstall; virtual;
    Procedure DoOnRun; virtual;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
  Published
    Property DaemonDefs : TDaemonDefs Read FDaemonDefs Write SetDaemonDefs;
    Property OnCreate : TNotifyEvent Read FOnCreate Write FOnCreate;
    Property OnDestroy : TNotifyEvent Read FOnDestroy Write FOnDestroy;
    Property OnRun : TNotifyEvent Read FOnRun Write FOnRun;
    Property OnInstall : TNotifyEvent Read FOnInstall Write FOnInstall;
    Property OnUnInstall : TNotifyEvent Read FOnUnInStall Write FOnUninStall;
  end;
  
  { TDaemonMapper }

  TDaemonMapper = Class(TCustomDaemonMapper)
    Constructor Create(AOwner : TComponent); override;
    Constructor CreateNew(AOwner : TComponent; Dummy : Integer = 0);
  end;
  
  TCustomDaemonMapperClass = Class of TCustomDaemonMapper;
  
  { TDaemonThread }

  TDaemonThread = Class(TThread)
  Private
    FDaemon : TCustomDaemon;
  Protected
    procedure StartServiceExecute; virtual;
    procedure HandleControlCode(ACode : DWord); virtual;
  Public
    Constructor Create(ADaemon : TCustomDaemon);
    Procedure Execute; override;
    Procedure CheckControlMessage(WaitForMessage : Boolean);
    Function StopDaemon : Boolean; virtual;
    Function PauseDaemon : Boolean; virtual;
    Function ContinueDaemon : Boolean; virtual;
    Function ShutDownDaemon : Boolean; virtual;
    Function InterrogateDaemon : Boolean; virtual;
    Property Daemon : TCustomDaemon Read FDaemon;
  end;

  { TCustomDaemonApplication }
  TGuiLoopEvent = Procedure Of Object;
  
  TCustomDaemonApplication = Class(TCustomApplication)
  private
    FGUIHandle: THandle;
    FGUIMainLoop: TGuiLoopEvent;
    FLogger: TEventLog;
    FMapper : TCustomDaemonMapper;
    FOnRun: TNotifyEvent;
    FRunMode: TDaemonRunMode;
    FSysData: TObject;
    FControllerCount : Integer;
    procedure BindDaemonDefs(AMapper: TCustomDaemonMapper);
    function  InstallRun: Boolean;
    procedure SysInstallDaemon(Daemon: TCustomDaemon);
    procedure SysUnInstallDaemon(Daemon: TCustomDaemon);
    function  UnInstallRun: Boolean;
    function  RunDaemonsRun: Boolean;
    Procedure Main(Argc : DWord; Args : PPchar);
    Function  RunGUIloop(P : Pointer) : integer;
  Protected
    // OS (System) dependent calls
    Procedure SysStartUnInstallDaemons;
    Procedure SysEndUnInstallDaemons;
    Procedure SysStartInstallDaemons;
    Procedure SysEndInstallDaemons;
    Procedure SysStartRunDaemons;
    Procedure SysEndRunDaemons;

    // Customizable behaviour
    procedure CreateDaemonController(Var AController : TDaemonController); virtual;
    Procedure CreateServiceMapper(Var AMapper : TCustomDaemonMapper); virtual;
    Procedure CreateDaemonInstance(Var ADaemon : TCustomDaemon; DaemonDef : TDaemonDef); virtual;
    Procedure RemoveController(AController : TDaemonController); virtual;
    procedure SetupLogger;
    procedure StopLogger;
    Procedure DoRun; override;
    Property  OnRun : TNotifyEvent Read FOnRun Write FOnRun;
    Property SysData : TObject Read FSysData Write FSysData;
  Public
    Procedure ShowException(E : Exception); override;
    Function CreateDaemon(DaemonDef : TDaemonDef) : TCustomDaemon;
    Procedure StopDaemons(Force : Boolean);
    procedure InstallDaemons;
    procedure RunDaemons;
    procedure UnInstallDaemons;
    procedure CreateForm(InstanceClass: TComponentClass; var Reference); virtual;
    Property Logger : TEventLog Read FLogger;
    Property GUIMainLoop : TGuiLoopEvent Read FGUIMainLoop Write FGuiMainLoop;
    Property GuiHandle : THandle Read FGUIHandle Write FGUIHandle;
    Property RunMode : TDaemonRunMode Read FRunMode;
  end;
  TCustomDaemonApplicationClass = Class of TCustomDaemonApplication;
  
  TDaemonApplication = Class(TCustomDaemonApplication);

  EDaemon = Class(Exception);

Function Application : TCustomDaemonApplication;
Procedure RegisterDaemonMapper(AMapperClass : TCustomDaemonMapperClass);
Procedure RegisterDaemonClass(AClass : TCustomDaemonClass);
Procedure RegisterDaemonApplicationClass(AClass : TCustomDaemonApplicationClass);
Procedure DaemonError(Msg : String);
Procedure DaemonError(Fmt : String; Args : Array of const);


Resourcestring
  SErrNoServiceMapper           = 'No daemon mapper class registered.';
  SErrOnlyOneMapperAllowed      = 'Not changing daemon mapper class %s with %s: Only 1 mapper allowed.';
  SErrNothingToDo               = 'Options do not allow determining what needs to be done.';
  SErrDuplicateName             = 'Duplicate daemon name: %s';
  SErrUnknownDaemonClass        = 'Unknown daemon class name: %s';
  SErrDaemonStartFailed         = 'Failed to start daemon %s : %s';
  SDaemonStatus                 = 'Daemon %s current status: %s';
  SControlFailed                = 'Control code %s handling failed: %s';
  SCustomCode                   = '[Custom code %d]';
  SErrServiceManagerStartFailed = 'Failed to start service manager: %s';
  SErrNoDaemonForStatus         = '%s: No daemon for status report';
  SErrNoDaemonDefForStatus      = '%s: No daemon definition for status report';
  SErrWindowClass               = 'Could not register window class';
  SErrApplicationAlreadyCreated = 'An application instance of class %s was already created.';
  
{ $define svcdebug}

{$ifdef svcdebug}
Procedure DebugLog(Msg : String);
{$endif}

Var
  CurrentStatusNames : Array[TCurrentStatus] of string =
    ('Stopped', 'Start Pending', 'Stop Pending', 'Running',
     'Continue Pending', 'Pause Pending', 'Paused');
  SStatus : Array[1..5] of string =
    ('Stop','Pause','Continue','Interrogate','Shutdown');
  DefaultDaemonOptions : TDaemonOptions =  [doAllowStop,doAllowPause];
  
implementation

// This must come first, so a uses clause can be added.
{$i daemonapp.inc}

Var
  AppInstance   : TCustomDaemonApplication;
  MapperClass   : TCustomDaemonMapperClass;
  DesignMapper  : TCustomDaemonMapper;
  DaemonClasses : TStringList;
  AppClass      : TCustomDaemonApplicationClass;
  
{$ifdef svcdebug}
Var
  FL : Text;
  LCS : TRTLCriticalSection;
  
Procedure StartLog;

begin
{$ifdef win32}
  Assign(FL,'c:\service.log');
{$else}
  Assign(FL,'/tmp/service.log');
{$endif}
  Rewrite(FL);
  InitCriticalSection(LCS);
  DebugLog('Start logging');
end;

Procedure DebugLog(Msg : String);
begin
  EnterCriticalSection(LCS);
  try
    Writeln(FL,Msg);
    Flush(FL);
  Finally
    LeaveCriticalSection(LCS);
  end;
end;

Procedure EndLog;

begin
  DebugLog('Done logging');
  Close(FL);
  DoneCriticalSection(LCS);
end;
{$endif svcdebug}

Procedure RegisterDaemonApplicationClass(AClass : TCustomDaemonApplicationClass);

begin
  If (AppInstance<>Nil) then
    DaemonError(SErrApplicationAlreadyCreated,[AppInstance.ClassName]);
  AppClass:=AClass;  
end;

Procedure RegisterDaemonClass(AClass : TCustomDaemonClass);

Var
  DN : String;
  I  : Integer;
  
begin
  If Not Assigned(DaemonClasses) then
    begin
    DaemonClasses:=TStringList.Create;
    DaemonClasses.Sorted:=True;
    end;
  DN:=AClass.ClassName;
  I:=DaemonClasses.IndexOf(DN);
  If (I=-1) then
    I:=DaemonClasses.Add(DN);
  DaemonClasses.Objects[I]:=TObject(AClass);
end;

Procedure CreateDaemonApplication;

begin
  If (AppClass=Nil) then
    AppClass:=TCustomDaemonApplication;
  AppInstance:=AppClass.Create(Nil);
end;

Procedure DoneDaemonApplication;

begin
  FreeAndNil(AppInstance);
  FreeAndNil(DaemonClasses);
end;

function Application: TCustomDaemonApplication;
begin
  If (AppInstance=Nil) then
    CreateDaemonApplication;
  Result:=AppInstance;
end;

Procedure RegisterDaemonMapper(AMapperClass : TCustomDaemonMapperClass);

begin
  If Assigned(MapperClass) then
    DaemonError(SErrOnlyOneMapperAllowed,[MapperClass.ClassName,AMapperClass.ClassName]);
  MapperClass:=AMapperClass;
end;

procedure DaemonError(Msg: String);
begin
  Raise EDaemon.Create(MSg);
end;

procedure DaemonError(Fmt: String; Args: array of const);
begin
  Raise EDaemon.CreateFmt(Fmt,Args);
end;

{ TDaemon }

function TDaemon.Start: Boolean;
begin
  Result:=inherited Start;
  If assigned(FOnStart) then
    FOnStart(Self,Result);
end;

function TDaemon.Stop: Boolean;
begin
  Result:=inherited Stop;
  If assigned(FOnStop) then
    FOnStop(Self,Result);
end;

function TDaemon.Pause: Boolean;
begin
  Result:=inherited Pause;
  If assigned(FOnPause) then
    FOnPause(Self,Result);
end;

function TDaemon.Continue: Boolean;
begin
  Result:=inherited Continue;
  If assigned(FOnContinue) then
    FOnContinue(Self,Result);
end;

function TDaemon.Execute: Boolean;
begin
  Result:=Assigned(FOnExecute);
  If Result Then
    FOnExecute(Self);
end;

function TDaemon.ShutDown: Boolean;
begin
  Result:=Inherited ShutDown;
  If Assigned(FOnShutDown) then
    FOnShutDown(Self);
end;

function TDaemon.Install: Boolean;
begin
  If Assigned(FBeforeInstall) then
    FBeforeInstall(Self);
  Result:=inherited Install;
  If Assigned(FAfterInstall) then
    FAfterInstall(Self)
end;

function TDaemon.UnInstall: boolean;
begin
  If Assigned(FBeforeUnInstall) then
    FBeforeUnInstall(Self);
  Result:=inherited UnInstall;
  If Assigned(FAfterUnInstall) then
    FAfterUnInstall(Self)
end;

function TDaemon.HandleCustomCode(ACode: DWord): Boolean;
begin
  Result:=Assigned(FOnCustomControl);
  If Result then
    FOnCustomControl(Self,ACode,Result);
end;

{ TCustomDaemon }

Function TCustomDaemon.Start : Boolean;

begin
  Result:=True;
end;

Function  TCustomDaemon.Stop : Boolean;
begin
  Result:=True;
end;

Function TCustomDaemon.Pause : Boolean;
begin
  Result:=True;
end;

Function TCustomDaemon.Continue : Boolean;
begin
  Result:=True;
end;

function TCustomDaemon.Execute: Boolean;
begin
  Result:=False;
end;

Function TCustomDaemon.ShutDown : Boolean;
begin
  Result:=True;
end;

Procedure TCustomDaemon.ReportStatus;
begin
  Controller.ReportStatus;
end;



procedure TCustomDaemon.LogMessage(Msg: String);
begin
  Application.Logger.Error(Msg);
end;

function TCustomDaemon.GetLogger: TEventLog;
begin
  Result:=Application.Logger;
end;

procedure TCustomDaemon.SetStatus(const AValue: TCurrentStatus);
begin
  FStatus:=AValue;
  Controller.ReportStatus;
end;

Function TCustomDaemon.Install : Boolean;
begin
  Result:=True;
  Application.SysInstallDaemon(Self);
end;


Function TCustomDaemon.UnInstall : Boolean;
begin
  Result:=True;
  Application.SysUnInstallDaemon(Self);
end;

function TCustomDaemon.HandleCustomCode(ACode: DWord): Boolean;
begin
  Result:=False
end;

{ TCustomServiceApplication }


procedure TCustomDaemonApplication.CreateServiceMapper(Var AMapper : TCustomDaemonMapper);

begin
  AMapper:=MapperClass.Create(Self);
  BindDaemonDefs(Amapper);
end;

procedure TCustomDaemonApplication.BindDaemonDefs(AMapper : TCustomDaemonMapper);

begin
  AMApper.DaemonDefs.BindClasses;
end;

procedure TCustomDaemonApplication.CreateDaemonController(Var AController : TDaemonController);

begin
  ACOntroller:=TDaemonController.Create(Self);
end;

Function TCustomDaemonApplication.RunDaemonsRun : Boolean;

begin
  Result:=HasOption('r','run');
  // No Borland compatibility needed, as the install will take care of the -r
end;

procedure TCustomDaemonApplication.Main(Argc: DWord; Args: PPchar);

Var
  SN : String;
  DD : TDaemonDef;
  
begin
  If (Args=Nil) then
    Exit;
  SN:=StrPas(Args^);
  DD:=FMapper.DaemonDefs.FindDaemonDef(SN);
  If (DD<>Nil) then
    DD.Instance.Controller.Main(Argc,Args);
end;


Function TCustomDaemonApplication.InstallRun : Boolean;

begin
  Result:=HasOption('i','install');
  // Borland compatibility.
  If not Result then
    Result:=FindCmdLineSwitch ('install',['/'],True);
end;



Function TCustomDaemonApplication.UnInstallRun : Boolean;

begin
  Result:=HasOption('u','uninstall');
  // Borland compatibility.
  If not Result then
    Result:=FindCmdLineSwitch ('uninstall',['/'],True);
end;



Procedure TCustomDaemonApplication.InstallDaemons;

Var
  D : TCustomDaemon;
  DD : TDaemonDef;
  C : TDaemonController;
  I : Integer;

begin
  FrunMode:=drmInstall;
  SysStartInstallDaemons;
  try
    FMapper.DoOnInstall;
    For I:=0 to FMapper.DaemonDefs.Count-1 do
      begin
      DD:=FMapper.DaemonDefs[i];
      If DD.Enabled then
        begin
        D:=CreateDaemon(DD);
        Try
          // Need to call this because of the before/after events.
           D.Install;
        Finally
          D.Free;
        end;
        end;
      end;
  Finally
    SysEndInstallDaemons;
  end;
end;

Procedure TCustomDaemonApplication.UnInstallDaemons;

Var
  D : TCustomDaemon;
  DD : TDaemonDef;
  I : Integer;

begin
  FrunMode:=drmUnInstall;
  SysStartUnInstallDaemons;
  Try
    FMapper.DoOnUnInstall;
    // Uninstall in reverse order. One never knows.
    For I:=FMapper.DaemonDefs.Count-1 downto 0 do
      begin
      DD:=FMapper.DaemonDefs[i];
      If DD.Enabled then
        begin
        D:=CreateDaemon(FMapper.DaemonDefs[i]);
        Try
          // Need to call this because of the before/after events.
          D.UnInstall 
        Finally
          D.Free;
        end;
        end;
      end;
  Finally
    SysEndUnInstallDaemons;
  end;
end;

procedure TCustomDaemonApplication.CreateForm(InstanceClass: TComponentClass;
  var Reference);
  
Var
  Instance: TComponent;
  
begin
  // Allocate the instance, without calling the constructor
  Instance := TComponent(InstanceClass.NewInstance);
  // set the Reference before the constructor is called, so that
  // events and constructors can refer to it
  TComponent(Reference) := Instance;
  try
    Instance.Create(Self);
  except
    TComponent(Reference) := nil;
    Raise;
  end;
end;

Procedure TCustomDaemonApplication.RunDaemons;

Var
  D : TCustomDaemon;
  DD : TDaemonDef;
  I : Integer;

begin
  FRunMode:=drmRun;
  SysStartRunDaemons;
  FMapper.DoOnRun;
  For I:=0 to FMapper.DaemonDefs.Count-1 do
    begin
    DD:=FMapper.DaemonDefs[i];
    If DD.Enabled then
      D:=CreateDaemon(FMapper.DaemonDefs[i]);
    end;
  try
    SysEndRunDaemons;
  except
    HandleException(Self);
    Terminate;
  end;
end;

procedure TCustomDaemonApplication.SetupLogger;

begin
  FLogger:=TEventlog.Create(Self);
  FLogger.RegisterMessageFile('');
end;

procedure TCustomDaemonApplication.StopLogger;

begin
  Flogger.Active:=False;
  FreeAndNil(Flogger);
end;

procedure TCustomDaemonApplication.DoRun;

begin
  SetupLogger;
  Try
    try
      If Not Assigned(MapperClass) then
        DaemonError(SErrNoServiceMapper);
      CreateServiceMapper(FMapper);
      If InstallRun then
        InstallDaemons
      else If UnInstallRun then
        UnInstallDaemons
      else if RunDaemonsRun then
        RunDaemons
      else if Assigned(OnRun) then
       OnRun(Self)
      else
        DaemonError(SErrNothingToDo);
      {$ifdef svcdebug}DebugLog('Terminating');{$endif svcdebug}
      Terminate;
      {$ifdef svcdebug}DebugLog('Terminated');{$endif svcdebug}
    except
      Terminate;
      Raise
    end;
  Finally
    StopLogger;
  end;
end;

procedure TCustomDaemonApplication.ShowException(E: Exception);
begin
  If assigned(Flogger) then
    FLogger.Error(E.Message)
  else
   inherited ShowException(E)
end;

Procedure TCustomDaemonApplication.CreateDaemonInstance(Var ADaemon : TCustomDaemon; DaemonDef : TDaemonDef); 

begin
  ADaemon:=DaemonDef.DaemonClass.CreateNew(Self,0);
end;

function TCustomDaemonApplication.CreateDaemon(DaemonDef: TDaemonDef): TCustomDaemon;

Var
  C : TDaemonController;

begin
  CreateDaemonInstance(Result,DaemonDef);
  CreateDaemonController(C);
  C.FDaemon:=Result;
  Result.FController:=C;
  Result.FDaemonDef:=DaemonDef;
  If (Daemondef.Instance=Nil) then
    DaemonDef.Instance:=Result;
end;

procedure TCustomDaemonApplication.StopDaemons(Force: Boolean);

Const
  ControlCodes : Array[Boolean] of DWord
               = (SERVICE_CONTROL_STOP,SERVICE_CONTROL_SHUTDOWN);

Var
  L : TFPList;
  I : Integer;

begin
  L:=TFPList.Create;
  try
    For I:=0 to ComponentCount-1 do
      If Components[i] is TDaemonController then
        L.Add(Components[i]);
    For I:=L.Count-1 downto 0 do
      TDaemonController(L[i]).Controller(ControlCodes[Force],0,Nil);
  finally
    L.Free;
  end;
end;




{ TDaemonDefs }

function TDaemonDefs.GetDaemonDef(Index : Integer): TDaemonDef;
begin
  Result:=TDaemonDef(Items[index]);
end;

procedure TDaemonDefs.SetDaemonDef(Index : Integer; const AValue: TDaemonDef);
begin
  Items[Index]:=AValue;
end;

procedure TDaemonDefs.BindClasses;

Var
  D : TDaemonDef;
  I,J : Integer;
  
begin
  For I:=0 to Count-1 do
    begin
    D:=GetDaemonDef(I);
    J:=DaemonClasses.IndexOf(D.DaemonClassName);
    If (J=-1) then
      DaemonError(SErrUnknownDaemonClass,[D.DaemonClassName])
    else
      D.FDaemonClass:=TCustomDaemonClass(DaemonClasses.Objects[J]);
    end;

end;

function TDaemonDefs.GetOwner: TPersistent;
begin
  Result:=FOwner;
end;

constructor TDaemonDefs.Create(AOwner: TPersistent; AClass : TCollectionItemClass);
begin
  Inherited Create(AClass);
  FOwner:=AOwner;
  
end;

function TDaemonDefs.IndexOfDaemonDef(Const DaemonName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetDaemonDef(Result).Name,DaemonName)<>0) do
    Dec(Result);
end;

function TDaemonDefs.FindDaemonDef(Const DaemonName: String): TDaemonDef;

Var
  I : Integer;

begin
  I:=IndexOfDaemonDef(DaemonName);
  If I<>-1 then
    Result:=GetDaemonDef(I)
  else
    Result:=Nil;
end;

function TDaemonDefs.DaemonDefByName(Const DaemonName: String): TDaemonDef;
begin
  Result:=FindDaemonDef(DaemonName);
end;

{ TDaemonDef }

procedure TDaemonDef.SetName(const AValue: String);
begin
  If (AValue<>FName) then
    begin
    If (AValue<>'') and (Collection<>Nil)
       and (Collection is TDaemonDefs)
       and ((Collection as TDaemonDefs).IndexOfDaemonDef(AValue)<>-1) then
      DaemonError(SErrDuplicateName,[Avalue]);
    FName:=AValue;
    end;
end;

procedure TDaemonDef.SetWinBindings(const AValue: TWinBindings);
begin
  FWinBindings.Assign(AValue);
end;

function TDaemonDef.GetDisplayName: string;
begin
  Result:=Name;
end;

constructor TDaemonDef.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FWinBindings:=TWinBindings.Create;
  FEnabled:=True;
  FOptions:=DefaultDaemonOptions;
end;

destructor TDaemonDef.Destroy;
begin
  FreeAndNil(FWinBindings);
  inherited Destroy;
end;

{ TCustomDaemonMapper }

procedure TCustomDaemonMapper.SetDaemonDefs(const AValue: TDaemonDefs);
begin
  if (FDaemonDefs=AValue) then
    exit;
  FDaemonDefs.Assign(AValue);
end;

procedure TCustomDaemonMapper.CreateDefs;
begin
  FDaemonDefs:=TDaemonDefs.Create(Self,TDaemonDef);
end;

procedure TCustomDaemonMapper.DoOnCreate;
begin
  If Assigned(FOnCreate) then
    FOnCreate(Self);
end;

procedure TCustomDaemonMapper.DoOnDestroy;
begin
  If Assigned(FOnDestroy) then
    FOnDestroy(Self);
end;

procedure TCustomDaemonMapper.DoOnInstall;
begin
  If Assigned(FOnInstall) then
    FOnInstall(Self);
end;

procedure TCustomDaemonMapper.DoOnUnInstall;
begin
  If Assigned(FOnUnInstall) then
    FOnUnInstall(Self);
end;

procedure TCustomDaemonMapper.DoOnRun;
begin
  If Assigned(FOnRun) then
    FOnRun(Self);
end;

constructor TCustomDaemonMapper.Create(AOwner: TComponent);
begin
  CreateDefs; // First, otherwise streaming will fail.
  inherited Create(AOwner);
  DoOnCreate;
end;

destructor TCustomDaemonMapper.Destroy;
begin
  DoOnDestroy;
  FreeAndNil(FDaemonDefs);
  inherited Destroy;
end;

{ TDaemonThread }

constructor TDaemonThread.Create(ADaemon: TCustomDaemon);
begin
  FDaemon:=ADAemon;
  FDaemon.FThread:=Self;
  FreeOnTerminate:=False;
  Inherited Create(True);
end;

procedure TDaemonThread.Execute;

begin
  If FDaemon.Start then
    begin
    FDaemon.Status:=csRunning;
    StartServiceExecute;
    if not FDaemon.Execute then
      begin
      While Not Terminated do
        CheckControlMessage(True);
      CheckControlMessage(False);
      end;
    end;
end;


procedure TDaemonThread.HandleControlCode(ACode : DWord);

Var
  CS : TCurrentStatus;
  CC,OK : Boolean;
  S : String;

begin
  CS:=FDaemon.Status;
  Try
    OK:=True;
    CC:=False;
    Case ACode of
      SERVICE_CONTROL_STOP        : OK:=StopDaemon;
      SERVICE_CONTROL_PAUSE       : OK:=PauseDaemon;
      SERVICE_CONTROL_CONTINUE    : OK:=ContinueDaemon;
      SERVICE_CONTROL_SHUTDOWN    : OK:=ShutDownDaemon;
      SERVICE_CONTROL_INTERROGATE : OK:=InterrogateDaemon;
    else
      CC:=True;
      FDaemon.HandleCustomCode(ACode);
    end;
    If not OK then
      FDaemon.Status:=CS;
  Except
    On E : Exception do
      begin
      // Shutdown MUST be done, in all other cases roll back status.
      If (ACode<>SERVICE_CONTROL_SHUTDOWN) then
        FDaemon.Status:=CS;
      If (ACode in [1..5]) then
        S:=SStatus[ACode]
      else
        S:=Format(SCustomCode,[ACode]);
      Application.Logger.Error(SControlFailed,[S,E.Message]);
      end;
  end;
end;

function TDaemonThread.StopDaemon: Boolean;

begin
  FDaemon.Status:=csStopPending;
  Result:=FDaemon.Stop;
  If Result then
    begin
    FDaemon.Status:=csStopped;
    Terminate;
    end;
end;

function TDaemonThread.PauseDaemon: Boolean;
begin
  FDaemon.Status:=csPausePending;
  Result:=FDaemon.Pause;
  If Result then
    begin
    FDaemon.Status:=csPaused;
    Suspend;
    end;
end;

function TDaemonThread.ContinueDaemon: Boolean;
begin
  FDaemon.Status:=csContinuePending;
  Result:=FDaemon.Continue;
  If Result then
    FDaemon.Status:=csRunning;
end;

function TDaemonThread.ShutDownDaemon: Boolean;
begin
  FDaemon.Status:=csStopPending;
  Try
    Result:=FDaemon.ShutDown;
  except
    FDaemon.Status:=csStopped;
    Terminate;
  end;
end;

Function TDaemonThread.InterrogateDaemon: Boolean;
begin
  FDaemon.ReportStatus;
  Result:=True;
end;

{ ---------------------------------------------------------------------
  TDaemonController - Global implementation
  ---------------------------------------------------------------------}


constructor TDaemonController.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FParams:=TStringList.Create;
end;

destructor TDaemonController.Destroy;
begin
  FreeAndNil(FSysData);
  FreeAndNil(FParams);
  inherited Destroy;
end;



{ TWinBindings }

procedure TWinBindings.SetDependencies(const AValue: TDependencies);
begin
  if (FDependencies<>AValue) then
    FDependencies.Assign(AValue);
end;

Constructor TWinBindings.Create;
begin
  FDependencies:=TDependencies.Create(Self);
end;

destructor TWinBindings.Destroy;
begin
  FreeAndNil(FDependencies);
  inherited Destroy;
end;

procedure TWinBindings.Assign(Source: TPersistent);

Var
  WB : TWinBindings;

begin
  if Source is TWinBindings then
    begin
    WB:=Source as TWinBindings;
    GroupName:=WB.GroupName;
    Password:=WB.PassWord;
    UserName:=WB.UserName;
    StartType:=WB.StartType;
    WaitHint:=WB.WaitHint;
    IDTag:=WB.IDTag;
    ServiceType:=WB.ServiceType;
    ErrorSeverity:=WB.ErrorSeverity;
    Dependencies.Assign(WB.Dependencies);
    ErrCode:=WB.ErrCode;
    Win32ErrCode:=WB.Win32ErrCode;
    end
  else
    inherited Assign(Source);
end;

{ TDependency }

function TDependency.GetDisplayName: string;
begin
  Result:=Name;
end;

procedure TDependency.Assign(Source: TPersistent);

Var
  D : TDependency;

begin
  if Source is TDependency then
    begin
    D:=Source as TDependency;
    Name:=D.Name;
    IsGroup:=D.IsGroup;
    end
  else
    inherited Assign(Source);
end;

{ TDependencies }

function TDependencies.GetItem(Index: Integer): TDependency;
begin
  Result:=TDependency(Inherited GetItem(Index));
end;

procedure TDependencies.SetItem(Index: Integer; Value: TDependency);
begin
  Inherited SetItem(Index,Value);
end;

function TDependencies.GetOwner: TPersistent;
begin
  Result:=FOwner;
end;

constructor TDependencies.Create(AOwner: TPersistent);
begin
  Inherited Create(TDependency);
  FOwner:=AOwner;
end;

{ TDaemonMapper }

constructor TDaemonMapper.Create(AOwner: TComponent);
begin
 CreateNew(AOwner,0);
 if (ClassType<>TDaemonMapper) and not (csDesigning in ComponentState) then
    begin
    if not InitInheritedComponent(Self,TDaemonMapper) then
      raise EStreamError.CreateFmt(SErrNoSTreaming, [ClassName]);
    end;
end;

constructor TDaemonMapper.CreateNew(AOwner: TComponent; Dummy: Integer);
begin
  inherited Create(AOwner);
end;

Initialization
{$ifdef svcdebug}
  StartLog;
{$endif}
  SysInitDaemonApp;

Finalization
  SysDoneDaemonApp;
  DoneDaemonApplication;
{$ifdef svcdebug}
  EndLog;
{$endif}
end.

