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
{$mode objfpc}
{$h+}
unit ServiceManager;

interface

uses
  Windows, SysUtils, Classes, jwawinnt, jwawinsvc;

type

  TServiceEntry = Class(TCollectionItem)
  Private
    FServiceName,
    FDisplayName : String;
    FServiceType,
    FCurrentState,
    FControlsAccepted,
    FWin32ExitCode,
    FServiceSpecificExitCode,
    FCheckPoint,
    FWaitHint: DWORD;
  Private
    Procedure SetStatusFields(Const Status : TServiceStatus);
  Public
    Property ServiceName : String Read FServiceName;
    Property DisplayName : String read FDIsplayName;
    Property ServiceType : DWord Read FServiceType;
    Property CurrentState : DWord Read FCurrentState;
    Property ControlsAccepted : DWord Read FControlsAccepted;
    Property Win32ExitCode : DWord Read FWin32ExitCode;
    Property ServiceSpecificExitCode : DWord Read FServiceSpecificExitCode;
    Property CheckPoint : DWord Read FCheckPoint;
    Property WaitHint: DWORD Read FWaitHint;
  end;

  TServiceEntries = Class(TOwnedCollection)
  Private
    Function GetService (Index : Integer) : TServiceEntry;
  Public
    Function FindService(ServiceName : String) : TServiceEntry;
    Function ServiceByName(ServiceName : String) : TServiceEntry;
    Property Items [index : Integer] : TServiceEntry Read GetService;default;
  end;

  { Record used in
    registerservice,
    configservice or
    queryserviceconfig
   }

  TServiceDescriptor = Record
    Name : ShortString;
    DisplayName : ShortString;
    DesiredAccess : DWord;
    ServiceType : DWord;
    StartType : DWord;
    ErrorControl : DWord;
    CommandLine : String;
    LoadOrderGroup : String;
    TagID : DWord;
    Dependencies : String; // Separated by slash signs (/)
    UserName : String;
    Password : String;
  end;

  TServiceManager = class(TComponent)
  private
    { Private declarations }
    FReconnect : Boolean;
    FMachineName : String;
    FAccess : DWord;
    FHandle : THandle;
    FDBLock : SC_LOCK;
    FServices : TServiceEntries;
    FAfterRefresh : TNotifyEvent;
    FAfterConnect: TNotifyEvent;
    FRefreshOnConnect: Boolean;
    FBeforeDisConnect: TNotifyEvent;
    function GetConnected: Boolean;
    procedure SetConnected(const Value: Boolean);
    procedure SetMachineName(const Value: string);
  protected
    { Protected declarations }
    procedure Loaded;override;
    Procedure SMError(Msg : String);
    Procedure CheckConnected(Msg : String);
    Procedure DoBeforeDisConnect; virtual;
    Procedure DoAfterConnect; virtual;
    Procedure DoAfterRefresh; virtual;
  public
    { Public declarations }
    Constructor Create(AOwner: TComponent); override;
    Destructor Destroy; override;
    Procedure ClearServices;
    Procedure Refresh;
    Procedure Connect;
    Procedure Disconnect;
    function GetServiceHandle(ServiceName: String; SAccess: DWord): THandle;
    procedure ContinueService(SHandle: THandle); overload;
    procedure ContinueService(ServiceName : String); overload;
    procedure StartService(SHandle: THandle; Args: TStrings);overload;
    procedure StartService(ServiceName : String; Args: TStrings); overload;
    procedure StopService(ServiceName: String; StopDependent: Boolean); overload;
    procedure StopService(SHandle : THandle; StopDependent: Boolean); overload;
    procedure PauseService(SHandle: THandle);overload;
    procedure PauseService(ServiceName: String);Overload;
    procedure CustomControlService(ServiceName : String; ControlCode : DWord); overload;
    procedure CustomControlService(Shandle : THandle; ControlCode : DWord); overload;
    procedure ListDependentServices(SHandle: THandle; ServiceState: DWord;  List: TStrings); overload;
    procedure ListDependentServices(ServiceName : String; ServiceState : DWord; List : TStrings); overload;
    Procedure LockServiceDatabase;
    Procedure UnlockServiceDatabase;
    procedure QueryServiceConfig(SHandle : THandle; Var Config : TServiceDescriptor);overload;
    procedure QueryServiceConfig(ServiceName : String; Var Config : TServiceDescriptor);overload;
    Function  RegisterService(Var Desc : TServiceDescriptor) : THandle;
    procedure SetStartupType(ServiceName: String; StartupType: DWord); overload;
    procedure SetStartupType(SHandle : THandle; StartupType: DWord); overload;
    Procedure UnregisterService(ServiceName : String);
    procedure ConfigService(SHandle: THandle; Config: TServiceDescriptor); overload;
    procedure ConfigService(ServiceName : string; Config: TServiceDescriptor); overload;
    procedure RefreshServiceStatus(ServiceName: String);
    procedure GetServiceStatus(SHandle : THandle; Var Status : TServiceStatus); overload;
    procedure GetServiceStatus(ServiceName : String; Var Status : TServiceStatus); overload;
    Property  Handle : THandle Read FHandle;
    Property  Acces : DWord read FAccess Write FAccess;
    Property  Services : TServiceEntries Read FServices;
  published
    { Published declarations }
    Property Connected : Boolean Read GetConnected Write SetConnected;
    Property MachineName : string Read FMachineName Write SetMachineName;
    Property RefreshOnConnect : Boolean Read FRefreshOnConnect Write FrefreshOnConnect;
    Property AfterRefresh : TNotifyEvent Read FAfterRefresh Write FAfterRefresh;
    Property AfterConnect : TNotifyEvent Read FAfterConnect Write FAfterConnect;
    Property BeforeDisConnect : TNotifyEvent Read FBeforeDisConnect Write FBeforeDisConnect;
  end;

  EServiceManager = Class(Exception);

Const
  StartTypes : Array[0..4] of DWord = (
    SERVICE_AUTO_START,SERVICE_BOOT_START, SERVICE_DEMAND_START,
    SERVICE_SYSTEM_START, SERVICE_DISABLED );
  ServiceTypes : Array[0..3] of DWord = (
    SERVICE_FILE_SYSTEM_DRIVER, SERVICE_KERNEL_DRIVER,
    SERVICE_WIN32_OWN_PROCESS, SERVICE_WIN32_SHARE_PROCESS );
  StartErrors : Array[0..3] of DWord = (
    SERVICE_ERROR_IGNORE, SERVICE_ERROR_NORMAL,
    SERVICE_ERROR_SEVERE, SERVICE_ERROR_CRITICAL);

Function ServiceTypeToString(AType : Dword) : String;
Function ServiceStateToString(AState : DWord) : String;
Function ControlsAcceptedToString(AValue : DWord) : String;
Function IsInteractiveService(AType : Dword) : Boolean;

implementation


ResourceString
  SErrConnected       = 'Operation not permitted while connected to Service Control Manager';
  SErrNotConnected    = 'Not connected to Service control manager. Cannot %s';
  SErrInvalidControlCode = 'Invalid custom control code : %d';
  SQueryServiceList   = 'Query service list';
  SActive             = 'Active';
  SInactive           = 'Inactive';
  SStopped            = 'Stopped';
  SStartPending       = 'Start pending';
  SStopPending        = 'Stop pending';
  SRunning            = 'Running';
  SContinuePending    = 'Continue pending';
  SPausePending       = 'Pause pending';
  SPaused             = 'Paused';
  SUnknownState       = 'Unknown State (%d)';
  SUnknownType        = 'Unknown type (%d)';
  SStop               = 'Stop';
  SPauseContinue      = 'Pause/continue';
  SShutDown           = 'Shutdown';
  SDeviceDriver       = 'Device driver';
  SFileSystemDriver   = 'Filesystem driver';
  SAdapter            = 'Adapter';
  SRecognizer         = 'Recognizer';
  SService            = 'Service';
  SSHaredService      = 'Service (shared)';
  SErrServiceNotFound = 'Service "%s" not found.';


{ TServiceManager }

{$ifdef ver130}

Type
  PPChar = ^PChar;
  PCharArray = Array[Word] of PChar;
  PPCharArray = ^PCharArray;

Procedure RaiseLastOSError;

begin
  RaiseLastWin32Error;
end;
{$endif}

procedure TServiceManager.CheckConnected(Msg: String);
begin
  If Not Connected then
    SMError(Format(SErrNotConnected,[Msg]));
end;

procedure TServiceManager.ClearServices;
begin
  FServices.Clear;
end;

procedure TServiceManager.Connect;

Var
   P : PChar;

begin
  If (FHandle=0) then
    begin
    P:=Nil;
    If (MachineName<>'') then
      P:=PChar(MachineName);
    FHandle:=OpenSCManager(P,Nil,FAccess);
    If (FHandle=0) then
      RaiseLastOSError;
    DoAfterConnect;
    If RefreshOnConnect then
      Refresh;
    end;
end;

constructor TServiceManager.Create(AOwner: TComponent);
begin
  inherited;
  FServices:=TServiceEntries.Create(Self,TServiceEntry);
  FAccess:=SC_MANAGER_ALL_ACCESS;
end;

destructor TServiceManager.Destroy;
begin
  FServices.Free;
  Inherited;
end;

procedure TServiceManager.Disconnect;
begin
  IF (FHandle<>0) then
    begin
    DoBeforeDisConnect;
    CloseServiceHandle(FHandle);
    FHandle:=0;
    end;
end;

function TServiceManager.GetConnected: Boolean;
begin
  Result:=(Handle<>0);
end;

procedure TServiceManager.Refresh;

Var
  BytesNeeded,
  ServicesReturned,
  ResumeHandle : DWord;
  Info,P : PEnumServiceStatus;
  E : TServiceEntry;
  I : integer;

begin
  ClearServices;
  CheckConnected(SQueryServiceList);
  BytesNeeded:=0;
  ServicesReturned:=0;
  ResumeHandle:=0;
  Info:=Nil;
  EnumServicesStatus(FHandle,SERVICE_WIN32,SERVICE_STATE_ALL,Info,0,
                     BytesNeeded,ServicesReturned,Resumehandle);
  if (GetLastError<>ERROR_MORE_DATA) then
    RaiseLastOSError;
  Getmem(Info,BytesNeeded);
  Try
    P:=Info;
    If Not EnumServicesStatus(FHandle,SERVICE_WIN32,SERVICE_STATE_ALL,Info,BytesNeeded,
                       BytesNeeded,ServicesReturned,Resumehandle) then
      RaiseLastOSError;
    For I:=1 to Servicesreturned do
      begin
      E:=FServices.Add as TServiceEntry;
      With E,P^ do
        begin
        FServiceName:=StrPas(lpServiceName);
        FDisplayName:=StrPas(lpDisplayName);
        SetStatusFields(ServiceStatus);
        end;
      PChar(P):=Pchar(P)+SizeOf(TEnumServiceStatus);
      end;
    Finally
    FreeMem(Info);
  end;
  DoAfterRefresh;
end;

procedure TServiceManager.SetConnected(const Value: Boolean);
begin
  If (([csLoading,csdesigning] * ComponentState)<>[]) then
    FReconnect:=Value
  else
    If Value<>GetConnected then
      If Value then
        Connect
      Else
        Disconnect;
end;

procedure TServiceManager.Loaded;

begin
  Inherited;
  If FReconnect then
    Connect;
end;

procedure TServiceManager.SetMachineName(const Value: string);
begin
  If Connected then
    SMError(SErrConnected);
  FMachineName := Value;
end;

procedure TServiceManager.SMError(Msg: String);
begin
  raise EServiceManager.Create(Msg);
end;

Function ServiceTypeToString(AType : Dword) : String;

begin
  Case (AType and $FF) of
    SERVICE_KERNEL_DRIVER       : Result:=SDeviceDriver;
    SERVICE_FILE_SYSTEM_DRIVER  : Result:=SFileSystemDriver;
    SERVICE_ADAPTER             : Result:=SAdapter;
    SERVICE_RECOGNIZER_DRIVER   : Result:=SRecognizer;
    SERVICE_WIN32_OWN_PROCESS   : Result:=SService;
    SERVICE_WIN32_SHARE_PROCESS : Result:=SSHaredService;
  else
    Result:=Format(SUnknownType,[AType]);
  end;
end;

Function IsInteractiveService(AType : Dword) : Boolean;

begin
  Result:=(Atype and SERVICE_INTERACTIVE_PROCESS)<>0;
end;

Function ServiceStateToString(AState : Dword) : String;

begin
  Case AState of
    SERVICE_STOPPED          : Result:=SStopped;
    SERVICE_START_PENDING    : Result:=SStartPending;
    SERVICE_STOP_PENDING     : Result:=SStopPending;
    SERVICE_RUNNING          : Result:=SRunning;
    SERVICE_CONTINUE_PENDING : Result:=SContinuePending;
    SERVICE_PAUSE_PENDING    : Result:=SPausePending;
    SERVICE_PAUSED           : Result:=SPaused;
  else
    Result:=Format(SUnknownState,[AState]);
  end;
end;

Function ControlsAcceptedToString(AValue : DWord) : String;

  Procedure AddToResult(S : String);
  begin
    If (Result='') then
      Result:=S
    else
      Result:=Result+','+S
  end;

begin
  Result:='';
  If (AValue and SERVICE_ACCEPT_STOP)<>0 then
    AddToResult(SStop);
  If (AValue and SERVICE_ACCEPT_PAUSE_CONTINUE)<>0 then
    AddToResult(SPauseContinue);
  If (AValue and SERVICE_ACCEPT_SHUTDOWN)<>0 then
    AddToResult(SShutDown)
end;

procedure TServiceManager.DoAfterConnect;
begin
  If Assigned(FAfterConnect) then
    FAfterConnect(Self);
end;

procedure TServiceManager.DoAfterRefresh;
begin
  If Assigned(FAfterRefresh) then
    FAfterRefresh(Self);

end;

procedure TServiceManager.DoBeforeDisConnect;
begin
  If Assigned(FBeforeDisconnect) then
    FBeforeDisconnect(Self);
end;

Function AllocDependencyList (Const S : String) : PChar;

Var
  I,L : Integer;

begin
  Result:=Nil;
  If (S<>'') then
    begin
    // Double Null terminated list of null-terminated strings.
    L:=Length(S);
    GetMem(Result,L+3);
    Move(S[1],Result^,L+1); // Move terminating null as well.
    Result[L+1]:=#0;
    Result[L+2]:=#0;
    For I:=0 to L-1 do
      If Result[i]='/' then // Change / to #0.
        Result[i]:=#0;
    end;
end;

Function TServiceManager.RegisterService(var Desc: TServiceDescriptor) : Thandle;

Var
  PDep,PLO,PUser,PPWd : PChar; // We need Nil for some things.
  N,D : String;
  ReturnTag : DWord;

begin
  With Desc do
    begin
    N:=Name;
    D:=DisplayName;
    If (LoadOrderGroup='') then
      PLO:=Nil
    else
      PLO:=PChar(LoadOrderGroup);
    PPwd:=Nil;
    PUser:=Nil;
    If (UserName<>'') then
      begin
      PUser:=PChar(UserName);
      If (Password<>'') then
        PPWd:=PChar(Password);
      end;
    PDep:=AllocDependencyList(Dependencies);
    Try
      Result:=CreateService(Self.Handle,PChar(N),PChar(D),DesiredAccess,ServiceType,
                            StartType,ErrorControl,PChar(CommandLine),PLO,Nil,
                            PDep,PUser,PPwd);
      If (Result=0) then
        RaiseLastOSError;
    Finally
      If PDep<>Nil then
        FreeMem(PDep);
    end;
    end;
end;

procedure TServiceManager.ListDependentServices(ServiceName : String; ServiceState : DWord; List : TStrings);

Var
  H : THandle;

begin
  H:=OpenService(Handle,PChar(ServiceName),SERVICE_ENUMERATE_DEPENDENTS);
  try
    ListDependentServices(H,ServiceState,List);
  Finally
    CloseServiceHandle(H);
  end;
end;


procedure TServiceManager.ListDependentServices(SHandle: THandle; ServiceState : DWord; List : TStrings);

Var
  P,E : PEnumServiceStatus;
  I,BytesNeeded,Count : DWord;

begin
  P:=Nil;
  List.Clear;
  // If call succeeds with size 0, then there are no dependent services...
  if Not EnumDependentServices(SHandle,ServiceState,P,0,BytesNeeded,Count) then
    begin
    If (GetLastError<>ERROR_MORE_DATA) then
      RaiseLastOSError;
    GetMem(P,BytesNeeded);
    Try
      If Not EnumDependentServices(SHandle,ServiceState,P,bytesNeeded,BytesNeeded,Count) Then
        RaiseLastOSError;
      E:=P;
      For I:=0 to Count-1 do
        begin
        List.Add(StrPas(E^.lpServiceName));
        Pchar(E):=PChar(E)+SizeOf(TEnumServiceStatus);
        end;
    Finally
      FreeMem(P);
    end;
    end;
end;


Procedure TServiceManager.StopService(SHandle : THandle; StopDependent : Boolean);

Var
  I : Integer;
  List : TStrings;
  Status : TServiceStatus;

begin
  If Not QueryServiceStatus(SHandle,Status) then
    RaiseLastOSError;
  If Not (Status.dwCurrentState=SERVICE_STOPPED) then
    begin
    If StopDependent then
      begin
      List:=TStringList.Create;
      Try
        ListDependentServices(SHandle,SERVICE_ACTIVE,List);
        For I:=0 to List.Count-1 do
          StopService(List[i],False); // Do not recurse !!
      Finally
        List.Free;
      end;
      end;
    If Not ControlService(SHandle,SERVICE_CONTROL_STOP,Status) then
      RaiseLastOSError;
    end;
end;

Procedure TServiceManager.StopService(ServiceName : String; StopDependent : Boolean);

Var
  H : THandle;
  A : DWORD;

begin
  A:=SERVICE_STOP or SERVICE_QUERY_STATUS;
  If StopDependent then
    A:=A or SERVICE_ENUMERATE_DEPENDENTS;
  H:=OpenService(Handle,PChar(ServiceName),A);
  Try
    StopService(H,StopDependent);
  Finally
    CloseServiceHandle(H);
  end;
end;


Function TServiceManager.GetServiceHandle(ServiceName : String; SAccess : DWord) : THandle;

begin
  Result:=OpenService(Handle,PChar(ServiceName),SAccess);
  If (Result=0) then
    RaiseLastOSError;
end;

procedure TServiceManager.UnregisterService(ServiceName: String);

Var
  H : THandle;
  Status : TServiceStatus;

begin
  StopService(ServiceName,True);
  H:=GetServiceHandle(ServiceName,SERVICE_STOP or SERVICE_QUERY_STATUS or SERVICE_DELETE);
  Try
    If Not DeleteService(H) then
      RaiseLastOSError;
  Finally
    CloseServiceHandle(H);
  end;
end;

Procedure TServiceManager.PauseService(SHandle : THandle);

Var
  Status : TServiceStatus;

begin
  If Not ControlService(SHandle,SERVICE_CONTROL_PAUSE,Status) then
    RaiseLastOSError;
end;

Procedure TServiceManager.PauseService(ServiceName : String);

Var
  H : THandle;

begin
  H:=GetServiceHandle(ServiceName,SERVICE_PAUSE_CONTINUE);
  Try
    PauseService(H);
  Finally
    CloseServiceHandle(H);
  end;
end;

Procedure TServiceManager.ContinueService(SHandle : THandle);

Var
  Status : TServiceStatus;

begin
  If Not ControlService(SHandle,SERVICE_CONTROL_CONTINUE,Status) then
    RaiseLastOSError;
end;

Procedure TServiceManager.ContinueService(ServiceName : String);

Var
  H : THandle;

begin
  H:=GetServiceHandle(ServiceName,SERVICE_PAUSE_CONTINUE);
  Try
    ContinueService(H);
  Finally
    CloseServiceHandle(H);
  end;
end;

Function StringsToPCharList(List : TStrings) : PPChar;

Var
  I : Integer;
  S : String;

begin
  I:=(List.Count)+1;
  GetMem(Result,I*sizeOf(PChar));
  PPCharArray(Result)^[List.Count]:=Nil;
  For I:=0 to List.Count-1 do
    begin
    S:=List[i];
    PPCharArray(Result)^[i]:=StrNew(PChar(S));
    end;
end;

Procedure FreePCharList(List : PPChar);

Var
  I : integer;

begin
  I:=0;
  While PPChar(List)[i]<>Nil do
    begin
    StrDispose(PPChar(List)[i]);
    Inc(I);
    end;
  FreeMem(List);
end;

Procedure TServiceManager.StartService(SHandle : THandle; Args : TStrings);

Var
  Argc : DWord;
  PArgs : PPchar;

begin
  If (Args=Nil) or (Args.Count>0) then
    begin
    Argc:=0;
    Pargs:=Nil;
    end
  else
    begin
    ArgC:=Args.Count;
    Pargs:=StringsToPcharList(Args);
    end;
  Try
    If not jwawinsvc.StartService(SHandle,Argc,PArgs^) then
      RaiseLastOSError;
  Finally
    If (PArgs<>Nil) then
      FreePCharList(PArgs);
  end;
end;


Procedure TServiceManager.StartService(ServiceName : String; Args : TStrings);

Var
  H : THandle;

begin
  H:=GetServiceHandle(ServiceName,SERVICE_START);
  Try
    StartService(H,Args);
  Finally
    CloseServiceHandle(H);
  end;
end;

Procedure TServiceManager.LockServiceDatabase;

begin
  FDBLock:=jwawinsvc.LockServiceDatabase(Handle);
  If FDBLock=Nil then
    RaiseLastOSError;
end;

procedure TServiceManager.UnlockServiceDatabase;
begin
  If (FDBLock<>Nil) then
    begin
    Try
      If Not jwawinsvc.UnLockServiceDatabase(FDBLock) then
        RaiseLastOSError;
    Finally
      FDBLock:=Nil;
    end;
    end;
end;

procedure TServiceManager.QueryServiceConfig(SHandle : THandle; Var Config : TServiceDescriptor);

Var
  SvcCfg : PQueryServiceConfig;
  BytesNeeded : DWord;

begin
  jwawinsvc.QueryServiceConfig(SHandle,Nil,0,BytesNeeded);
  If (GetLastError<>ERROR_INSUFFICIENT_BUFFER) then
    RaiseLastOSError;
  GetMem(SvcCfg,BytesNeeded);
  Try
    If Not jwawinsvc.QueryServiceConfig(SHandle,SvcCfg,BytesNeeded,BytesNeeded) then
      RaiseLastOSError;
    With config,SvcCfg^ do
      begin
      Password:='';
      Name:='';
      DesiredAccess:=0;
      ErrorControl:=dwErrorControl;
      ServiceType:=dwServiceType;
      StartType:=dwStartType;
      TagID:=dwTagID;
      CommandLine:=lpBinaryPathName;
      LoadOrderGroup:=lpLoadOrderGroup;
      Dependencies:=lpDependencies;
      UserName:=lpServiceStartName;
      DisplayName:=lpDisplayName;
      end;
  Finally
    FreeMem(SvcCfg,BytesNeeded);
  end;
end;

procedure TServiceManager.QueryServiceConfig(ServiceName : String; Var Config : TServiceDescriptor);

Var
  H : THandle;

begin
  H:=GetServiceHandle(ServiceName,SERVICE_QUERY_CONFIG);
  Try
    QueryServiceConfig(H,Config);
  Finally
    CloseServiceHandle(H);
  end;
end;

procedure TServiceManager.SetStartupType(ServiceName : String; StartupType : DWord);

Var
  H : THandle;

begin
  H:=GetServiceHandle(ServiceName,SERVICE_CHANGE_CONFIG);
  Try
    SetStartupType(H,StartupType);
  Finally
    CloseServiceHandle(H);
  end;
end;

procedure TServiceManager.SetStartupType(SHandle : THandle; StartupType: DWord);

Const
  SNC = SERVICE_NO_CHANGE;  // Shortcut

begin
  If Not ChangeServiceConfig(SHandle,SNC,StartupType,SNC,Nil,Nil,Nil,Nil,Nil,Nil,Nil) then
    RaiseLastOSError;
end;

procedure TServiceManager.ConfigService(SHandle : THandle ; Config : TServiceDescriptor);

  Function SToPchar(Var S : String) : PChar;

  begin
    If (S='') then
      Result:=Nil
    else
      Result:=PChar(S);
  end;

Var
  PDep,PLO,PUser,PPWd,PCmd,PDisp : PChar; // We need Nil for some things.
  D : String;
  ReturnTag : DWord;

begin
  With Config do
    begin
    PCmd:=SToPChar(CommandLine);
    D:=DisplayName;
    PDisp:=StoPChar(D);
    PLO:=SToPChar(LoadOrderGroup);
    PUser:=SToPChar(UserName);
    PPwd:=SToPchar(Password);
    PDep:=AllocDependencyList(Dependencies);
    Try
      If Not ChangeServiceConfig(SHandle,ServiceType,StartType,ErrorControl,
                                  PCmd,PLO,Nil,PDep,PUser,PPwd,PDisp) then
        RaiseLastOSError;
    Finally
      If PDep<>Nil then
        FreeMem(PDep);
    end;
    end;
end;

procedure TServiceManager.GetServiceStatus(SHandle : THandle; Var Status: TServiceStatus);

begin
  If Not QueryServiceStatus(SHandle,Status) then
    RaiseLastOSError;
end;

procedure TServiceManager.GetServiceStatus(ServiceName : String; Var Status: TServiceStatus);

Var
  H : THandle;

begin
  H:=GetServiceHandle(ServiceName,SERVICE_QUERY_STATUS);
  Try
    GetServiceStatus(H,Status);
  Finally
    CloseServiceHandle(H);
  end;
end;

procedure TServiceManager.RefreshServiceStatus(ServiceName : String);

Var
  Status : TServiceStatus;
  SE : TServiceEntry;


begin
  SE:=Services.ServiceByName(ServiceName);
  GetServiceStatus(ServiceName,Status);
  SE.SetStatusFields(Status);
end;


procedure TServiceManager.ConfigService(ServiceName : String; Config : TServiceDescriptor);

Var
  H : THandle;

begin
  H:=GetServiceHandle(ServiceName,SERVICE_CHANGE_CONFIG);
  Try
    ConfigService(H,Config);
  Finally
    CloseServiceHandle(H);
  end;
end;


procedure TServiceManager.CustomControlService(ServiceName: String; ControlCode: DWord);

Var
  H : THandle;

begin
  H:=GetServiceHandle(ServiceName,SERVICE_USER_DEFINED_CONTROL);
  Try
    CustomControlService(H,ControlCode);
  Finally
    CloseServiceHandle(H);
  end;
end;

procedure TServiceManager.CustomControlService(Shandle: THandle;
  ControlCode: DWord);

Var
  Status : TServiceStatus;

begin
  If (ControlCode<128) or (ControlCode>255) then
    Raise EServiceManager.CreateFmt(SErrInvalidControlCode,[ControlCode]);
  If Not ControlService(SHandle,ControlCode,Status) then
    RaiseLastOSError;
end;

{ TServiceEntries }

function TServiceEntries.FindService(ServiceName: String): TServiceEntry;

Var
  I : Integer;

begin
  Result:=Nil;
  I:=Count-1;
  While (I>=0) and (Result=Nil) do
    If CompareText(Items[i].ServiceName,ServiceName)=0 then
      Result:=Items[i]
    else
      Dec(I);
end;

function TServiceEntries.GetService(Index: Integer): TServiceEntry;
begin
  Result:=inherited Items[Index] as TServiceEntry;
end;

function TServiceEntries.ServiceByName(ServiceName: String): TServiceEntry;

begin
  Result:=FindService(ServiceName);
  If Result=Nil then
    Raise EServiceManager.CreateFmt(SErrServiceNotFound,[ServiceName]);
end;

{ TServiceEntry }

procedure TServiceEntry.SetStatusFields(const Status: TServiceStatus);
begin
  With Status do
    begin
    FServiceType:=dwServiceType;
    FCurrentState:=dwCurrentState;
    FControlsAccepted:=dwControlsAccepted;
    FWin32ExitCode:=dwWin32ExitCode;
    FServiceSpecificExitCode:=dwServiceSpecificExitCode;
    FCheckPoint:=dwCheckPoint;
    FWaitHint:=dwWaitHint;
    end;
end;

end.
