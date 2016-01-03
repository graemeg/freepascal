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
{ Win32 implementation of service application }

uses windows,jwawinsvc;

Const
  CM_SERVICE_CONTROL_CODE = WM_USER+1;
  
Resourcestring
  SErrRegisterHandler  = 'Could not register control handler, error code %d';
  SErrNoControlContext = 'Not handling Control message without control context: (%d %d %d).';
  SControlCodeReceived = 'Control message received: (%d %d %d).';

function StartServiceCtrlDispatcher(lpServiceStartTable: LPSERVICE_TABLE_ENTRY): BOOL; stdcall; external 'advapi32.dll' name 'StartServiceCtrlDispatcherA';
function RegisterServiceCtrlHandlerEx(lpServiceName: LPCSTR;lpHandlerProc: LPHANDLER_FUNCTION_EX; lpContext: LPVOID): SERVICE_STATUS_HANDLE; stdcall;external 'advapi32.dll' name 'RegisterServiceCtrlHandlerExA';
function SetServiceStatus(hServiceStatus: SERVICE_STATUS_HANDLE; const lpServiceStatus: SERVICE_STATUS): BOOL; stdcall; external 'advapi32.dll' name 'SetServiceStatus';

{ ---------------------------------------------------------------------
  Win32 entry points
  ---------------------------------------------------------------------}

Function ServiceControlHandlerEntry(dwControl,dwEventType: DWord; lpEventData,lpContext : Pointer) : DWord; StdCall;

begin
  If (Nil<>lpContext) then
    TDaemonController(lpContext).Controller(dwControl,dwEventType,lpEventData)
  else
    If Assigned(Application.Logger) then
      Application.Logger.Error(SerrNoControlContext,[dwControl,dwEventType,ptrint(lpEventData)]);
end;

Procedure ServiceMainEntry(Argc : DWord; Args : Pchar); stdcall;

begin
  If Assigned(Application) then
    Application.Main(Argc,PPChar(Args));
{$ifdef svcdebug}Debuglog('Main end');{$endif}
end;

{ ---------------------------------------------------------------------
  TDaemonStartThread
  ---------------------------------------------------------------------}
Type
  TDaemonStartThread = Class(TThread)
    FEntryTable : PServiceTableEntry;
    FLoopHandle : THandle;
  Public
    Constructor Create(T : PServiceTableEntry; LoopHandle : THandle);
    Procedure DoTerminate; override;
    Procedure Execute; override;
    Property ReturnValue;
  end;

{ TDaemonStartThread }

constructor TDaemonStartThread.create(T: PServiceTableEntry; LoopHandle : THandle);
begin
  FEntryTable:=T;
  FLoopHandle:=LoopHandle;
  FreeOnTerminate:=False;
  ReturnValue:=0;
  Inherited Create(False,DefaultStackSize*2);
end;

procedure TDaemonStartThread.DoTerminate;
begin
  Inherited DoTerminate;
  If (FLoopHandle<>0) then
    PostMessage(FLoopHandle,WM_QUIT,0,0);
end;

procedure TDaemonStartThread.Execute;
begin
{$ifdef svcdebug}DebugLog('Calling service dispatcher');{$endif svcdebug}
  if StartServiceCtrlDispatcher(FEntryTable) then
    ReturnValue:=0
  else
    ReturnValue:=GetLastError;
{$ifdef svcdebug}DebugLog('Called service dispatcher');{$endif svcdebug}
end;

{ ---------------------------------------------------------------------
  TSCMData : private data of controller.
  ---------------------------------------------------------------------}

Type
  TSCMData = Class(TObject)
    FHandle : SERVICE_STATUS_HANDLE;
    Constructor Create(AHandle : SERVICE_STATUS_HANDLE);
  end;

Constructor TSCMData.Create(AHandle : SERVICE_STATUS_HANDLE);
begin
  FHandle:=AHandle;
end;

{ ---------------------------------------------------------------------
  TSMData : private data of Application.
  ---------------------------------------------------------------------}

Type

  { TSMData }

  TSMData = Class(TObject)
  Private
    FHandle : SC_HANDLE;
  Public
    Constructor Create(AHandle : SC_HANDLE);
    Destructor Destroy; override;
  end;

Constructor TSMData.Create(AHandle : SC_HANDLE);
begin
  FHandle:=AHandle;
end;

destructor TSMData.Destroy;
begin
  inherited;
end;

{ ---------------------------------------------------------------------
  TCustomDaemonApplication
  ---------------------------------------------------------------------}
  
const
  WinServiceTypes : array[TServiceType] of Integer
                  = (SERVICE_WIN32_OWN_PROCESS, SERVICE_KERNEL_DRIVER,
                     SERVICE_FILE_SYSTEM_DRIVER);

  WinStartTypes : array[TStartType] of Integer
                = (SERVICE_BOOT_START, SERVICE_SYSTEM_START,
                   SERVICE_AUTO_START, SERVICE_DEMAND_START,
                   SERVICE_DISABLED);
  WinErrorSeverities : array[TErrorSeverity] of Integer
                     = (SERVICE_ERROR_IGNORE, SERVICE_ERROR_NORMAL,
                        SERVICE_ERROR_SEVERE, SERVICE_ERROR_CRITICAL);

// Careful, result of this function must be freed !!

Function GetDependencies(D : TDependencies) : PChar;
  
var
  I,L : Integer;
  P   : PChar;

begin
  Result:=Nil;
  L:=0;
  for i:=0 to D.Count-1 do
    Inc(L, Length(D[i].Name)+1+Ord(D[i].IsGroup));
  if (L<>0) then
    begin
    Inc(L); // For final null-terminator;
    GetMem(Result,L);
    P:=Result;
    for i:=0 to D.Count - 1 do
      begin
      if D[i].IsGroup then
        begin
        P^:=Char(SC_GROUP_IDENTIFIER);
        Inc(P);
        end;
      P:=StrECopy(P,PChar(D[i].Name));
      Inc(P);
      end;
    P^:=#0;
    end;
end;

Procedure TCustomDaemonApplication.SysInstallDaemon(Daemon : TCustomDaemon);

Var
  SM,SV: SC_HANDLE;
  N,DN,E,LG,UN,UP : String;
  DD : TDaemonDef;
  ST,STT,ES: Integer;
  IDTag : DWord;
  PIDTag : LPDWord;
  PDeps,PN,PP : PChar;
  
begin
  SM:=TSMData(FSysData).FHandle;
  DD:=Daemon.Definition;
  E:=Paramstr(0);
  If (Pos(' ',E)<>0) then
    E:='"'+E+'"';
  E:=E+' --run'; // Add --run argument;
  If (DD.RunArguments<>'') then
    E:=E+' '+DD.RunArguments;
  N:=DD.Name;
  DN:=DD.DisplayName;
  
  With DD.WinBindings do
    begin
    LG:=GroupName;
    UN:=UserName;
    If (UN='') then
      PN:=Nil
    else
      PN:=PChar(UN);
    UP:=Password;
    if (UP='') then
      PP:=Nil
    else
      PP:=PChar(UP);
    // ServiceType ST
      ST:=WinServiceTypes[ServiceType];
    if (doInteractive in DD.Options) and (ServiceType=stWin32)  then
      ST:=ST or SERVICE_INTERACTIVE_PROCESS;
    // Actually, we should check count of enabled definitons only :/
    if (ServiceType=stWin32) and (FMapper.DaemonDefs.Count>1) then
      ST:=(ST xor SERVICE_WIN32_OWN_PROCESS) or SERVICE_WIN32_SHARE_PROCESS;
    // StartType STT
    STT:=WinStartTypes[StartType];
    if (StartType in [stBoot, stSystem]) and (ServiceType<>stDevice) then
      STT:=SERVICE_AUTO_START;
    IDTag:=IDTag;
    If (IDTag<>0) then
      PIDTag:=@IDTag
    else
      PIDTag:=Nil;
    ES:=WinErrorSeverities[ErrorSeverity];
    PDeps:=GetDependencies(Dependencies);
    end;
  Try
    flush(output);
    SV:=CreateService(SM, PChar(N), PChar(DN), SERVICE_ALL_ACCESS, ST, STT, ES,
                      PChar(E), PChar(LG), PIDTag, PDeps, PN, PP);
    If (SV=0) then
      RaiseLastOSError;
    Try
      If (PIDTag<>Nil) then
        DD.WinBindings.IDTag:=IDTag;
    finally
      CloseServiceHandle(SV);
    end
  Finally
    If (PDeps<>Nil) then
      FreeMem(PDeps);
  end;
end;

procedure TCustomDaemonApplication.SysUnInstallDaemon(Daemon: TCustomDaemon);

var
  SM,SV : SC_HANDLE;
  DN : String;
begin
  SM:=TSMData(FSysData).FHandle;
  DN:=Daemon.Definition.Name;
  SV:=OpenService(SM,PChar(DN),SERVICE_ALL_ACCESS);
  if (SV=0) then
    RaiseLastOSError;
  try
    if Not DeleteService(SV) then
      RaiseLastOSError;
  finally
    CloseServiceHandle(SV);
  end;
end;

procedure TCustomDaemonApplication.SysStartUnInstallDaemons;
Var
  SM : SC_HANDLE;

begin
  SM:=OpenSCManager(Nil,Nil,SC_MANAGER_ALL_ACCESS);
  if (SM=0) then
    RaiseLastOSError;
  FSysData:=TSMData.Create(SM);
end;

procedure TCustomDaemonApplication.SysEndUnInstallDaemons;
begin
  CloseServiceHandle(TSMData(FSysData).FHandle);
  FreeandNil(FSysData);
end;


procedure TCustomDaemonApplication.SysStartInstallDaemons;

Var
  SM : SC_HANDLE;
  
begin
  SM:=OpenSCManager(Nil,Nil,SC_MANAGER_ALL_ACCESS);
  if (SM=0) then
    RaiseLastOSError;
  FSysData:=TSMData.Create(SM);
end;

procedure TCustomDaemonApplication.SysEndInstallDaemons;
begin
  CloseServiceHandle(TSMData(FSysData).FHandle);
  FreeandNil(FSysData);
end;

procedure TCustomDaemonApplication.SysStartRunDaemons;
begin
  // Do nothing.
end;

Function TCustomDaemonApplication.RunGUIloop(P : Pointer) : integer;

Const
  HandleOpts=WS_POPUP or WS_CAPTION or WS_CLIPSIBLINGS or WS_SYSMENU or WS_MINIMIZEBOX;

Var
  T : TDaemonStartThread;
  Msg : TMsg;
  TClass: TWndClass;
  AWClass: TWndClass = (
    style: 0;
    lpfnWndProc: @DefWindowProc;
    cbClsExtra: 0;
    cbWndExtra: 0;
    hInstance: 0;
    hIcon: 0;
    hCursor: 0;
    hbrBackground: 0;
    lpszMenuName: nil;
    lpszClassName: 'TDaemonApplication');

begin
  If (GUIHandle=0) then
    begin
    if not GetClassInfo(HInstance,AWClass.lpszClassName,TClass) then
      begin
      AWClass.hInstance := HInstance;
      if Windows.RegisterClass(AWClass) = 0 then
        DaemonError(SErrWindowClass);
      end;
    GUIHandle := CreateWindow(AWClass.lpszClassName, Pchar(Title),
      HandleOpts, 1,1, 0, 0, 0, 0, HInstance, nil);
    end;
  T:=TDaemonStartThread.Create(P,GUIHandle);
  Try
    If Assigned(GuiMainLoop) then
       GUIMainLoop
    else
      begin
      // Run a message loop.
      Msg.Message:=0;
      Repeat
        if PeekMessage(Msg, 0, 0, 0, PM_REMOVE) then
          begin
          if (Msg.Message<>WM_QUIT) and (Msg.Message<>WM_ENDSESSION) then
            begin
            TranslateMessage(Msg);
            DispatchMessage(Msg);
            end
          else
            Terminate;
          end;
      Until Terminated;
      end;
  finally
    Result:=T.ReturnValue;
    T.Free;
  end;
end;

procedure TCustomDaemonApplication.SysEndRunDaemons;

Var
  P : PServiceTableEntry;
  I,C : Integer;
  RV : Integer;
  
begin
  GetMem(P,SizeOf(TServiceTableEntry)*FMapper.DaemonDefs.Count+1);
  Try
    C:=FMapper.DaemonDefs.Count;
    For I:=0 to C-1 do
      If Assigned(FMapper.DaemonDefs[i].Instance) then
        begin
        P[i].lpServiceName:=Pchar(FMapper.DaemonDefs[i].Name);
        P[i].lpServiceProc:=@ServiceMainEntry;
        end;
    // Set last entry to Nil.
    P[C].lpServiceName:=Nil;
    P[C].lpServiceProc:=Nil;
    If IsConsole then
      begin
       {$ifdef svcdebug}DebugLog('Starting ctrl dispatcher');{$endif svcdebug}
      Try
        if StartServiceCtrlDispatcher(P) then
          begin
          {$ifdef svcdebug}DebugLog('Return of dispatcher OK');{$endif svcdebug}
          RV:=0;
          end
        else
          begin
          RV:=GetLastError;
          {$ifdef svcdebug}DebugLog('Return of dispatcher NOK');{$endif svcdebug}
          end;
      except
        On E : Exception do
          begin
          {$ifdef svcdebug}Debuglog('Caught exception : '+E.MEssage);{$endif svcdebug}
          Raise;
          end;
      end;
      end
    else
      begin
      RV:=RunGuiLoop(P);
      end;
    {$ifdef svcdebug}DebugLog('SysRun Terminating');{$endif svcdebug}
    Terminate;
    If (RV<>0) then
      Logger.Error(SErrServiceManagerStartFailed,[SysErrorMessage(RV)]);
  Finally
    FreeMem(P);
  end;
end;

procedure TCustomDaemonApplication.RemoveController(
  AController: TDaemonController);
  
begin
  FreeAndNil(AController.FDaemon);
  AController.Free;
end;

{ ---------------------------------------------------------------------
  TDaemonThread
  ---------------------------------------------------------------------}

procedure TDaemonThread.StartServiceExecute;

Var
  Msg : TMsg;
  
begin
  PeekMessage(Msg,0,WM_USER,WM_USER,PM_NOREMOVE);
end;


procedure TDaemonThread.CheckControlMessage(WaitForMessage : Boolean);

Var
  Msg : TMsg;
  StopLoop : Boolean;

begin
  StopLoop:=False;
  Repeat
    StopLoop:=Terminated and WaitForMessage;
    If Not StopLoop then
      begin
      If WaitForMessage Then
        StopLoop:=Not GetMessage(Msg,0,0,0)
      else
        StopLoop:=Not PeekMessage(Msg,0,0,0,PM_REMOVE);
      If Not StopLoop then
        begin
        If (Msg.hwnd<>0) or (Msg.Message<>CM_SERVICE_CONTROL_CODE) then
          DispatchMessage(Msg)
        else
          HandleControlCode(Msg.wParam);
        end;
      end;
  Until StopLoop;
end;

{ ---------------------------------------------------------------------
  TDaemonController
  ---------------------------------------------------------------------}

procedure TDaemonController.StartService;
begin
  Main(0,Nil);
end;


procedure TDaemonController.Main(Argc: DWord; Args: PPChar);

Var
  T : TThread;
  H : SERVICE_STATUS_HANDLE;
  I : Integer;
  
begin
  For I:=0 to Argc-1 do
    FParams.Add(StrPas(Args[I]));
  H:=RegisterServiceCtrlHandlerEx(Args[0],@ServiceControlHandlerEntry,Self);
  if (H=0) then
    Application.Logger.Error(SErrRegisterHandler,[getlasterror]);
  FSysData:=TSCMData.Create(H);
  FDaemon.Status:=csStartPending;
  Try
    T:=TDaemonThread.Create(FDaemon);
    T.Resume;
    T.WaitFor;
    FreeAndNil(T);
    FDaemon.FThread:=Nil;
  except
    On E : Exception do
      FDaemon.Logmessage(Format(SErrDaemonStartFailed,[FDaemon.Definition.Name,E.Message]));
  end;
end;

procedure TDaemonController.Controller(ControlCode, EventType: DWord;
  EventData: Pointer);

Var
  TID : THandle;

begin
  if Assigned(FDaemon.FThread) then
    begin
    TID:=FDaemon.FThread.ThreadID;
    If FDaemon.FThread.Suspended then
      FDaemon.FThread.Resume;
    PostThreadMessage(TID,CM_SERVICE_CONTROL_CODE,ControlCode,EventType);
    end;
end;


function TDaemonController.ReportStatus: Boolean;

  Function GetAcceptedCodes : Integer;

  begin
    Result := SERVICE_ACCEPT_SHUTDOWN;
    if doAllowStop in FDAemon.Definition.Options then
      Result := Result or SERVICE_ACCEPT_STOP;
    if doAllowPause in FDAemon.Definition.Options then
      Result := Result or SERVICE_ACCEPT_PAUSE_CONTINUE;
  end;

Var
  S : String;
  DD : TDaemonDef;
  SS : TServiceStatus;
  WB : TWinBindings;

Const
  WinServiceStatus : array[TCurrentStatus] of Integer
                   = (SERVICE_STOPPED, SERVICE_START_PENDING,
                      SERVICE_STOP_PENDING, SERVICE_RUNNING,
                      SERVICE_CONTINUE_PENDING, SERVICE_PAUSE_PENDING,
                      SERVICE_PAUSED);
  PendingStatus : set of TCurrentStatus
                = [csStartPending, csStopPending,
                   csContinuePending,csPausePending];

begin
  If not Assigned(FDaemon) then
    begin
    Application.Logger.Error(SErrNoDaemonForStatus,[Name]);
    Exit;
    end;
  DD:=FDaemon.Definition;
  If not Assigned(DD) then
    begin
    Application.Logger.Error(SErrNoDaemonDefForStatus,[Name]);
    Exit;
    end;
  DD.LogStatusReport:=True;
{$ifndef svcdebug}
  If DD.LogStatusReport then
{$endif svcdebug}
    With FDaemon do
      begin
      S:=Format(SDaemonStatus,[Definition.DisplayName, CurrentStatusNames[Status]]);
      Application.Logger.Info(S);
      {$ifdef svcdebug}DebugLog(S);{$endif svcdebug}
      end;
  FillChar(SS,SizeOf(SS),0);
  WB:=DD.WinBindings;
  with SS do
    begin
    dwWaitHint := WB.WaitHint;
    dwServiceType :=WinServiceTypes[WB.ServiceType];
    if (FDaemon.Status=csStartPending) then
      dwControlsAccepted := 0
    else
      dwControlsAccepted := GetAcceptedCodes;
    if (FDaemon.Status in PendingStatus) and (FDaemon.Status = LastStatus) then
      Inc(FCheckPoint)
    else
      FCheckPoint := 0;
    dwCheckPoint:=FCheckPoint;
    FLastStatus := FDaemon.Status;
    dwCurrentState := WinServiceStatus[FDaemon.Status];
    dwServiceSpecificExitCode:=WB.ErrCode;
    if (WB.ErrCode<>0) then
      dwWin32ExitCode:=ERROR_SERVICE_SPECIFIC_ERROR
    else
      dwWin32ExitCode := WB.Win32ErrCode;
    if not SetServiceStatus(TSCMData(FSysData).FHandle, SS) then
      Application.Logger.Error(SysErrorMessage(GetLastError));
  end;
end;

Procedure TDaemonController.ThreadTerminated(Sender : TObject);

begin
end;


{ ---------------------------------------------------------------------
  Global initialization/Finalization
  ---------------------------------------------------------------------}

Procedure SysInitDaemonApp;

begin
end;


Procedure SysDoneDaemonApp;

begin
end;


