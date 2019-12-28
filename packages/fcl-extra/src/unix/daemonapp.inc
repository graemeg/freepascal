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
{ System dependent service stuff }

uses baseunix;

Resourcestring
  SErrFailedToFork          = 'Failed to fork daemon process.';

procedure DaemonizeProgram;
var pid, sid : TPid;
begin
  pid := FpFork;
  if (pid<0) then
    raise Exception.Create(SErrFailedToFork);
  if pid>0 then
    begin
    // We are now in the main program, which has to terminate
    FpExit(0);
    end
  else
    begin
    // Here we are in the daemonized proces
    sid := FpSetsid;
    if sid < 0 then
      raise Exception.Create(SErrFailedToFork);
    // Reset the file-mask
    FpUmask(0);
    // Change the current directory, to avoid locking the current directory
    chdir('/');
    FpClose(StdInputHandle);
    FpClose(StdOutputHandle);
    FpClose(StdErrorHandle);
    end;
end;

{ ---------------------------------------------------------------------
  TCustomDaemonApplication
  ---------------------------------------------------------------------}
Const
  SERVICE_CONTROL_STOP                  = $00000001;
  SERVICE_CONTROL_PAUSE                 = $00000002;
  SERVICE_CONTROL_CONTINUE              = $00000003;
  SERVICE_CONTROL_INTERROGATE           = $00000004;
  SERVICE_CONTROL_SHUTDOWN              = $00000005;
  
function TCustomDaemonApplication.RunGUIloop(P: Pointer): integer;
begin

end;


procedure TCustomDaemonApplication.SysInstallDaemon(Daemon: TCustomDaemon);
begin

end;

procedure TCustomDaemonApplication.SysUnInstallDaemon(Daemon: TCustomDaemon);
begin

end;

procedure TCustomDaemonApplication.SysStartUnInstallDaemons;
begin

end;

procedure TCustomDaemonApplication.SysEndUnInstallDaemons;
begin

end;


procedure TCustomDaemonApplication.SysStartInstallDaemons;
begin

end;

procedure TCustomDaemonApplication.SysEndInstallDaemons;
begin

end;

procedure TCustomDaemonApplication.SysStartRunDaemons;
begin
  if Application.HasOption('b','background') then
    DaemonizeProgram;
end;

procedure TCustomDaemonApplication.SysEndRunDaemons;

Var
  I : Integer;
  DC : TDaemonController;
  
begin
  For I:=ComponentCount-1 downto 0 do
    If Components[i] is TDaemoncontroller then
      begin
      DC:=Components[i] as TDaemoncontroller;
      DC.Main(0,Nil); // Returns after starting thread.
      end;
  if Assigned(GUIMainLoop) then
    GuiMainLoop
  else
    // Simply wait till everything terminates.
    While Not Terminated do
      fpPause;
end;

procedure TCustomDaemonApplication.RemoveController(
  AController: TDaemonController);
  
Var
  I : Integer;
  HC : Boolean;
  
begin
  FreeAndNil(AController.FDaemon);
  AController.Free;
end;


{ ---------------------------------------------------------------------
  TDaemonThread
  ---------------------------------------------------------------------}

procedure TDaemonThread.StartServiceExecute;
begin

end;

procedure TDaemonThread.CheckControlMessage(WaitForMessage : Boolean);
begin
  If WaitForMessage then // Don't hog CPU.
    Sleep(50);
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

begin
  FDaemon.Status:=csStartPending;
  Try
    T:=TDaemonThread.Create(FDaemon);
    T.FreeOnTerminate:=True;
    T.OnTerminate := @FDaemon.DoThreadTerminate;
    T.Resume;
  except
    On E : Exception do
      FDaemon.Logmessage(Format(SErrDaemonStartFailed,[FDaemon.Definition.Name,E.Message]));
  end;
end;

procedure TDaemonController.Controller(ControlCode, EventType: DWord;
  EventData: Pointer);

begin
  // Send control code to daemon thread.
  TDaemonThread(Daemon.DaemonThread).HandleControlCode(ControlCode);
end;

function TDaemonController.ReportStatus: Boolean;

Var
  S : String;
  doLog : Boolean;

begin
  S:='';
  DoLog:=True;
  If Assigned(FDaemon) then
    With FDaemon do
      begin
      S:=Format(SDaemonStatus,[Definition.DisplayName,
                               CurrentStatusNames[Status]]);
      if Assigned(Definition) then
        doLog:=Definition.LogStatusReport;
      end;  
  if DoLog then    
    Application.Log(etInfo,S);
end;

{ ---------------------------------------------------------------------
  Global initialization/Finalization
  ---------------------------------------------------------------------}
 
{$if defined(BeOS) and not defined(Haiku)}
Procedure DoShutDown(Sig : Longint; SigContext: PSigContextRec; uContext : Pvregs);cdecl;
{$else}
Procedure DoShutDown(Sig : Longint; Info : PSigInfo; Context : PSigContext); cdecl;
{$endif}
begin
  Application.StopDaemons(True);
  Application.Terminate;
end;

Procedure SysInitDaemonApp;

  Procedure installhandler(aSig : Longint);

  Var
    old,new : SigactionRec;

  begin
    FillChar(New,SizeOf(Sigactionrec),#0);
    FillChar(Old,SizeOf(Sigactionrec),#0);
    New.sa_handler:=@DoShutDown;
    fpSigaction(aSig,@New,@Old);
  end;

begin
  InstallHandler(SIGQUIT);
  InstallHandler(SIGTERM);
  InstallHandler(SIGINT);
end;


Procedure SysDoneDaemonApp;

begin
end;


