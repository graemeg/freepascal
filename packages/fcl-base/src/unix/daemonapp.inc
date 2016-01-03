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
    T.Resume;
    T.WaitFor;
    FDaemon.FThread:=Nil;
  except
    On E : Exception do
      FDaemon.Logmessage(Format(SErrDaemonStartFailed,[FDaemon.Definition.Name,E.Message]));
  end;
end;

procedure TDaemonController.Controller(ControlCode, EventType: DWord;
  EventData: Pointer);

begin
  // Send control code to daemon thread.
end;

function TDaemonController.ReportStatus: Boolean;

Var
  S : String;

begin
  S:='';
  If Assigned(FDaemon) then
    With FDaemon do
      S:=Format(SDaemonStatus,[Definition.DisplayName,
                               CurrentStatusNames[Status]]);
  Application.Logger.Info(S);
end;

Procedure TDaemonController.ThreadTerminated(Sender : TObject);

begin
end;


{ ---------------------------------------------------------------------
  Global initialization/Finalization
  ---------------------------------------------------------------------}

Procedure DoShutDown(Sig : Longint; Info : PSigInfo; Context : PSigContext); cdecl;

begin
  Application.StopDaemons(True);
  Application.Terminate;
end;

Procedure SysInitDaemonApp;

Var
  old,new : SigactionRec;

begin
  New.sa_handler:=@DoShutDown;
  fpSigaction(SIGQUIT,@New,@Old);
  fpSigaction(SIGTERM,@New,@Old);
  fpSigaction(SIGINT,@New,@Old);
end;


Procedure SysDoneDaemonApp;

begin
end;


