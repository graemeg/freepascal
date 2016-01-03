program daemon;

{$mode objfpc}{$H+}
{$define usecthreads}
{$apptype gui}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  sysutils,
  Classes
  { add your units here }, daemonapp;

Type

  { TTestDaemon }

  { TTestThread }

  TTestThread = Class(TThread)
    Procedure Execute; override;
  end;

  TTestDaemon = Class(TCustomDaemon)
  Private
    FThread : TTestThread;
    Procedure ThreadStopped (Sender : TObject);
  public
    Function Start : Boolean; override;
    Function Stop : Boolean; override;
    Function Pause : Boolean; override;
    Function Continue : Boolean; override;
    Function Execute : Boolean; override;
    Function ShutDown : Boolean; override;
    Function Install : Boolean; override;
    Function UnInstall: boolean; override;
  end;

{ TTestThread }

procedure TTestThread.Execute;

Var
  C : Integer;

begin
  C:=0;
  Repeat
    Sleep(1000);
    inc(c);
    Application.Logger.Info(Format('Tick : %d',[C]));
  Until Terminated;
end;

Procedure AWriteln(MSg : String; B : Boolean);

begin
  Application.Logger.Info(Msg+BoolToStr(B));
end;

{ TTestDaemon }

procedure TTestDaemon.ThreadStopped(Sender: TObject);
begin
  FreeAndNil(FThread);
end;

function TTestDaemon.Start: Boolean;
begin
  Result:=inherited Start;
  AWriteln('Daemon Start',Result);
  FThread:=TTestThread.Create(True);
  FThread.OnTerminate:=@ThreadStopped;
  FThread.FreeOnTerminate:=False;
  FThread.Resume;
end;

function TTestDaemon.Stop: Boolean;
begin
  Result:=inherited Stop;
  AWriteln('Daemon Stop: ',Result);
  FThread.Terminate;
end;

function TTestDaemon.Pause: Boolean;
begin
  Result:=inherited Pause;
  AWriteln('Daemon pause: ',Result);
  FThread.Suspend;
end;

function TTestDaemon.Continue: Boolean;
begin
  Result:=inherited Continue;
  AWriteln('Daemon continue: ',Result);
  FThread.Resume;
end;

function TTestDaemon.Execute: Boolean;
begin
  Result:=inherited Execute;
  AWriteln('Daemon execute: ',Result);
end;

function TTestDaemon.ShutDown: Boolean;
begin
  Result:=inherited ShutDown;
  AWriteln('Daemon Shutdown: ',Result);
  FThread.Terminate;
end;

function TTestDaemon.Install: Boolean;
begin
  Result:=inherited Install;
  AWriteln('Daemon Install: ',Result);
end;

function TTestDaemon.UnInstall: boolean;
begin
  Result:=inherited UnInstall;
  AWriteln('Daemon UnInstall: ',Result);
end;

Type

  { TTestDaemonMapper }

  TTestDaemonMapper = Class(TCustomDaemonMapper)
    Constructor Create(AOwner : TComponent); override;
  end;

{ TTestDaemonMapper }

constructor TTestDaemonMapper.Create(AOwner: TComponent);

Var
  D : TDaemonDef;

begin
  inherited Create(AOwner);
  D:=DaemonDefs.Add as TDaemonDef;
  D.DisplayName:='Test daemon';
  D.Name:='TestDaemon';
  D.DaemonClassName:='TTestDaemon';
  D.WinBindings.ServiceType:=stWin32;
end;

begin
  RegisterDaemonClass(TTestDaemon);
  RegisterDaemonMapper(TTestDaemonMapper);
  Application.Title:='Daemon test application';
  Application.Run;
end.

