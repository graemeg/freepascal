unit daemonunit2;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DaemonApp;

type

  { TDaemon2 }

  TDaemon2 = class(TDaemon)
    procedure DataModuleExecute(Sender: TCustomDaemon);
  private

  public

  end;

var
  Daemon2: TDaemon2;

implementation

procedure RegisterDaemon;
begin
  RegisterDaemonClass(TDaemon2)
end;

{$R *.lfm}

{ TDaemon2 }

procedure TDaemon2.DataModuleExecute(Sender: TCustomDaemon);
Var
  I : Integer;
begin
  I := 0;
  Application.EventLog.Log('TDaemon2 execution start');
  While Self.Status = csRunning Do Begin
    Sleep(10);
  end;
  Application.EventLog.Log('TDaemon2 execution stop');
end;


initialization
  RegisterDaemon;
end.

