unit DaemonUnit1;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DaemonApp;

type

  { TDaemon1 }

  TDaemon1 = class(TDaemon)
    procedure DataModuleExecute(Sender: TCustomDaemon);
  private

  public

  end;

var
  Daemon1: TDaemon1;

implementation

procedure RegisterDaemon;
begin
  RegisterDaemonClass(TDaemon1)
end;

{$R *.lfm}

{ TDaemon1 }

procedure TDaemon1.DataModuleExecute(Sender: TCustomDaemon);
Var
  I : Integer;
begin
  I := 0;
  Application.EventLog.Log('TDaemon1 execution start');
  While Self.Status = csRunning Do Begin
    Sleep(10);
  end;
  Application.EventLog.Log('TDaemon1 execution stop');
end;


initialization
  RegisterDaemon;
end.

