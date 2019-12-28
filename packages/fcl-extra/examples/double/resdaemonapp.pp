{
 *****************************************************************************
  See the file COPYING.modifiedLGPL.txt, included in this distribution,
  for details about the license.
 *****************************************************************************
}
{$mode objfpc}
{$h+}
unit resdaemonapp;

interface

uses daemonapp;

Type
  TResDaemonApplication = Class(TCustomDaemonApplication)
    Procedure CreateDaemonInstance(Var ADaemon : TCustomDaemon; DaemonDef : TDaemonDef); override;
  end;

implementation

uses classes;

Procedure TResDaemonApplication.CreateDaemonInstance(Var ADaemon : TCustomDaemon; DaemonDef : TDaemonDef); 

begin
  ADaemon:=DaemonDef.DaemonClass.Create(Self);
end;

Initialization
  RegisterDaemonApplicationClass(TResDaemonApplication)
end.
