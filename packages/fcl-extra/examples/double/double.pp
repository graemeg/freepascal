Program double;

Uses
{$IFDEF UNIX}
  CThreads,
{$ENDIF}
  ResDaemonApp, DaemonApp,  DaemonMapperUnit1, DaemonUnit1, daemonunit2, SysUtils, eventlog
  { add your units here };

Var
  AExecutableFilenamePath : String;
begin
  AExecutableFilenamePath := ParamStr(0);
  AExecutableFilenamePath := ExpandFileName(AExecutableFilenamePath);
  AExecutableFilenamePath := ExtractFilePath(AExecutableFilenamePath);
  Application.Title:='Daemon application';
  Application.Initialize;
  Application.EventLog.FileName := SysUtils.ConcatPaths([AExecutableFilenamePath, 'event-log.txt']);
  Application.EventLog.LogType := ltFile;
  Application.EventLog.AppendContent := False;
  Application.EventLog.Active := True;
  Application.Run;
end.
