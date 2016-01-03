{
  Dummy process.inc
}

procedure TProcess.CloseProcessHandles;
begin
end;

Function TProcess.PeekExitStatus : Boolean;
begin
end;

Procedure TProcess.Execute;
begin
end;

Function TProcess.WaitOnExit : Boolean;
begin
  Result:=False;
end;

Function TProcess.Suspend : Longint;
begin
  Result:=0;
end;

Function TProcess.Resume : LongInt;

begin
  Result:=0;
end;

Function TProcess.Terminate(AExitCode : Integer) : Boolean;
begin
  Result:=False;
end;

Procedure TProcess.SetShowWindow (Value : TShowWindowOptions);
begin
end;


