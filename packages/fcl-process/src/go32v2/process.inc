{
  Dummy process.inc
}

{$if defined(VER2_2) or defined(VER2_3_1)}
{$warning Temporary workaround - unit does nothing}
{$else}
{$fatal Proper implementation of TProcess needed}
{$endif}

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


