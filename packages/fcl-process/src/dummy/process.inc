{
  Dummy process.inc - the simplest version based on SysUtils.ExecuteProcess
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

Resourcestring
  SNoCommandLine        = 'Cannot execute empty command-line';
  SErrCannotExecute     = 'Failed to execute %s : %d';
  SErrNoSuchProgram     = 'Executable not found: "%s"';

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{
  prevent compilation error for the versions mentioned below
} 
{$if defined(VER2_4) or defined(VER2_5_1)}
{$warning Temporary workaround - unit does nothing}
{$else}
{$fatal Proper implementation of TProcess for version of this target needed}
{$endif}
>>>>>>> graemeg/cpstrnew

procedure TProcess.CloseProcessHandles;
begin
end;

Function TProcess.PeekExitStatus : Boolean;
begin
  Result := true; (* Dummy version assumes always synchronous execution *)
end;

function GetNextWordPos (const S: string): integer;
const
  WhiteSpace = [' ', #9, #10, #13];
  Literals = ['"', ''''];
var
  WStart: integer;
  InLiteral: boolean;
  LastLiteral: char;
begin
  WStart := 1;
(* Skip whitespaces at the beginning *)
  while (WStart <= Length (S)) and (S [WStart] in WhiteSpace) do
   Inc (WStart);
  InLiteral := false;
  LastLiteral := #0;
  while (WStart <= Length (S)) and
                               (not (S [WStart] in WhiteSpace) or InLiteral) do
   begin
    if S [WStart] in Literals then
     if InLiteral then
      InLiteral := not (S [WStart] = LastLiteral)
     else
      begin
       InLiteral := true;
       LastLiteral := S [WStart];
      end;
     Inc (WStart);
    end;
(* Skip whitespaces at the end *)
  while (WStart <= Length (S)) and (S [WStart] in WhiteSpace) do
   Inc (WStart);
  Result := WStart;
end;

function MaybeQuote (const S: string): string;
begin
  if (Pos (' ', S) <> 0) then
   Result := '"' + S + '"'
  else
   Result := S;
end;

Procedure TProcess.Execute;
var
  I: integer;
  ExecName, FoundName: string;
  E2: EProcess;
  OrigDir: string;
  Params: string;
begin
  if (ApplicationName = '') and (CommandLine = '') and (Executable = '') then
   raise EProcess.Create (SNoCommandline);
  if (FApplicationName <> '') then
   ExecName := FApplicationName;
  if (FCommandLine <> '') then
   begin
    Params := FCommandLine;
    if ExecName = '' then
     begin
      I := GetNextWordPos (Params);
      ExecName := Copy (Params, 1, Pred (I));
      Trim (ExecName);
      Delete (Params, 1, Pred (I));
     end
    else if Copy (FCommandLine, 1, Length (ExecName)) = ExecName then
     Delete (Params, 1, Succ (Length (ExecName)))
    else
     Delete (Params, 1, Pred (GetNextWordPos (Params)));
    Trim (Params);
   end
  else
   for I := 1 to Pred (Parameters.Count) do
    Params := Params + ' ' + MaybeQuote (Parameters [I]);
  if (FExecutable <> '') and (ExecName = '') then
   ExecName := Executable;
  if not FileExists (ExecName) then
   begin
    FoundName := ExeSearch (ExecName, '');
    if FoundName <> '' then
     ExecName := FoundName
    else
     raise EProcess.CreateFmt (SErrNoSuchProgram, [ExecName]);
   end;
  if (FCurrentDirectory <> '') then
   begin
    GetDir (0, OrigDir);
    ChDir (FCurrentDirectory);
   end;
  try
   FExitCode := ExecuteProcess (ExecName, Params);
  except
(* Normalize the raised exception so that it is aligned to other platforms. *)
    On E: EOSError do
     begin
      raise EProcess.CreateFmt (SErrCannotExecute, [FCommandLine, E.ErrorCode]);
      if (FCurrentDirectory <> '') then
       ChDir (OrigDir);
      end;
  end;
  if (FCurrentDirectory <> '') then
   ChDir (OrigDir);
end;

Function TProcess.WaitOnExit : Boolean;
begin
  Result:=True;
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


