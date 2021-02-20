{
  Dummy process.inc - the simplest version based on SysUtils.ExecuteProcess
}

uses
  Exec, AmigaDos, Utility;

Resourcestring
  SNoCommandLine        = 'Cannot execute empty command-line';
  SErrCannotExecute     = 'Failed to execute %s : %d';
  SErrNoSuchProgram     = 'Executable not found: "%s"';


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

var
  UID: Integer = 0;

{$ifdef MorphOS}
const
  BUF_LINE = 0; // flush on \n, etc
  BUF_FULL = 1; // never flush except when needed
  BUF_NONE = 2; // no buffering
{$endif}

Procedure TProcess.Execute;
var
  I: integer;
  ExecName, FoundName: string;
  E2: EProcess;
  OrigDir: string;
  Params: string;
  TempName: string;
  cos: BPTR;
  {$ifdef MorphOS}
  inA, inB, OutA, OutB: BPTR;
  Res: Integer;
  {$endif}
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
      ExecName := Trim (ExecName);
      Delete (Params, 1, Pred (I));
     end
    else if Copy (FCommandLine, 1, Length (ExecName)) = ExecName then
     Delete (Params, 1, Succ (Length (ExecName)))
    else
     Delete (Params, 1, Pred (GetNextWordPos (Params)));
    Params := Trim (Params);
   end
  else
   for I := 0 to Pred (Parameters.Count) do
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
    {$ifdef MorphOS}
    if (poUsePipes in Options) and (not (poWaitOnExit in Options)) then
    begin
      FProcessID := 0;
      // Pipenames, should be unique
      TempName := 'PIPE:PrO_' + HexStr(Self) + HexStr(GetTickCount, 8);
      inA := DOSOpen(PChar(TempName), MODE_OLDFILE);
      inB := DOSOpen(PChar(TempName), MODE_NEWFILE);
      TempName := TempName + 'o';
      outA := DOSOpen(PChar(TempName), MODE_OLDFILE);
      outB := DOSOpen(PChar(TempName), MODE_NEWFILE);
      // set buffer for all pipes
      SetVBuf(inA, nil, BUF_NONE, -1);
      SetVBuf(inB, nil, BUF_LINE, -1);
      SetVBuf(outA, nil, BUF_NONE, -1);
      SetVBuf(outB, nil, BUF_LINE, -1);
      // the actual Start of the command with given parameter and streams
      Res := SystemTags(PChar(ExecName + ' ' + Params),
                        [SYS_Input, AsTag(outA),
                         SYS_Output, AsTag(inB),
                         SYS_Asynch, AsTag(True),
                         TAG_END]);
      // the two streams will be destroyed by system, we do not need to care about
      // the other two we will destroy when the PipeStreams they are attached to are destroyed
      if Res <> -1 then
      begin
        FProcessID := 1;
        CreateStreams(THandle(outB), THandle(inA),0);
      end
      else
      begin
        // if the command did not start, we need to delete all Streams
        if outB <> BPTR(0) then DosClose(outB);
        if outA <> BPTR(0) then DosClose(outA);
        if inB <> BPTR(0) then DosClose(inB);
        if inA <> BPTR(0) then DosClose(inA);
      end;
    end
    else
    {$endif}
    begin
      // if no streams needed we still use the old sychronous way
      FProcessID := 0;
      cos := BPTR(0);
      repeat
        Inc(UID);
        TempName := 'T:PrO_'+ HexStr(FindTask(nil)) + '_' + IntToHex(UID,8);
      until not FileExists(TempName);
      //sysdebugln('TProcess start: "' + ExecName + ' ' + Params+'"  >' + TempName);
      cos := AmigaDos.DosOpen(PChar(TempName), MODE_READWRITE);
      FExitCode := LongInt(amigados.Execute(PChar(ExecName + ' ' + Params), BPTR(0), cos));
      DosSeek(cos, 0, OFFSET_BEGINNING);
      CreateStreams(0, THandle(cos),0);
    end;
    //FExitCode := ExecuteProcess (ExecName, Params);
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

Function TProcess.WaitOnExit(Timeout : DWord) : Boolean;
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


