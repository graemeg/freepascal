{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2008 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
 
{$DEFINE OS_HASEXITCODE}
uses
   ctypes,
   UnixType,
   Unix,
   Baseunix;

Resourcestring
  SNoCommandLine        = 'Cannot execute empty command-line';
  SErrNoSuchProgram     = 'Executable not found: "%s"';
  SErrNoTerminalProgram = 'Could not detect X-Terminal program';
  SErrCannotFork        = 'Failed to Fork process';
  SErrCannotCreatePipes = 'Failed to create pipes';

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Const
  PriorityConstants : Array [TProcessPriority] of Integer =
                      (20,20,0,-20);

Const
  GeometryOption : String = '-geometry';
  TitleOption : String ='-title';

procedure TProcess.CloseProcessHandles;

begin
 // Do nothing. Win32 call.
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TProcess.GetExitCode : Integer;

begin
  if (Not Running) and wifexited(FExitCode) then
    Result:=wexitstatus(FExitCode)
  else
    Result:=0;
end;

Function TProcess.PeekExitStatus : Boolean;
var
  res: cint;
begin
  repeat
    res:=fpWaitPid(Handle,pcint(@FExitCode),WNOHANG);
  until (res<>-1) or (fpgeterrno<>ESysEINTR);
  result:=res=Handle;
  If Not Result then
    FexitCode:=cardinal(-1); // was 0, better testable for abnormal exit.
=======
=======
>>>>>>> origin/fixes_2_2
Function TProcess.PeekExitStatus : Boolean;

begin
  Result:=fpWaitPid(Handle,pcint(@FExitCode),WNOHANG)=Handle;
  If Result then
    FExitCode:=wexitstatus(FExitCode)
  else
    FexitCode:=0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Type
  TPCharArray = Array[Word] of pchar;
  PPCharArray = ^TPcharArray;

Function StringsToPCharList(List : TStrings) : PPChar;

Var
  I : Integer;
  S : String;

begin
  I:=(List.Count)+1;
  GetMem(Result,I*sizeOf(PChar));
  PPCharArray(Result)^[List.Count]:=Nil;
  For I:=0 to List.Count-1 do
    begin
    S:=List[i];
    Result[i]:=StrNew(PChar(S));
    end;
end;

Procedure FreePCharList(List : PPChar);

Var
  I : integer;

begin
  I:=0;
  While List[i]<>Nil do
    begin
    StrDispose(List[i]);
    Inc(I);
    end;
  FreeMem(List);
end;


<<<<<<< HEAD
<<<<<<< HEAD

Function DetectXterm : String;

  Function TestTerminal(S : String) : Boolean;

  begin
    Result:=FileSearch(s,GetEnvironmentVariable('PATH'),False)<>'';
    If Result then
      XTermProgram:=S;
  end;

  Function TestTerminals(Terminals : Array of String) : Boolean;

  Var
    I : integer;
  begin
    I:=Low(Terminals);
    Result:=False;
    While (Not Result) and (I<=High(Terminals)) do
      begin
      Result:=TestTerminal(Terminals[i]);
      inc(i);
      end;
  end;

Const
  Konsole   = 'konsole';
  GNomeTerm = 'gnome-terminal';
  DefaultTerminals : Array [1..6] of string
                   = ('x-terminal-emulator','xterm','aterm','wterm','rxvt','xfce4-terminal');

Var
  D :String;

begin
  If (XTermProgram='') then
    begin
    // try predefined
    If Length(TryTerminals)>0 then
      TestTerminals(TryTerminals);
    // try session-specific terminal
    if (XTermProgram='') then
      begin
      D:=LowerCase(GetEnvironmentVariable('DESKTOP_SESSION'));
      If (Pos('kde',D)<>0) then
        begin
        TestTerminal('konsole');
        end
      else if (D='gnome') then
        begin
        TestTerminal('gnome-terminal');
        end
      else if (D='windowmaker') then
        begin
        If not TestTerminal('aterm') then
          TestTerminal('wterm');
        end
      else if (D='xfce') then
        TestTerminal('xfce4-terminal');
      end;
    if (XTermProgram='') then
      TestTerminals(DefaultTerminals)
    end;
  Result:=XTermProgram;
  If (Result='') then
    Raise EProcess.Create(SErrNoTerminalProgram);
end;

Function MakeCommand(P : TProcess) : PPchar;

{$ifdef darwin}
Const
  TerminalApp = 'open';
{$endif}
{$ifdef haiku}
Const
  TerminalApp = 'Terminal';
{$endif}
  
=======
=======
>>>>>>> origin/fixes_2_2
Procedure CommandToList(S : String; List : TStrings);

  Function GetNextWord : String;

  Const
    WhiteSpace = [' ',#8,#10];
    Literals = ['"',''''];

  Var
    Wstart,wend : Integer;
    InLiteral : Boolean;
    LastLiteral : char;

  begin
    WStart:=1;
    While (WStart<=Length(S)) and (S[WStart] in WhiteSpace) do
      Inc(WStart);
    WEnd:=WStart;
    InLiteral:=False;
    LastLiteral:=#0;
    While (Wend<=Length(S)) and (Not (S[Wend] in WhiteSpace) or InLiteral) do
      begin
      if S[Wend] in Literals then
        If InLiteral then
          InLiteral:=Not (S[Wend]=LastLiteral)
        else
          begin
          InLiteral:=True;
          LastLiteral:=S[Wend];
          end;
       inc(wend);
       end;

     Result:=Copy(S,WStart,WEnd-WStart);

     if  (Length(Result) > 0)
     and (Result[1] = Result[Length(Result)]) // if 1st char = last char and..
     and (Result[1] in Literals) then // it's one of the literals, then
       Result:=Copy(Result, 2, Length(Result) - 2); //delete the 2 (but not others in it)

     While (WEnd<=Length(S)) and (S[Wend] in WhiteSpace) do
       inc(Wend);
     Delete(S,1,WEnd-1);

  end;

Var
  W : String;

begin
  While Length(S)>0 do
    begin
    W:=GetNextWord;
    If (W<>'') then
      List.Add(W);
    end;
end;


Function MakeCommand(P : TProcess) : PPchar;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Var
  Cmd : String;
  S  : TStringList;
  G : String;
  
begin
<<<<<<< HEAD
<<<<<<< HEAD
  If (P.ApplicationName='') and (P.CommandLine='') and (P.Executable='') then
    Raise EProcess.Create(SNoCommandline);
  S:=TStringList.Create;
  try
    if (P.ApplicationName='') and (P.CommandLine='') then
      begin
      S.Assign(P.Parameters);
      S.Insert(0,P.Executable);
      end
    else
      begin
      If (P.CommandLine='') then
        Cmd:=P.ApplicationName
      else
        Cmd:=P.CommandLine;
      CommandToList(Cmd,S);
      end;
    if poNewConsole in P.Options then
      begin
      {$ifdef haiku}
      If (P.ApplicationName<>'') then
        begin
        S.Insert(0,P.ApplicationName);
        S.Insert(0,'--title');
        end;
      {$endif}
      {$if defined(darwin) or defined(haiku)}
      S.Insert(0,TerminalApp);
      {$else}
=======
=======
>>>>>>> origin/fixes_2_2
  if (P.ApplicationName='') then
    begin
    If (P.CommandLine='') then
      Raise EProcess.Create(SNoCommandline);
    Cmd:=P.CommandLine;
    end
  else
    begin
    If (P.CommandLine='') then
      Cmd:=P.ApplicationName
    else
      Cmd:=P.CommandLine;
    end;
  S:=TStringList.Create;
  try
    CommandToList(Cmd,S);
    if poNewConsole in P.Options then
      begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      S.Insert(0,'-e');
      If (P.ApplicationName<>'') then
        begin
        S.Insert(0,P.ApplicationName);
        S.Insert(0,'-title');
        end;
      if suoUseCountChars in P.StartupOptions then
        begin
        S.Insert(0,Format('%dx%d',[P.dwXCountChars,P.dwYCountChars]));
        S.Insert(0,'-geometry');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      If (P.XTermProgram<>'') then
        S.Insert(0,P.XTermProgram)
      else
        S.Insert(0,DetectXterm);
=======
      S.Insert(0,'xterm');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      {$endif}
      end;
    {$ifndef haiku}
=======
      S.Insert(0,'xterm');
      end;
>>>>>>> graemeg/fixes_2_2
=======
      S.Insert(0,'xterm');
      end;
>>>>>>> origin/fixes_2_2
    if (P.ApplicationName<>'') then
      begin
      S.Add(TitleOption);
      S.Add(P.ApplicationName);
      end;
    G:='';
    if (suoUseSize in P.StartupOptions) then
      g:=format('%dx%d',[P.dwXSize,P.dwYsize]);
    if (suoUsePosition in P.StartupOptions) then
      g:=g+Format('+%d+%d',[P.dwX,P.dwY]);
    if G<>'' then
      begin
      S.Add(GeometryOption);
      S.Add(g);
      end;
<<<<<<< HEAD
<<<<<<< HEAD
    {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Result:=StringsToPcharList(S);
  Finally
    S.free;
  end;
end;

Function GetLastError : Integer;

begin
  Result:=-1;
end;

Type
  TPipeEnd = (peRead,peWrite);
  TPipePair = Array[TPipeEnd] of cint;

Procedure CreatePipes(Var HI,HO,HE : TPipePair; CE : Boolean);

  Procedure CreatePair(Var P : TPipePair);

   begin
    If not CreatePipeHandles(P[peRead],P[peWrite]) then
<<<<<<< HEAD
<<<<<<< HEAD
      Raise EProcess.Create(SErrCannotCreatePipes);
=======
      Raise EProcess.Create('Failed to create pipes');
>>>>>>> graemeg/fixes_2_2
=======
      Raise EProcess.Create('Failed to create pipes');
>>>>>>> origin/fixes_2_2
   end;

  Procedure ClosePair(Var P : TPipePair);

  begin
    if (P[peRead]<>-1) then
      FileClose(P[peRead]);
    if (P[peWrite]<>-1) then
      FileClose(P[peWrite]);
  end;

begin
  HO[peRead]:=-1;HO[peWrite]:=-1;
  HI[peRead]:=-1;HI[peWrite]:=-1;
  HE[peRead]:=-1;HE[peWrite]:=-1;
  Try
    CreatePair(HO);
    CreatePair(HI);
    If CE then
      CreatePair(HE);
  except
    ClosePair(HO);
    ClosePair(HI);
    If CE then
      ClosePair(HE);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function safefpdup2(fildes, fildes2 : cInt): cInt;
begin
  repeat
    safefpdup2:=fpdup2(fildes,fildes2);
  until (safefpdup2<>-1) or (fpgeterrno<>ESysEINTR);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Procedure TProcess.Execute;

Var
  HI,HO,HE : TPipePair;
  PID      : Longint;
  FEnv     : PPChar;
  Argv     : PPChar;
  fd       : Integer;
<<<<<<< HEAD
<<<<<<< HEAD
  res      : cint;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FoundName,
  PName    : String;

begin
  If (poUsePipes in FProcessOptions) then
    CreatePipes(HI,HO,HE,Not (poStdErrToOutPut in FProcessOptions));
  Try
    if FEnvironment.Count<>0 then
      FEnv:=StringsToPcharList(FEnvironment)
    else
      FEnv:=Nil;
    Try
      Argv:=MakeCommand(Self);
      Try
        If (Argv<>Nil) and (ArgV[0]<>Nil) then
          PName:=StrPas(Argv[0])
        else
          begin
          // This should never happen, actually.
          PName:=ApplicationName;
          If (PName='') then
            PName:=CommandLine;
          end;

        if not FileExists(PName) then begin
          FoundName := ExeSearch(Pname,fpgetenv('PATH'));
          if FoundName<>'' then
            PName:=FoundName
          else
            raise EProcess.CreateFmt(SErrNoSuchProgram,[PName]);
        end;

<<<<<<< HEAD
<<<<<<< HEAD
{$if (defined(DARWIN) or defined(SUNOS))}
        { can't use vfork in case the child has to be
          suspended immediately, because with vfork the
          child borrows the execution thread of the parent
          unit it either exits or execs -> potential 
          deadlock depending on how quickly the SIGSTOP
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          signal is delivered

          We also can't use vfork in case we have to change the working
          directory, use pipes or not use a console since calling anything but
          exec* or _exit after vfork is unsupported. For the same reason, also
          don't use vfork in case there is a forkevent (since we don't know
          what that one will call) }
        if (([poRunSuspended,PoUsePipes,poNoConsole] * Options) = []) and
           (FCurrentDirectory='') and
           not assigned(FForkEvent) then
=======
          signal is delivered }
        if not(poRunSuspended in Options) then
>>>>>>> graemeg/cpstrnew
=======
          signal is delivered }
        if not(poRunSuspended in Options) then
>>>>>>> graemeg/cpstrnew
=======
          signal is delivered }
        if not(poRunSuspended in Options) then
>>>>>>> graemeg/cpstrnew
=======
          signal is delivered }
        if not(poRunSuspended in Options) then
>>>>>>> origin/cpstrnew
          Pid:=fpvfork
        else
          Pid:=fpfork;
{$else}
        Pid:=fpfork;
{$endif}
        if Pid<0 then
          Raise EProcess.Create(SErrCannotFork);
        if (PID>0) then
          begin
            // Parent process. Copy process information.
            FProcessHandle:=PID;
            FThreadHandle:=PID;
            FProcessId:=PID;
            //FThreadId:=PID;
          end
        else
          begin
            { We're in the child }
            if (FCurrentDirectory<>'') then
               begin
{$push}{$i-}
                 ChDir(FCurrentDirectory);
                 { exit if the requested working directory does not exist (can
                   use DirectoryExists, that would not be atomic; cannot change
                   before forking because that would also change the CWD of the
                   parent, which could influence other threads }
                 if ioresult<>0 then
                   fpexit(127);
{$pop}
               end;
            if PoUsePipes in Options then
              begin
                FileClose(HI[peWrite]);
                safefpdup2(HI[peRead],0);
                FileClose(HO[peRead]);
                safefpdup2(HO[peWrite],1);
                if (poStdErrToOutPut in Options) then
                  safefpdup2(HO[peWrite],2)
                else
                  begin
                    FileClose(HE[peRead]);
                    safefpdup2(HE[peWrite],2);
                  end
              end
            else if poNoConsole in Options then
              begin
                fd:=FileOpen('/dev/null',fmOpenReadWrite or fmShareDenyNone);
                safefpdup2(fd,0);
                safefpdup2(fd,1);
                safefpdup2(fd,2);
              end;
            if Assigned(FForkEvent) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              FForkEvent(Self);
=======
              FForkEvent;
>>>>>>> graemeg/cpstrnew
=======
              FForkEvent;
>>>>>>> graemeg/cpstrnew
=======
              FForkEvent;
>>>>>>> graemeg/cpstrnew
=======
              FForkEvent;
>>>>>>> origin/cpstrnew
            if (poRunSuspended in Options) then
              fpkill(fpgetpid,SIGSTOP);
            if FEnv<>Nil then
              fpexecve(PName,Argv,Fenv)
            else
              fpexecv(PName,argv);
            fpExit(127);
=======
=======
>>>>>>> origin/fixes_2_2
        Pid:=fpfork;
        if Pid<0 then
          Raise EProcess.Create('Failed to Fork process');
        if (PID>0) then
          begin
          // Parent process. Copy process information.
          FProcessHandle:=PID;
          FThreadHandle:=PID;
          FProcessId:=PID;
          //FThreadId:=PID;
          end
        else
          begin
          { We're in the child }
          if (FCurrentDirectory<>'') then
             ChDir(FCurrentDirectory);
          if PoUsePipes in Options then
            begin
            fpclose(HI[peWrite]);
            fpdup2(HI[peRead],0);
            fpclose(HO[peRead]);
            fpdup2(HO[peWrite],1);
            if (poStdErrToOutPut in Options) then
              fpdup2(HO[peWrite],2)
            else
              begin
              fpclose(HE[peRead]);
              fpdup2(HE[peWrite],2);
              end
            end
          else if poNoConsole in Options then
            begin
            fd:=FileOpen('/dev/null',fmOpenReadWrite);
            fpdup2(fd,0);
            fpdup2(fd,1);
            fpdup2(fd,2);
            end;
          if (poRunSuspended in Options) then
            sigraise(SIGSTOP);
          if FEnv<>Nil then
            fpexecve(PName,Argv,Fenv)
          else
            fpexecv(PName,argv);
          Halt(127);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          end
      Finally
        FreePcharList(Argv);
      end;
    Finally
      If (FEnv<>Nil) then
        FreePCharList(FEnv);
    end;
  Finally
    if POUsePipes in FProcessOptions then
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        FileClose(HO[peWrite]);
        FileClose(HI[peRead]);
        if Not (poStdErrToOutPut in FProcessOptions) then
          FileClose(HE[peWrite]);
        CreateStreams(HI[peWrite],HO[peRead],HE[peRead]);
=======
=======
>>>>>>> origin/fixes_2_2
      FileClose(HO[peWrite]);
      FileClose(HI[peRead]);
      if Not (poStdErrToOutPut in FProcessOptions) then
        FileClose(HE[peWrite]);
      CreateStreams(HI[peWrite],HO[peRead],HE[peRead]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;
  end;
  FRunning:=True;
  if not (csDesigning in ComponentState) and // This would hang the IDE !
     (poWaitOnExit in FProcessOptions) and
      not (poRunSuspended in FProcessOptions) then
    WaitOnExit;
end;

Function TProcess.WaitOnExit : Boolean;

Var
  R : Dword;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FRunning then
    fexitcode:=waitprocess(handle);
  Result:=(fexitcode>=0);
=======
=======
>>>>>>> origin/fixes_2_2
  R:=fpWaitPid(Handle,pcint(@FExitCode),0);
  Result:=(R=Handle);
  If Result then
    FExitCode:=WExitStatus(FExitCode);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FRunning:=False;
end;

Function TProcess.Suspend : Longint;

begin
  If fpkill(Handle,SIGSTOP)<>0 then
    Result:=-1
  else
    Result:=1;
end;

Function TProcess.Resume : LongInt;

begin
  If fpKill(Handle,SIGCONT)<>0 then
    Result:=-1
  else
    Result:=0;
end;

Function TProcess.Terminate(AExitCode : Integer) : Boolean;

begin
  Result:=False;
  Result:=fpkill(Handle,SIGTERM)=0;
  If Result then
    begin
    If Running then
      Result:=fpkill(Handle,SIGKILL)=0;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
  { the fact that the signal has been sent does not
    mean that the process has already handled the
    signal -> wait instead of calling getexitstatus }
  if Result then
    WaitOnExit;
=======
  GetExitStatus;
>>>>>>> graemeg/fixes_2_2
=======
  GetExitStatus;
>>>>>>> origin/fixes_2_2
end;

Procedure TProcess.SetShowWindow (Value : TShowWindowOptions);

begin
  FShowWindow:=Value;
end;

