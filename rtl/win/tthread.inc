{ Thread management routines }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TThread.SysCreate(CreateSuspended: Boolean;
                            const StackSize: SizeUInt);
=======
constructor TThread.Create(CreateSuspended: Boolean;
                           const StackSize: SizeUInt = DefaultStackSize);
>>>>>>> graemeg/cpstrnew
=======
constructor TThread.Create(CreateSuspended: Boolean;
                           const StackSize: SizeUInt = DefaultStackSize);
>>>>>>> graemeg/cpstrnew
=======
constructor TThread.Create(CreateSuspended: Boolean;
                           const StackSize: SizeUInt = DefaultStackSize);
>>>>>>> graemeg/cpstrnew
=======
constructor TThread.Create(CreateSuspended: Boolean;
                           const StackSize: SizeUInt = DefaultStackSize);
>>>>>>> origin/cpstrnew
begin
  FSuspended := CreateSuspended;
  FInitialSuspended := CreateSuspended;
  { Always start in suspended state, will be resumed in AfterConstruction if necessary
    See Mantis #16884 }
  FHandle := BeginThread(nil, StackSize, @ThreadProc, pointer(self), CREATE_SUSPENDED,
                         FThreadID);
  if FHandle = TThreadID(0) then
    raise EThread.CreateFmt(SThreadCreateError, [SysErrorMessage(getlasterror)]);

  FFatalException := nil;
end;


procedure TThread.SysDestroy;
begin
  if FHandle<>0 then
    begin
      { Don't check Suspended. If the thread has been externally suspended (which is
        deprecated and strongly discouraged), it's better to deadlock here than
        to silently free the object and leave OS resources leaked. }
      if not FFinished {and not Suspended} then
        begin
          Terminate;
          { Allow the thread function to perform the necessary cleanup. Since
            we've just set Terminated flag, it won't call Execute. }
          if FInitialSuspended then
            Start;
          WaitFor;
        end;
    end;

  FFatalException.Free;
  FFatalException := nil;
end;

procedure TThread.CallOnTerminate;
begin
  FOnTerminate(Self);
end;

procedure TThread.DoTerminate;
begin
  if Assigned(FOnTerminate) then
    Synchronize(@CallOnTerminate);
end;

const
  Priorities: array [TThreadPriority] of Integer =
   (THREAD_PRIORITY_IDLE, THREAD_PRIORITY_LOWEST, THREAD_PRIORITY_BELOW_NORMAL,
    THREAD_PRIORITY_NORMAL, THREAD_PRIORITY_ABOVE_NORMAL,
    THREAD_PRIORITY_HIGHEST, THREAD_PRIORITY_TIME_CRITICAL);

function TThread.GetPriority: TThreadPriority;
var
  P: Integer;
  I: TThreadPriority;
begin
  P := GetThreadPriority(FHandle);
  Result := tpNormal;
  for I := Low(TThreadPriority) to High(TThreadPriority) do
    if Priorities[I] = P then Result := I;
end;

procedure TThread.SetPriority(Value: TThreadPriority);
begin
  SetThreadPriority(FHandle, Priorities[Value]);
end;


procedure TThread.SetSuspended(Value: Boolean);
begin
  if Value <> FSuspended then
    if Value then
      Suspend
    else
      Resume;
end;

procedure TThread.Suspend;
begin
  FSuspended := True;
  SuspendThread(FHandle);
end;

procedure TThread.Resume;
begin
  if ResumeThread(FHandle) = 1 then FSuspended := False;
end;

procedure TThread.Terminate;
begin
  FTerminated := True;
end;

function TThread.WaitFor: Integer;
var
  Msg: TMsg;
  WaitHandles : array[0..1] of THandle;
begin
  if GetCurrentThreadID = MainThreadID then
    begin
      WaitHandles[0]:=FHandle;
      WaitHandles[1]:=THandle(SynchronizeTimeoutEvent);
      while true do
        begin
          case MsgWaitForMultipleObjects(2, WaitHandles, False, INFINITE, QS_SENDMESSAGE) of
            WAIT_OBJECT_0:
              break;
            WAIT_OBJECT_0+1:
              CheckSynchronize;
            WAIT_OBJECT_0+2:
              PeekMessage(Msg, 0, 0, 0, PM_NOREMOVE)
          end;
        end;
    end
  else
    WaitForSingleObject(ulong(FHandle), INFINITE);
  GetExitCodeThread(FHandle, DWord(Result));
end;
