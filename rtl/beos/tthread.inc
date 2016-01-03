{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by Peter Vreman

    BeOS TThread implementation

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{$IFDEF VER1_0} // leaving the old implementation in for now...
type
  PThreadRec=^TThreadRec;
  TThreadRec=record
    thread : TThread;
    next   : PThreadRec;
  end;

var
  ThreadRoot : PThreadRec;
  ThreadsInited : boolean;
//  MainThreadID: longint;

Const
  ThreadCount: longint = 0;

function ThreadSelf:TThread;
var
  hp : PThreadRec;
  sp : Pointer;
begin
  sp:=SPtr;
  hp:=ThreadRoot;
  while assigned(hp) do
   begin
     if (sp<=hp^.Thread.FStackPointer) and
        (sp>(hp^.Thread.FStackPointer-hp^.Thread.FStackSize)) then
      begin
        Result:=hp^.Thread;
        exit;
      end;
     hp:=hp^.next;
   end;
  Result:=nil;
end;


//function SIGCHLDHandler(Sig: longint): longint; cdecl;//this is std linux C declaration as function
procedure SIGCHLDHandler(Sig: longint); cdecl;

begin
  fpwaitpid(-1, nil, WNOHANG);
end;

procedure InitThreads;
var
  Act, OldAct: Baseunix.PSigActionRec;
begin
  ThreadRoot:=nil;
  ThreadsInited:=true;


// This will install SIGCHLD signal handler
// signal() installs "one-shot" handler,
// so it is better to install and set up handler with sigaction()

  GetMem(Act, SizeOf(SigActionRec));
  GetMem(OldAct, SizeOf(SigActionRec));

  Act^.sa_handler := TSigAction(@SIGCHLDHandler);
  Act^.sa_flags := SA_NOCLDSTOP {or SA_NOMASK or SA_RESTART};
  Fillchar(Act^.sa_mask,sizeof(Act^.sa_mask),0); //Do not block all signals ??. Don't need if SA_NOMASK in flags
  FpSigAction(SIGCHLD, Act, OldAct);

  FreeMem(Act, SizeOf(SigActionRec));
  FreeMem(OldAct, SizeOf(SigActionRec));
end;


procedure DoneThreads;
var
  hp : PThreadRec;
begin
  while assigned(ThreadRoot) do
   begin
     ThreadRoot^.Thread.Destroy;
     hp:=ThreadRoot;
     ThreadRoot:=ThreadRoot^.Next;
     dispose(hp);
   end;
  ThreadsInited:=false;
end;


procedure AddThread(t:TThread);
var
  hp : PThreadRec;
begin
  { Need to initialize threads ? }
  if not ThreadsInited then
   InitThreads;

  { Put thread in the linked list }
  new(hp);
  hp^.Thread:=t;
  hp^.next:=ThreadRoot;
  ThreadRoot:=hp;

  inc(ThreadCount, 1);
end;


procedure RemoveThread(t:TThread);
var
  lasthp,hp : PThreadRec;
begin
  hp:=ThreadRoot;
  lasthp:=nil;
  while assigned(hp) do
   begin
     if hp^.Thread=t then
      begin
        if assigned(lasthp) then
         lasthp^.next:=hp^.next
        else
         ThreadRoot:=hp^.next;
        dispose(hp);
        exit;
      end;
     lasthp:=hp;
     hp:=hp^.next;
   end;

  Dec(ThreadCount, 1);
  if ThreadCount = 0 then DoneThreads;
end;


{ TThread }
function ThreadProc(args:pointer): Integer;//cdecl;
var
  FreeThread: Boolean;
  Thread : TThread absolute args;
begin
  while Thread.FHandle = 0 do fpsleep(1);
  if Thread.FSuspended then Thread.suspend();
  try
<<<<<<< HEAD
<<<<<<< HEAD
    CurrentThreadVar := Thread;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Thread.Execute;
  except
    Thread.FFatalException := TObject(AcquireExceptionObject);
  end;
  FreeThread := Thread.FFreeOnTerminate;
  Result := Thread.FReturnValue;
  Thread.FFinished := True;
  Thread.DoTerminate;
  if FreeThread then
    Thread.Free;
  fpexit(Result);
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TThread.SysCreate(CreateSuspended: Boolean;
                            const StackSize: SizeUInt);
var
  Flags: Integer;
begin
=======
=======
>>>>>>> origin/fixes_2_2
constructor TThread.Create(CreateSuspended: Boolean);
var
  Flags: Integer;
begin
  inherited Create;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  AddThread(self);
  FSuspended := CreateSuspended;
  Flags := CLONE_VM + CLONE_FS + CLONE_FILES + CLONE_SIGHAND + SIGCHLD;
  { Setup 16k of stack }
  FStackSize:=16384;
<<<<<<< HEAD
<<<<<<< HEAD
  Getmem(FStackPointer,StackSize);
  inc(FStackPointer,StackSize);
=======
  Getmem(FStackPointer,FStackSize);
  inc(FStackPointer,FStackSize);
>>>>>>> graemeg/fixes_2_2
=======
  Getmem(FStackPointer,FStackSize);
  inc(FStackPointer,FStackSize);
>>>>>>> origin/fixes_2_2
  FCallExitProcess:=false;
  { Clone }
  FHandle:= Clone(@ThreadProc,FStackPointer,Flags,self);
//  if FSuspended then Suspend;
  FThreadID := FHandle;
  IsMultiThread := TRUE;
  FFatalException := nil;
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TThread.SysDestroy;
=======
destructor TThread.Destroy;
>>>>>>> graemeg/fixes_2_2
=======
destructor TThread.Destroy;
>>>>>>> origin/fixes_2_2
begin
  if not FFinished and not Suspended then
   begin
     Terminate;
     WaitFor;
   end;
  if FHandle <> -1 then
    fpkill(FHandle, SIGKILL);
  dec(FStackPointer,FStackSize);
  Freemem(FStackPointer);
  FFatalException.Free;
  FFatalException := nil;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  inherited Destroy;
>>>>>>> graemeg/fixes_2_2
=======
  inherited Destroy;
>>>>>>> origin/fixes_2_2
  RemoveThread(self);
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
{ I Don't know idle or timecritical, value is also 20, so the largest other
  possibility is 19 (PFV) }
  Priorities: array [TThreadPriority] of Integer =
   (-20,-19,-10,9,10,19,20);

function TThread.GetPriority: TThreadPriority;
var
  P: Integer;
  I: TThreadPriority;
begin
  P := fpGetPriority(Prio_Process,FHandle);
  Result := tpNormal;
  for I := Low(TThreadPriority) to High(TThreadPriority) do
    if Priorities[I] = P then
      Result := I;
end;


procedure TThread.SetPriority(Value: TThreadPriority);
begin
  fpSetPriority(Prio_Process,FHandle,Priorities[Value]);
end;


procedure TThread.Synchronize(Method: TThreadMethod);
begin
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  FSynchronizeException := nil;
  FMethod := Method;
{  SendMessage(ThreadWindow, CM_EXECPROC, 0, Longint(Self)); }
  if Assigned(FSynchronizeException) then
    raise FSynchronizeException;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
  FSuspended := true;
  fpKill(FHandle, SIGSTOP);
end;


procedure TThread.Resume;
begin
  fpKill(FHandle, SIGCONT);
  FSuspended := False;
end;


procedure TThread.Terminate;
begin
  FTerminated := True;
end;

function TThread.WaitFor: Integer;
var
  status : longint;
begin
  if FThreadID = MainThreadID then
    fpwaitpid(0,@status,0)
  else
    fpwaitpid(FHandle,@status,0);
  Result:=status;
end;
{$ELSE}

{
  What follows, is a short description on my implementation of TThread.
  Most information can also be found by reading the source and accompanying
  comments.
  
  A thread is created using BeginThread, which in turn calls
  pthread_create. So the threads here are always posix threads.
  Posix doesn't define anything for suspending threads as this is
  inherintly unsafe. Just don't suspend threads at points they cannot
  control. Therefore, I didn't implement .Suspend() if its called from
  outside the threads execution flow (except on Linux _without_ NPTL).
  
  The implementation for .suspend uses a semaphore, which is initialized
  at thread creation. If the thread tries to suspend itself, we simply
  let it wait on the semaphore until it is unblocked by someone else
  who calls .Resume.

  If a thread is supposed to be suspended (from outside its own path of
  execution) on a system where the symbol LINUX is defined, two things
  are possible.
  1) the system has the LinuxThreads pthread implementation
  2) the system has NPTL as the pthread implementation.
  
  In the first case, each thread is a process on its own, which as far as
  know actually violates posix with respect to signal handling.
  But we can detect this case, because getpid(2) will
  return a different PID for each thread. In that case, sending SIGSTOP
  to the PID associated with a thread will actually stop that thread
  only.
  In the second case, this is not possible. But getpid(2) returns the same
  PID across all threads, which is detected, and TThread.Suspend() does
  nothing in that case. This should probably be changed, but I know of
  no way to suspend a thread when using NPTL.
  
  If the symbol LINUX is not defined, then the unimplemented
  function SuspendThread is called.
  
  Johannes Berg <johannes@sipsolutions.de>, Sunday, November 16 2003
}

// ========== semaphore stuff ==========
{
  I don't like this. It eats up 2 filedescriptors for each thread,
  and those are a limited resource. If you have a server programm
  handling client connections (one per thread) it will not be able
  to handle many if we use 2 fds already for internal structures.
  However, right now I don't see a better option unless some sem_*
  functions are added to systhrds.
  I encapsulated all used functions here to make it easier to
  change them completely.
}

{BeOS implementation}

function SemaphoreInit: Pointer;
begin
  SemaphoreInit := GetMem(SizeOf(TFilDes));
  fppipe(PFilDes(SemaphoreInit)^);
end;

procedure SemaphoreWait(const FSem: Pointer);
var
  b: byte;
begin
  fpread(PFilDes(FSem)^[0], b, 1);
end;

procedure SemaphorePost(const FSem: Pointer);
var
  b : byte;
begin
  b := 0;
  fpwrite(PFilDes(FSem)^[1], b, 1);
end;

procedure SemaphoreDestroy(const FSem: Pointer);
begin
  fpclose(PFilDes(FSem)^[0]);
  fpclose(PFilDes(FSem)^[1]);
  FreeMemory(FSem);
end;

// =========== semaphore end ===========

var
  ThreadsInited: boolean = false;
{$IFDEF LINUX}
  GMainPID: LongInt = 0;
{$ENDIF}
const
  // stupid, considering its not even implemented...
  Priorities: array [TThreadPriority] of Integer =
   (-20,-19,-10,0,9,18,19);

procedure InitThreads;
begin
  if not ThreadsInited then begin
    ThreadsInited := true;
    {$IFDEF LINUX}
    GMainPid := fpgetpid();
    {$ENDIF}
  end;
end;

procedure DoneThreads;
begin
  ThreadsInited := false;
end;

{ ok, so this is a hack, but it works nicely. Just never use
  a multiline argument with WRITE_DEBUG! }
{$MACRO ON}
{$IFDEF DEBUG_MT}
{$define WRITE_DEBUG := writeln} // actually write something
{$ELSE}
{$define WRITE_DEBUG := //}      // just comment out those lines
{$ENDIF}

function ThreadFunc(parameter: Pointer): LongInt; // cdecl;
var
  LThread: TThread;
  c: char;
begin
  WRITE_DEBUG('ThreadFunc is here...');
  LThread := TThread(parameter);
  {$IFDEF LINUX}
  // save the PID of the "thread"
  // this is different from the PID of the main thread if
  // the LinuxThreads implementation is used
  LThread.FPid := fpgetpid();
  {$ENDIF}
  WRITE_DEBUG('thread initing, parameter = ', LongInt(LThread));
  try
    if LThread.FInitialSuspended then begin
      SemaphoreWait(LThread.FSem);
      if not LThread.FInitialSuspended then begin
<<<<<<< HEAD
<<<<<<< HEAD
        CurrentThreadVar := LThread;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        WRITE_DEBUG('going into LThread.Execute');
        LThread.Execute;
      end;
    end else begin
<<<<<<< HEAD
<<<<<<< HEAD
      CurrentThreadVar := LThread;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      WRITE_DEBUG('going into LThread.Execute');
      LThread.Execute;
    end;
  except
    on e: exception do begin
      WRITE_DEBUG('got exception: ',e.message);
      LThread.FFatalException :=  TObject(AcquireExceptionObject);
      // not sure if we should really do this...
      // but .Destroy was called, so why not try FreeOnTerminate?
      if e is EThreadDestroyCalled then LThread.FFreeOnTerminate := true;
    end;
  end;
  WRITE_DEBUG('thread done running');
  Result := LThread.FReturnValue;
  WRITE_DEBUG('Result is ',Result);
  LThread.FFinished := True;
  LThread.DoTerminate;
  if LThread.FreeOnTerminate then begin
    WRITE_DEBUG('Thread should be freed');
    LThread.Free;
    WRITE_DEBUG('Thread freed');
  end;
  WRITE_DEBUG('thread func exiting');
end;

{ TThread }
<<<<<<< HEAD
<<<<<<< HEAD
procedure TThread.SysCreate(CreateSuspended: Boolean; const StackSize: SizeUInt);
=======
constructor TThread.Create(CreateSuspended: Boolean; const StackSize: SizeUInt = DefaultStackSize);
>>>>>>> graemeg/fixes_2_2
=======
constructor TThread.Create(CreateSuspended: Boolean; const StackSize: SizeUInt = DefaultStackSize);
>>>>>>> origin/fixes_2_2
var
  data : pointer;
begin
  // lets just hope that the user doesn't create a thread
  // via BeginThread and creates the first TThread Object in there!
  InitThreads;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  inherited Create;
>>>>>>> graemeg/fixes_2_2
=======
  inherited Create;
>>>>>>> origin/fixes_2_2
  FSem := SemaphoreInit;
  FSuspended := CreateSuspended;
  FSuspendedExternal := false;
  FInitialSuspended := CreateSuspended;
  FFatalException := nil;
  WRITE_DEBUG('creating thread, self = ',longint(self));
  FHandle:= BeginThread(@ThreadFunc, Pointer(Self), FThreadID);
  WRITE_DEBUG('TThread.Create done');
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TThread.SysDestroy;
=======
destructor TThread.Destroy;
>>>>>>> graemeg/fixes_2_2
=======
destructor TThread.Destroy;
>>>>>>> origin/fixes_2_2
begin
  if FThreadID = GetCurrentThreadID then begin
    raise EThreadDestroyCalled.Create('A thread cannot destroy itself except by setting FreeOnTerminate and leaving!');
  end;
  // if someone calls .Free on a thread with
  // FreeOnTerminate, then don't crash!
  FFreeOnTerminate := false;
  if not FFinished and not FSuspended then begin
    Terminate;
    WaitFor;
  end;
  if (FInitialSuspended) then begin
    // thread was created suspended but never woken up.
    SemaphorePost(FSem);
    WaitFor;
  end;
  FFatalException.Free;
  FFatalException := nil;
  SemaphoreDestroy(FSem);
<<<<<<< HEAD
<<<<<<< HEAD
=======
  inherited Destroy;
>>>>>>> graemeg/fixes_2_2
=======
  inherited Destroy;
>>>>>>> origin/fixes_2_2
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
  if not FSuspended then begin
    if FThreadID = GetCurrentThreadID then begin
      FSuspended := true;
      SemaphoreWait(FSem);
    end else begin
      FSuspendedExternal := true;
{$IFDEF LINUX}
      // naughty hack if the user doesn't have Linux with NPTL...
      // in that case, the PID of threads will not be identical
      // to the other threads, which means that our thread is a normal
      // process that we can suspend via SIGSTOP...
      // this violates POSIX, but is the way it works on the
      // LinuxThreads pthread implementation. Not with NPTL, but in that case
      // getpid(2) also behaves properly and returns the same PID for
      // all threads. Thats actually (FINALLY!) native thread support :-)
      if FPid <> GMainPID then begin
        FSuspended := true;
        fpkill(FPid, SIGSTOP);
      end;
{$ELSE}
      SuspendThread(FHandle);
{$ENDIF}
    end;
  end;
end;


procedure TThread.Resume;
begin
  if (not FSuspendedExternal) then begin
    if FSuspended then begin
      SemaphorePost(FSem);
      FInitialSuspended := false;
      FSuspended := False;
    end;
  end else begin
{$IFDEF LINUX}
    // see .Suspend
    if FPid <> GMainPID then begin
      fpkill(FPid, SIGCONT);
      FSuspended := False;
    end;
{$ELSE}
    ResumeThread(FHandle);
{$ENDIF}
    FSuspendedExternal := false;
  end;
end;


procedure TThread.Terminate;
begin
  FTerminated := True;
end;

function TThread.WaitFor: Integer;
begin
  WRITE_DEBUG('waiting for thread ',FHandle);
  WaitFor := WaitForThreadTerminate(FHandle, 0);
  WRITE_DEBUG('thread terminated');
end;

procedure TThread.CallOnTerminate;
begin
  // no need to check if FOnTerminate <> nil, because
  // thats already done in DoTerminate
  FOnTerminate(self);
end;

procedure TThread.DoTerminate;
begin
  if Assigned(FOnTerminate) then
    Synchronize(@CallOnTerminate);
end;

function TThread.GetPriority: TThreadPriority;
var
  P: Integer;
  I: TThreadPriority;
begin
  P := ThreadGetPriority(FHandle);
  Result := tpNormal;
  for I := Low(TThreadPriority) to High(TThreadPriority) do
    if Priorities[I] = P then
      Result := I;
end;

(*
procedure TThread.Synchronize(Method: TThreadMethod);
begin
{$TODO someone with more clue of the GUI stuff will have to do this}
end;
*)
procedure TThread.SetPriority(Value: TThreadPriority);
begin
  ThreadSetPriority(FHandle, Priorities[Value]);
end;
{$ENDIF}

