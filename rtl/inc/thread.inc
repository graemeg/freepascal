{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 2000 by the Free Pascal development team

    OS independent thread functions/overloads

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


Var
  CurrentTM : TThreadManager;
{$ifndef THREADVAR_RELOCATED_ALREADY_DEFINED}
  fpc_threadvar_relocate_proc : TRelocateThreadVarHandler; public name 'FPC_THREADVAR_RELOCATE';
{$endif THREADVAR_RELOCATED_ALREADY_DEFINED}

{$ifndef HAS_GETCPUCOUNT}
    function GetCPUCount: LongWord;
      begin
        Result := 1;
      end;
{$endif}


{*****************************************************************************
                           Threadvar initialization
*****************************************************************************}

    procedure InitThread(stklen:SizeUInt);
      begin
{$ifndef FPUNONE}
        SysResetFPU;
        SysInitFPU;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
{$ifndef HAS_MEMORYMANAGER}
        { initialize this thread's heap }
        InitHeapThread;
{$endif HAS_MEMORYMANAGER}
        if MemoryManager.InitThread <> nil then
          MemoryManager.InitThread();
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
        if assigned(widestringmanager.ThreadInitProc) then
          widestringmanager.ThreadInitProc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        { ExceptAddrStack and ExceptObjectStack are threadvars       }
        { so every thread has its on exception handling capabilities }
        SysInitExceptions;
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
{$ifndef EMBEDDED}
        { Open all stdio fds again }
        SysInitStdio;
        InOutRes:=0;
        // ErrNo:=0;
{$endif EMBEDDED}
{$endif FPC_HAS_FEATURE_CONSOLEIO}
{$ifdef FPC_HAS_FEATURE_STACKCHECK}
        { Stack checking }
        StackLength:= CheckInitialStkLen(stkLen);
        StackBottom:=Sptr - StackLength;
{$endif FPC_HAS_FEATURE_STACKCHECK}
        ThreadID := CurrentTM.GetCurrentThreadID();
      end;

    procedure DoneThread;
      begin
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
        if assigned(widestringmanager.ThreadFiniProc) then
          widestringmanager.ThreadFiniProc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef HAS_MEMORYMANAGER}
        FinalizeHeap;
{$endif HAS_MEMORYMANAGER}
        if MemoryManager.DoneThread <> nil then
          MemoryManager.DoneThread();
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
        { Open all stdio fds again }
        SysFlushStdio;
{$endif FPC_HAS_FEATURE_CONSOLEIO}
        { Support platforms where threadvar memory is managed outside of the RTL:
          reset ThreadID and allow ReleaseThreadVars to be unassigned }
        ThreadID := TThreadID(0);
        if assigned(CurrentTM.ReleaseThreadVars) then
          CurrentTM.ReleaseThreadVars;
=======
        CurrentTM.ReleaseThreadVars;
>>>>>>> graemeg/fixes_2_2
=======
        CurrentTM.ReleaseThreadVars;
>>>>>>> origin/fixes_2_2
      end;

{*****************************************************************************
                            Overloaded functions
*****************************************************************************}

    function BeginThread(ThreadFunction : tthreadfunc) : TThreadID;
      var
        dummy : TThreadID;
      begin
        BeginThread:=BeginThread(nil,DefaultStackSize,ThreadFunction,nil,0,dummy);
      end;


    function BeginThread(ThreadFunction : tthreadfunc;p : pointer) : TThreadID;
      var
        dummy : TThreadID;
      begin
        BeginThread:=BeginThread(nil,DefaultStackSize,ThreadFunction,p,0,dummy);
      end;


    function BeginThread(ThreadFunction : tthreadfunc;p : pointer;var ThreadId : TThreadID) : TThreadID;
      begin
        BeginThread:=BeginThread(nil,DefaultStackSize,ThreadFunction,p,0,ThreadId);
      end;

    function BeginThread(ThreadFunction : tthreadfunc;p : pointer;
                     var ThreadId : TThreadID; const stacksize: SizeUInt) : TThreadID;
      begin
        BeginThread:=BeginThread(nil,stacksize,ThreadFunction,p,0,ThreadId);
      end;

    procedure EndThread;
      begin
        EndThread(0);
      end;

function BeginThread(sa : Pointer;stacksize : SizeUInt; ThreadFunction : tthreadfunc;p : pointer;creationFlags : dword;  var ThreadId : TThreadID) : TThreadID;

begin
  Result:=CurrentTM.BeginThread(sa,stacksize,threadfunction,P,creationflags,ThreadID);
end;

procedure FlushThread;

begin
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
  SysFlushStdio;
{$endif FPC_HAS_FEATURE_CONSOLEIO}
end;

procedure EndThread(ExitCode : DWord);

begin
  CurrentTM.EndThread(ExitCode);
end;

function  SuspendThread (threadHandle : TThreadID) : dword;

begin
  Result:=CurrentTM.SuspendThread(ThreadHandle);
end;

function ResumeThread  (threadHandle : TThreadID) : dword;

begin
  Result:=CurrentTM.ResumeThread(ThreadHandle);
end;

function CloseThread  (threadHandle : TThreadID):dword;

begin
  result:=CurrentTM.CloseThread(ThreadHandle);
end;

procedure ThreadSwitch;

begin
  CurrentTM.ThreadSwitch;
end;

function  KillThread (threadHandle : TThreadID) : dword;

begin
  Result:=CurrentTM.KillThread(ThreadHandle);
end;

function  WaitForThreadTerminate (threadHandle : TThreadID; TimeoutMs : longint) : dword;

begin
  Result:=CurrentTM.WaitForThreadTerminate(ThreadHandle,TimeOutMS);
end;

function  ThreadSetPriority (threadHandle : TThreadID; Prio: longint): boolean;
begin
  Result:=CurrentTM.ThreadSetPriority(ThreadHandle,Prio);
end;

function  ThreadGetPriority (threadHandle : TThreadID): longint;

begin
  Result:=CurrentTM.ThreadGetPriority(ThreadHandle);
end;

function  GetCurrentThreadId : TThreadID;

begin
  Result:=CurrentTM.GetCurrentThreadID();
end;

procedure InitCriticalSection(var cs : TRTLCriticalSection);

begin
  CurrentTM.InitCriticalSection(cs);
end;

procedure DoneCriticalsection(var cs : TRTLCriticalSection);

begin
  CurrentTM.DoneCriticalSection(cs);
end;

procedure EnterCriticalsection(var cs : TRTLCriticalSection);

begin
  CurrentTM.EnterCriticalSection(cs);
end;

function TryEnterCriticalsection(var cs : TRTLCriticalSection):longint;

begin
  result:=CurrentTM.TryEnterCriticalSection(cs);
end;

procedure LeaveCriticalsection(var cs : TRTLCriticalSection);

begin
  CurrentTM.LeaveCriticalSection(cs);
end;

Function GetThreadManager(Var TM : TThreadManager) : Boolean;

begin
  TM:=CurrentTM;
  Result:=True;
end;

Function SetThreadManager(Const NewTM : TThreadManager; Var OldTM : TThreadManager) : Boolean;

begin
  GetThreadManager(OldTM);
  Result:=SetThreadManager(NewTM);
end;

Function SetThreadManager(Const NewTM : TThreadManager) : Boolean;

begin
  Result:=True;
  If Assigned(CurrentTM.DoneManager) then
    Result:=CurrentTM.DoneManager();
  If Result then
    begin
    CurrentTM:=NewTM;
    If Assigned(CurrentTM.InitManager) then
      Result:=CurrentTM.InitManager();
    end;
end;

function  BasicEventCreate(EventAttributes : Pointer; AManualReset,InitialState : Boolean;const Name : ansistring):pEventState;

begin
  result:=currenttm.BasicEventCreate(EventAttributes,AManualReset,InitialState, Name);
end;

procedure basiceventdestroy(state:peventstate);

begin
  currenttm.basiceventdestroy(state);
end;

procedure basiceventResetEvent(state:peventstate);

begin
  currenttm.basiceventResetEvent(state);
end;

procedure basiceventSetEvent(state:peventstate);

begin
  currenttm.basiceventSetEvent(state);
end;

function  basiceventWaitFor(Timeout : Cardinal;state:peventstate) : longint;

begin
 result:=currenttm.basiceventWaitFor(Timeout,state);
end;

function  RTLEventCreate :PRTLEvent;

begin
  result:=currenttm.rtleventcreate();
end;


procedure RTLeventdestroy(state:pRTLEvent);

begin
  currenttm.rtleventdestroy(state);
end;

procedure RTLeventSetEvent(state:pRTLEvent);

begin
  currenttm.rtleventsetEvent(state);
end;

procedure RTLeventResetEvent(state:pRTLEvent);

begin
  currenttm.rtleventResetEvent(state);
end;

procedure RTLeventWaitFor(state:pRTLEvent);

begin
  currenttm.rtleventWaitFor(state);
end;

procedure RTLeventWaitFor(state:pRTLEvent;timeout : longint);

begin
  currenttm.rtleventWaitForTimeout(state,timeout);
end;

function SemaphoreInit: pointer;
begin
  result:=currenttm.SemaphoreInit();
end;

procedure SemaphoreDestroy(const sem: pointer);
begin
  currenttm.SemaphoreDestroy(sem);
end;

procedure SemaphoreWait(const sem: pointer);
begin
  currenttm.SemaphoreWait(sem);
end;

procedure SemaphorePost(const sem: pointer);
begin
  currenttm.SemaphorePost(sem);
end;

{ ---------------------------------------------------------------------
    ThreadManager which gives run-time error. Use if no thread support.
  ---------------------------------------------------------------------}

{$ifndef DISABLE_NO_THREAD_MANAGER}

{ resourcestrings are not supported by the system unit,
  they are in the objpas unit and not available for fpc/tp modes }
const
  SNoThreads = 'This binary has no thread support compiled in.';
  SRecompileWithThreads = 'Recompile the application with a thread-driver in the program uses clause before other units using thread.';

Procedure NoThreadError;

begin
{$ifndef EMBEDDED}
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
  If IsConsole then
    begin
    Writeln(StdErr,SNoThreads);
    Writeln(StdErr,SRecompileWithThreads);
    end;
{$endif FPC_HAS_FEATURE_CONSOLEIO}
{$endif EMBEDDED}
  RunError(232)
end;

function NoBeginThread(sa : Pointer;stacksize : PtrUInt;
                     ThreadFunction : tthreadfunc;p : pointer;
                     creationFlags : dword; var ThreadId : TThreadID) : TThreadID;
begin
  NoThreadError;
  result:=tthreadid(-1);
end;

procedure NoEndThread(ExitCode : DWord);
begin
  NoThreadError;
end;

function  NoThreadHandler (threadHandle : TThreadID) : dword;
begin
  NoThreadError;
  result:=dword(-1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure NoThreadSwitch;  {give time to other threads}
begin
  NoThreadError;
>>>>>>> graemeg/cpstrnew
end;

function NoCloseThread  (threadHandle : TThreadID):dword;

begin
 Result:=0;      // avoid warnings.
 NoThreadError;
end;

function NoCloseThread  (threadHandle : TThreadID):dword;

begin
 Result:=0;      // avoid warnings.
 NoThreadError;
end;

function NoCloseThread  (threadHandle : TThreadID):dword;

begin
 Result:=0;      // avoid warnings.
 NoThreadError;
end;

function NoCloseThread  (threadHandle : TThreadID):dword;

begin
 Result:=0;      // avoid warnings.
 NoThreadError;
end;

function  NoWaitForThreadTerminate (threadHandle : TThreadID; TimeoutMs : longint) : dword;  {0=no timeout}
begin
  NoThreadError;
  result:=dword(-1);
end;

function  NoThreadSetPriority (threadHandle : TThreadID; Prio: longint): boolean; {-15..+15, 0=normal}
begin
  NoThreadError;
  result:=false;
end;

function  NoThreadGetPriority (threadHandle : TThreadID): longint;
begin
  NoThreadError;
  result:=-1;
end;

function  NoGetCurrentThreadId : TThreadID;
begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
  result:=TThreadID(1);
end;

procedure NoCriticalSection(var CS);

begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
end;

function NoTryEnterCriticalSection(var CS):longint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
  Result:=-1;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
end;

procedure NoInitThreadvar(var offset : dword;size : dword);

begin
  NoThreadError;
end;

function NoRelocateThreadvar(offset : dword) : pointer;

begin
  NoThreadError;
  result:=nil;
end;


procedure NoAllocateThreadVars;

begin
  NoThreadError;
>>>>>>> graemeg/cpstrnew
end;

procedure NoInitThreadvar(var offset : {$ifdef cpu16}word{$else}dword{$endif};size : dword);

begin
  NoThreadError;
end;

function NoRelocateThreadvar(offset : {$ifdef cpu16}word{$else}dword{$endif}) : pointer;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  NoThreadError;
  result:=nil;
end;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
  result:=nil;
end;

procedure nobasiceventdestroy(state:peventstate);

begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
end;
>>>>>>> graemeg/cpstrnew

function  NoBasicEventCreate(EventAttributes : Pointer; AManualReset,InitialState : Boolean;const Name : ansistring):pEventState;

begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  result:=nil;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure NoBasicEvent(state:peventstate);

begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
end;

function  NoBasicEventWaitFor(Timeout : Cardinal;state:peventstate) : longint;

begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
  result:=-1;
end;

function  NoRTLEventCreate :PRTLEvent;

begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
  result:=nil;
end;

procedure NoRTLEvent(state:pRTLEvent);

begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true
end;

procedure NoRTLEventWaitForTimeout(state:pRTLEvent;timeout : longint);
begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
<<<<<<< HEAD
end;

=======
end;

procedure NORTLeventResetEvent(state:pRTLEvent);

begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure NORTLeventResetEvent(state:pRTLEvent);

begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure NORTLeventWaitFor(state:pRTLEvent);
  begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
  end;


procedure NORTLeventWaitForTimeout(state:pRTLEvent;timeout : longint);
  begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
  end;


procedure NORTLeventsync(m:trtlmethod;p:tprocedure);
  begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
  end;
>>>>>>> graemeg/cpstrnew

function NoSemaphoreInit: Pointer;
begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
  result:=nil;
end;

procedure NoSemaphoreWait(const FSem: Pointer);
begin
  NoThreadError;
end;

procedure NoSemaphore(const FSem: Pointer);
begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure NoSemaphoreDestroy(const FSem: Pointer);
begin
  if IsMultiThread then
    NoThreadError
  else
    ThreadingAlreadyUsed:=true;
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
end;

const
  NoThreadManager : TThreadManager = (
         InitManager            : Nil;
         DoneManager            : Nil;
{$ifdef EMBEDDED}
         { while this is pretty hacky, it reduces the size of typical embedded programs
           and works fine on arm and avr }
         BeginThread            : @NoBeginThread;
         EndThread              : TEndThreadHandler(@NoThreadError);
         SuspendThread          : TThreadHandler(@NoThreadError);
         ResumeThread           : TThreadHandler(@NoThreadError);
         KillThread             : TThreadHandler(@NoThreadError);
         CloseThread            : TThreadHandler(@NoThreadError);
         ThreadSwitch           : TThreadSwitchHandler(@NoThreadError);
         WaitForThreadTerminate : TWaitForThreadTerminateHandler(@NoThreadError);
         ThreadSetPriority      : TThreadSetPriorityHandler(@NoThreadError);
         ThreadGetPriority      : TThreadGetPriorityHandler(@NoThreadError);
         GetCurrentThreadId     : @NoGetCurrentThreadId;
         InitCriticalSection    : TCriticalSectionHandler(@NoThreadError);
         DoneCriticalSection    : TCriticalSectionHandler(@NoThreadError);
         EnterCriticalSection   : TCriticalSectionHandler(@NoThreadError);
         TryEnterCriticalSection: TCriticalSectionHandlerTryEnter(@NoThreadError);
         LeaveCriticalSection   : TCriticalSectionHandler(@NoThreadError);
         InitThreadVar          : TInitThreadVarHandler(@NoThreadError);
         RelocateThreadVar      : TRelocateThreadVarHandler(@NoThreadError);
         AllocateThreadVars     : @NoThreadError;
         ReleaseThreadVars      : @NoThreadError;
         BasicEventCreate       : TBasicEventCreateHandler(@NoThreadError);
         basiceventdestroy      : TBasicEventHandler(@NoThreadError);
         basiceventResetEvent   : TBasicEventHandler(@NoThreadError);
         basiceventSetEvent     : TBasicEventHandler(@NoThreadError);
         basiceventWaitFor      : TBasicEventWaitForHandler(@NoThreadError);
         rtlEventCreate         : TRTLCreateEventHandler(@NoThreadError);
         rtleventdestroy        : TRTLEventHandler(@NoThreadError);
         rtleventSetEvent       : TRTLEventHandler(@NoThreadError);
         rtleventResetEvent     : TRTLEventHandler(@NoThreadError);
         rtleventWaitFor        : TRTLEventHandler(@NoThreadError);
         rtleventwaitfortimeout : TRTLEventHandlerTimeout(@NoThreadError);
         SemaphoreInit          : TSempahoreInitHandler(@NoThreadError);
         SemaphoreDestroy       : TSemaphoreDestroyHandler(@NoThreadError);
         SemaphorePost          : TSemaphorePostHandler(@NoThreadError);
         SemaphoreWait          : TSemaphoreWaitHandler(@NoThreadError)
{$else EMBEDDED}
         BeginThread            : @NoBeginThread;
         EndThread              : @NoEndThread;
         SuspendThread          : @NoThreadHandler;
         ResumeThread           : @NoThreadHandler;
         KillThread             : @NoThreadHandler;
         CloseThread            : @NoThreadHandler;
         ThreadSwitch           : @NoThreadError;
         WaitForThreadTerminate : @NoWaitForThreadTerminate;
         ThreadSetPriority      : @NoThreadSetPriority;
         ThreadGetPriority      : @NoThreadGetPriority;
         GetCurrentThreadId     : @NoGetCurrentThreadId;
         InitCriticalSection    : @NoCriticalSection;
         DoneCriticalSection    : @NoCriticalSection;
         EnterCriticalSection   : @NoCriticalSection;
         TryEnterCriticalSection: @NoTryEnterCriticalSection;
         LeaveCriticalSection   : @NoCriticalSection;
         InitThreadVar          : @NoInitThreadVar;
         RelocateThreadVar      : @NoRelocateThreadVar;
         AllocateThreadVars     : @NoThreadError;
         ReleaseThreadVars      : @NoThreadError;
         BasicEventCreate       : @NoBasicEventCreate;
         basiceventdestroy      : @NoBasicEvent;
         basiceventResetEvent   : @NoBasicEvent;
         basiceventSetEvent     : @NoBasicEvent;
         basiceventWaitFor      : @NoBasiceventWaitFor;
         rtlEventCreate         : @NoRTLEventCreate;
         rtleventdestroy        : @NoRTLevent;
         rtleventSetEvent       : @NoRTLevent;
         rtleventResetEvent     : @NoRTLEvent;
         rtleventWaitFor        : @NoRTLEvent;
         rtleventwaitfortimeout : @NoRTLEventWaitForTimeout;
         SemaphoreInit          : @NoSemaphoreInit;
         SemaphoreDestroy       : @NoSemaphore;
         SemaphorePost          : @NoSemaphore;
         SemaphoreWait          : @NoSemaphoreWait
{$endif EMBEDDED}
      );

Procedure SetNoThreadManager;

begin
<<<<<<< HEAD
=======
  With NoThreadManager do
    begin
    InitManager            :=Nil;
    DoneManager            :=Nil;
    BeginThread            :=@NoBeginThread;
    EndThread              :=@NoEndThread;
    SuspendThread          :=@NoThreadHandler;
    ResumeThread           :=@NoThreadHandler;
    KillThread             :=@NoThreadHandler;
    CloseThread		   :=@NoCloseThread;
    ThreadSwitch           :=@NoThreadSwitch;
    WaitForThreadTerminate :=@NoWaitForThreadTerminate;
    ThreadSetPriority      :=@NoThreadSetPriority;
    ThreadGetPriority      :=@NoThreadGetPriority;
    GetCurrentThreadId     :=@NoGetCurrentThreadId;
    InitCriticalSection    :=@NoCriticalSection;
    DoneCriticalSection    :=@NoCriticalSection;
    EnterCriticalSection   :=@NoCriticalSection;
    TryEnterCriticalSection:=@NoTryEnterCriticalSection;
    LeaveCriticalSection   :=@NoCriticalSection;
    InitThreadVar          :=@NoInitThreadVar;
    RelocateThreadVar      :=@NoRelocateThreadVar;
    AllocateThreadVars     :=@NoAllocateThreadVars;
    ReleaseThreadVars      :=@NoReleaseThreadVars;
    BasicEventCreate       :=@NoBasicEventCreate;
    basiceventdestroy      :=@Nobasiceventdestroy;
    basiceventResetEvent   :=@NobasiceventResetEvent;
    basiceventSetEvent     :=@NobasiceventSetEvent;
    basiceventWaitFor      :=@NobasiceventWaitFor;
    rtlEventCreate         :=@NortlEventCreate;
    rtleventdestroy        :=@Nortleventdestroy;
    rtleventSetEvent       :=@NortleventSetEvent;
    rtleventResetEvent     :=@NortleventResetEvent;
    rtleventWaitFor        :=@NortleventWaitFor;
    rtleventsync           :=@Nortleventsync;
    rtleventwaitfortimeout :=@NortleventWaitForTimeout;
    // semaphores stuff
    SemaphoreInit          :=@NoSemaphoreInit;
    SemaphoreDestroy       :=@NoSemaphoreDestroy;
    SemaphoreWait          :=@NoSemaphoreWait;
    SemaphorePost          :=@NoSemaphorePost;
    end;
>>>>>>> graemeg/cpstrnew
  SetThreadManager(NoThreadManager);
end;

Procedure InitSystemThreads; public name '_FPC_InitSystemThreads';
begin
  { This should be changed to a real value during
    thread driver initialization if appropriate. }
  ThreadID := TThreadID(1);
  SetNoThreadManager;
end;

{$endif DISABLE_NO_THREAD_MANAGER}
