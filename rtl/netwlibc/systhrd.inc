{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 by Peter Vreman,
    member of the Free Pascal development team.

    Linux (pthreads) threading support implementation

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ ok, so this is a hack, but it works nicely. Just never use
  a multiline argument with WRITE_DEBUG! }
{$MACRO ON}
{$IFDEF DEBUG_MT}
{$define WRITE_DEBUG := ConsolePrintf} // actually write something
{$ELSE}
{$define WRITE_DEBUG := //}      // just comment out those lines
{$ENDIF}

{*****************************************************************************
                             Threadvar support
*****************************************************************************}

    const
      threadvarblocksize : dword = 0;
      thredvarsmainthread: pointer = nil; // to free the threadvars in the signal handler


    var
      TLSKey : pthread_key_t;
      ThVarAllocResourceTag : rtag_t;

    procedure SysInitThreadvar(var offset : dword;size : dword);
      begin
        offset:=threadvarblocksize;
        inc(threadvarblocksize,size);
      end;

    function SysRelocateThreadvar(offset : dword) : pointer;
      begin
        SysRelocateThreadvar:=pthread_getspecific(tlskey)+Offset;
      end;


    procedure SysAllocateThreadVars;
      var
        dataindex : pointer;
      begin
        { we've to allocate the memory from system  }
        { because the FPC heap management uses      }
        { exceptions which use threadvars but       }
        { these aren't allocated yet ...            }
        { allocate room on the heap for the thread vars }
        DataIndex:=_Alloc(threadvarblocksize,ThVarAllocResourceTag);
        //DataIndex:=Pointer(Fpmmap(nil,threadvarblocksize,3,MAP_PRIVATE+MAP_ANONYMOUS,-1,0));
        FillChar(DataIndex^,threadvarblocksize,0);
        pthread_setspecific(tlskey,dataindex);
        if thredvarsmainthread = nil then
          thredvarsmainthread := dataindex;
        WRITE_DEBUG ('SysAllocateThreadVars'#13#10);
      end;


    procedure SysReleaseThreadVars;
      begin
        WRITE_DEBUG ('SysReleaseThreadVars'#13#10);
        _Free (pthread_getspecific(tlskey));
      end;

    function SysSetThreadDataAreaPtr (newPtr:pointer):pointer;
    begin
      SysSetThreadDataAreaPtr := pthread_getspecific(tlskey);  // return current
      if newPtr = nil then                                  // if nil
        newPtr := thredvarsmainthread;                      // set main thread vars
      pthread_setspecific(tlskey,newPtr);
    end;



{*****************************************************************************
                            Thread starting
*****************************************************************************}

    type
      pthreadinfo = ^tthreadinfo;
      tthreadinfo = record
        f : tthreadfunc;
        p : pointer;
        stklen : cardinal;
      end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

>>>>>>> graemeg/fixes_2_2
    function ThreadMain(param : pointer) : pointer;cdecl;
      var
        ti : tthreadinfo;
      begin
        WRITE_DEBUG('New thread started, initing threadvars'#13#10);
        { Allocate local thread vars, this must be the first thing,
          because the exception management and io depends on threadvars }
        SysAllocateThreadVars;
        { Copy parameter to local data }
        WRITE_DEBUG('New thread started, initialising ...'#13#10);
        ti:=pthreadinfo(param)^;
        dispose(pthreadinfo(param));
        { Initialize thread }
        InitThread(ti.stklen);
        { Start thread function }
        WRITE_DEBUG('Jumping to thread function'#13#10);
        ThreadMain:=pointer(ti.f(ti.p));
        WRITE_DEBUG('DoneThread, releasing threadvars'#13#10);
        DoneThread;
        //pthread_detach(pointer(pthread_self));
        pthread_exit (nil);
      end;


    function SysBeginThread(sa : Pointer;stacksize : SizeUInt;
                         ThreadFunction : tthreadfunc;p : pointer;
                         creationFlags : dword; var ThreadId : THandle) : DWord;
      var
        ti : pthreadinfo;
        thread_attr : pthread_attr_t;
      begin
        WRITE_DEBUG('SysBeginThread: Creating new thread'#13#10);
        { Initialize multithreading if not done }
        if not IsMultiThread then
         begin
          { We're still running in single thread mode, setup the TLS }
           pthread_key_create(@TLSKey,nil);
           InitThreadVars(@SysRelocateThreadvar);
           IsMultiThread:=true;
         end;
        { the only way to pass data to the newly created thread
          in a MT safe way, is to use the heap }
        getmem(ti,sizeof(pthreadinfo));
        ti^.f:=ThreadFunction;
        ti^.p:=p;
        ti^.stklen:=stacksize;
        { call pthread_create }
        WRITE_DEBUG('SysBeginThread: Starting new thread'#13#10);
        pthread_attr_init(@thread_attr);
        pthread_attr_setinheritsched(@thread_attr, PTHREAD_EXPLICIT_SCHED);

        // will fail under linux -- apparently unimplemented
        pthread_attr_setscope(@thread_attr, PTHREAD_SCOPE_PROCESS);

        // don't create detached, we need to be able to join (waitfor) on
        // the newly created thread!
        //pthread_attr_setdetachstate(@thread_attr, PTHREAD_CREATE_DETACHED);
        if pthread_create(@threadid, @thread_attr, @ThreadMain,ti) <> 0 then begin
          threadid := 0;
        end;
        SysBeginThread:=threadid;
        WRITE_DEBUG('SysBeginThread returning %d'#13#10,SysBeginThread);
      end;


    procedure SysEndThread(ExitCode : DWord);
      begin
        WRITE_DEBUG('DoneThread, releasing threadvars'#13#10);
        DoneThread;
        pthread_detach(pointer(pthread_self));
        pthread_exit(pointer(ExitCode));
      end;


    function  SysSuspendThread (threadHandle : dword) : dword;
    begin
      {$Warning SuspendThread needs to be implemented}
      SysSuspendThread := $0FFFFFFFF;
    end;

    function  SysResumeThread  (threadHandle : dword) : dword;
    begin
      {$Warning ResumeThread needs to be implemented}
      SysResumeThread := $0FFFFFFFF;
    end;

    procedure SysThreadSwitch;  {give time to other threads}
    begin
      pthread_yield;
    end;

    function  SysKillThread (threadHandle : dword) : dword;
    begin
      pthread_detach(pointer(threadHandle));
      SysKillThread := pthread_cancel(Pointer(threadHandle));
    end;

    function  SysWaitForThreadTerminate (threadHandle : dword; TimeoutMs : longint) : dword;  {0=no timeout}
    var
      LResultP: Pointer;
      LResult: DWord;
    begin
      LResult := 0;
      LResultP := @LResult;
      WRITE_DEBUG('SysWaitForThreadTerminate: waiting for %d, timeout %d'#13#10,threadHandle,timeoutMS);
      pthread_join(Pointer(threadHandle), @LResultP);
      SysWaitForThreadTerminate := LResult;
    end;

    function  SysThreadSetPriority (threadHandle : dword; Prio: longint): boolean; {-15..+15, 0=normal}
    begin
      {priority is ignored on netware}
      SysThreadSetPriority := true;
    end;


    function  SysThreadGetPriority (threadHandle : dword): longint;
    begin
      {priority is ignored on netware}
      SysThreadGetPriority := 0;
    end;

    function  SysGetCurrentThreadId : dword;
    begin
      SysGetCurrentThreadId:=dword(pthread_self);
    end;


{*****************************************************************************
                          Delphi/Win32 compatibility
*****************************************************************************}

    procedure SysInitCriticalSection(var CS);

    Var
      P : PRTLCriticalSection;

      begin
         P:=PRTLCriticalSection(@CS);
         FillChar (p^,sizeof(p^),0);
         pthread_mutex_init(P,NIL);
      end;

    procedure SysEnterCriticalSection(var CS);
      begin
         pthread_mutex_lock(PRTLCriticalSection(@CS));
      end;

    procedure SysLeaveCriticalSection(var CS);
      begin
         pthread_mutex_unlock(PRTLCriticalSection(@CS));
      end;

    procedure SysDoneCriticalSection(var CS);
      begin
         pthread_mutex_destroy(PRTLCriticalSection(@CS));
      end;


type
     Tbasiceventstate=record
         FSem: Pointer;
         FManualReset: Boolean;
         FEventSection: ppthread_mutex_t;
        end;
     plocaleventstate = ^tbasiceventstate;
//     peventstate=pointer;

Const
        wrSignaled = 0;
        wrTimeout  = 1;
        wrAbandoned= 2;
        wrError    = 3;

function IntBasicEventCreate(EventAttributes : Pointer; AManualReset,InitialState : Boolean;const Name : ansistring):pEventState;

var
  MAttr : pthread_mutex_attr_t;
  res   : cint;


begin
  //new(plocaleventstate(result));
  getmem (result,sizeof(plocaleventstate));
  plocaleventstate(result)^.FManualReset:=AManualReset;
  plocaleventstate(result)^.FSem:=New(PSemaphore);  //sem_t.
//  plocaleventstate(result)^.feventsection:=nil;
  res:=pthread_mutexattr_init(@MAttr);
  if Res=0 then
    try
      Res:=pthread_mutexattr_settype(@MAttr,longint(PTHREAD_MUTEX_RECURSIVE));
      if Res=0 then
        Res:=pthread_mutex_init(@plocaleventstate(result)^.feventsection,@MAttr);
    finally
      pthread_mutexattr_destroy(@MAttr);
    end;
  sem_init(psem_t(plocaleventstate(result)^.FSem),ord(False),Ord(InitialState));
end;

procedure Intbasiceventdestroy(state:peventstate);

begin
  sem_destroy(psem_t(  plocaleventstate(state)^.FSem));
end;

procedure IntbasiceventResetEvent(state:peventstate);

begin
  While sem_trywait(psem_t( plocaleventstate(state)^.FSem))=0 do
    ;
end;

procedure IntbasiceventSetEvent(state:peventstate);

Var
  Value : Longint;

begin
  pthread_mutex_lock(@plocaleventstate(state)^.feventsection);
  Try
    sem_getvalue(plocaleventstate(state)^.FSem,@value);
    if Value=0 then
      sem_post(psem_t( plocaleventstate(state)^.FSem));
  finally
    pthread_mutex_unlock(@plocaleventstate(state)^.feventsection);
  end;
end;

function IntbasiceventWaitFor(Timeout : Cardinal;state:peventstate) : longint;

begin
  If TimeOut<>Cardinal($FFFFFFFF) then
    result:=wrError
  else
    begin
    sem_wait(psem_t(plocaleventstate(state)^.FSem));
    result:=wrSignaled;
    if plocaleventstate(state)^.FManualReset then
      begin
        pthread_mutex_lock(@plocaleventstate(state)^.feventsection);
        Try
            intbasiceventresetevent(State);
            sem_post(psem_t( plocaleventstate(state)^.FSem));
          Finally
        pthread_mutex_unlock(@plocaleventstate(state)^.feventsection);
      end;
    end;
    end;
end;

Var
  NWThreadManager : TThreadManager;

Procedure InitSystemThreads;

begin
  With NWThreadManager do
  begin
    InitManager            :=nil;
    DoneManager            :=nil;
    BeginThread            :=@SysBeginThread;
    EndThread              :=@SysEndThread;
    SuspendThread          :=@SysSuspendThread;
    ResumeThread           :=@SysResumeThread;
    KillThread             :=@SysKillThread;
    ThreadSwitch           :=@SysThreadSwitch;
    WaitForThreadTerminate :=@SysWaitForThreadTerminate;
    ThreadSetPriority      :=@SysThreadSetPriority;
    ThreadGetPriority      :=@SysThreadGetPriority;
    GetCurrentThreadId     :=@SysGetCurrentThreadId;
    InitCriticalSection    :=@SysInitCriticalSection;
    DoneCriticalSection    :=@SysDoneCriticalSection;
    EnterCriticalSection   :=@SysEnterCriticalSection;
    LeaveCriticalSection   :=@SysLeaveCriticalSection;
    InitThreadVar          :=@SysInitThreadVar;
    RelocateThreadVar      :=@SysRelocateThreadVar;
    AllocateThreadVars     :=@SysAllocateThreadVars;
    ReleaseThreadVars      :=@SysReleaseThreadVars;
    BasicEventCreate       :=@intBasicEventCreate;
    BasicEventDestroy      :=@intBasicEventDestroy;
    BasicEventResetEvent   :=@intBasicEventResetEvent;
    BasicEventSetEvent     :=@intBasicEventSetEvent;
    BasiceventWaitFor      :=@intBasiceventWaitFor;
    end;
  SetThreadManager(NWThreadManager);
  ThVarAllocResourceTag := AllocateResourceTag(getnlmhandle,'Threadvar Memory',AllocSignature);
  NWSysSetThreadFunctions (@SysAllocateThreadVars,
                           @SysReleaseThreadVars,
                           @SysSetThreadDataAreaPtr);
end;



