{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 by Peter Vreman,
    member of the Free Pascal development team.

    pthreads threading support implementation

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$ifdef linux}
{ we can combine both compile-time linking and dynamic loading, in order to:
    a) solve a problem on some systems with dynamically loading libpthread if
       it's not linked at compile time
    b) still enabling dynamically checking whether or not certain functions
       are available (could also be implemented via weak linking)
}
{$linklib pthread}
{$define dynpthreads} // Useless on BSD, since they are in libc
{$endif}


{ sem_init is best, since it does not consume any file descriptors.    }
{ sem_open is second best, since it consumes only one file descriptor  }
{ per semaphore.                                                       }
{ If neither is available, pipe is used as fallback, which consumes 2  }
{ file descriptors per semaphore.                                      }

{ Darwin doesn't support nameless semaphores in at least }
{ Mac OS X 10.4.8/Darwin 8.8                             }
{$if not defined(darwin) and not defined(iphonesim)}
{$define has_sem_init}
{$define has_sem_getvalue}
{$else }
{$if defined(darwin) or defined(iphonesim)}
{$define has_sem_open}
{$endif}
{$endif}

{$if defined(linux) or defined(aix) or defined(android)}
{$define has_sem_timedwait}
{$endif}

unit cthreads;
interface
{$S-}

{$ifndef dynpthreads}   // If you have problems compiling this on FreeBSD 5.x
 {$linklib c}           // try adding -Xf
 {$if not defined(Darwin) and not defined(iphonesim) and not defined(Android)}
   {$ifndef haiku}
     {$linklib pthread}
   {$endif haiku}
 {$endif darwin}
{$endif}

{$define basicevents_with_pthread_cond}

Procedure SetCThreadManager;

implementation

Uses
{$if defined(Linux) and not defined(Android)}
  Linux,
{$endif}
  BaseUnix,
  unix,
  unixtype,
  initc
{$ifdef dynpthreads}
  ,dl
{$endif}
  ;

{*****************************************************************************
                             System unit import
*****************************************************************************}

procedure fpc_threaderror; [external name 'FPC_THREADERROR'];

{*****************************************************************************
                             Generic overloaded
*****************************************************************************}

{ Include OS specific parts. }
{$i pthread.inc}

Type  PINTRTLEvent = ^TINTRTLEvent;
      TINTRTLEvent = record
        condvar: pthread_cond_t;
        mutex: pthread_mutex_t;
        isset: boolean;
       end;

      TTryWaitResult = (tw_error, tw_semwasunlocked, tw_semwaslocked);

{*****************************************************************************
                             Threadvar support
*****************************************************************************}

    const
      threadvarblocksize : dword = 0;


    var
      TLSKey,
      CleanupKey : pthread_key_t;

    procedure CInitThreadvar(var offset : dword;size : dword);
      begin
        {$ifdef cpusparc}
        threadvarblocksize:=align(threadvarblocksize,16);
        {$endif cpusparc}
        
        {$ifdef cpusparc64}
        threadvarblocksize:=align(threadvarblocksize,16);
        {$endif cpusparc64}

        {$ifdef cpupowerpc}
        threadvarblocksize:=align(threadvarblocksize,8);
        {$endif cpupowerc}

        {$ifdef cpui386}
        threadvarblocksize:=align(threadvarblocksize,8);
        {$endif cpui386}

        {$ifdef cpuarm}
        threadvarblocksize:=align(threadvarblocksize,4);
        {$endif cpuarm}

        {$ifdef cpum68k}
        threadvarblocksize:=align(threadvarblocksize,2);
        {$endif cpum68k}

        {$ifdef cpux86_64}
        threadvarblocksize:=align(threadvarblocksize,16);
        {$endif cpux86_64}

        {$ifdef cpupowerpc64}
        threadvarblocksize:=align(threadvarblocksize,16);
        {$endif cpupowerpc64}

        {$ifdef cpuaarch64}
        threadvarblocksize:=align(threadvarblocksize,16);
        {$endif cpuaarch64}

        offset:=threadvarblocksize;

        inc(threadvarblocksize,size);
      end;



    procedure CAllocateThreadVars;
      var
        dataindex : pointer;
      begin
{$ifndef FPC_SECTION_THREADVARS}
        { we've to allocate the memory from system  }
        { because the FPC heap management uses      }
        { exceptions which use threadvars but       }
        { these aren't allocated yet ...            }
        { allocate room on the heap for the thread vars }
        DataIndex:=Pointer(Fpmmap(nil,threadvarblocksize,3,MAP_PRIVATE+MAP_ANONYMOUS,-1,0));
        FillChar(DataIndex^,threadvarblocksize,0);
        pthread_setspecific(tlskey,dataindex);
{$endif FPC_SECTION_THREADVARS}
      end;


    procedure CthreadCleanup(p: pointer); cdecl;
    {$ifdef DEBUG_MT}
      var
        s: string[100]; // not an ansistring
{$endif DEBUG_MT}
      begin
{$ifdef DEBUG_MT}
        s := 'finishing externally started thread'#10;
        fpwrite(0,s[1],length(s));
{$endif DEBUG_MT}
        { Restore tlskey value as it may already have been set to null,
          in which case
            a) DoneThread can't release the memory
            b) accesses to threadvars from DoneThread or anything it
               calls would allocate new threadvar memory
        }
        pthread_setspecific(tlskey,p);
        { clean up }
        DoneThread;
        { the pthread routine that calls us is supposed to do this, but doesn't
          at least on Mac OS X 10.6 }
        pthread_setspecific(CleanupKey,nil);
        pthread_setspecific(tlskey,nil);
      end;


    procedure HookThread;
      begin
        { Allocate local thread vars, this must be the first thing,
          because the exception management and io depends on threadvars }
        CAllocateThreadVars;
        { we cannot know the stack size of the current thread, so pretend it
          is really large to prevent spurious stack overflow errors }
        InitThread(1000000000);
        { instruct the pthreads system to clean up this thread when it exits.
          Use current tlskey as value so that if tlskey is cleared before
          CleanupKey is called, we still know its value (the order in which
          pthread tls data is zeroed by pthreads is undefined, and under some
          systems the tlskey is cleared first) }
        pthread_setspecific(CleanupKey,pthread_getspecific(tlskey));
      end;


    function CRelocateThreadvar(offset : dword) : pointer;
      var
        P : Pointer;
      begin
        P:=pthread_getspecific(tlskey);
        { a thread which we did not create? }
        if (P=Nil) then
          begin
            HookThread;
            // If this also goes wrong: bye bye threadvars...
            P:=pthread_getspecific(tlskey);
          end;
        CRelocateThreadvar:=P+Offset;
      end;


    procedure CReleaseThreadVars;
      begin
{$ifndef FPC_SECTION_THREADVARS}
        Fpmunmap(pointer(pthread_getspecific(tlskey)),threadvarblocksize);
{$endif FPC_SECTION_THREADVARS}
      end;

{ Include OS independent Threadvar initialization }



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

    function ThreadMain(param : pointer) : pointer;cdecl;
      var
        ti : tthreadinfo;
        nset: tsigset;
{$if defined(linux) and not defined(FPC_USE_LIBC)}
        nlibcset: tlibc_sigset;
{$endif linux/no FPC_USE_LIBC}
{$ifdef DEBUG_MT}
        // in here, don't use write/writeln before having called
        // InitThread! I wonder if anyone ever debugged these routines,
        // because they will have crashed if DEBUG_MT was enabled!
        // this took me the good part of an hour to figure out
        // why it was crashing all the time!
        // this is kind of a workaround, we simply write(2) to fd 0
        s: string[100]; // not an ansistring
{$endif DEBUG_MT}
      begin
{$ifdef DEBUG_MT}
        s := 'New thread started, initing threadvars'#10;
        fpwrite(0,s[1],length(s));
{$endif DEBUG_MT}
        { unblock all signals we are interested in (may be blocked by }
        { default in new threads on some OSes, see #9073)             }
        fpsigemptyset(nset);
        fpsigaddset(nset,SIGSEGV);
        fpsigaddset(nset,SIGBUS);
        fpsigaddset(nset,SIGFPE);
        fpsigaddset(nset,SIGILL);
{$if defined(linux) and not defined(FPC_USE_LIBC)}
        { sigset_t has a different size for linux/kernel and linux/libc }
        fillchar(nlibcset,sizeof(nlibcset),0);
        if (sizeof(nlibcset)>sizeof(nset)) then
          move(nset,nlibcset,sizeof(nset))
        else
          move(nset,nlibcset,sizeof(nlibcset));
        pthread_sigmask(SIG_UNBLOCK,@nlibcset,nil);
{$else linux}
        pthread_sigmask(SIG_UNBLOCK,@nset,nil);
{$endif linux}
        { Allocate local thread vars, this must be the first thing,
          because the exception management and io depends on threadvars }
        CAllocateThreadVars;
        { Copy parameter to local data }
{$ifdef DEBUG_MT}
        s := 'New thread started, initialising ...'#10;
        fpwrite(0,s[1],length(s));
{$endif DEBUG_MT}
        ti:=pthreadinfo(param)^;

        { Initialize thread }
        InitThread(ti.stklen);

        dispose(pthreadinfo(param));
        { Start thread function }
{$ifdef DEBUG_MT}
        writeln('Jumping to thread function');
{$endif DEBUG_MT}
        ThreadMain:=pointer(ti.f(ti.p));
        DoneThread;
        pthread_exit(ThreadMain);
      end;


  var
    TLSInitialized : longbool = FALSE;

  Procedure InitCTLS;
  
  begin
    if (InterLockedExchange(longint(TLSInitialized),ord(true)) = 0) then
      begin
        { We're still running in single thread mode, setup the TLS }
        pthread_key_create(@TLSKey,nil);
        InitThreadVars(@CRelocateThreadvar);
        { used to clean up threads that we did not create ourselves:
           a) the default value for a key (and hence also this one) in
              new threads is NULL, and if it's still like that when the
              thread terminates, nothing will happen
           b) if it's non-NULL, the destructor routine will be called
              when the thread terminates
         -> we will set it to 1 if the threadvar relocation routine is
            called from a thread we did not create, so that we can
            clean up everything at the end }
        pthread_key_create(@CleanupKey,@CthreadCleanup);
      end
  end;

  function CBeginThread(sa : Pointer;stacksize : PtrUInt;
                       ThreadFunction : tthreadfunc;p : pointer;
                       creationFlags : dword; var ThreadId : TThreadId) : TThreadID;
    var
      ti : pthreadinfo;
      thread_attr : pthread_attr_t;
    begin
{$ifdef DEBUG_MT}
      writeln('Creating new thread');
{$endif DEBUG_MT}
      { Initialize multithreading if not done }
      if not TLSInitialized then
        InitCTLS;
      IsMultiThread:=true;

      { the only way to pass data to the newly created thread
        in a MT safe way, is to use the heap }
      new(ti);
      ti^.f:=ThreadFunction;
      ti^.p:=p;
      ti^.stklen:=stacksize;
      { call pthread_create }
{$ifdef DEBUG_MT}
      writeln('Starting new thread');
{$endif DEBUG_MT}
      pthread_attr_init(@thread_attr);
      {$if not defined(HAIKU)and not defined(BEOS) and not defined(ANDROID)}
      {$if defined (solaris) or defined (netbsd) }
      pthread_attr_setinheritsched(@thread_attr, PTHREAD_INHERIT_SCHED);
      {$else not solaris}
      pthread_attr_setinheritsched(@thread_attr, PTHREAD_EXPLICIT_SCHED);
      {$endif not solaris}
      {$ifend}

      // will fail under linux -- apparently unimplemented
      pthread_attr_setscope(@thread_attr, PTHREAD_SCOPE_PROCESS);

      // don't create detached, we need to be able to join (waitfor) on
      // the newly created thread!
      //pthread_attr_setdetachstate(@thread_attr, PTHREAD_CREATE_DETACHED);

      // set the stack size
      if (pthread_attr_setstacksize(@thread_attr, stacksize)<>0) or
         // and create the thread
         (pthread_create(ppthread_t(@threadid), @thread_attr, @ThreadMain,ti) <> 0) then

        begin
          dispose(ti);
          threadid := TThreadID(0);
        end;
      CBeginThread:=threadid;
      pthread_attr_destroy(@thread_attr);
{$ifdef DEBUG_MT}
      writeln('BeginThread returning ',ptrint(CBeginThread));
{$endif DEBUG_MT}
    end;


  procedure CEndThread(ExitCode : DWord);
    begin
      DoneThread;
      pthread_detach(pthread_t(pthread_self()));
      pthread_exit(pointer(ptrint(ExitCode)));
    end;



  function  CSuspendThread (threadHandle : TThreadID) : dword;
    begin
    {  pthread_kill(SIGSTOP) cannot be used, because posix-compliant
       implementations then freeze the entire process instead of only
       the target thread. Suspending a particular thread is not
       supported by posix nor by most *nix implementations, presumably
       because of concerns mentioned in E.4 at
       http://pauillac.inria.fr/~xleroy/linuxthreads/faq.html#E and in
       http://java.sun.com/j2se/1.4.2/docs/guide/misc/threadPrimitiveDeprecation.html
    }
//      result := pthread_kill(threadHandle,SIGSTOP);
      result:=dword(-1);
    end;


  function  CResumeThread  (threadHandle : TThreadID) : dword;
    begin
      result:=dword(-1);
    end;


  procedure sched_yield; cdecl; external 'c' name 'sched_yield';

  procedure CThreadSwitch;  {give time to other threads}
    begin
      { At least on Mac OS X, the pthread_yield_np calls through to this. }
      { Further, sched_yield is in POSIX and supported on FreeBSD 4+,     }
      { Linux, Mac OS X and Solaris, while the thread-specific yield      }
      { routines are called differently everywhere and non-standard.      }
      sched_yield;
    end;


  function  CKillThread (threadHandle : TThreadID) : dword;
    begin
      pthread_detach(pthread_t(threadHandle));
{$ifndef android}
      CKillThread := pthread_cancel(pthread_t(threadHandle));
{$else}
      CKillThread := dword(-1);
{$endif}
    end;

  function CCloseThread (threadHandle : TThreadID) : dword;
    begin
      result:=0;
    end;

  function  CWaitForThreadTerminate (threadHandle : TThreadID; TimeoutMs : longint) : dword;  {0=no timeout}
    var
      LResultP: Pointer;
    begin
      pthread_join(pthread_t(threadHandle), @LResultP);
      CWaitForThreadTerminate := dword(LResultP);
    end;

    function  CThreadSetPriority (threadHandle : TThreadID; Prio: longint): boolean; {-15..+15, 0=normal}
    begin
      {$Warning ThreadSetPriority needs to be implemented}
      result:=false;
    end;


  function  CThreadGetPriority (threadHandle : TThreadID): Integer;
    begin
      {$Warning ThreadGetPriority needs to be implemented}
      result:=0;
    end;


  function  CGetCurrentThreadId : TThreadID;
    begin
      CGetCurrentThreadId := TThreadID (pthread_self());
    end;


{*****************************************************************************
                          Delphi/Win32 compatibility
*****************************************************************************}

    procedure CInitCriticalSection(var CS);

    var
      MAttr : pthread_mutexattr_t;
      res: longint;
    begin
      res:=pthread_mutexattr_init(@MAttr);
      if res=0 then
        begin
          res:=pthread_mutexattr_settype(@MAttr,longint(_PTHREAD_MUTEX_RECURSIVE));
          if res=0 then
            res := pthread_mutex_init(@CS,@MAttr)
          else
            { No recursive mutex support :/ }
            fpc_threaderror
        end
      else
        res:= pthread_mutex_init(@CS,NIL);
      pthread_mutexattr_destroy(@MAttr);
      if res <> 0 then
        fpc_threaderror;
    end;

    procedure CEnterCriticalSection(var CS);
      begin
         if pthread_mutex_lock(@CS) <> 0 then
           fpc_threaderror
      end;

    function CTryEnterCriticalSection(var CS):longint;
      begin
         if pthread_mutex_Trylock(@CS)=0 then
           result:=1  // succes
         else
           result:=0; // failure
      end;

    procedure CLeaveCriticalSection(var CS);
      begin
         if pthread_mutex_unlock(@CS) <> 0 then
           fpc_threaderror
      end;

    procedure CDoneCriticalSection(var CS);
      begin
         { unlock as long as unlocking works to unlock it if it is recursive
           some Delphi code might call this function with a locked mutex      }
         while pthread_mutex_unlock(@CS)=0 do
           ;

         if pthread_mutex_destroy(@CS) <> 0 then
           fpc_threaderror;
      end;


{*****************************************************************************
                           Semaphore routines
*****************************************************************************}


type
     TPthreadCondition = pthread_cond_t;
     TPthreadMutex = pthread_mutex_t;
     Tbasiceventstate=record
         FCondVar: TPthreadCondition;
{$if defined(Linux) and not defined(Android)}         
         FAttr: pthread_condattr_t;
         FClockID: longint;
{$ifend}        
         FEventSection: TPthreadMutex;
         FWaiters: longint;
         FIsSet,
         FManualReset,
         FDestroying : Boolean;
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
  MAttr : pthread_mutexattr_t;
  res   : cint;  
{$if defined(Linux) and not defined(Android)}  
  timespec: ttimespec;
{$ifend}  
begin
  new(plocaleventstate(result));
  plocaleventstate(result)^.FManualReset:=AManualReset;
  plocaleventstate(result)^.FWaiters:=0;
  plocaleventstate(result)^.FDestroying:=False;
  plocaleventstate(result)^.FIsSet:=InitialState;
{$if defined(Linux) and not defined(Android)}  
  res := pthread_condattr_init(@plocaleventstate(result)^.FAttr);
  if (res <> 0) then
  begin
    FreeMem(result);
    fpc_threaderror;  
  end;
  
  if clock_gettime(CLOCK_MONOTONIC_RAW, @timespec) = 0 then
  begin
    res := pthread_condattr_setclock(@plocaleventstate(result)^.FAttr, CLOCK_MONOTONIC_RAW);
  end
  else
  begin
    res := -1; // No support for CLOCK_MONOTONIC_RAW   
  end;
  
  if (res = 0) then
  begin
    plocaleventstate(result)^.FClockID := CLOCK_MONOTONIC_RAW;
  end
  else
  begin
    res := pthread_condattr_setclock(@plocaleventstate(result)^.FAttr, CLOCK_MONOTONIC);
    if (res = 0) then
    begin
      plocaleventstate(result)^.FClockID := CLOCK_MONOTONIC;
    end
    else
    begin
      pthread_condattr_destroy(@plocaleventstate(result)^.FAttr);
      FreeMem(result);
      fpc_threaderror;  
    end;    
  end;  

  res := pthread_cond_init(@plocaleventstate(result)^.FCondVar, @plocaleventstate(result)^.FAttr);
  if (res <> 0) then
  begin
    pthread_condattr_destroy(@plocaleventstate(result)^.FAttr);  
    FreeMem(result);
    fpc_threaderror;
  end;
{$else}
  res := pthread_cond_init(@plocaleventstate(result)^.FCondVar, nil);
  if (res <> 0) then
  begin
    FreeMem(result);
    fpc_threaderror;
  end; 
{$ifend} 

  res:=pthread_mutexattr_init(@MAttr);
  if res=0 then
    begin
      res:=pthread_mutexattr_settype(@MAttr,longint(_PTHREAD_MUTEX_RECURSIVE));
      if Res=0 then
        Res:=pthread_mutex_init(@plocaleventstate(result)^.feventsection,@MAttr)
      else
        res:=pthread_mutex_init(@plocaleventstate(result)^.feventsection,nil);
    end
  else
    res:=pthread_mutex_init(@plocaleventstate(result)^.feventsection,nil);

  pthread_mutexattr_destroy(@MAttr);
  if res <> 0 then
    begin
      pthread_cond_destroy(@plocaleventstate(result)^.FCondVar);
{$if defined(Linux) and not defined(Android)}  
      pthread_condattr_destroy(@plocaleventstate(result)^.FAttr);	
{$ifend}      
      FreeMem(result);
      fpc_threaderror;
    end;
end;

procedure Intbasiceventdestroy(state:peventstate);
begin
  { safely mark that we are destroying this event }
  pthread_mutex_lock(@plocaleventstate(state)^.feventsection);
  plocaleventstate(state)^.FDestroying:=true;

  { send a signal to all threads that are waiting }
  pthread_cond_broadcast(@plocaleventstate(state)^.FCondVar);
  pthread_mutex_unlock(@plocaleventstate(state)^.feventsection);

  { now wait until they've finished their business }
  while (plocaleventstate(state)^.FWaiters <> 0) do
    cThreadSwitch;

  { and clean up }
  pthread_cond_destroy(@plocaleventstate(state)^.Fcondvar);
{$if defined(Linux) and not defined(Android)}  
  pthread_condattr_destroy(@plocaleventstate(state)^.FAttr);	
{$ifend}  
  pthread_mutex_destroy(@plocaleventstate(state)^.FEventSection);
  dispose(plocaleventstate(state));
end;


procedure IntbasiceventResetEvent(state:peventstate);
begin
  pthread_mutex_lock(@plocaleventstate(state)^.feventsection);
  plocaleventstate(state)^.fisset:=false;
  pthread_mutex_unlock(@plocaleventstate(state)^.feventsection);
end;

procedure IntbasiceventSetEvent(state:peventstate);
begin
  pthread_mutex_lock(@plocaleventstate(state)^.feventsection);
  plocaleventstate(state)^.Fisset:=true;
  if not(plocaleventstate(state)^.FManualReset) then
    pthread_cond_signal(@plocaleventstate(state)^.Fcondvar)
  else
    pthread_cond_broadcast(@plocaleventstate(state)^.Fcondvar);
  pthread_mutex_unlock(@plocaleventstate(state)^.feventsection);
end;

function IntbasiceventWaitFor(Timeout : Cardinal;state:peventstate) : longint;
var
  timespec: ttimespec;
  errres: cint;
  isset: boolean;
  tnow : timeval;
begin

  { safely check whether we are being destroyed, if so immediately return. }
  { otherwise (under the same mutex) increase the number of waiters        }
  pthread_mutex_lock(@plocaleventstate(state)^.feventsection);
  if (plocaleventstate(state)^.FDestroying) then
    begin
      pthread_mutex_unlock(@plocaleventstate(state)^.feventsection);
      result := wrAbandoned;
      exit;
    end;
  { not a regular inc() because it may happen simulatneously with the }
  { interlockeddecrement() at the end                                 }
  interlockedincrement(plocaleventstate(state)^.FWaiters);

  //Wait without timeout using pthread_cond_wait
  if Timeout = $FFFFFFFF then
    begin
      while (not plocaleventstate(state)^.FIsSet) and (not plocaleventstate(state)^.FDestroying) do
        pthread_cond_wait(@plocaleventstate(state)^.Fcondvar, @plocaleventstate(state)^.feventsection);
    end
  else
    begin
      //Wait with timeout using pthread_cond_timedwait
{$if defined(Linux) and not defined(Android)}
      if clock_gettime(plocaleventstate(state)^.FClockID, @timespec) <> 0 then
      begin
        Result := Ord(wrError);
        Exit;
      end;
      timespec.tv_sec  := timespec.tv_sec + (clong(timeout) div 1000);
      timespec.tv_nsec := ((clong(timeout) mod 1000) * 1000000) + (timespec.tv_nsec);
{$else}
      // TODO: FIX-ME: Also use monotonic clock for other *nix targets
      fpgettimeofday(@tnow, nil);
      timespec.tv_sec  := tnow.tv_sec + (clong(timeout) div 1000);
      timespec.tv_nsec := ((clong(timeout) mod 1000) * 1000000) + (tnow.tv_usec * 1000);
{$ifend}
      if timespec.tv_nsec >= 1000000000 then
        begin
          inc(timespec.tv_sec);
          dec(timespec.tv_nsec, 1000000000);
        end;
      errres := 0;
      while (not plocaleventstate(state)^.FDestroying) and
            (not plocaleventstate(state)^.FIsSet) and 
            (errres<>ESysETIMEDOUT) do
        errres := pthread_cond_timedwait(@plocaleventstate(state)^.Fcondvar,
                                         @plocaleventstate(state)^.feventsection, 
                                         @timespec);
    end;

  isset := plocaleventstate(state)^.FIsSet;

  { if ManualReset=false, reset the event immediately. }
  if (plocaleventstate(state)^.FManualReset=false) then
    plocaleventstate(state)^.FIsSet := false;

  //check the results...
  if plocaleventstate(state)^.FDestroying then
    Result := wrAbandoned
  else
    if IsSet then
      Result := wrSignaled
    else
      begin
        if errres=ESysETIMEDOUT then
          Result := wrTimeout
        else
          Result := wrError;
      end;

  pthread_mutex_unlock(@plocaleventstate(state)^.feventsection);

  { don't put this above the previous pthread_mutex_unlock, because    }
  { otherwise we can get errors in case an object is destroyed between }
  { end of the wait/sleep loop and the signalling above.               }
  { The pthread_mutex_unlock above takes care of the memory barrier    }
  interlockeddecrement(plocaleventstate(state)^.FWaiters);
end;

function intRTLEventCreate: PRTLEvent;

var p:pintrtlevent;

begin
  new(p);
  if not assigned(p) then
    fpc_threaderror;
  if pthread_cond_init(@p^.condvar, nil)<>0 then
    begin
      dispose(p);
      fpc_threaderror;
    end;
  if pthread_mutex_init(@p^.mutex, nil)<>0 then
    begin
      pthread_cond_destroy(@p^.condvar);
      dispose(p);
      fpc_threaderror;
    end;
  p^.isset:=false;
  result:=PRTLEVENT(p);
end;

procedure intRTLEventDestroy(AEvent: PRTLEvent);

var p:pintrtlevent;

begin
  p:=pintrtlevent(aevent);
  pthread_cond_destroy(@p^.condvar);
  pthread_mutex_destroy(@p^.mutex);
  dispose(p);
end;

procedure intRTLEventSetEvent(AEvent: PRTLEvent);
var p:pintrtlevent;

begin
  p:=pintrtlevent(aevent);
  pthread_mutex_lock(@p^.mutex);
  p^.isset:=true;
  pthread_cond_signal(@p^.condvar);
  pthread_mutex_unlock(@p^.mutex);
end;


procedure intRTLEventResetEvent(AEvent: PRTLEvent);
var p:pintrtlevent;

begin
  p:=pintrtlevent(aevent);
  pthread_mutex_lock(@p^.mutex);
  p^.isset:=false;
  pthread_mutex_unlock(@p^.mutex);
end;


procedure intRTLEventWaitFor(AEvent: PRTLEvent);
var p:pintrtlevent;

begin
  p:=pintrtlevent(aevent);
  pthread_mutex_lock(@p^.mutex);
  while not p^.isset do pthread_cond_wait(@p^.condvar, @p^.mutex);
  p^.isset:=false;
  pthread_mutex_unlock(@p^.mutex);
end;

procedure intRTLEventWaitForTimeout(AEvent: PRTLEvent;timeout : longint);
  var
    p : pintrtlevent;
    errres : cint;
    timespec : ttimespec;
    tnow : timeval;

  begin
    p:=pintrtlevent(aevent);
    fpgettimeofday(@tnow,nil);
    timespec.tv_sec:=tnow.tv_sec+(timeout div 1000);
    timespec.tv_nsec:=(timeout mod 1000)*1000000 + tnow.tv_usec*1000;
    if timespec.tv_nsec >= 1000000000 then
    begin
      inc(timespec.tv_sec);
      dec(timespec.tv_nsec, 1000000000);
    end;
    errres:=0;
    pthread_mutex_lock(@p^.mutex);
    while (not p^.isset) and
          (errres <> ESysETIMEDOUT) do
      begin
        errres:=pthread_cond_timedwait(@p^.condvar, @p^.mutex, @timespec);
      end;
    p^.isset:=false;
    pthread_mutex_unlock(@p^.mutex);
  end;


type
  threadmethod = procedure of object;


Function CInitThreads : Boolean;

begin
{$ifdef DEBUG_MT}
  Writeln('Entering InitThreads.');
{$endif}
{$ifndef dynpthreads}
  Result:=True;
{$else}
  Result:=LoadPthreads;
{$endif}
  ThreadID := TThreadID (pthread_self());
{$ifdef DEBUG_MT}
  Writeln('InitThreads : ',Result);
{$endif DEBUG_MT}
  // We assume that if you set the thread manager, the application is multithreading.
  InitCTLS;
end;

Function CDoneThreads : Boolean;

begin
{$ifndef dynpthreads}
  Result:=True;
{$else}
  Result:=UnloadPthreads;
{$endif}
end;


Var
  CThreadManager : TThreadManager;

Procedure SetCThreadManager;

begin
  With CThreadManager do begin
    InitManager            :=@CInitThreads;
    DoneManager            :=@CDoneThreads;
    BeginThread            :=@CBeginThread;
    EndThread              :=@CEndThread;
    SuspendThread          :=@CSuspendThread;
    ResumeThread           :=@CResumeThread;
    KillThread             :=@CKillThread;
    ThreadSwitch           :=@CThreadSwitch;
    CloseThread	           :=@CCloseThread;
    WaitForThreadTerminate :=@CWaitForThreadTerminate;
    ThreadSetPriority      :=@CThreadSetPriority;
    ThreadGetPriority      :=@CThreadGetPriority;
    GetCurrentThreadId     :=@CGetCurrentThreadId;
    InitCriticalSection    :=@CInitCriticalSection;
    DoneCriticalSection    :=@CDoneCriticalSection;
    EnterCriticalSection   :=@CEnterCriticalSection;
    TryEnterCriticalSection:=@CTryEnterCriticalSection;
    LeaveCriticalSection   :=@CLeaveCriticalSection;
    InitThreadVar          :=@CInitThreadVar;
    RelocateThreadVar      :=@CRelocateThreadVar;
    AllocateThreadVars     :=@CAllocateThreadVars;
    ReleaseThreadVars      :=@CReleaseThreadVars;
    BasicEventCreate       :=@intBasicEventCreate;
    BasicEventDestroy      :=@intBasicEventDestroy;
    BasicEventResetEvent   :=@intBasicEventResetEvent;
    BasicEventSetEvent     :=@intBasicEventSetEvent;
    BasiceventWaitFor      :=@intBasiceventWaitFor;
    rtlEventCreate         :=@intrtlEventCreate;
    rtlEventDestroy        :=@intrtlEventDestroy;
    rtlEventSetEvent       :=@intrtlEventSetEvent;
    rtlEventResetEvent     :=@intrtlEventResetEvent;
    rtleventWaitForTimeout :=@intrtleventWaitForTimeout;
    rtleventWaitFor        :=@intrtleventWaitFor;
  end;
  SetThreadManager(CThreadManager);
end;


initialization
  if ThreadingAlreadyUsed then
    begin
      writeln('Threading has been used before cthreads was initialized.');
      writeln('Make cthreads one of the first units in your uses clause.');
      runerror(211);
    end;
  SetCThreadManager;
finalization
end.
