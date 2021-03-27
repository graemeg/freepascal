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


{*****************************************************************************
                           Local WINApi imports
*****************************************************************************}

const
  { LocalAlloc flags  }
  LMEM_FIXED = 0;
  LMEM_ZEROINIT = 64;

{$ifndef WINCE}
function TlsAlloc : DWord;
  stdcall;external KernelDLL name 'TlsAlloc';
function TlsFree(dwTlsIndex : DWord) : LongBool;
  stdcall;external KernelDLL name 'TlsFree';
{$endif WINCE}
function TlsGetValue(dwTlsIndex : DWord) : pointer;
  {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'TlsGetValue';
function TlsSetValue(dwTlsIndex : DWord;lpTlsValue : pointer) : LongBool;
  {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'TlsSetValue';
function CreateThread(lpThreadAttributes : pointer;
  dwStackSize : SIZE_T; lpStartAddress : pointer;lpParameter : pointer;
  dwCreationFlags : DWord;var lpThreadId : DWord) : THandle;
  {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'CreateThread';
procedure ExitThread(dwExitCode : DWord);
  {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'ExitThread';
function LocalAlloc(uFlags:DWord; dwBytes:SIZE_T):Pointer;
  {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'LocalAlloc';
function LocalFree(hMem : Pointer):Pointer; {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'LocalFree';
procedure Sleep(dwMilliseconds: DWord); {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'Sleep';
function  WinSuspendThread (threadHandle : THandle) : dword; {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'SuspendThread';
function  WinResumeThread  (threadHandle : THandle) : dword; {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'ResumeThread';
function  WinCloseHandle  (threadHandle : THandle) : dword; {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'CloseHandle';
function  TerminateThread  (threadHandle : THandle; var exitCode : dword) : boolean; {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'TerminateThread';
function  WaitForSingleObject (hHandle : THandle;Milliseconds: dword): dword; {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'WaitForSingleObject';
function  WinThreadSetPriority (threadHandle : THandle; Prio: longint): boolean; {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'SetThreadPriority';
function  WinThreadGetPriority (threadHandle : THandle): LongInt; {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'GetThreadPriority';
{$ifndef WINCE}
function WinGetCurrentThread: THandle; stdcall; external KernelDLL name 'GetCurrentThread';
function WinOpenThread(dwDesiredAccess: DWord; bInheritHandle: Boolean; dwThreadId: DWord): THandle; stdcall; external KernelDLL name 'OpenThread';
function WinIsDebuggerPresent: Boolean; stdcall; external KernelDLL name 'IsDebuggerPresent';
type
  TSetThreadDescription = function(threadHandle: THandle; lpThreadDescription: PWideChar): HResult; stdcall;
var
  WinSetThreadDescription: TSetThreadDescription;
function  CreateEvent(lpEventAttributes:pointer;bManualReset:longbool;bInitialState:longbool;lpName:pchar): THandle; stdcall; external KernelDLL name 'CreateEventA';
function  ResetEvent(hEvent:THandle):LONGBOOL; stdcall; external KernelDLL name 'ResetEvent';
function  SetEvent(hEvent:THandle):LONGBOOL; stdcall; external KernelDLL name 'SetEvent';
{$endif WINCE}

procedure WinInitCriticalSection(var cs : TRTLCriticalSection);
  {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'InitializeCriticalSection';

procedure WinDoneCriticalSection(var cs : TRTLCriticalSection);
  {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'DeleteCriticalSection';

procedure WinEnterCriticalSection(var cs : TRTLCriticalSection);
  {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'EnterCriticalSection';

procedure WinLeaveCriticalSection(var cs : TRTLCriticalSection);
  {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'LeaveCriticalSection';

CONST
   WAIT_OBJECT_0 = 0;
   WAIT_ABANDONED_0 = $80;
   WAIT_TIMEOUT = $102;
   WAIT_IO_COMPLETION = $c0;
   WAIT_ABANDONED = $80;
   WAIT_FAILED = $ffffffff;

{$ifndef SUPPORT_WIN95}
function WinTryEnterCriticalSection(var cs : TRTLCriticalSection):longint;
  {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'TryEnterCriticalSection';
{$else SUPPORT_WIN95}
type
  TTryEnterCriticalSection = function(var cs : TRTLCriticalSection):longint; stdcall;
var
  WinTryEnterCriticalSection : TTryEnterCriticalSection;
{$endif SUPPORT_WIN95}

{*****************************************************************************
                             Threadvar support
*****************************************************************************}

    var
      // public names are used by heaptrc unit
      threadvarblocksize : dword; public name '_FPC_TlsSize';
      {$ifdef FPC_HAS_INDIRECT_ENTRY_INFORMATION}
      TLSKey : PDword = nil; public name '_FPC_TlsKey';
      {$else FPC_HAS_INDIRECT_ENTRY_INFORMATION}
      TLSKeyVar : DWord = $ffffffff;
      TLSKey : PDWord = @TLSKeyVar; public name '_FPC_TlsKey';
      {$endif FPC_HAS_INDIRECT_ENTRY_INFORMATION}

    var
      MainThreadIdWin32 : DWORD;

    procedure SysInitThreadvar(var offset : dword;size : dword);
      begin
        offset:=threadvarblocksize;
      {$ifdef CPUARM}
        // Data must be allocated at 4 bytes boundary for ARM
        size:=(size + 3) and not dword(3);
      {$endif CPUARM}
        inc(threadvarblocksize,size);
      end;


    procedure SysAllocateThreadVars; public name '_FPC_SysAllocateThreadVars';
      var
        dataindex : pointer;
        errorsave : dword;
      begin
        { we've to allocate the memory from system  }
        { because the FPC heap management uses      }
        { exceptions which use threadvars but       }
        { these aren't allocated yet ...            }
        { allocate room on the heap for the thread vars }
        errorsave:=GetLastError;
        if tlskey^=$ffffffff then
          RunError(226);
        dataindex:=TlsGetValue(tlskey^);
        if dataindex=nil then
          begin
            dataindex:=pointer(LocalAlloc(LMEM_FIXED or LMEM_ZEROINIT,threadvarblocksize));
            if dataindex=nil then
              RunError(226);
            TlsSetValue(tlskey^,dataindex);
          end;
        SetLastError(errorsave);
      end;

    function SysRelocateThreadvar(offset : dword) : pointer; forward;

    procedure SysInitTLS;
      begin
        { do not check IsMultiThread, as program could have altered it, out of Delphi habit }

        { the thread attach/detach code uses locks to avoid multiple calls of this }
        if TLSKey^=$ffffffff then
         begin
           { We're still running in single thread mode, setup the TLS }
           TLSKey^:=TlsAlloc;
           InitThreadVars(@SysRelocateThreadvar);
         end;
      end;


    procedure SysFiniMultithreading;
      begin
        if TLSKey^<>$ffffffff then
          TlsFree(TLSKey^);
        TLSKey^:=$ffffffff;
      end;

    function SysRelocateThreadvar(offset : dword) : pointer;
      var
        dataindex : pointer;
        errorsave : dword;
      begin
        errorsave:=GetLastError;
        dataindex:=TlsGetValue(tlskey^);
        if dataindex=nil then
          begin
            SysAllocateThreadVars;
            dataindex:=TlsGetValue(tlskey^);
            InitThread($1000000);
          end;
        SetLastError(errorsave);
        SysRelocateThreadvar:=DataIndex+Offset;
      end;


    procedure SysReleaseThreadVars;
      var
        p: pointer;
      begin
        if TLSKey^<>$ffffffff then
          begin
            p:=TlsGetValue(tlskey^);
            if Assigned(p) then
              LocalFree(p);
            TlsSetValue(tlskey^, nil);
          end;
      end;


{*****************************************************************************
                            Thread starting
*****************************************************************************}

    type
      pthreadinfo = ^tthreadinfo;
      tthreadinfo = record
        f : tthreadfunc;
        p : pointer;
        stklen : ptruint;
      end;

    function ThreadMain(param : pointer) : Longint; {$ifdef wince}cdecl{$else}stdcall{$endif};
      var
        ti : tthreadinfo;
      begin
        { Copy parameter to local data }
        ti:=pthreadinfo(param)^;

        { Handle all possible threadvar models:
          - dynamic threadvars: initialized either in DllMain,
            or upon accessing the threadvar ThreadID;
          - static threadvars+TLS callback: initialized in TLS callback;
          - static threadvars, no callback: ThreadID remains 0 and
            initialization happens here. }
        if ThreadID=TThreadID(0) then
          InitThread(ti.stklen);

        dispose(pthreadinfo(param));

        { Start thread function }
{$ifdef DEBUG_MT}
        writeln('Jumping to thread function of thread ',Win32GetCurrentThreadId);
{$endif DEBUG_MT}
{$if defined(FPC_USE_WIN64_SEH) or defined(FPC_USE_WIN32_SEH)}
        { use special 'top-level' exception handler around the thread function }
        ThreadMain:=main_wrapper(ti.p,pointer(ti.f));
{$else FPC_USE_WIN64_SEH}
        ThreadMain:=ti.f(ti.p);
{$endif FPC_USE_WIN64_SEH or FPC_USE_WIN32_SEH}
      end;


    function SysBeginThread(sa : Pointer;stacksize : ptruint;
                         ThreadFunction : tthreadfunc;p : pointer;
                         creationFlags : dword;var ThreadId : TThreadID) : TThreadID;
      var
        ti : pthreadinfo;
        _threadid : dword;
      begin
{$ifdef DEBUG_MT}
        writeln('Creating new thread');
{$endif DEBUG_MT}
        { Initialize multithreading if not done }
        SysInitTLS;
        if not IsMultiThread then
          begin
           { lazy initialize thread support }
            LazyInitThreading;
            IsMultiThread:=true;
          end;

        { the only way to pass data to the newly created thread
          in a MT safe way, is to use the heap }
        new(ti);
        ti^.f:=ThreadFunction;
        ti^.p:=p;
        ti^.stklen:=stacksize;
{$ifdef DEBUG_MT}
        writeln('Starting new thread');
{$endif DEBUG_MT}
        _threadid:=0;
        SysBeginThread:=CreateThread(sa,stacksize,@ThreadMain,ti,creationflags,_threadid);

        { creation failed? if yes, we dispose the parameter record }
        if SysBeginThread=0 then
          begin
{$ifdef DEBUG_MT}
            writeln('Thread creation failed');
{$endif DEBUG_MT}
            dispose(ti);
          end;

        ThreadID:=_threadid;
      end;


    procedure SysEndThread(ExitCode : DWord);
      begin
        DoneThread;
        ExitThread(ExitCode);
      end;


    procedure SysThreadSwitch;
    begin
      Sleep(0);
    end;


    function  SysSuspendThread (threadHandle : TThreadID) : dword;
    begin
      SysSuspendThread:=WinSuspendThread(threadHandle);
    end;


    function  SysResumeThread  (threadHandle : TThreadID) : dword;
    begin
      SysResumeThread:=WinResumeThread(threadHandle);
    end;


    function  SysKillThread (threadHandle : TThreadID) : dword;
    var exitCode : dword;
    begin
      if not TerminateThread (threadHandle, exitCode) then
        SysKillThread := GetLastError
      else
        SysKillThread := 0;
    end;

    function  SysCloseThread (threadHandle : TThreadID) : dword;
    begin
      SysCloseThread:=winCloseHandle(threadHandle);
    end;

    function  SysWaitForThreadTerminate (threadHandle : TThreadID; TimeoutMs : longint) : dword;
    begin
      if timeoutMs = 0 then dec (timeoutMs);  // $ffffffff is INFINITE
      SysWaitForThreadTerminate := WaitForSingleObject(threadHandle, TimeoutMs);
    end;


    function  SysThreadSetPriority (threadHandle : TThreadID; Prio: longint): boolean;            {-15..+15, 0=normal}
    begin
      SysThreadSetPriority:=WinThreadSetPriority(threadHandle,Prio);
    end;


    function  SysThreadGetPriority (threadHandle : TThreadID): longint;
    begin
      SysThreadGetPriority:=WinThreadGetPriority(threadHandle);
    end;

    function  SysGetCurrentThreadId : TThreadID;
    begin
      SysGetCurrentThreadId:=Win32GetCurrentThreadId;
    end;

{$ifndef WINCE}
    { following method is supported on older Windows versions AND currently only supported method by GDB }
    procedure RaiseMSVCExceptionMethod(threadHandle: TThreadID; const ThreadName: AnsiString);
    const
      MS_VC_EXCEPTION: DWord = $406D1388;
    type
      THREADNAME_INFO = record
        dwType: DWord; // Must be 0x1000.
        szName: PAnsiChar; // Pointer to name (in user addr space).
        dwThreadID: DWord; // Thread ID (-1=caller thread).
        dwFlags: DWord; // Reserved for future use, must be zero.
      end;
    var
      thrdinfo: THREADNAME_INFO;
    begin
      thrdinfo.dwType:=$1000;
      thrdinfo.szName:=@ThreadName[1];
      thrdinfo.dwThreadID:=threadHandle;
      thrdinfo.dwFlags:=0;
      try
        RaiseException(MS_VC_EXCEPTION, 0, SizeOf(thrdinfo) div SizeOf(DWord), @thrdinfo);
      except
        {do nothing}
      end;
    end;

    { following method needs at least Windows 10 version 1607 or Windows Server 2016 }
    procedure SetThreadDescriptionMethod(threadHandle: TThreadID; const ThreadName: UnicodeString);
    var
      thrdhandle: THandle;
      ClosingNeeded: Boolean;
    begin
      if threadHandle=TThreadID(-1) then
      begin
        thrdhandle:=WinGetCurrentThread;
        ClosingNeeded:=False;
      end
      else
      begin
        thrdhandle:=WinOpenThread($0400, False, threadHandle);
        ClosingNeeded:=True;
      end;

      WinSetThreadDescription(thrdhandle, @ThreadName[1]);

      if ClosingNeeded then
      begin
        CloseHandle(thrdhandle);
      end;
    end;
{$endif WINCE}

    procedure SysSetThreadDebugNameA(threadHandle: TThreadID; const ThreadName: AnsiString);
    begin
{$ifndef WINCE}
      if ThreadName = '' then
        Exit;

      if WinIsDebuggerPresent then
      begin
        RaiseMSVCExceptionMethod(threadHandle, ThreadName);
      end;

      if Assigned(WinSetThreadDescription) then
      begin
        SetThreadDescriptionMethod(threadHandle, UnicodeString(ThreadName));
      end;
{$else WINCE}
      {$Warning SetThreadDebugNameA needs to be implemented}
{$endif WINCE}
    end;

    procedure SysSetThreadDebugNameU(threadHandle: TThreadID; const ThreadName: UnicodeString);
    begin
{$ifndef WINCE}
      if ThreadName = '' then
        Exit;

      if WinIsDebuggerPresent then
      begin
        RaiseMSVCExceptionMethod(threadHandle, AnsiString(ThreadName));
      end;

      if Assigned(WinSetThreadDescription) then
      begin
        SetThreadDescriptionMethod(threadHandle, ThreadName);
      end;
{$else WINCE}
      {$Warning SetThreadDebugNameU needs to be implemented}
{$endif WINCE}
    end;

{*****************************************************************************
                          Delphi/Win32 compatibility
*****************************************************************************}

procedure SySInitCriticalSection(var cs);
begin
  WinInitCriticalSection(PRTLCriticalSection(@cs)^);
end;


procedure SysDoneCriticalSection(var cs);
begin
  WinDoneCriticalSection(PRTLCriticalSection(@cs)^);
end;


procedure SysEnterCriticalSection(var cs);
begin
  WinEnterCriticalSection(PRTLCriticalSection(@cs)^);
end;

{$ifdef SUPPORT_WIN95}
function Win95TryEnterCriticalSection(var cs : TRTLCriticalSection):longint;stdcall;
var
  MyThreadID : DWORD;
begin
  MyThreadId:=GetCurrentThreadId();
  if InterlockedIncrement(cs.LockCount)=0 then
    begin
      cs.OwningThread:=MyThreadId;
      cs.RecursionCount:=1;
      result:=1;
    end
  else
    begin
      if cs.OwningThread=MyThreadId then
        begin
          InterlockedDecrement(cs.LockCount);
          InterlockedIncrement(cs.RecursionCount);
          result:=1;
        end
      else
        begin
          InterlockedDecrement(cs.LockCount);
          result:=0;
        end;
    end;
end;
{$endif SUPPORT_WIN95}

function SysTryEnterCriticalSection(var cs):longint;
begin
  result:=WinTryEnterCriticalSection(PRTLCriticalSection(@cs)^);
end;

procedure SySLeaveCriticalSection(var cs);
begin
  WinLeaveCriticalSection(PRTLCriticalSection(@cs)^);
end;


Const
  wrSignaled = 0;
  wrTimeout  = 1;
  wrAbandoned= 2;
  wrError    = 3;

function intBasicEventCreate(EventAttributes : Pointer;
AManualReset,InitialState : Boolean;const Name : ansistring):pEventState;
var
  n : PChar;
begin
  if Length(Name) = 0 then
    n := Nil
  else
    n := PChar(Name);
  Result := PEventState(CreateEvent(EventAttributes, AManualReset, InitialState,n));
end;

procedure intbasiceventdestroy(state:peventstate);

begin
  closehandle(THandle(state));
end;

procedure intbasiceventResetEvent(state:peventstate);

begin
  ResetEvent(THandle(state))
end;

procedure intbasiceventSetEvent(state:peventstate);

begin
  SetEvent(THandle(state));
end;

function intbasiceventWaitFor(Timeout : Cardinal;state:peventstate) : longint;

begin
  case WaitForSingleObject(THandle(state), Timeout) of
    WAIT_ABANDONED: Result := wrAbandoned;
    WAIT_OBJECT_0: Result := wrSignaled;
    WAIT_TIMEOUT: Result := wrTimeout;
    WAIT_FAILED: Result := wrError;
  else
    Result := wrError;
  end;
end;

function intRTLEventCreate: PRTLEvent;
begin
  Result := PRTLEVENT(CreateEvent(nil, false, false, nil));
end;

procedure intRTLEventDestroy(AEvent: PRTLEvent);
begin
  CloseHandle(THANDLE(AEvent));
end;

procedure intRTLEventSetEvent(AEvent: PRTLEvent);
begin
  SetEvent(THANDLE(AEvent));
end;

procedure intRTLEventResetEvent(AEvent: PRTLEvent);
begin
  ResetEvent(THANDLE(AEvent));
end;

procedure intRTLEventWaitFor(AEvent: PRTLEvent);
const
  INFINITE=dword(-1);
begin
  WaitForSingleObject(THANDLE(AEvent), INFINITE);
end;

procedure intRTLEventWaitForTimeout(AEvent: PRTLEvent;timeout : longint);
begin
  WaitForSingleObject(THANDLE(AEvent), timeout);
end;


Var
  WinThreadManager : TThreadManager;

Procedure InitSystemThreads;public name '_FPC_InitSystemThreads';
{$ifndef WINCE}
var
  KernelHandle : THandle;
{$endif}
begin
  With WinThreadManager do
    begin
    InitManager            :=Nil;
    DoneManager            :=Nil;
    BeginThread            :=@SysBeginThread;
    EndThread              :=@SysEndThread;
    SuspendThread          :=@SysSuspendThread;
    ResumeThread           :=@SysResumeThread;
    KillThread             :=@SysKillThread;
    ThreadSwitch           :=@SysThreadSwitch;
    CloseThread            :=@SysCloseThread;
    WaitForThreadTerminate :=@SysWaitForThreadTerminate;
    ThreadSetPriority      :=@SysThreadSetPriority;
    ThreadGetPriority      :=@SysThreadGetPriority;
    GetCurrentThreadId     :=@SysGetCurrentThreadId;
    SetThreadDebugNameA    :=@SysSetThreadDebugNameA;
    SetThreadDebugNameU    :=@SysSetThreadDebugNameU;
    InitCriticalSection    :=@SysInitCriticalSection;
    DoneCriticalSection    :=@SysDoneCriticalSection;
    EnterCriticalSection   :=@SysEnterCriticalSection;
    TryEnterCriticalSection:=@SysTryEnterCriticalSection;
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
    RTLEventCreate         :=@intRTLEventCreate;
    RTLEventDestroy        :=@intRTLEventDestroy;
    RTLEventSetEvent       :=@intRTLEventSetEvent;
    RTLEventResetEvent     :=@intRTLEventResetEvent;
    RTLEventWaitFor        :=@intRTLEventWaitFor;
    RTLEventWaitForTimeout :=@intRTLEventWaitForTimeout;
    end;
  SetThreadManager(WinThreadManager);
  ThreadID := GetCurrentThreadID;
{$ifndef FPC_USE_TLS_DIRECTORY}
  if IsLibrary then
{$endif}
    SysInitTLS;

{$ifndef WINCE}
  KernelHandle:=GetModuleHandle(KernelDLL);
{$endif}

{$IFDEF SUPPORT_WIN95}
  { Try to find TryEnterCriticalSection function }
  if KernelHandle<>0 then
    WinTryEnterCriticalSection:=TTryEnterCriticalSection(WinGetProcAddress(KernelHandle,'TryEnterCriticalSection'));
  if not assigned(WinTryEnterCriticalSection) then
    WinTryEnterCriticalSection:=@Win95TryEnterCriticalSection;
{$ENDIF SUPPORT_WIN95}

{$ifndef WINCE}
  if KernelHandle<>0 then
  begin
    WinSetThreadDescription:=TSetThreadDescription(WinGetProcAddress(KernelHandle, 'SetThreadDescription'));
  end;
{$endif WINCE}
end;

