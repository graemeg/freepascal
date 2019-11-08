{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.
    Copyright (c) 2001-2011 by Armin Diehl.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{ no stack check in system }
{$S-}
unit System;

interface

{$define FPC_IS_SYSTEM}
{$define StdErrToConsole}
{$define useLongNamespaceByDefault}
{$define autoHeapRelease}
{$define DISABLE_NO_THREAD_MANAGER}

{$ifdef SYSTEMDEBUG}
  {$define SYSTEMEXCEPTIONDEBUG}
{$endif SYSTEMDEBUG}

{$ifdef cpui386}
  {$define Set_i386_Exception_handler}
{$endif cpui386}

{ include system-independent routine headers }

{$I systemh.inc}

{Platform specific information}
const
 LineEnding = #13#10;
 LFNSupport : boolean = false;
 DirectorySeparator = '/';
 DriveSeparator = ':';
 ExtensionSeparator = '.';
 PathSeparator = ';';
 AllowDirectorySeparators : set of char = ['\','/'];
 AllowDriveSeparators : set of char = [':'];
{ FileNameCaseSensitive and FileNameCasePreserving are defined separately below!!! }
 maxExitCode = 255;
 MaxPathLen = 256;
 AllFilesMask = '*';
 SharedSuffix = 'nlm';

CONST
  { Default filehandles }
   UnusedHandle    : THandle = -1;
   StdInputHandle  : THandle = 0;
   StdOutputHandle : THandle = 0;
   StdErrorHandle  : THandle = 0;

   FileNameCaseSensitive : boolean = false;
   FileNameCasePreserving: boolean = true; (* Not really correct on older Netware versions without the LFN support switched on... *)
   CtrlZMarksEOF: boolean = false; (* #26 not considered as end of file *)

   sLineBreak = LineEnding;
   DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

TYPE
   TNWCheckFunction = procedure (var code : longint);

VAR
   ArgC   : INTEGER;
   ArgV   : ppchar;
   NetwareCheckFunction    : TNWCheckFunction;
   NetwareMainThreadGroupID: longint;
   NetwareUnloadProc       : pointer = nil;  {like exitProc but for nlm unload only}

CONST
   envp   : ppchar = nil;   {dummy to make heaptrc happy}


procedure ConsolePrintf (FormatStr : PCHAR; Param : LONGINT); CDecl; external 'clib' name 'printf';
procedure ConsolePrintf (FormatStr : PCHAR; Param : pchar); CDecl; external 'clib' name 'printf';
procedure ConsolePrintf (FormatStr : PCHAR; P1,P2 : LONGINT);  CDecl; external 'clib' name 'printf';
procedure ConsolePrintf (FormatStr : PCHAR; P1,P2,P3 : LONGINT);  CDecl; external 'clib' name 'printf';
procedure ConsolePrintf (FormatStr : PCHAR);  CDecl; external 'clib' name 'printf';
// this gives internal compiler error 200404181
// procedure ConsolePrintf (FormatStr : PCHAR; Param : array of const); CDecl; EXTERNAL 'clib' name 'ConsolePrintf';

procedure __EnterDebugger; cdecl; external 'clib' name 'EnterDebugger';

type
  TSysCloseAllRemainingSemaphores = procedure;
  TSysReleaseThreadVars = procedure;
  TSysSetThreadDataAreaPtr = function (newPtr:pointer):pointer;

procedure NWSysSetThreadFunctions (crs:TSysCloseAllRemainingSemaphores;
                                   rtv:TSysReleaseThreadVars;
                                   stdata:TSysSetThreadDataAreaPtr);

function NWGetCodeStart : pointer;  // needed for lineinfo


implementation
{ Indicate that stack checking is taken care by OS}
{$DEFINE NO_GENERIC_STACK_CHECK}

{ include system independent routines }
{$I system.inc}

//procedure __EnterDebugger; cdecl; external 'clib' name 'EnterDebugger';


procedure PASCALMAIN;external name 'PASCALMAIN';
procedure fpc_do_exit;external name 'FPC_DO_EXIT';


{*****************************************************************************
                         Startup
*****************************************************************************}


PROCEDURE nlm_main (_ArgC : LONGINT; _ArgV : ppchar); CDECL; [public,alias: '_nlm_main'];
BEGIN
  // Initialize of BSS now done in nwpre
  ArgC := _ArgC;
  ArgV := _ArgV;
  fpc_threadvar_relocate_proc := nil;
  PASCALMAIN;
END;

var dottext : ptruint; external name '__text_start__';

function NWGetCodeStart : pointer;  // needed for lineinfo
begin
  NWGetCodeStart := @dottext;
end;


{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

var SigTermHandlerActive : boolean;

Procedure system_exit;
begin
  if TerminatingThreadID <> 0 then
    if TerminatingThreadID <> ThreadId then
      if TerminatingThreadID <> _GetThreadID then
      begin
        {$ifdef DEBUG_MT}
        ConsolePrintf ('Terminating Thread %x because halt was called while Thread %x terminates nlm'#13#10,_GetThreadId,TerminatingThreadId);
        {$endif}
        ExitThread (EXIT_THREAD,0);
        // only for the case ExitThread fails
        while true do
          _ThreadSwitchWithDelay;
      end;
  if assigned (CloseAllRemainingSemaphores) then CloseAllRemainingSemaphores;
  if assigned (ReleaseThreadVars) then ReleaseThreadVars;

  {$ifdef autoHeapRelease}
  FreeSbrkMem;            { free memory allocated by heapmanager }
  {$endif}

  if not SigTermHandlerActive then
  begin
    if ExitCode <> 0 Then   { otherwise we dont see runtime-errors }
      _SetAutoScreenDestructionMode (false);

    _exit (ExitCode);
  end;
end;

{*****************************************************************************
                         Stack check code
*****************************************************************************}

const StackErr : boolean = false;

procedure fpc_stackcheck(stack_size:SizeUInt);[public,alias:'FPC_STACKCHECK']; compilerproc;
{
  called when trying to get local stack if the compiler directive $S
  is set this function must preserve all registers

  With a 2048 byte safe area used to write to StdIo without crossing
  the stack boundary
}
begin
  if StackErr then exit;  // avoid recursive calls
  asm
    pusha
  end;
  stackerr := ( _stackavail < stack_size + 2048);
  asm
    popa
  end;
  if not StackErr then exit;
  StackErr := true;
  HandleError (202);
end;
{*****************************************************************************
                              ParamStr/Randomize
*****************************************************************************}

{ number of args }
function paramcount : longint;
begin
  paramcount := argc - 1;
end;

{ argument number l }
function paramstr(l : longint) : string;
begin
  if (l>=0) and (l+1<=argc) then
  begin
    paramstr:=strpas(argv[l]);
    if l = 0 then  // fix nlm path
    begin
      DoDirSeparators(paramstr);
    end;
  end else
   paramstr:='';
end;

{ set randseed to a new pseudo random value }
procedure randomize;
begin
  randseed := _time (NIL);
end;



{*****************************************************************************
                             Thread Handling
*****************************************************************************}

{ if return-value is <> 0, netware shows the message
  Unload Anyway ?
  To Disable unload at all, SetNLMDontUnloadFlag can be used on
  Netware >= 4.0 }
function CheckFunction : longint; CDECL; [public,alias: 'FPC_NW_CHECKFUNCTION'];
var oldTG:longint;
    oldPtr: pointer;
begin
  if assigned (NetwareCheckFunction) then
  begin
    { this function is called without clib context, to allow clib
      calls, we set the thread group id before calling the
      user-function }
    oldTG := _SetThreadGroupID (NetwareMainThreadGroupID);
    { to allow use of threadvars, we simply set the threadvar-memory
      from the main thread }
    if assigned (SetThreadDataAreaPtr) then
      oldPtr := SetThreadDataAreaPtr (NIL);  { nil means main threadvars }
    result := 0;
    NetwareCheckFunction (result);
    if assigned (SetThreadDataAreaPtr) then
      SetThreadDataAreaPtr (oldPtr);

    _SetThreadGroupID (oldTG);
  end else
    result := 0;
end;



{$ifdef StdErrToConsole}
var ConsoleBuff : array [0..512] of char;

Function ConsoleWrite(Var F: TextRec): Integer;
var
  i : longint;
Begin
  if F.BufPos>0 then
  begin
     if F.BufPos>sizeof(ConsoleBuff)-1 then
       i:=sizeof(ConsoleBuff)-1
     else
       i:=F.BufPos;
     Move(F.BufPtr^,ConsoleBuff,i);
     ConsoleBuff[i] := #0;
     ConsolePrintf(@ConsoleBuff[0]);
  end;
  F.BufPos:=0;
  ConsoleWrite := 0;
End;


Function ConsoleClose(Var F: TextRec): Integer;
begin
  ConsoleClose:=0;
end;


Function ConsoleOpen(Var F: TextRec): Integer;
Begin
  TextRec(F).InOutFunc:=@ConsoleWrite;
  TextRec(F).FlushFunc:=@ConsoleWrite;
  TextRec(F).CloseFunc:=@ConsoleClose;
  ConsoleOpen:=0;
End;


procedure AssignStdErrConsole(Var T: Text);
begin
  Assign(T,'');
  TextRec(T).OpenFunc:=@ConsoleOpen;
  Rewrite(T);
end;
{$endif}


{ this will be called if the nlm is unloaded. It will NOT be
  called if the program exits i.e. with halt.
  Halt (or _exit) can not be called from this callback procedure }
procedure TermSigHandler (Sig:longint); CDecl;
var oldTG : longint;
    oldPtr: pointer;
    err   : longint;
    current_exit : procedure;
    ThreadName   : array [0..20] of char;
    HadExitProc  : boolean;
    Count        : longint;
begin

  oldTG := _SetThreadGroupID (NetwareMainThreadGroupID); { this is only needed for nw 3.11 }

  { _GetThreadDataAreaPtr will not be valid because the signal
    handler is called by netware with a differnt thread. To avoid
    problems in the exit routines, we set the data of the main thread
    here }
  if assigned (SetThreadDataAreaPtr) then
    oldPtr := SetThreadDataAreaPtr (NIL);  { nil means main thread }

  {this signal handler is called within the console command
   thread, the main thread is still running. Via NetwareUnloadProc
   running threads may terminate itself}
  TerminatingThreadID := _GetThreadID;
  {$ifdef DEBUG_MT}
  ConsolePrintf (#13'TermSigHandler Called, MainThread:%x, OurThread: %x'#13#10,ThreadId,TerminatingThreadId);
  if NetwareUnloadProc <> nil then
    ConsolePrintf (#13'Calling NetwareUnloadProcs'#13#10);
  {$endif}
  HadExitProc := false;
  {we need to finalize winock to release threads
   waiting on a blocking socket call. If that thread
   calls halt, we have to avoid that unit finalization
   is called by that thread because we are doing it
   here

   like the old exitProc, mainly to allow winsock to release threads
   blocking in a winsock calls }
  while NetwareUnloadProc<>nil Do
  Begin
    InOutRes:=0;
    current_exit:=tProcedure(NetwareUnloadProc);
    NetwareUnloadProc:=nil;
    current_exit();
    _ThreadSwitchWithDelay;
    hadExitProc := true;
  End;

  err := 0;
  if hadExitProc then
  begin  {give the main thread a little bit of time to terminate}
    count := 0;
    repeat
      err := _GetThreadName(ThreadID,ThreadName);
      if err = 0 then _Delay (200);
      inc(count);
    until (err <> 0) or (count > 100);  {about 20 seconds}
    {$ifdef DEBUG_MT}
    if err = 0 then
      ConsolePrintf (#13,'Main Thread not terminated'#13#10)
    else
      ConsolePrintf (#13'Main Thread has ended'#13#10);
    {$endif}
  end;

  if err = 0 then
  {$ifdef DEBUG_MT}
  begin
    err := _SuspendThread(ThreadId);
    ConsolePrintf (#13'SuspendThread(%x) returned %d'#13#10,ThreadId,err);
  end;
  {$else}
  _SuspendThread(ThreadId);
  {$endif}
  _ThreadSwitchWithDelay;

  {$ifdef DEBUG_MT}
  ConsolePrintf (#13'Calling do_exit'#13#10);
  {$endif}
  SigTermHandlerActive := true;  { to avoid that system_exit calls _exit }
  internal_do_exit;              { calls finalize units }
  if assigned (SetThreadDataAreaPtr) then
    SetThreadDataAreaPtr (oldPtr);
  _SetThreadGroupID (oldTG);
  {$ifdef DEBUG_MT}
  ConsolePrintf (#13'TermSigHandler: all done'#13#10);
  {$endif}
end;


procedure SysInitStdIO;
begin
{ Setup stdin, stdout and stderr }
  StdInputHandle := _fileno (LONGINT (_GetStdIn^));    // GetStd** returns **FILE
  StdOutputHandle:= _fileno (LONGINT (_GetStdOut^));
  StdErrorHandle := _fileno (LONGINT (_GetStdErr^));

  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);

  {$ifdef StdErrToConsole}
  AssignStdErrConsole(StdErr);
  AssignStdErrConsole(ErrOutput);
  {$else}
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
  OpenStdIO(ErrOutput,fmOutput,StdErrorHandle);
  {$endif}
end;

function GetProcessID: SizeUInt;
begin
 GetProcessID := SizeUInt (GetNlmHandle);
end;


function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;
{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

Begin
  StackLength := CheckInitialStkLen(initialstklen);
  StackBottom := SPtr - StackLength;
  SigTermHandlerActive := false;
  NetwareCheckFunction := nil;
  NetwareMainThreadGroupID := _GetThreadGroupID;

  _Signal (_SIGTERM, @TermSigHandler);

  {$ifdef useLongNamespaceByDefault}
  if _getenv ('FPC_DISABLE_LONG_NAMESPACE') = nil then
  begin
    if _SetCurrentNameSpace (NW_NS_LONG) <> 255 then
    begin
      if _SetTargetNamespace (NW_NS_LONG) <> 255 then
        LFNSupport := true
      else
        _SetCurrentNameSpace (NW_NS_DOS);
    end;
  end;
  {$endif useLongNamespaceByDefault}

{ Setup heap }
  InitHeap;
  SysInitExceptions;

{ Reset IO Error }
  InOutRes:=0;

  ThreadID := _GetThreadID;
  {$ifdef DEBUG_MT}
  ConsolePrintf (#13'Start system, ThreadID: %x'#13#10,ThreadID);
  {$endif}

  initunicodestringmanager;
  SysInitStdIO;

{Delphi Compatible}
  IsConsole := TRUE;
  ExitCode  := 0;
  InitSystemThreads;
  InitSystemDynLibs;
End.
