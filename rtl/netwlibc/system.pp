{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2004 by the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

    System.pp for Netware libc environment
 **********************************************************************}
{ no stack check in system }
{$S-}
unit System;

interface

{$define FPC_IS_SYSTEM}
{$define netware}
{$define netware_libc}

{$define StdErrToConsole}
{$define autoHeapRelease}
{$define IOpossix}
{$define DisableArrayOfConst}
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
 maxExitCode = $ffff;
 MaxPathLen = 256;
 AllFilesMask = '*';

CONST
  { Default filehandles }
   UnusedHandle    : THandle = -1;
   StdInputHandle  : THandle = 0;
   StdOutputHandle : THandle = 0;
   StdErrorHandle  : THandle = 0;

   FileNameCaseSensitive : boolean = false;
   FileNameCasePreserving: boolean = true;
   CtrlZMarksEOF: boolean = false; (* #26 not considered as end of file *)

   sLineBreak = LineEnding;
   DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

type
   TNWCheckFunction = procedure (var code : longint);
   TDLL_Process_Entry_Hook = function (dllparam : longint) : longbool;
   TDLL_Entry_Hook = procedure (dllparam : longint);

VAR
   ArgC                : INTEGER;
   ArgV                : ppchar;
   NetwareCheckFunction: TNWCheckFunction;
   NWLoggerScreen      : pointer = nil;

const
  Dll_Process_Attach_Hook : TDLL_Process_Entry_Hook = nil;
  Dll_Process_Detach_Hook : TDLL_Entry_Hook = nil;
  Dll_Thread_Attach_Hook  : TDLL_Entry_Hook = nil;
  Dll_Thread_Detach_Hook  : TDLL_Entry_Hook = nil;
  NetwareUnloadProc       : pointer = nil;  {like exitProc but for nlm unload only}
  envp : ppchar = nil;



type
  //TSysCloseAllRemainingSemaphores = procedure;
  TSysReleaseThreadVars = procedure;
  TSysSetThreadDataAreaPtr = function (newPtr:pointer):pointer;

procedure NWSysSetThreadFunctions (atv:TSysReleaseThreadVars;
                                   rtv:TSysReleaseThreadVars;
                                   stdata:TSysSetThreadDataAreaPtr);


procedure _ConsolePrintf (s :shortstring);
procedure _ConsolePrintf (FormatStr : PCHAR; Param : LONGINT);
procedure _ConsolePrintf (FormatStr : PCHAR; Param : pchar);
procedure _ConsolePrintf (FormatStr : PCHAR; P1,P2 : LONGINT);
procedure _ConsolePrintf (FormatStr : PCHAR; P1,P2,P3 : LONGINT);
procedure _ConsolePrintf (FormatStr : PCHAR);
procedure __EnterDebugger;cdecl;external '!netware' name 'EnterDebugger';

function NWGetCodeStart : pointer;  // needed for Lineinfo
function NWGetCodeLength : dword;
function NWGetDataStart : pointer;
function NWGetDataLength : dword;

implementation
{ Indicate that stack checking is taken care by OS}
{$DEFINE NO_GENERIC_STACK_CHECK}

{ include system independent routines }
{$I system.inc}


procedure PASCALMAIN;external name 'PASCALMAIN';
procedure fpc_do_exit;external name 'FPC_DO_EXIT';


{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

var SigTermHandlerActive : boolean;

Procedure system_exit;
begin
  if TerminatingThreadID <> 0 then
    if TerminatingThreadID <> ThreadId then
      if TerminatingThreadID <> dword(pthread_self) then
      begin
        {$ifdef DEBUG_MT}
        _ConsolePrintf ('Terminating Thread %x because halt was called while Thread %x terminates nlm'#13#10,dword(pthread_self),TerminatingThreadId);
        {$endif}
        pthread_exit (nil);
        // only for the case ExitThread fails
        while true do
          NXThreadYield;
      end;
  if assigned (ReleaseThreadVars) then ReleaseThreadVars;

  {$ifdef autoHeapRelease}
  FreeSbrkMem;              { free memory allocated by heapmanager }
  {$endif}

  if not SigTermHandlerActive then
  begin
    if Erroraddr <> nil then   { otherwise we dont see runtime-errors }
      SetScreenMode (0);

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

  With a 5k byte safe area used to write to StdIo and some libc
  functions without crossing the stack boundary
}
begin
  if StackErr then exit;  // avoid recursive calls
  asm
    pusha
  end;
  stackerr := (stackavail < stack_size + 5120);   // we really need that much, at least on nw6.5
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
  randseed := time (NIL);
end;


{*****************************************************************************
                             Thread Handling
*****************************************************************************}

{ if return-value is <> 0, netware shows the message
  Unload Anyway ?
  To Disable unload at all, SetNLMDontUnloadFlag can be used on
  Netware >= 4.0 }

function CheckFunction : longint; CDECL; [public,alias: '_NonAppCheckUnload'];
var oldPtr : pointer;
begin
  //_ConsolePrintf ('CheckFunction'#13#10);
  if assigned (NetwareCheckFunction) then
  begin
    if assigned (SetThreadDataAreaPtr) then
      oldPtr := SetThreadDataAreaPtr (NIL);  { nil means main thread }

    result := 0;
    NetwareCheckFunction (result);

    if assigned (SetThreadDataAreaPtr) then
      SetThreadDataAreaPtr (oldPtr);

  end else
    result := 0;
end;


procedure _ConsolePrintf (s : shortstring);
begin
  if length(s) > 254 then
    byte(s[0]) := 254;
  s := s + #0;
  _ConsolePrintf (@s[1]);
end;

procedure _ConsolePrintf (FormatStr : PCHAR);
begin
  if NWLoggerScreen = nil then
    NWLoggerScreen := getnetwarelogger;
  if NWLoggerScreen <> nil then
    screenprintf (NWLoggerScreen,FormatStr);
end;

procedure _ConsolePrintf (FormatStr : PCHAR; Param : LONGINT);
begin
  if NWLoggerScreen = nil then
    NWLoggerScreen := getnetwarelogger;
  if NWLoggerScreen <> nil then
    screenprintf (NWLoggerScreen,FormatStr,Param);
end;

procedure _ConsolePrintf (FormatStr : PCHAR; Param : pchar);
begin
  _ConsolePrintf (FormatStr,longint(Param));
end;

procedure _ConsolePrintf (FormatStr : PCHAR; P1,P2 : LONGINT);
begin
  if NWLoggerScreen = nil then
    NWLoggerScreen := getnetwarelogger;
  if NWLoggerScreen <> nil then
    screenprintf (NWLoggerScreen,FormatStr,P1,P2);
end;

procedure _ConsolePrintf (FormatStr : PCHAR; P1,P2,P3 : LONGINT);
begin
  if NWLoggerScreen = nil then
    NWLoggerScreen := getnetwarelogger;
  if NWLoggerScreen <> nil then
    screenprintf (NWLoggerScreen,FormatStr,P1,P2,P3);
end;

var NWUts : Tutsname;

procedure getCodeAddresses;
begin
  if Fpuname(NWUts) < 0 then
    FillChar(NWuts,sizeof(NWUts),0);
end;

function NWGetCodeStart : pointer;
begin
  NWGetCodeStart := NWUts.codeoffset;
  NXThreadYield;
end;

function NWGetCodeLength : dword;
begin
  NWGetCodeLength := NWUts.codelength;
  NXThreadYield;
end;

function NWGetDataStart : pointer;
begin
  NWGetDataStart := NWUts.dataoffset;
  NXThreadYield;
end;

function NWGetDataLength : dword;
begin
  NWGetDataLength := NWUts.datalength;
  NXThreadYield;
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
     screenprintf (NWLoggerScreen,@ConsoleBuff);
  end;
  F.BufPos:=0;
  ConsoleWrite := 0;
  NXThreadYield;
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


function GetProcessID: SizeUInt;
begin
 GetProcessID := SizeUInt (getnlmhandle);
end;


{ this will be called if the nlm is unloaded. It will NOT be
  called if the program exits i.e. with halt.
  Halt (or _exit) can not be called from this callback procedure }
procedure TermSigHandler (Sig:longint); CDecl;
var oldPtr : pointer;
    current_exit : procedure;
begin
  { Threadvar Pointer will not be valid because the signal
    handler is called by netware with a differnt thread. To avoid
    problems in the exit routines, we set the data of the main thread
    here }
  if assigned (SetThreadDataAreaPtr) then
    oldPtr := SetThreadDataAreaPtr (NIL);  { nil means main thread }

  TerminatingThreadID := dword(pthread_self);

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
    NXThreadYield;
    //hadExitProc := true;
  End;


  SigTermHandlerActive := true;  { to avoid that system_exit calls _exit }
  do_exit;                       { calls finalize units }
  if assigned (SetThreadDataAreaPtr) then
    SetThreadDataAreaPtr (oldPtr);
end;


procedure SysInitStdIO;
begin
{ Setup stdin, stdout and stderr }
  {$ifdef IOpossix}
  StdInputHandle := THandle (fileno (___stdin^));    // GetStd** returns **FILE !
  StdOutputHandle:= THandle (fileno (___stdout^));
  StdErrorHandle := THandle (fileno (___stderr^));
  {$else}
  StdInputHandle := THandle (___stdin^);    // GetStd** returns **FILE !
  StdOutputHandle:= THandle (___stdout^);
  StdErrorHandle := THandle (___stderr^);
  {$endif}

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

// this is called by main.as, setup args and call PASCALMAIN
procedure nlm_main (_ArgC : LONGINT; _ArgV : ppchar); cdecl; [public,alias: '_FPC_NLM_Entry'];
BEGIN
  ArgC := _ArgC;
  ArgV := _ArgV;
  isLibrary := false;
  PASCALMAIN;
  do_exit;    // currently not needed
END;

function _DLLMain (hInstDLL:pointer; fdwReason:dword; DLLParam:longint):longbool; cdecl;
[public, alias : '_FPC_DLL_Entry'];
var res : longbool;
begin
  {$ifdef DEBUG_MT}
  _ConsolePrintf ('_FPC_DLL_Entry called');
  {$endif}
  _DLLMain := false;
  isLibrary := true;
  case fdwReason of
    DLL_ACTUAL_DLLMAIN  : _DLLMain := true;
    DLL_NLM_STARTUP     : begin
                            //_ConsolePrintf ('DLL_NLM_STARTUP');
                            if assigned(Dll_Process_Attach_Hook) then
                            begin
                              res:=Dll_Process_Attach_Hook(DllParam);
                              if not res then
                                exit(false);
                            end;
                            PASCALMAIN;
                            _DLLMain := true;
                          end;
    DLL_NLM_SHUTDOWN    : begin
                            //_ConsolePrintf ('DLL_NLM_SHUTDOWN');
                            TermSigHandler(0);
                            _DLLMain := true;
                          end;
     { standard DllMain() messages...  }
    DLL_THREAD_ATTACH,
    DLL_PROCESS_ATTACH  : begin
                            //__ConsolePrintf ('DLL_PROCESS/THREAD_ATTACH');
                            if assigned(AllocateThreadVars) then
                              AllocateThreadVars;
                            if assigned(Dll_Thread_Attach_Hook) then
                              Dll_Thread_Attach_Hook(DllParam);

                            _DLLMain := true;
                          end;
    DLL_THREAD_DETACH,
    DLL_PROCESS_DETACH  : begin
                            //__ConsolePrintf ('DLL_PROCESS/THREAD_DETACH');
                            if assigned(Dll_Thread_Detach_Hook) then
                              Dll_Thread_Detach_Hook(DllParam);
                            if assigned(ReleaseThreadVars) then
                              ReleaseThreadVars;
                            _DLLMain := true;
                          end;
  end;
end;


function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;
{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

Begin
  getCodeAddresses;
  StackLength := CheckInitialStkLen(initialStkLen);
  StackBottom := SPtr - StackLength;
  SigTermHandlerActive := false;
  NetwareCheckFunction := nil;
  {$ifdef StdErrToConsole}
  NWLoggerScreen := getnetwarelogger;
  {$endif}
  CheckFunction;  // avoid check function to be removed by the linker

  envp := ____environ^;
  NLMHandle := getnlmhandle;
  { allocate resource tags to see what kind of memory i forgot to release }
  HeapAllocResourceTag :=
    AllocateResourceTag(NLMHandle,'Heap Memory',AllocSignature);
  {$ifdef autoHeapRelease}
  HeapListAllocResourceTag :=
    AllocateResourceTag(NLMHandle,'Heap Memory List',AllocSignature);
  {$endif}
  FpSignal (SIGTERM, @TermSigHandler);

{ Setup heap }
  InitHeap;
  SysInitExceptions;

{ Reset IO Error }
  InOutRes:=0;

  ThreadID := dword(pthread_self);

  initunicodestringmanager;
  SysInitStdIO;

{Delphi Compatible}
  IsConsole := TRUE;
  ExitCode  := 0;
  InitSystemThreads;
  InitSystemDynLibs;
End.
