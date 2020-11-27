{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2004-2006 by Karoly Balogh

    AROS conversion
    Copyright (c) 2011 by Marcus Sackrow

    System unit for AROS

    Uses parts of the Free Pascal 1.0.x for Commodore Amiga/68k port
    by Carl Eric Codere and Nils Sjoholm

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit System;

interface

{$define FPC_IS_SYSTEM}

{$I systemh.inc}
{$I osdebugh.inc}

const
  LineEnding = #10;
  LFNSupport = True;
  DirectorySeparator = '/';
  DriveSeparator = ':';
  ExtensionSeparator = '.';
  PathSeparator = ';';
  AllowDirectorySeparators : set of char = ['\','/'];
  AllowDriveSeparators : set of char = [':'];
  maxExitCode = 255;
  MaxPathLen = 256;
  AllFilesMask = '#?';

const
  UnusedHandle    : THandle = 0;
  StdInputHandle  : THandle = 0;
  StdOutputHandle : THandle = 0;
  StdErrorHandle  : THandle = 0;

  FileNameCaseSensitive : Boolean = False;
  FileNameCasePreserving: boolean = True;
  CtrlZMarksEOF: Boolean = false; (* #26 not considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsLF;

  BreakOn : Boolean = True;



var
  AOS_ExecBase   : Pointer; external name '_ExecBase';
  AOS_DOSBase    : Pointer;
  AOS_UtilityBase: Pointer;
  AROS_ThreadLib : Pointer; public name 'AROS_THREADLIB';

  ASYS_heapPool : Pointer; { pointer for the OS pool for growing the heap }
  ASYS_fileSemaphore: Pointer; { mutex semaphore for filelist access arbitration }
  ASYS_origDir  : LongInt; { original directory on startup }
  AOS_wbMsg    : Pointer;
  AOS_ConName  : PChar ='CON:10/30/620/100/FPC Console Output/AUTO/CLOSE/WAIT';
  AOS_ConHandle: THandle;

  SysDebugBase: Pointer = nil;

  argc: LongInt;
  argv: PPChar;
  envp: PPChar;
  killed : Boolean = False;

function GetLibAdress(Base: Pointer; Offset: LongInt): Pointer;
procedure Debug(s: string);
procedure Debugln(s: string);
procedure EnableBackTraceStr;

implementation

{$I system.inc}
{$I osdebug.inc}
type
    PWBArg = ^TWBArg;
    TWBArg = record
        wa_Lock         : LongInt;      { a lock descriptor }
        wa_Name         : PChar;       { a string relative to that lock }
    end;

    WBArgList = array[1..MaxInt] of TWBArg; { Only 1..smNumArgs are valid }
    PWBArgList = ^WBArgList;


    PWBStartup = ^TWBStartup;
    TWBStartup = record
        sm_Message      : TMessage;      { a standard message structure }
        sm_Process      : Pointer;   { the process descriptor for you }
        sm_Segment      : Pointer;     { a descriptor for your code }
        sm_NumArgs      : Longint;      { the number of elements in ArgList }
        sm_ToolWindow   : Pointer;       { description of window }
        sm_ArgList      : PWBArgList; { the arguments themselves }
    end;

{*****************************************************************************
                       Misc. System Dependent Functions
*****************************************************************************}

procedure haltproc(e:longint); cdecl; external name '_haltproc';

procedure System_exit;
var
  oldDirLock: LongInt;
begin
  if Killed then
    Exit;
  Killed := True;

  { Dispose the thread init/exit chains }
  CleanupThreadProcChain(threadInitProcList);
  CleanupThreadProcChain(threadExitProcList);

  { Closing opened files }
  CloseList(ASYS_fileList);
  { Changing back to original directory if changed }
  if ASYS_OrigDir <> 0 then begin
    oldDirLock:=CurrentDir(ASYS_origDir);
    { unlock our lock if its safe, so we won't leak the lock }
    if (oldDirLock<>0) and (oldDirLock<>ASYS_origDir) then
      Unlock(oldDirLock);
  end;
  // debug lib
  if SysDebugBase <> nil then
    CloseLibrary(SysDebugBase);
  SysDebugBase := nil;
  // utility
  if AOS_UtilityBase <> nil then
    CloseLibrary(AOS_UtilityBase);
  // Heap
  if ASYS_heapPool <> nil then
    DeletePool(ASYS_heapPool);
  AOS_UtilityBase := nil;
  ASYS_HeapPool := nil;
  // dos
  if AOS_DOSBase<>nil then
    CloseLibrary(AOS_DOSBase);
  AOS_DOSBase := nil;
  //
  if AOS_wbMsg <> nil then
  begin
    // forbid -> Amiga RKM Libraries Manual
    Forbid();
    // Reply WBStartupMessage
    ReplyMsg(AOS_wbMsg);
  end;
  //
  HaltProc(ExitCode);
end;

{*****************************************************************************
                          Parameterhandling
                       as include in amicommon
*****************************************************************************}

{$I paramhandling.inc}

{*****************************************************************************
                             Randomize
*****************************************************************************}

{ set randseed to a new pseudo random value }
procedure Randomize;
var
  tmpTime: TDateStamp;
begin
  DateStamp(@tmpTime);
  randseed := tmpTime.ds_tick;
end;




{ AmigaOS specific startup }
procedure SysInitAmigaOS;
var
  self: PProcess;
begin
  self := PProcess(FindTask(nil));
  if self^.pr_CLI = 0 then begin
    { if we're running from Ambient/Workbench, we catch its message }
    WaitPort(@self^.pr_MsgPort);
    AOS_wbMsg:=GetMsg(@self^.pr_MsgPort);
  end;

  AOS_DOSBase := OpenLibrary('dos.library', 0);
  if AOS_DOSBase = nil then
    Halt(1);
  AOS_UtilityBase := OpenLibrary('utility.library', 0);
  if AOS_UtilityBase = nil then
    Halt(1);

  { Creating the memory pool for growing heap }
  ASYS_heapPool := CreatePool(MEMF_ANY or MEMF_SEM_PROTECTED, growheapsize2, growheapsize1);
  if ASYS_heapPool = nil then
    Halt(1);

  { Initialize semaphore for filelist access arbitration }
  ASYS_fileSemaphore:=AllocPooled(ASYS_heapPool,sizeof(TSignalSemaphore));
  if ASYS_fileSemaphore = nil then
    Halt(1);
  InitSemaphore(ASYS_fileSemaphore);

  if AOS_wbMsg = nil then begin
    StdInputHandle := THandle(dosInput);
    StdOutputHandle := THandle(dosOutput);
    StdErrorHandle := THandle(DosError1);
  end else begin
    AOS_ConHandle := Open(AOS_ConName, MODE_OLDFILE);
    if AOS_ConHandle <> 0 then begin
      StdInputHandle := AOS_ConHandle;
      StdOutputHandle := AOS_ConHandle;
      StdErrorHandle := AOS_ConHandle;
    end else
      Halt(1);
  end;
end;

function AROSBackTraceStr(Addr: CodePointer): ShortString;
const
  DL_Dummy = TAG_USER + $03e00000;
  DL_ModuleName = DL_Dummy + 1;
  DL_SymbolName = DL_Dummy + 7;
var
  SymName, ModName: PChar;
  Tags: array[0..5] of PtrUInt;
  s: AnsiString;
  Res: AnsiString;
begin
  if Assigned(SysDebugBase) then
  begin
    ModName := nil;
    SymName := nil;
    Tags[0] := DL_Modulename;
    Tags[1] := PtrUInt(@ModName);
    Tags[2] := DL_SymbolName;
    Tags[3] := PtrUInt(@SymName);
    Tags[4] := 0;
    Tags[5] := 0;
    DecodeLocation(Addr, @Tags[0]);
    s := '-';
    if not Assigned(ModName) then
      ModName := @S[1];
    if not Assigned(SymName) then
      SymName := @S[1];
    Res := '  $' + HexStr(Addr) + ' ' + ModName  + ' ' + SymName;
    AROSBackTraceStr := Copy(Res, 1, 254);
  end
  else
  begin
    AROSBackTraceStr := '  $' + HexStr(Addr) + ' - ';
  end;
end;

procedure EnableBackTraceStr;
begin
  if not Assigned(SysDebugBase) then
  begin
    SysDebugBase := OpenLibrary('debug.library', 0);
    if Assigned(SysDebugBase) then
      BackTraceStrFunc := @AROSBackTraceStr;
  end;
end;


procedure SysInitStdIO;
begin
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
end;

function GetProcessID: SizeUInt;
begin
  GetProcessID := SizeUInt(FindTask(NIL));
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

begin
  IsConsole := TRUE;
  SysResetFPU;
  if not (IsLibrary) then
    SysInitFPU;
  StackLength := CheckInitialStkLen(InitialStkLen);
  StackBottom := Sptr - StackLength;
{ OS specific startup }
  AOS_wbMsg := nil;
  ASYS_origDir := 0;
  ASYS_fileList := nil;
  envp := nil;
  SysInitAmigaOS;
{ Set up signals handlers }
  //InstallSignals;
{ Setup heap }
  InitHeap;
  SysInitExceptions;
  initunicodestringmanager;
{ Setup stdin, stdout and stderr }
  SysInitStdIO;
{ Reset IO Error }
  InOutRes:=0;
  { Arguments }
  GenerateArgs;
  InitSystemThreads;
  InitSystemDynLibs;
end.
