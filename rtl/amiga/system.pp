{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2004-2006 by Karoly Balogh

    System unit for AmigaOS 3.x/4.x

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
{$define FPC_ANSI_TEXTFILEREC}

{$if defined(AMIGA_V1_0_ONLY) or defined(AMIGA_V1_2_ONLY)}
{$define AMIGA_LEGACY}
{$endif}

{$ifdef AMIGA_LEGACY}
{$.define FPC_AMIGA_USE_TINYHEAP}
{$endif}

{$ifdef FPC_AMIGA_USE_TINYHEAP}
{$define HAS_MEMORYMANAGER}
{$endif FPC_AMIGA_USE_TINYHEAP}

{$I systemh.inc}
{$ifdef FPC_AMIGA_USE_TINYHEAP}
{$i tnyheaph.inc}
{$endif FPC_AMIGA_USE_TINYHEAP}
{$I osdebugh.inc}

{$if defined(cpum68k) and defined(fpusoft)}
{$define fpc_softfpu_interface}
{$i softfpu.pp}
{$undef fpc_softfpu_interface}
{$endif defined(cpum68k) and defined(fpusoft)}

const
{$if defined(AMIGA_V1_0_ONLY)}
  AMIGA_OS_MINVERSION = 0;
{$else}
{$if defined(AMIGA_V1_2_ONLY)}
  AMIGA_OS_MINVERSION = 33;
{$else}
{$if defined(AMIGA_V2_0_ONLY)}
  AMIGA_OS_MINVERSION = 37;
{$else}
{$ifndef cpupowerpc}
  AMIGA_OS_MINVERSION = 39;
{$else}
  AMIGA_OS_MINVERSION = 50;
{$endif}
{$endif}
{$endif}
{$endif}

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
  UnusedHandle    : LongInt = -1;
  StdInputHandle  : LongInt = 0;
  StdOutputHandle : LongInt = 0;
  StdErrorHandle  : LongInt = 0;

  FileNameCaseSensitive : Boolean = False;
  FileNameCasePreserving: boolean = True;
  CtrlZMarksEOF: boolean = false; (* #26 not considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsLF;

  BreakOn : Boolean = True;

{ FIX ME: remove the kludge required by amunits package, which needs a huge rework }
var
  AOS_ExecBase   : Pointer; external name '_ExecBase';
  _ExecBase: Pointer; external name '_ExecBase'; { amunits compatibility kludge }
  AOS_DOSBase    : Pointer; public name '_DOSBase'; { the "public" part is amunits compatibility kludge }
  _DOSBase: Pointer; external name '_DOSBase'; { amunits compatibility kludge }
  AOS_UtilityBase: Pointer; public name '_UtilityBase'; { the "public" part is amunits compatibility kludge }
  _UtilityBase: Pointer; external name '_UtilityBase'; { amunits compatibility kludge }
  AOS_IntuitionBase: Pointer; public name '_IntuitionBase'; { amunits compatibility kludge }
  _IntuitionBase: Pointer; external name '_IntuitionBase'; { amunits compatibility kludge }

{$IFDEF AMIGAOS4}
{$WARNING iExec, iDOS and iUtility should be typed pointer later}
var
  IExec : Pointer; external name '_IExec';
  IDOS : Pointer;
  IUtility : Pointer;
{$ENDIF}

  ASYS_heapPool : Pointer; { pointer for the OS pool for growing the heap }
  ASYS_heapSemaphore: Pointer; { 68k OS from 3.x has no MEMF_SEM_PROTECTED for pools, have to do it ourselves }
  ASYS_fileSemaphore: Pointer; { mutex semaphore for filelist access arbitration }
  ASYS_origDir  : LongInt; { original directory on startup }
  AOS_wbMsg    : Pointer; public name '_WBenchMsg'; { the "public" part is amunits compatibility kludge }
  _WBenchMsg   : Pointer; external name '_WBenchMsg'; { amunits compatibility kludge }
  AOS_ConName  : PChar ='CON:10/30/620/100/FPC Console Output/AUTO/CLOSE/WAIT';
  AOS_ConHandle: LongInt;

  argc: LongInt;
  argv: PPChar;
  envp: PPChar;


implementation

{$if defined(cpum68k) and defined(fpusoft)}

{$define fpc_softfpu_implementation}
{$define softfpu_compiler_mul32to64}
{$define softfpu_inline}
{$i softfpu.pp}
{$undef fpc_softfpu_implementation}

{ we get these functions and types from the softfpu code }
{$define FPC_SYSTEM_HAS_float64}
{$define FPC_SYSTEM_HAS_float32}
{$define FPC_SYSTEM_HAS_flag}
{$define FPC_SYSTEM_HAS_extractFloat64Frac0}
{$define FPC_SYSTEM_HAS_extractFloat64Frac1}
{$define FPC_SYSTEM_HAS_extractFloat64Exp}
{$define FPC_SYSTEM_HAS_extractFloat64Sign}
{$define FPC_SYSTEM_HAS_ExtractFloat32Frac}
{$define FPC_SYSTEM_HAS_extractFloat32Exp}
{$define FPC_SYSTEM_HAS_extractFloat32Sign}
{$endif defined(cpum68k) and defined(fpusoft)}

{$I system.inc}
{$ifdef FPC_AMIGA_USE_TINYHEAP}
{$i tinyheap.inc}
{$endif FPC_AMIGA_USE_TINYHEAP}
{$I osdebug.inc}

{$IFDEF AMIGAOS4}
  // Required to allow opening of utility library interface...
  {$include utilf.inc}
{$ENDIF}


{$IFDEF ASYS_FPC_FILEDEBUG}
{$WARNING Compiling with file debug enabled!}
{$ENDIF}

{$IFDEF ASYS_FPC_MEMDEBUG}
{$WARNING Compiling with memory debug enabled!}
{$ENDIF}

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

procedure haltproc(e:longint);cdecl;external name '_haltproc';

procedure System_exit;
var
  oldDirLock: LongInt;
begin
  { Dispose the thread init/exit chains }
  CleanupThreadProcChain(threadInitProcList);
  CleanupThreadProcChain(threadExitProcList);

  { We must remove the CTRL-C FLAG here because halt }
  { may call I/O routines, which in turn might call  }
  { halt, so a recursive stack crash                 }
  if BreakOn then begin
    if (SetSignal(0,0) and SIGBREAKF_CTRL_C)<>0 then
      SetSignal(0,SIGBREAKF_CTRL_C);
  end;

  { Closing opened files }
  CloseList(ASYS_fileList);

  { Changing back to original directory if changed }
  if ASYS_origDir<>0 then begin
    oldDirLock:=CurrentDir(ASYS_origDir);
    { unlock our lock if its safe, so we won't leak the lock }
    if (oldDirLock<>0) and (oldDirLock<>ASYS_origDir) then
      Unlock(oldDirLock);
  end;

{$IFDEF AMIGAOS4}
  if iDOS<>nil then DropInterface(iDOS);
  if iUtility<>nil then DropInterface(iUtility);
{$ENDIF}

  if AOS_IntuitionBase<>nil then CloseLibrary(AOS_IntuitionBase); { amunits kludge }
  if AOS_UtilityBase<>nil then CloseLibrary(AOS_UtilityBase);
  if AOS_DOSBase<>nil then CloseLibrary(AOS_DOSBase);
  if ASYS_heapPool<>nil then DeletePool(ASYS_heapPool);

  { If in Workbench mode, replying WBMsg }
  if AOS_wbMsg<>nil then begin
    Forbid;
    ReplyMsg(AOS_wbMsg);
  end;

  haltproc(ExitCode);
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
procedure randomize;
var tmpTime: TDateStamp;
begin
  DateStamp(@tmpTime);
  randseed:=tmpTime.ds_tick;
end;


{ AmigaOS specific startup }
procedure SysInitAmigaOS;
var self: PProcess;
begin
  self:=PProcess(FindTask(nil));
  if self^.pr_CLI=0 then begin
    { if we're running from Ambient/Workbench, we catch its message }
    WaitPort(@self^.pr_MsgPort);
    AOS_wbMsg:=GetMsg(@self^.pr_MsgPort);
  end;

  AOS_DOSBase:=OpenLibrary('dos.library',AMIGA_OS_MINVERSION);
  if AOS_DOSBase=nil then Halt(1);
{$ifndef AMIGA_LEGACY}
  AOS_UtilityBase:=OpenLibrary('utility.library',AMIGA_OS_MINVERSION);
  if AOS_UtilityBase=nil then Halt(1);
{$endif}
  AOS_IntuitionBase:=OpenLibrary('intuition.library',AMIGA_OS_MINVERSION); { amunits support kludge }
  if AOS_IntuitionBase=nil then Halt(1);

{$IFDEF AMIGAOS4}
  { Open main interfaces on OS4 }
  iDOS := GetInterface(AOS_DOSBase,'main',1,nil);
  iUtility := GetInterface(AOS_UtilityBase,'main',1,nil);
{$ENDIF}

  { Creating the memory pool for growing heap }
  ASYS_heapPool:=CreatePool(MEMF_ANY,growheapsize2,growheapsize1);
  if ASYS_heapPool=nil then Halt(1);
  ASYS_heapSemaphore:=AllocPooled(ASYS_heapPool,sizeof(TSignalSemaphore));
  if ASYS_heapSemaphore = nil then Halt(1);
  InitSemaphore(ASYS_heapSemaphore);

  { Initialize semaphore for filelist access arbitration }
  ASYS_fileSemaphore:=AllocPooled(ASYS_heapPool,sizeof(TSignalSemaphore));
  if ASYS_fileSemaphore = nil then Halt(1);
  InitSemaphore(ASYS_fileSemaphore);

  if AOS_wbMsg=nil then begin
    StdInputHandle:=dosInput;
    StdOutputHandle:=dosOutput;
    StdErrorHandle:=StdOutputHandle;
  end else begin
    AOS_ConHandle:=Open(AOS_ConName,MODE_OLDFILE);
    if AOS_ConHandle<>0 then begin
      StdInputHandle:=AOS_ConHandle;
      StdOutputHandle:=AOS_ConHandle;
      StdErrorHandle:=AOS_ConHandle;
    end else
      Halt(1);
  end;
end;


procedure SysInitStdIO;
begin
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(ErrOutput,fmOutput,StdErrorHandle);
{$ifndef FPC_STDOUT_TRUE_ALIAS}
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
{$endif FPC_STDOUT_TRUE_ALIAS}
end;

function GetProcessID: SizeUInt;
begin
  GetProcessID:=SizeUInt(FindTask(NIL));
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;


begin
  IsConsole := TRUE;
  StackLength := CheckInitialStkLen(InitialStkLen);
  StackBottom := StackTop - StackLength;
{ OS specific startup }
  AOS_wbMsg:=nil;
  ASYS_origDir:=0;
  ASYS_fileList:=nil;
  envp:=nil;
  SysInitAmigaOS;
{ Set up signals handlers }
//  InstallSignals;
{$ifndef FPC_AMIGA_USE_TINYHEAP}
{ Setup heap }
  InitHeap;
{$endif FPC_AMIGA_USE_TINYHEAP}
  SysInitExceptions;
{$ifdef cpum68k}
  fpc_cpucodeinit;
{$endif}
  initunicodestringmanager;
{ Setup stdin, stdout and stderr }
  SysInitStdIO;
{ Reset IO Error }
  InOutRes:=0;
{ Arguments }
  GenerateArgs;
  InitSystemThreads;
{$ifdef FPC_HAS_FEATURE_DYNLIBS}
  InitSystemDynLibs;
{$endif}
end.
