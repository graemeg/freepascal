{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2004 by Karoly Balogh for Genesi S.a.r.l.

    System unit for MorphOS/PowerPC

    Uses parts of the Commodore Amiga/68k port by Carl Eric Codere
    and Nils Sjoholm

    MorphOS port was done on a free Pegasos II/G4 machine
    provided by Genesi S.a.r.l. <www.genesi.lu>

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
  UnusedHandle    : LongInt = -1;
  StdInputHandle  : LongInt = 0;
  StdOutputHandle : LongInt = 0;
  StdErrorHandle  : LongInt = 0;

  FileNameCaseSensitive : Boolean = False;
  FileNameCasePreserving: boolean = true;
  CtrlZMarksEOF: boolean = false; { #26 not considered as end of file }

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsLF;

  BreakOn : Boolean = True;


var
  MOS_ExecBase   : Pointer; external name '_ExecBase';
  MOS_DOSBase    : Pointer; public name 'AOS_DOSBASE';
  AOS_DOSBase    : Pointer; external name 'AOS_DOSBASE'; { common Amiga code compatibility kludge }
  MOS_UtilityBase: Pointer;

  ASYS_heapPool : Pointer; { pointer for the OS pool for growing the heap }
  ASYS_fileSemaphore: Pointer; { mutex semaphore for filelist access arbitration }
  ASYS_origDir  : LongInt; { original directory on startup }
  MOS_ambMsg   : Pointer;
  MOS_ConName  : PChar ='CON:10/30/620/100/FPC Console Output/AUTO/CLOSE/WAIT';
  MOS_ConHandle: LongInt;
  AOS_wbMsg: Pointer absolute MOS_ambMsg;  { common Amiga code compatibility kludge }

  argc: LongInt;
  argv: PPChar;
  envp: PPChar;


implementation

{$define FPC_SYSTEM_HAS_STACKTOP}

{$I system.inc}
{$I osdebug.inc}

function StackTop: pointer;
begin
  StackTop:=PETask(FindTask(nil)^.tc_ETask)^.PPCSPUpper;
end;


{$IFDEF MOSFPC_FILEDEBUG}
{$WARNING Compiling with file debug enabled!}
{$ENDIF}

{$IFDEF MOSFPC_MEMDEBUG}
{$WARNING Compiling with memory debug enabled!}
{$ENDIF}


type
  pWBArg = ^tWBArg;
  tWBArg = record
    wa_Lock: longint;
    wa_Name: PChar;
  end;

  WBArgList = array[1..MaxInt] of TWBArg; { Only 1..smNumArgs are valid }
  PWBArgList = ^WBArgList;

  pWBStartup = ^tWBStartup;
  tWBStartup = packed record
    sm_Message   : tMessage;
    sm_Process   : pMsgPort;
    sm_Segment   : longint;
    sm_NumArgs   : longint;
    sm_ToolWindow: PChar;
    sm_ArgList   : PWBArgList;
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

  { Closing CON: when in Ambient mode }
  if MOS_ConHandle<>0 then dosClose(MOS_ConHandle);

  if MOS_UtilityBase<>nil then CloseLibrary(MOS_UtilityBase);
  if MOS_DOSBase<>nil then CloseLibrary(MOS_DOSBase);
  if ASYS_heapPool<>nil then DeletePool(ASYS_heapPool);

  { If in Ambient mode, replying WBMsg }
  if MOS_ambMsg<>nil then begin
    Forbid;
    ReplyMsg(MOS_ambMsg);
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


{ MorphOS specific startup }
procedure SysInitMorphOS;
var self: PProcess;
begin
 self:=PProcess(FindTask(nil));
 if self^.pr_CLI=0 then begin
   { if we're running from Ambient/Workbench, we catch its message }
   WaitPort(@self^.pr_MsgPort);
   MOS_ambMsg:=GetMsg(@self^.pr_MsgPort);
 end;

 MOS_DOSBase:=OpenLibrary('dos.library',50);
 if MOS_DOSBase=nil then Halt(1);
 MOS_UtilityBase:=OpenLibrary('utility.library',50);
 if MOS_UtilityBase=nil then Halt(1);

 { Creating the memory pool for growing heap }
 ASYS_heapPool:=CreatePool(MEMF_FAST or MEMF_SEM_PROTECTED,growheapsize2,growheapsize1);
 if ASYS_heapPool=nil then Halt(1);

 { Initialize semaphore for filelist access arbitration }
 ASYS_fileSemaphore:=AllocPooled(ASYS_heapPool,sizeof(TSignalSemaphore));
 if ASYS_fileSemaphore = nil then Halt(1);
 InitSemaphore(ASYS_fileSemaphore);

 if MOS_ambMsg=nil then begin
   MOS_ConHandle:=0;
   StdInputHandle:=dosInput;
   StdOutputHandle:=dosOutput;
   StdErrorHandle:=StdOutputHandle;
 end else begin
   MOS_ConHandle:=Open(MOS_ConName,MODE_OLDFILE);
   if MOS_ConHandle<>0 then begin
     StdInputHandle:=MOS_ConHandle;
     StdOutputHandle:=MOS_ConHandle;
     StdErrorHandle:=MOS_ConHandle;
   end else
     Halt(1);
 end;
end;


procedure SysInitStdIO;
begin
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);

  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
  OpenStdIO(ErrOutput,fmOutput,StdErrorHandle);
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
  MOS_ambMsg:=nil;
  ASYS_origDir:=0;
  ASYS_fileList:=nil;
  envp:=nil;
  SysInitMorphOS;
{ Set up signals handlers }
//  InstallSignals;
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
