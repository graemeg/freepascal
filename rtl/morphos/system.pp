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
<<<<<<< HEAD
<<<<<<< HEAD
  AllFilesMask = '#?';
=======
  AllFilesMask = '*';
>>>>>>> graemeg/fixes_2_2
=======
  AllFilesMask = '*';
>>>>>>> origin/fixes_2_2

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

  argc: LongInt;
  argv: PPChar;
  envp: PPChar;


implementation

{$I system.inc}
{$I osdebug.inc}

{$IFDEF MOSFPC_FILEDEBUG}
{$WARNING Compiling with file debug enabled!}
{$ENDIF}

{$IFDEF MOSFPC_MEMDEBUG}
{$WARNING Compiling with memory debug enabled!}
{$ENDIF}


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

{ Generates correct argument array on startup }
procedure GenerateArgs;
var
  argvlen : longint;

  procedure allocarg(idx,len:longint);
    var
      i,oldargvlen : longint;
    begin
      if idx>=argvlen then
        begin
          oldargvlen:=argvlen;
          argvlen:=(idx+8) and (not 7);
          sysreallocmem(argv,argvlen*sizeof(pointer));
          for i:=oldargvlen to argvlen-1 do
            argv[i]:=nil;
        end;
      ArgV [Idx] := SysAllocMem (Succ (Len));
    end;

var
  count: word;
  start: word;
  localindex: word;
  p : pchar;
  temp : string;

begin
  p:=GetArgStr;
  argvlen:=0;

  { Set argv[0] }
  temp:=paramstr(0);
  allocarg(0,length(temp));
  move(temp[1],argv[0]^,length(temp));
  argv[0][length(temp)]:=#0;

  { check if we're started from Ambient }
  if MOS_ambMsg<>nil then begin
    argc:=0;
    exit;
  end;

  { Handle the other args }
  count:=0;
  { first index is one }
  localindex:=1;
  while (p[count]<>#0) do
    begin
      while (p[count]=' ') or (p[count]=#9) or (p[count]=LineEnding) do inc(count);
      start:=count;
      while (p[count]<>#0) and (p[count]<>' ') and (p[count]<>#9) and (p[count]<>LineEnding) do inc(count);
      if (count-start>0) then
        begin
          allocarg(localindex,count-start);
          move(p[start],argv[localindex]^,count-start);
          argv[localindex][count-start]:=#0;
          inc(localindex);
        end;
    end;
  argc:=localindex;
end;

function GetProgDir: String;
var
  s1     : String;
  alock  : LongInt;
  counter: Byte;
begin
  GetProgDir:='';
  FillChar(s1,255,#0);
  { GetLock of program directory }
  alock:=GetProgramDir;
  if alock<>0 then begin
    if NameFromLock(alock,@s1[1],255) then begin
      counter:=1;
      while (s1[counter]<>#0) and (counter<>0) do Inc(counter);
      s1[0]:=Char(counter-1);
      GetProgDir:=s1;
    end;
  end;
end;

function GetProgramName: String;
{ Returns ONLY the program name }
var
  s1     : String;
  counter: Byte;
begin
  GetProgramName:='';
  FillChar(s1,255,#0);
  if GetProgramName(@s1[1],255) then begin
    { now check out and assign the length of the string }
    counter := 1;
    while (s1[counter]<>#0) and (counter<>0) do Inc(counter);
    s1[0]:=Char(counter-1);

    { now remove any component path which should not be there }
    for counter:=length(s1) downto 1 do
      if (s1[counter] = '/') or (s1[counter] = ':') then break;
    { readjust counterv to point to character }
    if counter<>1 then Inc(counter);

    GetProgramName:=copy(s1,counter,length(s1));
  end;
end;

function GetArgv0Ambient: String;
{ Returns program full path+name, when in Ambient mode }
{ Required for paramstr(0) support in Ambient mode }
type
  pWBArg = ^tWBArg;
  tWBArg = record
    wa_Lock: longint;
    wa_Name: PChar;
  end;

  pWBStartup = ^tWBStartup;
  tWBStartup = packed record
    sm_Message   : tMessage;
    sm_Process   : pMsgPort;
    sm_Segment   : longint;
    sm_NumArgs   : longint;
    sm_ToolWindow: PChar;
    sm_ArgList   : pWBArg;
  end;

var
  tmpbuf  : String;
  counter : longint;
  progname: PChar;
  dlock   : longint;

begin
  GetArgv0Ambient:='';

  if MOS_ambMsg<>nil then begin
    dlock:=pWBStartup(MOS_ambMsg)^.sm_argList^.wa_Lock;
    if dlock<>0 then begin
      FillDWord(tmpbuf,256 div 4,0);
      if NameFromLock(dlock,@tmpbuf[1],255) then begin
        counter:=1;
        while tmpbuf[counter]<>#0 do
          inc(counter);
        tmpbuf[0]:=Char(counter-1);
        GetArgv0Ambient:=tmpbuf;
        { Append slash,if we're not in root directory of a volume }
<<<<<<< HEAD
<<<<<<< HEAD
        if tmpbuf[counter-1]<>':' then
          GetArgv0Ambient:=GetArgv0Ambient+'/';
=======
=======
>>>>>>> origin/fixes_2_2
        if tmpbuf[counter-1]<>':' then GetArgv0Ambient+='/';
>>>>>>> graemeg/fixes_2_2
      end;
    end;

    { Fetch the progname, and copy it to the buffer }
    progname:=pWBStartup(MOS_ambMsg)^.sm_argList^.wa_Name;
    if progname<>nil then begin
      FillDWord(tmpbuf,256 div 4,0);
      counter:=0;
      while (progname[counter]<>#0) do begin
        tmpbuf[counter+1]:=progname[counter];
<<<<<<< HEAD
<<<<<<< HEAD
        inc(counter);
=======
        counter+=1;
>>>>>>> graemeg/fixes_2_2
=======
        counter+=1;
>>>>>>> origin/fixes_2_2
      end;
      tmpbuf[0]:=Char(counter);
      GetArgv0Ambient:=GetArgv0Ambient+tmpbuf;
    end;
  end;
end;




{*****************************************************************************
                             ParamStr/Randomize
*****************************************************************************}

{ number of args }
function paramcount : longint;
begin
  if MOS_ambMsg<>nil then
    paramcount:=0
  else
    paramcount:=argc-1;
end;

{ argument number l }
function paramstr(l : longint) : string;
var
  s1: String;
begin
  paramstr:='';
  if MOS_ambMsg<>nil then begin
    if l=0 then begin
      paramstr:=GetArgv0Ambient;
      exit;
    end else
      exit;
  end;

  if l=0 then begin
    s1:=GetProgDir;
    if s1[length(s1)]=':' then paramstr:=s1+GetProgramName
                          else paramstr:=s1+'/'+GetProgramName;
  end else begin
    if (l>0) and (l+1<=argc) then paramstr:=strpas(argv[l]);
  end;
end;

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
  StackBottom := Sptr - StackLength;
  SysResetFPU;
  if not(IsLibrary) then
    SysInitFPU;
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
