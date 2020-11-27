unit system;

interface

{$define FPC_IS_SYSTEM}
{$DEFINE FPC_NO_DEFAULT_HEAP}
{$DEFINE FPC_NO_DEFAULT_MEMORYMANAGER}
{$DEFINE HAS_MEMORYMANAGER}

{$DEFINE FPC_INCLUDE_SOFTWARE_MUL}
{$DEFINE FPC_INCLUDE_SOFTWARE_MOD_DIV}

{$DEFINE FPC_USE_SMALL_DEFAULTSTACKSIZE}
{ To avoid warnings in thread.inc code,
  but value must be really given after
  systemh.inc is included otherwise the
  $mode switch is not effective }

{$DEFINE HAS_CMDLINE}

{$DEFINE DISABLE_NO_DYNLIBS_MANAGER}
{$DEFINE FPC_SYSTEM_HAS_SYSDLH}

{$I systemh.inc}
{$IFDEF FPC_X86_DATA_NEAR}
{$I locheaph.inc}
{$ELSE FPC_X86_DATA_NEAR}
{$I glbheaph.inc}
{$ENDIF FPC_X86_DATA_NEAR}

const
  LineEnding = #13#10;
  { LFNSupport is a variable here, defined below!!! }
  DirectorySeparator = '\';
  DriveSeparator = ':';
  ExtensionSeparator = '.';
  PathSeparator = ';';
  AllowDirectorySeparators : set of char = ['\','/'];
  AllowDriveSeparators : set of char = [':'];
  { FileNameCaseSensitive and FileNameCasePreserving are defined separately below!!! }
  maxExitCode = 255;
  MaxPathLen = 256;

const
{ Default filehandles }
  UnusedHandle    = $ffff;{ instead of -1, as it is a word value}
  StdInputHandle  = 0;
  StdOutputHandle = 1;
  StdErrorHandle  = 2;

  FileNameCaseSensitive : boolean = false;
  FileNameCasePreserving: boolean = false;
  CtrlZMarksEOF: boolean = true; (* #26 is considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

{ Default memory segments (Tp7 compatibility) }
{  seg0040: Word = $0040;
  segA000: Word = $A000;
  segB000: Word = $B000;
  segB800: Word = $B800;}

type
  LPSTR = ^Char;far;
  PFarChar = ^Char;far;
  PHugeChar = ^Char;huge;

var
{ Mem[] support }
  mem  : array[0..$7fff-1] of byte absolute $0:$0;
  memw : array[0..($7fff div sizeof(word))-1] of word absolute $0:$0;
  meml : array[0..($7fff div sizeof(longint))-1] of longint absolute $0:$0;
{ C-compatible arguments and environment }
  argc:smallint; //!! public name 'operatingsystem_parameter_argc';
  argv:PPchar; //!! public name 'operatingsystem_parameter_argv';

{ The DOS Program Segment Prefix segment (TP7 compatibility) }
  PrefixSeg:Word;public name '__fpc_PrefixSeg';

{ BP7 compatible windows variables }
{ In C, these are the parameters to WinMain }
  CmdLine: LPSTR;public name '__fpc_CmdLine';
  CmdShow: SmallInt;public name '__fpc_CmdShow';
  HInstance: Word{HINST};public name '__fpc_HInstance';
  HPrevInst: Word{HINST};public name '__fpc_HPrevInst';
{ The value that needs to be added to the segment to move the pointer by
  64K bytes (BP7 compatibility) }
  SelectorInc: Word;public name '__fpc_SelectorInc';

{  SaveInt00: FarPointer;public name '__SaveInt00';}

  { Required for i8086.inc Stack check code }
  __stkbottom : pointer;public name '__stkbottom';


  AllFilesMask: string [3];
{$ifndef RTLLITE}
{ System info }
  LFNSupport : boolean;
{$ELSE RTLLITE}
const
  LFNSupport = false;
{$endif RTLLITE}


implementation

const
  fCarry = 1;

  { used for an offset fixup for accessing the proc parameters in asm routines
    that use nostackframe. We can't use the parameter name directly, because
    i8086 doesn't support sp relative addressing. }
{$ifdef FPC_X86_CODE_FAR}
  extra_param_offset = 2;
{$else FPC_X86_CODE_FAR}
  extra_param_offset = 0;
{$endif FPC_X86_CODE_FAR}
{$if defined(FPC_X86_DATA_FAR) or defined(FPC_X86_DATA_HUGE)}
  extra_data_offset = 2;
{$else}
  extra_data_offset = 0;
{$endif}

type
  PFarByte = ^Byte;far;
  PFarWord = ^Word;far;
  PPFarChar = ^PFarChar;

  { structure, located at DS:0, initialized by InitTask }
  PAutoDataSegHeader = ^TAutoDataSegHeader;
  TAutoDataSegHeader = record
    null: Word;
    oOldSP: Word;
    hOldSS: Word;
    pLocalHeap: Word;
    pAtomTable: Word;
    pStackTop: Word;
    pStackMin: Word;
    pStackBot: Word;
  end;

var
  dos_env_count:smallint;public name '__dos_env_count';

{$I registers.inc}

procedure MsDos(var Regs: Registers); external name 'FPC_MSDOS';

{ invokes int 21h with the carry flag set on entry; used for the LFN functions
  to ensure that the carry flag is set on exit on older DOS versions which don't
  support them }
procedure MsDos_Carry(var Regs: Registers); external name 'FPC_MSDOS_CARRY';

{$define SYSTEMUNIT}
{$I wintypes.inc}
{$I winprocsh.inc}
{$I winprocs.inc}

{ in protected mode, loading invalid values into segment registers causes an
  exception, so we use this function to initialize our Registers structure }
procedure ZeroSegRegs(var regs: Registers); inline;
begin
  regs.DS:=0;
  regs.ES:=0;
end;

{$I system.inc}

{$IFDEF FPC_X86_DATA_NEAR}
{$I locheap.inc}
{$ELSE FPC_X86_DATA_NEAR}
{$I glbheap.inc}
{$ENDIF FPC_X86_DATA_NEAR}


{*****************************************************************************
                              FinalizeHeap
   Dummy FinalizeHeap procedure added to fix compilation
*****************************************************************************}

procedure FinalizeHeap;
begin
end;

{*****************************************************************************
                              ParamStr/Randomize
*****************************************************************************}

var
  internal_envp : PPFarChar = nil;

procedure setup_environment;
var
  env_count : smallint;
  cp, dos_env: PFarChar;
begin
  env_count:=0;
  dos_env:=GetDOSEnvironment;
  cp:=dos_env;
  while cp^<>#0 do
    begin
      inc(env_count);
      while (cp^ <> #0) do
        inc(cp); { skip to NUL }
      inc(cp); { skip to next character }
    end;
  internal_envp := getmem((env_count+1) * sizeof(PFarChar));
  cp:=dos_env;
  env_count:=0;
  while cp^<>#0 do
    begin
      internal_envp[env_count] := cp;
      inc(env_count);
      while (cp^ <> #0) do
        inc(cp); { skip to NUL }
      inc(cp); { skip to next character }
    end;
  internal_envp[env_count]:=nil;
  dos_env_count := env_count;
end;

function envp:PPFarChar;public name '__fpc_envp';
begin
  if not assigned(internal_envp) then
    setup_environment;
  envp:=internal_envp;
end;


procedure setup_arguments;
var
  I: SmallInt;
  pc: PChar;
  pfc: PFarChar;
  quote: Char;
  count: SmallInt;
  arglen, argv0len: SmallInt;
  argblock: PChar;
  arg: PChar;
  argv0_arr: array [0..255] of Char;
{$IfDef SYSTEM_DEBUG_STARTUP}
  debug_output: Text;
{$EndIf}
begin
{$IfDef SYSTEM_DEBUG_STARTUP}
  Assign(debug_output,'debug.txt');
  Rewrite(debug_output);
  Writeln(debug_output,'Dos command line is #',CmdLine,'#');
{$EndIf}
  { find argv0len }
  argv0len:=GetModuleFileName(hInstance,FarAddr(argv0_arr),SizeOf(argv0_arr));
{$IfDef SYSTEM_DEBUG_STARTUP}
  Writeln(debug_output,'arv0 is #',argv0_arr,'# len=', argv0len);
{$EndIf}
  { parse dos commandline }
  pfc:=CmdLine;
  count:=1;
  { calc total arguments length and count }
  arglen:=argv0len+1;
  while pfc^<>#0 do
    begin
      { skip leading spaces }
      while pfc^ in [#1..#32] do
        inc(pfc);
      if pfc^=#0 then
        break;
      { calc argument length }
      quote:=' ';
      while (pfc^<>#0) do
        begin
          case pfc^ of
            #1..#32 :
              begin
                if quote<>' ' then
                  inc(arglen)
                else
                  break;
              end;
            '"' :
              begin
                if quote<>'''' then
                  begin
                    if pfarchar(pfc+1)^<>'"' then
                      begin
                        if quote='"' then
                          quote:=' '
                        else
                          quote:='"';
                      end
                    else
                     inc(pfc);
                  end
                else
                  inc(arglen);
              end;
            '''' :
              begin
                if quote<>'"' then
                  begin
                    if pfarchar(pfc+1)^<>'''' then
                      begin
                        if quote=''''  then
                         quote:=' '
                        else
                         quote:='''';
                      end
                    else
                      inc(pfc);
                  end
                else
                  inc(arglen);
              end;
            else
              inc(arglen);
          end;
          inc(pfc);
        end;
      inc(arglen);  { for the null terminator }
      inc(count);
    end;
  { set argc and allocate argv }
  argc:=count;
  argv:=AllocMem((count+1)*SizeOf(PChar));
  { allocate a single memory block for all arguments }
  argblock:=GetMem(arglen);
  { create argv[0] }
  argv[0]:=argblock;
  arg:=argblock;
  if argv0len>0 then
    begin
      pc:=@argv0_arr;
      while pc^<>#0 do
        begin
          arg^:=pc^;
          Inc(arg);
          Inc(pc);
        end;
    end;
  arg^:=#0;
  Inc(arg);

  pfc:=CmdLine;
  count:=1;
  while pfc^<>#0 do
    begin
      { skip leading spaces }
      while pfc^ in [#1..#32] do
        inc(pfc);
      if pfc^=#0 then
        break;
      { copy argument }
      argv[count]:=arg;
      quote:=' ';
      while (pfc^<>#0) do
        begin
          case pfc^ of
            #1..#32 :
              begin
                if quote<>' ' then
                  begin
                    arg^:=pfc^;
                    inc(arg);
                  end
                else
                  break;
              end;
            '"' :
              begin
                if quote<>'''' then
                  begin
                    if pfarchar(pfc+1)^<>'"' then
                      begin
                        if quote='"' then
                          quote:=' '
                        else
                          quote:='"';
                      end
                    else
                      inc(pfc);
                  end
                else
                  begin
                    arg^:=pfc^;
                    inc(arg);
                  end;
              end;
            '''' :
              begin
                if quote<>'"' then
                  begin
                    if pfarchar(pfc+1)^<>'''' then
                      begin
                        if quote=''''  then
                          quote:=' '
                        else
                          quote:='''';
                      end
                    else
                      inc(pfc);
                  end
                else
                  begin
                    arg^:=pfc^;
                    inc(arg);
                  end;
              end;
            else
              begin
                arg^:=pfc^;
                inc(arg);
              end;
          end;
          inc(pfc);
        end;
      arg^:=#0;
      Inc(arg);
{$IfDef SYSTEM_DEBUG_STARTUP}
      Writeln(debug_output,'dos arg ',count,' #',strlen(argv[count]),'#',argv[count],'#');
{$EndIf SYSTEM_DEBUG_STARTUP}
      inc(count);
    end;
{$IfDef SYSTEM_DEBUG_STARTUP}
  Close(debug_output);
{$EndIf SYSTEM_DEBUG_STARTUP}
end;


function paramcount : longint;
begin
  if argv=nil then
    setup_arguments;
  paramcount := argc - 1;
end;


function paramstr(l : longint) : string;
begin
  if argv=nil then
    setup_arguments;
  if (l>=0) and (l+1<=argc) then
    paramstr:=strpas(argv[l])
  else
    paramstr:='';
end;

procedure randomize;
begin
  randseed:=GetTickCount;
end;

{****************************************************************************
                    Error Message writing using messageboxes
****************************************************************************}

const
  ErrorBufferLength = 1024;
  ErrorMessageBoxFlags = MB_OK or MB_ICONHAND or MB_TASKMODAL;
var
  ErrorBuf : array[0..ErrorBufferLength] of char;
  ErrorLen : SizeInt;

procedure ErrorWrite(Var F: TextRec);
{
  An error message should always end with #13#10#13#10
}
var
  i : SizeInt;
Begin
  while F.BufPos>0 do
    begin
      begin
        if F.BufPos+ErrorLen>ErrorBufferLength then
          i:=ErrorBufferLength-ErrorLen
        else
          i:=F.BufPos;
        Move(F.BufPtr^,ErrorBuf[ErrorLen],i);
        inc(ErrorLen,i);
        ErrorBuf[ErrorLen]:=#0;
      end;
      if ErrorLen=ErrorBufferLength then
        begin
          if not NoErrMsg then
{$IFDEF FPC_X86_DATA_NEAR}
            MessageBox(0,Ptr(Seg(ErrorBuf),Ofs(ErrorBuf)),nil,ErrorMessageBoxFlags);
{$ELSE FPC_X86_DATA_NEAR}
            MessageBox(0,@ErrorBuf,nil,ErrorMessageBoxFlags);
{$ENDIF FPC_X86_DATA_NEAR}
          ErrorLen:=0;
        end;
      Dec(F.BufPos,i);
    end;
End;


procedure ShowErrMsg;
begin
  if ErrorLen>0 then
    begin
{$IFDEF FPC_X86_DATA_NEAR}
      MessageBox(0,Ptr(Seg(ErrorBuf),Ofs(ErrorBuf)),nil,ErrorMessageBoxFlags);
{$ELSE FPC_X86_DATA_NEAR}
      MessageBox(0,@ErrorBuf,nil,ErrorMessageBoxFlags);
{$ENDIF FPC_X86_DATA_NEAR}
      ErrorLen:=0;
    end;
end;


procedure ErrorClose(Var F: TextRec);
begin
  ShowErrMsg;
end;


procedure ErrorOpen(Var F: TextRec);
Begin
  TextRec(F).Handle:=StdErrorHandle;
  TextRec(F).Mode:=fmOutput;
  TextRec(F).InOutFunc:=@ErrorWrite;
  TextRec(F).FlushFunc:=@ErrorWrite;
  TextRec(F).CloseFunc:=@ErrorClose;
  ErrorLen:=0;
End;


procedure AssignError(Var T: Text);
begin
  Assign(T,'');
  TextRec(T).OpenFunc:=@ErrorOpen;
  Rewrite(T);
end;

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

procedure system_exit;
{var
  h : byte;}
begin
(*  RestoreInterruptHandlers;
  for h:=0 to max_files-1 do
    if openfiles[h] then
      begin
{$ifdef SYSTEMDEBUG}
         writeln(stderr,'file ',opennames[h],' not closed at exit');
{$endif SYSTEMDEBUG}
         if h>=5 then
           do_close(h);
      end;
{$ifndef FPC_MM_TINY}
  if not CheckNullArea then
    writeln(stderr, 'Nil pointer assignment');
{$endif FPC_MM_TINY}*)
  Close(stderr);
  Close(stdout);
  Close(erroutput);
  Close(Input);
  Close(Output);
  ShowErrMsg;
  asm
    mov al, byte [exitcode]
    mov ah, 4Ch
    int 21h
  end;
end;

{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

procedure InitWin16Heap;
begin
{$ifdef FPC_X86_DATA_NEAR}
  SetMemoryManager(LocalHeapMemoryManager);
{$else FPC_X86_DATA_NEAR}
  SetMemoryManager(GlobalHeapMemoryManager);
{$endif FPC_X86_DATA_NEAR}
end;

function CheckLFN:boolean;
var
  regs     : Registers;
  RootName : pchar;
  buf      : array [0..31] of char;
begin
{ Check LFN API on drive c:\ }
  RootName:='C:\';
{ Call 'Get Volume Information' ($71A0) }
{ no need to ZeroSegRegs(regs), because we initialize both DS and ES }
  regs.AX:=$71a0;
  regs.ES:=Seg(buf);
  regs.DI:=Ofs(buf);
  regs.CX:=32;
  regs.DS:=Seg(RootName^);
  regs.DX:=Ofs(RootName^);
  MsDos_Carry(regs);
{ If carryflag=0 and LFN API bit in ebx is set then use Long file names }
  CheckLFN:=(regs.Flags and fCarry=0) and (regs.BX and $4000=$4000);
end;

procedure SysInitStdIO;
begin
  AssignError(stderr);
  AssignError(StdOut);
  Assign(Output,'');
  Assign(Input,'');
  Assign(ErrOutput,'');
end;

function GetProcessID: SizeUInt;
begin
  GetProcessID := PrefixSeg;
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

begin
{$if defined(FPC_X86_DATA_FAR) or defined(FPC_X86_DATA_HUGE)}
  with PAutoDataSegHeader(Ptr(DSeg,0))^ do
    begin
      StackBottom := Ptr(SSeg,pStackTop);
      StackLength := pStackBot-pStackTop;
    end;
{$else}
  with PAutoDataSegHeader(0)^ do
    begin
      StackBottom := NearPointer(pStackTop);
      StackLength := pStackBot-pStackTop;
    end;
{$endif}
  __stkbottom := StackBottom;
  { To be set if this is a GUI or console application }
  IsConsole := FALSE;
  { To be set if this is a library and not a program  }
  IsLibrary := FALSE;
{ Setup heap }
  InitWin16Heap;
  SysInitExceptions;
  initunicodestringmanager;
{ Setup stdin, stdout and stderr }
  SysInitStdIO;
{ Use LFNSupport LFN }
  LFNSupport:=CheckLFN;
  if LFNSupport then
   begin
    FileNameCasePreserving:=true;
    AllFilesMask := '*';
   end
  else
   AllFilesMask := '*.*';
  InitSystemDynLibs;
{ Reset IO Error }
  InOutRes:=0;
{$ifdef FPC_HAS_FEATURE_THREADING}
  InitSystemThreads;
{$endif}
end.
