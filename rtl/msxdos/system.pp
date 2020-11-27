unit System;


interface

{$define FPC_IS_SYSTEM}
{ The heap for MSDOS is implemented
  in tinyheap.inc include file,
  but it uses default SysGetMem names }

{$define HAS_MEMORYMANAGER}
{ define TEST_FPU_INT10 to force keeping local int10,
  for testing purpose only }


{$DEFINE FPC_INCLUDE_SOFTWARE_MUL}
{$DEFINE FPC_INCLUDE_SOFTWARE_MOD_DIV}

{$DEFINE FPC_USE_SMALL_DEFAULTSTACKSIZE}
{ To avoid warnings in thread.inc code,
  but value must be really given after
  systemh.inc is included otherwise the
  $mode switch is not effective }

{ Use Ansi Char for files }
{$define FPC_ANSI_TEXTFILEREC}
{$define FPC_STDOUT_TRUE_ALIAS}

{$ifdef NO_WIDESTRINGS}
  { Do NOT use wide Char for files }
  {$undef FPC_HAS_FEATURE_WIDESTRINGS}
{$endif NO_WIDESTRINGS}

{$I systemh.inc}
{$I tnyheaph.inc}
{$I portsh.inc}

{$ifndef FPUNONE}
{$ifdef FPC_HAS_FEATURE_SOFTFPU}

{$define fpc_softfpu_interface}
{$i softfpu.pp}
{$undef fpc_softfpu_interface}

{$endif FPC_HAS_FEATURE_SOFTFPU}
{$endif FPUNONE}

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
  { MSX-DOS does not have a separate StdErr }
  StdErrorHandle  = 1;

  FileNameCaseSensitive : boolean = false;
  FileNameCasePreserving: boolean = false;
  CtrlZMarksEOF: boolean = true; (* #26 is considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

var
{ Mem[] support }
  mem  : array[0..$7fff-1] of byte absolute $0;
  memw : array[0..($7fff div sizeof(word))-1] of word absolute $0;
  meml : array[0..($7fff div sizeof(longint))-1] of longint absolute $0;
{ C-compatible arguments and environment }
  argc:smallint; //!! public name 'operatingsystem_parameter_argc';
  argv:PPchar; //!! public name 'operatingsystem_parameter_argv';

{ The DOS Program Segment Prefix segment (TP7 compatibility) }
  PrefixSeg:Word;public name '__fpc_PrefixSeg';

  SaveInt00: FarPointer;public name '__SaveInt00';
  SaveInt10: FarPointer;public name '__SaveInt10';
  SaveInt75: FarPointer;public name '__SaveInt75';
  fpu_status: word;public name '__fpu_status';

const
  AllFilesMask: string [3] = '*.*';

const
  LFNSupport = false;

implementation

procedure DebugWrite(s: PChar); forward;
procedure DebugWrite(const S: string); forward;
procedure DebugWriteLn(const S: string); forward;

{$ifdef todo}
const
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
  PFarByte = ^Byte;//far;
  PFarChar = ^Char;//far;
  PFarWord = ^Word;//far;
  PPFarChar = ^PFarChar;
{$endif}

var
  stklen: word; external name '__stklen';

  __heapsize: Word;external name '__heapsize';
  __fpc_initialheap: array[0..0] of byte;external name '__fpc_initialheap';

var
  __stktop : pointer;public name '__stktop';
  dos_version:Word;public name 'dos_version';
  dos_env_count:smallint;public name '__dos_env_count';
  dos_argv0 : PChar;public name '__fpc_dos_argv0';

{$I registers.inc}

procedure Intr(IntNo: Byte; var Regs: Registers); external name 'FPC_INTR';

procedure MsxDos(var Regs: Registers); assembler; nostackframe; public name 'FPC_MSXDOS';
asm
  //in a, (0x2e)
  { store registers contents }
  push AF
  push BC
  push DE
  push HL
  push IX
  push IY
  { allocate an additional scratch space }
  push IY
  { Regs now resides at SP + 16 }

  { IY is not used for parameters, so base everything on that;
    for that we need to load the address of Regs into IY }
  ld IX, 0x10
  add IX, SP

  ld L,(IX+0)
  ld H,(IX+1)

  push HL
  pop IY

  { fill IX with the help of HL }
  ld L,(IY+8)
  ld H,(IY+9)

  push HL
  pop IX

  ld B,(IY+1)
  ld C,(IY+0)
  ld D,(IY+3)
  ld E,(IY+2)
  // load A last
  //ld A,(IY+4)
  ld H,(IY+7)
  ld L,(IY+6)

  ld A,(IY+4)

  { store IY to scratch location }
  ex (SP),IY

  { call to DOS }
  call 0x0005

  { store IY to scratch and restore pointer address of Regs }
  ex (SP),IY

  ld (IY+1),B
  ld (IY+0),C
  ld (IY+3),D
  ld (IY+2),E
  ld (IY+4),A
  // skip F
  ld (IY+7),H
  ld (IY+6),L

  { store IX with the help of HL }
  push IX
  pop HL
  ld (IY+8),L
  ld (IY+9),H

  { store the stored IY with the help of HL }
  ex (SP),HL

  ld (IY+10),L
  ld (IY+11),H

  { cleanup stack }
  pop IY
  pop IY
  pop IX
  pop HL
  pop DE
  pop BC
  pop AF
end;

procedure InstallInterruptHandlers; external name 'FPC_INSTALL_INTERRUPT_HANDLERS';
procedure RestoreInterruptHandlers; external name 'FPC_RESTORE_INTERRUPT_HANDLERS';

function CheckNullArea: Boolean; external name 'FPC_CHECK_NULLAREA';

{$I system.inc}

{$I tinyheap.inc}

{$I ports.inc}

{$ifndef FPUNONE}
{$ifdef FPC_HAS_FEATURE_SOFTFPU}

{$define fpc_softfpu_implementation}
{$i softfpu.pp}
{$undef fpc_softfpu_implementation}

{ we get these functions and types from the softfpu code }
{$define FPC_SYSTEM_HAS_float64}
{$define FPC_SYSTEM_HAS_float32}
{$define FPC_SYSTEM_HAS_flag}
{$define FPC_SYSTEM_HAS_extractFloat64Frac0}
{$define FPC_SYSTEM_HAS_extractFloat64Frac1}
{$define FPC_SYSTEM_HAS_extractFloat64Exp}
{$define FPC_SYSTEM_HAS_extractFloat64Frac}
{$define FPC_SYSTEM_HAS_extractFloat64Sign}
{$define FPC_SYSTEM_HAS_ExtractFloat32Frac}
{$define FPC_SYSTEM_HAS_extractFloat32Exp}
{$define FPC_SYSTEM_HAS_extractFloat32Sign}

{$endif FPC_HAS_FEATURE_SOFTFPU}
{$endif FPUNONE}

procedure DebugWrite(S: PChar);
var
  regs: Registers;
begin
  while S^ <> #0 do begin
    regs.C := $02;
    regs.E := Ord(S^);
    MsxDos(regs);
    Inc(S);
  end;
end;

procedure DebugWrite(const S: string);
var
  regs: Registers;
  i: Byte;
begin
  for i := 1 to Length(S) do begin
    regs.C := $02;
    regs.E := Ord(S[i]);
    MsxDos(regs);
  end;
end;

procedure DebugWriteLn(const S: string);
begin
  DebugWrite(S);
  DebugWrite(#13#10);
end;

{*****************************************************************************
                              ParamStr/Randomize
*****************************************************************************}

var
  internal_envp : PPChar = nil;

procedure setup_environment;
{$ifdef todo}
var
  env_count : smallint;
  cp, dos_env: PFarChar;
{$endif}
begin
{$ifdef todo}
  env_count:=0;
  dos_env:=Ptr(MemW[PrefixSeg:$2C], 0);
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
  if dos_version >= $300 then
    begin
      if cp=dos_env then
        inc(cp);
      inc(cp, 3);
      dos_argv0 := cp;
    end
  else
    dos_argv0 := nil;
{$endif}
end;

function envp:PPChar;public name '__fpc_envp';
begin
  if not assigned(internal_envp) then
    setup_environment;
  envp:=internal_envp;
end;

function GetEnvVar(aName: PChar): String;
var
  regs: Registers;
  i: SizeInt;
begin
  SetLength(Result, 255);
  regs.C := $6B;
  regs.HL := PtrUInt(aName);
  regs.DE := PtrUInt(@Result[1]);
  regs.B := 255;
  regs.A := 0;
  MsxDos(regs);
  if regs.A = 0 then begin
    i := 1;
    aName := PChar(@Result[1]);
    while i < 256 do begin
      if aName^ = #0 then begin
        SetLength(Result, i);
        Break;
      end;
      Inc(i);
      Inc(aName);
    end;
  end else
    SetLength(Result, 0);
end;

procedure setup_arguments;
var
  i: SmallInt;
  pc: PChar;
  quote: Char;
  count: SmallInt;
  arglen, argv0len: SmallInt;
  argblock: PChar;
  arg: PChar;
  doscmd   : string[129];  { Dos commandline copied from PSP, max is 128 chars +1 for terminating zero }
  tmp: String;
  regs: Registers;
begin
  tmp := GetEnvVar('PROGRAM');
  argv0len := Length(tmp);

  tmp := GetEnvVar('PARAMETERS');
{$IfDef SYSTEM_DEBUG_STARTUP}
  Writeln(stderr,'Dos command line is #',tmp,'# size = ',length(tmp));
{$EndIf }
  { parse dos commandline }
  pc:=@tmp[1];
  count:=1;
  { calc total arguments length and count }
  arglen:=argv0len+1;
  while pc^<>#0 do
    begin
      { skip leading spaces }
      while pc^ in [#1..#32] do
        inc(pc);
      if pc^=#0 then
        break;
      { calc argument length }
      quote:=' ';
      while (pc^<>#0) do
        begin
          case pc^ of
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
                    if pchar(pc+1)^<>'"' then
                      begin
                        if quote='"' then
                          quote:=' '
                        else
                          quote:='"';
                      end
                    else
                     inc(pc);
                  end
                else
                  inc(arglen);
              end;
            '''' :
              begin
                if quote<>'"' then
                  begin
                    if pchar(pc+1)^<>'''' then
                      begin
                        if quote=''''  then
                         quote:=' '
                        else
                         quote:='''';
                      end
                    else
                      inc(pc);
                  end
                else
                  inc(arglen);
              end;
            else
              inc(arglen);
          end;
          inc(pc);
        end;
      inc(arglen);  { for the null terminator }
      inc(count);
    end;
  Writeln(stderr,'Arg count: ', count, ', size: ', arglen);
  { set argc and allocate argv }
  argc:=count;
  argv:=AllocMem((count+1)*SizeOf(PChar));
  { allocate a single memory block for all arguments }
  argblock:=GetMem(arglen);
  writeln('Allocated arg vector at ', hexstr(argv), ' and block at ', hexstr(argblock));
  { create argv[0] }
  argv[0]:=argblock;
  arg:=argblock+argv0len;

  arg^:=#0;
  Inc(arg);

  pc:=@tmp[1];
  count:=1;
  while pc^<>#0 do
    begin
      { skip leading spaces }
      while pc^ in [#1..#32] do
        inc(pc);
      if pc^=#0 then
        break;
      { copy argument }
      //writeln('Setting arg ',count,' to ', hexstr(arg));
      asm
        in a,(0x2e)
      end ['a'];
      argv[count]:=arg;
      quote:=' ';
      while (pc^<>#0) do
        begin
          case pc^ of
            #1..#32 :
              begin
                if quote<>' ' then
                  begin
                    arg^:=pc^;
                    inc(arg);
                  end
                else
                  break;
              end;
            '"' :
              begin
                if quote<>'''' then
                  begin
                    if pchar(pc+1)^<>'"' then
                      begin
                        if quote='"' then
                          quote:=' '
                        else
                          quote:='"';
                      end
                    else
                      inc(pc);
                  end
                else
                  begin
                    arg^:=pc^;
                    inc(arg);
                  end;
              end;
            '''' :
              begin
                if quote<>'"' then
                  begin
                    if pchar(pc+1)^<>'''' then
                      begin
                        if quote=''''  then
                          quote:=' '
                        else
                          quote:='''';
                      end
                    else
                      inc(pc);
                  end
                else
                  begin
                    arg^:=pc^;
                    inc(arg);
                  end;
              end;
            else
              begin
                arg^:=pc^;
                inc(arg);
              end;
          end;
          inc(pc);
        end;
      arg^:=#0;
      Inc(arg);
 {$IfDef SYSTEM_DEBUG_STARTUP}
      Writeln(stderr,'dos arg ',count,' #',strlen(argv[count]),'#',argv[count],'#');
 {$EndIf SYSTEM_DEBUG_STARTUP}
      inc(count);
    end;

  arg:=argblock;
  tmp:=GetEnvVar('PROGRAM');
  pc:=@tmp[1];
  while pc^ <> #0 do
    begin
      arg^ := pc^;
      Inc(arg);
      Inc(pc);
    end;

  for count:=0 to argc-1 do
    writeln('arg ',count,' at ',hexstr(argv[count]));
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
{$ifdef todo}
var
  hl   : longint;
  regs : Registers;
{$endif}
begin
{$ifdef todo}
  regs.AH:=$2C;
  MsDos(regs);
  hl:=regs.DX;
  randseed:=hl*$10000+ regs.CX;
{$endif}
end;

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

procedure system_exit;
var
  h : byte;
begin
{$ifdef todo}
  RestoreInterruptHandlers;
{$endif}
  for h:=0 to max_files-1 do
    if openfiles[h] then
      begin
{$ifdef SYSTEMDEBUG}
         writeln(stderr,'file ',h,' "',opennames[h],'" not closed at exit');
{$endif SYSTEMDEBUG}
         if h>=5 then
           do_close(h);
      end;
{$ifndef FPC_MM_TINY}
{$ifdef todo}
  if not CheckNullArea then
    writeln(stderr, 'Nil pointer assignment');
{$endif}
{$endif FPC_MM_TINY}
  asm
    ld a, exitcode
    ld b, a
    ld c, 0x62
    call 0x0005
  end;
end;

{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

procedure InitDosHeap;
begin
  RegisterTinyHeapBlock_Simple_Prealigned(@__fpc_initialheap,__heapsize);
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
  GetProcessID := PrefixSeg;
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

procedure InitDosVersion;
var
  regs: Registers;
begin
  regs.C := $6F;
  regs.A := 0;
  MsxDos(regs);
  if regs.A <> 0 then
    dos_version := 0
  else if regs.B < 2 then
    dos_version := $100
  else
    dos_version := regs.DE;
end;

begin
  StackLength := stklen;
  StackBottom := __stktop - stklen;
  InitDosVersion;
  { for now we don't support MSX-DOS 1 }
  if dos_version < $100 then
    Halt($85);
{$ifdef todo}
  InstallInterruptHandlers;
{$endif}
  { To be set if this is a GUI or console application }
  IsConsole := TRUE;
{$ifdef FPC_HAS_FEATURE_DYNLIBS}
  { If dynlibs feature is disabled,
    IsLibrary is a constant, which can thus not be set to a value }
  { To be set if this is a library and not a program  }
  IsLibrary := FALSE;
{$endif def FPC_HAS_FEATURE_DYNLIBS}
{ Setup heap }
  InitDosHeap;
  SysInitExceptions;
{$ifdef FPC_HAS_FEATURE_UNICODESTRINGS}
  initunicodestringmanager;
{$endif def FPC_HAS_FEATURE_UNICODESTRINGS}
{ Setup stdin, stdout and stderr }
  SysInitStdIO;
{ Setup environment and arguments }
  { Done on  request only Setup_Environment; }
  { Done on request only Setup_Arguments; }
{ Reset IO Error }
  InOutRes:=0;
{$ifdef FPC_HAS_FEATURE_THREADING}
  InitSystemThreads;
{$endif}
end.
