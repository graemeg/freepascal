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
  seg0040: Word = $0040;
  segA000: Word = $A000;
  segB000: Word = $B000;
  segB800: Word = $B800;
{ The value that needs to be added to the segment to move the pointer by
  64K bytes (BP7 compatibility) }
  SelectorInc: Word = $1000;

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

  SaveInt00: FarPointer;public name '__SaveInt00';
  SaveInt10: FarPointer;public name '__SaveInt10';
  SaveInt75: FarPointer;public name '__SaveInt75';
  fpu_status: word;public name '__fpu_status';


  AllFilesMask: string [3];
{$ifndef RTLLITE}
{ System info }
  LFNSupport : boolean;
{$ELSE RTLLITE}
const
  LFNSupport = false;
{$endif RTLLITE}

implementation

procedure DebugWrite(const S: string); forward;
procedure DebugWriteLn(const S: string); forward;

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
  PFarChar = ^Char;far;
  PFarWord = ^Word;far;
  PPFarChar = ^PFarChar;

var
  __stktop : pointer;public name '__stktop';
  __stkbottom : pointer;public name '__stkbottom';
  __nearheap_start: pointer;public name '__nearheap_start';
  __nearheap_end: pointer;public name '__nearheap_end';
  dos_version:Word;public name 'dos_version';
  dos_env_count:smallint;public name '__dos_env_count';
  dos_argv0 : PFarChar;public name '__fpc_dos_argv0';

{$I registers.inc}

procedure Intr(IntNo: Byte; var Regs: Registers); external name 'FPC_INTR';
procedure MsDos(var Regs: Registers); external name 'FPC_MSDOS';

{ invokes int 21h with the carry flag set on entry; used for the LFN functions
  to ensure that the carry flag is set on exit on older DOS versions which don't
  support them }
procedure MsDos_Carry(var Regs: Registers); external name 'FPC_MSDOS_CARRY';

procedure InstallInterruptHandlers; external name 'FPC_INSTALL_INTERRUPT_HANDLERS';
procedure RestoreInterruptHandlers; external name 'FPC_RESTORE_INTERRUPT_HANDLERS';

function CheckNullArea: Boolean; external name 'FPC_CHECK_NULLAREA';


var
  test_fpu_jmpbuf : jmp_buf;

Procedure InterceptInvalidInstruction;
begin
  longjmp(test_fpu_jmpbuf, 1);
end;

{ Use msdos int21 set/get Interrupt address
  to check if coprocessor is present }

{$define FPC_SYSTEM_HAS_SYSINITFPU}
Procedure SysInitFPU;

  const
    CR0_NE = $20;
    CR0_NOT_NE = $FFFF - CR0_NE;
  var
    prevInt06 : FarPointer;
    _newcr0_lw : word;
    restore_old_int10 : boolean;


  begin
    restore_old_int10:=false;
    asm
      fninit
      fldcw   Default8087CW
      fwait
    end;
    if Test8087 < 3 then { i8087/i80287 do not have "native" exception mode (CR0:NE) }
      begin
        restore_old_int10:=true;
      end
    else
      begin
        asm
          push es
          push ds
          { Get previous interrupt 06 handler }
          mov ax, $3506
          int $21
          mov word [prevInt06],bx
          mov dx,es
          mov word [prevInt06+2],dx
          { Install local interrupt 06 handler }
    {$ifdef FPC_MM_TINY}
          { Do not use SEG here, as this introduces a relocation that
            is incompatible with COM executable generation }
          mov dx, cs
    {$else FPC_MM_TINY}
          mov dx, SEG InterceptInvalidInstruction
    {$endif FPC_MM_TINY}
          mov ds, dx
          mov dx, Offset InterceptInvalidInstruction
          mov ax, $2506
          int $21
          pop ds
          pop es
        end;
        if setjmp(test_fpu_jmpbuf)=0 then
          begin
            asm
              db $0f, $20, $c0 { mov eax,cr0 }
              { Reset CR0  Numeric Error bit,
                to trigger IRQ13 - interrupt $75,
                and thus avoid the generation of a $10 trap
                which iterferes with video interrupt handler }
              and ax,CR0_NOT_NE
              db $0f, $22, $c0 { mov cr0,eax }
            end;
            //writeln(stderr,'Change of cr0 succeeded');
            // Test that NE bit is indeed reset
            asm
              db $0f, $20, $c0 { mov eax,cr0 }
              mov _newcr0_lw, ax
            end;
            if (_newcr0_lw and CR0_NE) = 0 then
              restore_old_int10:=true;

          end
        else
          begin
            //writeln(stderr,'Change of cr0 failed');
          end;
        { Restore previous interrupt 06 handler }
        asm
          push ds
          mov ax, $2506
          lds dx,[prevInt06]
          int $21
          pop ds
        end;
      end;
      { Special handler of interrupt $10
        not needed anymore
        Restore previous interrupt $10 handler }
      {$ifndef TEST_FPU_INT10}
      if restore_old_int10 then
        asm
          push ds
          mov ax, $2510
          lds dx,[SaveInt10]
          int $21
          pop ds
        end;
      {$endif ndef TEST_FPU_INT10}
  end;

{$I system.inc}

{$I tinyheap.inc}

{$I ports.inc}

procedure DebugWrite(const S: string);
begin
  asm
{$if defined(FPC_X86_DATA_FAR) or defined(FPC_X86_DATA_HUGE)}
    push ds
	lds si, S
{$else}
    mov si, S
{$endif}
{$ifdef FPC_ENABLED_CLD}
    cld
{$endif FPC_ENABLED_CLD}
    lodsb
    mov cl, al
    xor ch, ch
    jcxz @@zero_length
    mov ah, 2

@@1:
    lodsb
    mov dl, al
    int 21h
    loop @@1
@@zero_length:
{$if defined(FPC_X86_DATA_FAR) or defined(FPC_X86_DATA_HUGE)}
    pop ds
{$endif}
  end ['ax','bx','cx','dx','si','di'];
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
  internal_envp : PPFarChar = nil;

procedure setup_environment;
var
  env_count : smallint;
  cp, dos_env: PFarChar;
begin
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
  doscmd   : string[129];  { Dos commandline copied from PSP, max is 128 chars +1 for terminating zero }
begin
  { force environment to be setup so dos_argv0 is loaded }
  envp;
  { load commandline from psp }
  SetLength(doscmd, Mem[PrefixSeg:$80]);
  for I := 1 to length(doscmd) do
    doscmd[I] := Chr(Mem[PrefixSeg:$80+I]);
  doscmd[length(doscmd)+1]:=#0;
{$IfDef SYSTEM_DEBUG_STARTUP}
  Writeln(stderr,'Dos command line is #',doscmd,'# size = ',length(doscmd));
{$EndIf }
  { find argv0len }
  argv0len:=0;
  if dos_argv0<>nil then
    begin
      pfc:=dos_argv0;
      while pfc^<>#0 do
        begin
          Inc(argv0len);
          Inc(pfc);
        end;
    end;
  { parse dos commandline }
  pc:=@doscmd[1];
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
  { set argc and allocate argv }
  argc:=count;
  argv:=AllocMem((count+1)*SizeOf(PChar));
  { allocate a single memory block for all arguments }
  argblock:=GetMem(arglen);
  { create argv[0] }
  argv[0]:=argblock;
  arg:=argblock;
  if dos_argv0<>nil then
    begin
      pfc:=dos_argv0;
      while pfc^<>#0 do
        begin
          arg^:=pfc^;
          Inc(arg);
          Inc(pfc);
        end;
    end;
  arg^:=#0;
  Inc(arg);

  pc:=@doscmd[1];
  count:=1;
  while pc^<>#0 do
    begin
      { skip leading spaces }
      while pc^ in [#1..#32] do
        inc(pc);
      if pc^=#0 then
        break;
      { copy argument }
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
var
  hl   : longint;
  regs : Registers;
begin
  regs.AH:=$2C;
  MsDos(regs);
  hl:=regs.DX;
  randseed:=hl*$10000+ regs.CX;
end;

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

procedure system_exit;
var
  h : byte;
begin
  RestoreInterruptHandlers;
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
  if not CheckNullArea then
    writeln(stderr, 'Nil pointer assignment');
{$endif FPC_MM_TINY}
  asm
    mov al, byte [exitcode]
    mov ah, 4Ch
    int 21h
  end;
end;

{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

procedure InitDosHeap;
type
{$if defined(FPC_X86_DATA_FAR) or defined(FPC_X86_DATA_HUGE)}
  TPointerArithmeticType = HugePointer;
{$else}
  TPointerArithmeticType = Pointer;
{$endif}
begin
  RegisterTinyHeapBlock_Simple_Prealigned(__nearheap_start, TPointerArithmeticType(__nearheap_end) - TPointerArithmeticType(__nearheap_start));
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

begin
  StackBottom := __stkbottom;
  StackLength := __stktop - __stkbottom;
  InstallInterruptHandlers;
  DetectFPU;
  if Test8087>0 then
    SysInitFPU;
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
{$ifndef RTLLITE}
{ Use LFNSupport LFN }
  LFNSupport:=CheckLFN;
  if LFNSupport then
   begin
    FileNameCasePreserving:=true;
    AllFilesMask := '*';
   end
  else
{$endif ndef RTLLITE}
   AllFilesMask := '*.*';
{ Reset IO Error }
  InOutRes:=0;
{$ifdef FPC_HAS_FEATURE_THREADING}
  InitSystemThreads;
{$endif}
end.
