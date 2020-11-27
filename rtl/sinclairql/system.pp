{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 by Karoly Balogh

    System unit for the Sinclair QL

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit System;

interface

{$define FPC_IS_SYSTEM}
{$define FPC_STDOUT_TRUE_ALIAS}
{$define FPC_ANSI_TEXTFILEREC}
{$define FPC_QL_USE_TINYHEAP}

{$ifdef FPC_QL_USE_TINYHEAP}
{$define HAS_MEMORYMANAGER}
{$endif FPC_QL_USE_TINYHEAP}

{$i systemh.inc}
{$ifdef FPC_QL_USE_TINYHEAP}
{$i tnyheaph.inc}
{$endif FPC_QL_USE_TINYHEAP}

{Platform specific information}
const
    LineEnding = #10;
    LFNSupport = false;
    CtrlZMarksEOF: boolean = false; (* #26 not considered as end of file *)
    DirectorySeparator = '\';
    DriveSeparator = ':';
    ExtensionSeparator = '.';
    PathSeparator = ';';
    AllowDirectorySeparators : set of char = ['\','/'];
    AllowDriveSeparators : set of char = [':'];
    FileNameCaseSensitive = false;
    FileNameCasePreserving = false;
    maxExitCode = 255;
    MaxPathLen = 255;
    AllFilesMask = '*.*';

    sLineBreak = LineEnding;
    DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsLF;

const
    UnusedHandle    = -1;
    StdInputHandle: longint = UnusedHandle;
    StdOutputHandle: longint = UnusedHandle;
    StdErrorHandle: longint = UnusedHandle;

var
    args: PChar;
    argc: LongInt;
    argv: PPChar;
    envp: PPChar;

    heapStart: pointer;


    {$if defined(FPUSOFT)}

    {$define fpc_softfpu_interface}
    {$i softfpu.pp}
    {$undef fpc_softfpu_interface}

    {$endif defined(FPUSOFT)}


implementation

  {$if defined(FPUSOFT)}

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

  {$endif defined(FPUSOFT)}

  {$i system.inc}
  {$ifdef FPC_QL_USE_TINYHEAP}
  {$i tinyheap.inc}
  {$endif FPC_QL_USE_TINYHEAP}


function GetProcessID:SizeUInt;
begin
  GetProcessID := mt_inf(nil, nil);
end;

procedure SysInitParamsAndEnv;
begin
end;

procedure randomize;
begin
  {$WARNING: randseed is uninitialized}
  randseed:=0;
end;

procedure PrintStr(ch: longint; const s: shortstring);
begin
  io_sstrg(ch,-1,@s[1],ord(s[0]));
end;

procedure DebugStr(const s: shortstring); public name '_dbgstr';
var
  i: longint;
begin
  PrintStr(stdOutputHandle,s);
  for i:=0 to 10000 do begin end;
end;

{$ifdef FPC_QL_USE_TINYHEAP}
procedure InitQLHeap;
begin
  HeapOrg:=nil;
  HeapEnd:=nil;
  FreeList:=nil;
  HeapPtr:=nil;
end;
{$endif}

{*****************************************************************************
                        System Dependent Entry code
*****************************************************************************}
{ QL/QDOS specific startup }
procedure SysInitQDOS;
var
  r: TQLRect;
begin
  stdInputHandle:=io_open('con_',Q_OPEN);
  stdOutputHandle:=stdInputHandle;
  stdErrorHandle:=stdInputHandle;

  r.q_width:=512;
  r.q_height:=256;
  r.q_x:=0;
  r.q_y:=0;

  sd_wdef(stdInputHandle,-1,2,1,@r);
  sd_clear(stdInputHandle,-1);
end;

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

procedure haltproc(e:longint); external name '_haltproc';

procedure system_exit;
const
  anyKey: string = 'Press any key to exit';
begin
  io_sstrg(stdOutputHandle, -1, @anyKey[1], ord(anyKey[0]));
  io_fbyte(stdInputHandle, -1);

  stdInputHandle:=UnusedHandle;
  stdOutputHandle:=UnusedHandle;
  stdErrorHandle:=UnusedHandle;
  haltproc(exitcode);
end;

{*****************************************************************************
                         System Unit Initialization
*****************************************************************************}

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

function CheckInitialStkLen (StkLen: SizeUInt): SizeUInt;
begin
  CheckInitialStkLen := StkLen;
end;


begin
  StackLength := CheckInitialStkLen (InitialStkLen);
{ Initialize ExitProc }
  ExitProc:=Nil;
  SysInitQDOS;
{$ifndef FPC_QL_USE_TINYHEAP}
{ Setup heap }
  InitHeap;
{$else FPC_QL_USE_TINYHEAP}
  InitQLHeap;
{$endif FPC_QL_USE_TINYHEAP}
  SysInitExceptions;
{$ifdef FPC_HAS_FEATURE_UNICODESTRINGS}
  InitUnicodeStringManager;
{$endif FPC_HAS_FEATURE_UNICODESTRINGS}
{ Setup stdin, stdout and stderr }
  SysInitStdIO;
{ Reset IO Error }
  InOutRes:=0;
{ Setup command line arguments }
  SysInitParamsAndEnv;
{$ifdef FPC_HAS_FEATURE_THREADING}
  InitSystemThreads;
{$endif FPC_HAS_FEATURE_THREADING}
end.
