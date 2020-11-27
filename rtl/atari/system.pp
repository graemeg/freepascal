{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2016 the Free Pascal development team

    Portions based on the Atari RTL for FPC 1.x
    Copyright (c) 1999-2000 by Carl Eric Codere
    member of the Free Pascal development team

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
{$define FPC_ATARI_USE_TINYHEAP}

{$ifdef FPC_ATARI_USE_TINYHEAP}
{$define HAS_MEMORYMANAGER}
{$endif FPC_ATARI_USE_TINYHEAP}

{$i systemh.inc}
{$ifdef FPC_ATARI_USE_TINYHEAP}
{$i tnyheaph.inc}
{$endif FPC_ATARI_USE_TINYHEAP}

{Platform specific information}
const
    LineEnding = #13#10;
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
    DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

const
    UnusedHandle    = $ffff;
    StdInputHandle  = 0;
    StdOutputHandle = 1;
    StdErrorHandle  = $ffff;

var
    args: PChar;
    argc: LongInt;
    argv: PPChar;
    envp: PPChar;


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
    {$ifdef FPC_ATARI_USE_TINYHEAP}
    {$i tinyheap.inc}
    {$endif FPC_ATARI_USE_TINYHEAP}
    {$i syspara.inc}

  var
    basepage: PPD; external name '__base';


  function GetProcessID:SizeUInt;
  begin
    {$WARNING To be checked by platform maintainer}
    GetProcessID := 1;
  end;


  procedure SysInitParamsAndEnv;
  begin
    // [0] index contains the args length...
    args:=@basepage^.p_cmdlin[1];
    GenerateArgs;
  end;


  procedure randomize;
  begin
    {$WARNING: randseed initial value is 24bit}
    randseed:=xbios_random;
  end;

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}
Procedure system_exit;
begin
  gemdos_pterm(ExitCode);
end;

{*****************************************************************************
                         SystemUnit Initialization
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
{$ifndef FPC_ATARI_USE_TINYHEAP}
{ Setup heap }
  InitHeap;
{$endif FPC_ATARI_USE_TINYHEAP}
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
