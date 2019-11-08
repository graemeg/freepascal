{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
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
{$DEFINE HAS_MEMORYMANAGER}
{$DEFINE FPC_ANSI_TEXTFILEREC}

{$i systemh.inc}
{$i tnyheaph.inc}

{Platform specific information}
const
    LineEnding = #10;
    LFNSupport = false;
    DirectorySeparator = '/';
    DriveSeparator = ':';
    ExtensionSeparator = '.';
    PathSeparator = ';';
    AllowDirectorySeparators : set of char = ['\','/'];
    AllowDriveSeparators : set of char = [':'];
    FileNameCaseSensitive = false;
    FileNameCasePreserving = true;
    CtrlZMarksEOF: boolean = false; (* #26 not considered as end of file *)
    maxExitCode = 255; {.$ERROR TODO: CONFIRM THIS}
    MaxPathLen = 256;
    AllFilesMask = '*';

    sLineBreak = LineEnding;
    DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsLF;

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
{$i tinyheap.inc}
{$i syspara.inc}

  var
    palmAppInfo: SysAppInfoPtr; external name '__appInfo';

  procedure SysInitParamsAndEnv;
  begin
    {$WARNING: make sure argv/argc will be correct here}
    GenerateArgs;
  end;

  procedure randomize;
  begin
    {$WARNING: randseed initial value is zero!}
    randseed:=0;
  end;


{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}
procedure haltproc(e:longint);cdecl; external name 'haltproc';

procedure system_exit;
begin
  haltproc(ExitCode);
end;

function GetProcessID: SizeUInt;
begin
  GetProcessID := 1;
end;

{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

procedure SysInitStdIO;
begin
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);

  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
  OpenStdIO(ErrOutput,fmOutput,StdErrorHandle);
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

begin
{$ifdef FPC_HAS_FEATURE_STACKCHECK}
  StackLength := CheckInitialStkLen(InitialStkLen);
{$endif FPC_HAS_FEATURE_STACKCHECK}
{ Initialize ExitProc }
  ExitProc:=Nil;

{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
  SysInitExceptions;
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
{$ifdef FPC_HAS_FEATURE_UNICODESTRINGS}
  InitUnicodeStringManager;
{$endif FPC_HAS_FEATURE_UNICODESTRINGS}
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
{ Setup stdin, stdout and stderr }
  SysInitStdIO;
{$endif FPC_HAS_FEATURE_CONSOLEIO}
{ Reset IO Error }
  InOutRes:=0;
{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
{ Setup command line arguments }
  SysInitParamsAndEnv;
{$endif FPC_HAS_FEATURE_COMMANDARGS}
end.
