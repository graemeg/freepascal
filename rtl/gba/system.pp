{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006 by Francesco Lombardi.

    System unit for Gameboy Advance

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit System;

interface
{$define __ARM__}

{$define FPC_IS_SYSTEM}

{$i systemh.inc}
{$i gbabiosh.inc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$i libch.inc}


=======
{$i systemh.inc}
>>>>>>> graemeg/fixes_2_2
=======
{$i systemh.inc}
>>>>>>> origin/fixes_2_2
=======
{$i libch.inc}


>>>>>>> origin/fixes_2.4

{$define fpc_softfpu_interface}
  {$i softfpu.pp}
{$undef fpc_softfpu_interface}


const
 LineEnding = #10;
 LFNSupport = true;
 CtrlZMarksEOF: boolean = false;
 DirectorySeparator = '/';
 DriveSeparator = ':';
 ExtensionSeparator = '.';
 PathSeparator = ';';
 AllowDirectorySeparators : set of char = ['\','/'];
 AllowDriveSeparators : set of char = [':'];
 FileNameCaseSensitive = false;
 FileNameCasePreserving = true;
 maxExitCode = 255;
 MaxPathLen = 255;
 AllFilesMask = '*';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 sLineBreak = LineEnding;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 sLineBreak : string[1] = LineEnding;
>>>>>>> graemeg/cpstrnew
 DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

  UnusedHandle    = $ffff;
  StdInputHandle  = 0;
  StdOutputHandle = 1;
  StdErrorHandle  = $ffff;

<<<<<<< HEAD
=======



<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
var
  argc: LongInt = 0;
  argv: PPChar;
  envp: PPChar;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
//  errno: integer;
>>>>>>> graemeg/cpstrnew
=======
//  errno: integer;
>>>>>>> graemeg/cpstrnew
=======
//  errno: integer;
>>>>>>> graemeg/cpstrnew
=======
//  errno: integer;
>>>>>>> origin/cpstrnew
=======
//  errno: integer;
>>>>>>> origin/fixes_2.4
  fake_heap_end: ^byte; cvar; external;


procedure randomize(value: integer);
=======
  errno: integer;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

implementation

{$linklib sysbase}

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
{$define FPC_SYSTEM_HAS_extractFloat64Sign}
{$define FPC_SYSTEM_HAS_extractFloat32Frac}
{$define FPC_SYSTEM_HAS_extractFloat32Exp}
{$define FPC_SYSTEM_HAS_extractFloat32Sign}

{$i system.inc}
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

>>>>>>> graemeg/fixes_2_2
{$i gbabios.inc}

{$i libc.inc}

{$ifdef FPC_HAS_FEATURE_PROCESSES}
function GetProcessID: SizeUInt;
begin
  GetProcessID := 0;
end;
{$endif}


{*****************************************************************************
                       Misc. System Dependent Functions
*****************************************************************************}
procedure System_exit;
begin

end;



{*****************************************************************************
                             ParamStr/Randomize
*****************************************************************************}
const
  QRAN_SHIFT  = 15;
  QRAN_MASK   = ((1 shl QRAN_SHIFT) - 1);
  QRAN_MAX    = QRAN_MASK;
  QRAN_A      = 1664525;
  QRAN_C      = 1013904223;

{ set randseed to a new pseudo random value }
procedure randomize();
begin
  RandSeed := 63458;
end;

procedure randomize(value: integer);
begin
  RandSeed := value;
end;

function random(): integer;
begin	
	RandSeed := QRAN_A * RandSeed + QRAN_C;
	random := (RandSeed shr 16) and QRAN_MAX;
end;

function random(value: integer): integer;
var
  a: integer;
begin	
	RandSeed := QRAN_A * RandSeed + QRAN_C;
	a := (RandSeed shr 16) and QRAN_MAX;
  random := (a * value) shr 15;
end;


{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
{ number of args }
function paramcount : longint;
begin
  paramcount:=0;
end;

{ argument number l }
function paramstr(l : longint) : string;
begin
  paramstr:='';
end;
{$endif FPC_HAS_FEATURE_COMMANDARGS}

{$ifdef FPC_HAS_FEATURE_TEXTIO}
procedure SysInitStdIO;
begin
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
end;
{$endif FPC_HAS_FEATURE_TEXTIO}


function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;


begin
  StackLength := CheckInitialStkLen(InitialStkLen);
<<<<<<< HEAD
<<<<<<< HEAD
  StackBottom := Sptr - StackLength;
{ OS specific startup }

{ Set up signals handlers }
//  fpc_cpucodeinit;
=======
  StackBottom := StackTop - StackLength;
{ OS specific startup }
>>>>>>> graemeg/fixes_2_2
=======
  StackBottom := StackTop - StackLength;
{ OS specific startup }
>>>>>>> origin/fixes_2_2

{ Set up signals handlers }
  fpc_cpucodeinit;

{ Setup heap }
  InitHeap;
  SysInitExceptions;
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
  { Setup stdin, stdout and stderr }
  SysInitStdIO;
  { Reset IO Error }
  InOutRes:=0;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif FPC_HAS_FEATURE_CONSOLEIO}

{$ifdef FPC_HAS_FEATURE_THREADING}
  { threading }
=======
{ Arguments }
>>>>>>> graemeg/fixes_2_2
=======
{ Arguments }
>>>>>>> origin/fixes_2_2
  InitSystemThreads;
{$endif FPC_HAS_FEATURE_THREADING}
end.
