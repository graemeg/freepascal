unit system;

interface

{$define FPC_IS_SYSTEM}

{ The heap for ZX Spectrum is implemented
  in tinyheap.inc include file,
  but it uses default SysGetMem names }

{$define HAS_MEMORYMANAGER}

{ Use Ansi Char for files }
{$define FPC_ANSI_TEXTFILEREC}
{$define FPC_STDOUT_TRUE_ALIAS}
{$define FPC_STDERR_IS_ALIAS_FOR_STDOUT}

{$I systemh.inc}
{$I tnyheaph.inc}

{$ifndef FPUNONE}
{$ifdef FPC_HAS_FEATURE_SOFTFPU}

{$define fpc_softfpu_interface}
{$i softfpu.pp}
{$undef fpc_softfpu_interface}

{$endif FPC_HAS_FEATURE_SOFTFPU}

{$endif FPUNONE}

var
{ Mem[] support }
  mem  : array[0..$7fff-1] of byte absolute $0;
{  memw : array[0..($7fff div sizeof(word))-1] of word absolute $0:$0;
  meml : array[0..($7fff div sizeof(longint))-1] of longint absolute $0:$0;}

{ OpenChannel(2) opens the upper screen
  OpenChannel(1) opens the lower screen
  OpenChannel(3) opens the ZX Printer }
procedure OpenChannel(Chan: Byte);
procedure PrintChar(Ch: Char);
procedure PrintLn;
procedure PrintShortString(const s: ShortString);
procedure PrintHexDigit(const d: byte);
procedure PrintHexByte(const b: byte);
procedure PrintHexWord(const w: word);
procedure Ink(colour: Byte);
procedure Paper(colour: Byte);
procedure GotoXY(X, Y: Byte);
function ReadKey: Char;
function KeyPressed: Boolean;

implementation

const
  LineEnding = #13;
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

{ Default filehandles }
  UnusedHandle    = $ffff;{ instead of -1, as it is a word value}
  StdInputHandle  = 0;
  StdOutputHandle = 1;
  StdErrorHandle  = 2;

  FileNameCaseSensitive : boolean = false;
  FileNameCasePreserving: boolean = false;
  CtrlZMarksEOF: boolean = true; (* #26 is considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCR;

var
  fpc_stackarea_start: word; external name '__fpc_stackarea_start';
  fpc_stackarea_end: word; external name '__fpc_stackarea_end';
  __heapsize: Word;external name '__heapsize';
  __fpc_initialheap: array[0..0] of byte;external name '__fpc_initialheap';

{$I system.inc}
{$I tinyheap.inc}

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

procedure randomize;
begin
end;

function GetProcessID: SizeUInt;
begin
  GetProcessID:=0;
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

procedure system_exit;
begin
  repeat
  until false;
end;

var
  save_iy: Word; public name 'FPC_SAVE_IY';
  LastKey: Char absolute 23560;

function ReadKey: Char;
begin
  repeat
    ReadKey:=LastKey;
  until ReadKey<>#0;
  LastKey:=#0;
end;

function KeyPressed: Boolean;
begin
  KeyPressed:=LastKey<>#0;
end;

procedure OpenChannel(Chan: Byte);assembler;
asm
  ld iy,(save_iy)
  ld a, (Chan)
  push ix
  call 5633
  pop ix
  ld (save_iy),iy
end;

procedure PrintChar(Ch: Char);assembler;
asm
  ld iy,(save_iy)
  ld a, (Ch)
  push ix
  rst 16
  pop ix
  ld (save_iy),iy
end;

procedure PrintLn;
begin
  PrintChar(#13);
end;

procedure PrintHexDigit(const d: byte);
begin
  { the code generator is still to broken to compile this, so we do it in a stupid way }
{  if (d >= 0) or (d <= 9) then
    PrintChar(Char(d + Ord('0')))
  else if (d >= 10) and (d <= 15) then
    PrintChar(Char(d + (Ord('A') - 10)));}
  if d=0 then
    PrintChar('0')
  else if d=1 then
    PrintChar('1')
  else if d=2 then
    PrintChar('2')
  else if d=3 then
    PrintChar('3')
  else if d=4 then
    PrintChar('4')
  else if d=5 then
    PrintChar('5')
  else if d=6 then
    PrintChar('6')
  else if d=7 then
    PrintChar('7')
  else if d=8 then
    PrintChar('8')
  else if d=9 then
    PrintChar('9')
  else if d=10 then
    PrintChar('A')
  else if d=11 then
    PrintChar('B')
  else if d=12 then
    PrintChar('C')
  else if d=13 then
    PrintChar('D')
  else if d=14 then
    PrintChar('E')
  else if d=15 then
    PrintChar('F')
  else
    PrintChar('?');
end;

procedure PrintHexByte(const b: byte);
begin
  PrintHexDigit(b shr 4);
  PrintHexDigit(b and $F);
end;

procedure PrintHexWord(const w: word);
begin
  PrintHexByte(Byte(w shr 8));
  PrintHexByte(Byte(w));
end;

procedure Ink(colour: Byte);
begin
  PrintChar(#16);
  PrintChar(Char(colour));
end;

procedure Paper(colour: Byte);
begin
  PrintChar(#17);
  PrintChar(Char(colour));
end;

procedure GotoXY(X, Y: Byte);
begin
  PrintChar(#22);
  PrintChar(Char(Y-1));
  PrintChar(Char(X-1));
end;

procedure PrintShortString(const s: ShortString);
var
  i: byte;
begin
  for i:=1 to length(s) do
    PrintChar(s[i]);
end;

{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

procedure InitZXHeap;
begin
  RegisterTinyHeapBlock_Simple_Prealigned(@__fpc_initialheap,__heapsize);
end;

procedure SysInitStdIO;
begin
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
{$ifndef FPC_STDERR_IS_ALIAS_FOR_STDOUT}
  OpenStdIO(ErrOutput,fmOutput,StdErrorHandle);
{$endif FPC_STDERR_IS_ALIAS_FOR_STDOUT}
{$ifndef FPC_STDOUT_TRUE_ALIAS}
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
{$endif FPC_STDOUT_TRUE_ALIAS}
end;

begin
  StackBottom:=@fpc_stackarea_start;
  StackLength:=(@fpc_stackarea_end-@fpc_stackarea_start)+1;
  { To be set if this is a GUI or console application }
  IsConsole := TRUE;
{$ifdef FPC_HAS_FEATURE_DYNLIBS}
  { If dynlibs feature is disabled,
    IsLibrary is a constant, which can thus not be set to a value }
  { To be set if this is a library and not a program  }
  IsLibrary := FALSE;
{$endif def FPC_HAS_FEATURE_DYNLIBS}
{ Setup heap }
  InitZXHeap;
  SysInitExceptions;
{$ifdef FPC_HAS_FEATURE_UNICODESTRINGS}
  initunicodestringmanager;
{$endif def FPC_HAS_FEATURE_UNICODESTRINGS}
{ Setup stdin, stdout and stderr }
  SysInitStdIO;
{ Reset IO Error }
  InOutRes:=0;
{$ifdef FPC_HAS_FEATURE_THREADING}
  InitSystemThreads;
{$endif}
end.
