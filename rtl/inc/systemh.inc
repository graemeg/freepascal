{
    This file contains the OS independent declarations of the system unit

    This file is part of the Free Pascal Run time library.
    Copyright (c) 1999-2005 by the Free Pascal development team

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{****************************************************************************
                             Needed switches
****************************************************************************}

{$I-,Q-,H-,R-,V-}
{$mode objfpc}
{$modeswitch advancedrecords}

{ At least 2.4.0 is required }
{$if defined(VER1) or defined(VER2_0) or defined(VER2_2) }
  {$fatal You need at least FPC 2.4.0 to build this version of FPC}
{$endif}

{ Using inlining for small system functions/wrappers }
{$inline on}
{$define SYSTEMINLINE}

{ don't use FPU registervariables on the i386 and i8086 }
{$if defined(CPUI386) or defined(CPUI8086)}
  {$maxfpuregisters 0}
{$endif CPUI386 or CPUI8086}

{ the assembler helpers need this}
{$ifdef CPUPOWERPC}
  {$goto+}
{$endif CPUPOWERPC}

{$ifdef CPUAVR}
  {$goto+}
{$endif CPUAVR}


{ needed for insert,delete,readln }
{$P+}
{ stack checking always disabled
  for system unit. This is because
  the startup code might not
  have been called yet when we
  get a stack error, this will
  cause big crashes
}
{$S-}

{$ifndef FPC_HAS_FEATURE_SUPPORT}
{$define FPC_HAS_FEATURE_HEAP}
{$define FPC_HAS_FEATURE_INITFINAL}
{$define FPC_HAS_FEATURE_RTTI}
{$define FPC_HAS_FEATURE_CLASSES}
{$define FPC_HAS_FEATURE_EXCEPTIONS}
{$define FPC_HAS_FEATURE_EXITCODE}
{$define FPC_HAS_FEATURE_ANSISTRINGS}
{$define FPC_HAS_FEATURE_WIDESTRINGS}
{$define FPC_HAS_FEATURE_TEXTIO}
{$define FPC_HAS_FEATURE_CONSOLEIO}
{$define FPC_HAS_FEATURE_FILEIO}
{$define FPC_HAS_FEATURE_RANDOM}
{$define FPC_HAS_FEATURE_VARIANTS}
{$define FPC_HAS_FEATURE_OBJECTS}
{$define FPC_HAS_FEATURE_DYNARRAYS}
{$define FPC_HAS_FEATURE_THREADING}
{$define FPC_HAS_FEATURE_COMMANDARGS}
{$define FPC_HAS_FEATURE_PROCESSES}
{$define FPC_HAS_FEATURE_STACKCHECK}
{$define FPC_HAS_FEATURE_DYNLIBS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$define FPC_HAS_FEATURE_OBJECTIVEC1}
{$define FPC_HAS_FEATURE_UNICODESTRINGS}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$endif FPC_HAS_FEATURE_SUPPORT}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ bootstrapping, the starting compiler cannot be a downstripped compiler without unicodestring support }
{$ifdef VER2_6}
{$define FPC_HAS_FEATURE_UNICODESTRINGS}
{$endif VER2_6}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{****************************************************************************
                         Global Types and Constants
****************************************************************************}

{ some values which are used in RTL for TSystemCodePage type }
const
  CP_ACP     = 0;     // default to ANSI code page
  CP_OEMCP   = 1;     // default to OEM (console) code page
  CP_UTF16   = 1200;  // utf-16
  CP_UTF16BE = 1201;  // unicodeFFFE
  CP_UTF7    = 65000; // utf-7
  CP_UTF8    = 65001; // utf-8
  CP_ASCII   = 20127; // us-ascii
  CP_NONE    = $FFFF; // rawbytestring encoding

Type
  { The compiler has all integer types defined internally. Here
    we define only aliases }
  DWord    = LongWord;
  Cardinal = LongWord;
  Integer  = SmallInt;
  UInt64   = QWord;

  { moved here from psystem.pas
    Delphi allows chose of overloaded procedure depending
    on Real <-> Double, so use type here, see also tw7425.pp (FK) }
{$ifndef FPUNONE}
  Real = type Double;
{$endif}

{ Can be individually defined/undefined on a per-platform basis }
{ define FLOAT_ASCII_FALLBACK}

{$ifdef CPUI386}
  {$define CPU32}

  {$define DEFAULT_EXTENDED}

  {$define SUPPORT_SINGLE}
  {$define SUPPORT_DOUBLE}
  {$define SUPPORT_EXTENDED}
  {$define SUPPORT_COMP}

  {$ifndef FPUNONE}
    ValReal = Extended;
  {$endif}

  {$ifndef VER2_6}
  FarPointer = NearFsPointer;
  {$endif}
{$endif CPUI386}

{$ifdef CPUI8086}
  {$define CPU16}

  {$define DEFAULT_EXTENDED}

  {$define SUPPORT_SINGLE}
  {$define SUPPORT_DOUBLE}
  {$define SUPPORT_EXTENDED}
  {$define SUPPORT_COMP}

  {$ifndef FPUNONE}
    ValReal = Extended;
  {$endif}

  {$define MOVE_HAS_SIZEUINT_COUNT}
  {$define FILLCHAR_HAS_SIZEUINT_COUNT}

  {$if defined(FPC_MM_TINY)}
    {$define FPC_X86_CODE_NEAR}
    {$define FPC_X86_DATA_NEAR}
  {$elseif defined(FPC_MM_SMALL)}
    {$define FPC_X86_CODE_NEAR}
    {$define FPC_X86_DATA_NEAR}
  {$elseif defined(FPC_MM_MEDIUM)}
    {$define FPC_X86_CODE_FAR}
    {$define FPC_X86_DATA_NEAR}
  {$elseif defined(FPC_MM_COMPACT)}
    {$define FPC_X86_CODE_NEAR}
    {$define FPC_X86_DATA_FAR}
  {$elseif defined(FPC_MM_LARGE)}
    {$define FPC_X86_CODE_FAR}
    {$define FPC_X86_DATA_FAR}
  {$elseif defined(FPC_MM_HUGE)}
    {$define FPC_X86_CODE_FAR}
    {$define FPC_X86_DATA_HUGE}
  {$else}
    {$fatal No memory model defined}
  {$endif}
{$endif CPUI8086}

{$ifdef CPUX86_64}
{$ifdef FPC_HAS_TYPE_EXTENDED}
  { win64 doesn't support the legacy fpu }
  {$define DEFAULT_EXTENDED}
  {$define SUPPORT_EXTENDED}
  {$define SUPPORT_COMP}
  {$ifndef FPUNONE}
    ValReal = Extended;
  {$endif}
{$else FPC_HAS_TYPE_EXTENDED}
  {$define DEFAULT_DOUBLE}
  {$ifndef FPUNONE}
    ValReal = Double;
  {$endif}

  { map comp to int64, but this doesn't mean we compile the comp support in! }
  Comp = Int64;
  PComp = ^Comp;
{$endif FPC_HAS_TYPE_EXTENDED}

  {$define SUPPORT_SINGLE}
  {$define SUPPORT_DOUBLE}

  {$ifndef VER2_6}
  FarPointer = Pointer;
  {$endif}
{$endif CPUX86_64}

{$ifdef CPUM68K}
  {$define DEFAULT_DOUBLE}
<<<<<<< HEAD
<<<<<<< HEAD

  {$ifdef FPUSOFT}
    {$define FPC_INCLUDE_SOFTWARE_MOD_DIV}
    {$define FPC_INCLUDE_SOFTWARE_MUL}
  {$endif}

  { m68k int64 shl/shr uses soft helper for non constaznt values }
  {$define FPC_INCLUDE_SOFTWARE_SHIFT_INT64}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  {$define SUPPORT_SINGLE}
  {$define SUPPORT_DOUBLE}

<<<<<<< HEAD
<<<<<<< HEAD
  {$ifndef FPUNONE}
    ValReal = Real;
  {$endif}
=======
  ValReal = Real;
>>>>>>> graemeg/fixes_2_2
=======
  ValReal = Real;
>>>>>>> origin/fixes_2_2

  { Comp type does not exist on fpu }
  Comp    = int64;
  PComp = ^Comp;

  FarPointer = Pointer;
{$endif CPUM68K}

{$ifdef CPUPOWERPC}
  {$define DEFAULT_DOUBLE}

  {$ifndef FPUNONE}
    {$define SUPPORT_SINGLE}
    {$define SUPPORT_DOUBLE}

    {$define FPC_INCLUDE_SOFTWARE_INT64_TO_DOUBLE}

    ValReal = Double;
  {$endif}

  { map comp to int64, but this doesn't mean we compile the comp support in! }
  Comp = Int64;
  PComp = ^Comp;

  FarPointer = Pointer;
{$endif CPUPOWERPC}

{$ifdef CPUSPARC}
  {$define DEFAULT_DOUBLE}

  {$define SUPPORT_SINGLE}
  {$define SUPPORT_DOUBLE}

  {$define FPC_INCLUDE_SOFTWARE_SHIFT_INT64}

  {$ifndef FPUNONE}
    ValReal = Double;
  {$endif}

  { map comp to int64, but this doesn't mean we compile the comp support in! }
  Comp = Int64;
  PComp = ^Comp;

  FarPointer = Pointer;
{$endif CPUSPARC}

{$if defined(CPUMIPS32) or defined(CPUMIPSEL32)}
  {$define DEFAULT_DOUBLE}

  {$define SUPPORT_SINGLE}
  {$define SUPPORT_DOUBLE}

  {$define FPC_INCLUDE_SOFTWARE_SHIFT_INT64}

  {$ifndef FPUNONE}
    {$define FPC_INCLUDE_SOFTWARE_INT64_TO_DOUBLE}
    ValReal = Double;
  {$endif}

  { map comp to int64, but this doesn't mean we compile the comp support in! }
  Comp = Int64;
  PComp = ^Comp;

  FarPointer = Pointer;
{$endif CPUMIPS32}


{$ifdef CPUARM}
  {$define DEFAULT_DOUBLE}

  {$define SUPPORT_SINGLE}
  {$define SUPPORT_DOUBLE}

  {$define FPC_INCLUDE_SOFTWARE_MOD_DIV}
  {$define FPC_INCLUDE_SOFTWARE_SHIFT_INT64}
  {$define FPC_INCLUDE_SOFTWARE_INT64_TO_DOUBLE}

  {$ifndef FPUNONE}
    ValReal = Real;
  {$endif}

  { map comp to int64, but this doesn't mean we compile the comp support in! }
  Comp = Int64;
  PComp = ^Comp;

  FarPointer = Pointer;
{$endif CPUARM}

{$ifdef CPUAVR}
  {$define DEFAULT_SINGLE}

  {$define FPC_INCLUDE_SOFTWARE_MOD_DIV}
  {$define FPC_INCLUDE_SOFTWARE_MUL}
  {$define FPC_INCLUDE_SOFTWARE_SHIFT_INT64}

  {$ifndef FPUNONE}
    {$define SUPPORT_SINGLE}
    {$define SUPPORT_DOUBLE}

    {$define FPC_INCLUDE_SOFTWARE_INT64_TO_DOUBLE}

    ValReal = Real;
  {$endif}

  { map comp to int64, but this doesn't mean we compile the comp support in! }
  Comp = Int64;
  PComp = ^Comp;

  FarPointer = Pointer;
{$endif CPUAVR}

{$ifdef CPUAARCH64}
  {$define DEFAULT_DOUBLE}

  {$define SUPPORT_SINGLE}
  {$define SUPPORT_DOUBLE}

  ValReal = Double;

  { map comp to int64, but this doesn't mean we compile the comp support in! }
  Comp = Int64;
  PComp = ^Comp;

  FarPointer = Pointer;
{$endif CPUAARCH64}


{$ifdef CPU64}
  SizeInt = Int64;
  SizeUInt = QWord;
  PtrInt = Int64;
  PtrUInt = QWord;
  ValSInt = int64;
  ValUInt = qword;
  CodePointer = Pointer;
  CodePtrInt = PtrInt;
  CodePtrUInt = PtrUInt;
{$endif CPU64}

{$ifdef CPU32}
  SizeInt = Longint;
  SizeUInt = DWord;
  PtrInt = Longint;
  PtrUInt = DWord;
  ValSInt = Longint;
  ValUInt = Cardinal;
  CodePointer = Pointer;
  CodePtrInt = PtrInt;
  CodePtrUInt = PtrUInt;
{$endif CPU32}

{$ifdef CPU16}
  SizeInt = Integer;
  SizeUInt = Word;
  {$if defined(FPC_X86_DATA_FAR) or defined(FPC_X86_DATA_HUGE)}
    PtrInt = Longint;
    PtrUInt = DWord;
  {$else}
    PtrInt = Integer;
    PtrUInt = Word;
  {$endif}
  {$if defined(FPC_X86_CODE_FAR)}
    CodePointer = FarPointer;
    CodePtrInt = Longint;
    CodePtrUInt = DWord;
  {$elseif defined(FPC_X86_CODE_NEAR)}
    CodePointer = NearPointer;
    CodePtrInt = Integer;
    CodePtrUInt = Word;
  {$else}
    CodePointer = Pointer;
    CodePtrInt = PtrInt;
    CodePtrUInt = PtrUInt;
  {$endif}
  ValSInt = Integer;
  ValUInt = Word;
{$endif CPU16}

{$if defined(VER2) or defined(VER3_0)}
{$if defined(CPU16)}
{$define CPUINT16}
{$elseif defined(CPU32)}
{$define CPUINT32}
{$elseif defined(CPU64)}
{$define CPUINT64}
{$endif defined(CPU64)}
{$endif defined(VER2) or defined(VER3_0)}

{$if defined(CPUINT8)}
  ALUSInt = ShortInt;
  ALUUInt = Byte;
{$elseif defined(CPUINT16)}
  ALUSInt = SmallInt;
  ALUUInt = Word;
{$elseif defined(CPUINT32)}
  ALUSInt = Longint;
  ALUUInt = DWord;
{$elseif defined(CPUINT64)}
  ALUSInt = Int64;
  ALUUInt = QWord;
{$endif defined(CPUINT64)}

  { NativeInt and NativeUInt are Delphi compatibility types. Even though Delphi
    has IntPtr and UIntPtr, the Delphi documentation for NativeInt states that
    'The size of NativeInt is equivalent to the size of the pointer on the
    current platform'. Because of the misleading names, these types shouldn't be
    used in the FPC RTL. Note that on i8086 their size changes between 16-bit
    and 32-bit according to the memory model, so they're not really a 'native
    int' type there at all. }
  NativeInt  = PtrInt;
  NativeUInt = PtrUInt;

  Int8    = ShortInt;
  Int16   = SmallInt;
  Int32   = Longint;
  IntPtr  = PtrInt;
  UInt8   = Byte;
  UInt16  = Word;
  UInt32  = Cardinal;
  UIntPtr = PtrUInt;

{ Zero - terminated strings }
  PChar               = ^Char;
  PPChar              = ^PChar;
  PPPChar             = ^PPChar;

  { AnsiChar is equivalent of Char, so we need
    to use type renamings }
  TAnsiChar           = Char;
  AnsiChar            = Char;
  PAnsiChar           = PChar;
  PPAnsiChar          = PPChar;
  PPPAnsiChar         = PPPChar;

  UCS4Char            = type 0..$10ffff;
  PUCS4Char           = ^UCS4Char;
{$ifdef CPU16}
  TUCS4CharArray      = array[0..32767 div sizeof(UCS4Char)-1] of UCS4Char;
{$else CPU16}
  TUCS4CharArray      = array[0..$effffff] of UCS4Char;
{$endif CPU16}
  PUCS4CharArray      = ^TUCS4CharArray;
  UCS4String          = array of UCS4Char;

{$ifdef FPC_HAS_CPSTRING}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  UTF8String          = type AnsiString(CP_UTF8);
=======
  UTF8String          = String<65001>;
>>>>>>> graemeg/cpstrnew
=======
  UTF8String          = String<65001>;
>>>>>>> graemeg/cpstrnew
=======
  UTF8String          = String<65001>;
>>>>>>> graemeg/cpstrnew
=======
  UTF8String          = String<65001>;
>>>>>>> origin/cpstrnew
{$else FPC_HAS_CPSTRING}
  UTF8String          = type ansistring;
{$endif FPC_HAS_CPSTRING}
  PUTF8String         = ^UTF8String;

{$ifdef FPC_HAS_CPSTRING}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  RawByteString       = type AnsiString(CP_NONE);
=======
  RawByteString       = String<$ffff>;
>>>>>>> graemeg/cpstrnew
=======
  RawByteString       = String<$ffff>;
>>>>>>> graemeg/cpstrnew
=======
  RawByteString       = String<$ffff>;
>>>>>>> graemeg/cpstrnew
=======
  RawByteString       = String<$ffff>;
>>>>>>> origin/cpstrnew
{$else FPC_HAS_CPSTRING}
  RawByteString       = ansistring;
{$endif FPC_HAS_CPSTRING}

  HRESULT             = type Longint;
{$ifndef FPUNONE}
  TDateTime           = type Double;
  TDate               = type TDateTime;
  TTime               = type TDateTime;
{$endif}
  TError               = type Longint;

{$ifndef FPUNONE}
  PSingle             = ^Single;
  PDouble             = ^Double;
  PExtended           = ^Extended;

  PPDouble            = ^PDouble;
{$endif}
  PCurrency           = ^Currency;
{$ifdef SUPPORT_COMP}
  PComp               = ^Comp;
{$endif SUPPORT_COMP}

  PSmallInt           = ^Smallint;
  PShortInt           = ^Shortint;
  PInteger            = ^Integer;
  PByte               = ^Byte;
  PWord               = ^word;
  PDWord              = ^DWord;
  PLongWord           = ^LongWord;
  PLongint            = ^Longint;
  PCardinal           = ^Cardinal;
  PQWord              = ^QWord;
  PInt64              = ^Int64;
  PUInt64             = ^UInt64;
  PPtrInt             = ^PtrInt;
  PPtrUInt            = ^PtrUInt;
  PSizeInt            = ^SizeInt;
  PSizeUInt           = ^SizeUInt;

  PPByte              = ^PByte;
  PPLongint           = ^PLongint;

  PPByte              = ^PByte;
  PPLongint           = ^PLongint;

  PPByte              = ^PByte;
  PPLongint           = ^PLongint;

  PPByte              = ^PByte;
  PPLongint           = ^PLongint;

  PPByte              = ^PByte;
  PPLongint           = ^PLongint;

  PPointer            = ^Pointer;
  PPPointer           = ^PPointer;

  PCodePointer        = ^CodePointer;
  PPCodePointer       = ^PCodePointer;

  PBoolean            = ^Boolean;
  PWordBool           = ^WordBool;
  PLongBool           = ^LongBool;

  PNativeInt 	      = ^NativeInt;
  PNativeUInt	      = ^NativeUint;
  pInt8   	      = PShortInt;
  pInt16  	      = PSmallint;
  pInt32  	      = PLongint;
  PIntPtr 	      = PPtrInt;
  pUInt8  	      = PByte;
  pUInt16 	      = PWord;
  pUInt32 	      = PDWord;
  PUintPtr	      = PPtrUInt;

  PShortString        = ^ShortString;
  PAnsiString         = ^AnsiString;

{$ifndef FPUNONE}
  PDate               = ^TDateTime;
<<<<<<< HEAD
<<<<<<< HEAD
  PDateTime           = ^TDateTime;
{$endif}
=======
  PDateTime	      = ^TDateTime;
>>>>>>> graemeg/fixes_2_2
=======
  PDateTime	      = ^TDateTime;
>>>>>>> origin/fixes_2_2
  PError              = ^TError;
  PVariant            = ^Variant;
  POleVariant         = ^OleVariant;

  PWideChar           = ^WideChar;
  PPWideChar          = ^PWideChar;
  PPPWideChar         = ^PPWideChar;
  WChar               = Widechar;
  UCS2Char            = WideChar;
  PUCS2Char           = PWideChar;
  PWideString         = ^WideString;

  UnicodeChar         = WideChar;
  PUnicodeChar        = ^UnicodeChar;
  PUnicodeString      = ^UnicodeString;

  TSystemCodePage     = Word;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef VER2_6}
  { the size of textrec/filerec is hardcoded in the 2.6 compiler binary }
  {$define FPC_ANSI_TEXTFILEREC}
{$endif}

  TFileTextRecChar    = {$if defined(FPC_ANSI_TEXTFILEREC) or not(defined(FPC_HAS_FEATURE_WIDESTRINGS))}AnsiChar{$else}UnicodeChar{$endif};
  PFileTextRecChar    = ^TFileTextRecChar;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { Needed for fpc_get_output }
  PText               = ^Text;
>>>>>>> graemeg/cpstrnew

  TTextLineBreakStyle = (tlbsLF,tlbsCRLF,tlbsCR);

{ procedure type }
  TProcedure  = Procedure;

{ platform-dependent types }
{$i sysosh.inc}

<<<<<<< HEAD
<<<<<<< HEAD
{ platform-dependent defines }
{$i rtldefs.inc}

{*****************************************************************************
                   TextRec/FileRec exported to allow compiler to take size
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_FILEIO}
{$i filerec.inc}
{$endif FPC_HAS_FEATURE_FILEIO}

{$i textrec.inc}


type
  { Needed for fpc_get_output }
  PText               = ^Text;

=======
type
>>>>>>> graemeg/fixes_2_2
=======
type
>>>>>>> origin/fixes_2_2
  TEntryInformation = record
    InitFinalTable : Pointer;
    ThreadvarTablesTable : Pointer;
    asm_exit : Procedure;stdcall;
    PascalMain : Procedure;stdcall;
    valgrind_used : boolean;
  end;


const
{ Maximum value of the biggest signed and unsigned integer type available}
  MaxSIntValue = High(ValSInt);
  MaxUIntValue = High(ValUInt);

{ max. values for longint and int}
  maxLongint  = $7fffffff;
  maxSmallint = 32767;

  maxint   = maxsmallint;

type
{$ifdef CPU16}
  IntegerArray  = array[0..maxSmallint div sizeof(Integer)-1] of Integer;
{$else CPU16}
  IntegerArray  = array[0..$effffff] of Integer;
{$endif CPU16}
  PIntegerArray = ^IntegerArray;
{$ifdef CPU16}
  PointerArray = array [0..32767 div sizeof(Pointer)-1] of Pointer;
{$else CPU16}
  PointerArray = array [0..512*1024*1024-2] of Pointer;
{$endif CPU16}
  PPointerArray = ^PointerArray;

  TBoundArray = array of SizeInt;

{$ifdef CPU16}
  TPCharArray = packed array[0..(MaxSmallint div SizeOf(PChar))-1] of PChar;
{$else CPU16}
  TPCharArray = packed array[0..(MaxLongint div SizeOf(PChar))-1] of PChar;
{$endif CPU16}
  PPCharArray = ^TPCharArray;

(* CtrlBreak set to true signalizes Ctrl-Break signal, otherwise Ctrl-C. *)
(* Return value of true means that the signal has been processed, false  *)
(* means that default handling should be used. *)
  TCtrlBreakHandler = function (CtrlBreak: boolean): boolean;

const
{$ifdef cpui386}
  { Always i386 or newer }
  Test8086 : byte = 2;
  { Always 387 or newer. Emulated if needed. }
  Test8087 : byte = 3;
  { will be detected at startup }
  has_sse_support : boolean = false;
  has_sse2_support : boolean = false;
  has_sse3_support : boolean = false;
  has_mmx_support : boolean = false;
{$endif cpui386}
{$ifdef cpui8086}
  { will be detected at startup }
  { 0=8086/8088/80186/80188/NEC V20/NEC V30, 1=80286, 2=80386 or newer }
  Test8086 : byte = 0; public name '__Test8086';
  { will be detected at startup }
  { 0=NO FPU, 1=8087, 2=80287, 3=80387 or newer }
  Test8087 : byte = 0;
  { will be detected at startup }
  has_sse_support : boolean = false;
  has_mmx_support : boolean = false;
{$endif cpui8086}
{$ifdef cpum68k}
  Test68000 : byte = 0;      { Must be determined at startup for both }
  Test68881 : byte = 0;
{$endif cpum68k}

{ max level in dumping on error }
  Max_Frame_Dump : Word = 8;

{ Exit Procedure handling consts and types  }
  ExitProc : codepointer = nil;
  Erroraddr: codepointer = nil;
  Errorcode: Word    = 0;

{ file input modes }
  fmClosed = $D7B0;
  fmInput  = $D7B1;
  fmOutput = $D7B2;
  fmInOut  = $D7B3;
  fmAppend = $D7B4;
  Filemode : byte = 2;
(* Value should be changed during system initialization as appropriate. *)

  { assume that this program will not spawn other threads, when the
    first thread is started the following constants need to be filled }
  IsMultiThread : longbool = FALSE;
  { set to true, if a threading helper is used before a thread
    manager has been installed }
  ThreadingAlreadyUsed : boolean = FALSE;
  { Indicates if there was an error }
  StackError : boolean = FALSE;
<<<<<<< HEAD
  InitProc : CodePointer = nil;
=======
  InitProc : Pointer = nil;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { compatibility }
  ModuleIsLib : Boolean = FALSE;
  ModuleIsPackage : Boolean = FALSE;
  ModuleIsCpp : Boolean = FALSE;

var
  ExitCode    : Longint; public name 'operatingsystem_result';
  RandSeed    : Cardinal;
  { Delphi compatibility }

{$ifdef FPC_HAS_FEATURE_DYNLIBS}
  IsLibrary : boolean = false; public name 'operatingsystem_islibrary';
{$else FPC_HAS_FEATURE_DYNLIBS}
const
  IsLibrary = false;
var
{$endif FPC_HAS_FEATURE_DYNLIBS}
  IsConsole : boolean = false; public name 'operatingsystem_isconsole';
  NoErrMsg: Boolean platform = False; // For Delphi compatibility, not used in FPC.
  FirstDotAtFileNameStartIsExtension : Boolean = False;

  DefaultSystemCodePage,
  DefaultUnicodeCodePage,
  { the code page to use when sending paths/file names to OS file system API
    calls using single byte strings, and to interpret the results gotten back
    from such API calls }
  DefaultFileSystemCodePage,
  { the code page to use to return file names from single byte file system calls
    in the RTL that return ansistrings (by default, same as a above) }
  DefaultRTLFileSystemCodePage,
  UTF8CompareLocale : TSystemCodePage;


  DefaultSystemCodePage,
  DefaultUnicodeCodePage,
  UTF8CompareLocale : TSystemCodePage;


  DefaultSystemCodePage,
  DefaultUnicodeCodePage,
  UTF8CompareLocale : TSystemCodePage;


  DefaultSystemCodePage,
  DefaultUnicodeCodePage,
  UTF8CompareLocale : TSystemCodePage;


  DefaultSystemCodePage,
  DefaultUnicodeCodePage,
  UTF8CompareLocale : TSystemCodePage;


{$ifndef HAS_CMDLINE}
{Value should be changed during system initialization as appropriate.}
var cmdline:Pchar=nil;
{$endif}

{$ifdef FPC_HAS_FEATURE_THREADING}
ThreadVar
{$else FPC_HAS_FEATURE_THREADING}
Var
{$endif FPC_HAS_FEATURE_THREADING}
  ThreadID    : TThreadID;
  { Standard In- and Output }
  ErrOutput,
  Output,
  Input,
  StdOut,
  StdErr      : Text;
  InOutRes    : Word;
  { Stack checking }
  StackBottom : Pointer;
  StackLength : SizeUInt;
  
Var
  WriteErrorsToStdErr : Boolean = True;

function StackTop: Pointer;

{ Numbers for routines that have compiler magic }
{$I innr.inc}


{****************************************************************************
                        Processor specific routines
****************************************************************************}

{$ifdef FPC_USE_LIBC}
  {$ifdef SYSTEMINLINE}
    {$define INLINEGENERICS}
  {$endif}
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
Procedure Move(const source;var dest;count:{$ifdef MOVE_HAS_SIZEUINT_COUNT}SizeUInt{$else}SizeInt{$endif});
Procedure FillChar(var x;count:{$ifdef FILLCHAR_HAS_SIZEUINT_COUNT}SizeUInt{$else}SizeInt{$endif};Value:Byte);
Procedure FillChar(var x;count:{$ifdef FILLCHAR_HAS_SIZEUINT_COUNT}SizeUInt{$else}SizeInt{$endif};Value:Boolean);
Procedure FillChar(var x;count:{$ifdef FILLCHAR_HAS_SIZEUINT_COUNT}SizeUInt{$else}SizeInt{$endif};Value:Char);
procedure FillByte(var x;count:{$ifdef FILLCHAR_HAS_SIZEUINT_COUNT}SizeUInt{$else}SizeInt{$endif};value:byte);
Procedure FillWord(var x;count:SizeInt;Value:Word);
procedure FillDWord(var x;count:SizeInt;value:DWord);
procedure FillQWord(var x;count:SizeInt;value:QWord);
function  IndexChar(const buf;len:SizeInt;b:char):SizeInt;
function  IndexByte(const buf;len:SizeInt;b:byte):SizeInt;
function  Indexword(const buf;len:SizeInt;b:word):SizeInt;
function  IndexDWord(const buf;len:SizeInt;b:DWord):SizeInt;
function  IndexQWord(const buf;len:SizeInt;b:QWord):SizeInt;
function  CompareChar(const buf1,buf2;len:SizeInt):SizeInt;
function  CompareByte(const buf1,buf2;len:SizeInt):SizeInt;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure Move(const source;var dest;count:SizeInt);{$ifdef INLINEGENERICS}inline;{$endif}
Procedure FillChar(var x;count:SizeInt;Value:Byte);{$ifdef INLINEGENERICS}inline;{$endif}
Procedure FillChar(var x;count:SizeInt;Value:Boolean);{$ifdef SYSTEMINLINE}inline;{$endif}
Procedure FillChar(var x;count:SizeInt;Value:Char);{$ifdef SYSTEMINLINE}inline;{$endif}
procedure FillByte(var x;count:SizeInt;value:byte);{$ifdef SYSTEMINLINE}inline;{$endif}
Procedure FillWord(var x;count:SizeInt;Value:Word);
procedure FillDWord(var x;count:SizeInt;value:DWord);
function  IndexChar(const buf;len:SizeInt;b:char):SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function  IndexByte(const buf;len:SizeInt;b:byte):SizeInt;{$ifdef INLINEGENERICS}inline;{$endif}
function  Indexword(const buf;len:SizeInt;b:word):SizeInt;
function  IndexDWord(const buf;len:SizeInt;b:DWord):SizeInt;
function  CompareChar(const buf1,buf2;len:SizeInt):SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function  CompareByte(const buf1,buf2;len:SizeInt):SizeInt;{$ifdef INLINEGENERICS}inline;{$endif}
>>>>>>> graemeg/fixes_2_2
function  CompareWord(const buf1,buf2;len:SizeInt):SizeInt;
function  CompareDWord(const buf1,buf2;len:SizeInt):SizeInt;
procedure MoveChar0(const buf1;var buf2;len:SizeInt);
function  IndexChar0(const buf;len:SizeInt;b:char):SizeInt;
function  CompareChar0(const buf1,buf2;len:SizeInt):SizeInt;
procedure prefetch(const mem);[internproc:fpc_in_prefetch_var];
<<<<<<< HEAD
<<<<<<< HEAD
procedure ReadBarrier;
procedure ReadDependencyBarrier;
procedure ReadWriteBarrier;
procedure WriteBarrier;
=======
=======
>>>>>>> origin/fixes_2_2
procedure ReadBarrier;{$ifdef INLINEGENERICS}inline;{$endif}
procedure ReadDependencyBarrier;{$ifdef INLINEGENERICS}inline;{$endif}
procedure ReadWriteBarrier;{$ifdef INLINEGENERICS}inline;{$endif}
procedure WriteBarrier;{$ifdef INLINEGENERICS}inline;{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{****************************************************************************
                          Math Routines
****************************************************************************}

Function  lo(B: Byte):Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
Function  hi(b : Byte) : Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
Function  lo(i : Integer) : byte;  [INTERNPROC: fpc_in_lo_Word];
Function  lo(w : Word) : byte;     [INTERNPROC: fpc_in_lo_Word];
Function  lo(l : Longint) : Word;  [INTERNPROC: fpc_in_lo_long];
Function  lo(l : DWord) : Word;    [INTERNPROC: fpc_in_lo_long];
Function  lo(i : Int64) : DWord;   [INTERNPROC: fpc_in_lo_qword];
Function  lo(q : QWord) : DWord;   [INTERNPROC: fpc_in_lo_qword];
Function  hi(i : Integer) : byte;  [INTERNPROC: fpc_in_hi_Word];
Function  hi(w : Word) : byte;     [INTERNPROC: fpc_in_hi_Word];
Function  hi(l : Longint) : Word;  [INTERNPROC: fpc_in_hi_long];
Function  hi(l : DWord) : Word;    [INTERNPROC: fpc_in_hi_long];
Function  hi(i : Int64) : DWord;   [INTERNPROC: fpc_in_hi_qword];
Function  hi(q : QWord) : DWord;   [INTERNPROC: fpc_in_hi_qword];

Function swap (X : Word) : Word;{$ifdef SYSTEMINLINE}inline;{$endif}[internconst:fpc_in_const_swap_word];
Function Swap (X : Integer) : Integer;{$ifdef SYSTEMINLINE}inline;{$endif}[internconst:fpc_in_const_swap_word];
Function swap (X : Longint) : Longint;{$ifdef SYSTEMINLINE}inline;{$endif}[internconst:fpc_in_const_swap_long];
Function Swap (X : Cardinal) : Cardinal;{$ifdef SYSTEMINLINE}inline;{$endif}[internconst:fpc_in_const_swap_long];
Function Swap (X : QWord) : QWord;{$ifdef SYSTEMINLINE}inline;{$endif}[internconst:fpc_in_const_swap_qword];
Function swap (X : Int64) : Int64;{$ifdef SYSTEMINLINE}inline;{$endif}[internconst:fpc_in_const_swap_qword];

Function Align (Addr : PtrUInt; Alignment : PtrUInt) : PtrUInt;{$ifdef SYSTEMINLINE}inline;{$endif}
Function Align (Addr : Pointer; Alignment : PtrUInt) : Pointer;{$ifdef SYSTEMINLINE}inline;{$endif}

{$ifdef FPC_HAS_FEATURE_RANDOM}
Function  Random(l:longint):longint;
Function  Random(l:int64):int64;
{$ifndef FPUNONE}
Function  Random: extended;
{$endif}
Procedure Randomize;
{$endif FPC_HAS_FEATURE_RANDOM}

{$ifdef FPC_HAS_INTERNAL_ABS_LONG}
{$define FPC_SYSTEM_HAS_ABS_LONGINT}
Function abs(l:longint):longint;[internproc:fpc_in_abs_long];
{$else FPC_HAS_INTERNAL_ABS_LONG}
Function abs(l:Longint):Longint;[internconst:fpc_in_const_abs];{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_ABS_LONG}
{$ifdef FPC_HAS_INTERNAL_ABS_INT64}
{$define FPC_SYSTEM_HAS_ABS_INT64}
Function abs(l:Int64):Int64;[internproc:fpc_in_abs_long];
{$else FPC_HAS_INTERNAL_ABS_INT64}
Function abs(l:Int64):Int64;[internconst:fpc_in_const_abs];{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_ABS_INT64}
Function sqr(l:Longint):Longint;[internconst:fpc_in_const_sqr];{$ifdef SYSTEMINLINE}inline;{$endif}
Function sqr(l:Int64):Int64;[internconst:fpc_in_const_sqr];{$ifdef SYSTEMINLINE}inline;{$endif}
Function sqr(l:QWord):QWord;[internconst:fpc_in_const_sqr];{$ifdef SYSTEMINLINE}inline;{$endif}
Function odd(l:Longint):Boolean;[internconst:fpc_in_const_odd];{$ifdef SYSTEMINLINE}inline;{$endif}
Function odd(l:Longword):Boolean;[internconst:fpc_in_const_odd];{$ifdef SYSTEMINLINE}inline;{$endif}
Function odd(l:Int64):Boolean;[internconst:fpc_in_const_odd];{$ifdef SYSTEMINLINE}inline;{$endif}
Function odd(l:QWord):Boolean;[internconst:fpc_in_const_odd];{$ifdef SYSTEMINLINE}inline;{$endif}

function SwapEndian(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function SwapEndian(const AValue: Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
function SwapEndian(const AValue: LongInt): LongInt;
function SwapEndian(const AValue: DWord): DWord;
function SwapEndian(const AValue: Int64): Int64;
function SwapEndian(const AValue: QWord): QWord;

function BEtoN(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function BEtoN(const AValue: Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
function BEtoN(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function BEtoN(const AValue: DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
function BEtoN(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
function BEtoN(const AValue: QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}

function LEtoN(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function LEtoN(const AValue: Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
function LEtoN(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function LEtoN(const AValue: DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
function LEtoN(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
function LEtoN(const AValue: QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}

function NtoBE(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoBE(const AValue: Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoBE(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoBE(const AValue: DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoBE(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoBE(const AValue: QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}

function NtoLE(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoLE(const AValue: Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoLE(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoLE(const AValue: DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoLE(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoLE(const AValue: QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}

{$ifdef FPC_HAS_INTERNAL_ROX}

{$if defined(cpux86_64) or defined(cpui386)}
{$define FPC_HAS_INTERNAL_ROX_BYTE}
{$define FPC_HAS_INTERNAL_ROX_WORD}
{$endif defined(cpux86_64) or defined(cpui386)}

{$if defined(cpux86_64) or defined(cpui386) or defined(arm) or defined(powerpc) or defined(powerpc64) or defined(cpuaarch64)}
{$define FPC_HAS_INTERNAL_ROX_DWORD}
{$endif defined(cpux86_64) or defined(cpui386) or defined(arm) or defined(powerpc) or defined(powerpc64) or defined(cpuaarch64)}

{$if defined(cpux86_64) or defined(powerpc64) or defined(cpuaarch64)}
{$define FPC_HAS_INTERNAL_ROX_QWORD}
{$endif defined(cpux86_64) or defined(powerpc64) or defined(cpuaarch64)}

{$endif FPC_HAS_INTERNAL_ROX}

{$ifdef FPC_HAS_INTERNAL_ROX_BYTE}
function RorByte(Const AValue : Byte): Byte;[internproc:fpc_in_ror_x];
function RorByte(Const AValue : Byte;const Dist : Byte): Byte;[internproc:fpc_in_ror_x_x];

function RolByte(Const AValue : Byte): Byte;[internproc:fpc_in_rol_x];
function RolByte(Const AValue : Byte;const Dist : Byte): Byte;[internproc:fpc_in_rol_x_x];
{$else FPC_HAS_INTERNAL_ROX_BYTE}
function RorByte(Const AValue : Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
function RorByte(Const AValue : Byte;const Dist : Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}

function RolByte(Const AValue : Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
function RolByte(Const AValue : Byte;const Dist : Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_ROX_BYTE}


{$ifdef FPC_HAS_INTERNAL_ROX_WORD}
function RorWord(Const AValue : Word): Word;[internproc:fpc_in_ror_x];
function RorWord(Const AValue : Word;const Dist : Byte): Word;[internproc:fpc_in_ror_x_x];

function RolWord(Const AValue : Word): Word;[internproc:fpc_in_rol_x];
function RolWord(Const AValue : Word;const Dist : Byte): Word;[internproc:fpc_in_rol_x_x];
{$else FPC_HAS_INTERNAL_ROX_WORD}
function RorWord(Const AValue : Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
function RorWord(Const AValue : Word;const Dist : Byte): Word;{$ifdef SYSTEMINLINE}inline;{$endif}

function RolWord(Const AValue : Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
function RolWord(Const AValue : Word;const Dist : Byte): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_ROX_WORD}


{$ifdef FPC_HAS_INTERNAL_ROX_DWORD}
function RorDWord(Const AValue : DWord): DWord;[internproc:fpc_in_ror_x];
function RorDWord(Const AValue : DWord;const Dist : Byte): DWord;[internproc:fpc_in_ror_x_x];

function RolDWord(Const AValue : DWord): DWord;[internproc:fpc_in_rol_x];
function RolDWord(Const AValue : DWord;const Dist : Byte): DWord;[internproc:fpc_in_rol_x_x];
{$else FPC_HAS_INTERNAL_ROX_DWORD}
function RorDWord(Const AValue : DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
function RorDWord(Const AValue : DWord;const Dist : Byte): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}

function RolDWord(Const AValue : DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
function RolDWord(Const AValue : DWord;const Dist : Byte): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_ROX_DWORD}


{$ifdef FPC_HAS_INTERNAL_ROX_QWORD}
function RorQWord(Const AValue : QWord): QWord;[internproc:fpc_in_ror_x];
function RorQWord(Const AValue : QWord;const Dist : Byte): QWord;[internproc:fpc_in_ror_x_x];

function RolQWord(Const AValue : QWord): QWord;[internproc:fpc_in_rol_x];
function RolQWord(Const AValue : QWord;const Dist : Byte): QWord;[internproc:fpc_in_rol_x_x];
{$else FPC_HAS_INTERNAL_ROX_QWORD}
function RorQWord(Const AValue : QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
function RorQWord(Const AValue : QWord;const Dist : Byte): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}

function RolQWord(Const AValue : QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
function RolQWord(Const AValue : QWord;const Dist : Byte): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_ROX_QWORD}

{$ifdef FPC_HAS_INTERNAL_SAR}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(cpux86_64) or defined(cpui386) or defined(mips) or defined(mipsel) or defined(sparc)}
{$define FPC_HAS_INTERNAL_SAR_BYTE}
{$define FPC_HAS_INTERNAL_SAR_WORD}
{$endif defined(cpux86_64) or defined(cpui386) or defined(mips) or defined(mipsel) or defined(sparc)}

{ currently, all supported CPUs have an internal 32 bit sar implementation }
{ $if defined(cpux86_64) or defined(cpui386) or defined(arm) or defined(powerpc) or defined(powerpc64) or defined(mips) or defined(mipsel)}
{$define FPC_HAS_INTERNAL_SAR_DWORD}
{ $endif defined(cpux86_64) or defined(cpui386) or defined(arm) or defined(powerpc) or defined(powerpc64) or defined(mips) or defined(mipsel)}

{$if defined(cpux86_64) or defined(powerpc64) or defined(cpuaarch64)}
{$define FPC_HAS_INTERNAL_SAR_QWORD}
{$endif defined(cpux86_64) or defined(powerpc64) or defined(cpuaarch64)}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$if defined(cpux86_64) or defined(cpui386)}
{$define FPC_HAS_INTERNAL_SAR_BYTE}
{$define FPC_HAS_INTERNAL_SAR_WORD}
{$endif defined(cpux86_64) or defined(cpui386)}

{ currently, all supported CPUs have an internal 32 bit sar implementation }
{ $if defined(cpux86_64) or defined(cpui386) or defined(arm) or defined(powerpc) or defined(powerpc64)}
{$define FPC_HAS_INTERNAL_SAR_DWORD}
{ $endif defined(cpux86_64) or defined(cpui386) or defined(arm) or defined(powerpc) or defined(powerpc64)}

{$if defined(cpux86_64) or defined(powerpc64)}
{$define FPC_HAS_INTERNAL_SAR_QWORD}
{$endif defined(cpux86_64) or defined(powerpc64)}
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

{$endif FPC_HAS_INTERNAL_SAR}

{$ifdef FPC_HAS_INTERNAL_SAR_BYTE}
function SarShortint(Const AValue : Shortint): Shortint;[internproc:fpc_in_sar_x];
function SarShortint(Const AValue : Shortint;Shift : Byte): Shortint;[internproc:fpc_in_sar_x_y];
{$else FPC_HAS_INTERNAL_ROX_BYTE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function SarShortint(Const AValue : Shortint;const Shift : Byte = 1): Shortint;
=======
function SarShortint(Const AValue : Shortint;Shift : Byte = 1): Shortint;
>>>>>>> graemeg/cpstrnew
=======
function SarShortint(Const AValue : Shortint;Shift : Byte = 1): Shortint;
>>>>>>> graemeg/cpstrnew
=======
function SarShortint(Const AValue : Shortint;Shift : Byte = 1): Shortint;
>>>>>>> graemeg/cpstrnew
=======
function SarShortint(Const AValue : Shortint;Shift : Byte = 1): Shortint;
>>>>>>> origin/cpstrnew
{$endif FPC_HAS_INTERNAL_ROX_BYTE}

{$ifdef FPC_HAS_INTERNAL_SAR_WORD}
function SarSmallint(Const AValue : Smallint): Smallint;[internproc:fpc_in_sar_x];
function SarSmallint(Const AValue : Smallint;Shift : Byte): Smallint;[internproc:fpc_in_sar_x_y];
{$else FPC_HAS_INTERNAL_SAR_WORD}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function SarSmallint(Const AValue : Smallint;const Shift : Byte = 1): Smallint;
=======
function SarSmallint(Const AValue : Smallint;Shift : Byte = 1): Smallint;
>>>>>>> graemeg/cpstrnew
=======
function SarSmallint(Const AValue : Smallint;Shift : Byte = 1): Smallint;
>>>>>>> graemeg/cpstrnew
=======
function SarSmallint(Const AValue : Smallint;Shift : Byte = 1): Smallint;
>>>>>>> graemeg/cpstrnew
=======
function SarSmallint(Const AValue : Smallint;Shift : Byte = 1): Smallint;
>>>>>>> origin/cpstrnew
{$endif FPC_HAS_INTERNAL_SAR_WORD}

{$ifdef FPC_HAS_INTERNAL_SAR_DWORD}
function SarLongint(Const AValue : Longint): Longint;[internproc:fpc_in_sar_x];
function SarLongint(Const AValue : Longint;Shift : Byte): Longint;[internproc:fpc_in_sar_x_y];
{$else FPC_HAS_INTERNAL_SAR_DWORD}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function SarLongint(Const AValue : Longint;const Shift : Byte = 1): Longint;
{$endif FPC_HAS_INTERNAL_SAR_DWORD}

function SarInt64(Const AValue : Int64): Int64;[internproc:fpc_in_sar_x];
function SarInt64(Const AValue : Int64;Shift : Byte): Int64;[internproc:fpc_in_sar_x_y];
{$ifndef FPC_HAS_INTERNAL_SAR_QWORD}
function fpc_SarInt64(Const AValue : Int64;const Shift : Byte): Int64;compilerproc;
{$endif FPC_HAS_INTERNAL_SAR_QWORD}

{$ifdef FPC_HAS_INTERNAL_BSF}
{$if defined(cpui386) or defined(cpux86_64) or defined(cpuarm) or defined(cpuaarch64) or defined(cpupowerpc32) or defined(cpupowerpc64)}
{$define FPC_HAS_INTERNAL_BSF_BYTE}
{$define FPC_HAS_INTERNAL_BSF_WORD}
{$define FPC_HAS_INTERNAL_BSF_DWORD}
{$endif}
{$if defined(cpux86_64) or defined(cpuaarch64) or defined(cpupowerpc64)}
{$define FPC_HAS_INTERNAL_BSF_QWORD}
{$endif}
{$endif}

{$ifdef FPC_HAS_INTERNAL_BSR}
{$if defined(cpui386) or defined(cpux86_64) or defined(cpuarm) or defined(cpuaarch64) or defined(cpupowerpc32) or defined(cpupowerpc64)}
{$define FPC_HAS_INTERNAL_BSR_BYTE}
{$define FPC_HAS_INTERNAL_BSR_WORD}
{$define FPC_HAS_INTERNAL_BSR_DWORD}
{$endif}
{$if defined(cpux86_64) or defined(cpuaarch64) or defined(cpupowerpc64)}
{$define FPC_HAS_INTERNAL_BSR_QWORD}
{$endif}
{$endif}


{$ifdef FPC_HAS_INTERNAL_BSF_BYTE}
function BsfByte(Const AValue: Byte): Byte;[internproc:fpc_in_bsf_x];
{$else}
function BsfByte(Const AValue: Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_BSF_BYTE}
{$ifdef FPC_HAS_INTERNAL_BSR_BYTE}
function BsrByte(Const AValue: Byte): Byte;[internproc:fpc_in_bsr_x];
{$else}
function BsrByte(Const AValue: Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_BSR_BYTE}

{$ifdef FPC_HAS_INTERNAL_BSF_WORD}
function BsfWord(Const AValue: Word): cardinal;[internproc:fpc_in_bsf_x];
{$else}
function BsfWord(Const AValue: Word): {$ifdef CPU16}byte{$else}cardinal{$endif};{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_BSF_WORD}
{$ifdef FPC_HAS_INTERNAL_BSR_WORD}
function BsrWord(Const AValue: Word): cardinal;[internproc:fpc_in_bsr_x];
{$else}
function BsrWord(Const AValue: Word): {$ifdef CPU16}byte{$else}cardinal{$endif};{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_BSR_WORD}

{$ifdef FPC_HAS_INTERNAL_BSF_DWORD}
function BsfDWord(Const AValue : DWord): cardinal;[internproc:fpc_in_bsf_x];
{$else}
function BsfDWord(Const AValue : DWord): {$ifdef CPU16}byte{$else}cardinal{$endif};{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_BSF_DWORD}
{$ifdef FPC_HAS_INTERNAL_BSR_DWORD}
function BsrDWord(Const AValue : DWord): cardinal;[internproc:fpc_in_bsr_x];
{$else}
function BsrDWord(Const AValue : DWord): {$ifdef CPU16}byte{$else}cardinal{$endif};{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_BSR_DWORD}

{$ifdef FPC_HAS_INTERNAL_BSF_QWORD}
function BsfQWord(Const AValue : QWord): cardinal;[internproc:fpc_in_bsf_x];
{$else}
function BsfQWord(Const AValue : QWord): {$ifdef CPU16}byte{$else}cardinal{$endif};{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_BSF_QWORD}
{$ifdef FPC_HAS_INTERNAL_BSR_QWORD}
function BsrQWord(Const AValue : QWord): cardinal;[internproc:fpc_in_bsr_x];
{$else}
function BsrQWord(Const AValue : QWord): {$ifdef CPU16}byte{$else}cardinal{$endif};{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_BSR_QWORD}

function PopCnt(Const AValue: Byte): Byte;[internproc:fpc_in_popcnt_x];
function PopCnt(Const AValue: Word): Word;[internproc:fpc_in_popcnt_x];
function PopCnt(Const AValue : DWord): DWord;[internproc:fpc_in_popcnt_x];
function PopCnt(Const AValue : QWord): QWord;[internproc:fpc_in_popcnt_x];
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function SarLongint(Const AValue : Longint;Shift : Byte = 1): Longint;
{$endif FPC_HAS_INTERNAL_SAR_DWORD}

{$ifdef FPC_HAS_INTERNAL_SAR_QWORD}
function SarInt64(Const AValue : Int64): Int64;[internproc:fpc_in_sar_x];
function SarInt64(Const AValue : Int64;Shift : Byte): Int64;[internproc:fpc_in_sar_x_y];
{$else FPC_HAS_INTERNAL_SAR_QWORD}
function SarInt64(Const AValue : Int64;Shift : Byte = 1): Int64;
{$endif FPC_HAS_INTERNAL_SAR_QWORD}

{$ifdef FPC_HAS_INTERNAL_BSX}
{$if defined(cpui386) or defined(cpux86_64)}
{$define FPC_HAS_INTERNAL_BSX_BYTE}
{$define FPC_HAS_INTERNAL_BSX_WORD}
{$define FPC_HAS_INTERNAL_BSX_DWORD}
{$endif}
{$if defined(cpux86_64)}
{$define FPC_HAS_INTERNAL_BSX_QWORD}
{$endif}
{$endif}

{$ifdef FPC_HAS_INTERNAL_BSX_BYTE}
function BsfByte(Const AValue: Byte): Byte;[internproc:fpc_in_bsf_x];
function BsrByte(Const AValue: Byte): Byte;[internproc:fpc_in_bsr_x];
{$else}
function BsfByte(Const AValue: Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
function BsrByte(Const AValue: Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif}

{$ifdef FPC_HAS_INTERNAL_BSX_WORD}
function BsfWord(Const AValue: Word): cardinal;[internproc:fpc_in_bsf_x];
function BsrWord(Const AValue: Word): cardinal;[internproc:fpc_in_bsr_x];
{$else}
function BsfWord(Const AValue: Word): cardinal;{$ifdef SYSTEMINLINE}inline;{$endif}
function BsrWord(Const AValue: Word): cardinal;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif}

{$ifdef FPC_HAS_INTERNAL_BSX_DWORD}
function BsfDWord(Const AValue : DWord): cardinal;[internproc:fpc_in_bsf_x];
function BsrDWord(Const AValue : DWord): cardinal;[internproc:fpc_in_bsr_x];
{$else}
function BsfDWord(Const AValue : DWord): cardinal;{$ifdef SYSTEMINLINE}inline;{$endif}
function BsrDWord(Const AValue : DWord): cardinal;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_BSX_DWORD}

{$ifdef FPC_HAS_INTERNAL_BSX_QWORD}
function BsfQWord(Const AValue : QWord): cardinal;[internproc:fpc_in_bsf_x];
function BsrQWord(Const AValue : QWord): cardinal;[internproc:fpc_in_bsr_x];
{$else}
function BsfQWord(Const AValue : QWord): cardinal;{$ifdef SYSTEMINLINE}inline;{$endif}
function BsrQWord(Const AValue : QWord): cardinal;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_INTERNAL_BSF_QWORD}
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

{$ifndef FPUNONE}
{ float math routines }
{$I mathh.inc}
{$endif}
{ currency math routines }
{$I currh.inc}

{****************************************************************************
                         Addr/Pointer Handling
****************************************************************************}

Function  ptr(sel,off:{$ifdef CPU16}Word{$else}Longint{$endif}):farpointer;[internconst:fpc_in_const_ptr];{$ifdef SYSTEMINLINE}inline;{$endif}
Function  Cseg:Word;{$ifdef SYSTEMINLINE}inline;{$endif}
Function  Dseg:Word;{$ifdef SYSTEMINLINE}inline;{$endif}
Function  Sseg:Word;{$ifdef SYSTEMINLINE}inline;{$endif}

{****************************************************************************
                      PChar and String Handling
****************************************************************************}

function strpas(p:pchar):shortstring;{$ifdef SYSTEMINLINE}inline;{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
function strlen(p:pchar):sizeint;external name 'FPC_PCHAR_LENGTH';

{ result:
  <0: invalid sequence detected after processing "-result" bytes
  0: incomplete (may still be valid if MaxLookAhead is increased)
  >0: sequence of result bytes forms a codepoint (+ combining diacritics if that
      parameter was true)
}
function Utf8CodePointLen(P: PAnsiChar; MaxLookAhead: SizeInt; IncludeCombiningDiacriticalMarks: Boolean): SizeInt;
=======
=======
>>>>>>> origin/fixes_2_2
function strlen(p:pchar):longint;external name 'FPC_PCHAR_LENGTH';
>>>>>>> graemeg/fixes_2_2

{ Shortstring functions }
Procedure Delete(var s:shortstring;index:SizeInt;count:SizeInt);
Procedure Insert(const source:shortstring;var s:shortstring;index:SizeInt);
Procedure Insert(source:Char;var s:shortstring;index:SizeInt);
Function  Pos(const substr:shortstring;const s:shortstring; Offset: Sizeint = 1):SizeInt;
Function  Pos(C:Char;const s:shortstring; Offset: Sizeint = 1):SizeInt;
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function  Pos(const Substr : ShortString; const Source : RawByteString; Offset: Sizeint = 1) : SizeInt;

{$ifdef FPC_HAS_CPSTRING}
Procedure fpc_setstring_ansistr_pansichar(out S : RawByteString; Buf : PAnsiChar; Len : SizeInt; cp: TSystemCodePage); rtlproc; compilerproc;
Procedure fpc_setstring_ansistr_pwidechar(out S : RawByteString; Buf : PWideChar; Len : SizeInt; cp: TSystemCodePage); rtlproc; compilerproc;
{$else}
Procedure SetString(out S : AnsiString; Buf : PAnsiChar; Len : SizeInt);
Procedure SetString(out S : AnsiString; Buf : PWideChar; Len : SizeInt);
{$endif}
=======
Function  Pos (const Substr : ShortString; const Source : RawByteString) : SizeInt;
Procedure SetString (out S : AnsiString; Buf : PChar; Len : SizeInt);
Procedure SetString (out S : AnsiString; Buf : PWideChar; Len : SizeInt);
>>>>>>> graemeg/cpstrnew
=======
Function  Pos (const Substr : ShortString; const Source : RawByteString) : SizeInt;
Procedure SetString (out S : AnsiString; Buf : PChar; Len : SizeInt);
Procedure SetString (out S : AnsiString; Buf : PWideChar; Len : SizeInt);
>>>>>>> graemeg/cpstrnew
=======
Function  Pos (const Substr : ShortString; const Source : RawByteString) : SizeInt;
Procedure SetString (out S : AnsiString; Buf : PChar; Len : SizeInt);
Procedure SetString (out S : AnsiString; Buf : PWideChar; Len : SizeInt);
>>>>>>> graemeg/cpstrnew
=======
Function  Pos (const Substr : ShortString; const Source : RawByteString) : SizeInt;
Procedure SetString (out S : AnsiString; Buf : PChar; Len : SizeInt);
Procedure SetString (out S : AnsiString; Buf : PWideChar; Len : SizeInt);
>>>>>>> origin/cpstrnew
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
<<<<<<< HEAD
Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_shortstr{$else}SetString{$endif}(out S : Shortstring; Buf : PChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}
=======
Procedure SetString (out S : Shortstring; Buf : PChar; Len : SizeInt);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function  ShortCompareText(const S1, S2: shortstring): SizeInt;
Function  upCase(const s:shortstring):shortstring;
Function  lowerCase(const s:shortstring):shortstring; overload;
Function  Space(b:byte):shortstring;
Function  hexStr(Val:Longint;cnt:byte):shortstring;
Function  OctStr(Val:Longint;cnt:byte):shortstring;
Function  binStr(Val:Longint;cnt:byte):shortstring;
Function  hexStr(Val:int64;cnt:byte):shortstring;
Function  OctStr(Val:int64;cnt:byte):shortstring;
Function  binStr(Val:int64;cnt:byte):shortstring;
Function  hexStr(Val:qword;cnt:byte):shortstring;
Function  OctStr(Val:qword;cnt:byte):shortstring;
Function  binStr(Val:qword;cnt:byte):shortstring;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef CPUI8086}
Function  hexStr(Val:NearPointer):shortstring;
Function  hexStr(Val:FarPointer):shortstring;
Function  hexStr(Val:HugePointer):shortstring;{$ifdef SYSTEMINLINE}inline;{$endif}
{$else CPUI8086}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Function  hexStr(Val:Pointer):shortstring;
{$endif CPUI8086}

{ Char functions }
Function chr(b : byte) : Char;      [INTERNPROC: fpc_in_chr_byte];
Function  upCase(c:Char):Char;
Function  lowerCase(c:Char):Char; overload;
function  pos(const substr : shortstring;c:char; Offset: Sizeint = 1): SizeInt;


{****************************************************************************
                             AnsiString Handling
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
<<<<<<< HEAD
Procedure UniqueString(var S : RawByteString);{$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif FPC_HAS_CPSTRING}external name 'FPC_ANSISTR_UNIQUE';
Function  Pos (const Substr : RawByteString; const Source : RawByteString; Offset: Sizeint = 1) : SizeInt;
Function  Pos (c : AnsiChar; const s : RawByteString; Offset: Sizeint = 1) : SizeInt;
Procedure Insert (const Source : RawByteString; var S : RawByteString; Index : SizeInt);{$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif FPC_HAS_CPSTRING}
Procedure Delete (var S : RawByteString; Index,Size: SizeInt);{$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif FPC_HAS_CPSTRING}
Function  StringOfChar(c : Ansichar;l : SizeInt) : AnsiString;
=======
Procedure UniqueString(var S : AnsiString);external name 'FPC_ANSISTR_UNIQUE';
Function  Pos (const Substr : RawByteString; const Source : RawByteString) : SizeInt;
Function  Pos (c : Char; const s : RawByteString) : SizeInt;
Procedure Insert (const Source : AnsiString; var S : AnsiString; Index : SizeInt);
Procedure Delete (var S : AnsiString; Index,Size: SizeInt);
Function  StringOfChar(c : char;l : SizeInt) : AnsiString;
>>>>>>> graemeg/cpstrnew
function  upcase(const s : ansistring) : ansistring;
function  lowercase(const s : ansistring) : ansistring;

function StringCodePage(const S : RawByteString): TSystemCodePage; overload;
function StringElementSize(const S : RawByteString): Word; overload;
function StringRefCount(const S : RawByteString): SizeInt; overload;
procedure SetCodePage(var s : RawByteString; CodePage : TSystemCodePage; Convert : Boolean = True);
procedure SetMultiByteConversionCodePage(CodePage: TSystemCodePage);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure SetMultiByteFileSystemCodePage(CodePage: TSystemCodePage);
procedure SetMultiByteRTLFileSystemCodePage(CodePage: TSystemCodePage);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


{****************************************************************************
                             WideString Handling
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$i ustringh.inc}
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    {$i wstringh.inc}
  {$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
{$endif FPC_HAS_FEATURE_WIDESTRINGS}


{****************************************************************************
                          Untyped File Management
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_FILEIO}
Procedure Assign(out f:File;const Name: ShortString);
Procedure Assign(out f:File;const p: PAnsiChar);
Procedure Assign(out f:File;const c: AnsiChar);
Procedure Rename(var f:File;const s : ShortString);
Procedure Rename(var f:File;const p : PAnsiChar);
Procedure Rename(var f:File;const c : AnsiChar);
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure Assign(out f:File;const Name: UnicodeString);
Procedure Rename(var f:File;const s : UnicodeString);
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure Assign(out f:File;const Name: RawByteString);
Procedure Rename(var f:File;const s : RawByteString);
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
Procedure Rewrite(var f:File;l:Longint);
Procedure Rewrite(var f:File);
Procedure Reset(var f:File;l:Longint);
Procedure Reset(var f:File);
Procedure Close(var f:File);
Procedure BlockWrite(var f:File;const Buf;Count:Int64;var Result:Int64);
Procedure BlockWrite(var f:File;const Buf;Count:Longint;var Result:Longint);
Procedure BlockWrite(var f:File;const Buf;Count:Cardinal;var Result:Cardinal);
Procedure BlockWrite(var f:File;const Buf;Count:Word;var Result:Word);
Procedure BlockWrite(var f:File;const Buf;Count:Word;var Result:Integer);
Procedure BlockWrite(var f:File;const Buf;Count:Longint);
Procedure BlockRead(var f:File;var Buf;count:Int64;var Result:Int64);
Procedure BlockRead(var f:File;var Buf;count:Longint;var Result:Longint);
Procedure BlockRead(var f:File;var Buf;count:Cardinal;var Result:Cardinal);
Procedure BlockRead(var f:File;var Buf;count:Word;var Result:Word);
Procedure BlockRead(var f:File;var Buf;count:Word;var Result:Integer);
Procedure BlockRead(var f:File;var Buf;count:Int64);
Function  FilePos(var f:File):Int64;
Function  FileSize(var f:File):Int64;
Procedure Seek(var f:File;Pos:Int64);
Function  EOF(var f:File):Boolean;
Procedure Erase(var f:File);
Procedure Truncate (var F:File);
{$endif FPC_HAS_FEATURE_FILEIO}


{****************************************************************************
                           Typed File Management
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_FILEIO}
Procedure Assign(out f:TypedFile;const Name:shortstring);
Procedure Assign(out f:TypedFile;const p:PAnsiChar);
Procedure Assign(out f:TypedFile;const c:AnsiChar);
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure Assign(out f:TypedFile;const Name:unicodestring);
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure Assign(out f:TypedFile;const Name:rawbytestring);
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
Procedure Reset(var f : TypedFile);   [INTERNPROC: fpc_in_Reset_TypedFile];
Procedure Rewrite(var f : TypedFile); [INTERNPROC: fpc_in_Rewrite_TypedFile];
{$endif FPC_HAS_FEATURE_FILEIO}

{****************************************************************************
                            Text File Management
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_TEXTIO}
Procedure Assign(out t:Text;const s:shortstring);
Procedure Rename(var t:Text;const s:shortstring);
Procedure Assign(out t:Text;const p:PAnsiChar);
Procedure Rename(var t:Text;const p:PAnsiChar);
Procedure Assign(out t:Text;const c:AnsiChar);
Procedure Rename(var t:Text;const c:AnsiChar);
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure Assign(out t:Text;const s:unicodestring);
Procedure Rename(var t:Text;const s:unicodestring);
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure Rename(var t:Text;const s:rawbytestring);
Procedure Assign(out t:Text;const s:rawbytestring);
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
Procedure Close(var t:Text);
Procedure Rewrite(var t:Text);
Procedure Reset(var t:Text);
Procedure Append(var t:Text);
Procedure Flush(var t:Text);
Procedure Erase(var t:Text);
Function  EOF(var t:Text):Boolean;
Function  EOF:Boolean;
Function  EOLn(var t:Text):Boolean;
Function  EOLn:Boolean;
Function  SeekEOLn (var t:Text):Boolean;
Function  SeekEOF (var t:Text):Boolean;
Function  SeekEOLn:Boolean;
Function  SeekEOF:Boolean;
Procedure SetTextBuf(var f:Text; var Buf);[INTERNPROC:fpc_in_settextbuf_file_x];
Procedure SetTextBuf(var f:Text; var Buf; Size:SizeInt);
Procedure SetTextLineEnding(var f:Text; Ending:string);
function GetTextCodePage(var T: Text): TSystemCodePage;
procedure SetTextCodePage(var T: Text; CodePage: TSystemCodePage);
{$endif FPC_HAS_FEATURE_TEXTIO}

{****************************************************************************
                            Directory Management
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_FILEIO}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure chdir(const s:shortstring); overload;
Procedure mkdir(const s:shortstring); overload;
Procedure rmdir(const s:shortstring); overload;
Procedure getdir(drivenr:byte;var dir:shortstring);overload;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Procedure chdir(const s:string); overload;
Procedure mkdir(const s:string); overload;
Procedure rmdir(const s:string); overload;
// the pchar versions are exported via alias for use in objpas

Procedure getdir(drivenr:byte;var dir:shortstring);
>>>>>>> graemeg/cpstrnew
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure chdir(const s:rawbytestring); overload;
Procedure mkdir(const s:rawbytestring); overload;
Procedure rmdir(const s:rawbytestring); overload;
// defaultrtlfilesystemcodepage is returned here
Procedure getdir(drivenr:byte;var dir: rawbytestring);overload;{$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif FPC_HAS_CPSTRING}
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure chdir(const s:unicodestring); overload;
Procedure mkdir(const s:unicodestring); overload;
Procedure rmdir(const s:unicodestring); overload;
Procedure getdir(drivenr:byte;var dir: unicodestring);overload;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

{$endif FPC_HAS_FEATURE_FILEIO}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{*****************************************************************************
                             Miscellaneous
*****************************************************************************}

{ os independent calls to allow backtraces }
{$IFDEF INTERNAL_BACKTRACE}
// inserted in compiler/psystem.pas
//function get_frame:pointer;[INTERNPROC:fpc_in_get_frame];
(*
// still defined externally
function get_caller_addr(framebp:pointer;addr:pointer=nil):pointer;[INTERNPROC:fpc_in_get_caller_addr];
function get_caller_frame(framebp:pointer;addr:pointer=nil):pointer;[INTERNPROC:fpc_in_get_caller_frame];
*)
{$ELSE}
function get_frame:pointer;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ENDIF}

Function Get_pc_addr : CodePointer;

{ Writes at most 'count' caller stack frames to pre-allocated buffer pointed to
  by 'frames', skipping 'skipframes' initial frames. Returns number of frames written. }
function CaptureBacktrace(skipframes,count:sizeint;frames:PCodePointer):sizeint;

<<<<<<< HEAD
function get_caller_addr(framebp:pointer;addr:codepointer=nil):codepointer;
function get_caller_frame(framebp:pointer;addr:codepointer=nil):pointer;
procedure get_caller_stackinfo(var framebp : pointer; var addr : codepointer);

Function IOResult:Word;
<<<<<<< HEAD
Function Sptr:Pointer;[internconst:fpc_in_const_ptr];
=======
=======
Function IOResult:Word;
>>>>>>> origin/fixes_2_2
Function Sptr:Pointer;{$ifdef SYSTEMINLINE}inline;{$endif}[internconst:fpc_in_const_ptr];
>>>>>>> graemeg/fixes_2_2

{$ifdef FPC_HAS_FEATURE_PROCESSES}
Function GetProcessID:SizeUInt;
Function GetThreadID:TThreadID;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_FEATURE_PROCESSES}

function InterLockedIncrement (var Target: longint) : longint; public name 'FPC_INTERLOCKEDINCREMENT';
function InterLockedDecrement (var Target: longint) : longint; public name 'FPC_INTERLOCKEDDECREMENT';
function InterLockedExchange (var Target: longint;Source : longint) : longint; public name 'FPC_INTERLOCKEDEXCHANGE';
function InterLockedExchangeAdd (var Target: longint;Source : longint) : longint; public name 'FPC_INTERLOCKEDEXCHANGEADD';
function InterlockedCompareExchange(var Target: longint; NewValue: longint; Comperand: longint): longint; public name 'FPC_INTERLOCKEDCOMPAREEXCHANGE';
{$ifdef cpu64}
function InterLockedIncrement64 (var Target: int64) : int64; public name 'FPC_INTERLOCKEDINCREMENT64';
function InterLockedDecrement64 (var Target: int64) : int64; public name 'FPC_INTERLOCKEDDECREMENT64';
function InterLockedExchange64 (var Target: int64;Source : int64) : int64; public name 'FPC_INTERLOCKEDEXCHANGE64';
function InterLockedExchangeAdd64 (var Target: int64;Source : int64) : int64; public name 'FPC_INTERLOCKEDEXCHANGEADD64';
function InterlockedCompareExchange64(var Target: int64; NewValue: int64; Comperand: int64): int64; public name 'FPC_INTERLOCKEDCOMPAREEXCHANGE64';
{$endif cpu64}
{ Pointer overloads }
{$ifdef cpu64}
function InterLockedIncrement (var Target: Pointer) : Pointer; external name 'FPC_INTERLOCKEDINCREMENT64';
function InterLockedDecrement (var Target: Pointer) : Pointer; external name 'FPC_INTERLOCKEDDECREMENT64';
function InterLockedExchange (var Target: Pointer;Source : Pointer) : Pointer; external name 'FPC_INTERLOCKEDEXCHANGE64';
function InterLockedExchangeAdd (var Target: Pointer;Source : Pointer) : Pointer; external name 'FPC_INTERLOCKEDEXCHANGEADD64';
function InterlockedCompareExchange(var Target: Pointer; NewValue: Pointer; Comperand: Pointer): Pointer; external name 'FPC_INTERLOCKEDCOMPAREEXCHANGE64';
{$else cpu64}
function InterLockedIncrement (var Target: Pointer) : Pointer; external name 'FPC_INTERLOCKEDINCREMENT';
function InterLockedDecrement (var Target: Pointer) : Pointer; external name 'FPC_INTERLOCKEDDECREMENT';
function InterLockedExchange (var Target: Pointer;Source : Pointer) : Pointer; external name 'FPC_INTERLOCKEDEXCHANGE';
function InterLockedExchangeAdd (var Target: Pointer;Source : Pointer) : Pointer; external name 'FPC_INTERLOCKEDEXCHANGEADD';
function InterlockedCompareExchange(var Target: Pointer; NewValue: Pointer; Comperand: Pointer): Pointer; external name 'FPC_INTERLOCKEDCOMPAREEXCHANGE';
{$endif cpu64}
{ unsigned overloads }
function InterLockedIncrement (var Target: cardinal) : cardinal; external name 'FPC_INTERLOCKEDINCREMENT';
function InterLockedDecrement (var Target: cardinal) : cardinal; external name 'FPC_INTERLOCKEDDECREMENT';
function InterLockedExchange (var Target: cardinal;Source : cardinal) : cardinal; external name 'FPC_INTERLOCKEDEXCHANGE';
function InterLockedExchangeAdd (var Target: cardinal;Source : cardinal) : cardinal; external name 'FPC_INTERLOCKEDEXCHANGEADD';
function InterlockedCompareExchange(var Target: cardinal; NewValue: cardinal; Comperand: cardinal): cardinal; external name 'FPC_INTERLOCKEDCOMPAREEXCHANGE';
{$ifdef cpu64}
function InterLockedIncrement64 (var Target: qword) : qword; external name 'FPC_INTERLOCKEDINCREMENT64';
function InterLockedDecrement64 (var Target: qword) : qword; external name 'FPC_INTERLOCKEDDECREMENT64';
function InterLockedExchange64 (var Target: qword;Source : qword) : qword; external name 'FPC_INTERLOCKEDEXCHANGE64';
function InterLockedExchangeAdd64 (var Target: qword;Source : qword) : qword; external name 'FPC_INTERLOCKEDEXCHANGEADD64';
function InterlockedCompareExchange64(var Target: qword; NewValue: qword; Comperand: qword): int64; external name 'FPC_INTERLOCKEDCOMPAREEXCHANGE64';
{$endif cpu64}


{*****************************************************************************
                          Init / Exit / ExitProc
*****************************************************************************}

type
  TRuntimeError =
    (reNone, reOutOfMemory, reInvalidPtr, reDivByZero, reRangeError,
     reIntOverflow, reInvalidOp, reZeroDivide, reOverflow, reUnderflow,
     reInvalidCast, reAccessViolation, rePrivInstruction, reControlBreak,
     reStackOverflow, reVarTypeCast, reVarInvalidOp, reVarDispatch,
     reVarArrayCreate, reVarNotArray, reVarArrayBounds, reAssertionFailed,
     reExternalException, reIntfCastError, reSafeCallError, reQuit,
     reCodesetConversion);

Const
  // Please keep locations corresponding to location in array above
  RuntimeErrorExitCodes : Array[TRuntimeError] of Byte = (
     0  , 203, 204, 200,  201,
     215, 207, 200, 205,  206,
     219, 216, 218, 217,
     202, 220, 221, 222,
     223, 224, 225, 227,
     212, 228, 229, 233,
     234);

Procedure Error(RunTimeError : TRunTimeError);
{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
Function  ParamCount:Longint;
Function  ParamStr(l:Longint):string;
{$endif FPC_HAS_FEATURE_COMMANDARGS}

<<<<<<< HEAD
Procedure Dump_Stack(var f : text;fp:pointer;addr : codepointer = nil);
procedure Dump_Stack(var f : text;skipframes : longint);
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
procedure DumpExceptionBackTrace(var f:text);
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
=======
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
Procedure Dump_Stack(var f : text;bp:pointer);
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
procedure DumpExceptionBackTrace(var f:text);
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
{$endif FPC_HAS_FEATURE_CONSOLEIO}
>>>>>>> graemeg/fixes_2_2

Procedure RunError(w:Word);
Procedure RunError;{$ifdef SYSTEMINLINE}inline;{$endif}
Procedure halt(errnum:Longint);
{$ifdef FPC_HAS_FEATURE_HEAP}
Procedure AddExitProc(Proc:TProcedure);
{$endif FPC_HAS_FEATURE_HEAP}
Procedure halt;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Need to be exported for threads unit }
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
Procedure SysInitExceptions;
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
{$ifndef EMBEDDED}
procedure SysInitStdIO;
procedure SysFlushStdIO;
{$endif EMBEDDED}
{$endif FPC_HAS_FEATURE_CONSOLEIO}
<<<<<<< HEAD
{$ifndef FPUNONE}
Procedure SysResetFPU;
Procedure SysInitFPU;
{$endif}
=======
Procedure SysResetFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
Procedure SysInitFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{*****************************************************************************
                         Abstract/Assert/Error Handling
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_HEAP}
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
function ArrayStringToPPchar(const S:Array of AnsiString;reserveentries:Longint):ppchar; // const ?
Function StringToPPChar(var S:AnsiString;ReserveEntries:integer):ppchar;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
Function StringToPPChar(S: PChar;ReserveEntries:integer):ppchar;
{$endif FPC_HAS_FEATURE_HEAP}


procedure AbstractError;external name 'FPC_ABSTRACTERROR';
procedure EmptyMethod;external name 'FPC_EMPTYMETHOD';
Function  SysBackTraceStr(Addr:CodePointer): ShortString;
Procedure SysAssert(const Msg,FName:ShortString;LineNo:Longint;ErrorAddr:Pointer);
(* Supposed to return address of previous CtrlBreakHandler *)
(* (may be nil), returned value of pointer (-1) means that *)
(* attempt to setup CtrlBreakHandler wasn't successful.    *)
function SysSetCtrlBreakHandler (Handler: TCtrlBreakHandler): TCtrlBreakHandler;

{ Error handlers }
Type
  TBackTraceStrFunc = Function (Addr: CodePointer): ShortString;
  TErrorProc = Procedure (ErrNo : Longint; Address : CodePointer; Frame : Pointer);
  TAbstractErrorProc = Procedure;
  TAssertErrorProc = Procedure(const msg,fname:ShortString;lineno:longint;erroraddr:pointer);
  TSafeCallErrorProc = Procedure(error : HResult;addr : pointer);


const
  BackTraceStrFunc  : TBackTraceStrFunc = @SysBackTraceStr;
  ErrorProc         : TErrorProc = nil;
  AbstractErrorProc : TAbstractErrorProc = nil;
  AssertErrorProc   : TAssertErrorProc = @SysAssert;
  SafeCallErrorProc : TSafeCallErrorProc = nil;
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
  ExceptObjProc     : Pointer = nil; { Used to convert OS exceptions to FPC exceptions. }
  ExceptClsProc     : Pointer = nil;
{$endif FPC_HAS_FEATURE_EXCEPTIONS}

{*****************************************************************************
                          SetJmp/LongJmp
*****************************************************************************}

{$i setjumph.inc}


{*****************************************************************************
                       Object Pascal support
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_CLASSES}
{$i objpash.inc}
{$endif FPC_HAS_FEATURE_CLASSES}

{*****************************************************************************
                           Variant support
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_VARIANTS}
{$i varianth.inc}
{$endif FPC_HAS_FEATURE_VARIANTS}

{*****************************************************************************
                           RTTI support
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_RTTI}
{$i rttih.inc}
{$endif FPC_HAS_FEATURE_RTTI}

{*****************************************************************************
                   Internal helper routines support
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
{$i dynarrh.inc}
{$endif FPC_HAS_FEATURE_DYNARRAYS}

{ documenting compiler proc. is useless, they shouldn't be used by the user anyways }
{$ifndef fpdocsystem}
{$i compproc.inc}
{$endif fpdocsystem}

{*****************************************************************************
                               Heap
*****************************************************************************}

{$i heaph.inc}

{*****************************************************************************
                          Thread support
*****************************************************************************}

{ Generic threadmanager }
{$ifdef FPC_HAS_FEATURE_THREADING}
{$i threadh.inc}
{$endif FPC_HAS_FEATURE_THREADING}

{*****************************************************************************
                          Dynamic library support
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_DYNLIBS}
{$i dynlibh.inc}
{$endif FPC_HAS_FEATURE_DYNLIBS}

{*****************************************************************************
                          Resources support
*****************************************************************************}

{$i resh.inc}


{*****************************************************************************
                          Exception support
*****************************************************************************}

{ contains the definition of the TExceptAddr type, which is required
  by the compiler to generate code for any routine containing
  implicit or explicit exceptions }
{$i excepth.inc}

{*****************************************************************************
                   FPDoc phony declarations.
*****************************************************************************}

{$ifdef fpdocsystem}
{$i system.fpd}
{$endif}
