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
                        Processor specific routines
****************************************************************************}

{$ifdef FPC_USE_LIBC}
  {$ifdef SYSTEMINLINE}
    {$define INLINEGENERICS}
  {$endif}
{$endif}
(*
Procedure Move(const source;var dest;count:SizeInt);
Procedure FillChar(var x;count:SizeInt;Value:Byte);
Procedure FillChar(var x;count:SizeInt;Value:Boolean);
Procedure FillChar(var x;count:SizeInt;Value:Char);
procedure FillByte(var x;count:SizeInt;value:byte);
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
function  CompareWord(const buf1,buf2;len:SizeInt):SizeInt;
function  CompareDWord(const buf1,buf2;len:SizeInt):SizeInt;
procedure MoveChar0(const buf1;var buf2;len:SizeInt);
function  IndexChar0(const buf;len:SizeInt;b:char):SizeInt;
function  CompareChar0(const buf1,buf2;len:SizeInt):SizeInt;
procedure prefetch(const mem);[internproc:fpc_in_prefetch_var];
procedure ReadBarrier;
procedure ReadDependencyBarrier;
procedure ReadWriteBarrier;
procedure WriteBarrier;
*)

procedure fillchar(var arr: array of jbyte; len: sizeint; val: byte);
{ boolean maps to a different signature }
procedure fillchar(var arr: array of jbyte; len: sizeint; val: boolean);
{ don't define since the signature would be the same as the one above (well,
  we could cheat by changing the case since the JVM is case-sensitive, but
  this way we also save on code size) -> map it to the byte version via
  "external" }
procedure fillchar(var arr: array of jbyte; len: sizeint; val: ansichar); external;
procedure fillchar(var arr: array of jbyte; len: sizeint; val: jbyte); external;
procedure fillchar(var arr: array of byte; len: sizeint; val: byte); external;
procedure fillchar(var arr: array of byte; len: sizeint; val: ansichar); external;
procedure fillchar(var arr: array of byte; len: sizeint; val: jbyte); external;
procedure fillchar(var arr: array of ansichar; len: sizeint; val: byte); external;
procedure fillchar(var arr: array of ansichar; len: sizeint; val: ansichar); external;
procedure fillchar(var arr: array of ansichar; len: sizeint; val: jbyte); external;
procedure fillchar(var arr: array of ansichar; len: sizeint; val: boolean); external;


procedure fillchar(var arr: array of boolean; len: sizeint; val: byte);
procedure fillchar(var arr: array of boolean; len: sizeint; val: boolean);
procedure fillchar(var arr: array of boolean; len: sizeint; val: ansichar); external;
procedure fillchar(var arr: array of boolean; len: sizeint; val: jbyte); external;


procedure fillchar(var arr: array of jshort; len: sizeint; val: byte);
procedure fillchar(var arr: array of jshort; len: sizeint; val: boolean);
procedure fillchar(var arr: array of jshort; len: sizeint; val: ansichar); external;
procedure fillchar(var arr: array of jshort; len: sizeint; val: jbyte); external;
procedure fillchar(var arr: array of word; len: sizeint; val: byte);  external;
procedure fillchar(var arr: array of word; len: sizeint; val: boolean);  external;
procedure fillchar(var arr: array of word; len: sizeint; val: ansichar); external;
procedure fillchar(var arr: array of word; len: sizeint; val: jbyte); external;

{ widechar maps to a different signature }
procedure fillchar(var arr: array of widechar; len: sizeint; val: byte);
procedure fillchar(var arr: array of widechar; len: sizeint; val: boolean);
procedure fillchar(var arr: array of widechar; len: sizeint; val: ansichar); external;
procedure fillchar(var arr: array of widechar; len: sizeint; val: jbyte); external;

procedure fillchar(var arr: array of jint; len: sizeint; val: byte);
procedure fillchar(var arr: array of jint; len: sizeint; val: boolean);
procedure fillchar(var arr: array of jint; len: sizeint; val: ansichar); external;
procedure fillchar(var arr: array of jint; len: sizeint; val: jbyte); external;
procedure fillchar(var arr: array of dword; len: sizeint; val: byte); external;
procedure fillchar(var arr: array of dword; len: sizeint; val: boolean); external;
procedure fillchar(var arr: array of dword; len: sizeint; val: ansichar); external;
procedure fillchar(var arr: array of dword; len: sizeint; val: jbyte); external;

procedure fillchar(var arr: array of jlong; len: sizeint; val: byte);
procedure fillchar(var arr: array of jlong; len: sizeint; val: boolean);
procedure fillchar(var arr: array of jlong; len: sizeint; val: ansichar); external;
procedure fillchar(var arr: array of jlong; len: sizeint; val: jbyte); external;
procedure fillchar(var arr: array of qword; len: sizeint; val: byte); external;
procedure fillchar(var arr: array of qword; len: sizeint; val: boolean); external;
procedure fillchar(var arr: array of qword; len: sizeint; val: ansichar); external;
procedure fillchar(var arr: array of qword; len: sizeint; val: jbyte); external;


function  IndexChar(const buf: array of boolean;len:SizeInt;b:ansichar):SizeInt;
function  IndexChar(const buf: array of jbyte;len:SizeInt;b:ansichar):SizeInt;
function  IndexChar(const buf: array of byte;len:SizeInt;b:ansichar):SizeInt; external;
function  IndexChar(const buf: array of ansichar;len:SizeInt;b:ansichar):SizeInt; external;

function  IndexByte(const buf: array of jbyte;len:SizeInt;b:jbyte):SizeInt;
function  IndexByte(const buf: array of boolean;len:SizeInt;b:jbyte):SizeInt;
function  IndexByte(const buf: array of byte;len:SizeInt;b:jbyte):SizeInt; external;
function  IndexByte(const buf: array of ansichar;len:SizeInt;b:jbyte):SizeInt; external;
function  IndexByte(const buf: array of bytebool;len:SizeInt;b:bytebool):SizeInt; external;


function  IndexWord(const buf: array of jshort;len:SizeInt;b:jshort):SizeInt;
function  IndexWord(const buf: array of jchar;len:SizeInt;b:jchar):SizeInt;
function  IndexWord(const buf: array of jchar;len:SizeInt;b:jshort):SizeInt;
function  IndexWord(const buf: array of word;len:SizeInt;b:word):SizeInt; external;
function  IndexWord(const buf: array of wordbool;len:SizeInt;b:wordbool):SizeInt; external;
function  IndexWord(const buf: array of boolean16;len:SizeInt;b:boolean16):SizeInt; external;


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
Function swap (X : Integer) : Integer;[internconst:fpc_in_const_swap_word]; external;
Function swap (X : Longint) : Longint;{$ifdef SYSTEMINLINE}inline;{$endif}[internconst:fpc_in_const_swap_long];
Function swap (X : Cardinal) : Cardinal;[internconst:fpc_in_const_swap_long]; external;
Function swap (X : QWord) : QWord;[internconst:fpc_in_const_swap_qword]; external;
Function swap (X : Int64) : Int64;{$ifdef SYSTEMINLINE}inline;{$endif}[internconst:fpc_in_const_swap_qword];

Function Align (Addr : PtrUInt; Alignment : PtrUInt) : PtrUInt;{$ifdef SYSTEMINLINE}inline;{$endif}
(*
Function Align (Addr : Pointer; Alignment : PtrUInt) : Pointer;{$ifdef SYSTEMINLINE}inline;{$endif}
*)

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
Function sqr(l:QWord):QWord;[internconst:fpc_in_const_sqr]; external;
Function odd(l:Longint):Boolean;[internconst:fpc_in_const_odd];{$ifdef SYSTEMINLINE}inline;{$endif}
Function odd(l:Longword):Boolean;[internconst:fpc_in_const_odd]; external;
Function odd(l:Int64):Boolean;[internconst:fpc_in_const_odd];{$ifdef SYSTEMINLINE}inline;{$endif}
Function odd(l:QWord):Boolean;[internconst:fpc_in_const_odd]; external;

function SwapEndian(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function SwapEndian(const AValue: Word): Word; external;
function SwapEndian(const AValue: LongInt): LongInt;
function SwapEndian(const AValue: DWord): DWord; external;
function SwapEndian(const AValue: Int64): Int64;
function SwapEndian(const AValue: QWord): QWord; external;

function BEtoN(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function BEtoN(const AValue: Word): Word; external;
function BEtoN(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function BEtoN(const AValue: DWord): DWord; external;
function BEtoN(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
function BEtoN(const AValue: QWord): QWord; external;

function LEtoN(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function LEtoN(const AValue: Word): Word; external;
function LEtoN(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function LEtoN(const AValue: DWord): DWord; external;
function LEtoN(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
function LEtoN(const AValue: QWord): QWord; external;

function NtoBE(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoBE(const AValue: Word): Word; external;
function NtoBE(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoBE(const AValue: DWord): DWord; external;
function NtoBE(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoBE(const AValue: QWord): QWord; external;

function NtoLE(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoLE(const AValue: Word): Word; external;
function NtoLE(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoLE(const AValue: DWord): DWord; external;
function NtoLE(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
function NtoLE(const AValue: QWord): QWord; external;

{$ifdef FPC_HAS_INTERNAL_ROX}

{$if defined(cpux86_64) or defined(cpui386)}
{$define FPC_HAS_INTERNAL_ROX_BYTE}
{$define FPC_HAS_INTERNAL_ROX_WORD}
{$endif defined(cpux86_64) or defined(cpui386)}

{$if defined(cpux86_64) or defined(cpui386) or defined(arm) or defined(powerpc) or defined(powerpc64)}
{$define FPC_HAS_INTERNAL_ROX_DWORD}
{$endif defined(cpux86_64) or defined(cpui386) or defined(arm) or defined(powerpc) or defined(powerpc64)}

{$if defined(cpux86_64) or defined(powerpc64)}
{$define FPC_HAS_INTERNAL_ROX_QWORD}
{$endif defined(cpux86_64) or defined(powerpc64)}

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

{$if defined(cpux86_64) or defined(cpui386) or defined(mips) or defined(mipsel) or defined(sparc)}
{$define FPC_HAS_INTERNAL_SAR_BYTE}
{$define FPC_HAS_INTERNAL_SAR_WORD}
{$endif defined(cpux86_64) or defined(cpui386) or defined(mips) or defined(mipsel) or defined(sparc)}

{ currently, all supported CPUs have an internal 32 bit sar implementation }
{ $if defined(cpux86_64) or defined(cpui386) or defined(arm) or defined(powerpc) or defined(powerpc64) or defined(mips) or defined(mipsel)}
{$define FPC_HAS_INTERNAL_SAR_DWORD}
{ $endif defined(cpux86_64) or defined(cpui386) or defined(arm) or defined(powerpc) or defined(powerpc64) or defined(mips) or defined(mipsel)}

{$if defined(cpux86_64) or defined(powerpc64)}
{$define FPC_HAS_INTERNAL_SAR_QWORD}
{$endif defined(cpux86_64) or defined(powerpc64)}

{$endif FPC_HAS_INTERNAL_SAR}

{$ifdef FPC_HAS_INTERNAL_SAR_BYTE}
function SarShortint(Const AValue : Shortint): Shortint;[internproc:fpc_in_sar_x];
function SarShortint(Const AValue : Shortint;Shift : Byte): Shortint;[internproc:fpc_in_sar_x_y];
{$else FPC_HAS_INTERNAL_ROX_BYTE}
function SarShortint(Const AValue : Shortint;const Shift : Byte = 1): Shortint;
{$endif FPC_HAS_INTERNAL_ROX_BYTE}

{$ifdef FPC_HAS_INTERNAL_SAR_WORD}
function SarSmallint(Const AValue : Smallint): Smallint;[internproc:fpc_in_sar_x];
function SarSmallint(Const AValue : Smallint;Shift : Byte): Smallint;[internproc:fpc_in_sar_x_y];
{$else FPC_HAS_INTERNAL_SAR_WORD}
function SarSmallint(Const AValue : Smallint;const Shift : Byte = 1): Smallint;
{$endif FPC_HAS_INTERNAL_SAR_WORD}

{$ifdef FPC_HAS_INTERNAL_SAR_DWORD}
function SarLongint(Const AValue : Longint): Longint;[internproc:fpc_in_sar_x];
function SarLongint(Const AValue : Longint;Shift : Byte): Longint;[internproc:fpc_in_sar_x_y];
{$else FPC_HAS_INTERNAL_SAR_DWORD}
function SarLongint(Const AValue : Longint;const Shift : Byte = 1): Longint;
{$endif FPC_HAS_INTERNAL_SAR_DWORD}

function SarInt64(Const AValue : Int64): Int64;[internproc:fpc_in_sar_x];
function SarInt64(Const AValue : Int64;Shift : Byte): Int64;[internproc:fpc_in_sar_x_y];
{$ifndef FPC_HAS_INTERNAL_SAR_QWORD}
function fpc_SarInt64(Const AValue : Int64;const Shift : Byte): Int64;compilerproc;
{$endif FPC_HAS_INTERNAL_SAR_QWORD}

{$ifdef FPC_HAS_INTERNAL_BSF}
{$if defined(cpui386) or defined(cpux86_64) or defined(cpuarm)}
{$define FPC_HAS_INTERNAL_BSF_BYTE}
{$define FPC_HAS_INTERNAL_BSF_WORD}
{$define FPC_HAS_INTERNAL_BSF_DWORD}
{$endif}
{$if defined(cpux86_64)}
{$define FPC_HAS_INTERNAL_BSF_QWORD}
{$endif}
{$endif}

{$ifdef FPC_HAS_INTERNAL_BSR}
{$if defined(cpui386) or defined(cpux86_64) or defined(cpuarm)}
{$define FPC_HAS_INTERNAL_BSR_BYTE}
{$define FPC_HAS_INTERNAL_BSR_WORD}
{$define FPC_HAS_INTERNAL_BSR_DWORD}
{$endif}
{$if defined(cpux86_64)}
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

{$ifndef FPUNONE}
{ float math routines }
{$I mathh.inc}
{$endif}
{ currency math routines }
{$I currh.inc}

{****************************************************************************
                         Addr/Pointer Handling
****************************************************************************}
(*
Function  ptr(sel,off:{$ifdef CPU16}Word{$else}Longint{$endif}):farpointer;[internconst:fpc_in_const_ptr];{$ifdef SYSTEMINLINE}inline;{$endif}
Function  Cseg:Word;{$ifdef SYSTEMINLINE}inline;{$endif}
Function  Dseg:Word;{$ifdef SYSTEMINLINE}inline;{$endif}
Function  Sseg:Word;{$ifdef SYSTEMINLINE}inline;{$endif}
*)

{****************************************************************************
                      PChar and String Handling
****************************************************************************}
(*
function strpas(p:pchar):shortstring;{$ifdef SYSTEMINLINE}inline;{$endif}
function strlen(p:pchar):sizeint;external name 'FPC_PCHAR_LENGTH';
*)

{ result:
  <0: invalid sequence detected after processing "-result" bytes
  0: incomplete (may still be valid if MaxLookAhead is increased)
  >0: sequence of result bytes forms a codepoint (+ combining diacritics if that
      parameter was true)
}
function Utf8CodePointLen(P: PAnsiChar; MaxLookAhead: SizeInt; IncludeCombiningDiacriticalMarks: Boolean): SizeInt;


var
  { separated compared to generic version, for Java type safety }
  FPC_EMPTYANSICHAR : array[0..0] of ansichar;
  FPC_EMPTYWIDECHAR : array[0..0] of widechar;


{ Shortstring functions }
{$ifdef VER3_0}
Procedure Delete(var s:shortstring;index:SizeInt;count:SizeInt);
Procedure Insert(const source:shortstring;var s:shortstring;index:SizeInt);
Procedure Insert(source:Char;var s:shortstring;index:SizeInt);
{$endif VER3_0}
Function  Pos(const substr:shortstring;const s:shortstring; Offset: Sizeint = 1):SizeInt;
Function  Pos(C:Char;const s:shortstring; Offset: Sizeint = 1):SizeInt;
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Function  Pos(const Substr : ShortString; const Source : RawByteString; Offset: Sizeint = 1) : SizeInt;

{$ifdef FPC_HAS_CPSTRING}
Procedure fpc_setstring_ansistr_pansichar(out S : RawByteString; Buf : PAnsiChar; Len : SizeInt; cp: TSystemCodePage); rtlproc; compilerproc;
Procedure fpc_setstring_ansistr_pwidechar(out S : RawByteString; Buf : PWideChar; Len : SizeInt; cp: TSystemCodePage); rtlproc; compilerproc;
{$else}
Procedure SetString(out S : AnsiString; Buf : PAnsiChar; Len : SizeInt);
Procedure SetString(out S : AnsiString; Buf : PWideChar; Len : SizeInt);
{$endif}
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_shortstr{$else}SetString{$endif}(out S : Shortstring; Buf : PChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}
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
Function  hexStr(Val:qword;cnt:byte):shortstring; {$ifdef cpujvm}external;{$endif}
Function  OctStr(Val:qword;cnt:byte):shortstring; {$ifdef cpujvm}external;{$endif}
Function  binStr(Val:qword;cnt:byte):shortstring; {$ifdef cpujvm}external;{$endif}
{$ifdef CPUI8086}
Function  hexStr(Val:NearPointer):shortstring;
Function  hexStr(Val:FarPointer):shortstring;
Function  hexStr(Val:HugePointer):shortstring;{$ifdef SYSTEMINLINE}inline;{$endif}
{$else CPUI8086}
Function  hexStr(Val:Pointer):shortstring;
{$endif CPUI8086}

{ Char functions }
{$if defined(VER3_2) or defined(VER3_0)}
Function Chr(b : byte) : Char;      [INTERNPROC: fpc_in_chr_byte];
{$endif defined(VER3_2) or defined(VER3_0)}
Function  upCase(c:Char):Char;
Function  lowerCase(c:Char):Char; overload;
function  pos(const substr : shortstring;c:char; Offset : Sizeint=1): SizeInt;


{****************************************************************************
                             AnsiString Handling
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure UniqueString(var S : RawByteString);{$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif FPC_HAS_CPSTRING}{$ifdef SYSTEMINLINE}inline;{$endif}
Function  Pos (const Substr : RawByteString; const Source : RawByteString; Offset: Sizeint = 1) : SizeInt;
Function  Pos (c : AnsiChar; const s : RawByteString; Offset: Sizeint = 1) : SizeInt;
{$ifdef VER3_0}
Procedure Insert (const Source : RawByteString; var S : RawByteString; Index : SizeInt);{$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif FPC_HAS_CPSTRING}
Procedure Delete (var S : RawByteString; Index,Size: SizeInt);{$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif FPC_HAS_CPSTRING}
{$endif VER3_0}
Function  StringOfChar(c : Ansichar;l : SizeInt) : AnsiString;
function  upcase(const s : ansistring) : ansistring;
function  lowercase(const s : ansistring) : ansistring;

function StringCodePage(const S : RawByteString): TSystemCodePage; overload;
function StringElementSize(const S : RawByteString): Word; overload;
function StringRefCount(const S : RawByteString): SizeInt; overload;
procedure SetCodePage(var s : RawByteString; CodePage : TSystemCodePage; Convert : Boolean = True);
procedure SetMultiByteConversionCodePage(CodePage: TSystemCodePage);
procedure SetMultiByteFileSystemCodePage(CodePage: TSystemCodePage);
procedure SetMultiByteRTLFileSystemCodePage(CodePage: TSystemCodePage);
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
Procedure chdir(const s:shortstring); overload;
Procedure mkdir(const s:shortstring); overload;
Procedure rmdir(const s:shortstring); overload;
Procedure getdir(drivenr:byte;var dir:shortstring);overload;
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

(*
{ Writes at most 'count' caller stack frames to pre-allocated buffer pointed to
  by 'frames', skipping 'skipframes' initial frames. Returns number of frames written. }
function CaptureBacktrace(skipframes,count:sizeint;frames:PCodePointer):sizeint;

function get_caller_addr(framebp:pointer;addr:codepointer=nil):codepointer;
function get_caller_frame(framebp:pointer;addr:codepointer=nil):pointer;
procedure get_caller_stackinfo(var framebp : pointer; var addr : codepointer);
*)
//Function IOResult:Word;
//Function Sptr:Pointer;[internconst:fpc_in_const_ptr];

{$ifdef FPC_HAS_FEATURE_PROCESSES}
Function GetProcessID:SizeUInt;
Function GetThreadID:TThreadID;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_HAS_FEATURE_PROCESSES}

(*
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
*)

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
     reCodesetConversion, reNoDynLibsSupport, reThreadError);

Const
  // Please keep locations corresponding to location in array above
  RuntimeErrorExitCodes : Array[TRuntimeError] of Byte = (
     0  , 203, 204, 200,  201,
     215, 207, 200, 205,  206,
     219, 216, 218, 217,
     202, 220, 221, 222,
     223, 224, 225, 227,
     212, 228, 229, 233,
     234, 235, 236);

Procedure Error(RunTimeError : TRunTimeError);
{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
Function  Paramcount:Longint;
Function  ParamStr(l:Longint):string;
{$endif FPC_HAS_FEATURE_COMMANDARGS}

{$ifndef JVM}
Procedure Dump_Stack(var f : text;fp:pointer;addr : codepointer = nil);
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
procedure DumpExceptionBackTrace(var f:text);
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
{$endif JVM}

Procedure RunError(w:Word);
Procedure RunError;{$ifdef SYSTEMINLINE}inline;{$endif}
Procedure halt(errnum:Longint);
{$ifdef FPC_HAS_FEATURE_HEAP}
Procedure AddExitProc(Proc:TProcedure);
{$endif FPC_HAS_FEATURE_HEAP}
Procedure halt;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Need to be exported for threads unit }
(*
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
Procedure SysInitExceptions;
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
*)
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
{$if not defined(EMBEDDED) and not defined(jvm)}
procedure SysInitStdIO;
procedure SysFlushStdIO;
{$endif EMBEDDED or jvm}
{$endif FPC_HAS_FEATURE_CONSOLEIO}
{$ifndef FPUNONE}
Procedure SysResetFPU;
Procedure SysInitFPU;
{$endif}

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


(*
procedure AbstractError;external name 'FPC_ABSTRACTERROR';
procedure EmptyMethod;external name 'FPC_EMPTYMETHOD';
Function  SysBackTraceStr(Addr:CodePointer): ShortString;
Procedure SysAssert(const Msg,FName:ShortString;LineNo:Longint;ErrorAddr:Pointer);
*)
(* Supposed to return address of previous CtrlBreakHandler *)
(* (may be nil), returned value of pointer (-1) means that *)
(* attempt to setup CtrlBreakHandler wasn't successful.    *)
(*
function SysSetCtrlBreakHandler (Handler: TCtrlBreakHandler): TCtrlBreakHandler;
*)

{ Error handlers }
Type
(*
  TBackTraceStrFunc = Function (Addr: CodePointer): ShortString;
*)
  TErrorProc = Procedure (ErrNo : Longint; Address : CodePointer; Frame : Pointer);
(*
  TAbstractErrorProc = Procedure;
  TAssertErrorProc = Procedure(const msg,fname:ShortString;lineno:longint;erroraddr:pointer);
  TSafeCallErrorProc = Procedure(error : HResult;addr : pointer);
*)

const
(*
  BackTraceStrFunc  : TBackTraceStrFunc = @SysBackTraceStr;
*)
  ErrorProc         : TErrorProc = nil;
(*
  AbstractErrorProc : TAbstractErrorProc = nil;
  AssertErrorProc   : TAssertErrorProc = @SysAssert;
  SafeCallErrorProc : TSafeCallErrorProc = nil;
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
  ExceptObjProc     : Pointer = nil; { Used to convert OS exceptions to FPC exceptions. }
  ExceptClsProc     : Pointer = nil;
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
*)

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

{$i rttih.inc}

{*****************************************************************************
                   Internal helper routines support
*****************************************************************************}
(*
{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
{$i dynarrh.inc}
{$endif FPC_HAS_FEATURE_DYNARRAYS}
*)

{ documenting compiler proc. is useless, they shouldn't be used by the user anyways }
{$ifndef fpdocsystem}
{$i jcompproc.inc}
{$endif fpdocsystem}

{*****************************************************************************
                               Heap
*****************************************************************************}

{$ifndef JVM}
{$i heaph.inc}
{$endif JVM}

{*****************************************************************************
                          Thread support
*****************************************************************************}

{ Generic threadmanager }
{$if defined(FPC_HAS_FEATURE_THREADING) and not defined(cpujvm)}
{$i threadh.inc}
{$endif FPC_HAS_FEATURE_THREADING and not cpujvm}

{*****************************************************************************
                          Resources support
*****************************************************************************}
(*
{$i resh.inc}
*)
{*****************************************************************************
                   FPDoc phony declarations.
*****************************************************************************}

{$ifdef fpdocsystem}
{$i system.fpd}
{$endif}
