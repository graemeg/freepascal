{

    This file is part of the Free Pascal Run time library.
    Copyright (c) 1999-2008 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    For details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ ObjpasInt is the integer type, equivalent to Objpas.Integer (the Integer
  type in ObjFpc and Delphi modes). It is defined here for use in the
  implementation part of the System unit. }
{$ifdef CPU16}
type
  ObjpasInt = SmallInt;
{$else CPU16}
type
  ObjpasInt = LongInt;
{$endif CPU16}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ The RTTI is implemented through a series of constants : }

Const
   // please update tkManagedTypes below if you add new
   // values
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   tkUnknown        = 0;
   tkInteger        = 1;
   tkChar           = 2;
   tkEnumeration    = 3;
   tkFloat          = 4;
   tkSet            = 5;
   tkMethod         = 6;
   tkSString        = 7;
   tkString         = tkSString;
   tkLString        = 8;
   tkAString        = 9;
   tkWString        = 10;
   tkVariant        = 11;
   tkArray          = 12;
   tkRecord         = 13;
   tkInterface      = 14;
   tkClass          = 15;
   tkObject         = 16;
   tkWChar          = 17;
   tkBool           = 18;
   tkInt64          = 19;
   tkQWord          = 20;
   tkDynArray       = 21;
   tkInterfaceCorba = 22;
   tkProcVar        = 23;
   tkUString        = 24;
   tkHelper         = 26;
   tkFile           = 27;
   tkClassRef       = 28;
   tkPointer        = 29;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   tkUnknown       = 0;
   tkInteger       = 1;
   tkChar          = 2;
   tkEnumeration   = 3;
   tkFloat         = 4;
   tkSet           = 5;
   tkMethod        = 6;
   tkSString       = 7;
   tkString        = tkSString;
   tkLString       = 8;
   tkAString       = 9;
   tkWString       = 10;
   tkVariant       = 11;
   tkArray         = 12;
   tkRecord        = 13;
   tkInterface     = 14;
   tkClass         = 15;
   tkObject        = 16;
   tkWChar         = 17;
   tkBool          = 18;
   tkInt64         = 19;
   tkQWord         = 20;
   tkDynArray      = 21;
   tkInterfaceCorba = 22;
   tkProcVar       = 23;
   tkUString       = 24;
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

  // all potentially managed types
  tkManagedTypes   = [tkAstring,tkWstring,tkUstring,tkArray,
                     tkObject,tkRecord,tkDynArray,tkInterface,tkVariant];

{****************************************************************************
                                Local types
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_EXITCODE}
  {$if High(errorcode)<>maxExitCode}
    {$define FPC_LIMITED_EXITCODE}
  {$endif}
{$endif FPC_HAS_FEATURE_EXITCODE}

Procedure HandleError (Errno : Longint); external name 'FPC_HANDLEERROR';
Procedure HandleErrorFrame (Errno : longint;frame : Pointer); forward;
Procedure HandleErrorAddrFrame (Errno : longint;addr : CodePointer; frame : Pointer); forward;
Procedure HandleErrorAddrFrameInd (Errno : longint;addr : CodePointer; frame : Pointer); forward;

{$ifdef FPC_HAS_FEATURE_TEXTIO}
type
  FileFunc = Procedure(var t : TextRec);
{$endif FPC_HAS_FEATURE_TEXTIO}

const
  STACK_MARGIN = 16384;    { Stack size margin for stack checking }
{ Random / Randomize constants }
  OldRandSeed : Cardinal = 0;

{ For Error Handling.}
  ErrorBase : Pointer = nil;public name 'FPC_ERRORBASE';

{ Used by the ansi/widestrings and maybe also other things in the future }
var
  { widechar, because also used by widestring -> pwidechar conversions }
  emptychar : widechar;public name 'FPC_EMPTYCHAR';
{$ifndef FPC_NO_GENERIC_STACK_CHECK}
  { if the OS does the stack checking, we don't need any stklen from the
    main program }
  initialstklen : SizeUint;external name '__stklen';
{$endif FPC_NO_GENERIC_STACK_CHECK}

{ checks whether the given suggested size for the stack of the current
 thread is acceptable. If this is the case, returns it unaltered.
 Otherwise it should return an acceptable value.

 Operating systems that automatically expand their stack on demand, should
 simply return a very large value.
 Operating systems which do not have a possibility to retrieve stack size
 information, should simply return the given stklen value (This is the default
 implementation).
}
{$ifdef FPC_HAS_FEATURE_STACKCHECK}
function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt; forward;
{$endif FPC_HAS_FEATURE_STACKCHECK}

{*****************************************************************************
                       OS dependent Helpers/Syscalls
*****************************************************************************}

{ for some OSes do_isdevice is defined in sysos.inc, but for others (win32)
  it isn't, and is used before the actual definition is encountered         }

{$ifdef FPC_HAS_FEATURE_FILEIO}
function do_isdevice(handle:thandle):boolean;forward;
{$endif FPC_HAS_FEATURE_FILEIO}


{$i sysos.inc}


{****************************************************************************
                    Include processor specific routines
****************************************************************************}

{$ifdef FPC_USE_LIBC}
{ Under Haiku, bcopy cause a problem when searching for include file
  in the compiler. So, we use the internal implementation for now
  under BeOS and Haiku.  }
{$ifndef BEOS}
{ prefer libc implementations over our own, as they're most likely faster }
{$i cgeneric.inc}
{ is now declared as external reference to another routine in the interface }
{$i cgenstr.inc}
{$endif}
{$endif FPC_USE_LIBC}

{$ifdef cpui386}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i i386.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpui386}

{$ifdef cpui8086}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i i8086.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpui8086}

{$ifdef cpum68k}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i m68k.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpum68k}

{$ifdef cpux86_64}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i x86_64.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpux86_64}

{$ifdef cpupowerpc32}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i powerpc.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpupowerpc32}

{$ifdef cpupowerpc64}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i powerpc64.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpupowerpc64}

{$ifdef cpualpha}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i alpha.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpualpha}

{$ifdef cpuiA64}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i ia64.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpuiA64}

{$ifdef cpusparc}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i sparc.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpusparc}

{$ifdef cpuarm}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$i armdefines.inc}
  {$if defined(CPUTHUMB2)}
    {$i thumb2.inc}  { Case dependent, don't change }
  {$else}
    {$if defined(CPUTHUMB)}
      {$i thumb.inc}  { Case dependent, don't change }
    {$else}
      {$i arm.inc}  { Case dependent, don't change }
    {$endif}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  {$if defined(CPUCORTEXM3) or defined(CPUARMV7M)}
    {$i thumb2.inc}  { Case dependent, don't change }
  {$else}
    {$i arm.inc}  { Case dependent, don't change }
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
  {$endif}
  {$define SYSPROCDEFINED}
{$endif cpuarm}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef cpuavr}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i avr.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpuavr}

{$ifdef cpumipsel}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  { there is no mipsel.inc, we use mips.inc instead }
  {$i mips.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$else not cpumipsel}
{$ifdef cpumips}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i mips.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpumips}
{$endif not cpumipsel}

{$ifdef cpuaarch64}
  {$ifdef SYSPROCDEFINED}
    {$Error Can't determine processor type !}
  {$endif}
  {$i aarch64.inc}  { Case dependent, don't change }
  {$define SYSPROCDEFINED}
{$endif cpuaarch64}

{$ifndef SYSPROCDEFINED}
  {$Error Can't determine processor type !}
{$endif}

procedure fillchar(var x;count : {$ifdef FILLCHAR_HAS_SIZEUINT_COUNT}SizeUInt{$else}SizeInt{$endif};value : boolean);
=======
=======
>>>>>>> origin/fixes_2_2

procedure fillchar(var x;count : SizeInt;value : boolean);{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/fixes_2_2
begin
  fillchar(x,count,byte(value));
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure fillchar(var x;count : {$ifdef FILLCHAR_HAS_SIZEUINT_COUNT}SizeUInt{$else}SizeInt{$endif};value : char);
=======
=======
>>>>>>> origin/fixes_2_2
procedure fillchar(var x;count : SizeInt;value : char);{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/fixes_2_2
begin
  fillchar(x,count,byte(value));
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure FillByte (var x;count : {$ifdef FILLCHAR_HAS_SIZEUINT_COUNT}SizeUInt{$else}SizeInt{$endif};value : byte );
=======
procedure FillByte (var x;count : SizeInt;value : byte ); {$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/fixes_2_2
=======
procedure FillByte (var x;count : SizeInt;value : byte ); {$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> origin/fixes_2_2
begin
  FillChar (X,Count,VALUE);
end;


<<<<<<< HEAD
<<<<<<< HEAD
function IndexChar(Const buf;len:SizeInt;b:char):SizeInt;
=======
function IndexChar(Const buf;len:SizeInt;b:char):SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/fixes_2_2
=======
function IndexChar(Const buf;len:SizeInt;b:char):SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> origin/fixes_2_2
begin
  IndexChar:=IndexByte(Buf,Len,byte(B));
end;


<<<<<<< HEAD
<<<<<<< HEAD
function CompareChar(Const buf1,buf2;len:SizeInt):SizeInt;
=======
function CompareChar(Const buf1,buf2;len:SizeInt):SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/fixes_2_2
=======
function CompareChar(Const buf1,buf2;len:SizeInt):SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> origin/fixes_2_2
begin
  CompareChar:=CompareByte(buf1,buf2,len);
end;

<<<<<<< HEAD
<<<<<<< HEAD

procedure fpc_zeromem(p:pointer;len:ptruint);
begin
  FillChar(p^,len,0);
end;


procedure fpc_fillmem(out data;len:ptruint;b : byte);
begin
  FillByte(data,len,b);
end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ Include generic pascal only routines which are not defined in the processor
  specific include file }
{$I generic.inc}


{****************************************************************************
                                Set Handling
****************************************************************************}

{ Include set support which is processor specific}
{$i set.inc}
{ Include generic pascal routines for sets if the processor }
{ specific routines are not available.                      }
{$i genset.inc}


{****************************************************************************
                               Math Routines
****************************************************************************}

function Hi(b : byte): byte;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   Hi := b shr 4
end;

function Lo(b : byte): byte;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   Lo := b and $0f
end;

Function Swap (X : Word) : Word;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  Swap := SwapEndian(X);
End;

Function Swap (X : Integer) : Integer;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  Swap := SwapEndian(X);
End;

Function Swap (X : Longint) : Longint;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  Swap:=(X shl 16) + (X shr 16);
End;

Function Swap (X : Cardinal) : Cardinal;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  Swap:=(X shl 16) + (X shr 16);
End;

Function Swap (X : QWord) : QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  Swap:=(X shl 32) + (X shr 32);
End;

Function Swap (X : Int64) : Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  Swap:=(X shl 32) + (X shr 32);
End;

{$ifdef SUPPORT_DOUBLE}
operator := (b:real48) d:double;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
 D:=real2double(b);
end;
{$endif SUPPORT_DOUBLE}

{$ifdef SUPPORT_EXTENDED}
operator := (b:real48) e:extended;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
 e:=real2double(b);
end;
{$endif SUPPORT_EXTENDED}

{$ifndef FPUNONE}
{$ifdef FPC_USE_LIBC}
{ Include libc versions }
{$i cgenmath.inc}
{$endif FPC_USE_LIBC}
{ Include processor specific routines }
{$I math.inc}
{ Include generic version }
{$I genmath.inc}
{$endif}

{$i gencurr.inc}


function aligntoptr(p : pointer) : pointer;inline;
  begin
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
    result:=align(p,sizeof(p));
{$else FPC_REQUIRES_PROPER_ALIGNMENT}
    result:=p;
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
  end;


{****************************************************************************
                  Subroutines for String handling
****************************************************************************}

{ Needs to be before RTTI handling }

{$i sstrings.inc}

{ requires sstrings.inc for initval }
{$I int64p.inc}
{$I int64.inc}

{Requires int64.inc, since that contains the VAL functions for int64 and qword}
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
{$i astrings.inc}
{$endif FPC_HAS_FEATURE_ANSISTRINGS}

{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    {$i wstrings.inc}
  {$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
  {$i ustrings.inc}
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

{$i aliases.inc}

{*****************************************************************************
                        Dynamic Array support
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
{$i dynarr.inc}
{$endif FPC_HAS_FEATURE_DYNARRAYS}

{*****************************************************************************
                        Object Pascal support
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_CLASSES}
{$i objpas.inc}
{$endif FPC_HAS_FEATURE_CLASSES}

{*****************************************************************************
                            Variant support
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_VARIANTS}
{$i variant.inc}
{$endif FPC_HAS_FEATURE_VARIANTS}

{****************************************************************************
                         Run-Time Type Information (RTTI)
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_RTTI}
{$i rtti.inc}
{$endif FPC_HAS_FEATURE_RTTI}

{$if defined(FPC_HAS_FEATURE_RANDOM)}

{----------------------------------------------------------------------
   Mersenne Twister: A 623-Dimensionally Equidistributed Uniform
   Pseudo-Random Number Generator.

   What is Mersenne Twister?
   Mersenne Twister(MT) is a pseudorandom number generator developped by
   Makoto Matsumoto and Takuji Nishimura (alphabetical order) during
   1996-1997. MT has the following merits:
   It is designed with consideration on the flaws of various existing
   generators.
   Far longer period and far higher order of equidistribution than any
   other implemented generators. (It is proved that the period is 2^19937-1,
   and 623-dimensional equidistribution property is assured.)
   Fast generation. (Although it depends on the system, it is reported that
   MT is sometimes faster than the standard ANSI-C library in a system
   with pipeline and cache memory.)
   Efficient use of the memory. (The implemented C-code mt19937.c
   consumes only 624 words of working area.)

   home page
     http://www.math.keio.ac.jp/~matumoto/emt.html
   original c source
     http://www.math.keio.ac.jp/~nisimura/random/int/mt19937int.c

   Coded by Takuji Nishimura, considering the suggestions by
   Topher Cooper and Marc Rieffel in July-Aug. 1997.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later
   version.
   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
   See the GNU Library General Public License for more details.
   You should have received a copy of the GNU Library General
   Public License along with this library; if not, write to the
   Free Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307  USA

   Copyright (C) 1997, 1999 Makoto Matsumoto and Takuji Nishimura.
   When you use this, send an email to: matumoto@math.keio.ac.jp
   with an appropriate reference to your work.

   REFERENCE
   M. Matsumoto and T. Nishimura,
   "Mersenne Twister: A 623-Dimensionally Equidistributed Uniform
   Pseudo-Random Number Generator",
   ACM Transactions on Modeling and Computer Simulation,
   Vol. 8, No. 1, January 1998, pp 3--30.


  Translated to OP and Delphi interface added by Roman Krejci (6.12.1999)

  http://www.rksolution.cz/delphi/tips.htm

  Revised 21.6.2000: Bug in the function RandInt_MT19937 fixed

  2003/10/26: adapted to use the improved intialisation mentioned at
  <http://www.math.keio.ac.jp/~matumoto/MT2002/emt19937ar.html> and
  removed the assembler code

 ----------------------------------------------------------------------}

{$R-} {range checking off}
{$Q-} {overflow checking off}

{ Period parameter }
Const
  MT19937N=624;

Type
  tMT19937StateArray = array [0..MT19937N-1] of longint; // the array for the state vector

{ Period parameters }
const
  MT19937M=397;
  MT19937MATRIX_A  =$9908b0df;  // constant vector a
  MT19937UPPER_MASK=longint($80000000);  // most significant w-r bits
  MT19937LOWER_MASK=longint($7fffffff);  // least significant r bits

{ Tempering parameters }
  TEMPERING_MASK_B=longint($9d2c5680);
  TEMPERING_MASK_C=longint($efc60000);


VAR
  mt : tMT19937StateArray;
  mti: longint=MT19937N+1; // mti=MT19937N+1 means mt[] is not initialized

{ Initializing the array with a seed }
procedure sgenrand_MT19937(seed: longint);
var
  i: longint;
begin
  mt[0] := seed;
  for i := 1 to MT19937N-1 do
    begin
      mt[i] := 1812433253 * (mt[i-1] xor (mt[i-1] shr 30)) + i;
      { See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier. }
      { In the previous versions, MSBs of the seed affect   }
      { only MSBs of the array mt[].                        }
      { 2002/01/09 modified by Makoto Matsumoto             }
    end;
  mti := MT19937N;
end;


function genrand_MT19937: longint;
const
  mag01 : array [0..1] of longint =(0, longint(MT19937MATRIX_A));
var
  y: longint;
  kk: longint;
begin
  if RandSeed<>OldRandSeed then
    mti:=MT19937N+1;
  if (mti >= MT19937N) { generate MT19937N longints at one time }
  then begin
     if mti = (MT19937N+1) then  // if sgenrand_MT19937() has not been called,
       begin
         sgenrand_MT19937(randseed);   // default initial seed is used
         { hack: randseed is not used more than once in this algorithm. Most }
         {  user changes are re-initialising reandseed with the value it had }
         {  at the start -> with the "not", we will detect this change.      }
         {  Detecting other changes is not useful, since the generated       }
         {  numbers will be different anyway.                                }
         randseed := not(randseed);
         oldrandseed := randseed;
       end;
     for kk:=0 to MT19937N-MT19937M-1 do begin
        y := (mt[kk] and MT19937UPPER_MASK) or (mt[kk+1] and MT19937LOWER_MASK);
        mt[kk] := mt[kk+MT19937M] xor (y shr 1) xor mag01[y and $00000001];
     end;
     for kk:= MT19937N-MT19937M to MT19937N-2 do begin
       y := (mt[kk] and MT19937UPPER_MASK) or (mt[kk+1] and MT19937LOWER_MASK);
       mt[kk] := mt[kk+(MT19937M-MT19937N)] xor (y shr 1) xor mag01[y and $00000001];
     end;
     y := (mt[MT19937N-1] and MT19937UPPER_MASK) or (mt[0] and MT19937LOWER_MASK);
     mt[MT19937N-1] := mt[MT19937M-1] xor (y shr 1) xor mag01[y and $00000001];
     mti := 0;
  end;
  y := mt[mti]; inc(mti);
  y := y xor (y shr 11);
  y := y xor (y shl 7)  and TEMPERING_MASK_B;
  y := y xor (y shl 15) and TEMPERING_MASK_C;
  y := y xor (y shr 18);
  Result := y;
end;


function random(l:longint): longint;
begin
  { otherwise we can return values = l (JM) }
  if (l < 0) then
    inc(l);
  random := longint((int64(cardinal(genrand_MT19937))*l) shr 32);
end;

function random(l:int64): int64;
begin
  { always call random, so the random generator cycles (TP-compatible) (JM) }
  random := int64((qword(cardinal(genrand_MT19937)) or ((qword(cardinal(genrand_MT19937)) shl 32))) and $7fffffffffffffff);
  if (l<>0) then
    random := random mod l
  else
    random := 0;
end;

{$ifndef FPUNONE}
function random: extended;
begin
  random := cardinal(genrand_MT19937) * (extended(1.0)/(int64(1) shl 32));
end;
{$endif}
{$endif FPC_HAS_FEATURE_RANDOM}


{****************************************************************************
                            Memory Management
****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_PTR}
Function Ptr(sel,off : {$ifdef CPU16}Word{$else}Longint{$endif}) : farpointer;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  ptr:=farpointer((sel shl 4)+off);
End;
{$endif not FPC_SYSTEM_HAS_PTR}

{$ifndef FPC_SYSTEM_HAS_CSEG}
Function CSeg : Word;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  Cseg:=0;
End;
{$endif not FPC_SYSTEM_HAS_CSEG}

{$ifndef FPC_SYSTEM_HAS_DSEG}
Function DSeg : Word;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  Dseg:=0;
End;
{$endif not FPC_SYSTEM_HAS_DSEG}

{$ifndef FPC_SYSTEM_HAS_SSEG}
Function SSeg : Word;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  Sseg:=0;
End;
{$endif not FPC_SYSTEM_HAS_SSEG}



{$push}
{$R-}
{$I-}
{$Q-}

{*****************************************************************************
                             Miscellaneous
*****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_STACKTOP}
function StackTop: pointer;
begin
  result:=StackBottom+StackLength;
end;
{$endif FPC_SYSTEM_HAS_STACKTOP}

{$ifndef FPC_SYSTEM_HAS_GET_PC_ADDR}
  { This provides a dummy implementation
    of get_pc_addr function, for CPU's that don't need
    the instruction address to walk the stack. }
function get_pc_addr : codepointer;
begin
  get_pc_addr:=nil;
end;
{$endif ndef FPC_SYSTEM_HAS_GET_PC_ADDR}

{$ifndef FPC_SYSTEM_HAS_GET_CALLER_STACKINFO}
  { This provides a simple implementation
    of get_caller_stackinfo procedure,
    using get_caller_addr and get_caller_frame
    functions. }
procedure get_caller_stackinfo(var framebp : pointer; var addr : codepointer);
var
  nextbp : pointer;
  nextaddr : codepointer;
begin
  nextbp:=get_caller_frame(framebp,addr);
  nextaddr:=get_caller_addr(framebp,addr);
  framebp:=nextbp;
  addr:=nextaddr;
end;
{$endif ndef FPC_SYSTEM_HAS_GET_CALLER_STACKINFO}

procedure fpc_objecterror; compilerproc;
begin
  HandleErrorAddrFrameInd(210,get_pc_addr,get_frame);
end;

procedure fpc_rangeerror;[public,alias:'FPC_RANGEERROR']; compilerproc;
begin
  HandleErrorAddrFrameInd(201,get_pc_addr,get_frame);
end;


procedure fpc_divbyzero;[public,alias:'FPC_DIVBYZERO']; compilerproc;
begin
  HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
end;


procedure fpc_overflow;[public,alias:'FPC_OVERFLOW']; compilerproc;
begin
  HandleErrorAddrFrameInd(215,get_pc_addr,get_frame);
end;


procedure fpc_threaderror; [public,alias:'FPC_THREADERROR'];
begin
  HandleErrorAddrFrameInd(6,get_pc_addr,get_frame);
end;


procedure fpc_invalidpointer; [public,alias:'FPC_INVALIDPOINTER'];
begin
  HandleErrorAddrFrameInd(216,get_pc_addr,get_frame);
end;


procedure fpc_threaderror; [public,alias:'FPC_THREADERROR'];
begin
  HandleErrorFrame(6,get_frame);
end;


procedure fpc_threaderror; [public,alias:'FPC_THREADERROR'];
begin
  HandleErrorFrame(6,get_frame);
end;


procedure fpc_iocheck;[public,alias:'FPC_IOCHECK']; compilerproc;
var
  l : longint;
  HInoutRes : PWord;
begin
  HInOutRes:=@InoutRes;
  if HInOutRes^<>0 then
   begin
     l:=HInOutRes^;
     HInOutRes^:=0;
<<<<<<< HEAD
<<<<<<< HEAD
     HandleErrorAddrFrameInd(l,get_pc_addr,get_frame);
=======
=======
>>>>>>> origin/fixes_2_2
     HandleErrorFrame(l,get_frame);
>>>>>>> graemeg/fixes_2_2
   end;
end;


Function IOResult:Word;
var
  HInoutRes : PWord;
Begin
  HInoutRes:=@InoutRes;
  IOResult:=HInOutRes^;
  HInOutRes^:=0;
End;


Function GetThreadID:TThreadID;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
(* ThreadID is stored in a threadvar and made available in interface *)
(* to allow setup of this value during thread initialization.        *)
  GetThreadID := ThreadID;
end;


function fpc_safecallcheck(res : hresult) : hresult;[public,alias:'FPC_SAFECALLCHECK']; compilerproc; {$ifdef CPU86} register; {$endif}
begin
  if res<0 then
    begin
      if assigned(SafeCallErrorProc) then
        SafeCallErrorProc(res,get_frame);
      HandleErrorAddrFrameInd(229,get_pc_addr,get_frame);
    end;
  result:=res;
end;


{*****************************************************************************
                         Stack check code
*****************************************************************************}

{ be compatible with old code }
{$ifdef FPC_NO_GENERIC_STACK_CHECK}
{$define NO_GENERIC_STACK_CHECK}
{$endif FPC_NO_GENERIC_STACK_CHECK}

{$IFNDEF NO_GENERIC_STACK_CHECK}

{$PUSH}
{$S-}
procedure fpc_stackcheck(stack_size:SizeUInt);[public,alias:'FPC_STACKCHECK']; compilerproc;
var
  c : Pointer;
begin
  { Avoid recursive calls when called from the exit routines }
  if StackError then
   exit;
  { don't use sack_size, since the stack pointer has already been
    decreased when this routine is called
  }
  c := Sptr - STACK_MARGIN;
  if (c <= StackBottom) then
   begin
     StackError:=true;
     HandleError(202);
   end;
end;
{$POP}

{$ENDIF NO_GENERIC_STACK_CHECK}

{*****************************************************************************
                        Initialization / Finalization
*****************************************************************************}

const
  maxunits=1024; { See also files.pas of the compiler source }
type
  TInitFinalRec=record
    InitProc,
    FinalProc : TProcedure;
  end;
  TInitFinalTable = record
    TableCount,
    InitCount  : {$if defined(VER2)}longint
                 {$elseif defined(VER3_0)}sizeint
                 {$else}ALUUInt{$endif}
                 ;
    Procs      : array[1..maxunits] of TInitFinalRec;
  end;
  PInitFinalTable = ^TInitFinalTable;
<<<<<<< HEAD
<<<<<<< HEAD


=======


>>>>>>> graemeg/fixes_2_2
=======


>>>>>>> origin/fixes_2_2
{$ifndef FPC_HAS_INDIRECT_MAIN_INFORMATION}
var
  InitFinalTable : TInitFinalTable;external name 'INITFINAL';
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}


procedure fpc_InitializeUnits;[public,alias:'FPC_INITIALIZEUNITS']; compilerproc;
var
  i : ALUUInt;
{$ifdef DEBUG}
  pt : PInitFinalTable;
{$endif}
begin
  { call cpu/fpu initialisation routine }
  fpc_cpuinit;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef DEBUG}
  pt := PInitFinalTable(EntryInformation.InitFinalTable);
{$endif}
  with PInitFinalTable(EntryInformation.InitFinalTable)^ do
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
{$ifdef DEBUG}
  pt := @InitFinalTable;
{$endif}
=======
  with PInitFinalTable(EntryInformation.InitFinalTable)^ do
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
>>>>>>> graemeg/cpstrnew
=======
  with PInitFinalTable(EntryInformation.InitFinalTable)^ do
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
>>>>>>> graemeg/cpstrnew
=======
  with PInitFinalTable(EntryInformation.InitFinalTable)^ do
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
>>>>>>> graemeg/cpstrnew
=======
  with PInitFinalTable(EntryInformation.InitFinalTable)^ do
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
>>>>>>> origin/cpstrnew
  with InitFinalTable do
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
=======
  with {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PInitFinalTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
    InitFinalTable
    {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION} do
>>>>>>> graemeg/fixes_2_2
=======
  with {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PInitFinalTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
    InitFinalTable
    {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION} do
>>>>>>> origin/fixes_2_2
   begin
     for i:=1 to ALUUInt(TableCount) do
      begin
        if assigned(Procs[i].InitProc) then
         Procs[i].InitProc();
        InitCount:=i;
      end;
   end;
  if assigned(InitProc) then
    TProcedure(InitProc)();
end;


procedure internal_initializeunits; external name 'FPC_INITIALIZEUNITS';

procedure fpc_LibInitializeUnits;[public,alias:'FPC_LIBINITIALIZEUNITS'];compilerproc;
begin
{$ifdef FPC_HAS_FEATURE_DYNLIBS}
  IsLibrary:=true;
  { must also be set to true for packages when implemented }
  ModuleIsLib:=true;
  internal_initializeunits;
{$endif FPC_HAS_FEATURE_DYNLIBS}
end;


{$ifdef CPUAVR}
procedure FinalizeUnits; external name 'FPC_FINALIZE_FUNC_TABLE';

{$else CPUAVR}
procedure FinalizeUnits;[public,alias:'FPC_FINALIZEUNITS'];
begin
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}
  with PInitFinalTable(EntryInformation.InitFinalTable)^ do
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
  with InitFinalTable do
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
=======
  with {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PInitFinalTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
    InitFinalTable
    {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION} do
>>>>>>> graemeg/fixes_2_2
=======
  with {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PInitFinalTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
    InitFinalTable
    {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION} do
>>>>>>> origin/fixes_2_2
   begin
     while (InitCount>0) do
      begin
        // we've to decrement the cound before calling the final. code
        // else a halt in the final. code leads to a endless loop
        dec(InitCount);
        if assigned(Procs[InitCount+1].FinalProc) then
         Procs[InitCount+1].FinalProc();
      end;
   end;
end;
{$endif CPUAVR}

{*****************************************************************************
                          Error / Exit / ExitProc
*****************************************************************************}

Procedure system_exit;forward;
{$ifdef FPC_HAS_FEATURE_HEAP}
{$ifndef HAS_MEMORYMANAGER}
//not needed if independant memory manager
Procedure FinalizeHeap;forward;
{$endif HAS_MEMORYMANAGER}
{$endif FPC_HAS_FEATURE_HEAP}

{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
procedure SysFlushStdIO;
begin
  { Make sure that all output is written to the redirected file }
  if Textrec(Output).Mode=fmOutput then
    Flush(Output);
  if Textrec(ErrOutput).Mode=fmOutput then
    Flush(ErrOutput);
  if Textrec(stdout).Mode=fmOutput then
    Flush(stdout);
  if Textrec(StdErr).Mode=fmOutput then
    Flush(StdErr);
end;
{$endif FPC_HAS_FEATURE_CONSOLEIO}

Procedure InternalExit;
var
  current_exit : Procedure;
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
  pstdout : ^Text;
{$endif}
{$if defined(MSWINDOWS) or defined(OS2)}
  i : longint;
{$endif}
Begin
{$ifdef SYSTEMDEBUG}
  writeln('InternalExit');
{$endif SYSTEMDEBUG}
{$ifndef CPUAVR}
  while exitProc<>nil Do
   Begin
     InOutRes:=0;
     current_exit:=tProcedure(exitProc);
     exitProc:=nil;
     current_exit();
   End;
{$endif CPUAVR}

{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
  { the embedded system unit itself contains no routines for console i/o
    console i/o is done by the Consoleio unit which can do things like
    redirection to seriell i/o }
{$ifndef EMBEDDED}
  { Show runtime error and exit }
  if WriteErrorsToStdErr then
    pstdout:=@stderr
  else  
    pstdout:=@stdout;
  If erroraddr<>nil Then
   Begin
     Writeln(pstdout^,'Runtime error ',Errorcode,' at $',hexstr(erroraddr));
     { to get a nice symify }
     Writeln(pstdout^,BackTraceStrFunc(Erroraddr));
     dump_stack(pstdout^,ErrorBase,ErrorAddr);
     Writeln(pstdout^,'');
   End;
<<<<<<< HEAD
  SysFlushStdIO;
{$endif EMBEDDED}
=======

  { Make sure that all output is written to the redirected file }
  if Textrec(Output).Mode=fmOutput then
    Flush(Output);
  if Textrec(ErrOutput).Mode=fmOutput then
    Flush(ErrOutput);
  if Textrec(pstdout^).Mode=fmOutput then
    Flush(pstdout^);
  if Textrec(StdErr).Mode=fmOutput then
    Flush(StdErr);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif FPC_HAS_FEATURE_CONSOLEIO}

  { Finalize units }
  FinalizeUnits;

{$if defined(MSWINDOWS) or defined(OS2)}
  { finally release the heap if possible, especially
    important for DLLs.
    Reset the array to nil, and finally also argv itself to
    avoid double freeing problem in case this function gets called twice. }
  if assigned(argv) then
    begin
      for i:=0 to argc-1 do
        if assigned(argv[i]) then
          begin
            sysfreemem(argv[i]);
            argv[i]:=nil;
          end;
      sysfreemem(argv);
      argv:=nil;
    end;
{$endif}
{$ifdef LINUX}
  {sysfreemem already checks for nil}
  { Do not try to do anything if the heap manager already reported an error }
  if (errorcode<>203) and (errorcode<>204) then
    sysfreemem(calculated_cmdline);
{$endif}
{$ifdef BSD}
  { Do not try to do anything if the heap manager already reported an error }
  if (errorcode<>203) and (errorcode<>204) then
    sysfreemem(cmdline);
{$endif}

{$ifdef FPC_HAS_FEATURE_HEAP}
{$ifndef HAS_MEMORYMANAGER}
{$ifndef FPC_NO_DEFAULT_HEAP}
  FinalizeHeap;
{$endif not FPC_NO_DEFAULT_HEAP}
{$endif not HAS_MEMORYMANAGER}
{$endif FPC_HAS_FEATURE_HEAP}
End;


Procedure fpc_do_exit;[Public,Alias:'FPC_DO_EXIT']; compilerproc;
begin
  InternalExit;
  System_exit;
end;

procedure internal_do_exit; external name 'FPC_DO_EXIT';


Procedure lib_exit;[Public,Alias:'FPC_LIB_EXIT'];
begin
  InternalExit;
end;


Procedure Halt(ErrNum: Longint);
Begin
{$ifdef FPC_HAS_FEATURE_EXITCODE}
{$ifdef FPC_LIMITED_EXITCODE}
  if ErrNum > maxExitCode then
    ExitCode:=255
  else
{$endif FPC_LIMITED_EXITCODE}
    ExitCode:=ErrNum;
{$endif FPC_HAS_FEATURE_EXITCODE}
  internal_do_exit;
end;


function SysBackTraceStr (Addr: CodePointer): ShortString;
begin
  SysBackTraceStr:='  $'+hexstr(addr);
end;


{$ifndef FPC_SYSTEM_HAS_CAPTUREBACKTRACE}
function CaptureBacktrace(skipframes,count:sizeint;frames:PCodePointer):sizeint;
var
  curr_frame,prev_frame: pointer;
  curr_addr: codepointer;
  i: sizeint;
begin
  curr_frame:=get_frame;
  curr_addr:=get_pc_addr;
  prev_frame:=curr_frame;
  get_caller_stackinfo(curr_frame,curr_addr);
  i:=-skipframes;
  while (i<count) and (curr_frame>prev_frame) and
     (curr_frame<StackTop) do
    begin
      if (curr_addr=nil) or
         (curr_frame=nil) then
        break;
      if (i>=0) then
        frames[i]:=curr_addr;
      inc(i);
      prev_frame:=curr_frame;
      get_caller_stackinfo(curr_frame,curr_addr);
    end;
  if i<0 then
    result:=0
  else
    result:=i;
end;
{$endif FPC_SYSTEM_HAS_CAPTUREBACKTRACE}


Procedure HandleErrorAddrFrame (Errno : longint;addr : CodePointer; frame : Pointer);[public,alias:'FPC_BREAK_ERROR']; {$ifdef CPUI386} register; {$endif}
begin
  If codepointer(ErrorProc)<>Nil then
    ErrorProc(Errno,addr,frame);
  errorcode:=word(Errno);
  erroraddr:=addr;
  errorbase:=frame;
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
  if ExceptAddrStack <> nil then
    raise TObject(nil) at addr,frame;
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
  Halt(errorcode);
end;

{ This is used internally by system skip first level,
  and generated the same output as before, when
  HandleErrorFrame function was used internally. }
Procedure HandleErrorAddrFrameInd (Errno : longint;addr : CodePointer; frame : Pointer);
begin
  get_caller_stackinfo (frame, addr);
  HandleErrorAddrFrame (Errno,addr,frame);
end;

Procedure HandleErrorFrame (Errno : longint;frame : Pointer);
{
  Procedure to handle internal errors, i.e. not user-invoked errors
  Internal function should ALWAYS call HandleError instead of RunError.
  Can be used for exception handlers to specify the frame
}
begin
  HandleErrorAddrFrame(Errno,get_caller_addr(frame),get_caller_frame(frame));
end;


procedure fpc_handleerror (Errno : longint); compilerproc; [public,alias : 'FPC_HANDLEERROR'];
{
  Procedure to handle internal errors, i.e. not user-invoked errors
  Internal function should ALWAYS call HandleError instead of RunError.
}
begin
  HandleErrorAddrFrameInd(Errno,get_pc_addr,get_frame);
end;


procedure RunError(w : word);[alias: 'FPC_RUNERROR'];
var
  bp : pointer;
  pcaddr : codepointer;
begin
  errorcode:=w;
  pcaddr:=get_pc_addr;
  bp:=get_frame;
  get_caller_stackinfo(bp,pcaddr);
  erroraddr:=pcaddr;
  errorbase:=bp;
  Halt(errorcode);
end;


Procedure RunError;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  RunError (0);
End;


Procedure Halt;{$ifdef SYSTEMINLINE}inline;{$endif}
Begin
  Halt(0);
End;


Procedure Error(RunTimeError : TRunTimeError);
begin
  RunError(RuntimeErrorExitCodes[RunTimeError]);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure dump_stack(var f : text;fp : Pointer; addr : CodePointer);
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
Procedure dump_stack(var f : text;bp : Pointer);
>>>>>>> graemeg/fixes_2_2
var
  i : Longint;
  prevfp : Pointer;
  is_dev : boolean;
Begin
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
  try
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
    { Frame of this procedure acts as StackBottom, fp values below that are invalid. }
    prevfp:=get_frame;
    i:=0;
    is_dev:=do_isdevice(textrec(f).Handle);
    { sanity checks, new frame pointer must be always greater than the old one, further
      it must point into the stack area, else something went wrong }
    while (fp>prevfp) and (fp<StackTop) do
     Begin
       prevfp:=fp;
       get_caller_stackinfo(fp,addr);
       if (addr=nil) then
         break;
       Writeln(f,BackTraceStrFunc(addr));
       if (fp=nil) then
         break;
       Inc(i);
       If ((i>max_frame_dump) and is_dev) or (i>256) Then
         break;
     End;
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
   except
     { prevent endless dump if an exception occured }
   end;
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
End;


<<<<<<< HEAD
<<<<<<< HEAD
procedure dump_stack(var f: text; skipframes: longint);
var
  i,count: longint;
  frames: array [0..255] of codepointer;
begin
  if do_isdevice(textrec(f).handle) then
    count:=max_frame_dump
  else
    count:=255;
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
  try
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
    count:=CaptureBacktrace(skipframes+1,count,@frames[0]);
    for i:=0 to count-1 do
      writeln(f,BackTraceStrFunc(frames[i]));
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
  except
  end;
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
end;


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
procedure DumpExceptionBackTrace(var f:text);
var
  FrameNumber,
  FrameCount   : longint;
<<<<<<< HEAD
<<<<<<< HEAD
  Frames       : PCodePointer;
=======
  Frames       : PPointer;
>>>>>>> graemeg/fixes_2_2
=======
  Frames       : PPointer;
>>>>>>> origin/fixes_2_2
begin
  if RaiseList=nil then
    exit;
  WriteLn(f,BackTraceStrFunc(RaiseList^.Addr));
  FrameCount:=RaiseList^.Framecount;
  Frames:=RaiseList^.Frames;
  for FrameNumber := 0 to FrameCount-1 do
    WriteLn(f,BackTraceStrFunc(Frames[FrameNumber]));
end;
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

{$endif FPC_HAS_FEATURE_CONSOLEIO}
>>>>>>> graemeg/fixes_2_2


{$ifdef FPC_HAS_FEATURE_HEAP}
Type
  PExitProcInfo = ^TExitProcInfo;
  TExitProcInfo = Record
    Next     : PExitProcInfo;
    SaveExit : CodePointer;
    Proc     : TProcedure;
  End;
const
  ExitProcList: PExitProcInfo = nil;

Procedure DoExitProc;
var
  P    : PExitProcInfo;
  Proc : TProcedure;
Begin
  P:=ExitProcList;
  ExitProcList:=P^.Next;
  ExitProc:=P^.SaveExit;
  Proc:=P^.Proc;
  DisPose(P);
  Proc();
End;


Procedure AddExitProc(Proc: TProcedure);
var
  P : PExitProcInfo;
Begin
  New(P);
  P^.Next:=ExitProcList;
  P^.SaveExit:=ExitProc;
  P^.Proc:=Proc;
  ExitProcList:=P;
  ExitProc:=@DoExitProc;
End;
{$endif FPC_HAS_FEATURE_HEAP}


{$ifdef FPC_HAS_FEATURE_HEAP}
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
function ArrayStringToPPchar(const S:Array of AnsiString;reserveentries:Longint):ppchar; // const ?
// Extra allocate reserveentries pchar's at the beginning (default param=0 after 1.0.x ?)
// Note: for internal use by skilled programmers only
// if "s" goes out of scope in the parent procedure, the pointer is dangling.

var p   : ppchar;
    i   : LongInt;
begin
  if High(s)<Low(s) Then Exit(NIL);
  Getmem(p,sizeof(pchar)*(high(s)-low(s)+ReserveEntries+2));  // one more for NIL, one more
                                              // for cmd
  if p=nil then
    begin
      {$ifdef xunix}
      fpseterrno(ESysEnomem);
      {$endif}
      exit(NIL);
    end;
  for i:=low(s) to high(s) do
     p[i+Reserveentries]:=pchar(s[i]);
  p[high(s)+1+Reserveentries]:=nil;
  ArrayStringToPPchar:=p;
end;


Function StringToPPChar(Var S:AnsiString;ReserveEntries:integer):ppchar;
{
  Create a PPChar to structure of pchars which are the arguments specified
  in the string S. Especially useful for creating an ArgV for Exec-calls
}

begin
  StringToPPChar:=StringToPPChar(PChar(S),ReserveEntries);
end;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


Function StringToPPChar(S: PChar;ReserveEntries:integer):ppchar;

var
  i,nr  : longint;
  Buf : ^char;
  p   : ppchar;

begin
  buf:=s;
  nr:=1;
  while (buf^<>#0) do                   // count nr of args
   begin
     while (buf^ in [' ',#9,#10]) do    // Kill separators.
      inc(buf);
     inc(nr);
     if buf^='"' Then                   // quotes argument?
      begin
        inc(buf);
        while not (buf^ in [#0,'"']) do // then end of argument is end of string or next quote
         inc(buf);
        if buf^='"' then                // skip closing quote.
          inc(buf);
      end
     else
       begin                            // else std
         while not (buf^ in [' ',#0,#9,#10]) do
           inc(buf);
       end;
   end;
  getmem(p,(ReserveEntries+nr)*sizeof(pchar));
  StringToPPChar:=p;
  if p=nil then
   begin
     {$ifdef xunix}
     fpseterrno(ESysEnomem);
     {$endif}
     exit;
   end;
  for i:=1 to ReserveEntries do inc(p); // skip empty slots
  buf:=s;
  while (buf^<>#0) do
   begin
     while (buf^ in [' ',#9,#10]) do    // Kill separators.
      begin
       buf^:=#0;
       inc(buf);
      end;
     if buf^='"' Then                   // quotes argument?
      begin
        inc(buf);
        p^:=buf;
        inc(p);
        p^:=nil;
        while not (buf^ in [#0,'"']) do // then end of argument is end of string or next quote
         inc(buf);
        if buf^='"' then                // skip closing quote.
          begin
            buf^:=#0;
            inc(buf);
          end;
      end
     else
       begin
        p^:=buf;
        inc(p);
        p^:=nil;
         while not (buf^ in [' ',#0,#9,#10]) do
           inc(buf);
       end;
   end;
end;
{$endif FPC_HAS_FEATURE_HEAP}


{*****************************************************************************
                          Abstract/Assert support.
*****************************************************************************}

procedure fpc_emptymethod;[public,alias : 'FPC_EMPTYMETHOD'];
begin
end;


procedure fpc_AbstractErrorIntern;compilerproc;[public,alias : 'FPC_ABSTRACTERROR'];
begin
  If codepointer(AbstractErrorProc)<>nil then
    AbstractErrorProc();
  HandleErrorAddrFrameInd(211,get_pc_addr,get_frame);
end;


Procedure fpc_assert(Const Msg,FName:Shortstring;LineNo:Longint;
   ErrorAddr:Pointer); [Public,Alias : 'FPC_ASSERT']; compilerproc;
begin
  if codepointer(AssertErrorProc)<>nil then
    AssertErrorProc(Msg,FName,LineNo,ErrorAddr)
  else
    HandleErrorAddrFrameInd(227,get_pc_addr,get_frame);
end;


Procedure SysAssert(Const Msg,FName:Shortstring;LineNo:Longint;ErrorAddr:Pointer);
begin
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
  If msg='' then
    write(stderr,'Assertion failed')
  else
    write(stderr,msg);
  Writeln(stderr,' (',FName,', line ',LineNo,').');
  Writeln(stderr,'');
  Halt(227);
{$endif FPC_HAS_FEATURE_CONSOLEIO}
end;


{*****************************************************************************
                       SetJmp/LongJmp support.
*****************************************************************************}

{$i setjump.inc}


<<<<<<< HEAD
{$pop} //{$I-,R-,Q-} before 'procedure fpc_rangeerror'
=======
{$ifdef IOCheckWasOn}
{$I+}
{$endif}

{$ifdef RangeCheckWasOn}
{$R+}
{$endif}

{$ifdef OverflowCheckWasOn}
{$Q+}
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*****************************************************************************
                               Heap
*****************************************************************************}

{$i sysheap.inc}

{$i heap.inc}

{*****************************************************************************
                          Thread support
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_THREADING}
{ Generic threadmanager }
{$i thread.inc}

{$ifndef FPC_SECTION_THREADVARS}
{ Generic threadvar support }
{$i threadvr.inc}
{$endif FPC_SECTION_THREADVARS}

{$ifdef DISABLE_NO_THREAD_MANAGER}
{ OS Dependent implementation }
{$i systhrd.inc}
{$endif DISABLE_NO_THREAD_MANAGER}
{$endif FPC_HAS_FEATURE_THREADING}


{*****************************************************************************
                          Dynamic library support
*****************************************************************************}


{$ifdef FPC_HAS_FEATURE_DYNLIBS}
{$i dynlib.inc}

{$ifdef DISABLE_NO_DYNLIBS_MANAGER}
{ OS Dependant implementation }
{$i sysdl.inc}
{$endif DISABLE_NO_DYNLIBS_MANAGER}
{$endif FPC_HAS_FEATURE_DYNLIBS}


{*****************************************************************************
                            File Handling
*****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_FEATURE_FILEIO}
{ Allow slash and backslash as separators }
procedure DoDirSeparators(var p: pchar; inplace: boolean = true);
var
  i : longint;
  len : sizeint;
  newp : pchar;
begin
  len:=length(p);
  newp:=nil;
  for i:=0 to len do
    if p[i] in AllowDirectorySeparators then
      begin
        if not inplace and
           not assigned(newp) then
          begin
            getmem(newp,len+1);
            move(p^,newp^,len+1);
            p:=newp;
          end;
        p[i]:=DirectorySeparator;
      end;
end;

procedure DoDirSeparators(var p: pwidechar; inplace: boolean = true);
var
  i : longint;
  len : sizeint;
  newp : pwidechar;
begin
  len:=length(p);
  newp:=nil;
  for i:=0 to len do
    if (ord(p[i])<255) and
       (ansichar(ord(p[i])) in AllowDirectorySeparators) then
      begin
        if not inplace and
           not assigned(newp) then
          begin
            getmem(newp,(len+1)*2);
            move(p^,newp^,(len+1)*2);
            p:=newp;
          end;
        p[i]:=DirectorySeparator;
      end;
=======
=======
>>>>>>> origin/fixes_2_2
{ Allow slash and backslash as separators }
procedure DoDirSeparators(p:Pchar);
var
  i : longint;
begin
  for i:=0 to strlen(p) do
    if p[i] in AllowDirectorySeparators then
      p[i]:=DirectorySeparator;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure DoDirSeparators(var p:shortstring);
var
  i : longint;
begin
  for i:=1 to length(p) do
    if p[i] in AllowDirectorySeparators then
      p[i]:=DirectorySeparator;
end;

<<<<<<< HEAD
<<<<<<< HEAD

{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
procedure DoDirSeparators(var ps:RawByteString);
var
  i : longint;
  p : pchar;
  unique : boolean;
begin
  unique:=false;
  for i:=1 to length(ps) do
    if ps[i] in AllowDirectorySeparators then
      begin
        if not unique then
          begin
            uniquestring(ps);
            p:=pchar(ps);
            unique:=true;
          end;
        p[i-1]:=DirectorySeparator;
      end;
end;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


{$ifdef FPC_HAS_FEATURE_UNICODESTRINGS}
procedure DoDirSeparators(var ps:UnicodeString);
var
  i : longint;
  p : pwidechar;
  unique : boolean;
begin
  unique:=false;
  for i:=1 to length(ps) do
    if ps[i] in AllowDirectorySeparators then
      begin
        if not unique then
          begin
            uniquestring(ps);
            p:=pwidechar(ps);
            unique:=true;
          end;
        p[i-1]:=DirectorySeparator;
      end;
end;
{$endif FPC_HAS_FEATURE_UNICODESTRINGS}

{$endif FPC_HAS_FEATURE_FILEIO}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ OS dependent low level file functions }
{$ifdef FPC_HAS_FEATURE_FILEIO}
{$i sysfile.inc}

{$ifndef FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
{$ifdef FPC_ANSI_TEXTFILEREC}
procedure do_open(var f; p: pansichar; flags: longint; pchangeable: boolean);
var
  u: UnicodeString;
begin
  widestringmanager.Ansi2UnicodeMoveProc(p,DefaultFileSystemCodePage,u,length(p));
  do_open(f,pwidechar(u),flags,true);
end;

procedure do_erase(p: pansichar; pchangeable: boolean);
var
  u: UnicodeString;
begin
  widestringmanager.Ansi2UnicodeMoveProc(p,DefaultFileSystemCodePage,u,length(p));
  do_erase(pwidechar(u),true);
end;

procedure do_rename(src, dst: pansichar; srcchangeable, dstchangeable: boolean);
var
  usrc, udst: UnicodeString;
begin
  widestringmanager.Ansi2UnicodeMoveProc(src,DefaultFileSystemCodePage,usrc,length(src));
  widestringmanager.Ansi2UnicodeMoveProc(dst,DefaultFileSystemCodePage,udst,length(dst));
  do_rename(pwidechar(usrc),pwidechar(udst),true,true);
end;

procedure do_rename(src: pansichar; dst: pwidechar; srcchangeable, dstchangeable: boolean);
var
  usrc: UnicodeString;
begin
  widestringmanager.Ansi2UnicodeMoveProc(src,DefaultFileSystemCodePage,usrc,length(src));
  do_rename(pwidechar(usrc),dst,true,dstchangeable);
end;
{$endif FPC_ANSI_TEXTFILEREC}
{$endif not FPCRTL_FILESYSTEM_SINGLE_BYTE_API}


{$ifndef FPCRTL_FILESYSTEM_TWO_BYTE_API}
{$ifndef FPC_ANSI_TEXTFILEREC}
procedure do_open(var f; p: pwidechar; flags: longint; pchangeable: boolean);
var
  s: RawByteString;
begin
  widestringmanager.Unicode2AnsiMoveProc(p,s,DefaultFileSystemCodePage,length(p));
  do_open(f,pansichar(s),flags,true);
end;

procedure do_erase(p: pwidechar; pchangeable: boolean);
var
  s: RawByteString;
begin
  widestringmanager.Unicode2AnsiMoveProc(p,s,DefaultFileSystemCodePage,length(p));
  do_erase(pansichar(s),true);
end;

procedure do_rename(src, dst: pwidechar; srcchangeable, dstchangeable: boolean);
var
  rsrc, rdst: RawByteString;
begin
  widestringmanager.Unicode2AnsiMoveProc(src,rsrc,DefaultFileSystemCodePage,length(src));
  widestringmanager.Unicode2AnsiMoveProc(dst,rdst,DefaultFileSystemCodePage,length(dst));
  do_rename(pansichar(rsrc),pansichar(rdst),true,true);
end;

procedure do_rename(src: pwidechar; dst: pansichar; srcchangeable, dstchangeable: boolean);
var
  rsrc: RawByteString;
begin
  widestringmanager.Unicode2AnsiMoveProc(src,rsrc,DefaultFileSystemCodePage,length(src));
  do_rename(pansichar(rsrc),dst,true,dstchangeable);
end;
{$endif not FPC_ANSI_TEXTFILEREC}
{$endif not FPCRTL_FILESYSTEM_TWO_BYTE_API}

{$endif FPC_HAS_FEATURE_FILEIO}

{ helper for targets supporting no ansistrings, it is used
  by non-ansistring code }
function min(v1,v2 : SizeInt) : SizeInt;
  begin
    if v1<v2 then
      result:=v1
    else
      result:=v2;
  end;

{$ifdef FPC_HAS_FEATURE_TEXTIO}
{ Text file }
{$i text.inc}
{$endif FPC_HAS_FEATURE_TEXTIO}

{$ifdef FPC_HAS_FEATURE_FILEIO}
{ Untyped file }
{$i file.inc}

{ Typed file }
{$i typefile.inc}
{$endif FPC_HAS_FEATURE_FILEIO}


{*****************************************************************************
                            Directory Handling
*****************************************************************************}

{$ifdef FPC_HAS_FEATURE_FILEIO}
{ OS dependent dir functions }
{$i sysdir.inc}
{$endif FPC_HAS_FEATURE_FILEIO}

{$if defined(FPC_HAS_FEATURE_FILEIO) and defined(FPC_HAS_FEATURE_ANSISTRINGS)}
Procedure getdir(drivenr:byte;Var dir:ansistring);
{ this is needed to also allow ansistrings, the shortstring version is
  OS dependent }
var
  s : shortstring;
begin
  getdir(drivenr,s);
  dir:=s;
end;
{$endif}

{$if defined(FPC_HAS_FEATURE_FILEIO)}

Procedure MkDir(Const s: String);
Var
  Buffer: Array[0..255] of Char;
Begin
  If (s='') or (InOutRes <> 0) then
   exit;
  Move(s[1], Buffer, Length(s));
  Buffer[Length(s)] := #0;
  MkDir(@buffer[0],length(s));
End;

Procedure RmDir(Const s: String);
Var
  Buffer: Array[0..255] of Char;
Begin
  If (s='') or (InOutRes <> 0) then
   exit;
  Move(s[1], Buffer, Length(s));
  Buffer[Length(s)] := #0;
  RmDir(@buffer[0],length(s));
End;

Procedure ChDir(Const s: String);
Var
  Buffer: Array[0..255] of Char;
Begin
  If (s='') or (InOutRes <> 0) then
   exit;
  Move(s[1], Buffer, Length(s));
  Buffer[Length(s)] := #0;
  ChDir(@buffer[0],length(s));
End;
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}

{$ifndef FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
procedure do_getdir(drivenr : byte;var dir : rawbytestring);
var
  u: unicodestring;
begin
  Do_getdir(drivenr,u);
  widestringmanager.Unicode2AnsiMoveProc(pwidechar(u),dir,DefaultRTLFileSystemCodePage,length(u));
end;
{$endif FPCRTL_FILESYSTEM_SINGLE_BYTE_API}

Procedure MkDir(Const s: RawByteString);[IOCheck];
Begin
  If (s='') or (InOutRes <> 0) then
   exit;
{$ifdef FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  Do_mkdir(ToSingleByteFileSystemEncodedFileName(S));
{$else FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  Do_mkdir(S);
{$endif}
end;


Procedure RmDir(Const s: RawByteString);[IOCheck];
Begin
  If (s='') or (InOutRes <> 0) then
   exit;
{$ifdef FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  Do_rmdir(ToSingleByteFileSystemEncodedFileName(S));
{$else FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  Do_rmdir(S);
{$endif}
End;


Procedure ChDir(Const s: RawByteString);[IOCheck];
Begin
  If (s='') or (InOutRes <> 0) then
   exit;
{$ifdef FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  Do_chdir(ToSingleByteFileSystemEncodedFileName(S));
{$else FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  Do_chdir(S);
{$endif}
End;


Procedure getdir(drivenr:byte;Var dir:rawbytestring);
begin
  Do_getdir(drivenr,dir);
  { we should return results in the DefaultRTLFileSystemCodePage -> convert if
    necessary }
  setcodepage(dir,DefaultRTLFileSystemCodePage,true);
end;

{ the generic shortstring ones are only implemented elsewhere for systems *not*
  supporting ansi/unicodestrings; for now assume there are no systems that
  support unicodestrings but not ansistrings }

{ avoid double string conversions }
{$ifdef FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
function GetDirStrFromShortstring(const s: shortstring): RawByteString;
begin
  GetDirStrFromShortstring:=ToSingleByteFileSystemEncodedFileName(ansistring(s));
end;
{$else FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
function GetDirStrFromShortstring(const s: shortstring): UnicodeString;
begin
  GetDirStrFromShortstring:=s;
end;
{$endif FPCRTL_FILESYSTEM_SINGLE_BYTE_API}

Procedure MkDir(Const s: shortstring);[IOCheck];
Begin
  If (s='') or (InOutRes <> 0) then
   exit;
  Do_mkdir(GetDirStrFromShortstring(S));
End;


Procedure RmDir(Const s: shortstring);[IOCheck];
Begin
  If (s='') or (InOutRes <> 0) then
   exit;
  Do_rmdir(GetDirStrFromShortstring(S));
End;


Procedure ChDir(Const s: shortstring);[IOCheck];
Begin
  If (s='') or (InOutRes <> 0) then
   exit;
  Do_chdir(GetDirStrFromShortstring(S));
End;


Procedure getdir(drivenr:byte;Var dir:shortstring);
var
  s: rawbytestring;
begin
  Do_getdir(drivenr,s);
  if length(s)<=high(dir) then
    dir:=s
  else
    inoutres:=3;
end;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


{$if defined(FPC_HAS_FEATURE_WIDESTRINGS)}

{$ifndef FPCRTL_FILESYSTEM_TWO_BYTE_API}
{ overloads required for mkdir/rmdir/chdir to ensure that the string is
  converted to the right code page }
procedure do_mkdir(const s: unicodestring); {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  do_mkdir(ToSingleByteFileSystemEncodedFileName(s));
end;


procedure do_rmdir(const s: unicodestring); {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  do_rmdir(ToSingleByteFileSystemEncodedFileName(s));
end;


procedure do_chdir(const s: unicodestring); {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  do_chdir(ToSingleByteFileSystemEncodedFileName(s));
end;


procedure do_getdir(drivenr : byte;var dir : unicodestring);
var
  s: rawbytestring;
begin
  Do_getdir(drivenr,s);
  dir:=unicodestring(s);
end;
{$endif FPCRTL_FILESYSTEM_TWO_BYTE_API}

Procedure MkDir(Const s: UnicodeString);[IOCheck];
Begin
  if (s='') or (InOutRes <> 0) then
   exit;
  Do_mkdir(S);
End;


Procedure RmDir(Const s: UnicodeString);[IOCheck];
Begin
  if (s='') or (InOutRes <> 0) then
   exit;
  Do_rmdir(S);
End;


Procedure ChDir(Const s: UnicodeString);[IOCheck];
Begin
  if (s='') or (InOutRes <> 0) then
   exit;
  Do_chdir(S);
End;


Procedure getdir(drivenr:byte;Var dir:unicodestring);
begin
  Do_getdir(drivenr,dir);
end;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

{$endif FPC_HAS_FEATURE_FILEIO}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{*****************************************************************************
                            Resources support
*****************************************************************************}

{$i sysres.inc}

const
  CtrlBreakHandler: TCtrlBreakHandler = nil;

{$IFNDEF FPC_HAS_SETCTRLBREAKHANDLER}
(* It is possible to provide platform specific implementation performing   *)
(* special initialization; default implementation just sets the procedural *)
(* variable to make it available for use from the exception handler.       *)
function SysSetCtrlBreakHandler (Handler: TCtrlBreakHandler): TCtrlBreakHandler;
begin
  (* Return either nil or previous handler *)
  SysSetCtrlBreakHandler := CtrlBreakHandler;
  CtrlBreakHandler := Handler;
end;
<<<<<<< HEAD
=======

const
  CtrlBreakHandler: TCtrlBreakHandler = nil;

{$IFNDEF FPC_HAS_SETCTRLBREAKHANDLER}
(* It is possible to provide platform specific implementation performing   *)
(* special initialization; default implementation just sets the procedural *)
(* variable to make it available for use from the exception handler.       *)
function SysSetCtrlBreakHandler (Handler: TCtrlBreakHandler): TCtrlBreakHandler;
begin
  (* Return either nil or previous handler *)
  SysSetCtrlBreakHandler := CtrlBreakHandler;
  CtrlBreakHandler := Handler;
end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======

const
  CtrlBreakHandler: TCtrlBreakHandler = nil;

{$IFNDEF FPC_HAS_SETCTRLBREAKHANDLER}
(* It is possible to provide platform specific implementation performing   *)
(* special initialization; default implementation just sets the procedural *)
(* variable to make it available for use from the exception handler.       *)
function SysSetCtrlBreakHandler (Handler: TCtrlBreakHandler): TCtrlBreakHandler;
begin
  (* Return either nil or previous handler *)
  SysSetCtrlBreakHandler := CtrlBreakHandler;
  CtrlBreakHandler := Handler;
end;
>>>>>>> origin/fixes_2_2
{$ENDIF FPC_HAS_SETCTRLBREAKHANDLER}
