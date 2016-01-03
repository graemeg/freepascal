{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 2004 by Olle Raab

    This unit contain procedures specific for mode MacPas.
    It should be platform independant.

    See the file COPYING.FPC, included in this distribution,
    For details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}

unit MacPas;

interface

{ Using inlining for small system functions/wrappers }
{$inline on}
{$define SYSTEMINLINE}

type
{$ifndef FPUNONE}
  LongDouble = ValReal;
{$endif}
  FourCharArray = packed array[1..4] of char;

  UnsignedByte = Byte;
  UnsignedWord = Word;
  UnsignedLong = Longword;

{FourCharCode coercion
This routine coreces string literals to a FourCharCode.}
function FCC(const literal: string): LongWord; {$ifdef systeminline}inline;{$endif}

{Same as FCC, to be compatible with GPC}
function FOUR_CHAR_CODE(const literal: string): LongWord; {$ifdef systeminline}inline;{$endif}

{ Same as the "is" operator }
Function Member (Instance : TObject; AClass : TClass) : boolean; {$ifdef systeminline}inline;{$endif}

function ord4(i: smallint): smallint; {$ifdef systeminline}inline;{$endif}
function ord4(l: longint): longint; {$ifdef systeminline}inline;{$endif}
function ord4(c: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
function ord4(p: pointer): ptrint; {$ifdef systeminline}inline;{$endif}

function BAnd(i,j: longint): longint; {$ifdef systeminline}inline;{$endif}
function BAnd(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
function BAnd(i,j: int64): int64; {$ifdef systeminline}inline;{$endif}
function BAnd(i,j: qword): qword; {$ifdef systeminline}inline;{$endif}

function BOr(i,j: longint): longint; {$ifdef systeminline}inline;{$endif}
function BOr(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
function BOr(i,j: int64): int64; {$ifdef systeminline}inline;{$endif}
function BOr(i,j: qword): qword; {$ifdef systeminline}inline;{$endif}

function BXor(i,j: longint): longint; {$ifdef systeminline}inline;{$endif}
function BXor(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
function BXor(i,j: int64): int64; {$ifdef systeminline}inline;{$endif}
function BXor(i,j: qword): qword; {$ifdef systeminline}inline;{$endif}

function Bsr(i: longint; j: cardinal): longint; {$ifdef systeminline}inline;{$endif}
function Bsr(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
function Bsr(i: int64; j: cardinal): int64; {$ifdef systeminline}inline;{$endif}
function Bsr(i: qword; j: cardinal): qword; {$ifdef systeminline}inline;{$endif}

function Bsl(i: longint; j: cardinal): longint; {$ifdef systeminline}inline;{$endif}
function Bsl(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
function Bsl(i: int64; j: cardinal): int64; {$ifdef systeminline}inline;{$endif}
function Bsl(i: qword; j: cardinal): qword; {$ifdef systeminline}inline;{$endif}

function BTst(i: longint; j: cardinal): boolean; {$ifdef systeminline}inline;{$endif}
function BTst(i,j: cardinal): boolean; {$ifdef systeminline}inline;{$endif}
function BTst(i: int64; j: cardinal): boolean; {$ifdef systeminline}inline;{$endif}
function BTst(i: qword; j: cardinal): boolean; {$ifdef systeminline}inline;{$endif}

procedure BSet(var i: longint; j: cardinal); {$ifdef systeminline}inline;{$endif}
procedure BSet(var i: cardinal; j: cardinal); {$ifdef systeminline}inline;{$endif}
procedure BSet(var i: int64; j: cardinal); {$ifdef systeminline}inline;{$endif}
procedure BSet(var i: qword; j: cardinal); {$ifdef systeminline}inline;{$endif}

procedure BClr(var i: longint; j: cardinal); {$ifdef systeminline}inline;{$endif}
procedure BClr(var i: cardinal; j: cardinal); {$ifdef systeminline}inline;{$endif}
procedure BClr(var i: int64; j: cardinal); {$ifdef systeminline}inline;{$endif}
procedure BClr(var i: qword; j: cardinal); {$ifdef systeminline}inline;{$endif}

function BRotL(i: longint; j: cardinal): longint; {$ifdef systeminline}inline;{$endif}
function BRotL(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
function BRotL(i: int64; j: cardinal): int64; {$ifdef systeminline}inline;{$endif}
function BRotL(i: qword; j: cardinal): qword; {$ifdef systeminline}inline;{$endif}

function BRotR(i: longint; j: cardinal): longint; {$ifdef systeminline}inline;{$endif}
function BRotR(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
function BRotR(i: int64; j: cardinal): int64; {$ifdef systeminline}inline;{$endif}
function BRotR(i: qword; j: cardinal): qword; {$ifdef systeminline}inline;{$endif}

function BNot(i: longint): longint; {$ifdef systeminline}inline;{$endif}
function BNot(i: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
function BNot(i: int64): int64; {$ifdef systeminline}inline;{$endif}
function BNot(i: qword): qword; {$ifdef systeminline}inline;{$endif}


implementation

uses
  math;

{$r-}
{$q-}


function FCC(const literal: string): LongWord; {$ifdef systeminline}inline;{$endif}
begin
{$ifdef FPC_LITTLE_ENDIAN}
  FCC := (ord(literal[1]) shl 24) or (ord(literal[2]) shl 16) or (ord(literal[3]) shl 8) or ord(literal[4]);
{$else FPC_LITTLE_ENDIAN}
  FCC := PLongWord(@literal[1])^;
{$endif FPC_LITTLE_ENDIAN}
end;

function FOUR_CHAR_CODE(const literal: string): LongWord; {$ifdef systeminline}inline;{$endif}
begin
{$ifdef FPC_LITTLE_ENDIAN}
  FOUR_CHAR_CODE := (ord(literal[1]) shl 24) or (ord(literal[2]) shl 16) or (ord(literal[3]) shl 8) or ord(literal[4]);
{$else FPC_LITTLE_ENDIAN}
  FOUR_CHAR_CODE := PLongWord(@literal[1])^;
{$endif FPC_LITTLE_ENDIAN}
end;

Function Member (Instance : TObject; AClass : TClass) : boolean; {$ifdef systeminline}inline;{$endif}
begin
  Result:=Instance is AClass;
end;


function ord4(i: smallint): smallint; {$ifdef systeminline}inline;{$endif}
begin
  result:=i;
end;


function ord4(l: longint): longint; {$ifdef systeminline}inline;{$endif}
begin
  result := l;
end;


function ord4(c: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
begin
  result := c;
end;


function ord4(p: pointer): ptrint; {$ifdef systeminline}inline;{$endif}
begin
  result := ptrint(p);
end;



function BAnd(i,j: longint): longint; {$ifdef systeminline}inline;{$endif}
begin
  result := i and j;
end;

function BAnd(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
begin
  result := i and j;
end;

function BAnd(i,j: int64): int64; {$ifdef systeminline}inline;{$endif}
begin
  result := i and j;
end;

function BAnd(i,j: qword): qword; {$ifdef systeminline}inline;{$endif}
begin
  result := i and j;
end;


function BOr(i,j: longint): longint; {$ifdef systeminline}inline;{$endif}
begin
  result := i or j;
end;

function BOr(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
begin
  result := i or j;
end;

function BOr(i,j: int64): int64; {$ifdef systeminline}inline;{$endif}
begin
  result := i or j;
end;

function BOr(i,j: qword): qword; {$ifdef systeminline}inline;{$endif}
begin
  result := i or j;
end;


function BXor(i,j: longint): longint; {$ifdef systeminline}inline;{$endif}
begin
  result := i xor j;
end;

function BXor(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
begin
  result := i xor j;
end;

function BXor(i,j: int64): int64; {$ifdef systeminline}inline;{$endif}
begin
  result := i xor j;
end;

function BXor(i,j: qword): qword; {$ifdef systeminline}inline;{$endif}
begin
  result := i xor j;
end;


function Bsr(i: longint; j: cardinal): longint; {$ifdef systeminline}inline;{$endif}
begin
  result := i shr j;
end;

function Bsr(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
begin
  result := i shr j;
end;

function Bsr(i: int64; j: cardinal): int64; {$ifdef systeminline}inline;{$endif}
begin
  result := i shr j;
end;

function Bsr(i: qword; j: cardinal): qword; {$ifdef systeminline}inline;{$endif}
begin
  result := i shr j;
end;


function Bsl(i: longint; j: cardinal): longint; {$ifdef systeminline}inline;{$endif}
begin
  result := i shl j;
end;

function Bsl(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
begin
  result := i shl j;
end;

function Bsl(i: int64; j: cardinal): int64; {$ifdef systeminline}inline;{$endif}
begin
  result := i shl j;
end;

function Bsl(i: qword; j: cardinal): qword; {$ifdef systeminline}inline;{$endif}
begin
  result := i shl j;
end;


function BTst(i: longint; j: cardinal): boolean; {$ifdef systeminline}inline;{$endif}
begin
  result := ((i shr j) and 1) <> 0;
end;

function BTst(i,j: cardinal): boolean; {$ifdef systeminline}inline;{$endif}
begin
  result := ((i shr j) and 1) <> 0;
end;

function BTst(i: int64; j: cardinal): boolean; {$ifdef systeminline}inline;{$endif}
begin
  result := (cardinal(i shr j) and 1) <> 0;
end;

function BTst(i: qword; j: cardinal): boolean; {$ifdef systeminline}inline;{$endif}
begin
  result := (cardinal(i shr j) and 1) <> 0;
end;


procedure BSet(var i: longint; j: cardinal); {$ifdef systeminline}inline;{$endif}
begin
  i := i or (1 shl j);
end;

procedure BSet(var i: cardinal; j: cardinal); {$ifdef systeminline}inline;{$endif}
begin
  i := i or (cardinal(1) shl j);
end;

procedure BSet(var i: int64; j: cardinal); {$ifdef systeminline}inline;{$endif}
begin
  i := i or (int64(1) shl j);
end;

procedure BSet(var i: qword; j: cardinal); {$ifdef systeminline}inline;{$endif}
begin
  i := i or (qword(1) shl j);
end;


procedure BClr(var i: longint; j: cardinal); {$ifdef systeminline}inline;{$endif}
begin
  i := i and not (1 shl j);
end;

procedure BClr(var i: cardinal; j: cardinal); {$ifdef systeminline}inline;{$endif}
begin
  i := i and not (1 shl j);
end;

procedure BClr(var i: int64; j: cardinal); {$ifdef systeminline}inline;{$endif}
begin
  i := i and not (int64(1) shl j);
end;

procedure BClr(var i: qword; j: cardinal); {$ifdef systeminline}inline;{$endif}
begin
  i := i and not (qword(1) shl j);
end;

function BRotL(i: longint; j: cardinal): longint; {$ifdef systeminline}inline;{$endif}
begin
  result := (i shl j) or (i shr (32-j));
end;

function BRotL(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
begin
  result := (i shl j) or (i shr (32-j));
end;

function BRotL(i: int64; j: cardinal): int64; {$ifdef systeminline}inline;{$endif}
begin
  result := (i shl j) or (i shr (64-j));
end;

function BRotL(i: qword; j: cardinal): qword; {$ifdef systeminline}inline;{$endif}
begin
  result := (i shl j) or (i shr (64-j));
end;

function BRotR(i: longint; j: cardinal): longint; {$ifdef systeminline}inline;{$endif}
begin
  result := (i shr j) or (i shl (32-j));
end;

function BRotR(i,j: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
begin
  result := (i shr j) or (i shl (32-j));
end;

function BRotR(i: int64; j: cardinal): int64; {$ifdef systeminline}inline;{$endif}
begin
  result := (i shr j) or (i shl (64-j));
end;

function BRotR(i: qword; j: cardinal): qword; {$ifdef systeminline}inline;{$endif}
begin
  result := (i shr j) or (i shl (64-j));
end;

function BNot(i: longint): longint; {$ifdef systeminline}inline;{$endif}
begin
  result := not(i);
end;

function BNot(i: cardinal): cardinal; {$ifdef systeminline}inline;{$endif}
begin
  result := not(i);
end;

function BNot(i: int64): int64; {$ifdef systeminline}inline;{$endif}
begin
  result := not(i);
end;

function BNot(i: qword): qword; {$ifdef systeminline}inline;{$endif}
begin
  result := not(i);
end;


<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
begin
  SetExceptionMask([exInvalidOp, exDenormalized, exZeroDivide, exOverflow, exUnderflow, exPrecision])
{$endif}
=======
begin
  SetExceptionMask([exInvalidOp, exDenormalized, exZeroDivide, exOverflow, exUnderflow, exPrecision])
>>>>>>> graemeg/fixes_2_2
=======
begin
  SetExceptionMask([exInvalidOp, exDenormalized, exZeroDivide, exOverflow, exUnderflow, exPrecision])
>>>>>>> origin/fixes_2_2
end.
