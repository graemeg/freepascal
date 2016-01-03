<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_INTERFACE}
//  Math coprocessor register definitions
const
  REG_DIVCNT          : pcuint16 = pointer($04000280);
  REG_DIV_NUMER       : pcint64 = pointer($04000290);
  REG_DIV_NUMER_L     : pcint32 = pointer($04000290);
  REG_DIV_NUMER_H     : pcint32 = pointer($04000294);
  REG_DIV_DENOM       : pcint64 = pointer($04000298);
  REG_DIV_DENOM_L     : pcint32 = pointer($04000298);
  REG_DIV_DENOM_H     : pcint32 = pointer($0400029C);
  REG_DIV_RESULT      : pcint64 = pointer($040002A0);
  REG_DIV_RESULT_L    : pcint32 = pointer($040002A0);
  REG_DIV_RESULT_H    : pcint32 = pointer($040002A4);
  REG_DIVREM_RESULT   : pcint64 = pointer($040002A8);
  REG_DIVREM_RESULT_L : pcint32 = pointer($040002A8);
  REG_DIVREM_RESULT_H : pcint32 = pointer($040002AC);
  REG_SQRTCNT         : pcuint16 = pointer($040002B0);
  REG_SQRT_PARAM      : pcint64 = pointer($040002B8);
  REG_SQRT_PARAM_L    : pcint32 = pointer($040002B8);
  REG_SQRT_PARAM_H    : pcint32 = pointer($040002BC);
  REG_SQRT_RESULT     : pcuint32 = pointer($040002B4);
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: math.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  	Copyright (C) 2005
  		Jason Rogers (dovoto)
  		Dave Murphy (WinterMute)
  	
  	This software is provided 'as-is', without any express or implied
  	warranty.  In no event will the authors be held liable for any
  	damages arising from the use of this software.
  	
  	Permission is granted to anyone to use this software for any
  	purpose, including commercial applications, and to alter it and
  	redistribute it freely, subject to the following restrictions:
  	
  	1.	The origin of this software must not be misrepresented; you
  		must not claim that you wrote the original software. If you use
  		this software in a product, an acknowledgment in the product
  		documentation would be appreciated but is not required.
  	
  	2.	Altered source versions must be plainly marked as such, and
  		must not be misrepresented as being the original software.
  	
  	3.	This notice may not be removed or altered from any source
  		distribution.
  ------------------------------------------------------------------------------
    
    
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  $Log$

*)

{$ifdef NDS_INTERFACE}
//  Math coprocessor register definitions
const
  DIV_CR				    : pcuint16 = pointer($04000280);
  DIV_NUMERATOR64		: pcint64 = pointer($04000290);
  DIV_NUMERATOR32		: pcint32 = pointer($04000290);
  DIV_DENOMINATOR64	: pcint64 = pointer($04000298);
  DIV_DENOMINATOR32	: pcint32 = pointer($04000298);
  DIV_RESULT64		  : pcint64 = pointer($040002A0);
  DIV_RESULT32		  : pcint32 = pointer($040002A0);
  DIV_REMAINDER64		: pcint64 = pointer($040002A8);
  DIV_REMAINDER32		: pcint32 = pointer($040002A8);

  SQRT_CR           : pcuint16 = pointer($040002B0);
  SQRT_PARAM64      : pcint64 = pointer($040002B8);
  SQRT_RESULT32     : pcint32 = pointer($040002B4);
  SQRT_PARAM32      : pcint32 = pointer($040002B8);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

//  Math coprocessor modes

  DIV_64_64		= 2;
  DIV_64_32		= 1;
  DIV_32_32		= 0;
  DIV_BUSY		= (1 shl 15);

  SQRT_64			= 1;
  SQRT_32			= 0;
  SQRT_BUSY		= (1 shl 15);
<<<<<<< HEAD
<<<<<<< HEAD

function inttof32(n: cint): cint32; inline;
function f32toint(n: cint32): cint; inline;
function floattof32(n: cfloat): cint32; inline;  //inlining it makes impossible to pass it to another function :/
function f32tofloat(n: cint32): cfloat; inline;
  
function divf32(num: cint32; den: cint32): cint32; inline;
function mulf32(a, b: cint32): cint32; inline;
function sqrtf32(a: cint32): cint32; inline;
function div32(num, den: cint32): cint32; inline;
function mod32(num, den: cint32): cint32; inline;
function div64(num: cint64; den: cint32): cint32; inline;
function mod64(num: cint64; den: cint32): cint32; inline;
function sqrt32(a: cint32): cuint32; inline;
function sqrt64(a: cint64): cuint32; inline;
procedure crossf32(a: pcint32; b: pcint32; res: pcint32); inline;
function dotf32(a, b: pcint32): cint32; inline;
procedure normalizef32(a: pcint32); inline;  
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function inttof32(n: cint): cint32; inline;
begin
  inttof32 := ((n) shl 12);
end;

function f32toint(n: cint32): cint; inline;
begin
  f32toint := ((n) shr 12);
end;

function floattof32(n: cfloat): cint32; inline;
begin
  floattof32 := trunc((n) * (1 shl 12));
end;

// check it!
function f32tofloat(n: cint32): cfloat; inline;
begin
  f32tofloat := cfloat(n * 1.0) / cfloat((1 shl 12) * 1.0);
end;

function divf32(num: cint32; den: cint32): cint32; inline;
begin
  REG_DIVCNT^ := DIV_64_32;

  while (REG_DIVCNT^ and DIV_BUSY) <> 0 do;

	REG_DIV_NUMER^ := cint64(num) shl 12;
	REG_DIV_DENOM_L^ := den;

	while (REG_DIVCNT^ and DIV_BUSY) <> 0 do;

	divf32 := REG_DIV_RESULT_L^;
=======
=======
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function divf32(num: cint32; den: cint32): cint32; inline;
begin
	DIV_CR^ := DIV_64_32;

	while (DIV_CR^ and DIV_BUSY) <> 0 do;

	DIV_NUMERATOR64^ := cint64(num) shl 12;
	DIV_DENOMINATOR32^ := den;

	while (DIV_CR^ and DIV_BUSY) <> 0 do;

	divf32 := DIV_RESULT32^;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function mulf32(a, b: cint32): cint32; inline;
var
  rslt: clonglong;
begin
	rslt := clonglong(a) * clonglong(b);
	mulf32 := cint32(rslt shr 12);
end;

//  Fixed point square root
//	Takes 1.19.12 fixed point value and
//	returns the fixed point result
function sqrtf32(a: cint32): cint32; inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
	REG_SQRTCNT^ := SQRT_64;

	while (REG_SQRTCNT^ and SQRT_BUSY) <> 0 do;

	REG_SQRT_PARAM^ := cint64(a) shl 12;

	while (REG_SQRTCNT^ and SQRT_BUSY) <> 0 do;

	sqrtf32 := REG_SQRT_RESULT^;
=======
=======
>>>>>>> origin/fixes_2_2
	SQRT_CR^ := SQRT_64;

	while (SQRT_CR^ and SQRT_BUSY) <> 0 do;

	SQRT_PARAM64^ := cint64(a) shl 12;

	while (SQRT_CR^ and SQRT_BUSY) <> 0 do;

	sqrtf32 := SQRT_RESULT32^;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

//  Integer versions

//  Integer divide
//  Takes a 32 bit numerator and 32 bit
//	denominator and returns 32 bit result
function div32(num, den: cint32): cint32; inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
	REG_DIVCNT^ := DIV_32_32;

	while (REG_DIVCNT^ and DIV_BUSY) <> 0 do;

	REG_DIV_NUMER_L^ := num;
	REG_DIV_DENOM_L^ := den;

	while (REG_DIVCNT^ and DIV_BUSY) <> 0 do;

	div32 := REG_DIV_RESULT_L^;
=======
=======
>>>>>>> origin/fixes_2_2
	DIV_CR^ := DIV_32_32;

	while (DIV_CR^ and DIV_BUSY) <> 0 do;

	DIV_NUMERATOR32^ := num;
	DIV_DENOMINATOR32^ := den;

	while (DIV_CR^ and DIV_BUSY) <> 0 do;

	div32 := DIV_RESULT32^;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

//  Integer divide
//  Takes a 32 bit numerator and 32 bit
//	denominator and returns 32 bit result
function mod32(num, den: cint32): cint32; inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
	REG_DIVCNT^ := DIV_32_32;

	while (REG_DIVCNT^ and DIV_BUSY) <> 0 do;

	REG_DIV_NUMER_L^ := num;
	REG_DIV_DENOM_L^ := den;

	while (REG_DIVCNT^ and DIV_BUSY) <> 0 do;

	mod32 := REG_DIVREM_RESULT_L^;
=======
=======
>>>>>>> origin/fixes_2_2
	DIV_CR^ := DIV_32_32;

	while (DIV_CR^ and DIV_BUSY) <> 0 do;

	DIV_NUMERATOR32^ := num;
	DIV_DENOMINATOR32^ := den;

	while (DIV_CR^ and DIV_BUSY) <> 0 do;

	mod32 := DIV_REMAINDER32^;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

//  Integer divide
//	Takes a 64 bit numerator and 32 bit
//  denominator are returns 32 bit result
function div64(num: cint64; den: cint32): cint32; inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
	REG_DIVCNT^ := DIV_64_32;

	while (REG_DIVCNT^ and DIV_BUSY) <> 0 do;

	REG_DIV_NUMER^ := num;
	REG_DIV_DENOM_L^ := den;

	while (REG_DIVCNT^ and DIV_BUSY) <> 0 do;

	div64 := REG_DIV_RESULT_L^;
=======
=======
>>>>>>> origin/fixes_2_2
	DIV_CR^ := DIV_64_32;

	while (DIV_CR^ and DIV_BUSY) <> 0 do;

	DIV_NUMERATOR64^ := num;
	DIV_DENOMINATOR32^ := den;

	while (DIV_CR^ and DIV_BUSY) <> 0 do;

	div64 := DIV_RESULT32^;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

//  Integer divide
//	Takes a 64 bit numerator and 32 bit
//  denominator are returns 32 bit result
function mod64(num: cint64; den: cint32): cint32; inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
	REG_DIVCNT^ := DIV_64_32;

	while (REG_DIVCNT^ and DIV_BUSY) <> 0 do;

	REG_DIV_NUMER^ := num;
	REG_DIV_DENOM_L^ := den;

	while (REG_DIVCNT^ and DIV_BUSY) <> 0 do;

	mod64 := REG_DIVREM_RESULT_L^;
=======
=======
>>>>>>> origin/fixes_2_2
	DIV_CR^ := DIV_64_32;

	while (DIV_CR^ and DIV_BUSY) <> 0 do;

	DIV_NUMERATOR64^ := num;
	DIV_DENOMINATOR32^ := den;

	while (DIV_CR^ and DIV_BUSY) <> 0 do;

	mod64 := DIV_REMAINDER32^;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

//  Integer square root
//  takes a 32 bit integer and returns
//	32 bit result
<<<<<<< HEAD
<<<<<<< HEAD
function sqrt32(a: cint32): cuint32; inline;
begin
	REG_SQRTCNT^ := SQRT_32;

	while(REG_SQRTCNT^ and SQRT_BUSY) <> 0 do;

	REG_SQRT_PARAM_L^ := a;

	while(REG_SQRTCNT^ and SQRT_BUSY) <> 0 do;

	sqrt32 := REG_SQRT_RESULT^;
end;

function sqrt64(a: cint64): cuint32; inline;
begin
	REG_SQRTCNT^ := SQRT_64;

	while(REG_SQRTCNT^ and SQRT_BUSY) <> 0 do;

	REG_SQRT_PARAM^ := a;

	while(REG_SQRTCNT^ and SQRT_BUSY) <> 0 do;

	sqrt64 := REG_SQRT_RESULT^;
end;

=======
=======
>>>>>>> origin/fixes_2_2
function sqrt32(a: cint32): cint32; inline;
begin
	SQRT_CR^ := SQRT_32;

	while(SQRT_CR^ and SQRT_BUSY) <> 0 do;

	SQRT_PARAM32^ := a;

	while(SQRT_CR^ and SQRT_BUSY) <> 0 do;

	sqrt32 := SQRT_RESULT32^;
end;

//  Trig Functions  1.19.12 fixed point

// Cross product
// x = Ay * Bz - By * Az
// y = Az * Bx - Bz * Ax
// z = Ax * By - Bx * Ay
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure crossf32(a: pcint32; b: pcint32; res: pcint32); inline;
begin
	res[0] := mulf32(a[1], b[2]) - mulf32(b[1], a[2]);
	res[1] := mulf32(a[2], b[0]) - mulf32(b[2], a[0]);
	res[2] := mulf32(a[0], b[1]) - mulf32(b[0], a[1]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
// Dot Product
// result = Ax * Bx + Ay * By + Az * Bz
>>>>>>> graemeg/fixes_2_2
=======
// Dot Product
// result = Ax * Bx + Ay * By + Az * Bz
>>>>>>> origin/fixes_2_2
function dotf32(a, b: pcint32): cint32; inline;
begin
	dotf32 := mulf32(a[0], b[0]) + mulf32(a[1], b[1]) + mulf32(a[2], b[2]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
// Normalize
// Ax = Ax / mag
// Ay = Ay / mag
// Az = Az / mag
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure normalizef32(a: pcint32); inline;
var
  magnitude: cint32;
begin
<<<<<<< HEAD
<<<<<<< HEAD
	magnitude := sqrtf32( mulf32(a[0], a[0]) + mulf32(a[1], a[1]) + mulf32(a[2], a[2]) );
=======
	// magnitude = sqrt ( Ax^2 + Ay^2 + Az^2 )
	magnitude := sqrtf32( mulf32(a[0], a[0]) + mulf32(a[1], a[1]) + mulf32(a[2], a[2]) );

>>>>>>> graemeg/fixes_2_2
=======
	// magnitude = sqrt ( Ax^2 + Ay^2 + Az^2 )
	magnitude := sqrtf32( mulf32(a[0], a[0]) + mulf32(a[1], a[1]) + mulf32(a[2], a[2]) );

>>>>>>> origin/fixes_2_2
	a[0] := divf32(a[0], magnitude);
	a[1] := divf32(a[1], magnitude);
	a[2] := divf32(a[2], magnitude);
end;
{$endif NDS_IMPLEMENTATION}
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

{$ifdef NDS_INTERFACE}
function divf32(num: cint32; den: cint32): cint32; inline;
function mulf32(a, b: cint32): cint32; inline;
function sqrtf32(a: cint32): cint32; inline;
function div32(num, den: cint32): cint32; inline;
function mod32(num, den: cint32): cint32; inline;
function div64(num: cint64; den: cint32): cint32; inline;
function mod64(num: cint64; den: cint32): cint32; inline;
function sqrt32(a: cint32): cint32; inline;
procedure crossf32(a: pcint32; b: pcint32; res: pcint32); inline;
function dotf32(a, b: pcint32): cint32; inline;
procedure normalizef32(a: pcint32); inline;
{$endif NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
