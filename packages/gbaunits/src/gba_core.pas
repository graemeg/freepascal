(*
  gba_core.pas  01/09/2006 19.17.35
  ------------------------------------------------------------------------------
  This lib is a raw porting of tonclib library for gba (you can find it at
  http://user.chem.tue.nl/jakvijn/index.htm).
  
  As this is a direct port from c, I'm pretty sure that something could not work
  as you expect. I am even more sure that this code could be written better, so 
  if you think that I have made some mistakes or you have some better 
  implemented functions, let me know [francky74 (at) gmail (dot) com]
  Enjoy!

  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
  
  Copyright (C) 2006  Francesco Lombardi
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
  ------------------------------------------------------------------------------
*)
unit gba_core;
{$i def.inc}

interface

uses 
  gba_types;


// random numbers  -- courtesy of Cearn (TONClib)
const
  QRAN_SHIFT  = 15;
  QRAN_MASK   = ((1 shl QRAN_SHIFT) - 1);
  QRAN_MAX    = QRAN_MASK;
  QRAN_A      = 1664525;
  QRAN_C      = 1013904223;

var
  gbaRandSeed: dword = 42;

function gbaRandomize(seed: dword): dword;
function gbaRand(): dword;
function gbaRand(value: integer): dword;

procedure memset16(dest: pointer; hw: word; hwcount: dword); cdecl; external;
procedure memcpy16(dest: pointer; const src: pointer; hwcount: dword); cdecl; external;

procedure memset32(dest: pointer; wd: dword; wcount: dword); cdecl; external;
procedure memcpy32(dest: pointer; const src: pointer; wcount: dword); cdecl; external;

procedure DebugPrint(s: string); assembler; inline;

implementation

function gbaRandomize(seed: dword): dword;
var
  old: dword;
begin	
	old := gbaRandSeed;
	gbaRandSeed := seed; 
	gbaRandomize := old;	
end;

function gbaRand(): dword;
begin	
	gbaRandSeed := QRAN_A * gbaRandSeed + QRAN_C;
	gbaRand := (gbaRandSeed shr 16) and QRAN_MAX;
end;

function gbaRand(value: integer): dword;
var
  a: dword;
begin	
	gbaRandSeed := QRAN_A * gbaRandSeed + QRAN_C;
	a := (gbaRandSeed shr 16) and QRAN_MAX;
  gbaRand := (a * value) shr 15;
end;

// memory handling routines
// these are in ASM and optimized; use when possible
{$l core_asm.o}


{$OPTIMIZATION OFF}
procedure DebugPrint(s: string); assembler; inline;
asm
  mov r0,s
  swi #0xff0000
end['r0'];
{$OPTIMIZATION ON}                  


end.
