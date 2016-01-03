<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_INTERFACE}
type
  time_t = cint;
  ptime_t = ^time_t;

  Ptm = ^tm;
  tm = record
    tm_sec: cint;
    tm_min: cint;
    tm_hour: cint;
    tm_mday: cint;
    tm_mon: cint;
    tm_year: cint;
    tm_wday: cint;
    tm_yday: cint;
    tm_isdst: cint;
  end;


(* Some libc functions *)
//function printf(format: Pchar; args: array of const): longint; cdecl; external;
function printf(format: Pchar): longint; cdecl; varargs; external;
//function sprintf(s: Pchar; format: Pchar; args: array of const): longint; cdecl; external;
function sprintf(s: Pchar; format: Pchar): longint; varargs; cdecl; external;
//function iprintf(format: Pchar; args: array of const): longint; cdecl; external;
function iprintf(format: Pchar): longint; varargs; cdecl; external;
//function scanf(format: Pchar; args: array of const): longint; cdecl; external;
function scanf(format: Pchar): longint; cdecl; varargs; external;
//function sscanf(s: Pchar; format: Pchar; args: array of const): longint; cdecl; external;
function sscanf(s: Pchar; format: Pchar): longint; cdecl; varargs; external;
function strcmp(s1: Pchar; s2: Pchar): longint; cdecl; external;

function malloc(size: integer): pointer; cdecl; external;
function realloc(ptr: pointer; size: integer): pointer; cdecl; external;
procedure free(ptr: pointer); cdecl; external;
function memcpy(dest: pointer; src: pointer; n: integer): pointer; cdecl; external;

function gmtime(timer: ptime_t): ptm; cdecl; external;
function time(timer: ptime_t): time_t; cdecl; external;

type
  TSort = function (const a, b: pointer): integer;
procedure qsort(__base: pointer; __nmemb: integer; __size: integer; __compar: TSort); cdecl; external;

function __errno: pcint; cdecl; export;
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function __errno: pcint;  cdecl; export;
begin

end;
{$endif NDS_IMPLEMENTATION}

=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: helper.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
    Copyright (C) 2007  Francesco Lombardi
  	
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
   
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  $Log$

*)

(* +------------------------------------------------------------------------+ *)
(*    Begin Random function                                                   *)
(* +------------------------------------------------------------------------+ *)
{$ifdef NDS_INTERFACE}
const
  QRAN_SHIFT  = 15;
  QRAN_MASK   = ((1 shl QRAN_SHIFT) - 1);
  QRAN_MAX    = QRAN_MASK;
  QRAN_A      = 1664525;
  QRAN_C      = 1013904223;

var
  RandSeed: dword = 42;
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function Randomize(seed: cint): cint;  
var
  old: cint;
begin	
	old := RandSeed;
	RandSeed := seed; 
	Randomize := old;	
end;

function Rand(): cint; 
begin	
	RandSeed := QRAN_A * RandSeed + QRAN_C;
	Rand := (RandSeed shr 16) and QRAN_MAX;
end;

function Rand(value: cint): cint; 
var
  a: cint;
begin	
	RandSeed := QRAN_A * RandSeed + QRAN_C;
	a := (RandSeed shr 16) and QRAN_MAX;
  Rand := (a * value) shr 15;
end;
(* +------------------------------------------------------------------------+ *)
(*    End Random function                                                     *)
(* +------------------------------------------------------------------------+ *)
{$endif NDS_IMPLEMENTATION}


{$ifdef NDS_INTERFACE}
(* Some libc functions *)
function printf(format: Pchar; args: array of const): longint; cdecl; external;
function printf(format: Pchar): longint; cdecl; varargs; external;
function sprintf(s: Pchar; format: Pchar; args: array of const): longint; cdecl; external;
function sprintf(s: Pchar; format: Pchar): longint; varargs; cdecl; external;
function iprintf(format: Pchar; args: array of const): longint; cdecl; external;
function iprintf(format: Pchar): longint; varargs; cdecl; external;
function scanf(format: Pchar; args: array of const): longint; cdecl; external;
function scanf(format: Pchar): longint; cdecl; varargs; external;
function sscanf(s: Pchar; format: Pchar; args: array of const): longint; cdecl; external;
function sscanf(s: Pchar; format: Pchar): longint; cdecl; varargs; external;


(* libc file handling types and routines *)
type
  _FILE = record
    firstCluster: cuint;
    length: cuint;
    curPos: cuint;
    curClus: cuint;			// Current cluster to read from
    curSect: integer;			// Current sector within cluster
    curByte: integer;			// Current byte within sector
    readBuffer: array [0..511] of byte;	// Buffer used for unaligned reads
    appClus: cuint;			// Cluster to append to
    appSect: integer;			// Sector within cluster for appending
    appByte: integer;			// Byte within sector for appending
    read: boolean;	// Can read from file
    write: boolean;	// Can write to file
    append: boolean;// Can append to file
    inUse: boolean;	// This file is open
    dirEntSector: cuint;	// The sector where the directory entry is stored
    dirEntOffset: integer;	// The offset within the directory sector
  end;
  P_FILE = ^_FILE;

const
   SEEK_SET = 0;
   SEEK_CUR = 1;
   SEEK_END = 2;

function fopen(filename: Pchar; modes: Pchar): P_FILE; cdecl; external;
function fread(ptr: pointer; size: longint; n: longint; stream: P_FILE): longint; cdecl; external;
function fwrite(ptr: pointer; size: longint; n: longint; s: P_FILE): longint; cdecl; external;
function ftell(stream: P_FILE): longint; cdecl; external;
function fseek(stream: P_FILE; off: longint; whence: longint): longint; cdecl; external;
function fclose(stream: P_FILE): longint; cdecl; external;
{$endif NDS_INTERFACE}




{$ifdef NDS_INTERFACE}
function Randomize(seed: cint): cint;  
function Rand(): cint; 
function Rand(value: cint): cint; 
{$endif NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
