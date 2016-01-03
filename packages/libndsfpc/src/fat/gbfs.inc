(*
  $Id: gbfs.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
    Copyright 
      2002-2004 Damian Yerrick
      Additional code Dave Murphy
  	
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
(* to make a 300 KB space called samples do GBFS_SPACE(samples, 300) *)

{$define GBFS_SPACE(filename, kbytes) := filename: array [(kbytes)*1024 - 1] of char = 'PinEightGBFSSpace-' + filename + '-' + kbytes}

type
<<<<<<< HEAD
<<<<<<< HEAD
  GBFS_FILE = record
=======
  GBFS_FILE = packed record
>>>>>>> graemeg/fixes_2_2
=======
  GBFS_FILE = packed record
>>>>>>> origin/fixes_2_2
    magic: array [0..15] of char;   // "PinEightGBFS\r\n\032\n" 
    total_len: cuint32;              // total length of archive
    dir_off: cuint16;               // offset in bytes to directory 
    dir_nmemb: cuint16;             // number of files 
    reserved: array [0..7] of char; // for future use 
  end;
  PGBFS_FILE = ^GBFS_FILE;

<<<<<<< HEAD
<<<<<<< HEAD
  GBFS_ENTRY = record
=======
  GBFS_ENTRY = packed record
>>>>>>> graemeg/fixes_2_2
=======
  GBFS_ENTRY = packed record
>>>>>>> origin/fixes_2_2
    name: array [0..23] of char;      // filename, nul-padded
    len: cuint32;                     // length of object in bytes 
    data_offset: cuint32;             // in bytes from beginning of file
  end;
  PGBFS_ENTRY = ^GBFS_ENTRY;

procedure gbfs_search_range(gbfs_1st_limit: cuint32; gbfs_2nd_start: cuint32; 
                            gbfs_2nd_limit: cuint32; gbfs_stride: cuint32); cdecl; external;

function find_first_gbfs_file(const start: pointer): PGBFS_FILE; cdecl; external;
function skip_gbfs_file(const _file: PGBFS_FILE): pointer; cdecl; external;
function gbfs_get_obj(const _file: PGBFS_FILE; const name: pchar; len: pcuint32): pointer; cdecl; external;
function gbfs_get_nth_obj(const _file: PGBFS_FILE; n: cint {was size_t}; name: pchar; len: pcuint32): pointer; cdecl; external;
function gbfs_copy_obj(dst: pointer; const _file: PGBFS_FILE; const name: pchar): pointer; cdecl; external;
function gbfs_count_objs(const _file: PGBFS_FILE): cint {was size_t}; cdecl; external;

{$endif NDS_INTERFACE}

