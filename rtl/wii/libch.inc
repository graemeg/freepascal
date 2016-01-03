{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2002 by the Free Pascal development team

    libc functions unit for Nintendo Wii
    Copyright (c) 2011 by Francesco Lombardi

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 *****************************************************************************}

type
  time_t = longint;
  ptime_t = ^time_t;

  Ptm = ^tm;
  tm = record
    tm_sec: longint;
    tm_min: longint;
    tm_hour: longint;
    tm_mday: longint;
    tm_mon: longint;
    tm_year: longint;
    tm_wday: longint;
    tm_yday: longint;
    tm_isdst: longint;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Ptimeval = ^Ttimeval;
  Ttimeval = record
    tv_sec  : longint;
    tv_usec : longint;
  end;
  Timeval = TTimeval;
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
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
function memalign(alignment: integer; size: integer): pointer; cdecl; external;
function memset(s: pointer; c: longint; n: integer): pointer; cdecl; external;


function gmtime(timer: ptime_t): ptm; cdecl; external;
function time(timer: ptime_t): time_t; cdecl; external;

type
  TSort = function (const a, b: pointer): integer;
procedure qsort(__base: pointer; __nmemb: integer; __size: integer; __compar: TSort); cdecl; external;

function __errno: plongint; cdecl; export;

type
  _FILE = record
    firstCluster: longword;
    length: longword;
    curPos: longword;
    curClus: longword;                       // Current cluster to read from
    curSect: integer;                     // Current sector within cluster
    curByte: integer;                     // Current byte within sector
    readBuffer: array [0..511] of byte;   // Buffer used for unaligned reads
    appClus: longword;                       // Cluster to append to
    appSect: integer;                     // Sector within cluster for appending
    appByte: integer;                     // Byte within sector for appending
    read: boolean;                        // Can read from file
    write: boolean;                       // Can write to file
    append: boolean;                      // Can append to file
    inUse: boolean;                       // This file is open
    dirEntSector: longword;                  // The sector where the directory entry is stored
    dirEntOffset: integer;                // The offset within the directory sector
  end;
  P_FILE = ^_FILE;

const
  SEEK_SET = 0;
  SEEK_CUR = 1;
  SEEK_END = 2;

(*
  ------------------------------------------------------------------------------
    Directory iterator for mantaining state between dir* calls
  ------------------------------------------------------------------------------
*)
type
  DIR_ITER = record
    device: longint;
    dirStruct: pointer;
  end;
  PDIR_ITER = ^DIR_ITER;

  stat = packed record
    st_dev: longint;
    st_ino: longword;
    st_mode : longword;
    st_nlink : word;
    st_uid : word;
    st_gid : word;
    st_rdev : longint;
    st_size : longint;
    st_atime : longint;

    st_spare1: longint;
    st_mtime: longint;
    st_spare2: longint;
    st_ctime: longint;
    st_spare3: longint;
    st_blksize: longint;
    st_blocks: longint;
    st_spare4: array [0..1] of longint;
  end;
  TStat = stat;
  PStat = ^stat;

const
  _IFMT    = 0170000;   // type of file
  _IFDIR   = 0040000;   // directory
  _IFCHR   = 0020000; 	// character special
  _IFBLK   = 0060000; 	// block special
  _IFREG   = 0100000; 	// regular
  _IFLNK   = 0120000; 	// symbolic link
  _IFSOCK  = 0140000; 	// socket
  _IFIFO   = 0010000; 	// fifo

  S_BLKSIZE = 1024;  // size of a block

  S_ISUID = 0004000; // set user id on execution
  S_ISGID = 0002000; // set group id on execution

  NAME_MAX = 767;

function S_ISBLK(m: longint): boolean; inline;
function S_ISCHR(m: longint): boolean; inline;
function S_ISDIR(m: longint): boolean; inline;
function S_ISFIFO(m: longint): boolean; inline;
function S_ISREG(m: longint): boolean; inline;
function S_ISLNK(m: longint): boolean; inline;
function S_ISSOCK(m: longint): boolean; inline;


type
  dirent = record
    d_ino: longint;
    d_name: array [0..NAME_MAX] of char;
  end;
  PDirent = ^dirent;
  PPDirent = ^PDirent;

  DIR = record
    position: longint;
    dirData: PDIR_ITER;
    fileData: dirent;
  end;
  PDIR = ^DIR;

(* DIR handling *)
function closedir(dirp: PDIR): longint; cdecl; external;
function opendir(const dirname: pchar): PDIR; cdecl; external;
function readdir(dirp: PDIR): PDirent; cdecl; external;
function readdir_r(dirp: PDIR; entry: PDirent; result: PPDirent): longint; cdecl; external;
procedure rewinddir(dirp: PDIR); cdecl; external;
procedure seekdir(dirp: PDIR; loc: longint); cdecl; external;
function telldir(dirp: PDIR): longint; cdecl; external;


function diropen(const path: pchar): PDIR_ITER; cdecl; external;
function dirreset(dirState: PDIR_ITER): longint; cdecl; external;
function dirnext(dirState: PDIR_ITER; filename: pchar; filestat: Pstat): longint; cdecl; external;
function dirclose(dirState: PDIR_ITER): longint; cdecl; external;

(* File handling *)
function fopen(filename: Pchar; modes: Pchar): P_FILE; cdecl; external;
function fread(ptr: pointer; size: longint; n: longint; stream: P_FILE): longint; cdecl; external;
function fread(var ptr; size: longint; n: longint; var stream: _FILE): longint; cdecl; external;
function fwrite(ptr: pointer; size: longint; n: longint; s: P_FILE): longint; cdecl; external;
function fwrite(var ptr; size: longint; n: longint; var s: _FILE): longint; cdecl; external;
function ftell(stream: P_FILE): longint; cdecl; external;
function ftell(var stream: _FILE): longint; cdecl; external;
function fseek(stream: P_FILE; off: longint; whence: longint): longint; cdecl; external;
function fseek(var stream: _FILE; off: longint; whence: longint): longint; cdecl; external;
function fclose(stream: P_FILE): longint; cdecl; external;
function fclose(var stream: _FILE): longint; cdecl; external;
function isatty(fildes: longint): longint; cdecl; external;
function fileno(para1: P_FILE): longint; cdecl; external;
function fileno(var para1: _FILE): longint; cdecl; external;
function fstat(fildes: longint; buf: PStat): longint; cdecl; external;
function fstat(fildes: longint; var buf: TStat): longint; cdecl; external;
function _stat(__file:Pchar; var __buf:Tstat):longint; cdecl; external name 'stat';
function ftruncate(fildes: longint; len: longint): longint; cdecl; external;
function unlink(path: Pchar): longint; cdecl; external;
function rename(para1: Pchar; para2: Pchar): longint; cdecl; external;
