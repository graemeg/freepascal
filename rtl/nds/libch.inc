{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2002 by the Free Pascal development team

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    libc unit for Nintendo DS
=======
    BIOS functions unit for Nintendo DS
>>>>>>> graemeg/cpstrnew
=======
    BIOS functions unit for Nintendo DS
>>>>>>> graemeg/cpstrnew
=======
    BIOS functions unit for Nintendo DS
>>>>>>> graemeg/cpstrnew
=======
    BIOS functions unit for Nintendo DS
>>>>>>> origin/cpstrnew
=======
    BIOS functions unit for Nintendo DS
>>>>>>> origin/fixes_2.4
    Copyright (c) 2006 by Francesco Lombardi

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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  errno: plongint; external name '__errno'; 

type
{
=======
function __errno: plongint; cdecl; export;

type
>>>>>>> graemeg/cpstrnew
=======
function __errno: plongint; cdecl; export;

type
>>>>>>> graemeg/cpstrnew
=======
function __errno: plongint; cdecl; export;

type
>>>>>>> graemeg/cpstrnew
=======
function __errno: plongint; cdecl; export;

type
>>>>>>> origin/cpstrnew
=======
function __errno: plongint; cdecl; export;

type
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 }
 
   P_iobuf = ^_iobuf;
   _iobuf = record
        reserved : longint;
     end;
   _FILE = _iobuf;
   P_FILE = ^_FILE;
   PP_FILE = ^P_FILE;
 
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
=======

>>>>>>> origin/fixes_2.4
const
  SEEK_SET = 0;
  SEEK_CUR = 1;
  SEEK_END = 2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   R_OK = 1;
   W_OK = 2;
   X_OK = 4;
   F_OK = 8;
   L_SET = SEEK_SET;
   L_INCR = SEEK_CUR;
   L_XTND = SEEK_END;
   EFF_ONLY_OK = 8;
   STDIN_FILENO = 0;
   STDOUT_FILENO = 1;
   STDERR_FILENO = 2;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  stat = record
=======
  stat = packed record
>>>>>>> graemeg/cpstrnew
=======
  stat = packed record
>>>>>>> graemeg/cpstrnew
=======
  stat = packed record
>>>>>>> graemeg/cpstrnew
=======
  stat = packed record
>>>>>>> origin/cpstrnew
=======
  stat = packed record
>>>>>>> origin/fixes_2.4
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

<<<<<<< HEAD
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
=======

>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function _stat(__file: Pchar; var __buf: Tstat): longint; cdecl; external name 'stat';
function _stat(path: Pchar; buf: Pstat): longint; cdecl; external name 'stat';
function ftruncate(fildes: longint; len: longint): longint; cdecl; external;
function unlink(path: Pchar): longint; cdecl; external;
function rename(para1: Pchar; para2: Pchar): longint; cdecl; external;

function _open(path: Pchar; oflag: longint): longint; cdecl; external name 'open';
function _open(path: Pchar; oflag, mode: longint): longint; cdecl; external name 'open';  
function _read(fildes:longint; buf:pointer; nbytes:dword):longint;cdecl;external name 'read';
function _read(fildes:longint; var buf; nbytes:dword):longint;cdecl;external name 'read';
function _write(fildes:longint; buf:pointer; nbytes:dword):longint;cdecl;external name 'write';
function _write(fildes:longint; var buf; nbytes:dword):longint;cdecl;external name 'write';
function _lseek(fildes:longint; offset:longint; whence:longint):longint;cdecl;external name 'lseek';
function _close(fildes:longint): longint; cdecl; external name 'close';
function _unlink(path:Pchar): longint; cdecl; external name 'unlink';
function _rename(para1: Pchar; para2: Pchar): longint; cdecl; external name 'rename';
function _access(path: Pchar; mode: longint): longint; cdecl; external name 'access';
function _tell(fildes: longint): longint; cdecl; external name 'tell';
function _isatty(fildes: longint): longint; cdecl; external name 'isatty';
function _truncate(fildes: longint; len: longint): longint; cdecl; external name 'truncate';



const
 F_GETFL      = 1;        // get file status flags
 F_SETFL      = 2;        // set file status flags
 F_DUPFD      = 3;        // duplicate file descriptor
 F_GETFD      = 4;        // get file descriptor flags
 F_SETFD      = 5;        // set file descriptor flags
 F_SETLK      = 6;        // set record locking info
 F_SETLK64    = 16;       // set record locking info (64-bit)
 F_GETLK      = 7;        // get record locking info
 F_GETLK64    = 17;       // get record locking info (64-bit)
 F_SETLKW     = 8;        // get record locking info; wait if blocked
 F_SETLKW64   = 18;       // get record locking info (64-bit)
 F_CLOEXEC    = 9;        // close on execute

// values for 'l_type' field of 'struct flock'...
 F_RDLCK      = 1;        // shared or read lock
 F_WRLCK      = 2;        // exclusive or write lock
 F_UNLCK      = 3;        // unlock

// values for 'oflag' in open()...
 O_RDONLY     =$00000000; // open for read only
 O_WRONLY     =$00000001; // open for write only
 O_RDWR       =$00000002; // open for read and write
 O_ACCMODE    =$00000003; // access flags mask
 O_reserved1  =$00000004; // reserved
 O_reserved2  =$00000008; // reserved
 O_APPEND     =$00000010; // writes done at end of file
 O_CREAT      =$00000020; // create new file
 O_TRUNC      =$00000040; // truncate existing file
 O_EXCL       =$00000080; // exclusive open
 O_NOCTTY     =$00000100; // no controlling terminal--unsupported
 O_BINARY     =$00000200; // binary file--all files
 O_NDELAY     =$00000400; // nonblocking flag
 O_reserved3  =$00000800; // reserved
 O_SYNC       =$00001000; // synchronized I/O file integrity
 O_DSYNC      =$00002000; // synchronized I/O data integrity
 O_RSYNC      =$00004000; // synchronized read I/O
 O_NONBLOCK   = O_NDELAY; // alias
 FD_CLOEXEC   =$00008000; // parent closes after call to process()
 O_UPDATE     =$00010000; // keep legacy files updated
 O_FIFO       =$00100000; // opening one end of a FIFO [non-standard]

// value for third argument when 'cmd' is F_SETFL in fcntl()...
 FNDELAY      = O_NDELAY;   // fcntl() non-blocking I/O

// 'shflag' values for sopen()...
 SH_DENYRW    = $00000010; // deny read/write mode
 SH_DENYWR    = $00000020; // deny write mode
 SH_DENYRD    = $00000030; // deny read mode
 SH_DENYNO    = $00000040; // deny none mode
 
Const

Sys_EPERM       = 1;    { Operation not permitted }
Sys_ENOENT      = 2;    { No such file or directory }
Sys_ESRCH       = 3;    { No such process }
Sys_EINTR       = 4;    { Interrupted system call }
Sys_EIO = 5;    { I/O error }
Sys_ENXIO       = 6;    { No such device or address }
Sys_E2BIG       = 7;    { Arg list too long }
Sys_ENOEXEC     = 8;    { Exec format error }
Sys_EBADF       = 9;    { Bad file number }
Sys_ECHILD      = 10;   { No child processes }
Sys_EAGAIN      = 11;   { Try again }
Sys_ENOMEM      = 12;   { Out of memory }
Sys_EACCES      = 13;   { Permission denied }
Sys_EFAULT      = 14;   { Bad address }
Sys_ENOTBLK     = 15;   { Block device required, NOT POSIX! }
Sys_EBUSY       = 16;   { Device or resource busy }
Sys_EEXIST      = 17;   { File exists }
Sys_EXDEV       = 18;   { Cross-device link }
Sys_ENODEV      = 19;   { No such device }
Sys_ENOTDIR     = 20;   { Not a directory }
Sys_EISDIR      = 21;   { Is a directory }
Sys_EINVAL      = 22;   { Invalid argument }
Sys_ENFILE      = 23;   { File table overflow }
Sys_EMFILE      = 24;   { Too many open files }
Sys_ENOTTY      = 25;   { Not a typewriter }
Sys_ETXTBSY     = 26;   { Text file busy. The new process was
                      a pure procedure (shared text) file which was
                      open for writing by another process, or file
                      which was open for writing by another process,
                      or while the pure procedure file was being
                      executed an open(2) call requested write access
                      requested write access.}
Sys_EFBIG       = 27;   { File too large }
Sys_ENOSPC      = 28;   { No space left on device }
Sys_ESPIPE      = 29;   { Illegal seek }
Sys_EROFS       = 30;   { Read-only file system }
Sys_EMLINK      = 31;   { Too many links }
Sys_EPIPE       = 32;   { Broken pipe }
Sys_EDOM        = 33;   { Math argument out of domain of func }
Sys_ERANGE      = 34;   { Math result not representable }
Sys_EDEADLK     = 35;   { Resource deadlock would occur }
Sys_ENAMETOOLONG= 36;   { File name too long }
Sys_ENOLCK      = 37;   { No record locks available }
Sys_ENOSYS      = 38;   { Function not implemented }
Sys_ENOTEMPTY= 39;      { Directory not empty }
Sys_ELOOP       = 40;   { Too many symbolic links encountered }
Sys_EWOULDBLOCK = Sys_EAGAIN;   { Operation would block }
Sys_ENOMSG      = 42;   { No message of desired type }
Sys_EIDRM       = 43;   { Identifier removed }
Sys_ECHRNG      = 44;   { Channel number out of range }
Sys_EL2NSYNC= 45;       { Level 2 not synchronized }
Sys_EL3HLT      = 46;   { Level 3 halted }
Sys_EL3RST      = 47;   { Level 3 reset }
Sys_ELNRNG      = 48;   { Link number out of range }
Sys_EUNATCH     = 49;   { Protocol driver not attached }
Sys_ENOCSI      = 50;   { No CSI structure available }
Sys_EL2HLT      = 51;   { Level 2 halted }
Sys_EBADE       = 52;   { Invalid exchange }
Sys_EBADR       = 53;   { Invalid request descriptor }
Sys_EXFULL      = 54;   { Exchange full }
Sys_ENOANO      = 55;   { No anode }
Sys_EBADRQC     = 56;   { Invalid request code }
Sys_EBADSLT     = 57;   { Invalid slot }
Sys_EDEADLOCK= 58;      { File locking deadlock error }
Sys_EBFONT      = 59;   { Bad font file format }
Sys_ENOSTR      = 60;   { Device not a stream }
Sys_ENODATA     = 61;   { No data available }
Sys_ETIME       = 62;   { Timer expired }
Sys_ENOSR       = 63;   { Out of streams resources }
Sys_ENONET      = 64;   { Machine is not on the network }
Sys_ENOPKG      = 65;   { Package not installed }
Sys_EREMOTE     = 66;   { Object is remote }
Sys_ENOLINK     = 67;   { Link has been severed }
Sys_EADV        = 68;   { Advertise error }
Sys_ESRMNT      = 69;   { Srmount error }
Sys_ECOMM       = 70;   { Communication error on send }
Sys_EPROTO      = 71;   { Protocol error }
Sys_EMULTIHOP= 72;      { Multihop attempted }
Sys_EDOTDOT     = 73;   { RFS specific error }
Sys_EBADMSG     = 74;   { Not a data message }
Sys_EOVERFLOW= 75;      { Value too large for defined data type }
Sys_ENOTUNIQ= 76;       { Name not unique on network }
Sys_EBADFD      = 77;   { File descriptor in bad state }
Sys_EREMCHG     = 78;   { Remote address changed }
Sys_ELIBACC     = 79;   { Can not access a needed shared library }
Sys_ELIBBAD     = 80;   { Accessing a corrupted shared library }
Sys_ELIBSCN     = 81;   { .lib section in a.out corrupted }
Sys_ELIBMAX     = 82;   { Attempting to link in too many shared libraries }
Sys_ELIBEXEC= 83;       { Cannot exec a shared library directly }
Sys_EILSEQ      = 84;   { Illegal byte sequence }
Sys_ERESTART= 85;       { Interrupted system call should be restarted }
Sys_ESTRPIPE= 86;       { Streams pipe error }
Sys_EUSERS      = 87;   { Too many users }
Sys_ENOTSOCK= 88;       { Socket operation on non-socket }
Sys_EDESTADDRREQ= 89;   { Destination address required }
Sys_EMSGSIZE= 90;       { Message too long }
Sys_EPROTOTYPE= 91;     { Protocol wrong type for socket }
Sys_ENOPROTOOPT= 92;    { Protocol not available }
Sys_EPROTONOSUPPORT= 93;        { Protocol not supported }
Sys_ESOCKTNOSUPPORT= 94;        { Socket type not supported }
Sys_EOPNOTSUPP= 95;     { Operation not supported on transport endpoint }
Sys_EPFNOSUPPORT= 96;   { Protocol family not supported }
Sys_EAFNOSUPPORT= 97;   { Address family not supported by protocol }
Sys_EADDRINUSE= 98;     { Address already in use }
Sys_EADDRNOTAVAIL= 99;  { Cannot assign requested address }
Sys_ENETDOWN= 100;      { Network is down }
Sys_ENETUNREACH= 101;   { Network is unreachable }
Sys_ENETRESET= 102;     { Network dropped connection because of reset }
Sys_ECONNABORTED= 103;  { Software caused connection abort }
Sys_ECONNRESET= 104;    { Connection reset by peer }
Sys_ENOBUFS     = 105;  { No buffer space available }
Sys_EISCONN     = 106;  { Transport endpoint is already connected }
Sys_ENOTCONN= 107;      { Transport endpoint is not connected }
Sys_ESHUTDOWN= 108;     { Cannot send after transport endpoint shutdown }
Sys_ETOOMANYREFS= 109;  { Too many references: cannot splice }
Sys_ETIMEDOUT= 110;     { Connection timed out }
Sys_ECONNREFUSED= 111;  { Connection refused }
Sys_EHOSTDOWN= 112;     { Host is down }
Sys_EHOSTUNREACH= 113;  { No route to host }
Sys_EALREADY= 114;      { Operation already in progress }
Sys_EINPROGRESS= 115;   { Operation now in progress }
Sys_ESTALE      = 116;  { Stale NFS file handle }
Sys_EUCLEAN     = 117;  { Structure needs cleaning }
Sys_ENOTNAM     = 118;  { Not a XENIX named type file }
Sys_ENAVAIL     = 119;  { No XENIX semaphores available }
Sys_EISNAM      = 120;  { Is a named type file }
Sys_EREMOTEIO= 121;     { Remote I/O error }
Sys_EDQUOT      = 122;  { Quota exceeded }


{ This value was suggested by Daniel
  based on infos from www.linuxassembly.org }

Sys_ERROR_MAX = $fff;
 
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
function _stat(__file:Pchar; var __buf:Tstat):longint; cdecl; external name 'stat';
function ftruncate(fildes: longint; len: longint): longint; cdecl; external;
function unlink(path: Pchar): longint; cdecl; external;
function rename(para1: Pchar; para2: Pchar): longint; cdecl; external;
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
