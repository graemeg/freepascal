
{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    Types and structures for the BaseUnix unit.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ***********************************************************************}

{***********************************************************************}
{                         Base Unix Structures                          }
{***********************************************************************}

{$IFDEF FPC_IS_SYSTEM}
  {$i ptypes.inc}
{$ENDIF}


CONST
//  SYS_NMLM                 = 65;
    UTSNAME_LENGTH           = SYS_NMLN;
    UTSNAME_NODENAME_LENGTH  = UTSNAME_LENGTH;
    {$ifdef usedomain}
    UTSNAME_DOMAIN_LENGTH    = UTSNAME_LENGTH;
    {$endif}

   FD_MAXFDSET     = 1024;
   BITSINWORD      = 8*sizeof(cuLong);
   wordsinsigset   = SIG_MAXSIG DIV BITSINWORD;         // words in sigset_t
   wordsinfdset    = FD_MAXFDSET DIV BITSINWORD;        // words in fdset_t
   {$ifdef cpu32}
   ln2bitsinword   = 5;                                 { 32bit : ln(32)/ln(2)=5 }
   {$else cpu32}
   {$ifdef cpu64}
   ln2bitsinword   = 6;                                 { 64bit : ln(64)/ln(2)=6 }
   {$endif cpu64}
   {$endif cpu32}
   ln2bitmask      = 1 shl ln2bitsinword - 1;


TYPE
   Blksize_t  = cuint;
   Blkcnt_t   = cuint;
   Blkcnt64_t = cuint64;

   TBlkSize   = BlkSize_t;
   PBlkSize   = ^BlkSize_t;
   TBlkCnt    = Blkcnt_t;
   PBlkCnt    = ^Blkcnt_t;

   { system information services }
   UtsName   = Record
                Sysname : Array[0..UTSNAME_LENGTH -1] OF Char;   // Name of this OS
                Nodename: Array[0..UTSNAME_NODENAME_LENGTH-1] OF Char;   // Name of this network node.
                Release : Array[0..UTSNAME_LENGTH -1] OF Char;   // Release level.
                Version : Array[0..UTSNAME_LENGTH -1] OF Char;   // Version level.
                Machine : Array[0..UTSNAME_LENGTH -1] OF Char;   // Hardware type.
               {$ifdef usedomain}
                Domain  : array[0..UTSNAME_DOMAIN_LENGTH-1] of char {$ifndef ver2_2} platform; {$endif}  // Linux addition "Domain"
               {$endif}
               end;
  TUtsName   = UtsName;
  PUtsName   = ^TUtsName;


{ Definition of (kernel) stat type }
{ see kernel/include/asm-<cpu>/stat.h, include/linux/types.h and }
{ include /include/asm-<cpu>/posix-types.h                       }

const
{$ifndef cpux86_64}
  _STAT_VER_LINUX_OLD = 1;
  _STAT_VER_KERNEL = 1;
  _STAT_VER_SVR4 = 2;
  _STAT_VER_LINUX = 3;
{$else}
  _STAT_VER_KERNEL = 0;
  _STAT_VER_LINUX = 1;
{$endif}
  _STAT_VER = _STAT_VER_LINUX;

type

{$i stat.inc}

  TStat      = Stat;
  PStat      = ^Stat;

  { directory services }
  { the Dirent type for getdents64 is no longer declared as ending with
    an array 0..255, but as ending with a variable-sized array. While the
    maximum file name length hasn't changed, the getdents64 system call
    code does always check whether the size of the struct is a multiple
    of 8 bytes. On platforms where the maximum alignment is 4 bytes, the
    size of the Dirent struct ending with an array of 256 chars is
    however only 276 bytes. As a result, getdents64 fails for long file
    names with that Dirent size.

    Rather than slightly increasing the size to 280 bytes, immediately
    make it much larger so that the kernel can cache more entries per
    system call and as a result we have to perform less system calls.
  }
  Dirent   = record
                d_fileno      : ino64_t;                        // file number of entry
                d_off         : off_t;
                d_reclen      : cushort;                        // length of string in d_name
                d_type        : cuchar;                         // file type, see below
                d_name        : array[0..4095-sizeof(ino64_t)-sizeof(off_t)-sizeof(cushort)-sizeof(cuchar)] of char;  // name buffer + padding for extra records; align total record size to a page size
               end;
  TDirent  = Dirent;
  pDirent  = ^Dirent;

{$ifdef oldreaddir}
           { Still old one. This is a userland struct}

   Dir       = record
                dd_fd     : longint;
                dd_loc    : longint;
                dd_size   : longint;
                dd_buf    : pdirent;
                {The following are used in libc, but NOT in the linux kernel sources ??}
                dd_nextoff: cardinal;
                dd_max : integer; {size of buf. Irrelevant, as buf is of type dirent}
                dd_lock   : pointer;
               end;
{$else}
        // new libc one. NOTE that off_t must be real, so 64-bit ifdef
   Dir       = Record   // packing doesn't matter. This is a userland struct.
                fd      : cint;
                data    : pchar;
                allocation: size_t;
                _size     : size_t;
                offset    : size_t;
                filepos   : off_t;
                end;
{$endif}

   TDir      = Dir;
   pDir      = ^Dir;


   UTimBuf   = Record
                 actime  : time_t;
                 modtime : time_t;
                end;

   TUtimBuf  = UtimBuf;
   pUtimBuf  = ^UtimBuf;

   kernel_off_t = clong;
   kernel_loff_t = clonglong;

{$if defined(cpu32) and not(defined(cpupowerpc)) and not(defined(cpumips)) and not defined(cpumipsel))}
   kernel_mode_t = cushort;
   kernel_uid_t = cushort;
   kernel_gid_t = cushort;
{$else}
   kernel_mode_t = cuint;
   kernel_uid_t = cuint;
   kernel_gid_t = cuint;
{$endif}

   FLock     = Record
                l_type  : cshort;       { lock type: read/write, etc. }
                l_whence: cshort;       { type of l_start }
                l_start : kernel_off_t; { starting offset }
                l_len   : kernel_off_t; { len = 0 means until end of file }
                l_pid   : pid_t;        { lock owner }
{$if defined(cpusparc) or defined(cpusparc64)}
                __pad   : cshort;
{$endif}
               End;

{$ifndef cpu64}
   FLock64   = Record
                l_type  : cshort;        { lock type: read/write, etc. }
                l_whence: cshort;        { type of l_start }
                l_start : kernel_loff_t; { starting offset }
                l_len   : kernel_loff_t; { len = 0 means until end of file }
                l_pid   : pid_t;         { lock owner }
{$ifdef cpusparc}
                __pad   : cshort;
{$endif}
               End;
{$endif}

   tms       = Record
                tms_utime  : clock_t;   { User CPU time }
                tms_stime  : clock_t;   { System CPU time }
                tms_cutime : clock_t;   { User CPU time of terminated child procs }
                tms_cstime : clock_t;   { System CPU time of terminated child procs }
               end;
   TTms      = tms;
   PTms      = ^tms;

  TFDSetEl  = cuLong;
  TFDSet    = ARRAY[0..(FD_MAXFDSET div BITSINWORD)-1] of TFDSetEl;
  pFDSet    = ^TFDSet;

  timezone = record
    tz_minuteswest,tz_dsttime:cint;
  end;
  ptimezone =^timezone;
  TTimeZone = timezone;

const
  POLLIN      = $0001;
  POLLPRI     = $0002;
  POLLOUT     = $0004;
  POLLERR     = $0008;
  POLLHUP     = $0010;
  POLLNVAL    = $0020;

  { XOpen, XPG 4.2 }
  POLLRDNORM  = $0040;
  POLLRDBAND  = $0080;
  POLLWRNORM  = $0100;
  POLLWRBAND  = $0200;

type
  pollfd = record
    fd: cint;
    events: cshort;
    revents: cshort;
  end;
  tpollfd = pollfd;
  ppollfd = ^pollfd;

{***********************************************************************}
{                  POSIX CONSTANT ROUTINE DEFINITIONS                   }
{***********************************************************************}
CONST
    { access routine - these maybe OR'ed together }
    F_OK      =          0;        { test for existence of file }
    R_OK      =          4;        { test for read permission on file }
    W_OK      =          2;        { test for write permission on file }
    X_OK      =          1;        { test for execute or search permission }
    { seek routine }
    SEEK_SET  =          0;        { seek from beginning of file }
    SEEK_CUR  =          1;        { seek from current position  }
    SEEK_END  =          2;        { seek from end of file       }
    { open routine                                 }
    { File access modes for `open' and `fcntl'.    }
    O_RDONLY  =          0;        { Open read-only.  }
    O_WRONLY  =          1;        { Open write-only. }
    O_RDWR    =          2;        { Open read/write. }
{$if defined(cpusparc) or defined(cpusparc64)}
    O_APPEND  =          8;
    O_CREAT   =       $200;
    O_TRUNC   =       $400;
    O_EXCL    =       $800;
    O_SYNC    =      $2000;
    O_NONBLOCK =     $4000;
    O_NDELAY  =      O_NONBLOCK or 4;
    O_NOCTTY  =      $8000;
    O_DIRECTORY =   $10000;
    O_NOFOLLOW =    $20000;
    O_DIRECT  =    $100000;
{$else : not (cpusparc or cpusparc64)}
{$ifdef cpumips}
    O_CREAT   =       $100;
    O_EXCL    =       $400;
    O_NOCTTY  =       $800;
    O_TRUNC   =       $200;
    O_APPEND  =         $8;
    O_NONBLOCK =       $80;
    O_NDELAY  =     O_NONBLOCK;
    O_SYNC    =        $10;
    O_DIRECT  =      $8000;
    O_DIRECTORY =   $10000;
    O_NOFOLLOW =    $20000;
{$else : not cpumips}
    O_CREAT   =        $40;
    O_EXCL    =        $80;
    O_NOCTTY  =       $100;
    O_TRUNC   =       $200;
    O_APPEND  =       $400;
    O_NONBLOCK =      $800;
    O_NDELAY  =     O_NONBLOCK;
    O_SYNC    =      $1000;
    O_DIRECT  =      $4000;
    O_DIRECTORY =   $10000;
    O_NOFOLLOW =    $20000;
{$endif not cpumips}
{$endif not (cpusparc or cpusparc64)}

    AT_FDCWD = -100;
    AT_SYMLINK_NOFOLLOW = $100;
    AT_REMOVEDIR = $200;
    AT_SYMLINK_FOLLOW = $400;
    AT_NO_AUTOMOUNT = $800;
    AT_EMPTY_PATH = $1000;
    AT_STATX_SYNC_TYPE = $6000;
    AT_STATX_SYNC_AS_STAT = $0000;
    AT_STATX_FORCE_SYNC = $2000;
    AT_STATX_DONT_SYNC = $4000;
    AT_RECURSIVE = $8000;

    { SIGCHLD or CLONE_CHILD_CLEARTID or CLONE_CHILD_SETTID }
    clone_flags_fork = $01200011;

{$if defined(cpuarm) or defined(cpualpha) or defined(cpublackfin) or defined(cpum68k) or defined(aarch64) or defined(riscv32) or defined(riscv64)}
    O_LARGEFILE =   $20000;
{$endif}
{$if defined(cpusparc) or defined(cpusparc64)}
    O_LARGEFILE =   $40000;
{$endif}
{$if defined(cpupowerpc)}
    O_LARGEFILE =   $10000;
{$endif}
{$if defined(cpui386) or defined(cpux86_64) or defined(cpuia64)}
    O_LARGEFILE =    $8000;
{$endif}
{$if defined(cpumips) or defined(cpumipsel)}
    O_LARGEFILE =    $2000;
{$endif}
    { mode_t possible values                                 }
    S_IRUSR =  %0100000000;     { Read permission for owner   }
    S_IWUSR =  %0010000000;     { Write permission for owner  }
    S_IXUSR =  %0001000000;     { Exec  permission for owner  }
    S_IRGRP =  %0000100000;     { Read permission for group   }
    S_IWGRP =  %0000010000;     { Write permission for group  }
    S_IXGRP =  %0000001000;     { Exec permission for group   }
    S_IROTH =  %0000000100;     { Read permission for world   }
    S_IWOTH =  %0000000010;     { Write permission for world  }
    S_IXOTH =  %0000000001;     { Exec permission for world   }
    S_IRWXU =  S_IRUSR or S_IWUSR or S_IXUSR;
    S_IRWXG =  S_IRGRP or S_IWGRP or S_IXGRP;
    S_IRWXO =  S_IROTH or S_IWOTH or S_IXOTH;

    { Used for waitpid }
    WNOHANG   =          1;     { don't block waiting               }
    WUNTRACED =          2;     { report status of stopped children }

  { File types }
  S_IFMT  = 61440; { type of file mask}
  S_IFIFO = 4096;  { named pipe (fifo)}
  S_IFCHR = 8192;  { character special}
  S_IFDIR = 16384; { directory }
  S_IFBLK = 24576; { block special}
  S_IFREG = 32768; { regular }
  S_IFLNK = 40960; { symbolic link }
  S_IFSOCK= 49152; { socket }
  S_ISUID = &4000;
  S_ISGID = &2000;
  S_ISVTX = &1000;

  { Constansts for MMAP }
 {$IFDEF FPC_IS_SYSTEM}
  MAP_PRIVATE   =2;
 {$ENDIF}

 {$if defined(cpumips) or  defined(cpumipsel)}
  MAP_ANONYMOUS  = $800;
  MAP_GROWSDOWN  = $1000;       { stack-like segment }
  MAP_DENYWRITE  = $2000;       { ETXTBSY }
  MAP_EXECUTABLE = $4000;      { mark it as an executable }
  MAP_LOCKED     = $8000;      { pages are locked }
  MAP_NORESERVE  = $4000;      { don't check for reservations }
 {$else}
  MAP_ANONYMOUS  = $20;
  MAP_GROWSDOWN  = $100;       { stack-like segment }
  MAP_DENYWRITE  = $800;       { ETXTBSY }
  MAP_EXECUTABLE = $1000;      { mark it as an executable }
  MAP_LOCKED     = $2000;      { pages are locked }
  MAP_NORESERVE  = $4000;      { don't check for reservations }
 {$endif cpumips}

  { For File control mechanism }
  F_GetFd  = 1;
  F_SetFd  = 2;
  F_GetFl  = 3;
  F_SetFl  = 4;
  F_GetLk  = 5;
  F_SetLk  = 6;
  F_SetLkW = 7;
  F_SetOwn = 8;
  F_GetOwn = 9;

{ getrlimit/ugetrlimit resource parameter constants }
const
  RLIMIT_CPU = 0;       { CPU time in ms  }
  RLIMIT_FSIZE = 1;     { Maximum filesize  }
  RLIMIT_DATA = 2;      { max data size  }
  RLIMIT_STACK = 3;     { max stack size  }
  RLIMIT_CORE = 4;      { max core file size  }
  RLIMIT_RSS = 5;       { max resident set size  }
  RLIMIT_NPROC = 6;     { max number of processes  }
  RLIMIT_NOFILE = 7;    { max number of open files  }
  RLIMIT_MEMLOCK = 8;   { max locked-in-memory address space  }
  RLIMIT_AS = 9;        { address space limit(?)  }
  RLIMIT_LOCKS = 10;    { maximum file locks held  }

type
  rlim_t = cULong;
  PRLimit = ^TRLimit;
  TRLimit = record
    rlim_cur : rlim_t;
    rlim_max : rlim_t;
  end;

  iovec = record
            iov_base : pointer;
	    iov_len  : size_t;
	   end;
  tiovec=iovec;
  piovec=^tiovec;		

{$if defined(cpupowerpc)}
const
  { FP exception related constants for prctl(); PowerPC specific }
  PR_GET_FPEXC    = 11;   { get floating point exception mode }
  PR_SET_FPEXC    = 12;   { set floating point exception mode }

  PR_FP_EXC_DISABLED = 0; { FP exceptions disabled }
  PR_FP_EXC_NONREC   = 1; { async non-recoverable exc. mode }
  PR_FP_EXC_ASYNC    = 2; { async recoverable exc. mode }
  PR_FP_EXC_PRECISE  = 3; { precise exception mode }
{$endif}

    {*************************************************************************}
    {                               SIGNALS                                   }
    {*************************************************************************}

{$i signal.inc}

