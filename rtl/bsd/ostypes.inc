{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    Types and structures for baseunix unit, also used in system.

    This file implements all the types/constants which must
    be defined to port FPC to a new POSIX compliant OS.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{***********************************************************************}
{                         POSIX STRUCTURES                              }
{***********************************************************************}

{$ifdef FPC_IS_SYSTEM}
  {$i ptypes.inc}
{$ENDIF}

{$ifdef cpupowerpc}
  {$ifdef netbsd}
    {$define netbsdpowerpc}
  {$endif}
{$endif}

{$if (defined(darwin) and (defined(cpuarm) or defined(cpuaarch64))) or defined(iphonesim)}
  {$define darwin_new_iostructs}
{$endif}

//      CONST SYS_NMLN=65;

// Can't find these two in Posix and in FreeBSD
//CONST
//    _UTSNAME_LENGTH = ;
//    _UTSNAME_NODENAME_LENGTH = ;

CONST                           // OS specific parameters for general<fd,sig>set behaviour
   BITSINWORD      = 8*sizeof(longint);
   FD_MAXFDSET     = 1024;
   ln2bitsinword   = 5;         { 32bit : ln(32)/ln(2)=5 }
   ln2bitmask      = 1 shl ln2bitsinword - 1;
   wordsinfdset    = FD_MAXFDSET DIV BITSINWORD;        // words in fdset_t
   wordsinsigset   = SIG_MAXSIG  DIV BITSINWORD;

TYPE
   { system information services }
   utsname = record
              sysname : Array[0..SYS_NMLN-1] OF Char;   // Name of this OS
              nodename: Array[0..SYS_NMLN-1] OF Char;   // Name of this network node.
              release : Array[0..SYS_NMLN-1] OF Char;   // Release level.
              version : Array[0..SYS_NMLN-1] OF Char;   // Version level.
              machine : Array[0..SYS_NMLN-1] OF Char;   // Hardware type.
             end;
  TUtsName= utsname;
  pUtsName= ^utsname;

  { file characteristics services }
   stat    = record { the types are real}
{$ifdef dragonfly}
        st_ino        : ino_t;             // inode's number
        st_nlink      : nlink_t;           // number of hard links
        st_dev        : dev_t;             // inode's device
        st_mode       : mode_t;            // inode protection mode
        st_padd1      : cuint16;
        st_uid        : uid_t;             // user ID of the file's owner
        st_gid        : gid_t;             // group ID of the file's group
        st_rdev       : dev_t;             // device type
        st_atime      : time_t;            // time of last access (half timespec)
        st_atimensec  : clong;             // nsec of last access (half timespec)
        st_mtime      : time_t;            // time of last data modification
        st_mtimensec  : clong;             // nsec of last data modification
        st_ctime      : time_t;            // time of last file status change
        st_ctimensec  : clong;             // nsec of last file status change
        st_size       : off_t;             // file size, in bytes
        st_blocks     : cint64;            // blocks allocated for file
        st_blksize    : cuint32;           // optimal blocksize for I/O
        st_flags      : cuint32;           // user defined flags for file
        st_gen        : cuint32;           // file generation number
        st_lspare     : cint32;
        st_qspare1    : cint64;            // was recursive change detect
        st_qspare2    : cint64;
{$else dragonfly}
        st_dev        : dev_t;             // inode's device
{$ifdef darwin_new_iostructs}
        st_mode       : mode_t;            // inode protection mode
        st_nlink      : nlink_t;           // number of hard links
        st_ino        : ino_t;             // inode's number
{$else}
{$ifdef netbsd_use_stat30}
     { order is inverted for better alignment probably }
        st_mode       : mode_t;            // inode protection mode
        st_ino        : ino_t;             // inode's number
{$else not netbsd}
        st_ino        : ino_t;             // inode's number
        st_mode       : mode_t;            // inode protection mode
{$endif not netbsd}
        st_nlink      : nlink_t;           // number of hard links
{$endif}
        st_uid        : uid_t;             // user ID of the file's owner
        st_gid        : gid_t;             // group ID of the file's group
        st_rdev       : dev_t;             // device type
{$ifdef openbsd}
        st_padd0      : cint;
{$endif}
        st_atime      : time_t;            // time of last access
        st_atimensec  : clong;             // nsec of last access
        st_mtime      : time_t;            // time of last data modification
        st_mtimensec  : clong;             // nsec of last data modification
        st_ctime      : time_t;            // time of last file status change
        st_ctimensec  : clong;             // nsec of last file status change
{$ifdef darwin_new_iostructs}
        st_birthtime  : time_t;            // File creation time
        st_birthtimensec : clong;          // nsec of file creation time
{$endif}
{$ifdef netbsdPowerpc}
        st_padd1              : cint;
{$endif}
        st_size       : off_t;             // file size, in bytes
        st_blocks     : cint64;            // blocks allocated for file
        st_blksize    : cuint32;           // optimal blocksize for I/O
        st_flags      : cuint32;           // user defined flags for file
        st_gen        : cuint32;           // file generation number
{$ifdef netbsdPowerpc}
        st_padd2              : cint;
{$endif}
{$ifndef NetBSD}
        st_lspare     : cint32;
{$endif}
{$ifdef openbsd}
        st_birthtime  : time_t;            // File creation time
        st_birthtimensec : clong;          // nsec of file creation time
{$endif}
        st_qspare     : array[0..1] Of cint64;
{$endif dragonfly}
   end;
   TStat = stat;
   pStat = ^stat;

  { directory services }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef darwin_new_iostructs}
{$ifdef dragonfly}
   dirent  = record
        d_fileno      : ino_t;                          // file number of entry
        d_namlen      : cuint16;                        // strlen (d_name)
        d_type        : cuint8;                         // file type, see below
        d_unused1     : cuint8;                         // padding, reserved
        d_unused2     : cuint32;                        // reserved
        d_name        : array[0..255] of char;          // name, null terminated
   end;
{$else}
=======
{$ifndef darwinarm}
>>>>>>> graemeg/cpstrnew
=======
{$ifndef darwinarm}
>>>>>>> graemeg/cpstrnew
=======
{$ifndef darwinarm}
>>>>>>> graemeg/cpstrnew
=======
{$ifndef darwinarm}
>>>>>>> origin/cpstrnew
   dirent  = record
        d_fileno      : cuint32;                        // file number of entry
        d_reclen      : cuint16;                        // length of this record
        d_type        : cuint8;                         // file type, see below
        d_namlen      : cuint8;                         // length of string in d_name
        d_name        : array[0..(255 + 1)-1] of char;  // name must be no longer than this
   end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
{$else not darwin_new_iostructs}
=======
{$else not darwinarm}
>>>>>>> graemeg/cpstrnew
=======
{$else not darwinarm}
>>>>>>> graemeg/cpstrnew
=======
{$else not darwinarm}
>>>>>>> graemeg/cpstrnew
=======
{$else not darwinarm}
>>>>>>> origin/cpstrnew
   {$packrecords 4}
   { available on Mac OS X 10.6 and later, and used by all iPhoneOS versions }
   dirent  = record
        d_fileno      : cuint64;                        // file number of entry
        d_seekoff     : cuint64;                        // seek offset (optional, used by servers)
        d_reclen      : cuint16;                        // length of this record
        d_namlen      : cuint16;                        // length of string in d_name
        d_type        : cuint8;                         // file type, see below
        d_name        : array[0..PATH_MAX-1] of char;        // name must be no longer than this
   end;
   {$packrecords c}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif darwin_new_iostructs}
=======
{$endif darwinarm}
>>>>>>> graemeg/cpstrnew
=======
{$endif darwinarm}
>>>>>>> graemeg/cpstrnew
=======
{$endif darwinarm}
>>>>>>> graemeg/cpstrnew
=======
{$endif darwinarm}
>>>>>>> origin/cpstrnew
   TDirent = dirent;
   pDirent = ^dirent;

   dir     = record
        dd_fd     : cint;         // file descriptor associated with directory
        dd_loc    : clong;        // offset in current buffer
        dd_size   : clong;        // amount of data returned by getdirentries
        dd_buf    : pchar;        // data buffer
        dd_len    : cint;         // size of data buffer
{$ifdef netbsdpowerpc}
        dd_pad1   : cint;
        dd_seek   : cint64;        // magic cookie returned by getdirentries
{$else}
        dd_seek   : clong;        // magic cookie returned by getdirentries
{$endif}
        dd_rewind : clong;        // magic cookie for rewinding
        dd_flags  : cint;         // flags for readdir
{$ifdef darwin}
        __dd_lock : pthread_mutex_t; // for thread locking
        __dd_td : pointer;        // telldir position recording
{$endif}
   end;
   TDir    = dir;
   pDir    = ^dir;

   utimbuf  = record
                actime  : time_t;
                modtime : time_t;
                end;
   TUtimBuf = utimbuf;
   putimbuf = ^utimbuf;

   flock    = record
                l_start : off_t;        { starting offset }
                l_len   : off_t;        { len = 0 means until end of file }
                l_pid   : pid_t;        { lock owner }
                l_type  : cshort;       { lock type: read/write, etc. }
                l_whence: cshort;       { type of l_start }
                end;
   TFlock   = flock;
   pFlock   = ^flock;

 tms = record
         tms_utime  : clock_t;  { User CPU time }
         tms_stime  : clock_t;  { System CPU time }
         tms_cutime : clock_t;  { User CPU time of terminated child procs }
         tms_cstime : clock_t;  { System CPU time of terminated child procs }
         end;
 TTms= tms;
 pTms= ^tms;

 TFDSet    = ARRAY[0..(FD_MAXFDSET div 32)-1] of Cardinal;
 pFDSet    = ^TFDSet;

{***********************************************************************}
{                  POSIX CONSTANT ROUTINE DEFINITIONS                   }
{***********************************************************************}
CONST
    { access routine - these maybe OR'ed together }
    F_OK        =     0;        { test for existence of file }
    R_OK        =     4;        { test for read permission on file }
    W_OK        =     2;        { test for write permission on file }
    X_OK        =     1;        { test for execute or search permission }
    { seek routine }
    SEEK_SET    =     0;        { seek from beginning of file }
    SEEK_CUR    =     1;        { seek from current position  }
    SEEK_END    =     2;        { seek from end of file       }
    { open routine                                 }
    { File access modes for `open' and `fcntl'.    }
    O_RDONLY    =     0;        { Open read-only.  }
    O_WRONLY    =     1;        { Open write-only. }
    O_RDWR      =     2;        { Open read/write. }
    { Bits OR'd into the second argument to open.  }
    O_CREAT     =  $200;        { Create file if it doesn't exist.  }
    O_EXCL      =  $800;        { Fail if file already exists.      }
    O_TRUNC     =  $400;        { Truncate file to zero length.     }
    O_NOCTTY    = $8000;        { Don't assign a controlling terminal. }
    { File status flags for `open' and `fcntl'.  }
    O_APPEND    =     8;        { Writes append to the file.        }
    O_NONBLOCK  =     4;        { Non-blocking I/O.                 }

{$if defined(freebsd) or defined(dragonfly)}
    { Other }
    O_SHLOCK    =   $10;        { Open with shared file lock }
    O_EXLOCK    =   $20;        { Open with exclusive file lock }
    O_ASYNC     =   $40;        { Signal pgrp when data ready }
    O_FSYNC     =   $80;        { Synchronous writes }
    O_SYNC      =   $80;        { POSIX synonym for O_FSYNC }
    O_NOFOLLOW  =  $100;        { Don't follow symlinks }
    O_DIRECT    =$10000;        { Attempt to bypass buffer cache }
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


  { For File control mechanism }
  F_DupFd          = 0;           { duplicate file descriptor }
  F_GetFd          = 1;           { get file descriptor flags }
  F_SetFd          = 2;           { set file descriptor flags }
  F_GetFl          = 3;           { get file status flags }
  F_SetFl          = 4;           { set file status flags }
  F_GetOwn         = 5;           { get SIGIO/SIGURG proc/pgrp }
  F_SetOwn         = 6;           { set SIGIO/SIGURG proc/pgrp }
{$ifdef freebsd}
  F_OGetLk         = 7;           { get record locking information }
  F_OSetLk         = 8;           { set record locking information }
  F_OSetLkW        = 9;           { F_SETLK; wait if blocked }
  F_Dup2Fd         = 10;          { duplicate file descriptor to arg }
  F_GetLk          = 11;          { get record locking information}
  F_SetLk          = 12;          { set record locking information }
  F_SetLkW         = 13;          { F_SETLK; wait if blocked }
  F_SetLkRemote    = 14;          { debugging support for remote locks }
{$endif}
{$ifdef dragonfly}
  F_GetLk          = 7;           { get record locking information}
  F_SetLk          = 8;           { set record locking information }
  F_SetLkW         = 9;           { F_SETLK; wait if blocked }
  F_Dup2Fd         = 10;          { duplicate file descriptor to arg }
  F_DupFd_CloExec  = 17;	  { close on exec duplicated fd }
{$endif}
{$ifdef netbsd}
  F_GetLk          = 7;           { get record locking information}
  F_SetLk          = 8;           { set record locking information }
  F_SetLkW         = 9;           { F_SETLK; wait if blocked }
  F_CloseM	   = 10;	  { close all fds >= to the one given }
  F_MaxFd          = 11;	  { return the max open fd }
  F_DupFd_CloExec  = 12;	  { close on exec duplicated fd }
{$endif}
{$ifdef openbsd}
  F_GetLk          = 7;           { get record locking information}
  F_SetLk          = 8;           { set record locking information }
  F_SetLkW         = 9;           { F_SETLK; wait if blocked }
  F_DupFd_CloExec  = 10;	  { duplicate with FD_CLOEXEC set }
{$endif}
{$ifdef darwin}
  F_GetLk          = 7;           { get record locking information}
  F_SetLk          = 8;           { set record locking information }
  F_SetLkW         = 9;           { F_SETLK; wait if blocked }
{$endif}

  { File descriptor flags (F_GETFD, F_SETFD) }
  FD_CLOEXEC	   = 1;		  { close-on-exec flag }

  { Record locking flags (F_GETLK, F_SETLK, F_SETLKW) }
  F_RDLCK          = 1;		  { shared or read lock }
  F_UNLCK          = 2;		  { unlock }
  F_WRLCK          = 3;		  { exclusive or write lock }
{$ifdef freebsd}
  F_UNLCKSYS       = 4;		  { purge locks for a given system ID }
  F_CANCEL         = 5;		  { cancel an async lock request }
{$endif}
{$ifndef darwin}
  F_WAIT	   = $10;	  { Wait until lock is granted }
  F_FLOCK          = $20;         { Use flock(2) semantics for lock }
  F_POSIX          = $40; 	  { Use POSIX semantics for lock }
{$endif}
{$ifdef freebsd}
  F_REMOTE	   = $80;	  { Lock owner is remote NFS client }
  F_NOINTR	   = $100;        { Ignore signals when waiting }
{$endif}

{$ifdef darwin}
  F_CHKCLEAN       = 41;          { Used for regression test }
  F_PREALLOCATE    = 42;          { Preallocate storage }
  F_SETSIZE        = 43;          { Truncate a file without zeroing space }
  F_RDADVISE       = 44;          { Issue an advisory read async with no copy to user }
  F_RDAHEAD        = 45;          { turn read ahead off/on }
  F_READBOOTSTRAP  = 46;          { Read bootstrap from disk }
  F_WRITEBOOTSTRAP = 47;          { Write bootstrap on disk }
  F_NOCACHE        = 48;          { turning data caching off/on }
  F_LOG2PHYS       = 49;          { file offset to device offset }
  F_GETPATH        = 50;          { return the full path of the fd }
  F_FULLFSYNC      = 51;          { fsync + ask the drive to flush to the media }
  F_PATHPKG_CHECK  = 52;          { find which component (if any) is a package }
  F_FREEZE_FS      = 53;          { "freeze" all fs operations }
  F_THAW_FS        = 54;          { "thaw" all fs operations }
  F_GLOBAL_NOCACHE = 55;          { turn data caching off/on (globally) for this file }
{$endif}


type
  timezone = record
    tz_minuteswest,
    tz_dsttime  : cint;
  end;
  ptimezone =^timezone;
  TTimeZone = timezone;

  rusage = record
        ru_utime    : timeval;          { user time used }
        ru_stime    : timeval;          { system time used }
        ru_maxrss   : clong;            { max resident set size }
        ru_ixrss    : clong;            { integral shared memory size }
        ru_idrss    : clong;            { integral unshared data " }
        ru_isrss    : clong;            { integral unshared stack " }
        ru_minflt   : clong;            { page reclaims }
        ru_majflt   : clong;            { page faults }
        ru_nswap    : clong;            { swaps }
        ru_inblock  : clong;            { block input operations }
        ru_oublock  : clong;            { block output operations }
        ru_msgsnd   : clong;            { messages sent }
        ru_msgrcv   : clong;            { messages received }
        ru_nsignals : clong;            { signals received }
        ru_nvcsw    : clong;            { voluntary context switches }
        ru_nivcsw   : clong;            { involuntary " }
        end;
// #define      ru_last         ru_nivcsw
// #define      ru_first        ru_ixrss


Const
  S_IFMT  = &170000;
  S_IFIFO =  &10000;
  S_IFCHR =  &20000;
  S_IFDIR =  &40000;
  S_IFBLK =  &60000;
  S_IFREG = &100000;
  S_IFLNK = &120000;
  S_IFSOCK= &140000;
  S_IFWHT = &160000;
  S_ISVTX =   &1000;

{
 * Resource limits from FreeBSD5. To be checked for the others.
}
  RLIMIT_CPU      = 0;              { cpu time in milliseconds }
  RLIMIT_FSIZE    = 1;              { maximum file size }
  RLIMIT_DATA     = 2;              { data size }
  RLIMIT_STACK    = 3;              { stack size }
  RLIMIT_CORE     = 4;              { core file size }
  RLIMIT_RSS      = 5;              { resident set size }
  RLIMIT_MEMLOCK  = 6;              { locked-in-memory address space }
  RLIMIT_NPROC    = 7;              { number of processes }
  RLIMIT_NOFILE   = 8;              { number of open files }
{$if defined(freebsd) or defined(dragonfly)}
  RLIMIT_SBSIZE   = 9;              { maximum size of all socket buffers }
  RLIMIT_VMEM     =10;              { virtual process size (inclusive of mmap) }
  RLIMIT_AS       = RLIMIT_VMEM;
{$ELSE}
  RLIMIT_AS       = 5;    	    { address space= resident set size}
{$ENDIF}

  {$ifdef FreeBSD}
  RLIM_NLIMITS    =11;              { number of resource limits }
  {$endif}
  {$ifdef dragonfly}
  RLIM_NLIMITS    =12;              { number of resource limits }
  {$endif}

  {$ifdef Darwin}   // OS X 10.3
  RLIM_NLIMITS    =9;              { number of resource limits }
  {$endif}

Type
        TRLimit  = record
                     rlim_cur,               { current (soft) limit }
          	     rlim_max : TRLim;     { maximum value for rlim_cur }
		    end;	
        PRLimit  = ^TRLimit;

  iovec = record
            iov_base : pointer;
	    iov_len  : size_t;
	   end;
  tiovec=iovec;
  piovec=^tiovec;		

CONST
 { Constansts for MMAP }
{$ifdef FPC_IS_SYSTEM}
  MAP_PRIVATE   =2;
{$endif}
  MAP_ANONYMOUS =$1000;

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
  POLLWRNORM  = POLLOUT;
  POLLWRBAND  = $0100;

type
  pollfd = record
    fd: cint;
    events: cshort;
    revents: cshort;
  end;
  tpollfd = pollfd;
  ppollfd = ^pollfd;

    {*************************************************************************}
    {                               SIGNALS                                   }
    {*************************************************************************}

{$i signal.inc}


