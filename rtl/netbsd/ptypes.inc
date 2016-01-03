{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    This file implements all the base types and limits required
    for a minimal POSIX compliant subset required to port the compiler
    to a new OS.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{***********************************************************************}
{                       POSIX TYPE DEFINITIONS                          }
{***********************************************************************}

{$I ctypes.inc}
{$packrecords c}

type

    dev_t    = cuint32;         { used for device numbers      }
    TDev     = dev_t;
    pDev     = ^dev_t;

    gid_t    = cuint32;         { used for group IDs           }
    TGid     = gid_t;
    pGid     = ^gid_t;
    TIOCtlRequest = cuLong;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NETBSD_USE_STAT30}
    ino_t    = cint64;          { used for file serial numbers }
{$else}
    ino_t = cuint32;
{$endif}
=======
=======
>>>>>>> origin/fixes_2_2

    ino_t    = clong;           { used for file serial numbers }
>>>>>>> graemeg/fixes_2_2
    TIno     = ino_t;
    pIno     = ^ino_t;

    mode_t   = cuint32;         { used for file attributes     }
    TMode    = mode_t;
    pMode    = ^mode_t;

    nlink_t  = cuint32;         { used for link counts         }
    TnLink   = nlink_t;
    pnLink   = ^nlink_t;

    off_t    = cint64;          { used for file sizes          }
    TOff     = off_t;
    pOff     = ^off_t;

    pid_t    = cint32;          { used as process identifier   }
    TPid     = pid_t;
    pPid     = ^pid_t;

 {$ifdef CPU64}
    size_t   = cuint64;
{$else}
    size_t   = cuint32;         { as definied in the C standard}
{$endif}
    TSize    = size_t;
    pSize    = ^size_t;
    pSize_t  = ^size_t;

{$ifdef CPU64}
    ssize_t  = cint64;          { used by function for returning number of bytes }
{$else}
    ssize_t  = cint32;          { used by function for returning number of bytes}
{$endif}

    TsSize   = ssize_t;
    psSize   = ^ssize_t;

    uid_t    = cuint32;         { used for user ID type        }
    TUid     = Uid_t;
    pUid     = ^Uid_t;

    wint_t    = cint32;
    wchar_t   = cint32;
    pwchar_t  = ^wchar_t;
    
    clock_t  = culong;
    TClock   = clock_t;
    pClock   = ^clock_t;

    time_t   = clong;           { used for returning the time  }
    // TTime    = time_t;    // Not allowed in system unit, -> unixtype
    pTime    = ^time_t;
    ptime_t  = ^time_t;

    socklen_t= cuint32;
    TSocklen = socklen_t;
    pSocklen = ^socklen_t;

Const
    MNAMELEN   = 90;      // found in sys/mount.h
    MNAMLEN = MNAMELEN; // Keep all constant name.
    MFSNAMELEN = 16;// found in sys/compat/sys/mount.h	
    MFSNamLen = MFSNAMELEN;	
type
{ Checked, found in sys/fstypes.h }
    fsid_t  = array[0..1] of cint32;
    // Kernel statfs
{ Checked, in sys/compat/sys/mount.h header }
(*
struct statfs12 {
	short	f_type;			/* type of file system */
	u_short	f_oflags;		/* deprecated copy of mount flags */
	long	f_bsize;		/* fundamental file system block size */
	long	f_iosize;		/* optimal transfer block size */
	long	f_blocks;		/* total data blocks in file system */
	long	f_bfree;		/* free blocks in fs */
	long	f_bavail;		/* free blocks avail to non-superuser */
	long	f_files;		/* total file nodes in file system */
	long	f_ffree;		/* free file nodes in fs */
	fsid_t	f_fsid;			/* file system id */
	uid_t	f_owner;		/* user that mounted the file system */
	long	f_flags;		/* copy of mount flags */
	long	f_syncwrites;		/* count of sync writes since mount */
	long	f_asyncwrites;		/* count of async writes since mount */
	long	f_spare[1];		/* spare for later */
	char	f_fstypename[MFSNAMELEN]; /* fs type name */
	char	f_mntonname[MNAMELEN];	  /* directory on which mounted */
	char	f_mntfromname[MNAMELEN];  /* mounted file system */
};
*)
    TStatfs12 = record
      f_type : shortint;
      f_oflags : word;
      bsize,             { fundamental block size}
      iosize,            { optimal block size }
      blocks,            {  total blocks}
      bfree,             { blocks free}
      bavail,            { block available for mortal users}
      files,             { Total file nodes}
      ffree          : clong ;    { file nodes free}
      fsid           : fsid_t;
      fowner         : tuid; {mounter uid}
      fflags         : clong; {copy of mount flags}
      fsyncwrites,
      fasyncwrites   : clong;
      fspare0        : array [0..0] of clong;
      fstypename     : array[0..MFSNamLen-1] of char;
      mountpoint     : array[0..MNAMLEN-1] of char;
      mnfromname     : array[0..MNAMLEN-1] of char;
    end;
    TStatFS = TStatFS12;   
    PStatFS=^TStatFS;
    timeval = packed record
                tv_sec,
                tv_usec : clong;
                end;
    ptimeval= ^timeval;
    TTimeval= timeval;

    timespec = packed record
      tv_sec   : time_t;
      tv_nsec  : clong;
    end;
  ptimespec= ^timespec;
  Ttimespec= timespec;

   mbstate_t = record
      case byte of
        0: (__mbstate8: array[0..127] of char);
        1: (_mbstateL: cint64); { for alignment }
    end;
   pmbstate_t = ^mbstate_t;
 
{ records transcripted fromm NetBSD 5.1 libpthread sources } 
   pthread_t            = pointer;
   pthread_attr_t       = record
     pta_magic : cuint;
     pta_flags : cint;
     pta_private : pointer;
   end;
   pthread_mutex_t      = {$i pmutext.inc}
   pthread_mutexattr_t = record
     ptma_magic : cint;
     ptma_private : pointer;
   end;
   pthread_spin_t = char;
   pthread_queue_t = record
       first, last : pointer;
     end;
   pthread_cond_t       = record
       ptc_magic : cuint;
       ptc_lock : pthread_spin_t;
       ptc_waiters : pthread_queue_t;
       ptc_mutex : ^pthread_mutex_t;
       ptc_private : pointer;
     end;
   pthread_condattr_t   = record
       ptca_magic : cuint;
       ptca_private : pointer;
     end;
   pthread_key_t        = cint;
   pthread_rwlock_t     = pointer;
   pthread_rwlockattr_t = pointer;

   sem_t                = pointer;
   rlim_t		= int64;
   TRlim		= rlim_t;

  {
     Mutex types (Single UNIX Specification, Version 2, 1997).

     Note that a mutex attribute with one of the following types:

     PTHREAD_MUTEX_NORMAL
     PTHREAD_MUTEX_RECURSIVE
     MUTEX_TYPE_FAST (deprecated)
     MUTEX_TYPE_COUNTING_FAST (deprecated)

       will deviate from POSIX specified semantics.
      }

  pthread_mutextype = (
    { Default POSIX mutex  }
    _PTHREAD_MUTEX_ERRORCHECK := 1,
    { Recursive mutex  }
    _PTHREAD_MUTEX_RECURSIVE := 2,
    { No error checking  }
    _PTHREAD_MUTEX_NORMAL    := 3,
    _MUTEX_TYPE_MAX
  );


const
  _PTHREAD_MUTEX_DEFAULT     = _PTHREAD_MUTEX_ERRORCHECK;
  _MUTEX_TYPE_FAST          = _PTHREAD_MUTEX_NORMAL;
  _MUTEX_TYPE_COUNTING_FAST = _PTHREAD_MUTEX_RECURSIVE;

  _PTHREAD_KEYS_MAX              = 256;
  _PTHREAD_STACK_MIN             = 1024;

   { System limits, POSIX value in parentheses, used for buffer and stack allocation }
    ARG_MAX  = 256*1024;   {4096}  { Maximum number of argument size     }
    NAME_MAX = 255;        {14}    { Maximum number of bytes in filename }
    PATH_MAX = 1024;       {255}   { Maximum number of bytes in pathname }

    SYS_NMLN = 32;              {BSD utsname struct limit}

    SIG_MAXSIG      = 128;      // highest signal version

    { For getting/setting priority }
    Prio_Process = 0;
    Prio_PGrp    = 1;
    Prio_User    = 2;
