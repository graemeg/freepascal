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

const 
    SEM_SAFE=255;
type

    dev_t    = cuint32;         { used for device numbers      }
    TDev     = dev_t;
    pDev     = ^dev_t;

    gid_t    = cuint32;         { used for group IDs           }
    TGid     = gid_t;
    pGid     = ^gid_t;
    TIOCtlRequest = cuLong;

    {$ifdef CPU64}
     ino_t    = cuint32;           { used for file serial numbers }
    {$else}	
     ino_t    = clong;           { used for file serial numbers }
    {$endif}
    TIno     = ino_t;
    pIno     = ^ino_t;

    mode_t   = cuint16;         { used for file attributes     }
    TMode    = mode_t;
    pMode    = ^mode_t;

    nlink_t  = cuint16;         { used for link counts         }
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


{$ifdef CPU64}
    clock_t  = cuint32;         // 32-bit even on Athlon64
{$else}
    clock_t  = culong;
{$endif}
    TClock   = clock_t;
    pClock   = ^clock_t;

    time_t   = clong;           { used for returning the time, clong
                                        is 64-bit on AMD64}

    // TTime    = time_t;    // Not allowed in system unit, -> unixtype

    pTime    = ^time_t;
    ptime_t =  ^time_t;

    socklen_t= cuint32;
    TSocklen = socklen_t;
    pSocklen = ^socklen_t;

  timeval  = packed record
    tv_sec,tv_usec:clong;
  end;
  ptimeval = ^timeval;
  TTimeVal = timeval;

  timespec = packed record
    tv_sec   : time_t;                  // should be time_t, bug compability
    tv_nsec  : clong;
  end;
  ptimespec= ^timespec;
  Ttimespec= timespec;

   pthread_t_rec        = record end;
   pthread_attr_t_rec   = record end;
//   pthread_mutex_t_rec   = record end;
   pthread_mutexattr_t_rec   = record end;
   pthread_cond_t_rec   = record end;
   pthread_condattr_t_rec   = record end;
   pthread_rwlock_t_rec   = record end;
   pthread_rwlockattr_t_rec   = record end;

   pthread_t            = ^pthread_t_rec;
   pthread_attr_t       = ^pthread_attr_t_rec;
   pthread_mutex_t      = {$i pmutext.inc}
   pthread_mutexattr_t  = ^pthread_mutexattr_t_rec;
   pthread_cond_t       = ^pthread_cond_t_rec;
   pthread_condattr_t   = ^pthread_condattr_t_rec;
   pthread_key_t        = cint;
   pthread_rwlock_t     = ^pthread_rwlock_t_rec;
   pthread_rwlockattr_t = ^pthread_rwlockattr_t_rec;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  
   psem_t = ^sem_t;
   ppsem_t= ^psem_t;
   semid_t= pointer;
   sem_t = record
       magic   : cuint32;
       lock    : pthread_mutex_t;
       gtzero  : pthread_cond_t;
       count   : cuint32;
       nwaiters: cuint32;
       semid   : semid_t;
       sysse   : cint;
       entry   : psem_t;
       backpointer : ppsem_t;
       spare   : array[0..SEM_SAFE] of char;

    end;

=======

   sem_t_rec		= record end;
   sem_t                = ^sem_t_rec;
>>>>>>> graemeg/cpstrnew
=======

   sem_t_rec		= record end;
   sem_t                = ^sem_t_rec;
>>>>>>> graemeg/cpstrnew
=======

   sem_t_rec		= record end;
   sem_t                = ^sem_t_rec;
>>>>>>> graemeg/cpstrnew
=======

   sem_t_rec		= record end;
   sem_t                = ^sem_t_rec;
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
=======

Const
     MNAMLEN   = 88-2*sizeof(clong); // slightly machine specific.
     MFSNamLen = 16;	
>>>>>>> graemeg/fixes_2_2
type
  fsid_t  = array[0..1] of cint32;

Const
     MNAMELEN   = 88;
     MFSNamELen = 16;	
     STATFS_VERSION = $20030518;
     OMNAMLEN   = 88-2*sizeof(clong); // slightly machine specific.
     OMFSNamLen = 16;	

// fieldnames start with f_ in headers. Probably 1.0.x simplification that somehow
// managed to escape the unix rewrite. Make an union+ deprecate after newstatfs is stable.

Type TStatFS =  {?} Record
       case boolean of
       0 : ( // current FreeBSD
        version     : cuint32;        { structure version number }
        ftype       : cuint32;        { type of filesystem }
        fflags      : cuint64;        { copy of mount exported flags }
        bsize       : cuint64;        { filesystem fragment size }
        iosize      : cuint64;        { optimal transfer block size }
        blocks      : cuint64;        { total data blocks in filesystem }
        bfree       : cuint64;        { free blocks in filesystem }
        bavail      : cint64;         { free blocks avail to non-superuser }
        files       : cuint64;        { total file nodes in filesystem }
        ffree       : cint64;         { free nodes avail to non-superuser }
        fsyncwrites  : cuint64;        { count of sync writes since mount }
        fasyncwrites : cuint64;        { count of async writes since mount }
        fsyncreads   : cuint64;        { count of sync reads since mount }
        fasyncreads  : cuint64;        { count of async reads since mount }
        spare       : array[0..9] of cuint64;  { unused spare }
        namemax     : cuint32;        { maximum filename length }
        owner       : tuid;           { user that mounted the filesystem }
        fsid        : fsid_t;                            { filesystem id }
        charspare   : array[0..80-1] of ansichar;         { spare string space }
        fstypename  : array[0..MFSNAMELEN-1] of ansichar; { filesystem type name }
        mnfromname  : array[0..MNAMELEN-1]   of ansichar; { mounted filesystem }
        mountpoint  : array[0..MNAMELEN-1]   of ansichar; { directory on which mounted }
        );
        1:( // union for old fieldname's sake.
        f_version     : cuint32;        { structure version number }
        f_type        : cuint32;        { type of filesystem }
        f_flags       : cuint64;        { copy of mount exported flags }
        f_bsize       : cuint64;        { filesystem fragment size }
        f_iosize      : cuint64;        { optimal transfer block size }
        f_blocks      : cuint64;        { total data blocks in filesystem }
        f_bfree       : cuint64;        { free blocks in filesystem }
        f_bavail      : cint64;         { free blocks avail to non-superuser }
        f_files       : cuint64;        { total file nodes in filesystem }
        f_ffree       : cint64;         { free nodes avail to non-superuser }
        f_syncwrites  : cuint64;        { count of sync writes since mount }
        f_asyncwrites : cuint64;        { count of async writes since mount }
        f_syncreads   : cuint64;        { count of sync reads since mount }
        f_asyncreads  : cuint64;        { count of async reads since mount }
        f_spare       : array[0..9] of cuint64;  { unused spare }
        f_namemax     : cuint32;        { maximum filename length }
        f_owner       : tuid;           { user that mounted the filesystem }
        f_fsid        : fsid_t;                            { filesystem id }
        f_charspare   : array[0..80-1] of ansichar;         { spare string space }
        f_fstypename  : array[0..MFSNAMELEN-1] of ansichar; { filesystem type name }
        f_mntfromname : array[0..MNAMELEN-1]   of ansichar; { mounted filesystem }
        f_mntonname   : array[0..MNAMELEN-1]   of ansichar; { directory on which mounted }
          );
	end;
 
     PStatFS=^TStatFS;


   TStatFS4 = packed record
    spare2,            { place holder}
    bsize,             { fundamental block size}
    iosize,            { optimal block size }
    blocks,            {  total blocks}
    bfree,             { blocks free}
    bavail,            { block available for mortal users}
    files,             { Total file nodes}
    ffree          : clong ;    { file nodes free}
    fsid           : fsid_t;
    fowner         : tuid; {mounter uid}
    ftype          : cint;
    fflags         : cint; {copy of mount flags}
    fsyncwrites,
    fasyncwrites   : clong;
    fstypename     : array[0..OMFSNamLen-1] of char;
    mountpoint     : array[0..OMNAMLEN-1] of char;
    fsyncreads,            { count of sync reads since mount }
    fasyncreads    : clong;
    fspares1       : cshort;
    mnfromname     : array[0..OMNAMLEN-1] of char;
    fspares2       : cshort;
    fspare3        : array[0..1] of clong;
  end;
 
  PStatFS4=^TStatFs4;

  mbstate_t = record
      case byte of
        0: (__mbstate8: array[0..127] of char);
        1: (_mbstateL: cint64); { for alignment }
    end;
    pmbstate_t = ^mbstate_t;

    mbstate_t = record
      case byte of
        0: (__mbstate8: array[0..127] of char);
        1: (_mbstateL: cint64); { for alignment }
    end;
    pmbstate_t = ^mbstate_t;

    mbstate_t = record
      case byte of
        0: (__mbstate8: array[0..127] of char);
        1: (_mbstateL: cint64); { for alignment }
    end;
    pmbstate_t = ^mbstate_t;

  ITimerVal= Record
              It_Interval,
              It_Value      : TimeVal;
             end;

const
  _PTHREAD_MUTEX_DEFAULT     = _PTHREAD_MUTEX_ERRORCHECK;
  _MUTEX_TYPE_FAST          = _PTHREAD_MUTEX_NORMAL;
  _MUTEX_TYPE_COUNTING_FAST = _PTHREAD_MUTEX_RECURSIVE;

  _PTHREAD_KEYS_MAX              = 256;
  _PTHREAD_STACK_MIN             = 1024;

   { System limits, POSIX value in parentheses, used for buffer and stack allocation }
{$ifdef CPU64}
    ARG_MAX  = 262144;   {4096}  { Maximum number of argument size     }
{$else}
    ARG_MAX  = 65536;   {4096}  { Maximum number of argument size     }
{$endif}

    NAME_MAX = 255;     {14}    { Maximum number of bytes in filename }
    PATH_MAX = 1024;    {255}   { Maximum number of bytes in pathname }
    SYS_NMLN = 32;              {BSD utsname struct limit, kernel mode}

    SIG_MAXSIG      = 128;      // highest signal version
//  wordsinsigset   = 4;               // words in sigset_t


  { For getting/setting priority }
   Prio_Process = 0;
   Prio_PGrp    = 1;
   Prio_User    = 2;

