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

{$i ctypes.inc}

type
  fsblkcnt_t = clonglong;
  TStatfs = packed record
    bsize 			: Cardinal;
    frsize			: Cardinal;
    blocks			: fsblkcnt_t;
    bfree			: fsblkcnt_t;
    bavail			: fsblkcnt_t;
    files			: fsblkcnt_t;
    ffree			: fsblkcnt_t;
    favail			: fsblkcnt_t;
    fsid			: Cardinal;
    flag			: Cardinal;
    namemax			: Cardinal;
  end;
  PStatFS=^TStatFS;

    dev_t    = cuint32;         { used for device numbers      }
    TDev     = dev_t;
    pDev     = ^dev_t;

    gid_t    = cuint32;         { used for group IDs           }
    TGid     = gid_t;
    pGid     = ^gid_t;
    TIOCtlRequest = cuLong;

    ino_t    = clonglong;           { used for file serial numbers }
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

    wint_t	 = cint32;
    size_t   = cuint32;         { as definied in the C standard}
    TSize    = size_t;
    pSize    = ^size_t;
    psize_t   = pSize;		

    ssize_t  = cint32;          { used by function for returning number of bytes }
    TsSize   = ssize_t;
    psSize   = ^ssize_t;		

    uid_t    = cuint32;         { used for user ID type        }
    TUid     = Uid_t;
    pUid     = ^Uid_t;

    clock_t  = culong;
    TClock   = clock_t;
    pClock   = ^clock_t;

    time_t   = clong;           { used for returning the time  }
<<<<<<< HEAD
<<<<<<< HEAD

    // TTime    = time_t;    // Not allowed in system unit, -> unixtype
=======
    TTime    = time_t; 
>>>>>>> graemeg/fixes_2_2
=======
    TTime    = time_t; 
>>>>>>> origin/fixes_2_2
    pTime    = ^time_t;
    ptime_t =  ^time_t;
    
    wchar_t   = cint32;
    pwchar_t  = ^wchar_t;

    socklen_t= cuint32;
    TSocklen = socklen_t;
    pSocklen = ^socklen_t;

  timeval  = packed record
    tv_sec,tv_usec:clong;
  end;
  ptimeval = ^timeval;
  TTimeVal = timeval;

  timespec = packed record
    tv_sec   : time_t;
    tv_nsec  : clong;
  end;
  ptimespec= ^timespec;
  Ttimespec= timespec;
  
  pthread_t = culong;
  
  sched_param = record
    __sched_priority: cint;
  end;

  pthread_attr_t = record
    __detachstate: cint;
    __schedpolicy: cint;
    __schedparam: sched_param;
    __inheritsched: cint;
    __scope: cint;
    __guardsize: size_t;
    __stackaddr_set: cint;
    __stackaddr: pointer;
    __stacksize: size_t;
  end;

  _pthread_fastlock = record
    __status: clong;
    __spinlock: cint;
  end;

  pthread_mutex_t = record
    __m_reserved: cint;
    __m_count: cint;
    __m_owner: pointer;
    __m_kind:  cint;
    __m_lock: _pthread_fastlock;
  end;

  pthread_mutexattr_t = record
    __mutexkind: cint;
  end;

  pthread_cond_t = record
    __c_lock: _pthread_fastlock;
    __c_waiting: pointer;
    __padding: array[0..48-1-sizeof(_pthread_fastlock)-sizeof(pointer)-sizeof(clonglong)] of byte;
    __align: clonglong;
  end;
    
  pthread_condattr_t = record
    __dummy: cint;
  end;

  pthread_key_t = cuint;

  pthread_rwlock_t = record
    __rw_readers: cint;
    __rw_writer: pointer;
    __rw_read_waiting: pointer;
    __rw_write_waiting: pointer;
    __rw_kind: cint;
    __rw_pshared: cint;
  end;

  pthread_rwlockattr_t = record
    __lockkind: cint;
    __pshared: cint;
  end;
  
  sem_t = record
     __sem_lock: _pthread_fastlock;
     __sem_value: cint;
     __sem_waiting: pointer;
  end;

   rlim_t		= int64;
   TRlim		= rlim_t;


CONST
    _PTHREAD_MUTEX_TIMED_NP      = 0;
    _PTHREAD_MUTEX_RECURSIVE_NP  = 1;
    _PTHREAD_MUTEX_ERRORCHECK_NP = 2;
    _PTHREAD_MUTEX_ADAPTIVE_NP   = 3;
  
    _PTHREAD_MUTEX_NORMAL     = _PTHREAD_MUTEX_TIMED_NP;
    _PTHREAD_MUTEX_RECURSIVE  = _PTHREAD_MUTEX_RECURSIVE_NP;
    _PTHREAD_MUTEX_ERRORCHECK = _PTHREAD_MUTEX_ERRORCHECK_NP;
    _PTHREAD_MUTEX_DEFAULT    = _PTHREAD_MUTEX_NORMAL;
    _PTHREAD_MUTEX_FAST_NP    = _PTHREAD_MUTEX_ADAPTIVE_NP;

     _PTHREAD_KEYS_MAX              = 256;
     _PTHREAD_STACK_MIN             = 1024;

CONST
   { System limits, POSIX value in parentheses, used for buffer and stack allocation }
    ARG_MAX  = 65536;   {4096}  { Maximum number of argument size     }
    NAME_MAX = 255;     {14}    { Maximum number of bytes in filename }
    PATH_MAX = 1024;    {255}   { Maximum number of bytes in pathname }

    SYS_NMLN = 32;              {BSD utsname struct limit}
    
    SIG_MAXSIG = 32; //128;	// highest signal version  // BeOS  

const
  { For getting/setting priority }
  Prio_Process = 0;
  Prio_PGrp    = 1;
  Prio_User    = 2;
