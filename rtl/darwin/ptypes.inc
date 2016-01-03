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
{$packrecords c}

type
    { the following type definitions are compiler dependant }
    { and system dependant                                  }

    dev_t    = cuint32;         { used for device numbers      }
    TDev     = dev_t;
    pDev     = ^dev_t;

    gid_t    = cuint32;         { used for group IDs           }
    TGid     = gid_t;
    pGid     = ^gid_t;
    TIOCtlRequest = cuLong;

{$if not defined(cpuarm) and not defined(aarch64) and not defined(iphonesim)}
    ino_t    = cuint32;         { used for file serial numbers }
{$else}
    ino_t    = cuint64;
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

    size_t   = culong;          { as definied in the C standard}
    TSize    = size_t;
    pSize    = ^size_t;
    psize_t  = ^size_t;

    ssize_t  = clong;           { used by function for returning number of bytes }
    TsSize   = ssize_t;
    psSize   = ^ssize_t;

    uid_t    = cuint32;         { used for user ID type        }
    TUid     = Uid_t;
    pUid     = ^Uid_t;

    clock_t  = culong;
    TClock   = clock_t;
    pClock   = ^clock_t;

    time_t   = clong;           { used for returning the time  }
    // TTime    = time_t;    // Not allowed in system unit, -> unixtype

    pTime    = ^time_t;
    ptime_t  = ^time_t;

    wchar_t  = cint32;
    pwchar_t = ^wchar_t;
    wint_t   = cint32;

    socklen_t= cuint32;
    TSocklen = socklen_t;
    pSocklen = ^socklen_t;

    suseconds_t = cint32;

    timeval = record
        tv_sec: time_t;
        tv_usec: suseconds_t;
    end;
    ptimeval = ^timeval;
    TTimeVal = timeval;

    timespec = record
      tv_sec   : time_t;
      tv_nsec  : clong;
    end;
    ptimespec= ^timespec;
    Ttimespec= timespec;

    rlim_t		= int64;
    TRlim		= rlim_t;

CONST
   { System limits, POSIX value in parentheses, used for buffer and stack allocation }
    ARG_MAX  = 65536;   {4096}  { Maximum number of argument size     }
    NAME_MAX = 255;     {14}    { Maximum number of bytes in filename }
    PATH_MAX = 1024;    {255}   { Maximum number of bytes in pathname }

    SYS_NMLN = 256;              {BSD utsname struct limit}

    SIG_MAXSIG      = 32;       // highest signal version

    MFSNAMELEN = 15;
    MNAMELEN   = 90;


type
  pthread_mutextype = (
    _PTHREAD_MUTEX_NORMAL    := 0,
    _PTHREAD_MUTEX_ERRORCHECK := 1,
    _PTHREAD_MUTEX_RECURSIVE := 2
  );


  const
     _PTHREAD_MUTEX_DEFAULT    = _PTHREAD_MUTEX_NORMAL;
     _MUTEX_TYPE_FAST          = _PTHREAD_MUTEX_NORMAL;
     _MUTEX_TYPE_COUNTING_FAST = _PTHREAD_MUTEX_RECURSIVE;

     _PTHREAD_KEYS_MAX              = 128;
     _PTHREAD_STACK_MIN             = 8192;

  type
     fsid_t = record
          val: array[0..1] of cint32;
     end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(cpuarm) or defined(cpuaarch64) or defined(iphonesim)}
=======
{$ifdef cpuarm}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef cpuarm}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef cpuarm}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef cpuarm}
>>>>>>> origin/cpstrnew
     { structure used on iPhoneOS and available on Mac OS X 10.6 and later }
     tstatfs = record
          bsize : cuint32;
          iosize : cint32;
          blocks : cuint64;
          bfree : cuint64;
          bavail : cuint64;
          files : cuint64;
          ffree : cuint64;
          fsid : fsid_t;
          owner : uid_t;
          ftype : cuint32;
          fflags : cuint32;
          fssubtype : cuint32;
          fstypename : array[0..(MFSNAMELEN)-1] of char;
          mountpoint : array[0..(PATH_MAX)-1] of char;
          mntfromname : array[0..(PATH_MAX)-1] of char;
          reserved: array[0..7] of cuint32;
     end;
{$else}
     tstatfs = record
          otype : cint16;
          oflags : cint16;
          bsize : clong;
          iosize : clong;
          blocks : clong;
          bfree : clong;
          bavail : clong;
          files : clong;
          ffree : clong;
          fsid : fsid_t;
          fowner : uid_t;
          reserved1 : cint16;
          ftype : cint16;
          fflags : clong;
          reserved2 : array[0..1] of clong;
          fstypename : array[0..(MFSNAMELEN)-1] of char;
          mountpoint : array[0..(MNAMELEN)-1] of char;
          mntfromname : array[0..(MNAMELEN)-1] of char;
          f_reserved3: char;
          reserved4: array[0..3] of clong;
       end;
{$endif}
    pstatfs = ^tstatfs;

    mbstate_t = record
      case byte of
        0: (__mbstate8: array[0..127] of char);
        1: (_mbstateL: clonglong); { for alignment }
    end;
    pmbstate_t = ^mbstate_t;

   pthread_t            = pointer;
   pthread_attr_t       = record sig: clong; opaque: array[0..{$ifdef cpu64}56{$else}36{$endif}-1] of byte; end;
   pthread_mutex_t      = {$i pmutext.inc}
   pthread_mutexattr_t  = record sig: clong; opaque: array[0..8-1] of byte; end;
   pthread_cond_t       = record sig: clong; opaque: array[0..{$ifdef cpu64}40{$else}24{$endif}-1] of byte; end;
   pthread_condattr_t   = record sig: clong; opaque: array[0..{$ifdef cpu64}8{$else}4{$endif}-1] of byte; end;
   pthread_key_t        = culong;
   pthread_rwlock_t     = record sig: clong; opaque: array[0..{$ifdef cpu64}192{$else}124{$endif}-1] of byte; end;
   pthread_rwlockattr_t = record sig: clong; opaque: array[0..{$ifdef cpu64}16{$else}12{$endif}-1] of byte; end;

   sem_t                = cint;

// for get/setpriority
Const
 { For getting/setting priority }
  Prio_Process = 0;
  Prio_PGrp    = 1;
  Prio_User    = 2;

