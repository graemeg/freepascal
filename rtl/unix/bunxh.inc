{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 by Marco van de Voort

    The interface part of the baseunix unit for syscall based rtls.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

Type TGrpArr = Array [0..0] of TGid;            { C style array workarounds}
     pGrpArr = ^TGrpArr;
     TFilDes = Array [0..1] of cInt;
     pFilDes = ^TFilDes;

// if you are looking for macro definitions or non C template overloaded versions, they are moved to bunxovlh.inc

    Function  FpSigProcMask  (how : cInt; nset : pSigSet; oset : pSigSet): cInt; external name 'FPC_SYSC_SIGPROCMASK';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Function  FpSigProcMask  (how : cInt; constref nset : TSigSet; var oset : TSigSet): cInt; external name 'FPC_SYSC_SIGPROCMASK';
    Function  FpSigPending   (var nset : TSigSet): cInt;
    Function  FpSigSuspend   (Const sigmask : TSigSet): cInt;
{$ifndef FPC_SYS_SIGTIMEDWAIT_UNAVAILABLE}
    Function  FpSigTimedWait (Const sigset : TSigSet; info : Psiginfo; timeout:Ptimespec): cInt;
{$endif ndef FPC_SYS_SIGTIMEDWAIT_UNAVAILABLE}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Function  FpSigProcMask  (how : cInt; Const nset : TSigSet; var oset : TSigSet): cInt; external name 'FPC_SYSC_SIGPROCMASK';
    Function  FpSigPending   (var nset : TSigSet): cInt;
    Function  FpSigSuspend   (Const sigmask : TSigSet): cInt;
    Function  FpSigTimedWait (Const sigset : TSigSet; info : Psiginfo; timeout:Ptimespec): cInt;
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

    Function  FpUmask      (cmask : TMode): TMode;
    Function  FpLink       (existing : pChar; newone : pChar): cInt;
    Function  FpMkfifo     (path : pChar; Mode : TMode): cInt;
    Function  FpChmod      (path : pChar; Mode : TMode): cInt;
    Function  FpChown      (path : pChar; owner : TUid; group : TGid): cInt;
    Function  FpUtime      (path : pChar; times : putimbuf): cInt;
    Function  FpPipe       (var fildes : tfildes):cInt;
    Function  FpDup        (fildes : cInt): cInt;  external name 'FPC_SYSC_DUP';
    Function  FpDup2       (fildes, fildes2 : cInt): cInt; external name 'FPC_SYSC_DUP2';
    Function  FpTimes      (var buffer : tms): TClock;

    Function  FpAlarm      (seconds : cuint): cuint;
    Function  FpPause : cInt;
    Function  FpSleep      (seconds : cuint): cuint;

    Function  FpGetpid  : TPid; external name  'FPC_SYSC_GETPID';
    Function  FpGetppid : TPid;
    Function  FpGetuid  : TUid;
    Function  FpGeteuid : TUid;
    Function  FpGetgid  : TGid;
    Function  FpGetegid : TGid;
    Function  FpSetuid     (uid : TUid): cInt;
    Function  FpSetgid     (gid : TGid): cInt;
    Function  FpGetgroups (gidsetsize : cInt; var grouplist : tgrparr): cInt;
    Function  FpGetpgrp : TPid;
    Function  FpSetsid  : TPid;
    Function  FpGetsid (pid:TPid)  : TPid;
    Function  FpFcntl      (fildes : cInt; cmd : cInt): cInt;
    Function  FpFcntl      (fildes : cInt; cmd : cInt; arg : cInt): cInt;
    Function  FpFcntl      (fildes : cInt; cmd : cInt; var arg : flock): cInt;

    Function  FpGetcwd     (path:pChar; siz:TSize):pChar;  external name 'FPC_SYSC_GETCWD';
    Function  FpFork : TPid; external name 'FPC_SYSC_FORK';
    Function  FpExecve     (path : pChar; argv : ppChar; envp: ppChar): cInt;
    Function  FpExecv      (path : pChar; argv : ppChar): cInt;
    Function  FpWaitpid    (pid : TPid; stat_loc : pcInt; options: cInt): TPid; external name 'FPC_SYSC_WAITPID';
    Function  FpWait       (var stat_loc : cInt): TPid;
    Procedure FpExit       (Status : cInt);  external name 'FPC_SYSC_EXIT';
    Function  FpKill       (pid : TPid; sig: cInt): cInt;
    Function  FpUname      (var name: utsname): cInt;
    Function  FpOpendir    (dirname : pChar): pDir;  external name 'FPC_SYSC_OPENDIR';
    Function  FpReaddir    (var dirp : Dir) : pDirent; external name 'FPC_SYSC_READDIR';
    Function  FpClosedir   (var dirp : Dir): cInt; external name 'FPC_SYSC_CLOSEDIR';
    Function  FpChdir      (path : pChar): cInt;  external name 'FPC_SYSC_CHDIR';
    Function  FpOpen       (path : pChar; flags : cInt; Mode: TMode):cInt; external name 'FPC_SYSC_OPEN';
    Function  FpMkdir      (path : pChar; Mode: TMode):cInt;  external name 'FPC_SYSC_MKDIR';
    Function  FpUnlink     (path : pChar): cInt;  external name 'FPC_SYSC_UNLINK';
    Function  FpRmdir      (path : pChar): cInt; external name 'FPC_SYSC_RMDIR';
    Function  FpRename     (old  : pChar; newpath: pChar): cInt;   external name 'FPC_SYSC_RENAME';
    Function  FpFStat      (fd : cInt; var sb : stat): cInt; external name 'FPC_SYSC_FSTAT';
    Function  FpStat       (path: pChar; var buf : stat): cInt;  external name 'FPC_SYSC_STAT';
    Function  FpAccess     (pathname : pChar; aMode : cInt): cInt; external name 'FPC_SYSC_ACCESS';
    Function  FpClose      (fd : cInt): cInt;  external name 'FPC_SYSC_CLOSE';

    Function  FpRead       (fd : cInt; buf: pChar; nbytes : TSize): TSsize; external name 'FPC_SYSC_READ';
    Function  FpPRead      (fd : cInt; buf: pChar; nbytes : TSize; offset:Toff): TSsize;
    function  FpReadV	   (fd: cint; const iov : piovec; iovcnt : cint):TSSize;
    Function  FpWrite      (fd : cInt; buf:pChar; nbytes : TSize): TSsize;  external name 'FPC_SYSC_WRITE';
    Function  FpPWrite     (fd : cInt; buf:pChar; nbytes : TSize; offset:Toff): TSSize;
    function  FpWriteV	   (fd: cint; const iov : piovec; iovcnt : cint):TSSize;

    Function  FpLseek      (fd : cInt; offset : TOff; whence : cInt): TOff; external name 'FPC_SYSC_LSEEK';
    Function  FpTime       (var tloc : TTime): TTime; external name 'FPC_SYSC_TIME';
    Function  FpFtruncate  (fd : cInt; flength : TOff): cInt;  external name 'FPC_SYSC_FTRUNCATE';
    Function  FPSigaction  (sig: cInt; act : pSigActionRec; oact : pSigActionRec): cint;  external name 'FPC_SYSC_SIGACTION';
    Function  FPSelect     (N:cint;readfds,writefds,exceptfds:pfdSet;TimeOut:PTimeVal):cint;
<<<<<<< HEAD
<<<<<<< HEAD
    Function  FpPoll       (fds: ppollfd; nfds: cuint; timeout: clong): cint;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Function  FpIOCtl      (Handle:cint;Ndx: TIOCtlRequest; Data: Pointer):cint; external name  'FPC_SYSC_IOCTL';
    Function  FpNanoSleep  (req : ptimespec;rem : ptimespec):cint; external name 'FPC_SYSC_NANOSLEEP';
    Function  fpLstat(path:pchar;Info:pstat):cint;
    Function  fpSymlink(oldname,newname:pchar):cint;
    Function  fpReadLink(name,linkname:pchar;maxlen:size_t):cint; external name  'FPC_SYSC_READLINK';

    function  fpNice(N:cint):cint;
    Function  fpGetPriority(Which,Who:cint):cint;
    Function  fpSetPriority(Which,Who,What:cint):cint;
    Function  Fpmmap(start:pointer;len:size_t;prot:cint;flags:cint;fd:cint;offst:off_t):pointer; external name 'FPC_SYSC_MMAP';
    Function  Fpmunmap(start:pointer;len:size_t):cint;  external name 'FPC_SYSC_MUNMAP';

    Function  FpGetEnv (name : pChar): pChar; external name 'FPC_SYSC_FPGETENVPCHAR';
    function  fpsettimeofday(tp:ptimeval;tzp:ptimezone):cint;

    function FpGetRLimit(resource:cint;rlim:PRLimit):cint; external name 'FPC_SYSC_GETRLIMIT';
    function FpSetRLimit(Resource:cint;rlim:PRLimit):cint; external name 'FPC_SYSC_SETRLIMIT';

