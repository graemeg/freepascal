{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by the Free Pascal development team

    This file should become an alternative to the syscalls in due time,
    to import the base calls from libc.
    Be very careful though. Kernel types and libc types are often not the
    same on Linux.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ***********************************************************************}

{   var
     Errno : cint; external clib name 'errno';}

{$if defined(aix) and not defined(FPC_IS_SYSTEM)}
{ in AIX, several UNIX routines are part of the "compatibility interface for
  older Berkeley Software Distribution (BSD) system programs" }
{$linklib libbsd}
{$endif aix}

Type TGrpArr = Array [0..0] of TGid;            { C style array workarounds}
     pGrpArr = ^TGrpArr;
     TFilDes = Array [0..1] of cInt;
     pFilDes = ^TFilDes;

const
{$if (defined(linux) and defined(cpu32) and not defined(fs32bit)) or defined(aix)}
    suffix64bit = '64';
{$else}    
    suffix64bit = '';
{$endif}    

<<<<<<< HEAD
    function  FPSigProcMask(how:cint;nset : psigset;oset : psigset):cint;cdecl; external clib name 'sigprocmask';
    function  FPSigProcMask(how:cint;const nset : sigset;var oset : sigset):cint;cdecl; external clib name 'sigprocmask';
    Function  FpSigPending   (var nset : sigset): cInt; external clib name 'sigpending';
    Function  FpSigSuspend   (constref sigmask : TSigSet): cInt; external clib name 'sigsuspend';
{$if not defined(darwin) and not defined(haiku)}
    Function  FpSigTimedWait (Constref sigset : TSigSet; info : Psiginfo; timeout:Ptimespec): cInt; cdecl; external clib name 'sigtimedwait';
=======
    function  FpAccess  (pathname : pchar; amode : cint): cint; cdecl; external clib name 'access';
    function  FpChdir   (path : pchar): cint; cdecl; external clib name 'chdir';
    Function  FpChown   (path : pChar; owner : TUid; group : TGid): cInt; cdecl; external clib name 'chown';
    function  FpClose   (fd : cint): cint; cdecl; external clib name 'close';
    Function  FpClosedir (var dirp : Dir): cInt; cdecl; external clib name 'closedir';
    function  FpClosedir (dirp : pdir): cint; cdecl; external clib name 'closedir';
    function  FpDup     (oldd:cint):cint; cdecl; external clib name 'dup';
    function  FpDup2    (oldd:cint;newd:cint):cint; cdecl; external clib name 'dup2';
    function  FpExecve  (path : pchar; argv : ppchar; envp: ppchar): cint; cdecl; external clib name 'execve';
    function  FpFork  : TPid; cdecl; external clib name 'fork';
    function  FpvFork  : TPid; cdecl; external clib name 'vfork';
    function  FpFtruncate(fd : cint; flength : TOff): cint; cdecl; external clib name 'ftruncate'+suffix64bit;
    function  FpLseek   (fd : cint; offset : TOff; whence : cint): TOff; cdecl; external clib name 'lseek'+suffix64bit;
    function  FpMkdir   (path : pchar; mode: TMode):cint; cdecl; external clib name 'mkdir';
    function  FpOpen    (path: pchar; flags : cint; mode: TMode):cint;
{$if defined(linux) and not defined(fs32bit)}
    { we need a wrapper for linux to automatically pass O_LARGEFILE with flags }
                                              {$ifdef FPC_IS_SYSTEM}forward;{$endif} inline;
    function  __FpOpen  (path: pchar; flags : cint; mode: TMode):cint; 
>>>>>>> graemeg/fixes_2_2
{$endif}
    Function  FpUmask       (cmask : TMode): TMode; cdecl; external clib name 'umask';
    Function  FpLink       (existing : pChar; newone : pChar): cInt; cdecl; external clib name 'link';
    Function  FpMkfifo     (path: pchar; mode: tmode): cint; cdecl; external clib name 'mkfifo';
    Function  FpChmod      (path : pChar; Mode : TMode): cInt; cdecl; external clib name 'chmod';
    Function  FpChown   (path : pChar; owner : TUid; group : TGid): cInt; cdecl; external clib name 'chown';
    Function  FPUtime(path:pchar;times:putimbuf):cint; cdecl; external clib name 'utime';
<<<<<<< HEAD
    Function  FpPipe       (var fildes : tfildes):cInt; cdecl;external clib name 'pipe';
    function  FpDup     (oldd:cint):cint; cdecl; external clib name 'dup';
    function  FpDup2    (oldd:cint;newd:cint):cint; cdecl; external clib name 'dup2';
=======
    Function  FpGetcwd  (path:pChar; siz:TSize):pChar; cdecl;  external clib name 'getcwd';
    function  FPSigProcMask(how:cint;nset : psigset;oset : psigset):cint;cdecl; external clib name 'sigprocmask';
    function  FPSigProcMask(how:cint;const nset : sigset;var oset : sigset):cint;cdecl; external clib name 'sigprocmask';
{$if not defined(darwin) and not defined(haiku)}
    Function  FpSigTimedWait (Const sigset : TSigSet; info : Psiginfo; timeout:Ptimespec): cInt; cdecl; external clib name 'sigtimedwait';
    Function  FpSigTimedWait (Const sigset : TSigSet; info : Psiginfo; var timeout:Ttimespec): cInt; cdecl; external clib name 'sigtimedwait';
{$endif}
    function  FpTime       (tloc:ptime_t): time_t; cdecl; external clib name 'time';
    Function  FpTime       (var tloc : Time_t): Time_t; cdecl; external clib name 'time';
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
    function  FpTimes	(var buffer : tms): TClock; cdecl; external clib name 'times';
<<<<<<< HEAD

    Function  FpAlarm      (seconds : cuint) : cuint; cdecl;external clib name 'alarm';
    Function  FpPause   : cInt; cdecl; external clib name 'pause';
    Function  FpSleep      (seconds : cuint): cuint; external clib name 'sleep';

=======
    function  FpUname   (var name: utsname): cint; cdecl; external clib name 'uname';
    function  FpUnlink  (path: pchar): cint; cdecl; external clib name 'unlink';
    function  FpWaitpid (pid : TPid; stat_loc : pcint; options: cint): TPid; cdecl; external clib name 'waitpid';
    function  FpWrite   (fd: cint;buf:pchar; nbytes : TSize): TSSize; cdecl; external clib name 'write';
    procedure FpExit    (status : cint); cdecl; external clib name '_exit';
    function  fpmmap    (addr:pointer;len:size_t;prot:cint;flags:cint;fd:cint;ofs:off_t):pointer; cdecl; external clib name 'mmap'+suffix64bit;
    function  fpmunmap  (addr:pointer;len:size_t):cint; cdecl; external clib name 'munmap';
{$if defined(bsd) or defined(solaris)}
    { The BSD/Solaris version has "..." as third parameter -> wrap for }
    { interface compatibility with Linux                               }
    function  FpIOCtl   (Handle:cint;Ndx: TIOCtlRequest;Data: Pointer):cint;
                                         {$ifdef FPC_IS_SYSTEM}forward;{$endif}
{$else bsd or solaris}
    function  FpIOCtl   (Handle:cint;Ndx: TIOCtlRequest;Data: Pointer):cint; cdecl; external clib name 'ioctl';
{$endif bsd or solaris}
{$ifdef beos}
    Function  FPSelect  (N:cint;readfds,writefds,exceptfds:pfdSet;TimeOut:PTimeVal):cint; cdecl; external 'net' name 'select';
{$else}
    Function  FPSelect  (N:cint;readfds,writefds,exceptfds:pfdSet;TimeOut:PTimeVal):cint; cdecl; external clib name 'select';
{$endif}
    function  fpgetenv  (name : pchar):pchar; cdecl; external clib name 'getenv';
    Function  FpKill       (pid : TPid; sig: cInt): cInt; cdecl; external clib name 'kill';
>>>>>>> graemeg/fixes_2_2
    Function  FpGetpid  : TPid; cdecl;external clib name 'getpid';
    Function  FpGetppid : TPid;cdecl;external clib name 'getppid';
    Function  FpGetuid  : TUid;cdecl;external clib name 'getuid';
    Function  FpGeteuid : TUid;cdecl;external clib name 'geteuid';
    Function  FpGetgid  : TGid;cdecl;external clib name 'getgid';
    Function  FpGetegid : TGid;cdecl;external clib name 'getegid';
    Function  FpSetuid     (uid : TUid): cInt;cdecl;external clib name 'setuid';
    Function  FpSetgid     (gid : TGid): cInt;cdecl;external clib name 'setgid';
    Function  FpGetgroups  (gidsetsize : cInt; var grouplist : tgrparr):cint; cdecl;external clib name 'getgroups';
    Function  FpGetpgrp : TPid;  cdecl;external clib name 'getpgrp';
    Function  FpSetsid  : TPid; cdecl;external clib name 'setsid';
<<<<<<< HEAD
    Function  FpGetsid (pid:TPid)  : TPid; cdecl; external clib name 'getsid';
=======
    Function  FpPipe       (var fildes : tfildes):cInt; cdecl;external clib name 'pipe';
    Function  FpFcntl      (fildes : cInt; cmd : cInt): cInt; cdecl; external clib name 'fcntl';
    Function  FpFcntl      (fildes : cInt; cmd : cInt; arg :cInt): cInt; cdecl; external clib name 'fcntl';
    Function  FpFcntl      (fildes : cInt; cmd : cInt; var arg : flock): cInt; cdecl external clib name 'fcntl';
    Function  FpPause   : cInt; cdecl external clib name 'pause';
{$ifdef solaris}   
{$linklib md5} 
{$linklib aio}
    Function  FPnanosleep  (const rqtp: ptimespec; rmtp: ptimespec): cint; cdecl; external 'rt' name 'nanosleep';
{$else solaris}
{$ifndef beos}
    Function  FPnanosleep  (const rqtp: ptimespec; rmtp: ptimespec): cint; cdecl; external clib name 'nanosleep';
{$else}
{$endif}
{$endif solaris}
    function  fpNice       (N:cint):cint; cdecl; external clib name 'nice';
    Function  fpGetPriority (Which,Who:cint):cint;      cdecl; external clib name 'getpriority';
    Function  fpSetPriority (Which,Who,What:cint):cint; cdecl; external clib name 'setpriority';
    Function  fpSymlink    (oldname,newname:pchar):cint;   cdecl; external clib name 'symlink';
    Function  fpReadLink           (name,linkname:pchar;maxlen:size_t):cint;  cdecl; external clib name 'readlink';
    Function  FpUmask       (cmask : TMode): TMode; cdecl; external clib name 'umask';
{$ifndef beos}    
    function  fpsettimeofday(tp:ptimeval;tzp:ptimezone):cint; cdecl; external clib name 'settimeofday';
{$else}
//    function  fpsettimeofday(tp:ptimeval;tzp:ptimezone):cint;
{$endif}
    function FpGetRLimit(resource : cInt; rlim : PRLimit) : cInt; cdecl; external clib name 'getrlimit';
    {$ifdef HAS_UGETRLIMIT}
    { there is no ugetrlimit libc call, just map it to the getrlimit call in these cases }
    function FpUGetRLimit(resource : cInt; rlim : PRLimit) : cInt; cdecl; external clib name 'getrlimit';
    {$endif}
>>>>>>> graemeg/fixes_2_2

    { The libc version has "..." as third parameter -> wrap for }
    { interface compatibility with syscalls                     }
    Function  FpFcntl      (fildes : cInt; cmd : cInt): cInt; {$ifdef FPC_IS_SYSTEM}forward;{$endif}
    Function  FpFcntl      (fildes : cInt; cmd : cInt; arg :cInt): cInt; {$ifdef FPC_IS_SYSTEM}forward;{$endif}
    Function  FpFcntl      (fildes : cInt; cmd : cInt; var arg : flock): cInt; {$ifdef FPC_IS_SYSTEM}forward;{$endif}

    Function  FpGetcwd  (path:pChar; siz:TSize):pChar; cdecl;  external clib name 'getcwd';
    function  FpFork  : TPid; cdecl; external clib name 'fork';
    function  FpExecve  (path : pchar; argv : ppchar; envp: ppchar): cint; cdecl; external clib name 'execve';
    function  FpExecv  (path : pchar; argv : ppchar): cint; cdecl; external clib name 'execv';
    function  FpWaitpid (pid : TPid; stat_loc : pcint; options: cint): TPid; cdecl; external clib name 'waitpid';
    Function  FpWait       (var stat_loc : cInt): TPid; cdecl; external clib name 'waitpid';
    procedure FpExit    (status : cint); cdecl; external clib name '_exit';
    Function  FpKill       (pid : TPid; sig: cInt): cInt; cdecl; external clib name 'kill';
    function  FpUname   (var name: utsname): cint; cdecl; external clib name 'uname';
    function  FpOpendir (dirname : pchar): pdir; cdecl; external clib name 'opendir'{$ifdef aix}+suffix64bit{$endif};
    Function  FpReaddir (var dirp : Dir) : pDirent;cdecl; external clib name 'readdir'+suffix64bit;
    Function  FpClosedir (var dirp : Dir): cInt; cdecl; external clib name 'closedir'{$ifdef aix}+suffix64bit{$endif};
    function  FpChdir   (path : pchar): cint; cdecl; external clib name 'chdir';
    { emulate the bunxovl version that automatically passes
     638 as mode }
    function  FpOpen    (path: pchar; flags : cint; mode: TMode):cint; {$ifdef FPC_IS_SYSTEM}forward;{$endif}
    function  FpMkdir   (path : pchar; mode: TMode):cint; cdecl; external clib name 'mkdir';
    function  FpUnlink  (path: pchar): cint; cdecl; external clib name 'unlink';
    function  FpRmdir   (path : pchar): cint; cdecl; external clib name 'rmdir';
    function  FpRename  (old : pchar; newpath: pchar): cint; cdecl;external clib name 'rename';
{$ifdef linux}
{$ifndef FPC_IS_SYSTEM}
    { those functions are macros on linux }
    function  FpFstat   (fd : cint; var sb : stat): cint;inline;
    Function  fpLstat       (path:pchar;Info:pstat):cint;inline;
    function  FpStat      (path: pchar; var buf : stat): cint;inline;
{$endif FPC_IS_SYSTEM}
{$else linux}
    function  FpFstat   (fd : cint; var sb : stat): cint; cdecl; external clib name 'fstat'+suffix64bit;
    Function  fpLstat   (path:pchar;Info:pstat):cint;     cdecl; external clib name 'lstat'+suffix64bit;
    function  FpStat    (path: pchar; var buf : stat): cint; cdecl; external clib name 'stat'+suffix64bit;
{$endif linux}
    function  FpAccess  (pathname : pchar; amode : cint): cint; cdecl; external clib name 'access';
    function  FpClose   (fd : cint): cint; cdecl; external clib name 'close';

    function  FpRead    (fd: cint; buf: pchar; nbytes : TSize): TSSize; cdecl; external clib name 'read';
    Function  FpPRead      (fd : cInt; buf: pChar; nbytes : TSize; offset:Toff): TSsize;   cdecl; external clib name 'pread'+suffix64bit;
    function  FpReadV	   (fd: cint; const iov : piovec; iovcnt : cint):TSSize;  cdecl; external clib name 'readv';
    function  FpWrite   (fd: cint;buf:pchar; nbytes : TSize): TSSize; cdecl; external clib name 'write';
    Function  FpPWrite     (fd : cInt; buf:pChar; nbytes : TSize; offset:Toff): TSSize;  cdecl; external clib name 'pwrite'+suffix64bit;
    function  FpWriteV	   (fd: cint; const iov : piovec; iovcnt : cint):TSSize;  cdecl; external clib name 'writev';

<<<<<<< HEAD
    function  FpLseek   (fd : cint; offset : TOff; whence : cint): TOff; cdecl; external clib name 'lseek'+suffix64bit;
    Function  FpTime       (var tloc : Time_t): Time_t; cdecl; external clib name 'time';
    function  FpFtruncate(fd : cint; flength : TOff): cint; cdecl; external clib name 'ftruncate'+suffix64bit;
    Function  FPSigaction (sig: cInt; act :pSigActionRec;oact:pSigActionRec):cint;cdecl; external clib name 'sigaction';
{$ifdef beos}
  {$ifdef haiku}
    Function  FPSelect  (N:cint;readfds,writefds,exceptfds:pfdSet;TimeOut:PTimeVal):cint; cdecl; external 'network' name 'select';  
    Function  FpPoll    (fds: ppollfd; nfds: cuint; timeout: clong): cint; cdecl; external clib name 'poll';
  {$else}
    Function  FPSelect  (N:cint;readfds,writefds,exceptfds:pfdSet;TimeOut:PTimeVal):cint; cdecl; external 'net' name 'select';
    Function  FpPoll    (fds: ppollfd; nfds: cuint; timeout: clong): cint; cdecl; external clib name 'poll';
  {$endif}  
{$else}
    Function  FPSelect  (N:cint;readfds,writefds,exceptfds:pfdSet;TimeOut:PTimeVal):cint; cdecl; external clib name 'select';
    Function  FpPoll    (fds: ppollfd; nfds: cuint; timeout: clong): cint; cdecl; external clib name 'poll';
{$endif}
    { The libc version has "..." as third parameter -> wrap for }
    { interface compatibility with syscalls                     }
    function  FpIOCtl   (Handle:cint;Ndx: TIOCtlRequest;Data: Pointer):cint; {$ifdef FPC_IS_SYSTEM}forward;{$endif}
{$ifdef solaris}   
{$linklib md5} 
{$linklib aio}
    Function  FPnanosleep  (const rqtp: ptimespec; rmtp: ptimespec): cint; cdecl; external 'rt' name 'nanosleep';
{$else solaris}
{$ifndef beos}
    Function  FPnanosleep  (const rqtp: ptimespec; rmtp: ptimespec): cint; cdecl; external clib name 'nanosleep';
{$else}
=======
{$ifdef linux}
    function  FpPrCtl(options : cInt; const args : ptruint) : cint; cdecl; external clib name 'prctl';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
{$endif}
{$endif solaris}
    Function  fpSymlink    (oldname,newname:pchar):cint;   cdecl; external clib name 'symlink';
    Function  fpReadLink           (name,linkname:pchar;maxlen:size_t):cint;  cdecl; external clib name 'readlink';

    function  fpNice       (N:cint):cint; cdecl; external clib name 'nice';
    Function  fpGetPriority (Which,Who:cint):cint;      cdecl; external clib name 'getpriority';
    Function  fpSetPriority (Which,Who,What:cint):cint; cdecl; external clib name 'setpriority';
    function  fpmmap    (addr:pointer;len:size_t;prot:cint;flags:cint;fd:cint;ofs:off_t):pointer; cdecl; external clib name 'mmap'+suffix64bit;
    function  fpmunmap  (addr:pointer;len:size_t):cint; cdecl; external clib name 'munmap';

    function  fpgetenv  (name : pchar):pchar; cdecl; external clib name 'getenv';
{$ifndef beos}    
    function  fpsettimeofday(tp:ptimeval;tzp:ptimezone):cint; cdecl; external clib name 'settimeofday';
{$else}
//    function  fpsettimeofday(tp:ptimeval;tzp:ptimezone):cint;
{$endif}

    function FpGetRLimit(resource : cInt; rlim : PRLimit) : cInt; cdecl; external clib name 'getrlimit';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function FpSetRLimit(resource : cInt; rlim : PRLimit) : cInt; cdecl; external clib name 'setrlimit';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    { not supported with syscalls }
    function  FpvFork  : TPid; cdecl; external clib name 'vfork';

    { overloads not available in bunxh.inc }
    function  FpReaddir (dirp : pdir) : pdirent;cdecl; external clib name 'readdir'+suffix64bit;
    function  FpClosedir (dirp : pdir): cint; cdecl; external clib name 'closedir'{$ifdef aix}+suffix64bit{$endif};
    function  FpSigaction (sig: cint;var act : sigactionrec; var oact : sigactionrec): cint; cdecl; external clib name 'sigaction';
{$if not defined(darwin) and not defined(haiku)}
    Function  FpSigTimedWait (Const sigset : TSigSet; info : Psiginfo; var timeout:Ttimespec): cInt; cdecl; external clib name 'sigtimedwait';
=======
>>>>>>> origin/fixes_2_2
{$endif}
    function  FpTime       (tloc:ptime_t): time_t; cdecl; external clib name 'time';


