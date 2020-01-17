{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 by Marco van de Voort

    Calls needed for the baseunix unit, but not for system.
    Some calls that can be used for both Linux and *BSD will be
    moved to a /unix/ includedfile later.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

Function fpKill(Pid:pid_t;Sig:cint):cint;
{
  Send signal 'sig' to a process, or a group of processes.
  If Pid >  0 then the signal is sent to pid
     pid=-1                         to all processes except process 1
     pid < -1                         to process group -pid
  Return value is zero, except for case three, where the return value
  is the number of processes to which the signal was sent.
}

begin
 fpkill:=do_syscall(syscall_nr_kill,TSysParam(pid),TSysParam(sig));
// if kill<0 THEN
//  Kill:=0;
end;

Function fpSigPending(var nset: TSigSet):cint;
{
  Allows examination of pending signals. The signal mask of pending
  signals is set in SSet
}
begin
  fpsigpending:=do_syscall(syscall_nr_rt_sigpending,TSysParam(@nset));
end;

function fpsigsuspend(const sigmask:TSigSet):cint;
{
 Set the signal mask with Mask, and suspend the program until a signal
 is received.
}

begin
  fpsigsuspend:= do_syscall(syscall_nr_rt_sigsuspend,TSysParam(@sigmask),TSysParam(8));
end;

function fpsigtimedwait(const sigset:TSigSet;info:Psiginfo;timeout:Ptimespec):cint;

begin
  {Sizeof(Tsigset)=16 for Free Pascal, but the Linux kernel has a different idea,
   it wants a Tsigset of 8 bytes. So we have to hardcode :( }
  FpSigTimedWait:=do_syscall(syscall_nr_rt_sigtimedwait,
                             Tsysparam(@sigset),
                             Tsysparam(info),
                             Tsysparam(timeout),
                             Tsysparam(8 {sizeof(Tsigset)}));
end;


Type
  ITimerVal= Record
              It_Interval,
              It_Value      : TimeVal;
             end;

Const   ITimer_Real    =0;
        ITimer_Virtual =1;
        ITimer_Prof    =2;

Function SetITimer(Which : Longint;Const value : ItimerVal; var VarOValue:ItimerVal):Longint;

Begin
  SetItimer:=Do_Syscall(syscall_nr_setitimer,Which,TSysParam(@Value),TSysParam(@varovalue));
End;

Function GetITimer(Which : Longint;Var value : ItimerVal):Longint;

Begin
  GetItimer:=Do_Syscall(syscall_nr_getItimer,Which,TSysParam(@value));
End;

Function fpalarm(Seconds: cuint):cuint;

Var it,oitv : Itimerval;
    retval  : cuint;

Begin
//      register struct itimerval *itp = &it;

 it.it_interval.tv_sec:=0;
 it.it_interval.tv_usec:=0;
 it.it_value.tv_usec:=0;
 it.it_value.tv_sec:=seconds;
 If SetITimer(ITIMER_REAL,it,oitv)<0 Then
   Exit(0);                     // different from *BSD!

 retval:= oitv.it_value.tv_usec;
 if retval<>0 Then
   inc(retval);
 fpAlarm:=retval;
End;

// The following versions are for internal use _ONLY_
// This because it works for the first 32 signals _ONLY_, but that
// is enough since they are depreciated, and for legacy applications
// anyway.

function sigblock(mask:cuint):cint;

var nset,oset: TSigSet;

begin
 fpsigemptyset(nset);
 // fpsigaddset(nset,mask);   needs _mask_
 nset[0]:=mask;
 sigblock:= fpsigprocmask(SIG_BLOCK,@nset,@oset);   // SIG_BLOCK=1
 if sigblock=0 Then
  sigblock:=oset[0];
end;

function sigpause(sigmask:cint):cint;

var nset: TSigSet;

begin
 fpsigemptyset(nset);
 nset[0]:=sigmask;
 sigpause:= fpsigsuspend(nset);
end;

function fppause:cint;

begin
  fppause:=sigpause(sigblock(cuint(0)));
end;

function fpsleep(seconds:cuint):cuint;
{see comments in libc}

var time_to_sleep,time_remaining : timespec;
    nset,oset  : TSigSet;
    oerrno     : cint;
    oact       : sigactionrec;

begin
        time_to_sleep.tv_sec := seconds;
        time_to_sleep.tv_nsec := 0;
         fpsigemptyset(nset);
         fpsigaddset  (nset,SIGCHLD);
         if fpsigprocmask(SIG_BLOCK,@nset,@oset)=-1 Then
          exit(cuint(-1));
        if fpsigismember(oset,SIGCHLD)<>0 Then
          Begin
            fpsigemptyset(nset);
            fpsigaddset  (nset,SIGCHLD);
            if fpsigaction(SIGCHLD,NIL,@oact)<0 Then
              begin
                oerrno:=fpgeterrno;
                fpsigprocmask(SIG_SETMASK,@oset,NIL);
                fpseterrno(oerrno);
                exit(cuint(-1));
              End;
            if oact.sa_handler=SigActionhandler(SIG_IGN) Then
             Begin
               fpsleep:=fpnanosleep(@time_to_sleep, @time_remaining);
               oerrno:=fpgeterrno;
               fpsigprocmask(SIG_SETMASK,@oset,NIL);
               fpseterrno(oerrno);
             End
            Else
             Begin
               fpsigprocmask(SIG_SETMASK,@oset,NIL);
               fpsleep:=fpnanosleep(@time_to_sleep, @time_remaining)
             End;
          end
        else
            fpsleep:=fpnanosleep(@time_to_sleep, @time_remaining);
        if fpsleep<>0 Then
         if time_remaining.tv_nsec>=500000000 Then
          inc(fpsleep);
End;

function fpuname(var name:utsname):cint; [public,alias:'FPC_SYSC_UNAME'];

begin
  fpuname:=Do_Syscall(syscall_nr_uname,TSysParam(@name));
end;

Function fpGetDomainName(Name:PChar; NameLen:size_t):cint;

Var
        srec  : utsname;
        tsize : size_t;
Begin
        if fpuname(srec)<0 Then
          exit(-1);
        tsize:=strlen(@srec.domain[0]);
        if tsize>(namelen-1) Then
         tsize:=namelen-1;
        move(srec.domain[0],name[0],tsize);
        name[namelen-1]:=#0;
        fpgetDomainName:=0;
End;

function fpGetHostName(Name:PChar; NameLen:size_t):cint;

Var
        srec  : utsname;
        tsize : size_t;
begin
        if fpuname(srec)<0 Then
          exit(-1);
        tsize:=strlen(@srec.nodename[0]);
        if tsize>(namelen-1) Then
         tsize:=namelen-1;
        move(srec.nodename[0],name[0],tsize);
        name[namelen-1]:=#0;
        fpgethostName:=0;
End;

const WAIT_ANY = -1;

function fpwait(var stat_loc:cint): pid_t;
{
  Waits until a child with PID Pid exits, or returns if it is exited already.
  Any resources used by the child are freed.
  The exit status is reported in the adress referred to by Status. It should
  be a longint.
}

begin // actually a wait4() call with 4th arg 0.
 fpWait:=do_syscall(syscall_nr_Wait4,WAIT_ANY,TSysParam(@Stat_loc),0,0);
end;

//function fpgetpid : pid_t;

// begin
//  fpgetpid:=do_syscall(syscall_nr_getpid);
// end;

function fpgetppid : pid_t;

begin
 fpgetppid:=do_syscall(syscall_nr_getppid);
end;

function fpgetuid : uid_t;

begin
 fpgetuid:=do_syscall(syscall_nr_getuid);
end;

function fpgeteuid : uid_t;

begin
 fpgeteuid:=do_syscall(syscall_nr_geteuid);
end;

function fpgetgid : gid_t;

begin
 fpgetgid:=do_syscall(syscall_nr_getgid);
end;

function fpgetegid : gid_t;

begin
 fpgetegid:=do_syscall(syscall_nr_getegid);
end;

function fpsetuid(uid : uid_t): cint;

begin
 fpsetuid:=do_syscall(syscall_nr_setuid,uid);
end;

function fpsetgid(gid : gid_t): cint;

begin
 fpsetgid:=do_syscall(syscall_nr_setgid,gid);
end;

// type tgrparr=array[0..0] of gid_t;

function fpgetgroups(gidsetsize : cint; var grouplist:tgrparr): cint;

begin
 fpgetgroups:=do_syscall(syscall_nr_getgroups,gidsetsize,TSysParam(@grouplist));
end;

function fpgetpgrp : pid_t;

begin
{$if defined(generic_linux_syscalls)}
 fpgetpgrp:=do_syscall(syscall_nr_getpgid,0);
{$else}
 fpgetpgrp:=do_syscall(syscall_nr_getpgrp);
{$endif}
end;

function fpsetsid : pid_t;

begin
 fpsetsid:=do_syscall(syscall_nr_setsid);
end;

function fpgetsid (pid:TPid): pid_t;

begin
 fpgetsid:=do_syscall(syscall_nr_getsid,pid);
end;

Function fpumask(cmask:mode_t):mode_t;
{
  Sets file creation mask to (Mask and 0777 (octal) ), and returns the
  previous value.
}
begin
 fpumask:=Do_syscall(syscall_nr_umask,cmask);
end;

Function fplink(existing:pchar;newone:pchar):cint;
{
  Proceduces a hard link from new to old.
  In effect, new will be the same file as old.
}
begin
{$if defined(generic_linux_syscalls)}
  fpLink:=Do_Syscall(syscall_nr_linkat,AT_FDCWD,TSysParam(existing),AT_FDCWD,TSysParam(newone),0);
{$else}
  fpLink:=Do_Syscall(syscall_nr_link,TSysParam(existing),TSysParam(newone));
{$endif}
end;

Function fpmkfifo(path:pchar;mode:mode_t):cint;

begin

{$if defined(generic_linux_syscalls)}
fpmkfifo:=do_syscall(syscall_nr_mknodat,AT_FDCWD,TSysParam(path),TSysParam(mode or S_IFIFO),TSysParam(0));
{$else}
fpmkfifo:=do_syscall(syscall_nr_mknod,TSysParam(path),TSysParam(mode or S_IFIFO),TSysParam(0));
{$endif}
end;

Function fpchmod(path:pchar;mode:mode_t):cint;

begin
{$if defined(generic_linux_syscalls)}
  fpchmod:=do_syscall(syscall_nr_fchmodat,AT_FDCWD,TSysParam(path),TSysParam(mode),0);
{$else}
  fpchmod:=do_syscall(syscall_nr_chmod,TSysParam(path),TSysParam(mode));
{$endif}
end;

Function fpchown(path:pchar;owner:uid_t;group:gid_t):cint;

begin
{$if defined(generic_linux_syscalls)}
  fpChOwn:=do_syscall(syscall_nr_fchownat,AT_FDCWD,TSysParam(path),TSysParam(owner),TSysParam(group),0);
{$else}
  fpChOwn:=do_syscall(syscall_nr_chown,TSysParam(path),TSysParam(owner),TSysParam(group));
{$endif}
end;

{$if defined(generic_linux_syscalls)}

Function fpUtime(path:pchar;times:putimbuf):cint;
var
  tsa: Array[0..1] of timespec;
begin
  tsa[0].tv_sec := times^.actime;
  tsa[0].tv_nsec := 0;
  tsa[1].tv_sec := times^.modtime;
  tsa[1].tv_nsec := 0;
  fputime:=do_syscall(syscall_nr_utimensat,AT_FDCWD,TSysParam(path),
                      TSysParam(@tsa),0);
end;

{$elseif not defined(NO_SYSCALL_UTIME)}

Function fpUtime(path:pchar;times:putimbuf):cint;

begin
  fputime:=do_syscall(syscall_nr_utime,TSysParam(path),TSysParam(times));
end;

{$else}

Function fpUtime(path:pchar;times:putimbuf):cint;
var
  tva: Array[0..1] of timeval;
begin
  tva[0].tv_sec := times^.actime;
  tva[0].tv_usec := 0;
  tva[1].tv_sec := times^.modtime;
  tva[1].tv_usec := 0;
  fputime:=do_syscall(syscall_nr_utimes,TSysParam(path),TSysParam(@tva));
end;

{$endif}


{$ifndef FPC_BASEUNIX_HAS_FPPIPE}
Function fppipe(var fildes : tfildes):cint;

begin
{$if defined(generic_linux_syscalls)}
 fppipe:=do_syscall(syscall_nr_pipe2,TSysParam(@fildes),0);
{$else}
 fppipe:=do_syscall(syscall_nr_pipe,TSysParam(@fildes));
{$endif}
end;
{$endif FPC_BASEUNIX_HAS_FPPIPE}


function fpfcntl(fildes:cint;Cmd:cint;Arg:cint):cint;

begin
 fpfcntl:=do_syscall(syscall_nr_fcntl,fildes,cmd,arg);
end;

function fpfcntl(fildes:cint;Cmd:cint;var Arg:flock):cint;

begin
 fpfcntl:=do_syscall(syscall_nr_fcntl,fildes,cmd,TSysParam(@arg));
end;

function fpfcntl(fildes:cint;Cmd:cint):cint;

begin
 fpfcntl:=do_syscall(syscall_nr_fcntl,fildes,cmd);
end;

function fpexecve(path:pchar;argv:ppchar;envp:ppchar):cint;

Begin
  fpexecve:=do_syscall(syscall_nr_Execve,TSysParam(path),TSysParam(argv),TSysParam(envp));
End;

function fpexecv(path:pchar;argv:ppchar):cint;

Begin
  fpexecv:=do_syscall(syscall_nr_Execve,TSysParam(path),TSysParam(argv),TSysParam(envp));
End;

function fptimes(var buffer : tms):clock_t;
begin
  fptimes:=Do_syscall(syscall_nr_times,TSysParam(@buffer));
end;

Function fpSelect(N:cint;readfds,writefds,exceptfds:pfdSet;TimeOut:PTimeVal):cint;
{
  Select checks whether the file descriptor sets in readfs/writefs/exceptfs
  have changed.
}
{$if defined(generic_linux_syscalls) and not defined(NO_SYSCALL_PSELECT6)}

var ts : timespec;
    pts : PTimeSpec;
begin
  pts:=nil;
  if assigned(timeout) then
    begin
      pts:=@ts;
      ts.tv_sec := timeout^.tv_sec;
      ts.tv_nsec := timeout^.tv_usec * 1000;
    end;
  fpSelect:=do_syscall(syscall_nr_pselect6,n,
                       tsysparam(readfds),tsysparam(writefds),
                       tsysparam(exceptfds),tsysparam(pts),0);
end;

{$else}

begin
{$ifdef cpux86_64}
  {$define bunxfunc_fpselect_implemented}
  fpSelect:=do_syscall(syscall_nr_select,n,tsysparam(readfds),tsysparam(writefds),tsysparam(exceptfds),tsysparam(timeout));
{$else}
  {$define bunxfunc_fpselect_implemented}
  fpSelect:=do_syscall(syscall_nr__newselect,n,tsysparam(readfds),tsysparam(writefds),tsysparam(exceptfds),tsysparam(timeout));
{$endif}
{$ifndef bunxfunc_fpselect_implemented}
  {$error Implement fpselect}
{$endif bunxfunc_fpselect_implemented}
end;

{$endif}

function fpPoll(fds: ppollfd; nfds: cuint; timeout: clong): cint;
{$if defined(generic_linux_syscalls) and not defined(NO_SYSCALL_PPOLL)}
var ts : timespec;
begin
  if timeout<0 then
    fpPoll:=do_syscall(syscall_nr_ppoll,tsysparam(fds),tsysparam(nfds),0,0)
  else
    begin
      ts.tv_sec := timeout div 1000;
      ts.tv_nsec := (timeout mod 1000) * 1000000;
      fpPoll:=do_syscall(syscall_nr_ppoll,tsysparam(fds),tsysparam(nfds),
                         tsysparam(@ts),0);
    end
end;
{$else}
begin
  fpPoll:=do_syscall(syscall_nr_poll,tsysparam(fds),tsysparam(nfds),tsysparam(timeout));
end;
{$endif}

Function fpLstat(path:pchar;Info:pstat):cint;
{
  Get all information on a link (the link itself), and return it in info.
}

begin
{$if defined(generic_linux_syscalls)}
 fpLStat:=do_syscall(syscall_nr_fstatat,AT_FDCWD,TSysParam(path),TSysParam(info),AT_SYMLINK_NOFOLLOW)
{$else}
 fpLStat:=do_syscall(
{$ifdef cpu64}
    syscall_nr_lstat,
{$else}
    syscall_nr_lstat64,
{$endif}
    TSysParam(path),TSysParam(info));
{$endif}
end;


function fpNice(N:cint):cint;
{
  Set process priority. A positive N means a lower priority.
  A negative N increases priority.

Doesn't exist in BSD. Linux emu uses setpriority in a construct as below:
}

{$if defined(generic_linux_syscalls) or defined(cpux86_64)}
var
  oldprio : cint;
{$endif}
begin
{$if defined(generic_linux_syscalls) or defined(cpux86_64)}
  oldprio:=fpGetPriority(Prio_Process,0);
  fpNice:=fpSetPriority(Prio_Process,0,oldprio+N);
  if fpNice=0 then
    fpNice:=fpGetPriority(Prio_Process,0);
{$else}
  fpNice:=do_syscall(Syscall_nr_nice,N);
{$endif}
end;

Function fpGetPriority(Which,Who:cint):cint;
{
  Get Priority of process, process group, or user.
   Which : selects what kind of priority is used.
           can be one of the following predefined Constants :
              Prio_User.
              Prio_PGrp.
              Prio_Process.
   Who : depending on which, this is , respectively :
              Uid
              Pid
              Process Group id
   Errors are reported in linuxerror _only_. (priority can be negative)
}
begin
  if (which<prio_process) or (which>prio_user) then
   begin
     { We can save an interrupt here }
     fpgetpriority:=-1;
     fpsetErrno(ESysEinval);
   end
  else
     fpGetPriority:=do_syscall(syscall_nr_GetPriority,which,who);
end;

Function fpSetPriority(Which,Who,What:cint):cint;
{
 Set Priority of process, process group, or user.
   Which : selects what kind of priority is used.
           can be one of the following predefined Constants :
              Prio_User.
              Prio_PGrp.
              Prio_Process.
   Who : depending on value of which, this is, respectively :
              Uid
              Pid
              Process Group id
   what : A number between -20 and 20. -20 is most favorable, 20 least.
          0 is the default.
}
begin
  if ((which<prio_process) or (which>prio_user)) or ((what<-20) or (what>20)) then
   fpseterrno(ESyseinval)  { We can save an interrupt here }
  else
   begin
     fpSetPriority:=do_syscall(Syscall_nr_Setpriority,which,who,what);
   end;
end;


Function fpSymlink(oldname,newname:pchar):cint;
{
  We need this for erase
}

begin
{$if defined(generic_linux_syscalls)}
  fpsymlink:=do_syscall(syscall_nr_symlinkat,TSysParam(oldname),AT_FDCWD,TSysParam(newname));
{$else}
  fpsymlink:=do_syscall(syscall_nr_symlink,TSysParam(oldname),TSysParam(newname));
{$endif}
end;

{ this is used by the Fppread/Fppwrite below. for more information,
  check the syscall() Linux man page (KB) }
{$if defined(FPC_ABI_EABI) or defined(CPUMIPS32) or defined(CPUMIPSEL32) or defined(CPUPOWERPC32)}
{$define FPC_ALIGN_DUMMY}
{$endif}

function Fppread(fd: cint; buf: pchar; nbytes : size_t; offset:Toff): ssize_t; [public, alias : 'FPC_SYSC_PREAD'];

begin
{$ifdef CPU64}
  Fppread:=do_syscall(syscall_nr_pread64,Fd,TSysParam(buf),nbytes,TSysParam(OffSet));
{$else}
  Fppread:=do_syscall(syscall_nr_pread64,Fd,TSysParam(buf),nbytes,
    {$ifdef FPC_ALIGN_DUMMY}   0,   {$endif FPC_ALIGN_DUMMY} { align parameters as required with dummy }
    {$ifdef FPC_BIG_ENDIAN}    hi(offset),lo(offset){$endif}
    {$ifdef FPC_LITTLE_ENDIAN} lo(offset),hi(offset){$endif}
   );
{$endif}
end;

function Fppwrite(fd: cint;buf:pchar; nbytes : size_t; offset:Toff): ssize_t; [public, alias : 'FPC_SYSC_PWRITE'];

begin
{$ifdef CPU64}
  Fppwrite:=do_syscall(syscall_nr_pwrite64,Fd,TSysParam(buf),nbytes,TSysParam(OffSet));
{$else}
  Fppwrite:=do_syscall(syscall_nr_pwrite64,Fd,TSysParam(buf),nbytes,
    {$ifdef FPC_ALIGN_DUMMY}   0,   {$endif FPC_ALIGN_DUMMY} { align parameters as required with dummy }
    {$ifdef FPC_BIG_ENDIAN}    hi(offset),lo(offset){$endif}
    {$ifdef FPC_LITTLE_ENDIAN} lo(offset),hi(offset){$endif}
  );
{$endif}
end;

{$undef FPC_ALIGN_DUMMY}


function Fpreadv(fd: cint; const iov : piovec; iovcnt : cint):ssize_t; [public, alias : 'FPC_SYSC_READV'];

begin
  Fpreadv:=do_syscall(syscall_nr_readv,Fd,TSysParam(iov),iovcnt);
end;

function Fpwritev(fd: cint; const iov : piovec; iovcnt : cint):ssize_t;  [public, alias : 'FPC_SYSC_WRITEV'];

begin
  Fpwritev:=do_syscall(syscall_nr_writev,Fd,TSysParam(iov),iovcnt);
end;

