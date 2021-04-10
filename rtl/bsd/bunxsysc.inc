{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 by Marco van de Voort

    Calls needed for the POSIX unit, but not for system.
    Some calls that can be used for both Linux and *BSD will be
    moved to a /unix/ includedfile later.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

Function FPKill(Pid:pid_t;Sig:cint):cint;
{
  Send signal 'sig' to a process, or a group of processes.
  If Pid >  0 then the signal is sent to pid
     pid=-1                         to all processes except process 1
     pid < -1                         to process group -pid
  Return value is zero, except for case three, where the return value
  is the number of processes to which the signal was sent.
}

begin
 FPkill:=do_syscall(syscall_nr_kill,pid,sig);
// if kill<0 THEN
//  Kill:=0;
end;

Function FPSigPending(var nset: sigset_t):cint;
{
  Allows examination of pending signals. The signal mask of pending
  signals is set in SSet
}
begin
  FPsigpending:=do_syscall(syscall_nr_sigpending,TSysParam(@nset));
end;

function FPsigsuspend(const sigmask:sigset_t):cint;
{
 Set the signal mask with Mask, and suspend the program until a signal
 is received.
}

begin
{$ifdef OPENBSD}
  { OpenBSD sigsuspend syscall wants a simple int as arg }
  FPsigsuspend:= do_syscall(syscall_nr_sigsuspend,TSysParam(sigmask[0]));
{$else}
  FPsigsuspend:= do_syscall(syscall_nr_sigsuspend,TSysParam(@sigmask));
{$endif}
end;

{$ifndef FPC_SYS_SIGTIMEDWAIT_UNAVAILABLE}
function fpsigtimedwait(const sigset:TSigSet;info:Psiginfo;timeout:Ptimespec):cint;

begin
  FpSigTimedWait:=do_syscall(syscall_nr_sigtimedwait,
                             Tsysparam(@sigset),
                             Tsysparam(info),
                             Tsysparam(timeout));
end;
{$endif ndef FPC_SYS_SIGTIMEDWAIT_UNAVAILABLE}


Type // implementation side for now. Should move to BSD unit.
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

Function FPalarm(Seconds: cuint):cuint;

Var it,oitv : Itimerval;

Begin
//      register struct itimerval *itp = &it;

 it.it_interval.tv_sec:=0;
 it.it_interval.tv_usec:=0;
 it.it_value.tv_sec:=seconds;
 it.it_value.tv_usec:=0;
 If SetITimer(ITIMER_REAL,it,oitv)<0 Then
   Exit(cuint(-1));

 if oitv.it_value.tv_usec<>0 Then
   Inc(oitv.it_value.tv_sec);
 FPAlarm:=oitv.it_value.tv_sec;
End;

function sigblock(mask:cuint):cint;
{Depreciated, but used by pause.}

var nset,oset: sigset_t;

begin
 FPsigemptyset(nset);
 nset[0]:=mask;
 sigblock:= FPsigprocmask(SIG_BLOCK,@nset,@oset);   // SIG_BLOCK=1
 if sigblock=0 Then
  sigblock:=oset[0];
end;

function sigpause(sigmask:cint):cint;
{Depreciated, but used by pause.}

var nset: sigset_t;

begin
 FPsigemptyset(nset);
 nset[0]:=sigmask;
 sigpause:= FPsigsuspend(nset);
end;

function FPpause:cint;

begin
  FPpause:=sigpause(sigblock(cuint(0)));
end;

function FPsleep(seconds:cuint):cuint;

var time_to_sleep,time_remaining : timespec;

begin
        {
         * Avoid overflow when `seconds' is huge.  This assumes that
         * the maximum value for a time_t is >= INT_MAX.
         }
        if seconds > high(cint) Then
                FPsleep:= (seconds - high(cint)) + FPsleep(HIGH(cint));

        time_to_sleep.tv_sec := seconds;
        time_to_sleep.tv_nsec := 0;
        if (FPnanosleep(@time_to_sleep, @time_remaining) <> -1) Then
         Exit(0);
        if (fpgeterrno <> ESysEINTR) Then       // EAGAIN?
         Exit (seconds);                     { best guess }
        FPsleep:= time_remaining.tv_sec;
        if   (time_remaining.tv_nsec <> 0) Then
         inc(FPsleep);
End;

function FPuname(var name:utsname):cint; [public,alias:'FPC_SYSC_UNAME'];

Var
        mib  : array[0..1] of cint;
        rval : cint;
        len  : size_t;
        i    : longint;
        oerrno : cint;

procedure Doone(pz:pchar;pzsize:cint;val1,val2:cint);

Begin
        mib[0] := val1;
        mib[1] := val2;
        len    := pzsize;
        oerrno := fpgeterrno;

        if (FPsysctl(@mib, 2, pz, @len, NIL, 0) = -1) Then
           Begin
                if (fpgeterrno = ESysENOMEM) Then
                        fpseterrno(oerrno)
                else
                        rval := -1;
           End;
         pz[pzsize- 1] := #0;
End;

Begin
        rval := 0;
        DoOne(@name.sysname,sizeof(name.sysname),CTL_KERN,KERN_OSTYPE);
        DoOne(@name.nodename,sizeof(name.nodename),CTL_KERN,KERN_HOSTNAME);
        DoOne(@name.release,sizeof(name.release),CTL_KERN,KERN_OSRELEASE);
        { The version may have newlines in it, turn them into spaces. }
        DoOne(@name.version,sizeof(name.version),CTL_KERN,KERN_VERSION);

        For I:=0 to sizeof(name.sysname)-2 Do
          If (name.version[i]=#13) or (name.version[i]=#9) Then
            name.version[i]:=' ';
        DoOne(@name.machine,sizeof(name.machine),CTL_HW,HW_MACHINE);
        FPUname:=rval;
end;

function GetDomainName(Name:PChar; NameLen:Cint):cint; [public,alias:'FPC_SYSC_GETDOMAINNAME'];

Const Mib_GetDomainName : array[0..1] of cint=(CTL_KERN,{$ifdef OpenBSD}KERN_DOMAINNAME{$ELSE}KERN_NISDOMAINNAME{$endif});

VAR
        tsize : size_t;
begin
        tsize := namelen;
        if (FPsysctl(@Mib_GetDomainname, 2, name, @tsize, NIL, 0) = -1) Then
          GetDomainName:=-1
        Else
          GetDomainName:=0;
end;

function GetHostName(Name:PChar; NameLen:Cint):cint;[public,alias:'FPC_SYSC_GETHOSTNAME'];

Const Mib_GetHostName : array[0..1] of cint=(CTL_KERN,KERN_HOSTNAME);

Var
        tsize : size_t;
begin
        tsize := namelen;
        if (FPsysctl(@Mib_GetHostName, 2, name, @tsize, NIL, 0) = -1) Then
          GetHostName:=-1
        Else
          GetHostName:=0;
End;

const WAIT_ANY = -1;

function FPwait(var stat_loc:cint): pid_t;
{
  Waits until a child with PID Pid exits, or returns if it is exited already.
  Any resources used by the child are freed.
  The exit status is reported in the adress referred to by Status. It should
  be a longint.
}

begin // actually a wait4() call with 4th arg 0.
 FPWait:=do_syscall(syscall_nr_WaitPID,WAIT_ANY,TSysParam(@Stat_loc),0,0);
end;

//function FPgetpid : pid_t;

// begin
//  FPgetpid:=do_syscall(syscall_nr_getpid);
// end;

function FPgetppid : pid_t;

begin
 FPgetppid:=do_syscall(syscall_nr_getppid);
end;

function FPgetuid : uid_t;

begin
 FPgetuid:=do_syscall(syscall_nr_getuid);
end;

function FPgeteuid : uid_t;

begin
 FPgeteuid:=do_syscall(syscall_nr_geteuid);
end;

function FPgetgid : gid_t;

begin
 FPgetgid:=do_syscall(syscall_nr_getgid);
end;

function FPgetegid : gid_t;

begin
 FPgetegid:=do_syscall(syscall_nr_getegid);
end;

function FPsetuid(uid : uid_t): cint;

begin
 FPsetuid:=do_syscall(syscall_nr_setuid,uid);
end;

function FPsetgid(gid : gid_t): cint;

begin
 FPsetgid:=do_syscall(syscall_nr_setgid,gid);
end;

// type tgrparr=array[0..0] of gid_t;

function FPgetgroups(gidsetsize : cint; var grouplist:tgrparr): cint;

begin
 FPgetgroups:=do_syscall(syscall_nr_getgroups,gidsetsize,TSysParam(@grouplist));
end;

function FPgetpgrp : pid_t;

begin
 FPgetpgrp:=do_syscall(syscall_nr_getpgrp);
end;

function FPsetsid : pid_t;

begin
 FPsetsid:=do_syscall(syscall_nr_setsid);
end;

function fpgetsid (pid:TPid): pid_t;

begin
 fpgetsid:=do_syscall(syscall_nr_getsid,TSysParam(pid));
end;

Function FPumask(cmask:mode_t):mode_t;
{
  Sets file creation mask to (Mask and 0777 (octal) ), and returns the
  previous value.
}
begin
 FPumask:=Do_syscall(syscall_nr_umask,cmask);
end;

Function FPlink(existing:pchar;newone:pchar):cint;
{
  Proceduces a hard link from new to old.
  In effect, new will be the same file as old.
}
begin
  FPLink:=Do_Syscall(syscall_nr_link,TSysParam(existing),TSysParam(newone));
end;

Function FPmkfifo(path:pchar;mode:mode_t):cint;

begin
  FPmkfifo:=do_syscall(syscall_nr_mkfifo,TSysParam(path),TSysParam(mode));
end;

Function FPchmod(path:pchar;mode:mode_t):cint;

begin
  FPchmod:=do_syscall(syscall_nr_chmod,TSysParam(path),TSysParam(mode));
end;

Function FPchown(path:pchar;owner:uid_t;group:gid_t):cint;

begin
  FPChOwn:=do_syscall(syscall_nr_chown,TSysParam(path),TSysParam(owner),TSysParam(group));
end;

Function FPUtime(path:pchar;times:putimbuf):cint;

var tv  : array[0..1] of timeval;
    tvp : ^timeval;

begin
 if times=nil Then
   tvp:=nil
 else
   begin
    tv[0].tv_sec :=times^.actime;
    tv[1].tv_sec :=times^.modtime;
    tv[0].tv_usec:=0;
    tv[1].tv_usec:=0;
    tvp:=@tv;
   end;
 FPutime:=do_syscall(syscall_nr_utimes,TSysParam(path),TSysParam(tvp));
end;

function __pipe_call(sysnr:TSysParam):TSysResult; {$ifdef cpui386}oldfpccall{$endif} external name 'FPC_DOSYS0';

{$if (defined(freebsd) or defined (dragonfly)) and (defined(CPUi386) or defined(CPUX86_64))}
  {$define PIPE_RESULT_IN_EAX_AND_EDX}
{$endif}
Function FPpipe(var fildes : tfildes):cint;
{$ifndef PIPE_RESULT_IN_EAX_AND_EDX}
begin
  fppipe:=do_syscall(syscall_nr_pipe,TSysParam(@fildes));
end;
{$else}
var
  a, b: cInt;
begin
  asm
  {$ifdef CPUi386}
    pushl syscall_nr_pipe
    call __pipe_call
    movl %eax, a
    movl %edx, b
  {$else}
    movq syscall_nr_pipe, %rdi
    call __pipe_call
    movl %eax, a
    movl %edx, b
  {$endif}
  end;

  fpPipe := a; // eax is in a, no matter if it worked or not
  fildes[0] := a;
  fildes[1] := b;
end;
{$endif}

function FPfcntl(fildes:cint;Cmd:cint;Arg:cint):cint;

begin
 FPfcntl:=do_syscall(syscall_nr_fcntl,fildes,cmd,arg);
end;

function FPfcntl(fildes:cint;Cmd:cint;var Arg:flock):cint;

begin
 FPfcntl:=do_syscall(syscall_nr_fcntl,fildes,cmd,TSysParam(@arg));
end;

function FPfcntl(fildes:cint;Cmd:cint):cint;

begin
 FPfcntl:=do_syscall(syscall_nr_fcntl,fildes,cmd);
end;

function FPexecve(path:pchar;argv:ppchar;envp:ppchar):cint;

Begin
  FPexecve:=do_syscall(syscall_nr_Execve,TSysParam(path),TSysParam(argv),TSysParam(envp));
End;

function FPexecv(path:pchar;argv:ppchar):cint;

Begin
  FPexecv:=do_syscall(syscall_nr_Execve,TSysParam(path),TSysParam(argv),TSysParam(envp));
End;

CONST RUSAGE_SELF       = 0;
      RUSAGE_CHILDREN   = -1;

function FPgetrusage(who:cint;var ru : rusage):cint;

begin
 FPgetrusage:=do_syscall(syscall_nr_getrusage,longint(who),TSysParam(@ru));
end;

function FPtimes(var buffer : tms):clock_t;

var ru : rusage;
    t  : timeval;

CONST CLK_TCK=128;

function CONVTCK(r:timeval):clock_t;
{
 * Convert usec to clock ticks; could do (usec * CLK_TCK) / 1000000,
 * but this would overflow if we switch to nanosec.
 }
begin
 CONVTCK:=(r.tv_sec * CLK_TCK + r.tv_usec DIV (1000000 DIV CLK_TCK));
end;

begin

        if (FPgetrusage(RUSAGE_SELF, ru) < 0) Then
            exit(clock_t(-1));
        buffer.tms_utime := CONVTCK(ru.ru_utime);
        buffer.tms_stime := CONVTCK(ru.ru_stime);
        if (FPgetrusage(RUSAGE_CHILDREN, ru) < 0) Then
            exit(clock_t(-1));
        buffer.tms_cutime := CONVTCK(ru.ru_utime);
        buffer.tms_cstime := CONVTCK(ru.ru_stime);
        if do_syscall(syscall_nr_gettimeofday,TSysParam(@t),0)<>0 Then
                    exit(clock_t(-1));
        FPtimes:=clock_t(CONVTCK(t));
end;

Function fpSelect(N:cint;readfds,writefds,exceptfds:pfdSet;TimeOut:PTimeVal):cint;
{
  Select checks whether the file descriptor sets in readfs/writefs/exceptfs
  have changed.
}

begin
 fpSelect:=do_syscall(syscall_nr_select,n,TSysParam(readfds),TSysParam(writefds),TSysParam(exceptfds),TSysParam(timeout));
end;

function fpPoll(fds: ppollfd; nfds: cuint; timeout: clong): cint;
begin
  fpPoll:=do_syscall(syscall_nr_poll,tsysparam(fds),tsysparam(nfds),tsysparam(timeout));
end;

Function fpLstat(path:pchar;Info:pstat):cint;
{
  Get all information on a link (the link itself), and return it in info.
}

begin
 fpLStat:=do_syscall(syscall_nr_lstat,TSysParam(path),TSysParam(info));
end;

function fpNice(N:cint):cint;
{
  Set process priority. A positive N means a lower priority.
  A negative N decreases priority.

Doesn't exist in BSD. Linux emu uses setpriority in a construct as below:
}

var prio : cint;

begin
  fpseterrno(0);
  prio:=fpgetpriority(PRIO_PROCESS,0);
  if (prio=-1) and (fpgeterrno<>0) then
      exit(-1);
  fpNice:=fpSetPriority(Prio_Process,0,prio+N);
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
     fpgetpriority:=0;
     fpseterrno(ESysEinval);
   end
  else
   begin
     fpGetPriority:=do_syscall(syscall_nr_GetPriority,which,who);
   end;
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
 fpsymlink:=do_syscall(syscall_nr_symlink,TSysParam(oldname),TSysParam(newname));
end;

function Fppread(fd: cint; buf: pchar; nbytes : size_t; offset:Toff): ssize_t; [public, alias : 'FPC_SYSC_PREAD'];

begin

 	{$ifdef CPU64}
	    Fppread:=do_syscall(syscall_nr_pread,Fd,TSysParam(buf),nbytes,TSysParam(OffSet));
	{$else}
           Fppread:=do_syscall(syscall_nr_pread,Fd,TSysParam(buf),nbytes,
           {$ifdef 64bitfs}
	     {$ifdef FPC_BIG_ENDIAN}    hi(offset),lo(offset){$endif}
	     {$ifdef FPC_LITTLE_ENDIAN} lo(offset),hi(offset){$endif}
           {$else}
	     {$ifdef FPC_BIG_ENDIAN}    0,lo(offset){$endif}
	     {$ifdef FPC_LITTLE_ENDIAN} lo(offset),0{$endif}
	   {$endif}
            );
        {$endif}
end;

function Fppwrite(fd: cint;buf:pchar; nbytes : size_t; offset:Toff): ssize_t; [public, alias : 'FPC_SYSC_PWRITE'];

begin
 	{$ifdef CPU64}
           Fppwrite:=do_syscall(syscall_nr_pwrite,Fd,TSysParam(buf),nbytes,TSysParam(OffSet));
	{$else}
           Fppwrite:=do_syscall(syscall_nr_pwrite,Fd,TSysParam(buf),nbytes,
	  // ,0  = possible alignment here.
           {$ifdef 64bitfs}
	     {$ifdef FPC_BIG_ENDIAN}    hi(offset),lo(offset){$endif}
	     {$ifdef FPC_LITTLE_ENDIAN} lo(offset),hi(offset){$endif}
           {$else}
	     {$ifdef FPC_BIG_ENDIAN}    0,lo(offset){$endif}
	     {$ifdef FPC_LITTLE_ENDIAN} lo(offset),0{$endif}
	   {$endif}
            );
        {$endif}
end;

function Fpreadv(fd: cint; const iov : piovec; iovcnt : cint):ssize_t; [public, alias : 'FPC_SYSC_READV'];

begin
  Fpreadv:=do_syscall(syscall_nr_readv,Fd,TSysParam(iov),iovcnt);
end;

function Fpwritev(fd: cint; const iov : piovec; iovcnt : cint):ssize_t;  [public, alias : 'FPC_SYSC_WRITEV'];

begin
  Fpwritev:=do_syscall(syscall_nr_writev,Fd,TSysParam(iov),iovcnt);
end;

