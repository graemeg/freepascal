{
    Copyright (c) 2002 by Marco van de Voort

    The base Linux syscalls required to implement the system unit. These
    are aliased for use in other units (to avoid poluting the system units
    interface)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ****************************************************************************
}

{*****************************************************************************
                     --- Main:The System Call Self ---
*****************************************************************************}

function Fptime(tloc:pTime_t): Time_t; [public, alias : 'FPC_SYSC_TIME'];
{$if defined(generic_linux_syscalls) or defined(FPC_USEGETTIMEOFDAY)}
VAR tv     : timeval;
    tz     : timezone;
    retval : longint;

begin
  Retval:=do_syscall(syscall_nr_gettimeofday,TSysParam(@tv),TSysParam(@tz));
  If retval=-1 then
   Fptime:=-1
  else
   Begin
   If Assigned(tloc) Then
     TLoc^:=tv.tv_sec;
    Fptime:=tv.tv_sec;
   End;
End;
{$else FPC_USEGETTIMEOFDAY}
begin
  Fptime:=do_syscall(syscall_nr_time,TSysParam(tloc));
end;
{$endif FPC_USEGETTIMEOFDAY}

{*****************************************************************************
               --- File:File handling related calls ---
*****************************************************************************}

function Fpopen(path: pchar; flags : cint; mode: mode_t):cint; [public, alias : 'FPC_SYSC_OPEN'];

Begin
{$if defined(generic_linux_syscalls)}
 Fpopen:=do_syscall(syscall_nr_openat,AT_FDCWD,TSysParam(path),TSysParam(flags or O_LARGEFILE),TSysParam(mode));
{$else}
 Fpopen:=do_syscall(syscall_nr_open,TSysParam(path),TSysParam(flags or O_LARGEFILE),TSysParam(mode));
{$endif}
End;

function Fpclose(fd : cint): cint; [public, alias : 'FPC_SYSC_CLOSE'];

begin
 Fpclose:=do_syscall(syscall_nr_close,fd);
end;

function Fplseek(fd : cint; offset : off_t; whence : cint): off_t; [public, alias : 'FPC_SYSC_LSEEK'];
begin
{$if defined(cpu64)}
  result:=do_syscall(syscall_nr_lseek,tsysparam(fd),tsysparam(offset),tsysparam(whence));
{$else}
  if do_syscall(syscall_nr__llseek,tsysparam(fd),
      tsysparam(hi(offset)),tsysparam(lo(offset)),
      tsysparam(@result), tsysparam(whence)) = -1 then
    result:=off_t(-1);
{$endif}
end;

function Fpread(fd: cint; buf: pchar; nbytes : size_t): ssize_t; [public, alias : 'FPC_SYSC_READ'];

begin
  Fpread:=do_syscall(syscall_nr_read,Fd,TSysParam(buf),nbytes);
end;

function Fpwrite(fd: cint;buf:pchar; nbytes : size_t): ssize_t; [public, alias : 'FPC_SYSC_WRITE'];

begin
 Fpwrite:=do_syscall(syscall_nr_write,Fd,TSysParam(buf),nbytes);
end;

function Fpunlink(path: pchar): cint; [public, alias : 'FPC_SYSC_UNLINK'];

begin
{$if defined(generic_linux_syscalls)}
  Fpunlink:=do_syscall(syscall_nr_unlinkat,AT_FDCWD,TSysParam(path),0);
{$else}
  Fpunlink:=do_syscall(syscall_nr_unlink,TSysParam(path));
{$endif}
end;

function Fprename(old : pchar; newpath: pchar): cint; [public, alias : 'FPC_SYSC_RENAME'];

begin
{$if defined(generic_linux_syscalls)}
  {$if defined(userenameat)}
    Fprename:=do_syscall(syscall_nr_renameat,AT_FDCWD,TSysParam(old),AT_FDCWD,TSysParam(newpath));
  {$else}
    Fprename:=do_syscall(syscall_nr_renameat2,AT_FDCWD,TSysParam(old),AT_FDCWD,TSysParam(newpath),0);
  {$endif}
{$else}
  Fprename:=do_syscall(syscall_nr_rename,TSysParam(old),TSysParam(newpath));
{$endif}
end;

function Fpstat(path: pchar; var buf: stat):cint; [public, alias : 'FPC_SYSC_STAT'];

begin
{ standard stat call is 32 bit on sparc64, so explicitly force a stat64 call }
{$if (defined(cpu64) or defined(riscv32)) and not(defined(cpusparc64))}
  {$if defined(generic_linux_syscalls)}
    Fpstat:=do_syscall(syscall_nr_fstatat,AT_FDCWD,TSysParam(path),TSysParam(@buf),0);
  {$else}
    Fpstat:=do_syscall(syscall_nr_stat,TSysParam(path),TSysParam(@buf));
  {$endif}
{$else}
  Fpstat:=do_syscall(syscall_nr_stat64,TSysParam(path),TSysParam(@buf));
{$endif}
end;

{*****************************************************************************
               --- Directory:Directory related calls ---
*****************************************************************************}

function Fpchdir(path : pchar): cint; [public, alias : 'FPC_SYSC_CHDIR'];

begin
 Fpchdir:=do_syscall(syscall_nr_chdir,TSysParam(path));
end;

function Fpmkdir(path : pchar; mode: mode_t):cint; [public, alias : 'FPC_SYSC_MKDIR'];

begin
{$if defined(generic_linux_syscalls)}
  Fpmkdir:=do_syscall(syscall_nr_mkdirat,AT_FDCWD,TSysParam(path),TSysParam(mode));
{$else}
  Fpmkdir:=do_syscall(syscall_nr_mkdir,TSysParam(path),TSysParam(mode));
{$endif}
end;

function Fprmdir(path : pchar): cint;  [public, alias : 'FPC_SYSC_RMDIR'];

begin
{$if defined(generic_linux_syscalls)}
 Fprmdir:=do_syscall(syscall_nr_unlinkat,AT_FDCWD,TSysParam(path),AT_REMOVEDIR);
{$else}
 Fprmdir:=do_syscall(syscall_nr_rmdir,TSysParam(path));
{$endif}
end;

function Fpopendir(dirname : pchar): pdir;  [public, alias : 'FPC_SYSC_OPENDIR'];

var
  fd:cint;
  st:stat;
  ptr:pdir;

begin
  Fpopendir:=nil;
  if Fpstat(dirname,st)<0 then
   exit;
{ Is it a dir ? }
  if not((st.st_mode and $f000)=$4000)then
   begin
     errno:=ESysENOTDIR;
     exit
   end;
{ Open it}
  fd:=Fpopen(dirname,O_RDONLY,438);
  if fd<0 then
   exit;
  new(ptr);
  if ptr=nil then
   exit;
  new(ptr^.dd_buf);
  if ptr^.dd_buf=nil then
   exit;
  ptr^.dd_fd:=fd;
  ptr^.dd_loc:=0;
  ptr^.dd_size:=0;
  ptr^.dd_nextoff:=0;
  ptr^.dd_max:=sizeof(ptr^.dd_buf^);
  Fpopendir:=ptr;
end;

function Fpclosedir(dirp : pdir): cint; [public, alias : 'FPC_SYSC_CLOSEDIR'];
begin
  repeat
    Fpclosedir:=Fpclose(dirp^.dd_fd);
  until (Fpclosedir=0) or (errno<>ESysEINTR);
  dispose(dirp^.dd_buf);
  dispose(dirp);
end;

Function Fpreaddir(dirp : pdir) : pdirent; [public, alias: 'FPC_SYSC_READDIR'];
var bytes : longint;
    dp    : pdirent;
begin
  repeat
    if dirp^.dd_nextoff >= dirp^.dd_size then
      begin
        bytes := do_SysCall(SysCall_nr_getdents64,TSysParam(dirp^.dd_fd),TSysParam(dirp^.dd_buf),TSysParam(dirp^.dd_max));
        if bytes <= 0 then
          begin
            fpreaddir := nil;
            exit;
          end;
        dirp^.dd_size := bytes;
        dirp^.dd_nextoff := 0;
      end;
    dp := Pdirent(ptruint(dirp^.dd_buf)+dirp^.dd_nextoff);
    inc(dirp^.dd_nextoff,dp^.d_reclen);
    inc(dirp^.dd_loc,dp^.d_reclen);
  until dp^.d_fileno <> 0; // Don't show deleted files
  Fpreaddir := dp;
end;


{*****************************************************************************
        --- Process:Process & program handling - related calls ---
*****************************************************************************}

procedure Fpexit(status : cint); [public, alias : 'FPC_SYSC_EXIT'];

begin
  do_syscall(syscall_nr_exit_group,status);
end;

{
  Change action of process upon receipt of a signal.
  Signum specifies the signal (all except SigKill and SigStop).
  If Act is non-nil, it is used to specify the new action.
  If OldAct is non-nil the previous action is saved there.
}


{$if defined(cpusparc) or defined(cpusparc64)}
procedure Fprt_sigreturn_stub;assembler;nostackframe;
asm
  mov syscall_nr_rt_sigreturn,%g1
  {$ifdef cpusparc}
  ta  0x10
  {$else}
  ta 0x6d
  {$endif}
end;
{$endif cpusparc or cpusparc64}

{$if defined(android)}

{$if defined(cpux86_64)}
  {$define NEED_USER_TRAMPOLINE}
{$endif}

{$else not android}

{$if defined(cpui386) or defined(cpuarm) or defined(cpux86_64)}
  {$define NEED_USER_TRAMPOLINE}
{$endif}
{$if defined(cpui386) or defined(cpuarm)}
  {$define NEED_USER_TRAMPOLINE_RT_DIFFERENT}
{$endif}

{$endif android}

{$ifdef NEED_USER_TRAMPOLINE}

procedure linux_restore; cdecl; nostackframe; assembler;
{$ifdef cpuarm}
asm
{$ifdef FPC_ABI_EABI}
  mov r7, syscall_nr_sigreturn
{$ifdef CPUTHUMB}
  // svc   #0x0
  // GNU 2.22 does not like svc #0x0
  .byte 0,0xdf
{$else CPUTHUMB}
  swi #0x0
{$endif CPUTHUMB}
{$else}
  swi syscall_nr_sigreturn
{$endif}
end;
{$endif}
{$ifdef cpui386}
asm
  popl %eax
  movl $syscall_nr_sigreturn, %eax
  int $0x80
end;
{$endif}
{$ifdef cpux86_64}
asm
  movq $syscall_nr_rt_sigreturn, %rax
  syscall
end;
{$endif}

{$endif}

{$ifdef NEED_USER_TRAMPOLINE_RT_DIFFERENT}

procedure linux_restore_rt; cdecl; nostackframe; assembler;
{$ifdef cpuarm}
asm
{$ifdef FPC_ABI_EABI}
  mov r7, syscall_nr_rt_sigreturn
{$ifdef CPUTHUMB}
  // svc   #0x0
  // GNU 2.22 does not like svc #0x0
  .byte 0,0xdf
{$else CPUTHUMB}
  swi #0x0
{$endif CPUTHUMB}
{$else}
  swi syscall_nr_rt_sigreturn
{$endif}
end;
{$endif}
{$ifdef cpui386}
asm
  movl $syscall_nr_rt_sigreturn, %eax
  int $0x80
end;
{$endif}

{$endif}

function Fpsigaction(sig: cint; new_action, old_action: psigactionrec): cint; [public, alias : 'FPC_SYSC_SIGACTION'];
{
  Change action of process upon receipt of a signal.
  Signum specifies the signal (all except SigKill and SigStop).
  If Act is non-nil, it is used to specify the new action.
  If OldAct is non-nil the previous action is saved there.
}
begin
{$if defined(cpusparc) or defined(cpusparc64)}
  { Sparc has an extra stub parameter }
{$ifdef cpusparc}
  { it seems that rt_sigreturn only works correctly for 32-bit if SA_SIGINFO is set }
  if new_action <> nil then
    new_action^.sa_flags:=new_action^.sa_flags or SA_SIGINFO;
{$endif}
  { the -8 stems from the fact that the adress is supposed to be a value from a sub-routine
    call stored into register %i7 to which +8 is added upon return }
  Fpsigaction:=do_syscall(syscall_nr_rt_sigaction,TSysParam(sig),TSysParam(new_action),TSysParam(old_action),TSysParam(ptruint(@Fprt_sigreturn_stub)-8),TSysParam(8));
{$else not (defined(cpusparc) or defined(cpusparc64))}
{$ifdef NEED_USER_TRAMPOLINE}
  if new_action <> nil then
  begin
    // a different stack (SA_ONSTACK) requires sa_restorer field
    if (new_action^.sa_flags and (SA_RESTORER or SA_ONSTACK)) = 0 then
    begin
      new_action^.sa_flags := new_action^.sa_flags or SA_RESTORER;
{$ifdef NEED_USER_TRAMPOLINE_RT_DIFFERENT}
      if (new_action^.sa_flags and SA_SIGINFO) <> 0 then
        new_action^.sa_restorer := @linux_restore_rt
      else
{$endif}
        new_action^.sa_restorer := @linux_restore;
    end;
  end;
{$endif}
  Fpsigaction:=do_syscall(syscall_nr_rt_sigaction,TSysParam(sig),
                          TSysParam(new_action),TSysParam(old_action),
  {$ifdef cpumips}
                          TSysParam(16{should be wordsinsigset})
  {$else not cpumips}
                          TSysParam(8)
  {$endif not cpumips}
                          );
{$endif cpusparc}
end;

{ this is used by the FpFtruncate below. for more information,
  check the syscall() Linux man page (KB) }
{$if defined(FPC_ABI_EABI) or defined(CPUMIPS32) or defined(CPUMIPSEL32) or defined(CPUPOWERPC32)}
{$define FPC_ALIGN_DUMMY}
{$endif}

function Fpftruncate(fd : cint; flength : off_t): cint; [public, alias : 'FPC_SYSC_FTRUNCATE'];
{ See notes lseek. This one is completely similar for the parameter (but
doesn't have the returnvalue 64-bit problem)}

begin
{$if defined(cpu64)}
 Fpftruncate:=Do_syscall(syscall_nr_ftruncate,TSysParam(fd),TSysParam(flength));
{$else}
 Fpftruncate:=Do_syscall(syscall_nr_ftruncate64,TSysParam(fd),
   {$ifdef FPC_ALIGN_DUMMY}   0,  {$endif FPC_ALIGN_DUMMY} { align parameters as required with dummy }
   {$ifdef FPC_BIG_ENDIAN}    tsysparam(hi(flength)),tsysparam(lo(flength)){$endif}
   {$ifdef FPC_LITTLE_ENDIAN} tsysparam(lo(flength)),tsysparam(hi(flength)){$endif}
   );
{$endif}
end;

{$undef FPC_ALIGN_DUMMY}

function Fpfstat(fd : cint; var sb : stat): cint;  [public, alias : 'FPC_SYSC_FSTAT'];

begin
{$if defined(cpu64) and not(defined(cpusparc64))}
  {$if defined(generic_linux_syscalls)}
    FpFStat:=do_SysCall(syscall_nr_fstat,TSysParam(fd),TSysParam(@sb));
  {$else}
    FpFStat:=do_SysCall(syscall_nr_fstat,TSysParam(fd),TSysParam(@sb));
  {$endif}
{$else}
  FpFStat:=do_SysCall(syscall_nr_fstat64,TSysParam(fd),TSysParam(@sb));
{$endif}
end;


{$ifndef FPC_SYSTEM_HAS_FPFORK}
function Fpfork : pid_t;  [public, alias : 'FPC_SYSC_FORK'];
{
  This function issues the 'fork' System call. the program is duplicated in memory
  and Execution continues in parent and child process.
  In the parent process, fork returns the PID of the child. In the child process,
  zero is returned.
  A negative value indicates that an error has occurred, the error is returned in
  LinuxError.
}
{$if defined(generic_linux_syscalls)}
var
  pid : Int64;
{$endif}
Begin
{$if defined(generic_linux_syscalls)}
 Fpfork:=Do_syscall(syscall_nr_clone,clone_flags_fork,0,0,0,TSysParam(@pid));
{$else}
 Fpfork:=Do_syscall(SysCall_nr_fork);
{$endif}
End;
{$endif FPC_SYSTEM_HAS_FPFORK}

// Look at execve variants later, when overloaded is determined.
{
function Fpexecve(path : pathstr; argv : ppchar; envp: ppchar): cint;
}
{
  Replaces the current program by the program specified in path,
  arguments in args are passed to Execve.
  environment specified in ep is passed on.
}

{
Begin
  path:=path+#0;
  do_syscall(syscall_nr_Execve,TSysParam(@path[1]),TSysParam(Argv),TSysParam(envp));
End;
}
{
function Fpexecve(path : pchar; argv : ppchar; envp: ppchar): cint;  [public, alias : 'FPC_SYSC_EXECVE'];
}
{
  Replaces the current program by the program specified in path,
  arguments in args are passed to Execve.
  environment specified in ep is passed on.
}
{
Begin
  do_syscall(syscall_nr_Execve,TSysParam(path),TSysParam(Argv),TSysParam(envp));
End;
}

function Fpwaitpid(pid : pid_t; stat_loc : pcint; options: cint): pid_t; [public, alias : 'FPC_SYSC_WAITPID'];
{
  Waits until a child with PID Pid exits, or returns if it is exited already.
  Any resources used by the child are freed.
  The exit status is reported in the adress referred to by Status. It should
  be a longint.
}

begin
{$if defined(generic_linux_syscalls) or defined(WAIT4)}
 FpWaitPID:=do_syscall(syscall_nr_Wait4,PID,TSysParam(Stat_loc),options,0);
{$else WAIT4}
 FpWaitPID:=do_syscall(syscall_nr_WaitPID,PID,TSysParam(Stat_loc),options);
{$endif WAIT4}
end;

function Fpaccess(pathname : pchar; amode : cint): cint; [public, alias : 'FPC_SYSC_ACCESS'];
{
  Test users access rights on the specified file.
  Mode is a mask xosisting of one or more of R_OK, W_OK, X_OK, F_OK.
  R,W,X stand for read,write and Execute access, simultaneously.
  F_OK checks whether the test would be allowed on the file.
  i.e. It checks the search permissions in all directory components
  of the path.
  The test is done with the real user-ID, instead of the effective.
  If access is denied, or an error occurred, false is returned.
  If access is granted, true is returned.
  Errors other than no access,are reported in unixerror.
}

begin
{$if defined(generic_linux_syscalls)}
 FpAccess:=do_syscall(syscall_nr_faccessat,AT_FDCWD,TSysParam(pathname),amode,0);
{$else}
 FpAccess:=do_syscall(syscall_nr_access,TSysParam(pathname),amode);
{$endif}
end;

(* overloaded
function Fpaccess(pathname : pathstr; amode : cint): cint;

{
  Test users access rights on the specified file.
  Mode is a mask xosisting of one or more of R_OK, W_OK, X_OK, F_OK.
  R,W,X stand for read,write and Execute access, simultaneously.
  F_OK checks whether the test would be allowed on the file.
  i.e. It checks the search permissions in all directory components
  of the path.
  The test is done with the real user-ID, instead of the effective.
  If access is denied, or an error occurred, false is returned.
  If access is granted, true is returned.
  Errors other than no access,are reported in unixerror.
}

begin
 pathname:=pathname+#0;
 Access:=do_syscall(syscall_nr_access, TSysParam(@pathname[1]),mode)=0;
end;
*)

Function FpDup(fildes:cint):cint; [public, alias : 'FPC_SYSC_DUP'];

begin
  Fpdup:=Do_syscall(syscall_nr_dup,TSysParam(fildes));
end;

Function FpDup2(fildes,fildes2:cint):cint; [public, alias : 'FPC_SYSC_DUP2'];

begin
{$if defined(generic_linux_syscalls)}
 if fildes<>fildes2 then
   Fpdup2:=do_syscall(syscall_nr_dup3,TSysParam(fildes),TSysParam(fildes2),0)
 else if do_syscall(syscall_nr_fcntl,TSysParam(fildes),1)<>-1 then
   Fpdup2:=fildes2
 else
   Fpdup2:=-1;
{$else}
 Fpdup2:=do_syscall(syscall_nr_dup2,TSysParam(fildes),TSysParam(fildes2));
{$endif}
end;


type
  tmmapargs = record
    address : TSysParam;
    size    : TSysParam;
    prot    : TSysParam;
    flags   : TSysParam;
    fd      : TSysParam;
    offset  : TSysParam;
  end;

Function Fpmmap(adr:pointer;len:size_t;prot:cint;flags:cint;fd:cint;off:off_t):pointer;  [public, alias : 'FPC_SYSC_MMAP'];
// OFF_T procedure, and returns a pointer, NOT cint.

{$ifdef OLDMMAP}
var
  mmapargs : tmmapargs;
begin
  mmapargs.address:=TSysParam(adr);
  mmapargs.size:=TSysParam(len);
  mmapargs.prot:=TSysParam(prot);
  mmapargs.flags:=TSysParam(flags);
  mmapargs.fd:=TSysParam(fd);
  mmapargs.offset:=TSysParam(off);
  Fpmmap:=pointer(do_syscall(syscall_nr_mmap,TSysParam(@MMapArgs)));
end;
{$else OLDMMAP}
{$ifdef MMAP2}
begin
  {$if sizeof(TSysParam)>=sizeof(off_t)}
  Fpmmap:= pointer(do_syscall(syscall_nr_mmap2,TSysParam(adr),TSysParam(len),
    TSysParam(prot),TSysParam(flags),TSysParam(fd),TSysParam(off div 4096)));
  {$else off_t is larger than TsysParam }
  {$message warning need mmap64 syscall, hi(off) not used}
  Fpmmap:= pointer(do_syscall(syscall_nr_mmap2,TSysParam(adr),TSysParam(len),
    TSysParam(prot),TSysParam(flags),TSysParam(fd),TSysParam(lo(off div 4096))));
 {$endif}
end;
{$else MMAP2}
begin
  {$if sizeof(TSysParam)>=sizeof(off_t)}
  Fpmmap:= pointer(do_syscall(syscall_nr_mmap,TSysParam(adr),TSysParam(len),
    TSysParam(prot),TSysParam(flags),TSysParam(fd),TSysParam(off)));
  {$else off_t is larger than TsysParam }
  {$message warning need mmap64 syscall, hi(off) not used}
  Fpmmap:= pointer(do_syscall(syscall_nr_mmap,TSysParam(adr),TSysParam(len),
    TSysParam(prot),TSysParam(flags),TSysParam(fd),TSysParam(lo(off))));
 {$endif}
end;
{$endif MMAP2}
{$endif OLDMMAP}


Function Fpmunmap(adr:pointer;len:size_t):cint; [public, alias :'FPC_SYSC_MUNMAP'];
begin
  Fpmunmap:=do_syscall(syscall_nr_munmap,TSysParam(Adr),TSysParam(Len));
end;

{$define FPC_SYSTEM_HAS_MREMAP}
Function Fpmremap(adr:pointer;old_len,new_len:size_t;flags:cint;new_adr:pointer):pointer; [public, alias :'FPC_SYSC_MREMAP'];
begin
  Fpmremap:=pointer(do_syscall(syscall_nr_mremap,TSysParam(Adr),TSysParam(old_len),TSysParam(new_len),TSysParam(flags),TSysParam(new_adr)));
end;

Function Fpmprotect(adr:pointer;len:size_t;prot:cint):cint; [public, alias : 'FPC_SYSC_MPROTECT'];
begin
  Fpmprotect:=do_syscall(syscall_nr_mprotect,TSysParam(adr),TSysParam(len),TSysParam(prot));
end;

{
  Interface to Unix ioctl call.
  Performs various operations on the filedescriptor Handle.
  Ndx describes the operation to perform.
  Data points to data needed for the Ndx function. The structure of this
  data is function-dependent.
}

// prototype is cint __P(cint,culong,....)
// actual meaning of return value depends on request.

Function FpIOCtl(fd:cint;request:TIOCtlRequest;Data: Pointer):cint;  [public, alias : 'FPC_SYSC_IOCTL'];
// This was missing here, instead hardcoded in Do_IsDevice
begin
  FpIOCtl:=do_SysCall(syscall_nr_ioctl,tsysparam(fd),tsysparam(Request),TSysParam(data));
end;


Function FpGetPid:pid_t;   [public, alias : 'FPC_SYSC_GETPID'];
{
  Get Process ID.
}

begin
 FpGetPID:=do_syscall(syscall_nr_getpid);
end;

Function FpReadLink(name,linkname:pchar;maxlen:size_t):cint;  [public, alias : 'FPC_SYSC_READLINK'];

begin
{$if defined(generic_linux_syscalls)}
  Fpreadlink:=do_syscall(syscall_nr_readlinkat,AT_FDCWD,TSysParam(name),TSysParam(linkname),maxlen);
{$else}
  Fpreadlink:=do_syscall(syscall_nr_readlink, TSysParam(name),TSysParam(linkname),maxlen);
{$endif}
end;


function FPSigProcMask(how:cint;nset : psigset;oset : psigset):cint; [public, alias : 'FPC_SYSC_SIGPROCMASK'];

{
  Change the list of currently blocked signals.
  How determines which signals will be blocked :
   SigBlock   : Add SSet to the current list of blocked signals
   SigUnBlock : Remove the signals in SSet from the list of blocked signals.
   SigSetMask : Set the list of blocked signals to SSet
  if OldSSet is non-null, the old set will be saved there.
}

begin
  FPsigprocmask:=do_syscall(syscall_nr_rt_sigprocmask,TSysParam(how),
		                    TSysParam(nset),TSysParam(oset),
{$ifdef CPUMIPS}
							TSysParam(16)
{$else not CPUMIPS}
							TSysParam(8)
{$endif not CPUMIPS}
					);
end;

Function FpNanoSleep(req : ptimespec;rem : ptimespec):cint; [public, alias : 'FPC_SYSC_NANOSLEEP'];
begin
  FpNanoSleep:=Do_SysCall(syscall_nr_nanosleep,TSysParam(req),TSysParam(rem));
end;

function fpgetcwd(path : pchar; siz:tsize):pchar;  [public, alias : 'FPC_SYSC_GETCWD'];

begin
  fpgetcwd:=pchar(Do_Syscall(Syscall_nr_getcwd,TSysParam(Path),TSysParam(siz)));
end;

function fpgettimeofday(tp: ptimeval;tzp:ptimezone):cint; [public, alias: 'FPC_SYSC_GETTIMEOFDAY'];

begin
 fpgettimeofday:=do_syscall(syscall_nr_gettimeofday,TSysParam(tp),TSysParam(tzp));
end;


function FpGetRLimit(resource : cInt; rlim : PRLimit) : cInt; [public, alias : 'FPC_SYSC_GETRLIMIT'];
begin
{$ifndef NO_SYSCALL_GETRLIMIT}
  FpGetRLimit := do_syscall(syscall_nr_getrlimit,
    TSysParam(resource), TSysParam(rlim));
{$else}
  FpGetRLimit := do_syscall(syscall_nr_ugetrlimit,
    TSysParam(resource), TSysParam(rlim));
{$endif}
end;


{$ifdef HAS_UGETRLIMIT}
function fpugetrlimit(resource : cInt; rlim : PRLimit) : cInt;
begin
  FpUGetRLimit := do_syscall(syscall_nr_ugetrlimit,
    TSysParam(resource), TSysParam(rlim));
end;
{$endif}

{$if defined(cpupowerpc)}
{ fpprctl() call }
function fpprctl(option : cint; const arg : ptruint) : cint;
begin
  fpprctl := do_syscall(syscall_nr_prctl, option, arg);
end;
{$endif}

function FpSetRLimit(Resource:cint;rlim:PRLimit):cint; [public, alias : 'FPC_SYSC_SETRLIMIT'];
begin
  fpsetrlimit:=do_syscall(syscall_nr_setrlimit,TSysParam(Resource),TSysParam(rlim));
end;

