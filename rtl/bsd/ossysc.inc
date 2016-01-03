{
    Copyright (c) 2002 by Marco van de Voort

    The base *BSD syscalls required to implement the system unit. These
    are aliased for use in other units (to avoid poluting the system units
    interface)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ****************************************************************************
}


function Fptime( tloc:ptime): time_t; [public, alias : 'FPC_SYSC_TIME'];

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

{*****************************************************************************
               --- File:File handling related calls ---
*****************************************************************************}

function Fpopen(path: pchar; flags : cint; mode: mode_t):cint; [public, alias : 'FPC_SYSC_OPEN'];

Begin
 Fpopen:=do_syscall(syscall_nr_open,TSysParam(path),TSysParam(flags),TSysParam(mode));
End;

function Fpclose(fd : cint): cint; [public, alias : 'FPC_SYSC_CLOSE'];

begin
 Fpclose:=do_syscall(syscall_nr_close,fd);
end;

{$ifdef netbsd}
  {$ifdef cpupowerpc}
    {$define netbsdmacppc}
  {$endif}
{$endif}

{$ifdef netbsdmacppc}
{$i sysofft.inc}                        // odd ball calling convention.
{$else}
  // generic versions.
function Fplseek(fd : cint; offset : off_t; whence : cint): off_t; [public, alias : 'FPC_SYSC_LSEEK'];

{
this one is special for the return value being 64-bit..
hi/lo offset not yet tested.

NetBSD: ok, but implicit return value in edx:eax
FreeBSD: same implementation as NetBSD.
}

begin
  {$ifdef CPU64}
    Fplseek:=do_syscall(syscall_nr___syscall,syscall_nr_lseek,TSysParam(fd),0,Offset,whence);
  {$else}
    Fplseek:=do_syscall(syscall_nr___syscall,syscall_nr_lseek,0,TSysParam(fd),0,lo(Offset),{0} hi(offset),Whence);
  {$endif}
end;

function Fpftruncate(fd : cint; flength : off_t): cint; [public, alias : 'FPC_SYSC_FTRUNCATE'];

begin
 {$ifdef CPU64}
   Fpftruncate:=Do_syscall(syscall_nr___syscall,syscall_nr_ftruncate, fd  ,0   ,flength);
 {$else}
   Fpftruncate:=Do_syscall(syscall_nr___syscall,syscall_nr_ftruncate,0,fd,0,lo(flength),hi(flength));
 {$endif}

end;


Function Fpmmap(start:pointer;len:size_t;prot:cint;flags:cint;fd:cint;offst:off_t):pointer; [public, alias:  'FPC_SYSC_MMAP'];

begin
 {$ifdef CPU64}
<<<<<<< HEAD
<<<<<<< HEAD
  Fpmmap:=pointer(ptruint(do_syscall(TSysParam(syscall_nr_mmap),TSysParam(Start),TSysParam(Len),TSysParam(Prot),TSysParam(Flags),TSysParam(fd),0,TSysParam(offst))));
{$else}
 Fpmmap:=pointer(ptruint(do_syscall(syscall_nr_mmap,TSysParam(Start),Len,Prot,Flags,fd,0,
         {$ifdef FPC_BIG_ENDIAN}    hi(offst),lo(offst){$endif}
         {$ifdef FPC_LITTLE_ENDIAN} lo(offst),hi(offst){$endif}
         )));
=======
=======
>>>>>>> origin/fixes_2_2
  Fpmmap:=pointer(ptruint(do__syscall(TSysParam(syscall_nr_mmap),TSysParam(Start),TSysParam(Len),TSysParam(Prot),TSysParam(Flags),TSysParam(fd),TSysParam(offst),0,0)));
{$else}
  Fpmmap:=pointer(ptruint(do_syscall(syscall_nr_mmap,TSysParam(Start),Len,Prot,Flags,fd,
        {$ifdef FPC_BIG_ENDIAN}    hi(offst),lo(offst){$endif}
        {$ifdef FPC_LITTLE_ENDIAN} lo(offst),hi(offst){$endif},0
        )));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif}
end;

{$endif}


function Fpread(fd: cint; buf: pchar; nbytes : size_t): ssize_t; [public, alias : 'FPC_SYSC_READ'];

begin
  Fpread:=do_syscall(syscall_nr_read,Fd,TSysParam(buf),nbytes);
end;

function Fpwrite(fd: cint;buf:pchar; nbytes : size_t): ssize_t; [public, alias : 'FPC_SYSC_WRITE'];

begin
 Fpwrite:=do_syscall(syscall_nr_write,Fd,TSysParam(buf),nbytes);
end;

function Fpunlink(const path: pchar): cint; [public, alias : 'FPC_SYSC_UNLINK'];

begin
  Fpunlink:=do_syscall(syscall_nr_unlink,TSysParam(path));
end;

function Fprename(old : pchar; newpath: pchar): cint; [public, alias : 'FPC_SYSC_RENAME'];

begin
  Fprename:=do_syscall(syscall_nr_rename,TSysParam(old),TSysParam(newpath));
end;

function Fpstat(const path: pchar; var buf : stat):cint; [public, alias : 'FPC_SYSC_STAT'];

begin
 Fpstat:=do_syscall(syscall_nr_stat,TSysParam(path),TSysParam(@buf));
end;


{*****************************************************************************
               --- Directory:Directory related calls ---
*****************************************************************************}

function Fpchdir(path : pchar): cint; [public, alias : 'FPC_SYSC_CHDIR'];

begin
 Fpchdir:=do_syscall(syscall_nr_chdir,TSysParam(path));
end;

function Fpmkdir(path : pchar; mode: mode_t):cint; [public, alias : 'FPC_SYSC_MKDIR'];

begin {Mode is 16-bit on F-BSD 4!}
  Fpmkdir:=do_syscall(syscall_nr_mkdir,TSysParam(path),mode);
end;

function Fprmdir(path : pchar): cint;  [public, alias : 'FPC_SYSC_RMDIR'];

begin
 Fprmdir:=do_syscall(syscall_nr_rmdir,TSysParam(path));
end;

{$ifndef NewReaddir}

const DIRBLKSIZ=1024;


function Fpopendir(dirname : pchar): pdir;  [public, alias : 'FPC_SYSC_OPENDIR'];

var
  fd:longint;
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
   Begin
    Errno:=-1;
    exit;
   End;
  new(ptr);
  if ptr=nil then
   Begin
    Errno:=1;
    exit;
   End;
  Getmem(ptr^.dd_buf,2*DIRBLKSIZ);
  if ptr^.dd_buf=nil then
   exit;
  ptr^.dd_fd:=fd;
  ptr^.dd_loc:=-1;
  ptr^.dd_rewind:=ptrint(ptr^.dd_buf);
  ptr^.dd_size:=0;
//  ptr^.dd_max:=sizeof(ptr^.dd_buf^);
  Fpopendir:=ptr;
end;

function Fpclosedir(dirp : pdir): cint; [public, alias : 'FPC_SYSC_CLOSEDIR'];

begin
  Fpclosedir:=Fpclose(dirp^.dd_fd);
  Freemem(dirp^.dd_buf);
  dispose(dirp);
end;

var
  use_openbsd_getdirentries_49 : boolean = false;
  use_getdirentries_syscall : boolean = true;

function Fpreaddir(dirp : pdir) : pdirent; [public, alias : 'FPC_SYSC_READDIR'];

{Different from Linux, Readdir on BSD is based on Getdents, due to the
missing of the readdir syscall.
Getdents requires the buffer to be larger than the blocksize.
This usually the sectorsize =512 bytes, but maybe tapedrives and harddisks
with blockmode have this higher?}

function readbuffer:longint;

var retval :longint;
{$ifdef FPC_USE_GETDIRENTRIES_SYSCALL}
    basepp : pointer;
{$ifdef FPC_USE_GETDIRENTRIES_I49_SYSCALL}
  { OpenBSD i49 getDirEntries system call uses off_t type for last parameter }
    basep_off_t : off_t;
{$endif not FPC_USE_GETDIRENTRIES_I49_SYSCALL}
    basep : clong;
{$endif FPC_USE_GETDIRENTRIES_SYSCALL}
begin
{$ifdef FPC_USE_GETDIRENTRIES_SYSCALL}
{$ifdef FPC_USE_GETDIRENTRIES_I49_SYSCALL}
 if use_openbsd_getdirentries_49 then
   basepp:=@basep_off_t
 else
{$endif FPC_USE_GETDIRENTRIES_I49_SYSCALL}
   basepp:=@basep;
 if use_getdirentries_syscall then
   Retval:=do_syscall(syscall_nr_getdirentries,TSysParam(dirp^.dd_fd),TSysParam(@dirp^.dd_buf^),DIRBLKSIZ {sizeof(getdentsbuffer)},TSysParam(basepp))
 else
 Retval:=do_syscall(syscall_nr_getdents,TSysParam(dirp^.dd_fd),TSysParam(@dirp^.dd_buf^),DIRBLKSIZ {sizeof(getdentsbuffer)});
{$else not FPC_USE_GETDIRENTRIES_SYSCALL}
 Retval:=do_syscall(syscall_nr_getdents,TSysParam(dirp^.dd_fd),TSysParam(@dirp^.dd_buf^),DIRBLKSIZ {sizeof(getdentsbuffer)});
{$endif not FPC_USE_GETDIRENTRIES_SYSCALL}
   dirp^.dd_rewind:=TSysParam(dirp^.dd_buf);
   if retval=0 then
    begin
     dirp^.dd_rewind:=0;
     dirp^.dd_loc:=0;
    end
   else
    dirP^.dd_loc:=retval;
   dirP^.dd_size:=retval;
 readbuffer:=retval;
end;

var
    FinalEntry     : pdirent;
    novalid        : boolean;
    Reclen         : Longint;
    CurEntry       : PDirent;

begin
 if (dirp^.dd_buf=nil) or (dirp^.dd_loc=0) THEN
  exit(nil);
 if (dirp^.dd_loc=-1)   OR     {First readdir on this pdir. Initial fill of buffer}
   (dirp^.dd_rewind>=(ptrint(dirp^.dd_buf)+dirp^.dd_size)) then  {no more entries left?}
  Begin
    if readbuffer=0 then        {succesful read?}
     Exit(NIL);                 {No more data}
  End;
 FinalEntry:=NIL;
 CurEntry:=nil;
 repeat
  novalid:=false;
  CurEntry:=pdirent(dirp^.dd_rewind);
{$ifdef dragonfly}
  RecLen:=(CurEntry^.d_namlen + 24) and $FFFFFFF8;
{$else}
  RecLen:=CurEntry^.d_reclen;
{$endif}
  if RecLen<>0 Then
   begin {valid direntry?}
    if CurEntry^.d_fileno<>0 then
     FinalEntry:=CurEntry;
    inc(dirp^.dd_rewind,Reclen);
   end
  else
   begin {block entirely searched or reclen=0}
    Novalid:=True;
    if dirp^.dd_loc<>0 THEN             {blocks left?}
     if readbuffer()<>0 then        {succesful read?}
      novalid:=false;
   end;
 until (FinalEntry<>nil) or novalid;
 If novalid then
  FinalEntry:=nil;
 FpReadDir:=FinalEntry;
end;
{$endif}

{*****************************************************************************
        --- Process:Process & program handling - related calls ---
*****************************************************************************}

procedure Fpexit(status : cint); [public, alias : 'FPC_SYSC_EXIT'];

begin
  do_syscall(syscall_nr_exit,status);
end;

{
  Change action of process upon receipt of a signal.
  Signum specifies the signal (all except SigKill and SigStop).
  If Act is non-nil, it is used to specify the new action.
  If OldAct is non-nil the previous action is saved there.
}

{$ifdef USE_SIGACTION_SIGTRAMP}
  procedure signal_trampoline; cdecl; forward;
{$endif def USE_SIGACTION_SIGTRAMP}

function Fpsigaction(sig: cint; act, oact: psigactionrec): cint; [public, alias : 'FPC_SYSC_SIGACTION'];

{
  Change action of process upon receipt of a signal.
  Signum specifies the signal (all except SigKill and SigStop).
  If Act is non-nil, it is used to specify the new action.
  If OldAct is non-nil the previous action is saved there.
}

begin
{$ifdef USE_SIGACTION_SIGTRAMP}
  fpsigaction:=do_syscall(syscall_nr___sigaction_sigtramp,TSysParam(sig),TSysParam(act),TSysParam(oact),
                          TSysParam(@signal_trampoline),2);
{$else not USE_SIGACTION_SIGTRAMP}
  fpsigaction:=do_syscall(syscall_nr_sigaction,TSysParam(sig),TSysParam(act),TSysParam(oact));
{$endif not USE_SIGACTION_SIGTRAMP}
end;

(*=================== MOVED from sysunix.inc ========================*)


function Fpfstat(fd : cint; var sb : stat): cint;  [public, alias : 'FPC_SYSC_FSTAT'];

begin
  fpFStat:=do_SysCall(syscall_nr_fstat,fd,TSysParam(@sb));
end;

{$ifdef NewReaddir}
{$I readdir.inc}
{$endif}

function Fpfork : pid_t;  [public, alias : 'FPC_SYSC_FORK'];
{
  This function issues the 'fork' System call. the program is duplicated in memory
  and Execution continues in parent and child process.
  In the parent process, fork returns the PID of the child. In the child process,
  zero is returned.
  A negative value indicates that an error has occurred, the error is returned in
  LinuxError.
}

Begin
 Fpfork:=Do_syscall(SysCall_nr_fork);
End;

{
function Fpexecve(const path : pathstr; const argv : ppchar; const envp: ppchar): cint;
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
function Fpexecve(const path : pchar; const argv : ppchar; const envp: ppchar): cint;  [public, alias : 'FPC_SYSC_EXECVE'];
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

begin // actually a wait4() call with 4th arg 0.
 FpWaitPID:=do_syscall(syscall_nr_WaitPID,PID,TSysParam(Stat_loc),options,0);
end;

function Fpaccess(const pathname : pchar; amode : cint): cint; [public, alias : 'FPC_SYSC_ACCESS'];
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
 FpAccess:=do_syscall(syscall_nr_access,TSysParam(pathname),amode);
end;
(*
function Fpaccess(const pathname : pathstr; amode : cint): cint;

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
 Fpdup2:=do_syscall(syscall_nr_dup2,TSysParam(fildes),TSysParam(fildes2));
end;



Function Fpmunmap(start:pointer;len:size_t):cint;    [public, alias :'FPC_SYSC_MUNMAP'];
begin
  Fpmunmap:=do_syscall(syscall_nr_munmap,TSysParam(start),Len);
end;


{
  Interface to Unix ioctl call.
  Performs various operations on the filedescriptor Handle.
  Ndx describes the operation to perform.
  Data points to data needed for the Ndx function. The structure of this
  data is function-dependent.
}

Function FpIOCtl(Handle:cint;Ndx: TIOCtlRequest;Data: Pointer):cint; [public, alias : 'FPC_SYSC_IOCTL'];
// This was missing here, instead hardcoded in Do_IsDevice
begin
  FpIOCtl:=do_SysCall(syscall_nr_ioctl,handle,Ndx,TSysParam(data));
end;


Function FpGetPid:LongInt;   [public, alias : 'FPC_SYSC_GETPID'];
{
  Get Process ID.
}

begin
 FpGetPID:=do_syscall(syscall_nr_getpid);
end;

function fpgettimeofday(tp: ptimeval;tzp:ptimezone):cint; [public, alias: 'FPC_SYSC_GETTIMEOFDAY'];

begin
 fpgettimeofday:=do_syscall(syscall_nr_gettimeofday,TSysParam(tp),TSysParam(tzp));
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

{$ifdef OpenBSD}
  { OpenBSD sigprocmask signal uses
    sigset_t that are cint type
    the value of nset^[0] must be passed as second parameter
    the old mask value is in return value }
  { How do we know if the call failed... PM }
{$define OS_SIGPROCMASK_RETURNS_OVAL}
{$endif}
{$ifdef OS_SIGPROCMASK_RETURNS_OVAL}
  var
    res : cint;
{$endif OS_SIGPROCMASK_RETURNS_OVAL}
begin
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef OS_SIGPROCMASK_RETURNS_OVAL}
  res:=do_syscall(syscall_nr_sigprocmask,tsysparam(how),TSysParam(nset^[0]));
  if assigned(oset) then
    oset^[0]:=res;
  FPsigprocmask:=0;

{$else OS_SIGPROCMASK_RETURNS_OVAL}
  FPsigprocmask:=do_syscall(syscall_nr_sigprocmask,tsysparam(how),TSysParam(nset),TSysParam(oset));
{$endif OS_SIGPROCMASK_RETURNS_OVAL}
=======
  FPsigprocmask:=do_syscall(syscall_nr_sigprocmask,tsysparam(how),TSysParam(nset),TSysParam(oset));
>>>>>>> graemeg/fixes_2_2
end;
{$warning user BLA!}
=======
  FPsigprocmask:=do_syscall(syscall_nr_sigprocmask,tsysparam(how),TSysParam(nset),TSysParam(oset));
end;
{$user BLA!}
>>>>>>> origin/fixes_2_2
Function FpNanoSleep(req : ptimespec;rem : ptimespec) : cint; [public, alias : 'FPC_SYSC_NANOSLEEP'];
begin
  FpNanoSleep:=Do_SysCall(syscall_nr_nanosleep,TSysParam(req),TSysParam(rem));
end;

function Fpgetcwd(pt:pchar; _size:size_t):pchar;[public, alias :'FPC_SYSC_GETCWD'];
const intpathmax = 1024-4;      // didn't use POSIX data in libc
                                // implementation.
var ept,bpt : pchar;
    c       : char;
    ret     : cint;

begin
   if pt=NIL Then
    begin
      // POSIX: undefined. (exit(nil) ?)
      // BSD  : allocate mem for path.
      getmem(pt,intpathmax);
      if pt=nil Then
        exit(nil);
      ept:=pt+intpathmax;
    end
   else
    Begin
      if (_size=0) Then
        Begin
          seterrno(ESysEINVAL);
          exit(nil);
        End;
      if (_size=1) Then
        Begin
          seterrno(ESysERANGE);
          exit(nil);
        End;
      ept:=pt+_size;
    end;

    ret := do_syscall(syscall_nr___getcwd,TSysParam(pt),TSysParam( ept - pt));
    If (ret = 0) Then
      begin
        If (pt[0] <> '/') Then
           Begin
             bpt := pt;
             ept := pt + strlen(pt) - 1;
             While (bpt < ept) Do
               Begin
                 c := bpt^;
                 bpt^:=ept^;
                 inc(bpt);
                 ept^:=c;
                 dec(ept);
               End;
           End
      end
{$if defined(openbsd) or defined (netbsd)}
   { At least for openbsd, a positive return value is
     the length of the returned pchar }
   else if (ret<0) then
{$else not opensd}
   else
{$endif openbsd}
     begin
       seterrno(-ret);
       pt:=nil;
     end;
 Fpgetcwd:=pt;
end;

Function fpReadLink(name,linkname:pchar;maxlen:size_t):cint; [public, alias : 'FPC_SYSC_READLINK'];

begin
  fpreadlink:=do_syscall(syscall_nr_readlink, TSysParam(name),TSysParam(linkname),maxlen);
end;

function FpGetRLimit(resource:cint;rlim:PRLimit):cint; [public, alias : 'FPC_SYSC_GETRLIMIT'];
begin
  fpgetrlimit:=do_syscall(syscall_nr_getrlimit,TSysParam(Resource),TSysParam(rlim));
end;

function FpSetRLimit(Resource:cint;rlim:PRLimit):cint; [public, alias : 'FPC_SYSC_SETRLIMIT'];
begin
  fpsetrlimit:=do_syscall(syscall_nr_setrlimit,TSysParam(Resource),TSysParam(rlim));
end;

