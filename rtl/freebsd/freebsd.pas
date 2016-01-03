Unit FreeBSD;
{
   This file is part of the Free Pascal run time library.
   (c) 2005 by Marco van de Voort
   member of the Free Pascal development team.
   based on the sendfile conversion of Ales Katona 30.01.2006

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   Unit for FreeBSD specific calls. Calls may move to "BSD" unit in time,
   if turns out that more BSDs include them. 
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

{$IFDEF FPC}
  {$PACKRECORDS C}
  {$inline on}
  {$Macro On}
  {$ifdef FPC_USE_LIBC}
     {$define extdecl:=cdecl; external 'c'}
  {$else}
     {$define extdecl:=inline}
  {$endif}
{$ENDIF}
              
interface

uses
  BaseUnix;

const
  SF_NODISKIO = $00000001;  // don't wait for disk IO, similar to non-blocking socket setting
  
  // kernel threads

  KSE_VER_0        = 0;
  KSE_VERSION      = KSE_VER_0;

  {* These flags are kept in km_flags *}
  KMF_NOUPCALL      = $01;
  KMF_NOCOMPLETED   = $02;
  KMF_DONE          = $04;
  KMF_BOUND         = $08;
  KMF_WAITSIGEVENT  = $10;

  {* These flags are kept in tm_flags *}
  TMF_NOUPCALL      = $01;

  {* These flags are kept in tm_dlfags *}
  TMDF_SSTEP        = $01;
  TMDF_SUSPEND      = $02;

  {* Flags for kse_switchin *}
  KSE_SWITCHIN_SETTMBX = $01;

  {* Commands for kse_thr_interrupt *}
  KSE_INTR_INTERRUPT   = 1;
  KSE_INTR_RESTART     = 2;
  KSE_INTR_SENDSIG     = 3;
  KSE_INTR_SIGEXIT     = 4;
  KSE_INTR_DBSUSPEND   = 5;
  KSE_INTR_EXECVE      = 6;
  
{$i ucontexth.inc} // required for kse threads

Type  
  SF_HDTR = record
    headers: PIOVec;        {* pointer to an array of header struct iovec's *}
    hdr_cnt: cint;          {* number of header iovec's *}
    trailers: PIOVec;       {* pointer to an array of trailer struct iovec's *}
    trl_cnt: cint;          {* number of trailer iovec's *}
  end;
  TSF_HDTR = SF_HDTR;
  PSF_HDTR = ^TSF_HDTR;
  
  kld_file_stat = record
    Version: cInt;            {* set to sizeof(linker_file_stat) *}
    Name: array[0..MAXPATHLEN-1] of Char;
    Refs: cInt;
    ID: cInt;
    Address: pChar;           {* load address *}
    Size: size_t;             {* size in bytes *}
  end;
  tkld_file_stat = kld_file_stat;
  pkld_file_stat = ^kld_file_stat;
  TKldFileStat = kld_file_stat;
  PKldFileStat = ^kld_file_stat;
  
  kld_sym_lookup = record
    Version: cInt;            {* sizeof(struct kld_sym_lookup) *}
    SymName: pChar;           {* Symbol name we are looking up *}
    SymValue: culong;
    SymSize: size_t;
  end;
  tkld_sym_lookup = kld_sym_lookup;
  pkld_sym_lookup = ^kld_sym_lookup;
  TKldSymLookup = kld_sym_lookup;
  PKldSymLookup = ^kld_sym_lookup;
  
  // kernel threads

  pkse_mailbox = ^kse_mailbox;
  
  pkse_func_t = ^kse_func_t;
  kse_func_t = procedure(mbx: pkse_mailbox);
  TKseFunc = kse_func_t;
  PKseFunc = pkse_func_t;
  
  {*
   * Thread mailbox.
   *
   * This describes a user thread to the kernel scheduler.
   *}
  pkse_thr_mailbox = ^kse_thr_mailbox;
  kse_thr_mailbox = record
    tm_context: ucontext_t;	  {* User and machine context *}
    tm_flags: cuint32;            {* Thread flags *}
    tm_next: pkse_thr_mailbox;    {* Next thread in list *}
    tm_udata: Pointer;            {* For use by the UTS *}
    tm_uticks: cuint32;           {* Time in userland *}
    tm_sticks: cuint32;           {* Time in kernel *}
    tm_syncsig: siginfo_t;
    tm_dflags: cuint32;           {* Debug flags *}
    tm_lwp: lwpid_t;              {* kernel thread UTS runs on *}
    __spare__: array [0..5] of cuint32;
  end;
  TKseThrMailBox = kse_thr_mailbox;
  PKseThrMailBox = pkse_thr_mailbox;

  
  {*
   * KSE mailbox.
   *
   * Communication path between the UTS and the kernel scheduler specific to
   * a single KSE.
   *}
   
  kse_mailbox = record
    km_version: cuint32;             {* Mailbox version *}
    km_curthread: pkse_thr_mailbox;  {* Currently running thread *}
    km_completed: pkse_thr_mailbox;  {* Threads back from kernel *}
    km_sigscaught: sigset_t;	     {* Caught signals *}
    km_flags: cuint32;               {* Mailbox flags *}
    km_func: pkse_func_t;            {* UTS function *}
    km_stack: stack_t;               {* UTS stack *}
    km_udata: Pointer;               {* For use by the UTS *}
    km_timeofday: TTimeSpec;         {* Time of day *}
    km_quantum: cuint32;             {* Upcall quantum in msecs *}
    km_lwp: lwpid_t;                 {* kernel thread UTS runs on *}
    __spare2__: array[0..6] of cuint32
  end;
  TKseMailBox = kse_mailbox;
  PKseMailBox = pkse_mailbox;


  function sendfile(fd: cint; s: cint; Offset: TOff; nBytes: TSize;
                      HDTR: PSF_HDTR; sBytes: POff; Flags: cint): cint; extdecl;
                      
  // Kernel modules support
                    
  function kldload(FileName: pChar): cInt; extdecl;

  function kldunload(fileid: cInt): cInt; extdecl;

  function kldfind(FileName: pChar): cInt; extdecl;

  function kldnext(fileid: cInt): cInt; extdecl;

  function kldstat(fileid: cInt; kld_file_stat: pKldFileStat): cInt; extdecl;

  function kldfirstmod(fileid: cInt): cInt; extdecl;

  function kldsym(fileid: cInt; command: cInt; data: PKldSymLookup): cInt; extdecl;
  
  // kernel threads support
  
  function kse_exit: cInt; extdecl;
  function kse_wakeup(mbx: PKseMailBox): cInt; extdecl;
  function kse_create(mbx: PKseMailBox; newgroup: cInt): cInt; extdecl;
  function kse_thr_interrupt(tmbx: PKseThrMailBox; cmd: cInt; data: cLong): cInt; extdecl;
  function kse_release(timeout: PTimeSpec): cInt; extdecl;
  function kse_switchin(tmbx: PKseThrMailBox; flags: cInt): cInt; extdecl;

<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_USE_LIBC}
function fpgetfsstat(buf:pstatfs;bufsize:clong;flags:cint):cint;
{$endif} 

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Const
 MAP_FILE         = $0000;  { map from file (default) }
 MAP_ANON         = $1000;  { allocated from memory, swap space }
   
 MAP_RENAME       = $0020; { Sun: rename private pages to file }
 MAP_NORESERVE    = $0040; { Sun: don't reserve needed swap area }
 //  MAP_INHERIT      = $0080; { region is retained after exec. not anymore in 5.x? }
 //  MAP_NOEXTEND     = $0100; { for MAP_FILE, don't change file size. not anymore in 5.x? }
 MAP_HASSEMAPHORE = $0200; { region may contain semaphores }
 MAP_STACK        = $0400; { region grows down, like a stack }
 MAP_NOSYNC       = $0800; { page to but do not sync underlying file}
 MAP_NOCORE       = $20000;{ dont include these pages in a coredump}

<<<<<<< HEAD
<<<<<<< HEAD

function kernproc_getgenvalue(pid:pid_t;kernproc_variable:cint;var s:ansistring):cint;
function kernproc_getargs(pid:pid_t;var fn:ansistring):cint;
function kernproc_getpath(pid:pid_t;var fn:ansistring):cint;


// Taken from /usr/include/time.h
const
  CLOCK_REALTIME = 0;
  CLOCK_VIRTUAL = 1;
  CLOCK_PROF = 2;
  CLOCK_MONOTONIC = 4;
  // FreeBSD specific
  CLOCK_UPTIME = 5;
  CLOCK_UPTIME_PRECISE = 7;
  CLOCK_UPTIME_FAST = 8;
  CLOCK_REALTIME_PRECISE = 9;
  CLOCK_REALTIME_FAST = 10;
  CLOCK_MONOTONIC_PRECISE = 11;
  CLOCK_MONOTONIC_FAST = 12;
  CLOCK_SECOND = 13;
  CLOCK_THREAD_CPUTIME_ID = 14;
  CLOCK_PROCESS_CPUTIME_ID = 15;

Type
  clockid_t = cint;

function clock_getres(clk_id: clockid_t; res: ptimespec): cint; {$ifdef FPC_USE_LIBC} cdecl; external name 'clock_getres'; {$ENDIF}
function clock_gettime(clk_id: clockid_t; tp: ptimespec): cint;  {$ifdef FPC_USE_LIBC} cdecl; external name 'clock_gettime'; {$ENDIF}
function clock_settime(clk_id: clockid_t; tp: ptimespec): cint; {$ifdef FPC_USE_LIBC} cdecl; external name 'clock_settime'; {$ENDIF}


<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/graemeg/clock_gettime
implementation

Uses Sysctl,
{$ifndef FPC_USE_LIBC}  SysCall; {$else} InitC; {$endif}

{$IFNDEF FPC_USE_LIBC}

function SendFile(fd: cint; s: cint; Offset: TOff; nBytes: TSize;
                  HDTR: PSF_HDTR; sBytes: POff; Flags: cint): cint;
begin
  SendFile:=Do_Syscall(syscall_nr_sendfile, fd, s,
 {$IFNDEF CPU64} 
   {$IFDEF LITTLE_ENDIAN} // little endian is lo - hi
      Lo(Offset), Hi(Offset), 
   {$ELSE}  	          // big endian is hi - lo
      Hi(Offset), Lo(Offset), 
   {$ENDIF}
 {$ELSE}  // 64-bit doesn't care. 
    TSysParam(Offset),
 {$ENDIF}
    nBytes, TSysParam(HDTR), TSysParam(sBytes), Flags);
end;

// kernel modules

function kldload(FileName: pChar): cInt;
begin
  kldload:=do_sysCall(syscall_nr_kldload, TSysParam(FileName));
end;

function kldunload(fileid: cInt): cInt;
begin
  kldunload:=do_sysCall(syscall_nr_kldunload, TSysParam(fileid));
end;

function kldfind(FileName: pChar): cInt;
begin
  kldfind:=do_sysCall(syscall_nr_kldfind, TSysParam(FileName));
end;

function kldnext(fileid: cInt): cInt;
begin
  kldnext:=do_sysCall(syscall_nr_kldnext, TSysParam(fileid));
end;

function kldstat(fileid: cInt; kld_file_stat: pKldFileStat): cInt;
begin
  kldstat:=do_sysCall(syscall_nr_kldstat, TSysParam(fileid),
                                          TSysParam(kld_file_stat));
end;

function kldfirstmod(fileid: cInt): cInt;
begin
  kldfirstmod:=do_sysCall(syscall_nr_kldfirstmod, TSysParam(fileid));
end;

function kldsym(fileid: cInt; command: cInt; data: PKldSymLookup): cInt;
begin
  kldsym:=do_sysCall(syscall_nr_kldsym, TSysParam(fileid), TSysParam(command),
                     TSysParam(data));
end;

// kernel threads

function kse_exit: cInt;
begin
  kse_exit:=do_sysCall(TSysParam(syscall_nr_kse_exit));
end;

function kse_wakeup(mbx: PKseMailBox): cInt;
begin
  kse_wakeup:=do_sysCall(syscall_nr_kse_wakeup, TSysParam(mbx));
end;

function kse_create(mbx: PKseMailBox; newgroup: cInt): cInt;
begin
  kse_create:=do_sysCall(syscall_nr_kse_create, TSysParam(mbx),
                         TSysParam(newgroup));
end;

function kse_thr_interrupt(tmbx: PKseThrMailBox; cmd: cInt; data: cLong): cInt;
begin
  kse_thr_interrupt:=do_SysCall(syscall_nr_kse_thr_interrupt, TSysParam(tmbx),
                               TSysParam(cmd), TSysParam(data));
end;

function kse_release(timeout: PTimeSpec): cInt;
begin
  kse_release:=do_SysCall(syscall_nr_kse_release, TSysParam(timeout));
end;

function kse_switchin(tmbx: PKseThrMailBox; flags: cInt): cInt;
begin
  kse_switchin:=do_SysCall(syscall_nr_kse_switchin, TSysParam(tmbx), TSysParam(flags));
end;

function fpgetfsstat(buf:pstatfs;bufsize:clong;flags:cint):cint;
begin
  fpgetfsstat:=do_syscall(syscall_nr_getfsstat,TSysParam(buf),TSysParam(Bufsize),TSysParam(Flags));
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/graemeg/clock_gettime

function clock_getres(clk_id: clockid_t; res: ptimespec): cint;
begin
  clock_getres := do_SysCall(syscall_nr_clock_getres,tsysparam(clk_id),tsysparam(res));
end;

function clock_gettime(clk_id: clockid_t; tp: ptimespec): cint;
begin
  clock_gettime := do_SysCall(syscall_nr_clock_gettime,tsysparam(clk_id),tsysparam(tp));
end;

function clock_settime(clk_id: clockid_t; tp: ptimespec): cint;
begin
  clock_settime := do_SysCall(syscall_nr_clock_settime,tsysparam(clk_id),tsysparam(tp));
end;

<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/graemeg/clock_gettime
{$ENDIF}

function kernproc_getgenvalue(pid:pid_t;kernproc_variable:cint;var s:ansistring):cint;

var mib: array[0..3] of cint;
    argv_len : size_t;
    ret:cint;
Begin
  mib[0]:=CTL_KERN;
  mib[1]:=kern_proc;
  Mib[2]:=kernproc_variable;
  Mib[3]:=pid;
  setlength(s,arg_max);
  argv_len:=ARG_MAX;
  ret:=fpsysctl(@mib, high(mib)+1, @s[1], @argv_len, NIL, 0);
  if ret<>-1 then
    setlength(s,argv_len)
   else
    setlength(s,0);
  kernproc_getgenvalue:=ret;
end;

function kernproc_getargs(pid:pid_t;var fn:ansistring):cint;
begin
  kernproc_getargs:=kernproc_getgenvalue(pid,KERN_PROC_ARGS,fn);
end;

function kernproc_getpath(pid:pid_t;var fn:ansistring):cint;
begin
  kernproc_getpath:=kernproc_getgenvalue(pid,KERN_PROC_PATHNAME,fn);
end;

end.
