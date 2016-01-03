{
   This file is part of the Free Pascal run time library.
   Copyright (c) 1999-2000 by Michael Van Canneyt,
   BSD parts (c) 2000 by Marco van de Voort
   members of the Free Pascal development team.

   New linux unit. Linux only calls only. Will be renamed to linux.pp
   when 1.0.x support is killed off.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}
unit Linux;

{$i osdefs.inc}

{$packrecords c}
{$ifdef FPC_USE_LIBC} 
 {$linklib rt} // for clock* functions
{$endif}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
  BaseUnix, unixtype;
=======
  BaseUnix;//, ctypes;
>>>>>>> graemeg/fixes_2_2
=======
  BaseUnix;//, ctypes;
>>>>>>> origin/fixes_2_2

type
  TSysInfo = record
    uptime: clong;                     //* Seconds since boot */
    loads: array[0..2] of culong;      //* 1, 5, and 15 minute load averages */
    totalram: culong;                  //* Total usable main memory size */
    freeram: culong;                   //* Available memory size */
    sharedram: culong;                 //* Amount of shared memory */
    bufferram: culong;                 //* Memory used by buffers */
    totalswap: culong;                 //* Total swap space size */
    freeswap: culong;                  //* swap space still available */
    procs: cushort;                    //* Number of current processes */
    pad: cushort;                      //* explicit padding for m68k */
    totalhigh: culong;                 //* Total high memory size */
    freehigh: culong;                  //* Available high memory size */
    mem_unit: cuint;                   //* Memory unit size in bytes */
{$ifndef cpu64}
    { the upper bound of the array below is negative for 64 bit cpus }
    _f: array[0..19-2*sizeof(clong)-sizeof(cint)] of cChar;  //* Padding: libc5 uses this.. */
{$endif cpu64}
  end;
  PSysInfo = ^TSysInfo;

function Sysinfo(Info: PSysinfo): cInt; {$ifdef FPC_USE_LIBC} cdecl; external name 'sysinfo'; {$endif}

const
  CSIGNAL              = $000000ff; // signal mask to be sent at exit
  CLONE_VM             = $00000100; // set if VM shared between processes
  CLONE_FS             = $00000200; // set if fs info shared between processes
  CLONE_FILES          = $00000400; // set if open files shared between processes
  CLONE_SIGHAND        = $00000800; // set if signal handlers shared
  CLONE_PID            = $00001000; // set if pid shared
  CLONE_PTRACE         = $00002000; // Set if tracing continues on the child.
  CLONE_VFORK          = $00004000; // Set if the parent wants the child to wake it up on mm_release.
  CLONE_PARENT         = $00008000; // Set if we want to have the same parent as the cloner.
  CLONE_THREAD         = $00010000; // Set to add to same thread group.
  CLONE_NEWNS          = $00020000; // Set to create new namespace.
  CLONE_SYSVSEM        = $00040000; // Set to shared SVID SEM_UNDO semantics.
  CLONE_SETTLS         = $00080000; // Set TLS info.
  CLONE_PARENT_SETTID  = $00100000; // Store TID in userlevel buffer before MM copy.
  CLONE_CHILD_CLEARTID = $00200000; // Register exit futex and memory location to clear.
  CLONE_DETACHED       = $00400000; // Create clone detached.
  CLONE_UNTRACED       = $00800000; // Set if the tracing process can't force CLONE_PTRACE on this clone.
  CLONE_CHILD_SETTID   = $01000000; // Store TID in userlevel buffer in the child.
  CLONE_STOPPED        = $02000000; // Start in stopped state.


  FUTEX_WAIT            = 0;
  FUTEX_WAKE            = 1;
  FUTEX_FD              = 2;
  FUTEX_REQUEUE         = 3;
  FUTEX_CMP_REQUEUE     = 4;
  FUTEX_WAKE_OP         = 5;
  FUTEX_LOCK_PI         = 6;
  FUTEX_UNLOCK_PI       = 7;
  FUTEX_TRYLOCK_PI      = 8;

  FUTEX_OP_SET          = 0;   // *(int *)UADDR2 = OPARG;
  FUTEX_OP_ADD          = 1;   // *(int *)UADDR2 += OPARG;
  FUTEX_OP_OR           = 2;   // *(int *)UADDR2 |= OPARG;
  FUTEX_OP_ANDN         = 3;   // *(int *)UADDR2 &= ~OPARG;
  FUTEX_OP_XOR          = 4;   // *(int *)UADDR2 ^= OPARG;

  FUTEX_OP_OPARG_SHIFT  = 8;   // Use (1 << OPARG) instead of OPARG.

  FUTEX_OP_CMP_EQ       = 0;   // if (oldval == CMPARG) wake
  FUTEX_OP_CMP_NE       = 1;   // if (oldval != CMPARG) wake
  FUTEX_OP_CMP_LT       = 2;   // if (oldval < CMPARG) wake
  FUTEX_OP_CMP_LE       = 3;   // if (oldval <= CMPARG) wake
  FUTEX_OP_CMP_GT       = 4;   // if (oldval > CMPARG) wake
  FUTEX_OP_CMP_GE       = 5;   // if (oldval >= CMPARG) wake

{ FUTEX_WAKE_OP will perform atomically
   int oldval = *(int *)UADDR2;
   *(int *)UADDR2 = oldval OP OPARG;
   if (oldval CMP CMPARG)
     wake UADDR2; }

{$ifndef FPC_USE_LIBC}
<<<<<<< HEAD
<<<<<<< HEAD
function futex(uaddr:Pcint;op,val:cint;timeout:Ptimespec;addr2:Pcint;val3:cint):cint;{$ifdef SYSTEMINLINE}inline;{$endif}
function futex(var uaddr;op,val:cint;timeout:Ptimespec;var addr2;val3:cint):cint;{$ifdef SYSTEMINLINE}inline;{$endif}
function futex(var uaddr;op,val:cint;var timeout:Ttimespec;var addr2;val3:cint):cint;{$ifdef SYSTEMINLINE}inline;{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// general aliases:
function futex(uaddr:Pcint;op,val:cint;timeout:Ptimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}
function futex(var uaddr;op,val:cint;timeout:Ptimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}
function futex(var uaddr;op,val:cint;var timeout:Ttimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}
{$else}
// futex is currently not exposed by glibc
//function futex(uaddr:Pcint;op,val:cint;timeout:Ptimespec;addr2:Pcint;val3:cint):cint; cdecl; external name 'futex';
//function futex(var uaddr;op,val:cint;timeout:Ptimespec;var addr2;val3:cint):cint; cdecl; external name 'futex';
//function futex(var uaddr;op,val:cint;var timeout:Ttimespec;var addr2;val3:cint):cint; cdecl; external name 'futex';
{$endif}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$else}
function futex(uaddr:Pcint;op,val:cint;timeout:Ptimespec;addr2:Pcint;val3:cint):cint; cdecl; external name 'futex';
function futex(var uaddr;op,val:cint;timeout:Ptimespec;var addr2;val3:cint):cint; cdecl; external name 'futex';
function futex(var uaddr;op,val:cint;var timeout:Ttimespec;var addr2;val3:cint):cint; cdecl; external name 'futex';
{$endif}
function futex(uaddr:Pcint;op,val:cint;timeout:Ptimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}
function futex(var uaddr;op,val:cint;timeout:Ptimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}
function futex(var uaddr;op,val:cint;var timeout:Ttimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}
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

{$ifndef FPC_USE_LIBC}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function futex_op(op, oparg, cmp, cmparg: cint): cint; {$ifdef SYSTEMINLINE}inline;{$endif}
{$endif}

const
  POLLMSG     = $0400;
  POLLREMOVE  = $1000;
  POLLRDHUP   = $2000;

  EPOLLIN  = $01; { The associated file is available for read(2) operations. }
  EPOLLPRI = $02; { There is urgent data available for read(2) operations. }
  EPOLLOUT = $04; { The associated file is available for write(2) operations. }
  EPOLLERR = $08; { Error condition happened on the associated file descriptor. }
  EPOLLHUP = $10; { Hang up happened on the associated file descriptor. }
  EPOLLONESHOT = $40000000; { Sets the One-Shot behaviour for the associated file descriptor. }
  EPOLLET  = $80000000; { Sets  the  Edge  Triggered  behaviour  for  the  associated file descriptor. }

  { Valid opcodes ( "op" parameter ) to issue to epoll_ctl }
  EPOLL_CTL_ADD = 1;
  EPOLL_CTL_DEL = 2;
  EPOLL_CTL_MOD = 3;

  {Some console iotcl's.}
  GIO_FONT        = $4B60;  {gets font in expanded form}
  PIO_FONT        = $4B61;  {use font in expanded form}
  GIO_FONTX       = $4B6B;  {get font using struct consolefontdesc}
  PIO_FONTX       = $4B6C;  {set font using struct consolefontdesc}
  PIO_FONTRESET   = $4B6D;  {reset to default font}
  GIO_CMAP        = $4B70;  {gets colour palette on VGA+}
  PIO_CMAP        = $4B71;  {sets colour palette on VGA+}
  KIOCSOUND       = $4B2F;  {start sound generation (0 for off)}
  KDMKTONE        = $4B30;  {generate tone}
  KDGETLED        = $4B31;  {return current led state}
  KDSETLED        = $4B32;  {set led state [lights, not flags]}
  KDGKBTYPE       = $4B33;  {get keyboard type}
  KDADDIO         = $4B34;  {add i/o port as valid}
  KDDELIO         = $4B35;  {del i/o port as valid}
  KDENABIO        = $4B36;  {enable i/o to video board}
  KDDISABIO       = $4B37;  {disable i/o to video board}
  KDSETMODE       = $4B3A;  {set text/graphics mode}
  KDGETMODE       = $4B3B;  {get current mode}
  KDMAPDISP       = $4B3C;  {map display into address space}
  KDUNMAPDISP     = $4B3D;  {unmap display from address space}
  GIO_SCRNMAP     = $4B40;  {get screen mapping from kernel}
  PIO_SCRNMAP     = $4B41;  {put screen mapping table in kernel}
  GIO_UNISCRNMAP  = $4B69;  {get full Unicode screen mapping}
  PIO_UNISCRNMAP  = $4B6A;  {set full Unicode screen mapping}
  GIO_UNIMAP      = $4B66;  {get unicode-to-font mapping from kernel}
  PIO_UNIMAP      = $4B67;  {put unicode-to-font mapping in kernel}
  PIO_UNIMAPCLR   = $4B68;  {clear table, possibly advise hash algorithm}
  KDGKBDIACR      = $4B4A;  {read kernel accent table}
  KDSKBDIACR      = $4B4B;  {write kernel accent table}
  KDGETKEYCODE    = $4B4C;  {read kernel keycode table entry}
  KDSETKEYCODE    = $4B4D;  {write kernel keycode table entry}
  KDSIGACCEPT     = $4B4E;  {accept kbd generated signals}
  KDFONTOP        = $4B72;  {font operations}

  {Keyboard types (for KDGKBTYPE)}
  KB_84           = 1;
  KB_101          = 2;    {Normal PC keyboard.}
  KB_OTHER        = 3;

  {Keyboard LED constants.}
  LED_SCR         = 1;    {scroll lock led}
  LED_NUM         = 2;    {num lock led}
  LED_CAP         = 4;    {caps lock led}
    
  {Tty modes. (for KDSETMODE)}
  KD_TEXT         = 0;
  KD_GRAPHICS     = 1;
  KD_TEXT0        = 2;    {obsolete}
  KD_TEXT1        = 3;    {obsolete}

<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(cpumips) or defined(cpumipsel)}
  MAP_GROWSDOWN  = $1000;       { stack-like segment }
  MAP_DENYWRITE  = $2000;       { ETXTBSY }
  MAP_EXECUTABLE = $4000;      { mark it as an executable }
  MAP_LOCKED     = $8000;      { pages are locked }
  MAP_NORESERVE  = $4000;      { don't check for reservations; not defined for linux/mips? }
{$else cpumips}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  MAP_GROWSDOWN  = $100;       { stack-like segment }
  MAP_DENYWRITE  = $800;       { ETXTBSY }
  MAP_EXECUTABLE = $1000;      { mark it as an executable }
  MAP_LOCKED     = $2000;      { pages are locked }
  MAP_NORESERVE  = $4000;      { don't check for reservations }
<<<<<<< HEAD
<<<<<<< HEAD
{$endif cpumips}
=======
>>>>>>> graemeg/fixes_2_2

type
  TCloneFunc = function(args:pointer):longint;cdecl;

{$ifdef cpui386}
  {$define clone_implemented}
{$endif}
<<<<<<< HEAD
=======
{$ifdef cpum68k}
  {$define clone_implemented}
{$endif}
>>>>>>> graemeg/fixes_2_2

=======

type
  TCloneFunc = function(args:pointer):longint;cdecl;

{$ifdef cpui386}
  {$define clone_implemented}
{$endif}
{$ifdef cpum68k}
  {$define clone_implemented}
{$endif}

>>>>>>> origin/fixes_2_2
{$ifdef clone_implemented}
function clone(func:TCloneFunc;sp:pointer;flags:longint;args:pointer):longint; {$ifdef FPC_USE_LIBC} cdecl; external name 'clone'; {$endif}
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_USE_LIBC}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$if defined(cpui386) or defined(cpux86_64)}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
const
  MODIFY_LDT_CONTENTS_DATA       = 0;
  MODIFY_LDT_CONTENTS_STACK      = 1;
  MODIFY_LDT_CONTENTS_CODE       = 2;

{ Flags for user_desc.flags }
  UD_SEG_32BIT            = $01;
  UD_CONTENTS_DATA        = MODIFY_LDT_CONTENTS_DATA  shl 1;
  UD_CONTENTS_STACK       = MODIFY_LDT_CONTENTS_STACK shl 1;
  UD_CONTENTS_CODE        = MODIFY_LDT_CONTENTS_CODE  shl 1;
  UD_READ_EXEC_ONLY       = $08;
  UD_LIMIT_IN_PAGES       = $10;
  UD_SEG_NOT_PRESENT      = $20;
  UD_USEABLE              = $40;
  UD_LM                   = $80;

type
<<<<<<< HEAD
<<<<<<< HEAD
  user_desc = record
    entry_number  : cuint;
=======
  user_desc = packed record
    entry_number  : cint;
>>>>>>> graemeg/fixes_2_2
=======
  user_desc = packed record
    entry_number  : cint;
>>>>>>> origin/fixes_2_2
    base_addr     : cuint;
    limit         : cuint;
    flags         : cuint;
  end;

  TUser_Desc = user_desc;
  PUser_Desc = ^user_desc;

<<<<<<< HEAD
<<<<<<< HEAD
function modify_ldt(func:cint;p:pointer;bytecount:culong):cint;
{$endif cpui386 or cpux86_64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

procedure sched_yield; {$ifdef FPC_USE_LIBC} cdecl; external name 'sched_yield'; {$endif}

=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
type
  EPoll_Data = record
    case integer of
      0: (ptr: pointer);
      1: (fd: cint);
      2: (u32: cuint);
      3: (u64: cuint64);
  end;
  TEPoll_Data =  Epoll_Data;
  PEPoll_Data = ^Epoll_Data;

  EPoll_Event = {$ifdef cpu64} packed {$endif} record
    Events: cuint32;
    Data  : TEpoll_Data;
  end;

  TEPoll_Event =  Epoll_Event;
  PEpoll_Event = ^Epoll_Event;


{ open an epoll file descriptor }
function epoll_create(size: cint): cint; {$ifdef FPC_USE_LIBC} cdecl; external name 'epoll_create'; {$endif}
{ control interface for an epoll descriptor }
function epoll_ctl(epfd, op, fd: cint; event: pepoll_event): cint; {$ifdef FPC_USE_LIBC} cdecl; external name 'epoll_ctl'; {$endif}
{ wait for an I/O event on an epoll file descriptor }
function epoll_wait(epfd: cint; events: pepoll_event; maxevents, timeout: cint): cint; {$ifdef FPC_USE_LIBC} cdecl; external name 'epoll_wait'; {$endif}

type Puser_cap_header=^user_cap_header;
<<<<<<< HEAD
<<<<<<< HEAD
     user_cap_header=record
       version: cuint32;
       pid:cint;
     end;

     Puser_cap_data=^user_cap_data;
     user_cap_data=record
        effective,permitted,inheritable:cuint32;
=======
=======
>>>>>>> origin/fixes_2_2
     user_cap_header=packed record
       version,pid:cardinal;
     end;
     
     Puser_cap_data=^user_cap_data;
     user_cap_data=packed record
        effective,permitted,inheritable:cardinal;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     end;

{Get a capability.}
function capget(header:Puser_cap_header;data:Puser_cap_data):cint;{$ifdef FPC_USE_LIBC} cdecl; external name 'capget'; {$endif}
{Set a capability.}
function capset(header:Puser_cap_header;data:Puser_cap_data):cint;{$ifdef FPC_USE_LIBC} cdecl; external name 'capset'; {$endif}

<<<<<<< HEAD
<<<<<<< HEAD

=======
     
>>>>>>> graemeg/fixes_2_2
=======
     
>>>>>>> origin/fixes_2_2
const CAP_CHOWN            = 0;
      CAP_DAC_OVERRIDE     = 1;
      CAP_DAC_READ_SEARCH  = 2;
      CAP_FOWNER           = 3;
      CAP_FSETID           = 4;
      CAP_FS_MASK          = $1f;
      CAP_KILL             = 5;
      CAP_SETGID           = 6;
      CAP_SETUID           = 7;
      CAP_SETPCAP          = 8;
      CAP_LINUX_IMMUTABLE  = 9;
      CAP_NET_BIND_SERVICE = 10;
      CAP_NET_BROADCAST    = 11;
      CAP_NET_ADMIN        = 12;
      CAP_NET_RAW          = 13;
      CAP_IPC_LOCK         = 14;
      CAP_IPC_OWNER        = 15;
      CAP_SYS_MODULE       = 16;
      CAP_SYS_RAWIO        = 17;
      CAP_SYS_CHROOT       = 18;
      CAP_SYS_PTRACE       = 19;
      CAP_SYS_PACCT        = 20;
      CAP_SYS_ADMIN        = 21;
      CAP_SYS_BOOT         = 22;
      CAP_SYS_NICE         = 23;
      CAP_SYS_RESOURCE     = 24;
      CAP_SYS_TIME         = 25;
      CAP_SYS_TTY_CONFIG   = 26;
      CAP_MKNOD            = 27;
      CAP_LEASE            = 28;
      CAP_AUDIT_WRITE      = 29;
      CAP_AUDIT_CONTROL    = 30;

      LINUX_CAPABILITY_VERSION = $19980330;


//***********************************************SPLICE from kernel 2.6.17+****************************************

const
{* Flags for SPLICE and VMSPLICE.  *}
  SPLICE_F_MOVE		= 1;   { Move pages instead of copying.  }
  SPLICE_F_NONBLOCK	= 2;   {* Don't block on the pipe splicing
                            (but we may still block on the fd
                                        we splice from/to).  *}
  SPLICE_F_MORE	    = 4;   {* Expect more data.  *}
  SPLICE_F_GIFT	    = 8;   {* Pages passed in are a gift.  *}

{$ifdef cpu86}
{* Splice address range into a pipe.  *}
function vmsplice (fdout: cInt; iov: PIOVec; count: size_t; flags: cuInt): cInt; {$ifdef FPC_USE_LIBC} cdecl; external name 'vmsplice'; {$ENDIF}

{* Splice two files together.  *}
<<<<<<< HEAD
<<<<<<< HEAD
function splice (fdin: cInt; offin: off64_t; fdout: cInt;
                             offout: off64_t; len: size_t; flags: cuInt): cInt; {$ifdef FPC_USE_LIBC} cdecl; external name 'splice'; {$ENDIF}

=======
=======
>>>>>>> origin/fixes_2_2
// NOTE: offin and offout should be "off64_t" but we don't have that type. It's an "always 64 bit offset" so I use cint64
function splice (fdin: cInt; offin: cInt64; fdout: cInt;
                             offout: cInt64; len: size_t; flags: cuInt): cInt; {$ifdef FPC_USE_LIBC} cdecl; external name 'splice'; {$ENDIF}
                             
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function tee(fd_in: cInt; fd_out: cInt; len: size_t; flags: cuInt): cInt; {$ifdef FPC_USE_LIBC} cdecl; external name 'tee'; {$ENDIF}

{$endif} // x86

<<<<<<< HEAD
<<<<<<< HEAD
const
  { flags for sync_file_range }
  SYNC_FILE_RANGE_WAIT_BEFORE = 1;
  SYNC_FILE_RANGE_WRITE       = 2;
  SYNC_FILE_RANGE_WAIT_AFTER  = 4;

function sync_file_range(fd: cInt; offset, nbytes: off64_t; flags: cuInt): cInt; {$ifdef FPC_USE_LIBC} cdecl; external name 'sync_file_range'; {$ENDIF}
function fdatasync (fd: cint): cint; {$ifdef FPC_USE_LIBC} cdecl; external name 'sync_file_range'; {$ENDIF}

{$PACKRECORDS 1}

  { Flags for the parameter of inotify_init1.   }

Const
  IN_CLOEXEC  = &02000000;
  IN_NONBLOCK = &00004000;

Type
  inotify_event = record
    wd     : cint;
    mask   : cuint32;
    cookie : cuint32;
    len    : cuint32;
    name   : char;
  end;
  Pinotify_event = ^inotify_event;

  { Supported events suitable for MASK parameter of INOTIFY_ADD_WATCH.   }

  const
    IN_ACCESS        = $00000001;     { File was accessed.   }
    IN_MODIFY        = $00000002;     { File was modified.   }
    IN_ATTRIB        = $00000004;     { Metadata changed.   }
    IN_CLOSE_WRITE   = $00000008;     { Writtable file was closed.   }
    IN_CLOSE_NOWRITE = $00000010;     { Unwrittable file closed.   }
    IN_OPEN          = $00000020;     { File was opened.   }
    IN_MOVED_FROM    = $00000040;     { File was moved from X.   }
    IN_MOVED_TO      = $00000080;     { File was moved to Y.   }

    IN_CLOSE         = IN_CLOSE_WRITE or IN_CLOSE_NOWRITE;      { Close.   }
    IN_MOVE          = IN_MOVED_FROM or IN_MOVED_TO;      { Moves.   }

    IN_CREATE        = $00000100;     { Subfile was created.   }
    IN_DELETE        = $00000200;     { Subfile was deleted.   }
    IN_DELETE_SELF   = $00000400;     { Self was deleted.   }
    IN_MOVE_SELF     = $00000800;     { Self was moved.   }

  { Events sent by the kernel.   }
    IN_UNMOUNT       = $00002000;     { Backing fs was unmounted.   }
    IN_Q_OVERFLOW    = $00004000;     { Event queued overflowed.   }
    IN_IGNORED       = $00008000;     { File was ignored.   }

  { Special flags.   }
    IN_ONLYDIR       = $01000000;     { Only watch the path if it is a directory.   }
    IN_DONT_FOLLOW   = $02000000;     { Do not follow a sym link.   }
    IN_MASK_ADD      = $20000000;     { Add to the mask of an already  existing watch.   }
    IN_ISDIR         = $40000000;     { Event occurred against dir.   }
    IN_ONESHOT       = $80000000;     { Only send event once.   }

  { All events which a program can wait on.   }
    IN_ALL_EVENTS = IN_ACCESS or IN_MODIFY or IN_ATTRIB or IN_CLOSE
                    or IN_OPEN or IN_MOVE or IN_CREATE or IN_DELETE
                    or IN_DELETE_SELF or IN_MOVE_SELF;


// these have _THROW in the header.
{ Create and initialize inotify instance.   }
function inotify_init: cint;  {$ifdef FPC_USE_LIBC} cdecl; external name 'inotify_init'; {$ENDIF}
{ Create and initialize inotify instance.   }
function inotify_init1(flags:cint):cint;  {$ifdef FPC_USE_LIBC} cdecl; external name 'inotify_init1'; {$ENDIF}

{ Add watch of object NAME to inotify instance FD.
  Notify about events specified by MASK.   }
function inotify_add_watch(fd:cint; name:Pchar; mask:cuint32):cint;  {$ifdef FPC_USE_LIBC} cdecl; external name 'inotify_add_watch'; {$ENDIF}

{ Remove the watch specified by WD from the inotify instance FD.   }
function inotify_rm_watch(fd:cint; wd: cint):cint;  {$ifdef FPC_USE_LIBC} cdecl; external name 'inotify_rm_watch'; {$ENDIF}

{ clock_gettime, clock_settime, clock_getres }

Const
  // Taken from linux/time.h
  // Posix timers
  CLOCK_REALTIME                  = 0;
  CLOCK_MONOTONIC                 = 1;
  CLOCK_PROCESS_CPUTIME_ID        = 2;
  CLOCK_THREAD_CPUTIME_ID         = 3;
  CLOCK_MONOTONIC_RAW             = 4;
  CLOCK_REALTIME_COARSE           = 5;
  CLOCK_MONOTONIC_COARSE          = 6;
  // Linux specific
  CLOCK_SGI_CYCLE                 = 10;
  MAX_CLOCKS                      = 16;
  CLOCKS_MASK                     = CLOCK_REALTIME or CLOCK_MONOTONIC;
  CLOCKS_MONO                     = CLOCK_MONOTONIC;

Type
  clockid_t = cint;

// FPC_USE_LIBC unchecked, just to get it compiling again.
function clock_getres(clk_id : clockid_t; res : ptimespec) : cint; {$ifdef FPC_USE_LIBC} cdecl; external name 'clock_getres'; {$ENDIF}
function clock_gettime(clk_id : clockid_t; tp: ptimespec) : cint;  {$ifdef FPC_USE_LIBC} cdecl; external name 'clock_gettime'; {$ENDIF}
function clock_settime(clk_id : clockid_t; tp : ptimespec) : cint; {$ifdef FPC_USE_LIBC} cdecl; external name 'clock_settime'; {$ENDIF}
function setregid(rgid,egid : uid_t): cint; {$ifdef FPC_USE_LIBC} cdecl; external name 'setregid'; {$ENDIF} 
function setreuid(ruid,euid : uid_t): cint; {$ifdef FPC_USE_LIBC} cdecl; external name 'setreuid'; {$ENDIF} 

implementation


{$if not defined(FPC_USE_LIBC) or defined(cpui386) or defined(cpux86_64)}
{ needed for modify_ldt on x86 }
=======
=======
>>>>>>> origin/fixes_2_2
implementation


{$ifndef FPC_USE_LIBC}
>>>>>>> graemeg/fixes_2_2
Uses Syscall;
{$endif not defined(FPC_USE_LIBC) or defined(cpui386) or defined(cpux86_64)}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_USE_LIBC}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function Sysinfo(Info: PSysinfo): cInt;
begin
  Sysinfo := do_SysCall(SysCall_nr_Sysinfo, TSysParam(info));
end;

{$ifdef clone_implemented}
function clone(func:TCloneFunc;sp:pointer;flags:longint;args:pointer):longint;

begin
  if (pointer(func)=nil) or (sp=nil) then
   exit(-1); // give an error result
{$ifdef cpui386}
{$ASMMODE ATT}
  asm
        { Insert the argument onto the new stack. }
        movl    sp,%ecx
        subl    $8,%ecx
        movl    args,%eax
        movl    %eax,4(%ecx)

        { Save the function pointer as the zeroth argument.
          It will be popped off in the child in the ebx frobbing below. }
        movl    func,%eax
        movl    %eax,0(%ecx)

        { Do the system call }
        pushl   %ebx
        movl    flags,%ebx
        movl    syscall_nr_clone,%eax
        int     $0x80
        popl    %ebx
        test    %eax,%eax
        jnz     .Lclone_end

        { We're in the new thread }
        subl    %ebp,%ebp       { terminate the stack frame }
        call    *%ebx
        { exit process }
        movl    %eax,%ebx
        movl    syscall_nr_exit,%eax
        int     $0x80

.Lclone_end:
        movl    %eax,__RESULT
  end;
{$endif cpui386}
end;
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD

procedure sched_yield;

begin
  do_syscall(syscall_nr_sched_yield);
end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function epoll_create(size: cint): cint;
begin
{$if defined(generic_linux_syscalls)}
  epoll_create := do_syscall(syscall_nr_epoll_create1,0)
{$else}
  epoll_create := do_syscall(syscall_nr_epoll_create,tsysparam(size));
{$endif}
end;

function epoll_ctl(epfd, op, fd: cint; event: pepoll_event): cint;
begin
  epoll_ctl := do_syscall(syscall_nr_epoll_ctl, tsysparam(epfd),
    tsysparam(op), tsysparam(fd), tsysparam(event));
end;

function epoll_wait(epfd: cint; events: pepoll_event; maxevents, timeout: cint): cint;
begin
{$if defined(generic_linux_syscalls)}
  epoll_wait := do_syscall(syscall_nr_epoll_pwait, tsysparam(epfd),
    tsysparam(events), tsysparam(maxevents), tsysparam(timeout),0);
{$else}
  epoll_wait := do_syscall(syscall_nr_epoll_wait, tsysparam(epfd),
    tsysparam(events), tsysparam(maxevents), tsysparam(timeout));
{$endif}
end;

function capget(header:Puser_cap_header;data:Puser_cap_data):cint;

begin
  capget:=do_syscall(syscall_nr_capget,Tsysparam(header),Tsysparam(data));
end;

<<<<<<< HEAD
=======
function capget(header:Puser_cap_header;data:Puser_cap_data):cint;

begin
  capget:=do_syscall(syscall_nr_capget,Tsysparam(header),Tsysparam(data));
end;

>>>>>>> graemeg/fixes_2_2
function capset(header:Puser_cap_header;data:Puser_cap_data):cint;

begin
  capset:=do_syscall(syscall_nr_capset,Tsysparam(header),Tsysparam(data));
end;

<<<<<<< HEAD
// TODO: update also on non x86!
{$ifdef cpu86} // didn't update syscall_nr on others yet

function vmsplice (fdout: cInt; iov: PIOVec; count: size_t; flags: cuInt): cInt;
begin
<<<<<<< HEAD
  vmsplice := do_syscall(syscall_nr_vmsplice, TSysParam(fdout), TSysParam(iov),
    TSysParam(count), TSysParam(flags));
end;

function splice (fdin: cInt; offin: off64_t; fdout: cInt; offout: off64_t; len: size_t; flags: cuInt): cInt;
begin
  splice := do_syscall(syscall_nr_splice, TSysParam(fdin), TSysParam(@offin),
    TSysParam(fdout), TSysParam(@offout), TSysParam(len), TSysParam(flags));
=======
=======
function capget(header:Puser_cap_header;data:Puser_cap_data):cint;

begin
  capget:=do_syscall(syscall_nr_capget,Tsysparam(header),Tsysparam(data));
end;

function capset(header:Puser_cap_header;data:Puser_cap_data):cint;

begin
  capset:=do_syscall(syscall_nr_capset,Tsysparam(header),Tsysparam(data));
end;

// TODO: update also on non x86!
{$ifdef cpu86} // didn't update syscall_nr on others yet

function vmsplice (fdout: cInt; iov: PIOVec; count: size_t; flags: cuInt): cInt;
begin
>>>>>>> origin/fixes_2_2
  vmsplice := do_syscall(syscall_nr_vmsplice, TSysParam(fdout), TSysParam(iov), TSysParam(count), TSysParam(flags));
end;

function splice (fdin: cInt; offin: cint64; fdout: cInt; offout: cint64; len: size_t; flags: cuInt): cInt; 
begin
  splice := do_syscall(syscall_nr_splice, TSysParam(fdin), TSysParam(offin), TSysParam(fdout), TSysParam(offout), 
                       TSysParam(len), TSysParam(flags));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function tee(fd_in: cInt; fd_out: cInt; len: size_t; flags: cuInt): cInt;
begin
  tee := do_syscall(syscall_nr_tee, TSysParam(fd_in), TSysParam(fd_out),
                    TSysParam(len), TSysParam(flags));
end;

{$endif} // x86

<<<<<<< HEAD
<<<<<<< HEAD
function sync_file_range(fd: cInt; offset: off64_t; nbytes: off64_t; flags: cuInt): cInt;
begin
{$if defined(cpupowerpc) or defined(cpuarm)}
  sync_file_range := do_syscall(syscall_nr_sync_file_range2, TSysParam(fd), TSysParam(flags),
    TSysParam(hi(offset)), TSysParam(lo(offset)), TSysParam(hi(nbytes)), TSysParam(lo(nbytes)));
{$else}
{$if defined(cpupowerpc64)}
  sync_file_range := do_syscall(syscall_nr_sync_file_range2, TSysParam(fd), TSysParam(flags),
    TSysParam(offset), TSysParam(nbytes));
{$else}
{$ifdef cpu64}
  sync_file_range := do_syscall(syscall_nr_sync_file_range, TSysParam(fd), TSysParam(offset),
    TSysParam(nbytes), TSysParam(flags));
{$else}
  sync_file_range := do_syscall(syscall_nr_sync_file_range, TSysParam(fd), TSysParam(lo(offset)),
    TSysParam(hi(offset)), TSysParam(lo(nbytes)), TSysParam(hi(nbytes)), TSysParam(flags));
{$endif}
{$endif}
{$endif}
end;

function fdatasync (fd: cint): cint;
begin
  fdatasync:=do_SysCall(syscall_nr_fdatasync, fd);
end;

function futex(uaddr:Pcint;op,val:cint;timeout:Ptimespec;addr2:Pcint;val3:cint):cint;{$ifdef SYSTEMINLINE}inline;{$endif}

begin
  futex:=do_syscall(syscall_nr_futex,Tsysparam(uaddr),Tsysparam(op),Tsysparam(val),Tsysparam(timeout),
                    Tsysparam(addr2),Tsysparam(val3));
end;

function futex(var uaddr;op,val:cint;timeout:Ptimespec;var addr2;val3:cint):cint;{$ifdef SYSTEMINLINE}inline;{$endif}

begin
  futex:=do_syscall(syscall_nr_futex,Tsysparam(@uaddr),Tsysparam(op),Tsysparam(val),Tsysparam(timeout),
                    Tsysparam(@addr2),Tsysparam(val3));
end;

function futex(var uaddr;op,val:cint;var timeout:Ttimespec;var addr2;val3:cint):cint;{$ifdef SYSTEMINLINE}inline;{$endif}

begin
  futex:=do_syscall(syscall_nr_futex,Tsysparam(@uaddr),Tsysparam(op),Tsysparam(val),Tsysparam(@timeout),
                    Tsysparam(@addr2),Tsysparam(val3));
end;

function futex(uaddr:Pcint;op,val:cint;timeout:Ptimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}

begin
  futex:=do_syscall(syscall_nr_futex,Tsysparam(uaddr),Tsysparam(op),Tsysparam(val),Tsysparam(timeout));
end;

function futex(var uaddr;op,val:cint;timeout:Ptimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}

begin
  futex:=do_syscall(syscall_nr_futex,Tsysparam(@uaddr),Tsysparam(op),Tsysparam(val),Tsysparam(timeout));
end;

function futex(var uaddr;op,val:cint;var timeout:Ttimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}

begin
  futex:=do_syscall(syscall_nr_futex,Tsysparam(@uaddr),Tsysparam(op),Tsysparam(val),Tsysparam(@timeout));
end;

{$else}

{Libc case.}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(*
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
function futex(uaddr:Pcint;op,val:cint;timeout:Ptimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}

begin
  futex:=futex(uaddr,op,val,nil,nil,0);
end;

function futex(var uaddr;op,val:cint;timeout:Ptimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}

begin
  futex:=futex(@uaddr,op,val,nil,nil,0);
=======
{$endif} // non-libc

{ FUTEX_OP is a macro, doesn't exist in libC as function}
function FUTEX_OP(op, oparg, cmp, cmparg: cint): cint; {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  FUTEX_OP := ((op and $F) shl 28) or ((cmp and $F) shl 24) or ((oparg and $FFF) shl 12) or (cmparg and $FFF);
>>>>>>> origin/fixes_2_2
end;

function futex(var uaddr;op,val:cint;var timeout:Ttimespec):cint;{$ifdef SYSTEMINLINE}inline;{$endif}

begin
  futex:=futex(@uaddr,op,val,@timeout,nil,0);
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
*)
{$endif} // non-libc

{$ifndef FPC_USE_LIBC}
=======

{$endif} // non-libc

>>>>>>> graemeg/cpstrnew
=======

{$endif} // non-libc

>>>>>>> graemeg/cpstrnew
=======

{$endif} // non-libc

>>>>>>> graemeg/cpstrnew
=======

{$endif} // non-libc

>>>>>>> origin/cpstrnew
{$if defined(cpui386) or defined(cpux86_64)}
{ does not exist as a wrapper in glibc, and exists only for x86 }
function modify_ldt(func:cint;p:pointer;bytecount:culong):cint;

begin
  modify_ldt:=do_syscall(syscall_nr_modify_ldt,Tsysparam(func),
                                               Tsysparam(p),
                                               Tsysparam(bytecount));
end;
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

=======
{$endif} // non-libc

>>>>>>> graemeg/fixes_2_2
{ FUTEX_OP is a macro, doesn't exist in libC as function}
function FUTEX_OP(op, oparg, cmp, cmparg: cint): cint; {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  FUTEX_OP := ((op and $F) shl 28) or ((cmp and $F) shl 24) or ((oparg and $FFF) shl 12) or (cmparg and $FFF);
end;
<<<<<<< HEAD

{$ifndef FPC_USE_LIBC}
function inotify_init:cint;
=======
>>>>>>> graemeg/fixes_2_2

begin
  inotify_init:=inotify_init1(0);
end;

function inotify_init1(flags:cint):cint;

begin
{$if defined(generic_linux_syscalls)}
  inotify_init1:=do_SysCall(syscall_nr_inotify_init1,tsysparam(flags));
{$else}
  inotify_init1:=do_SysCall(syscall_nr_inotify_init,tsysparam(flags));
{$endif}
end;

function inotify_add_watch(fd:cint; name:Pchar; mask:cuint32):cint;

begin
  inotify_add_watch:=do_SysCall(syscall_nr_inotify_add_watch,tsysparam(fd),tsysparam(name),tsysparam(mask));
end;

function inotify_rm_watch(fd:cint; wd:cint):cint;

begin
  inotify_rm_watch:=do_SysCall(syscall_nr_inotify_rm_watch,tsysparam(fd),tsysparam(wd));
end;

function clock_getres(clk_id : clockid_t; res : ptimespec) : cint;

begin
  clock_getres:=do_SysCall(syscall_nr_clock_getres,tsysparam(clk_id),tsysparam(res));
end;

function clock_gettime(clk_id : clockid_t; tp: ptimespec) : cint;

begin
  clock_gettime:=do_SysCall(syscall_nr_clock_gettime,tsysparam(clk_id),tsysparam(tp));
end;

function clock_settime(clk_id : clockid_t; tp : ptimespec) : cint;

begin
  clock_settime:=do_SysCall(syscall_nr_clock_settime,tsysparam(clk_id),tsysparam(tp));
end;

function setregid(rgid,egid : uid_t): cint;

begin
  setregid:=do_syscall(syscall_nr_setregid,rgid,egid);
end;
 
function setreuid(ruid,euid : uid_t): cint;
begin
  setreuid:=do_syscall(syscall_nr_setreuid,ruid,euid);
end;

{$endif}
end.
