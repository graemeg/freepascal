{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Carl Eric Codere development team

    Base Unix unit modelled after POSIX 2001.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
Unit BaseUnix;

Interface
<<<<<<< HEAD
<<<<<<< HEAD
{$modeswitch out}
{$inline on}
Uses UnixType;

{$i osdefs.inc}       { Compile time defines }
=======

uses UnixType;
>>>>>>> graemeg/fixes_2_2
=======

uses UnixType;
>>>>>>> origin/fixes_2_2

{$i aliasptp.inc}

{$packrecords C}
<<<<<<< HEAD
<<<<<<< HEAD

{$ifndef FPC_USE_LIBC}
  {$define FPC_USE_SYSCALL}
{$endif}

{$i errno.inc}          { Error numbers }
{$i ostypes.inc}

{$ifdef FPC_USE_LIBC}
  const clib = 'root';
  const netlib = 'net';
  {$i oscdeclh.inc}
{$ELSE}
  {$i bunxh.inc}		{ Functions}
=======
=======
>>>>>>> origin/fixes_2_2
{$define oldreaddir}		// Keep using readdir system call instead
				// of userland getdents stuff.
{$define usedomain}		// Allow uname with "domain" entry.
				// (which is a GNU extension)
{$define posixworkaround}	// Temporary ugly workaround for signal handler.
				// (mainly until baseunix migration is complete)

{$ifndef FPC_USE_LIBC}
{$define FPC_USE_SYSCALL}
{$endif}

{$i errno.inc}		{ Error numbers }
{$i ostypes.inc}

{$ifdef FPC_USE_LIBC}
const clib = 'root';
const netlib = 'net';
{$i oscdeclh.inc}
{$ELSE}
{$i bunxh.inc}		{ Functions}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF}

function fpgeterrno:longint; 
procedure fpseterrno(err:longint); 

{$ifndef ver1_0}
property errno : cint read fpgeterrno write fpseterrno;
{$endif}

{$i bunxovlh.inc}

{$ifdef FPC_USE_LIBC}
{$ifdef beos}
function  fpsettimeofday(tp:ptimeval;tzp:ptimezone):cint;
Function fpFlock (var fd : text; mode : longint) : cint; 
Function fpFlock (var fd : File; mode : longint) : cint; 
Function fpFlock (fd, mode : longint) : cint; 
Function  FpNanoSleep  (req : ptimespec;rem : ptimespec):cint;
{$endif}
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
{$i genfunch.inc}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ Fairly portable constants. I'm not going to waste time to duplicate and alias
them anywhere}

Const
  MAP_FAILED    = pointer(-1);  { mmap() has failed }
  MAP_SHARED    =  $1;          { Share changes }
  MAP_PRIVATE   =  $2;          { Changes are private }
  MAP_TYPE      =  $f;          { Mask for type of mapping }
  MAP_FIXED     = $10;          { Interpret addr exactly }

// MAP_ANON(YMOUS) is OS dependant but used in the RTL and in ostypes.inc
// Under BSD without -YMOUS, so alias it:
  MAP_ANON	= MAP_ANONYMOUS;

  PROT_READ     =  $1;          { page can be read }
  PROT_WRITE    =  $2;          { page can be written }
  PROT_EXEC     =  $4;          { page can be executed }
  PROT_NONE     =  $0;          { page can not be accessed }

implementation

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef hassysctl}
Uses Sysctl;
{$endif}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$i genfuncs.inc}       // generic calls. (like getenv)
{$I gensigset.inc}     // general sigset funcs implementation.
{$I genfdset.inc}      // general fdset funcs.

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_USE_LIBC}
  {$i oscdecl.inc}        // implementation of wrappers in oscdeclh.inc
{$else}
  {$i syscallh.inc}       // do_syscall declarations themselves
  {$i sysnr.inc}          // syscall numbers.
  {$i bsyscall.inc}       // cpu specific syscalls
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_USE_LIBC}
  {$i syscallh.inc}       // do_syscall declarations themselves
  {$i sysnr.inc}          // syscall numbers.
  {$i bsyscall.inc}  			// cpu specific syscalls
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$i bunxsysc.inc}       // syscalls in system unit.
//  {$i settimeo.inc}
{$endif}
{$i settimeo.inc}
{$i osmacro.inc}        { macro implenenations }
{$i bunxovl.inc}        { redefs and overloads implementation }

{$ifndef ver1_0}
function fpgeterrno:longint; external name 'FPC_SYS_GETERRNO';
procedure fpseterrno(err:longint); external name 'FPC_SYS_SETERRNO';
{$else}
// workaround for 1.0.10 bugs.

function intgeterrno:longint; external name 'FPC_SYS_GETERRNO';
procedure intseterrno(err:longint); external name 'FPC_SYS_SETERRNO';

function fpgeterrno:longint; 
begin
  fpgeterrno:=intgeterrno;
end;

procedure fpseterrno(err:longint); 
begin
  intseterrno(err);
end;

{$endif}

function fpsettimeofday(tp:ptimeval;tzp:ptimezone):cint;
begin
  fpsettimeofday := settimeofday(tp, tzp);
end;

Function fpFlock (var fd : File; mode : longint) : cint; 
begin
  {$warning TODO BeOS fpFlock implementation}  
end;

Function fpFlock (var fd : Text; mode : longint) : cint; 
begin
  {$warning TODO BeOS fpFlock implementation}  
end;

Function fpFlock (fd, mode : longint) : cint; 
begin
  {$warning TODO BeOS fpFlock implementation}  
end;

function snooze(microseconds : bigtime_t) : status_t; cdecl; external 'root' name 'snooze';

Function  FpNanoSleep  (req : ptimespec;rem : ptimespec):cint;
begin
  case snooze((req^.tv_nsec div 1000) + (req^.tv_sec * 1000 * 1000)) of
    B_OK : FpNanoSleep := 0;
    B_INTERRUPTED : FpNanoSleep := - 1;
    else
      FpNanoSleep := - 1;
  end;
end;

end.
