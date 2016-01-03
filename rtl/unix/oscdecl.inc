{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by the Free Pascal development team
    
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

<<<<<<< HEAD
<<<<<<< HEAD

{ ********************************************************************* }
{ fpioctl                                                               }
{ ********************************************************************* }

=======
{$if defined(bsd) or defined(solaris)}
>>>>>>> graemeg/fixes_2_2
=======
{$if defined(bsd) or defined(solaris)}
>>>>>>> origin/fixes_2_2
function  real_FpIOCtl   (Handle:cint;Ndx: TIOCtlRequest):cint; cdecl; varargs; external clib name 'ioctl';

function  FpIOCtl   (Handle:cint;Ndx: TIOCtlRequest;Data: Pointer):cint;
begin
  FpIOCtl:=real_FpIOCtl(Handle, Ndx, Data);
end;
<<<<<<< HEAD
<<<<<<< HEAD


{ ********************************************************************* }
{ fpfcntl                                                               }
{ ********************************************************************* }

function real_FpFcntl (fildes : cInt; cmd : cInt): cInt; cdecl; varargs; external clib name 'fcntl';

Function  FpFcntl      (fildes : cInt; cmd : cInt): cInt;
begin
  FpFcntl:=real_FpFcntl(fildes, cmd);
end;


Function  FpFcntl      (fildes : cInt; cmd : cInt; arg :cInt): cInt;
begin
  FpFcntl:=real_FpFcntl(fildes, cmd, arg);
end;


Function  FpFcntl      (fildes : cInt; cmd : cInt; var arg : flock): cInt;
begin
  FpFcntl:=real_FpFcntl(fildes, cmd, @arg);
end;


{ ********************************************************************* }
{ fpopen                                                                }
{ ********************************************************************* }

function  real_FpOpen(path: pchar; flags : cint):cint; varargs; cdecl; external clib name 'open'{$ifdef aix}+suffix64bit{$endif};

function  FpOpen    (path: pchar; flags : cint; mode: TMode):cint;
begin
{$if defined(linux) and defined(fs32bit)}
  flags:=flags or O_LARGEFILE;
{$endif}
  { emulate what the bunxovl(h).inc version of fpopen does. Required because
    existing code depends on this (it doesn't always pass a valid mode when
    using fmCreate) }
  FpOpen:=real_FpOpen(path,flags,mode);
end;
=======
{$endif bsd or solaris}

>>>>>>> graemeg/fixes_2_2
=======
{$endif bsd or solaris}

>>>>>>> origin/fixes_2_2
