{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    cdecl; definitions needed for unix unit.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ***********************************************************************}

type filedesarray=array[0..1] of cint;

{$if defined(solaris) or defined(aix)}
Function fpFlock (fd,mode : longint) : cint;{$ifdef IN_SYSTEM}forward;{$endif IN_SYSTEM}
<<<<<<< HEAD
{$else solaris or aix}
{$ifndef beos}
Function fpFlock (fd,mode : longint) : cint; cdecl; external clib name 'flock';
{$endif beos}
{$endif solaris or aix}

{$if defined(beos) or defined(solaris) }
Function fpfStatFS(Fd:Longint; Info:pstatfs):cint; cdecl; external clib name 'fstatvfs';
{$else beos or solaris }
Function fpfStatFS(Fd:Longint; Info:pstatfs):cint; cdecl; external clib name 'fstatfs';
{$endif beos or solaris }

Function fpfsync (fd : cint) : cint; cdecl; external clib name 'fsync';

{$if defined(beos) or defined(solaris) }
Function fpStatFS  (Path:pchar; Info:pstatfs):cint; cdecl; external clib name 'statvfs';
{$else beos or solaris }
Function fpStatFS  (Path:pchar; Info:pstatfs):cint; cdecl; external clib name 'statfs';
{$endif beos or solaris }

=======
{$else solaris}
{$ifndef beos}
Function fpFlock (fd,mode : longint) : cint; cdecl; external clib name 'flock';
{$endif beos}
{$endif solaris}

{$ifdef beos}
Function fpfStatFS(Fd:Longint; Info:pstatfs):cint; cdecl; external clib name 'fstatvfs';
{$else beos}
Function fpfStatFS(Fd:Longint; Info:pstatfs):cint; cdecl; external clib name 'fstatfs';
{$endif beos}

Function fpfsync (fd : cint) : cint; cdecl; external clib name 'fsync';

{$ifdef beos}
Function fpStatFS  (Path:pchar; Info:pstatfs):cint; cdecl; external clib name 'statvfs';
{$else beos}
Function fpStatFS  (Path:pchar; Info:pstatfs):cint; cdecl; external clib name 'statfs';
{$endif beos}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function pipe (var fildes: filedesarray):cint;  cdecl; external clib name 'pipe';
function fpgettimeofday(tp: ptimeval;tzp:ptimezone):cint;   cdecl; external clib name 'gettimeofday';
