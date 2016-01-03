<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos,aix,android }
{ %needlibrary }
{ %delfiles=tw16949a }
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
{ %needlibrary }
>>>>>>> graemeg/cpstrnew
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
{ %needlibrary }
>>>>>>> graemeg/cpstrnew
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
{ %needlibrary }
>>>>>>> graemeg/cpstrnew
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
{ %needlibrary }
>>>>>>> origin/cpstrnew

program ptest;

{$ifdef fpc}{$mode objfpc}{$H+}{$endif fpc}

const
{$if defined(windows) or defined(mswindows)}
  libname='tw16949a.dll';
{$else}
  libname='tw16949a';
  {$linklib tw16949a}
{$ifend}

function foo: LongInt; cdecl; external libname;

begin
  if foo<>12345 then
    halt(1);
end.
