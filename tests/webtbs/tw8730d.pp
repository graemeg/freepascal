<<<<<<< HEAD
<<<<<<< HEAD
{ %target=darwin,linux,freebsd,solaris,beos,aix,android}
{ %NEEDLIBRARY }
{ %delfiles=tw8730a tw8730b }
=======
{ %target=darwin,linux,freebsd,solaris,beos}
{ %NEEDLIBRARY }
>>>>>>> graemeg/fixes_2_2
=======
{ %target=darwin,linux,freebsd,solaris,beos}
{ %NEEDLIBRARY }
>>>>>>> origin/fixes_2_2

{ same as tw8730c, but linking to libc so it uses different }
{ startup code                                              }

{$mode delphi}
program MainApp;

uses
  initc, sysutils;

const
{$ifdef windows}
  libname='tw8730b.dll';
{$else}
  libname='tw8730b';
  {$linklib tw8730b}
{$endif}

function Lib2Func: pchar; CDecl; external libname name 'Lib2Func';

var
  error: byte;
begin
  error:=0;
  WriteLn( Lib2Func );
  if not(fileexists('tw8730a.txt')) or
     not(fileexists('tw8730b.txt')) then
   error:=1;
  if (fileexists('tw8730a.txt')) then
    deletefile('tw8730a.txt');
  if (fileexists('tw8730b.txt')) then
    deletefile('tw8730b.txt');
  halt(error);
end.
