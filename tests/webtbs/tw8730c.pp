<<<<<<< HEAD
<<<<<<< HEAD
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos,aix,android }
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
>>>>>>> graemeg/fixes_2_2
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
>>>>>>> origin/fixes_2_2
{ %NEEDLIBRARY }

{$mode delphi}
program MainApp;

uses
  sysutils;

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
<<<<<<< HEAD
<<<<<<< HEAD
  if error<>0 then
    writeln('tw8730c did not complete successfully');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  halt(error);
end.
