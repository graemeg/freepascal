{ %norun }
<<<<<<< HEAD
<<<<<<< HEAD
{ %needlibrary }
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos,aix,android }
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
>>>>>>> graemeg/fixes_2_2
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
>>>>>>> origin/fixes_2_2

library tw9089a;

{$mode objfpc}{$H+}

var
  myvar: longint; cvar;

exports
  myvar;

initialization
  Writeln('INIT');
  myvar:=-1;

finalization
  Writeln('FINI');
  if (myvar<>1) then
    halt(1);
end.
