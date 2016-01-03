<<<<<<< HEAD
<<<<<<< HEAD
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos,aix,android }
{ %needlibrary }
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos }
>>>>>>> graemeg/fixes_2_2
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos }
>>>>>>> origin/fixes_2_2
{ %norun }

library tw6586a;
{$H+}{$MODE OBJFPC}

uses cmem;

procedure ExportTest1(input: longint); stdcall;
begin
 input:= 5;
end;

exports
  ExportTest1;

begin
end.

