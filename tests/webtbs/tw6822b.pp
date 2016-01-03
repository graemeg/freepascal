{ %needlibrary }
<<<<<<< HEAD
<<<<<<< HEAD
{ %delfiles=tw6822a }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

program loader;
{$mode objfpc}{$H+}

uses
<<<<<<< HEAD
<<<<<<< HEAD
  popuperr,
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  dynlibs;
var
  h: TLibHandle;
const
{$ifdef unix}
<<<<<<< HEAD
<<<<<<< HEAD
libname = './libtw6822a.'+SharedSuffix;
{$else unix}
libname = 'tw6822a.' + SharedSuffix;
{$endif unix}

=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef darwin}
libname = './libtw6822a.dylib';
{$else darwin}
libname = './libtw6822a.so';
{$endif darwin}
{$endif unix}

{$ifdef windows}
libname = 'tw6822a.dll';
{$endif windows}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  writeln('hello from loader program');
  h:= loadlibrary(libname);
  if h = nilhandle then
  begin
    write('could not load library');
    exit;
  end;
  freelibrary(h);
end.
