<<<<<<< HEAD
<<<<<<< HEAD
{ %target=win32,win64,linux,android}
{ %needlibrary }
{ %delfiles=tw7838a }
=======
{ %target=win32,win64,linux}
>>>>>>> graemeg/fixes_2_2
=======
{ %target=win32,win64,linux}
>>>>>>> origin/fixes_2_2

program prog;
{$mode objfpc}

uses
<<<<<<< HEAD
<<<<<<< HEAD
  popuperr,
  dynlibs;
=======
 dynlibs;
>>>>>>> graemeg/fixes_2_2
=======
 dynlibs;
>>>>>>> origin/fixes_2_2

// this function is exported from the EXE
function exetest: longint; {public name 'exetest';}
begin
  writeln('exe test');
  result:=5;
end;

exports
  exetest name 'exetest';

const
{$ifdef unix}
{$ifdef darwin}
  libname = './libtw7838a.dylib';
{$else}
  libname = './libtw7838a.so';
{$endif}
{$endif}
{$ifdef windows}
  libname = 'tw7838a.dll';
{$endif}

var
  dllf: function: longint;
  lh: tlibhandle;

begin

  lh:= loadlibrary(libname); // load dyn.so (unix) or dyn.dll (ms windows)
  if lh = nilhandle then
    begin
      writeln('dyn library returned nil handle');
      halt(1);
    end;
  pointer(dllf):= getprocaddress(lh, 'dllf'); // get function from dll

<<<<<<< HEAD
<<<<<<< HEAD
  // call function in dll, which calls function in exe, and then prints
=======
  // call function in dll, which calls function in exe, and then prints 
>>>>>>> graemeg/fixes_2_2
=======
  // call function in dll, which calls function in exe, and then prints 
>>>>>>> origin/fixes_2_2
  // a result number 5
  if (dllf()<>5) then
    halt(1);
  writeln(dllf());
  writeln('end of program');
  freelibrary(lh);
end.
