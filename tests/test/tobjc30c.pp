{ %target=darwin }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %cpu=powerpc,powerpc64,i386,x86_64,arm,aarch64 }
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> origin/cpstrnew

{ Written by Jonas Maebe in 2009, released into the Public Domain }

{$mode objfpc}
{$modeswitch objectivec1}

uses
  uobjc30c;

type
  tla = objcclass(NSObject)
    function mytest(const c: shortstring): longint; message 'mystest:';
  end;

function tla.mytest(const c: shortstring): longint;
begin
  halt(1);
  result:=-1;
end;

var
  a: id;
begin
  a:=ta.alloc.init;
  ta(a).field:=123;
  if (a.mytest('c')<>123) then
    halt(2);
  a.release
end.
