{ %target=darwin }
<<<<<<< HEAD
{ %cpu=powerpc,powerpc64,i386,x86_64,arm,aarch64 }
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> graemeg/cpstrnew

{ Written by Jonas Maebe in 2009, released into the public domain }

{$modeswitch objectivec1}

uses
  uobjc26;

var
  a: ta;

begin
  a:=ta(ta.alloc).init;
  a.taproc;
  a.release
end.
