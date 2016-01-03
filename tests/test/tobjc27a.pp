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
{ %recompile }

{ Written by Jonas Maebe in 2009, released into the public domain }

{$modeswitch objectivec1}

uses
  uobjc27a,uobjc27b;

var
  a: ta;
  c: tachild;

begin
  a:=ta(ta.alloc).init;
  if a.da_categorymethod<>2 then
    halt(1);
  a.release;

  c:=tachild(tachild.alloc).init;
  if c.da_categorymethod<>2 then
    halt(2);
  if c.eachild_categorymethod<>3 then
    halt(3);
  c.release;
end.
