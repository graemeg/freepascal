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

{ Written by Jonas Maebe in 2009, released into the public domain }

{$mode objfpc}
{$modeswitch objectivec1}

var
  a: NSObject;
begin
  a:=NSObject(NSObject(NSObject.alloc).init);
  if a.respondstoselector_(objcselector('isKindOfClass:')) then

    writeln('ok string selector!')
  else
    halt(1);
  if a.respondstoselector_(objcselector(NSObject.init)) then
    writeln('ok method selector!')
  else
    halt(2);

  if (a.self<>id(a)) then
    halt(3);
  if (a.superclass<>nil) then
    halt(4);
  a.release;
end.
