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

{ Written by Jonas Maebe in 2010, released into the public domain }

{$mode objfpc}
{$modeswitch objectivec1}

type
  MyOverride = objcclass(NSObject)
    class procedure testClassOverride; message 'testClassOverride';
  end;

  MyOverride2 = objcclass(MyOverride)
    class procedure testClassOverride; override;
  end;

  tmyoverrideclass = class of NSObject;

var
  selfshouldbe: tmyoverrideclass;

class procedure MyOverride.testClassOverride;
begin
  if (self<>selfshouldbe) then
    halt(20);

  writeln('MyOverride.testClassOverride');
end;

class procedure MyOverride2.testClassOverride;
begin
  if (self<>selfshouldbe) then
    halt(21);

  writeln('MyOverride2.testClassOverride');
end;

var
  a: MyOverride;
  classclass: tmyoverrideclass;
begin
  classclass:=MyOverride;
  a:=classclass.alloc.init;
  selfshouldbe:=MyOverride;
  a.testClassOverride;
  a.release;

  classclass:=MyOverride2;
  a:=classclass.alloc.init;
  selfshouldbe:=MyOverride2;
  a.testClassOverride;
  a.release;
end.
