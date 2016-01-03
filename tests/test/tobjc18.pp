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

type
  MyOverride = objcclass(NSObject)
    procedure release; override;
    class procedure testClassOverride; message 'testClassOverride';
  end;

  MyOverride2 = objcclass(MyOverride)
    procedure release; override;
    class procedure testClassOverride; override;
  end;
  tmyoverrideclass = class of myoverride;

var
  selfshouldbe: tmyoverrideclass;
  selfshouldbetestinlinetypedefinition: class of myoverride2;
  overridescalled: longint;

procedure MyOverride.release;
begin
  writeln('releasing override!');
  if (overridescalled<>3) then
    halt(1);
  inc(overridescalled);
  inherited release;
end;

class procedure MyOverride.testClassOverride;
begin
  if (self<>selfshouldbe) then
    halt(20);

  writeln('MyOverride.testClassOverride');
  if (overridescalled<>1) then
    halt(3);
  inc(overridescalled);
end;

procedure MyOverride2.release;
begin
  inherited testClassOverride;
  writeln('releasing override2!');
  if (overridescalled<>2) then
    halt(2);
  inc(overridescalled);
  inherited release;
end;

class procedure MyOverride2.testClassOverride;
begin
  if (self<>selfshouldbe) then
    halt(21);

  if (overridescalled<>0) then
    halt(5);
  writeln('MyOverride2.testClassOverride');
  inc(overridescalled);
  inherited testClassOverride;
end;

var
  a: MyOverride;
  b: id;
begin
  { test type compatibility conversions between id and class ref }
  b:=MyOverride2;
  selfshouldbetestinlinetypedefinition:=b;
  b:=selfshouldbetestinlinetypedefinition;

  { tested calling class methods and inherited class methods }
  a:=MyOverride2.alloc;
  a:=a.init;
  selfshouldbe:=MyOverride2;
  MyOverride2.testClassOverride;
  if (overridescalled<>2) then
    halt(6);
  dec(overridescalled);
  selfshouldbe:=MyOverride;
  MyOverride.testClassOverride;
  if (overridescalled<>2) then
    halt(7);
  overridescalled:=0;
  selfshouldbe:=MyOverride2;
  a.testClassOverride;
  overridescalled:=1;
  a.release;
end.
