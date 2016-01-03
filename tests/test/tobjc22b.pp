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

program protocoltest;

{$mode objfpc}{$H+}
{$modeswitch objectivec1}

type
  MyProtocolA = objcprotocol
    function newMethod: longint; message 'newMethod';
   optional
    function optionalMethod: longint; message 'optionalMethod';
  end;

  MyProtocolB = objcprotocol(MyProtocolA)
    class function newClassMethod: longint; message 'newClassMethod';
   optional
    class function optionalClassMethod: longint; message 'optionalClassMethod';
  end;


  { TMyObject }

  TMyObjectA = objcclass(NSObject, MyProtocolA)
    function newMethod: longint;
    function optionalMethod: longint;
  end;

  TMyObjectB = objcclass(NSObject,MyProtocolB)
    function newMethod: longint;
    class function newClassMethod: longint;
    class function optionalClassMethod: longint;
  end;

{ TMyObjectA }

function TMyObjectA.newMethod: longint;
begin
  result:=1;
end;

function TMyObjectA.optionalMethod: longint;
begin
  result:=2;
end;

{ TMyObjectB }

function TMyObjectB.newMethod: longint;
begin
  result:=3;
end;

class function TMyObjectB.newClassMethod: longint;
begin
  result:=4;
end;

class function TMyObjectB.optionalClassMethod: longint;
begin
  result:=5;
end;

var
  a   : MyProtocolA;
  b   : MyProtocolB;
begin
  a:=TMyObjectA.alloc.init;
  b:=TMyObjectB.alloc.init;
  if a.newMethod<>1 then
    halt(1);
  if a.optionalMethod<>2 then
    halt(2);
  if b.newMethod<>3 then
    halt(3);
  if b.newclassmethod<>4 then
    halt(4);
  if b.optionalclassmethod<>5 then
    halt(5);
  if not id(a).conformsToProtocol_(objcprotocol(MyProtocolA)) then
    halt(6);
  if not id(b).conformsToProtocol_(objcprotocol(MyProtocolA)) then
    halt(7);
  if not id(b).conformsToProtocol_(objcprotocol(MyProtocolB)) then
    halt(8);
  id(a).release;
  id(b).release;
end.

