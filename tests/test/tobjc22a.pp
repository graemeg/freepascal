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
    function newMethod: longint; message 'n';
  end;

  MyProtocolB = objcprotocol(MyProtocolA)
    class function newClassMethod: longint; message 'newClassMethod';
  end;


  { TMyObject }

  TMyObjectA = objcclass(NSObject, MyProtocolA)
    function newMethod: longint;
  end;

  TMyObjectB = objcclass(NSObject,MyProtocolB)
    function newMethod: longint;
    class function newClassMethod: longint;
  end;

{ TMyObjectA }

function TMyObjectA.newMethod: longint;
begin
  result:=1;
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


var
  a   : MyProtocolA;
  b   : MyProtocolB;
begin
  a:=TMyObjectA.alloc.init;
  b:=TMyObjectB.alloc.init;
  if a.newMethod<>1 then
    halt(1);
  if b.newMethod<>3 then
    halt(3);
  if b.newclassmethod<>4 then
    halt(4);
  id(a).release;
  id(b).release;
end.

