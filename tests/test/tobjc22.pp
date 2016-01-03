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

  TMyObjectA = objcclass(NSObject, MyProtocolB)
    function newMethod: longint;
    class function newClassMethod: longint;
  end;

  TMyObjectB = objcclass(NSObject,MyProtocolA)
    function newMethod: longint; message 'n';
    class function newClassMethod: longint; message 'newClassMethod';
  end;

{ TMyObjectA }

function TMyObjectA.newMethod: longint;
begin
  result:=1;
end;

class function TMyObjectA.newClassMethod: longint;
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


var
  pMyProtocolA : Protocol;
  pMyProtocolB : Protocol;
  pNSProxy     : Protocol;
  a   : TMyObjectA;
  b   : TMyObjectB;
begin
  pMyProtocolA:=objcprotocol(MyProtocolA);
  pMyProtocolB:=objcprotocol(MyProtocolB);
  pNSProxy:=Protocol(objc_getprotocol('NSProxy'));
  writeln('TMyObjectA conforms to MyProtocolA protocol: ',  TMyObjectA.classconformsToProtocol_(pMyProtocolA)); {true}
  if not TMyObjectA.classconformsToProtocol_(pMyProtocolA) then
    halt(1);
  writeln('TMyObjectA conforms to MyProtocolB protocol: ',  TMyObjectA.classconformsToProtocol_(pMyProtocolB)); {true}
  if not TMyObjectA.classconformsToProtocol_(pMyProtocolB) then
    halt(2);
  writeln('TMyObjectB conforms to MyProtocolA protocol: ',  TMyObjectB.classconformsToProtocol_(pMyProtocolA)); {true}
  if not TMyObjectB.classconformsToProtocol_(pMyProtocolA) then
    halt(3);
  writeln('TMyObjectB conforms to MyProtocolB protocol: ',  TMyObjectB.classconformsToProtocol_(pMyProtocolB)); {false}
  if TMyObjectB.classconformsToProtocol_(pMyProtocolB) then
    halt(4);
  writeln('TMyObjectA conforms to NSProxy protocol:     ',  TMyObjectA.classconformsToProtocol_(pNSProxy));    {false}
  if TMyObjectA.classconformsToProtocol_(pNSProxy) then
    halt(5);

  if TMyObjectA.newClassMethod<>2 then
    halt(11);
  if TMyObjectB.newClassMethod<>4 then
    halt(12);

  a := TMyObjectA.alloc;
  writeln('TMyObjectA instance conforms to MyProtocolA protocol: ',  a.classconformsToProtocol_(pMyProtocolA)); {true}
  if not a.classconformsToProtocol_(pMyProtocolA) then
    halt(6);
  writeln('TMyObjectA instance conforms to MyProtocolB protocol: ',  a.classconformsToProtocol_(pMyProtocolB)); {true}
  if not a.classconformsToProtocol_(pMyProtocolB) then
    halt(7);
  writeln('TMyObjectA instance conforms to NSProxy protocol:     ',  a.classconformsToProtocol_(pNSProxy));     {false}
  if a.classconformsToProtocol_(pNSProxy) then
    halt(8);
  if a.newMethod<>1 then
    halt(21);
  a.Release;

  b := TMyObjectB.alloc;
  writeln('TMyObjectB instance conforms to MyProtocolA protocol: ',  b.conformsToProtocol_(pMyProtocolA)); {true}
  if not b.conformsToProtocol_(pMyProtocolA) then
    halt(6);
  writeln('TMyObjectB instance conforms to MyProtocolB protocol: ',  b.conformsToProtocol_(pMyProtocolB)); {false}
  if b.conformsToProtocol_(pMyProtocolB) then
    halt(7);
  if b.newMethod<>3 then
    halt(31);
  b.Release;
end.

