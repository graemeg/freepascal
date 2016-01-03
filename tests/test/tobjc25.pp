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
  tbaseclass = objccategory(NSObject)
    function tabaseproc(cp: longint): double; message 'tabaseproc:';
    class function taclassproc: longint; message 'taclassproc';
  end;

  ta = objcclass(NSObject)
    a: longint;
    procedure taproc; message 'taproc';
    function tabaseproc(cp: longint): double; message 'tabaseproc:'; override;
    class function taclassproc: longint; message 'taclassproc'; override;
  end;

  ca = objccategory(ta)
    procedure categorymethod; message 'categorymethod';
    function tabaseproc(cp: longint): double; reintroduce;
  end;

  da = objccategory(ta)
    procedure anothercategorymethod; message 'anothercategorymethod';
    class function taclassproc: longint; reintroduce;
  end;

class function tbaseclass.taclassproc: longint;
begin
  writeln('tbaseclass.taclassproc');
  result:=654321;
end;

function tbaseclass.tabaseproc(cp: longint): double;
begin
  writeln('tbaseclass.tabaseproc');
  if (cp<>98765) then
    halt(12);
  result:=1234.875;
end;

procedure ta.taproc;
begin
  a:=0;
  categorymethod;
  if (a<>1) then
    halt(1);
  anothercategorymethod;
  if (a<>2) then
    halt(2);
  if taclassproc<>123456 then
    halt(5);
end;

function ta.tabaseproc(cp: longint): double;
begin
  { should be replaced/hidden by ca.tabaseproc }
  halt(9);
  result:=-1.0;
end;

class function ta.taclassproc: longint;
begin
  { should be replaced/hidden by da.taclassproc }
  halt(3);
  result:=0;
end;

procedure ca.categorymethod;
begin
  a:=1;
  if tabaseproc(555) <> 1.0 then
    halt(16);
end;

function ca.tabaseproc(cp: longint): double;
begin
  writeln('start ca.tabaseproc');
  if (cp<>555) then
    halt(13);
  if inherited tabaseproc(98765)<>1234.875 then
    halt(11);
  writeln('end ca.tabaseproc');
  result:=1.0;
end;

procedure da.anothercategorymethod;
begin
  a:=2;
  if tabaseproc(555)<>1.0 then
    halt(15);
end;

class function da.taclassproc: longint;
begin
  writeln('start da.taclassproc, calling inherited');
  if inherited taclassproc<>654321 then
    halt(4);
  writeln('end da.taclassproc');
  result:=123456;
end;

var
  a: ta;
begin
  a:=ta(ta.alloc).init;
  a.taproc;
  a.a:=0;
  a.categorymethod;
  if (a.a<>1) then
    halt(6);
  a.anothercategorymethod;
  if (a.a<>2) then
    halt(7);
  if a.taclassproc<>123456 then
    halt(8);
  if (a.tabaseproc(555)<>1.0) then
    halt(14);
  a.release;
end.
