<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %target=darwin,linux,freebsd,solaris,haiku,aix }
=======
{ %target=darwin,linux,freebsd,solaris,haiku }
>>>>>>> graemeg/cpstrnew
=======
{ %target=darwin,linux,freebsd,solaris,haiku }
>>>>>>> graemeg/cpstrnew
=======
{ %target=darwin,linux,freebsd,solaris,haiku }
>>>>>>> graemeg/cpstrnew
=======
{ %target=darwin,linux,freebsd,solaris,haiku }
>>>>>>> origin/cpstrnew

{$mode objfpc}
uses
  cthreads, pthreads, classes, unixtype;

type
  tc = class(tthread)
    procedure execute;override;
  end;

procedure tc.execute;
begin
end;

function threadproc(arg: pointer): pointer; cdecl;
var
  p: pointer;
  a: ansistring;
begin
  setlength(a,4000000);
  getmem(p,5);
  writeln('hi from thread ',ptruint(arg));
  freemem(p);
  result:=pointer(ptruint(arg)+10);
end;

var
  t1, t2, t3: pthread_t;
  res: pointer;
begin
  { initialise threading system }
  with tc.create(false) do
    begin
      waitfor;
      free;
    end;
  if pthread_create(@t1,nil,@threadproc,pointer(1))<>0 then
    begin
      writeln('error creating 1');
      halt(1);
    end;
  if pthread_create(@t2,nil,@threadproc,pointer(2))<>0 then
    begin
      writeln('error creating 2');
      halt(1);
    end;
  if pthread_create(@t3,nil,@threadproc,pointer(3))<>0 then
    begin
      writeln('error creating 3');
      halt(1);
    end;

  if pthread_join(t1,@res)<>0 then
    begin
      writeln('error joining 1');
      halt(1);
    end;
  if res<>pointer(11) then
    begin
      writeln('error 1');
      halt(1);
    end;

  if pthread_join(t2,@res)<>0 then
    begin
      writeln('error joining 1');
      halt(1);
    end;
  if res<>pointer(12) then
    begin
      writeln('error 2');
      halt(2);
    end;

  if pthread_join(t3,@res)<>0 then
    begin
      writeln('error joining 1');
      halt(1);
    end;
  if res<>pointer(13) then
    begin
      writeln('error 3');
      halt(3);
    end;
end.
